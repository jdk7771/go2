"""Independent saved-result audit; no fitting module imported."""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial import Delaunay


def main():
    root = Path(__file__).resolve().parent
    output = root/"outputs/table_multilayer"
    manifest = json.loads((output/"manifest.json").read_text())
    cfg = manifest["settings"]
    checks = []
    for case in manifest["cases"]:
        source = manifest["sources"][case["source"]]
        path = root/source["path"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == source["sha256"]
        original = np.load(path)
        expected = original["inducing"][:, :3] if case["source"] == "sparse" else original["points"]
        saved = np.load(output/"arrays"/f"{case['key']}.npz")
        points, xy = saved["points"], saved["xy"]
        np.testing.assert_array_equal(points, expected)
        data = json.loads((output/case["file"]).read_text())
        gate = case["height_gate_m"]
        audited_queries, audited_probes, faces = 0, 0, 0
        centres = saved["centres"]
        delta = np.abs(points[:, 2, None]-centres[None, :])
        labels = np.argmin(delta, axis=1)
        labels[delta.min(axis=1) > gate] = -1
        np.testing.assert_array_equal(saved["labels"], labels)
        for i, centre in enumerate(centres):
            prefix = f"layer{i}_"
            height = saved[prefix+"height"]
            valid = np.isfinite(height)
            ids = saved[prefix+"neighbor_ids"][valid]
            assert ids.shape == (int(valid.sum()), 30)
            assert np.all(ids >= 0) and np.all(labels[ids] == i)
            assert np.all(np.abs(points[ids, 2]-centre) <= gate+1e-12)
            distance = np.linalg.norm(points[ids]-np.column_stack((xy[valid], np.full(valid.sum(), centre)))[:, None, :], axis=2)
            assert np.all(distance <= cfg["radius_m"]+1e-12)
            assert np.all(np.abs(height[valid]-centre) <= gate+1e-12)
            triangles = saved[prefix+"triangles"]
            assert np.all(valid[triangles])
            assert np.all(np.ptp(height[triangles], axis=1) <= cfg["max_triangle_height_jump_m"]+1e-12)
            # All triangle XY edges are a grid edge or cell diagonal (no long bridges).
            vertices = xy[triangles]
            for edge in ((0, 1), (1, 2), (2, 0)):
                assert np.all(np.linalg.norm(vertices[:, edge[0]]-vertices[:, edge[1]], axis=1) <= np.sqrt(2)*.05+1e-10)
            exported = np.array([np.nan if v is None else v for v in data["layers"][i]["height"]])
            np.testing.assert_allclose(exported, height, atol=5.01e-8, rtol=0, equal_nan=True)
            np.testing.assert_array_equal(np.asarray(data["layers"][i]["triangles"]).reshape(-1, 3), triangles)
            for probe in data["probes"]:
                location = np.asarray(probe["xy"])
                q = np.argmin(np.linalg.norm(xy-location, axis=1))
                entry = probe["layers"][i]
                candidates = np.flatnonzero(labels == i)
                dist = np.linalg.norm(points[candidates]-[*xy[q], centre], axis=1)
                eligible = candidates[dist <= cfg["radius_m"]]
                assert entry["available"] == len(eligible)
                selected = np.array(entry["ids"], dtype=int)
                assert len(selected) == min(30, len(eligible))
                # Ties may choose different ID sets; compare distance multisets.
                chosen = np.linalg.norm(points[selected]-[*xy[q], centre], axis=1)
                np.testing.assert_allclose(np.sort(chosen), np.sort(dist[dist <= cfg["radius_m"]])[:30], atol=1e-12)
                if np.isfinite(height[q]):
                    assert Delaunay(points[selected, :2]).find_simplex(xy[q]) >= 0
                    u, v = (points[selected, :2]-xy[q]).T
                    design = np.column_stack((np.ones(30), u, v, u*u, u*v, v*v))
                    # Independent unregularized SVD prediction should agree up to tiny ridge.
                    coefficient = np.linalg.lstsq(design, points[selected, 2], rcond=None)[0]
                    assert abs(coefficient[0]-height[q]) < 1e-7
                if len(eligible) < 30:
                    assert not np.isfinite(height[q])
                audited_probes += 1
            audited_queries += int(valid.sum())
            faces += len(triangles)
        checks.append(dict(case=case["key"], valid_query_layer_pairs=audited_queries,
                           probes=audited_probes, triangles=faces, passed=True))
    result = dict(passed=True, cases=len(checks), checks=checks,
                  assertions=["input hashes and exact XYZ", "exclusive nearest-mode assignment",
                              "exactly 30 same-layer gated neighbours for every valid fit",
                              "3D radius and predicted height bounds", "finite same-layer mesh and max jump",
                              "JSON matches full precision arrays", "brute-force nearest-neighbour probe check",
                              "independent Delaunay hull and SVD prediction probe checks"])
    for folder in (output, root/"outputs/visualization/table-layers"):
        (folder/"independent-audit.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()

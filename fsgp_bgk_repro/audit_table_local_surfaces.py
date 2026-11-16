"""Independent output audit, without importing the local fitting module."""
import hashlib
import json
from pathlib import Path

import numpy as np


def main():
    root = Path(__file__).resolve().parent
    output = root/"outputs/table_local_surfaces"
    manifest = json.loads((output/"manifest.json").read_text())
    cfg = manifest["settings"]
    raw = np.load(root/manifest["input_path"])["points"]
    saved = np.load(output/"results.npz")
    data = json.loads((output/"data.json").read_text())
    points = saved["points"]
    np.testing.assert_array_equal(raw, points)
    assert hashlib.sha256((root/manifest["input_path"]).read_bytes()).hexdigest() == manifest["input_sha256"]
    assert hashlib.sha256((root/"table_local_surfaces.py").read_bytes()).hexdigest() == manifest["code_sha256"]
    k = cfg["neighbors"]
    valid = np.isfinite(saved["height"])
    max_svd_error = 0.
    for i in range(len(points)):
        distance = np.linalg.norm(points-points[i], axis=1)
        gate = (distance <= cfg["radius_m"]) & (np.abs(points[:, 2]-points[i, 2]) <= cfg["seed_height_gate_m"])
        candidates = np.flatnonzero(gate)
        assert len(candidates) == saved["candidate_counts"][i]
        ids = saved["neighbor_ids"][i]
        if len(candidates) < k:
            assert np.all(ids == -1) and not valid[i]
            continue
        assert len(np.unique(ids)) == k and np.all(gate[ids])
        np.testing.assert_allclose(np.sort(distance[ids]), np.sort(distance[candidates])[:k], atol=1e-12, rtol=0)
        if valid[i]:
            u, v = (points[ids, :2]-points[i, :2]).T
            design = np.column_stack((np.ones(k), u, v, u*u, u*v, v*v))
            target = points[ids, 2]-points[i, 2]
            expected = np.linalg.lstsq(design, target, rcond=None)[0]
            error = abs(points[i, 2]+expected[0]-saved["height"][i])
            max_svd_error = max(max_svd_error, error)
            assert error < 1e-7
            rmse = np.sqrt(np.mean((design@saved["coefficient"][i]-target)**2))
            assert abs(rmse-saved["rmse"][i]) < 1e-12
            assert rmse <= cfg["max_fit_rmse_m"]
    pairs = saved["edges"]
    a, b = pairs.T
    predicted = points.copy(); predicted[:, 2] = saved["height"]
    assert np.all(valid[pairs])
    assert np.all(np.linalg.norm(predicted[a]-predicted[b], axis=1) <= cfg["connect_radius_m"]+1e-12)
    for source, target in ((a, b), (b, a)):
        u, v = (points[target, :2]-points[source, :2]).T
        design = np.column_stack((np.ones(len(a)), u, v, u*u, u*v, v*v))
        z = points[source, 2]+np.einsum("ni,ni->n", design, saved["coefficient"][source])
        assert np.all(np.abs(z-saved["height"][target]) <= cfg["max_model_disagreement_m"]+1e-12)
    cosine = np.einsum("ni,ni->n", saved["normal"][a], saved["normal"][b])
    assert np.all(cosine >= np.cos(np.deg2rad(cfg["max_normal_angle_deg"]))-1e-12)
    # Reconstruct connectivity with union-find instead of scipy.csgraph.
    parent = np.arange(len(points))
    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for i, j in pairs:
        parent[find(i)] = find(j)
    groups = {}
    for i in np.flatnonzero(valid):
        groups.setdefault(find(i), []).append(i)
    major = [np.array(v) for v in groups.values() if len(v) >= cfg["min_component_points"]]
    assert len(major) == len(data["components"])
    exported = [set(c["point_ids"]) for c in data["components"]]
    assert all(set(ids) in exported for ids in major)
    edges = {tuple(sorted(pair)) for pair in pairs.tolist()}
    triangle_count, max_dz = 0, 0.
    for component in data["components"]:
        index = component["id"]
        triangles = saved[f"component{index}_triangles"]
        assert np.all(saved["labels"][triangles] == index)
        np.testing.assert_array_equal(triangles, np.array(component["triangles"]).reshape(-1, 3))
        for triangle in triangles:
            for i, j in ((0, 1), (1, 2), (2, 0)):
                assert tuple(sorted(triangle[[i, j]])) in edges
        if len(triangles):
            max_dz = max(max_dz, float(np.ptp(saved["height"][triangles], axis=1).max()))
        triangle_count += len(triangles)
        assert abs(np.median(saved["height"][component["point_ids"]])-component["median_z"]) < 5.01e-8
    # Validate every rendered patch vertex against the owning fitted polynomial.
    vertices = np.array(data["patch_vertices"]).reshape(-1, 3)
    owners = np.array(data["patch_owners"])
    u, v = (vertices[:, :2]-points[owners, :2]).T
    design = np.column_stack((np.ones(len(u)), u, v, u*u, u*v, v*v))
    expected_z = points[owners, 2]+np.einsum("ni,ni->n", design, saved["coefficient"][owners])
    assert np.max(np.abs(expected_z-vertices[:, 2])) < 1e-6
    audit = dict(passed=True, input_points=len(points), brute_force_neighbor_checks=len(points),
                 valid_fits=int(valid.sum()), independent_svd_max_height_error_m=max_svd_error,
                 compatible_edges=len(pairs), union_find_major_components=len(major),
                 triangles=triangle_count, observed_max_triangle_height_span_m=max_dz,
                 rendered_patch_vertices_checked=len(vertices),
                 checks=["input and source hashes", "exact input XYZ", "brute-force radius and seed-relative gate before nearest 30",
                         "independent SVD fitting and residuals", "mutual model and normal compatibility",
                         "independent union-find connectivity with no preset component count",
                         "all mesh edges belong to compatibility graph", "rendered patches are actual polynomial samples"])
    for folder in (output, root/"outputs/visualization/table-local"):
        (folder/"independent-audit.json").write_text(json.dumps(audit, indent=2))
    print(json.dumps(audit))


if __name__ == "__main__":
    main()

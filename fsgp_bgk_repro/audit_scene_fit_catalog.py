"""Re-read and verify every delivered full-scene quadratic result."""
import csv
import json
from pathlib import Path
import numpy as np

from .scene_fit_catalog import OUT, ROOT, ORDER, source_cloud, validate, exact_neighbors, fit_centers, file_sha
from .table_local_surfaces import LocalConfig, fit_seeds


def audit():
    records = []
    for name in ORDER:
        directory = OUT / name
        meta = json.loads((directory / "data.json").read_text())
        full, source = source_cloud(name)
        assert source == meta["source"]
        assert np.array_equal(np.fromfile(directory / "points.bin", dtype="<f4").reshape(-1, 3), full)
        with np.load(directory / "results.npz", allow_pickle=False) as data:
            n = len(data["points"])
            assert n == meta["seed_count"]
            assert np.array_equal(data["points"], full[data["source_ids"]])
            neighbors = np.fromfile(directory / "neighbors.bin", dtype="<i4").reshape(n, 30)
            assert np.array_equal(neighbors, data["neighbor_ids"])
            binary = np.fromfile(directory / "seeds.bin", dtype="<f8").reshape(n, 14)
            np.testing.assert_array_equal(binary[:, :3], data["points"])
            np.testing.assert_array_equal(binary[:, 3:9], data["coefficients"])
            np.testing.assert_array_equal(binary[:, 9], data["height"])
            np.testing.assert_array_equal(binary[:, 10], data["rmse"])
            np.testing.assert_array_equal(binary[:, 11], data["labels"])
            assert np.array_equal(binary[:, 12].astype(bool), np.isfinite(data["height"]))
            triangles = np.fromfile(directory / "mesh.bin", dtype="<u4").reshape(-1, 3)
            assert np.array_equal(triangles, data["triangles"])
            components = [{"triangles":triangles[c["triangle_start"]:c["triangle_start"]+c["triangle_count"]]} for c in meta["components"]]
            result = {"points":data["points"], "source_ids":data["source_ids"], "labels":data["labels"],
                      "components":components, "fits":{"neighbor_ids":neighbors, "coefficient":data["coefficients"],
                                                        "height":data["height"], "rmse":data["rmse"]}}
            validate(full.astype(float), result, LocalConfig())
            valid = np.isfinite(data["height"])
            solved = neighbors[:, 0] >= 0
            assert int(valid.sum()) == meta["valid"] and int(solved.sum()) == meta["solved"]
            for key, values in [("training_residual_all", data["rmse"][solved]), ("training_residual_valid", data["rmse"][valid])]:
                assert meta[key]["count"] == len(values)
                np.testing.assert_allclose([meta[key]["median_m"], meta[key]["p95_m"], meta[key]["max_m"]],
                                           [np.median(values), np.percentile(values, 95), values.max()], atol=1e-14)
            max_errors = np.fromfile(directory / "max-error.bin", dtype="<f8")
            assert len(max_errors) == n and np.isnan(max_errors[~solved]).all()
            active = np.flatnonzero(solved)
            for start in range(0, len(active), 2048):
                ids = active[start:start+2048]
                local = full[neighbors[ids]].astype(float)
                xy = local[:, :, :2] - data["points"][ids, None, :2]
                u, v = xy[:, :, 0], xy[:, :, 1]
                design = np.stack((np.ones_like(u), u, v, u*u, u*v, v*v), axis=2)
                prediction = np.einsum("nki,ni->nk", design, data["coefficients"][ids]) + data["points"][ids, None, 2]
                expected = np.max(np.abs(prediction-local[:, :, 2]), axis=1)
                np.testing.assert_allclose(max_errors[ids], expected, atol=1e-11)
            for label, selected in (("all", max_errors[solved]), ("valid", max_errors[valid])):
                stats = meta["max_abs_error"][label]
                assert stats["count"] == len(selected)
                np.testing.assert_allclose([stats["median_m"], stats["p95_m"], stats["max_m"]],
                                           [np.median(selected), np.percentile(selected, 95), np.max(selected)], atol=1e-14)
            with (directory / "max-error.csv").open() as stream:
                count = 0
                for row in csv.DictReader(stream):
                    i = int(row["seed"])
                    assert i == count
                    count += 1
                    if not solved[i]:
                        assert row["training_max_abs_error_m"] == "" and row["worst_neighbor_point"] == ""
                    else:
                        assert float(row["training_max_abs_error_m"]) == max_errors[i]
                        assert int(row["worst_neighbor_point"]) in neighbors[i]
                        assert abs(abs(float(row["worst_signed_residual_m"]))-max_errors[i]) < 1e-12
                        assert abs(float(row["worst_predicted_z_m"])-float(row["worst_observed_z_m"])-float(row["worst_signed_residual_m"])) < 1e-11
                assert count == n
        with (directory / "per-fit.csv").open() as stream:
            rows = 0
            for row in csv.DictReader(stream):
                rows += 1
                if row["solved"] == "0":
                    assert row["training_rmse_m"] == ""
            assert rows == n
        assert len(meta["timings"]["repeats"]) == 3
        for key, median in meta["timings"]["medians"].items():
            assert median == float(np.median([r[key] for r in meta["timings"]["repeats"]]))
        records.append({"id":name, "passed":True, "raw_points":len(full), "seeds":n,
                        "result_sha256":file_sha(directory / "results.npz"), "all_residuals_recomputed":True,
                        "all_max_abs_errors_independently_recomputed":True, "max_error_csv_all_centers":True})
        print(f"Audited {name}: {n} centers", flush=True)
    # Check the accelerated neighbor selection and solve against the actual
    # frozen 2125-point experiment, not just synthetic polynomial examples.
    evidence = ROOT / "outputs/table_cpu/table_m125_evidence.npz"
    with np.load(evidence, allow_pickle=False) as data:
        points = data["points"].astype(float)
    config = LocalConfig()
    previous = fit_seeds(points, config)
    ids = exact_neighbors(points, points, config)
    np.testing.assert_array_equal(ids, previous["neighbor_ids"])
    new = fit_centers(points, points, ids, config)
    for key in ("coefficient", "height", "rmse", "full_rank"):
        np.testing.assert_allclose(new[key], previous[key], atol=1e-12, equal_nan=True)
    result = {"passed":True, "port":42482, "scene_count":len(records), "scenes":records,
              "frozen_2125_point_neighbor_and_coefficient_parity":True,
              "source_files":{"scene_fit_catalog.py":file_sha(ROOT / "scene_fit_catalog.py"),
                              "table_local_surfaces.py":file_sha(ROOT / "table_local_surfaces.py")}}
    (OUT / "catalog-audit.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    return result


if __name__ == "__main__":
    result = audit()
    print(json.dumps({"passed":result["passed"], "scenes":result["scene_count"],
                      "frozen_2125_parity":result["frozen_2125_point_neighbor_and_coefficient_parity"]}))

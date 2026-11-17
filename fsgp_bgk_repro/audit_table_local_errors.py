"""Audit every per-polynomial error/footprint record independently of exporter."""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree, distance


def main():
    root = Path(__file__).resolve().parent
    folder = root/"outputs/table_local_errors"
    data = json.loads((folder/"data.json").read_text())
    path = root/"outputs/table_local_surfaces/results.npz"
    saved = np.load(path)
    assert hashlib.sha256(path.read_bytes()).hexdigest() == data["provenance"]["results_sha256"]
    assert hashlib.sha256((root/"analyze_table_local_errors.py").read_bytes()).hexdigest() == data["provenance"]["source_sha256"]
    records, points = data["records"], saved["points"]
    assert len(records) == 2125
    active = np.flatnonzero(saved["neighbor_ids"][:, 0] >= 0)
    assert sum(r["solved"] for r in records) == len(active) == 1964
    assert sum(r["valid"] for r in records) == np.isfinite(saved["height"]).sum() == 1955
    truth_queries, truth_values = [], []
    for i, record in enumerate(records):
        assert record["seed"] == i
        if not record["solved"]:
            assert record["fit_rmse_m"] is None and record["truth_neighbor_rmse_m"] is None
            continue
        ids = saved["neighbor_ids"][i]
        xy = points[ids, :2]-points[i, :2]
        u, v = xy.T
        design = np.column_stack((np.ones(30), u, v, u*u, u*v, v*v))
        z = points[i, 2]+design@saved["coefficient"][i]
        residual = z-points[ids, 2]
        np.testing.assert_allclose(residual, record["neighbor_residuals_m"], atol=1e-12, rtol=0)
        assert abs(np.sqrt(np.mean(residual**2))-record["fit_rmse_m"]) < 1e-12
        assert abs(record["fit_rmse_m"]-saved["rmse"][i]) < 1e-12
        assert abs(np.sqrt(np.mean(np.array(record["neighbor_true_distances_m"])**2))-record["truth_neighbor_rmse_m"]) < 1e-12
        assert abs(np.linalg.norm(xy, axis=1).max()-record["radius_xy_m"]) < 1e-12
        assert abs(distance.pdist(points[ids, :2]).max()-record["diameter_xy_m"]) < 1e-12
        assert abs(np.ptp(points[ids, 0])-record["width_x_m"]) < 1e-12
        assert abs(np.ptp(points[ids, 1])-record["width_y_m"]) < 1e-12
        truth_queries.append(np.column_stack((points[ids, :2], z)))
        truth_values.extend(record["neighbor_true_distances_m"])
    # Distance to a sampled point on a true surface is an independent upper
    # bound on distance to the complete analytic surface. Source PCD includes
    # float32 coordinates, hence a 2e-6 numeric tolerance.
    full = np.load(root/"outputs/visualization/large-table/large_table.npz")["points"]
    upper = cKDTree(full).query(np.concatenate(truth_queries))[0]
    exact = np.array(truth_values)
    assert np.all(exact >= 0) and np.all(exact <= upper+2e-6)
    for metric, statistics in data["summary"]["valid_statistics"].items():
        values = np.array([r[metric] for r in records if r["valid"]])
        assert statistics["n"] == len(values)
        for field, actual in (("median", np.median(values)), ("p05", np.percentile(values, 5)), ("p95", np.percentile(values, 95)), ("maximum", values.max())):
            assert abs(statistics[field]-actual) < 1e-12
    rows = list(csv.DictReader((folder/"per-curve.csv").open()))
    assert len(rows) == len(records)
    for row, record in zip(rows, records):
        assert int(row["seed"]) == record["seed"]
        assert (float(row["fit_rmse_m"]) if row["fit_rmse_m"] else None) == record["fit_rmse_m"]
    example = data["triangle_example"]
    a, b = example["a"], example["b"]
    def predict(seed, query):
        u, v = points[query, :2]-points[seed, :2]
        return points[seed, 2]+np.array([1,u,v,u*u,u*v,v*v])@saved["coefficient"][seed]
    expected = max(abs(predict(a,a)-predict(b,a)), abs(predict(a,b)-predict(b,b)))
    assert abs(expected-example["worst_disagreement_m"]) < 1e-12 and expected > .02
    result = dict(passed=True, records=len(records), solved=len(active), valid=int(np.isfinite(saved["height"]).sum()),
                  individual_residuals_checked=len(active)*30, truth_sample_upper_bound_checks=len(exact),
                  statistics_recomputed=True, csv_matches_json=True, triangle_example_recomputed=True,
                  original_fit_result_hash_unchanged=True)
    for target in (folder, root/"outputs/table_local_surfaces/errors", root/"outputs/visualization/table-local/errors"):
        (target/"audit.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result))


if __name__ == "__main__":
    main()

"""Recompute metrics from saved evidence and verify experiment integrity."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def read_csv(path):
    with open(path, newline="") as stream:
        return list(csv.DictReader(stream))


def audit(output):
    protocol = json.loads((output / "protocol.json").read_text())
    rows = read_csv(output / "per_frame.csv")
    summaries = read_csv(output / "summary.csv")
    combinations = len(protocol["scenes"]) * len(protocol["inducing_counts"])
    assert len(rows) == combinations * protocol["frames"] * 3, "missing frame rows"
    assert len(summaries) == combinations * 3, "missing summary rows"
    root = Path(__file__).resolve().parent
    for name, digest in protocol["implementation_sha256"].items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == digest, f"code changed: {name}"
    assert hashlib.sha256((root / "sources.lock.json").read_bytes()).hexdigest() == protocol["source_lock_sha256"]
    hashes = {}
    for row in rows:
        key = (row["scene"], row["frame"])
        previous = hashes.setdefault(key, row["scan_sha256"])
        assert previous == row["scan_sha256"], "methods or M ablations received different scans"
    checked_maps = 0
    for scene in protocol["scenes"]:
        for count in protocol["inducing_counts"]:
            prefix = output / f"{scene}_m{count}"
            with np.load(str(prefix) + "_evidence.npz", allow_pickle=False) as evidence:
                reference = evidence["reference_cost"].ravel()
                scan_hash = hashlib.sha256(evidence["points"].tobytes()).hexdigest()
            maps = {}
            for method, suffix in (("SGP-XY", "sgp"), ("FSGP", "fsgp"), ("FSGP-BGK", "bgk")):
                with np.load(str(prefix) + f"_{suffix}.npz", allow_pickle=False) as data:
                    maps[method] = {key: data[key] for key in data.files}
            common = maps["SGP-XY"]["observed"] & maps["FSGP"]["observed"] & np.isfinite(reference)
            for method, data in maps.items():
                cost = data["cost"].ravel()
                assert np.array_equal(np.isfinite(cost), data["known"]), "unknown/NaN mismatch"
                assert np.all(data["known"] | ~data["observed"]), "current support lost"
                assert np.all(np.isfinite(data["variance"])) and np.all(data["variance"] >= 0)
                row = next(r for r in rows if r["scene"] == scene and int(r["inducing_points"]) == count
                           and r["method"] == method and int(r["frame"]) == protocol["frames"] - 1)
                assert row["scan_sha256"] == scan_hash, "saved evidence has different input"
                assert int(row["evaluation_cells"]) == int(common.sum())
                error = cost[common] - reference[common]
                if len(error):
                    np.testing.assert_allclose(float(row["cost_mae"]), np.abs(error).mean(), atol=1e-12)
                    np.testing.assert_allclose(float(row["cost_rmse"]), np.sqrt(np.mean(error**2)), atol=1e-12)
                    np.testing.assert_allclose(float(row["signed_error_variance"]), error.var(), atol=1e-12)
                np.testing.assert_allclose(float(row["known_coverage"]), data["known"].mean(), atol=1e-12)
                np.testing.assert_allclose(float(row["current_coverage"]), data["observed"].mean(), atol=1e-12)
                checked_maps += 1
    for summary in summaries:
        group = [r for r in rows if r["scene"] == summary["scene"] and
                 r["inducing_points"] == summary["inducing_points"] and r["method"] == summary["method"]]
        valid = [r for r in group if r["cost_mae"]]
        if valid:
            counts = np.array([int(r["evaluation_cells"]) for r in valid])
            mae = np.average([float(r["cost_mae"]) for r in valid], weights=counts)
            rmse = np.sqrt(np.average([float(r["cost_rmse"])**2 for r in valid], weights=counts))
            np.testing.assert_allclose(float(summary["cost_mae"]), mae, atol=1e-12)
            np.testing.assert_allclose(float(summary["cost_rmse"]), rmse, atol=1e-12)
        np.testing.assert_allclose(float(summary["median_runtime_ms"]),
                                   np.median([float(r["runtime_ms"]) for r in group]), atol=1e-12)
    report = {"status": "passed", "frame_metric_rows": len(rows), "summary_rows": len(summaries),
              "saved_maps_recomputed": checked_maps, "identical_scans_across_methods_and_m": True,
              "unknown_masks_and_variances_valid": True, "implementation_hashes_match": True}
    (output / "audit.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    audit(parser.parse_args().output)

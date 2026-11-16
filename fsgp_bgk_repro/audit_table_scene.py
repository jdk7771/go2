"""Independently recompute all table frames and the reported interior statistics."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .audit_results import audit
from .experiment import ROOT
from .io import read_cloud


def run(output):
    output = Path(output)
    audit(output)
    protocol = json.loads((output / "protocol.json").read_text())
    source_path = Path(protocol["source_pcd"])
    assert hashlib.sha256(source_path.read_bytes()).hexdigest() == protocol["source_pcd_sha256"]
    source = read_cloud(source_path)
    scene = json.loads((source_path.parent / "scene.json").read_text())
    with (output / "per_frame.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    with (output / "summary.csv").open() as stream:
        summaries = list(csv.DictReader(stream))
    checked = 0
    for count in protocol["inducing_counts"]:
        pool = {method: {key: [] for key in ("height", "slope", "height_variation", "cost")}
                for method in ("SGP-XY", "FSGP", "FSGP-BGK")}
        for frame in range(protocol["frames"]):
            with np.load(output / "scans" / f"frame_{frame:03d}.npz") as evidence:
                digest = hashlib.sha256(evidence["points"].tobytes()).hexdigest()
            maps = {}
            for method, suffix in (("SGP-XY", "sgp"), ("FSGP", "fsgp"), ("FSGP-BGK", "bgk")):
                path = output / "frames" / f"m{count}" / f"frame_{frame:03d}_{suffix}.npz"
                with np.load(path) as data:
                    maps[method] = {key: data[key] for key in data.files}
            xy = maps["FSGP"]["xy"]
            half = protocol["config"]["resolution"] / 2
            valid = np.all((xy-half >= np.asarray(scene["floor"]["min"])[:2]-1e-6)
                           & (xy+half <= np.asarray(scene["floor"]["max"])[:2]+1e-6), axis=1)
            for leg in scene["legs"]:
                intersects = np.all((xy+half > np.asarray(leg["min"])[:2])
                                    & (xy-half < np.asarray(leg["max"])[:2]), axis=1)
                valid &= ~intersects
            common = maps["SGP-XY"]["observed"] & maps["FSGP"]["observed"] & valid
            inside = (np.abs(xy[:, 0]) <= 1+1e-6) & (np.abs(xy[:, 1]) <= .5+1e-6)
            for method, data in maps.items():
                row = next(r for r in rows if int(r["inducing_points"]) == count
                           and int(r["frame"]) == frame and r["method"] == method)
                assert row["scan_sha256"] == digest
                assert int(row["evaluation_cells"]) == int(common.sum())
                cost = data["cost"].ravel()
                np.testing.assert_array_equal(np.isfinite(cost), data["known"])
                np.testing.assert_allclose(float(row["cost_mae"]), np.mean(np.abs(cost[common])), atol=1e-12)
                np.testing.assert_allclose(float(row["cost_rmse"]), np.sqrt(np.mean(cost[common]**2)), atol=1e-12)
                under = inside & valid & data["observed"]
                assert int(row["under_cells"]) == int(under.sum())
                np.testing.assert_allclose(float(row["under_height_median_m"]), np.median(data["height"][under]), atol=1e-12)
                for name, key in (("under_slope_mean", "slope"), ("under_height_variation_mean_m", "height_variation"),
                                  ("under_cost_mean", "cost")):
                    np.testing.assert_allclose(float(row[name]), np.asarray(data[key]).ravel()[under].mean(), atol=1e-12)
                for key in pool[method]:
                    pool[method][key].extend(np.asarray(data[key]).ravel()[inside & common].tolist())
                checked += 1
            np.testing.assert_array_equal(maps["FSGP"]["height"], maps["FSGP-BGK"]["height"])
        for method, values in pool.items():
            row = next(r for r in summaries if int(r["inducing_points"]) == count and r["method"] == method)
            np.testing.assert_allclose(float(row["under_height_median_m"]), np.median(values["height"]), atol=1e-12)
            np.testing.assert_allclose(float(row["under_slope_mean"]), np.mean(values["slope"]), atol=1e-12)
            np.testing.assert_allclose(float(row["under_cost_mean"]), np.mean(values["cost"]), atol=1e-12)
    # Check the identical-input / different-height ambiguity directly from geometry.
    central = source[np.all(np.isclose(source[:, :2], 0, atol=1e-6), axis=1)]
    np.testing.assert_allclose(central[:, 2], [0, 1.1, 1.2], atol=1e-6)
    _, nearest = cKDTree(source).query(central, k=protocol["config"]["neighbors"])
    np.testing.assert_allclose(source[nearest, 2], np.broadcast_to(central[:, 2, None], nearest.shape), atol=1e-12)
    with np.load(output / "control_floor_only_clean.npz") as floor:
        np.testing.assert_allclose(floor["height"], 0, atol=1e-12)
        np.testing.assert_allclose(floor["slope"], 0, atol=1e-12)
        np.testing.assert_allclose(floor["cost"][np.isfinite(floor["cost"])], 0, atol=1e-12)
    report = {"status": "passed", "saved_frame_maps_recomputed": checked,
              "floor_reference_uses_cell_leg_intersection": True,
              "interior_statistics_recomputed": True, "input_pcd_hash_matches": True,
              "multilayer_ambiguity_verified_from_geometry": True, "floor_only_zero_control_verified": True}
    (output / "table_audit.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, nargs="?", default=ROOT / "outputs/table_cpu")
    run(parser.parse_args().output)

"""Construct an overhanging table without collapsing its multiple height layers.

This is complete, noise-free surface geometry, not an occlusion-aware LiDAR scan.
Run from the repository root: /usr/bin/python3 -m fsgp_bgk_repro.table_scene
"""

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent
DEFAULT_OUTPUT = ROOT / "outputs/visualization/large-table"


def geometry():
    legs = []
    for x in (-1.32, 1.32):
        for y in (-0.82, 0.82):
            legs.append({"label": len(legs) + 2, "name": f"leg_{len(legs) + 1}",
                         "min": [x - 0.06, y - 0.06, 0.0],
                         "max": [x + 0.06, y + 0.06, 1.1]})
    return {
        "name": "large_table", "units": "m", "up_axis": "z",
        "description": "Complete synthetic surfaces; no simulated sensor occlusion or noise.",
        "floor": {"label": 0, "name": "floor", "min": [-3.0, -3.0, 0.0],
                  "max": [3.0, 3.0, 0.0]},
        "tabletop": {"label": 1, "name": "tabletop", "min": [-1.5, -1.0, 1.1],
                     "max": [1.5, 1.0, 1.2]},
        "legs": legs,
        "dimensions": {"table_length": 3.0, "table_width": 2.0, "table_height": 1.2,
                       "tabletop_thickness": 0.1, "clear_height": 1.1,
                       "leg_width": 0.12, "gap_between_legs_x": 2.52,
                       "gap_between_legs_y": 1.52},
    }


def _axis(lo, hi, spacing):
    values = np.linspace(lo, hi, max(1, int(np.ceil((hi - lo) / spacing))) + 1)
    return np.unique(np.append(values, 0.0)) if lo < 0 < hi else values


def _inside_legs_xy(points, legs, tolerance=0.0):
    mask = np.zeros(len(points), dtype=bool)
    for leg in legs:
        lo, hi = np.asarray(leg["min"]), np.asarray(leg["max"])
        mask |= np.all((points[:, :2] >= lo[:2] - tolerance)
                       & (points[:, :2] <= hi[:2] + tolerance), axis=1)
    return mask


def _face(lo, hi, fixed_axis, side, spacing):
    free = [axis for axis in range(3) if axis != fixed_axis]
    a, b = np.meshgrid(*[_axis(lo[k], hi[k], spacing) for k in free], indexing="ij")
    result = np.empty((a.size, 3), dtype=np.float64)
    result[:, fixed_axis] = (lo, hi)[side][fixed_axis]
    result[:, free[0]], result[:, free[1]] = a.ravel(), b.ravel()
    return result


def make_cloud(spacing=0.04):
    if not np.isfinite(spacing) or spacing <= 0:
        raise ValueError("spacing must be finite and positive")
    scene = geometry()
    parts, labels = [], []
    for box in [scene["floor"], scene["tabletop"], *scene["legs"]]:
        lo, hi = np.asarray(box["min"]), np.asarray(box["max"])
        if box["label"] == 0:
            faces = [_face(lo, hi, 2, 0, spacing)]
        else:
            faces = [_face(lo, hi, axis, side, spacing)
                     for axis in range(3) for side in range(2)
                     if not (box["label"] >= 2 and axis == 2)]
        cloud = np.concatenate(faces)
        # Contact faces beneath legs and between tabletop/legs are not exposed.
        if box["label"] in (0, 1):
            contact = np.isclose(cloud[:, 2], 0.0 if box["label"] == 0 else 1.1)
            cloud = cloud[~(contact & _inside_legs_xy(cloud, scene["legs"]))]
        parts.append(cloud)
        labels.append(np.full(len(cloud), box["label"], dtype=np.uint8))
    points = np.concatenate(parts).astype("<f4")
    labels = np.concatenate(labels)
    points, indices = np.unique(points, axis=0, return_index=True)
    labels = labels[indices]
    return points, labels, scene


def validate_cloud(points, labels, scene):
    """Check the open volume as well as all component surface memberships."""
    eps = 2e-6
    assert points.ndim == 2 and points.shape[1] == 3
    assert labels.shape == (len(points),) and np.isfinite(points).all()
    assert len(np.unique(points, axis=0)) == len(points)
    assert set(labels.tolist()) == set(range(6))
    under = (points[:, 2] > eps) & (points[:, 2] < 1.1 - eps)
    forbidden = under & ~_inside_legs_xy(points, scene["legs"], eps)
    assert not forbidden.any(), "Found material in the clear space under the table"
    for box in [scene["floor"], scene["tabletop"], *scene["legs"]]:
        p = points[labels == box["label"]]
        lo, hi = np.asarray(box["min"]), np.asarray(box["max"])
        assert ((p >= lo - eps) & (p <= hi + eps)).all()
        assert np.any(np.isclose(p, lo, atol=eps, rtol=0)
                      | np.isclose(p, hi, atol=eps, rtol=0), axis=1).all()
    floor = points[labels == 0]
    assert np.any(np.all(np.isclose(floor, [0, 0, 0], atol=eps, rtol=0), axis=1))
    return {"forbidden_points_under_table": int(forbidden.sum()),
            "duplicate_points": 0, "finite_points": True,
            "floor_under_table_present": True, "all_points_on_component_surfaces": True}


def write_pcd(path, points, labels):
    # LABEL is optional in the existing XYZ reader; retain it for other tools.
    packed = np.empty(len(points), dtype=[("x", "<f4"), ("y", "<f4"),
                                         ("z", "<f4"), ("label", "<u4")])
    for index, name in enumerate(("x", "y", "z")):
        packed[name] = points[:, index]
    packed["label"] = labels
    header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
              "FIELDS x y z label\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n"
              f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {len(points)}\nDATA binary\n")
    path.write_bytes(header.encode("ascii") + packed.tobytes())


def export(output=DEFAULT_OUTPUT, spacing=0.04):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    points, labels, scene = make_cloud(spacing)
    checks = validate_cloud(points, labels, scene)
    write_pcd(output / "large_table.pcd", points, labels)
    np.savez_compressed(output / "large_table.npz", points=points, labels=labels)
    np.column_stack([points, labels]).astype("<f4").tofile(output / "points.bin")
    scene.update({"point_count": len(points), "max_surface_spacing_m": spacing,
                  "point_counts_by_label": {str(k): int(np.sum(labels == k)) for k in range(6)},
                  "point_buffer": {"url": "points.bin", "dtype": "little-endian float32",
                                   "columns": ["x", "y", "z", "label"]},
                  "checks": checks,
                  "pcd_sha256": hashlib.sha256((output / "large_table.pcd").read_bytes()).hexdigest()})
    (output / "scene.json").write_text(json.dumps(scene, indent=2, ensure_ascii=False) + "\n")
    assets = ROOT / "visualization/table"
    for name in ("index.html", "viewer.js", "viewer.css"):
        shutil.copyfile(assets / name, output / name)
    vendor_source = ROOT.parent / "m20_3d_planner/visualization/assets/vendor"
    (output / "vendor").mkdir(exist_ok=True)
    for name in ("three.module.min.js", "OrbitControls.js", "LICENSE.txt"):
        shutil.copyfile(vendor_source / name, output / "vendor" / name)
    return scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--spacing", type=float, default=0.04)
    args = parser.parse_args()
    scene = export(args.output, args.spacing)
    print(json.dumps({"output": str(args.output), "points": scene["point_count"],
                      "dimensions": scene["dimensions"], "checks": scene["checks"]},
                     indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

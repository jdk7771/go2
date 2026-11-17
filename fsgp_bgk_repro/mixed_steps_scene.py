"""Deterministic mixed-height stairs with exposed treads, risers and sides."""
import json
from pathlib import Path
import shutil

import numpy as np

from .composite_scene import write_pcd

ROOT = Path(__file__).resolve().parent
WEB = ROOT / "outputs/visualization"
OUTPUT = WEB / "mixed-steps"
BREAKS = np.array([-3., -2.5, -2., -1.5, -1., -.5, .5, 1.1, 1.7, 2.3])
HEIGHTS = np.array([0., .08, .26, .38, .68, .78, 1., .75, .65, .25, 0.])
HALF_WIDTH = 1.5


def axis(low, high, spacing):
    n = max(1, int(np.ceil((high-low)/spacing-1e-9)))
    values = np.linspace(low, high, n+1)
    return np.unique(np.append(values, 0.)) if low < 0 < high else values


def build(spacing=.04):
    if not np.isfinite(spacing) or not 0 < spacing <= .08:
        raise ValueError("spacing must be in (0, 0.08] m")
    pieces, parts, bands = [], [], []

    def add(xyz, part, band=-1):
        pieces.append(xyz)
        parts.append(np.full(len(xyz), part, dtype=np.uint8))
        bands.append(np.full(len(xyz), band, dtype=np.int16))

    x, y = np.meshgrid(axis(-4, 4, spacing), axis(-3, 3, spacing))
    ground = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    hidden = ((ground[:, 0] > BREAKS[0]+1e-8) & (ground[:, 0] < BREAKS[-1]-1e-8)
              & (np.abs(ground[:, 1]) < HALF_WIDTH-1e-8))
    add(ground[~hidden], 0)
    for band in range(1, len(HEIGHTS)-1):
        lo, hi, height = BREAKS[band-1], BREAKS[band], HEIGHTS[band]
        x, y = np.meshgrid(axis(lo, hi, spacing), axis(-HALF_WIDTH, HALF_WIDTH, spacing))
        add(np.column_stack((x.ravel(), y.ravel(), np.full(x.size, height))), 1, band)
        x, z = np.meshgrid(axis(lo, hi, spacing), axis(0, height, spacing))
        for side, part in ((-HALF_WIDTH, 3), (HALF_WIDTH, 4)):
            add(np.column_stack((x.ravel(), np.full(x.size, side), z.ravel())), part, band)
    for i, edge in enumerate(BREAKS):
        low, high = sorted(HEIGHTS[i:i+2])
        y, z = np.meshgrid(axis(-HALF_WIDTH, HALF_WIDTH, spacing), axis(low, high, spacing))
        add(np.column_stack((np.full(y.size, edge), y.ravel(), z.ravel())), 2, i)
    points = np.concatenate(pieces).astype("<f4")
    part_ids, band_ids = np.concatenate(parts), np.concatenate(bands)
    # Collapse numeric seam coincidences before the float32 final cast.
    _, keep = np.unique(np.round(points.astype(float), 6), axis=0, return_index=True)
    keep.sort()
    points, part_ids, band_ids = points[keep], part_ids[keep], band_ids[keep]
    risers = [dict(index=i+1, x_m=float(x), from_height_m=float(HEIGHTS[i]),
                   to_height_m=float(HEIGHTS[i+1]), signed_height_m=float(HEIGHTS[i+1]-HEIGHTS[i]),
                   height_cm=float(round(abs(HEIGHTS[i+1]-HEIGHTS[i])*100, 6)))
              for i, x in enumerate(BREAKS)]
    edges = np.r_[-4., BREAKS, 4.]
    profile = [dict(x_min_m=float(edges[i]), x_max_m=float(edges[i+1]), height_m=float(z))
               for i, z in enumerate(HEIGHTS)]
    # Check exposure and geometry independently of the fit, never pass labels
    # or heights into the neighbor selection or polynomial solver.
    assert np.isfinite(points).all()
    assert len(np.unique(points, axis=0)) == len(points)
    assert np.allclose(points.min(0), [-4, -3, 0]) and np.allclose(points.max(0), [4, 3, 1])
    ground = points[part_ids == 0]
    assert not np.any((ground[:, 0] > -3+1e-6) & (ground[:, 0] < 2.3-1e-6)
                      & (np.abs(ground[:, 1]) < 1.5-1e-6))
    for i, edge in enumerate(BREAKS):
        wall = points[(part_ids == 2) & (band_ids == i)]
        # Corners can have been retained under another part during seam dedup.
        assert len(wall) > 0 and np.allclose(wall[:, 0], edge)
        assert np.max(wall[:, 2]) <= max(HEIGHTS[i:i+2])+1e-6
        assert np.min(wall[:, 2]) >= min(HEIGHTS[i:i+2])-1e-6
        assert np.isclose(points[:, 0], edge).any()
    meta = dict(name="高低组合多级台阶", spacing_m=spacing, raw_points=len(points),
                bounds=[points.min(0).tolist(), points.max(0).tolist()], ground_size_m=[8, 6],
                stair_width_m=3., maximum_height_m=1., riser_count=10, ascent_count=6,
                descent_count=4, risers=risers, profile_segments=profile,
                parts={str(i):name for i, name in enumerate(("地面", "踏面", "台阶立面", "南侧壁", "北侧壁"))},
                part_counts={str(i):int((part_ids == i).sum()) for i in range(5)},
                sampling="Complete exposed surfaces, no added noise or simulated occlusion; hidden ground below the solid stair block is omitted.",
                checks=dict(unique_xyz=True, finite_xyz=True, all_ten_risers_sampled=True,
                            six_ascending_four_descending=True, hidden_ground_removed=True,
                            height_range_0_to_1_m=True, semantic_ids_not_fit_inputs=True))
    return points, part_ids, band_ids, meta


def export(output=OUTPUT, spacing=.04):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    points, parts, bands, meta = build(spacing)
    np.savez_compressed(output / "mixed_steps.npz", points=points, parts=parts, band_ids=bands)
    write_pcd(output / "mixed_steps.pcd", points, parts, np.maximum(bands, 0),
              np.full(len(points), 3, dtype=np.uint8), description="Mixed-height stairs, exposed surfaces")
    (output / "scene.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2)+"\n")
    (output / "README.md").write_text(
        "# 高低组合多级台阶\n\n8 × 6 m 场景，台阶宽 3 m、最高 1 m；6 级上升和 4 级下降。\n\n"
        "上升高差：8、18、12、30、10、22 cm；下降高差：25、10、40、25 cm。\n\n"
        "包含踏面、立面和两侧壁，不包含实体台阶内部的地面。标记仅作导出说明，不参与拟合。\n\n"
        "查看：http://127.0.0.1:42482/?scene=steps&view=overall&color=uniform\n")
    return meta


def archive_previous():
    old, archive = WEB / "scene-fits/steps", WEB / "scene-fits/steps-single"
    if archive.exists() or not (old / "data.json").exists():
        return
    meta = json.loads((old / "data.json").read_text())
    assert meta["source"] == {"kind":"generated", "generator":"experiment.make_scene", "scene":"steps"}
    shutil.copytree(old, archive)
    meta.update(id="steps-single", name="原单级台阶（历史）")
    (archive / "data.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2)+"\n")


if __name__ == "__main__":
    archive_previous()
    meta = export()
    print(json.dumps({k:meta[k] for k in ("name", "raw_points", "riser_count", "maximum_height_m")}, ensure_ascii=False))

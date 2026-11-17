"""Three standalone open-ended corridors, with only a small shared floor.

Clearance is measured from the floor (z=0) to the roof underside. These scenes
are independent of the single hill/forest/table composite scene.
"""

import argparse
import json
import shutil
from pathlib import Path

import numpy as np

from .composite_scene import ROOT, digest, grid, write_pcd
from .io import read_cloud
from .table_scene import _face

WEB = ROOT / "outputs/visualization"
CLEARANCES = (.4, .6, .8)
NAMES = ["地面", "顶板上下面", "两侧墙体"]
COLORS = ["#b9b3a0", "#d48141", "#688598"]


def scene_id(clearance):
    return f"corridor-{int(round(clearance * 100)):03d}"


def box_surface(lo, hi, spacing, axes=(0, 1, 2)):
    return np.concatenate([_face(np.asarray(lo), np.asarray(hi), axis, side, spacing)
                           for axis in axes for side in (0, 1)])


def build(clearance, spacing=.04):
    if clearance not in CLEARANCES:
        raise ValueError("Supported clearances are 0.4, 0.6 and 0.8 metres")
    if not np.isfinite(spacing) or not 0 < spacing <= .1:
        raise ValueError("spacing must be in (0, 0.1] metres")
    points, labels, parts, origins, input_ids = [], [], [], [], []

    def add(p, label, part=0, origin=3):
        points.append(p)
        labels.append(np.full(len(p), label, dtype=np.uint8))
        parts.append(np.full(len(p), part, dtype=np.uint8))
        origins.append(np.full(len(p), origin, dtype=np.uint8))
        input_ids.append(np.full(len(p), -1, dtype=np.int32))

    # Corridor runs along X. Its two ends remain open. The interior width is
    # 2 m, not the outside width of the roof or walls (2.24 m).
    half_length, half_width, thickness = 2., 1., .12
    xy = grid(-3, 3, -2, 2, spacing)
    contact = (np.abs(xy[:, 0]) <= half_length + 1e-8)
    contact &= (np.abs(xy[:, 1]) >= half_width - 1e-8) & (np.abs(xy[:, 1]) <= half_width + thickness + 1e-8)
    xy = xy[~contact]
    add(np.column_stack((xy, np.zeros(len(xy)))), 0)

    roof_min = [-half_length, -half_width - thickness, clearance]
    roof_max = [half_length, half_width + thickness, clearance + thickness]
    # Sample a local 12 cm slab, then translate; roof sample layout is identical
    # at all three heights, avoiding float-ceil differences in its thickness.
    roof = box_surface([-half_length, -half_width - thickness, 0.],
                       [half_length, half_width + thickness, thickness], spacing)
    roof[:, 2] += clearance
    covered_contact = np.isclose(roof[:, 2], clearance, atol=1e-9, rtol=0)
    covered_contact &= np.abs(roof[:, 1]) > half_width + 1e-9
    roof = roof[~covered_contact]
    add(roof, 1, part=1, origin=4)
    walls = []
    for part, y0, y1, name in [(2, -1.12, -1., "南侧墙"), (3, 1., 1.12, "北侧墙")]:
        lo, hi = [-half_length, y0, 0.], [half_length, y1, clearance]
        # Only vertical wall faces are exposed. Bottom and top contact faces
        # lie inside the floor/roof and must not be treated as free surfaces.
        wall = box_surface(lo, hi, spacing, axes=(0, 1))
        add(wall, 2, part=part, origin=4)
        walls.append({"part": part, "name": name, "min": lo, "max": hi})
    p = np.concatenate(points).astype("<f4")
    p, unique = np.unique(p, axis=0, return_index=True)
    lab, part, origin, ids = [np.concatenate(v)[unique] for v in (labels, parts, origins, input_ids)]
    eps = 2e-6
    interior = (np.abs(p[:, 0]) <= half_length + eps) & (np.abs(p[:, 1]) < half_width - eps)
    interior &= (p[:, 2] > eps) & (p[:, 2] < clearance - eps)
    assert not np.any(interior), "Corridor interior or open ends were accidentally filled"
    assert np.isfinite(p).all()
    # Both the center floor and the center roof underside/upper side exist.
    center_z = np.sort(p[np.linalg.norm(p[:, :2], axis=1) < eps, 2])
    np.testing.assert_allclose(center_z, [0., clearance, clearance + thickness], atol=eps)
    return p, lab, part, origin, ids, {
        "name": scene_id(clearance), "title": f"净空 {clearance:.2f} m 通道",
        "kind": "covered_corridor", "scope": "standalone", "units": "m", "up_axis": "z",
        "description": "Standalone constructed open-ended covered corridor and 6 x 4 m floor; 4 m length, 2 m interior width; only corridor height differs between variants. Not a simulated LiDAR scan.",
        "point_count": len(p), "nominal_size_m": [6, 4],
        "bounds_m": [p.min(0).tolist(), p.max(0).tolist()], "generated_spacing_m": spacing,
        "sources": [{"kind": "generated", "generator": "corridor_scenes.py", "parts": NAMES}],
        "corridor": {"axis": "x", "x_range_m": [-2, 2], "y_inner_range_m": [-1, 1],
                     "length_m": 4., "inner_width_m": 2., "clear_height_m": clearance,
                     "floor_z_m": 0., "roof_thickness_m": thickness, "wall_thickness_m": thickness,
                     "roof_top_z_m": clearance + thickness,
                     "roof": {"part": 1, "min": roof_min, "max": roof_max}, "walls": walls,
                     "open_ends": ["x=-2 m", "x=+2 m"]},
        "layout": {"floor": {"x": [-3, 3], "y": [-2, 2]}, "corridor_center": [0, 0, 0]},
        "layers": [{"id": i, "name": name, "color": COLORS[i], "count": int((lab == i).sum())}
                   for i, name in enumerate(NAMES)],
        "variants": [{"id": scene_id(h), "clear_height_m": h} for h in CLEARANCES],
        "checks": {"finite_and_unique": True, "empty_interior_and_open_ends": True,
                   "center_floor_and_roof_z_m": center_z.tolist(), "standalone_corridor_and_floor_only": True},
    }


def comparison_figure(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    from matplotlib.patches import Rectangle
    font = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.3), sharex=True, sharey=True)
    for ax, height in zip(axes, CLEARANCES):
        ax.add_patch(Rectangle((-1.5, -.055), 3, .055, facecolor="#b9b3a0"))
        for left in [-1.12, 1.]:
            ax.add_patch(Rectangle((left, 0), .12, height, facecolor=COLORS[2]))
        ax.add_patch(Rectangle((-1.12, height), 2.24, .12, facecolor=COLORS[1]))
        ax.annotate("", (-.55, height), (-.55, 0), arrowprops={"arrowstyle": "<->", "color": "#365d49"})
        ax.text(-.45, height / 2, f"{height:.2f} m", va="center", fontsize=12, color="#365d49")
        ax.annotate("", (-1, -.19), (1, -.19), arrowprops={"arrowstyle": "<->", "color": "#66786b"})
        ax.text(0, -.28, "2.00 m", ha="center", fontsize=11, color="#66786b")
        ax.set_title(f"净空 {height:.2f} 米", fontproperties=font, fontsize=15)
        ax.set_xlim(-1.6, 1.6); ax.set_ylim(-.35, 1.15); ax.set_aspect("equal")
        ax.set_xlabel("Y / m"); ax.grid(alpha=.12); ax.spines[["top", "right"]].set_visible(False)
    axes[0].set_ylabel("Z / m")
    fig.suptitle("三个通道横截面：相同净宽、相同地面、仅改变净高（同一比例尺）", fontproperties=font, fontsize=14)
    fig.tight_layout(); fig.savefig(path, dpi=170, bbox_inches="tight"); plt.close(fig)


def audit_saved(output):
    """Re-read delivered files and check only the three requested parts exist."""
    meta = json.loads((output / "scene.json").read_text())
    with np.load(output / "corridor_scene.npz", allow_pickle=False) as data:
        p, labels = data["points"], data["labels"]
        assert np.array_equal(read_cloud(output / "corridor_scene.pcd"), p)
        binary = np.fromfile(output / "points.bin", dtype="<f4").reshape(-1, 4)
        assert np.array_equal(binary[:, :3], p) and np.array_equal(binary[:, 3], labels)
        assert set(labels.tolist()) == {0, 1, 2}
        assert set(data["origins"].tolist()) == {3, 4}
        assert np.all(data["input_ids"] == -1)
        assert np.all(np.abs(p[:, 0]) <= 3 + 1e-6) and np.all(np.abs(p[:, 1]) <= 2 + 1e-6)
        assert np.all(np.abs(p[labels == 0, 2]) < 1e-7)
        assert np.all(data["parts"][labels == 1] == 1)
        assert set(data["parts"][labels == 2].tolist()) == {2, 3}
        assert len(p) == len(np.unique(p, axis=0)) == meta["point_count"]
    result = {"passed": True, "point_count": len(p), "standalone_corridor_and_floor_only": True,
              "pcd_and_browser_buffer_exact": True, "duplicate_points": 0, **meta["checks"]}
    (output / "audit.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    return result


def export_all(output_root=WEB, spacing=.04):
    output_root = Path(output_root)
    records = []
    reference_ground = None
    for height in CLEARANCES:
        output = output_root / scene_id(height)
        output.mkdir(parents=True, exist_ok=True)
        p, labels, parts, origins, ids, meta = build(height, spacing)
        ground = p[labels == 0]
        if reference_ground is None:
            reference_ground = ground
        else:
            assert np.array_equal(ground, reference_ground), "Variants have different ground/terrain"
        write_pcd(output / "corridor_scene.pcd", p, labels, parts, origins,
                  description="Standalone open-ended corridor")
        np.savez_compressed(output / "corridor_scene.npz", points=p, labels=labels, parts=parts,
                            origins=origins, input_ids=ids)
        np.column_stack((p, labels)).astype("<f4").tofile(output / "points.bin")
        meta["pcd_sha256"] = digest(output / "corridor_scene.pcd")
        meta["checks"]["identical_ground_across_variants"] = True
        (output / "scene.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n")
        for name in ("index.html", "viewer.js", "viewer.css"):
            shutil.copyfile(ROOT / "visualization/corridor" / name, output / name)
        (output / "vendor").mkdir(exist_ok=True)
        for name in ("three.module.min.js", "OrbitControls.js", "LICENSE.txt"):
            shutil.copyfile(WEB / "large-table/vendor" / name, output / "vendor" / name)
        comparison_figure(output / "clearance-comparison.png")
        (output / "README.md").write_text(
            f"# 净空 {height:.2f} 米独立通道\n\n只包含通道结构和 6 × 4 m 地面，{len(p):,} 点。"
            f"通道沿 X 方向，长 4 m、净宽 2 m、净高 {height:.2f} m，两端敞开。"
            f"地面 Z=0，顶板底面 Z={height:.2f} m，上表面 Z={height + .12:.2f} m。墙厚与板厚均为 0.12 m。\n\n"
            "三个场景的全部地面逐点一致，只改变通道净高；墙体接触地面的区域不采样内部接触面。"
            "图层为地面、顶板和两侧墙体，颜色表示部件。\n\n"
            "这是完整构造表面，尚未进行 LiDAR 遮挡/噪声模拟，也没有执行拟合或通行性判断。"
            "净高指地面至顶板底面的垂直距离，不是顶板上表面的高度。\n\n"
            "PCD 字段：x/y/z、label（0 地面、1 顶板、2 两侧墙）、part（0 地面，1 顶板，2 南墙，3 北墙）、"
            "origin（3 地面、4 通道结构）。NPZ 的 input_ids 全为 −1，表示生成点。\n"
        )
        audit = audit_saved(output)
        records.append({"id": scene_id(height), "clear_height_m": height,
                        "point_count": len(p), "checks": audit})
    if output_root.resolve() == WEB:
        home = output_root / "index.html"
        html = home.read_text()
        if 'name="go2-scene-fit-home"' not in html and 'id="corridor-scenes-entry"' not in html:
            links = " · ".join(f'<a href="./{scene_id(h)}/" style="color:#d8eefc;font-weight:bold">净空 {h:.2f} m →</a>' for h in CLEARANCES)
            entry = '<nav id="corridor-scenes-entry" style="padding:17px 24px;background:#294356;color:white;font:15px/1.8 sans-serif"><strong>三个独立通道场景 · 长 4 m、净宽 2 m</strong><br>' + links + '</nav>'
            home.write_text(html.replace("<body>", "<body>\n" + entry, 1))
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=WEB)
    parser.add_argument("--spacing", type=float, default=.04)
    args = parser.parse_args()
    print(json.dumps(export_all(args.output_root, args.spacing), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

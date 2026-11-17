"""Join the released hill/forest clouds and the existing open-underneath table.

Original terrain points are only translated. Added floor and transition strips
are explicitly identified; this is a constructed scene, not a recorded scan.
"""

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .io import read_cloud
from .table_scene import geometry, _inside_legs_xy

ROOT = Path(__file__).resolve().parent
DEFAULT_OUTPUT = ROOT / "outputs/visualization/combined-scene"
LAYER_NAMES = ["丘陵", "森林", "中央空地", "过渡坡", "桌面与桌底", "四条桌腿", "丘陵边沿补地"]
LAYER_COLORS = ["#78934c", "#30796a", "#b9b3a0", "#b29269", "#de8840", "#8e522a", "#a4aa77"]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def grid(x0, x1, y0, y1, spacing):
    x = np.linspace(x0, x1, int(np.ceil((x1 - x0) / spacing)) + 1)
    y = np.linspace(y0, y1, int(np.ceil((y1 - y0) / spacing)) + 1)
    xx, yy = np.meshgrid(x, y, indexing="xy")
    return np.column_stack((xx.ravel(), yy.ravel()))


def boundary_ground(tree, points, xy, low_envelope=False):
    """IDW boundary height; forest uses the lowest local third to avoid crowns.

    This is a ground estimate for newly generated connectors only. No terrain
    source point is altered, filtered by height, or reduced to a height field.
    """
    distance, ids = tree.query(xy, k=min(30, len(points)))
    if ids.ndim == 1:
        ids, distance = ids[:, None], distance[:, None]
    z = points[ids, 2]
    weight = 1 / np.maximum(distance, 0.005) ** 2
    if low_envelope:
        weight *= z <= np.quantile(z, 1 / 3, axis=1)[:, None] + 1e-9
    return np.sum(z * weight, axis=1) / weight.sum(axis=1)


def write_pcd(path, points, labels, parts, origins, description="Composite hill / forest / table"):
    data = np.empty(len(points), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                       ("label", "<u4"), ("part", "<u4"), ("origin", "<u4")])
    for i, axis in enumerate("xyz"):
        data[axis] = points[:, i]
    data["label"], data["part"], data["origin"] = labels, parts, origins
    header = (f"# .PCD v0.7 - {description}, units: metres\n"
              "VERSION 0.7\nFIELDS x y z label part origin\nSIZE 4 4 4 4 4 4\n"
              "TYPE F F F U U U\nCOUNT 1 1 1 1 1 1\n"
              f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
              f"POINTS {len(points)}\nDATA binary\n")
    path.write_bytes(header.encode("ascii") + data.tobytes())


def build(spacing=0.04):
    if not np.isfinite(spacing) or spacing <= 0 or spacing > 0.25:
        raise ValueError("spacing must be in (0, 0.25] metres")
    paths = [ROOT / "data/hill.pcd",
             ROOT.parent / "quadric_lidar_repro/data/fsgp_terrain/forest.pcd",
             ROOT / "outputs/visualization/large-table/large_table.npz"]
    inputs = [read_cloud(paths[0]), read_cloud(paths[1])]
    with np.load(paths[2], allow_pickle=False) as data:
        inputs.append(data["points"].astype(float))
        table_labels = data["labels"].copy()
    translations = [np.array([-10., 0, 0]), np.array([11., 0, 0]), np.zeros(3)]
    hill, forest, table = [p + t for p, t in zip(inputs, translations)]
    points, labels, parts, origins, input_ids = [], [], [], [], []

    def add(p, layer, origin=3, part=0, ids=None):
        points.append(p)
        labels.append(np.broadcast_to(layer, (len(p),)).copy())
        parts.append(np.broadcast_to(part, (len(p),)).copy())
        origins.append(np.full(len(p), origin, dtype=np.uint8))
        input_ids.append(np.arange(len(p), dtype=np.int32) if ids is None and origin < 3
                         else np.full(len(p), -1, dtype=np.int32) if ids is None else ids)

    add(hill, 0, 0)
    add(forest, 1, 1)
    add(table, np.where(table_labels == 0, 2, np.where(table_labels == 1, 4, 5)), 2,
        part=table_labels)
    # Extend the existing 6 x 6 m table floor north and south to 6 x 12 m.
    xy = grid(-3, 3, -6, 6, spacing)
    xy = xy[np.abs(xy[:, 1]) > 3 + 1e-7]
    add(np.column_stack((xy, np.zeros(len(xy)))), 2)
    hill_tree, forest_tree = cKDTree(hill[:, :2]), cKDTree(forest[:, :2])
    # Extend the 10 m hill footprint to the common 12 m scene width. Heights
    # continue from its nearest edge; all original hill samples are retained.
    for y0, y1 in [(-6, -5), (5, 6)]:
        xy = grid(-15, -5, y0, y1, spacing)
        edge = xy.copy()
        edge[:, 1] = np.clip(edge[:, 1], -5, 5)
        z = boundary_ground(hill_tree, hill, edge)
        add(np.column_stack((xy, z)), 6)
    connectors = []
    for name, x0, x1, boundary_x, cloud, tree, low, terrain_at_left in [
        ("丘陵至空地", -5, -3, -5, hill, hill_tree, False, True),
        ("空地至森林", 3, 5, 5, forest, forest_tree, True, False),
    ]:
        xy = grid(x0, x1, -6, 6, spacing)
        boundary = np.column_stack((np.full(len(xy), boundary_x), xy[:, 1]))
        if terrain_at_left:
            boundary[:, 1] = np.clip(boundary[:, 1], -5, 5)
        edge_z = boundary_ground(tree, cloud, boundary, low_envelope=low)
        t = (xy[:, 0] - x0) / (x1 - x0)
        blend = t * t * (3 - 2 * t)
        z = edge_z * (1 - blend if terrain_at_left else blend)
        add(np.column_stack((xy, z)), 3)
        connectors.append({"name": name, "x_range_m": [x0, x1], "width_m": x1 - x0,
                           "height_range_m": [float(z.min()), float(z.max())],
                           "boundary_ground": "lowest-third IDW, 30 XY neighbors" if low else "IDW, 30 XY neighbors",
                           "blend": "smoothstep to zero-height clearing"})
    p = np.concatenate(points).astype("<f4")
    lab = np.concatenate(labels).astype(np.uint8)
    part = np.concatenate(parts).astype(np.uint8)
    origin = np.concatenate(origins)
    ids = np.concatenate(input_ids)
    # Retain the first occurrence so source geometry wins over generated floor
    # or connector samples at shared borders. Do not voxel-reduce source data.
    p, unique = np.unique(p, axis=0, return_index=True)
    lab, part, origin, ids = lab[unique], part[unique], origin[unique], ids[unique]
    source_records = []
    for i, (name, path, original, translation) in enumerate(zip(
            ["hill", "forest", "large_table"], paths, inputs, translations)):
        selected = origin == i
        expected = (original[ids[selected]] + translation).astype("<f4")
        assert np.array_equal(p[selected], expected), "Source geometry changed beyond translation"
        assert selected.sum() == len(np.unique((original + translation).astype("<f4"), axis=0))
        matrix = np.eye(4)
        matrix[:3, 3] = translation
        source_records.append({"id": i, "name": name, "path": str(path.relative_to(ROOT.parent)),
                               "sha256": digest(path), "input_points": len(original),
                               "retained_unique_points": int(selected.sum()),
                               "translation_m": translation.tolist(), "transform_4x4": matrix.tolist()})
    # The merged scene must preserve the open volume under the table and all
    # original parts, including the actual floor below the tabletop.
    under = (np.abs(p[:, 0]) < 1.5) & (np.abs(p[:, 1]) < 1)
    under &= (p[:, 2] > 2e-6) & (p[:, 2] < 1.1 - 2e-6)
    forbidden = under & ~_inside_legs_xy(p, geometry()["legs"], 2e-6)
    assert not forbidden.any(), "Under-table clearance was filled during composition"
    assert np.isfinite(p).all()
    assert set(part[origin == 2].tolist()) == set(range(6))
    return p, lab, part, origin, ids, {
        "name": "hill_forest_table", "units": "m", "up_axis": "z",
        "description": "Constructed composite; translated released terrain clouds plus existing synthetic table, generated clearing extensions and connectors. Not a LiDAR scan.",
        "point_count": len(p), "bounds_m": [p.min(0).tolist(), p.max(0).tolist()],
        "nominal_size_m": [32, 12], "generated_spacing_m": spacing,
        "sources": source_records, "connectors": connectors,
        "table": geometry(), "layers": [
            {"id": i, "name": name, "color": LAYER_COLORS[i], "count": int((lab == i).sum())}
            for i, name in enumerate(LAYER_NAMES)],
        "layout": {"hill": {"x": [-15, -5], "y": [-5, 5]},
                   "clearing": {"x": [-3, 3], "y": [-6, 6]},
                   "forest": {"x": [5, 17], "y": [-6, 6]},
                   "table_center": [0, 0, 0]},
        "checks": {"all_finite": True, "duplicate_points": 0,
                   "source_points_only_translated": True, "table_parts_preserved": True,
                   "forbidden_points_under_table": int(forbidden.sum()),
                   "floor_under_table_present": bool(np.any(np.all(np.isclose(p, [0, 0, 0], atol=2e-6), axis=1)))},
    }


def preview(output, points, labels):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    fig = plt.figure(figsize=(16, 6.5))
    for position, view, title in [(121, (42, -65), "丘陵 · 中央空地与桌子 · 森林"),
                                  (122, (28, -62), "桌子局部：桌面、桌底与四条腿")]:
        ax = fig.add_subplot(position, projection="3d")
        for i in range(7):
            mask = labels == i
            if position == 122:
                mask &= (np.abs(points[:, 0]) < 3) & (np.abs(points[:, 1]) < 3)
            p = points[mask]
            if len(p) > 60000:
                p = p[np.linspace(0, len(p) - 1, 60000, dtype=int)]
            ax.scatter(*p.T, s=.15 if position == 121 else .7, c=LAYER_COLORS[i],
                       alpha=.8, rasterized=True, linewidths=0)
        ax.view_init(*view)
        if position == 121:
            ax.set_xlim(-15, 17); ax.set_ylim(-6, 6); ax.set_zlim(0, 3)
            ax.set_box_aspect((32, 12, 4.5))
        else:
            ax.set_xlim(-3, 3); ax.set_ylim(-3, 3); ax.set_zlim(0, 1.5)
            ax.set_box_aspect((6, 6, 2.2))
        ax.set_title(title, fontproperties=font, fontsize=14)
        ax.set_xlabel("X / m"); ax.set_ylabel("Y / m"); ax.set_zlabel("Z / m")
    fig.tight_layout()
    fig.savefig(output / "overview.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def export(output=DEFAULT_OUTPUT, spacing=.04):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    p, labels, parts, origins, ids, scene = build(spacing)
    write_pcd(output / "combined_scene.pcd", p, labels, parts, origins)
    np.savez_compressed(output / "combined_scene.npz", points=p, labels=labels,
                        parts=parts, origins=origins, input_ids=ids)
    np.column_stack((p, labels)).astype("<f4").tofile(output / "points.bin")
    scene["point_buffer"] = {"url": "points.bin", "dtype": "little-endian float32", "columns": ["x", "y", "z", "label"]}
    scene["pcd_sha256"] = digest(output / "combined_scene.pcd")
    (output / "scene.json").write_text(json.dumps(scene, ensure_ascii=False, indent=2) + "\n")
    for name in ("index.html", "viewer.js", "viewer.css"):
        shutil.copyfile(ROOT / "visualization/composite" / name, output / name)
    (output / "vendor").mkdir(exist_ok=True)
    for name in ("three.module.min.js", "OrbitControls.js", "LICENSE.txt"):
        shutil.copyfile(ROOT / "outputs/visualization/large-table/vendor" / name, output / "vendor" / name)
    preview(output, p, labels)
    layers = "\n".join(f"| {layer['name']} | {layer['count']:,} |" for layer in scene["layers"])
    (output / "README.md").write_text(
        f"# 丘陵、森林、桌子拼接场景\n\n总计 {len(p):,} 点，名义范围 32 × 12 m，单位为米，Z 向上。\n\n"
        "丘陵全部点沿 X 平移 −10 m，森林全部点沿 X 平移 +11 m。两者不缩放、不改变高度。"
        "原有 3 × 2 m 桌子完整保留，桌面高 1.20 m、桌底高 1.10 m；中央空地扩展至 6 × 12 m。\n\n"
        "丘陵和空地之间、空地和森林之间各添加 2 m 过渡带。连接带使用边界附近 30 个 XY 邻点估计地面，"
        "森林取最低三分之一点的加权高度，随后 smoothstep 接到零高空地。丘陵南北各补 1 m 边沿地面。"
        "新增几何采样间隔为 4 cm（默认）；这只控制新增点，不降低原始点云密度。\n\n"
        "这是拼接构造场景，未模拟传感器遮挡或噪声，也未在此运行拟合、分割或可通行性算法。"
        "图层颜色表示点的来源/部件，不是算法分割结果。\n\n"
        "| 图层 | 点数 |\n|---|---:|\n" + layers + "\n\n"
        "PCD 包含 x/y/z、label（图层）、part（原桌子部件，其他为 0）、origin（0 丘陵、1 森林、2 原桌子、3 新增几何）。"
        "NPZ 另外保存 input_ids（在对应原始点云中的索引，新增点为 −1）。scene.json 保存输入哈希、刚体变换和校验结果。\n"
    )
    # Only add a navigation entry outside the existing embedded visualization.
    home = output.parent / "index.html"
    if output == DEFAULT_OUTPUT and home.exists():
        html = home.read_text()
        if 'id="combined-scene-entry"' not in html:
            entry = ('\n<nav id="combined-scene-entry" style="padding:17px 24px;background:#294637;color:white;font:15px/1.7 sans-serif">'
                     '<strong>新场景：丘陵 + 森林 + 大桌子 · 32 × 12 米</strong><br>'
                     '<a href="./combined-scene/" style="color:#d8f3b7;font-weight:bold">打开完整拼接点云 / 下载 PCD →</a></nav>')
            home.write_text(html.replace("<body>", "<body>" + entry, 1))
    return scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--spacing", type=float, default=.04)
    args = parser.parse_args()
    scene = export(args.output, args.spacing)
    print(json.dumps({"output": str(args.output), "points": scene["point_count"],
                      "bounds_m": scene["bounds_m"], "layers": scene["layers"],
                      "checks": scene["checks"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

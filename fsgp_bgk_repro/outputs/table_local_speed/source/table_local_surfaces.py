"""Seed-local K=30 quadratic fits and geometry-consistent surface assembly.

No global height histogram, height modes, layer count, known face heights or
scene labels enter the algorithm. Every original XYZ observation is a seed;
the height gate is relative to that seed. Surface components are an output.
This diagnostic is not a method from the FSGP paper.
"""

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree, Delaunay, QhullError

from .visualize_regression import clean_json

ROOT = Path(__file__).resolve().parent
PALETTE = ["#27896d", "#427ec4", "#df9735", "#9974bc", "#ba647e", "#52a8a5", "#b2a244"]


@dataclass(frozen=True)
class LocalConfig:
    neighbors: int = 30
    radius_m: float = .75
    seed_height_gate_m: float = .04
    rank_ratio: float = 1e-10
    relative_ridge: float = 1e-10
    max_fit_rmse_m: float = .02
    connect_radius_m: float = .35
    max_model_disagreement_m: float = .02
    max_normal_angle_deg: float = 15.
    min_component_points: int = 30
    patch_half_width_m: float = .12
    patch_grid_size: int = 5


def select_neighbors(points, config):
    """Intersection of radius and relative-height gates, then nearest K.

    Radius query first is only an efficient candidate search; height filtering
    still happens BEFORE taking K. Exactly K including the seed, or no fit.
    Original points with equal XY but different Z remain separate seeds.
    """
    k, n = config.neighbors, len(points)
    ids = np.full((n, k), -1, dtype=np.int32)
    counts = np.zeros(n, dtype=np.int32)
    candidates = cKDTree(points).query_ball_point(points, config.radius_m)
    for i, nearby in enumerate(candidates):
        eligible = np.asarray(nearby, dtype=np.int32)
        eligible = eligible[np.abs(points[eligible, 2]-points[i, 2]) <= config.seed_height_gate_m]
        counts[i] = len(eligible)
        if len(eligible) >= k:
            distance2 = np.sum((points[eligible]-points[i])**2, axis=1)
            order = np.argsort(distance2, kind="stable")[:k]
            ids[i] = eligible[order]
    return ids, counts


def evaluate_models(points, coefficient, seeds, xy):
    """Physical coefficients in seed-relative XY, seed-relative height."""
    delta = np.asarray(xy)-points[seeds, :2]
    u, v = delta[..., 0], delta[..., 1]
    c = coefficient[seeds]
    return (points[seeds, 2]+c[..., 0]+c[..., 1]*u+c[..., 2]*v+
            c[..., 3]*u*u+c[..., 4]*u*v+c[..., 5]*v*v)


def fit_seeds(points, config):
    ids, counts = select_neighbors(points, config)
    n = len(points)
    active = np.flatnonzero(ids[:, 0] >= 0)
    height = np.full(n, np.nan)
    coefficient = np.full((n, 6), np.nan)
    rmse = np.full(n, np.nan)
    full_rank = np.zeros(n, dtype=bool)
    if len(active):
        local = points[ids[active]]
        delta = local[:, :, :2]-points[active, None, :2]
        scale = np.maximum(np.sqrt(np.mean(np.sum(delta**2, axis=2), axis=1)), 1e-10)
        u, v = (delta/scale[:, None, None]).transpose(2, 0, 1)
        design = np.stack((np.ones_like(u), u, v, u*u, u*v, v*v), axis=2)
        gram = np.einsum("nki,nkj->nij", design, design)
        eigenvalues = np.linalg.eigvalsh(gram)
        full_rank[active] = eigenvalues[:, 0] > np.maximum(eigenvalues[:, -1], 1e-30)*config.rank_ratio
        target = local[:, :, 2]-points[active, None, 2]
        rhs = np.einsum("nki,nk->ni", design, target)
        ridge = config.relative_ridge*np.trace(gram, axis1=1, axis2=2)/6
        normalized = np.linalg.solve(gram+ridge[:, None, None]*np.eye(6), rhs)
        coefficient[active] = normalized/np.column_stack((np.ones(len(active)), scale, scale, scale**2, scale**2, scale**2))
        prediction = points[active, 2]+normalized[:, 0]
        residual = np.einsum("nki,ni->nk", design, normalized)-target
        rmse[active] = np.sqrt(np.mean(residual**2, axis=1))
        valid = (full_rank[active] & np.isfinite(prediction) &
                 (np.abs(prediction-points[active, 2]) <= config.seed_height_gate_m) &
                 (rmse[active] <= config.max_fit_rmse_m))
        height[active[valid]] = prediction[valid]
    normal = np.column_stack((-coefficient[:, 1], -coefficient[:, 2], np.ones(n)))
    normal /= np.linalg.norm(normal, axis=1)[:, None]
    return dict(neighbor_ids=ids, candidate_counts=counts, coefficient=coefficient,
                height=height, rmse=rmse, full_rank=full_rank, normal=normal)


def compatible_edges(points, fits, config):
    valid = np.flatnonzero(np.isfinite(fits["height"]))
    xyz = points[valid].copy()
    xyz[:, 2] = fits["height"][valid]
    local_pairs = cKDTree(xyz).query_pairs(config.connect_radius_m, output_type="ndarray")
    if not len(local_pairs):
        return np.empty((0, 2), dtype=np.int32)
    pairs = valid[local_pairs]
    a, b = pairs.T
    ab = np.abs(evaluate_models(points, fits["coefficient"], a, points[b, :2])-fits["height"][b])
    ba = np.abs(evaluate_models(points, fits["coefficient"], b, points[a, :2])-fits["height"][a])
    normal_dot = np.einsum("ni,ni->n", fits["normal"][a], fits["normal"][b])
    keep = (np.maximum(ab, ba) <= config.max_model_disagreement_m) & (normal_dot >= np.cos(np.deg2rad(config.max_normal_angle_deg)))
    return pairs[keep].astype(np.int32)


def assemble(points, fits, config):
    pairs = compatible_edges(points, fits, config)
    rows = np.r_[pairs[:, 0], pairs[:, 1]]
    cols = np.r_[pairs[:, 1], pairs[:, 0]]
    graph = coo_matrix((np.ones(len(rows), dtype=bool), (rows, cols)), shape=(len(points), len(points))).tocsr()
    count, raw_labels = connected_components(graph, directed=False)
    valid = np.isfinite(fits["height"])
    sizes = np.bincount(raw_labels[valid], minlength=count)
    groups = np.flatnonzero(sizes >= config.min_component_points)
    # Sort by support count, never by known height/face identity.
    groups = groups[np.argsort(-sizes[groups], kind="stable")]
    labels = np.full(len(points), -1, dtype=np.int32)
    components = []
    for index, group in enumerate(groups):
        ids = np.flatnonzero(valid & (raw_labels == group))
        labels[ids] = index
        try:
            triangulation = Delaunay(points[ids, :2])
        except QhullError:
            components.append(dict(point_ids=ids, triangles=np.empty((0, 3), dtype=np.int32), triangulation=None, kept_simplex=np.empty(0, dtype=bool)))
            continue
        triangles = ids[triangulation.simplices]
        keep = np.ones(len(triangles), dtype=bool)
        for a, b in ((0, 1), (1, 2), (2, 0)):
            # Every triangle edge must pass the actual local-model graph test.
            keep &= np.asarray(graph[triangles[:, a], triangles[:, b]]).ravel().astype(bool)
        centroid = points[triangles, :2].mean(axis=1)
        z_centroid = fits["height"][triangles].mean(axis=1)
        for corner in range(3):
            prediction = evaluate_models(points, fits["coefficient"], triangles[:, corner], centroid)
            keep &= np.abs(prediction-z_centroid) <= config.max_model_disagreement_m
        components.append(dict(point_ids=ids, triangles=triangles[keep].astype(np.int32),
                               triangulation=triangulation, kept_simplex=keep))
    return labels, components, pairs, dict(raw_components=int((sizes > 0).sum()),
                 small_component_valid_points=int(sizes[(sizes > 0) & (sizes < config.min_component_points)].sum()))


def reconstruct(points, config=LocalConfig()):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("Expected finite N×3 XYZ")
    fits = fit_seeds(points, config)
    labels, components, pairs, summary = assemble(points, fits, config)
    return dict(fits=fits, labels=labels, components=components, edges=pairs, summary=summary)


def component_profile(points, fits, component, xy):
    values = np.full(len(xy), np.nan)
    tri = component["triangulation"]
    if tri is None:
        return values
    simplex = tri.find_simplex(xy)
    active = np.flatnonzero(simplex >= 0)
    active = active[component["kept_simplex"][simplex[active]]]
    transform = tri.transform[simplex[active]]
    first = np.einsum("nij,nj->ni", transform[:, :2], xy[active]-transform[:, 2])
    barycentric = np.column_stack((first, 1-first.sum(axis=1)))
    ids = component["point_ids"][tri.simplices[simplex[active]]]
    values[active] = np.sum(barycentric*fits["height"][ids], axis=1)
    return values


def patch_mesh(points, fits, config):
    """Render genuine local polynomial samples, without cross-seed stitching."""
    axis = np.linspace(-config.patch_half_width_m, config.patch_half_width_m, config.patch_grid_size)
    x, y = np.meshgrid(axis, axis)
    offsets = np.column_stack((x.ravel(), y.ravel()))
    a = (np.arange(len(axis)-1)[:, None]*len(axis)+np.arange(len(axis)-1)).ravel()
    face = np.concatenate((np.column_stack((a, a+1, a+len(axis))),
                           np.column_stack((a+1, a+len(axis)+1, a+len(axis)))))
    vertices, triangles, owners = [], [], []
    for seed in np.flatnonzero(np.isfinite(fits["height"])):
        xy = points[seed, :2]+offsets
        z = evaluate_models(points, fits["coefficient"], np.full(len(xy), seed), xy)
        try:
            inside = Delaunay(points[fits["neighbor_ids"][seed], :2]).find_simplex(xy) >= 0
        except QhullError:
            continue
        inside &= np.abs(z-points[seed, 2]) <= config.seed_height_gate_m
        selected = face[np.all(inside[face], axis=1)]
        start = len(vertices)
        vertices.extend(np.column_stack((xy, z)))
        owners.extend([seed]*len(xy))
        triangles.extend(selected+start)
    return np.asarray(vertices).reshape(-1, 3), np.asarray(triangles, dtype=np.int32).reshape(-1, 3), np.asarray(owners, dtype=np.int32)


def audit(points, result, config):
    fit = result["fits"]
    seeds = np.flatnonzero(np.isfinite(fit["height"]))
    ids = fit["neighbor_ids"][seeds]
    assert ids.shape == (len(seeds), config.neighbors) and np.all(ids >= 0)
    dz = np.abs(points[ids, 2]-points[seeds, None, 2])
    distance = np.linalg.norm(points[ids]-points[seeds, None, :], axis=2)
    assert np.all(dz <= config.seed_height_gate_m+1e-12)
    assert np.all(distance <= config.radius_m+1e-12)
    total = 0
    for i, component in enumerate(result["components"]):
        faces = component["triangles"]
        assert np.all(result["labels"][faces] == i)
        assert np.all(np.isfinite(fit["height"][faces]))
        total += len(faces)
    return dict(passed=True, input_points=len(points), seed_queries=len(points),
                valid_seed_fits=len(seeds), component_count=len(result["components"]),
                triangles=total, exact_neighbors=config.neighbors,
                max_selected_height_delta_m=float(dz.max()) if dz.size else None,
                max_selected_distance_m=float(distance.max()) if distance.size else None,
                global_height_histogram_used=False, preset_surface_count=None,
                scene_labels_or_known_heights_used_for_fitting=False)


def figures(points, result, patches, data, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    font = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    plt.rcParams.update({"font.family":font.get_name(), "axes.unicode_minus":False})
    fit, labels = result["fits"], result["labels"]
    vertices, triangles, owners = patches
    fig = plt.figure(figsize=(16, 5.8), facecolor="#f4f7fb")
    axes = [fig.add_subplot(1, 3, i+1, projection="3d") for i in range(3)]
    axes[0].scatter(*points.T, s=2.4, c="#80909f", depthshade=False)
    # Independent fitting precedes grouping. Use one color here, rather than
    # back-coloring patches with the subsequent connected-component labels.
    axes[1].add_collection3d(Poly3DCollection(vertices[triangles], facecolor="#5291bc", linewidth=0))
    for i, component in enumerate(result["components"]):
        color = PALETTE[i % len(PALETTE)]
        xyz = points.copy(); xyz[:, 2] = fit["height"]
        axes[2].add_collection3d(Poly3DCollection(xyz[component["triangles"]], facecolor=color, linewidth=0))
    for ax, title in zip(axes, ["输入：同一帧 2125 点", "独立局部面片：每次 30 点拟合（统一颜色）", "按局部一致性组合：面片数量为输出"]):
        ax.set(xlim=(-2.3, 3.1), ylim=(-3, 3), zlim=(-.08, 1.33), xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)", title=title)
        ax.set_box_aspect((5.4, 6, 1.4)); ax.view_init(elev=20, azim=-60); ax.tick_params(labelsize=7)
    fig.suptitle(f"不做全局高度分层：局部 30 近邻拟合后得到 {len(result['components'])} 个主连通面片", fontsize=15)
    fig.text(.5, .035, "相对每个原始点限高 ±4 cm，半径 0.75 m；未输入地面/桌底/桌面标签、已知高度或曲面数量。", ha="center", fontsize=10)
    fig.subplots_adjust(left=.01, right=.98, top=.86, bottom=.15, wspace=.15)
    fig.savefig(output/"surfaces.png", dpi=160, bbox_inches="tight"); plt.close(fig)
    fig, ax = plt.subplots(figsize=(12, 4.8), constrained_layout=True)
    band = np.abs(points[:, 1]) <= .10
    ax.scatter(points[band, 0], points[band, 2], s=13, c="#8995a3", alpha=.65, label="输入点 Y=0±0.10 m")
    for component in data["components"]:
        z = np.asarray(component["profile_height"]).reshape(101, 101)[50]
        ax.plot(data["x"], z, c=component["color"], lw=2.4, label=f"面片 {component['id']+1}，中位 Z={component['median_z']:.3f} m")
    ax.set(xlim=(-2.5, 2.5), ylim=(-.1, 1.34), xlabel="X (m)", ylabel="Z (m)", title="Y=0 的组合网格剖面：不同高度的曲面同时存在")
    ax.legend(ncol=2, fontsize=9); ax.grid(alpha=.15)
    fig.savefig(output/"profile.png", dpi=180); plt.close(fig)


def export(output, web_root):
    output.mkdir(parents=True, exist_ok=True)
    path = ROOT/"outputs/table_cpu/table_m125_evidence.npz"
    points = np.asarray(np.load(path)["points"], dtype=float)
    config = LocalConfig()
    result = reconstruct(points, config)
    check = audit(points, result, config)
    fit, labels = result["fits"], result["labels"]
    patches = patch_mesh(points, fit, config)
    axis = np.linspace(-2.5, 2.5, 101)
    x, y = np.meshgrid(axis, axis)
    xy = np.column_stack((x.ravel(), y.ravel()))
    components = []
    for i, component in enumerate(result["components"]):
        ids = component["point_ids"]
        profile = component_profile(points, fit, component, xy)
        components.append(dict(id=i, name=f"连通面片 {i+1}", color=PALETTE[i % len(PALETTE)],
                               point_ids=ids, points=len(ids), triangles=component["triangles"].ravel(),
                               median_z=float(np.median(fit["height"][ids])),
                               z_range=[float(fit["height"][ids].min()), float(fit["height"][ids].max())],
                               median_fit_rmse_m=float(np.median(fit["rmse"][ids])), profile_height=profile))
    candidates = []
    for c in components:
        ids = np.asarray(c["point_ids"])
        closest = ids[np.argmin(np.sum(points[ids, :2]**2, axis=1))]
        candidates.append(dict(name=f"面片 {c['id']+1} · 中心附近", seed=int(closest)))
    for target in ((1.35, 0.), (1.3, .8)):
        order = np.argsort(np.sum((points[:, :2]-target)**2, axis=1))
        selected = next((int(i) for i in order if np.isfinite(fit["height"][i])), int(order[0]))
        candidates.append(dict(name=f"附近点 ({target[0]}, {target[1]})", seed=selected))
    data = dict(n=len(points), points=points.ravel(), labels=labels, settings=asdict(config),
                seed_height=fit["height"], coefficients=fit["coefficient"], neighbor_ids=fit["neighbor_ids"],
                candidate_counts=fit["candidate_counts"], fit_rmse=fit["rmse"],
                components=components, x=axis, y=axis, probes=candidates,
                patch_vertices=patches[0].ravel(), patch_triangles=patches[1].ravel(), patch_owners=patches[2],
                summary={**result["summary"], **check, "unassigned_points":int((labels < 0).sum()),
                         "insufficient_neighbor_points":int((fit["candidate_counts"] < config.neighbors).sum()),
                         "quality_rejected_points":int(((fit["candidate_counts"] >= config.neighbors) & ~np.isfinite(fit["height"])).sum())})
    (output/"data.json").write_text(json.dumps(clean_json(data), ensure_ascii=False, separators=(",", ":"), allow_nan=False))
    arrays = dict(points=points, labels=labels, edges=result["edges"], **fit)
    for i, component in enumerate(result["components"]):
        arrays[f"component{i}_point_ids"] = component["point_ids"]
        arrays[f"component{i}_triangles"] = component["triangles"]
    np.savez_compressed(output/"results.npz", **arrays)
    figures(points, result, patches, data, output)
    manifest = dict(input_path=str(path.relative_to(ROOT)), input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                    code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), settings=asdict(config),
                    method="Seed-local quadratic fitting and geometry graph assembly; no global height layers; not the FSGP paper method.")
    (output/"manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False))
    (output/"audit.json").write_text(json.dumps(check, indent=2, allow_nan=False))
    report(data, output)
    for p in (ROOT/"visualization/local_surfaces").iterdir():
        if p.is_file(): shutil.copy2(p, output/p.name)
    shutil.copytree(web_root/"fsgp-fit/vendor", output/"vendor", dirs_exist_ok=True)
    shutil.copytree(output, web_root/"table-local", dirs_exist_ok=True)
    homepage = web_root/"index.html"
    html = homepage.read_text()
    banner = '<nav id="table-local-entry" style="padding:17px 24px;background:#163e57;color:white;font:15px/1.7 sans-serif"><strong>新结果：不预设三层的桌子局部拟合</strong><br>每个原始点独立取 30 近邻；不做全局高度分层，曲面数量由局部一致性连接得到。<a href="./table-local/?v=1" style="color:#a8e5f0;margin-left:16px;font-weight:bold">查看局部面片与组合效果 →</a></nav>'
    if 'id="table-local-entry"' not in html:
        homepage.write_text(html.replace("<body>", "<body>\n"+banner, 1))
    print(json.dumps(check, ensure_ascii=False), flush=True)
    print([(c["name"], c["points"], c["median_z"], len(c["triangles"])//3) for c in components], flush=True)


def report(data, output):
    summary = data["summary"]
    lines = ["# 不做全局高度分层：局部 30 点拟合与组合", "",
             "输入为与原页面完全相同的一帧 2125 点。没有先检测全局高度峰，没有指定面数或地面/桌底/桌面高度，没有读取语义标签。每个原始 XYZ 点都单独作为一个种子；同 XY 的不同 Z 不合并。", "",
             "1. 使用一棵 3D KDTree 搜索每个种子半径 0.75 m 的候选，再按相对种子的 |ΔZ|≤4 cm 过滤，取距离最近的恰好 30 点（包含种子自身）。不足 30 留空。",
             "2. 每个种子用这 30 点拟合 z−z_seed=c0+c1u+c2v+c3u²+c4uv+c5v²；XY 以种子为原点，RMS 归一化；相对 ridge=1e-10。秩比>1e-10，拟合 RMSE≤2 cm，中心预测相对种子不超过 4 cm 才有效。",
             "3. 在拟合后的三维种子点之间建立半径 0.35 m 的候选边。两个局部多项式在对方 XY 处的预测与对方拟合高度之差都≤2 cm，且中心法向夹角≤15°，才能连接。面片数量由图的连通分量得到，仅将少于 30 个有效种子的碎片保留为未分组点。",
             "4. 在每个连通面片内做 XY Delaunay；三条边都必须是步骤 3 的兼容边，三角形重心插值高度与三个局部函数的预测也都相差≤2 cm。不同面片之间不连网。",
             "5. 左图为原始点；中图为每个有效种子的真实局部二次函数在 ±12 cm 范围采样的独立小面片，裁到其 30 邻点 XY 凸包及相对高度带内；右图为拼接三角网格，顶点高度来自局部拟合。网格剖面是该网格的线性离散近似，不额外作 GP 或全局曲面回归。", "",
             "这些局部残差、法向及连接规则是本次二次函数实验的明确参数，不是 FSGP 论文方法。取消的是全局高度分层和面数假设；仍需要几何一致性规则来避免把不同表面的邻域接起来。", "",
             f"共 {summary['valid_seed_fits']} 个有效局部拟合；{summary['insufficient_neighbor_points']} 个种子邻点不足，{summary['quality_rejected_points']} 个未通过拟合质量判断；{summary['small_component_valid_points']} 个有效种子属于小碎片。", "",
             f"自动得到 {summary['component_count']} 个主要连通面片，{summary['triangles']} 个组合三角形。组件按支持点数排序命名，不按预设的三层高度归类。", "",
             "|自动组件|有效种子数|拟合高度中位数 m|组合三角形数|", "|---|---:|---:|---:|"]
    for component in data["components"]:
        lines.append(f"|{component['name']}|{component['points']}|{component['median_z']:.6f}|{len(component['triangles'])//3}|")
    lines += ["", "这些中位高度是拟合完成后的统计输出。根据已知合成场景，可以解释为地面、桌底和桌面；该解释未参与分割。桌板侧边、桌腿接触处有混合邻域，不保证完全分开。高度函数不能可靠表示竖直面，因此灰色观测点与未拟合位置保留，不补成实心物体。", "",
              "相对种子的水平高度门限依然是一种局部假设：大坡度或高曲率处可能因同表面候选不足而留空。这不是任意三维表面的通用重建或完美分割。输入的随机丢点不是 LiDAR 遮挡射线模型。", "",
              "本次每个原始点查询一次，与旧页面 101×101 网格每个全局高度层查询一次的工作量不同。旧 151 ms 不能直接作为本实现的耗时。results.npz 保存所有近邻、系数、法向、连通标签、图边和三角形；manifest.json 保存源代码及原始输入哈希。", ""]
    (output/"REPORT.md").write_text("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/table_local_surfaces")
    parser.add_argument("--web-root", type=Path, default=ROOT/"outputs/visualization")
    args = parser.parse_args()
    export(args.output, args.web_root)


if __name__ == "__main__":
    main()

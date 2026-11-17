"""Per-polynomial errors, actual K=30 footprints and a rejected mesh example.

Reads frozen fitted coefficients. No fitting, grouping, threshold or mesh is
changed. Analytic scene surfaces are used only for post-fit error evaluation.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial import ConvexHull, Delaunay, distance

from .table_scene import geometry

ROOT = Path(__file__).resolve().parent


def subtract_hole(rectangle, hole):
    """Disjoint XY rectangle pieces after removing a rectangular contact."""
    x0, y0, x1, y1 = rectangle
    a, b, c, d = max(x0, hole[0]), max(y0, hole[1]), min(x1, hole[2]), min(y1, hole[3])
    if a >= c or b >= d:
        return [rectangle]
    candidates = [(x0, y0, a, y1), (c, y0, x1, y1), (a, y0, c, b), (a, d, c, y1)]
    return [r for r in candidates if r[2] > r[0] and r[3] > r[1]]


def scene_rectangles():
    """Exposed finite rectangle faces; internal leg contacts are excluded."""
    scene = geometry()
    faces = []
    contacts = [(*leg["min"][:2], *leg["max"][:2]) for leg in scene["legs"]]
    for box, z, holes in [(scene["floor"], 0., contacts), (scene["tabletop"], 1.1, contacts),
                          (scene["tabletop"], 1.2, [])]:
        lo, hi = box["min"], box["max"]
        pieces = [(lo[0], lo[1], hi[0], hi[1])]
        for hole in holes:
            pieces = [piece for r in pieces for piece in subtract_hole(r, hole)]
        faces.extend([([a, b, z], [c, d, z]) for a, b, c, d in pieces])
    for box in [scene["tabletop"], *scene["legs"]]:
        lo, hi = np.array(box["min"]), np.array(box["max"])
        for axis in (0, 1):
            for value in (lo[axis], hi[axis]):
                lower, upper = lo.copy(), hi.copy()
                lower[axis] = upper[axis] = value
                faces.append((lower, upper))
    return np.array([v[0] for v in faces]), np.array([v[1] for v in faces])


def true_surface_distance(xyz):
    """Euclidean distance to the nearest *exposed* analytic scene surface.

    Not a signed height error, not a face-correspondence metric, and not an
    independent scan test set. It is post-fit geometric accuracy at given XY.
    """
    points = np.asarray(xyz, dtype=float)
    lower, upper = scene_rectangles()
    best = np.full(len(points), np.inf)
    for lo, hi in zip(lower, upper):
        delta = np.maximum(np.maximum(lo-points, points-hi), 0.)
        best = np.minimum(best, np.linalg.norm(delta, axis=1))
    return best


def polynomial(points, coefficient, seed_ids, xy):
    u, v = (np.asarray(xy)-points[seed_ids, :2]).T
    design = np.column_stack((np.ones(len(u)), u, v, u*u, u*v, v*v))
    return points[seed_ids, 2]+np.einsum("ni,ni->n", design, coefficient[seed_ids])


def stat(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    if not len(values): return None
    return dict(n=len(values), p05=float(np.percentile(values, 5)), median=float(np.median(values)),
                p95=float(np.percentile(values, 95)), maximum=float(values.max()))


def clean(value):
    if isinstance(value, np.ndarray): return clean(value.tolist())
    if isinstance(value, (np.floating, float)): return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer): return int(value)
    if isinstance(value, (list, tuple)): return [clean(v) for v in value]
    if isinstance(value, dict): return {k:clean(v) for k, v in value.items()}
    return value


def rejected_example(points, coefficient, height, labels, config):
    # Select the near-zero major component AFTER fitting, solely to explain
    # the already-reported 68 ground mesh rejections. This is not segmentation.
    groups = np.unique(labels[labels >= 0])
    group = min(groups, key=lambda g: abs(float(np.median(height[labels == g]))))
    ids = np.flatnonzero(labels == group)
    faces = ids[Delaunay(points[ids, :2]).simplices]
    short = np.ones(len(faces), dtype=bool)
    disagreements, directed_values = [], []
    for left, right in ((0, 1), (1, 2), (2, 0)):
        a, b = faces[:, left], faces[:, right]
        r = np.linalg.norm(np.column_stack((points[a, :2]-points[b, :2], height[a]-height[b])), axis=1)
        short &= r <= config["connect_radius_m"]
        ab = polynomial(points, coefficient, a, points[b, :2])
        ba = polynomial(points, coefficient, b, points[a, :2])
        disagreements.append(np.maximum(np.abs(ab-height[b]), np.abs(ba-height[a])))
        directed_values.append((ab, ba))
    largest = np.max(disagreements, axis=0)
    rejected = np.flatnonzero(short & (largest > config["max_model_disagreement_m"]))
    assert len(rejected) == 68, "Frozen ground rejection count changed"
    # A representative violation (median excess), not the most dramatic case.
    order = rejected[np.argsort(largest[rejected])]
    index = order[len(order)//2]
    edge = int(np.argmax(np.array(disagreements)[:, index]))
    left, right = ((0, 1), (1, 2), (2, 0))[edge]
    a, b = int(faces[index, left]), int(faces[index, right])
    ab, ba = directed_values[edge]
    return dict(component=int(group), triangle_ids=faces[index], a=a, b=b,
                at_a=dict(a_model=float(height[a]), b_model=float(ba[index])),
                at_b=dict(a_model=float(ab[index]), b_model=float(height[b])),
                worst_disagreement_m=float(largest[index]), threshold_m=config["max_model_disagreement_m"],
                ground_first_model_failure_count=len(rejected))


def calculate(saved, config):
    points, coefficient = saved["points"], saved["coefficient"]
    n = len(points)
    records = []
    active = np.flatnonzero(saved["neighbor_ids"][:, 0] >= 0)
    # Evaluation at the 30 training XY locations, against the noise-free
    # analytic scene geometry, without applying training observations as truth.
    ids = saved["neighbor_ids"][active]
    prediction = polynomial(points, coefficient, np.repeat(active, ids.shape[1]), points[ids, :2].reshape(-1, 2)).reshape(ids.shape)
    residual = prediction-points[ids, 2]
    truth_distance = true_surface_distance(np.column_stack((points[ids, :2].reshape(-1, 2), prediction.ravel()))).reshape(ids.shape)
    rmse = np.sqrt(np.mean(residual**2, axis=1))
    np.testing.assert_allclose(rmse, saved["rmse"][active], atol=1e-12, rtol=0)
    predicted_center = points[active, 2]+coefficient[active, 0]
    center_distance = true_surface_distance(np.column_stack((points[active, :2], predicted_center)))
    for i in range(n):
        records.append(dict(seed=i, component=int(saved["labels"][i]), xyz=points[i],
                            status="insufficient", solved=False, valid=False, candidate_count=int(saved["candidate_counts"][i]),
                            fit_rmse_m=None, truth_neighbor_rmse_m=None, truth_center_distance_m=None,
                            neighbor_residuals_m=[], neighbor_true_distances_m=[], radius_xy_m=None,
                            radius_3d_m=None, diameter_xy_m=None, width_x_m=None, width_y_m=None, hull_area_m2=None))
    for row, seed in enumerate(active):
        local = points[ids[row]]
        delta = local-points[seed]
        r = records[seed]
        valid = bool(np.isfinite(saved["height"][seed]))
        r.update(status="valid" if valid else "rejected", solved=True, valid=valid,
                 fit_rmse_m=float(rmse[row]), truth_neighbor_rmse_m=float(np.sqrt(np.mean(truth_distance[row]**2))),
                 truth_center_distance_m=float(center_distance[row]), neighbor_residuals_m=residual[row],
                 neighbor_true_distances_m=truth_distance[row], radius_xy_m=float(np.linalg.norm(delta[:, :2], axis=1).max()),
                 radius_3d_m=float(np.linalg.norm(delta, axis=1).max()),
                 diameter_xy_m=float(distance.pdist(local[:, :2]).max()),
                 width_x_m=float(np.ptp(local[:, 0])), width_y_m=float(np.ptp(local[:, 1])),
                 hull_area_m2=float(ConvexHull(local[:, :2]).volume))
    accepted = [r for r in records if r["valid"]]
    summary = dict(seeds=n, solved=len(active), insufficient=n-len(active), valid=len(accepted), rejected=len(active)-len(accepted),
                   valid_statistics={key:stat([r[key] for r in accepted]) for key in
                       ("fit_rmse_m", "truth_neighbor_rmse_m", "truth_center_distance_m", "radius_xy_m", "radius_3d_m", "diameter_xy_m", "width_x_m", "width_y_m", "hull_area_m2")},
                   components=[])
    for group in np.unique(saved["labels"][saved["labels"] >= 0]):
        subset = [r for r in accepted if r["component"] == group]
        summary["components"].append(dict(component=int(group), n=len(subset),
               **{key:stat([r[key] for r in subset]) for key in ("fit_rmse_m", "truth_neighbor_rmse_m", "radius_xy_m", "diameter_xy_m", "width_x_m", "width_y_m")}))
    limits = dict(fit_rmse_m=config["max_fit_rmse_m"],
                  truth_neighbor_rmse_m=max(.01, np.ceil(max(r["truth_neighbor_rmse_m"] for r in accepted)*100)/100))
    return dict(summary=summary, records=records, color_limits_m=limits,
                triangle_example=rejected_example(points, coefficient, saved["height"], saved["labels"], config),
                definitions={"fit_rmse_m":"sqrt(mean((f_i(x_j,y_j)-z_j)^2)), j are the 30 selected observations; training residual, not independent accuracy",
                             "truth_neighbor_rmse_m":"RMSE of Euclidean distances from 30 fitted positions (same neighbor XY) to the nearest exposed analytic scene surface; geometry used only after fitting; not corresponding-face error or independent test scan",
                             "radius_xy_m":"Maximum XY distance from the original seed to one of its 30 selected points",
                             "diameter_xy_m":"Maximum XY pairwise distance among the 30 selected points"})


def figures(data, saved, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    from matplotlib.colors import Normalize
    from matplotlib.cm import ScalarMappable
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from .table_local_surfaces import LocalConfig, patch_mesh
    font = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    plt.rcParams.update({"font.family":font.get_name(), "axes.unicode_minus":False})
    records = data["records"]
    fig, axes = plt.subplots(2, 1, figsize=(15, 7), constrained_layout=True, sharex=True)
    for ax, metric, title in zip(axes, ("fit_rmse_m", "truth_neighbor_rmse_m"),
            ("每个实际求解的局部二次函数：30 个拟合点的高度残差 RMSE", "同一函数在 30 个邻点 XY 处：到最近无噪声真实表面的距离 RMSE")):
        for valid, color, marker, label in [(True, "#367caa", ".", "通过检查的 1955 个局部函数"), (False, "#d24742", "x", "求解后未通过检查的 9 个局部函数")]:
            subset = [r for r in records if r["solved"] and r["valid"] == valid]
            ax.scatter([r["seed"] for r in subset], [r[metric]*100 for r in subset], s=10 if valid else 25, c=color, marker=marker, label=label, linewidths=.8)
        if metric == "fit_rmse_m": ax.axhline(2, color="#d89b4f", ls="--", lw=1, label="拟合残差筛选上限 2 cm")
        ax.set(title=title, ylabel="RMSE (cm)", xlim=(-10, len(records)+10)); ax.grid(alpha=.15); ax.legend(loc="upper right", fontsize=8, ncol=2)
    axes[-1].set_xlabel("原始种子编号：0–2124；161 个邻点不足的位置没有误差值，不按 0 处理")
    fig.suptitle("1964 次实际拟合逐个绘出；这里的点不是组合网格的三角形", fontsize=14)
    fig.savefig(output/"per-curve-errors.png", dpi=170); plt.close(fig)
    fit = {name:saved[name] for name in ("neighbor_ids", "height", "coefficient")}
    vertices, faces, owners = patch_mesh(saved["points"], fit, LocalConfig())
    fig = plt.figure(figsize=(14, 6), facecolor="#f5f8fb")
    for index, (metric, title) in enumerate((("fit_rmse_m", "每个小曲面的拟合残差"), ("truth_neighbor_rmse_m", "每个小曲面的解析几何误差"))):
        ax = fig.add_subplot(1, 2, index+1, projection="3d")
        norm = Normalize(0, data["color_limits_m"][metric]*100)
        value = np.array([records[int(seed)][metric]*100 for seed in owners[faces[:, 0]]])
        ax.add_collection3d(Poly3DCollection(vertices[faces], facecolors=plt.get_cmap("viridis")(norm(value)), linewidth=0))
        ax.set(xlim=(-2.3, 3.1), ylim=(-3, 3), zlim=(-.08, 1.33), xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)", title=title)
        ax.set_box_aspect((5.4, 6, 1.4)); ax.view_init(elev=22, azim=-60)
        fig.colorbar(ScalarMappable(norm=norm, cmap="viridis"), ax=ax, shrink=.5, pad=.09, label="每个函数的 RMSE (cm)")
    fig.suptitle("颜色表示每个局部函数的误差，未按地面/桌底/桌面分组上色", fontsize=14)
    fig.subplots_adjust(left=.01, right=.96, top=.87, bottom=.05, wspace=.20)
    fig.savefig(output/"error-map.png", dpi=170, bbox_inches="tight"); plt.close(fig)
    example = data["triangle_example"]
    points, coefficient = saved["points"], saved["coefficient"]
    a, b = example["a"], example["b"]
    ids = np.asarray(example["triangle_ids"])
    fig, (left, right) = plt.subplots(1, 2, figsize=(12, 4.8), constrained_layout=True)
    loop = np.r_[ids, ids[:1]]
    left.plot(points[loop, 0], points[loop, 1], c="#8293a4", lw=2)
    left.plot(points[[a,b], 0], points[[a,b], 1], c="#d24742", lw=3)
    left.scatter(points[ids, 0], points[ids, 1], s=65, c="#367caa", zorder=3)
    for seed in ids:
        label = "A" if seed == a else "B" if seed == b else "C"
        left.annotate(f"{label}: 种子 #{seed}", points[seed, :2], xytext=(7,7), textcoords="offset points", fontsize=10)
    left.set(aspect="equal", xlabel="X (m)", ylabel="Y (m)", title="真实的一个候选三角形：顶点是三个拟合中心")
    left.margins(.30); left.grid(alpha=.15)
    t = np.linspace(0, 1, 101)
    xy = (1-t[:, None])*points[a, :2]+t[:, None]*points[b, :2]
    along = t*np.linalg.norm(points[a, :2]-points[b, :2])
    za = polynomial(points, coefficient, np.full(len(t), a), xy)
    zb = polynomial(points, coefficient, np.full(len(t), b), xy)
    right.plot(along, za*100, lw=2.5, c="#367caa", label=f"A 的局部二次函数 #{a}")
    right.plot(along, zb*100, lw=2.5, c="#df9735", label=f"B 的局部二次函数 #{b}")
    at = -1 if abs(za[-1]-zb[-1]) >= abs(za[0]-zb[0]) else 0
    right.plot([along[at]]*2, [za[at]*100, zb[at]*100], c="#d24742", lw=3)
    right.annotate(f"同一 XY 处相差 {example['worst_disagreement_m']*100:.2f} cm\n超过 2 cm 连接门限", (along[at], (za[at]+zb[at])*50), xytext=(35 if at == 0 else -145, 15), textcoords="offset points", color="#b43d3b", fontsize=10)
    right.set(xlabel="沿 A→B 的 XY 距离 (m)", ylabel="预测高度 Z (cm)", title="两张局部曲面在同一个位置的预测不同")
    right.grid(alpha=.15); right.legend(loc="best", fontsize=9)
    fig.suptitle("三角形是后续网格单元；不是二次函数拟合的输入点数，也不是几何真值误差", fontsize=12)
    fig.savefig(output/"triangle-example.png", dpi=170); plt.close(fig)


def write_report(data, output):
    s, example = data["summary"], data["triangle_example"]
    lines = ["# 每个局部二次函数的误差与邻域范围", "",
             f"2125 个原始种子；{s['insufficient']} 个不足 30 点未求解；{s['solved']} 个实际求解；{s['rejected']} 个求解后被质量检查剔除，{s['valid']} 个保留。全图三个主连通面片不是三个全局二次函数。30 点包括种子本身。", "",
             "## 两种误差", "",
             "拟合 RMSE = sqrt(mean((f_i(x_j,y_j)−z_j)²))，j 为该函数的 30 个原始拟合点。这是训练残差，不是独立测试准确率。",
             "解析几何 RMSE：将同一函数在这 30 个 XY 上的预测点，分别求到合成场景最近暴露表面的欧氏距离，再求 RMSE。真实表面使用有限矩形面，排除地面/桌底的桌腿接触内部面。它是事后几何评价，没有用于选点、拟合或分组；没有对应表面身份约束，不是独立新扫描测试。", "",
             "每个种子保留一行数据。161 个未拟合点的误差为空，不按零统计。9 个已求解但未通过的函数也画在误差图中，避免只看筛选后的结果。", "",
             "|有效函数统计|中位数|P95|最大值|", "|---|---:|---:|---:|"]
    for key, title, factor, unit in [("fit_rmse_m", "拟合残差", 100, "cm"), ("truth_neighbor_rmse_m", "解析几何误差", 100, "cm"),
                                    ("radius_xy_m", "最远邻点到种子的 XY 距离", 1, "m"), ("diameter_xy_m", "30点的 XY 最大点对距离", 1, "m"),
                                    ("width_x_m", "XY包围框 X 跨度", 1, "m"), ("width_y_m", "XY包围框 Y 跨度", 1, "m"),
                                    ("hull_area_m2", "30点 XY 凸包面积", 1, "m²")]:
        v = s["valid_statistics"][key]
        lines.append(f"|{title}|{v['median']*factor:.4f} {unit}|{v['p95']*factor:.4f} {unit}|{v['maximum']*factor:.4f} {unit}|")
    lines += ["", "搜索门限是三维半径 0.75 m；实际 30 点覆盖范围随密度、种子位置和相对高度门限变化，并不是固定 24×24 cm。24×24 cm 仅是中图每个局部函数的显示采样范围。", "",
              "## 68 个‘预测不一致’候选三角形", "",
              "先完成局部二次函数拟合，再在同一连通组件的拟合中心 XY 上做 Delaunay 三角化。三个拟合中心形成的候选三角形仅是后续网格单元；不是用三个点拟合二次函数。",
              "对一条 A–B 边，比较 A 的函数在 B 位置的预测与 B 自己在 B 处的预测，同时反向比较 B 在 A 处与 A 自己的预测。两个差值只要有一个超过 2 cm，就不连接这条边。有一条边不兼容，该三角形不保留。该 2 cm 是当前实现的拼接参数，不是与真值比较得到的拟合误差。",
              "地面 2695 个候选三角形中，184 个先因边长>0.35m被删除；在剩余候选中，68 个因上述预测差被删除，保留 2443 个。", "",
              f"真实示例：三角形种子编号 {list(map(int, example['triangle_ids']))}，其中 A=#{example['a']}、B=#{example['b']}；两函数在 A/B 端点的最大预测差 {example['worst_disagreement_m']*100:.4f} cm。",
              f"A位置：A模型 {example['at_a']['a_model']*100:.4f} cm，B模型 {example['at_a']['b_model']*100:.4f} cm；B位置：A模型 {example['at_b']['a_model']*100:.4f} cm，B模型 {example['at_b']['b_model']*100:.4f} cm。", "",
              "per-curve.csv 包含全部 2125 个种子的状态、拟合/几何误差、邻域半径、直径、包围框和凸包面积。data.json 另含每个已拟合函数 30 个逐点残差，供页面点选查看。所有数据读自冻结结果，没有重新拟合或修改已计时算法。", ""]
    (output/"REPORT.md").write_text("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/table_local_errors")
    args = parser.parse_args()
    output = args.output; output.mkdir(parents=True, exist_ok=True)
    path = ROOT/"outputs/table_local_surfaces/results.npz"
    manifest = json.loads((ROOT/"outputs/table_local_surfaces/manifest.json").read_text())
    saved = np.load(path)
    data = calculate(saved, manifest["settings"])
    data["provenance"] = dict(results_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
         source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), fit_manifest=manifest,
         changed_fits=False, independent_test_set=False)
    (output/"data.json").write_text(json.dumps(clean(data), ensure_ascii=False, separators=(",", ":"), allow_nan=False))
    fields = ["seed", "component", "status", "solved", "valid", "candidate_count", "fit_rmse_m", "truth_neighbor_rmse_m", "truth_center_distance_m", "radius_xy_m", "radius_3d_m", "diameter_xy_m", "width_x_m", "width_y_m", "hull_area_m2"]
    with (output/"per-curve.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader()
        for record in data["records"]: writer.writerow({key:record[key] for key in fields})
    figures(data, saved, output)
    write_report(data, output)
    for destination in (ROOT/"outputs/table_local_surfaces/errors", ROOT/"outputs/visualization/table-local/errors"):
        shutil.copytree(output, destination, dirs_exist_ok=True)
    print(json.dumps(clean(dict(summary=data["summary"], triangle_example=data["triangle_example"]))), flush=True)


if __name__ == "__main__":
    main()

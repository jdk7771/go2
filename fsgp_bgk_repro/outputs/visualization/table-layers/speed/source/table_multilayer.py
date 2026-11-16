"""Table diagnostic: height modes -> exactly 30 gated neighbours -> local fits.

This is a user-requested extension, not the FSGP paper's segmentation method.
Only XYZ enter mode detection and regression. Table dimensions enter evaluation
and captions only. A separate height graph is kept for every detected mode.
Vertical faces remain observations; they are not fitted by z=f(x,y).
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks
from scipy.spatial import cKDTree

from .visualize_regression import clean_json, local_quadratic

ROOT = Path(__file__).resolve().parent
SETTINGS = dict(neighbors=30, radius_m=0.75, nearest_xy_m=0.20,
                height_gate_m=0.04, histogram_bin_m=0.02,
                histogram_sigma_bins=0.7, mode_refinement_m=0.04,
                max_triangle_height_jump_m=0.04, rank_ratio=1e-10,
                relative_ridge=1e-10, grid_step_m=0.05)
COLORS = ["#21856b", "#417cc4", "#e49b35"]
PROBES = [("桌子中心", (0., 0.)), ("桌边内侧", (1.35, 0.)),
          ("桌边外侧", (1.65, 0.)), ("桌腿附近", (1.30, .80))]


def height_modes(points):
    """Height density modes, without semantic labels or known table heights.

    A scene-specific horizontal-layer diagnostic, not a general segmentation
    algorithm for arbitrary ramps/vertical/curved surfaces.
    """
    z = np.asarray(points, dtype=float)[:, 2]
    width = SETTINGS["histogram_bin_m"]
    edges = np.arange(np.floor(z.min()/width)*width-2*width,
                      np.ceil(z.max()/width)*width+3*width, width)
    counts, _ = np.histogram(z, edges)
    smooth = gaussian_filter1d(counts.astype(float), SETTINGS["histogram_sigma_bins"])
    peaks, _ = find_peaks(smooth, distance=3,
                         prominence=max(2., .03*float(smooth.max())))
    centres = []
    for peak in peaks:
        centre = float((edges[peak]+edges[peak+1])/2)
        for _ in range(3):
            local = z[np.abs(z-centre) <= SETTINGS["mode_refinement_m"]]
            if len(local):
                centre = float(np.median(local))
        if len(local) >= 8 and all(abs(centre-c) > width for c in centres):
            centres.append(centre)
    return np.array(sorted(centres))


def assign_layers(points, centres, gate):
    if not len(centres):
        return np.full(len(points), -1, dtype=int)
    delta = np.abs(points[:, 2, None]-centres[None, :])
    labels = np.argmin(delta, axis=1)
    labels[np.min(delta, axis=1) > gate] = -1
    return labels


def fit_layer(points, xy, point_ids, centre, gate, settings=None):
    """Filter height BEFORE KNN. Fewer than K inside radius means unknown.

    local convex-hull containment rejects lateral extrapolation. Fits are local
    normalized six-coefficient quadratics; no height clamping or hole filling.
    Returned IDs refer to the original cloud and allow independent auditing.
    """
    cfg = SETTINGS if settings is None else settings
    k, n = cfg["neighbors"], len(xy)
    points, xy = np.asarray(points), np.asarray(xy)
    point_ids = np.asarray(point_ids, dtype=int)
    # Enforce the height gate here too, even when the caller supplied candidates.
    point_ids = point_ids[np.abs(points[point_ids, 2]-centre) <= gate]
    height = np.full(n, np.nan)
    ids = np.full((n, k), -1, dtype=np.int32)
    distances = np.full((n, k), np.inf)
    nearest = np.full(n, np.inf)
    counts = np.zeros(n, dtype=np.int32)
    hull, rank, bounded = [np.zeros(n, dtype=bool) for _ in range(3)]
    if len(point_ids):
        tree = cKDTree(points[point_ids])
        query = np.column_stack((xy, np.full(n, centre)))
        distances, local_ids = tree.query(query, k=k, distance_upper_bound=cfg["radius_m"])
        good = np.isfinite(distances)
        counts = good.sum(axis=1)
        ids[good] = point_ids[local_ids[good]]
        nearest = cKDTree(points[point_ids, :2]).query(xy)[0]
        active = np.flatnonzero((counts == k) & (nearest <= cfg["nearest_xy_m"]))
        if len(active):
            local = points[ids[active]]
            delta = local[:, :, :2]-xy[active, None, :]
            angle = np.sort(np.arctan2(delta[:, :, 1], delta[:, :, 0]), axis=1)
            gaps = np.diff(np.column_stack((angle, angle[:, :1]+2*np.pi)), axis=1)
            inside = (gaps.max(axis=1) <= np.pi+1e-9) | (np.linalg.norm(delta, axis=2).min(axis=1) < 1e-9)
            hull[active] = inside
            scale = np.maximum(np.sqrt(np.mean(np.sum(delta**2, axis=2), axis=1)), 1e-10)
            u, v = (delta/scale[:, None, None]).transpose(2, 0, 1)
            design = np.stack((np.ones_like(u), u, v, u*u, u*v, v*v), axis=2)
            gram = np.einsum("nki,nkj->nij", design, design)
            spectrum = np.linalg.eigvalsh(gram)
            rank[active] = spectrum[:, 0] > np.maximum(spectrum[:, -1], 1e-30)*cfg["rank_ratio"]
            rhs = np.einsum("nki,nk->ni", design, local[:, :, 2]-centre)
            ridge = cfg["relative_ridge"]*np.trace(gram, axis1=1, axis2=2)/6
            coefficient = np.linalg.solve(gram+ridge[:, None, None]*np.eye(6), rhs)
            prediction = centre+coefficient[:, 0]
            bounded[active] = np.isfinite(prediction) & (np.abs(prediction-centre) <= gate)
            valid = inside & rank[active] & bounded[active]
            height[active[valid]] = prediction[valid]
    return dict(height=height, neighbor_ids=ids, neighbor_distance=distances,
                neighbor_count=counts, nearest_xy=nearest, inside_hull=hull,
                full_rank=rank, height_bounded=bounded, point_ids=point_ids)


def fit_multilayer(points, xy, gate=0.04):
    centres = height_modes(points)
    labels = assign_layers(points, centres, gate)
    fits = [fit_layer(points, xy, np.flatnonzero(labels == i), c, gate)
            for i, c in enumerate(centres)]
    return centres, labels, fits


def grid_triangles(height, nx, ny, max_jump=0.04):
    """Called once PER layer. No triangle can join distinct height modes."""
    a = (np.arange(ny-1)[:, None]*nx+np.arange(nx-1)[None, :]).ravel()
    triangles = np.concatenate((np.column_stack((a, a+1, a+nx)),
                                np.column_stack((a+1, a+nx+1, a+nx))))
    z = height[triangles]
    keep = np.isfinite(z).all(axis=1)
    if max_jump is not None:
        keep &= np.ptp(z, axis=1) <= max_jump
    return triangles[keep].astype(np.int32)


def truth_domain(xy, layer):
    # Known synthetic geometry is ONLY used for evaluation, never segmentation.
    floor = np.ones(len(xy), dtype=bool)
    contact = np.zeros(len(xy), dtype=bool)
    for x in (-1.32, 1.32):
        for y in (-.82, .82):
            contact |= (np.abs(xy[:, 0]-x) < .06-1e-8) & (np.abs(xy[:, 1]-y) < .06-1e-8)
    face = (np.abs(xy[:, 0]) <= 1.5+1e-8) & (np.abs(xy[:, 1]) <= 1.+1e-8)
    return [floor & ~contact, face & ~contact, face][layer]


def evaluate(xy, height, layer):
    domain = truth_domain(xy, layer)
    valid = np.isfinite(height)
    keep = domain & valid
    error = np.abs(height[keep]-[0., 1.1, 1.2][layer])
    return dict(valid_queries=int(valid.sum()), truth_queries=int(domain.sum()),
                covered_truth_queries=int(keep.sum()), coverage=float(keep.sum()/domain.sum()),
                outside_face_queries=int((valid & ~domain).sum()),
                mae_m=float(error.mean()) if len(error) else None,
                p95_m=float(np.percentile(error, 95)) if len(error) else None)


def probe_data(points, xy, centres, labels, fits, gate):
    probes = []
    for name, location in PROBES:
        q = int(np.argmin(np.sum((xy-location)**2, axis=1)))
        layers = []
        for i, (centre, fit) in enumerate(zip(centres, fits)):
            ids = fit["neighbor_ids"][q]
            ids = ids[ids >= 0]
            candidates = fit["point_ids"]
            available = int(np.sum(np.linalg.norm(points[candidates]-[*xy[q], centre], axis=1) <= SETTINGS["radius_m"]))
            valid = np.isfinite(fit["height"][q])
            if available < SETTINGS["neighbors"]:
                reason = "半径内不足 30 点，留空"
            elif fit["nearest_xy"][q] > SETTINGS["nearest_xy_m"]:
                reason = "附近 XY 观测不足，留空"
            elif not fit["inside_hull"][q]:
                reason = "查询点在局部点云凸包外，留空"
            elif not fit["full_rank"][q]:
                reason = "二次设计矩阵退化，留空"
            elif not valid:
                reason = "预测超出本层高度门限，留空"
            else:
                reason = "有效：使用本层 30 点独立拟合"
            layers.append(dict(layer=i, ids=ids, available=available,
                               height=fit["height"][q], valid=bool(valid), reason=reason,
                               neighbor_z_range=[float(points[ids, 2].min()), float(points[ids, 2].max())] if len(ids) else None,
                               max_height_delta=float(np.abs(points[ids, 2]-centre).max()) if len(ids) else None,
                               max_distance=float(fit["neighbor_distance"][q, :len(ids)].max()) if len(ids) else None))
        probes.append(dict(name=name, xy=xy[q], layers=layers))
    return probes


def audit(points, xy, centres, labels, fits, triangles, gate):
    result = []
    for i, (centre, fit, faces) in enumerate(zip(centres, fits, triangles)):
        valid = np.isfinite(fit["height"])
        ids = fit["neighbor_ids"][valid]
        assert ids.shape == (int(valid.sum()), SETTINGS["neighbors"])
        assert np.all(ids >= 0)
        assert np.all(labels[ids] == i)
        delta = np.abs(points[ids, 2]-centre)
        assert np.all(delta <= gate+1e-12)
        distance = np.linalg.norm(points[ids]-np.column_stack((xy[valid], np.full(valid.sum(), centre)))[:, None, :], axis=2)
        assert np.all(distance <= SETTINGS["radius_m"]+1e-12)
        assert np.all(fit["inside_hull"][valid])
        assert np.all(np.isfinite(fit["height"][faces]))
        assert np.all(np.ptp(fit["height"][faces], axis=1) <= SETTINGS["max_triangle_height_jump_m"]+1e-12)
        result.append(dict(layer=i, valid_queries=int(valid.sum()),
                           max_neighbor_height_delta_m=float(delta.max()) if delta.size else None,
                           max_neighbor_distance_m=float(distance.max()) if distance.size else None,
                           triangles=len(faces), cross_layer_triangles=0))
    return result


def figures(data, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    font = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    plt.rcParams.update({"font.family": font.get_name(), "axes.unicode_minus": False})
    points = np.asarray(data["points"]).reshape(-1, 3)
    nx, ny = len(data["x"]), len(data["y"])
    x, y = np.meshgrid(data["x"], data["y"])
    xy = np.column_stack((x.ravel(), y.ravel()))
    labels = np.asarray(data["labels"])
    fig = plt.figure(figsize=(16, 5.7), facecolor="#f4f7fb")
    axes = [fig.add_subplot(1, 3, i+1, projection="3d") for i in range(3)]
    for i, layer in enumerate(data["layers"]):
        p = points[labels == i]
        axes[0].scatter(*p.T, s=2, c=COLORS[i % len(COLORS)], depthshade=False)
        height = np.asarray(layer["height"])
        vertices = np.column_stack((xy, height))
        faces = np.asarray(layer["triangles"]).reshape(-1, 3)
        axes[2].add_collection3d(Poly3DCollection(vertices[faces], facecolor=COLORS[i % len(COLORS)], linewidth=0, alpha=1.))
    p = points[labels < 0]
    axes[0].scatter(*p.T, s=3, c="#82909f", depthshade=False)
    old = np.column_stack((xy, data["old_height"]))
    axes[1].add_collection3d(Poly3DCollection(old[np.asarray(data["old_triangles"]).reshape(-1, 3)], facecolor="#9878b9", linewidth=0, alpha=1.))
    for ax, title in zip(axes, ["同一帧输入：按观测高度分层", "单层对照：K=30，无高度门限", "多层二次拟合：每层 K=30，门限 ±4 cm"]):
        ax.set(xlim=(-2.5, 2.5), ylim=(-2.5, 2.5), zlim=(-.08, 1.32), xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)", title=title)
        ax.set_box_aspect((5, 5, 1.4)); ax.view_init(elev=20, azim=-60); ax.tick_params(labelsize=7)
    fig.suptitle("大桌子：同一 XY 同时保留地面、桌底和桌面", fontsize=15)
    fig.text(.5, .035, "仅用 XYZ 自动检测高度峰；先按高度筛点，再取半径 0.75 m 内的 30 近邻。竖直桌腿保留为点云。", ha="center", fontsize=10)
    fig.subplots_adjust(left=.01, right=.98, top=.86, bottom=.16, wspace=.15)
    fig.savefig(output/"surfaces.png", dpi=160, bbox_inches="tight"); plt.close(fig)
    row = ny//2
    fig, ax = plt.subplots(figsize=(12, 4.8), constrained_layout=True)
    band = np.abs(points[:, 1]-data["y"][row]) <= .1
    ax.scatter(points[band, 0], points[band, 2], s=13, c="#8c98a6", alpha=.65, label="输入点（Y=0±0.1 m）")
    ax.plot(data["x"], np.asarray(data["old_height"]).reshape(ny, nx)[row], c="#9878b9", alpha=.8, ls="--", lw=1.6, label="单层对照 K=30")
    for i, layer in enumerate(data["layers"]):
        ax.plot(data["x"], np.asarray(layer["height"]).reshape(ny, nx)[row], c=COLORS[i % 3], lw=2.6, label=f"{layer['name']}：{layer['centre']:.3f} m")
    ax.set(xlim=(-2.5, 2.5), ylim=(-.10, 1.35), xlabel="X (m)", ylabel="Z (m)", title="真实高度剖面 Y=0：中间区域有三条独立曲线（未展开高度）")
    ax.grid(alpha=.15); ax.legend(loc="upper left", ncol=3, fontsize=9)
    fig.savefig(output/"profile.png", dpi=180); plt.close(fig)


def export(output, web_root):
    output.mkdir(parents=True, exist_ok=True)
    (output/"cases").mkdir(exist_ok=True)
    (output/"arrays").mkdir(exist_ok=True)
    paths = dict(scan=ROOT/"outputs/table_cpu/table_m125_evidence.npz",
                 full=ROOT/"outputs/visualization/large-table/large_table.npz",
                 sparse=ROOT/"outputs/table_cpu/table_m125_fsgp.npz")
    names = dict(scan="原页面同一帧 · 2125 点（含噪声）", full="完整合成点云 · 32683 点（无噪声）", sparse="论文筛选后 M=125 · 稀疏性对照")
    axis = np.linspace(-2.5, 2.5, 101)
    x, y = np.meshgrid(axis, axis)
    xy = np.column_stack((x.ravel(), y.ravel()))
    manifest = dict(settings=SETTINGS, sources={}, cases=[], method="User-requested multilayer quadratic diagnostic; not an FSGP paper method.")
    audits, all_metrics, default_data = [], [], None
    for source, path in paths.items():
        raw = np.load(path)
        points = np.asarray(raw["inducing"][:, :3] if source == "sparse" else raw["points"], dtype=float)
        manifest["sources"][source] = dict(name=names[source], path=str(path.relative_to(ROOT)),
                                            sha256=hashlib.sha256(path.read_bytes()).hexdigest(), n=len(points))
        old_height, _ = local_quadratic(points, xy, neighbors=30)
        old_height[cKDTree(points[:, :2]).query(xy)[0] > SETTINGS["nearest_xy_m"]] = np.nan
        old_faces = grid_triangles(old_height, len(axis), len(axis), max_jump=None)
        for gate in (.02, .03, .04):
            key = f"{source}_{round(gate*100):02d}cm"
            centres, labels, fits = fit_multilayer(points, xy, gate)
            faces = [grid_triangles(f["height"], len(axis), len(axis), SETTINGS["max_triangle_height_jump_m"]) for f in fits]
            checks = audit(points, xy, centres, labels, fits, faces, gate)
            # Detected modes happen to match the known three synthetic faces.
            # Reject unexpected modes instead of silently scoring another face.
            assert len(centres) == 3 and np.max(np.abs(centres-[0, 1.1, 1.2])) < .04
            layers = []
            for i, (centre, fit, triangles) in enumerate(zip(centres, fits, faces)):
                score = evaluate(xy, fit["height"], i)
                layers.append(dict(id=i, name=["地面", "桌底", "桌面"][i], centre=centre,
                                   color=COLORS[i], points=int((labels == i).sum()),
                                   height=fit["height"], triangles=triangles.ravel(), metrics=score))
                all_metrics.append(dict(case=key, source=source, height_gate_m=gate, layer=i, **score))
            data = dict(key=key, source=source, source_name=names[source], n=len(points),
                        height_gate_m=gate, x=axis, y=axis, points=points.ravel(), labels=labels,
                        layers=layers, old_height=old_height, old_triangles=old_faces.ravel(),
                        unassigned=int((labels < 0).sum()), probes=probe_data(points, xy, centres, labels, fits, gate))
            if key == "scan_04cm":
                default_data = data
            case_path = output/"cases"/f"{key}.json"
            case_path.write_text(json.dumps(clean_json(data), ensure_ascii=False, separators=(",", ":"), allow_nan=False))
            arrays = dict(points=points, xy=xy, centres=centres, labels=labels, old_height=old_height, old_triangles=old_faces)
            for i, fit in enumerate(fits):
                for field, values in fit.items():
                    arrays[f"layer{i}_{field}"] = values
                arrays[f"layer{i}_triangles"] = faces[i]
            np.savez_compressed(output/"arrays"/f"{key}.npz", **arrays)
            audits.append(dict(case=key, checks=checks, case_sha256=hashlib.sha256(case_path.read_bytes()).hexdigest()))
            manifest["cases"].append(dict(key=key, source=source, height_gate_m=gate, file=f"cases/{key}.json"))
            print(key, "points", [f["point_ids"].size for f in fits], "valid", [int(np.isfinite(f["height"]).sum()) for f in fits], flush=True)
    figures(default_data, output)
    manifest["code_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (output/"manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False, allow_nan=False))
    (output/"audit.json").write_text(json.dumps(audits, indent=2, ensure_ascii=False, allow_nan=False))
    (output/"metrics.json").write_text(json.dumps(all_metrics, indent=2, ensure_ascii=False, allow_nan=False))
    report(default_data, all_metrics, output)
    for path in (ROOT/"visualization/layers").iterdir():
        if path.is_file():
            shutil.copy2(path, output/path.name)
    shutil.copytree(ROOT/"outputs/visualization/fsgp-fit/vendor", output/"vendor", dirs_exist_ok=True)
    shutil.copytree(output, web_root/"table-layers", dirs_exist_ok=True)
    banner = '<nav id="table-layer-entry" style="padding:17px 24px;background:#143b37;color:white;font:15px/1.7 sans-serif"><strong>大桌子 · 30 近邻 + 高度约束 · 三层曲面</strong><br>地面、桌底、桌面分别拟合；同一 XY 保留多个高度。<a href="./table-layers/?v=1" style="color:#a8f0d3;margin-left:16px;font-weight:bold">打开重新拟合结果 →</a></nav>'
    homepage = web_root/"index.html"
    html = homepage.read_text()
    if 'id="table-layer-entry"' not in html:
        homepage.write_text(html.replace("<body>", "<body>\n"+banner, 1))


def report(data, rows, output):
    lines = ["# 大桌子：30 近邻、高度门限与多层二次拟合", "",
             "这是按用户要求新增的诊断，不是 FSGP 论文中的分割或 GP 算法。原结果保留。", "",
             "默认：与原页面同一帧 2125 点（含 1.5 cm Z 噪声），未使用几何标签筛点。另含完整 32683 点与原 M=125 两组输入，均有 ±2/3/4 cm 三档高度门限。", "",
             "1. 从 XYZ 的 2 cm 高度直方图自动检测密度峰，平滑 σ=0.7 格，峰值间距至少 3 格，显著性 max(2, 最高峰×3%)；在 ±4 cm 内迭代中位数 3 次细化峰值，至少 8 个支持点。",
             "2. 按最近峰及高度门限分层；每点最多属于一层。先筛高度，再在该层 3D KDTree 中取半径 0.75 m 内 30 近邻。不足 30 不拟合，不跨层补点。",
             "3. 每个 XY 对每层分别拟合 z=c0+c1u+c2v+c3u²+c4uv+c5v²。XY 以查询点为原点、RMS 距离归一化；六系数最小二乘相对 ridge=1e-10。",
             "4. 最近 XY 点需在 0.20 m 内，查询需在这 30 点的 XY 凸包内，特征值比需大于 1e-10，预测需在本层高度门限内。失败留空，未做高度裁剪、补洞或跨层平滑。",
             "5. 各层独立输出 101×101（5 cm）网格；只连接同层三个有效顶点，顶点高差超过 4 cm 的三角形剔除。竖直桌腿和桌边仅保留原始点云。", "",
             "同输入的单层对照也改为 K=30，沿用旧局部函数的最近 XY 高度锚点→3D KNN→单个 z；没有分层或高度门限。只限制最近 XY 支持为 0.20 m，绘图沿用旧连续网格规则。它用于说明表示和连网问题，不冒充原页面 M=125、K=20 的原结果。", "",
             "## 默认同帧结果（全真值表面范围统计）", "",
             "|层|观测高度峰 m|本层点数|有效/真值查询|覆盖率|有效处 MAE cm|范围外有效格|", "|---|---:|---:|---:|---:|---:|---:|"]
    for layer in data["layers"]:
        e = layer["metrics"]
        lines.append(f"|{layer['name']}|{layer['centre']:.6f}|{layer['points']}|{e['covered_truth_queries']}/{e['truth_queries']}|{e['coverage']:.1%}|{e['mae_m']*100:.3f}|{e['outside_face_queries']}|")
    lines += ["", "覆盖率分母是整个 5×5 m 窗口中该层的解析真值范围，包含观测不足及桌底腿接触处的缺口；不以仅有效区域冒充完整重建。MAE 只计有效且落在真实表面内的查询；范围外格另列。", "",
              "高度密度分层适用于本桌子的近水平层，不是任意坡面/曲面通用分割。水平高度函数不能完整表示竖直腿和桌边；靠近接触面的少量竖直点可能进入高度带。无噪声完整点云不是遮挡模拟，原同帧是随机丢点，不是激光射线可见性仿真。", "",
              "完整数值、每个查询的原点云近邻 ID、支持距离、有效性和三角形保存在 arrays/*.npz。audit.json 对所有有效拟合核验严格 30 点、同层、门限与半径；网格分层输出，因此跨层三角形为零。metrics.json 含全部 9 组，不隐藏稀疏输入失败。", "",
              "打开 index.html 查看同步三维、任意 Y 剖面、逐层开关、指定位置的 30 邻点。‘展开高度层’只平移三维显示，真实剖面和拟合数值不变。"]
    (output/"REPORT.md").write_text("\n".join(lines)+"\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"outputs/table_multilayer")
    parser.add_argument("--web-root", type=Path, default=ROOT/"outputs/visualization")
    args = parser.parse_args()
    export(args.output, args.web_root)


if __name__ == "__main__":
    main()

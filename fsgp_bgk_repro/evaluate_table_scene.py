"""Evaluate the table with unchanged paper-equation CPU mapping code.

The reference is the analytic flat floor, excluding cells intersecting legs.
No cost label or pass/fail threshold is invented for the occupied leg cells.
"""

import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from .core import Config, TraversabilityMapper, extract_features
from .experiment import ROOT, environment, errors, sample_scan, write_csv
from .io import read_cloud, save_result
from .table_scene import DEFAULT_OUTPUT, export as export_scene, validate_cloud


METHODS = (("SGP-XY", "sgp"), ("FSGP", "fsgp"), ("FSGP-BGK", "bgk"))


def floor_reference(xy, shape, scene, resolution):
    """Zero ideal-floor score, with any leg-overlapping/out-of-floor cell masked."""
    half = resolution / 2
    floor_min, floor_max = np.asarray(scene["floor"]["min"]), np.asarray(scene["floor"]["max"])
    valid = np.all((xy - half >= floor_min[:2] - 1e-6)
                   & (xy + half <= floor_max[:2] + 1e-6), axis=1)
    for leg in scene["legs"]:
        lo, hi = np.asarray(leg["min"]), np.asarray(leg["max"])
        overlap = np.all((xy + half > lo[:2]) & (xy - half < hi[:2]), axis=1)
        valid &= ~overlap
    return np.where(valid, 0.0, np.nan).reshape(shape)


def interior_mask(xy):
    # A strictly interior 2 x 1 m patch, distant from all four legs and table edges.
    return (np.abs(xy[:, 0]) <= 1.0 + 1e-6) & (np.abs(xy[:, 1]) <= .5 + 1e-6)


def region_stats(result, reference):
    finite = result["observed"] & np.isfinite(reference.ravel())
    under = finite & interior_mask(result["xy"])
    outside = finite & ((np.abs(result["xy"][:, 0]) >= 1.9)
                        | (np.abs(result["xy"][:, 1]) >= 1.5))
    cost = result["cost"].ravel()
    return {"under_cells": int(under.sum()),
            "under_height_median_m": float(np.median(result["height"][under])),
            "under_height_abs_error_mean_m": float(np.abs(result["height"][under]).mean()),
            "under_slope_mean": float(result["slope"][under].mean()),
            "under_height_variation_mean_m": float(result["height_variation"][under].mean()),
            "under_cost_mean": float(cost[under].mean()),
            "outside_cost_mean": float(cost[outside].mean()) if outside.any() else None}


def decorate(ax, scene):
    from matplotlib.patches import Rectangle
    ax.add_patch(Rectangle((-1.5, -1), 3, 2, fill=False, edgecolor="#3b9c8d", linewidth=1))
    for leg in scene["legs"]:
        x, y = leg["min"][:2]
        ax.add_patch(Rectangle((x, y), .12, .12, fill=False, edgecolor="#38d7b0", linewidth=.9))
    ax.set(xlabel="world x / m", ylabel="world y / m", aspect="equal")


def plot_result(path, maps, reference, scene, count):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    result = maps["fsgp"]
    xy, shape = result["xy"], result["shape"]
    half = .1
    extent = [xy[:, 0].min()-half, xy[:, 0].max()+half,
              xy[:, 1].min()-half, xy[:, 1].max()+half]
    maximum = max(float(np.nanmax(m["cost"])) for m in maps.values())
    panels = [(reference, "平坦地面解析参考；腿占据格排除", "magma", 0, maximum),
              (result["height"].reshape(shape), "FSGP 预测高程 / m；真实地面为 0", "coolwarm", None, None),
              (result["slope"].reshape(shape), "FSGP 坡度比 |dz/dxy|", "viridis", 0, None),
              (100*result["height_variation"].reshape(shape), "邻域平均绝对高差 / cm", "viridis", 0, None),
              (maps["sgp"]["cost"], "SGP-XY 对照代价", "magma", 0, maximum),
              (maps["fsgp"]["cost"], "FSGP 单帧代价", "magma", 0, maximum),
              (maps["bgk"]["cost"], "FSGP-BGK 融合代价", "magma", 0, maximum),
              (result["variance"].reshape(shape), "单帧 GP 潜在方差", "viridis", 0, None)]
    fig, axes = plt.subplots(2, 4, figsize=(16, 8), constrained_layout=True)
    for ax, (values, title, cmap, vmin, vmax) in zip(axes.ravel(), panels):
        im = ax.imshow(values, origin="lower", extent=extent, cmap=cmap, vmin=vmin, vmax=vmax)
        ax.set_title(title, fontproperties=font, fontsize=10)
        decorate(ax, scene)
        fig.colorbar(im, ax=ax, shrink=.72)
    fig.suptitle(f"大桌子 · 第 8 帧 · M={count} · 青线为桌面轮廓/桌腿位置", fontproperties=font)
    fig.savefig(path, dpi=155)
    plt.close(fig)


def isolation_controls(output, points, labels, scene, config):
    """Noiseless full geometry versus a declared floor-only ablation."""
    records, maps = [], {}
    control_config = replace(config, inducing_points=125)
    for name, cloud in (("complete_clean", points), ("floor_only_clean", points[labels == 0])):
        mapper = TraversabilityMapper(control_config)
        result = mapper.fuse(mapper.assess(cloud, (0, 0), 0))
        save_result(output / f"control_{name}.npz", result)
        reference = floor_reference(result["xy"], result["shape"], scene, config.resolution)
        records.append({"input": name, "points_before_local_crop": len(cloud),
                        "points_after_local_crop": len(result["points"]),
                        **region_stats(result, reference)})
        maps[name] = result
    (output / "controls.json").write_text(json.dumps(records, indent=2) + "\n")
    features = extract_features(points, config.neighbors)
    central = features[np.all(np.isclose(features[:, :2], 0, atol=1e-6), axis=1)]
    central = central[np.argsort(central[:, 2])]
    np.testing.assert_allclose(central[:, 2], [0, 1.1, 1.2], atol=1e-6)
    np.testing.assert_allclose(central[:, 3:], 0, atol=1e-12)
    ambiguity = {"columns": ["x", "y", "target_z", "curvature", "height_variation"],
                 "center_surface_features": central.tolist(),
                 "explanation": "The three layers have the same (x,y,kappa,g) input but different target heights. A single GP prediction for this query does not encode all three layers or clearance."}
    (output / "multilayer_ambiguity.json").write_text(json.dumps(ambiguity, indent=2) + "\n")
    plot_controls(output / "controls.png", points, maps)
    return records


def plot_controls(path, points, maps):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font = FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    full, floor = maps["complete_clean"], maps["floor_only_clean"]
    xy = full["xy"]
    row = np.isclose(xy[:, 1], .1)
    band = np.abs(points[:, 1] - .1) <= .021
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.4), constrained_layout=True)
    axes[0].scatter(points[band, 0], points[band, 2], s=3, color=".65", label="真实表面点")
    axes[0].plot(xy[row, 0], full["height"][row], color="#bc453d", label="完整桌子输入的 GP 高程")
    axes[0].plot(xy[row, 0], floor["height"][row], color="#2e7969", linestyle="--", label="仅地面输入的 GP 高程")
    axes[0].set(ylabel="z / m")
    axes[0].set_title("无噪声高度剖面：同一 XY 的多层表面", fontproperties=font)
    axes[1].plot(xy[row, 0], full["cost"].ravel()[row], color="#bc453d", label="完整桌子输入的代价")
    axes[1].plot(xy[row, 0], floor["cost"].ravel()[row], color="#2e7969", linestyle="--", label="仅地面输入的代价")
    axes[1].set(ylabel="raw cost")
    axes[1].set_title("无噪声代价剖面：真实平坦地面参考为 0", fontproperties=font)
    for ax in axes:
        ax.set(xlabel="world x / m", xlim=(xy[row, 0].min(), xy[row, 0].max()))
        for edge in [-1.5, 1.5]:
            ax.axvline(edge, color=".75", linewidth=.8, linestyle=":")
        ax.grid(alpha=.15)
        legend_font = font.copy()
        legend_font.set_size(9)
        ax.legend(prop=legend_font, loc="upper center", bbox_to_anchor=(.5, -.18), frameon=False)
    fig.suptitle("M=125 · 单帧完整无噪声几何 · y=0.1 m 剖面；虚线竖线为桌面边界", fontproperties=font)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def write_report(output, protocol, summaries, controls):
    selected = [r for r in summaries if r["inducing_points"] == 125]
    lines = ["# 大桌子场景：FSGP-BGK 实测", "",
             "按论文 v2 公式 (1)–(11) 的现有 CPU 参考实现运行，没有修改核心算法或为桌子调参。",
             "这不是作者 GPU 实现、论文 2500 场景基准或导航成功率复现。", "",
             "## 输入和参考", "",
             "- 桌面 3×2 m，总高 1.2 m，桌下净高 1.1 m，四条腿截面 0.12×0.12 m；地面 z=0。",
             "- 主实验保留桌面、桌底、桌腿和地面的全部高度层；不做地面分割、不删除悬空桌面。",
             "- 每帧局部点云先随机取最多 2500 点，再随机丢弃 15%，添加 0.015 m 高程标准差噪声。",
             "  这是点随机丢失模型，没有模拟真实 LiDAR 射线遮挡；同一帧各方法和各 M 输入完全一致。",
             "- 5×5 m 局部窗口、0.2 m 栅格、8 帧、帧间隔 0.1 s，窗口中心 x 从 -0.8 到 0.8 m。",
             "- 独立参考为平坦地面：理想曲率、高差、坡度都为 0，故式 (8) 的参考代价为 0。",
             "  凡栅格与桌腿相交或超出完整地面范围，参考值设为 NaN，不参与代价偏差计算。",
             "- MAE/RMSE 是有效平坦地面上的代价偏差，未归一化，不能当成论文精度或碰撞分类准确率。",
             "- 桌下内部区域定义为 |x|≤1 m、|y|≤0.5 m，避开桌腿和桌面边缘。", "",
             "## 连续八帧汇总", "",
             "| M | 方法 | 地面代价 MAE | 地面代价 RMSE | 中位耗时 ms | 桌下预测高程中位数 m | 桌下坡度均值 | 桌下代价均值 |",
             "|---:|---|---:|---:|---:|---:|---:|---:|"]
    for r in summaries:
        lines.append(f"| {r['inducing_points']} | {r['method']} | {r['cost_mae']:.5f} | {r['cost_rmse']:.5f} | "
                     f"{r['median_runtime_ms']:.2f} | {r['under_height_median_m']:.3f} | "
                     f"{r['under_slope_mean']:.3f} | {r['under_cost_mean']:.5f} |")
    lines += ["", "高程和坡度属于当前帧 GP，FSGP 与 FSGP-BGK 共用；BGK 只融合同一二维单元的代价/方差。",
              "耗时包含特征、GP、评分/平滑，BGK 额外包含融合；排除扫描抽样、文件读写、绘图和 ROS。", "",
              "## 无噪声对照（M=125，单帧）", "",
              "| 输入 | 裁剪后点数 | 桌下高程中位数 m | 桌下高程绝对误差均值 m | 桌下坡度均值 | 桌下代价均值 |",
              "|---|---:|---:|---:|---:|---:|"]
    for r in controls:
        lines.append(f"| {r['input']} | {r['points_after_local_crop']} | {r['under_height_median_m']:.4f} | "
                     f"{r['under_height_abs_error_mean_m']:.4f} | {r['under_slope_mean']:.4f} | {r['under_cost_mean']:.5f} |")
    fsgp = next(r for r in selected if r["method"] == "FSGP")
    bgk = next(r for r in selected if r["method"] == "FSGP-BGK")
    lines += ["", "## 结果解释", "",
              f"M=125 时，桌下内部的真实地面高程为 0，而预测高程中位数为 {fsgp['under_height_median_m']:.3f} m；"
              f"坡度比均值为 {fsgp['under_slope_mean']:.3f}。",
              f"同一区域的 FSGP/BGK 代价均值分别为 {fsgp['under_cost_mean']:.5f}/{bgk['under_cost_mean']:.5f}。",
              "这些值说明当前实现对多层输入的响应；单个高程/代价栅格没有输出桌底高度或净空。",
              "不能只凭代价高低宣称机器人能穿桌或必然碰撞，也没有以调阈值方式制造成功率。",
              "移除桌面和腿的 floor_only_clean 是单独标明的诊断对照，不是对主实验输入进行预处理。", "",
              "在 (x,y)=(0,0) 处，地面/桌底/桌面三个点的 κ、g 都为 0，GP 输入 (x,y,κ,g) 相同，",
              "目标高程却分别为 0、1.1、1.2 m；详见 `multilayer_ambiguity.json`。",
              "这说明这一输入表示本身存在多值目标，不能从单个栅格高程直接恢复三层表面。", "",
              "![无噪声剖面对照](controls.png)", "",
              "## 图和复核", ""]
    for count in protocol["inducing_counts"]:
        lines.append(f"![M={count} 第八帧](table_m{count}.png)")
    lines += ["", "`per_frame.csv` / `summary.csv` 保存原始指标；`scans/` 保存每一帧实际输入；",
              "`frames/` 保存各 M、各方法的每一帧地图，根目录 `table_m*_*.npz` 为第八帧。",
              "`protocol.json` 固定算法、场景、参数与环境哈希；`audit.json` 为独立重算复核。", "",
              "方法来源：https://arxiv.org/html/2503.04134v2", ""]
    (output / "REPORT.md").write_text("\n".join(lines))


def evaluate(output=ROOT / "outputs/table_cpu", scene_folder=DEFAULT_OUTPUT):
    started = perf_counter()
    output, scene_folder = Path(output), Path(scene_folder)
    output.mkdir(parents=True, exist_ok=True)
    if not (scene_folder / "large_table.npz").exists():
        export_scene(scene_folder)
    scene = json.loads((scene_folder / "scene.json").read_text())
    with np.load(scene_folder / "large_table.npz", allow_pickle=False) as data:
        points, labels = data["points"].astype(float), data["labels"]
    validate_cloud(points, labels, scene)
    np.testing.assert_array_equal(read_cloud(scene_folder / "large_table.pcd"), points)
    config = Config(**json.loads((ROOT / "configs/paper_cpu.json").read_text()))
    frames, counts = 8, [50, 125, 500]
    scans = []
    rng = np.random.default_rng(config.seed + 5000)
    (output / "scans").mkdir(exist_ok=True)
    for frame in range(frames):
        center, timestamp = (-.8 + 1.6*frame/(frames-1), 0), frame*.1
        scan = sample_scan(points, center, config, rng)
        scans.append((scan, center, timestamp))
        np.savez_compressed(output / "scans" / f"frame_{frame:03d}.npz",
                            points=scan, center=center, timestamp=timestamp)
    rows, summaries = [], []
    for count in counts:
        current_config = replace(config, inducing_points=count)
        sgp_mapper, fsgp_mapper = TraversabilityMapper(current_config), TraversabilityMapper(current_config)
        frame_folder = output / "frames" / f"m{count}"
        frame_folder.mkdir(parents=True, exist_ok=True)
        all_under = {suffix: {key: [] for key in ("height", "slope", "height_variation", "cost")}
                     for _, suffix in METHODS}
        for frame, (scan, center, timestamp) in enumerate(scans):
            sgp = sgp_mapper.assess(scan, center, timestamp, feature_based=False)
            fsgp = fsgp_mapper.assess(scan, center, timestamp)
            maps = {"sgp": sgp, "fsgp": fsgp, "bgk": fsgp_mapper.fuse(fsgp)}
            reference = floor_reference(fsgp["xy"], fsgp["shape"], scene, config.resolution)
            common = sgp["observed"] & fsgp["observed"] & np.isfinite(reference.ravel())
            for method, suffix in METHODS:
                result = maps[suffix]
                save_result(frame_folder / f"frame_{frame:03d}_{suffix}.npz", result)
                rows.append({"scene": "table", "inducing_points": count, "method": method,
                             "frame": frame, "timestamp": timestamp, "scan_points": len(scan),
                             "scan_sha256": hashlib.sha256(scan.tobytes()).hexdigest(),
                             "center_x": center[0], "center_y": center[1],
                             "actual_inducing_points": len(result["inducing"]),
                             "evaluation_cells": int(common.sum()), **errors(result, reference, common),
                             "current_coverage": float(result["observed"].mean()),
                             "known_coverage": float(result["known"].mean()),
                             "runtime_ms": result["timing_ms"]["total"], **region_stats(result, reference)})
                under = common & interior_mask(result["xy"])
                for key in all_under[suffix]:
                    all_under[suffix][key].extend(np.asarray(result[key]).ravel()[under].tolist())
            print(f"table M={count} frame={frame+1}/{frames}: BGK floor cost MAE={rows[-1]['cost_mae']:.5f}", flush=True)
        prefix = output / f"table_m{count}"
        for _, suffix in METHODS:
            save_result(str(prefix) + f"_{suffix}.npz", maps[suffix])
        np.savez_compressed(str(prefix) + "_evidence.npz", points=scan, reference_cost=reference)
        plot_result(str(prefix) + ".png", maps, reference, scene, count)
        for method, suffix in METHODS:
            subset = [r for r in rows if r["inducing_points"] == count and r["method"] == method]
            weights = np.asarray([r["evaluation_cells"] for r in subset])
            under = all_under[suffix]
            summaries.append({"scene": "table", "inducing_points": count, "method": method,
                              "cost_mae": float(np.average([r["cost_mae"] for r in subset], weights=weights)),
                              "cost_rmse": float(np.sqrt(np.average([r["cost_rmse"]**2 for r in subset], weights=weights))),
                              "evaluated_cell_frames": int(weights.sum()),
                              "median_runtime_ms": float(np.median([r["runtime_ms"] for r in subset])),
                              "mean_current_coverage": float(np.mean([r["current_coverage"] for r in subset])),
                              "mean_known_coverage": float(np.mean([r["known_coverage"] for r in subset])),
                              "under_height_median_m": float(np.median(under["height"])),
                              "under_height_abs_error_mean_m": float(np.abs(under["height"]).mean()),
                              "under_slope_mean": float(np.mean(under["slope"])),
                              "under_height_variation_mean_m": float(np.mean(under["height_variation"])),
                              "under_cost_mean": float(np.mean(under["cost"]))})
    controls = isolation_controls(output, points, labels, scene, config)
    write_csv(output / "per_frame.csv", rows)
    write_csv(output / "summary.csv", summaries)
    protocol = {"scope": "paper-equation CPU reference; analytic floor diagnostic, not official benchmark",
                "config": config.as_dict(), "scenes": ["table"], "inducing_counts": counts, "frames": frames,
                "frame_interval_seconds": .1, "noise_z_std_m": .015, "random_point_dropout": .15,
                "sensor_points_before_dropout_cap": 2500, "source_points": len(points),
                "source_pcd": str((scene_folder / "large_table.pcd").resolve()),
                "source_pcd_sha256": hashlib.sha256((scene_folder / "large_table.pcd").read_bytes()).hexdigest(),
                "reference": "analytic z=0 floor with zero ideal curvature, height variation and slope; leg-intersecting/out-of-floor cells excluded",
                "evaluation_mask": "common SGP-XY/FSGP observed support and valid analytic floor reference",
                "under_region": "|x| <= 1 m and |y| <= 0.5 m, observed cells only",
                "input_preprocessing": "all surface layers retained; random dropout, no visibility ray simulation or ground segmentation",
                "timing": "algorithm only, includes BGK for BGK; excludes scan preparation, IO, plotting, ROS",
                "environment": environment(), "wall_seconds": perf_counter()-started,
                "implementation_sha256": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                                          for name in ("core.py", "experiment.py", "io.py", "__main__.py", "evaluate_table_scene.py")},
                "source_lock_sha256": hashlib.sha256((ROOT / "sources.lock.json").read_bytes()).hexdigest()}
    (output / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    write_report(output, protocol, summaries, controls)
    from .audit_results import audit
    audit(output)
    print(json.dumps({"M125": [r for r in summaries if r["inducing_points"] == 125], "controls": controls}, indent=2))
    return summaries


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/table_cpu")
    parser.add_argument("--scene", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    evaluate(args.output, args.scene)

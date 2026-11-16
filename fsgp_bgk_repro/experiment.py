"""Deterministic diagnostic experiments; NOT the unreleased paper benchmark."""

from dataclasses import replace
from pathlib import Path
from time import perf_counter
import csv
import hashlib
import json
import os
import platform
import sys

import numpy as np
import scipy
from scipy.spatial import cKDTree

from .core import (Config, TraversabilityMapper, extract_features, interpolate_features,
                   smooth_known)
from .io import read_cloud, save_result

ROOT = Path(__file__).resolve().parent
SCENES = ("flat", "ramp", "steps", "hill", "quick")


def voxel_representatives(points, size):
    _, keep = np.unique(np.floor(points / size).astype(np.int64), axis=0, return_index=True)
    return points[keep]


def make_scene(name):
    if name in ("hill", "quick"):
        # Retain a compact full-data reference; avoid quadratic GPU KNN allocation.
        return voxel_representatives(read_cloud(ROOT / "data" / f"{name}.pcd"), 0.06)
    axis = np.arange(-4.0, 4.001, 0.08)
    x, y = np.meshgrid(axis, axis)
    if name == "flat":
        z = np.zeros_like(x)
    elif name == "ramp":
        z = 0.25 * x + 0.10 * y
    elif name == "steps":
        z = 0.45 * (x > 0.15) + 0.15 * np.exp(-((x + 1.2)**2 + (y - 0.5)**2) / 0.4)
    else:
        raise ValueError(f"unknown scene {name}")
    return np.column_stack((x.ravel(), y.ravel(), z.ravel()))


class GeometricReference:
    """Dense clean-cloud features plus local plane normals, independent of GP.

    Official scenes have no released cost labels. This is a declared diagnostic
    reference, not author ground truth. Analytic normals are used on flat/ramp.
    """

    def __init__(self, points, config, scene):
        self.features = extract_features(points, config.neighbors)
        _, indices = cKDTree(points).query(points, k=min(config.neighbors, len(points)))
        centered = points[indices] - points[indices].mean(axis=1, keepdims=True)
        covariance = np.einsum("nki,nkj->nij", centered, centered)
        _, vectors = np.linalg.eigh(covariance)
        normal = vectors[:, :, 0]
        self.slope = np.linalg.norm(normal[:, :2], axis=1) / np.maximum(np.abs(normal[:, 2]), 1e-3)
        if scene == "flat":
            self.slope[:] = 0
        elif scene == "ramp":
            self.slope[:] = np.hypot(0.25, 0.10)
        self.config = config

    def evaluate(self, xy, shape):
        c = self.config
        test, distance = interpolate_features(self.features, xy, c.interpolation_neighbors)
        distances, indices = cKDTree(self.features[:, :2]).query(xy, k=c.interpolation_neighbors)
        weight = 1 / (distances + 1e-6)
        weight /= weight.sum(axis=1, keepdims=True)
        slope = np.sum(weight * self.slope[indices], axis=1)
        cost = c.weights[0] * test[:, 2] + c.weights[1] * test[:, 3] + c.weights[2] * slope
        known = distance <= c.support_radius
        return smooth_known(cost.reshape(shape), known.reshape(shape), c.smoothing_sigma)


def sample_scan(points, center, config, rng, max_points=2500, occlusion=0.15, noise=0.015):
    margin = config.length / 2 + config.support_radius
    keep = np.max(np.abs(points[:, :2] - np.asarray(center)), axis=1) <= margin
    local = points[keep]
    # Cap sensor density first, then withhold the declared fraction of this frame.
    if len(local) > max_points:
        local = local[rng.choice(len(local), max_points, replace=False)]
    count = int(np.floor(len(local) * (1 - occlusion)))
    if count < 3:
        raise ValueError("scan has fewer than three points after occlusion")
    local = local[rng.choice(len(local), count, replace=False)].copy()
    local[:, 2] += rng.normal(0, noise, count)
    return local


def environment():
    cpu = platform.processor()
    cpuinfo = Path("/proc/cpuinfo")
    if cpuinfo.exists():
        cpu = next((line.split(":", 1)[1].strip() for line in cpuinfo.read_text().splitlines()
                    if line.startswith("model name")), cpu)
    return {"python": sys.version, "executable": sys.executable,
            "numpy": np.__version__, "scipy": scipy.__version__,
            "platform": platform.platform(), "processor": cpu,
            "thread_environment": {key: os.environ.get(key) for key in
                                   ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")},
            "backend": "CPU / NumPy + SciPy", "cuda_used": False}


def errors(result, reference, mask):
    delta = result["cost"].ravel()[mask] - reference.ravel()[mask]
    if len(delta) == 0:
        return {"cost_mae": None, "cost_rmse": None, "signed_error_variance": None}
    return {"cost_mae": float(np.abs(delta).mean()),
            "cost_rmse": float(np.sqrt(np.mean(delta**2))),
            "signed_error_variance": float(np.var(delta))}


def write_csv(path, rows):
    with open(path, "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_frame(path, scene, points, sgp, fsgp, fused, reference, frame_rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 4, figsize=(17, 8), constrained_layout=True)
    xy = fused["xy"]
    resolution = xy[1, 0] - xy[0, 0] if fused["shape"][1] > 1 else 0.2
    half = resolution / 2
    extent = [xy[:, 0].min() - half, xy[:, 0].max() + half,
              xy[:, 1].min() - half, xy[:, 1].max() + half]
    axes[0, 0].scatter(points[:, 0], points[:, 1], c=points[:, 2], s=2, cmap="terrain")
    axes[0, 0].set_title(f"{scene}: observed scan (height)")
    axes[0, 1].scatter(points[:, 0], points[:, 1], c="0.85", s=1)
    inducing = fsgp["inducing"]
    axes[0, 1].scatter(inducing[:, 0], inducing[:, 1], c=inducing[:, 2], s=12, cmap="terrain")
    axes[0, 1].set_title(f"FSGP inducing points: {len(inducing)}")
    arrays = [reference, sgp["cost"], fsgp["cost"], fused["cost"]]
    finite = np.concatenate([a[np.isfinite(a)] for a in arrays])
    maximum = max(float(np.quantile(finite, 0.99)), 0.03)
    for ax, values, title in zip([axes[0, 2], axes[0, 3], axes[1, 0], axes[1, 1]], arrays,
                                ["Dense geometric reference", "SGP-XY control", "FSGP single frame", "FSGP + BGK (paper equations)"]):
        im = ax.imshow(values, origin="lower", extent=extent, cmap="magma", vmin=0, vmax=maximum)
        ax.set_title(title)
        fig.colorbar(im, ax=ax, shrink=0.8, label="cost (higher = harder)")
    im = axes[1, 2].imshow(fused["variance"].reshape(fused["shape"]), origin="lower",
                           extent=extent, cmap="viridis", vmin=0)
    axes[1, 2].set_title("Single-frame latent GP variance")
    fig.colorbar(im, ax=axes[1, 2], shrink=0.8)
    for method in ("SGP-XY", "FSGP", "FSGP-BGK"):
        rows = [r for r in frame_rows if r["method"] == method]
        axes[1, 3].plot([r["frame"] for r in rows], [r["cost_mae"] for r in rows], label=method)
    axes[1, 3].set(xlabel="frame", ylabel="cost MAE on common current support",
                    title="Diagnostic errors; not paper Table I")
    axes[1, 3].legend(fontsize=8)
    for ax in axes.ravel()[:-1]:
        ax.set(xlabel="world x (m)", ylabel="world y (m)", aspect="equal")
    fig.savefig(path, dpi=160)
    plt.close(fig)


def benchmark(output, scenes, inducing_counts, frames=8, base_config=None):
    if frames < 1:
        raise ValueError("frames must be positive")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    base_config = base_config or Config()
    all_rows, summaries = [], []
    start = perf_counter()
    for scene in scenes:
        scene_index = SCENES.index(scene)
        points = make_scene(scene)
        reference_model = GeometricReference(points, base_config, scene)
        for count in inducing_counts:
            config = replace(base_config, inducing_points=count)
            sgp_mapper, fsgp_mapper = TraversabilityMapper(config), TraversabilityMapper(config)
            # Reset sensor RNG for each M: every ablation receives identical scans.
            sensor_rng = np.random.default_rng(config.seed + 1000 * scene_index)
            rows = []
            for frame in range(frames):
                center = (-0.8 + 1.6 * frame / max(frames - 1, 1), 0.0)
                scan = sample_scan(points, center, config, sensor_rng)
                scan_digest = hashlib.sha256(scan.tobytes()).hexdigest()
                timestamp = frame * 0.1
                sgp = sgp_mapper.assess(scan, center, timestamp, feature_based=False)
                fsgp = fsgp_mapper.assess(scan, center, timestamp)
                # Reuse EXACTLY the same FSGP prediction for the temporal ablation.
                fused = fsgp_mapper.fuse(fsgp)
                reference = reference_model.evaluate(fsgp["xy"], fsgp["shape"])
                common = sgp["observed"] & fsgp["observed"] & np.isfinite(reference.ravel())
                for method, result in (("SGP-XY", sgp), ("FSGP", fsgp), ("FSGP-BGK", fused)):
                    row = {"scene": scene, "inducing_points": count, "method": method,
                           "frame": frame, "timestamp": timestamp, "scan_points": len(scan),
                           "scan_sha256": scan_digest, "center_x": center[0], "center_y": center[1],
                           "actual_inducing_points": len(result["inducing"]),
                           "evaluation_cells": int(common.sum()), **errors(result, reference, common),
                           "current_coverage": float(result["observed"].mean()),
                           "known_coverage": float(result["known"].mean()),
                           "runtime_ms": result["timing_ms"]["total"]}
                    rows.append(row)
                print(f"{scene} M={count} frame={frame + 1}/{frames}: " +
                      ", ".join(f"{r['method']} MAE={format_metric(r['cost_mae'])}" for r in rows[-3:]), flush=True)
            all_rows.extend(rows)
            prefix = output / f"{scene}_m{count}"
            for method, result in (("sgp", sgp), ("fsgp", fsgp), ("bgk", fused)):
                save_result(str(prefix) + f"_{method}.npz", result)
            np.savez_compressed(str(prefix) + "_evidence.npz", points=scan, reference_cost=reference)
            plot_frame(str(prefix) + ".png", scene, scan, sgp, fsgp, fused, reference, rows)
            for method in ("SGP-XY", "FSGP", "FSGP-BGK"):
                subset = [r for r in rows if r["method"] == method]
                metrics = [r for r in subset if r["cost_mae"] is not None]
                total_cells = sum(r["evaluation_cells"] for r in metrics)
                # Pool cells across frames, not an unweighted average of frame RMSEs.
                mae = (sum(r["cost_mae"] * r["evaluation_cells"] for r in metrics) /
                       total_cells) if total_cells else None
                rmse = np.sqrt(sum(r["cost_rmse"]**2 * r["evaluation_cells"] for r in metrics) /
                               total_cells) if total_cells else None
                summaries.append({"scene": scene, "inducing_points": count, "method": method,
                                  "cost_mae": mae, "cost_rmse": float(rmse) if rmse is not None else None,
                                  "evaluated_cell_frames": total_cells,
                                  "median_runtime_ms": float(np.median([r["runtime_ms"] for r in subset])),
                                  "mean_current_coverage": float(np.mean([r["current_coverage"] for r in subset])),
                                  "mean_known_coverage": float(np.mean([r["known_coverage"] for r in subset]))})
    write_csv(output / "per_frame.csv", all_rows)
    write_csv(output / "summary.csv", summaries)
    protocol = {"scope": "paper-equation CPU reproduction and diagnostic benchmark; not author benchmark",
                "config": base_config.as_dict(), "scenes": list(scenes), "inducing_counts": list(inducing_counts),
                "frames": frames, "frame_interval_seconds": 0.1, "noise_z_std_m": 0.015,
                "random_point_dropout": 0.15, "sensor_points_before_dropout_cap": 2500,
                "reference_voxel_m": 0.06, "evaluation_mask": "intersection of SGP-XY and FSGP current support and finite reference",
                "timing": "cold-start-inclusive algorithm time; excludes reference generation, loading and plotting; no GPU",
                "environment": environment(), "wall_seconds": perf_counter() - start,
                "implementation_sha256": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                                          for name in ("core.py", "experiment.py", "io.py", "__main__.py")},
                "source_lock_sha256": hashlib.sha256((ROOT / "sources.lock.json").read_bytes()).hexdigest()}
    (output / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    write_report(output, summaries, protocol)
    return summaries


def format_metric(value):
    return f"{value:.6f}" if value is not None else "N/A"


def write_report(output, summaries, protocol):
    text = ["# FSGP-BGK CPU 方法复现结果", "",
            "本报告由实际运行生成。使用论文 arXiv v2 公式 (1)–(11) 的 CPU 参考实现。",
            "官方 hill/quick 点云已固定版本；flat/ramp/steps 为本地生成的诊断地形。",
            "**这些 MAE 与耗时不能当作论文 Table I–III 的复现指标。** 官方未发布原始 2500 场景、",
            "完整标签生成公式及批量评测程序；本实验也没有使用论文的 RTX 4060。", "",
            "## 本次实测", "",
            "| 场景 | M | 方法 | cost MAE ↓ | cost RMSE ↓ | 中位耗时 ms | 当前覆盖率 | 含历史覆盖率 |",
            "|---|---:|---|---:|---:|---:|---:|---:|"]
    for row in summaries:
        text.append(f"| {row['scene']} | {row['inducing_points']} | {row['method']} | "
                    f"{format_metric(row['cost_mae'])} | {format_metric(row['cost_rmse'])} | {row['median_runtime_ms']:.2f} | "
                    f"{row['mean_current_coverage']:.1%} | {row['mean_known_coverage']:.1%} |")
    pairs = []
    for row in summaries:
        if row["method"] == "FSGP-BGK":
            prior = next(r for r in summaries if r["scene"] == row["scene"] and
                         r["inducing_points"] == row["inducing_points"] and r["method"] == "FSGP")
            if row["cost_mae"] is not None and prior["cost_mae"] is not None:
                pairs.append((row, row["cost_mae"] < prior["cost_mae"]))
    improvements = sum(improved for _, improved in pairs)
    text += ["", f"BGK 在 {len(pairs)} 个场景/诱导点配置中有 {improvements} 个配置降低了相对 FSGP 的 MAE。",
             "未强制调参以得到论文宣称的优势；不改善的结果同样保留。", "",
             "## 图与证据", ""]
    for scene in protocol["scenes"]:
        for count in protocol["inducing_counts"]:
            text.append(f"![{scene}, M={count}]({scene}_m{count}.png)")
    text += ["", "## 评测解释", "",
             "- 参考 cost：完整无噪声点云的 KNN 曲率、平均绝对高差及局部 PCA 法向坡度；",
             "  平面和斜坡使用解析坡度。该参考不使用 GP。官方点云没有 cost 标签，故它是诊断参考。",
             "- 每帧相同扫描输入 SGP-XY 与 FSGP；BGK 直接融合同一份 FSGP 输出。不同 M 使用同一组扫描。",
             "- SGP-XY 是去除 GP 特征输入及特征采样的本地对照，评分仍使用相同曲率/高差；不等同于论文引用基线的官方实现。",
             "- 相同当前可观测格子参与各方法误差评测，未知格子不当作零代价。历史覆盖率单独报告。",
             "- cost 按式 (8) 使用原始属性，越高越难通行；论文未明确给出所有量纲归一化细节。",
             "- BGK 严格使用式 (9),(10)，仅旧方差进入式 (9) 的置信度。很小的旧方差可能令地图更新缓慢。",
             "- 所有方法使用相同的二维高斯平滑。BGK 方差是论文启发式融合量，不是严格校准的 cost 后验。",
             "- 延迟包括特征提取、GP、评分和平滑，BGK 包括额外融合；不含文件读取、参考计算及绘图。",
             "- CPU 单线程 BLAS；代码没有实现 GPU 加速，不能据此声称达到论文实时性能。", "",
             "## 运行环境与参数", "", "```json", json.dumps(protocol, ensure_ascii=False, indent=2), "```", "",
             "原始逐帧指标见 `per_frame.csv`，汇总见 `summary.csv`，末帧各层见 `*_sgp/fsgp/bgk.npz`。",
             "每幅图的扫描及参考 cost 保存在 `*_evidence.npz`，可独立重算图示末帧误差。", ""]
    (output / "REPORT.md").write_text("\n".join(text))

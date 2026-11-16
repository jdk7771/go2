"""Analyze saved hill terrain features without altering the mapper or benchmark.

Primary population: observed grid cells of hill / M=125 / final (8th) frame.
Correlations are descriptive; neighboring cells share data and are not
independent samples. No iid p-values or inferential intervals are reported.
"""

from pathlib import Path
import csv
import hashlib
import json
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/fsgp-bgk-matplotlib")

import numpy as np
from scipy.spatial import cKDTree
from scipy.stats import linregress, pearsonr, rankdata, spearmanr

from .core import extract_features, interpolate_features
from .experiment import ROOT


def correlations(slope, variation):
    """Reconcile library outputs with centered dot products and rank Pearson."""
    x, y = np.asarray(slope, float), np.asarray(variation, float)
    centered_x, centered_y = x - x.mean(), y - y.mean()
    manual = float(centered_x @ centered_y / np.sqrt((centered_x @ centered_x) *
                                                   (centered_y @ centered_y)))
    pearson = float(pearsonr(x, y)[0])
    spearman = float(spearmanr(x, y)[0])
    np.testing.assert_allclose(manual, pearson, atol=1e-14)
    np.testing.assert_allclose(pearsonr(rankdata(x), rankdata(y))[0], spearman, atol=1e-14)
    fit = linregress(x, y)
    coefficients, *_ = np.linalg.lstsq(np.column_stack((np.ones(len(x)), x)), y, rcond=None)
    np.testing.assert_allclose(coefficients, [fit.intercept, fit.slope], atol=1e-12)
    return {"n_cells": len(x), "pearson_r": pearson, "spearman_rho": spearman,
            "linear_fit_g_m": {"intercept_m": float(fit.intercept),
                               "coefficient_m_per_slope": float(fit.slope),
                               "in_sample_r_squared": float(fit.rvalue**2)},
            "slope_range": [float(x.min()), float(x.max())],
            "height_variation_m_range": [float(y.min()), float(y.max())]}


def main():
    run = ROOT / "outputs" / "cpu"
    output = ROOT / "outputs" / "visualization" / "hill-feature-analysis"
    output.mkdir(parents=True, exist_ok=True)
    datasets = {}
    source_files = []
    for count in (50, 125, 500):
        path = run / f"hill_m{count}_fsgp.npz"
        source_files.append(path)
        with np.load(path, allow_pickle=False) as data:
            datasets[count] = {name: data[name] for name in data.files}
    data = datasets[125]
    source_files.append(run / "hill_m125_evidence.npz")
    with np.load(source_files[-1], allow_pickle=False) as evidence:
        points = evidence["points"]
    valid = (data["observed"] & np.isfinite(data["slope"]) &
             np.isfinite(data["height_variation"]))
    assert len(np.unique(data["cell_ids"], axis=0)) == len(data["cell_ids"])
    primary = correlations(data["slope"][valid], data["height_variation"][valid])

    # Reproduce g directly from the saved scan, independent of the saved g layer.
    features = extract_features(points, 20)
    interpolated, _ = interpolate_features(features, data["xy"], 5)
    np.testing.assert_array_equal(interpolated[:, 3], data["height_variation"])

    # Alternative geometric slope: same scan, same 20-point neighborhood,
    # orthogonal plane fit, then the same 5-neighbor XY interpolation.
    _, indices = cKDTree(points).query(points, k=20)
    local = points[indices]
    centered = local - local.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered)
    _, eigenvectors = np.linalg.eigh(covariance)
    normal = eigenvectors[:, :, 0]
    plane_slope = np.linalg.norm(normal[:, :2], axis=1) / np.maximum(np.abs(normal[:, 2]), 1e-3)
    mean_horizontal_distance = np.linalg.norm(local[:, :, :2] - points[:, None, :2], axis=2).mean(axis=1)
    distances, neighbor = cKDTree(points[:, :2]).query(data["xy"], k=5)
    weights = 1 / (distances + 1e-6)
    weights /= weights.sum(axis=1, keepdims=True)
    grid_plane_slope = np.sum(weights * plane_slope[neighbor], axis=1)
    grid_horizontal_distance = np.sum(weights * mean_horizontal_distance[neighbor], axis=1)

    common = np.logical_and.reduce([d["observed"] for d in datasets.values()])
    sensitivity = {}
    for count, other in datasets.items():
        np.testing.assert_array_equal(other["xy"], data["xy"])
        np.testing.assert_array_equal(other["height_variation"], data["height_variation"])
        sensitivity[str(count)] = correlations(other["slope"][common], other["height_variation"][common])

    # Aggregate into 25 disjoint 1m x 1m blocks as a spatial-scale diagnostic.
    # Aggregation does not prove independence between the resulting blocks.
    shape = tuple(map(int, data["shape"]))
    assert shape == (25, 25) and valid.all()
    block_s = data["slope"].reshape(shape).reshape(5, 5, 5, 5).mean(axis=(1, 3)).ravel()
    block_g = data["height_variation"].reshape(shape).reshape(5, 5, 5, 5).mean(axis=(1, 3)).ravel()

    fit = primary["linear_fit_g_m"]
    predicted = fit["intercept_m"] + fit["coefficient_m_per_slope"] * data["slope"]
    residual = data["height_variation"] - predicted
    # A: largest negative residual; B: largest positive residual among points
    # whose GP slope differs by <=10% from A. These illustrate departures from
    # proportionality, not representative average effects.
    index_a = int(np.flatnonzero(valid)[np.argmin(residual[valid])])
    candidates = np.flatnonzero(valid & (np.abs(data["slope"] / data["slope"][index_a] - 1) <= 0.10))
    index_b = int(candidates[np.argmax(residual[candidates])])
    examples = []
    for label, index in (("A", index_a), ("B", index_b)):
        examples.append({"label": label, "grid_index": index, "xy_m": data["xy"][index].tolist(),
                         "gp_slope": float(data["slope"][index]),
                         "gp_angle_degrees": float(np.degrees(np.arctan(data["slope"][index]))),
                         "height_variation_m": float(data["height_variation"][index]),
                         "height_variation_cm": float(100 * data["height_variation"][index]),
                         "local_plane_slope": float(grid_plane_slope[index]),
                         "mean_horizontal_neighbor_distance_m": float(grid_horizontal_distance[index]),
                         "gp_latent_variance": float(data["variance"][index]),
                         "curvature": float(data["curvature"][index])})
    rows = []
    for i in range(len(valid)):
        rows.append({"grid_index": i, "x_m": float(data["xy"][i, 0]), "y_m": float(data["xy"][i, 1]),
                     "observed": bool(valid[i]), "gp_slope": float(data["slope"][i]),
                     "height_variation_m": float(data["height_variation"][i]),
                     "local_plane_slope": float(grid_plane_slope[i]),
                     "mean_horizontal_neighbor_distance_m": float(grid_horizontal_distance[i]),
                     "linear_fit_residual_m": float(residual[i])})
    with (output / "grid_values.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)

    source_files.extend([ROOT / "core.py", run / "protocol.json", Path(__file__)])
    results = {"scene": "official hill point cloud", "method": "FSGP", "inducing_points": 125,
               "frame": 8, "timestamp_seconds": float(data["timestamp"]), "shape_yx": list(shape),
               "resolution_m": 0.2, "primary": primary, "examples": examples,
               "example_selection": "A=min negative OLS residual; B=max positive residual within +/-10% of A's GP slope",
               "example_height_variation_ratio_B_over_A": examples[1]["height_variation_m"] / examples[0]["height_variation_m"],
               "common_mask_inducing_sensitivity": sensitivity,
               "one_meter_block_means": correlations(block_s, block_g),
               "same_scan_local_plane_slope": correlations(grid_plane_slope[valid], data["height_variation"][valid]),
               "definitions": {"gp_slope": "norm of GP XY partial derivatives with interpolated kappa,g held fixed; m/m",
                               "height_variation": "20-neighbor mean(abs(z_neighbor-z_point)), including self, interpolated with 5-neighbor XY IDW; metres"},
               "limits": ["One scene and one frame; no population-wide inference",
                          "Adjacent cells share neighborhoods; no iid significance tests",
                          "g is itself a GP input; these are statistically coupled features",
                          "The local-plane estimator is a diagnostic, not independent ground truth",
                          "OLS R-squared is in-sample, not predictive validation"],
               "validation": {"height_variation_recomputed_exactly": True,
                              "correlations_and_OLS_independently_reconciled": True,
                              "duplicate_world_cells": 0},
               "sources": [{"path": str(path.relative_to(ROOT)),
                            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()} for path in source_files]}
    (output / "statistics.json").write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font_file = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if font_file.exists():
        font_manager.fontManager.addfont(str(font_file))
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(font_file)).get_name()
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["font.size"] = 10
    fig, axes = plt.subplots(1, 3, figsize=(17, 5.2), constrained_layout=True)
    xy = data["xy"]
    extent = [xy[:, 0].min() - .1, xy[:, 0].max() + .1, xy[:, 1].min() - .1, xy[:, 1].max() + .1]
    for ax, layer, cmap, title, unit in (
            (axes[0], data["slope"], "viridis", "GP 坡度", "坡度 / (m/m)"),
            (axes[1], data["height_variation"] * 100, "magma", "邻域平均绝对高差", "平均绝对高差 / cm")):
        im = ax.imshow(layer.reshape(shape), origin="lower", extent=extent, cmap=cmap, vmin=0)
        ax.set(title=title, xlabel="世界 x / m", ylabel="世界 y / m", aspect="equal")
        fig.colorbar(im, ax=ax, shrink=.85, label=unit)
        for label, index, marker in (("A", index_a, "s"), ("B", index_b, "^")):
            ax.scatter(*xy[index], marker=marker, s=95, facecolor="none", edgecolor="white", linewidth=1.8)
            ax.annotate(label, xy[index], xytext=(9, 1), textcoords="offset points", color="white", weight="bold")
    x, y = data["slope"][valid], data["height_variation"][valid] * 100
    ax = axes[2]
    ax.scatter(x, y, s=14, alpha=.5, color="#266e9a", linewidths=0)
    line_x = np.linspace(0, x.max(), 150)
    ax.plot(line_x, 100 * (fit["intercept_m"] + fit["coefficient_m_per_slope"] * line_x),
            color="#222222", linestyle="--", label="同帧线性拟合")
    for label, index, marker, color in (("A", index_a, "s", "#b7410e"), ("B", index_b, "^", "#7c3b88")):
        ax.scatter(data["slope"][index], data["height_variation"][index] * 100, marker=marker, s=85,
                   color=color, edgecolor="white", linewidth=.8, zorder=5)
        ax.annotate(label, (data["slope"][index], data["height_variation"][index] * 100),
                    xytext=(8, 4), textcoords="offset points", weight="bold", color=color)
    ax.set(title=f"625 个有效栅格：r={primary['pearson_r']:.3f}，ρ={primary['spearman_rho']:.3f}",
           xlabel="GP 坡度 / (m/m)，数值为坡度比", ylabel="邻域平均绝对高差 / cm", xlim=(0, x.max() * 1.03), ylim=(0, y.max() * 1.1))
    ax.grid(alpha=.18)
    ax.legend(loc="upper left")
    fig.suptitle("丘陵 hill · M=125 · 第8帧：坡度与高度变化较强正相关，但不成固定比例", fontsize=14)
    fig.savefig(output / "feature-correlation.png", dpi=170)
    plt.close(fig)
    print(json.dumps({"primary": primary, "examples": examples, "M_common_mask": sensitivity,
                      "block_pearson": results["one_meter_block_means"]["pearson_r"],
                      "local_plane_pearson": results["same_scan_local_plane_slope"]["pearson_r"],
                      "output": str(output)}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

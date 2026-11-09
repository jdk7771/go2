"""Compare saved point-selection runs without rerunning or rescoring plans."""

from . import experiment  # Set single-thread BLAS limits first.

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .settings import MODES
from .visualization.data import SavedRun
from .visualization.web import selection_label


def compare(paths: list[Path], output: Path, mode: str = "full") -> dict:
    runs = [SavedRun(path) for path in paths]
    reference = runs[0]
    if reference.metadata.get("point_selection", {}).get("method") != "all":
        raise ValueError("The first run must be the full-cloud timing reference")
    names = reference.scenes
    matched_keys = (
        "initial_sha256",
        "weights",
        "robot",
        "maxiter",
        "workers",
        "ftol",
        "gtol",
        "maxls",
        "activation_distance_m",
        "validation_subdivisions",
    )
    tables = []
    for run in runs:
        if run.scenes != names:
            raise ValueError("Runs must contain the same ordered scenes")
        for key in matched_keys:
            if run.metadata[key] != reference.metadata[key]:
                raise ValueError(f"Unmatched protocol {key}: {run.path}")
        table = {row["scene"]: row for row in run.rows if row["mode"] == mode}
        if len(table) != len(names):
            raise ValueError(f"Missing {mode} trials in {run.path}")
        tables.append(table)
    summaries, paired = [], []
    baseline_time = sum(tables[0][name]["total_planning_seconds"] for name in names)
    for run, table in zip(runs, tables):
        rows = [table[name] for name in names]
        for name, row in zip(names, rows):
            baseline = tables[0][name]
            for key in ("cloud_sha256", "initial_sha256"):
                if row[key] != baseline[key]:
                    raise ValueError(f"Unmatched {key}: {name}")
            with np.load(reference.path / "trials" / f"{name}__{mode}.npz") as data:
                original = data["optimized_trajectory"]
            with np.load(run.path / "trials" / f"{name}__{mode}.npz") as data:
                final = data["optimized_trajectory"]
            paired.append(
                {
                    "run": run.path.name,
                    "scene": name,
                    "clearance_mm": row["minimum_signed_distance_m"] * 1000,
                    "clearance_change_mm": (
                        row["minimum_signed_distance_m"]
                        - baseline["minimum_signed_distance_m"]
                    )
                    * 1000,
                    "total_planning_seconds": row["total_planning_seconds"],
                    "speedup": baseline["total_planning_seconds"]
                    / row["total_planning_seconds"],
                    "wall_seconds": row["wall_seconds"],
                    "iterations": row["iterations"],
                    "evaluations": row["evaluations"],
                    "formal_success": row["formal_success"],
                    "mean_selected_points_per_pose": row[
                        "mean_selected_points_per_pose"
                    ],
                    "retained_fraction": row["mean_selected_points_per_pose"]
                    / row["planning_cloud_points"],
                    "selection_query_seconds": row["selection_query_seconds"],
                    "maximum_node_position_change_m": float(
                        np.linalg.norm(final[:, :3] - original[:, :3], axis=1).max()
                    ),
                    "maximum_node_angle_change_deg": float(
                        np.degrees(np.abs(final[:, 3:] - original[:, 3:])).max()
                    ),
                }
            )
        seconds = sum(r["total_planning_seconds"] for r in rows)
        summaries.append(
            {
                "run": run.path.name,
                "label": selection_label(run.metadata),
                "point_selection": run.metadata["point_selection"],
                "collision_free": sum(r["collision_free_strict"] for r in rows),
                "clearance_20mm": sum(r["clearance_20mm_strict"] for r in rows),
                "continuous_20mm_certificates": sum(
                    r["conservative_continuous_20mm_certificate"] for r in rows
                ),
                "formal_convergence": sum(r["formal_success"] for r in rows),
                "total_planning_seconds": seconds,
                "median_planning_seconds": float(
                    np.median([r["total_planning_seconds"] for r in rows])
                ),
                "speedup_total": baseline_time / seconds,
                "mean_retained_fraction": float(
                    np.mean(
                        [
                            r["mean_selected_points_per_pose"]
                            / r["planning_cloud_points"]
                            for r in rows
                        ]
                    )
                ),
                "mean_selected_points_per_pose": float(
                    np.mean([r["mean_selected_points_per_pose"] for r in rows])
                ),
                "solver_ms_per_evaluation": 1000
                * sum(r["wall_seconds"] for r in rows)
                / sum(r["evaluations"] for r in rows),
                "total_selection_query_seconds": sum(
                    r["selection_query_seconds"] for r in rows
                ),
                "total_index_setup_seconds": sum(
                    r["selection_setup_seconds"] for r in rows
                ),
                "total_iterations": sum(r["iterations"] for r in rows),
                "total_evaluations": sum(r["evaluations"] for r in rows),
                "median_projected_gradient_inf_norm": float(
                    np.median([r["projected_gradient_inf_norm"] for r in rows])
                ),
            }
        )
    output.mkdir(parents=True, exist_ok=True)
    result = {
        "runs": [str(run.path) for run in runs],
        "mode": mode,
        "scene_count": len(names),
        "summaries": summaries,
        "paired": paired,
    }
    experiment.write_json(output / "comparison.json", result)
    with (output / "comparison.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(paired[0]))
        writer.writeheader()
        writer.writerows(paired)
    lines = [
        "# Body 中心首命中筛选对照",
        "",
        f"{len(names)} 个场景，{mode}，逐条核对点云和初值 SHA256、权重、机器人尺寸、{reference.metadata['maxiter']} 次上限及求解条件。",
        f"共 {len(runs)} 组，每组 {reference.metadata['workers']} 个工作进程、BLAS 单线程。每场景一个固定噪声种子；时间为单次运行，无置信区间。",
        "计时包含每次筛选；总规划时间另包含 KD-tree 构建与初始 wrench。点云生成、完整表面验收、文件和绘图不计入。",
        "",
        "| 方法 | 平均保留比例 | 单场景规划中位数 | 总规划加速比 | 无碰撞 | ≥20 mm | 连续 ≥20 mm 证书 | 正式收敛 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for s in summaries:
        lines.append(
            f"| {s['label']} | {s['mean_retained_fraction']:.1%} | {s['median_planning_seconds']:.3f} s | {s['speedup_total']:.2f}× | {s['collision_free']}/{len(names)} | {s['clearance_20mm']}/{len(names)} | {s['continuous_20mm_certificates']}/{len(names)} | {s['formal_convergence']}/{len(names)} |"
        )
    lines += [
        "",
        "| 方法 | 总迭代 | 总目标评估 | 求解时间/评估 | 筛选查询总耗时 | 末投影梯度中位数 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for s in summaries:
        lines.append(
            f"| {s['label']} | {s['total_iterations']} | {s['total_evaluations']} | {s['solver_ms_per_evaluation']:.3f} ms | {s['total_selection_query_seconds']:.3f} s | {s['median_projected_gradient_inf_norm']:.3g} |"
        )
    lines += ["", "## 配对变化", ""]
    for s in summaries[1:]:
        changes = [row for row in paired if row["run"] == s["run"]]
        unchanged = all(
            row["maximum_node_position_change_m"] == 0
            and row["maximum_node_angle_change_deg"] == 0
            and row["clearance_change_mm"] == 0
            for row in changes
        )
        if unchanged:
            lines.append(
                f"- {s['label']}：全部最终节点及密集间隙与完整点云对照完全一致。"
            )
        else:
            worst = min(changes, key=lambda row: row["clearance_change_mm"])
            lines.append(
                f"- {s['label']}：最大间隙下降出现在 `{worst['scene']}`，变化 {worst['clearance_change_mm']:.2f} mm，最终 {worst['clearance_mm']:.2f} mm。"
            )
        lines.append(
            f"  实际筛选耗时 {s['total_selection_query_seconds']:.2f} s，总规划 {s['total_planning_seconds']:.2f} s；不能只用保留点比例估计加速。"
        )
    lines += [
        "",
        "![完整表面间隙与规划时间](comparison.png)",
        "",
        "## 每场景间隙 / 规划总时间",
        "",
        "| 场景 | " + " | ".join(s["label"] for s in summaries) + " |",
        "|---|" + "---:|" * len(summaries),
    ]
    for name in names:
        cells = [
            f"{t[name]['minimum_signed_distance_m'] * 1000:.2f} mm / {t[name]['total_planning_seconds']:.3f} s"
            for t in tables
        ]
        lines.append(f"| {name} | " + " | ".join(cells) + " |")
    lines += [
        "",
        "## 解释与边界",
        "",
        "- 最大长度从 body 中心起算，各组值见配置和方法标签。世界球面方位角/仰角固定分格，每格只留最近原始点；坐标和点权重不变。",
        "- 后续仍按点到 OBB 的有符号距离计算势能、力和表面力臂力矩。本报告只读保存结果，不按几何评分重新求解或挑选轨迹。",
        "- 若最大长度大于机身半对角线＋惩罚激活距离，单独半径截取不会删除任何有非零惩罚的点。浮点差异仍可能让非凸优化的终点不同。",
        "- 每方向首命中可能遮掉仍对长方体重要的点，且删点减少碰撞能量总和。这里按要求没有重新归一化或增强权重。",
        "- 首命中/方向格子切换会使目标不连续。正式收敛、迭代上限、速度与完整几何安全必须分别看；较快停止不一定是较好优化。",
        "- 所有间隙来自未筛选的独立密集表面，每段 50 等分、共 1101 姿态。密集 ≥20 mm 与保守连续证书分别报告。",
        "- 完整点云基线也是此次重新运行；旧记录的机器负载/并发不同，不拿旧计时作为加速分母。",
        "- 原始逐次轨迹、收敛诊断与协议在各子目录。comparison.csv 提供全部配对差值和耗时；comparison.json 含机器可读汇总。",
        "",
    ]
    (output / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")
    fig, axes = plt.subplots(1, 2, figsize=(17, 10))
    y = np.arange(len(names))
    width = 0.8 / len(runs)
    for i, (run, table) in enumerate(zip(runs, tables)):
        offset = (i - (len(runs) - 1) / 2) * width
        for ax, key, scale in (
            (axes[0], "minimum_signed_distance_m", 1000),
            (axes[1], "total_planning_seconds", 1),
        ):
            ax.barh(
                y + offset,
                [table[n][key] * scale for n in names],
                height=width,
                label=run.path.name,
            )
    for ax in axes:
        ax.set_yticks(y, names, fontsize=7)
        ax.invert_yaxis()
        ax.grid(axis="x", alpha=0.2)
        ax.legend(fontsize=8)
    axes[0].axvline(0, color="red", lw=1)
    axes[0].axvline(20, color="black", ls="--", lw=1)
    axes[0].set_xlabel("Held-out dense minimum clearance [mm]")
    axes[1].set_xlabel("Planning including selection and setup [s]")
    fig.tight_layout()
    fig.savefig(output / "comparison.png", dpi=160)
    plt.close(fig)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=Path, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", choices=MODES, default="full")
    args = parser.parse_args(argv)
    result = compare(args.runs, args.output, args.mode)
    print(json.dumps(result["summaries"], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

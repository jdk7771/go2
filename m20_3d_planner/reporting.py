"""Static figures and the experiment report; no optimizer side effects."""

import csv
from pathlib import Path
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from .settings import LABELS, COLORS, KEY_SCENES


def plot_results(
    output: Path, rows: list[dict], scenes: list, half: np.ndarray, height: float
) -> None:
    run_dir = output
    output = run_dir / "report"
    output.mkdir(exist_ok=True)
    names = [scene.name for scene in scenes]
    modes = list(dict.fromkeys(row["mode"] for row in rows))
    indexed = {(row["scene"], row["mode"]): row for row in rows}
    fig, axes = plt.subplots(
        1, 2, figsize=(16, 9), gridspec_kw={"width_ratios": [1.8, 1]}
    )
    y = np.arange(len(names))
    offsets = np.linspace(-0.24, 0.24, len(modes)) if len(modes) > 1 else [0]
    for offset, mode in zip(offsets, modes):
        selected = [indexed[name, mode] for name in names]
        axes[0].barh(
            y + offset,
            [1000 * r["minimum_signed_distance_m"] for r in selected],
            height=0.23,
            label=LABELS[mode],
            color=COLORS[mode],
        )
        axes[1].scatter(
            [r["cpu_seconds"] for r in selected],
            [r["projected_gradient_inf_norm"] for r in selected],
            label=LABELS[mode],
            color=COLORS[mode],
            alpha=0.75,
        )
    axes[0].set_yticks(y, names, fontsize=8)
    axes[0].invert_yaxis()
    axes[0].axvline(0, color="tab:red", linewidth=1)
    axes[0].axvline(20, color="k", linestyle="--", label="20 mm target")
    axes[0].set_xlabel("Minimum clearance on held-out dense surfaces [mm]")
    axes[0].legend()
    axes[1].set_yscale("log")
    axes[1].set_xlabel("Optimization process CPU time [s]")
    axes[1].set_ylabel("Final projected gradient infinity norm")
    axes[1].grid(alpha=0.25)
    axes[1].legend()
    fig.suptitle("Uniform straight initialization | same collision law and weights")
    fig.tight_layout()
    fig.savefig(output / "ablation_summary.png", dpi=160)
    plt.close(fig)

    diagnostic_names = [
        scene.name
        for scene in scenes
        if scene.name in KEY_SCENES or scene.params.get("cross_section")
    ]
    for name in diagnostic_names:
        if name not in names:
            continue
        fig, axes = plt.subplots(2, 2, figsize=(13, 8))
        for mode in modes:
            prefix = run_dir / "trials" / f"{name}__{mode}"
            with np.load(prefix.with_suffix(".npz")) as data:
                trajectory = data["optimized_trajectory"]
            with np.load(prefix.with_suffix(".validation.npz")) as data:
                dense, distances = data["dense_poses"], data["dense_clearance"]
            with prefix.with_suffix(".history.csv").open() as f:
                history = list(csv.DictReader(f))
            axes[0, 0].plot(
                trajectory[:, 0],
                trajectory[:, 2],
                "o-",
                ms=2,
                color=COLORS[mode],
                label=LABELS[mode],
            )
            axes[0, 1].plot(
                trajectory[:, 0],
                np.degrees(trajectory[:, 3]),
                color=COLORS[mode],
                label=LABELS[mode],
            )
            axes[1, 0].plot(
                dense[:, 0], 1000 * distances, color=COLORS[mode], label=LABELS[mode]
            )
            axes[1, 1].semilogy(
                [int(r["iteration"]) for r in history],
                [max(float(r["projected_gradient_inf_norm"]), 1e-12) for r in history],
                color=COLORS[mode],
                label=LABELS[mode],
            )
        axes[0, 0].axhline(height, color="0.4", linestyle="--", label="initial height")
        axes[1, 0].axhline(20, color="k", linestyle="--", label="20 mm target")
        axes[1, 0].axhline(0, color="tab:red", linewidth=1)
        params = next(scene.params for scene in scenes if scene.name == name)
        if "side_inner_y_m" in params:
            for ax in (axes[0, 0], axes[0, 1], axes[1, 0]):
                ax.axvspan(
                    *params["x_extent_m"],
                    color="0.5",
                    alpha=0.10,
                    label="channel x range",
                )
                for x in params["x_extent_m"]:
                    ax.axvline(x, color="0.5", linestyle=":", lw=0.8)
        for ax, title, ylabel in zip(
            axes.ravel(),
            (
                "Body height",
                "Roll discovered from zero",
                "Independent clearance",
                "Convergence history",
            ),
            (
                "z [m]",
                "roll [deg]",
                "clearance [mm]",
                "projected gradient infinity norm",
            ),
        ):
            ax.set_title(title)
            ax.set_ylabel(ylabel)
            ax.set_xlabel("iteration" if ax is axes[1, 1] else "path x [m]")
            ax.grid(alpha=0.25)
            ax.legend(fontsize=8)
        fig.suptitle(name + " | identical zero-angle initialization")
        fig.tight_layout()
        fig.savefig(output / f"{name}.png", dpi=155)
        plt.close(fig)

    plot_cross_sections(run_dir, scenes, half)


def plot_cross_sections(output: Path, scenes: list, half: np.ndarray) -> None:
    """Equal-scale y-z geometry and actual center poses, independent of camera."""
    from matplotlib.patches import Polygon
    from scipy.spatial import ConvexHull
    from .visualization.geometry import obb_corners

    selected = [
        scene
        for scene in scenes
        if scene.params.get("cross_section")
        and (output / "trials" / f"{scene.name}__full.npz").exists()
    ]
    if not selected:
        return
    fig, axes = plt.subplots(
        len(selected), 2, figsize=(12, 4.6 * len(selected)), squeeze=False
    )
    for row, scene in enumerate(selected):
        with np.load(output / "trials" / f"{scene.name}__full.npz") as data:
            initial, final = data["initial_trajectory"], data["optimized_trajectory"]
        for ax, poses, title in zip(
            axes[row],
            (initial, final),
            ("Uniform initial pose", "Optimized full 6D pose"),
        ):
            for segment in scene.params["cross_section"]:
                yz = np.array(segment["yz"])
                ax.plot(*yz.T, color=segment["color"], lw=3)
                ax.plot(*yz.T, ".", color=segment["color"], ms=7)
            pose = poses[np.argmin(np.abs(poses[:, 0]))]
            corners = obb_corners(pose, half)[:, 1:]
            hull = corners[ConvexHull(corners).vertices]
            ax.add_patch(
                Polygon(
                    hull,
                    facecolor="tab:blue"
                    if title.startswith("Optimized")
                    else "tab:orange",
                    alpha=0.5,
                    edgecolor="0.1",
                )
            )
            ax.plot(pose[1], pose[2], "+", color="0.1")
            geometry_title = (
                "Horizontal ceiling / floor: 0° then 30°"
                if "floor_profile_yz_m" in scene.params
                else "Horizontal floor / 30° ceiling"
            )
            ax.set(
                title=f"{geometry_title}\n{title}\nroll={np.degrees(pose[3]):+.1f}°, y={pose[1]:+.3f}, z={pose[2]:.3f} m",
                xlabel="lateral y [m]",
                ylabel="height z [m]",
                xlim=(-0.68, 0.54),
                ylim=(-0.02, 0.8),
                aspect="equal",
            )
            ax.grid(alpha=0.2)
    fig.suptitle(
        "Transverse cross-section | purple: upper surface | brown: lower surface | no angle exaggeration"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(output / "report" / "cross_sections.png", dpi=160)
    plt.close(fig)


def write_report(output: Path, metadata: dict, rows: list[dict]) -> None:
    names = list(dict.fromkeys(r["scene"] for r in rows))
    modes = list(dict.fromkeys(r["mode"] for r in rows))
    lines = [
        "# M20 统一直线初始化与自由度消融",
        "",
        f"本轮包含 {len(names)} 个场景，使用固定随机种子、相同点云流程、47 mm 激活距离、目标函数、权重、",
        f"边界与 L-BFGS-B 停止条件。本轮自由度模式：{', '.join(LABELS[m] for m in modes)}；输入完整点云和初始轨迹：",
        f"`[x_i, 0, {metadata['robot']['nominal_height_m']}, 0, 0, 0]`。不使用场景倾角、梁高、建议位姿或真值生成初值。",
        "所有优化结束后才对留出的密集表面评分，不按评分调参、重试或筛选轨迹。",
        f"点云筛选配置：`{metadata.get('point_selection', {'method': 'all'})}`。",
        "`rays` 从当前 body 中心出发，世界坐标系球面方向分格，每格保留半径内最近原始点；"
        "`radius` 仅截取半径，`all` 沿用完整点云。距离从中心算，后续 OBB 势能、原始点权重和 wrench 不变。",
        "筛选每次目标评估重新计算；首命中切换使目标分段光滑，不能把停止标志当成安全证明。"
        "删点会改变能量总和，不作重加权补偿。验收始终使用未筛选的留出表面。",
        "",
        "## 机器人参数来源",
        "",
        f"- 来源：`{metadata['robot']['kind']}`。",
        f"- 机身尺寸：`{metadata['robot']['body_size_m']}` m；名义高度：{metadata['robot']['nominal_height_m']:.3f} m。",
        "- 本机缺少原始 URDF/config 时，显式复用已提交结果中的机器人常量；不宣称本轮重新核验了原始模型。",
        "",
        "## 汇总",
        "",
        "| 方法 | 严格无碰撞 | 严格 ≥20 mm | 保守连续 ≥20 mm 证书 | 正式收敛 | CPU 时间中位数 | 墙钟时间中位数 |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for mode in modes:
        group = [r for r in rows if r["mode"] == mode]
        lines.append(
            f"| {LABELS[mode]} | {sum(r['collision_free_strict'] for r in group)}/{len(group)} "
            f"| {sum(r['clearance_20mm_strict'] for r in group)}/{len(group)} "
            f"| {sum(r['conservative_continuous_20mm_certificate'] for r in group)}/{len(group)} "
            f"| {sum(r['formal_success'] for r in group)}/{len(group)} "
            f"| {np.median([r['cpu_seconds'] for r in group]):.2f} s "
            f"| {np.median([r['wall_seconds'] for r in group]):.2f} s |"
        )
    lines += [
        "",
        "## 每场景的最小间隙",
        "",
        "| 场景 | "
        + " | ".join(LABELS[m] for m in modes)
        + " | 6D 最大 roll/pitch/yaw |",
        "|---|" + "---:|" * len(modes) + "---|",
    ]
    for name in names:
        group = {r["mode"]: r for r in rows if r["scene"] == name}
        full = group.get("full")
        clearances = " | ".join(
            f"{group[m]['minimum_signed_distance_m'] * 1000:.2f} mm" for m in modes
        )
        angles = (
            f"{full['max_abs_roll_deg']:.1f}/{full['max_abs_pitch_deg']:.1f}/{full['max_abs_yaw_deg']:.1f}°"
            if full
            else "—"
        )
        lines.append(f"| {name} | {clearances} | {angles} |")
    lines += [
        "",
        "## 姿态与高度变化",
        "",
        "| 场景（6D） | 初始→最终最小间隙 | 最低高度 | 中心 roll | 出口区高度恢复最大误差 |",
        "|---|---|---:|---:|---:|",
    ]
    for name in names:
        selected = [r for r in rows if r["scene"] == name and r["mode"] == "full"]
        if selected:
            row = selected[0]
            lines.append(
                f"| {name} | {row['baseline_min_distance_m'] * 1000:.2f} → "
                f"{row['minimum_signed_distance_m'] * 1000:.2f} mm | {row['minimum_base_z_m']:.3f} m "
                f"| {row['roll_at_center_deg']:.2f}° | {row['exit_height_recovery_max_error_m'] * 1000:.2f} mm |"
            )
    cross_sections = {
        name: entry["params"]
        for name, entry in metadata.get("scenes", {}).items()
        if entry["params"].get("cross_section") and "full" in modes
    }
    if cross_sections:
        lines += [
            "",
            "## 非平行截面几何",
            "",
            "![等比例横截面初始与最终姿态](cross_sections.png)",
            "",
        ]
        for name, params in cross_sections.items():
            lines += [f"### {name}", "", params["description"], ""]
            if "floor_profile_yz_m" in params:
                lines += [
                    f"- 下方折线 `[y,z]`（m）：`{params['floor_profile_yz_m']}`。",
                    f"- 顶板下表面 z={params['ceiling_height_m']:.3f} m，沿 x 范围 `{params['x_extent_m']}`。",
                    "- 下方表面是封闭抬升平台，包含入口/出口端面；平台外仍有 z=0 的地面。",
                ]
            else:
                lines += [
                    f"- 地面 z=0；上方下表面为 `z={params['roof_intercept_m']:.3f}+tan(30°)·y`（m）。"
                ]
            if "side_inner_y_m" in params:
                lines += [
                    f"- 通道沿 x 范围 `{params['x_extent_m']}` m，两侧内表面 y=`{params['side_inner_y_m']}` m（净宽 {np.diff(params['side_inner_y_m'])[0]:.3f} m）。",
                    "- 起终点位于通道外的空旷区，均固定为 z=0.4 m、零姿态。所有中间节点仍从水平直线开始，无降高/侧倾提示。",
                    "- 两侧竖直壁限制横移；本场景同时考察入口降高、侧倾及离开通道的过渡，未改变规划器权重或位姿边界。",
                ]
            lines += [
                "",
                "这些合成尺寸是公开的测试夹具参数；只有采样点云交给优化器，截面轮廓只用于显示。",
                "",
            ]
        lines += [
            "场景的 30° 几何倾角不要求机身 roll=30°。上下表面、可用净空和横移共同决定解，",
            "若 XYZ 也能通过，该场景就不能作为必须依靠侧倾的证据。这里仍未验证足端接触或真实地形行走。",
            "",
        ]
        for name in cross_sections:
            full = next(r for r in rows if r["scene"] == name and r["mode"] == "full")
            lines += [
                f"- `{name}`：6D 最大 roll={full['max_abs_roll_deg']:.2f}°，最大横移幅度={full['maximum_abs_y_m']:.3f} m；",
                f"  密集复检间隙 {full['minimum_signed_distance_m'] * 1000:.2f} mm。",
                f"  6D 正式收敛：{full['formal_success']}；保守连续间隙下界 {full['continuous_lower_bound_against_synthetic_surfaces_m'] * 1000:.2f} mm。",
            ]
        lines += [
            "",
            "密集采样通过 20 mm 目标与连续 ≥20 mm 证书分开报告；两者不能互相替代。",
            "",
        ]
        for name, params in cross_sections.items():
            if "side_inner_y_m" not in params:
                continue
            with np.load(output / "trials" / f"{name}__full.npz") as trial:
                poses = trial["optimized_trajectory"]
            xmin, xmax = params["x_extent_m"]
            inside = poses[(poses[:, 0] >= xmin) & (poses[:, 0] <= xmax)]
            pose = inside[np.argmax(np.abs(inside[:, 3]))]
            lines += [
                f"同一通道内节点 x={pose[0]:.3f} m、z={pose[2]:.3f} m、roll={np.degrees(pose[3]):.2f}°，确认降高与侧倾同时存在。",
                "",
                "通道过渡的高度/侧倾及入口出口位置见下图。淡色区表示机身中心 x 位于通道范围；",
                "机身具有长度，前后端会更早进入、更晚离开。起终点高度和姿态为固定边界，出口恢复不能全归因于碰撞势能。",
                "",
                f"![空旷区进入通道的轨迹]({name}.png)",
                "",
            ]
    lines += [
        "",
        "## 收敛诊断",
        "",
        "投影梯度采用单位步长映射 `x - clip(x - grad, lower, upper)`。",
        "记录每次接受迭代的目标值、原始梯度、投影梯度、最低高度与最大姿态角。",
        "最后 50 次迭代相对下降量使用 `Δcost / max(1, |cost_at_tail_start|)`；",
        "少于 50 次时取所有可用迭代。正式收敛、梯度小、几何安全分别报告。",
        "",
        "| 场景 / 方法（未正式收敛） | 迭代 | 投影梯度 ∞ 范数 | 末 50 次相对下降 |",
        "|---|---:|---:|---:|",
    ]
    for r in rows:
        if not r["formal_success"]:
            lines.append(
                f"| {r['scene']} / {LABELS[r['mode']]} | {r['iterations']} "
                f"| {r['projected_gradient_inf_norm']:.3e} | {r['tail_relative_cost_drop']:.3e} |"
            )
    lines += [
        "",
        "## 验收与实验边界",
        "",
        f"- 每个节点区间做 {metadata['validation_subdivisions']} 等分复检，严格按 `d >= 0` / `d >= 0.020` 计数。",
        "- 同时保留原代码的 0.5 mm 碰撞、1.5 mm 安全目标容差字段，主结果不使用这些容差。",
        "- 连续证书先用相邻复检姿态的平移量与 `||half||·Σ|Δrpy|` 给出区间运动上界，",
        "  从端点间隙减去半个运动上界，再减去合成表面的最大网格覆盖半径（15.56 mm）。",
        "  这是对本实验有限合成表面的保守充分条件；无证书不代表必然碰撞。",
        "  它不包含真实感知、定位或跟踪误差，也不认证实机安全。",
        "- 47 mm 是软惩罚激活距离，不是硬安全约束。本轮为隔离初始化/自由度影响，未改为 barrier 或约束优化。",
        f"- 并发场景数：{metadata['workers']}；BLAS 单线程。墙钟时间受并发调度影响，另报每进程 CPU 时间。",
        "- `wall_seconds` 包括求解中的每次筛选；`total_planning_seconds` 还包括索引构建、初始 wrench 和记录准备。"
        "完整点云生成、独立密集验收、写文件及绘图不计入规划时间。",
        "- 每场景固定一个噪声种子；未做多种子成功率或统计显著性检验。",
        "- 14 帧点云来自沿全路线的离线采样，含未来视角，不代表在线探索或单帧感知。",
        "- 姿态范围沿用原规划器的盒约束，尚未由腿轮接触、关节可达性或动力学验证。",
        "- 本轮结果只反映此固定初始化、固定求解预算下的表现，不把局部优化失败解释为方法本身无效。",
        "",
        "## 文件",
        "",
        "- `results.json`、`results.csv`：全部方法的指标、协议与软件版本。",
        "- `trials/*.history.csv`：完整迭代诊断；`*.npz`：初始/最终轨迹和初始碰撞 wrench。",
        "- `scenes/*.npz`：三方法共享的点云、留出表面和统一初值。",
        "- `ablation_summary.png` 与 A/B/C 场景图：自由度、姿态发现和收敛情况。",
        "",
    ]
    (output / "report" / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")

"""Static figures and the experiment report; no optimizer side effects."""

import csv
import json
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
    protocol = json.loads((run_dir / "protocol.json").read_text())
    curved = protocol.get("initialization") == "cloud_scored_smooth_bows_zero_speed"
    initialization_label = ("Cloud-selected curved initialization" if curved else
                            "Unoptimized small-bend initialization" if protocol.get("initialization") == "unoptimized_small_bend_zero_speed"
                            else "Uniform straight initialization")
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
    fig.suptitle(initialization_label + " | same collision law and weights")
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
                "Roll after curved initialization" if curved else "Roll discovered from zero",
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
        fig.suptitle(name + " | " + initialization_label)
        fig.tight_layout()
        fig.savefig(output / f"{name}.png", dpi=155)
        plt.close(fig)

    plot_cross_sections(run_dir, scenes, half)
    plot_tall_detours(run_dir, rows, scenes, half, initialization_label)


def plot_tall_detours(
    output: Path, rows: list[dict], scenes: list, half: np.ndarray,
    initialization_label: str = "Uniform straight initialization",
) -> None:
    """Show the actual route, body footprints, height and accepted costs."""
    from matplotlib.patches import Polygon, Rectangle
    from scipy.spatial import ConvexHull
    from .visualization.geometry import obb_corners

    for scene in scenes:
        if scene.family != "tall_obstacle":
            continue
        fig, axes = plt.subplots(2, 2, figsize=(13, 8))
        center = np.array(scene.params["obstacle_center_m"])
        size = np.array(scene.params["obstacle_size_m"])
        axes[0, 0].add_patch(
            Rectangle(
                center[:2] - size[:2] / 2, *size[:2], color="0.4", alpha=0.3,
                label=f"{size[2]:.2f} m tall block",
            )
        )
        modes = [row["mode"] for row in rows if row["scene"] == scene.name]
        for mode in modes:
            prefix = output / "trials" / f"{scene.name}__{mode}"
            with np.load(prefix.with_suffix(".npz")) as data:
                poses = data["optimized_trajectory"]
                initial = data["initial_trajectory"]
                costs = data["iteration_costs"]
            with np.load(prefix.with_suffix(".validation.npz")) as data:
                dense, clearance = data["dense_poses"], data["dense_clearance"]
            axes[0, 0].plot(*poses[:, :2].T, "o-", ms=2, color=COLORS[mode], label=LABELS[mode])
            if mode == "full":
                for pose in poses[3:-3:3]:
                    corners = obb_corners(pose, half)[:, :2]
                    axes[0, 0].add_patch(
                        Polygon(
                            corners[ConvexHull(corners).vertices],
                            color=COLORS[mode], alpha=0.12,
                        )
                    )
            axes[0, 1].plot(poses[:, 0], poses[:, 2], color=COLORS[mode], label=LABELS[mode])
            axes[1, 0].plot(dense[:, 0], clearance * 1000, color=COLORS[mode], label=LABELS[mode])
            axes[1, 1].semilogy(np.arange(len(costs)), costs, color=COLORS[mode], label=LABELS[mode])
        axes[0, 0].plot(*initial[:, :2].T, "--", color="tab:orange", label="initialization")
        axes[0, 0].set(aspect="equal", ylabel="lateral y [m]", title="Top view / full 6D body footprints")
        axes[0, 1].axhline(initial[0, 2], color="0.5", linestyle="--")
        axes[0, 1].set(ylabel="body center z [m]", title="Body height (block top: 1.40 m)")
        axes[1, 0].axhline(20, color="k", linestyle="--")
        axes[1, 0].axhline(0, color="tab:red")
        axes[1, 0].set(ylabel="clearance [mm]", title="Held-out dense surface clearance")
        axes[1, 1].set(ylabel="objective", title="Actual accepted optimizer iterations")
        for ax in axes.ravel():
            ax.set_xlabel("iteration" if ax is axes[1, 1] else "path x [m]")
            ax.legend(fontsize=8)
            ax.grid(alpha=0.25)
        fig.suptitle("Tall obstacle detour | " + initialization_label)
        fig.tight_layout()
        fig.savefig(output / "report" / f"{scene.name}.png", dpi=155)
        plt.close(fig)


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
    curved = metadata.get("initialization") == "cloud_scored_smooth_bows_zero_speed"
    small_bend = metadata.get("initialization") == "unoptimized_small_bend_zero_speed"
    lines = [
        ("# M20 点云弯曲初始化与虚拟动力学" if curved else
         "# M20 穿障初值与开源多场景动力学测试" if small_bend else "# M20 统一直线初始化与自由度消融"),
        "",
        f"本轮包含 {len(names)} 个场景，使用固定随机种子、相同点云流程、{metadata['activation_distance_m'] * 1000:g} mm 激活距离、目标函数、权重、",
        f"边界。本轮求解器 `{metadata.get('solver', 'lbfgsb')}`；自由度模式：{', '.join(LABELS[m] for m in modes)}；输入点云和初始轨迹：",
        ("由原始点云对双向弯曲、升降、双向侧倾候选评分选取；端点固定、初速度为零。同场景各自由度共享选中初值。"
         "该曲线同时作为 XY/姿态恢复参考；高度恢复仍指向名义高度。不使用场景元数据或真值生成初值。"
         if curved else "未经避障的轻微 S 弯（横向幅度上限 25 mm）、名义高度、零姿态、零初速。初始路径穿障情况按原始几何复检单独报告。"
         if small_bend else f"`[x_i, 0, {metadata['robot']['nominal_height_m']}, 0, 0, 0]`。不使用场景倾角、梁高、建议位姿或真值生成初值。"),
        ("本轮属于已知场景参数调试，参数选择参考了前序试验及碰撞诊断，结果不能作为独立泛化评估。规划器仍只读取点云；本轮全部指定场景在求解后统一复检并保留。"
         if metadata.get("tuning_note") else "所有优化结束后才对留出的密集表面评分，不按评分调参、重试或筛选轨迹。"),
        f"点云筛选配置：`{metadata.get('point_selection', {'method': 'all'})}`。",
        f"目标权重：`{metadata['weights']}`。",
        f"调参说明：{metadata.get('tuning_note') or '未提供额外调参说明'}。",
        f"机身中心横移边界：±{metadata.get('lateral_limit_m', 0.16):.2f} m；其余位姿边界沿用原规划器。",
        "`rays` 从当前 body 中心出发，世界坐标系球面方向分格，每格保留半径内最近原始点；"
        "`radius` 仅截取半径，`all` 沿用完整点云。距离从中心算，后续 OBB 势能、原始点权重和 wrench 不变。",
        "每个接受状态重新筛选；新射线动力学的稳定性重试固定同轮命中集合。首命中切换使目标分段光滑，不能把停止标志当成安全证明。"
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
    if metadata.get("solver") == "ray_dynamics":
        lines += [
            "", "## 射线牛顿—欧拉版本", "",
            "节点直接保存 T∈SE(3)、世界系线速度和机身系角速度。KD-tree 半径查询后按全空间角度格首命中筛选点云，属于有限角宽的射线近似；碰撞作用仍使用点到机身 OBB 最近表面点、法向及表面力臂。",
            "加入路径/高度恢复、SO(3) 对数姿态恢复、相邻旋转增量及其差分弹性。旋转弹性由 SO(3) 对数雅可比映射为力矩，不沿用欧拉角弹簧。",
            "F_total=F_elastic-Dv*v；tau_total_body=R.T*tau_elastic-Dw*omega。a=F_total/m；alpha=I^-1*(tau_total_body-omega×I*omega)。先更新速度，再更新位置及 R_next=R*Exp(dt*omega_next)。未用 L-BFGS-B 或 cost_grad 决定运动。",
            f"配置：`{metadata['dynamics_config']}`。时间步上限按几何序列从大到小调度；固定同轮命中集合检查积分机械能，必要时减半重试。接受步后按 {metadata['dynamics_config'].get('time_step_growth', 1.):g} 倍恢复实际步长（1 表示不恢复），始终不超过当前调度上限，且重新检查单步运动限制。重新筛选后势能可以跳变，不宣称射线切换时全局能量单调。",
            "首尾 T 固定且速度为零；位姿边界通过投影与约束冲量处理。合力包含阻尼；保存的加速度为世界系 a、机身系 alpha，不包含边界约束冲量。",
            "约束作用量残差与速度连续多步达到容差才标记收敛。求解后（包括未收敛情况）使用完整留出表面对线性平移、SO(3) 测地插值路径做密集检测及保守连续下界验证。收敛与安全分别报告。",
            "`trials/*.npz` 保存每步 transforms、twists、accelerations、net/elastic wrenches、body torques、实际与调度步长；history.csv 保存加速度模长和力平衡残差。虚拟时间不是实机时间。", "",
        ]
        limits = metadata["dynamics_config"]
        if limits.get("max_translation_step") is not None or limits.get("max_rotation_step") is not None:
            lines += [
                "### 单步运动限制", "",
                "在位姿边界裁剪前检查 dt*||v_next|| 与 dt*||omega_next||；转角使用未折返角度，避免超过整圈后旋转矩阵接近原姿态而漏检。超限保持原状态并缩小 dt 重算。裁剪后再次检查实际位置差与 SO(3) 相对转角，随后才做机械能检查。没有直接截断位移或修改加速度来满足限制。",
                "", "| 场景 | 实际最大单步平移 [mm] | 实际最大单步转角 [deg] | 运动超限重试 | 能量重试 |",
                "|---|---:|---:|---:|---:|",
            ]
            for row in rows:
                lines.append(f"| {row['scene']} | {1000 * row['maximum_node_translation_step']:.4f} | {np.degrees(row['maximum_node_rotation_step']):.4f} | {row['motion_limited_retries']} | {row['energy_rejected_steps']} |")
            lines += ["", "单步运动限制属于数值积分约束，不表示机器人实机速度或动力学可行性。", ""]
    if metadata.get("solver") == "virtual_wrench":
        config = metadata["dynamics_config"]
        lines += [
            "", "## 虚拟刚体动力学求解", "",
            "每个中间轨迹节点都是一个与机器人 body OBB 同形的虚拟刚体。直接计算点云合力、表面力臂力矩及路径恢复/耦合弹簧，积分线速度和机身坐标系角速度，再更新位置和旋转矩阵。未调用 L-BFGS-B 或 cost_grad 更新轨迹。",
            "碰撞势能、平移弹簧及欧拉角弹簧保留基线目标；角度弹簧通过虚功映射为世界系力矩，姿态按 SO(3) 指数映射积分。",
            f"虚拟参数：`{config}`。惯量按均匀箱体、虚拟质量计算，不宣称为机器人真实惯量或真实外力。",
            "求解采用带阻尼的半隐式积分，含机身坐标系陀螺项。端点固定；位姿边界通过投影及不增加动能的约束冲量处理。",
            "若总机械能（势能+动能）增加则减半数值步长重试；势能本身可以因惯性暂时增加，没有用势能线搜索决定运动方向。",
            "只在约束力平衡残差和线/角速度同时足够小时宣布收敛；达到步数上限或步长下限仍标记未收敛。虚拟松弛时间不是机器人执行时间。",
            "", "| 方法 | 积分步 | 虚拟时间 | 最终动能 | 拒绝步数 | 停止原因 |",
            "|---|---:|---:|---:|---:|---|",
        ]
        for row in rows:
            lines.append(
                f"| {LABELS[row['mode']]} | {row['iterations']} | {row['relaxation_time']:.3f} "
                f"| {row['final_kinetic_energy']:.3e} | {row['rejected_steps']} | {row['message']} |"
            )
        lines += ["", "`trials/*.npz` 额外保存每步合力/力矩、世界系线速度、机身系角速度、虚拟时间和动能。网页可在优化过程中逐节点核对。", ""]
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
    tall_scenes = {
        name: entry["params"] for name, entry in metadata.get("scenes", {}).items()
        if entry["family"] == "tall_obstacle"
    }
    for name, params in tall_scenes.items():
        lines += [
            "", "## 高障碍物绕行", "", params["description"], "",
            f"- 障碍物中心 `{params['obstacle_center_m']}` m、尺寸 `{params['obstacle_size_m']}` m，底部落地。",
            "- 原横移边界 ±0.16 m 不足以通过此立柱旁；新实验显式使用 --lateral-limit 0.60。较窄边界的对照仍按其实际协议执行。",
            "- 初值、目标函数、权重、点云种子和高度/姿态边界未提供绕行方向提示。局部优化不保证选择最短的一侧。",
            "", f"![俯视绕行、机身高度、独立间隙与目标值]({name}.png)", "",
            "| 方法 | y 范围 (m) | z 范围 (m) | 目标值 初始→最终 | 密集间隙 | 连续下界 |",
            "|---|---|---|---|---|---|",
        ]
        for row in [r for r in rows if r["scene"] == name]:
            with np.load(output / "trials" / f"{name}__{row['mode']}.npz") as trial:
                poses = trial["optimized_trajectory"]
            lines.append(
                f"| {LABELS[row['mode']]} | {poses[:, 1].min():.3f} ～ {poses[:, 1].max():.3f} "
                f"| {poses[:, 2].min():.3f} ～ {poses[:, 2].max():.3f} "
                f"| {row['initial_cost']:.2f} → {row['final_cost']:.2f} "
                f"| {row['minimum_signed_distance_m'] * 1000:.2f} mm "
                f"| {row['continuous_lower_bound_against_synthetic_surfaces_m'] * 1000:.2f} mm |"
            )
        lines += ["", "密集复检、正式收敛和连续证书分别报告。机身外形参考只用于显示；本实验没有腿部、接触或动力学模型。", ""]
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
                    ("- 起终点固定为 z=0.4 m、零姿态；中间节点从点云选择的弯曲/升降/侧倾候选开始。"
                     if curved else "- 起终点固定；中间节点由名义高度、零姿态的轻微 S 弯开始，不预设降高/侧倾。"
                     if small_bend else "- 起终点位于通道外的空旷区，均固定为 z=0.4 m、零姿态。所有中间节点仍从水平直线开始，无降高/侧倾提示。"),
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

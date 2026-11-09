#!/usr/bin/env python3
"""Make body attitude visible in verified, previously recorded experiments.

The scene, body dimensions, poses, RPY angles and validation clearances are
unchanged. A front projection and isolated RPY indicators complement the 3D
view. The indicators show individual Euler components, not another simulated
body. Display time is independent of solver time and robot execution time.
"""

from __future__ import annotations

from .. import experiment as ablation  # Configure BLAS before NumPy.

import argparse
import json
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.patches import Arc, Polygon
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

from .. import core
from . import geometry
from .data import verify_replay, load_history, SavedRun
from ..settings import DEFAULT_RUN


BACKGROUND = "#f4f7fb"
INK = "#16283c"
MUTED = "#64748b"
BODY_BLUE = "#378dcc"
GOLD = "#ffca43"
NOSE = "#e8584f"
ANGLE_COLORS = ("#168b69", "#be7610", "#7252c7")
FACE_COLORS = ("#25659c", NOSE, "#85bee3", "#3a8dc5", "#204d77", GOLD)
SCENES = {
    "tilted_corridor_30deg": ("30° 倾斜通道", "看侧倾：roll 从 0° 增至约 30°", 3),
    "asymmetric_overhang_slalom": (
        "非对称障碍",
        "看侧倾与转向：roll、yaw 随路径共同变化",
        3,
    ),
    "beam_h430_yaw00_tilt+0": (
        "430 mm 低梁",
        "看降高与俯仰：入口低头，通过后恢复高度",
        4,
    ),
    "flat_floor_sloped_ceiling_30deg": (
        "水平地面 · 上方倾斜 30°",
        "上方倾斜不预设机身角度：观察侧倾、横移与降高",
        3,
    ),
    "flat_ceiling_bent_floor_30deg": (
        "水平顶板 · 下方水平接 30° 斜面",
        "棕色为下方 150° 折线；观察零姿态出发的优化",
        3,
    ),
    "open_entry_flat_floor_channel_30deg": (
        "空旷区进入 · 水平地面＋30° 倾斜顶板通道",
        "零姿态出发：观察入口前降高、通道内侧倾与出口恢复",
        3,
    ),
}


def configure_style():
    font_path = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if font_path.exists():
        font_manager.fontManager.addfont(str(font_path))
        plt.rcParams["font.family"] = [
            font_manager.FontProperties(fname=font_path).get_name(),
            "DejaVu Sans",
        ]
    plt.rcParams.update(
        {
            "font.size": 11,
            "text.color": INK,
            "axes.labelcolor": MUTED,
            "axes.titlecolor": INK,
            "xtick.color": MUTED,
            "ytick.color": MUTED,
            "axes.unicode_minus": False,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )


def transform_local(local, pose):
    return np.asarray(local) @ core.rpy_matrix(pose[3:]).T + pose[:3]


def top_mark(half):
    """A painted forward chevron, entirely on the body top surface."""
    hx, hy, hz = half
    return np.array(
        [
            [-0.05, -hy * 0.53, hz + 0.001],
            [hx * 0.65, 0, hz + 0.001],
            [-0.05, hy * 0.53, hz + 0.001],
        ]
    )


def indicator(ax, index, half):
    """Show one Euler component, preserving that body's dimensional ratio."""
    plane_sizes = ((half[1], half[2]), (half[0], half[2]), (half[0], half[1]))
    u, v = plane_sizes[index]
    scale = 0.52 / u
    local = np.array([[-u, -v], [u, -v], [u, v], [-u, v]]) * scale
    reference = Polygon(
        local,
        closed=True,
        fill=False,
        linestyle=(0, (3, 3)),
        edgecolor="#a6b2c1",
        linewidth=1.3,
    )
    body = Polygon(
        local, closed=True, facecolor=BODY_BLUE, edgecolor=INK, linewidth=1.4
    )
    ax.add_patch(reference)
    ax.add_patch(body)
    (top,) = ax.plot(local[[3, 2], 0], local[[3, 2], 1], color=GOLD, linewidth=4)
    (direction,) = ax.plot([0, 0.60], [0, 0], color=ANGLE_COLORS[index], linewidth=2.2)
    ax.plot([-0.66, 0.66], [0, 0], "--", color="#bac4d0", lw=0.8, zorder=0)
    arc = Arc(
        (0, 0), 1.31, 1.31, theta1=0, theta2=0, color=ANGLE_COLORS[index], linewidth=2
    )
    ax.add_patch(arc)
    names = ("侧倾 roll", "俯仰 pitch", "转向 yaw")
    ax.set_title(
        names[index], fontsize=12, color=ANGLE_COLORS[index], pad=5, weight="bold"
    )
    value = ax.text(
        0.5,
        0.05,
        "",
        transform=ax.transAxes,
        ha="center",
        va="top",
        fontsize=20,
        weight="bold",
        color=ANGLE_COLORS[index],
    )
    ax.set(xlim=(-0.78, 0.78), ylim=(-0.78, 0.78), aspect="equal")
    ax.axis("off")

    def update(angle):
        # Under Rz(yaw) Ry(pitch) Rx(roll), positive pitch lowers the +x nose.
        angle_2d = -angle if index == 1 else angle
        c, s = np.cos(angle_2d), np.sin(angle_2d)
        rotation = np.array([[c, -s], [s, c]])
        corners = local @ rotation.T
        body.set_xy(corners)
        top.set_data(corners[[3, 2], 0], corners[[3, 2], 1])
        direction.set_data([0, 0.60 * c], [0, 0.60 * s])
        degrees = np.degrees(angle_2d)
        arc.theta1, arc.theta2 = min(0, degrees), max(0, degrees)
        value.set_text(f"{np.degrees(angle):+.1f}°")

    return update


def build_scene(experiment: Path, scene: str, fps: int):
    with np.load(experiment / "scenes" / f"{scene}.npz") as data:
        cloud = data["planning_point_cloud"]
        initial = data["initial_trajectory"]
        half = data["body_half_extents"]
    histories, _ = load_history(experiment, scene, "full")
    with np.load(experiment / "trials" / f"{scene}__full.npz") as data:
        np.testing.assert_array_equal(histories[-1], data["optimized_trajectory"])
    with np.load(experiment / "trials" / f"{scene}__full.validation.npz") as data:
        dense, clearances = data["dense_poses"], data["dense_clearance"]
    title, hint, highlight_dimension = SCENES.get(
        scene, (scene, "观察机身位置与姿态的变化", 3)
    )
    center_node = len(initial) // 2
    peak_index = int(np.argmax(np.abs(dense[:, highlight_dimension])))
    worst_index = int(np.argmin(clearances))
    durations = (1, 7, 1, 3, 12, 2)
    boundaries = np.cumsum([0] + [s * fps for s in durations])
    optimization_iterations = np.rint(
        np.expm1(np.linspace(0, np.log1p(len(histories) - 1), durations[1] * fps))
    ).astype(int)
    chosen = np.unique(
        np.r_[
            np.arange(0, len(cloud), max(1, len(cloud) // 5500)),
            np.flatnonzero(cloud[:, 2] > 0.035)[::3],
        ]
    )
    shown_cloud = cloud[chosen]

    fig = plt.figure(figsize=(16, 9), facecolor=BACKGROUND)
    fig.text(
        0.035, 0.958, f"点云中的机身姿态优化  |  {title}", fontsize=22, weight="bold"
    )
    fig.text(0.036, 0.918, hint, fontsize=13, color=MUTED)
    phase_label = fig.text(0.036, 0.872, "", fontsize=13, weight="bold")
    clearance_label = fig.text(0.965, 0.924, "", ha="right", fontsize=12, weight="bold")
    fig.text(
        0.965,
        0.889,
        f"整条轨迹最小间隙 {clearances.min() * 1000:+.2f} mm",
        ha="right",
        fontsize=11,
        color=geometry.clearance_color(float(clearances.min())),
    )

    ax = fig.add_axes(
        [0.012, 0.282, 0.59, 0.55], projection="3d", computed_zorder=False
    )
    ax.set_facecolor(BACKGROUND)
    obstacle = shown_cloud[:, 2] > 0.035
    colors = np.zeros((len(shown_cloud), 4))
    colors[obstacle] = [0.24, 0.43, 0.55, 0.22]
    colors[~obstacle] = [0.55, 0.65, 0.73, 0.07]
    ax.scatter(
        *shown_cloud.T,
        c=colors,
        s=np.where(obstacle, 3, 1),
        depthshade=False,
        zorder=1,
        axlim_clip=True,
    )
    ax.plot(*initial[:, :3].T, "--", color="#8493a6", lw=1.5, zorder=2, axlim_clip=True)
    (path,) = ax.plot(
        *initial[:, :3].T, color="#2878b1", lw=2.4, zorder=3, axlim_clip=True
    )
    corners = geometry.obb_corners(initial[center_node], half)
    body = Poly3DCollection(
        [corners[list(f)] for f in geometry.FACES],
        facecolors=FACE_COLORS,
        edgecolors=INK,
        linewidths=1.25,
        alpha=1,
        zorder=8,
    )
    ax.add_collection3d(body)
    reference = Line3DCollection(
        geometry.segments(initial[[center_node]], half),
        colors="#8e9dad",
        linestyles="dashed",
        linewidths=1.15,
        zorder=4,
    )
    ax.add_collection3d(reference)
    (chevron,) = ax.plot([], [], [], color="white", lw=4, zorder=10)
    body_axes = [
        ax.plot([], [], [], color=c, lw=2.4, zorder=11)[0]
        for c in (NOSE, "#168b69", "#7460ce")
    ]
    axis_names = [
        ax.text(0, 0, 0, t, color=c, fontsize=10, weight="bold", zorder=12)
        for t, c in zip(("x", "y", "z"), (NOSE, "#168b69", "#7460ce"))
    ]
    ax.set(xlim=(-0.85, 0.85), ylim=(-0.58, 0.58), zlim=(0, 0.79))
    ax.set_proj_type("ortho")
    ax.set_box_aspect((1.7, 1.16, 0.79), zoom=1.1)
    ax.view_init(elev=23, azim=-28)
    ax.set_xlabel("x / m", labelpad=7)
    ax.set_ylabel("y / m", labelpad=6)
    ax.set_zlabel("z / m", labelpad=5)
    ax.set_yticks([-0.4, 0, 0.4])
    ax.set_zticks([0, 0.4, 0.8])
    ax.xaxis.set_major_locator(plt.MaxNLocator(4))
    ax.tick_params(labelsize=9, pad=0)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.set_alpha(0)
    position_label = ax.text2D(
        0.08, 0.96, "", transform=ax.transAxes, fontsize=12, weight="bold"
    )
    fig.text(
        0.034, 0.327, "金色 = 顶面    红色 = 机头    白色箭头 = 前进方向", fontsize=11
    )
    fig.text(
        0.034,
        0.299,
        "虚线盒 = 零姿态参考；左图跟随机身位置，相机方向固定",
        fontsize=10,
        color=MUTED,
    )

    front = fig.add_axes([0.655, 0.525, 0.31, 0.31], facecolor="white")
    front.set_title(
        "正视投影 y–z：直接观察机身侧倾", fontsize=13, weight="bold", pad=10
    )
    front.set(xlim=(-0.44, 0.44), ylim=(0, 0.8), aspect="equal")
    front.set_xlabel("y / m", labelpad=0)
    front.set_ylabel("z / m", labelpad=0)
    front.set_xticks([-0.4, 0, 0.4])
    front.set_yticks([0, 0.4, 0.8])
    front.tick_params(labelsize=9, pad=2)
    front.grid(alpha=0.12)
    front.axhline(0.4, ls="--", color="#c2cbd5", lw=1)
    front.text(-0.425, 0.408, "初始高度 0.40 m", fontsize=8, color=MUTED)
    front_cloud = front.scatter([], [], color="#678696", s=5, alpha=0.4, zorder=1)
    params = SavedRun(experiment).scene(scene)["params"]
    for segment in params.get("cross_section", []):
        yz = np.array(segment["yz"])
        front.plot(*yz.T, color=segment["color"], lw=2, zorder=2)
        ax.plot(
            np.zeros(len(yz)),
            yz[:, 0],
            yz[:, 1],
            color=segment["color"],
            lw=2,
            zorder=2,
        )
    projected_faces = PolyCollection(
        [corners[list(f), 1:] for f in geometry.FACES],
        facecolors=FACE_COLORS,
        edgecolors=INK,
        linewidths=1,
        zorder=3,
    )
    front.add_collection(projected_faces)
    projected_reference = LineCollection(
        [s[:, 1:] for s in geometry.segments(initial[[center_node]], half)],
        colors="#8795a6",
        linestyles="dashed",
        linewidths=1.2,
        zorder=2,
    )
    front.add_collection(projected_reference)
    (top_edge,) = front.plot([], [], color=GOLD, lw=3, zorder=4)
    (front_y_axis,) = front.plot([], [], color=ANGLE_COLORS[0], lw=2.2, zorder=5)
    front_note = front.text(
        0.02,
        0.03,
        "",
        transform=front.transAxes,
        fontsize=8,
        color=MUTED,
        bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.85, "pad": 2},
    )

    gauges = [
        indicator(fig.add_axes([0.64 + i * 0.116, 0.313, 0.104, 0.145]), i, half)
        for i in range(3)
    ]
    fig.text(
        0.812,
        0.276,
        "三个单轴角度示意 · 0° 虚线参考 · 角度不放大",
        ha="center",
        color=MUTED,
        fontsize=9,
    )

    angles_ax = fig.add_axes([0.066, 0.094, 0.501, 0.145], facecolor="white")
    angles_ax.set_title(
        "当前轨迹的姿态角", loc="left", fontsize=12, weight="bold", pad=7
    )
    angle_lines, angle_dots = [], []
    for i, name in enumerate(("roll", "pitch", "yaw")):
        (line,) = angles_ax.plot([], [], color=ANGLE_COLORS[i], lw=2, label=name)
        (dot,) = angles_ax.plot([], [], "o", color=ANGLE_COLORS[i], ms=6)
        angle_lines.append(line)
        angle_dots.append(dot)
    angle_cursor = angles_ax.axvline(0, color="#536779", lw=1.1, ls="--")
    angle_limit = max(
        15, 5 * np.ceil(np.abs(np.degrees(histories[:, :, 3:])).max() / 5) + 5
    )
    angles_ax.set(
        xlim=(-1.4, 1.4),
        ylim=(-angle_limit, angle_limit),
        ylabel="角度 / °",
        xlabel="路径位置 x / m",
    )
    angles_ax.grid(alpha=0.16)
    angles_ax.legend(loc="upper right", ncol=3, frameon=False, fontsize=9)
    angles_ax.tick_params(labelsize=9)

    height_ax = fig.add_axes([0.677, 0.094, 0.281, 0.145], facecolor="white")
    height_ax.set_title(
        "当前轨迹的机身高度", loc="left", fontsize=12, weight="bold", pad=7
    )
    (height_line,) = height_ax.plot([], [], color="#2878b1", lw=2)
    (height_dot,) = height_ax.plot([], [], "o", color="#2878b1", ms=6)
    height_cursor = height_ax.axvline(0, color="#536779", lw=1.1, ls="--")
    height_ax.axhline(0.4, color="#9caaba", lw=1, ls="--")
    height_ax.set(
        xlim=(-1.4, 1.4),
        ylim=(0.26, 0.55),
        ylabel="高度 z / m",
        xlabel="路径位置 x / m",
    )
    height_ax.grid(alpha=0.16)
    height_ax.tick_params(labelsize=9)
    fig.text(
        0.5,
        0.019,
        "原实验点云与轨迹 · 完整 6D · 跟随位置，固定相机方向 · 尺寸与姿态未放大 · 几何回放，播放速度不代表求解或执行速度",
        fontsize=9,
        color=MUTED,
        ha="center",
    )

    def update(frame):
        phase = int(np.searchsorted(boundaries[1:], frame, side="right"))
        local_frame = frame - boundaries[phase]
        iteration = 0 if phase == 0 else len(histories) - 1
        location = None
        if phase == 1:
            iteration = int(optimization_iterations[local_frame])
        poses = histories[iteration]
        if phase < 3:
            pose = poses[center_node]
            phase_text = (
                "初始轨迹：水平机身，所有姿态角为 0°"
                if phase == 0
                else f"优化迭代 {iteration:3d} / {len(histories) - 1}  ·  固定观察第 {center_node + 1} 个节点"
            )
        else:
            if phase == 3:
                location = peak_index
                component_name = "俯仰" if highlight_dimension == 4 else "侧倾"
                phase_text = f"关键姿态停留：最终轨迹中最大{component_name}的位置"
            elif phase == 4:
                fraction = local_frame / max(1, durations[4] * fps - 1)
                location = int(round(fraction * (len(dense) - 1)))
                phase_text = "沿最终轨迹慢速回放：观察姿态的形成与恢复"
            else:
                location = worst_index
                phase_text = "最小间隙位置检查：保留原实验的安全与碰撞结果"
            pose = dense[location]
        phase_label.set_text(phase_text)
        if location is None:
            distance = core.obb_signed_distance(
                cloud, pose[:3], core.rpy_matrix(pose[3:]), half
            )[0].min()
            source = "点云"
        else:
            distance = float(clearances[location])
            source = "真值"
        state = (
            "碰撞"
            if distance < 0
            else ("间隙不足 20 mm" if distance < 0.020 else "间隙 ≥20 mm")
        )
        clearance_label.set_text(
            f"当前位姿（{source}）：{distance * 1000:+.1f} mm  |  {state}"
        )
        clearance_label.set_color(geometry.clearance_color(float(distance)))
        ax.set_xlim(pose[0] - 0.85, pose[0] + 0.85)
        path.set_data_3d(*poses[:, :3].T)
        corners = geometry.obb_corners(pose, half)
        body.set_verts([corners[list(f)] for f in geometry.FACES])
        zero_pose = pose.copy()
        zero_pose[3:] = 0
        reference.set_segments(geometry.segments(zero_pose[None], half))
        chevron.set_data_3d(*transform_local(top_mark(half), pose).T)
        for i, (line, name) in enumerate(zip(body_axes, axis_names)):
            tip_local = np.eye(3)[i] * (half[i] + 0.085)
            tips = transform_local(np.array([np.zeros(3), tip_local]), pose)
            line.set_data_3d(*tips.T)
            name.set_position_3d(tips[-1])
        position_label.set_text(
            f"x {pose[0]:+.2f} m    y {pose[1]:+.2f} m    z {pose[2]:.3f} m"
        )
        nearby = cloud[np.abs(cloud[:, 0] - pose[0]) < 0.10]
        front_cloud.set_offsets(nearby[:, 1:])
        # View from +x: sort by world x so the nearest faces are drawn last.
        ordered = np.argsort([corners[list(face), 0].mean() for face in geometry.FACES])
        projected_faces.set_verts(
            [corners[list(geometry.FACES[i]), 1:] for i in ordered]
        )
        projected_faces.set_facecolors([FACE_COLORS[i] for i in ordered])
        projected_reference.set_segments(
            [s[:, 1:] for s in geometry.segments(zero_pose[None], half)]
        )
        top_corners = corners[[1, 3, 7, 5, 1]]
        top_edge.set_data(top_corners[:, 1], top_corners[:, 2])
        y_tips = transform_local(
            [[0, -half[1] * 1.30, 0], [0, half[1] * 1.30, 0]], pose
        )
        front_y_axis.set_data(y_tips[:, 1], y_tips[:, 2])
        front_note.set_text(f"当前 x={pose[0]:+.2f} m 附近点云；机身为完整投影")
        for i in range(3):
            gauges[i](pose[3 + i])
            angle_lines[i].set_data(poses[:, 0], np.degrees(poses[:, 3 + i]))
            angle_dots[i].set_data([pose[0]], [np.degrees(pose[3 + i])])
        angle_cursor.set_xdata([pose[0], pose[0]])
        height_line.set_data(poses[:, 0], poses[:, 2])
        height_dot.set_data([pose[0]], [pose[2]])
        height_cursor.set_xdata([pose[0], pose[0]])
        return pose.copy()

    preview_frames = {
        "initial": 0,
        "optimized": int(boundaries[2]),
        "peak_attitude": int(boundaries[3]),
        "worst_clearance": int(boundaries[5]),
    }
    metadata = {
        "scene": scene,
        "method": "full",
        "fps": fps,
        "frames": int(boundaries[-1]),
        "duration_seconds": int(sum(durations)),
        "angle_scale": 1.0,
        "geometry_scale": 1.0,
        "rotation_convention": "Rz(yaw) Ry(pitch) Rx(roll)",
        "source": str(experiment.resolve()),
        "initial_sha256": ablation.array_digest(initial),
        "final_sha256": ablation.array_digest(histories[-1]),
        "display_phases_seconds": dict(
            zip(
                (
                    "initial",
                    "optimizer_snapshots",
                    "optimized_hold",
                    "peak_attitude",
                    "trajectory_playback",
                    "worst_clearance",
                ),
                durations,
            )
        ),
        "peak_pose": dense[peak_index].tolist(),
        "peak_validation_index": peak_index,
        "worst_clearance_m": float(clearances.min()),
        "maximum_absolute_rpy_deg": np.abs(np.degrees(dense[:, 3:]))
        .max(axis=0)
        .tolist(),
    }
    return fig, update, preview_frames, metadata


def render_scene(args, scene):
    verified = verify_replay(args.experiment, scene, "full")
    fig, update, previews, metadata = build_scene(args.experiment, scene, args.fps)
    metadata["verification"] = verified
    if args.preview_only:
        update(previews["optimized"])
        preview = args.experiment / "report" / f"{scene}_pose.png"
        preview.parent.mkdir(exist_ok=True)
        fig.savefig(preview, dpi=120, facecolor=BACKGROUND)
    else:
        manifest_path = args.output / "manifest.json"
        manifest = (
            json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
        )
        path = args.output / manifest.get(scene, {}).get("file", f"{scene}.mp4")
        writer = animation.FFMpegWriter(
            fps=args.fps,
            codec="libx264",
            bitrate=-1,
            extra_args=[
                "-crf",
                "19",
                "-preset",
                "fast",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
            ],
        )
        with writer.saving(fig, str(path), dpi=120):
            for frame in range(metadata["frames"]):
                update(frame)
                writer.grab_frame(facecolor=BACKGROUND)
                if frame % (args.fps * 4) == 0:
                    print(f"{scene}: {frame}/{metadata['frames']} frames", flush=True)
        metadata["file"] = path.name
        manifest[scene] = metadata
        ablation.write_json(manifest_path, manifest)
    plt.close(fig)
    print(f"Completed {scene}", flush=True)


def make_roll_comparison(experiment: Path, output: Path):
    """An immediately readable, equal-scale initial/final front-view pair."""
    scene = "tilted_corridor_30deg"
    with np.load(experiment / "scenes" / f"{scene}.npz") as data:
        cloud, half, initial = (
            data["planning_point_cloud"],
            data["body_half_extents"],
            data["initial_trajectory"],
        )
    final = load_history(experiment, scene, "full")[0][-1]
    node = len(initial) // 2
    fig = plt.figure(figsize=(12, 6.8), facecolor=BACKGROUND)
    fig.text(
        0.5,
        0.945,
        "同一倾斜通道：水平机身 → 自动侧倾",
        ha="center",
        fontsize=23,
        weight="bold",
    )
    fig.text(
        0.5,
        0.895,
        "正视投影 y–z · 左右比例相同 · 原始尺寸与姿态角",
        ha="center",
        fontsize=12,
        color=MUTED,
    )
    for index, (name, pose) in enumerate(
        (("统一初始轨迹", initial[node]), ("优化后的同一节点", final[node]))
    ):
        ax = fig.add_axes([0.045 + index * 0.485, 0.24, 0.425, 0.57], facecolor="white")
        nearby = cloud[np.abs(cloud[:, 0] - pose[0]) < 0.10]
        ax.scatter(nearby[:, 1], nearby[:, 2], s=9, alpha=0.40, color="#678696")
        corners = geometry.obb_corners(pose, half)
        ordered = np.argsort([corners[list(f), 0].mean() for f in geometry.FACES])
        ax.add_collection(
            PolyCollection(
                [corners[list(geometry.FACES[i]), 1:] for i in ordered],
                facecolors=[FACE_COLORS[i] for i in ordered],
                edgecolors=INK,
                linewidths=1.6,
                zorder=3,
            )
        )
        zero = pose.copy()
        zero[3:] = 0
        ax.add_collection(
            LineCollection(
                [s[:, 1:] for s in geometry.segments(zero[None], half)],
                colors="#96a2b1",
                linestyles="dashed",
                linewidths=1.4,
                zorder=2,
            )
        )
        top = corners[[1, 3, 7, 5, 1]]
        ax.plot(top[:, 1], top[:, 2], color=GOLD, lw=4, zorder=4)
        tips = transform_local([[0, -half[1] * 1.35, 0], [0, half[1] * 1.35, 0]], pose)
        ax.plot(tips[:, 1], tips[:, 2], color=ANGLE_COLORS[0], lw=3, zorder=5)
        ax.set(
            xlim=(-0.42, 0.42),
            ylim=(0.05, 0.75),
            aspect="equal",
            xlabel="y / m",
            ylabel="z / m",
        )
        ax.grid(alpha=0.12)
        ax.set_title(
            f"{name}：roll {np.degrees(pose[3]):+.1f}°",
            fontsize=17,
            weight="bold",
            pad=12,
        )
        fig.text(
            0.257 + index * 0.485,
            0.12,
            f"z = {pose[2]:.3f} m    pitch {np.degrees(pose[4]):+.1f}°    yaw {np.degrees(pose[5]):+.1f}°",
            ha="center",
            fontsize=12,
        )
    fig.text(
        0.5,
        0.062,
        "金色 = 顶面；绿色线 = 机身横向轴；灰色虚线 = 同中心的水平参考",
        ha="center",
        fontsize=11,
        color=MUTED,
    )
    fig.text(
        0.5,
        0.022,
        "此图展示姿态变化；该场景整条轨迹仍有 1.27 mm 穿透。",
        ha="center",
        fontsize=10,
        color="#b13e35",
    )
    fig.savefig(output / "roll_before_after.png", dpi=140, facecolor=BACKGROUND)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", dest="experiment", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--scenes", nargs="+")
    parser.add_argument("--fps", type=int, default=18)
    parser.add_argument("--preview-only", action="store_true")
    args = parser.parse_args(argv)
    available = SavedRun(args.experiment).scenes
    args.scenes = (
        args.scenes or [name for name in SCENES if name in available] or available
    )
    if set(args.scenes) - set(available):
        parser.error("--scenes contains scene IDs absent from this run")
    args.output = args.output or args.experiment / "videos"
    if args.fps < 1:
        parser.error("--fps must be positive")
    configure_style()
    args.output.mkdir(parents=True, exist_ok=True)
    for scene in args.scenes:
        render_scene(args, scene)
    if "tilted_corridor_30deg" in args.scenes:
        make_roll_comparison(args.experiment, args.experiment / "report")


if __name__ == "__main__":
    main()

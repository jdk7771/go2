#!/usr/bin/env python3
"""Batch 3D point-cloud validation for the DeepRobotics M20 body planner.

This is intentionally an offline geometry suite.  It uses the M20 URDF body
collision geometry and the front/rear lidar locations from the M20 MJCF, then
simulates sparse noisy point clouds with field-of-view, angular occlusion,
dropout, voxelization, and isolated outliers.  A separate dense, noise-free
surface cloud is used only for final collision validation.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree

import m20_core as core


TARGET_CLEARANCE = 0.020
PLANNING_CLEARANCE = 0.047
POINT_VOXEL = 0.020
NODES = 23


@dataclass
class Scene:
    name: str
    family: str
    sensor_surface: np.ndarray
    truth_surface: np.ndarray
    params: dict
    seed_kind: str
    suggested_roll_deg: float = 0.0
    suggested_z_m: float = 0.40
    suggested_y_m: float = 0.0


def box_surface(
    center: np.ndarray,
    size: np.ndarray,
    rpy: np.ndarray,
    spacing: float,
) -> np.ndarray:
    """Sample all six faces of an oriented box."""
    center = np.asarray(center, dtype=float)
    size = np.asarray(size, dtype=float)
    half = size / 2.0
    R = core.rpy_matrix(np.asarray(rpy, dtype=float))
    faces = []
    for fixed_axis in range(3):
        varying = [a for a in range(3) if a != fixed_axis]
        u = core.grid_values(-half[varying[0]], half[varying[0]], spacing)
        v = core.grid_values(-half[varying[1]], half[varying[1]], spacing)
        uu, vv = np.meshgrid(u, v, indexing="ij")
        for sign in (-1.0, 1.0):
            local = np.zeros((uu.size, 3))
            local[:, fixed_axis] = sign * half[fixed_axis]
            local[:, varying[0]] = uu.ravel()
            local[:, varying[1]] = vv.ravel()
            faces.append(center + local @ R.T)
    return np.vstack(faces)


def box_center_for_min_z(
    xy: tuple[float, float], bottom_z: float, size: np.ndarray, rpy: np.ndarray
) -> np.ndarray:
    half = np.asarray(size) / 2.0
    corners = np.array(
        [[sx * half[0], sy * half[1], sz * half[2]]
         for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)],
        dtype=float,
    )
    rotated = corners @ core.rpy_matrix(np.asarray(rpy)).T
    return np.array([xy[0], xy[1], bottom_z - float(rotated[:, 2].min())])


def ground(spacing: float) -> np.ndarray:
    return core.horizontal_plane((-1.75, 1.75), (-0.78, 0.78), 0.0, spacing)[0]


def combine_points(*parts: np.ndarray) -> np.ndarray:
    return np.vstack([p for p in parts if len(p)])


def make_box_scene(
    name: str,
    family: str,
    boxes: list[tuple[np.ndarray, np.ndarray, np.ndarray]],
    params: dict,
    seed_kind: str,
    suggested_roll_deg: float = 0.0,
    suggested_z_m: float = 0.40,
    suggested_y_m: float = 0.0,
) -> Scene:
    sensor_parts = [ground(0.030)]
    truth_parts = [ground(0.022)]
    for center, size, rpy in boxes:
        sensor_parts.append(box_surface(center, size, rpy, 0.016))
        truth_parts.append(box_surface(center, size, rpy, 0.0075))
    return Scene(
        name=name,
        family=family,
        sensor_surface=combine_points(*sensor_parts),
        truth_surface=combine_points(*truth_parts),
        params=params,
        seed_kind=seed_kind,
        suggested_roll_deg=suggested_roll_deg,
        suggested_z_m=suggested_z_m,
        suggested_y_m=suggested_y_m,
    )


def build_scenes(nominal_height: float, half: np.ndarray) -> list[Scene]:
    scenes: list[Scene] = []

    # Nine low beams: three underside heights and three genuinely oriented boxes.
    orientations = [
        (0.0, 0.0),
        (20.0, 8.0),
        (40.0, -8.0),
    ]
    for bottom in (0.430, 0.450, 0.470):
        for yaw_deg, roll_deg in orientations:
            size = np.array([0.34, 1.10, 0.10])
            rpy = np.radians([roll_deg, 0.0, yaw_deg])
            center = box_center_for_min_z((0.0, 0.0), bottom, size, rpy)
            safe_z = max(0.285, bottom - half[2] - PLANNING_CLEARANCE)
            name = f"beam_h{int(bottom*1000):03d}_yaw{int(yaw_deg):02d}_tilt{roll_deg:+.0f}"
            scenes.append(
                make_box_scene(
                    name,
                    "low_beam",
                    [(center, size, rpy)],
                    {"bottom_m": bottom, "yaw_deg": yaw_deg, "tilt_deg": roll_deg},
                    "lower",
                    suggested_z_m=safe_z,
                )
            )

    # Two beams sloped along the travel direction, so their underside changes in x.
    for pitch_deg in (-10.0, 10.0):
        bottom = 0.450
        size = np.array([0.48, 1.02, 0.10])
        rpy = np.radians([0.0, pitch_deg, 25.0])
        center = box_center_for_min_z((0.0, 0.0), bottom, size, rpy)
        name = f"beam_h450_yaw25_pitch{pitch_deg:+.0f}"
        scenes.append(
            make_box_scene(
                name,
                "low_beam",
                [(center, size, rpy)],
                {"bottom_m": bottom, "yaw_deg": 25.0, "pitch_deg": pitch_deg},
                "lower",
                suggested_z_m=max(0.285, bottom - half[2] - PLANNING_CLEARANCE),
            )
        )

    # Four 3D tilted corridors.  These are extruded surfaces, but their sweep is
    # retained as the clean control for isolating roll torque.
    for angle_deg in (10.0, 20.0, 30.0, 35.0):
        angle = math.radians(angle_deg)
        sensor_wall = core.tilted_corridor(angle, 0.102, 0.018, center_z=nominal_height)[0]
        truth_wall = core.tilted_corridor(angle, 0.102, 0.0075, center_z=nominal_height)[0]
        scenes.append(
            Scene(
                name=f"tilted_corridor_{int(angle_deg):02d}deg",
                family="tilted_corridor",
                sensor_surface=combine_points(sensor_wall, ground(0.030)),
                truth_surface=combine_points(truth_wall, ground(0.022)),
                params={"corridor_angle_deg": angle_deg, "half_gap_m": 0.102},
                seed_kind="roll",
                suggested_roll_deg=angle_deg,
            )
        )

    # True 3D asymmetric scene: diagonal low beam plus staggered side obstacles.
    beam_size = np.array([0.38, 0.92, 0.10])
    beam_rpy = np.radians([8.0, 5.0, 28.0])
    beam_bottom = 0.462
    beam_center = box_center_for_min_z((0.02, 0.03), beam_bottom, beam_size, beam_rpy)
    boxes = [
        (beam_center, beam_size, beam_rpy),
        (np.array([-0.38, 0.235, 0.30]), np.array([0.28, 0.19, 0.60]), np.radians([0, 0, -8])),
        (np.array([0.36, -0.235, 0.27]), np.array([0.30, 0.19, 0.54]), np.radians([0, 0, 10])),
    ]
    scenes.append(
        make_box_scene(
            "asymmetric_overhang_slalom",
            "true_3d",
            boxes,
            {"beam_bottom_m": beam_bottom, "beam_rpy_deg": [8.0, 5.0, 28.0]},
            "combo",
            suggested_z_m=beam_bottom - half[2] - PLANNING_CLEARANCE,
            suggested_y_m=-0.025,
        )
    )

    # Second true 3D scene: two different-height overhangs and offset pillars.
    b1_size = np.array([0.30, 0.72, 0.11])
    b1_rpy = np.radians([-10.0, 7.0, -22.0])
    b1 = box_center_for_min_z((-0.35, 0.03), 0.455, b1_size, b1_rpy)
    b2_size = np.array([0.32, 0.72, 0.10])
    b2_rpy = np.radians([11.0, -6.0, 24.0])
    b2 = box_center_for_min_z((0.42, -0.02), 0.470, b2_size, b2_rpy)
    boxes2 = [
        (b1, b1_size, b1_rpy),
        (b2, b2_size, b2_rpy),
        (np.array([0.02, 0.245, 0.22]), np.array([0.24, 0.17, 0.44]), np.radians([0, 0, 15])),
    ]
    scenes.append(
        make_box_scene(
            "double_overhang_offset_pillar",
            "true_3d",
            boxes2,
            {"overhang_bottoms_m": [0.455, 0.470]},
            "combo",
            suggested_z_m=0.345,
            suggested_y_m=-0.035,
        )
    )
    return scenes


def angular_occlusion_scan(
    surface: np.ndarray,
    origin: np.ndarray,
    yaw: float,
    rng: np.random.Generator,
    angular_resolution_deg: float = 0.8,
) -> np.ndarray:
    R = core.rz(yaw)
    vec_world = surface - origin
    vec_sensor = vec_world @ R
    distance = np.linalg.norm(vec_sensor, axis=1)
    horizontal = np.arctan2(vec_sensor[:, 1], vec_sensor[:, 0])
    elevation = np.arctan2(vec_sensor[:, 2], np.linalg.norm(vec_sensor[:, :2], axis=1))
    keep = (
        (vec_sensor[:, 0] > 0.0)
        & (distance > 0.12)
        & (distance < 3.2)
        & (np.abs(horizontal) < math.radians(67.0))
        & (elevation > math.radians(-42.0))
        & (elevation < math.radians(34.0))
    )
    if not np.any(keep):
        return np.empty((0, 3))
    idx = np.flatnonzero(keep)
    resolution = math.radians(angular_resolution_deg)
    az_bin = np.floor((horizontal[idx] + math.radians(67.0)) / resolution).astype(np.int64)
    el_bin = np.floor((elevation[idx] + math.radians(42.0)) / resolution).astype(np.int64)
    key = az_bin * 1000 + el_bin
    order = np.lexsort((distance[idx], key))
    ordered_idx = idx[order]
    ordered_key = key[order]
    first = np.r_[True, ordered_key[1:] != ordered_key[:-1]]
    chosen = ordered_idx[first]
    chosen = chosen[rng.random(chosen.size) > 0.075]
    points = surface[chosen].copy()
    ray = points - origin
    ray_norm = np.linalg.norm(ray, axis=1)
    unit = ray / np.maximum(ray_norm[:, None], 1.0e-12)
    radial_sigma = 0.0035 + 0.0012 * ray_norm
    points += unit * rng.normal(0.0, radial_sigma)[:, None]
    points += rng.normal(0.0, 0.0012, size=points.shape)
    return points


def voxel_centroids(points: np.ndarray, voxel: float) -> np.ndarray:
    key = np.floor(points / voxel).astype(np.int64)
    unique, inverse = np.unique(key, axis=0, return_inverse=True)
    sums = np.zeros((len(unique), 3), dtype=float)
    np.add.at(sums, inverse, points)
    count = np.bincount(inverse)
    return sums / count[:, None]


def simulate_m20_lidar_cloud(
    surface: np.ndarray,
    nominal_height: float,
    seed: int,
) -> tuple[np.ndarray, dict]:
    rng = np.random.default_rng(seed)
    scans = []
    # M20 MJCF: front_lidar [0.32028, 0, -0.013], rear_lidar [-0.32028, 0, -0.013].
    stations = np.linspace(-1.20, 1.20, 7)
    per_scan_counts = []
    for base_x in stations:
        for local_x, yaw in ((0.32028, 0.0), (-0.32028, math.pi)):
            origin = np.array([base_x + local_x, 0.0, nominal_height - 0.013])
            scan = angular_occlusion_scan(surface, origin, yaw, rng)
            scans.append(scan)
            per_scan_counts.append(len(scan))
    merged = np.vstack(scans)
    outlier_count = max(8, int(0.006 * len(merged)))
    outliers = np.column_stack(
        (
            rng.uniform(-1.45, 1.45, outlier_count),
            rng.uniform(-0.65, 0.65, outlier_count),
            rng.uniform(0.04, 0.72, outlier_count),
        )
    )
    with_outliers = np.vstack((merged, outliers))
    tree = cKDTree(with_outliers)
    neighbor_count = tree.query_ball_point(with_outliers, r=0.055, return_length=True)
    filtered = with_outliers[neighbor_count >= 2]
    cloud = voxel_centroids(filtered, POINT_VOXEL)
    stats = {
        "scan_count": len(scans),
        "raw_return_count": int(len(merged)),
        "injected_outlier_count": int(outlier_count),
        "after_radius_filter_count": int(len(filtered)),
        "voxelized_point_count": int(len(cloud)),
        "mean_returns_per_scan": float(np.mean(per_scan_counts)),
        "voxel_size_m": POINT_VOXEL,
        "range_noise_model": "sigma=3.5mm+1.2mm/m plus 1.2mm isotropic",
        "dropout_probability": 0.075,
        "angular_resolution_deg": 0.8,
    }
    return cloud, stats


def initial_trajectory(nominal_height: float) -> np.ndarray:
    return core.make_initial_trajectory(NODES, nominal_height)


def make_scene_seed(scene: Scene, nominal_height: float) -> np.ndarray:
    seed = initial_trajectory(nominal_height)
    x = seed[:, 0]
    profile = np.exp(-((x / 0.72) ** 6))
    if scene.seed_kind in ("lower", "combo"):
        seed[:, 2] = nominal_height + (scene.suggested_z_m - nominal_height) * profile
    if scene.seed_kind == "roll":
        seed[:, 3] = math.radians(scene.suggested_roll_deg) * profile
    if scene.seed_kind == "combo":
        seed[:, 1] = scene.suggested_y_m * profile + 0.028 * np.sin(math.pi * (x + 1.3) / 2.6) * profile
        seed[:, 3] = math.radians(5.0) * profile
        seed[:, 4] = math.radians(-3.0) * np.sin(math.pi * x / 1.3) * profile
    seed[0] = initial_trajectory(nominal_height)[0]
    seed[-1] = initial_trajectory(nominal_height)[-1]
    return seed


def optimize(problem: core.TrajectoryProblem, seed: np.ndarray, maxiter: int = 300) -> tuple[np.ndarray, dict]:
    result = minimize(
        problem.cost_grad,
        problem.pack(seed),
        method="L-BFGS-B",
        jac=True,
        bounds=problem.bounds(),
        options={"maxiter": maxiter, "ftol": 2.0e-9, "gtol": 1.0e-4, "maxls": 60},
    )
    return problem.unpack(result.x), {
        "formal_success": bool(result.success),
        "message": str(result.message),
        "iterations": int(result.nit),
        "evaluations": int(result.nfev),
        "cost": float(result.fun),
        "gradient_inf_norm": float(np.linalg.norm(result.jac, ord=np.inf)),
    }


def plan_scene(
    scene: Scene,
    cloud: np.ndarray,
    half: np.ndarray,
    nominal_height: float,
) -> tuple[np.ndarray, dict, dict, dict]:
    initial = initial_trajectory(nominal_height)
    baseline = core.validate_trajectory(initial, scene.truth_surface, half, TARGET_CLEARANCE)
    weights = core.PlannerWeights(
        collision_stiffness=1_600_000.0,
        path_xy=18.0,
        height=32.0,
        attitude=0.30,
        velocity_translation=8.0,
        acceleration_translation=110.0,
        velocity_rotation=1.5,
        acceleration_rotation=20.0,
    )
    point_weights = np.full(len(cloud), POINT_VOXEL**2)
    problem = core.TrajectoryProblem(
        cloud,
        point_weights,
        initial,
        half,
        nominal_height,
        PLANNING_CLEARANCE,
        "full",
        weights,
    )
    seed = make_scene_seed(scene, nominal_height)
    trajectory, opt = optimize(problem, seed)
    validation = core.validate_trajectory(trajectory, scene.truth_surface, half, TARGET_CLEARANCE)

    # The dense truth surface is held out: it scores the final trajectory but
    # never changes the planning margin or triggers a retry.
    opt = {"first_pass": opt, "used_retry": False}
    return trajectory, baseline, validation, opt


BOX_EDGES = [
    (0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
    (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7),
]


def obb_corners(pose: np.ndarray, half: np.ndarray) -> np.ndarray:
    local = np.array(
        [[sx * half[0], sy * half[1], sz * half[2]]
         for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    )
    return pose[:3] + local @ core.rpy_matrix(pose[3:]).T


def draw_obb(ax, pose: np.ndarray, half: np.ndarray, color: str, alpha: float = 1.0, lw: float = 1.2):
    corners = obb_corners(pose, half)
    artists = []
    for i, j in BOX_EDGES:
        artists.extend(ax.plot(*zip(corners[i], corners[j]), color=color, alpha=alpha, lw=lw))
    return artists


def configure_3d_axes(ax) -> None:
    ax.set_xlim(-1.45, 1.45)
    ax.set_ylim(-0.62, 0.62)
    ax.set_zlim(0.0, 0.72)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.view_init(elev=25, azim=-61)
    ax.set_box_aspect((2.9, 1.24, 0.85))


def plot_scene(
    path: Path,
    scene: Scene,
    cloud: np.ndarray,
    trajectory: np.ndarray,
    half: np.ndarray,
    validation: dict,
) -> None:
    fig = plt.figure(figsize=(15.0, 8.5))
    ax = fig.add_subplot(2, 2, (1, 3), projection="3d")
    show = cloud[:: max(1, len(cloud) // 7000)]
    scatter = ax.scatter(show[:, 0], show[:, 1], show[:, 2], c=show[:, 2], s=2.0, cmap="viridis", alpha=0.62)
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color="tab:red", lw=2.3, label="optimized center path")
    for idx in np.linspace(0, len(trajectory) - 1, 8, dtype=int):
        draw_obb(ax, trajectory[idx], half, "tab:orange", alpha=0.62, lw=1.0)
    configure_3d_axes(ax)
    ax.set_title(
        f"{scene.name}\nM20 lidar-style point cloud + optimized body trajectory",
        fontsize=10,
        pad=8,
    )
    ax.legend(loc="upper left")
    fig.colorbar(scatter, ax=ax, shrink=0.50, pad=0.07, label="point z [m]")

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(trajectory[:, 0], np.degrees(trajectory[:, 3]), label="roll")
    ax2.plot(trajectory[:, 0], np.degrees(trajectory[:, 4]), label="pitch")
    ax2.plot(trajectory[:, 0], np.degrees(trajectory[:, 5]), label="yaw")
    ax2.set_xlabel("path x [m]")
    ax2.set_ylabel("angle [deg]")
    ax2.set_title("6D attitude trajectory")
    ax2.grid(alpha=0.25)
    ax2.legend()

    ax3 = fig.add_subplot(2, 2, 4)
    ax3.plot(trajectory[:, 0], trajectory[:, 2], label="base z")
    ax3.plot(trajectory[:, 0], trajectory[:, 1], label="base y")
    dense = validation["dense_poses"]
    ax3.plot(dense[:, 0], validation["dense_clearance"], label="truth clearance")
    ax3.axhline(TARGET_CLEARANCE, color="tab:green", lw=1, label="20 mm target")
    ax3.axhline(0.0, color="tab:red", lw=1)
    ax3.set_xlabel("path x [m]")
    ax3.set_ylabel("position / clearance [m]")
    ax3.set_title("Translation and independent dense clearance")
    ax3.grid(alpha=0.25)
    ax3.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(path, dpi=165)
    plt.close(fig)


def make_animation(
    path: Path,
    scene: Scene,
    cloud: np.ndarray,
    trajectory: np.ndarray,
    half: np.ndarray,
) -> None:
    dense = core.densify_trajectory(trajectory, subdivisions=4)
    fig = plt.figure(figsize=(10.5, 5.4))
    ax = fig.add_subplot(111, projection="3d")
    show = cloud[:: max(1, len(cloud) // 6500)]
    ax.scatter(show[:, 0], show[:, 1], show[:, 2], c=show[:, 2], s=2, cmap="viridis", alpha=0.55)
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], "--", color="0.35", lw=1.3)
    configure_3d_axes(ax)
    title = ax.set_title(scene.name)
    lines = draw_obb(ax, dense[0], half, "tab:red", alpha=1.0, lw=2.0)

    def update(frame: int):
        corners = obb_corners(dense[frame], half)
        for line, (i, j) in zip(lines, BOX_EDGES):
            line.set_data_3d(*zip(corners[i], corners[j]))
        pose = dense[frame]
        title.set_text(
            f"{scene.name} | x={pose[0]:+.2f} m, z={pose[2]:.3f} m, "
            f"r/p/y={np.degrees(pose[3]):+.1f}/{np.degrees(pose[4]):+.1f}/{np.degrees(pose[5]):+.1f}°"
        )
        return [*lines, title]

    ani = animation.FuncAnimation(fig, update, frames=len(dense), interval=50, blit=False)
    writer = animation.FFMpegWriter(fps=20, bitrate=2200)
    ani.save(path, writer=writer, dpi=130)
    plt.close(fig)


def clean_validation(validation: dict) -> dict:
    return {k: v for k, v in validation.items() if not isinstance(v, np.ndarray)}


def plot_suite_summary(path: Path, rows: list[dict]) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(15.0, 10.0))
    names = [r["name"] for r in rows]
    clearance = np.array([r["minimum_signed_distance_m"] * 1000 for r in rows])
    colors = ["tab:green" if c >= TARGET_CLEARANCE * 1000 - 1.5 else "tab:red" for c in clearance]
    ax = axes[0, 0]
    ax.barh(np.arange(len(rows)), clearance, color=colors)
    ax.axvline(TARGET_CLEARANCE * 1000, color="k", lw=1.2, label="20 mm target")
    ax.set_yticks(np.arange(len(rows)), names, fontsize=7)
    ax.invert_yaxis()
    ax.set_xlabel("independent minimum clearance [mm]")
    ax.set_title("All scenes: dense ground-truth validation")
    ax.legend()

    beam_rows = [r for r in rows if r["family"] == "low_beam" and "tilt_deg" in r["params"]]
    heights = sorted({r["params"]["bottom_m"] for r in beam_rows})
    yaws = sorted({r["params"]["yaw_deg"] for r in beam_rows})
    matrix = np.full((len(heights), len(yaws)), np.nan)
    for r in beam_rows:
        matrix[heights.index(r["params"]["bottom_m"]), yaws.index(r["params"]["yaw_deg"])] = r[
            "minimum_signed_distance_m"
        ] * 1000
    ax = axes[0, 1]
    im = ax.imshow(matrix, cmap="RdYlGn", vmin=0, vmax=max(30, np.nanmax(matrix)), aspect="auto")
    ax.set_xticks(range(len(yaws)), [f"{y:.0f}°" for y in yaws])
    ax.set_yticks(range(len(heights)), [f"{h:.3f} m" for h in heights])
    ax.set_xlabel("beam yaw (tilts are 0°, +8°, -8°)")
    ax.set_ylabel("minimum beam underside")
    ax.set_title("Low-beam sweep clearance [mm]")
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            ax.text(j, i, f"{matrix[i,j]:.1f}", ha="center", va="center", fontsize=9)
    fig.colorbar(im, ax=ax, shrink=0.8)

    ax = axes[1, 0]
    x = np.arange(len(rows))
    ax.plot(x, [r["max_abs_roll_deg"] for r in rows], "o-", label="max |roll|")
    ax.plot(x, [r["max_abs_pitch_deg"] for r in rows], "s-", label="max |pitch|")
    ax.plot(x, [r["max_abs_yaw_deg"] for r in rows], "^-", label="max |yaw|")
    ax.set_xticks(x, [str(i + 1) for i in x], fontsize=7)
    ax.set_xlabel("scene index (same order as clearance chart)")
    ax.set_ylabel("degrees")
    ax.set_title("Attitude used by the optimizer")
    ax.grid(alpha=0.25)
    ax.legend()

    ax = axes[1, 1]
    ax.scatter(
        [r["point_count"] for r in rows],
        [r["optimization_iterations"] for r in rows],
        c=clearance,
        cmap="viridis",
        s=55,
    )
    for i, r in enumerate(rows):
        ax.annotate(str(i + 1), (r["point_count"], r["optimization_iterations"]), fontsize=7)
    ax.set_xlabel("voxelized planning point count")
    ax.set_ylabel("final optimization iterations")
    ax.set_title("Point-cloud size and optimization work")
    ax.grid(alpha=0.25)

    fig.suptitle("M20 realistic 3D point-cloud virtual-wrench batch validation", fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(path, dpi=175)
    plt.close(fig)


def write_report(path: Path, metadata: dict, rows: list[dict]) -> None:
    passed = sum(r["safety_clearance_met"] for r in rows)
    collision_free = sum(r["collision_free"] for r in rows)
    true3d = [r for r in rows if r["family"] == "true_3d"]
    low = [r for r in rows if r["family"] == "low_beam"]
    corridors = [r for r in rows if r["family"] == "tilted_corridor"]
    worst = min(rows, key=lambda r: r["minimum_signed_distance_m"])
    table_lines = [
        "| 场景 | 类型 | 点数 | 原始轨迹最小距离 | 优化后最小距离 | 最低 z | 最大 roll/pitch/yaw | 20 mm |",
        "|---|---|---:|---:|---:|---:|---:|---|",
    ]
    for r in rows:
        table_lines.append(
            f"| {r['name']} | {r['family']} | {r['point_count']} | "
            f"{r['baseline_min_distance_m']*1000:.1f} mm | {r['minimum_signed_distance_m']*1000:.1f} mm | "
            f"{r['minimum_base_z_m']:.3f} m | {r['max_abs_roll_deg']:.1f}/{r['max_abs_pitch_deg']:.1f}/{r['max_abs_yaw_deg']:.1f}° | "
            f"{'通过' if r['safety_clearance_met'] else '未通过'} |"
        )
    text = f"""# M20 真实化三维点云批量验证

## 结论

本轮不再只用规则的二维截面。共测试 **{len(rows)} 个三维场景**：

- {len(low)} 个不同高度、yaw、横向倾角和纵向坡度的低梁；
- {len(corridors)} 个不同倾角的通道，作为虚拟 roll 力矩的受控对照；
- {len(true3d)} 个非对称组合场景，包含斜梁、不同高度悬障和左右错位立柱。

独立密集真值表面复检中，{collision_free}/{len(rows)} 条轨迹无穿透，
{passed}/{len(rows)} 条达到 20 mm 安全目标。最紧场景为 `{worst['name']}`，
最小距离 {worst['minimum_signed_distance_m']*1000:.1f} mm。

## 点云是否“真实一些”

规划器输入不是解析障碍，也不是最终验收用的密集表面，而是模拟 M20 前后雷达形成的
稀疏点云。传感器位置采用 M20 MJCF 的 `front_lidar=[0.32028,0,-0.013]` 和
`rear_lidar=[-0.32028,0,-0.013]`，并加入：

- 134° 水平、76° 垂直视场与 0.8° 角分辨率；
- 每个角度栅格只保留最近回波，形成遮挡；
- 7.5% 随机丢点；
- 距离相关高斯噪声、独立三轴噪声；
- 0.6% 孤立离群点，再经过半径滤波和 20 mm 体素化；
- 沿局部路线 7 个机身位置、前后雷达共 14 帧离线融合。

所有场景统一使用 47 mm 点云势能激活距离，以覆盖 20 mm 体素、噪声和遮挡误差。
最终碰撞验收使用另一套 7.5--22 mm 密集、无噪声真值表面并在轨迹节点间做 10 倍插值；
该真值只在全部规划结束后评分，不用于调参、重试或选择轨迹。

## 批量结果

{chr(10).join(table_lines)}

## 边界

这仍然是**仿真的雷达点云**，不是 M20 现场录制的 rosbag/PCD；“真实化”指视场、遮挡、
噪声、丢点、离群点和体素处理更接近传感器。当前只验证 `base_link` 参考轨迹，尚未加入
四轮接触、腿长/关节限位、动力学和控制跟踪。任何实机下发之前，必须先用真实点云离线
重放，再做轮端接触可行性和仿真低速跟踪。

## 复现

```bash
python run_3d_suite.py --output results
```

M20 URDF：`{metadata['urdf']}`
"""
    path.write_text(text, encoding="utf-8")


def run_suite(output: Path, urdf: Path, config: Path, make_video: bool) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    scene_dir = output / "scenes"
    figure_dir = output / "figures"
    scene_dir.mkdir(exist_ok=True)
    figure_dir.mkdir(exist_ok=True)

    _, half, mass = core.derive_m20_body_obb(urdf)
    nominal_height = core.read_nominal_height(config)
    scenes = build_scenes(nominal_height, half)
    rows: list[dict] = []
    saved = {}

    for index, scene in enumerate(scenes):
        print(f"[{index+1:02d}/{len(scenes):02d}] {scene.name}", flush=True)
        cloud, cloud_stats = simulate_m20_lidar_cloud(scene.sensor_surface, nominal_height, 1000 + index)
        trajectory, baseline, validation, optimizer = plan_scene(scene, cloud, half, nominal_height)
        first_or_retry = optimizer.get("retry") if optimizer.get("used_retry") else optimizer["first_pass"]
        row = {
            "index": index + 1,
            "name": scene.name,
            "family": scene.family,
            "params": scene.params,
            "point_count": int(len(cloud)),
            "truth_point_count": int(len(scene.truth_surface)),
            "baseline_min_distance_m": float(baseline["minimum_signed_distance_m"]),
            "minimum_signed_distance_m": float(validation["minimum_signed_distance_m"]),
            "minimum_safety_margin_m": float(validation["minimum_safety_margin_m"]),
            "collision_free": bool(validation["collision_free"]),
            "safety_clearance_met": bool(validation["safety_clearance_met"]),
            "minimum_base_z_m": float(trajectory[:, 2].min()),
            "maximum_abs_y_m": float(np.abs(trajectory[:, 1]).max()),
            "max_abs_roll_deg": float(np.abs(np.degrees(trajectory[:, 3])).max()),
            "max_abs_pitch_deg": float(np.abs(np.degrees(trajectory[:, 4])).max()),
            "max_abs_yaw_deg": float(np.abs(np.degrees(trajectory[:, 5])).max()),
            "optimization_iterations": int(first_or_retry["iterations"]),
            "optimizer_formal_success": bool(first_or_retry["formal_success"]),
            "used_adaptive_retry": bool(optimizer.get("used_retry", False)),
            "cloud_stats": cloud_stats,
            "optimizer": optimizer,
            "validation": clean_validation(validation),
        }
        rows.append(row)
        np.savez_compressed(
            scene_dir / f"{scene.name}.npz",
            planning_point_cloud=cloud,
            dense_truth_surface=scene.truth_surface,
            initial_trajectory=initial_trajectory(nominal_height),
            optimized_trajectory=trajectory,
            body_half_extents=half,
        )
        (scene_dir / f"{scene.name}.json").write_text(
            json.dumps(row, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        plot_scene(figure_dir / f"{scene.name}.png", scene, cloud, trajectory, half, validation)
        saved[scene.name] = (scene, cloud, trajectory, validation)

    metadata = {
        "urdf": str(urdf),
        "config": str(config),
        "m20_base_mass_kg": mass,
        "m20_body_obb_size_m": (2.0 * half).tolist(),
        "nominal_height_m": nominal_height,
        "target_clearance_m": TARGET_CLEARANCE,
        "planning_activation_distance_m": PLANNING_CLEARANCE,
        "scene_count": len(rows),
        "passed_20mm_count": int(sum(r["safety_clearance_met"] for r in rows)),
        "collision_free_count": int(sum(r["collision_free"] for r in rows)),
    }
    results = {"metadata": metadata, "scenes": rows}
    (output / "suite_results.json").write_text(
        json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    with (output / "suite_results.csv").open("w", newline="", encoding="utf-8") as f:
        columns = [
            "index", "name", "family", "point_count", "truth_point_count",
            "baseline_min_distance_m", "minimum_signed_distance_m",
            "minimum_safety_margin_m", "collision_free", "safety_clearance_met",
            "minimum_base_z_m", "maximum_abs_y_m", "max_abs_roll_deg",
            "max_abs_pitch_deg", "max_abs_yaw_deg", "optimization_iterations",
            "optimizer_formal_success", "used_adaptive_retry",
        ]
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    plot_suite_summary(output / "suite_summary.png", rows)
    write_report(output / "REPORT.md", metadata, rows)

    if make_video:
        animation_name = "asymmetric_overhang_slalom"
        scene, cloud, trajectory, _ = saved[animation_name]
        make_animation(output / "asymmetric_overhang_slalom.mp4", scene, cloud, trajectory, half)
    return results


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "results")
    parser.add_argument("--urdf", type=Path, default=core.DEFAULT_URDF)
    parser.add_argument("--m20-config", type=Path, default=core.DEFAULT_M20_CFG)
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()
    result = run_suite(args.output, args.urdf, args.m20_config, not args.no_video)
    print(json.dumps(result["metadata"], indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

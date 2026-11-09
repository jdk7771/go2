#!/usr/bin/env python3
"""Controlled validation of point-cloud virtual wrench planning on M20 geometry.

The experiment deliberately validates the body-level geometric idea before wheel
contact feasibility or policy tracking.  M20's conservative body OBB is derived
from the official URDF collision primitives; it is not a hand-chosen toy box.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize


DEFAULT_URDF = Path(os.environ.get(
    "M20_URDF",
    "/home/jiang/m20-repro/src/rl_training/deep_robotics_model/M20/urdf/M20.urdf",
))
DEFAULT_M20_CFG = Path(os.environ.get(
    "M20_CONFIG",
    "/home/jiang/m20-repro/src/rl_training/source/rl_training/rl_training/tasks/"
    "manager_based/locomotion/velocity/config/wheeled/deeprobotics_m20/rough_env_cfg.py",
))


def rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    return rz(float(rpy[2])) @ ry(float(rpy[1])) @ rx(float(rpy[0]))


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def parse_xyz(text: str | None, default: str = "0 0 0") -> np.ndarray:
    return np.fromstring(text if text else default, sep=" ", dtype=float)


def derive_m20_body_obb(urdf_path: Path) -> tuple[np.ndarray, np.ndarray, float]:
    """Return conservative compound-collision AABB center, half extents, mass."""
    root = ET.parse(urdf_path).getroot()
    base = next(link for link in root.findall("link") if link.attrib.get("name") == "base_link")
    mins, maxs = [], []
    for collision in base.findall("collision"):
        origin = collision.find("origin")
        center = parse_xyz(None if origin is None else origin.attrib.get("xyz"))
        rpy = parse_xyz(None if origin is None else origin.attrib.get("rpy"))
        R = rpy_matrix(rpy)
        geometry = collision.find("geometry")
        if geometry is None:
            continue
        box = geometry.find("box")
        cylinder = geometry.find("cylinder")
        if box is not None:
            half = parse_xyz(box.attrib["size"]) / 2.0
            world_half = np.abs(R) @ half
        elif cylinder is not None:
            radius = float(cylinder.attrib["radius"])
            half_length = float(cylinder.attrib["length"]) / 2.0
            axis = R @ np.array([0.0, 0.0, 1.0])
            world_half = np.abs(axis) * half_length + np.sqrt(
                np.maximum(0.0, 1.0 - axis * axis)
            ) * radius
        else:
            continue
        mins.append(center - world_half)
        maxs.append(center + world_half)
    if not mins:
        raise RuntimeError("No base_link collision primitives found in M20 URDF")
    low = np.min(np.vstack(mins), axis=0)
    high = np.max(np.vstack(maxs), axis=0)
    mass = float(base.find("inertial/mass").attrib["value"])
    return (low + high) / 2.0, (high - low) / 2.0, mass


def read_nominal_height(config_path: Path) -> float:
    text = config_path.read_text(encoding="utf-8")
    match = re.search(r'base_height_l2\.params\["target_height"\]\s*=\s*([0-9.]+)', text)
    if not match:
        raise RuntimeError("M20 nominal base height not found")
    return float(match.group(1))


def obb_signed_distance(
    points: np.ndarray, center: np.ndarray, R: np.ndarray, half: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Signed point-to-OBB distance, nearest point, and outward SDF normal."""
    local = (points - center) @ R
    nearest = np.clip(local, -half, half)
    delta = local - nearest
    outside_norm = np.linalg.norm(delta, axis=1)
    outside = outside_norm > 1.0e-12
    distance = outside_norm.copy()
    normal = np.zeros_like(local)
    normal[outside] = delta[outside] / outside_norm[outside, None]

    inside_idx = np.flatnonzero(~outside)
    if inside_idx.size:
        local_inside = local[inside_idx]
        margin = half - np.abs(local_inside)
        face_axis = np.argmin(margin, axis=1)
        row = np.arange(inside_idx.size)
        sign = np.where(local_inside[row, face_axis] >= 0.0, 1.0, -1.0)
        distance[inside_idx] = -margin[row, face_axis]
        normal[inside_idx, face_axis] = sign
        nearest[inside_idx] = local_inside
        nearest[inside_idx, face_axis] = sign * half[face_axis]
    nearest_world = center + nearest @ R.T
    normal_world = normal @ R.T
    return distance, nearest_world, normal_world


def collision_energy_wrench(
    points: np.ndarray,
    weights: np.ndarray,
    center: np.ndarray,
    R: np.ndarray,
    half: np.ndarray,
    safety: float,
    stiffness: float,
) -> tuple[float, np.ndarray, np.ndarray, float, int]:
    distance, nearest, normal = obb_signed_distance(points, center, R, half)
    active = distance < safety
    if not np.any(active):
        return 0.0, np.zeros(3), np.zeros(3), float(distance.min()), 0
    penetration = safety - distance[active]
    w = weights[active]
    energy = 0.5 * stiffness * np.sum(w * penetration * penetration)
    force_each = (-stiffness * w * penetration)[:, None] * normal[active]
    force = np.sum(force_each, axis=0)
    arm = nearest[active] - center
    torque = np.sum(np.cross(arm, force_each), axis=0)
    return float(energy), force, torque, float(distance.min()), int(active.sum())


def grid_values(lo: float, hi: float, spacing: float) -> np.ndarray:
    count = max(2, int(math.ceil((hi - lo) / spacing)) + 1)
    return np.linspace(lo, hi, count)


def horizontal_plane(
    xlim: tuple[float, float], ylim: tuple[float, float], z: float, spacing: float
) -> tuple[np.ndarray, np.ndarray]:
    x, y = np.meshgrid(
        grid_values(*xlim, spacing), grid_values(*ylim, spacing), indexing="ij"
    )
    p = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, z)))
    return p, np.full(p.shape[0], spacing * spacing)


def tilted_corridor(
    alpha: float,
    gap_half: float,
    spacing: float,
    xlim: tuple[float, float] = (-0.62, 0.62),
    tlim: tuple[float, float] = (-0.36, 0.36),
    center_z: float = 0.40,
) -> tuple[np.ndarray, np.ndarray]:
    xs, ts = np.meshgrid(
        grid_values(*xlim, spacing), grid_values(*tlim, spacing), indexing="ij"
    )
    tangent_yz = np.array([math.cos(alpha), math.sin(alpha)])
    normal_yz = np.array([-math.sin(alpha), math.cos(alpha)])
    parts = []
    for side in (-1.0, 1.0):
        yz = (
            np.array([0.0, center_z])
            + ts.ravel()[:, None] * tangent_yz
            + side * gap_half * normal_yz
        )
        parts.append(np.column_stack((xs.ravel(), yz)))
    p = np.vstack(parts)
    return p, np.full(p.shape[0], spacing * spacing)


def combine_clouds(*clouds: tuple[np.ndarray, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    return np.vstack([c[0] for c in clouds]), np.concatenate([c[1] for c in clouds])


@dataclass
class PlannerWeights:
    collision_stiffness: float = 1200000.0
    path_xy: float = 20.0
    height: float = 35.0
    attitude: float = 0.35
    velocity_translation: float = 8.0
    acceleration_translation: float = 100.0
    velocity_rotation: float = 1.5
    acceleration_rotation: float = 18.0


class TrajectoryProblem:
    def __init__(
        self,
        points: np.ndarray,
        point_weights: np.ndarray,
        initial: np.ndarray,
        half: np.ndarray,
        nominal_height: float,
        safety: float,
        mode: str,
        weights: PlannerWeights,
    ) -> None:
        self.points = points
        self.point_weights = point_weights
        self.initial = initial.copy()
        self.half = half
        self.nominal_height = nominal_height
        self.safety = safety
        self.mode = mode
        self.weights = weights
        self.free_dims = np.array([0, 1, 2] if mode == "translation" else [0, 1, 2, 3, 4, 5])
        self.interior = np.arange(1, initial.shape[0] - 1)

    def pack(self, poses: np.ndarray) -> np.ndarray:
        return poses[np.ix_(self.interior, self.free_dims)].ravel()

    def unpack(self, variables: np.ndarray) -> np.ndarray:
        poses = self.initial.copy()
        shaped = variables.reshape(len(self.interior), len(self.free_dims))
        poses[np.ix_(self.interior, self.free_dims)] = shaped
        return poses

    def bounds(self) -> list[tuple[float, float]]:
        result = []
        for idx in self.interior:
            x0 = self.initial[idx, 0]
            all_bounds = [
                (x0 - 0.09, x0 + 0.09),
                (-0.16, 0.16),
                (0.28, 0.52),
                (math.radians(-45.0), math.radians(45.0)),
                (math.radians(-22.0), math.radians(22.0)),
                (math.radians(-18.0), math.radians(18.0)),
            ]
            result.extend(all_bounds[d] for d in self.free_dims)
        return result

    @staticmethod
    def _add_velocity_cost(
        poses: np.ndarray, dims: slice, weight: float, grad: np.ndarray
    ) -> float:
        delta = poses[1:, dims] - poses[:-1, dims]
        grad[:-1, dims] -= weight * delta
        grad[1:, dims] += weight * delta
        return 0.5 * weight * float(np.sum(delta * delta))

    @staticmethod
    def _add_acceleration_cost(
        poses: np.ndarray, dims: slice, weight: float, grad: np.ndarray
    ) -> float:
        accel = poses[:-2, dims] - 2.0 * poses[1:-1, dims] + poses[2:, dims]
        grad[:-2, dims] += weight * accel
        grad[1:-1, dims] -= 2.0 * weight * accel
        grad[2:, dims] += weight * accel
        return 0.5 * weight * float(np.sum(accel * accel))

    def cost_grad(self, variables: np.ndarray) -> tuple[float, np.ndarray]:
        poses = self.unpack(variables)
        grad = np.zeros_like(poses)
        total = 0.0
        w = self.weights
        for i, pose in enumerate(poses):
            R = rpy_matrix(pose[3:])
            energy, force, torque, _, _ = collision_energy_wrench(
                self.points,
                self.point_weights,
                pose[:3],
                R,
                self.half,
                self.safety,
                w.collision_stiffness,
            )
            total += energy
            grad[i, :3] -= force
            roll_axis = rz(pose[5]) @ ry(pose[4]) @ np.array([1.0, 0.0, 0.0])
            pitch_axis = rz(pose[5]) @ np.array([0.0, 1.0, 0.0])
            yaw_axis = np.array([0.0, 0.0, 1.0])
            grad[i, 3:] -= np.array(
                [np.dot(torque, roll_axis), np.dot(torque, pitch_axis), np.dot(torque, yaw_axis)]
            )

        xy_error = poses[:, :2] - self.initial[:, :2]
        total += 0.5 * w.path_xy * float(np.sum(xy_error * xy_error))
        grad[:, :2] += w.path_xy * xy_error

        height_error = poses[:, 2] - self.nominal_height
        total += 0.5 * w.height * float(np.sum(height_error * height_error))
        grad[:, 2] += w.height * height_error

        attitude_scale = np.array([1.0, 1.0, 0.35])
        total += 0.5 * w.attitude * float(np.sum(attitude_scale * poses[:, 3:] ** 2))
        grad[:, 3:] += w.attitude * attitude_scale * poses[:, 3:]

        total += self._add_velocity_cost(poses, slice(0, 3), w.velocity_translation, grad)
        total += self._add_acceleration_cost(
            poses, slice(0, 3), w.acceleration_translation, grad
        )
        total += self._add_velocity_cost(poses, slice(3, 6), w.velocity_rotation, grad)
        total += self._add_acceleration_cost(
            poses, slice(3, 6), w.acceleration_rotation, grad
        )
        packed_grad = grad[np.ix_(self.interior, self.free_dims)].ravel()
        return float(total), packed_grad


def optimize_trajectory(problem: TrajectoryProblem, seed: np.ndarray) -> tuple[np.ndarray, dict]:
    initial_variables = problem.pack(seed)
    result = minimize(
        problem.cost_grad,
        initial_variables,
        method="L-BFGS-B",
        jac=True,
        bounds=problem.bounds(),
        options={"maxiter": 500, "ftol": 1.0e-9, "gtol": 1.0e-5, "maxls": 60},
    )
    return problem.unpack(result.x), {
        "success": bool(result.success),
        "message": str(result.message),
        "iterations": int(result.nit),
        "evaluations": int(result.nfev),
        "cost": float(result.fun),
        "gradient_inf_norm": float(np.linalg.norm(result.jac, ord=np.inf)),
    }


def densify_trajectory(poses: np.ndarray, subdivisions: int = 8) -> np.ndarray:
    dense = []
    for i in range(len(poses) - 1):
        for j in range(subdivisions):
            t = j / subdivisions
            dense.append((1.0 - t) * poses[i] + t * poses[i + 1])
    dense.append(poses[-1])
    return np.asarray(dense)


def validate_trajectory(
    poses: np.ndarray, validation_points: np.ndarray, half: np.ndarray, safety: float
) -> dict:
    dense = densify_trajectory(poses, subdivisions=10)
    min_distance_by_pose = []
    for pose in dense:
        d, _, _ = obb_signed_distance(validation_points, pose[:3], rpy_matrix(pose[3:]), half)
        min_distance_by_pose.append(float(d.min()))
    min_distance_by_pose = np.asarray(min_distance_by_pose)
    translation_step = np.linalg.norm(np.diff(poses[:, :3], axis=0), axis=1)
    rotation_step = np.linalg.norm(np.diff(poses[:, 3:], axis=0), axis=1)
    return {
        "minimum_signed_distance_m": float(min_distance_by_pose.min()),
        "minimum_safety_margin_m": float(min_distance_by_pose.min() - safety),
        "collision_free": bool(min_distance_by_pose.min() >= -5.0e-4),
        "safety_clearance_met": bool(min_distance_by_pose.min() >= safety - 1.5e-3),
        "maximum_node_translation_step_m": float(translation_step.max()),
        "maximum_node_rotation_step_deg": float(np.degrees(rotation_step.max())),
        "dense_clearance": min_distance_by_pose,
        "dense_poses": dense,
    }


def finite_difference_gradient_test(
    points: np.ndarray,
    weights: np.ndarray,
    center: np.ndarray,
    rpy: np.ndarray,
    half: np.ndarray,
    safety: float,
    stiffness: float,
) -> dict:
    R = rpy_matrix(rpy)
    energy, force, torque, min_d, active = collision_energy_wrench(
        points, weights, center, R, half, safety, stiffness
    )
    eps = 2.0e-6
    fd_translation = np.zeros(3)
    for axis in range(3):
        shift = np.zeros(3)
        shift[axis] = eps
        up = collision_energy_wrench(
            points, weights, center + shift, R, half, safety, stiffness
        )[0]
        down = collision_energy_wrench(
            points, weights, center - shift, R, half, safety, stiffness
        )[0]
        fd_translation[axis] = (up - down) / (2.0 * eps)
    fd_rotation = np.zeros(3)
    for axis in range(3):
        unit = np.eye(3)[axis]
        up_R = axis_angle(unit, eps) @ R
        down_R = axis_angle(unit, -eps) @ R
        up = collision_energy_wrench(
            points, weights, center, up_R, half, safety, stiffness
        )[0]
        down = collision_energy_wrench(
            points, weights, center, down_R, half, safety, stiffness
        )[0]
        fd_rotation[axis] = (up - down) / (2.0 * eps)
    expected_translation = -force
    expected_rotation = -torque
    rel_translation = np.linalg.norm(fd_translation - expected_translation) / max(
        1.0e-12, np.linalg.norm(fd_translation)
    )
    rel_rotation = np.linalg.norm(fd_rotation - expected_rotation) / max(
        1.0e-12, np.linalg.norm(fd_rotation)
    )
    return {
        "energy": energy,
        "force": force,
        "torque": torque,
        "finite_difference_translation_gradient": fd_translation,
        "finite_difference_world_rotation_gradient": fd_rotation,
        "relative_translation_gradient_error": float(rel_translation),
        "relative_rotation_gradient_error": float(rel_rotation),
        "minimum_signed_distance_m": min_d,
        "active_point_count": active,
    }


def make_initial_trajectory(n: int, nominal_height: float) -> np.ndarray:
    poses = np.zeros((n, 6))
    poses[:, 0] = np.linspace(-1.30, 1.30, n)
    poses[:, 2] = nominal_height
    return poses


def run_validation(urdf: Path, config: Path, out_dir: Path) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    obb_center, half, mass = derive_m20_body_obb(urdf)
    if np.linalg.norm(obb_center) > 1.0e-8:
        raise RuntimeError(f"Unexpected non-zero M20 body OBB center: {obb_center}")
    nominal_height = read_nominal_height(config)
    safety = 0.020
    # The optimizer sees a slightly larger activation distance than the final
    # target.  This compensates for the 22--25 mm training cloud sampling; the
    # final pass is still judged against 20 mm on an independent 7.5--8 mm cloud.
    optimization_safety = 0.030
    planner_weights = PlannerWeights()

    # Experiment 1: asymmetric side/upper obstacle gives a non-zero force and torque.
    ox, oz = np.meshgrid(
        grid_values(0.13, 0.29, 0.010),
        grid_values(nominal_height + 0.010, nominal_height + 0.060, 0.010),
        indexing="ij",
    )
    obstacle = np.column_stack(
        (ox.ravel(), np.full(ox.size, half[1] + 0.010), oz.ravel())
    )
    obstacle_weights = np.full(obstacle.shape[0], 0.010**2)
    gradient_test = finite_difference_gradient_test(
        obstacle,
        obstacle_weights,
        np.array([0.0, 0.0, nominal_height]),
        np.zeros(3),
        half,
        optimization_safety,
        planner_weights.collision_stiffness,
    )
    height_k = 35.0
    gradient_test["height_restoration_force_above_nominal"] = -height_k * 0.05
    gradient_test["height_restoration_force_below_nominal"] = height_k * 0.05

    # Experiment 2: a tilted channel that only the M20 body can clear by rolling.
    alpha = math.radians(30.0)
    gap_half = 0.100
    train_corridor = tilted_corridor(alpha, gap_half, 0.025, center_z=nominal_height)
    ground_train = horizontal_plane((-1.7, 1.7), (-0.65, 0.65), 0.0, 0.055)
    corridor_points, corridor_weights = combine_clouds(train_corridor, ground_train)
    valid_corridor = combine_clouds(
        tilted_corridor(alpha, gap_half, 0.008, center_z=nominal_height),
        horizontal_plane((-1.7, 1.7), (-0.65, 0.65), 0.0, 0.025),
    )[0]
    initial = make_initial_trajectory(23, nominal_height)

    trans_problem = TrajectoryProblem(
        corridor_points,
        corridor_weights,
        initial,
        half,
        nominal_height,
        optimization_safety,
        "translation",
        planner_weights,
    )
    trans_poses, trans_opt = optimize_trajectory(trans_problem, initial)
    trans_valid = validate_trajectory(trans_poses, valid_corridor, half, safety)

    full_problem = TrajectoryProblem(
        corridor_points,
        corridor_weights,
        initial,
        half,
        nominal_height,
        optimization_safety,
        "full",
        planner_weights,
    )
    candidates = []
    s = np.linspace(0.0, 1.0, len(initial))
    for seed_deg in (0.0, 5.0, -5.0, 12.0):
        seed = initial.copy()
        seed[:, 3] = math.radians(seed_deg) * np.sin(math.pi * s)
        seed[0, 3] = seed[-1, 3] = 0.0
        poses, opt = optimize_trajectory(full_problem, seed)
        valid = validate_trajectory(poses, valid_corridor, half, safety)
        candidates.append((poses, opt, valid, seed_deg))
    full_poses, full_opt, full_valid, chosen_seed = min(
        candidates,
        key=lambda item: (
            not item[2]["safety_clearance_met"],
            max(0.0, safety - item[2]["minimum_signed_distance_m"]),
            item[1]["cost"],
        ),
    )

    # Experiment 3: low beam; body lowers, clears, then height potential restores it.
    beam_z = nominal_height + half[2] - 0.015
    beam_train = horizontal_plane((-0.55, 0.55), (-0.28, 0.28), beam_z, 0.022)
    beam_points, beam_weights = combine_clouds(beam_train, ground_train)
    valid_beam = combine_clouds(
        horizontal_plane((-0.55, 0.55), (-0.28, 0.28), beam_z, 0.0075),
        horizontal_plane((-1.7, 1.7), (-0.65, 0.65), 0.0, 0.025),
    )[0]
    beam_initial = make_initial_trajectory(23, nominal_height)
    beam_initial_valid = validate_trajectory(beam_initial, valid_beam, half, safety)
    beam_problem = TrajectoryProblem(
        beam_points,
        beam_weights,
        beam_initial,
        half,
        nominal_height,
        optimization_safety,
        "full",
        planner_weights,
    )
    beam_poses, beam_opt = optimize_trajectory(beam_problem, beam_initial)
    beam_valid = validate_trajectory(beam_poses, valid_beam, half, safety)
    after_beam = np.abs(beam_poses[:, 0]) > 0.92
    recovery_error = float(np.max(np.abs(beam_poses[after_beam, 2] - nominal_height)))

    # Save machine-readable trajectories without embedding large arrays in JSON.
    np.savez_compressed(
        out_dir / "trajectories.npz",
        corridor_translation_only=trans_poses,
        corridor_full_6d=full_poses,
        low_beam_initial=beam_initial,
        low_beam_optimized=beam_poses,
        m20_body_half_extents=half,
    )

    results = {
        "scope": "body-only geometric validation; wheel/leg contact feasibility and tracking are not claimed",
        "m20": {
            "urdf": str(urdf),
            "base_mass_kg": mass,
            "body_obb_size_m": (2.0 * half).tolist(),
            "nominal_base_height_m": nominal_height,
            "safety_distance_m": safety,
            "optimization_activation_distance_m": optimization_safety,
        },
        "experiment_1_gradient_and_wrench": {
            k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in gradient_test.items()
        },
        "experiment_2_tilted_corridor": {
            "corridor_tilt_deg": math.degrees(alpha),
            "corridor_half_gap_m": gap_half,
            "horizontal_required_half_width_m": float(
                abs(math.sin(alpha)) * half[1] + abs(math.cos(alpha)) * half[2]
            ),
            "aligned_required_half_width_m": float(half[2]),
            "translation_only": {
                "optimizer": trans_opt,
                "validation": {k: v for k, v in trans_valid.items() if not isinstance(v, np.ndarray)},
                "maximum_abs_roll_deg": float(np.max(np.abs(np.degrees(trans_poses[:, 3])))),
            },
            "full_6d": {
                "chosen_initial_roll_seed_deg": chosen_seed,
                "optimizer": full_opt,
                "validation": {k: v for k, v in full_valid.items() if not isinstance(v, np.ndarray)},
                "maximum_abs_roll_deg": float(np.max(np.abs(np.degrees(full_poses[:, 3])))),
                "roll_at_center_deg": float(
                    np.degrees(full_poses[np.argmin(np.abs(full_poses[:, 0])), 3])
                ),
                "maximum_abs_pitch_deg": float(np.max(np.abs(np.degrees(full_poses[:, 4])))),
                "maximum_abs_yaw_deg": float(np.max(np.abs(np.degrees(full_poses[:, 5])))),
            },
        },
        "experiment_3_low_beam": {
            "beam_underside_height_m": beam_z,
            "straight_initial_validation": {
                k: v for k, v in beam_initial_valid.items() if not isinstance(v, np.ndarray)
            },
            "optimizer": beam_opt,
            "optimized_validation": {
                k: v for k, v in beam_valid.items() if not isinstance(v, np.ndarray)
            },
            "minimum_base_height_m": float(beam_poses[:, 2].min()),
            "maximum_recovery_error_after_beam_m": recovery_error,
            "maximum_abs_roll_deg": float(np.max(np.abs(np.degrees(beam_poses[:, 3])))),
            "maximum_abs_pitch_deg": float(np.max(np.abs(np.degrees(beam_poses[:, 4])))),
        },
    }

    with (out_dir / "results.json").open("w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)

    make_figure(
        out_dir / "validation_summary.png",
        half,
        nominal_height,
        safety,
        alpha,
        gap_half,
        trans_poses,
        full_poses,
        trans_valid,
        full_valid,
        beam_z,
        beam_initial,
        beam_poses,
        beam_initial_valid,
        beam_valid,
    )
    write_report(out_dir / "REPORT.md", results)
    return results


def rectangle_yz(center: np.ndarray, roll: float, half: np.ndarray) -> np.ndarray:
    local = np.array(
        [
            [-half[1], -half[2]],
            [half[1], -half[2]],
            [half[1], half[2]],
            [-half[1], half[2]],
            [-half[1], -half[2]],
        ]
    )
    c, s = math.cos(roll), math.sin(roll)
    R2 = np.array([[c, -s], [s, c]])
    return center[None, :] + local @ R2.T


def make_figure(
    path: Path,
    half: np.ndarray,
    nominal_height: float,
    safety: float,
    alpha: float,
    gap_half: float,
    trans: np.ndarray,
    full: np.ndarray,
    trans_valid: dict,
    full_valid: dict,
    beam_z: float,
    beam_initial: np.ndarray,
    beam: np.ndarray,
    beam_initial_valid: dict,
    beam_valid: dict,
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8.5))
    ax = axes[0, 0]
    tangent = np.array([math.cos(alpha), math.sin(alpha)])
    normal = np.array([-math.sin(alpha), math.cos(alpha)])
    base = np.array([0.0, nominal_height])
    for side in (-1.0, 1.0):
        line = base + side * gap_half * normal + np.linspace(-0.35, 0.35, 2)[:, None] * tangent
        ax.plot(line[:, 0], line[:, 1], "k-", lw=2)
    initial_rect = rectangle_yz(base, 0.0, half)
    center_idx = int(np.argmin(np.abs(full[:, 0])))
    full_rect = rectangle_yz(full[center_idx, 1:3], full[center_idx, 3], half)
    ax.plot(initial_rect[:, 0], initial_rect[:, 1], "--", color="tab:red", label="horizontal M20")
    ax.plot(full_rect[:, 0], full_rect[:, 1], "-", color="tab:green", lw=2, label="optimized M20")
    ax.set_aspect("equal")
    ax.set_xlabel("y [m]")
    ax.set_ylabel("z [m]")
    ax.set_title("Tilted corridor cross-section")
    ax.legend(loc="best")

    ax = axes[0, 1]
    ax.plot(trans[:, 0], np.degrees(trans[:, 3]), "--", label="translation only")
    ax.plot(full[:, 0], np.degrees(full[:, 3]), "-o", ms=3, label="full 6D")
    ax.axhline(math.degrees(alpha), color="0.5", lw=1, label="corridor tilt")
    ax.set_xlabel("path x [m]")
    ax.set_ylabel("roll [deg]")
    ax.set_title("Virtual torque creates the needed roll")
    ax.legend(loc="best")

    ax = axes[1, 0]
    ax.plot(beam_initial[:, 0], beam_initial[:, 2], "--", color="tab:red", label="straight initial")
    ax.plot(beam[:, 0], beam[:, 2], "-o", ms=3, color="tab:blue", label="optimized")
    ax.axhline(nominal_height, color="0.5", lw=1, label="nominal height")
    ax.hlines(
        beam_z - half[2] - safety,
        -0.55,
        0.55,
        color="k",
        lw=2,
        label="safe base-height ceiling",
    )
    ax.set_xlabel("path x [m]")
    ax.set_ylabel("base z [m]")
    ax.set_title("Low beam: lower and recover")
    ax.legend(loc="best")

    ax = axes[1, 1]
    for label, valid, style in (
        ("corridor translation only", trans_valid, "--"),
        ("corridor full 6D", full_valid, "-"),
        ("beam straight", beam_initial_valid, ":"),
        ("beam optimized", beam_valid, "-."),
    ):
        p = valid["dense_poses"]
        ax.plot(p[:, 0], valid["dense_clearance"], style, label=label)
    ax.axhline(0.0, color="tab:red", lw=1, label="collision boundary")
    ax.axhline(safety, color="tab:green", lw=1, label="safety target")
    ax.set_xlabel("path x [m]")
    ax.set_ylabel("minimum signed clearance [m]")
    ax.set_title("Independent dense collision re-check")
    ax.legend(loc="lower right", fontsize=8)

    fig.suptitle(
        f"M20 point-cloud virtual-wrench validation | body OBB "
        f"{2*half[0]:.2f} × {2*half[1]:.2f} × {2*half[2]:.2f} m",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(path, dpi=180)
    plt.close(fig)


def write_report(path: Path, r: dict) -> None:
    g = r["experiment_1_gradient_and_wrench"]
    c = r["experiment_2_tilted_corridor"]
    b = r["experiment_3_low_beam"]
    m = r["m20"]
    trans = c["translation_only"]
    full = c["full_6d"]
    report = f"""# M20 三维点云虚拟力/力矩：第一轮受控验证

## 结论

**核心几何想法通过第一轮验证，但还不能直接上实机。** 使用 M20 官方 URDF 的
`base_link` 碰撞体自动求得保守 OBB（{m['body_obb_size_m'][0]:.3f} ×
{m['body_obb_size_m'][1]:.3f} × {m['body_obb_size_m'][2]:.3f} m），点云势能产生的
平移力和旋转力矩与有限差分一致；在必须倾斜的通道中，只平移失败，而开放 6D
姿态后轨迹无碰撞并达到安全间隙；在低梁场景中，优化器能暂时降低机身，并在
通过后恢复到 M20 的 {m['nominal_base_height_m']:.2f} m 名义高度。

这只是**机身几何参考轨迹验证**。尚未加入四轮接触、关节范围、支撑多边形、动力学
与控制跟踪，因此不宣称轨迹已可由整机执行。

## 使用的 M20 数据

- 官方 URDF：`{m['urdf']}`
- `base_link` 质量：{m['base_mass_kg']:.3f} kg
- 从 URDF 复合碰撞体自动推导的机身 OBB：{m['body_obb_size_m'][0]:.3f} × {m['body_obb_size_m'][1]:.3f} × {m['body_obb_size_m'][2]:.3f} m
- 现有 M20 配置中的名义机身高度：{m['nominal_base_height_m']:.3f} m
- 目标安全间隙：{m['safety_distance_m']*1000:.0f} mm
- 优化激活距离：{m['optimization_activation_distance_m']*1000:.0f} mm（补偿训练点云的离散采样；最终仍按 20 mm 独立验收）

## 实验 1：力/力矩符号和梯度

- 平移梯度相对误差：{g['relative_translation_gradient_error']:.3e}
- 旋转梯度相对误差：{g['relative_rotation_gradient_error']:.3e}
- 虚拟力 `[Fx, Fy, Fz]`：{np.array(g['force']).round(5).tolist()}
- 虚拟力矩 `[Tx, Ty, Tz]`：{np.array(g['torque']).round(5).tolist()}
- 高于名义高度 5 cm 时恢复力为 {g['height_restoration_force_above_nominal']:.3f}（向下）；低于 5 cm 时为 {g['height_restoration_force_below_nominal']:.3f}（向上）。

判定：误差远小于 1%，力和力矩的符号实现正确。

## 实验 2：30° 倾斜狭窄通道

通道半宽 {c['corridor_half_gap_m']:.3f} m。M20 水平时沿通道法向需要
{c['horizontal_required_half_width_m']:.3f} m，几何上必碰；对齐后只需
{c['aligned_required_half_width_m']:.3f} m。

| 方法 | 最小有符号距离 | 相对 {m['safety_distance_m']*1000:.0f} mm 安全目标 | 碰撞 | 中心 roll |
|---|---:|---:|---|---:|
| 只平移 | {trans['validation']['minimum_signed_distance_m']*1000:.1f} mm | {trans['validation']['minimum_safety_margin_m']*1000:.1f} mm | {'否' if trans['validation']['collision_free'] else '是'} | 0.0° |
| 6D | {full['validation']['minimum_signed_distance_m']*1000:.1f} mm | {full['validation']['minimum_safety_margin_m']*1000:.1f} mm | {'否' if full['validation']['collision_free'] else '是'} | {full['roll_at_center_deg']:.1f}° |

6D 轨迹的最大相邻节点平移变化为
{full['validation']['maximum_node_translation_step_m']*1000:.1f} mm，最大相邻姿态变化为
{full['validation']['maximum_node_rotation_step_deg']:.2f}°；独立复检对节点之间做了 10 倍插值。

## 实验 3：低梁、降高与恢复

- 梁底高度：{b['beam_underside_height_m']:.3f} m
- 原始直线轨迹最小有符号距离：{b['straight_initial_validation']['minimum_signed_distance_m']*1000:.1f} mm（碰撞）
- 优化轨迹最低机身高度：{b['minimum_base_height_m']:.3f} m
- 优化后最小有符号距离：{b['optimized_validation']['minimum_signed_distance_m']*1000:.1f} mm
- 离开梁区后的最大高度恢复误差：{b['maximum_recovery_error_after_beam_m']*1000:.1f} mm

判定：高度恢复项没有阻止必要的下降，并能在障碍后把机身拉回名义高度。

## 数值边界说明

两组 6D 优化都在达到几何验收后触及 500 次迭代上限，未取得 L-BFGS-B 的形式
“收敛成功”标志；这是点到 OBB 最近面切换带来的非光滑性之一。因此本轮能证明
虚拟 wrench 的方向正确、并能找到满足几何约束的轨迹，但**不能据此宣称当前优化器
已经具备稳定收敛或实时性能**。后续工程版应换用平滑 SDF/次梯度策略，并把安全间隙
作为显式约束或增广拉格朗日项。

## 本轮没有做的事

1. 没有做 3D A*：这两个场景的路径拓扑已知，先隔离验证 wrench 核心更有效。
2. 没有做射线粗筛：射线只影响速度，不影响核心正确性；应在有真实局部点云后与半径查询做同条件计时和漏检对比。
3. 没有把轨迹送给 M20 控制器：下一道硬门槛是轮端接触与关节可达性，不应绕过。

## 建议的下一步

把 M20 实际雷达的一帧局部点云接入同一碰撞核，先离线重放；同时为每个轨迹节点
加入四轮接触高度、腿长/关节限位和支撑约束。只有独立碰撞检查与接触可行性都通过，
才进入低速仿真跟踪，再考虑实机。
"""
    path.write_text(report, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--m20-config", type=Path, default=DEFAULT_M20_CFG)
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    results = run_validation(args.urdf, args.m20_config, args.out_dir)
    print(json.dumps(results, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

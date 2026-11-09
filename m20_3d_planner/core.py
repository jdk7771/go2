#!/usr/bin/env python3
"""Controlled validation of point-cloud virtual wrench planning on M20 geometry.

The experiment deliberately validates the body-level geometric idea before wheel
contact feasibility or policy tracking.  M20's conservative body OBB is derived
from the official URDF collision primitives; it is not a hand-chosen toy box.
"""

from __future__ import annotations

import math
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .point_selection import BodyCenterSelector, PointSelection


DEFAULT_URDF = Path(
    os.environ.get(
        "M20_URDF",
        "/home/jiang/m20-repro/src/rl_training/deep_robotics_model/M20/urdf/M20.urdf",
    )
)
DEFAULT_M20_CFG = Path(
    os.environ.get(
        "M20_CONFIG",
        "/home/jiang/m20-repro/src/rl_training/source/rl_training/rl_training/tasks/"
        "manager_based/locomotion/velocity/config/wheeled/deeprobotics_m20/rough_env_cfg.py",
    )
)


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
    base = next(
        link for link in root.findall("link") if link.attrib.get("name") == "base_link"
    )
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
            world_half = (
                np.abs(axis) * half_length
                + np.sqrt(np.maximum(0.0, 1.0 - axis * axis)) * radius
            )
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
    match = re.search(
        r'base_height_l2\.params\["target_height"\]\s*=\s*([0-9.]+)', text
    )
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
    if not len(points):
        return 0.0, np.zeros(3), np.zeros(3), float("inf"), 0
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


def combine_clouds(
    *clouds: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
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
        point_selection: PointSelection | None = None,
    ) -> None:
        self.points = points
        self.point_weights = point_weights
        self.initial = initial.copy()
        self.half = half
        self.nominal_height = nominal_height
        self.safety = safety
        self.mode = mode
        self.weights = weights
        self.selector = BodyCenterSelector(points, point_selection or PointSelection())
        modes = {
            "translation": [0, 1, 2],
            "translation_yaw": [0, 1, 2, 5],
            "full": [0, 1, 2, 3, 4, 5],
        }
        if mode not in modes:
            raise ValueError(f"Unknown trajectory mode: {mode}")
        self.free_dims = np.array(modes[mode])
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
        selections = self.selector.select(poses[:, :3])
        for i, pose in enumerate(poses):
            R = rpy_matrix(pose[3:])
            indices = selections[i] if selections is not None else slice(None)
            energy, force, torque, _, _ = collision_energy_wrench(
                self.points[indices],
                self.point_weights[indices],
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
                [
                    np.dot(torque, roll_axis),
                    np.dot(torque, pitch_axis),
                    np.dot(torque, yaw_axis),
                ]
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

        total += self._add_velocity_cost(
            poses, slice(0, 3), w.velocity_translation, grad
        )
        total += self._add_acceleration_cost(
            poses, slice(0, 3), w.acceleration_translation, grad
        )
        total += self._add_velocity_cost(poses, slice(3, 6), w.velocity_rotation, grad)
        total += self._add_acceleration_cost(
            poses, slice(3, 6), w.acceleration_rotation, grad
        )
        packed_grad = grad[np.ix_(self.interior, self.free_dims)].ravel()
        return float(total), packed_grad


def densify_trajectory(poses: np.ndarray, subdivisions: int = 8) -> np.ndarray:
    dense = []
    for i in range(len(poses) - 1):
        for j in range(subdivisions):
            t = j / subdivisions
            dense.append((1.0 - t) * poses[i] + t * poses[i + 1])
    dense.append(poses[-1])
    return np.asarray(dense)


def make_initial_trajectory(n: int, nominal_height: float) -> np.ndarray:
    poses = np.zeros((n, 6))
    poses[:, 0] = np.linspace(-1.30, 1.30, n)
    poses[:, 2] = nominal_height
    return poses

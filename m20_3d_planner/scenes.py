"""Synthetic scenes and the frozen dual-lidar sampling pipeline."""

from __future__ import annotations

import math
from dataclasses import dataclass
import numpy as np
from scipy.spatial import cKDTree
from . import core
from .settings import NODES, POINT_VOXEL


@dataclass
class Scene:
    name: str
    family: str
    sensor_surface: np.ndarray
    truth_surface: np.ndarray
    params: dict


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
        [
            [sx * half[0], sy * half[1], sz * half[2]]
            for sx in (-1, 1)
            for sy in (-1, 1)
            for sz in (-1, 1)
        ],
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
    )


def extruded_floor_profile(
    profile_yz: np.ndarray,
    xlim: tuple[float, float],
    base_z: float,
    spacing: float,
) -> np.ndarray:
    """Closed solid below a continuous, piecewise-linear transverse floor.

    Sample the top along surface arc length, both entrance/exit caps, the two
    lateral faces and the bottom. The join belongs to both adjacent segments,
    so a sloped floor never silently becomes two disconnected parallel plates.
    """
    profile = np.asarray(profile_yz, dtype=float)
    if (
        len(profile) < 2
        or np.any(np.diff(profile[:, 0]) <= 0)
        or np.any(profile[:, 1] < base_z)
    ):
        raise ValueError("Floor profile needs increasing y and heights >= base_z")
    xs = core.grid_values(*xlim, spacing)
    parts = []
    for start, end in zip(profile[:-1], profile[1:]):
        t = core.grid_values(0.0, 1.0, spacing / np.linalg.norm(end - start))
        yz = start + t[:, None] * (end - start)
        xx, index = np.meshgrid(xs, np.arange(len(yz)), indexing="ij")
        parts.append(np.column_stack((xx.ravel(), yz[index.ravel()])))
    ys = core.grid_values(profile[0, 0], profile[-1, 0], spacing)
    for y, height in zip(ys, np.interp(ys, profile[:, 0], profile[:, 1])):
        zs = core.grid_values(base_z, height, spacing)
        for x in xlim:
            parts.append(
                np.column_stack((np.full(len(zs), x), np.full(len(zs), y), zs))
            )
    for y, height in profile[[0, -1]]:
        xx, zz = np.meshgrid(
            xs, core.grid_values(base_z, height, spacing), indexing="ij"
        )
        parts.append(np.column_stack((xx.ravel(), np.full(xx.size, y), zz.ravel())))
    parts.append(
        core.horizontal_plane(xlim, (profile[0, 0], profile[-1, 0]), base_z, spacing)[0]
    )
    return np.vstack(parts)


def nonparallel_scenes(nominal_height: float) -> list[Scene]:
    """Two nonparallel y-z cross-sections, extruded along travel direction x.

    Heights define synthetic fixtures, not measured terrain or planner hints.
    cross_section is display-only metadata, never passed to the optimizer.
    """
    angle = math.radians(30.0)
    roof_intercept, roof_thickness = nominal_height + 0.03, 0.08
    tilted_roof = make_box_scene(
        "flat_floor_sloped_ceiling_30deg",
        "nonparallel_cross_section",
        [
            (
                np.array(
                    [0.0, 0.0, roof_intercept + roof_thickness / (2 * math.cos(angle))]
                ),
                np.array([1.24, 1.20, roof_thickness]),
                np.array([angle, 0.0, 0.0]),
            )
        ],
        {
            "description": "下方水平，上方沿左右方向倾斜 30°；不预设机身侧倾角。",
            "roof_slope_deg": 30.0,
            "roof_intercept_m": roof_intercept,
            "default_view": "front",
            "cross_section": [
                {
                    "label": "水平地面",
                    "color": "#a46b35",
                    "yz": [[-0.78, 0.0], [0.78, 0.0]],
                },
                {
                    "label": "30° 上方斜面",
                    "color": "#7252b5",
                    "yz": [
                        [-0.48, roof_intercept - 0.48 * math.tan(angle)],
                        [0.48, roof_intercept + 0.48 * math.tan(angle)],
                    ],
                },
            ],
        },
    )
    floor_z, ceiling_z, join_y = nominal_height - 0.11, nominal_height + 0.11, -0.03
    ymin, ymax = -0.62, 0.30
    profile = np.array(
        [
            [ymin, floor_z],
            [join_y, floor_z],
            [ymax, floor_z + (ymax - join_y) * math.tan(angle)],
        ]
    )
    roof_center = np.array([0.0, (ymin + ymax) / 2, ceiling_z + 0.04])
    roof_size = np.array([1.24, ymax - ymin, 0.08])

    def surfaces(spacing, ground_spacing):
        return combine_points(
            ground(ground_spacing),
            extruded_floor_profile(profile, (-0.62, 0.62), 0.0, spacing),
            box_surface(roof_center, roof_size, np.zeros(3), spacing),
        )

    bent_floor = Scene(
        name="flat_ceiling_bent_floor_30deg",
        family="nonparallel_cross_section",
        sensor_surface=surfaces(0.016, 0.030),
        truth_surface=surfaces(0.0075, 0.022),
        params={
            "description": "上方水平；下方左段水平、右段向 +y 抬升 30°，接缝内角 150°。下方是抬升平台，入口/出口端面封闭。",
            "floor_profile_yz_m": profile.tolist(),
            "floor_slope_deg": 30.0,
            "join_interior_angle_deg": 150.0,
            "ceiling_height_m": ceiling_z,
            "x_extent_m": [-0.62, 0.62],
            "default_view": "front",
            "cross_section": [
                {
                    "label": "水平顶板",
                    "color": "#7252b5",
                    "yz": [[ymin, ceiling_z], [ymax, ceiling_z]],
                },
                {
                    "label": "水平段 + 30° 斜段",
                    "color": "#a46b35",
                    "yz": profile.tolist(),
                },
            ],
        },
    )
    return [tilted_roof, bent_floor]


def open_entry_channel(nominal_height: float) -> Scene:
    """Open approach -> narrow channel with a sloped roof and level ground."""
    angle = math.radians(30.0)
    xmin, xmax, side_y, wall_thickness = -0.35, 0.62, 0.22, 0.08
    roof_z, roof_thickness, wall_height = nominal_height - 0.02, 0.08, 0.80
    center_x, length = (xmin + xmax) / 2, xmax - xmin
    boxes = [
        (
            np.array([center_x, 0.0, roof_z + roof_thickness / (2 * math.cos(angle))]),
            np.array([length, 0.74, roof_thickness]),
            np.array([angle, 0.0, 0.0]),
        )
    ]
    for sign in (-1, 1):
        boxes.append(
            (
                np.array(
                    [center_x, sign * (side_y + wall_thickness / 2), wall_height / 2]
                ),
                np.array([length, wall_thickness, wall_height]),
                np.zeros(3),
            )
        )
    return make_box_scene(
        "open_entry_flat_floor_channel_30deg",
        "open_entry_channel",
        boxes,
        {
            "description": "从入口前空旷区域进入窄通道，再离开通道：地面始终水平，顶板沿左右方向倾斜 30°，两侧为竖直壁。统一初值 z=0.4 m、roll=0°，观察能否同时降高和侧倾。",
            "roof_slope_deg": 30.0,
            "roof_intercept_m": roof_z,
            "x_extent_m": [xmin, xmax],
            "side_inner_y_m": [-side_y, side_y],
            "side_wall_height_m": wall_height,
            "ground_height_m": 0.0,
            "default_view": "iso",
            "default_zoom": 1.0,
            "cross_section": [
                {
                    "label": "水平地面",
                    "color": "#a46b35",
                    "yz": [[-0.6, 0.0], [0.6, 0.0]],
                },
                {
                    "label": "30° 顶板下表面",
                    "color": "#7252b5",
                    "yz": [
                        [-side_y, roof_z - side_y * math.tan(angle)],
                        [side_y, roof_z + side_y * math.tan(angle)],
                    ],
                },
                {
                    "label": "竖直侧壁",
                    "color": "#617586",
                    "yz": [[-side_y, 0.0], [-side_y, wall_height]],
                },
                {
                    "label": "竖直侧壁",
                    "color": "#617586",
                    "yz": [[side_y, 0.0], [side_y, wall_height]],
                },
            ],
        },
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

            name = f"beam_h{int(bottom * 1000):03d}_yaw{int(yaw_deg):02d}_tilt{roll_deg:+.0f}"
            scenes.append(
                make_box_scene(
                    name,
                    "low_beam",
                    [(center, size, rpy)],
                    {"bottom_m": bottom, "yaw_deg": yaw_deg, "tilt_deg": roll_deg},
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
            )
        )

    # Four 3D tilted corridors.  These are extruded surfaces, but their sweep is
    # retained as the clean control for isolating roll torque.
    for angle_deg in (10.0, 20.0, 30.0, 35.0):
        angle = math.radians(angle_deg)
        sensor_wall = core.tilted_corridor(
            angle, 0.102, 0.018, center_z=nominal_height
        )[0]
        truth_wall = core.tilted_corridor(
            angle, 0.102, 0.0075, center_z=nominal_height
        )[0]
        scenes.append(
            Scene(
                name=f"tilted_corridor_{int(angle_deg):02d}deg",
                family="tilted_corridor",
                sensor_surface=combine_points(sensor_wall, ground(0.030)),
                truth_surface=combine_points(truth_wall, ground(0.022)),
                params={"corridor_angle_deg": angle_deg, "half_gap_m": 0.102},
            )
        )

    # True 3D asymmetric scene: diagonal low beam plus staggered side obstacles.
    beam_size = np.array([0.38, 0.92, 0.10])
    beam_rpy = np.radians([8.0, 5.0, 28.0])
    beam_bottom = 0.462
    beam_center = box_center_for_min_z((0.02, 0.03), beam_bottom, beam_size, beam_rpy)
    boxes = [
        (beam_center, beam_size, beam_rpy),
        (
            np.array([-0.38, 0.235, 0.30]),
            np.array([0.28, 0.19, 0.60]),
            np.radians([0, 0, -8]),
        ),
        (
            np.array([0.36, -0.235, 0.27]),
            np.array([0.30, 0.19, 0.54]),
            np.radians([0, 0, 10]),
        ),
    ]
    scenes.append(
        make_box_scene(
            "asymmetric_overhang_slalom",
            "true_3d",
            boxes,
            {"beam_bottom_m": beam_bottom, "beam_rpy_deg": [8.0, 5.0, 28.0]},
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
        (
            np.array([0.02, 0.245, 0.22]),
            np.array([0.24, 0.17, 0.44]),
            np.radians([0, 0, 15]),
        ),
    ]
    scenes.append(
        make_box_scene(
            "double_overhang_offset_pillar",
            "true_3d",
            boxes2,
            {"overhang_bottoms_m": [0.455, 0.470]},
        )
    )
    # Append to preserve the original 17 scene indices and lidar seeds.
    return (
        scenes
        + nonparallel_scenes(nominal_height)
        + [open_entry_channel(nominal_height)]
    )


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
    az_bin = np.floor((horizontal[idx] + math.radians(67.0)) / resolution).astype(
        np.int64
    )
    el_bin = np.floor((elevation[idx] + math.radians(42.0)) / resolution).astype(
        np.int64
    )
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

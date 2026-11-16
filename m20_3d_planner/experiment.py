#!/usr/bin/env python3
"""Matched-cloud trajectory experiments with straight or cloud-selected starts.

The optimizer accepts no Scene, obstacle parameters, seed hints, or truth cloud.
Every optimization finishes before the held-out surfaces are scored. Geometry
may be loaded from URDF/config or explicitly replayed from recorded metadata.
"""

from __future__ import annotations

import os

# Set before NumPy/SciPy imports, including in spawned workers. Prevent nested
# BLAS parallelism from obscuring process CPU time and oversubscribing the host.
for _thread_var in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[_thread_var] = "1"

import argparse
import csv
import hashlib
import json
import math
import platform
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
from datetime import datetime
from multiprocessing import get_context
from pathlib import Path
from types import SimpleNamespace
from zoneinfo import ZoneInfo

import matplotlib

matplotlib.use("Agg")
import numpy as np
import scipy
from scipy.optimize import minimize
from scipy.spatial import cKDTree

from . import core
from . import scenes as suite
from .settings import (
    MODES,
    LABELS,
    HERE,
    DEFAULT_RUN,
    TARGET_CLEARANCE,
    PLANNING_CLEARANCE,
    POINT_VOXEL,
    SOLVER_OPTIONS,
    planner_weights,
)
from .reporting import plot_results, write_report
from .point_selection import PointSelection
from .virtual_wrench import DynamicsConfig, relax, packed_gradient
from .ray_dynamics import RayDynamicsConfig, relax as relax_rays
from .initialization import cloud_bow_initialization, small_bend_initialization


def write_json(path: Path, value: dict | list) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def array_digest(array: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()


def projected_gradient(
    x: np.ndarray, gradient: np.ndarray, bounds: np.ndarray
) -> np.ndarray:
    """Unit-step gradient mapping x - projection_[lower,upper](x - grad)."""
    return x - np.clip(x - gradient, bounds[:, 0], bounds[:, 1])


def load_planner_weights(path: Path | None = None) -> core.PlannerWeights:
    """Merge explicit tuning overrides with the recorded baseline defaults."""
    values = asdict(planner_weights())
    if path is not None:
        overrides = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(overrides, dict) or set(overrides) - set(values):
            raise ValueError("Weights JSON must contain only known PlannerWeights fields")
        values.update(overrides)
    for name, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f"Weight {name} must be finite and nonnegative")
    return core.PlannerWeights(**values)


def load_robot(
    urdf: Path | None, config: Path | None, reference: Path
) -> tuple[np.ndarray, float, dict]:
    if (urdf is None) != (config is None):
        raise ValueError(
            "Supply both --urdf and --m20-config, or neither for metadata replay"
        )
    if urdf is not None:
        center, half, mass = core.derive_m20_body_obb(urdf)
        if np.linalg.norm(center) > 1.0e-8:
            raise ValueError(
                "The current collision kernel requires a centered body OBB"
            )
        height = core.read_nominal_height(config)
        provenance = {
            "kind": "urdf_and_config",
            "urdf": str(urdf),
            "config": str(config),
        }
    else:
        recorded = json.loads(reference.read_text(encoding="utf-8"))["metadata"]
        half = np.asarray(recorded["m20_body_obb_size_m"], dtype=float) / 2.0
        height = float(recorded["nominal_height_m"])
        mass = float(recorded["m20_base_mass_kg"])
        provenance = {
            "kind": "recorded_robot_geometry_replay",
            "reference": str(reference),
            "reference_sha256": hashlib.sha256(reference.read_bytes()).hexdigest(),
            "original_urdf": recorded["urdf"],
            "original_config": recorded["config"],
            "note": "Robot constants only; source URDF/config are not present or independently revalidated.",
        }
    provenance.update(
        body_size_m=(2 * half).tolist(), nominal_height_m=height, base_mass_kg=mass
    )
    return half, height, provenance


def optimize_from_cloud(
    cloud: np.ndarray,
    initial: np.ndarray,
    half: np.ndarray,
    nominal_height: float,
    mode: str,
    maxiter: int,
    pose_history: list[np.ndarray] | None = None,
    point_selection: PointSelection | None = None,
    lateral_limit: float = 0.16,
    solver: str = "lbfgsb",
    dynamics_config: DynamicsConfig | RayDynamicsConfig | None = None,
    dynamics_history: list[np.ndarray] | None = None,
    state_history: list[dict] | None = None,
    weights: core.PlannerWeights | None = None,
    activation_distance: float = PLANNING_CLEARANCE,
) -> tuple[np.ndarray, dict, list[dict], np.ndarray]:
    """Planning boundary: no access to scene metadata or validation surfaces."""
    planning_started = time.perf_counter()
    if solver not in ("lbfgsb", "virtual_wrench", "ray_dynamics"):
        raise ValueError(f"Unknown solver: {solver}")
    if not math.isfinite(activation_distance) or activation_distance <= 0:
        raise ValueError("activation_distance must be finite and positive")
    dynamics_config = dynamics_config or (RayDynamicsConfig() if solver == "ray_dynamics" else DynamicsConfig())
    if solver == "ray_dynamics":
        if not isinstance(dynamics_config, RayDynamicsConfig):
            raise ValueError("ray_dynamics requires RayDynamicsConfig")
        point_selection = point_selection or PointSelection("rays")
        if point_selection.method != "rays":
            raise ValueError("ray_dynamics requires --point-selection rays")
    problem = core.TrajectoryProblem(
        cloud,
        np.full(len(cloud), POINT_VOXEL**2),
        initial,
        half,
        nominal_height,
        activation_distance,
        mode,
        weights if weights is not None else planner_weights(),
        point_selection=point_selection,
        lateral_limit=lateral_limit,
    )
    bounds = np.asarray(problem.bounds())
    x0 = problem.pack(initial)
    history: list[dict] = []
    cached_x = None
    cached_value = None

    def evaluate(x):
        nonlocal cached_x, cached_value
        if cached_x is None or not np.array_equal(x, cached_x):
            cached_x = x.copy()
            cached_value = problem.cost_grad(x)
        return cached_value

    def record(x):
        cost, grad = evaluate(x)
        poses = problem.unpack(x)
        record_poses(poses, cost, grad)

    def record_poses(poses, cost, grad, extra=None):
        if pose_history is not None:
            pose_history.append(poses.copy())
        history.append(
            {
                "iteration": len(history),
                "cost": float(cost),
                "gradient_inf_norm": float(np.linalg.norm(grad, ord=np.inf)),
                "projected_gradient_inf_norm": float(
                    np.linalg.norm(projected_gradient(problem.pack(poses), grad, bounds), ord=np.inf)
                ),
                "minimum_base_z_m": float(poses[:, 2].min()),
                "max_abs_roll_deg": float(np.abs(np.degrees(poses[:, 3])).max()),
                "max_abs_pitch_deg": float(np.abs(np.degrees(poses[:, 4])).max()),
                "max_abs_yaw_deg": float(np.abs(np.degrees(poses[:, 5])).max()),
                **(extra or {}),
            }
        )

    initial_wrenches = []
    initial_selections = problem.selector.select(initial[:, :3])
    for i, pose in enumerate(initial):
        indices = (
            initial_selections[i] if initial_selections is not None else slice(None)
        )
        _, force, torque, _, _ = core.collision_energy_wrench(
            cloud[indices],
            problem.point_weights[indices],
            pose[:3],
            core.rpy_matrix(pose[3:]),
            half,
            problem.safety,
            problem.weights.collision_stiffness,
        )
        initial_wrenches.append(np.r_[force, torque])

    wall_start, cpu_start = time.perf_counter(), time.process_time()
    dynamics_summary = {}
    last_dynamic_gradient = None
    if solver == "lbfgsb":
        record(x0)
        result = minimize(
            evaluate, x0, method="L-BFGS-B", jac=True,
            bounds=problem.bounds(), callback=record,
            options={"maxiter": maxiter, **SOLVER_OPTIONS},
        )
    elif solver == "virtual_wrench":
        def record_dynamics(poses, force, torque, velocity, omega, extra, cost):
            nonlocal last_dynamic_gradient
            gradient = packed_gradient(problem, poses, force, torque)
            last_dynamic_gradient = gradient
            record_poses(poses, cost, gradient, extra)
            if dynamics_history is not None:
                dynamics_history.append(np.column_stack((force, torque, velocity, omega)))

        poses, dynamics_summary = relax(problem, maxiter, dynamics_config, record_dynamics)
        result = SimpleNamespace(
            x=problem.pack(poses), fun=history[-1]["cost"],
            jac=last_dynamic_gradient,
            success=dynamics_summary["formal_success"], message=dynamics_summary["message"],
            nit=dynamics_summary["iterations"], nfev=dynamics_summary["evaluations"],
        )
    else:
        def record_ray_state(state, force, torque, linear, angular, total_force, total_torque_body, extra, cost):
            nonlocal last_dynamic_gradient
            poses = state.poses()
            gradient = packed_gradient(problem, poses, force, torque)
            last_dynamic_gradient = gradient
            record_poses(poses, cost, gradient, extra)
            total_torque_world = np.einsum("nij,nj->ni", state.rotations, total_torque_body)
            if dynamics_history is not None:
                dynamics_history.append(np.column_stack((total_force, total_torque_world, state.velocity_world, state.omega_body)))
            if state_history is not None:
                state_history.append({
                    "transforms": state.transforms.copy(),
                    "accelerations": np.column_stack((linear, angular)),
                    "elastic_wrenches": np.column_stack((force, torque)),
                    "torques_body": total_torque_body.copy(),
                })

        state, dynamics_summary = relax_rays(problem, maxiter, dynamics_config, record_ray_state)
        poses = state.poses()
        result = SimpleNamespace(
            x=problem.pack(poses), fun=history[-1]["cost"],
            jac=last_dynamic_gradient,
            success=dynamics_summary["formal_success"], message=dynamics_summary["message"],
            nit=dynamics_summary["iterations"], nfev=dynamics_summary["evaluations"],
        )
    cpu_seconds = time.process_time() - cpu_start
    wall_seconds = time.perf_counter() - wall_start
    if history[-1]["cost"] != float(result.fun):
        record(result.x)
    tail_start = max(0, len(history) - 51)
    tail = history[tail_start:]
    tail_drop = tail[0]["cost"] - tail[-1]["cost"]
    optimizer = {
        "solver": solver,
        "formal_success": bool(result.success),
        "message": str(result.message),
        "iterations": int(result.nit),
        "evaluations": int(result.nfev),
        "initial_cost": history[0]["cost"],
        "final_cost": float(result.fun),
        "gradient_inf_norm": float(np.linalg.norm(result.jac, ord=np.inf)),
        "projected_gradient_inf_norm": float(
            np.linalg.norm(projected_gradient(result.x, result.jac, bounds), ord=np.inf)
        ),
        "tail_iteration_count": len(tail) - 1,
        "tail_cost_drop": float(tail_drop),
        "tail_relative_cost_drop": float(tail_drop / max(1.0, abs(tail[0]["cost"]))),
        "wall_seconds": wall_seconds,
        "cpu_seconds": cpu_seconds,
        "total_planning_seconds": time.perf_counter() - planning_started,
        "point_selection": asdict(problem.selector.config),
        "lateral_limit_m": lateral_limit,
        **problem.selector.statistics(),
        "mode": mode,
        "free_dimensions": problem.free_dims.tolist(),
        "weights": asdict(problem.weights),
        "activation_distance_m": problem.safety,
    }
    if solver in ("virtual_wrench", "ray_dynamics"):
        optimizer.update(
            dynamics_config=asdict(dynamics_config),
            virtual_inertia_body=dynamics_config.inertia(half).tolist(),
            rejected_steps=dynamics_summary["rejected_steps"],
            relaxation_time=dynamics_summary["relaxation_time"],
            final_kinetic_energy=history[-1]["kinetic_energy"],
        )
    if solver == "ray_dynamics":
        optimizer.update(
            algorithm_version=("ray_newton_euler_v2_motion_limited" if dynamics_config.max_translation_step is not None or dynamics_config.max_rotation_step is not None else "ray_newton_euler_v1"),
            node_state="T_SE3,v_world,omega_body",
            integration="semi_implicit_euler_SO3_exponential",
            rotational_springs="SO3_log_attitude_relative_rotation_and_bending",
            acceleration_frames="linear_world,angular_body; excludes constraint impulses",
            wrench_frames="net_force_world,net_torque_world; includes damping",
            motion_limited_retries=dynamics_summary["motion_limited_retries"],
            energy_rejected_steps=dynamics_summary["energy_rejected_steps"],
            maximum_node_translation_step=max(h["maximum_node_translation_step"] for h in history),
            maximum_node_rotation_step=max(h["maximum_node_rotation_step"] for h in history),
            maximum_proposed_translation_step=max(h["maximum_proposed_translation_step"] for h in history),
            maximum_proposed_rotation_step=max(h["maximum_proposed_rotation_step"] for h in history),
        )
    return problem.unpack(result.x), optimizer, history, np.asarray(initial_wrenches)


def plan_cloud(task: tuple) -> dict:
    (
        name, cloud, initial, half, height, maxiter, output, modes,
        point_selection, lateral_limit, solver, dynamics_config, weights, activation_distance,
    ) = task
    trial_dir = Path(output) / "trials"
    summaries = []
    for mode in modes:
        pose_history = []
        dynamics_history = []
        state_history = []
        trajectory, optimizer, history, initial_wrench = optimize_from_cloud(
            cloud,
            initial,
            half,
            height,
            mode,
            maxiter,
            pose_history=pose_history,
            point_selection=point_selection,
            lateral_limit=lateral_limit,
            solver=solver, dynamics_config=dynamics_config,
            dynamics_history=dynamics_history,
            state_history=state_history,
            weights=weights,
            activation_distance=activation_distance,
        )
        prefix = trial_dir / f"{name}__{mode}"
        np.savez_compressed(
            prefix.with_suffix(".npz"),
            optimized_trajectory=trajectory,
            initial_trajectory=initial,
            initial_collision_wrench=initial_wrench,
            iteration_poses=np.asarray(pose_history),
            iteration_costs=np.array([h["cost"] for h in history]),
            **({
                "iteration_wrenches": np.asarray(dynamics_history)[:, :, :6],
                "iteration_twists": np.asarray(dynamics_history)[:, :, 6:],
                "iteration_times": np.array([h["relaxation_time"] for h in history]),
                "iteration_kinetic_energy": np.array([h["kinetic_energy"] for h in history]),
            } if dynamics_history else {}),
            **({
                "iteration_transforms": np.asarray([s["transforms"] for s in state_history]),
                "iteration_accelerations": np.asarray([s["accelerations"] for s in state_history]),
                "iteration_elastic_wrenches": np.asarray([s["elastic_wrenches"] for s in state_history]),
                "iteration_torques_body": np.asarray([s["torques_body"] for s in state_history]),
                "iteration_time_steps": np.array([h["time_step"] for h in history]),
                "iteration_scheduled_time_steps": np.array([h["scheduled_time_step"] for h in history]),
                "iteration_motion_steps": np.array([[h["maximum_node_translation_step"], h["maximum_node_rotation_step"]] for h in history]),
                "iteration_proposed_motion_steps": np.array([[h["maximum_proposed_translation_step"], h["maximum_proposed_rotation_step"]] for h in history]),
            } if state_history else {}),
        )
        with prefix.with_suffix(".history.csv").open(
            "w", newline="", encoding="utf-8"
        ) as f:
            writer = csv.DictWriter(f, fieldnames=list(history[0]))
            writer.writeheader()
            writer.writerows(history)
        summary = {
            "scene": name,
            "mode": mode,
            "optimizer": optimizer,
            "initial_sha256": array_digest(initial),
            "cloud_sha256": array_digest(cloud),
        }
        write_json(prefix.with_suffix(".json"), summary)
        summaries.append(summary)
        print(
            f"  planned {name} / {mode}: {optimizer['iterations']} iterations, "
            f"{optimizer['wall_seconds']:.1f}s, pg={optimizer['projected_gradient_inf_norm']:.3g}",
            flush=True,
        )
    return {"scene": name, "trials": summaries}


def dense_validation(
    poses: np.ndarray, truth: np.ndarray, half: np.ndarray, subdivisions: int,
    rotation_interpolation: str = "euler",
    surface_cover_radius_m: float | None = None,
    surface_kind: str = "synthetic",
) -> dict:
    dense = core.densify_trajectory(poses, subdivisions, rotation_interpolation)
    # Exact pruning: any point farther than body circumradius + a known SDF
    # upper bound cannot improve that bound. This keeps large mesh scans cheap
    # without weakening the full-surface check or reusing planner ray hits.
    tree = cKDTree(truth)
    _, nearest = tree.query(dense[:, :3])
    radius = np.linalg.norm(half)
    distances = []
    for pose, index in zip(dense, nearest):
        rotation = core.rpy_matrix(pose[3:])
        upper = core.obb_signed_distance(truth[[index]], pose[:3], rotation, half)[0][0]
        indices = tree.query_ball_point(pose[:3], radius + max(0., upper) + 1e-10)
        distances.append(core.obb_signed_distance(truth[indices], pose[:3], rotation, half)[0].min())
    distances = np.array(distances)
    # Bound motion of any body point within each interval. Euler linear
    # interpolation rotates by at most sum(|delta_rpy|), independently of axis.
    motion_bound = np.linalg.norm(
        np.diff(dense[:, :3], axis=0), axis=1
    ) + np.linalg.norm(half) * np.abs(np.diff(dense[:, 3:], axis=0)).sum(axis=1)
    if rotation_interpolation == "so3":
        from scipy.spatial.transform import Rotation
        rotations = Rotation.from_euler("xyz", dense[:, 3:]).as_matrix()
        angular_distance = np.linalg.norm(Rotation.from_matrix(
            rotations[:-1].transpose(0, 2, 1) @ rotations[1:]
        ).as_rotvec(), axis=1)
        motion_bound = np.linalg.norm(np.diff(dense[:, :3], axis=0), axis=1) + np.linalg.norm(half) * angular_distance
    time_lower = float(
        (np.minimum(distances[:-1], distances[1:]) - motion_bound / 2).min()
    )
    # Every synthetic face is a rectangular grid with spacing <= 22 mm;
    # sqrt(2)/2 * spacing bounds the nearest-sample covering radius. This is
    # deliberately conservative (the obstacle grids themselves are <= 7.5 mm).
    cover_radius = math.sqrt(2) * 0.022 / 2 if surface_cover_radius_m is None else surface_cover_radius_m
    if not math.isfinite(cover_radius) or cover_radius < 0:
        raise ValueError("Surface covering radius must be finite and nonnegative")
    surface_lower = time_lower - cover_radius
    return {
        "minimum_signed_distance_m": float(distances.min()),
        "collision_free_strict": bool(distances.min() >= 0.0),
        "clearance_20mm_strict": bool(distances.min() >= TARGET_CLEARANCE),
        "legacy_collision_free_with_0p5mm_tolerance": bool(distances.min() >= -0.0005),
        "legacy_clearance_with_1p5mm_tolerance": bool(
            distances.min() >= TARGET_CLEARANCE - 0.0015
        ),
        "interpolation_subdivisions": subdivisions,
        "rotation_interpolation": rotation_interpolation,
        "checked_pose_count": len(dense),
        "continuous_lower_bound_against_truth_points_m": time_lower,
        "synthetic_surface_cover_radius_m": cover_radius,
        "surface_cover_radius_m": cover_radius,
        "surface_kind": surface_kind,
        "continuous_lower_bound_against_surfaces_m": surface_lower,
        "continuous_lower_bound_against_synthetic_surfaces_m": surface_lower,
        "conservative_continuous_20mm_certificate": bool(
            surface_lower >= TARGET_CLEARANCE
        ),
        "dense_poses": dense,
        "dense_clearance": distances,
    }


def run(args) -> dict:
    output = args.output.resolve()
    if (output / "protocol.json").exists():
        raise FileExistsError(f"Existing experiment at {output}; choose a new --output")
    output.mkdir(parents=True, exist_ok=True)
    for name in ("trials", "scenes", "source"):
        (output / name).mkdir(exist_ok=True)
    half, height, robot = load_robot(args.urdf, args.m20_config, args.reference_results)
    initial = suite.initial_trajectory(height)
    if args.initialization == "small_bend":
        initial = small_bend_initialization(initial)
    modes = tuple(args.modes)
    weights = getattr(args, "weights_config", None) or planner_weights()
    point_selection = PointSelection(
        args.point_selection, args.max_length, args.ray_angle_deg
    )
    scenes = suite.build_scenes(height, half)
    if args.suite in ("open_worlds", "mixed"):
        from .open_worlds import load_prepared_scenes
        imported = load_prepared_scenes()
        scenes = imported if args.suite == "open_worlds" else scenes + imported
    scene_seeds = {scene.name: 1000 + i for i, scene in enumerate(scenes)}
    selected_names = getattr(args, "scenes", None)
    if selected_names:
        unknown = set(selected_names) - scene_seeds.keys()
        if unknown:
            raise ValueError(f"Unknown scenes: {', '.join(sorted(unknown))}")
        scenes = [scene for scene in scenes if scene.name in selected_names]
    metadata = {
        "started_at": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
        "robot": robot,
        "initialization": {"cloud_bow": "cloud_scored_smooth_bows_zero_speed",
                           "small_bend": "unoptimized_small_bend_zero_speed",
                           "straight": "uniform_straight_zero_angles"}[args.initialization],
        "initial_sha256": None if args.initialization == "cloud_bow" else array_digest(initial),
        "straight_reference_sha256": array_digest(suite.initial_trajectory(height)),
        "suite": args.suite,
        "open_world_task_selection": "fixed from source geometry and endpoint/collision checks, before solving" if args.suite != "synthetic" else None,
        "initialization_details": {},
        "modes": list(modes),
        "point_selection": asdict(point_selection),
        "lateral_limit_m": args.lateral_limit,
        "solver": args.solver,
        "experiment_label": getattr(args, "label", None),
        "tuning_note": getattr(args, "tuning_note", None),
        "dynamics_config": asdict(args.dynamics_config) if args.solver in ("virtual_wrench", "ray_dynamics") else None,
        "rotation_interpolation": "so3" if args.solver == "ray_dynamics" else "euler",
        "maxiter": args.maxiter,
        "workers": args.workers,
        **SOLVER_OPTIONS,
        "target_clearance_m": TARGET_CLEARANCE,
        "activation_distance_m": args.activation_distance,
        "weights": asdict(weights),
        "validation_subdivisions": args.validation_subdivisions,
        "python": sys.version,
        "numpy": np.__version__,
        "scipy": scipy.__version__,
        "matplotlib": matplotlib.__version__,
        "platform": platform.platform(),
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "core_sha256": hashlib.sha256(Path(core.__file__).read_bytes()).hexdigest(),
        "suite_sha256": hashlib.sha256(Path(suite.__file__).read_bytes()).hexdigest(),
        "truth_used_for_planning": False,
        "scene_hints_used_for_initialization": False,
        "scene_seeds": {scene.name: scene_seeds[scene.name] for scene in scenes},
        "scenes": {
            scene.name: {"family": scene.family, "params": scene.params}
            for scene in scenes
        },
    }
    metadata["source_files"] = {}
    for name in (
        "core.py",
        "point_selection.py",
        "scenes.py",
        "settings.py",
        "experiment.py",
        "virtual_wrench.py",
        "ray_dynamics.py",
        "initialization.py",
        "open_worlds.py",
    ):
        content = (HERE / name).read_bytes()
        metadata["source_files"][name] = hashlib.sha256(content).hexdigest()
        (output / "source" / name).write_bytes(content)
    write_json(output / "protocol.json", metadata)
    tasks = []
    scene_initials = {}
    for scene in scenes:
        cloud, stats = suite.simulate_m20_lidar_cloud(
            scene.sensor_surface, height, scene_seeds[scene.name]
        )
        if args.initialization == "cloud_bow":
            scene_initial, init_info = cloud_bow_initialization(
                cloud, initial, half, args.lateral_limit, min(args.activation_distance, TARGET_CLEARANCE),
                allow_roll="full" in modes,
            )
            metadata["initialization_details"][scene.name] = init_info
            selected = init_info["selected"]
            print(f"Initialized {scene.name}: lateral={selected['lateral_amplitude_m']:+.2f}m, "
                  f"height={selected['vertical_amplitude_m']:+.2f}m, "
                  f"roll={selected['roll_amplitude_deg']:+.0f}deg, "
                  f"{init_info['candidate_count']} candidates / {init_info['seconds']:.2f}s", flush=True)
        else:
            scene_initial = initial.copy()
            if args.initialization == "small_bend":
                metadata["initialization_details"][scene.name] = dict(
                    method="unoptimized_small_bend", amplitude_bound_m=.025,
                    measured_lateral_amplitude_m=float(np.abs(scene_initial[:, 1]).max()),
                    seconds=0., cloud_used=False, candidate_search=False,
                    reference_role="initial small bend anchors XY and attitude restoration")
        scene_initials[scene.name] = scene_initial
        np.savez_compressed(
            output / "scenes" / f"{scene.name}.npz",
            planning_point_cloud=cloud,
            dense_truth_surface=scene.truth_surface,
            initial_trajectory=scene_initial,
            straight_reference_trajectory=suite.initial_trajectory(height),
            body_half_extents=half,
        )
        write_json(
            output / "scenes" / f"{scene.name}.json",
            {
                "cloud_stats": stats,
                "cloud_sha256": array_digest(cloud),
                "family": scene.family,
                "params": scene.params,
                "initial_sha256": array_digest(scene_initial),
                "initialization": metadata["initialization_details"].get(scene.name),
            },
        )
        # Workers get neither Scene nor truth; no hint generation is called.
        tasks.append(
            (
                scene.name,
                cloud,
                scene_initial,
                half,
                height,
                args.maxiter,
                str(output),
                modes,
                point_selection,
                args.lateral_limit,
                args.solver, args.dynamics_config, weights, args.activation_distance,
            )
        )
    write_json(output / "protocol.json", metadata)
    print(
        f"Planning {len(tasks)} scenes × {len(modes)} modes, selection={point_selection}; scoring starts after all planning.",
        flush=True,
    )
    all_plans = []
    with ProcessPoolExecutor(
        max_workers=args.workers, mp_context=get_context("spawn")
    ) as executor:
        pending = [executor.submit(plan_cloud, task) for task in tasks]
        for future in as_completed(pending):
            planned = future.result()
            all_plans.extend(planned["trials"])
            print(f"Finished planning {planned['scene']}", flush=True)
    write_json(output / "source" / "planning_complete.json", all_plans)
    print("All planning finished. Scoring held-out dense surfaces.", flush=True)
    indexed = {(r["scene"], r["mode"]): r for r in all_plans}
    rows = []
    for scene in scenes:
        baseline = dense_validation(
            scene_initials[scene.name], scene.truth_surface, half, args.validation_subdivisions,
            rotation_interpolation=metadata["rotation_interpolation"],
            surface_cover_radius_m=scene.params.get("truth_surface_cover_radius_m"),
            surface_kind=scene.params.get("surface_kind", "synthetic"),
        )
        for mode in modes:
            prefix = output / "trials" / f"{scene.name}__{mode}"
            with np.load(prefix.with_suffix(".npz")) as data:
                trajectory = data["optimized_trajectory"]
            valid = dense_validation(
                trajectory, scene.truth_surface, half, args.validation_subdivisions,
                rotation_interpolation=metadata["rotation_interpolation"],
                surface_cover_radius_m=scene.params.get("truth_surface_cover_radius_m"),
                surface_kind=scene.params.get("surface_kind", "synthetic"),
            )
            np.savez_compressed(
                prefix.with_suffix(".validation.npz"),
                dense_poses=valid.pop("dense_poses"),
                dense_clearance=valid.pop("dense_clearance"),
            )
            full = indexed[scene.name, mode]
            center = int(np.argmin(np.abs(trajectory[:, 0])))
            row = {
                "scene": scene.name,
                "family": scene.family,
                "mode": mode,
                "baseline_min_distance_m": baseline["minimum_signed_distance_m"],
                "initial_path_has_collision": baseline["minimum_signed_distance_m"] < 0,
                "initial_endpoint_minimum_distance_m": float(baseline["dense_clearance"][[0, -1]].min()),
                **valid,
                **full["optimizer"],
                "initialization_seconds": metadata["initialization_details"].get(scene.name, {}).get("seconds", 0.),
                "total_planning_with_initialization_seconds": full["optimizer"]["total_planning_seconds"]
                    + metadata["initialization_details"].get(scene.name, {}).get("seconds", 0.),
                "minimum_base_z_m": float(trajectory[:, 2].min()),
                "maximum_abs_y_m": float(np.abs(trajectory[:, 1]).max()),
                "max_abs_roll_deg": float(np.abs(np.degrees(trajectory[:, 3])).max()),
                "max_abs_pitch_deg": float(np.abs(np.degrees(trajectory[:, 4])).max()),
                "max_abs_yaw_deg": float(np.abs(np.degrees(trajectory[:, 5])).max()),
                "roll_at_center_deg": float(np.degrees(trajectory[center, 3])),
                "exit_height_recovery_max_error_m": float(
                    np.abs(trajectory[trajectory[:, 0] > 0.92, 2] - height).max()
                ),
                "cloud_sha256": full["cloud_sha256"],
                "initial_sha256": full["initial_sha256"],
            }
            rows.append(row)
        print(f"  scored {scene.name}", flush=True)
    metadata["finished_at"] = datetime.now(ZoneInfo("Asia/Shanghai")).isoformat()
    result = {"metadata": metadata, "trials": rows}
    write_json(output / "results.json", result)
    with (output / "results.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    plot_results(output, rows, scenes, half, height)
    write_report(output, metadata, rows)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--urdf", type=Path)
    parser.add_argument("--m20-config", type=Path)
    parser.add_argument("--reference-results", type=Path, default=HERE / "robot.json")
    parser.add_argument("--weights-json", type=Path,
                        help="Explicit PlannerWeights overrides; merged with baseline settings and recorded")
    parser.add_argument("--label", help="Viewer label for this experiment")
    parser.add_argument("--suite", choices=("synthetic", "open_worlds", "mixed"), default="synthetic")
    parser.add_argument("--initialization", choices=("straight", "small_bend", "cloud_bow"), default=None,
                        help="Default: small_bend for ray_dynamics; straight for historical solvers. "
                             "Selected curve is also the path restoration reference.")
    parser.add_argument("--tuning-note", help="Record whether these scenes were used for parameter tuning")
    parser.add_argument("--activation-distance", type=float, default=PLANNING_CLEARANCE,
                        help="Point-to-body collision activation distance [m], not the safety certificate threshold")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--solver", choices=("lbfgsb", "virtual_wrench", "ray_dynamics"), default="lbfgsb")
    parser.add_argument("--maxiter", type=int, default=None,
                        help="Default: 300 optimizer iterations / 4000 virtual dynamics steps")
    parser.add_argument("--wrench-dt", type=float, default=0.01)
    parser.add_argument("--wrench-final-dt", type=float, default=None,
                        help="ray_dynamics: fine time step (default min(0.001, initial dt))")
    parser.add_argument("--wrench-schedule-steps", type=int, default=1000,
                        help="ray_dynamics: geometric coarse-to-fine schedule duration")
    parser.add_argument("--wrench-settle-steps", type=int, default=5,
                        help="ray_dynamics: consecutive equilibrium/speed checks")
    parser.add_argument("--max-node-translation-step", type=float, default=.01,
                        help="ray_dynamics: maximum per-node displacement per integration step [m]")
    parser.add_argument("--max-node-rotation-step-deg", type=float, default=2.,
                        help="ray_dynamics: maximum per-node rotation per integration step [degrees]")
    parser.add_argument("--wrench-dt-growth", type=float, default=1.05,
                        help="ray_dynamics: accepted-step recovery factor, bounded by scheduled dt (1 disables recovery)")
    parser.add_argument("--virtual-mass", type=float, default=1.0)
    parser.add_argument("--linear-damping", type=float, default=40.0)
    parser.add_argument("--angular-damping", type=float, default=2.0)
    parser.add_argument(
        "--lateral-limit", type=float, default=0.16,
        help="Symmetric body-center y bound [m]; use 0.6 for the tall-obstacle experiment",
    )
    parser.add_argument("--validation-subdivisions", type=int, default=50)
    parser.add_argument("--modes", nargs="+", choices=MODES, default=list(MODES))
    parser.add_argument(
        "--point-selection", choices=("all", "radius", "rays"), default=None,
        help="Default: rays for ray_dynamics; all for historical solvers",
    )
    parser.add_argument(
        "--max-length", type=float, default=0.6, help="Range from body CENTER [m]"
    )
    parser.add_argument(
        "--ray-angle-deg", type=float, default=4.0, help="Angular bin width [deg]"
    )
    parser.add_argument(
        "--scenes",
        nargs="+",
        help="Only run these scene IDs; keep their original noise seeds",
    )
    args = parser.parse_args(argv)
    if args.initialization is None:
        args.initialization = "small_bend" if args.solver == "ray_dynamics" else "straight"
    try:
        args.weights_config = load_planner_weights(args.weights_json)
    except (OSError, ValueError, TypeError) as error:
        parser.error(str(error))
    if args.maxiter is None:
        args.maxiter = 4000 if args.solver in ("virtual_wrench", "ray_dynamics") else 300
    if args.point_selection is None:
        args.point_selection = "rays" if args.solver == "ray_dynamics" else "all"
    if args.solver == "ray_dynamics" and args.point_selection != "rays":
        parser.error("ray_dynamics requires --point-selection rays")
    try:
        config_type = RayDynamicsConfig if args.solver == "ray_dynamics" else DynamicsConfig
        extra = ({
            "final_time_step": args.wrench_final_dt if args.wrench_final_dt is not None else min(.001, args.wrench_dt),
            "schedule_steps": args.wrench_schedule_steps,
            "settle_steps": args.wrench_settle_steps,
            "max_translation_step": args.max_node_translation_step,
            "max_rotation_step": math.radians(args.max_node_rotation_step_deg),
            "time_step_growth": args.wrench_dt_growth,
        } if args.solver == "ray_dynamics" else {})
        args.dynamics_config = config_type(
            time_step=args.wrench_dt, mass=args.virtual_mass,
            linear_damping=args.linear_damping, angular_damping=args.angular_damping,
            **extra,
        )
    except ValueError as error:
        parser.error(str(error))
    if min(args.workers, args.maxiter, args.validation_subdivisions) < 1:
        parser.error("workers, maxiter and validation-subdivisions must be positive")
    if not math.isfinite(args.lateral_limit) or args.lateral_limit <= 0:
        parser.error("lateral-limit must be finite and positive")
    if not math.isfinite(args.activation_distance) or args.activation_distance <= 0:
        parser.error("activation-distance must be finite and positive")
    try:
        PointSelection(args.point_selection, args.max_length, args.ray_angle_deg)
    except ValueError as error:
        parser.error(str(error))
    result = run(args)
    for mode in args.modes:
        rows = [row for row in result["trials"] if row["mode"] == mode]
        print(
            f"{LABELS[mode]}: collision-free {sum(r['collision_free_strict'] for r in rows)}/{len(rows)}, "
            f"20mm {sum(r['clearance_20mm_strict'] for r in rows)}/{len(rows)}"
        )


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Uniform-initialization, matched-cloud 3/4/6 DOF ablation.

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
from zoneinfo import ZoneInfo

import matplotlib

matplotlib.use("Agg")
import numpy as np
import scipy
from scipy.optimize import minimize

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
) -> tuple[np.ndarray, dict, list[dict], np.ndarray]:
    """Planning boundary: no access to scene metadata or validation surfaces."""
    planning_started = time.perf_counter()
    problem = core.TrajectoryProblem(
        cloud,
        np.full(len(cloud), POINT_VOXEL**2),
        initial,
        half,
        nominal_height,
        PLANNING_CLEARANCE,
        mode,
        planner_weights(),
        point_selection=point_selection,
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
        if pose_history is not None:
            pose_history.append(poses.copy())
        history.append(
            {
                "iteration": len(history),
                "cost": float(cost),
                "gradient_inf_norm": float(np.linalg.norm(grad, ord=np.inf)),
                "projected_gradient_inf_norm": float(
                    np.linalg.norm(projected_gradient(x, grad, bounds), ord=np.inf)
                ),
                "minimum_base_z_m": float(poses[:, 2].min()),
                "max_abs_roll_deg": float(np.abs(np.degrees(poses[:, 3])).max()),
                "max_abs_pitch_deg": float(np.abs(np.degrees(poses[:, 4])).max()),
                "max_abs_yaw_deg": float(np.abs(np.degrees(poses[:, 5])).max()),
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
            PLANNING_CLEARANCE,
            problem.weights.collision_stiffness,
        )
        initial_wrenches.append(np.r_[force, torque])

    wall_start, cpu_start = time.perf_counter(), time.process_time()
    record(x0)
    result = minimize(
        evaluate,
        x0,
        method="L-BFGS-B",
        jac=True,
        bounds=problem.bounds(),
        callback=record,
        options={"maxiter": maxiter, **SOLVER_OPTIONS},
    )
    cpu_seconds = time.process_time() - cpu_start
    wall_seconds = time.perf_counter() - wall_start
    if history[-1]["cost"] != float(result.fun):
        record(result.x)
    tail_start = max(0, len(history) - 51)
    tail = history[tail_start:]
    tail_drop = tail[0]["cost"] - tail[-1]["cost"]
    optimizer = {
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
        **problem.selector.statistics(),
        "mode": mode,
        "free_dimensions": problem.free_dims.tolist(),
    }
    return problem.unpack(result.x), optimizer, history, np.asarray(initial_wrenches)


def plan_cloud(task: tuple) -> dict:
    name, cloud, initial, half, height, maxiter, output, modes, point_selection = task
    trial_dir = Path(output) / "trials"
    summaries = []
    for mode in modes:
        pose_history = []
        trajectory, optimizer, history, initial_wrench = optimize_from_cloud(
            cloud,
            initial,
            half,
            height,
            mode,
            maxiter,
            pose_history=pose_history,
            point_selection=point_selection,
        )
        prefix = trial_dir / f"{name}__{mode}"
        np.savez_compressed(
            prefix.with_suffix(".npz"),
            optimized_trajectory=trajectory,
            initial_trajectory=initial,
            initial_collision_wrench=initial_wrench,
            iteration_poses=np.asarray(pose_history),
            iteration_costs=np.array([h["cost"] for h in history]),
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
    poses: np.ndarray, truth: np.ndarray, half: np.ndarray, subdivisions: int
) -> dict:
    dense = core.densify_trajectory(poses, subdivisions)
    distances = np.array(
        [
            core.obb_signed_distance(truth, pose[:3], core.rpy_matrix(pose[3:]), half)[
                0
            ].min()
            for pose in dense
        ]
    )
    # Bound motion of any body point within each interval. Euler linear
    # interpolation rotates by at most sum(|delta_rpy|), independently of axis.
    motion_bound = np.linalg.norm(
        np.diff(dense[:, :3], axis=0), axis=1
    ) + np.linalg.norm(half) * np.abs(np.diff(dense[:, 3:], axis=0)).sum(axis=1)
    time_lower = float(
        (np.minimum(distances[:-1], distances[1:]) - motion_bound / 2).min()
    )
    # Every synthetic face is a rectangular grid with spacing <= 22 mm;
    # sqrt(2)/2 * spacing bounds the nearest-sample covering radius. This is
    # deliberately conservative (the obstacle grids themselves are <= 7.5 mm).
    cover_radius = math.sqrt(2) * 0.022 / 2
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
        "checked_pose_count": len(dense),
        "continuous_lower_bound_against_truth_points_m": time_lower,
        "synthetic_surface_cover_radius_m": cover_radius,
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
    modes = tuple(args.modes)
    point_selection = PointSelection(
        args.point_selection, args.max_length, args.ray_angle_deg
    )
    scenes = suite.build_scenes(height, half)
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
        "initialization": "uniform_straight_zero_angles",
        "initial_sha256": array_digest(initial),
        "modes": list(modes),
        "point_selection": asdict(point_selection),
        "maxiter": args.maxiter,
        "workers": args.workers,
        **SOLVER_OPTIONS,
        "target_clearance_m": TARGET_CLEARANCE,
        "activation_distance_m": PLANNING_CLEARANCE,
        "weights": asdict(planner_weights()),
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
    ):
        content = (HERE / name).read_bytes()
        metadata["source_files"][name] = hashlib.sha256(content).hexdigest()
        (output / "source" / name).write_bytes(content)
    write_json(output / "protocol.json", metadata)
    tasks = []
    for scene in scenes:
        cloud, stats = suite.simulate_m20_lidar_cloud(
            scene.sensor_surface, height, scene_seeds[scene.name]
        )
        np.savez_compressed(
            output / "scenes" / f"{scene.name}.npz",
            planning_point_cloud=cloud,
            dense_truth_surface=scene.truth_surface,
            initial_trajectory=initial,
            body_half_extents=half,
        )
        write_json(
            output / "scenes" / f"{scene.name}.json",
            {
                "cloud_stats": stats,
                "cloud_sha256": array_digest(cloud),
                "family": scene.family,
                "params": scene.params,
            },
        )
        # Workers get neither Scene nor truth; no hint generation is called.
        tasks.append(
            (
                scene.name,
                cloud,
                initial,
                half,
                height,
                args.maxiter,
                str(output),
                modes,
                point_selection,
            )
        )
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
            initial, scene.truth_surface, half, args.validation_subdivisions
        )
        for mode in modes:
            prefix = output / "trials" / f"{scene.name}__{mode}"
            with np.load(prefix.with_suffix(".npz")) as data:
                trajectory = data["optimized_trajectory"]
            valid = dense_validation(
                trajectory, scene.truth_surface, half, args.validation_subdivisions
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
                **valid,
                **full["optimizer"],
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
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--maxiter", type=int, default=300)
    parser.add_argument("--validation-subdivisions", type=int, default=50)
    parser.add_argument("--modes", nargs="+", choices=MODES, default=list(MODES))
    parser.add_argument(
        "--point-selection", choices=("all", "radius", "rays"), default="all"
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
    if min(args.workers, args.maxiter, args.validation_subdivisions) < 1:
        parser.error("workers, maxiter and validation-subdivisions must be positive")
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

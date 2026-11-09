"""Read saved experiments and verify real optimizer snapshots.

Both browser and video exporters use this module. Reading never reruns a
planner. Replaying a historical run is an explicit, separately invoked action.
"""

from __future__ import annotations

from .. import experiment  # Set BLAS limits before NumPy.

import argparse
import csv
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
from multiprocessing import get_context
from pathlib import Path

import numpy as np

from .. import core, settings
from ..point_selection import PointSelection


def history_path(run: Path, scene: str, mode: str) -> Path:
    trial = run / "trials" / f"{scene}__{mode}.npz"
    with np.load(trial) as data:
        if "iteration_poses" in data:
            return trial
    return run / "replay" / f"{scene}__{mode}.npz"


def load_history(run: Path, scene: str, mode: str) -> tuple[np.ndarray, np.ndarray]:
    path = history_path(run, scene, mode)
    if not path.exists():
        raise FileNotFoundError(
            f"No optimizer snapshots for {scene}/{mode}. Run the replay command first."
        )
    with np.load(path) as data:
        return (
            (data["iteration_poses"], data["iteration_costs"])
            if "iteration_poses" in data
            else (data["poses"], data["costs"])
        )


def verify_replay(run: Path, scene: str, mode: str) -> dict:
    poses, costs = load_history(run, scene, mode)
    trial = run / "trials" / f"{scene}__{mode}"
    with np.load(trial.with_suffix(".npz")) as data:
        expected = data["optimized_trajectory"]
        initial = data["initial_trajectory"]
    with trial.with_suffix(".history.csv").open() as f:
        recorded = np.array([float(row["cost"]) for row in csv.DictReader(f)])
    np.testing.assert_allclose(poses[0], initial, rtol=0, atol=1e-12)
    np.testing.assert_allclose(poses[-1], expected, rtol=1e-8, atol=1e-9)
    np.testing.assert_allclose(costs, recorded, rtol=1e-8, atol=1e-9)
    if len(poses) != len(recorded):
        raise ValueError(
            "The optimizer snapshot count differs from recorded accepted iterations"
        )
    with np.load(run / "scenes" / f"{scene}.npz") as data:
        digest = experiment.array_digest(data["planning_point_cloud"])
    summary = json.loads(trial.with_suffix(".json").read_text())
    if digest != summary["cloud_sha256"]:
        raise ValueError("Point cloud differs from the recorded trial")
    return {
        "scene": scene,
        "mode": mode,
        "accepted_iterations": len(poses) - 1,
        "cloud_sha256": digest,
        "maximum_final_pose_difference": float(np.abs(poses[-1] - expected).max()),
        "verified_against_original": True,
    }


class SavedRun:
    """The only on-disk format adapter used by visualization clients."""

    def __init__(self, path: Path):
        self.path = Path(path).resolve()
        results = json.loads((self.path / "results.json").read_text())
        self.metadata = results["metadata"]
        self.rows = results["trials"]
        self.scenes = list(dict.fromkeys(row["scene"] for row in self.rows))

    def scene(self, name: str) -> dict:
        if name not in self.scenes:
            raise ValueError(f"Unknown scene: {name}")
        with np.load(self.path / "scenes" / f"{name}.npz") as data:
            return {
                "params": self.metadata.get("scenes", {})
                .get(name, {})
                .get("params", {}),
                **{
                    key: data[key]
                    for key in (
                        "planning_point_cloud",
                        "initial_trajectory",
                        "body_half_extents",
                    )
                },
            }

    def trial(self, name: str, mode: str) -> dict:
        row = next(
            row for row in self.rows if row["scene"] == name and row["mode"] == mode
        )
        prefix = self.path / "trials" / f"{name}__{mode}"
        with np.load(prefix.with_suffix(".npz")) as data:
            result = {"final": data["optimized_trajectory"], "metrics": row}
        with np.load(prefix.with_suffix(".validation.npz")) as data:
            result.update(dense=data["dense_poses"], clearance=data["dense_clearance"])
        if history_path(self.path, name, mode).exists():
            result["history"], result["costs"] = load_history(self.path, name, mode)
            result["verification"] = verify_replay(self.path, name, mode)
        return result


def replay_trial(task: tuple[str, str, str]) -> dict:
    run_name, scene, mode = task
    run = Path(run_name)
    if history_path(run, scene, mode).exists():
        return verify_replay(run, scene, mode)
    protocol = json.loads((run / "protocol.json").read_text())
    if protocol["weights"] != asdict(settings.planner_weights()):
        raise ValueError("Refusing to replay with different objective weights")
    if protocol["activation_distance_m"] != settings.PLANNING_CLEARANCE:
        raise ValueError(
            "Refusing to replay with different collision activation distance"
        )
    if any(protocol[key] != value for key, value in settings.SOLVER_OPTIONS.items()):
        raise ValueError("Refusing to replay with different solver stopping settings")
    with np.load(run / "scenes" / f"{scene}.npz") as data:
        cloud, initial, half = (
            data["planning_point_cloud"],
            data["initial_trajectory"],
            data["body_half_extents"],
        )
    snapshots = []
    _, _, history, _ = experiment.optimize_from_cloud(
        cloud,
        initial,
        half,
        protocol["robot"]["nominal_height_m"],
        mode,
        protocol["maxiter"],
        pose_history=snapshots,
        point_selection=PointSelection(**protocol.get("point_selection", {})),
    )
    costs = np.array([entry["cost"] for entry in history])
    destination = run / "replay" / f"{scene}__{mode}.npz"
    destination.parent.mkdir(exist_ok=True)
    np.savez_compressed(destination, poses=np.asarray(snapshots), costs=costs)
    try:
        return verify_replay(run, scene, mode)
    except Exception:
        destination.unlink()
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description="补录并核对历史实验的真实优化迭代")
    parser.add_argument("--run", type=Path, default=settings.DEFAULT_RUN)
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args(argv)
    if args.workers < 1:
        parser.error("workers must be positive")
    saved = SavedRun(args.run)
    tasks = [
        (str(saved.path), name, mode)
        for name in saved.scenes
        for mode in saved.metadata.get("modes", settings.MODES)
    ]
    checks = []
    with ProcessPoolExecutor(
        max_workers=args.workers, mp_context=get_context("spawn")
    ) as pool:
        pending = [pool.submit(replay_trial, task) for task in tasks]
        for future in as_completed(pending):
            result = future.result()
            checks.append(result)
            print(
                f"Verified {result['scene']} / {result['mode']}: {result['accepted_iterations']} iterations",
                flush=True,
            )
    experiment.write_json(
        saved.path / "replay" / "verification.json",
        {
            "trials": checks,
            "current_core_sha256": hashlib.sha256(
                Path(core.__file__).read_bytes()
            ).hexdigest(),
            "historical_protocol": str(saved.path / "protocol.json"),
            "note": "Layout refactored; every accepted cost and terminal pose checked against the original run.",
        },
    )

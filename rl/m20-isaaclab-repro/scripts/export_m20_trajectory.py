#!/usr/bin/env python3
"""Run an M20 checkpoint without cameras and export its Isaac Sim trajectory.

This is deliberately separate from the normal ``play.py --video`` path.  It
does not create an Isaac Sim camera or render any frames, so it is useful on
servers where physics runs correctly but the native RTX scene renderer cannot
record video.  ``render_m20_trajectory.py`` converts the resulting trajectory
to an offline CPU-rendered video using the same M20 URDF geometry.
"""

import argparse
import importlib.metadata as metadata
import os
from pathlib import Path
import sys


REPRO_ROOT = Path(__file__).resolve().parents[1]
RSL_RL_SCRIPT_DIR = REPRO_ROOT / "src" / "rl_training" / "scripts" / "reinforcement_learning" / "rsl_rl"
sys.path.insert(0, str(RSL_RL_SCRIPT_DIR))
sys.path.insert(0, str(RSL_RL_SCRIPT_DIR.parent))

from isaaclab.app import AppLauncher

import cli_args


parser = argparse.ArgumentParser(description="Export an M20 policy trajectory from Isaac Sim.")
parser.add_argument("--task", type=str, required=True)
parser.add_argument("--agent", type=str, default="rsl_rl_cfg_entry_point")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--trajectory_out", type=Path, required=True)
parser.add_argument("--trajectory_steps", type=int, default=600)
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args

# Cameras stay disabled.  This avoids the Isaac Sim RTX video-rendering path.
args_cli.enable_cameras = False
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import gymnasium as gym
import torch
from packaging import version
from rsl_rl.runners import OnPolicyRunner
from rsl_rl.utils import resolve_callable

from isaaclab.envs import DirectMARLEnv, DirectMARLEnvCfg, DirectRLEnvCfg, ManagerBasedRLEnvCfg, multi_agent_to_single_agent
from isaaclab.utils.assets import retrieve_file_path
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

import rl_training.tasks  # noqa: F401
from rl_training.envs.amp_locomotion_env import AmpLocomotionEnv, AmpRslRlVecEnvWrapper


RSL_RL_VERSION = "5.0.1"
installed_version = metadata.version("rsl-rl-lib")
if version.parse(installed_version) < version.parse(RSL_RL_VERSION):
    raise RuntimeError(f"rsl-rl-lib {RSL_RL_VERSION} or newer is required; found {installed_version}.")


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlOnPolicyRunnerCfg):
    """Execute one deterministic policy rollout and save robot poses."""
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    if env_cfg.scene.terrain.terrain_generator is not None:
        terrain = env_cfg.scene.terrain.terrain_generator
        terrain.num_rows = 5
        terrain.num_cols = 5
        terrain.curriculum = False
        env_cfg.scene.terrain.max_init_terrain_level = None

    env_cfg.observations.policy.enable_corruption = False
    for attr in ("randomize_apply_external_force_torque", "push_robot"):
        if hasattr(env_cfg.events, attr):
            setattr(env_cfg.events, attr, None)
    if hasattr(env_cfg.curriculum, "command_levels"):
        env_cfg.curriculum.command_levels = None

    log_root = Path("logs") / "rsl_rl" / agent_cfg.experiment_name
    if args_cli.checkpoint:
        resume_path = Path(retrieve_file_path(args_cli.checkpoint))
    else:
        resume_path = Path(get_checkpoint_path(str(log_root.resolve()), agent_cfg.load_run, agent_cfg.load_checkpoint))

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode=None)
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)
    if isinstance(env.unwrapped, AmpLocomotionEnv):
        env = AmpRslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    else:
        env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    runner_cfg = agent_cfg.to_dict()
    runner_class_name = getattr(agent_cfg, "class_name", "OnPolicyRunner")
    is_custom_runner = runner_class_name not in ("OnPolicyRunner", "DistillationRunner")
    if version.parse(installed_version) >= version.parse("5.0.0"):
        runner_cfg = cli_args.convert_rsl_rl_cfg_dict(runner_cfg)
    runner_class = resolve_callable(runner_class_name) if is_custom_runner else OnPolicyRunner
    runner = runner_class(env, runner_cfg, log_dir=None, device=agent_cfg.device)
    runner.load(str(resume_path))
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    root_positions, root_orientations, joint_positions = [], [], []
    obs, _ = env.reset()
    robot = env.unwrapped.scene["robot"]

    def capture() -> None:
        root_positions.append(robot.data.root_pos_w[0].detach().cpu().numpy().copy())
        root_orientations.append(robot.data.root_quat_w[0].detach().cpu().numpy().copy())
        joint_positions.append(robot.data.joint_pos[0].detach().cpu().numpy().copy())

    capture()
    with torch.inference_mode():
        for _ in range(args_cli.trajectory_steps):
            obs, _, _, _ = env.step(policy(obs))
            capture()

    out_path = args_cli.trajectory_out.expanduser().resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out_path,
        root_pos_w=np.stack(root_positions),
        root_quat_w=np.stack(root_orientations),  # Isaac Sim order: w, x, y, z
        joint_pos=np.stack(joint_positions),
        joint_names=np.asarray(robot.joint_names),
        step_dt=np.asarray(env.unwrapped.step_dt, dtype=np.float64),
        checkpoint=np.asarray(str(resume_path)),
        seed=np.asarray(args_cli.seed, dtype=np.int64),
    )
    print(f"[TRAJECTORY] frames={len(root_positions)} dt={env.unwrapped.step_dt} output={out_path}")
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()

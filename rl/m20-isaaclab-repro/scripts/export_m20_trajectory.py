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
parser.add_argument(
    "--scenario",
    choices=("random", "stairs_up", "stairs_down"),
    default="random",
    help=(
        "Evaluation terrain. 'stairs_up' and 'stairs_down' create one fixed "
        "pyramid-stair terrain, disable domain randomization, and use a fixed command."
    ),
)
parser.add_argument(
    "--stair_height",
    type=float,
    default=0.10,
    help="Height in metres of every step in a fixed stair scenario (0.05--0.23 is the training range).",
)
parser.add_argument(
    "--command_vx",
    type=float,
    default=0.60,
    help="Fixed forward body-frame velocity command (m/s) for a stair scenario.",
)
parser.add_argument(
    "--command_vy",
    type=float,
    default=0.0,
    help="Fixed lateral body-frame velocity command (m/s) for a stair scenario.",
)
parser.add_argument(
    "--command_yaw",
    type=float,
    default=0.0,
    help="Fixed yaw-rate command (rad/s) for a stair scenario.",
)
parser.add_argument(
    "--spawn_x",
    type=float,
    default=-2.85,
    help="Spawn x offset from the centre of the fixed stair terrain (m).",
)
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
    if args_cli.scenario != "random" and not 0.05 <= args_cli.stair_height <= 0.23:
        raise ValueError("--stair_height must be within the trained stair range [0.05, 0.23] m.")
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    if env_cfg.scene.terrain.terrain_generator is not None:
        terrain = env_cfg.scene.terrain.terrain_generator
        if args_cli.scenario == "random":
            terrain.num_rows = 5
            terrain.num_cols = 5
            terrain.curriculum = False
            env_cfg.scene.terrain.max_init_terrain_level = None
        else:
            # A 1x1 terrain makes the result reproducible: it has exactly one
            # stair layout and no random terrain selection or difficulty draw.
            terrain.num_rows = 1
            terrain.num_cols = 1
            terrain.curriculum = False
            terrain.use_cache = False
            env_cfg.scene.terrain.max_init_terrain_level = 0
            active_stair = "pyramid_stairs" if args_cli.scenario == "stairs_up" else "pyramid_stairs_inv"
            for name, sub_terrain in terrain.sub_terrains.items():
                sub_terrain.proportion = 1.0 if name == active_stair else 0.0
            stair_cfg = terrain.sub_terrains[active_stair]
            stair_cfg.step_height_range = (args_cli.stair_height, args_cli.stair_height)
            stair_cfg.step_width = 0.30
            stair_cfg.platform_width = 3.0
            stair_cfg.border_width = 1.0
            stair_cfg.holes = False

    env_cfg.observations.policy.enable_corruption = False
    for attr in ("randomize_apply_external_force_torque", "push_robot"):
        if hasattr(env_cfg.events, attr):
            setattr(env_cfg.events, attr, None)
    if hasattr(env_cfg.curriculum, "command_levels"):
        env_cfg.curriculum.command_levels = None

    if args_cli.scenario != "random":
        # A video is an evaluation, not another domain-randomized training
        # sample.  Keep the physical instance and reset pose deterministic.
        for name in dir(env_cfg.events):
            if name.startswith("randomize_") or name == "push_robot":
                setattr(env_cfg.events, name, None)
        command_cfg = env_cfg.commands.base_velocity
        command_cfg.resampling_time_range = (10_000.0, 10_000.0)
        command_cfg.heading_command = False
        command_cfg.rel_standing_envs = 0.0
        for attr in ("rel_zero_vel_envs", "rel_only_lin_y_envs", "rel_only_lin_x_envs", "rel_only_ang_z_envs"):
            if hasattr(command_cfg, attr):
                setattr(command_cfg, attr, 0.0)

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
    commanded_velocities, base_velocities = [], []
    terrain_hits = []
    obs, _ = env.reset()
    robot = env.unwrapped.scene["robot"]
    command_term = env.unwrapped.command_manager.get_term("base_velocity")
    fixed_command = torch.tensor(
        [args_cli.command_vx, args_cli.command_vy, args_cli.command_yaw],
        device=env.unwrapped.device,
        dtype=robot.data.root_pos_w.dtype,
    )
    if args_cli.scenario != "random":
        # The generator origin lies on the centre platform.  We place the
        # robot on the first outer step and command it toward the centre: that
        # means ascending for a normal pyramid and descending for an inverted
        # pyramid.  M20's nominal base clearance is its configured 0.58 m.
        initial_surface_z = args_cli.stair_height if args_cli.scenario == "stairs_up" else -args_cli.stair_height
        initial_state = robot.data.default_root_state.clone()
        initial_state[:, :2] += env.unwrapped.scene.env_origins[:, :2]
        initial_state[:, 0] += args_cli.spawn_x
        initial_state[:, 2] = initial_surface_z + robot.data.default_root_state[:, 2]
        robot.write_root_pose_to_sim(initial_state[:, :7])
        robot.write_root_velocity_to_sim(initial_state[:, 7:13])
        command_term.vel_command_b[:] = fixed_command
        command_term.is_standing_env[:] = False
        if hasattr(command_term, "is_heading_env"):
            command_term.is_heading_env[:] = False
        # `write_root_*_to_sim` changes PhysX immediately, while Isaac Lab's
        # cached tensors still describe the earlier reset.  Synchronize them
        # before the first policy call so frame zero and the first observation
        # describe the same fixed stair start state.
        env.unwrapped.scene.write_data_to_sim()
        env.unwrapped.sim.forward()
        env.unwrapped.scene.update(dt=0.0)
        obs = env.get_observations()
    try:
        height_scanner = env.unwrapped.scene["height_scanner"]
    except KeyError:
        height_scanner = None

    def capture() -> None:
        root_positions.append(robot.data.root_pos_w[0].detach().cpu().numpy().copy())
        root_orientations.append(robot.data.root_quat_w[0].detach().cpu().numpy().copy())
        joint_positions.append(robot.data.joint_pos[0].detach().cpu().numpy().copy())
        commanded_velocities.append(
            env.unwrapped.command_manager.get_command("base_velocity")[0].detach().cpu().numpy().copy()
        )
        base_velocities.append(robot.data.root_lin_vel_b[0].detach().cpu().numpy().copy())
        if height_scanner is not None:
            terrain_hits.append(height_scanner.data.ray_hits_w[0].detach().cpu().numpy().copy())

    capture()
    with torch.inference_mode():
        for _ in range(args_cli.trajectory_steps):
            obs, _, _, _ = env.step(policy(obs))
            if args_cli.scenario != "random":
                # Preserve the same command after each environment step.  This
                # also guards against a future command-generator implementation
                # changing its resampling behaviour.
                command_term.vel_command_b[:] = fixed_command
            capture()

    out_path = args_cli.trajectory_out.expanduser().resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out_path,
        root_pos_w=np.stack(root_positions),
        root_quat_w=np.stack(root_orientations),  # Isaac Sim order: w, x, y, z
        joint_pos=np.stack(joint_positions),
        joint_names=np.asarray(robot.joint_names),
        command_velocity=np.stack(commanded_velocities),
        root_lin_vel_b=np.stack(base_velocities),
        step_dt=np.asarray(env.unwrapped.step_dt, dtype=np.float64),
        checkpoint=np.asarray(str(resume_path)),
        seed=np.asarray(args_cli.seed, dtype=np.int64),
        scenario=np.asarray(args_cli.scenario),
        stair_height=np.asarray(args_cli.stair_height, dtype=np.float64),
        requested_command_velocity=np.asarray(
            (args_cli.command_vx, args_cli.command_vy, args_cli.command_yaw), dtype=np.float64
        ),
        spawn_x=np.asarray(args_cli.spawn_x, dtype=np.float64),
        **({"terrain_hits_w": np.stack(terrain_hits)} if terrain_hits else {}),
    )
    print(f"[TRAJECTORY] frames={len(root_positions)} dt={env.unwrapped.step_dt} output={out_path}")
    env.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()

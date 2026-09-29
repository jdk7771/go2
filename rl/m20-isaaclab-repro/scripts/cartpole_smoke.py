"""Run a finite headless Isaac Lab Cartpole smoke test."""

import argparse

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--num_envs", type=int, default=128)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import parse_env_cfg


try:
    env_cfg = parse_env_cfg("Isaac-Cartpole-Direct-v0", device=args_cli.device, num_envs=args_cli.num_envs)
    env = gym.make("Isaac-Cartpole-Direct-v0", cfg=env_cfg)
    env.reset()
    for _ in range(16):
        env.step(torch.zeros(env.action_space.shape, device=env.unwrapped.device))
    env.close()
    print("Cartpole finite smoke test completed.")
finally:
    simulation_app.close()

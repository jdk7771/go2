#!/usr/bin/env bash
set -euo pipefail
experiment_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$experiment_dir/../.." && pwd)
dependency_dir="$repo_dir/.deps/unitree_mujoco"
revision=1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d
if [ ! -d "$dependency_dir/.git" ]; then
  git clone --filter=blob:none --no-checkout https://github.com/unitreerobotics/unitree_mujoco.git "$dependency_dir"
fi
git -C "$dependency_dir" sparse-checkout set unitree_robots/go2w
git -C "$dependency_dir" checkout --detach "$revision"
git -C "$dependency_dir" rev-parse HEAD

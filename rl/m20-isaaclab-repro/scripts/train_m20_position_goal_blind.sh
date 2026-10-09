#!/usr/bin/env bash
# Fresh PPO run for Experiment M20-PG-001.  Never resumes the velocity baseline.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RL_TRAINING_DIR="$ROOT_DIR/src/rl_training"
ISAACLAB_DIR="$ROOT_DIR/src/IsaacLab"
ARTIFACT_DIR="${M20_ARTIFACT_DIR:-/root/autodl-tmp/m20-artifacts}"
CONDA_ROOT="${CONDA_ROOT:-/root/miniconda3}"

"$ROOT_DIR/patches/apply_m20_position_goal_blind_patches.sh" "$RL_TRAINING_DIR" "$ISAACLAB_DIR"

source "$CONDA_ROOT/etc/profile.d/conda.sh"
conda activate m20

autodl_icd=/etc/vulkan/icd.d/autodl_nvidia_egl_icd.json
if [[ -f "$autodl_icd" ]]; then
    export VK_ICD_FILENAMES="$autodl_icd"
fi

mkdir -p "$ARTIFACT_DIR/logs"
cd "$ARTIFACT_DIR"
exec python "$RL_TRAINING_DIR/scripts/reinforcement_learning/rsl_rl/train.py" \
    --task=M20-PositionGoal-Blind-v0 \
    --headless --num_envs=2048 --max_iterations=5000 \
    "$@"

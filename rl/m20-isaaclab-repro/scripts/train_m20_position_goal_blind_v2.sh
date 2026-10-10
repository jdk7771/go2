#!/usr/bin/env bash
# Experiment M20-PG-002: fresh warm-up, then continue the same policy on the 6 m mixed task.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RL_TRAINING_DIR="$ROOT_DIR/src/rl_training"
ISAACLAB_DIR="$ROOT_DIR/src/IsaacLab"
ARTIFACT_DIR="${M20_V2_ARTIFACT_DIR:-/root/autodl-tmp/m20-v2-artifacts}"
CONDA_ROOT="${CONDA_ROOT:-/root/miniconda3}"
NUM_ENVS="${M20_V2_NUM_ENVS:-2048}"
WARMUP_ITERATIONS="${M20_V2_WARMUP_ITERATIONS:-1000}"
TOTAL_ITERATIONS="${M20_V2_TOTAL_ITERATIONS:-5000}"
WARMUP_CHECKPOINT="model_$((WARMUP_ITERATIONS - 1)).pt"
MIXED_ITERATIONS="$((TOTAL_ITERATIONS - WARMUP_ITERATIONS))"

if (( WARMUP_ITERATIONS <= 0 || MIXED_ITERATIONS <= 0 )); then
    echo "M20 V2 iteration counts must satisfy 0 < warm-up < total." >&2
    exit 2
fi

"$ROOT_DIR/patches/apply_m20_position_goal_blind_patches.sh" "$RL_TRAINING_DIR" "$ISAACLAB_DIR"

source "$CONDA_ROOT/etc/profile.d/conda.sh"
conda activate m20

autodl_icd=/etc/vulkan/icd.d/autodl_nvidia_egl_icd.json
if [[ -f "$autodl_icd" ]]; then
    export VK_ICD_FILENAMES="$autodl_icd"
fi

mkdir -p "$ARTIFACT_DIR/logs"
cd "$ARTIFACT_DIR"

python "$RL_TRAINING_DIR/scripts/reinforcement_learning/rsl_rl/train.py" \
    --task=M20-PositionGoal-Blind-Warmup-v0 \
    --headless --num_envs="$NUM_ENVS" --max_iterations="$WARMUP_ITERATIONS" \
    "$@"

exec python "$RL_TRAINING_DIR/scripts/reinforcement_learning/rsl_rl/train.py" \
    --task=M20-PositionGoal-Blind-v0 \
    --headless --num_envs="$NUM_ENVS" --max_iterations="$MIXED_ITERATIONS" \
    --resume --load_run='.*' --checkpoint="$WARMUP_CHECKPOINT" \
    "$@"

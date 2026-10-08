#!/usr/bin/env bash
# Start a fresh 10 m waypoint training run on the taller M20 terrain.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TRAIN_DIR="$ROOT_DIR/src/rl_training"
ARTIFACT_DIR="${M20_ARTIFACT_DIR:-/root/autodl-tmp/m20-artifacts}"
CONDA_ROOT="${CONDA_ROOT:-/root/miniconda3}"

"$ROOT_DIR/patches/apply_m20_goal10m_patch.sh" "$TRAIN_DIR"

source "$CONDA_ROOT/etc/profile.d/conda.sh"
conda activate m20

autodl_icd=/etc/vulkan/icd.d/autodl_nvidia_egl_icd.json
if [[ -f "$autodl_icd" ]]; then
    export VK_ICD_FILENAMES="$autodl_icd"
fi

mkdir -p "$ARTIFACT_DIR/logs"
cd "$ARTIFACT_DIR"
exec python "$TRAIN_DIR/scripts/reinforcement_learning/rsl_rl/train.py" \
    --task=Rough-Deeprobotics-M20-Goal10m-v0 \
    --headless --num_envs=2048 --max_iterations=5000 \
    "$@"

#!/usr/bin/env bash
# Verify the simulator, task registration, then a short M20 RSL-RL run.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAB_DIR="$ROOT_DIR/src/IsaacLab"
TRAIN_DIR="$ROOT_DIR/src/rl_training"
ARTIFACT_DIR="${M20_ARTIFACT_DIR:-/root/autodl-tmp/m20-artifacts}"
CONDA_ROOT="${CONDA_ROOT:-/root/miniconda3}"

source "$CONDA_ROOT/etc/profile.d/conda.sh"
if [[ "${TERM:-dumb}" == "dumb" ]]; then
    export TERM=xterm
fi
"$ROOT_DIR/scripts/preflight_gpu.sh"
conda activate m20

autodl_icd=/etc/vulkan/icd.d/autodl_nvidia_egl_icd.json
if [[ -f "$autodl_icd" ]]; then
    export VK_ICD_FILENAMES="$autodl_icd"
fi

mkdir -p "$ARTIFACT_DIR/logs"

cd "$LAB_DIR"
./isaaclab.sh -p "$ROOT_DIR/scripts/cartpole_smoke.py" \
    --num_envs 128 --headless \
    2>&1 | tee "$ARTIFACT_DIR/logs/isaaclab-cartpole-smoke.log"

cd "$TRAIN_DIR"
python scripts/reinforcement_learning/rsl_rl/train.py \
    --task=Rough-Deeprobotics-M20-v0 --headless --num_envs=64 --max_iterations=5 \
    2>&1 | tee "$ARTIFACT_DIR/logs/m20-five-iterations.log"
nvidia-smi | tee "$ARTIFACT_DIR/logs/nvidia-smi-after-smoke.log"

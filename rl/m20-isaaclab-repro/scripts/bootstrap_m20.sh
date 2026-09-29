#!/usr/bin/env bash
# Create the tested M20 training stack from a fresh Ubuntu 22.04 GPU host.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$ROOT_DIR/src"
ARTIFACT_DIR="${M20_ARTIFACT_DIR:-/root/autodl-tmp/m20-artifacts}"
PIP_CACHE_DIR="${PIP_CACHE_DIR:-/root/autodl-tmp/m20-cache/pip}"
CONDA_ROOT="${CONDA_ROOT:-/root/miniconda3}"

mkdir -p "$SRC_DIR" "$ARTIFACT_DIR/manifests" "$PIP_CACHE_DIR"
export PIP_CACHE_DIR

# Install the libraries used by the Isaac Sim headless renderer, then fail
# before a large Isaac Sim download if the provider has not exposed Vulkan.
if command -v apt-get >/dev/null; then
    apt-get update
    DEBIAN_FRONTEND=noninteractive apt-get install -y libxt6 libglu1-mesa libvulkan1 vulkan-tools
fi
"$ROOT_DIR/scripts/configure_autodl_vulkan.sh"
"$ROOT_DIR/scripts/preflight_gpu.sh"

source "$CONDA_ROOT/etc/profile.d/conda.sh"
if [[ "${TERM:-dumb}" == "dumb" ]]; then
    export TERM=xterm
fi

if ! conda env list | awk '{print $1}' | grep -qx m20; then
    conda create -n m20 python=3.11 -y
fi
conda activate m20

python -m pip install --upgrade pip
python -m pip install torch==2.7.0 torchvision==0.22.0 \
    --index-url https://download.pytorch.org/whl/cu128
python -m pip install "isaacsim[all,extscache]==5.1.0" \
    --extra-index-url https://pypi.nvidia.com

if [[ ! -d "$SRC_DIR/IsaacLab/.git" && ! -f "$SRC_DIR/IsaacLab/isaaclab.sh" ]]; then
    git clone --branch v2.3.2 --depth 1 \
        https://github.com/isaac-sim/IsaacLab.git "$SRC_DIR/IsaacLab"
fi
if [[ ! -d "$SRC_DIR/rl_training/.git" && ! -d "$SRC_DIR/rl_training/source/rl_training" ]]; then
    # The M20 task uses only M20 assets. A sparse model checkout avoids
    # downloading unrelated robot models while retaining the pinned commit.
    git clone --depth 1 https://github.com/DeepRoboticsLab/rl_training.git "$SRC_DIR/rl_training"
fi
RL_DIR="$SRC_DIR/rl_training"
MODEL_DIR="$RL_DIR/deep_robotics_model"
if [[ -f "$MODEL_DIR/M20/usd/M20.usd" && ! -d "$MODEL_DIR/.git" ]]; then
    # restore_from_bundle.sh supplies the required pinned M20 subtree as an
    # archive, so no network clone is necessary on the next server.
    echo "[INFO] Reusing restored M20 asset archive."
elif [[ ! -d "$MODEL_DIR/.git" ]]; then
    MODEL_COMMIT="$(git -C "$RL_DIR" rev-parse HEAD:deep_robotics_model)"
    git clone --depth 1 --filter=blob:none --sparse \
        https://github.com/DeepRoboticsLab/deep_robotics_model.git "$MODEL_DIR"
    git -C "$MODEL_DIR" sparse-checkout set M20
    git -C "$MODEL_DIR" fetch --depth 1 origin "$MODEL_COMMIT"
    git -C "$MODEL_DIR" checkout --detach "$MODEL_COMMIT"
else
    MODEL_COMMIT="$(git -C "$RL_DIR" rev-parse HEAD:deep_robotics_model)"
    git -C "$MODEL_DIR" sparse-checkout set M20
    git -C "$MODEL_DIR" fetch --depth 1 origin "$MODEL_COMMIT"
    git -C "$MODEL_DIR" checkout --detach "$MODEL_COMMIT"
fi

"$ROOT_DIR/patches/apply_m20_asset_fix.sh" "$RL_DIR"

"$SRC_DIR/IsaacLab/isaaclab.sh" -i rsl_rl
python -m pip install 'setuptools<81'
python -m pip install -e "$SRC_DIR/IsaacLab/source/isaaclab" --no-deps
python -m pip install -e "$RL_DIR/source/rl_training"
cd "$RL_DIR"
python scripts/tools/setup_conda_runtime.py
conda deactivate
conda activate m20

python - <<'PY' | tee "$ARTIFACT_DIR/manifests/python_and_gpu.txt"
import torch
print(f"torch={torch.__version__}")
print(f"cuda_available={torch.cuda.is_available()}")
print(f"gpu={torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'unavailable'}")
PY
python -m pip freeze --all | sort > "$ARTIFACT_DIR/manifests/pip-freeze.txt"
conda env export --no-builds > "$ARTIFACT_DIR/manifests/environment.yml"
conda list --explicit > "$ARTIFACT_DIR/manifests/conda-explicit.txt"
git -C "$SRC_DIR/IsaacLab" rev-parse HEAD > "$ARTIFACT_DIR/manifests/isaaclab.commit"
git -C "$SRC_DIR/rl_training" rev-parse HEAD > "$ARTIFACT_DIR/manifests/rl_training.commit"

echo "Bootstrap complete. Activate with: source $CONDA_ROOT/etc/profile.d/conda.sh && conda activate m20"

#!/usr/bin/env bash
# Create small, version-pinned source bundles and manifests for a future server.
# Training checkpoints remain under M20_ARTIFACT_DIR and should be copied separately.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$ROOT_DIR/src"
OUT_DIR="${M20_ARTIFACT_DIR:-/root/autodl-tmp/m20-artifacts}/transfer"
CONDA_ROOT="${CONDA_ROOT:-/root/miniconda3}"
mkdir -p "$OUT_DIR"
rm -f "$OUT_DIR/IsaacLab-v2.3.2.bundle" "$OUT_DIR/rl_training.bundle"

# Shallow Git bundles may refer to missing parent commits and cannot always be
# cloned on another service. Use source archives; commits are recorded below.
tar -C "$SRC_DIR" --exclude='IsaacLab/.git' -czf \
    "$OUT_DIR/IsaacLab-v2.3.2-source.tar.gz" IsaacLab
tar -C "$SRC_DIR" --exclude='rl_training/.git' --exclude='rl_training/deep_robotics_model' -czf \
    "$OUT_DIR/rl_training-source.tar.gz" rl_training
git -C "$SRC_DIR/IsaacLab" rev-parse HEAD > "$OUT_DIR/IsaacLab-v2.3.2.commit"
git -C "$SRC_DIR/rl_training" rev-parse HEAD > "$OUT_DIR/rl_training.commit"
# The server uses a sparse, blob-filtered checkout of this large submodule.
# M20 needs only this pinned asset directory; preserve it as a normal archive.
tar -C "$SRC_DIR/rl_training/deep_robotics_model" -czf \
    "$OUT_DIR/deep_robotics_model-M20.tar.gz" M20
git -C "$SRC_DIR/rl_training/deep_robotics_model" rev-parse HEAD > "$OUT_DIR/deep_robotics_model.commit"

source "$CONDA_ROOT/etc/profile.d/conda.sh"
conda activate m20
python -m pip freeze --all | sort > "$OUT_DIR/pip-freeze.txt"
conda env export --no-builds > "$OUT_DIR/environment.yml"
cp "$ROOT_DIR/docs/SETUP_AND_RUN.md" "$OUT_DIR/"
cp -a "$ROOT_DIR/scripts" "$OUT_DIR/"
cp -a "$ROOT_DIR/patches" "$OUT_DIR/"
WHEELHOUSE_DIR="${M20_WHEELHOUSE_DIR:-/root/autodl-tmp/m20-wheelhouse}"
if [[ -d "$WHEELHOUSE_DIR" ]]; then
    mkdir -p "$OUT_DIR/wheelhouse"
    cp -a "$WHEELHOUSE_DIR"/. "$OUT_DIR/wheelhouse/"
fi
(cd "$OUT_DIR" && find . -type f ! -name SHA256SUMS.txt -print0 | sort -z | \
    xargs -0 sha256sum > SHA256SUMS.txt)
echo "Portable source bundle: $OUT_DIR"

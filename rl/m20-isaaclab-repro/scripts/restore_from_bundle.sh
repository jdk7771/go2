#!/usr/bin/env bash
# Restore pinned source and M20 assets from create_portable_bundle.sh output.
set -euo pipefail

BUNDLE_DIR="${1:?Usage: restore_from_bundle.sh /path/to/transfer [target-root]}"
ROOT_DIR="${2:-/root/m20-repro}"

if [[ ! -f "$BUNDLE_DIR/SHA256SUMS.txt" ]]; then
    echo "ERROR: $BUNDLE_DIR is not a transfer bundle." >&2
    exit 2
fi
(cd "$BUNDLE_DIR" && sha256sum -c SHA256SUMS.txt)

if [[ -e "$ROOT_DIR/src/IsaacLab" || -e "$ROOT_DIR/src/rl_training" ]]; then
    echo "ERROR: source directories already exist under $ROOT_DIR; choose an empty target." >&2
    exit 3
fi

mkdir -p "$ROOT_DIR/src"
tar -C "$ROOT_DIR/src" -xzf "$BUNDLE_DIR/IsaacLab-v2.3.2-source.tar.gz"
tar -C "$ROOT_DIR/src" -xzf "$BUNDLE_DIR/rl_training-source.tar.gz"
mkdir -p "$ROOT_DIR/deep_robotics_model"
tar -C "$ROOT_DIR/deep_robotics_model" -xzf "$BUNDLE_DIR/deep_robotics_model-M20.tar.gz"

mkdir -p "$ROOT_DIR/docs"
cp -a "$BUNDLE_DIR/SETUP_AND_RUN.md" "$ROOT_DIR/docs/"
cp -a "$BUNDLE_DIR/scripts/." "$ROOT_DIR/scripts/"
cp -a "$BUNDLE_DIR/patches/." "$ROOT_DIR/patches/"
"$ROOT_DIR/patches/apply_m20_asset_fix.sh" "$ROOT_DIR/src/rl_training"

echo "Source restored at $ROOT_DIR. Run: bash $ROOT_DIR/scripts/bootstrap_m20.sh"

#!/usr/bin/env bash
# Export a compact, evenly spaced suite of deterministic M20 stair rollouts.
# Run from the reproducibility root after `conda activate m20`.
set -euo pipefail

REPRO_ROOT=${1:-/root/m20-repro}
RUN_DIR=${2:-/root/autodl-tmp/m20-artifacts/logs/rsl_rl/deeprobotics_m20_rough/2026-09-28_08-58-15}
OUT_DIR=${3:-/root/autodl-tmp/m20-artifacts/stair-video-suite}

mkdir -p "$OUT_DIR"
cd "$REPRO_ROOT"

# name checkpoint scenario height(m) seed
SCENARIOS=(
  "model_0_stairs_up_08 model_0.pt stairs_up 0.08 42"
  "model_4999_stairs_up_05 model_4999.pt stairs_up 0.05 42"
  "model_4999_stairs_up_08 model_4999.pt stairs_up 0.08 42"
  "model_4999_stairs_up_12 model_4999.pt stairs_up 0.12 42"
  "model_4999_stairs_up_16 model_4999.pt stairs_up 0.16 42"
  "model_4999_stairs_down_12 model_4999.pt stairs_down 0.12 42"
)

for entry in "${SCENARIOS[@]}"; do
  read -r name checkpoint scenario height seed <<< "$entry"
  out="$OUT_DIR/${name}.npz"
  echo "[SUITE] exporting $name"
  python scripts/export_m20_trajectory.py \
    --task Rough-Deeprobotics-M20-v0 \
    --checkpoint "$RUN_DIR/$checkpoint" \
    --headless --device cuda:0 \
    --seed "$seed" \
    --trajectory_out "$out" \
    --scenario "$scenario" --stair_height "$height" --command_vx 0.60 \
    --trajectory_steps 350
done

echo "[SUITE] trajectory export complete: $OUT_DIR"

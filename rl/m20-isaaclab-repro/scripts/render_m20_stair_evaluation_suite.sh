#!/usr/bin/env bash
# Render every exported stair-evaluation trajectory.  This is deliberately
# separate from GPU simulation: PyBullet TinyRenderer uses the supplied M20
# URDF only to visualize state exported by Isaac Sim.
set -euo pipefail

REPRO_ROOT=${1:-/root/m20-repro}
OUT_DIR=${2:-/root/autodl-tmp/m20-artifacts/stair-video-suite}
URDF="$REPRO_ROOT/src/rl_training/deep_robotics_model/M20/urdf/M20.urdf"
cd "$REPRO_ROOT"

render_one() {
  local name=$1
  local label=$2
  python scripts/render_m20_trajectory.py \
    --trajectory "$OUT_DIR/$name.npz" --urdf "$URDF" \
    --output "$OUT_DIR/$name.raw.mp4" --label "$label" \
    --camera fixed --frame_stride 2 --end_seconds 7.0
}

# Run three CPU renderers at once.  GPU simulation has already finished, and
# each renderer is independent; this shortens the rented-instance time.
render_one model_0_stairs_up_08 "Untrained model | up 8 cm steps | command +0.60 m/s" &
render_one model_4999_stairs_up_05 "Final policy | up 5 cm steps | command +0.60 m/s" &
render_one model_4999_stairs_up_08 "Final policy | up 8 cm steps | command +0.60 m/s" &
wait
render_one model_4999_stairs_up_12 "Final policy | up 12 cm steps | command +0.60 m/s" &
render_one model_4999_stairs_up_16 "Final policy | up 16 cm steps | command +0.60 m/s" &
render_one model_4999_stairs_down_12 "Final policy | down 12 cm steps | command +0.60 m/s" &
wait

echo "[SUITE] rendering complete: $OUT_DIR"

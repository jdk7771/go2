#!/usr/bin/env bash
# Record two deterministic M20 policy replays on a renderer-capable Isaac Sim host.
set -euo pipefail

if [[ $# -ne 3 ]]; then
  echo "Usage: $0 <initial_checkpoint.pt> <current_checkpoint.pt> <output_dir>" >&2
  exit 2
fi

INITIAL_CHECKPOINT=$1
CURRENT_CHECKPOINT=$2
OUTPUT_DIR=$3
PLAY_SCRIPT=/root/m20-repro/src/rl_training/scripts/reinforcement_learning/rsl_rl/play.py
PYTHON=/root/miniconda3/envs/m20/bin/python

for checkpoint in "$INITIAL_CHECKPOINT" "$CURRENT_CHECKPOINT"; do
  [[ -f "$checkpoint" ]] || { echo "Checkpoint not found: $checkpoint" >&2; exit 1; }
done

mkdir -p "$OUTPUT_DIR/initial" "$OUTPUT_DIR/current"
cp "$INITIAL_CHECKPOINT" "$OUTPUT_DIR/initial/model.pt"
cp "$CURRENT_CHECKPOINT" "$OUTPUT_DIR/current/model.pt"

export VK_ICD_FILENAMES=/etc/vulkan/icd.d/autodl_nvidia_egl_icd.json
export TERM=xterm
cd /root/autodl-tmp/m20-artifacts

record() {
  local label=$1
  local checkpoint="$OUTPUT_DIR/$label/model.pt"
  local portable_root="$OUTPUT_DIR/$label/kit-portable"

  "$PYTHON" "$PLAY_SCRIPT" \
    --task=Rough-Deeprobotics-M20-v0 \
    --checkpoint "$checkpoint" \
    --num_envs=1 \
    --seed=42 \
    --video \
    --video_length=600 \
    --headless \
    --kit_args "--portable-root $portable_root --reset-user"
}

record initial
record current

printf 'Videos:\n'
find "$OUTPUT_DIR" -path '*/videos/play/*.mp4' -type f -printf '%p\n'

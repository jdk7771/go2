#!/usr/bin/env bash
# Record the versioned, fixed M20 position-goal test suite for every checkpoint.
# Every test is a route from a fixed start to a fixed 6 m goal. T02--T08 put
# their obstacle on that direct route; they are not generic terrain-category demos.
set -euo pipefail

RUN_DIR="${1:?usage: $0 RUN_DIR [poll_seconds]}"
POLL_SECONDS="${2:-60}"
PROJECT_ROOT="${PROJECT_ROOT:-/root/m20-repro}"
TASK="M20-PositionGoal-Blind-v0"
SEED=20261009
VIDEO_LENGTH=1500  # 30 seconds at the 50 Hz policy rate
SCENARIOS=(T01 T02 T03 T04 T05 T06 T07 T08)
MARKER_DIR="$RUN_DIR/videos/checkpoint-evaluation-markers"
LOG_DIR="$RUN_DIR/videos/checkpoint-evaluation-logs"
mkdir -p "$MARKER_DIR" "$LOG_DIR"

run_checkpoint() {
    local checkpoint="$1"
    local label
    label="$(basename "${checkpoint%.pt}")"
    local marker="$MARKER_DIR/${label}.done"
    [[ -e "$marker" ]] && return 0

    echo "[$(date -Is)] evaluating $label"
    for scenario in "${SCENARIOS[@]}"; do
        python "$PROJECT_ROOT/src/rl_training/scripts/reinforcement_learning/rsl_rl/play.py" \
            --task="$TASK" --headless --num_envs=1 --seed="$SEED" \
            --checkpoint="$checkpoint" --eval_scenario="$scenario" \
            --video --video_length="$VIDEO_LENGTH" --video_prefix="${label}_${scenario}" \
            >"$LOG_DIR/${label}_${scenario}.log" 2>&1
    done
    touch "$marker"
    echo "[$(date -Is)] finished $label"
}

while true; do
    shopt -s nullglob
    checkpoints=("$RUN_DIR"/model_*.pt)
    shopt -u nullglob
    for checkpoint in "${checkpoints[@]}"; do
        run_checkpoint "$checkpoint"
    done
    sleep "$POLL_SECONDS"
done

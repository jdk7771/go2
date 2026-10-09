#!/usr/bin/env bash
# Apply the 6 m blind Position-Goal task after the existing M20 asset and Goal10m patches.
set -euo pipefail

if [[ $# -ne 2 ]]; then
    echo "Usage: $0 /path/to/rl_training /path/to/IsaacLab" >&2
    exit 2
fi

RL_TRAINING_DIR="$1"
ISAACLAB_DIR="$2"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

goal_registration="$RL_TRAINING_DIR/source/rl_training/rl_training/tasks/manager_based/locomotion/velocity/config/wheeled/deeprobotics_m20/__init__.py"
if ! grep -q 'Rough-Deeprobotics-M20-Goal10m-v0' "$goal_registration"; then
    "$SCRIPT_DIR/apply_m20_goal10m_patch.sh" "$RL_TRAINING_DIR"
fi

apply_once() {
    local repo_dir="$1"
    local patch_file="$2"
    local label="$3"
    if git -C "$repo_dir" apply --reverse --check "$patch_file" 2>/dev/null; then
        echo "$label is already installed."
    else
        git -C "$repo_dir" apply --check "$patch_file"
        git -C "$repo_dir" apply "$patch_file"
        echo "Installed $label."
    fi
}

apply_once "$RL_TRAINING_DIR" "$SCRIPT_DIR/m20_position_goal_blind.patch" "M20 Position-Goal task"
apply_once "$RL_TRAINING_DIR" "$SCRIPT_DIR/m20_checkpoint_video_evaluation.patch" "M20 checkpoint video evaluation"
apply_once "$ISAACLAB_DIR" "$SCRIPT_DIR/m20_position_goal_blind_isaaclab.patch" "Isaac Lab physics-step hook"

#!/usr/bin/env bash
# Add the separate M20 Goal10m task to the pinned DeepRobotics checkout.
set -euo pipefail

RL_DIR="${1:?usage: apply_m20_goal10m_patch.sh /path/to/rl_training}"
PATCH_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/m20_goal10m.patch"

if git -C "$RL_DIR" apply --reverse --check "$PATCH_FILE" 2>/dev/null; then
    echo "M20 Goal10m task is already installed."
    exit 0
fi

git -C "$RL_DIR" apply --check "$PATCH_FILE"
git -C "$RL_DIR" apply "$PATCH_FILE"
echo "Installed M20 Goal10m task."

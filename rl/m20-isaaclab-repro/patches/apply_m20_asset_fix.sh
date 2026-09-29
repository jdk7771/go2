#!/usr/bin/env bash
# Apply the local M20 asset-path fix to an upstream rl_training checkout.
set -euo pipefail

RL_DIR="${1:?usage: apply_m20_asset_fix.sh /path/to/rl_training}"
ASSETS_INIT="$RL_DIR/source/rl_training/rl_training/assets/__init__.py"
ROBOT_ASSETS="$RL_DIR/source/rl_training/rl_training/assets/deeprobotics.py"

sed -i \
  's|"../../deep_robotics_model"|"../../../../deep_robotics_model"|' \
  "$ASSETS_INIT"
sed -i \
  's|/M20/M20_usd/M20.usd|/M20/usd/M20.usd|' \
  "$ROBOT_ASSETS"

grep -Fq '"../../../../deep_robotics_model"' "$ASSETS_INIT"
grep -Fq '/M20/usd/M20.usd' "$ROBOT_ASSETS"

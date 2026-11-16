#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export MPLCONFIGDIR="${TMPDIR:-/tmp}/quadric-lidar-matplotlib"
quadric_python="${QUADRIC_PYTHON:-/usr/bin/python3}"
exec "$quadric_python" -m quadric_lidar_repro "$@"

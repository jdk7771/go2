#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONNOUSERSITE=1
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export MPLCONFIGDIR="${TMPDIR:-/tmp}/fsgp-bgk-matplotlib"
fsgp_python="${FSGP_PYTHON:-/usr/bin/python3}"
exec "$fsgp_python" -m fsgp_bgk_repro "$@"

#!/bin/bash
# Noise Sweep Matrix wrapper — runs --fast mode by default
# Usage: ./scripts/test_noise_sweep.sh [--full] [--output path]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTPUT="${NOISE_SWEEP_OUTPUT:-/tmp/noise_sweep_results.csv}"

python3 "${SCRIPT_DIR}/noise_sweep_matrix.py" --fast --output "${OUTPUT}" "$@"
echo "Results saved to: ${OUTPUT}"

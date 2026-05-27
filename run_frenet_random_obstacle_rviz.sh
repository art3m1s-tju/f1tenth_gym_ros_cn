#!/usr/bin/env bash
# One-command random-obstacle RViz visualization runner.
# Defaults to one generated seed and stops automatically after 45 seconds.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TIMEOUT="${FRENET_RANDOM_RVIZ_TIMEOUT:-45}"

exec "${SCRIPT_DIR}/run_frenet_random_obstacle_robustness.sh" \
  --rviz \
  --trials 1 \
  --laps 999 \
  --timeout "${TIMEOUT}" \
  "$@"

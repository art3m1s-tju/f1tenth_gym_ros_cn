#!/usr/bin/env bash
# Batch robustness test for Frenet static-obstacle avoidance.
# Usage:
#   ./run_frenet_random_obstacle_robustness.sh
#   ./run_frenet_random_obstacle_robustness.sh --trials 1 --laps 1 --timeout 80
#   ./run_frenet_random_obstacle_robustness.sh --target-speed 3.0 --avoidance-speed 1.2

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${F1TENTH_DOCKER_IMAGE:-f1tenth_gym_ros:latest}"
CONTAINER_PKG="/sim_ws/src/f1tenth_gym_ros"

TRIALS="${FRENET_RANDOM_TRIALS:-10}"
LAPS="${FRENET_RANDOM_LAPS:-3}"
TIMEOUT="${FRENET_RANDOM_TIMEOUT:-240}"
STUCK_DURATION="${FRENET_RANDOM_STUCK_DURATION:-3.0}"
TARGET_SPEED="${FRENET_TARGET_SPEED:-3.0}"
AVOIDANCE_SPEED="${FRENET_AVOIDANCE_SPEED:-1.2}"
OBSTACLE_COUNT="${FRENET_RANDOM_OBSTACLE_COUNT:-2}"
OBSTACLE_SIZE="${FRENET_RANDOM_OBSTACLE_SIZE:-0.4}"
SEED_START="${FRENET_RANDOM_SEED_START:-0}"
MIN_START_DISTANCE="${FRENET_RANDOM_MIN_START_DISTANCE:-8.0}"
MIN_SEPARATION="${FRENET_RANDOM_MIN_SEPARATION:-5.0}"
BATCH_NAME="${FRENET_RANDOM_BATCH_NAME:-}"
START_X="${FRENET_START_X:-3.0}"
START_Y="${FRENET_START_Y:-5.0}"
START_THETA="${FRENET_START_THETA:-3.1416}"

usage() {
  cat <<USAGE
Usage: $0 [options]

Options:
  --trials N              Number of random seeds to run. Default: ${TRIALS}
  --laps N                Required completed laps per seed. Default: ${LAPS}
  --timeout SEC           Timeout per seed. Default: ${TIMEOUT}
  --stuck-duration SEC    Stop/low-speed duration treated as stuck. Default: ${STUCK_DURATION}
  --target-speed MPS      LQR global target speed. Default: ${TARGET_SPEED}
  --avoidance-speed MPS   Frenet avoidance speed limit. Default: ${AVOIDANCE_SPEED}
  --obstacle-count N      Random obstacles per seed. Default: ${OBSTACLE_COUNT}
  --obstacle-size M       Square obstacle size in meters. Default: ${OBSTACLE_SIZE}
  --seed-start N          First random seed. Default: ${SEED_START}
  --min-start-distance M  Minimum obstacle distance from start. Default: ${MIN_START_DISTANCE}
  --min-separation M      Minimum obstacle separation. Default: ${MIN_SEPARATION}
  --batch-name NAME       Output batch folder name. Default: timestamp
  --sx X --sy Y --stheta R
                          Initial vehicle pose. Default: ${START_X}, ${START_Y}, ${START_THETA}
  -h, --help              Show this help.

Output:
  code/outputs/frenet_random_obstacle_robustness/<batch-name>/
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --trials)
      TRIALS="$2"; shift 2 ;;
    --trials=*)
      TRIALS="${1#*=}"; shift ;;
    --laps)
      LAPS="$2"; shift 2 ;;
    --laps=*)
      LAPS="${1#*=}"; shift ;;
    --timeout)
      TIMEOUT="$2"; shift 2 ;;
    --timeout=*)
      TIMEOUT="${1#*=}"; shift ;;
    --stuck-duration|--stuck-duration-s)
      STUCK_DURATION="$2"; shift 2 ;;
    --stuck-duration=*|--stuck-duration-s=*)
      STUCK_DURATION="${1#*=}"; shift ;;
    --target-speed|--targetspeed)
      TARGET_SPEED="$2"; shift 2 ;;
    --target-speed=*|--targetspeed=*)
      TARGET_SPEED="${1#*=}"; shift ;;
    --avoidance-speed|--avoidancespeed)
      AVOIDANCE_SPEED="$2"; shift 2 ;;
    --avoidance-speed=*|--avoidancespeed=*)
      AVOIDANCE_SPEED="${1#*=}"; shift ;;
    --obstacle-count)
      OBSTACLE_COUNT="$2"; shift 2 ;;
    --obstacle-count=*)
      OBSTACLE_COUNT="${1#*=}"; shift ;;
    --obstacle-size|--obstacle-size-m)
      OBSTACLE_SIZE="$2"; shift 2 ;;
    --obstacle-size=*|--obstacle-size-m=*)
      OBSTACLE_SIZE="${1#*=}"; shift ;;
    --seed-start)
      SEED_START="$2"; shift 2 ;;
    --seed-start=*)
      SEED_START="${1#*=}"; shift ;;
    --min-start-distance|--min-start-distance-m)
      MIN_START_DISTANCE="$2"; shift 2 ;;
    --min-start-distance=*|--min-start-distance-m=*)
      MIN_START_DISTANCE="${1#*=}"; shift ;;
    --min-separation|--min-separation-m)
      MIN_SEPARATION="$2"; shift 2 ;;
    --min-separation=*|--min-separation-m=*)
      MIN_SEPARATION="${1#*=}"; shift ;;
    --batch-name)
      BATCH_NAME="$2"; shift 2 ;;
    --batch-name=*)
      BATCH_NAME="${1#*=}"; shift ;;
    --sx)
      START_X="$2"; shift 2 ;;
    --sx=*)
      START_X="${1#*=}"; shift ;;
    --sy)
      START_Y="$2"; shift 2 ;;
    --sy=*)
      START_Y="${1#*=}"; shift ;;
    --stheta)
      START_THETA="$2"; shift 2 ;;
    --stheta=*)
      START_THETA="${1#*=}"; shift ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "[ERROR] Unknown argument: $1"
      usage
      exit 2
      ;;
  esac
done

RUNNER_ARGS=(
  --repo-root "${CONTAINER_PKG}"
  --trials "${TRIALS}"
  --laps "${LAPS}"
  --timeout "${TIMEOUT}"
  --stuck-duration "${STUCK_DURATION}"
  --seed-start "${SEED_START}"
  --target-speed "${TARGET_SPEED}"
  --avoidance-speed "${AVOIDANCE_SPEED}"
  --obstacle-count "${OBSTACLE_COUNT}"
  --obstacle-size-m "${OBSTACLE_SIZE}"
  --min-start-distance-m "${MIN_START_DISTANCE}"
  --min-separation-m "${MIN_SEPARATION}"
  --sx "${START_X}"
  --sy "${START_Y}"
  --stheta "${START_THETA}"
)

if [[ -n "${BATCH_NAME}" ]]; then
  RUNNER_ARGS+=(--batch-name "${BATCH_NAME}")
fi

printf -v RUNNER_ARGS_QUOTED ' %q' "${RUNNER_ARGS[@]}"

read -r -d '' INNER_CMD <<EOF || true
set -e
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source /sim_ws/install/local_setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest \
  ${CONTAINER_PKG}/test/test_frenet_planner.py \
  ${CONTAINER_PKG}/test/test_frenet_random_robustness.py \
  -q
python3 ${CONTAINER_PKG}/code/lqr_sweep/frenet_random_robustness.py${RUNNER_ARGS_QUOTED}
EOF

echo "=========================================="
echo " Frenet random obstacle robustness"
echo "=========================================="
echo " repo:            ${REPO_ROOT}"
echo " image:           ${IMAGE}"
echo " trials/laps:     ${TRIALS}/${LAPS}"
echo " timeout:         ${TIMEOUT}s per trial"
echo " stuck_duration:  ${STUCK_DURATION}s"
echo " target/avoid:    ${TARGET_SPEED}/${AVOIDANCE_SPEED} m/s"
echo " obstacles:       count=${OBSTACLE_COUNT}, size=${OBSTACLE_SIZE}m"
echo " seed_start:      ${SEED_START}"
echo " start_pose:      (${START_X}, ${START_Y}, ${START_THETA})"
echo "=========================================="

docker run --rm \
  --entrypoint /bin/bash \
  -v "${REPO_ROOT}:${CONTAINER_PKG}" \
  "${IMAGE}" \
  -lc "${INNER_CMD}"

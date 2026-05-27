#!/usr/bin/env bash
# One-command Frenet static obstacle validation runner.
# Usage:
#   ./run_frenet_test.sh                              # launch simulator + RViz
#   ./run_frenet_test.sh --target-speed 1.0          # set LQR cruise speed
#   ./run_frenet_test.sh --avoidance-speed 0.75      # set Frenet local speed cap
#   ./run_frenet_test.sh --sx 3.0 --sy 5.0           # override initial pose
#   ./run_frenet_test.sh --open-map                  # use the old free-space map
#   ./run_frenet_test.sh --headless                  # run unit tests + 25s headless launch
#   ./run_frenet_test.sh --speed-test --target-speed 1.0 # run 90s headless speed test

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${F1TENTH_DOCKER_IMAGE:-f1tenth_gym_ros:latest}"
CONTAINER_PKG="/sim_ws/src/f1tenth_gym_ros"
MODE="rviz"
TARGET_SPEED="${FRENET_TARGET_SPEED:-0.85}"
AVOIDANCE_SPEED="${FRENET_AVOIDANCE_SPEED:-0.75}"
MAP_VARIANT="${FRENET_MAP_VARIANT:-bounded}"
START_X="${FRENET_START_X:-3.0}"
START_Y="${FRENET_START_Y:-5.0}"
START_THETA="${FRENET_START_THETA:-3.1416}"

usage() {
  cat <<USAGE
Usage: $0 [--rviz|--headless|--speed-test] [--target-speed MPS] [--avoidance-speed MPS] [--max-path-length M] [--sx X] [--sy Y] [--stheta RAD] [--bounded-map|--open-map]

Options:
  --target-speed MPS   LQR global cruise speed in m/s. Default: ${TARGET_SPEED}
  --avoidance-speed MPS
                       Frenet local speed limit while avoiding. Default: ${AVOIDANCE_SPEED}
  --max-path-length M  Cap the published Frenet path length. Default: speed preset.
  --sx X               Initial vehicle x position. Default: ${START_X}
  --sy Y               Initial vehicle y position. Default: ${START_Y}
  --stheta RAD         Initial vehicle heading. Default: ${START_THETA}
  --rviz               Launch simulator with RViz. Default mode.
  --headless           Run unit tests + 25s headless launch.
  --speed-test         Run unit tests + 90s headless launch.
  --bounded-map        Use track-boundary map with static obstacles. Default.
  --open-map           Use the previous free-space static-obstacle map.

Environment:
  FRENET_TARGET_SPEED  Default target speed if --target-speed is omitted.
  FRENET_AVOIDANCE_SPEED
                       Default avoidance speed if --avoidance-speed is omitted.
  FRENET_MAP_VARIANT   bounded or open. Default: ${MAP_VARIANT}
  FRENET_START_X / FRENET_START_Y / FRENET_START_THETA
                       Override the initial vehicle pose.
  FRENET_GEOMETRY_TARGET_SPEED
                       Override Frenet geometric target speed.
  FRENET_V_MIN / FRENET_V_MAX
                       Override Frenet longitudinal speed samples.
  FRENET_GRID_FORWARD_M
                       Override Frenet local occupancy grid forward range.
  FRENET_D_STEP / FRENET_V_STEP / FRENET_TRAJECTORY_DT
                       Override Frenet candidate sampling density.
  FRENET_ACTIVATION_PATH_MARGIN
                       Extra distance beyond the published path before Frenet activates.
  FRENET_CENTERLINE_RETURN_LOOKAHEAD_M
                       Override smooth return-to-centerline length.
  FRENET_MAX_PUBLISHED_PATH_LENGTH_M
                       Override the published Frenet path length cap.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    rviz|--rviz|--headless|--speed-test)
      MODE="$1"
      shift
      ;;
    --target-speed|--targetspeed)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --target-speed requires a numeric value."
        usage
        exit 2
      fi
      TARGET_SPEED="$2"
      shift 2
      ;;
    --target-speed=*|--targetspeed=*)
      TARGET_SPEED="${1#*=}"
      shift
      ;;
    --avoidance-speed|--avoidancespeed)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --avoidance-speed requires a numeric value."
        usage
        exit 2
      fi
      AVOIDANCE_SPEED="$2"
      shift 2
      ;;
    --avoidance-speed=*|--avoidancespeed=*)
      AVOIDANCE_SPEED="${1#*=}"
      shift
      ;;
    --max-path-length|--max-published-path-length)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --max-path-length requires a numeric value."
        usage
        exit 2
      fi
      FRENET_MAX_PUBLISHED_PATH_LENGTH_M="$2"
      shift 2
      ;;
    --max-path-length=*|--max-published-path-length=*)
      FRENET_MAX_PUBLISHED_PATH_LENGTH_M="${1#*=}"
      shift
      ;;
    --sx)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --sx requires a numeric value."
        usage
        exit 2
      fi
      START_X="$2"
      shift 2
      ;;
    --sx=*)
      START_X="${1#*=}"
      shift
      ;;
    --sy)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --sy requires a numeric value."
        usage
        exit 2
      fi
      START_Y="$2"
      shift 2
      ;;
    --sy=*)
      START_Y="${1#*=}"
      shift
      ;;
    --stheta)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --stheta requires a numeric value."
        usage
        exit 2
      fi
      START_THETA="$2"
      shift 2
      ;;
    --stheta=*)
      START_THETA="${1#*=}"
      shift
      ;;
    --bounded-map)
      MAP_VARIANT="bounded"
      shift
      ;;
    --open-map)
      MAP_VARIANT="open"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1"
      usage
      exit 2
      ;;
  esac
done

if ! [[ "${TARGET_SPEED}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "[ERROR] target speed must be a non-negative number, got: ${TARGET_SPEED}"
  exit 2
fi
if ! [[ "${AVOIDANCE_SPEED}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "[ERROR] avoidance speed must be a non-negative number, got: ${AVOIDANCE_SPEED}"
  exit 2
fi
for pose_value in "${START_X}" "${START_Y}" "${START_THETA}"; do
  if ! [[ "${pose_value}" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
    echo "[ERROR] pose values must be numeric, got: ${pose_value}"
    exit 2
  fi
done
case "${MAP_VARIANT}" in
  bounded)
    MAP_NAME="frenet_test_two_blocks"
    TRACK_NAME="frenet_test_loop_processed_track.csv"
    ;;
  open)
    MAP_NAME="frenet_test_open_two_blocks"
    TRACK_NAME="frenet_test_loop_open_processed_track.csv"
    ;;
  *)
    echo "[ERROR] map variant must be 'bounded' or 'open', got: ${MAP_VARIANT}"
    exit 2
    ;;
esac

eval "$(
  PYTHONPATH="${REPO_ROOT}/code${PYTHONPATH:+:${PYTHONPATH}}" \
    python3 -m pnc_rc.frenet.preset \
      --target-speed "${TARGET_SPEED}" \
      --avoidance-speed "${AVOIDANCE_SPEED}" \
      --shell-defaults
)"
FRENET_GEOMETRY_TARGET_SPEED="${FRENET_GEOMETRY_TARGET_SPEED:-${FRENET_GEOMETRY_TARGET_SPEED_DEFAULT}}"
FRENET_V_MIN="${FRENET_V_MIN:-${FRENET_V_MIN_DEFAULT}}"
FRENET_V_MAX="${FRENET_V_MAX:-${FRENET_V_MAX_DEFAULT}}"
FRENET_GRID_FORWARD_M="${FRENET_GRID_FORWARD_M:-${FRENET_GRID_FORWARD_DEFAULT}}"
FRENET_V_STEP="${FRENET_V_STEP:-${FRENET_V_STEP_DEFAULT}}"
FRENET_D_STEP="${FRENET_D_STEP:-${FRENET_D_STEP_DEFAULT}}"
FRENET_TRAJECTORY_DT="${FRENET_TRAJECTORY_DT:-${FRENET_TRAJECTORY_DT_DEFAULT}}"
FRENET_HOLD_REPLAN_CLEARANCE="${FRENET_HOLD_REPLAN_CLEARANCE:-${FRENET_HOLD_REPLAN_CLEARANCE_DEFAULT}}"
FRENET_HOLD_MIN_REMAINING="${FRENET_HOLD_MIN_REMAINING:-${FRENET_HOLD_MIN_REMAINING_DEFAULT}}"
FRENET_REUSE_TIMEOUT="${FRENET_REUSE_TIMEOUT:-${FRENET_REUSE_TIMEOUT_DEFAULT}}"
FRENET_ACTIVATION_MAX="${FRENET_ACTIVATION_MAX:-${FRENET_ACTIVATION_MAX_DEFAULT}}"
FRENET_ACTIVATION_REACTION="${FRENET_ACTIVATION_REACTION:-${FRENET_ACTIVATION_REACTION_DEFAULT}}"
FRENET_APPROACH_EXTRA="${FRENET_APPROACH_EXTRA:-${FRENET_APPROACH_EXTRA_DEFAULT}}"
FRENET_ACTIVATION_PATH_MARGIN="${FRENET_ACTIVATION_PATH_MARGIN:-${FRENET_ACTIVATION_PATH_MARGIN_DEFAULT}}"
FRENET_CENTERLINE_RETURN_LOOKAHEAD_M="${FRENET_CENTERLINE_RETURN_LOOKAHEAD_M:-${FRENET_CENTERLINE_RETURN_LOOKAHEAD_DEFAULT}}"
FRENET_MAX_PUBLISHED_PATH_LENGTH_M="${FRENET_MAX_PUBLISHED_PATH_LENGTH_M:-${FRENET_MAX_PUBLISHED_PATH_LENGTH_DEFAULT}}"
FRENET_LAUNCH_ARGS="$(
  PYTHONPATH="${REPO_ROOT}/code${PYTHONPATH:+:${PYTHONPATH}}" \
    python3 -m pnc_rc.frenet.preset \
      --target-speed "${TARGET_SPEED}" \
      --avoidance-speed "${AVOIDANCE_SPEED}" \
      --geometry-target-speed "${FRENET_GEOMETRY_TARGET_SPEED}" \
      --v-min "${FRENET_V_MIN}" \
      --v-max "${FRENET_V_MAX}" \
      --v-step "${FRENET_V_STEP}" \
      --d-step "${FRENET_D_STEP}" \
      --trajectory-dt "${FRENET_TRAJECTORY_DT}" \
      --grid-forward "${FRENET_GRID_FORWARD_M}" \
      --hold-replan-clearance "${FRENET_HOLD_REPLAN_CLEARANCE}" \
      --hold-min-remaining "${FRENET_HOLD_MIN_REMAINING}" \
      --reuse-timeout "${FRENET_REUSE_TIMEOUT}" \
      --activation-max "${FRENET_ACTIVATION_MAX}" \
      --activation-reaction "${FRENET_ACTIVATION_REACTION}" \
      --approach-extra "${FRENET_APPROACH_EXTRA}" \
      --activation-path-margin "${FRENET_ACTIVATION_PATH_MARGIN}" \
      --centerline-return-lookahead "${FRENET_CENTERLINE_RETURN_LOOKAHEAD_M}" \
      --max-published-path-length "${FRENET_MAX_PUBLISHED_PATH_LENGTH_M}" \
      --shell-launch-args
)"

ENABLE_RVIZ="true"
RUN_PREFIX=""
RUN_SUFFIX=""
RUN_TESTS=""
TTY_ARGS=(-it)
if [[ "${MODE}" == "--headless" || "${MODE}" == "--speed-test" ]]; then
  ENABLE_RVIZ="false"
  HEADLESS_TIMEOUT_S="${FRENET_HEADLESS_TIMEOUT_S:-25}"
  if [[ "${MODE}" == "--speed-test" ]]; then
    HEADLESS_TIMEOUT_S="${FRENET_SPEED_TEST_TIMEOUT_S:-90}"
  fi
  RUN_PREFIX="timeout ${HEADLESS_TIMEOUT_S}s "
  RUN_SUFFIX=" || [[ \$? -eq 124 ]]"
  RUN_TESTS="PYTHONDONTWRITEBYTECODE=1 python3 -m pytest ${CONTAINER_PKG}/test/test_frenet_planner.py -q && "
  TTY_ARGS=()
fi

if [[ "${ENABLE_RVIZ}" == "true" && -z "${DISPLAY:-}" ]]; then
  echo "[ERROR] DISPLAY is empty. Run with --headless or enable X11 forwarding first."
  exit 1
fi

if [[ "${ENABLE_RVIZ}" == "true" ]] && command -v xhost >/dev/null 2>&1; then
  if ! xhost +local:docker >/dev/null 2>&1; then
    echo "[WARN] Could not grant Docker X11 access automatically."
    echo "[WARN] If RViz cannot open, run: xhost +local:docker"
  fi
fi

read -r -d '' INNER_CMD <<EOF || true
set -e
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source /sim_ws/install/local_setup.bash
${RUN_TESTS}${RUN_PREFIX}ros2 launch f1tenth_gym_ros pnc_sim_launch.py \
enable_rviz:=${ENABLE_RVIZ} \
enable_frenet_planner:=true \
map_path:=${CONTAINER_PKG}/maps/generated_static_obstacles/${MAP_NAME} \
track_csv:=${CONTAINER_PKG}/code/outputs/generated_tracks/${TRACK_NAME} \
trajectory_mode:=centerline \
sx:=${START_X} \
sy:=${START_Y} \
stheta:=${START_THETA} \
target_speed:=${TARGET_SPEED} \
min_speed:=0.20 \
max_lateral_accel:=1.5 \
max_accel:=1.0 \
max_decel:=2.0 \
curvature_speed_lookahead_m:=1.5 \
local_speed_limit_timeout_s:=1.0 \
${FRENET_LAUNCH_ARGS}${RUN_SUFFIX}
EOF

echo "=========================================="
echo " Frenet static obstacle test"
echo "=========================================="
echo " repo:        ${REPO_ROOT}"
echo " image:       ${IMAGE}"
echo " rviz:        ${ENABLE_RVIZ}"
echo " mode:        ${MODE}"
echo " map:         ${MAP_VARIANT} (${MAP_NAME})"
echo " target_speed: ${TARGET_SPEED} m/s"
echo " avoid_speed: ${AVOIDANCE_SPEED} m/s"
echo " start_pose: (${START_X}, ${START_Y}, ${START_THETA})"
echo " frenet_geom: target=${FRENET_GEOMETRY_TARGET_SPEED} v=[${FRENET_V_MIN}, ${FRENET_V_MAX}] v_step=${FRENET_V_STEP} d_step=${FRENET_D_STEP} dt=${FRENET_TRAJECTORY_DT}"
echo " frenet_grid: forward=${FRENET_GRID_FORWARD_M}m hold_clearance=${FRENET_HOLD_REPLAN_CLEARANCE}m hold_remaining=${FRENET_HOLD_MIN_REMAINING}m reuse=${FRENET_REUSE_TIMEOUT}s"
echo " activation: max=${FRENET_ACTIVATION_MAX}m reaction=${FRENET_ACTIVATION_REACTION}s path_margin=${FRENET_ACTIVATION_PATH_MARGIN}m approach_extra=${FRENET_APPROACH_EXTRA}m"
echo " centerline_return: lookahead=${FRENET_CENTERLINE_RETURN_LOOKAHEAD_M}m"
echo " published_path: max_length=${FRENET_MAX_PUBLISHED_PATH_LENGTH_M}m"
echo "=========================================="

DOCKER_ARGS=(
  run
  --rm
  "${TTY_ARGS[@]}"
  --entrypoint /bin/bash
  -v "${REPO_ROOT}:${CONTAINER_PKG}"
)

if [[ "${ENABLE_RVIZ}" == "true" ]]; then
  DOCKER_ARGS+=(
    -e "DISPLAY=${DISPLAY}"
    -e QT_X11_NO_MITSHM=1
    -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
    -v /tmp/.X11-unix:/tmp/.X11-unix
  )
  if [[ -n "${XAUTHORITY:-}" && -f "${XAUTHORITY}" ]]; then
    DOCKER_ARGS+=(
      -e "XAUTHORITY=/tmp/.docker.xauth"
      -v "${XAUTHORITY}:/tmp/.docker.xauth:ro"
    )
  fi
fi

DOCKER_ARGS+=(
  "${IMAGE}"
  -lc "${INNER_CMD}"
)

docker "${DOCKER_ARGS[@]}"

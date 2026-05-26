#!/usr/bin/env bash
# One-command Frenet static obstacle validation runner.
# Usage:
#   ./run_frenet_test.sh                              # launch simulator + RViz
#   ./run_frenet_test.sh --target-speed 1.0          # set LQR cruise speed
#   ./run_frenet_test.sh --avoidance-speed 0.75      # set Frenet local speed cap
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

usage() {
  cat <<USAGE
Usage: $0 [--rviz|--headless|--speed-test] [--target-speed MPS] [--avoidance-speed MPS] [--bounded-map|--open-map]

Options:
  --target-speed MPS   LQR global cruise speed in m/s. Default: ${TARGET_SPEED}
  --avoidance-speed MPS
                       Frenet local speed limit while avoiding. Default: ${AVOIDANCE_SPEED}
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
  FRENET_GEOMETRY_TARGET_SPEED
                       Override Frenet geometric target speed.
  FRENET_V_MIN / FRENET_V_MAX
                       Override Frenet longitudinal speed samples.
  FRENET_GRID_FORWARD_M
                       Override Frenet local occupancy grid forward range.
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
  python3 - <<PY
target = float("${TARGET_SPEED}")
avoidance = float("${AVOIDANCE_SPEED}")
geom_target = max(1.2, target, avoidance)
v_min = max(0.6, min(avoidance * 0.7, target * 0.5, geom_target))
v_max = max(1.8, geom_target * 1.25, avoidance * 1.5)
grid_forward = max(10.0, target * 6.0 + 2.0)
print(f'FRENET_GEOMETRY_TARGET_SPEED_DEFAULT="{geom_target:.3f}"')
print(f'FRENET_V_MIN_DEFAULT="{v_min:.3f}"')
print(f'FRENET_V_MAX_DEFAULT="{v_max:.3f}"')
print(f'FRENET_GRID_FORWARD_DEFAULT="{grid_forward:.3f}"')
PY
)"
FRENET_GEOMETRY_TARGET_SPEED="${FRENET_GEOMETRY_TARGET_SPEED:-${FRENET_GEOMETRY_TARGET_SPEED_DEFAULT}}"
FRENET_V_MIN="${FRENET_V_MIN:-${FRENET_V_MIN_DEFAULT}}"
FRENET_V_MAX="${FRENET_V_MAX:-${FRENET_V_MAX_DEFAULT}}"
FRENET_GRID_FORWARD_M="${FRENET_GRID_FORWARD_M:-${FRENET_GRID_FORWARD_DEFAULT}}"

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
  if ! xhost >/dev/null 2>&1; then
    echo "[WARN] xhost check failed. If RViz cannot open, run: xhost +local:docker"
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
sx:=0.0 \
sy:=5.0 \
stheta:=3.1416 \
target_speed:=${TARGET_SPEED} \
min_speed:=0.20 \
max_lateral_accel:=1.5 \
max_accel:=1.0 \
max_decel:=2.0 \
curvature_speed_lookahead_m:=1.5 \
local_speed_limit_timeout_s:=1.0 \
frenet_reference_closed_loop:=true \
frenet_centerline_speed_limit_mps:=-1.0 \
frenet_avoidance_speed_limit_mps:=${AVOIDANCE_SPEED} \
frenet_stop_speed_limit_mps:=0.0 \
frenet_target_speed:=${FRENET_GEOMETRY_TARGET_SPEED} \
frenet_v_min:=${FRENET_V_MIN} \
frenet_v_max:=${FRENET_V_MAX} \
frenet_v_step:=0.6 \
frenet_t_min:=4.0 \
frenet_t_max:=6.0 \
frenet_t_step:=2.0 \
frenet_trajectory_dt:=0.05 \
frenet_d_min:=-1.8 \
frenet_d_max:=1.8 \
frenet_d_step:=0.3 \
frenet_max_heading_jump:=0.85 \
frenet_grid_inflation_radius_m:=0.18 \
frenet_grid_forward_m:=${FRENET_GRID_FORWARD_M} \
frenet_grid_half_width_m:=3.2 \
frenet_max_curvature:=1.1 \
frenet_corridor_radius_m:=0.16 \
frenet_corridor_sample_step_m:=0.05 \
frenet_path_collision_sample_step_m:=0.05 \
frenet_footprint_front_m:=0.45 \
frenet_footprint_rear_m:=0.05 \
frenet_safe_clearance_m:=0.30 \
frenet_min_clearance_m:=0.06 \
frenet_published_path_lookahead_m:=0.25 \
frenet_min_path_publish_interval_s:=0.25 \
frenet_path_republish_distance_m:=0.50 \
frenet_path_republish_min_remaining_m:=2.0 \
frenet_centerline_return_lookahead_m:=5.0 \
frenet_centerline_threat_lookahead_m:=8.0 \
frenet_centerline_threat_corridor_radius_m:=0.22 \
frenet_activation_min_lookahead_m:=3.0 \
frenet_activation_max_lookahead_m:=8.0 \
frenet_activation_base_lookahead_m:=2.2 \
frenet_activation_reaction_time_s:=1.0 \
frenet_activation_decel_mps2:=2.0 \
frenet_reuse_last_candidate_timeout_s:=1.0${RUN_SUFFIX}
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
echo " frenet_geom: target=${FRENET_GEOMETRY_TARGET_SPEED} v=[${FRENET_V_MIN}, ${FRENET_V_MAX}] grid_forward=${FRENET_GRID_FORWARD_M}m"
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
    -v /tmp/.X11-unix:/tmp/.X11-unix
  )
fi

DOCKER_ARGS+=(
  "${IMAGE}"
  -lc "${INNER_CMD}"
)

docker "${DOCKER_ARGS[@]}"

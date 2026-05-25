#!/usr/bin/env bash
# One-command Frenet static obstacle validation runner.
# Usage:
#   ./run_frenet_test.sh            # launch simulator + RViz
#   ./run_frenet_test.sh --headless # run unit tests + 25s headless launch

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="${F1TENTH_DOCKER_IMAGE:-f1tenth_gym_ros:latest}"
CONTAINER_PKG="/sim_ws/src/f1tenth_gym_ros"
MODE="${1:-rviz}"

if [[ "${MODE}" != "rviz" && "${MODE}" != "--rviz" && "${MODE}" != "--headless" ]]; then
  echo "Usage: $0 [--rviz|--headless]"
  exit 2
fi

ENABLE_RVIZ="true"
RUN_PREFIX=""
RUN_SUFFIX=""
RUN_TESTS=""
TTY_ARGS=(-it)
if [[ "${MODE}" == "--headless" ]]; then
  ENABLE_RVIZ="false"
  RUN_PREFIX="timeout 25s "
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
map_path:=${CONTAINER_PKG}/maps/generated_static_obstacles/frenet_test_open_two_blocks \
track_csv:=${CONTAINER_PKG}/code/outputs/generated_tracks/frenet_test_loop_open_processed_track.csv \
trajectory_mode:=centerline \
sx:=0.0 \
sy:=5.0 \
stheta:=3.1416 \
frenet_reference_closed_loop:=false \
frenet_d_min:=-1.8 \
frenet_d_max:=1.8 \
frenet_max_heading_jump:=0.85 \
frenet_grid_inflation_radius_m:=0.22 \
frenet_max_curvature:=1.1 \
frenet_corridor_radius_m:=0.14 \
frenet_footprint_front_m:=0.38 \
frenet_footprint_rear_m:=0.05 \
frenet_safe_clearance_m:=0.35 \
frenet_min_clearance_m:=0.05${RUN_SUFFIX}
EOF

echo "=========================================="
echo " Frenet static obstacle test"
echo "=========================================="
echo " repo:        ${REPO_ROOT}"
echo " image:       ${IMAGE}"
echo " rviz:        ${ENABLE_RVIZ}"
echo " mode:        ${MODE}"
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

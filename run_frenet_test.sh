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
target_speed:=0.55 \
min_speed:=0.15 \
max_lateral_accel:=1.0 \
max_accel:=0.6 \
max_decel:=1.5 \
curvature_speed_lookahead_m:=1.5 \
frenet_reference_closed_loop:=false \
frenet_target_speed:=1.2 \
frenet_v_min:=0.8 \
frenet_v_max:=1.8 \
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
frenet_grid_forward_m:=8.0 \
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
frenet_centerline_threat_lookahead_m:=6.0 \
frenet_centerline_threat_corridor_radius_m:=0.22 \
frenet_reuse_last_candidate_timeout_s:=1.0${RUN_SUFFIX}
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

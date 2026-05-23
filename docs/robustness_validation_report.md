# ST Corridor Robustness Validation Report

Date: 2026-05-23
Branch: `codex/full-takeover-optimization-test`
Container: `codex_st_corridor_test`

## Scope

This report records the current evidence for the requested robustness target:

- No-obstacle LQR tracking from 0.5 m/s to 3.0 m/s.
- Static-obstacle ST-corridor avoidance from 0.5 m/s to 3.0 m/s.
- Random 2-3 obstacle blocks, each 0.5 m x 0.5 m, placed on the global trajectory.
- Five laps with no `ego_collision`.
- MP4 visualization for each tuned speed.

## Key Runtime Defaults

- `lqr_lookahead_distance_m = 0.0`
- `max_lateral_accel = 2.0`
- ST lookahead and avoidance speed are selected by speed in `code/run_robustness_validation.py`.

## LQR Baseline Evidence

Passing no-obstacle LQR manifests:

- `code/outputs/robustness/lqr_lowspeed_alat2p0_clean/manifest.csv`
- `code/outputs/robustness/lqr_highspeed_alat2p0/manifest.csv`

| Speed (m/s) | Pass | Collision Events | Duration (s) | Max Lateral Error (cm) |
| --- | --- | --- | --- | --- |
| 0.50 | 1 | 0 | 144.172 | 1.2355 |
| 1.00 | 1 | 0 | 73.784 | 2.8426 |
| 1.50 | 1 | 0 | 51.260 | 3.7182 |
| 2.00 | 1 | 0 | 45.220 | 3.8957 |
| 2.50 | 1 | 0 | 42.979 | 3.8252 |
| 3.00 | 1 | 0 | 43.157 | 4.3797 |

## ST Avoidance Evidence

Passing ST manifests:

- `code/outputs/robustness/st_seed7_obs3_0p5_to_3p0_alat2/manifest.csv`
- `code/outputs/robustness/st_seed13_obs2_v3p0_alat2/manifest.csv`

Obstacle manifests:

- `maps/generated_static_obstacles/robust_obs3_seed7.json`: 3 obstacles, all `size_m = 0.5`
- `maps/generated_static_obstacles/robust_obs2_seed13.json`: 2 obstacles, all `size_m = 0.5`

| Speed (m/s) | Pass | Collision Events | Duration (s) | Avoid Rows | Min Clearance (m) | Obstacle Manifest |
| --- | --- | --- | --- | --- | --- | --- |
| 0.50 | 1 | 0 | 133.820 | 20 | 0.107 | `robust_obs3_seed7.json` |
| 1.00 | 1 | 0 | 61.912 | 3 | 0.571 | `robust_obs3_seed7.json` |
| 1.50 | 1 | 0 | 43.477 | 2 | 0.587 | `robust_obs3_seed7.json` |
| 2.00 | 1 | 0 | 40.992 | 4 | 0.575 | `robust_obs3_seed7.json` |
| 2.50 | 1 | 0 | 40.956 | 7 | 0.577 | `robust_obs3_seed7.json` |
| 3.00 | 1 | 0 | 41.457 | 4 | 0.554 | `robust_obs2_seed13.json` |

## Visualizations

All MP4 files were checked with `ffprobe`; each is 960x720, 18 seconds, 360 frames.

- `code/outputs/robustness/videos/st_v0p5_seed7_obs3.mp4`
- `code/outputs/robustness/videos/st_v1p0_seed7_obs3.mp4`
- `code/outputs/robustness/videos/st_v1p5_seed7_obs3.mp4`
- `code/outputs/robustness/videos/st_v2p0_seed7_obs3.mp4`
- `code/outputs/robustness/videos/st_v2p5_seed7_obs3.mp4`
- `code/outputs/robustness/videos/st_v3p0_seed13_obs2.mp4`

## Final Checks

Commands run in the Docker container:

```bash
cd /sim_ws/src/f1tenth_gym_ros
python3 -m compileall code/lqr_sweep/validate_ros.py code/lqr_sweep/sim_harness.py code/run_robustness_validation.py code/render_st_run_video.py launch/pnc_sim_launch.py
python3 -m pytest -q test/test_st_corridor_offline.py

cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select f1tenth_gym_ros
```

Results:

- `pytest`: 2 passed
- `colcon build`: 1 package finished

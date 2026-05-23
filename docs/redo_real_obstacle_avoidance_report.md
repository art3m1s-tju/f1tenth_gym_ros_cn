# Redo Real Obstacle Avoidance Report

Date: 2026-05-23
Branch: `codex/redo-real-st-obstacle-avoidance`

## What Was Wrong

The previous validation relied on simulator `ego_collision` and ST log rows, but the rendered ego trajectory could still pass through the 0.5 m obstacle squares. A direct geometry audit confirmed this:

- Old 0.5 m/s run: 910 ego-center samples inside an obstacle square.
- Old 1.0 m/s run: 225 ego-center samples inside an obstacle square.

The redo adds a hard geometry gate: any ego-center sample inside a square obstacle fails the run, and the minimum center-to-square-edge clearance must be at least 0.05 m.

## Fixes

- Created new branch: `codex/redo-real-st-obstacle-avoidance`.
- ST planner now accepts `static_obstacle_manifest_path` and uses the generated random obstacle manifest as a static obstacle source.
- ST candidate path generation shifts laterally earlier before obstacles.
- When all candidates are conservative-collision candidates, the planner selects the lowest-risk moving avoidance path instead of stopping indefinitely.
- Avoidance speed no longer collapses to minimum speed for conservative internal clearance estimates.
- Batch validation now records:
  - `obstacle_center_hits`
  - `min_obstacle_center_clearance_m`

## Final ST Evidence

All final ST runs use `maps/generated_static_obstacles/robust_obs3_seed7.json`, which contains 3 random obstacle blocks, each `size_m = 0.5`.

| Speed (m/s) | Pass | Duration (s) | Collision Events | Obstacle Hits | Min Center Clearance (m) | Avoid Rows | Manifest |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.50 | 1 | 106.634 | 0 | 0 | 0.253 | 960 | `redo_final_st_seed7_obs3_geom_fast/manifest.csv` |
| 1.00 | 1 | 59.420 | 0 | 0 | 0.255 | 555 | `redo_final_st_seed7_obs3_geom_fast/manifest.csv` |
| 1.50 | 1 | 52.436 | 0 | 0 | 0.230 | 448 | `redo_final_st_seed7_obs3_geom_fast/manifest.csv` |
| 2.00 | 1 | 52.725 | 0 | 0 | 0.128 | 324 | `redo_final_st_seed7_obs3_geom_fast/manifest.csv` |
| 2.50 | 1 | 52.452 | 0 | 0 | 0.078 | 212 | `redo_final_st_seed7_obs3_v2p5_avoid1p5/manifest.csv` |
| 3.00 | 1 | 45.744 | 0 | 0 | 0.325 | 105 | `redo_final_st_seed7_obs3_v3p0_avoid2p0/manifest.csv` |

## LQR Baseline Evidence

The current LQR evidence remains valid after the redo:

- `code/outputs/robustness/lqr_lowspeed_alat2p0_clean/manifest.csv`
- `code/outputs/robustness/lqr_highspeed_alat2p0/manifest.csv`

Verified speeds: 0.50, 1.00, 1.50, 2.00, 2.50, 3.00 m/s.

## Final Videos

All videos were checked with `ffprobe`; each is 960x720, 18 seconds, 360 frames.

- `code/outputs/robustness/redo_videos/st_v0p5_seed7_obs3_realavoid.mp4`
- `code/outputs/robustness/redo_videos/st_v1p0_seed7_obs3_realavoid.mp4`
- `code/outputs/robustness/redo_videos/st_v1p5_seed7_obs3_realavoid.mp4`
- `code/outputs/robustness/redo_videos/st_v2p0_seed7_obs3_realavoid.mp4`
- `code/outputs/robustness/redo_videos/st_v2p5_seed7_obs3_realavoid.mp4`
- `code/outputs/robustness/redo_videos/st_v3p0_seed7_obs3_realavoid.mp4`

## Final Checks

Container checks:

```bash
python3 -m compileall code launch f1tenth_gym_ros test
python3 -m pytest -q test/test_st_corridor_offline.py
colcon build --symlink-install --packages-select f1tenth_gym_ros
```

Results:

- `pytest`: 2 passed
- `colcon build`: 1 package finished
- Geometry audit: `AUDIT_REAL_AVOID_OK`

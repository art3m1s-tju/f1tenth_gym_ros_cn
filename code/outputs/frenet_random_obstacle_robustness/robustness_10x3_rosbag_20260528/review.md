# Frenet Random Obstacle Robustness Review

Batch: `robustness_10x3_rosbag_20260528`

Local output root:

```text
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528
```

## Experiment Setup

Command:

```bash
./run_frenet_random_obstacle_robustness.sh \
  --trials 10 \
  --laps 3 \
  --timeout 240 \
  --stuck-duration 3.0 \
  --target-speed 3.0 \
  --avoidance-speed 1.2 \
  --obstacle-count 2 \
  --obstacle-size 0.4 \
  --seed-start 0 \
  --batch-name robustness_10x3_rosbag_20260528
```

Pass criterion: each seed must complete at least 3 laps with no collision and no stop/stall longer than the configured `3.0 s` stuck threshold.

Data recorded for every seed:

- `summary.csv` and `summary.json` at the batch root.
- Per-seed `tracking.csv`, `global_trajectory.csv`, `launch.log`, generated map files, and full `ros2 bag record -a` output.
- All 10 seeds produced rosbag metadata and sqlite `.db3` data. Total batch size is about `13G`.

Important fixed settings from `summary.json`:

| Parameter | Value |
|---|---:|
| target_speed | `3.0 m/s` |
| avoidance_speed | `1.2 m/s` |
| obstacle_count | `2` |
| obstacle_size | `0.4 m` |
| timeout | `240 s` |
| stuck_duration | `3.0 s` |
| v_min / v_max / v_step | `0.84 / 3.75 / 0.75 m/s` |
| d_step | `0.35 m` |
| trajectory_dt | `0.1 s` |
| activation_max_m | `10.6 m` |
| activation_path_margin_m | `1.25 m` |
| activation_reaction_s | `1.2 s` |
| approach_extra_m | `0.75 m` |
| max_published_path_length_m | `6.6 m` |
| held_path_replan_clearance_m | `0.10 m` |
| held_path_min_remaining_m | `0.90 m` |
| centerline_return_lookahead_m | `3.5 m` |
| grid_forward_m | `20.0 m` |

Planner launch parameters observed in `launch.log` also included:

- `safe_clearance_m:=0.30`
- `min_clearance_m:=0.06`
- `corridor_radius_m:=0.20`
- `corridor_sample_step_m:=0.05`
- `path_collision_sample_step_m:=0.05`
- `footprint_front_m:=0.45`
- `footprint_rear_m:=0.05`
- `max_heading_jump:=0.85`
- `min_progress_step_m:=0.20`
- `reuse_last_candidate_timeout_s:=2.000`
- `candidate_profile_consistency_weight:=20.0`
- `candidate_profile_max_jump_m:=0.35`
- `candidate_channel_memory_timeout_s:=1.5`
- `stop_path_length_m:=2.0`

## Result Summary

Overall result: `8 / 10` seeds passed. Success rate: `80%`.

| Seed | Result | Reason | Laps | No-safe stops | Collision count | Longest zero speed limit | Longest low-speed run | Rosbag size |
|---:|---|---|---:|---:|---:|---:|---:|---:|
| 000 | FAIL | collision | 0 | 1 | 1 | `0.048 s` | `0.000 s` | `415 MB` |
| 001 | PASS | - | 3 | 0 | 0 | `0.000 s` | `0.000 s` | `1186 MB` |
| 002 | PASS | - | 3 | 0 | 0 | `0.000 s` | `0.000 s` | `1197 MB` |
| 003 | PASS | - | 3 | 0 | 0 | `0.000 s` | `0.000 s` | `1155 MB` |
| 004 | PASS | transient stop | 3 | 5 | 0 | `1.360 s` | `1.284 s` | `1240 MB` |
| 005 | PASS | transient stop | 3 | 2 | 0 | `0.417 s` | `0.000 s` | `1106 MB` |
| 006 | PASS | - | 3 | 0 | 0 | `0.000 s` | `0.000 s` | `1075 MB` |
| 007 | PASS | - | 3 | 0 | 0 | `0.000 s` | `0.000 s` | `1068 MB` |
| 008 | FAIL | stuck_zero_speed_limit | 1 | 279 | 0 | `197.035 s` | `196.472 s` | `3246 MB` |
| 009 | PASS | transient stop | 3 | 1 | 0 | `0.480 s` | `0.000 s` | `1178 MB` |

## Failure Case: Seed 000 Collision

Obstacle centers:

- `(-7.7276745, -4.1904405)`
- `(8.9586213, 3.0543932)`

Observed failure:

- `launch.log` reports `Ego collision detected at (9.363, 2.530)`.
- Final tracked pose was approximately `(10.108, 3.255)`, about `1.167 m` from the obstacle center `(8.959, 3.054)`.
- The planner was operating with very small reported clearances shortly before the collision:
  - held path clearance values around `0.09-0.18 m`.
  - one reuse event at `clearance=0.09 m` when no safe candidate was available.
  - channel-consistent candidate selection sometimes chose a lower-clearance temporal/channel-consistent path over the raw lower-cost candidate.
- At the collision transition, `launch.log` shows all candidates rejected:
  - `total=110`
  - `safe=0`
  - `collision_reject=110`
  - `best_clearance=0.00m`
  - `occupied_cells=47266`

Tracking evidence in the `1779904970-1779904975` window:

- `max_abs_e_y = 1.625 m`
- `max_abs_e_psi = 2.483 rad`
- `delta_cmd` saturated at `0.36 rad`
- `v_path_signed` included negative samples, with `negative_v_path_count=51` in the trial summary.
- Local speed limit was still mostly `1.2 m/s` before the immediate collision aftermath.

Interpretation:

The collision was not a missing-data failure. The map, scan, planner, controller, and rosbag were all running. The weak point is robustness near a tight obstacle interaction: candidate reuse/channel consistency can keep the car committed to a path with marginal clearance, then once tracking error grows, the next cycles have no safe candidate and the vehicle has already entered an unrecoverable region.

## Failure Case: Seed 008 Long Stop Deadlock

Obstacle centers:

- `(9.0070681, -2.9905527)`
- `(9.3953596, 2.3834458)`

Observed failure:

- Trial stopped by the harness as `stuck_zero_speed_limit`.
- Completed only `1` lap.
- Final tracked pose was approximately `(8.440, -3.031)`, about `0.568 m` from the nearest obstacle center `(9.007, -2.991)`.
- Longest zero speed limit interval was `197.035 s`, from approximately `1779905602.291` to `1779905799.327`.
- `launch.log` repeatedly reports `No safe Frenet candidate; publishing stop path`.
- During the long deadlock, planner state stayed almost fixed:
  - `s=34.84`
  - `d=0.30`
  - `s_dot=0.00`
  - `speed=0.00`
- Repeated candidate statistics in the deadlock:
  - `total=110`
  - `safe=0`
  - `collision_reject=110`
  - `best_clearance=0.00m`
  - `occupied_cells` around `43940-43966`

Rosbag evidence:

- `seed_008` bag duration: about `241.7 s`
- message count: `395667`
- key recorded topics include `/scan`, `/ego_racecar/odom`, `/drive`, `/local_trajectory`, `/local_trajectory_speed_limit`, `/frenet/debug/candidates`, `/global_trajectory`, `/map`, `/tf`, `/tf_static`, and `/rosout`.

Interpretation:

This is a deadlock/recovery weakness. The car stops close enough to an obstacle that every forward Frenet candidate collides, so the planner keeps publishing a zero-speed stop path indefinitely. There is no recovery behavior that backs away, relaxes/changes the search topology, or explicitly generates a reverse/escape maneuver.

## Passed But Marginal Cases

Seeds `004`, `005`, and `009` completed 3 laps, but they entered no-safe fallback briefly:

- Seed `004`: `5` no-safe stops, longest zero speed limit `1.360 s`, longest low-speed run `1.284 s`.
- Seed `005`: `2` no-safe stops, longest zero speed limit `0.417 s`.
- Seed `009`: `1` no-safe stop, longest zero speed limit `0.480 s`.

These are below the configured `3.0 s` stuck threshold, so they pass this test. They still show that the current planner can enter all-collision/no-safe cycles even when the run eventually recovers.

## Robustness Issues Found

1. Tight-obstacle interactions have too little margin in practice.

   Seed `000` shows clearances around `0.09-0.18 m` shortly before collision, while the vehicle then experiences growing lateral and heading error. The planner can still reuse or hold a path near the clearance threshold long enough for tracking error to make the path unsafe.

2. Candidate continuity can choose a worse-clearance path.

   The logs show channel-consistent selection choosing temporal/channel-consistent candidates with lower clearance than the raw candidate. This helps avoid oscillation, but it can also preserve a poor side choice near an obstacle.

3. Stop fallback has no escape mode.

   Seed `008` demonstrates that once stopped near an obstacle, all forward candidates remain collision-rejected. The planner publishes stop paths for the rest of the trial and never recovers.

4. The pass/fail boundary is sensitive to transient no-safe episodes.

   Seeds `004`, `005`, and `009` passed only because the no-safe periods were shorter than `3.0 s`. The behavior is still a warning sign because the planner temporarily had no usable path.

5. Candidate rejection diagnostics are now good enough to locate the issue, but not yet enough to prove the best recovery action.

   The logs expose total/safe/collision/clearance/progress counts and state values. The rosbag contains the full topic stream needed for replay, but the review still needs visual replay or offline candidate reconstruction before choosing exact algorithm changes.

## Recommendations For Next Algorithm Iteration

No planning/control algorithm was changed after this robustness batch. Based on this fixed-run evidence, the next iteration should focus on:

1. Add a deadlock recovery mode for stop-near-obstacle states.

   Options include controlled reverse/backoff, temporary wider lateral candidate search, or a dedicated escape path generator. Seed `008` is the primary regression test for this.

2. Make held-path reuse stricter when clearance is near threshold and tracking error is growing.

   Seed `000` suggests that clearance alone is not enough. A combined gate using clearance, lateral error, heading error, and obstacle proximity should decide whether a held path is still acceptable.

3. Rebalance channel consistency against clearance.

   Channel memory should not dominate when the clearance gap between the raw candidate and the channel-consistent candidate is safety-relevant.

4. Treat transient no-safe events as a separate robustness metric.

   Even successful seeds with brief no-safe fallback should be tracked in CI-style experiments, because they are early indicators of future collision/deadlock failures.

5. Replay seed `000` and seed `008` rosbags in RViz before changing parameters.

   The most useful bags are:

   ```text
   code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_000/logs/rosbag/seed_000
   code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_008/logs/rosbag/seed_008
   ```

## Data Paths

Batch summary:

```text
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/summary.csv
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/summary.json
```

Failure logs:

```text
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_000/logs/launch.log
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_000/logs/tracking.csv
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_008/logs/launch.log
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_008/logs/tracking.csv
```

Rosbag roots:

```text
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_000/logs/rosbag/seed_000
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_001/logs/rosbag/seed_001
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_002/logs/rosbag/seed_002
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_003/logs/rosbag/seed_003
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_004/logs/rosbag/seed_004
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_005/logs/rosbag/seed_005
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_006/logs/rosbag/seed_006
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_007/logs/rosbag/seed_007
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_008/logs/rosbag/seed_008
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_009/logs/rosbag/seed_009
```

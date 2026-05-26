#!/usr/bin/env python3
"""Batch robustness validation for Frenet static-obstacle avoidance."""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lqr_sweep.generate_static_obstacle_test_map import write_obstacle_map


CONTAINER_PKG = Path("/sim_ws/src/f1tenth_gym_ros")


@dataclass(frozen=True)
class FrenetPreset:
    geometry_target_speed: float
    v_min: float
    v_max: float
    v_step: float
    d_step: float
    trajectory_dt: float
    grid_forward_m: float
    max_hold_age_s: float
    hold_replan_clearance_m: float
    hold_min_remaining_m: float
    reuse_timeout_s: float
    activation_max_m: float
    activation_reaction_s: float
    approach_extra_m: float
    candidate_consistency_weight: float
    candidate_side_switch_penalty: float
    candidate_side_deadband_m: float


@dataclass
class TrackingSummary:
    completed_laps: int = 0
    zero_limit_runs: int = 0
    low_speed_stuck_runs: int = 0
    max_zero_limit_duration_s: float = 0.0
    max_low_speed_duration_s: float = 0.0
    negative_v_path_count: int = 0
    row_count: int = 0
    duration_s: float = 0.0
    max_v_actual: float = 0.0
    max_v_cmd: float = 0.0
    min_local_speed_limit: float = math.nan
    max_local_speed_limit: float = math.nan


@dataclass
class LaunchSummary:
    ego_collision_count: int = 0
    no_safe_stop_count: int = 0
    zero_limit_log_count: int = 0


@dataclass
class TrialResult:
    seed: int
    success: bool
    failure_reason: str
    completed_laps: int
    obstacle_count: int
    map_prefix: str
    tracking_log: str
    launch_log: str
    duration_s: float
    zero_limit_runs: int
    low_speed_stuck_runs: int
    max_zero_limit_duration_s: float
    max_low_speed_duration_s: float
    negative_v_path_count: int
    ego_collision_count: int
    no_safe_stop_count: int
    max_v_actual: float
    max_v_cmd: float


def compute_frenet_preset(target_speed: float, avoidance_speed: float) -> FrenetPreset:
    target = float(target_speed)
    avoidance = float(avoidance_speed)
    geom_target = max(1.2, target, avoidance)
    v_min = max(0.6, min(avoidance * 0.7, target * 0.5, geom_target))
    v_max = max(1.8, geom_target * 1.25, avoidance * 1.5)
    return FrenetPreset(
        geometry_target_speed=geom_target,
        v_min=v_min,
        v_max=v_max,
        v_step=0.75 if target >= 2.0 else 0.6,
        d_step=0.35 if target >= 2.0 else 0.3,
        trajectory_dt=0.10 if target >= 2.0 else 0.05,
        grid_forward_m=max(10.0, target * 6.0 + 2.0),
        max_hold_age_s=0.80 if target >= 2.0 else 0.45,
        hold_replan_clearance_m=0.10 if target >= 2.0 else 0.20,
        hold_min_remaining_m=max(2.0, target * 1.0),
        reuse_timeout_s=2.0 if target >= 2.0 else 1.0,
        activation_max_m=max(8.0, 2.5 + target * 2.7),
        activation_reaction_s=1.2 if target >= 2.0 else 1.0,
        approach_extra_m=max(1.0, target * target / 4.0),
        candidate_consistency_weight=10.0 if target >= 2.0 else 8.0,
        candidate_side_switch_penalty=35.0 if target >= 2.0 else 25.0,
        candidate_side_deadband_m=0.20,
    )


def _float_value(row: dict[str, str], key: str, default: float = math.nan) -> float:
    try:
        value = row.get(key, "")
        return float(value) if value != "" else default
    except (TypeError, ValueError):
        return default


def count_laps_from_tracking(
    rows: list[dict[str, str]],
    start_x: float,
    start_y: float,
    threshold_m: float = 0.5,
) -> int:
    if len(rows) < 500:
        return 0
    indices: list[int] = []
    for row in rows:
        idx = _float_value(row, "closest_idx")
        if math.isfinite(idx):
            indices.append(int(idx))
    if not indices:
        return 0

    max_path_idx = max(indices)
    high_idx = 0.75 * max_path_idx
    low_idx = 0.25 * max_path_idx
    laps = 0
    previous_idx: int | None = None
    for row, idx in zip(rows, indices):
        if previous_idx is not None and previous_idx > high_idx and idx < low_idx:
            x = _float_value(row, "x")
            y = _float_value(row, "y")
            if math.isfinite(x) and math.isfinite(y):
                if math.hypot(x - start_x, y - start_y) < threshold_m:
                    laps += 1
        previous_idx = idx
    return laps


def load_reference_xy(path: Path) -> np.ndarray:
    points: list[tuple[float, float]] = []
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if "x" in row and "y" in row:
                points.append((float(row["x"]), float(row["y"])))
            else:
                points.append((float(row["pos_x"]), float(row["pos_y"])))
    if len(points) < 3:
        raise ValueError(f"reference path needs at least 3 points: {path}")
    return np.asarray(points, dtype=float)


def count_laps_from_positions(rows: list[dict[str, str]], reference_xy: np.ndarray) -> int:
    if len(rows) < 100 or len(reference_xy) < 3:
        return 0
    positions: list[tuple[float, float]] = []
    for row in rows:
        x = _float_value(row, "x")
        y = _float_value(row, "y")
        if math.isfinite(x) and math.isfinite(y):
            positions.append((x, y))
    if len(positions) < 100:
        return 0

    ref = np.asarray(reference_xy, dtype=float)
    ref_count = len(ref)
    nearest_indices: list[int] = []
    for position in positions:
        delta = ref - np.asarray(position, dtype=float)
        nearest_indices.append(int(np.argmin(np.einsum("ij,ij->i", delta, delta))))

    wraps = 0
    previous_idx = nearest_indices[0]
    start_unwrapped = previous_idx
    max_unwrapped = start_unwrapped
    half_count = 0.5 * ref_count
    for idx in nearest_indices[1:]:
        delta_idx = idx - previous_idx
        if delta_idx < -half_count:
            wraps += 1
        elif delta_idx > half_count:
            wraps -= 1
        unwrapped_idx = idx + wraps * ref_count
        max_unwrapped = max(max_unwrapped, unwrapped_idx)
        previous_idx = idx

    progress_laps = (max_unwrapped - start_unwrapped) / float(ref_count)
    return max(0, int(math.floor(progress_laps + 1e-6)))


def _count_runs(
    rows: list[dict[str, str]],
    predicate,
    min_duration_s: float,
) -> int:
    if not rows:
        return 0
    start_time = _float_value(rows[0], "time", 0.0)
    active_start: float | None = None
    active_end = start_time
    runs = 0
    for row in rows:
        t = _float_value(row, "time")
        if not math.isfinite(t):
            continue
        is_active = predicate(row, t, start_time)
        if is_active and active_start is None:
            active_start = t
        if is_active:
            active_end = t
        if not is_active and active_start is not None:
            if active_end - active_start >= min_duration_s:
                runs += 1
            active_start = None
    if active_start is not None and active_end - active_start >= min_duration_s:
        runs += 1
    return runs


def _max_run_duration(rows: list[dict[str, str]], predicate) -> float:
    if not rows:
        return 0.0
    start_time = _float_value(rows[0], "time", 0.0)
    active_start: float | None = None
    active_end = start_time
    max_duration = 0.0
    for row in rows:
        t = _float_value(row, "time")
        if not math.isfinite(t):
            continue
        is_active = predicate(row, t, start_time)
        if is_active and active_start is None:
            active_start = t
        if is_active:
            active_end = t
        if not is_active and active_start is not None:
            max_duration = max(max_duration, active_end - active_start)
            active_start = None
    if active_start is not None:
        max_duration = max(max_duration, active_end - active_start)
    return max_duration


def summarize_tracking_log(
    log_path: Path,
    start_x: float,
    start_y: float,
    reference_xy: np.ndarray | None = None,
) -> TrackingSummary:
    if not log_path.exists():
        return TrackingSummary()
    with log_path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return TrackingSummary()

    times = [_float_value(row, "time") for row in rows]
    finite_times = [value for value in times if math.isfinite(value)]
    v_actual = [_float_value(row, "v_actual", 0.0) for row in rows]
    v_cmd = [_float_value(row, "v_cmd", 0.0) for row in rows]
    speed_limits = [
        _float_value(row, "local_speed_limit")
        for row in rows
        if math.isfinite(_float_value(row, "local_speed_limit"))
    ]
    zero_limit_predicate = lambda row, t, start: (
        t > start + 1.0 and _float_value(row, "local_speed_limit") == 0.0
    )
    low_speed_predicate = lambda row, t, start: (
        t > start + 2.0
        and abs(_float_value(row, "v_actual", 0.0)) < 0.25
        and abs(_float_value(row, "v_cmd", 0.0)) < 0.35
    )
    zero_limit_runs = _count_runs(rows, zero_limit_predicate, min_duration_s=0.5)
    low_speed_stuck_runs = _count_runs(rows, low_speed_predicate, min_duration_s=1.0)
    negative_v_path_count = sum(
        1 for row in rows if _float_value(row, "v_path_signed", 0.0) < -0.05
    )
    return TrackingSummary(
        completed_laps=(
            count_laps_from_positions(rows, reference_xy)
            if reference_xy is not None
            else count_laps_from_tracking(rows, start_x, start_y)
        ),
        zero_limit_runs=zero_limit_runs,
        low_speed_stuck_runs=low_speed_stuck_runs,
        max_zero_limit_duration_s=_max_run_duration(rows, zero_limit_predicate),
        max_low_speed_duration_s=_max_run_duration(rows, low_speed_predicate),
        negative_v_path_count=negative_v_path_count,
        row_count=len(rows),
        duration_s=(max(finite_times) - min(finite_times)) if finite_times else 0.0,
        max_v_actual=max(v_actual) if v_actual else 0.0,
        max_v_cmd=max(v_cmd) if v_cmd else 0.0,
        min_local_speed_limit=min(speed_limits) if speed_limits else math.nan,
        max_local_speed_limit=max(speed_limits) if speed_limits else math.nan,
    )


def summarize_launch_log(log_path: Path) -> LaunchSummary:
    if not log_path.exists():
        return LaunchSummary()
    text = log_path.read_text(encoding="utf-8", errors="replace")
    return LaunchSummary(
        ego_collision_count=text.count("Ego collision detected"),
        no_safe_stop_count=text.count("No safe Frenet candidate; publishing stop path"),
        zero_limit_log_count=text.count("Local trajectory speed limit set to 0.00m/s"),
    )


def build_launch_cmd(
    *,
    repo_root: Path,
    map_prefix: Path,
    track_csv: Path,
    trajectory_csv: Path,
    tracking_log: Path,
    target_speed: float,
    avoidance_speed: float,
    start_x: float,
    start_y: float,
    start_theta: float,
    preset: FrenetPreset,
) -> list[str]:
    launch_file = repo_root / "launch" / "pnc_sim_launch.py"
    return [
        "ros2",
        "launch",
        str(launch_file),
        "enable_rviz:=false",
        "enable_frenet_planner:=true",
        f"map_path:={map_prefix}",
        f"track_csv:={track_csv}",
        f"trajectory_csv:={trajectory_csv}",
        "trajectory_mode:=centerline",
        f"sx:={start_x}",
        f"sy:={start_y}",
        f"stheta:={start_theta}",
        f"target_speed:={target_speed}",
        "min_speed:=0.20",
        "max_lateral_accel:=1.5",
        "max_accel:=1.0",
        "max_decel:=2.0",
        "curvature_speed_lookahead_m:=1.5",
        "local_speed_limit_timeout_s:=1.0",
        "frenet_reference_closed_loop:=true",
        "frenet_centerline_speed_limit_mps:=-1.0",
        f"frenet_avoidance_speed_limit_mps:={avoidance_speed}",
        "frenet_stop_speed_limit_mps:=0.0",
        f"frenet_target_speed:={preset.geometry_target_speed:.3f}",
        f"frenet_v_min:={preset.v_min:.3f}",
        f"frenet_v_max:={preset.v_max:.3f}",
        f"frenet_v_step:={preset.v_step:.3f}",
        "frenet_t_min:=4.0",
        "frenet_t_max:=6.0",
        "frenet_t_step:=2.0",
        f"frenet_trajectory_dt:={preset.trajectory_dt:.3f}",
        "frenet_d_min:=-1.8",
        "frenet_d_max:=1.8",
        f"frenet_d_step:={preset.d_step:.3f}",
        "frenet_max_heading_jump:=0.85",
        "frenet_grid_inflation_radius_m:=0.18",
        f"frenet_grid_forward_m:={preset.grid_forward_m:.3f}",
        "frenet_grid_half_width_m:=3.2",
        "frenet_max_curvature:=1.1",
        "frenet_corridor_radius_m:=0.16",
        "frenet_corridor_sample_step_m:=0.05",
        "frenet_path_collision_sample_step_m:=0.05",
        "frenet_footprint_front_m:=0.45",
        "frenet_footprint_rear_m:=0.05",
        "frenet_safe_clearance_m:=0.30",
        "frenet_min_clearance_m:=0.06",
        "frenet_published_path_lookahead_m:=0.25",
        "frenet_min_path_publish_interval_s:=0.25",
        "frenet_path_republish_distance_m:=0.50",
        "frenet_path_republish_min_remaining_m:=2.0",
        "frenet_centerline_return_lookahead_m:=5.0",
        "frenet_centerline_threat_lookahead_m:=8.0",
        "frenet_centerline_threat_corridor_radius_m:=0.22",
        "frenet_activation_min_lookahead_m:=3.0",
        f"frenet_activation_max_lookahead_m:={preset.activation_max_m:.3f}",
        "frenet_activation_base_lookahead_m:=2.2",
        f"frenet_activation_reaction_time_s:={preset.activation_reaction_s:.3f}",
        "frenet_activation_decel_mps2:=2.0",
        f"frenet_approach_slowdown_extra_m:={preset.approach_extra_m:.3f}",
        f"frenet_reuse_last_candidate_timeout_s:={preset.reuse_timeout_s:.3f}",
        f"frenet_max_held_path_age_s:={preset.max_hold_age_s:.3f}",
        f"frenet_held_path_replan_clearance_m:={preset.hold_replan_clearance_m:.3f}",
        f"frenet_held_path_min_remaining_m:={preset.hold_min_remaining_m:.3f}",
        (
            "frenet_candidate_lateral_consistency_weight:="
            f"{preset.candidate_consistency_weight:.3f}"
        ),
        f"frenet_candidate_side_switch_penalty:={preset.candidate_side_switch_penalty:.3f}",
        f"frenet_candidate_side_deadband_m:={preset.candidate_side_deadband_m:.3f}",
        f"log_path:={tracking_log}",
    ]


def terminate_process_group(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait(timeout=5)
    except ProcessLookupError:
        pass


def failure_reason(
    tracking: TrackingSummary,
    launch: LaunchSummary,
    required_laps: int,
    timed_out: bool,
    tracking_log: Path,
    stuck_duration_s: float,
) -> str:
    if not tracking_log.exists() or tracking.row_count == 0:
        return "no_log"
    if launch.ego_collision_count > 0:
        return "collision"
    if tracking.negative_v_path_count > 0:
        return "negative_velocity"
    if tracking.max_zero_limit_duration_s >= stuck_duration_s:
        return "stuck_zero_speed_limit"
    if tracking.max_low_speed_duration_s >= stuck_duration_s:
        return "low_speed_stuck"
    if tracking.completed_laps < required_laps:
        prefix = "timeout" if timed_out else "incomplete"
        return f"{prefix}_laps_{tracking.completed_laps}_of_{required_laps}"
    return ""


def run_trial(args: argparse.Namespace, batch_dir: Path, seed: int, preset: FrenetPreset) -> TrialResult:
    trial_name = f"seed_{seed:03d}"
    trial_dir = batch_dir / "trials" / trial_name
    maps_dir = trial_dir / "map"
    logs_dir = trial_dir / "logs"
    maps_dir.mkdir(parents=True, exist_ok=True)
    logs_dir.mkdir(parents=True, exist_ok=True)

    map_prefix = maps_dir / trial_name
    tracking_log = logs_dir / "tracking.csv"
    launch_log = logs_dir / "launch.log"
    trajectory_csv = logs_dir / "global_trajectory.csv"
    for path in (tracking_log, launch_log, trajectory_csv):
        if path.exists():
            path.unlink()

    obstacle_summary = write_obstacle_map(
        base_yaml=args.base_yaml.resolve(),
        trajectory_csv=args.obstacle_trajectory_csv.resolve(),
        output_prefix=map_prefix.resolve(),
        obstacle_count=max(1, int(args.obstacle_count)),
        obstacle_size_m=max(0.05, float(args.obstacle_size_m)),
        seed=int(seed),
        min_start_distance_m=max(0.0, float(args.min_start_distance_m)),
        min_separation_m=max(0.0, float(args.min_separation_m)),
        obstacle_distances_m=None,
    )
    reference_xy = load_reference_xy(args.obstacle_trajectory_csv.resolve())

    launch_cmd = build_launch_cmd(
        repo_root=args.repo_root,
        map_prefix=map_prefix.resolve(),
        track_csv=args.track_csv.resolve(),
        trajectory_csv=trajectory_csv.resolve(),
        tracking_log=tracking_log.resolve(),
        target_speed=args.target_speed,
        avoidance_speed=args.avoidance_speed,
        start_x=args.sx,
        start_y=args.sy,
        start_theta=args.stheta,
        preset=preset,
    )

    print("\n" + "=" * 72)
    print(f"Trial {trial_name}: map={map_prefix}")
    print(
        "  obstacles="
        + ", ".join(
            f"({obs['center_x_m']:.2f},{obs['center_y_m']:.2f})"
            for obs in obstacle_summary["obstacles"]
        )
    )
    start_time = time.time()
    timed_out = False
    with launch_log.open("w", encoding="utf-8") as log_file:
        proc = subprocess.Popen(
            launch_cmd,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        try:
            while time.time() - start_time < args.timeout:
                time.sleep(args.poll_interval)
                launch_summary = summarize_launch_log(launch_log)
                if launch_summary.ego_collision_count > 0:
                    print("  collision detected; stopping trial")
                    break
                tracking_summary = summarize_tracking_log(
                    tracking_log,
                    args.sx,
                    args.sy,
                    reference_xy,
                )
                if tracking_summary.completed_laps >= args.laps:
                    print(f"  completed {tracking_summary.completed_laps} laps; stopping trial")
                    break
                if proc.poll() is not None:
                    print(f"  launch exited early with code {proc.returncode}")
                    break
            else:
                timed_out = True
                print("  timeout reached; stopping trial")
        finally:
            terminate_process_group(proc)

    tracking = summarize_tracking_log(tracking_log, args.sx, args.sy, reference_xy)
    launch = summarize_launch_log(launch_log)
    reason = failure_reason(
        tracking,
        launch,
        max(1, int(args.laps)),
        timed_out,
        tracking_log,
        args.stuck_duration,
    )
    success = reason == ""
    print(
        f"  result={'PASS' if success else 'FAIL'} "
        f"laps={tracking.completed_laps}/{args.laps} reason={reason or 'ok'}"
    )
    return TrialResult(
        seed=seed,
        success=success,
        failure_reason=reason,
        completed_laps=tracking.completed_laps,
        obstacle_count=int(obstacle_summary["obstacle_count"]),
        map_prefix=str(map_prefix),
        tracking_log=str(tracking_log),
        launch_log=str(launch_log),
        duration_s=tracking.duration_s,
        zero_limit_runs=tracking.zero_limit_runs,
        low_speed_stuck_runs=tracking.low_speed_stuck_runs,
        max_zero_limit_duration_s=tracking.max_zero_limit_duration_s,
        max_low_speed_duration_s=tracking.max_low_speed_duration_s,
        negative_v_path_count=tracking.negative_v_path_count,
        ego_collision_count=launch.ego_collision_count,
        no_safe_stop_count=launch.no_safe_stop_count,
        max_v_actual=tracking.max_v_actual,
        max_v_cmd=tracking.max_v_cmd,
    )


def write_outputs(batch_dir: Path, args: argparse.Namespace, preset: FrenetPreset, results: list[TrialResult]) -> None:
    summary_csv = batch_dir / "summary.csv"
    fieldnames = list(asdict(results[0]).keys()) if results else list(TrialResult.__dataclass_fields__.keys())
    with summary_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for result in results:
            writer.writerow(asdict(result))

    successes = sum(1 for result in results if result.success)
    summary_json = batch_dir / "summary.json"
    summary = {
        "success_rate": successes / len(results) if results else 0.0,
        "successful_trials": successes,
        "total_trials": len(results),
        "args": vars(args),
        "preset": asdict(preset),
        "results": [asdict(result) for result in results],
    }
    serializable = json.loads(json.dumps(summary, default=str))
    summary_json.write_text(json.dumps(serializable, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print("\n" + "=" * 72)
    print(f"Robustness summary: {successes}/{len(results)} passed")
    print(f"Success rate: {summary['success_rate']:.3f}")
    print(f"Summary CSV: {summary_csv}")
    print(f"Summary JSON: {summary_json}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=CONTAINER_PKG)
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--laps", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--poll-interval", type=float, default=2.0)
    parser.add_argument("--stuck-duration", type=float, default=3.0)
    parser.add_argument("--target-speed", type=float, default=3.0)
    parser.add_argument("--avoidance-speed", type=float, default=1.2)
    parser.add_argument("--obstacle-count", type=int, default=2)
    parser.add_argument("--obstacle-size-m", type=float, default=0.4)
    parser.add_argument("--min-start-distance-m", type=float, default=8.0)
    parser.add_argument("--min-separation-m", type=float, default=5.0)
    parser.add_argument("--sx", type=float, default=3.0)
    parser.add_argument("--sy", type=float, default=5.0)
    parser.add_argument("--stheta", type=float, default=3.1416)
    parser.add_argument(
        "--base-yaml",
        type=Path,
        default=CONTAINER_PKG / "maps" / "frenet_test_loop.yaml",
    )
    parser.add_argument(
        "--track-csv",
        type=Path,
        default=CONTAINER_PKG / "code" / "outputs" / "generated_tracks" / "frenet_test_loop_processed_track.csv",
    )
    parser.add_argument(
        "--obstacle-trajectory-csv",
        type=Path,
        default=CONTAINER_PKG / "code" / "outputs" / "generated_tracks" / "frenet_test_loop_trajectory.csv",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=CONTAINER_PKG / "code" / "outputs" / "frenet_random_obstacle_robustness",
    )
    parser.add_argument("--batch-name", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.repo_root = args.repo_root.resolve()
    batch_name = args.batch_name or datetime.now().strftime("%Y%m%d_%H%M%S")
    batch_dir = args.output_root.resolve() / batch_name
    batch_dir.mkdir(parents=True, exist_ok=True)

    preset = compute_frenet_preset(args.target_speed, args.avoidance_speed)
    print(f"Batch dir: {batch_dir}")
    print(
        "Config: "
        f"trials={args.trials}, laps={args.laps}, "
        f"target={args.target_speed}, avoidance={args.avoidance_speed}, "
        f"obstacles={args.obstacle_count}"
    )
    print(f"Preset: {json.dumps(asdict(preset), sort_keys=True)}")

    results: list[TrialResult] = []
    for offset in range(max(1, int(args.trials))):
        seed = int(args.seed_start) + offset
        result = run_trial(args, batch_dir, seed, preset)
        results.append(result)
        write_outputs(batch_dir, args, preset, results)

    return 0 if all(result.success for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())

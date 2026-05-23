#!/usr/bin/env python3
"""Batch robustness validation for LQR and ST-corridor runs."""
from __future__ import annotations

import argparse
import csv
import json
import math
import random
import subprocess
from pathlib import Path

from lqr_sweep.validate_ros import run_ros_validation


DEFAULT_SPEEDS = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
CODE_DIR = Path("/sim_ws/src/f1tenth_gym_ros/code")
REPO_DIR = Path("/sim_ws/src/f1tenth_gym_ros")


def _speed_label(speed: float) -> str:
    return f"{speed:.1f}".replace(".", "p")


def _load_summary(summary_path: Path) -> dict:
    if not summary_path.exists():
        return {}
    data = json.loads(summary_path.read_text(encoding="utf-8"))
    if not data:
        return {}
    return data[0]


def _count_collision_events(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open("r", encoding="utf-8") as f:
        return sum(int(row.get("ego_collision", "0")) > 0 for row in csv.DictReader(f))


def _count_done_events(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open("r", encoding="utf-8") as f:
        return sum(int(row.get("done", "0")) > 0 for row in csv.DictReader(f))


def _raw_tracking_duration(path: Path) -> float:
    if not path.exists():
        return math.nan
    with path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 2:
        return math.nan
    return float(rows[-1]["time"]) - float(rows[0]["time"])


def _summarize_st_log(path: Path) -> dict:
    if not path.exists():
        return {
            "st_rows": 0,
            "st_avoid_rows": 0,
            "st_blocked_rows": 0,
            "st_obstacle_rows": 0,
            "st_min_clearance_m": math.nan,
        }
    finite_clearances = []
    rows = 0
    avoid_rows = 0
    blocked_rows = 0
    obstacle_rows = 0
    with path.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            rows += 1
            avoid_rows += int(row.get("mode") == "avoid")
            blocked_rows += int(row.get("mode") == "blocked")
            obstacle_rows += int(int(row.get("obstacle_count", "0")) > 0)
            clearance = row.get("min_obstacle_clearance", "inf")
            if clearance != "inf":
                finite_clearances.append(float(clearance))
    return {
        "st_rows": rows,
        "st_avoid_rows": avoid_rows,
        "st_blocked_rows": blocked_rows,
        "st_obstacle_rows": obstacle_rows,
        "st_min_clearance_m": min(finite_clearances) if finite_clearances else math.nan,
    }


def _summarize_obstacle_geometry(tracking_log: Path, obstacle_manifest: str) -> dict:
    if not tracking_log.exists() or not obstacle_manifest:
        return {
            "obstacle_center_hits": 0,
            "min_obstacle_center_clearance_m": math.nan,
        }
    manifest_path = Path(obstacle_manifest)
    if not manifest_path.exists():
        return {
            "obstacle_center_hits": 0,
            "min_obstacle_center_clearance_m": math.nan,
        }
    obstacles = json.loads(manifest_path.read_text(encoding="utf-8")).get("obstacles", [])
    hits = 0
    min_clearance = math.inf
    with tracking_log.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            x = float(row["x"])
            y = float(row["y"])
            for obstacle in obstacles:
                half_size = 0.5 * float(obstacle.get("size_m", 0.5))
                dx = abs(x - float(obstacle["x"])) - half_size
                dy = abs(y - float(obstacle["y"])) - half_size
                if dx <= 0.0 and dy <= 0.0:
                    hits += 1
                    clearance = 0.0
                else:
                    clearance = math.hypot(max(dx, 0.0), max(dy, 0.0))
                min_clearance = min(min_clearance, clearance)
    return {
        "obstacle_center_hits": hits,
        "min_obstacle_center_clearance_m": min_clearance if math.isfinite(min_clearance) else math.nan,
    }


def _generate_obstacle_map(
    seed: int,
    obstacle_count: int,
    name: str,
    *,
    base_map_yaml: str,
    trajectory_csv: str,
    output_dir: str,
) -> tuple[str, str, str]:
    cmd = [
        "python3",
        str(CODE_DIR / "make_static_obstacle_map.py"),
        "--base-map-yaml",
        base_map_yaml,
        "--trajectory-csv",
        trajectory_csv,
        "--seed",
        str(seed),
        "--obstacle-count",
        str(obstacle_count),
        "--output-dir",
        output_dir,
        "--name",
        name,
    ]
    subprocess.run(cmd, check=True)
    root = Path(output_dir) / name
    return str(root), str(root.with_suffix(".yaml")), str(root.with_suffix(".json"))


def _default_st_lookahead(speed: float) -> float:
    if speed <= 1.0:
        return 5.5
    if speed <= 1.5:
        return 7.0
    if speed <= 2.0:
        return 8.5
    if speed <= 2.5:
        return 10.0
    return 12.0


def _default_st_avoidance_speed(speed: float) -> float:
    if speed <= 0.5:
        return 0.5
    if speed <= 1.0:
        return min(speed, 0.95)
    if speed <= 1.5:
        return min(speed, 1.4)
    if speed <= 2.0:
        return min(speed, 1.8)
    if speed <= 2.5:
        return min(speed, 1.5)
    return min(speed, 2.0)


def _write_manifest(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    path.with_suffix(".json").write_text(json.dumps(rows, indent=2), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["lqr", "st"], required=True)
    parser.add_argument("--speeds", nargs="+", type=float, default=DEFAULT_SPEEDS)
    parser.add_argument("--laps", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=220.0)
    parser.add_argument("--batch-name", required=True)
    parser.add_argument("--output-root", default=str(CODE_DIR / "outputs" / "robustness"))
    parser.add_argument("--table", default=str(CODE_DIR / "outputs" / "sweep_stadium" / "lqr_gain_table.yaml"))
    parser.add_argument("--base-map-yaml", default=str(REPO_DIR / "maps" / "my_map.yaml"))
    parser.add_argument("--track-csv", default=str(CODE_DIR / "outputs" / "csv" / "processed_track.csv"))
    parser.add_argument("--trajectory-csv", default=str(CODE_DIR / "outputs" / "csv" / "global_trajectory.csv"))
    parser.add_argument("--obstacle-output-dir", default=str(REPO_DIR / "maps" / "generated_static_obstacles"))
    parser.add_argument("--obstacle-name-prefix", default="robust")
    parser.add_argument(
        "--randomize-obstacles-per-run",
        action="store_true",
        help="For ST mode, generate a fresh derived map for every speed run with 2-3 obstacles.",
    )
    parser.add_argument("--lqr-lookahead-distance-m", type=float, default=0.0)
    parser.add_argument("--max-lateral-accel", type=float, default=2.0)
    parser.add_argument("--max-steering-angle", type=float, default=0.36)
    parser.add_argument("--max-steering-rate", type=float, default=2.0)
    parser.add_argument("--disable-steering-rate-limit", action="store_true")
    parser.add_argument(
        "--trajectory-mode",
        choices=["min_curvature", "centerline", "control_friendly"],
        default="centerline",
    )
    parser.add_argument("--centerline-smoothing", type=float, default=5.0)
    parser.add_argument("--control-friendly-alpha", type=float, default=0.56)
    parser.add_argument("--control-friendly-auto-alpha", action="store_true")
    parser.add_argument("--control-friendly-smoothing", type=float, default=2.0)
    parser.add_argument("--control-friendly-max-curvature", type=float, default=1.0)
    parser.add_argument("--control-friendly-min-clearance", type=float, default=0.35)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--obstacle-count", type=int, choices=[2, 3], default=3)
    parser.add_argument("--st-avoidance-max-speed", type=float, default=None)
    parser.add_argument("--st-min-speed", type=float, default=0.35)
    parser.add_argument("--st-lookahead-m", type=float, default=None)
    parser.add_argument("--st-inflation-margin-m", type=float, default=0.10)
    parser.add_argument("--st-min-border-clearance-m", type=float, default=0.22)
    parser.add_argument("--st-max-lateral-offset-m", type=float, default=0.65)
    parser.add_argument("--st-lateral-offset-step-m", type=float, default=0.10)
    parser.add_argument("--st-takeover-distance-m", type=float, default=0.0)
    parser.add_argument("--st-desired-obstacle-clearance-m", type=float, default=0.12)
    parser.add_argument("--use-tf-pose", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_root = Path(args.output_root) / args.batch_name
    output_root.mkdir(parents=True, exist_ok=True)

    base_map_path = str(Path(args.base_map_yaml).with_suffix(""))

    rows = []
    for speed in args.speeds:
        label = f"{args.mode}_v{_speed_label(speed)}"
        run_dir = output_root / label
        logs_dir = run_dir / "logs"
        eval_dir = run_dir / "evaluation"
        logs_dir.mkdir(parents=True, exist_ok=True)
        eval_dir.mkdir(parents=True, exist_ok=True)

        lqr_log = logs_dir / f"{label}_tracking.csv"
        st_log = logs_dir / f"{label}_st_corridor.csv"
        collision_log = logs_dir / f"{label}_collision.csv"
        launch_log = logs_dir / f"{label}_launch.log"
        evaluator_log = logs_dir / f"{label}_evaluator.log"
        avoidance_speed = (
            args.st_avoidance_max_speed
            if args.st_avoidance_max_speed is not None
            else _default_st_avoidance_speed(speed)
        )
        st_lookahead = (
            args.st_lookahead_m
            if args.st_lookahead_m is not None
            else _default_st_lookahead(speed)
        )
        map_path = base_map_path
        map_yaml = args.base_map_yaml
        obstacle_manifest = ""
        obstacle_seed = args.seed
        obstacle_count = args.obstacle_count
        if args.mode == "st":
            if args.randomize_obstacles_per_run:
                run_rng = random.Random(args.seed + int(round(speed * 100.0)))
                obstacle_count = run_rng.choice([2, 3])
                obstacle_seed = run_rng.randrange(1, 1_000_000)
            name = (
                f"{args.obstacle_name_prefix}_v{_speed_label(speed)}_"
                f"obs{obstacle_count}_seed{obstacle_seed}"
            )
            map_path, map_yaml, obstacle_manifest = _generate_obstacle_map(
                obstacle_seed,
                obstacle_count,
                name,
                base_map_yaml=args.base_map_yaml,
                trajectory_csv=args.trajectory_csv,
                output_dir=args.obstacle_output_dir,
            )

        print("\n" + "=" * 72)
        print(f"Running {args.mode} robustness: speed={speed:.1f} m/s")
        result_dir = run_ros_validation(
            table_path=Path(args.table) if args.table and args.mode == "lqr" else None,
            target_speed=speed,
            output_dir=eval_dir,
            log_path=str(lqr_log),
            timeout_seconds=args.timeout,
            lap_count=args.laps,
            max_lateral_accel=args.max_lateral_accel,
            max_steering_angle=args.max_steering_angle,
            lqr_lookahead_distance_m=args.lqr_lookahead_distance_m,
            use_tf_pose=args.use_tf_pose,
            enable_steering_rate_limit=not args.disable_steering_rate_limit,
            max_steering_rate=args.max_steering_rate,
            trajectory_mode=args.trajectory_mode,
            centerline_smoothing=args.centerline_smoothing,
            control_friendly_alpha=args.control_friendly_alpha,
            control_friendly_auto_alpha=args.control_friendly_auto_alpha,
            control_friendly_smoothing=args.control_friendly_smoothing,
            control_friendly_max_curvature=args.control_friendly_max_curvature,
            control_friendly_min_clearance=args.control_friendly_min_clearance,
            enable_rviz=False,
            enable_st_corridor_avoidance=args.mode == "st",
            map_path=map_path,
            map_yaml=map_yaml,
            track_csv=args.track_csv,
            trajectory_csv=args.trajectory_csv,
            st_avoidance_max_speed=avoidance_speed,
            st_min_speed=args.st_min_speed,
            st_lookahead_m=st_lookahead,
            st_inflation_margin_m=args.st_inflation_margin_m,
            st_min_border_clearance_m=args.st_min_border_clearance_m,
            st_max_lateral_offset_m=args.st_max_lateral_offset_m,
            st_lateral_offset_step_m=args.st_lateral_offset_step_m,
            st_takeover_distance_m=args.st_takeover_distance_m,
            st_desired_obstacle_clearance_m=args.st_desired_obstacle_clearance_m,
            st_static_obstacle_manifest_path=obstacle_manifest if args.mode == "st" else "",
            st_log_path=str(st_log),
            collision_log_path=str(collision_log),
            launch_log_path=launch_log,
            evaluator_log_path=evaluator_log,
        )

        summary = _load_summary(eval_dir / "lookahead_summary.json")
        st_summary = _summarize_st_log(st_log)
        obstacle_geometry = _summarize_obstacle_geometry(lqr_log, obstacle_manifest) if args.mode == "st" else {
            "obstacle_center_hits": 0,
            "min_obstacle_center_clearance_m": math.nan,
        }
        duration_s = summary.get("raw_duration_s", _raw_tracking_duration(lqr_log))
        collision_events = _count_collision_events(collision_log)
        timed_out_or_failed = result_dir is None or math.isnan(float(duration_s))
        mean_abs_e_y_cm = summary.get("mean_abs_e_y_cm", math.nan)
        p95_abs_e_y_cm = summary.get("p95_abs_e_y_cm", math.nan)
        max_abs_e_y_cm = summary.get("max_abs_e_y_cm", math.nan)
        if args.mode == "st":
            pass_run = (
                not timed_out_or_failed
                and collision_events == 0
                and st_summary["st_avoid_rows"] > 0
                and st_summary["st_blocked_rows"] == 0
                and obstacle_geometry["obstacle_center_hits"] == 0
                and float(obstacle_geometry["min_obstacle_center_clearance_m"]) >= 0.05
            )
        else:
            pass_run = (
                not timed_out_or_failed
                and collision_events == 0
                and float(max_abs_e_y_cm) < 8.0
                and float(p95_abs_e_y_cm) < 4.0
            )
        row = {
            "mode": args.mode,
            "speed": f"{speed:.2f}",
            "pass": int(pass_run),
            "obstacle_count": obstacle_count if args.mode == "st" else 0,
            "obstacle_seed": obstacle_seed if args.mode == "st" else "",
            "laps_required": args.laps,
            "run_dir": str(run_dir),
            "evaluation_dir": str(result_dir or ""),
            "tracking_log": str(lqr_log),
            "st_log": str(st_log) if args.mode == "st" else "",
            "collision_log": str(collision_log),
            "obstacle_manifest": obstacle_manifest,
            "collision_events": collision_events,
            "done_events": _count_done_events(collision_log),
            "duration_s": duration_s,
            "mean_abs_e_y_cm": mean_abs_e_y_cm,
            "p95_abs_e_y_cm": p95_abs_e_y_cm,
            "max_abs_e_y_cm": max_abs_e_y_cm,
            "mean_abs_e_psi_deg": summary.get("mean_abs_e_psi_deg", math.nan),
            "p95_abs_e_psi_deg": summary.get("p95_abs_e_psi_deg", math.nan),
            "max_abs_e_psi_deg": summary.get("max_abs_e_psi_deg", math.nan),
            "steering_saturation_ratio": summary.get("steering_saturation_ratio", math.nan),
            "p95_compute_time_ms": summary.get("p95_compute_time_ms", math.nan),
            "max_lateral_accel": args.max_lateral_accel,
            "max_steering_angle": args.max_steering_angle,
            "trajectory_mode": args.trajectory_mode,
            "centerline_smoothing": args.centerline_smoothing,
            "st_avoidance_max_speed": avoidance_speed if args.mode == "st" else "",
            "st_lookahead_m": st_lookahead if args.mode == "st" else "",
            "st_inflation_margin_m": args.st_inflation_margin_m if args.mode == "st" else "",
            "st_max_lateral_offset_m": args.st_max_lateral_offset_m if args.mode == "st" else "",
            "st_takeover_distance_m": args.st_takeover_distance_m if args.mode == "st" else "",
            **obstacle_geometry,
            **st_summary,
        }
        rows.append(row)
        _write_manifest(output_root / "manifest.csv", rows)

    print(f"\nManifest: {output_root / 'manifest.csv'}")


if __name__ == "__main__":
    main()

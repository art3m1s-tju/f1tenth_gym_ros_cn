#!/usr/bin/env python3
"""LQR 参数扫描与评估的命令行入口。"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lqr_sweep.lookup_table import LqrParams, LqrLookupTable
from lqr_sweep.objective import compute_objective
from lqr_sweep.sim_harness import SimConfig, run_single_sim, load_trajectory_cache
from lqr_sweep.sweep_engine import SweepConfig, run_full_sweep, run_speed_point_sweep


def parse_args() -> argparse.Namespace:
    """解析命令行参数。

    Returns:
        包含运行模式、输入路径、扫描范围和单次仿真参数的命名空间。
    """

    p = argparse.ArgumentParser(description="LQR parameter sweep system")
    p.add_argument("--mode", choices=["full", "coarse-only", "single", "validate"],
                   default="full", help="Operation mode")
    p.add_argument("--map-path", default="/sim_ws/src/f1tenth_gym_ros/maps/my_map")
    p.add_argument("--trajectory-csv",
                   default="/sim_ws/src/f1tenth_gym_ros/code/outputs/csv/global_trajectory.csv")
    p.add_argument("--output-dir", default="/sim_ws/src/f1tenth_gym_ros/code/outputs/sweep")
    p.add_argument("--speeds", nargs="+", type=float,
                   default=[0.5, 1.0, 1.5, 2.0, 2.5, 3.0])
    p.add_argument("--coarse-grid", default="5,4,5,3",
                   help="Grid sizes: q_lat,q_head,r_steer,ff_gain")
    p.add_argument("--max-workers", type=int, default=None)
    p.add_argument("--laps", type=int, default=5,
                   help="Number of consecutive laps required for each simulation")
    p.add_argument("--max-sim-time", type=float, default=None,
                   help="Maximum simulated seconds per candidate; auto-estimated when omitted")
    p.add_argument("--max-lateral-accel", type=float, default=4.0,
                   help="Maximum lateral acceleration used by curvature speed limit")
    p.add_argument("--disable-curvature-speed-limit", action="store_true",
                   help="Disable curvature speed limiting for constant-speed sweeps")
    p.add_argument("--max-accel", type=float, default=1.0,
                   help="Speed ramp acceleration limit")
    p.add_argument("--max-decel", type=float, default=2.0,
                   help="Speed ramp deceleration limit")
    p.add_argument("--disable-speed-ramp", action="store_true",
                   help="Disable longitudinal speed command ramping")
    p.add_argument("--table", type=str, default=None,
                   help="Path to existing gain table (for validate mode)")

    # 单次仿真模式使用的 LQR 参数。
    p.add_argument("--speed", type=float, default=1.0)
    p.add_argument("--q-lateral", type=float, default=3.0)
    p.add_argument("--q-heading", type=float, default=1.2)
    p.add_argument("--r-steering", type=float, default=8.0)
    p.add_argument("--feedforward-gain", type=float, default=1.0)

    return p.parse_args()


def main() -> None:
    """根据命令行参数执行单次仿真、表验证或完整参数扫描。

    ``single`` 模式用于快速评估一组参数；``validate`` 模式用于检查已有
    查找表中每个速度点的表现；``full`` 和 ``coarse-only`` 模式用于生成
    新的速度分段增益表。
    """

    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    min_eval_speed = min([args.speed] if args.mode == "single" else args.speeds)
    if args.max_sim_time is None:
        trajectory_cache = load_trajectory_cache(args.trajectory_csv)
        max_sim_time = trajectory_cache.path_length * max(1, args.laps) / max(min_eval_speed, 0.1)
        max_sim_time = max(120.0, 1.5 * max_sim_time + 20.0)
    else:
        max_sim_time = args.max_sim_time

    sim_config = SimConfig(
        map_path=args.map_path,
        trajectory_csv=args.trajectory_csv,
        max_sim_time=max_sim_time,
        lap_count=args.laps,
        max_lateral_accel=args.max_lateral_accel,
        enable_curvature_speed_limit=not args.disable_curvature_speed_limit,
        enable_speed_ramp=not args.disable_speed_ramp,
        max_accel=args.max_accel,
        max_decel=args.max_decel,
    )

    if args.mode == "single":
        params = LqrParams(
            q_lateral=args.q_lateral,
            q_heading=args.q_heading,
            r_steering=args.r_steering,
            feedforward_gain=args.feedforward_gain,
        )
        print(f"Running single sim: speed={args.speed}, {params}")
        t0 = time.time()
        result = run_single_sim(sim_config, params, args.speed)
        elapsed = time.time() - t0
        print(f"  Elapsed: {elapsed:.2f}s")
        if result.success:
            score = compute_objective(result.metrics)
            print(f"  Lap time: {result.lap_time:.2f}s")
            print(f"  Score: {score:.4f}")
            for k, v in result.metrics.items():
                print(f"  {k}: {v:.6f}")
        else:
            print(f"  FAILED: {result.failure_reason}")
        return

    if args.mode == "validate":
        if not args.table:
            print("ERROR: --table required for validate mode")
            sys.exit(1)
        table = LqrLookupTable.load(Path(args.table))
        print(f"Loaded table with {len(table)} entries")
        print("Validating each speed point...")
        for entry in table.entries:
            params = entry.to_params()
            result = run_single_sim(sim_config, params, entry.speed)
            if result.success:
                score = compute_objective(result.metrics)
                print(f"  v={entry.speed:.1f}: score={score:.4f} "
                      f"mean_ey={result.metrics['mean_abs_e_y']*100:.2f}cm "
                      f"lap={result.lap_time:.1f}s")
            else:
                print(f"  v={entry.speed:.1f}: FAILED ({result.failure_reason})")
        return

    grid_sizes = tuple(int(x) for x in args.coarse_grid.split(","))
    assert len(grid_sizes) == 4

    sweep_config = SweepConfig(
        sim_config=sim_config,
        speed_points=args.speeds,
        coarse_grid_sizes=grid_sizes,
        max_workers=args.max_workers,
    )

    if args.mode == "coarse-only":
        sweep_config.refine_top_k = 0

    print(f"Starting LQR parameter sweep")
    print(f"  Speeds: {args.speeds}")
    print(f"  Grid: {grid_sizes}")
    print(f"  Laps: {args.laps}")
    print(f"  Max sim time per candidate: {max_sim_time:.1f}s")
    print(f"  Workers: {args.max_workers or 'auto'}")
    t0 = time.time()

    table = run_full_sweep(sweep_config, verbose=True)

    elapsed = time.time() - t0
    print(f"\nTotal sweep time: {elapsed:.1f}s")

    table_path = output_dir / "lqr_gain_table.yaml"
    table.save(table_path)
    print(f"Saved lookup table to: {table_path}")

    summary = {
        "sweep_time_s": elapsed,
        "speed_points": args.speeds,
        "grid_sizes": list(grid_sizes),
        "lap_count": args.laps,
        "max_sim_time": max_sim_time,
        "max_lateral_accel": args.max_lateral_accel,
        "enable_curvature_speed_limit": not args.disable_curvature_speed_limit,
        "enable_speed_ramp": not args.disable_speed_ramp,
        "max_accel": args.max_accel,
        "max_decel": args.max_decel,
        "entries": [
            {
                "speed": e.speed,
                "q_lateral": e.q_lateral,
                "q_heading": e.q_heading,
                "r_steering": e.r_steering,
                "feedforward_gain": e.feedforward_gain,
                "score": e.score,
            }
            for e in table.entries
        ],
    }
    summary_path = output_dir / "sweep_summary.json"
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"Saved summary to: {summary_path}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Benchmark scalar vs NumPy-vectorized Frenet candidate profile generation."""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from pnc_rc.frenet.planner import (  # noqa: E402
    FrenetState,
    evaluate_quartic,
    evaluate_quintic,
    solve_quartic_longitudinal,
    solve_quintic_lateral,
    _sample_range,
)
from pnc_rc.frenet.trajectory_generation import generate_candidate_profiles  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--d-step", type=float, default=0.10)
    parser.add_argument("--v-step", type=float, default=0.80)
    parser.add_argument("--t-min", type=float, default=1.0)
    parser.add_argument("--t-max", type=float, default=3.0)
    parser.add_argument("--t-step", type=float, default=1.0)
    parser.add_argument("--dt", type=float, default=0.10)
    parser.add_argument("--repeat", type=int, default=20)
    return parser.parse_args()


def scalar_generation(
    state: FrenetState,
    d_samples: np.ndarray,
    duration_samples: np.ndarray,
    speed_samples: np.ndarray,
    trajectory_dt: float,
) -> int:
    total_points = 0
    for d_final in d_samples:
        for duration in duration_samples:
            d_coeff = solve_quintic_lateral(state, float(d_final), float(duration))
            time_values = np.arange(
                0.0,
                float(duration) + 0.5 * trajectory_dt,
                trajectory_dt,
            )
            d_values, d_dot_values, _, d_jerk_values = evaluate_quintic(
                d_coeff,
                time_values,
            )
            total_points += len(d_values) + len(d_dot_values) + len(d_jerk_values)
            for speed_final in speed_samples:
                s_coeff = solve_quartic_longitudinal(
                    state,
                    float(speed_final),
                    float(duration),
                )
                s_values, s_dot_values, _, s_jerk_values = evaluate_quartic(
                    s_coeff,
                    time_values,
                )
                total_points += len(s_values) + len(s_dot_values) + len(s_jerk_values)
    return total_points


def time_call(repeat: int, fn) -> tuple[float, float]:
    samples = []
    for _ in range(max(1, repeat)):
        start = time.perf_counter()
        fn()
        samples.append((time.perf_counter() - start) * 1000.0)
    return statistics.mean(samples), statistics.stdev(samples) if len(samples) > 1 else 0.0


def main() -> None:
    args = parse_args()
    state = FrenetState(
        s=5.0,
        d=0.1,
        s_dot=1.2,
        d_dot=-0.1,
        s_ddot=0.1,
        d_ddot=0.0,
    )
    d_samples = _sample_range(-1.8, 1.8, args.d_step)
    duration_samples = _sample_range(args.t_min, args.t_max, args.t_step)
    speed_samples = _sample_range(0.0, 3.75, args.v_step)
    candidate_count = len(d_samples) * len(duration_samples) * len(speed_samples)
    print(
        "candidate_grid="
        f"d:{len(d_samples)} T:{len(duration_samples)} v:{len(speed_samples)} "
        f"total:{candidate_count}"
    )

    scalar_mean, scalar_std = time_call(
        args.repeat,
        lambda: scalar_generation(
            state,
            d_samples,
            duration_samples,
            speed_samples,
            args.dt,
        ),
    )
    print(f"scalar_python_ms={scalar_mean:.3f} +/- {scalar_std:.3f}")

    def run_numpy_batch() -> None:
        generate_candidate_profiles(
            state_s=state.s,
            state_d=state.d,
            state_s_dot=state.s_dot,
            state_d_dot=state.d_dot,
            state_s_ddot=state.s_ddot,
            state_d_ddot=state.d_ddot,
            d_samples=d_samples,
            duration_samples=duration_samples,
            speed_samples=speed_samples,
            trajectory_dt=args.dt,
        )

    batch_mean, batch_std = time_call(args.repeat, run_numpy_batch)
    print(f"numpy_batch_ms={batch_mean:.3f} +/- {batch_std:.3f}")
    if batch_mean > 0.0:
        print(f"numpy_batch_speedup_vs_scalar={scalar_mean / batch_mean:.2f}x")


if __name__ == "__main__":
    main()

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from pnc_rc.frenet.planner import (
    FrenetState,
    evaluate_quartic,
    evaluate_quintic,
    solve_quartic_longitudinal,
    solve_quintic_lateral,
)
from pnc_rc.frenet.trajectory_generation import generate_candidate_profiles


def test_batch_candidate_profiles_match_scalar_polynomials():
    state = FrenetState(
        s=2.0,
        d=0.2,
        s_dot=1.1,
        d_dot=-0.05,
        s_ddot=0.2,
        d_ddot=0.03,
    )
    d_samples = np.array([-0.4, 0.2], dtype=float)
    duration_samples = np.array([1.0, 1.5], dtype=float)
    speed_samples = np.array([0.0, 1.6], dtype=float)
    dt = 0.1

    profiles = generate_candidate_profiles(
        state_s=state.s,
        state_d=state.d,
        state_s_dot=state.s_dot,
        state_d_dot=state.d_dot,
        state_s_ddot=state.s_ddot,
        state_d_ddot=state.d_ddot,
        d_samples=d_samples,
        duration_samples=duration_samples,
        speed_samples=speed_samples,
        trajectory_dt=dt,
    )

    assert profiles.backend == "numpy"
    assert len(profiles.d_finals) == 8
    for idx in range(len(profiles.d_finals)):
        length = int(profiles.lengths[idx])
        duration = float(profiles.durations[idx])
        time_values = np.arange(0.0, duration + 0.5 * dt, dt)
        assert len(time_values) == length

        d_coeff = solve_quintic_lateral(state, float(profiles.d_finals[idx]), duration)
        s_coeff = solve_quartic_longitudinal(
            state,
            float(profiles.speed_finals[idx]),
            duration,
        )
        d_values, d_dot_values, _, d_jerk_values = evaluate_quintic(
            d_coeff,
            time_values,
        )
        s_values, s_dot_values, _, s_jerk_values = evaluate_quartic(
            s_coeff,
            time_values,
        )

        assert np.allclose(profiles.d_values[idx, :length], d_values)
        assert np.allclose(profiles.d_dot_values[idx, :length], d_dot_values)
        assert np.allclose(profiles.d_jerk_values[idx, :length], d_jerk_values)
        assert np.allclose(profiles.s_values[idx, :length], s_values)
        assert np.allclose(profiles.s_dot_values[idx, :length], s_dot_values)
        assert np.allclose(profiles.s_jerk_values[idx, :length], s_jerk_values)

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from pnc_rc.frenet.planner import (
    FrenetPlannerConfig,
    FrenetState,
    LocalGridConfig,
    ReferencePath,
    build_occupancy_grid,
    evaluate_quartic,
    evaluate_quintic,
    plan_frenet_path,
    solve_quartic_longitudinal,
    solve_quintic_lateral,
)


def test_quintic_lateral_boundary_conditions():
    state = FrenetState(s=0.0, d=0.2, s_dot=1.0, d_dot=0.1, s_ddot=0.0, d_ddot=-0.04)
    duration = 1.4
    coeff = solve_quintic_lateral(state, d_final=-0.3, duration=duration)

    start = np.array([0.0])
    end = np.array([duration])
    d0, d_dot0, d_ddot0, _ = evaluate_quintic(coeff, start)
    df, d_dotf, d_ddotf, _ = evaluate_quintic(coeff, end)

    assert np.isclose(d0[0], state.d)
    assert np.isclose(d_dot0[0], state.d_dot)
    assert np.isclose(d_ddot0[0], state.d_ddot)
    assert np.isclose(df[0], -0.3)
    assert abs(d_dotf[0]) < 1e-9
    assert abs(d_ddotf[0]) < 1e-9


def test_quartic_longitudinal_uses_initial_acceleration():
    state = FrenetState(s=2.0, d=0.0, s_dot=1.2, d_dot=0.0, s_ddot=0.5, d_ddot=0.0)
    duration = 1.5
    coeff = solve_quartic_longitudinal(state, speed_final=2.0, duration=duration)

    start = np.array([0.0])
    end = np.array([duration])
    s0, s_dot0, s_ddot0, _ = evaluate_quartic(coeff, start)
    _, s_dotf, s_ddotf, _ = evaluate_quartic(coeff, end)

    assert np.isclose(s0[0], state.s)
    assert np.isclose(s_dot0[0], state.s_dot)
    assert np.isclose(s_ddot0[0], state.s_ddot)
    assert np.isclose(s_dotf[0], 2.0)
    assert abs(s_ddotf[0]) < 1e-9


def test_reference_path_projection_and_sampling_round_trip():
    theta = np.linspace(0.0, 2.0 * math.pi, 80, endpoint=False)
    points = np.column_stack([3.0 * np.cos(theta), 3.0 * np.sin(theta)])
    reference = ReferencePath.from_points(points)
    original = np.array([3.0, 0.2])

    s, d, _, _ = reference.project(original)
    reconstructed, _, _ = reference.sample(s, d)

    assert np.linalg.norm(original - reconstructed) < 0.02


def test_occupancy_grid_inflates_scan_obstacle():
    cfg = LocalGridConfig(
        forward_m=4.0,
        rear_m=1.0,
        half_width_m=2.0,
        resolution_m=0.1,
        inflation_radius_m=0.3,
        scan_offset_x_m=0.0,
    )
    ranges = np.array([2.0])
    grid = build_occupancy_grid(
        ranges,
        angle_min=0.0,
        angle_increment=1.0,
        range_min=0.0,
        range_max=10.0,
        config=cfg,
    )
    collision, clearance = grid.query_path(np.array([[2.1, 0.0]]), (0.0, 0.0, 0.0))

    assert collision
    assert clearance == 0.0


def test_planner_prefers_centerline_without_obstacles():
    points = np.column_stack([np.linspace(0.0, 20.0, 80), np.zeros(80)])
    # Close the reference with a far return segment; candidates stay on the straight.
    points = np.vstack([points, [[20.0, 8.0], [0.0, 8.0]]])
    reference = ReferencePath.from_points(points)
    state = FrenetState(s=0.0, d=0.0, s_dot=1.0, d_dot=0.0, s_ddot=0.0, d_ddot=0.0)
    grid = build_occupancy_grid(
        np.array([], dtype=float),
        angle_min=0.0,
        angle_increment=1.0,
        range_min=0.0,
        range_max=10.0,
        config=LocalGridConfig(),
    )
    config = FrenetPlannerConfig(
        d_min=-0.2,
        d_max=0.2,
        d_step=0.2,
        t_min=1.0,
        t_max=1.0,
        t_step=1.0,
        v_min=1.0,
        v_max=1.0,
        v_step=1.0,
        target_speed=1.0,
    )

    candidate = plan_frenet_path(reference, state, grid, (0.0, 0.0, 0.0), config)

    assert candidate is not None
    assert abs(candidate.d[-1]) < 1e-9

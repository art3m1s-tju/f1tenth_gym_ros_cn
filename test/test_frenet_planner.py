import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from pnc_rc.frenet.planner import (
    FrenetPlanStats,
    FrenetPlannerConfig,
    FrenetState,
    LocalGridConfig,
    ReferencePath,
    build_occupancy_grid,
    densify_path_points,
    estimate_heading_jumps,
    estimate_open_path_curvature,
    evaluate_quartic,
    evaluate_quintic,
    initial_frenet_state,
    plan_frenet_path,
    solve_quartic_longitudinal,
    solve_quintic_lateral,
    swept_corridor_points,
    trim_path_to_position,
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


def test_occupancy_grid_ignores_zero_range_returns():
    cfg = LocalGridConfig(
        forward_m=4.0,
        rear_m=1.0,
        half_width_m=2.0,
        resolution_m=0.1,
        inflation_radius_m=0.3,
        scan_offset_x_m=0.0,
    )
    grid = build_occupancy_grid(
        np.array([0.0]),
        angle_min=0.0,
        angle_increment=1.0,
        range_min=0.0,
        range_max=10.0,
        config=cfg,
    )
    collision, clearance = grid.query_path(np.array([[0.1, 0.0]]), (0.0, 0.0, 0.0))

    assert not grid.has_obstacles
    assert not collision
    assert math.isinf(clearance)


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


def test_initial_frenet_state_d_dot_uses_single_normal_projection():
    points = np.column_stack([np.linspace(0.0, 10.0, 20), np.zeros(20)])
    points = np.vstack([points, [[10.0, 4.0], [0.0, 4.0]]])
    reference = ReferencePath.from_points(points)

    state = initial_frenet_state(
        reference=reference,
        position=np.array([1.0, 0.0]),
        yaw=math.pi / 4.0,
        velocity_xy=np.array([1.0, 1.0]),
        previous_s=None,
        previous_s_dot=None,
        dt=None,
    )

    assert np.isclose(state.s_dot, 1.0, atol=1e-6)
    assert np.isclose(state.d_dot, 1.0, atol=1e-6)


def test_initial_frenet_state_uses_previous_s_to_avoid_branch_jump():
    points = np.array(
        [
            [0.0, 0.0],
            [10.0, 0.0],
            [10.0, 1.0],
            [0.0, 1.0],
        ],
        dtype=float,
    )
    reference = ReferencePath.from_points(points)

    branch_a = reference.project(np.array([5.0, 0.1]))
    branch_b = reference.project(np.array([5.0, 0.9]))
    state = initial_frenet_state(
        reference=reference,
        position=np.array([5.0, 0.6]),
        yaw=0.0,
        velocity_xy=np.array([0.2, 0.0]),
        previous_s=branch_a[0],
        previous_s_dot=0.2,
        dt=0.1,
        projection_window_m=3.0,
    )

    assert abs(state.d) < 1.0
    assert abs(state.s - branch_a[0]) < 3.0
    assert abs(state.s - branch_b[0]) > 3.0


def test_heading_jump_estimator_detects_reversal():
    points = np.array(
        [
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
            [0.0, 1.0],
        ],
        dtype=float,
    )

    jumps = estimate_heading_jumps(points)

    assert len(jumps) == 2
    assert np.max(np.abs(jumps)) > math.pi / 3.0


def test_open_path_curvature_ignores_near_duplicate_start_samples():
    points = np.array(
        [
            [0.0, 0.0],
            [0.0005, 0.0],
            [0.0040, 0.0001],
            [0.0130, 0.0008],
            [0.0300, 0.0030],
            [0.0570, 0.0088],
            [0.0960, 0.0184],
            [0.1490, 0.0328],
            [0.2180, 0.0530],
            [0.3050, 0.0800],
        ],
        dtype=float,
    )

    curvature = estimate_open_path_curvature(points)

    assert len(curvature) >= 3
    assert float(np.max(np.abs(curvature))) < 1.8


def test_planner_rejects_candidates_with_insufficient_progress():
    points = np.column_stack([np.linspace(0.0, 20.0, 80), np.zeros(80)])
    points = np.vstack([points, [[20.0, 8.0], [0.0, 8.0]]])
    reference = ReferencePath.from_points(points)
    state = FrenetState(s=0.0, d=0.0, s_dot=0.0, d_dot=0.0, s_ddot=0.0, d_ddot=0.0)
    grid = build_occupancy_grid(
        np.array([], dtype=float),
        angle_min=0.0,
        angle_increment=1.0,
        range_min=0.0,
        range_max=10.0,
        config=LocalGridConfig(),
    )
    config = FrenetPlannerConfig(
        d_min=0.0,
        d_max=0.0,
        d_step=1.0,
        t_min=1.0,
        t_max=1.0,
        t_step=1.0,
        v_min=0.2,
        v_max=0.2,
        v_step=1.0,
        target_speed=0.2,
        min_progress_step_m=0.25,
    )

    candidate = plan_frenet_path(reference, state, grid, (0.0, 0.0, 0.0), config)

    assert candidate is None


def test_planner_allows_stationary_start_when_total_progress_is_sufficient():
    points = np.column_stack([np.linspace(0.0, 20.0, 80), np.zeros(80)])
    points = np.vstack([points, [[20.0, 8.0], [0.0, 8.0]]])
    reference = ReferencePath.from_points(points)
    state = FrenetState(s=0.0, d=0.0, s_dot=0.0, d_dot=0.0, s_ddot=0.0, d_ddot=0.0)
    grid = build_occupancy_grid(
        np.array([], dtype=float),
        angle_min=0.0,
        angle_increment=1.0,
        range_min=0.0,
        range_max=10.0,
        config=LocalGridConfig(),
    )
    config = FrenetPlannerConfig(
        d_min=0.0,
        d_max=0.0,
        d_step=1.0,
        t_min=2.0,
        t_max=2.0,
        t_step=1.0,
        v_min=0.6,
        v_max=0.6,
        v_step=1.0,
        target_speed=0.6,
        min_progress_step_m=0.2,
    )

    candidate = plan_frenet_path(reference, state, grid, (0.0, 0.0, 0.0), config)

    assert candidate is not None
    assert candidate.s[-1] - candidate.s[0] > 0.2


def test_planner_accepts_curved_reference_from_stationary_start():
    theta = np.linspace(0.0, 2.0 * math.pi, 120, endpoint=False)
    points = np.column_stack([5.0 * np.cos(theta), 5.0 * np.sin(theta)])
    reference = ReferencePath.from_points(points)
    position = np.array([5.0, 0.0], dtype=float)
    state = initial_frenet_state(
        reference=reference,
        position=position,
        yaw=math.pi / 2.0,
        velocity_xy=np.array([0.0, 0.0]),
        previous_s=None,
        previous_s_dot=None,
        dt=None,
    )
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
        d_step=0.1,
        t_min=2.0,
        t_max=2.0,
        t_step=1.0,
        v_min=0.6,
        v_max=1.2,
        v_step=0.3,
        target_speed=0.9,
        max_curvature=1.8,
        min_progress_step_m=0.2,
    )

    candidate = plan_frenet_path(reference, state, grid, (5.0, 0.0, math.pi / 2.0), config)

    assert candidate is not None
    assert candidate.max_curvature <= config.max_curvature


def test_swept_corridor_expands_path_laterally():
    points = np.array([[0.0, 0.0], [1.0, 0.0]], dtype=float)

    corridor = swept_corridor_points(points, corridor_radius_m=0.2, corridor_sample_step_m=0.1)

    assert len(corridor) > len(points)
    assert np.isclose(np.max(corridor[:, 1]), 0.2)
    assert np.isclose(np.min(corridor[:, 1]), -0.2)


def test_swept_corridor_includes_forward_footprint():
    points = np.array([[0.0, 0.0], [1.0, 0.0]], dtype=float)

    corridor = swept_corridor_points(
        points,
        corridor_radius_m=0.0,
        corridor_sample_step_m=0.1,
        footprint_front_m=0.35,
        footprint_rear_m=0.05,
    )

    assert np.isclose(np.max(corridor[:, 0]), 1.35)
    assert np.isclose(np.min(corridor[:, 0]), -0.05)


def test_densify_path_limits_segment_spacing():
    points = np.array([[0.0, 0.0], [0.23, 0.0]], dtype=float)

    dense = densify_path_points(points, max_step_m=0.05)

    segment_lengths = np.linalg.norm(np.diff(dense, axis=0), axis=1)
    assert np.all(segment_lengths <= 0.05 + 1e-9)
    assert np.allclose(dense[0], points[0])
    assert np.allclose(dense[-1], points[-1])


def test_occupancy_grid_detects_obstacle_between_sparse_path_points():
    cfg = LocalGridConfig(
        forward_m=4.0,
        rear_m=1.0,
        half_width_m=2.0,
        resolution_m=0.02,
        inflation_radius_m=0.03,
        scan_offset_x_m=0.0,
    )
    grid = build_occupancy_grid(
        np.array([1.0]),
        angle_min=0.0,
        angle_increment=1.0,
        range_min=0.0,
        range_max=10.0,
        config=cfg,
    )
    sparse_path = np.array([[0.0, 0.0], [2.0, 0.0]], dtype=float)

    sparse_collision, _ = grid.query_path(
        sparse_path,
        (0.0, 0.0, 0.0),
        path_sample_step_m=0.0,
    )
    dense_collision, _ = grid.query_path(
        sparse_path,
        (0.0, 0.0, 0.0),
        path_sample_step_m=0.05,
    )

    assert not sparse_collision
    assert dense_collision


def test_trim_path_to_position_reanchors_from_current_pose():
    points = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]], dtype=float)

    trimmed = trim_path_to_position(points, np.array([1.25, 0.0]), min_remaining_length_m=0.5)

    assert len(trimmed) >= 2
    assert np.isclose(trimmed[0, 0], 1.25, atol=1e-6)
    assert np.isclose(trimmed[0, 1], 0.0, atol=1e-6)
    assert trimmed[1, 0] >= 1.0


def test_occupancy_grid_corridor_detects_vehicle_width_collision():
    cfg = LocalGridConfig(
        forward_m=4.0,
        rear_m=1.0,
        half_width_m=2.0,
        resolution_m=0.05,
        inflation_radius_m=0.05,
        scan_offset_x_m=0.0,
    )
    obstacle_range = math.hypot(1.0, 0.2)
    obstacle_angle = math.atan2(0.2, 1.0)
    grid = build_occupancy_grid(
        np.array([obstacle_range]),
        angle_min=obstacle_angle,
        angle_increment=1.0,
        range_min=0.0,
        range_max=10.0,
        config=cfg,
    )
    centerline = np.array([[1.0, 0.0]], dtype=float)

    center_collision, _ = grid.query_path(centerline, (0.0, 0.0, 0.0))
    corridor_collision, _ = grid.query_path(
        centerline,
        (0.0, 0.0, 0.0),
        corridor_radius_m=0.25,
        corridor_sample_step_m=0.05,
    )

    assert not center_collision
    assert corridor_collision


def test_occupancy_grid_footprint_detects_front_bumper_collision():
    cfg = LocalGridConfig(
        forward_m=4.0,
        rear_m=1.0,
        half_width_m=2.0,
        resolution_m=0.05,
        inflation_radius_m=0.05,
        scan_offset_x_m=0.0,
    )
    grid = build_occupancy_grid(
        np.array([1.3]),
        angle_min=0.0,
        angle_increment=1.0,
        range_min=0.0,
        range_max=10.0,
        config=cfg,
    )
    base_link_path = np.array([[1.0, 0.0]], dtype=float)

    center_collision, _ = grid.query_path(base_link_path, (0.0, 0.0, 0.0))
    footprint_collision, _ = grid.query_path(
        base_link_path,
        (0.0, 0.0, 0.0),
        footprint_front_m=0.35,
        footprint_rear_m=0.05,
    )

    assert not center_collision
    assert footprint_collision


def test_planner_hard_rejects_low_clearance_candidate():
    points = np.column_stack([np.linspace(0.0, 20.0, 80), np.zeros(80)])
    points = np.vstack([points, [[20.0, 8.0], [0.0, 8.0]]])
    reference = ReferencePath.from_points(points)
    state = FrenetState(s=0.0, d=0.0, s_dot=0.8, d_dot=0.0, s_ddot=0.0, d_ddot=0.0)
    cfg = LocalGridConfig(
        forward_m=4.0,
        rear_m=1.0,
        half_width_m=2.0,
        resolution_m=0.02,
        inflation_radius_m=0.05,
        scan_offset_x_m=0.0,
    )
    obstacle_range = math.hypot(0.8, 0.18)
    obstacle_angle = math.atan2(0.18, 0.8)
    grid = build_occupancy_grid(
        np.array([obstacle_range]),
        angle_min=obstacle_angle,
        angle_increment=1.0,
        range_min=0.0,
        range_max=10.0,
        config=cfg,
    )
    config = FrenetPlannerConfig(
        d_min=0.0,
        d_max=0.0,
        d_step=1.0,
        t_min=1.0,
        t_max=1.0,
        t_step=1.0,
        v_min=0.8,
        v_max=0.8,
        v_step=1.0,
        target_speed=0.8,
        corridor_radius_m=0.1,
        corridor_sample_step_m=0.05,
        min_clearance_m=0.04,
    )
    stats = FrenetPlanStats()

    candidate = plan_frenet_path(reference, state, grid, (0.0, 0.0, 0.0), config, stats)

    assert candidate is None
    assert stats.clearance_rejections > 0


def test_reference_path_open_mode_does_not_wrap_at_endpoint():
    points = np.array([[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]], dtype=float)
    reference = ReferencePath.from_points(points, closed_loop=False)

    endpoint, _, _ = reference.sample(100.0, 0.0)

    assert np.allclose(endpoint, [20.0, 0.0])
    assert not reference.closed_loop

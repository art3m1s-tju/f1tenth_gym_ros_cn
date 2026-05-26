"""CPU Frenet local planner primitives for static obstacle avoidance."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage
from scipy.spatial import KDTree

from pnc_rc.lqr.geometry import (
    PathProjection,
    advance_projection_along_path,
    interpolate_angle,
    project_to_path,
)
from pnc_rc.lqr.math import compute_path_curvatures, compute_path_headings, wrap_angle


MIN_VALID_SCAN_RANGE_M = 1e-3
MIN_CURVATURE_SAMPLE_SPACING_M = 0.03


@dataclass(frozen=True)
class FrenetState:
    s: float
    d: float
    s_dot: float
    d_dot: float
    s_ddot: float
    d_ddot: float


@dataclass(frozen=True)
class FrenetPlannerConfig:
    d_min: float = -1.0
    d_max: float = 1.0
    d_step: float = 0.1
    t_min: float = 2.0
    t_max: float = 3.0
    t_step: float = 0.5
    v_min: float = 0.6
    v_max: float = 2.5
    v_step: float = 0.3
    trajectory_dt: float = 0.1
    target_speed: float = 1.5
    max_curvature: float = 1.1
    safe_clearance: float = 0.35
    min_clearance_m: float = 0.05
    corridor_radius_m: float = 0.14
    corridor_sample_step_m: float = 0.10
    path_collision_sample_step_m: float = 0.05
    footprint_front_m: float = 0.38
    footprint_rear_m: float = 0.05
    weight_lateral_jerk: float = 0.15
    weight_longitudinal_jerk: float = 0.08
    weight_time: float = 0.15
    weight_lateral_offset: float = 0.6
    weight_speed_error: float = 0.7
    weight_obstacle_clearance: float = 24.0
    weight_curvature: float = 2.5
    weight_curvature_rate: float = 0.8
    weight_lateral_shift: float = 1.2
    max_heading_jump: float = 0.65
    min_progress_step_m: float = 0.20


@dataclass(frozen=True)
class LocalGridConfig:
    forward_m: float = 7.0
    rear_m: float = 1.0
    half_width_m: float = 3.0
    resolution_m: float = 0.05
    inflation_radius_m: float = 0.28
    scan_offset_x_m: float = 0.275


@dataclass
class ReferencePath:
    points: np.ndarray
    headings: np.ndarray
    curvatures: np.ndarray
    segment_lengths: np.ndarray
    cumulative_s: np.ndarray
    kdtree: KDTree
    total_length: float
    closed_loop: bool = True

    @classmethod
    def from_points(cls, points: np.ndarray, closed_loop: bool = True) -> "ReferencePath":
        if len(points) < 3:
            raise ValueError("reference path needs at least 3 points")
        points = np.asarray(points, dtype=float)
        if closed_loop:
            path_for_segments = np.vstack([points, points[:1]])
        else:
            path_for_segments = points
        segment_lengths = np.linalg.norm(np.diff(path_for_segments, axis=0), axis=1)
        total_length = float(np.sum(segment_lengths))
        cumulative_s = np.concatenate([[0.0], np.cumsum(segment_lengths)])
        if closed_loop:
            cumulative_s = cumulative_s[:-1]
        return cls(
            points=points,
            headings=compute_path_headings(points, closed_loop=closed_loop),
            curvatures=compute_path_curvatures(points, closed_loop=closed_loop),
            segment_lengths=segment_lengths,
            cumulative_s=cumulative_s,
            kdtree=KDTree(points),
            total_length=total_length,
            closed_loop=closed_loop,
        )

    def project(self, position: np.ndarray) -> tuple[float, float, float, float]:
        projection = project_to_path(
            position,
            self.points,
            self.kdtree,
            self.headings,
            self.curvatures,
            closed_loop=self.closed_loop,
        )
        return self._projection_to_frenet(projection)

    def project_near(
        self,
        position: np.ndarray,
        s_hint: float,
        search_window_m: float,
    ) -> tuple[float, float, float, float]:
        if search_window_m <= 0.0:
            return self.project(position)
        wrapped_hint = (
            float(s_hint % self.total_length)
            if self.closed_loop
            else float(np.clip(s_hint, 0.0, self.total_length))
        )
        candidate_segments = self._candidate_segments_near_s(
            wrapped_hint,
            search_window_m,
        )
        if not candidate_segments:
            return self.project(position)
        projection = self._project_to_candidate_segments(position, candidate_segments)
        return self._projection_to_frenet(projection)

    def sample(self, s: float, d: float) -> tuple[np.ndarray, float, float]:
        if self.closed_loop:
            s = float(s % self.total_length)
        else:
            s = float(np.clip(s, 0.0, self.total_length))
        segment_idx = int(np.searchsorted(self.cumulative_s, s, side="right") - 1)
        max_segment_idx = len(self.points) - 1 if self.closed_loop else len(self.points) - 2
        segment_idx = int(np.clip(segment_idx, 0, max_segment_idx))
        segment_length = float(self.segment_lengths[segment_idx])
        if segment_length <= 1e-9:
            t = 0.0
        else:
            t = float((s - self.cumulative_s[segment_idx]) / segment_length)
        next_idx = (
            (segment_idx + 1) % len(self.points)
            if self.closed_loop
            else min(segment_idx + 1, len(self.points) - 1)
        )
        point = (
            self.points[segment_idx]
            + t * (self.points[next_idx] - self.points[segment_idx])
        )
        heading = interpolate_angle(
            float(self.headings[segment_idx]),
            float(self.headings[next_idx]),
            t,
        )
        curvature = float(
            (1.0 - t) * self.curvatures[segment_idx]
            + t * self.curvatures[next_idx]
        )
        normal = np.array([-math.sin(heading), math.cos(heading)])
        return point + d * normal, heading, curvature

    def _projection_to_frenet(
        self,
        projection: PathProjection,
    ) -> tuple[float, float, float, float]:
        segment_length = float(self.segment_lengths[projection.segment_idx])
        s = float(
            self.cumulative_s[projection.segment_idx]
            + projection.segment_t * segment_length
        )
        return s, projection.lateral_error, projection.heading, projection.curvature

    def _candidate_segments_near_s(
        self,
        s_hint: float,
        search_window_m: float,
    ) -> list[int]:
        segment_mid_s = self.cumulative_s[: len(self.segment_lengths)] + 0.5 * self.segment_lengths
        if self.closed_loop:
            segment_mid_s = segment_mid_s % self.total_length
            arc_distance = np.abs(
                ((segment_mid_s - s_hint + 0.5 * self.total_length) % self.total_length)
                - 0.5 * self.total_length
            )
        else:
            arc_distance = np.abs(segment_mid_s - s_hint)
        candidate_segments = np.flatnonzero(
            arc_distance <= (search_window_m + 0.5 * self.segment_lengths)
        )
        return [int(segment_idx) for segment_idx in candidate_segments]

    def _project_to_candidate_segments(
        self,
        position: np.ndarray,
        candidate_segments: list[int],
    ) -> PathProjection:
        best_dist = float("inf")
        best_point = self.points[candidate_segments[0]].copy()
        best_seg_idx = candidate_segments[0]
        best_t = 0.0
        best_heading = float(self.headings[best_seg_idx])
        best_curvature = float(self.curvatures[best_seg_idx])

        for seg_idx in candidate_segments:
            start = self.points[seg_idx]
            next_idx = (
                (seg_idx + 1) % len(self.points)
                if self.closed_loop
                else min(seg_idx + 1, len(self.points) - 1)
            )
            end = self.points[next_idx]
            segment = end - start
            segment_length_sq = float(segment @ segment)
            if segment_length_sq <= 1e-12:
                continue
            t = float(
                np.clip(((position - start) @ segment) / segment_length_sq, 0.0, 1.0)
            )
            projected = start + t * segment
            distance = float(np.linalg.norm(position - projected))
            if distance >= best_dist:
                continue
            best_dist = distance
            best_point = projected
            best_seg_idx = seg_idx
            best_t = t
            best_heading = interpolate_angle(
                float(self.headings[seg_idx]),
                float(self.headings[next_idx]),
                t,
            )
            best_curvature = float(
                (1.0 - t) * self.curvatures[seg_idx]
                + t * self.curvatures[next_idx]
            )

        normal = np.array([-math.sin(best_heading), math.cos(best_heading)])
        lateral_error = float((position - best_point) @ normal)
        closest_idx = (
            best_seg_idx
            if best_t < 0.5
            else (
                (best_seg_idx + 1) % len(self.points)
                if self.closed_loop
                else min(best_seg_idx + 1, len(self.points) - 1)
            )
        )
        return PathProjection(
            point=best_point,
            heading=best_heading,
            curvature=best_curvature,
            lateral_error=lateral_error,
            segment_idx=best_seg_idx,
            segment_t=best_t,
            closest_idx=closest_idx,
        )


@dataclass(frozen=True)
class OccupancyGrid:
    occupied: np.ndarray
    distance_to_obstacle_m: np.ndarray
    config: LocalGridConfig

    @property
    def has_obstacles(self) -> bool:
        return bool(np.any(self.occupied))

    def query_path(
        self,
        xy_points: np.ndarray,
        vehicle_pose: tuple[float, float, float],
        corridor_radius_m: float = 0.0,
        corridor_sample_step_m: float = 0.10,
        footprint_front_m: float = 0.0,
        footprint_rear_m: float = 0.0,
        path_sample_step_m: float = 0.05,
    ) -> tuple[bool, float]:
        if len(xy_points) == 0:
            return False, float("inf")
        dense_points = densify_path_points(xy_points, path_sample_step_m)
        query_points = swept_corridor_points(
            dense_points,
            corridor_radius_m,
            corridor_sample_step_m,
            footprint_front_m,
            footprint_rear_m,
        )
        local_xy = world_to_vehicle(query_points, vehicle_pose)
        rows, cols, inside = self.local_points_to_indices(local_xy)
        if not np.any(inside):
            return False, float("inf")
        inside_rows = rows[inside]
        inside_cols = cols[inside]
        collision = bool(np.any(self.occupied[inside_rows, inside_cols]))
        min_distance = float(np.min(self.distance_to_obstacle_m[inside_rows, inside_cols]))
        return collision, min_distance

    def local_points_to_indices(
        self,
        local_xy: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        return _local_points_to_indices(local_xy, self.occupied.shape, self.config)


@dataclass(frozen=True)
class CandidatePath:
    xy: np.ndarray
    s: np.ndarray
    d: np.ndarray
    s_dot: np.ndarray
    d_dot: np.ndarray
    cost: float
    min_clearance_m: float
    max_curvature: float


@dataclass
class FrenetPlanStats:
    total_candidates: int = 0
    collision_rejections: int = 0
    progress_rejections: int = 0
    heading_rejections: int = 0
    curvature_rejections: int = 0
    clearance_rejections: int = 0
    safe_candidates: int = 0
    best_clearance_m: float = 0.0


def build_occupancy_grid(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    config: LocalGridConfig,
    static_occupied: np.ndarray | None = None,
) -> OccupancyGrid:
    height = int(math.ceil((2.0 * config.half_width_m) / config.resolution_m))
    width = int(math.ceil((config.forward_m + config.rear_m) / config.resolution_m))
    raw = np.zeros((height, width), dtype=bool)
    if static_occupied is not None:
        raw |= static_occupied

    ranges = np.asarray(ranges, dtype=float)
    indices = np.arange(len(ranges), dtype=float)
    angles = angle_min + indices * angle_increment
    min_valid_range = max(float(range_min), MIN_VALID_SCAN_RANGE_M)
    valid = np.isfinite(ranges) & (ranges >= min_valid_range) & (ranges <= range_max)
    if np.any(valid):
        x = ranges[valid] * np.cos(angles[valid]) + config.scan_offset_x_m
        y = ranges[valid] * np.sin(angles[valid])
        rows, cols, inside = _local_points_to_indices(np.column_stack([x, y]), raw.shape, config)
        raw[rows[inside], cols[inside]] = True

    if not np.any(raw):
        occupied = np.zeros_like(raw, dtype=bool)
        clearance = np.full(raw.shape, float("inf"), dtype=float)
        return OccupancyGrid(
            occupied=occupied,
            distance_to_obstacle_m=clearance,
            config=config,
        )

    distance_to_raw = ndimage.distance_transform_edt(~raw) * config.resolution_m
    occupied = distance_to_raw <= config.inflation_radius_m
    clearance = np.maximum(0.0, distance_to_raw - config.inflation_radius_m)
    return OccupancyGrid(occupied=occupied, distance_to_obstacle_m=clearance, config=config)


def local_static_map_occupancy(
    map_occupied: np.ndarray,
    map_resolution: float,
    map_origin_xy: tuple[float, float],
    vehicle_pose: tuple[float, float, float],
    config: LocalGridConfig,
) -> np.ndarray:
    height = int(math.ceil((2.0 * config.half_width_m) / config.resolution_m))
    width = int(math.ceil((config.forward_m + config.rear_m) / config.resolution_m))
    col_values = (np.arange(width, dtype=float) + 0.5) * config.resolution_m - config.rear_m
    row_values = (np.arange(height, dtype=float) + 0.5) * config.resolution_m - config.half_width_m
    local_x, local_y = np.meshgrid(col_values, row_values)
    world_x, world_y = vehicle_to_world(local_x, local_y, vehicle_pose)
    map_rows, map_cols, inside = world_points_to_map_indices(
        world_x,
        world_y,
        map_resolution,
        map_origin_xy,
        map_occupied.shape,
    )
    occupied = np.ones((height, width), dtype=bool)
    occupied[inside] = map_occupied[map_rows[inside], map_cols[inside]]
    return occupied


def world_to_vehicle(points_xy: np.ndarray, vehicle_pose: tuple[float, float, float]) -> np.ndarray:
    x, y, yaw = vehicle_pose
    shifted = np.asarray(points_xy, dtype=float) - np.array([x, y], dtype=float)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return np.column_stack(
        [
            cos_yaw * shifted[:, 0] + sin_yaw * shifted[:, 1],
            -sin_yaw * shifted[:, 0] + cos_yaw * shifted[:, 1],
        ]
    )


def vehicle_to_world(
    local_x: np.ndarray,
    local_y: np.ndarray,
    vehicle_pose: tuple[float, float, float],
) -> tuple[np.ndarray, np.ndarray]:
    x, y, yaw = vehicle_pose
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    world_x = x + cos_yaw * local_x - sin_yaw * local_y
    world_y = y + sin_yaw * local_x + cos_yaw * local_y
    return world_x, world_y


def world_points_to_map_indices(
    world_x: np.ndarray,
    world_y: np.ndarray,
    map_resolution: float,
    map_origin_xy: tuple[float, float],
    map_shape: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    cols = np.floor((world_x - map_origin_xy[0]) / map_resolution).astype(int)
    rows_from_bottom = np.floor((world_y - map_origin_xy[1]) / map_resolution).astype(int)
    rows = map_shape[0] - 1 - rows_from_bottom
    inside = (
        (rows >= 0)
        & (rows < map_shape[0])
        & (cols >= 0)
        & (cols < map_shape[1])
    )
    return rows, cols, inside


def initial_frenet_state(
    reference: ReferencePath,
    position: np.ndarray,
    yaw: float,
    velocity_xy: np.ndarray,
    previous_s: float | None,
    previous_s_dot: float | None,
    dt: float | None,
    projection_window_m: float = 6.0,
) -> FrenetState:
    if previous_s is None:
        s, d, heading, _ = reference.project(position)
    else:
        s, d, heading, _ = reference.project_near(
            position,
            previous_s,
            projection_window_m,
        )
    tangent = np.array([math.cos(heading), math.sin(heading)])
    normal = np.array([-math.sin(heading), math.cos(heading)])
    s_dot = max(0.0, float(np.dot(velocity_xy, tangent)))
    d_dot = float(np.dot(velocity_xy, normal))
    if previous_s_dot is None or dt is None or dt <= 1e-6:
        s_ddot = 0.0
    else:
        s_ddot = float(np.clip((s_dot - previous_s_dot) / dt, -4.0, 4.0))
    return FrenetState(s=s, d=d, s_dot=s_dot, d_dot=d_dot, s_ddot=s_ddot, d_ddot=0.0)


def plan_frenet_path(
    reference: ReferencePath,
    state: FrenetState,
    occupancy: OccupancyGrid,
    vehicle_pose: tuple[float, float, float],
    config: FrenetPlannerConfig,
    stats: FrenetPlanStats | None = None,
    debug_candidates: list[CandidatePath] | None = None,
) -> CandidatePath | None:
    candidates: list[CandidatePath] = []
    planning_state = _forward_progress_planning_state(state)
    for d_final in _sample_range(config.d_min, config.d_max, config.d_step):
        for duration in _sample_range(config.t_min, config.t_max, config.t_step):
            d_coeff = solve_quintic_lateral(planning_state, d_final, duration)
            time_values = np.arange(
                0.0,
                duration + 0.5 * config.trajectory_dt,
                config.trajectory_dt,
            )
            d_values, d_dot_values, _, d_jerk_values = evaluate_quintic(d_coeff, time_values)
            for speed_final in _sample_range(config.v_min, config.v_max, config.v_step):
                if stats is not None:
                    stats.total_candidates += 1
                s_coeff = solve_quartic_longitudinal(planning_state, speed_final, duration)
                s_values, s_dot_values, _, s_jerk_values = evaluate_quartic(s_coeff, time_values)
                progress_ok = _has_valid_progress_profile(
                    s_values,
                    s_dot_values,
                    config,
                )
                if not progress_ok:
                    if stats is not None:
                        stats.progress_rejections += 1
                    continue
                xy = np.array([reference.sample(s, d)[0] for s, d in zip(s_values, d_values)])
                heading_jumps = estimate_heading_jumps(xy)
                max_heading_jump = (
                    float(np.max(np.abs(heading_jumps)))
                    if len(heading_jumps)
                    else 0.0
                )
                if max_heading_jump > config.max_heading_jump:
                    if stats is not None:
                        stats.heading_rejections += 1
                    continue
                collision, min_clearance = occupancy.query_path(
                    xy,
                    vehicle_pose,
                    config.corridor_radius_m,
                    config.corridor_sample_step_m,
                    config.footprint_front_m,
                    config.footprint_rear_m,
                    config.path_collision_sample_step_m,
                )
                if stats is not None:
                    stats.best_clearance_m = max(stats.best_clearance_m, min_clearance)
                if collision:
                    if stats is not None:
                        stats.collision_rejections += 1
                    continue
                if min_clearance < config.min_clearance_m:
                    if stats is not None:
                        stats.clearance_rejections += 1
                    continue
                scored_candidate = score_candidate(
                    xy,
                    s_values,
                    d_values,
                    s_dot_values,
                    d_dot_values,
                    d_jerk_values,
                    s_jerk_values,
                    duration,
                    min_clearance,
                    config,
                )
                if scored_candidate.max_curvature > config.max_curvature:
                    if stats is not None:
                        stats.curvature_rejections += 1
                    continue
                if stats is not None:
                    stats.safe_candidates += 1
                candidates.append(scored_candidate)
                if debug_candidates is not None:
                    debug_candidates.append(scored_candidate)
    if not candidates:
        return None
    candidates.sort(key=lambda candidate: candidate.cost)
    if debug_candidates is not None:
        debug_candidates.sort(key=lambda candidate: candidate.cost)
    return candidates[0]


def score_candidate(
    xy: np.ndarray,
    s_values: np.ndarray,
    d_values: np.ndarray,
    s_dot_values: np.ndarray,
    d_dot_values: np.ndarray,
    d_jerk_values: np.ndarray,
    s_jerk_values: np.ndarray,
    duration: float,
    min_clearance: float,
    config: FrenetPlannerConfig,
) -> CandidatePath:
    path_curvature = estimate_open_path_curvature(xy)
    if len(path_curvature):
        max_curvature = float(np.max(np.abs(path_curvature)))
    else:
        max_curvature = 0.0
    if len(path_curvature) > 1:
        curvature_rate = float(np.mean(np.abs(np.diff(path_curvature))))
    else:
        curvature_rate = 0.0
    lateral_shift = float(np.max(np.abs(np.diff(d_values)))) if len(d_values) > 1 else 0.0
    clearance_deficit = max(0.0, config.safe_clearance - min_clearance)
    speed_error = float((config.target_speed - s_dot_values[-1]) ** 2)

    cost = (
        config.weight_lateral_jerk * float(np.sum(d_jerk_values**2))
        + config.weight_longitudinal_jerk * float(np.sum(s_jerk_values**2))
        + config.weight_time * duration
        + config.weight_lateral_offset * float(d_values[-1] ** 2)
        + config.weight_speed_error * speed_error
        + config.weight_obstacle_clearance * clearance_deficit**2
        + config.weight_curvature * max(0.0, max_curvature - config.max_curvature) ** 2
        + config.weight_curvature_rate * curvature_rate
        + config.weight_lateral_shift * lateral_shift
    )
    return CandidatePath(
        xy=xy,
        s=s_values,
        d=d_values,
        s_dot=s_dot_values,
        d_dot=d_dot_values,
        cost=float(cost),
        min_clearance_m=float(min_clearance),
        max_curvature=max_curvature,
    )


def solve_quintic_lateral(state: FrenetState, d_final: float, duration: float) -> np.ndarray:
    a0 = state.d
    a1 = state.d_dot
    a2 = 0.5 * state.d_ddot
    t = duration
    matrix = np.array(
        [
            [t**3, t**4, t**5],
            [3.0 * t**2, 4.0 * t**3, 5.0 * t**4],
            [6.0 * t, 12.0 * t**2, 20.0 * t**3],
        ],
        dtype=float,
    )
    rhs = np.array(
        [
            d_final - (a0 + a1 * t + a2 * t**2),
            -(a1 + 2.0 * a2 * t),
            -(2.0 * a2),
        ],
        dtype=float,
    )
    a3, a4, a5 = np.linalg.solve(matrix, rhs)
    return np.array([a0, a1, a2, a3, a4, a5], dtype=float)


def solve_quartic_longitudinal(
    state: FrenetState,
    speed_final: float,
    duration: float,
) -> np.ndarray:
    b0 = state.s
    b1 = state.s_dot
    b2 = 0.5 * state.s_ddot
    t = duration
    matrix = np.array(
        [
            [3.0 * t**2, 4.0 * t**3],
            [6.0 * t, 12.0 * t**2],
        ],
        dtype=float,
    )
    rhs = np.array(
        [
            speed_final - (b1 + 2.0 * b2 * t),
            -(2.0 * b2),
        ],
        dtype=float,
    )
    b3, b4 = np.linalg.solve(matrix, rhs)
    return np.array([b0, b1, b2, b3, b4], dtype=float)


def evaluate_quintic(
    coefficients: np.ndarray,
    time_values: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    a0, a1, a2, a3, a4, a5 = coefficients
    t = time_values
    position = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
    velocity = a1 + 2.0 * a2 * t + 3.0 * a3 * t**2 + 4.0 * a4 * t**3 + 5.0 * a5 * t**4
    acceleration = 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t**2 + 20.0 * a5 * t**3
    jerk = 6.0 * a3 + 24.0 * a4 * t + 60.0 * a5 * t**2
    return position, velocity, acceleration, jerk


def evaluate_quartic(
    coefficients: np.ndarray,
    time_values: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    b0, b1, b2, b3, b4 = coefficients
    t = time_values
    position = b0 + b1 * t + b2 * t**2 + b3 * t**3 + b4 * t**4
    velocity = b1 + 2.0 * b2 * t + 3.0 * b3 * t**2 + 4.0 * b4 * t**3
    acceleration = 2.0 * b2 + 6.0 * b3 * t + 12.0 * b4 * t**2
    jerk = 6.0 * b3 + 24.0 * b4 * t
    return position, velocity, acceleration, jerk


def estimate_open_path_curvature(points: np.ndarray) -> np.ndarray:
    if len(points) < 3:
        return np.zeros(len(points), dtype=float)
    filtered = _compress_path_samples(points, MIN_CURVATURE_SAMPLE_SPACING_M)
    if len(filtered) < 3:
        return np.zeros(len(filtered), dtype=float)
    cumulative_s = _cumulative_arc_length(filtered)
    dx = np.gradient(filtered[:, 0], cumulative_s)
    dy = np.gradient(filtered[:, 1], cumulative_s)
    ddx = np.gradient(dx, cumulative_s)
    ddy = np.gradient(dy, cumulative_s)
    denominator = np.maximum((dx * dx + dy * dy) ** 1.5, 1e-6)
    return (dx * ddy - dy * ddx) / denominator


def estimate_heading_jumps(points: np.ndarray) -> np.ndarray:
    if len(points) < 3:
        return np.zeros(0, dtype=float)
    segment_vectors = np.diff(points, axis=0)
    segment_lengths = np.linalg.norm(segment_vectors, axis=1)
    valid = segment_lengths > 1e-6
    if np.count_nonzero(valid) < 2:
        return np.zeros(0, dtype=float)
    headings = np.arctan2(segment_vectors[valid, 1], segment_vectors[valid, 0])
    return np.array(
        [wrap_angle(float(curr - prev)) for prev, curr in zip(headings[:-1], headings[1:])],
        dtype=float,
    )


def swept_corridor_points(
    points: np.ndarray,
    corridor_radius_m: float,
    corridor_sample_step_m: float,
    footprint_front_m: float = 0.0,
    footprint_rear_m: float = 0.0,
) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if len(points) == 0:
        return points.reshape(0, 2)
    radius = max(0.0, float(corridor_radius_m))
    front = max(0.0, float(footprint_front_m))
    rear = max(0.0, float(footprint_rear_m))
    if radius <= 1e-9 and front <= 1e-9 and rear <= 1e-9:
        return points
    step = max(1e-3, float(corridor_sample_step_m))
    offsets = np.arange(-radius, radius + 0.5 * step, step, dtype=float)
    if offsets[-1] < radius:
        offsets = np.append(offsets, radius)
    if not np.any(np.isclose(offsets, 0.0)):
        offsets = np.sort(np.append(offsets, 0.0))

    headings = estimate_path_headings(points)
    longitudinal_offsets = np.arange(-rear, front + 0.5 * step, step, dtype=float)
    if longitudinal_offsets[-1] < front:
        longitudinal_offsets = np.append(longitudinal_offsets, front)
    if not np.any(np.isclose(longitudinal_offsets, 0.0)):
        longitudinal_offsets = np.sort(np.append(longitudinal_offsets, 0.0))
    tangents = np.column_stack([np.cos(headings), np.sin(headings)])
    normals = np.column_stack([-np.sin(headings), np.cos(headings)])
    expanded = (
        points[:, None, None, :]
        + longitudinal_offsets[None, :, None, None] * tangents[:, None, None, :]
        + offsets[None, None, :, None] * normals[:, None, None, :]
    )
    return expanded.reshape(-1, 2)


def densify_path_points(points: np.ndarray, max_step_m: float) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return points.reshape(-1, 2)
    max_step = float(max_step_m)
    if max_step <= 0.0:
        return points

    dense_points = [points[0]]
    for start, end in zip(points[:-1], points[1:]):
        delta = end - start
        distance = float(np.linalg.norm(delta))
        if distance <= 1e-12:
            continue
        subdivisions = max(1, int(math.ceil(distance / max_step)))
        for step_idx in range(1, subdivisions + 1):
            dense_points.append(start + (step_idx / subdivisions) * delta)
    return np.asarray(dense_points, dtype=float)


def trim_path_to_position(
    points: np.ndarray,
    position: np.ndarray,
    min_remaining_length_m: float = 0.0,
    anchor_lookahead_m: float = 0.0,
) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return points.reshape(-1, 2)

    headings = estimate_path_headings(points)
    curvatures = compute_path_curvatures(points, closed_loop=False)
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    kdtree = KDTree(points)
    projection = project_to_path(
        np.asarray(position, dtype=float),
        points,
        kdtree,
        headings,
        curvatures,
        closed_loop=False,
    )
    if anchor_lookahead_m > 0.0:
        projection = advance_projection_along_path(
            np.asarray(position, dtype=float),
            projection,
            float(anchor_lookahead_m),
            points,
            headings,
            curvatures,
            segment_lengths,
            closed_loop=False,
        )

    anchor = np.asarray(projection.point, dtype=float).reshape(1, 2)
    remaining_points = points[min(projection.segment_idx + 1, len(points) - 1) :]
    trimmed = np.vstack([anchor, remaining_points])
    if polyline_length(trimmed) < max(0.0, float(min_remaining_length_m)):
        return np.empty((0, 2), dtype=float)
    return trimmed


def sample_reference_segment(
    reference: ReferencePath,
    start_s: float,
    length_m: float,
    step_m: float,
    d: float = 0.0,
) -> np.ndarray:
    length = max(0.0, float(length_m))
    step = max(1e-3, float(step_m))
    sample_s = np.arange(0.0, length + 0.5 * step, step, dtype=float)
    if len(sample_s) == 0 or sample_s[-1] < length:
        sample_s = np.append(sample_s, length)
    return np.array(
        [reference.sample(start_s + offset, d)[0] for offset in sample_s],
        dtype=float,
    )


def polyline_length(points: np.ndarray) -> float:
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def estimate_path_headings(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return np.zeros(len(points), dtype=float)
    headings = np.zeros(len(points), dtype=float)
    for idx in range(len(points)):
        prev_point = points[max(0, idx - 1)]
        next_point = points[min(len(points) - 1, idx + 1)]
        delta = next_point - prev_point
        if float(delta @ delta) <= 1e-12:
            headings[idx] = headings[idx - 1] if idx > 0 else 0.0
        else:
            headings[idx] = math.atan2(float(delta[1]), float(delta[0]))
    return headings


def _has_valid_progress_profile(
    s_values: np.ndarray,
    s_dot_values: np.ndarray,
    config: FrenetPlannerConfig,
) -> bool:
    if np.any(s_dot_values < -1e-6):
        return False
    progress_steps = np.diff(s_values)
    if np.any(progress_steps < -1e-6):
        return False
    total_progress = float(s_values[-1] - s_values[0]) if len(s_values) else 0.0
    if total_progress < config.min_progress_step_m:
        return False
    return True


def _forward_progress_planning_state(state: FrenetState) -> FrenetState:
    s_ddot = state.s_ddot
    if abs(s_ddot) > 1.0:
        s_ddot = 0.0
    if s_ddot >= 0.0 and abs(s_ddot - state.s_ddot) <= 1e-9:
        return state
    # Odom/projection-derived acceleration is noisy around path switches and
    # obstacle transitions. Large seeds make quartic candidates either reverse
    # or surge into the obstacle before lateral motion has time to develop.
    return FrenetState(
        s=state.s,
        d=state.d,
        s_dot=state.s_dot,
        d_dot=state.d_dot,
        s_ddot=max(0.0, s_ddot),
        d_ddot=state.d_ddot,
    )


def _sample_range(start: float, stop: float, step: float) -> np.ndarray:
    if step <= 0.0:
        return np.array([start], dtype=float)
    count = int(math.floor((stop - start) / step + 0.5)) + 1
    return start + step * np.arange(max(1, count), dtype=float)


def _compress_path_samples(points: np.ndarray, min_spacing_m: float) -> np.ndarray:
    if len(points) < 2:
        return np.asarray(points, dtype=float)
    filtered = [np.asarray(points[0], dtype=float)]
    min_spacing_m = max(0.0, float(min_spacing_m))
    for point in np.asarray(points[1:], dtype=float):
        if np.linalg.norm(point - filtered[-1]) >= min_spacing_m:
            filtered.append(point)
    if np.linalg.norm(np.asarray(points[-1], dtype=float) - filtered[-1]) > 1e-9:
        filtered.append(np.asarray(points[-1], dtype=float))
    return np.asarray(filtered, dtype=float)


def _cumulative_arc_length(points: np.ndarray) -> np.ndarray:
    if len(points) < 2:
        return np.zeros(len(points), dtype=float)
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(segment_lengths)])


def _local_points_to_indices(
    local_xy: np.ndarray,
    shape: tuple[int, int],
    config: LocalGridConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    cols = np.floor((local_xy[:, 0] + config.rear_m) / config.resolution_m).astype(int)
    rows = np.floor((local_xy[:, 1] + config.half_width_m) / config.resolution_m).astype(int)
    inside = (cols >= 0) & (cols < shape[1]) & (rows >= 0) & (rows < shape[0])
    return rows, cols, inside

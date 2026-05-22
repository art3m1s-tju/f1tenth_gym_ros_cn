"""Pure geometry utilities for path projection (no ROS dependency)."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import KDTree

from pnc_rc.lqr.math import wrap_angle


def interpolate_angle(start_angle: float, end_angle: float, t: float) -> float:
    return wrap_angle(start_angle + t * wrap_angle(end_angle - start_angle))


def compute_curvature_limited_speed(
    target_speed: float,
    curvature_ref: float,
    delta_cmd: float,
    wheelbase: float,
    max_lateral_accel: float,
    min_speed: float,
) -> float:
    steering_curvature = abs(math.tan(delta_cmd)) / wheelbase
    curvature = max(abs(curvature_ref), steering_curvature, 1e-6)
    curve_speed = math.sqrt(max_lateral_accel / curvature)
    return float(np.clip(min(target_speed, curve_speed), min_speed, target_speed))


@dataclass(frozen=True)
class PathProjection:
    point: np.ndarray
    heading: float
    curvature: float
    lateral_error: float
    segment_idx: int
    segment_t: float
    closest_idx: int


def project_to_path(
    position: np.ndarray,
    points: np.ndarray,
    kdtree: KDTree,
    headings: np.ndarray,
    curvatures: np.ndarray,
    closed_loop: bool = True,
) -> PathProjection:
    _, closest_idx = kdtree.query(position)
    closest_idx = int(closest_idx)
    n = len(points)

    if closed_loop:
        candidates = [(closest_idx - 1) % n, closest_idx, (closest_idx + 1) % n]
    else:
        max_seg = n - 2
        candidates = sorted({
            int(np.clip(closest_idx - 1, 0, max_seg)),
            int(np.clip(closest_idx, 0, max_seg)),
        })

    best_dist = float("inf")
    best_heading = float(headings[closest_idx])
    best_curvature = float(curvatures[closest_idx])
    best_point = points[closest_idx].copy()
    best_seg_idx = candidates[0]
    best_t = 0.0

    for seg_idx in candidates:
        start = points[seg_idx]
        end = points[(seg_idx + 1) % n]
        seg = end - start
        seg_len_sq = float(seg @ seg)
        if seg_len_sq <= 1e-12:
            continue
        t = float(np.clip(((position - start) @ seg) / seg_len_sq, 0.0, 1.0))
        proj = start + t * seg
        dist = float(np.linalg.norm(position - proj))
        if dist < best_dist:
            best_dist = dist
            best_point = proj
            best_seg_idx = seg_idx
            best_t = t
            next_idx = (seg_idx + 1) % n
            best_heading = interpolate_angle(float(headings[seg_idx]), float(headings[next_idx]), t)
            best_curvature = float((1.0 - t) * curvatures[seg_idx] + t * curvatures[next_idx])

    normal = np.array([-math.sin(best_heading), math.cos(best_heading)])
    lateral_error = float((position - best_point) @ normal)

    return PathProjection(
        point=best_point,
        heading=best_heading,
        curvature=best_curvature,
        lateral_error=lateral_error,
        segment_idx=best_seg_idx,
        segment_t=best_t,
        closest_idx=closest_idx,
    )


def advance_projection_along_path(
    position: np.ndarray,
    projection: PathProjection,
    lookahead_distance: float,
    points: np.ndarray,
    headings: np.ndarray,
    curvatures: np.ndarray,
    segment_lengths: np.ndarray,
    closed_loop: bool = True,
) -> PathProjection:
    """Move a path projection forward and recompute error at the preview point."""
    if lookahead_distance <= 1e-9 or len(points) < 2:
        return projection

    point_count = len(points)
    segment_count = point_count if closed_loop else point_count - 1
    if segment_count <= 0:
        return projection

    segment_idx = int(np.clip(projection.segment_idx, 0, segment_count - 1))
    segment_t = float(np.clip(projection.segment_t, 0.0, 1.0))
    distance_left = lookahead_distance

    while distance_left > 1e-9:
        segment_length = float(segment_lengths[segment_idx])
        if segment_length <= 1e-9:
            next_segment = _next_segment_index(segment_idx, segment_count, closed_loop)
            if next_segment == segment_idx:
                break
            segment_idx = next_segment
            segment_t = 0.0
            continue

        remaining_segment = (1.0 - segment_t) * segment_length
        if remaining_segment <= 1e-9:
            next_segment = _next_segment_index(segment_idx, segment_count, closed_loop)
            if next_segment == segment_idx:
                segment_t = 1.0
                break
            segment_idx = next_segment
            segment_t = 0.0
            continue

        if distance_left <= remaining_segment:
            segment_t = min(1.0, segment_t + distance_left / segment_length)
            break

        distance_left -= remaining_segment
        next_segment = _next_segment_index(segment_idx, segment_count, closed_loop)
        if next_segment == segment_idx:
            segment_t = 1.0
            break
        segment_idx = next_segment
        segment_t = 0.0

    next_idx = (segment_idx + 1) % point_count
    if not closed_loop:
        next_idx = min(segment_idx + 1, point_count - 1)

    start = points[segment_idx]
    end = points[next_idx]
    preview_point = start + segment_t * (end - start)
    preview_heading = interpolate_angle(
        float(headings[segment_idx]),
        float(headings[next_idx]),
        segment_t,
    )
    preview_curvature = float(
        (1.0 - segment_t) * curvatures[segment_idx]
        + segment_t * curvatures[next_idx]
    )
    normal = np.array([-math.sin(preview_heading), math.cos(preview_heading)])
    lateral_error = float((position - preview_point) @ normal)
    closest_idx = segment_idx if segment_t < 0.5 else next_idx

    return PathProjection(
        point=preview_point,
        heading=preview_heading,
        curvature=preview_curvature,
        lateral_error=lateral_error,
        segment_idx=segment_idx,
        segment_t=segment_t,
        closest_idx=closest_idx,
    )


def _next_segment_index(
    segment_idx: int,
    segment_count: int,
    closed_loop: bool,
) -> int:
    if closed_loop:
        return (segment_idx + 1) % segment_count
    return min(segment_idx + 1, segment_count - 1)

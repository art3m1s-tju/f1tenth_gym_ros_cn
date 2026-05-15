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

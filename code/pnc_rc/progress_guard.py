"""Helpers for progress-consistent closest-point selection on closed-loop paths."""
from __future__ import annotations

import math

import numpy as np


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def select_progress_consistent_idx(
    *,
    candidate_indices: list[int],
    waypoints: np.ndarray,
    reference_headings: np.ndarray | None,
    current_pos: np.ndarray,
    prev_closest_idx: int,
    heading_threshold_rad: float,
) -> tuple[int, float]:
    """Pick a local closest point while preferring heading-continuous candidates."""
    if not candidate_indices:
        return prev_closest_idx, float("inf")

    prev_heading = None
    if (
        reference_headings is not None
        and 0 <= prev_closest_idx < len(reference_headings)
    ):
        prev_heading = float(reference_headings[prev_closest_idx])

    best_idx = prev_closest_idx
    best_distance_sq = float("inf")
    best_heading_consistent_idx = prev_closest_idx
    best_heading_consistent_distance_sq = float("inf")

    seen_indices: set[int] = set()
    for candidate_idx in candidate_indices:
        if candidate_idx in seen_indices:
            continue
        seen_indices.add(candidate_idx)
        delta = waypoints[candidate_idx] - current_pos
        distance_sq = float(np.dot(delta, delta))
        if distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_idx = candidate_idx

        if prev_heading is None:
            continue
        candidate_heading = float(reference_headings[candidate_idx])
        heading_error = abs(wrap_angle(candidate_heading - prev_heading))
        if heading_error <= heading_threshold_rad and distance_sq < best_heading_consistent_distance_sq:
            best_heading_consistent_distance_sq = distance_sq
            best_heading_consistent_idx = candidate_idx

    if best_heading_consistent_distance_sq < float("inf"):
        return best_heading_consistent_idx, math.sqrt(max(best_heading_consistent_distance_sq, 0.0))
    return best_idx, math.sqrt(max(best_distance_sq, 0.0))


def build_forward_progress_indices(
    *,
    point_count: int,
    start_idx: int,
    max_forward_count: int,
) -> list[int]:
    """Build a finite forward-only index window on a closed loop."""
    if point_count <= 0:
        return []
    clamped_count = max(1, min(point_count, int(max_forward_count)))
    return [((int(start_idx) + offset) % point_count) for offset in range(clamped_count)]


def build_nonwrapping_forward_progress_indices(
    *,
    point_count: int,
    start_idx: int,
    max_forward_count: int,
) -> list[int]:
    """Build a finite forward-only index window without wrapping around the seam."""
    if point_count <= 0:
        return []
    clamped_start = int(min(max(int(start_idx), 0), point_count - 1))
    clamped_count = max(1, int(max_forward_count))
    end_idx = min(point_count, clamped_start + clamped_count)
    return list(range(clamped_start, end_idx))


def compute_progress_search_limit(
    *,
    lookahead_distance: float,
    nominal_path_step: float,
    min_points: int,
    scale: float,
    point_count: int,
) -> int:
    """Convert a metric lookahead into a finite forward search span in points."""
    if point_count <= 0:
        return 0
    safe_step = max(float(nominal_path_step), 1e-3)
    desired = int(
        math.ceil(max(0.0, float(lookahead_distance)) * max(1.0, float(scale)) / safe_step)
    )
    return min(point_count, max(int(min_points), desired))

"""Vectorized Frenet candidate profile generation."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CandidateProfiles:
    """Batched Frenet polynomial profiles for candidate evaluation."""

    d_finals: np.ndarray
    durations: np.ndarray
    speed_finals: np.ndarray
    lengths: np.ndarray
    s_values: np.ndarray
    d_values: np.ndarray
    s_dot_values: np.ndarray
    d_dot_values: np.ndarray
    s_jerk_values: np.ndarray
    d_jerk_values: np.ndarray
    backend: str


def generate_candidate_profiles(
    *,
    state_s: float,
    state_d: float,
    state_s_dot: float,
    state_d_dot: float,
    state_s_ddot: float,
    state_d_ddot: float,
    d_samples: np.ndarray,
    duration_samples: np.ndarray,
    speed_samples: np.ndarray,
    trajectory_dt: float,
) -> CandidateProfiles:
    """Generate all Frenet polynomial profiles for the sampled endpoints.

    Candidate ordering matches the original nested loops:
    `d_final -> duration -> speed_final`.
    """
    d_finals, durations, speed_finals = _candidate_parameter_grid(
        d_samples,
        duration_samples,
        speed_samples,
    )
    lengths = _profile_lengths(durations, trajectory_dt)
    max_points = int(np.max(lengths)) if len(lengths) else 0
    count = len(d_finals)
    shape = (count, max_points)
    s_values = np.empty(shape, dtype=float)
    d_values = np.empty(shape, dtype=float)
    s_dot_values = np.empty(shape, dtype=float)
    d_dot_values = np.empty(shape, dtype=float)
    s_jerk_values = np.empty(shape, dtype=float)
    d_jerk_values = np.empty(shape, dtype=float)
    if count == 0:
        return CandidateProfiles(
            d_finals,
            durations,
            speed_finals,
            lengths,
            s_values,
            d_values,
            s_dot_values,
            d_dot_values,
            s_jerk_values,
            d_jerk_values,
            "numpy",
        )

    t = (np.arange(max_points, dtype=float) * trajectory_dt)[None, :]
    duration = np.maximum(1e-6, durations)[:, None]

    s0 = float(state_s)
    s1 = float(state_s_dot)
    s2 = 0.5 * float(state_s_ddot)
    target_delta_v = speed_finals[:, None] - s1 - 2.0 * s2 * duration
    s4 = -(target_delta_v + s2 * duration) / (2.0 * duration**3)
    s3 = (-2.0 * s2 - 12.0 * s4 * duration**2) / (6.0 * duration)

    d0 = float(state_d)
    d1 = float(state_d_dot)
    d2 = 0.5 * float(state_d_ddot)
    delta = d_finals[:, None] - (d0 + d1 * duration + d2 * duration**2)
    velocity_delta = -(d1 + 2.0 * d2 * duration)
    acceleration_delta = -2.0 * d2
    d3 = (
        10.0 * delta / duration**3
        - 4.0 * velocity_delta / duration**2
        + 0.5 * acceleration_delta / duration
    )
    d4 = (
        -15.0 * delta / duration**4
        + 7.0 * velocity_delta / duration**3
        - acceleration_delta / duration**2
    )
    d5 = (
        6.0 * delta / duration**5
        - 3.0 * velocity_delta / duration**4
        + 0.5 * acceleration_delta / duration**3
    )

    s_values[:, :] = s0 + s1 * t + s2 * t**2 + s3 * t**3 + s4 * t**4
    s_dot_values[:, :] = s1 + 2.0 * s2 * t + 3.0 * s3 * t**2 + 4.0 * s4 * t**3
    s_jerk_values[:, :] = 6.0 * s3 + 24.0 * s4 * t
    d_values[:, :] = d0 + d1 * t + d2 * t**2 + d3 * t**3 + d4 * t**4 + d5 * t**5
    d_dot_values[:, :] = (
        d1 + 2.0 * d2 * t + 3.0 * d3 * t**2 + 4.0 * d4 * t**3 + 5.0 * d5 * t**4
    )
    d_jerk_values[:, :] = 6.0 * d3 + 24.0 * d4 * t + 60.0 * d5 * t**2
    return CandidateProfiles(
        d_finals,
        durations,
        speed_finals,
        lengths,
        s_values,
        d_values,
        s_dot_values,
        d_dot_values,
        s_jerk_values,
        d_jerk_values,
        "numpy",
    )


def _candidate_parameter_grid(
    d_samples: np.ndarray,
    duration_samples: np.ndarray,
    speed_samples: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    d_grid, duration_grid, speed_grid = np.meshgrid(
        np.asarray(d_samples, dtype=float),
        np.asarray(duration_samples, dtype=float),
        np.asarray(speed_samples, dtype=float),
        indexing="ij",
    )
    return d_grid.ravel(), duration_grid.ravel(), speed_grid.ravel()


def _profile_lengths(durations: np.ndarray, trajectory_dt: float) -> np.ndarray:
    durations = np.asarray(durations, dtype=float)
    step = max(1e-6, float(trajectory_dt))
    return (np.floor(durations / step + 0.5).astype(np.int32) + 1)

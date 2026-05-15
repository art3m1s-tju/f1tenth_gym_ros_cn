#!/usr/bin/env python3
"""Evaluate minimal Pure Pursuit logs for lookahead-distance tuning."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/mpl")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from package_paths import get_default_output_root

DEFAULT_OUTPUT_DIR = get_default_output_root() / "evaluation" / "minimal_pp"
REQUIRED_COLUMNS = {"time", "e_y"}
P95_PERCENTILE = 95.0


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate one or more minimal Pure Pursuit logs. "
            "The main tuning targets are abs(e_y).mean and abs(e_y).max."
        )
    )
    parser.add_argument(
        "--log",
        nargs="+",
        required=True,
        help="One or more tracking CSV logs to evaluate.",
    )
    parser.add_argument(
        "--reference-trajectory-csv",
        nargs="+",
        default=None,
        help=(
            "Optional original trajectory CSV file(s) used for path-overlay plotting. "
            "Each file must contain x/y columns. Provide either one file to reuse for all "
            "logs, or the same number of files as --log."
        ),
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_OUTPUT_DIR),
        help="Directory used for plots and summary tables.",
    )
    parser.add_argument(
        "--lateral-threshold",
        type=float,
        default=0.05,
        help="Target lateral-error threshold in meters. Default: 0.05 m.",
    )
    parser.add_argument(
        "--heading-threshold-deg",
        type=float,
        default=3.0,
        help="Target heading-error threshold in degrees. Default: 3.0 deg.",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Skip figure generation and only print/save summary tables.",
    )
    parser.add_argument(
        "--disable-start-trim",
        action="store_true",
        help="Keep the initial off-track startup segment instead of trimming it automatically.",
    )
    parser.add_argument(
        "--start-trim-lateral-threshold",
        type=float,
        default=0.05,
        help="Auto-trim begins once abs(e_y) stays below this threshold. Default: 0.05 m.",
    )
    parser.add_argument(
        "--start-trim-heading-threshold-deg",
        type=float,
        default=5.0,
        help="Auto-trim begins once abs(e_psi) stays below this threshold. Default: 5.0 deg.",
    )
    parser.add_argument(
        "--start-trim-hold-s",
        type=float,
        default=0.5,
        help="Required duration for the start-trim condition to hold. Default: 0.5 s.",
    )
    parser.add_argument(
        "--disable-end-trim",
        action="store_true",
        help="Keep the terminal tail after the path end instead of trimming it automatically.",
    )
    parser.add_argument(
        "--end-trim-distance-threshold",
        type=float,
        default=0.25,
        help=(
            "Auto end-trim begins once the reference projection is within this distance of "
            "the final reference point. Default: 0.25 m."
        ),
    )
    parser.add_argument(
        "--focus-straight-segments",
        action="store_true",
        help="Evaluate only long, near-zero-curvature straight segments from the tracked reference path.",
    )
    parser.add_argument(
        "--straight-curvature-threshold",
        type=float,
        default=0.03,
        help="Maximum absolute reference curvature used to classify straight samples. Default: 0.03 1/m.",
    )
    parser.add_argument(
        "--straight-min-length-m",
        type=float,
        default=6.0,
        help="Minimum contiguous straight-segment length kept for evaluation. Default: 6.0 m.",
    )
    parser.add_argument(
        "--save-full-run-overlay",
        action="store_true",
        help=(
            "When the evaluation scope is narrowed, also save an additional path-overlay "
            "figure for the full unfiltered run."
        ),
    )
    return parser.parse_args()


def ensure_output_dir(path_str: str) -> Path:
    """Create the output directory if needed."""
    output_dir = Path(path_str).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def load_log(log_path: Path) -> pd.DataFrame:
    """Read and normalize one log file."""
    if not log_path.exists():
        raise FileNotFoundError(f"Tracking log does not exist: {log_path}")

    dataframe = pd.read_csv(log_path)
    missing = sorted(REQUIRED_COLUMNS - set(dataframe.columns))
    if missing:
        raise ValueError(f"Log is missing required columns: {missing}")

    numeric_columns = [
        "time",
        "e_y",
        "e_psi",
        "abs_e_y",
        "abs_e_psi_deg",
        "lookahead_distance",
        "delta_pp",
        "delta_heading",
        "delta_cmd",
        "v_actual",
        "v_ref",
        "yaw_vehicle",
        "compute_time_ms",
    ]
    for column in numeric_columns:
        if column in dataframe.columns:
            dataframe[column] = pd.to_numeric(dataframe[column], errors="coerce")

    dataframe = dataframe.replace([np.inf, -np.inf], np.nan)
    dataframe = dataframe.dropna(subset=["time", "e_y"])
    dataframe = dataframe.sort_values("time", kind="mergesort").drop_duplicates("time")
    dataframe = dataframe.reset_index(drop=True)

    if len(dataframe) < 2:
        raise ValueError(f"Not enough valid samples in log: {log_path}")

    dataframe["time_rel_s"] = dataframe["time"] - float(dataframe["time"].iloc[0])
    dataframe["abs_e_y"] = np.abs(dataframe["e_y"].to_numpy(dtype=float))

    if "e_psi" in dataframe.columns:
        dataframe["e_psi_deg"] = np.degrees(dataframe["e_psi"].to_numpy(dtype=float))
        dataframe["abs_e_psi_deg"] = np.abs(dataframe["e_psi_deg"].to_numpy(dtype=float))
    elif "abs_e_psi_deg" not in dataframe.columns:
        dataframe["abs_e_psi_deg"] = np.nan

    if "lookahead_distance" in dataframe.columns and dataframe["lookahead_distance"].notna().any():
        lookahead_distance = float(dataframe["lookahead_distance"].dropna().iloc[0])
    else:
        lookahead_distance = float("nan")

    dataframe.attrs["lookahead_distance"] = lookahead_distance
    dataframe.attrs["log_path"] = str(log_path.resolve())
    dataframe.attrs["duration_s"] = float(dataframe["time_rel_s"].iloc[-1])
    dataframe.attrs["raw_sample_count"] = int(len(dataframe))
    dataframe.attrs["raw_duration_s"] = float(dataframe["time_rel_s"].iloc[-1])
    dataframe.attrs["trim_start_s"] = 0.0
    dataframe.attrs["trim_end_s"] = 0.0
    return dataframe


def load_reference_trajectory_csv(csv_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load full reference-trajectory x/y arrays from a CSV file."""
    if not csv_path.exists():
        raise FileNotFoundError(f"Reference trajectory CSV does not exist: {csv_path}")

    dataframe = pd.read_csv(csv_path)
    normalized = {str(column).strip().lower(): column for column in dataframe.columns}
    if "x" not in normalized or "y" not in normalized:
        raise ValueError(
            f"Reference trajectory CSV must contain 'x' and 'y' columns: {csv_path}"
        )

    x_col = normalized["x"]
    y_col = normalized["y"]
    x = pd.to_numeric(dataframe[x_col], errors="coerce").to_numpy(dtype=float)
    y = pd.to_numeric(dataframe[y_col], errors="coerce").to_numpy(dtype=float)
    finite_mask = np.isfinite(x) & np.isfinite(y)
    x = x[finite_mask]
    y = y[finite_mask]

    if len(x) < 2:
        raise ValueError(
            f"Reference trajectory CSV needs at least 2 valid x/y points: {csv_path}"
        )
    return x, y


def trim_startup_segment(
    dataframe: pd.DataFrame,
    lateral_threshold: float,
    heading_threshold_deg: float,
    hold_time_s: float,
) -> pd.DataFrame:
    """Trim the initial off-track startup segment once the vehicle settles near the path."""
    if len(dataframe) < 3:
        return dataframe

    abs_e_y = dataframe["abs_e_y"].to_numpy(dtype=float)
    if "abs_e_psi_deg" in dataframe.columns:
        abs_e_psi_deg = dataframe["abs_e_psi_deg"].to_numpy(dtype=float)
        finite_heading = np.isfinite(abs_e_psi_deg)
        settled_mask = abs_e_y <= lateral_threshold
        settled_mask &= (~finite_heading) | (abs_e_psi_deg <= heading_threshold_deg)
    else:
        settled_mask = abs_e_y <= lateral_threshold

    time_rel_s = dataframe["time_rel_s"].to_numpy(dtype=float)
    dt = np.diff(time_rel_s)
    positive_dt = dt[dt > 0.0]
    nominal_dt = float(np.median(positive_dt)) if positive_dt.size > 0 else 0.01
    hold_samples = max(2, int(np.ceil(hold_time_s / max(nominal_dt, 1e-3))))

    trim_idx = 0
    for idx in range(len(dataframe) - hold_samples + 1):
        if np.all(settled_mask[idx : idx + hold_samples]):
            trim_idx = idx
            break

    if trim_idx <= 0:
        return dataframe

    trimmed = dataframe.iloc[trim_idx:].copy().reset_index(drop=True)
    trim_start_s = float(time_rel_s[trim_idx])
    trimmed["time_rel_s"] = trimmed["time"] - float(trimmed["time"].iloc[0])

    trimmed.attrs = dict(dataframe.attrs)
    trimmed.attrs["trim_start_s"] = trim_start_s
    trimmed.attrs["sample_count_after_trim"] = int(len(trimmed))
    trimmed.attrs["duration_s"] = float(trimmed["time_rel_s"].iloc[-1])
    return trimmed


def trim_terminal_tail(
    dataframe: pd.DataFrame,
    distance_threshold: float,
    reference_trajectory: tuple[np.ndarray, np.ndarray] | None = None,
) -> pd.DataFrame:
    """Trim the terminal tail after the vehicle has effectively reached the path end.

    The trim is based on the reference projection rather than the actual vehicle pose:
    once the projected reference point reaches a small neighbourhood of the final
    reference point, all later samples are treated as post-finish tail and removed.
    """
    if reference_trajectory is not None:
        reference_x, reference_y = reference_trajectory
        if len(reference_x) >= 3 and len(reference_y) >= 3:
            start = np.array([reference_x[0], reference_y[0]], dtype=float)
            end = np.array([reference_x[-1], reference_y[-1]], dtype=float)
            deltas = np.diff(np.column_stack((reference_x, reference_y)), axis=0)
            segment_lengths = np.linalg.norm(deltas, axis=1)
            finite_segment_lengths = segment_lengths[np.isfinite(segment_lengths) & (segment_lengths > 1e-6)]
            nominal_step = (
                float(np.median(finite_segment_lengths))
                if finite_segment_lengths.size > 0
                else 0.0
            )
            closed_threshold = max(distance_threshold * 2.0, nominal_step * 3.0, 0.30)
            if float(np.linalg.norm(end - start)) <= closed_threshold:
                return dataframe

    required = {"reference_x", "reference_y"}
    if not required.issubset(dataframe.columns) or len(dataframe) < 3:
        return dataframe

    reference_df = dataframe.dropna(subset=["reference_x", "reference_y"]).reset_index(drop=True)
    if len(reference_df) < 3:
        return dataframe

    final_reference = reference_df[["reference_x", "reference_y"]].to_numpy(dtype=float)[-1]
    reference_points = dataframe[["reference_x", "reference_y"]].to_numpy(dtype=float)
    finite_mask = np.isfinite(reference_points).all(axis=1)
    if not np.any(finite_mask):
        return dataframe

    distances = np.full(len(dataframe), np.inf, dtype=float)
    deltas = reference_points[finite_mask] - final_reference
    distances[finite_mask] = np.linalg.norm(deltas, axis=1)
    reached_mask = distances <= distance_threshold
    reached_indices = np.flatnonzero(reached_mask)
    if reached_indices.size == 0:
        return dataframe

    trim_end_idx = int(reached_indices[0])
    if trim_end_idx >= len(dataframe) - 1:
        return dataframe

    trimmed = dataframe.iloc[: trim_end_idx + 1].copy().reset_index(drop=True)
    trim_end_s = float(dataframe["time_rel_s"].iloc[-1] - dataframe["time_rel_s"].iloc[trim_end_idx])
    trimmed.attrs = dict(dataframe.attrs)
    trimmed.attrs["trim_end_s"] = trim_end_s
    trimmed.attrs["sample_count_after_trim"] = int(len(trimmed))
    trimmed.attrs["duration_s"] = float(trimmed["time_rel_s"].iloc[-1])
    return trimmed


def _finite_percentile(values: np.ndarray, percentile: float) -> float | None:
    """Return one percentile over the finite subset of ``values``."""
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return float(np.percentile(finite, percentile))


def _finite_rms(values: np.ndarray) -> float | None:
    """Return the RMS value over the finite subset of ``values``."""
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return float(np.sqrt(np.mean(np.square(finite))))


def _wrap_angle_array(values: np.ndarray) -> np.ndarray:
    """Wrap angle values to [-pi, pi]."""
    return np.arctan2(np.sin(values), np.cos(values))


def _finite_mean(values: np.ndarray) -> float | None:
    """Return the mean over finite values."""
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return float(np.mean(finite))


def _compute_speed_metrics(dataframe: pd.DataFrame) -> dict[str, object]:
    """Compute speed tracking metrics from v_actual/v_ref."""
    required = {"v_actual", "v_ref"}
    if not required.issubset(dataframe.columns):
        return {
            "mean_abs_speed_error_mps": None,
            "p95_abs_speed_error_mps": None,
            "max_abs_speed_error_mps": None,
            "speed_rms_error_mps": None,
        }

    speed_df = dataframe.dropna(subset=["v_actual", "v_ref"])
    if len(speed_df) == 0:
        return {
            "mean_abs_speed_error_mps": None,
            "p95_abs_speed_error_mps": None,
            "max_abs_speed_error_mps": None,
            "speed_rms_error_mps": None,
        }

    speed_error = (
        speed_df["v_actual"].to_numpy(dtype=float)
        - speed_df["v_ref"].to_numpy(dtype=float)
    )
    abs_speed_error = np.abs(speed_error)
    return {
        "mean_abs_speed_error_mps": _finite_mean(abs_speed_error),
        "p95_abs_speed_error_mps": _finite_percentile(abs_speed_error, P95_PERCENTILE),
        "max_abs_speed_error_mps": _finite_percentile(abs_speed_error, 100.0),
        "speed_rms_error_mps": _finite_rms(speed_error),
    }


def _compute_timing_metrics(dataframe: pd.DataFrame) -> dict[str, object]:
    """Compute controller compute-time and callback-period metrics."""
    metrics: dict[str, object] = {
        "mean_compute_time_ms": None,
        "p95_compute_time_ms": None,
        "max_compute_time_ms": None,
        "mean_loop_period_ms": None,
        "p95_loop_period_ms": None,
        "max_loop_period_ms": None,
    }

    if "compute_time_ms" in dataframe.columns:
        compute_time_ms = dataframe["compute_time_ms"].to_numpy(dtype=float)
        metrics["mean_compute_time_ms"] = _finite_mean(compute_time_ms)
        metrics["p95_compute_time_ms"] = _finite_percentile(compute_time_ms, P95_PERCENTILE)
        metrics["max_compute_time_ms"] = _finite_percentile(compute_time_ms, 100.0)

    if "time_rel_s" in dataframe.columns and len(dataframe) >= 2:
        time_rel_s = dataframe["time_rel_s"].to_numpy(dtype=float)
        dt_ms = np.diff(time_rel_s) * 1000.0
        dt_ms = dt_ms[dt_ms > 0.0]
        metrics["mean_loop_period_ms"] = _finite_mean(dt_ms)
        metrics["p95_loop_period_ms"] = _finite_percentile(dt_ms, P95_PERCENTILE)
        metrics["max_loop_period_ms"] = _finite_percentile(dt_ms, 100.0)

    return metrics


def _compute_yaw_rate_metrics(dataframe: pd.DataFrame) -> dict[str, object]:
    """Estimate yaw-rate metrics by differentiating yaw_vehicle."""
    if "yaw_vehicle" not in dataframe.columns or len(dataframe) < 2:
        return {
            "yaw_rate_rms_deg_s": None,
            "max_abs_yaw_rate_deg_s": None,
        }

    yaw_df = dataframe.dropna(subset=["time_rel_s", "yaw_vehicle"]).reset_index(drop=True)
    if len(yaw_df) < 2:
        return {
            "yaw_rate_rms_deg_s": None,
            "max_abs_yaw_rate_deg_s": None,
        }

    time_rel_s = yaw_df["time_rel_s"].to_numpy(dtype=float)
    yaw = yaw_df["yaw_vehicle"].to_numpy(dtype=float)
    dt = np.diff(time_rel_s)
    valid = dt > 0.0
    if not np.any(valid):
        return {
            "yaw_rate_rms_deg_s": None,
            "max_abs_yaw_rate_deg_s": None,
        }

    yaw_rate_rad_s = _wrap_angle_array(np.diff(yaw))[valid] / dt[valid]
    yaw_rate_deg_s = np.degrees(yaw_rate_rad_s)
    return {
        "yaw_rate_rms_deg_s": _finite_rms(yaw_rate_deg_s),
        "max_abs_yaw_rate_deg_s": _finite_percentile(np.abs(yaw_rate_deg_s), 100.0),
    }


def _compute_lane_change_response_metrics(dataframe: pd.DataFrame) -> dict[str, object]:
    """Compute overshoot and settling-time metrics for one smooth open-loop offset path."""
    required = {"time_rel_s", "x", "y", "reference_x", "reference_y"}
    if not required.issubset(dataframe.columns):
        return {
            "lane_change_reference_offset_m": None,
            "lane_change_actual_final_offset_m": None,
            "lane_change_overshoot_m": None,
            "lane_change_overshoot_pct": None,
            "lane_change_settling_time_s": None,
        }

    response_df = dataframe.dropna(subset=list(required)).reset_index(drop=True)
    if len(response_df) < 5:
        return {
            "lane_change_reference_offset_m": None,
            "lane_change_actual_final_offset_m": None,
            "lane_change_overshoot_m": None,
            "lane_change_overshoot_pct": None,
            "lane_change_settling_time_s": None,
        }

    reference_points = response_df[["reference_x", "reference_y"]].to_numpy(dtype=float)
    actual_points = response_df[["x", "y"]].to_numpy(dtype=float)
    start = reference_points[0]
    end = reference_points[-1]
    chord = end - start
    chord_norm = float(np.linalg.norm(chord))
    if chord_norm <= 1e-6:
        return {
            "lane_change_reference_offset_m": None,
            "lane_change_actual_final_offset_m": None,
            "lane_change_overshoot_m": None,
            "lane_change_overshoot_pct": None,
            "lane_change_settling_time_s": None,
        }

    tangent = chord / chord_norm
    normal = np.array([-tangent[1], tangent[0]], dtype=float)
    reference_offset = float(np.dot(end - start, normal))
    # If the full chord is mostly longitudinal, use the first-segment heading as the lane-change axis.
    segment_deltas = np.diff(reference_points, axis=0)
    segment_lengths = np.linalg.norm(segment_deltas, axis=1)
    first_valid = np.flatnonzero(segment_lengths > 1e-6)
    if first_valid.size > 0:
        first_tangent = segment_deltas[int(first_valid[0])] / segment_lengths[int(first_valid[0])]
        first_normal = np.array([-first_tangent[1], first_tangent[0]], dtype=float)
        first_offset = float(np.dot(end - start, first_normal))
        if abs(first_offset) > abs(reference_offset):
            normal = first_normal
            reference_offset = first_offset

    if abs(reference_offset) <= 1e-3:
        return {
            "lane_change_reference_offset_m": reference_offset,
            "lane_change_actual_final_offset_m": None,
            "lane_change_overshoot_m": None,
            "lane_change_overshoot_pct": None,
            "lane_change_settling_time_s": None,
        }

    direction = 1.0 if reference_offset >= 0.0 else -1.0
    reference_lateral = (reference_points - start) @ normal
    reference_lateral_signed = reference_lateral * direction
    target = abs(reference_offset)
    # Treat this metric as lane-change-specific: the reference should move mostly
    # one way and then settle, not loop around like a closed track.
    if (
        target < 0.10
        or float(np.nanmin(reference_lateral_signed)) < -0.05 * target
        or float(np.nanmax(reference_lateral_signed)) > 1.20 * target
    ):
        return {
            "lane_change_reference_offset_m": None,
            "lane_change_actual_final_offset_m": None,
            "lane_change_overshoot_m": None,
            "lane_change_overshoot_pct": None,
            "lane_change_settling_time_s": None,
        }

    actual_offset = (actual_points - start) @ normal
    final_offset = float(np.median(actual_offset[max(0, len(actual_offset) - 10) :]))
    signed_actual = actual_offset * direction
    overshoot_m = max(0.0, float(np.nanmax(signed_actual) - target))
    overshoot_pct = overshoot_m / target * 100.0

    tolerance = max(0.02, 0.05 * target)
    settling_time_s = None
    time_rel_s = response_df["time_rel_s"].to_numpy(dtype=float)
    abs_error_to_final = np.abs(signed_actual - target)
    for idx in range(len(response_df)):
        if np.all(abs_error_to_final[idx:] <= tolerance):
            settling_time_s = float(time_rel_s[idx] - time_rel_s[0])
            break

    return {
        "lane_change_reference_offset_m": reference_offset,
        "lane_change_actual_final_offset_m": final_offset,
        "lane_change_overshoot_m": overshoot_m,
        "lane_change_overshoot_pct": overshoot_pct,
        "lane_change_settling_time_s": settling_time_s,
    }


def _compute_polyline_curvature(points: np.ndarray) -> np.ndarray:
    """Estimate signed curvature for one polyline using a 3-point stencil."""
    point_count = len(points)
    curvatures = np.full(point_count, np.nan, dtype=float)
    if point_count < 3:
        return curvatures

    for idx in range(1, point_count - 1):
        prev_pt = points[idx - 1]
        curr_pt = points[idx]
        next_pt = points[idx + 1]
        chord_a = curr_pt - prev_pt
        chord_b = next_pt - curr_pt
        chord_c = next_pt - prev_pt
        denom = (
            float(np.linalg.norm(chord_a))
            * float(np.linalg.norm(chord_b))
            * float(np.linalg.norm(chord_c))
        )
        if denom <= 1e-9:
            continue
        cross_z = float(chord_a[0] * chord_b[1] - chord_a[1] * chord_b[0])
        curvatures[idx] = 2.0 * cross_z / denom

    curvatures[0] = curvatures[1]
    curvatures[-1] = curvatures[-2]
    return curvatures


def filter_straight_segments(
    dataframe: pd.DataFrame,
    curvature_threshold: float,
    min_length_m: float,
) -> pd.DataFrame:
    """Keep only long, low-curvature straight segments for evaluation."""
    required = {"reference_x", "reference_y"}
    if not required.issubset(dataframe.columns):
        raise ValueError(
            "Straight-segment evaluation requires reference_x/reference_y in the tracking log."
        )

    reference_df = dataframe.dropna(subset=["reference_x", "reference_y"]).copy()
    if len(reference_df) < 5:
        raise ValueError("Not enough reference samples to isolate straight segments.")

    reference_points = reference_df[["reference_x", "reference_y"]].to_numpy(dtype=float)
    curvature_abs = np.abs(_compute_polyline_curvature(reference_points))
    straight_mask = np.isfinite(curvature_abs) & (curvature_abs <= max(0.0, curvature_threshold))

    segment_step = np.zeros(len(reference_df), dtype=float)
    if len(reference_df) >= 2:
        segment_step[1:] = np.linalg.norm(np.diff(reference_points, axis=0), axis=1)

    keep_mask = np.zeros(len(reference_df), dtype=bool)
    run_start: int | None = None
    for idx, is_straight in enumerate(straight_mask):
        if is_straight and run_start is None:
            run_start = idx
        if (not is_straight or idx == len(straight_mask) - 1) and run_start is not None:
            run_end = idx if is_straight and idx == len(straight_mask) - 1 else idx - 1
            run_length_m = float(np.sum(segment_step[run_start : run_end + 1]))
            if run_length_m >= max(0.1, min_length_m):
                keep_mask[run_start : run_end + 1] = True
            run_start = None

    kept_indices = reference_df.index[keep_mask]
    if len(kept_indices) < 5:
        raise ValueError(
            "No straight segment satisfied the requested curvature/length thresholds."
        )

    filtered = dataframe.loc[kept_indices].copy().reset_index(drop=True)
    filtered.attrs = dict(dataframe.attrs)
    filtered.attrs["evaluation_scope"] = "straight_segments_only"
    filtered.attrs["straight_curvature_threshold"] = float(curvature_threshold)
    filtered.attrs["straight_min_length_m"] = float(min_length_m)
    filtered.attrs["straight_sample_count"] = int(len(filtered))
    filtered.attrs["duration_s"] = float(filtered["time_rel_s"].iloc[-1] - filtered["time_rel_s"].iloc[0])
    return filtered


def _compute_steering_metrics(dataframe: pd.DataFrame) -> dict[str, object]:
    """Compute steering smoothness metrics from the tracking log."""
    if "delta_cmd" not in dataframe.columns:
        return {
            "steering_rms_deg": None,
            "steering_rate_rms_deg_s": None,
            "steering_saturation_count": None,
            "steering_saturation_ratio": None,
        }

    steering_df = dataframe.dropna(subset=["time_rel_s", "delta_cmd"]).reset_index(drop=True)
    if len(steering_df) == 0:
        return {
            "steering_rms_deg": None,
            "steering_rate_rms_deg_s": None,
            "steering_saturation_count": None,
            "steering_saturation_ratio": None,
        }

    delta_cmd_rad = steering_df["delta_cmd"].to_numpy(dtype=float)
    steering_rms_deg = _finite_rms(np.degrees(delta_cmd_rad))

    steering_rate_rms_deg_s = None
    if len(steering_df) >= 2:
        time_rel_s = steering_df["time_rel_s"].to_numpy(dtype=float)
        dt = np.diff(time_rel_s)
        positive_dt_mask = dt > 0.0
        if np.any(positive_dt_mask):
            steering_rate_rad_s = np.diff(delta_cmd_rad)[positive_dt_mask] / dt[positive_dt_mask]
            steering_rate_rms_deg_s = _finite_rms(np.degrees(steering_rate_rad_s))

    steering_saturation_count = None
    steering_saturation_ratio = None
    if "steering_limit" in steering_df.columns:
        steering_limit_rad = steering_df["steering_limit"].to_numpy(dtype=float)
        valid_limit_mask = np.isfinite(delta_cmd_rad) & np.isfinite(steering_limit_rad)
        valid_limit_mask &= steering_limit_rad > 0.0
        if np.any(valid_limit_mask):
            saturation_mask = (
                np.abs(delta_cmd_rad[valid_limit_mask])
                >= steering_limit_rad[valid_limit_mask] - 1e-6
            )
            steering_saturation_count = int(np.count_nonzero(saturation_mask))
            steering_saturation_ratio = float(
                steering_saturation_count / np.count_nonzero(valid_limit_mask)
            )

    return {
        "steering_rms_deg": steering_rms_deg,
        "steering_rate_rms_deg_s": steering_rate_rms_deg_s,
        "steering_saturation_count": steering_saturation_count,
        "steering_saturation_ratio": steering_saturation_ratio,
    }


def summarize_log(
    dataframe: pd.DataFrame,
    lateral_threshold: float,
    heading_threshold_deg: float,
) -> dict[str, object]:
    """Return the key tuning metrics for one run."""
    abs_e_y = dataframe["abs_e_y"].to_numpy(dtype=float)
    abs_e_psi_deg = dataframe["abs_e_psi_deg"].to_numpy(dtype=float)
    finite_heading = abs_e_psi_deg[np.isfinite(abs_e_psi_deg)]
    steering_metrics = _compute_steering_metrics(dataframe)
    speed_metrics = _compute_speed_metrics(dataframe)
    timing_metrics = _compute_timing_metrics(dataframe)
    yaw_rate_metrics = _compute_yaw_rate_metrics(dataframe)
    lane_change_metrics = _compute_lane_change_response_metrics(dataframe)

    summary = {
        "log_path": dataframe.attrs["log_path"],
        "log_name": Path(dataframe.attrs["log_path"]).stem,
        "lookahead_distance_m": dataframe.attrs["lookahead_distance"],
        "sample_count": int(len(dataframe)),
        "duration_s": float(dataframe.attrs["duration_s"]),
        "trim_start_s": float(dataframe.attrs.get("trim_start_s", 0.0)),
        "trim_end_s": float(dataframe.attrs.get("trim_end_s", 0.0)),
        "raw_sample_count": int(dataframe.attrs.get("raw_sample_count", len(dataframe))),
        "raw_duration_s": float(dataframe.attrs.get("raw_duration_s", dataframe.attrs["duration_s"])),
        "evaluation_scope": str(dataframe.attrs.get("evaluation_scope", "full_run")),
        "mean_abs_e_y_m": float(np.mean(abs_e_y)),
        "p95_abs_e_y_m": float(_finite_percentile(abs_e_y, P95_PERCENTILE)),
        "max_abs_e_y_m": float(np.max(abs_e_y)),
        "mean_abs_e_y_cm": float(np.mean(abs_e_y) * 100.0),
        "p95_abs_e_y_cm": float(_finite_percentile(abs_e_y, P95_PERCENTILE) * 100.0),
        "max_abs_e_y_cm": float(np.max(abs_e_y) * 100.0),
        "pass_mean_target": bool(np.mean(abs_e_y) <= lateral_threshold),
        "pass_max_target": bool(np.max(abs_e_y) <= lateral_threshold),
        "mean_abs_e_psi_deg": None,
        "p95_abs_e_psi_deg": None,
        "max_abs_e_psi_deg": None,
        "pass_mean_heading_target": None,
        "pass_max_heading_target": None,
        "steering_rms_deg": steering_metrics["steering_rms_deg"],
        "steering_rate_rms_deg_s": steering_metrics["steering_rate_rms_deg_s"],
        "steering_saturation_count": steering_metrics["steering_saturation_count"],
        "steering_saturation_ratio": steering_metrics["steering_saturation_ratio"],
        "mean_abs_speed_error_mps": speed_metrics["mean_abs_speed_error_mps"],
        "p95_abs_speed_error_mps": speed_metrics["p95_abs_speed_error_mps"],
        "max_abs_speed_error_mps": speed_metrics["max_abs_speed_error_mps"],
        "speed_rms_error_mps": speed_metrics["speed_rms_error_mps"],
        "yaw_rate_rms_deg_s": yaw_rate_metrics["yaw_rate_rms_deg_s"],
        "max_abs_yaw_rate_deg_s": yaw_rate_metrics["max_abs_yaw_rate_deg_s"],
        "mean_compute_time_ms": timing_metrics["mean_compute_time_ms"],
        "p95_compute_time_ms": timing_metrics["p95_compute_time_ms"],
        "max_compute_time_ms": timing_metrics["max_compute_time_ms"],
        "mean_loop_period_ms": timing_metrics["mean_loop_period_ms"],
        "p95_loop_period_ms": timing_metrics["p95_loop_period_ms"],
        "max_loop_period_ms": timing_metrics["max_loop_period_ms"],
        "lane_change_reference_offset_m": lane_change_metrics["lane_change_reference_offset_m"],
        "lane_change_actual_final_offset_m": lane_change_metrics["lane_change_actual_final_offset_m"],
        "lane_change_overshoot_m": lane_change_metrics["lane_change_overshoot_m"],
        "lane_change_overshoot_pct": lane_change_metrics["lane_change_overshoot_pct"],
        "lane_change_settling_time_s": lane_change_metrics["lane_change_settling_time_s"],
    }

    if finite_heading.size > 0:
        summary["mean_abs_e_psi_deg"] = float(np.mean(finite_heading))
        summary["p95_abs_e_psi_deg"] = float(
            _finite_percentile(finite_heading, P95_PERCENTILE)
        )
        summary["max_abs_e_psi_deg"] = float(np.max(finite_heading))
        summary["pass_mean_heading_target"] = bool(
            float(np.mean(finite_heading)) <= heading_threshold_deg
        )
        summary["pass_max_heading_target"] = bool(
            float(np.max(finite_heading)) <= heading_threshold_deg
        )

    return summary


def save_single_run_plot(
    dataframe: pd.DataFrame,
    summary: dict[str, object],
    lateral_threshold: float,
    output_dir: Path,
) -> None:
    """Plot signed/absolute lateral error for one run."""
    time_s = dataframe["time_rel_s"].to_numpy(dtype=float)
    e_y = dataframe["e_y"].to_numpy(dtype=float)
    abs_e_y = dataframe["abs_e_y"].to_numpy(dtype=float)
    mean_abs_e_y = float(np.mean(abs_e_y))
    p95_abs_e_y = float(np.percentile(abs_e_y, P95_PERCENTILE))

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    axes[0].plot(time_s, e_y, color="#1f77b4", linewidth=1.5)
    axes[0].axhline(lateral_threshold, color="#d62728", linestyle="--", linewidth=1.0)
    axes[0].axhline(-lateral_threshold, color="#d62728", linestyle="--", linewidth=1.0)
    axes[0].set_ylabel("e_y [m]")
    axes[0].set_title(
        f"{summary['log_name']} | lookahead={summary['lookahead_distance_m']:.3f} m"
        if np.isfinite(summary["lookahead_distance_m"])
        else f"{summary['log_name']} | lookahead=unknown"
    )
    axes[0].grid(True, linestyle=":", linewidth=0.8)

    axes[1].plot(time_s, abs_e_y, color="#ff7f0e", linewidth=1.5, label="abs(e_y)")
    axes[1].axhline(
        lateral_threshold,
        color="#2ca02c",
        linestyle="--",
        linewidth=1.0,
        label=f"{lateral_threshold * 100.0:.1f} cm target",
    )
    axes[1].axhline(
        mean_abs_e_y,
        color="#9467bd",
        linestyle=":",
        linewidth=1.2,
        label=f"mean abs(e_y) = {mean_abs_e_y * 100.0:.2f} cm",
    )
    axes[1].axhline(
        p95_abs_e_y,
        color="#8c564b",
        linestyle="-.",
        linewidth=1.2,
        label=f"p95 abs(e_y) = {p95_abs_e_y * 100.0:.2f} cm",
    )
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("abs(e_y) [m]")
    axes[1].grid(True, linestyle=":", linewidth=0.8)
    axes[1].legend()

    fig.tight_layout()
    output_path = output_dir / f"{summary['log_name']}_lateral_error.png"
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def save_heading_error_plot(
    dataframe: pd.DataFrame,
    summary: dict[str, object],
    heading_threshold_deg: float,
    output_dir: Path,
) -> None:
    """Plot signed and absolute heading error for one run."""
    required = {"e_psi_deg", "abs_e_psi_deg"}
    if not required.issubset(dataframe.columns):
        return

    plot_df = dataframe.dropna(subset=["e_psi_deg", "abs_e_psi_deg"])
    if len(plot_df) < 2:
        return

    time_s = plot_df["time_rel_s"].to_numpy(dtype=float)
    e_psi_deg = plot_df["e_psi_deg"].to_numpy(dtype=float)
    abs_e_psi_deg = plot_df["abs_e_psi_deg"].to_numpy(dtype=float)
    mean_abs_e_psi_deg = float(np.mean(abs_e_psi_deg))
    p95_abs_e_psi_deg = float(np.percentile(abs_e_psi_deg, P95_PERCENTILE))

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    axes[0].plot(time_s, e_psi_deg, color="#1f77b4", linewidth=1.5)
    axes[0].axhline(heading_threshold_deg, color="#d62728", linestyle="--", linewidth=1.0)
    axes[0].axhline(-heading_threshold_deg, color="#d62728", linestyle="--", linewidth=1.0)
    axes[0].set_ylabel("e_psi [deg]")
    axes[0].set_title(
        f"{summary['log_name']} | heading error"
    )
    axes[0].grid(True, linestyle=":", linewidth=0.8)

    axes[1].plot(
        time_s,
        abs_e_psi_deg,
        color="#ff7f0e",
        linewidth=1.5,
        label="abs(e_psi) [deg]",
    )
    axes[1].axhline(
        heading_threshold_deg,
        color="#2ca02c",
        linestyle="--",
        linewidth=1.0,
        label=f"{heading_threshold_deg:.1f} deg target",
    )
    axes[1].axhline(
        mean_abs_e_psi_deg,
        color="#9467bd",
        linestyle=":",
        linewidth=1.2,
        label=f"mean abs(e_psi) = {mean_abs_e_psi_deg:.2f} deg",
    )
    axes[1].axhline(
        p95_abs_e_psi_deg,
        color="#8c564b",
        linestyle="-.",
        linewidth=1.2,
        label=f"p95 abs(e_psi) = {p95_abs_e_psi_deg:.2f} deg",
    )
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("abs(e_psi) [deg]")
    axes[1].grid(True, linestyle=":", linewidth=0.8)
    axes[1].legend()

    fig.tight_layout()
    output_path = output_dir / f"{summary['log_name']}_heading_error.png"
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def save_steering_breakdown_plot(
    dataframe: pd.DataFrame,
    summary: dict[str, object],
    output_dir: Path,
) -> None:
    """Plot Pure Pursuit steering, heading feedback, and final steering command."""
    required = {"delta_pp", "delta_heading", "delta_cmd"}
    if not required.issubset(dataframe.columns):
        return

    plot_df = dataframe.dropna(subset=["delta_pp", "delta_heading", "delta_cmd"])
    if len(plot_df) < 2:
        return

    time_s = plot_df["time_rel_s"].to_numpy(dtype=float)
    delta_pp_deg = np.degrees(plot_df["delta_pp"].to_numpy(dtype=float))
    delta_heading_deg = np.degrees(plot_df["delta_heading"].to_numpy(dtype=float))
    delta_cmd_deg = np.degrees(plot_df["delta_cmd"].to_numpy(dtype=float))

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

    axes[0].plot(time_s, delta_pp_deg, linewidth=1.6, label="delta_pp [deg]")
    axes[0].plot(
        time_s,
        delta_heading_deg,
        linewidth=1.6,
        label="delta_heading [deg]",
    )
    axes[0].set_ylabel("Component [deg]")
    axes[0].set_title(
        f"{summary['log_name']} | steering breakdown"
    )
    axes[0].grid(True, linestyle=":", linewidth=0.8)
    axes[0].legend()

    axes[1].plot(time_s, delta_cmd_deg, color="#d62728", linewidth=1.8, label="delta_cmd [deg]")
    axes[1].axhline(0.0, color="black", linewidth=0.8, linestyle="--")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Command [deg]")
    axes[1].grid(True, linestyle=":", linewidth=0.8)
    axes[1].legend()

    fig.tight_layout()
    output_path = output_dir / f"{summary['log_name']}_steering_breakdown.png"
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def save_speed_tracking_plot(
    dataframe: pd.DataFrame,
    summary: dict[str, object],
    output_dir: Path,
) -> None:
    """Plot reference speed, actual speed, and speed tracking error."""
    required = {"time_rel_s", "v_actual", "v_ref"}
    if not required.issubset(dataframe.columns):
        return

    plot_df = dataframe.dropna(subset=list(required))
    if len(plot_df) < 2:
        return

    time_s = plot_df["time_rel_s"].to_numpy(dtype=float)
    v_actual = plot_df["v_actual"].to_numpy(dtype=float)
    v_ref = plot_df["v_ref"].to_numpy(dtype=float)
    speed_error = v_actual - v_ref

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(time_s, v_ref, linewidth=1.8, label="v_ref [m/s]")
    axes[0].plot(time_s, v_actual, linewidth=1.6, label="v_actual [m/s]")
    axes[0].set_ylabel("Speed [m/s]")
    axes[0].set_title(f"{summary['log_name']} | speed tracking")
    axes[0].grid(True, linestyle=":", linewidth=0.8)
    axes[0].legend()

    axes[1].plot(time_s, speed_error, color="#d62728", linewidth=1.5)
    axes[1].axhline(0.0, color="black", linewidth=0.8, linestyle="--")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("v_actual - v_ref [m/s]")
    axes[1].grid(True, linestyle=":", linewidth=0.8)

    fig.tight_layout()
    fig.savefig(output_dir / f"{summary['log_name']}_speed_tracking.png", dpi=180)
    plt.close(fig)


def save_yaw_rate_plot(
    dataframe: pd.DataFrame,
    summary: dict[str, object],
    output_dir: Path,
) -> None:
    """Plot estimated yaw rate from logged yaw_vehicle."""
    required = {"time_rel_s", "yaw_vehicle"}
    if not required.issubset(dataframe.columns):
        return

    plot_df = dataframe.dropna(subset=list(required)).reset_index(drop=True)
    if len(plot_df) < 3:
        return

    time_s = plot_df["time_rel_s"].to_numpy(dtype=float)
    yaw = plot_df["yaw_vehicle"].to_numpy(dtype=float)
    dt = np.diff(time_s)
    valid = dt > 0.0
    if not np.any(valid):
        return
    yaw_rate_deg_s = np.degrees(_wrap_angle_array(np.diff(yaw))[valid] / dt[valid])
    rate_time_s = time_s[1:][valid]

    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(rate_time_s, yaw_rate_deg_s, linewidth=1.6)
    ax.axhline(0.0, color="black", linewidth=0.8, linestyle="--")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Yaw rate [deg/s]")
    ax.set_title(f"{summary['log_name']} | yaw rate")
    ax.grid(True, linestyle=":", linewidth=0.8)
    fig.tight_layout()
    fig.savefig(output_dir / f"{summary['log_name']}_yaw_rate.png", dpi=180)
    plt.close(fig)


def save_timing_plot(
    dataframe: pd.DataFrame,
    summary: dict[str, object],
    output_dir: Path,
) -> None:
    """Plot controller compute time and odometry callback period."""
    if "time_rel_s" not in dataframe.columns or len(dataframe) < 2:
        return

    time_s = dataframe["time_rel_s"].to_numpy(dtype=float)
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=False)
    plotted = False

    if "compute_time_ms" in dataframe.columns:
        compute_df = dataframe.dropna(subset=["time_rel_s", "compute_time_ms"])
        if len(compute_df) >= 2:
            axes[0].plot(
                compute_df["time_rel_s"].to_numpy(dtype=float),
                compute_df["compute_time_ms"].to_numpy(dtype=float),
                linewidth=1.4,
            )
            axes[0].set_ylabel("Compute [ms]")
            axes[0].set_title(f"{summary['log_name']} | compute time")
            axes[0].grid(True, linestyle=":", linewidth=0.8)
            plotted = True

    dt_ms = np.diff(time_s) * 1000.0
    valid = dt_ms > 0.0
    if np.any(valid):
        axes[1].plot(time_s[1:][valid], dt_ms[valid], color="#ff7f0e", linewidth=1.4)
        axes[1].set_xlabel("Time [s]")
        axes[1].set_ylabel("Loop period [ms]")
        axes[1].grid(True, linestyle=":", linewidth=0.8)
        plotted = True

    if not plotted:
        plt.close(fig)
        return

    fig.tight_layout()
    fig.savefig(output_dir / f"{summary['log_name']}_timing.png", dpi=180)
    plt.close(fig)


def save_lane_change_response_plot(
    dataframe: pd.DataFrame,
    summary: dict[str, object],
    output_dir: Path,
) -> None:
    """Plot lateral response for a single-offset open-loop trajectory."""
    target_offset = summary.get("lane_change_reference_offset_m")
    if target_offset is None or not np.isfinite(target_offset):
        return

    required = {"time_rel_s", "x", "y", "reference_x", "reference_y"}
    if not required.issubset(dataframe.columns):
        return

    plot_df = dataframe.dropna(subset=list(required)).reset_index(drop=True)
    if len(plot_df) < 5:
        return

    reference_points = plot_df[["reference_x", "reference_y"]].to_numpy(dtype=float)
    actual_points = plot_df[["x", "y"]].to_numpy(dtype=float)
    segment_deltas = np.diff(reference_points, axis=0)
    segment_lengths = np.linalg.norm(segment_deltas, axis=1)
    first_valid = np.flatnonzero(segment_lengths > 1e-6)
    if first_valid.size == 0:
        return

    first_tangent = segment_deltas[int(first_valid[0])] / segment_lengths[int(first_valid[0])]
    normal = np.array([-first_tangent[1], first_tangent[0]], dtype=float)
    start = reference_points[0]
    reference_offset = (reference_points - start) @ normal
    actual_offset = (actual_points - start) @ normal

    if np.nanmax(reference_offset) - np.nanmin(reference_offset) <= 1e-3:
        return

    time_s = plot_df["time_rel_s"].to_numpy(dtype=float)
    fig, ax = plt.subplots(figsize=(10, 4.8))
    ax.plot(time_s, reference_offset, linestyle="--", linewidth=1.8, label="reference lateral offset")
    ax.plot(time_s, actual_offset, linewidth=1.8, label="actual lateral offset")
    ax.axhline(
        float(target_offset),
        color="#2ca02c",
        linestyle=":",
        linewidth=1.2,
        label="final reference offset",
    )
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Lateral offset [m]")
    ax.set_title(f"{summary['log_name']} | lane-change response")
    ax.grid(True, linestyle=":", linewidth=0.8)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / f"{summary['log_name']}_lane_change_response.png", dpi=180)
    plt.close(fig)


def save_path_overlay_plot(
    plot_dataframe: pd.DataFrame,
    evaluated_dataframe: pd.DataFrame,
    summary: dict[str, object],
    output_dir: Path,
    reference_trajectory: tuple[np.ndarray, np.ndarray] | None = None,
    *,
    output_suffix: str = "path_overlay",
    title_scope: str | None = None,
) -> None:
    """Overlay the reference path and the driven path for one run."""
    required = {"x", "y"}
    if not required.issubset(plot_dataframe.columns):
        return

    plot_df = plot_dataframe.dropna(subset=["x", "y"])
    if len(plot_df) < 2:
        return

    actual_x = plot_df["x"].to_numpy(dtype=float)
    actual_y = plot_df["y"].to_numpy(dtype=float)

    # Prefer the reference path logged by the controller, because it already
    # reflects any runtime trajectory transform such as current-pose anchoring.
    fallback_required = {"reference_x", "reference_y"}
    logged_reference_available = fallback_required.issubset(plot_dataframe.columns)
    if logged_reference_available:
        fallback_df = plot_dataframe.dropna(subset=["reference_x", "reference_y"])
    else:
        fallback_df = pd.DataFrame()

    if len(fallback_df) >= 2:
        reference_x = fallback_df["reference_x"].to_numpy(dtype=float)
        reference_y = fallback_df["reference_y"].to_numpy(dtype=float)
        original_reference_x = None
        original_reference_y = None
        if reference_trajectory is not None:
            original_reference_x, original_reference_y = reference_trajectory
    elif reference_trajectory is not None:
        reference_x, reference_y = reference_trajectory
        original_reference_x = None
        original_reference_y = None
    else:
        return

    highlight_df = evaluated_dataframe.dropna(subset=["x", "y"]).copy()
    highlight_ref_df = (
        evaluated_dataframe.dropna(subset=["reference_x", "reference_y"]).copy()
        if {"reference_x", "reference_y"}.issubset(evaluated_dataframe.columns)
        else pd.DataFrame()
    )

    fig, ax = plt.subplots(figsize=(8, 8))
    if original_reference_x is not None and original_reference_y is not None:
        ax.plot(
            original_reference_x,
            original_reference_y,
            linestyle=":",
            linewidth=1.5,
            alpha=0.6,
            label="original csv path",
        )
    ax.plot(
        reference_x,
        reference_y,
        linestyle="--",
        linewidth=1.8,
        alpha=0.8,
        label="tracked reference path",
    )
    ax.plot(actual_x, actual_y, linewidth=1.8, alpha=0.8, label="actual path")
    if len(highlight_ref_df) >= 2:
        ax.plot(
            highlight_ref_df["reference_x"].to_numpy(dtype=float),
            highlight_ref_df["reference_y"].to_numpy(dtype=float),
            color="#2ca02c",
            linewidth=3.0,
            label="evaluated reference segment",
        )
    if len(highlight_df) >= 2:
        ax.plot(
            highlight_df["x"].to_numpy(dtype=float),
            highlight_df["y"].to_numpy(dtype=float),
            color="#d62728",
            linewidth=3.0,
            label="evaluated actual segment",
        )
    ax.scatter(reference_x[0], reference_y[0], marker="o", s=60, label="reference start")
    ax.scatter(actual_x[0], actual_y[0], marker="x", s=60, label="actual start")
    ax.scatter(actual_x[-1], actual_y[-1], marker="s", s=45, label="actual end")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    scope = title_scope or str(summary.get("evaluation_scope", "full_run"))
    ax.set_title(f"{summary['log_name']} | path overlay ({scope})")
    ax.grid(True, linestyle=":", linewidth=0.8)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / f"{summary['log_name']}_{output_suffix}.png", dpi=180)
    plt.close(fig)


def save_comparison_plot(
    summary_df: pd.DataFrame,
    lateral_threshold: float,
    output_dir: Path,
) -> None:
    """Plot mean/p95/max abs lateral error against lookahead distance."""
    if len(summary_df) < 2 or summary_df["lookahead_distance_m"].isna().all():
        return

    plot_df = summary_df.copy()
    plot_df = plot_df.dropna(subset=["lookahead_distance_m"])
    if len(plot_df) < 2:
        return

    plot_df = plot_df.sort_values("lookahead_distance_m", kind="mergesort")

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(
        plot_df["lookahead_distance_m"],
        plot_df["mean_abs_e_y_m"],
        marker="o",
        linewidth=1.8,
        label="mean abs(e_y)",
    )
    ax.plot(
        plot_df["lookahead_distance_m"],
        plot_df["p95_abs_e_y_m"],
        marker="^",
        linewidth=1.8,
        label="p95 abs(e_y)",
    )
    ax.plot(
        plot_df["lookahead_distance_m"],
        plot_df["max_abs_e_y_m"],
        marker="s",
        linewidth=1.8,
        label="max abs(e_y)",
    )
    ax.axhline(
        lateral_threshold,
        color="#d62728",
        linestyle="--",
        linewidth=1.0,
        label=f"{lateral_threshold * 100.0:.1f} cm target",
    )
    ax.set_xlabel("Lookahead distance [m]")
    ax.set_ylabel("Lateral error [m]")
    ax.set_title("Lookahead sweep summary")
    ax.grid(True, linestyle=":", linewidth=0.8)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "lookahead_sweep.png", dpi=180)
    plt.close(fig)


def save_heading_comparison_plot(
    summary_df: pd.DataFrame,
    heading_threshold_deg: float,
    output_dir: Path,
) -> None:
    """Plot mean/p95/max abs heading error against lookahead distance."""
    if len(summary_df) < 2 or summary_df["lookahead_distance_m"].isna().all():
        return

    required = {"mean_abs_e_psi_deg", "p95_abs_e_psi_deg", "max_abs_e_psi_deg"}
    if not required.issubset(summary_df.columns):
        return

    plot_df = summary_df.copy()
    plot_df = plot_df.dropna(
        subset=[
            "lookahead_distance_m",
            "mean_abs_e_psi_deg",
            "p95_abs_e_psi_deg",
            "max_abs_e_psi_deg",
        ]
    )
    if len(plot_df) < 2:
        return

    plot_df = plot_df.sort_values("lookahead_distance_m", kind="mergesort")

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(
        plot_df["lookahead_distance_m"],
        plot_df["mean_abs_e_psi_deg"],
        marker="o",
        linewidth=1.8,
        label="mean abs(e_psi) [deg]",
    )
    ax.plot(
        plot_df["lookahead_distance_m"],
        plot_df["p95_abs_e_psi_deg"],
        marker="^",
        linewidth=1.8,
        label="p95 abs(e_psi) [deg]",
    )
    ax.plot(
        plot_df["lookahead_distance_m"],
        plot_df["max_abs_e_psi_deg"],
        marker="s",
        linewidth=1.8,
        label="max abs(e_psi) [deg]",
    )
    ax.axhline(
        heading_threshold_deg,
        color="#d62728",
        linestyle="--",
        linewidth=1.0,
        label=f"{heading_threshold_deg:.1f} deg target",
    )
    ax.set_xlabel("Lookahead distance [m]")
    ax.set_ylabel("Heading error [deg]")
    ax.set_title("Lookahead sweep heading summary")
    ax.grid(True, linestyle=":", linewidth=0.8)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "heading_sweep.png", dpi=180)
    plt.close(fig)


def save_response_comparison_plot(
    summary_df: pd.DataFrame,
    output_dir: Path,
) -> None:
    """Plot lane-change response, speed tracking, yaw-rate, and timing summary."""
    if len(summary_df) < 2 or summary_df["lookahead_distance_m"].isna().all():
        return

    plot_df = summary_df.copy().dropna(subset=["lookahead_distance_m"])
    if len(plot_df) < 2:
        return
    plot_df = plot_df.sort_values("lookahead_distance_m", kind="mergesort")

    panels = [
        ("lane_change_overshoot_pct", "Overshoot [%]"),
        ("lane_change_settling_time_s", "Settling time [s]"),
        ("speed_rms_error_mps", "Speed RMS error [m/s]"),
        ("p95_compute_time_ms", "P95 compute time [ms]"),
        ("p95_loop_period_ms", "P95 loop period [ms]"),
        ("yaw_rate_rms_deg_s", "Yaw-rate RMS [deg/s]"),
    ]
    available = [panel for panel in panels if panel[0] in plot_df.columns and plot_df[panel[0]].notna().any()]
    if not available:
        return

    row_count = int(np.ceil(len(available) / 2.0))
    fig, axes = plt.subplots(row_count, 2, figsize=(11, 4.0 * row_count), squeeze=False)
    for ax, (column, ylabel) in zip(axes.reshape(-1), available):
        metric_df = plot_df.dropna(subset=[column])
        ax.plot(
            metric_df["lookahead_distance_m"],
            metric_df[column],
            marker="o",
            linewidth=1.8,
        )
        ax.set_xlabel("Lookahead distance [m]")
        ax.set_ylabel(ylabel)
        ax.grid(True, linestyle=":", linewidth=0.8)

    for ax in axes.reshape(-1)[len(available):]:
        ax.axis("off")

    fig.suptitle("Lookahead sweep response summary")
    fig.tight_layout()
    fig.savefig(output_dir / "response_sweep.png", dpi=180)
    plt.close(fig)


def build_recommendation(summary_df: pd.DataFrame) -> str:
    """Choose the current best lookahead candidate."""
    sortable = summary_df.copy()
    sortable["pass_lateral_both"] = sortable["pass_mean_target"] & sortable["pass_max_target"]
    if {"pass_mean_heading_target", "pass_max_heading_target"} <= set(sortable.columns):
        sortable["pass_heading_mean"] = sortable["pass_mean_heading_target"].fillna(False)
        sortable["pass_heading_max"] = sortable["pass_max_heading_target"].fillna(False)
    else:
        sortable["pass_heading_mean"] = False
        sortable["pass_heading_max"] = False
    sortable = sortable.sort_values(
        [
            "pass_lateral_both",
            "pass_heading_mean",
            "pass_heading_max",
            "p95_abs_e_y_m",
            "mean_abs_e_y_m",
            "p95_abs_e_psi_deg",
            "mean_abs_e_psi_deg",
            "lane_change_overshoot_pct",
            "lane_change_settling_time_s",
            "speed_rms_error_mps",
            "steering_rate_rms_deg_s",
            "steering_rms_deg",
            "steering_saturation_count",
            "max_abs_e_y_m",
        ],
        ascending=[False, False, False, True, True, True, True, True, True, True, True, True, True, True],
        kind="mergesort",
        na_position="last",
    )

    best = sortable.iloc[0]
    lookahead = best["lookahead_distance_m"]
    lookahead_text = "unknown" if pd.isna(lookahead) else f"{lookahead:.3f} m"
    mean_heading = best["mean_abs_e_psi_deg"]
    p95_heading = best["p95_abs_e_psi_deg"]
    max_heading = best["max_abs_e_psi_deg"]
    mean_heading_text = "-" if pd.isna(mean_heading) else f"{mean_heading:.2f} deg"
    p95_heading_text = "-" if pd.isna(p95_heading) else f"{p95_heading:.2f} deg"
    max_heading_text = "-" if pd.isna(max_heading) else f"{max_heading:.2f} deg"
    steering_rms = best["steering_rms_deg"]
    steering_rate_rms = best["steering_rate_rms_deg_s"]
    steering_sat = best["steering_saturation_count"]
    overshoot_pct = best.get("lane_change_overshoot_pct")
    settling_time_s = best.get("lane_change_settling_time_s")
    speed_rms = best.get("speed_rms_error_mps")
    p95_compute = best.get("p95_compute_time_ms")
    steering_rms_text = "-" if pd.isna(steering_rms) else f"{steering_rms:.2f} deg"
    steering_rate_text = "-" if pd.isna(steering_rate_rms) else f"{steering_rate_rms:.2f} deg/s"
    steering_sat_text = "-" if pd.isna(steering_sat) else str(int(steering_sat))
    overshoot_text = "-" if pd.isna(overshoot_pct) else f"{overshoot_pct:.1f}%"
    settling_text = "-" if pd.isna(settling_time_s) else f"{settling_time_s:.2f} s"
    speed_rms_text = "-" if pd.isna(speed_rms) else f"{speed_rms:.3f} m/s"
    p95_compute_text = "-" if pd.isna(p95_compute) else f"{p95_compute:.2f} ms"
    return (
        "Recommended run: "
        f"{best['log_name']} (lookahead={lookahead_text}, "
        f"trim_start={best['trim_start_s']:.2f} s, "
        f"mean_abs_e_y={best['mean_abs_e_y_cm']:.2f} cm, "
        f"p95_abs_e_y={best['p95_abs_e_y_cm']:.2f} cm, "
        f"max_abs_e_y={best['max_abs_e_y_cm']:.2f} cm, "
        f"mean_abs_e_psi={mean_heading_text}, "
        f"p95_abs_e_psi={p95_heading_text}, "
        f"max_abs_e_psi={max_heading_text}, "
        f"steering_rms={steering_rms_text}, "
        f"steering_rate_rms={steering_rate_text}, "
        f"steering_sat_count={steering_sat_text}, "
        f"overshoot={overshoot_text}, "
        f"settling_time={settling_text}, "
        f"speed_rms_error={speed_rms_text}, "
        f"p95_compute_time={p95_compute_text}, "
        f"pass_lat_mean={bool(best['pass_mean_target'])}, "
        f"pass_lat_max={bool(best['pass_max_target'])}, "
        f"pass_head_mean={bool(best['pass_mean_heading_target'])}, "
        f"pass_head_max={bool(best['pass_max_heading_target'])})"
    )


def print_summary(
    summary_df: pd.DataFrame,
    lateral_threshold: float,
    heading_threshold_deg: float,
) -> None:
    """Print an easy-to-read terminal report."""
    display_df = summary_df.copy()
    display_df["lookahead_distance_m"] = display_df["lookahead_distance_m"].map(
        lambda value: "-" if pd.isna(value) else f"{value:.3f}"
    )
    display_df["trim_start_s"] = display_df["trim_start_s"].map(lambda value: f"{value:.2f}")
    display_df["trim_end_s"] = display_df["trim_end_s"].map(lambda value: f"{value:.2f}")
    display_df["mean_abs_e_y_cm"] = display_df["mean_abs_e_y_cm"].map(lambda value: f"{value:.2f}")
    display_df["p95_abs_e_y_cm"] = display_df["p95_abs_e_y_cm"].map(lambda value: f"{value:.2f}")
    display_df["max_abs_e_y_cm"] = display_df["max_abs_e_y_cm"].map(lambda value: f"{value:.2f}")
    display_df["mean_abs_e_psi_deg"] = display_df["mean_abs_e_psi_deg"].map(
        lambda value: "-" if pd.isna(value) else f"{value:.2f}"
    )
    display_df["p95_abs_e_psi_deg"] = display_df["p95_abs_e_psi_deg"].map(
        lambda value: "-" if pd.isna(value) else f"{value:.2f}"
    )
    display_df["max_abs_e_psi_deg"] = display_df["max_abs_e_psi_deg"].map(
        lambda value: "-" if pd.isna(value) else f"{value:.2f}"
    )
    display_df["steering_rms_deg"] = display_df["steering_rms_deg"].map(
        lambda value: "-" if pd.isna(value) else f"{value:.2f}"
    )
    display_df["steering_rate_rms_deg_s"] = display_df["steering_rate_rms_deg_s"].map(
        lambda value: "-" if pd.isna(value) else f"{value:.2f}"
    )
    display_df["steering_saturation_count"] = display_df["steering_saturation_count"].map(
        lambda value: "-" if pd.isna(value) else str(int(value))
    )
    for column in [
        "mean_abs_speed_error_mps",
        "speed_rms_error_mps",
        "lane_change_overshoot_m",
        "lane_change_overshoot_pct",
        "lane_change_settling_time_s",
        "p95_compute_time_ms",
        "p95_loop_period_ms",
        "yaw_rate_rms_deg_s",
    ]:
        if column in display_df.columns:
            display_df[column] = display_df[column].map(
                lambda value: "-" if pd.isna(value) else f"{value:.3f}"
            )
    if "pass_mean_heading_target" in display_df.columns:
        display_df["pass_mean_heading_target"] = display_df["pass_mean_heading_target"].map(
            lambda value: "-" if pd.isna(value) else str(bool(value))
        )
    if "pass_max_heading_target" in display_df.columns:
        display_df["pass_max_heading_target"] = display_df["pass_max_heading_target"].map(
            lambda value: "-" if pd.isna(value) else str(bool(value))
        )

    columns = [
        "log_name",
        "lookahead_distance_m",
        "trim_start_s",
        "trim_end_s",
        "mean_abs_e_y_cm",
        "p95_abs_e_y_cm",
        "max_abs_e_y_cm",
        "mean_abs_e_psi_deg",
        "p95_abs_e_psi_deg",
        "max_abs_e_psi_deg",
        "steering_rms_deg",
        "steering_rate_rms_deg_s",
        "steering_saturation_count",
        "mean_abs_speed_error_mps",
        "speed_rms_error_mps",
        "lane_change_overshoot_pct",
        "lane_change_settling_time_s",
        "p95_compute_time_ms",
        "p95_loop_period_ms",
        "yaw_rate_rms_deg_s",
        "pass_mean_target",
        "pass_max_target",
        "pass_mean_heading_target",
        "pass_max_heading_target",
    ]

    print("")
    print(f"Target lateral error threshold: {lateral_threshold * 100.0:.1f} cm")
    print(f"Target heading error threshold: {heading_threshold_deg:.1f} deg")
    print("Auto-trim removes the initial startup segment and the terminal tail after the path end.")
    if "evaluation_scope" in summary_df.columns and len(summary_df["evaluation_scope"].unique()) == 1:
        print(f"Evaluation scope: {summary_df['evaluation_scope'].iloc[0]}")
    print(display_df[columns].to_string(index=False))
    print("")
    print(build_recommendation(summary_df))


def main() -> None:
    """CLI entry point."""
    args = parse_args()
    output_dir = ensure_output_dir(args.output_dir)

    reference_trajectory_paths: list[Path] | None = None
    if args.reference_trajectory_csv is not None:
        reference_trajectory_paths = [Path(path_str).resolve() for path_str in args.reference_trajectory_csv]
        if len(reference_trajectory_paths) not in (1, len(args.log)):
            raise ValueError(
                "--reference-trajectory-csv must provide either one file or the same number of files as --log."
            )

    summaries: list[dict[str, object]] = []
    loaded_runs: list[pd.DataFrame] = []
    loaded_plot_runs: list[pd.DataFrame] = []
    loaded_reference_trajectories: list[tuple[np.ndarray, np.ndarray] | None] = []

    for idx, log_str in enumerate(args.log):
        log_path = Path(log_str).resolve()
        dataframe = load_log(log_path)
        reference_trajectory = None
        if reference_trajectory_paths is not None:
            reference_path = (
                reference_trajectory_paths[0]
                if len(reference_trajectory_paths) == 1
                else reference_trajectory_paths[idx]
            )
            reference_trajectory = load_reference_trajectory_csv(reference_path)
        if not args.disable_start_trim:
            dataframe = trim_startup_segment(
                dataframe,
                lateral_threshold=args.start_trim_lateral_threshold,
                heading_threshold_deg=args.start_trim_heading_threshold_deg,
                hold_time_s=args.start_trim_hold_s,
            )
        if not args.disable_end_trim:
            dataframe = trim_terminal_tail(
                dataframe,
                distance_threshold=args.end_trim_distance_threshold,
                reference_trajectory=reference_trajectory,
            )
        plot_dataframe = dataframe.copy()
        if args.focus_straight_segments:
            dataframe = filter_straight_segments(
                dataframe,
                curvature_threshold=args.straight_curvature_threshold,
                min_length_m=args.straight_min_length_m,
            )
        summary = summarize_log(
            dataframe,
            lateral_threshold=args.lateral_threshold,
            heading_threshold_deg=args.heading_threshold_deg,
        )
        summaries.append(summary)
        loaded_runs.append(dataframe)
        loaded_plot_runs.append(plot_dataframe)
        loaded_reference_trajectories.append(reference_trajectory)

    summary_df = pd.DataFrame(summaries)
    summary_df = summary_df.sort_values(
        ["lookahead_distance_m", "mean_abs_e_y_m", "log_name"],
        ascending=[True, True, True],
        kind="mergesort",
        na_position="last",
    ).reset_index(drop=True)

    print_summary(
        summary_df,
        lateral_threshold=args.lateral_threshold,
        heading_threshold_deg=args.heading_threshold_deg,
    )

    summary_csv_path = output_dir / "lookahead_summary.csv"
    summary_json_path = output_dir / "lookahead_summary.json"
    summary_df.to_csv(summary_csv_path, index=False)
    summary_json_path.write_text(
        json.dumps(summary_df.to_dict(orient="records"), indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    if not args.no_plot:
        for dataframe, plot_dataframe, summary, reference_trajectory in zip(
            loaded_runs,
            loaded_plot_runs,
            summaries,
            loaded_reference_trajectories,
        ):
            save_single_run_plot(
                dataframe,
                summary,
                lateral_threshold=args.lateral_threshold,
                output_dir=output_dir,
            )
            save_heading_error_plot(
                dataframe,
                summary,
                heading_threshold_deg=args.heading_threshold_deg,
                output_dir=output_dir,
            )
            save_steering_breakdown_plot(
                dataframe,
                summary,
                output_dir=output_dir,
            )
            save_speed_tracking_plot(
                dataframe,
                summary,
                output_dir=output_dir,
            )
            save_yaw_rate_plot(
                dataframe,
                summary,
                output_dir=output_dir,
            )
            save_timing_plot(
                dataframe,
                summary,
                output_dir=output_dir,
            )
            save_lane_change_response_plot(
                dataframe,
                summary,
                output_dir=output_dir,
            )
            save_path_overlay_plot(
                plot_dataframe,
                dataframe,
                summary,
                output_dir=output_dir,
                reference_trajectory=reference_trajectory,
            )
            if args.save_full_run_overlay and (
                str(summary.get("evaluation_scope", "full_run")) != "full_run"
            ):
                full_run_summary = dict(summary)
                full_run_summary["evaluation_scope"] = "full_run"
                save_path_overlay_plot(
                    plot_dataframe,
                    plot_dataframe,
                    full_run_summary,
                    output_dir=output_dir,
                    reference_trajectory=reference_trajectory,
                    output_suffix="path_overlay_full_run",
                    title_scope="full_run",
                )
        save_comparison_plot(
            summary_df,
            lateral_threshold=args.lateral_threshold,
            output_dir=output_dir,
        )
        save_heading_comparison_plot(
            summary_df,
            heading_threshold_deg=args.heading_threshold_deg,
            output_dir=output_dir,
        )
        save_response_comparison_plot(
            summary_df,
            output_dir=output_dir,
        )

    print(f"Saved summary CSV: {summary_csv_path}")
    print(f"Saved summary JSON: {summary_json_path}")
    if not args.no_plot:
        print(f"Saved plots under: {output_dir}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Render an ST-corridor validation run to MP4 from saved logs."""
from __future__ import annotations

import argparse
import csv
import json
import math
from bisect import bisect_right
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Rectangle
import numpy as np


def _read_xy(path: Path, x_key: str = "x", y_key: str = "y") -> np.ndarray:
    with path.open("r", encoding="utf-8") as f:
        return np.array([[float(r[x_key]), float(r[y_key])] for r in csv.DictReader(f)], dtype=float)


def _read_track(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    with path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    left = np.array([[float(r["left_border_x"]), float(r["left_border_y"])] for r in rows], dtype=float)
    right = np.array([[float(r["right_border_x"]), float(r["right_border_y"])] for r in rows], dtype=float)
    center = np.array([[float(r["pos_x"]), float(r["pos_y"])] for r in rows], dtype=float)
    return left, right, center


def _read_tracking(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        row["_time"] = float(row["time"])
        row["_x"] = float(row["x"])
        row["_y"] = float(row["y"])
        row["_yaw"] = float(row.get("yaw", 0.0))
        row["_speed"] = float(row.get("v_actual", 0.0))
    return rows


def _read_st_log(path: Path) -> tuple[list[float], list[str]]:
    if not path.exists():
        return [], []
    times: list[float] = []
    modes: list[str] = []
    with path.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            times.append(float(row["time"]))
            modes.append(str(row.get("mode", "")))
    return times, modes


def _mode_at(times: list[float], modes: list[str], t: float) -> str:
    if not times:
        return "unknown"
    idx = max(0, bisect_right(times, t) - 1)
    return modes[idx]


def _obstacles(path: Path) -> list[dict]:
    data = json.loads(path.read_text(encoding="utf-8"))
    return list(data.get("obstacles", []))


def _frame_indices(row_count: int, max_frames: int) -> np.ndarray:
    if row_count <= max_frames:
        return np.arange(row_count, dtype=int)
    return np.unique(np.linspace(0, row_count - 1, max_frames).astype(int))


def render(args: argparse.Namespace) -> None:
    track_left, track_right, track_center = _read_track(Path(args.track_csv))
    global_path = _read_xy(Path(args.global_trajectory))
    tracking = _read_tracking(Path(args.tracking_log))
    st_times, st_modes = _read_st_log(Path(args.st_log))
    obstacles = _obstacles(Path(args.obstacle_manifest))

    frame_count = max(2, int(args.fps * args.duration))
    indices = _frame_indices(len(tracking), frame_count)
    xs = np.array([r["_x"] for r in tracking], dtype=float)
    ys = np.array([r["_y"] for r in tracking], dtype=float)

    all_points = np.vstack([track_left, track_right, track_center, global_path, np.column_stack([xs, ys])])
    pad = 0.8
    x_min, y_min = np.min(all_points, axis=0) - pad
    x_max, y_max = np.max(all_points, axis=0) + pad

    fig, ax = plt.subplots(figsize=(9.6, 7.2), dpi=100)
    fig.patch.set_facecolor("#f7f7f3")
    ax.set_facecolor("#ffffff")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, color="#e0e0dc", linewidth=0.6)

    ax.plot(track_left[:, 0], track_left[:, 1], color="#4b5563", linewidth=1.4, label="track border")
    ax.plot(track_right[:, 0], track_right[:, 1], color="#4b5563", linewidth=1.4)
    ax.plot(track_center[:, 0], track_center[:, 1], color="#cbd5e1", linewidth=0.8)
    ax.plot(global_path[:, 0], global_path[:, 1], color="#2563eb", linewidth=1.2, alpha=0.75, label="global path")

    for obs in obstacles:
        size = float(obs.get("size_m", 0.5))
        x = float(obs["x"]) - 0.5 * size
        y = float(obs["y"]) - 0.5 * size
        ax.add_patch(Rectangle((x, y), size, size, facecolor="#ef4444", edgecolor="#991b1b", alpha=0.8))

    trail_line, = ax.plot([], [], color="#0f766e", linewidth=2.2, label="ego trace")
    car_dot, = ax.plot([], [], marker="o", markersize=7, color="#111827")
    heading_line, = ax.plot([], [], color="#111827", linewidth=1.5)
    status = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=11,
        bbox={"facecolor": "white", "edgecolor": "#d1d5db", "alpha": 0.9, "boxstyle": "round,pad=0.3"},
    )
    ax.legend(loc="lower right", framealpha=0.9)

    writer = FFMpegWriter(fps=args.fps, metadata={"title": args.title}, bitrate=2200)
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)

    start_time = tracking[0]["_time"]
    with writer.saving(fig, args.output, dpi=100):
        for idx in indices:
            row = tracking[int(idx)]
            x, y, yaw = row["_x"], row["_y"], row["_yaw"]
            mode = _mode_at(st_times, st_modes, row["_time"])
            trail_line.set_data(xs[: idx + 1], ys[: idx + 1])
            car_dot.set_data([x], [y])
            heading_line.set_data(
                [x, x + 0.45 * math.cos(yaw)],
                [y, y + 0.45 * math.sin(yaw)],
            )
            color = "#dc2626" if mode == "avoid" else "#f59e0b" if mode == "blocked" else "#111827"
            car_dot.set_color(color)
            status.set_text(
                f"{args.title}\n"
                f"t={row['_time'] - start_time:5.1f}s  speed={row['_speed']:.2f} m/s  ST={mode}"
            )
            writer.grab_frame()
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track-csv", default="/sim_ws/src/f1tenth_gym_ros/code/outputs/csv/processed_track.csv")
    parser.add_argument("--global-trajectory", default="/sim_ws/src/f1tenth_gym_ros/code/outputs/csv/global_trajectory.csv")
    parser.add_argument("--tracking-log", required=True)
    parser.add_argument("--st-log", required=True)
    parser.add_argument("--obstacle-manifest", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--title", default="ST corridor validation")
    parser.add_argument("--duration", type=float, default=18.0)
    parser.add_argument("--fps", type=int, default=20)
    return parser.parse_args()


if __name__ == "__main__":
    render(parse_args())

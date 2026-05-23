#!/usr/bin/env python3
"""Generate a longer race-style F1TENTH track with map and CSV artifacts."""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np
from scipy.interpolate import splev, splprep


def _race_waypoints() -> np.ndarray:
    """Return a closed-loop control polygon with straights and hairpin sections."""

    return np.array(
        [
            [0.0, 0.0],
            [7.0, 0.0],
            [14.0, 0.0],
            [19.0, 2.0],
            [20.5, 6.0],
            [18.0, 9.5],
            [12.0, 9.8],
            [7.0, 8.0],
            [4.2, 4.8],
            [6.7, 1.8],
            [13.0, 2.0],
            [17.0, 4.5],
            [15.0, 7.5],
            [9.5, 7.0],
            [4.0, 6.2],
            [-2.5, 5.5],
            [-7.5, 8.5],
            [-12.5, 7.0],
            [-14.5, 3.5],
            [-12.0, 0.8],
            [-6.0, 0.2],
        ],
        dtype=float,
    )


def _resample_closed(points: np.ndarray, spacing: float) -> tuple[np.ndarray, np.ndarray]:
    closed = np.vstack([points, points[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    total = float(seg.sum())
    target_count = max(80, int(math.ceil(total / spacing)))
    u = np.concatenate([[0.0], np.cumsum(seg) / total])
    tck, _ = splprep([closed[:, 0], closed[:, 1]], u=u, s=0.8, per=True, k=3)
    sample_u = np.linspace(0.0, 1.0, target_count, endpoint=False)
    x, y = splev(sample_u, tck, der=0)
    dx, dy = splev(sample_u, tck, der=1)
    center = np.column_stack([x, y])
    yaw = np.arctan2(dy, dx)
    return center, yaw


def _curvature(points: np.ndarray) -> np.ndarray:
    prev_pt = np.roll(points, 1, axis=0)
    next_pt = np.roll(points, -1, axis=0)
    ds_prev = np.linalg.norm(points - prev_pt, axis=1)
    ds_next = np.linalg.norm(next_pt - points, axis=1)
    ds = np.maximum(0.5 * (ds_prev + ds_next), 1e-6)
    first = (next_pt - prev_pt) / np.maximum((ds_prev + ds_next)[:, None], 1e-6)
    second = (next_pt - 2.0 * points + prev_pt) / np.maximum(ds[:, None] ** 2, 1e-6)
    denom = np.maximum(np.linalg.norm(first, axis=1) ** 3, 1e-9)
    return (first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0]) / denom


def _write_trajectory(path: Path, center: np.ndarray, yaw: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "yaw"])
        for (x, y), heading in zip(center, yaw):
            writer.writerow([f"{x:.7f}", f"{y:.7f}", f"{heading:.7f}"])


def _write_track(path: Path, center: np.ndarray, yaw: np.ndarray, half_width: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lapdist = 0.0
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "left_border_x",
                "left_border_y",
                "right_border_x",
                "right_border_y",
                "pos_x",
                "pos_y",
                "Lapdist",
            ]
        )
        for idx, ((x, y), heading) in enumerate(zip(center, yaw)):
            if idx > 0:
                lapdist += float(np.linalg.norm(center[idx] - center[idx - 1]))
            nx = -math.sin(float(heading))
            ny = math.cos(float(heading))
            writer.writerow(
                [
                    f"{x + half_width * nx:.7f}",
                    f"{y + half_width * ny:.7f}",
                    f"{x - half_width * nx:.7f}",
                    f"{y - half_width * ny:.7f}",
                    f"{x:.7f}",
                    f"{y:.7f}",
                    f"{lapdist:.7f}",
                ]
            )


def _write_map(
    pgm_path: Path,
    yaml_path: Path,
    center: np.ndarray,
    half_width: float,
    resolution: float,
    margin: float,
) -> None:
    min_xy = np.min(center, axis=0) - (half_width + margin)
    max_xy = np.max(center, axis=0) + (half_width + margin)
    width = int(math.ceil((max_xy[0] - min_xy[0]) / resolution))
    height = int(math.ceil((max_xy[1] - min_xy[1]) / resolution))
    pixels = bytearray([0] * (width * height))
    free_radius_px = int(math.ceil(half_width / resolution))

    def world_to_pixel(point: np.ndarray) -> tuple[int, int]:
        col = int(round((float(point[0]) - float(min_xy[0])) / resolution))
        row_from_bottom = int(round((float(point[1]) - float(min_xy[1])) / resolution))
        return col, height - 1 - row_from_bottom

    for point in center:
        col, row = world_to_pixel(point)
        r0 = max(0, row - free_radius_px)
        r1 = min(height, row + free_radius_px + 1)
        c0 = max(0, col - free_radius_px)
        c1 = min(width, col + free_radius_px + 1)
        for rr in range(r0, r1):
            dy = (rr - row) * resolution
            offset = rr * width
            for cc in range(c0, c1):
                dx = (cc - col) * resolution
                if math.hypot(dx, dy) <= half_width:
                    pixels[offset + cc] = 254

    pgm_path.parent.mkdir(parents=True, exist_ok=True)
    with pgm_path.open("wb") as f:
        f.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        f.write(pixels)

    yaml_path.write_text(
        "\n".join(
            [
                f"image: {pgm_path.name}",
                "mode: trinary",
                f"resolution: {resolution}",
                f"origin: [{float(min_xy[0]):.6f}, {float(min_xy[1]):.6f}, 0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            ]
        ),
        encoding="ascii",
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-prefix", default="maps/race_style_long")
    parser.add_argument("--trajectory-csv", default="code/outputs/csv/race_style_long_trajectory.csv")
    parser.add_argument("--track-csv", default="code/outputs/csv/race_style_long_processed_track.csv")
    parser.add_argument("--track-width", type=float, default=4.0)
    parser.add_argument("--spacing", type=float, default=0.08)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--margin", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    center, yaw = _resample_closed(_race_waypoints(), args.spacing)
    half_width = 0.5 * args.track_width
    map_prefix = Path(args.map_prefix)
    pgm_path = map_prefix.with_suffix(".pgm")
    yaml_path = map_prefix.with_suffix(".yaml")
    _write_trajectory(Path(args.trajectory_csv), center, yaw)
    _write_track(Path(args.track_csv), center, yaw, half_width)
    _write_map(pgm_path, yaml_path, center, half_width, args.resolution, args.margin)
    lap_length = float(np.sum(np.linalg.norm(np.roll(center, -1, axis=0) - center, axis=1)))
    max_curvature = float(np.max(np.abs(_curvature(center))))
    print(f"map_path={pgm_path.with_suffix('')}")
    print(f"map_yaml={yaml_path}")
    print(f"track_csv={Path(args.track_csv)}")
    print(f"trajectory_csv={Path(args.trajectory_csv)}")
    print(f"points={len(center)} lap_length_m={lap_length:.2f} max_curvature_1pm={max_curvature:.3f}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Generate a reproducible map with square static obstacles on the reference path."""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import clean_map
from package_paths import SIM_WS_ROOT


def load_xy_csv(path: Path) -> np.ndarray:
    points: list[list[float]] = []
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append([float(row["x"]), float(row["y"])])
    if len(points) < 3:
        raise ValueError(f"trajectory needs at least 3 points: {path}")
    return np.asarray(points, dtype=float)


def world_to_image(
    point: np.ndarray,
    origin: list[float],
    resolution: float,
    height: int,
) -> tuple[int, int]:
    col = int(round((float(point[0]) - float(origin[0])) / resolution))
    row_from_bottom = int(round((float(point[1]) - float(origin[1])) / resolution))
    return height - 1 - row_from_bottom, col


def square_slice(
    center_row: int,
    center_col: int,
    side_px: int,
    shape: tuple[int, int],
) -> tuple[slice, slice]:
    half_low = side_px // 2
    half_high = side_px - half_low
    row_start = max(0, center_row - half_low)
    row_stop = min(shape[0], center_row + half_high)
    col_start = max(0, center_col - half_low)
    col_stop = min(shape[1], center_col + half_high)
    return slice(row_start, row_stop), slice(col_start, col_stop)


def choose_obstacle_centers(
    trajectory: np.ndarray,
    image: np.ndarray,
    origin: list[float],
    resolution: float,
    side_px: int,
    count: int,
    seed: int,
    min_start_distance_m: float,
    min_separation_m: float,
) -> list[np.ndarray]:
    rng = np.random.default_rng(seed)
    order = rng.permutation(np.arange(len(trajectory)))
    start = trajectory[0]
    centers: list[np.ndarray] = []

    for idx in order:
        point = trajectory[int(idx)]
        if np.linalg.norm(point - start) < min_start_distance_m:
            continue
        if any(np.linalg.norm(point - center) < min_separation_m for center in centers):
            continue

        row, col = world_to_image(point, origin, resolution, image.shape[0])
        rows, cols = square_slice(row, col, side_px, image.shape)
        if rows.stop - rows.start < side_px or cols.stop - cols.start < side_px:
            continue

        patch = image[rows, cols]
        if not np.all(patch >= 250):
            continue

        centers.append(point.copy())
        if len(centers) == count:
            return centers

    raise RuntimeError(
        f"could not place {count} square obstacles; try another seed or smaller size"
    )


def choose_obstacle_centers_by_distance(
    trajectory: np.ndarray,
    image: np.ndarray,
    origin: list[float],
    resolution: float,
    side_px: int,
    distances_m: list[float],
) -> list[np.ndarray]:
    cumulative = trajectory_cumulative_distance(trajectory)
    centers: list[np.ndarray] = []
    for distance_m in distances_m:
        idx = int(np.argmin(np.abs(cumulative - distance_m)))
        point = trajectory[idx]
        row, col = world_to_image(point, origin, resolution, image.shape[0])
        rows, cols = square_slice(row, col, side_px, image.shape)
        if rows.stop - rows.start < side_px or cols.stop - cols.start < side_px:
            raise RuntimeError(f"obstacle at {distance_m:.2f}m is too close to map edge")
        patch = image[rows, cols]
        if not np.all(patch >= 250):
            raise RuntimeError(
                f"obstacle at {distance_m:.2f}m is not fully on free space"
            )
        centers.append(point.copy())
    return centers


def trajectory_cumulative_distance(trajectory: np.ndarray) -> np.ndarray:
    diffs = np.diff(trajectory, axis=0)
    lengths = np.linalg.norm(diffs, axis=1)
    return np.concatenate([[0.0], np.cumsum(lengths)])


def write_obstacle_map(
    base_yaml: Path,
    trajectory_csv: Path,
    output_prefix: Path,
    obstacle_count: int,
    obstacle_size_m: float,
    seed: int,
    min_start_distance_m: float,
    min_separation_m: float,
    obstacle_distances_m: list[float] | None,
) -> dict[str, object]:
    metadata = clean_map.parse_simple_yaml(base_yaml)
    resolution = float(metadata["resolution"])
    origin = list(metadata["origin"])
    image_path = (base_yaml.parent / str(metadata["image"])).resolve()
    image = np.asarray(Image.open(image_path), dtype=np.uint8).copy()
    trajectory = load_xy_csv(trajectory_csv)

    side_px = max(1, int(round(obstacle_size_m / resolution)))
    if obstacle_distances_m:
        centers = choose_obstacle_centers_by_distance(
            trajectory,
            image,
            origin,
            resolution,
            side_px,
            obstacle_distances_m,
        )
        obstacle_count = len(centers)
    else:
        centers = choose_obstacle_centers(
            trajectory,
            image,
            origin,
            resolution,
            side_px,
            obstacle_count,
            seed,
            min_start_distance_m,
            min_separation_m,
        )

    obstacles: list[dict[str, object]] = []
    for center in centers:
        row, col = world_to_image(center, origin, resolution, image.shape[0])
        rows, cols = square_slice(row, col, side_px, image.shape)
        image[rows, cols] = 0
        obstacles.append(
            {
                "center_x_m": float(center[0]),
                "center_y_m": float(center[1]),
                "size_m": obstacle_size_m,
                "row_range": [int(rows.start), int(rows.stop)],
                "col_range": [int(cols.start), int(cols.stop)],
            }
        )

    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    output_image = output_prefix.with_suffix(".pgm")
    output_yaml = output_prefix.with_suffix(".yaml")
    output_json = output_prefix.with_suffix(".json")

    Image.fromarray(image, mode="L").save(output_image)
    yaml_out = dict(metadata)
    yaml_out["image"] = output_image.name
    clean_map.dump_simple_yaml(output_yaml, yaml_out)

    summary = {
        "base_yaml": str(base_yaml),
        "trajectory_csv": str(trajectory_csv),
        "output_yaml": str(output_yaml),
        "obstacle_count": obstacle_count,
        "obstacle_size_m": obstacle_size_m,
        "seed": seed,
        "obstacle_distances_m": obstacle_distances_m,
        "obstacles": obstacles,
    }
    output_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-yaml", type=Path, default=SIM_WS_ROOT / "maps" / "my_map.yaml")
    parser.add_argument(
        "--trajectory-csv",
        type=Path,
        default=SIM_WS_ROOT / "code" / "outputs" / "csv" / "global_trajectory.csv",
    )
    parser.add_argument(
        "--output-prefix",
        type=Path,
        default=SIM_WS_ROOT / "maps" / "generated_static_obstacles" / "two_blocks_seed7",
    )
    parser.add_argument("--obstacle-count", type=int, default=2)
    parser.add_argument("--obstacle-size-m", type=float, default=0.4)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--obstacle-distances-m",
        default="4.0,7.0",
        help=(
            "Comma-separated distances along global_trajectory. "
            "Use an empty string for random placement."
        ),
    )
    parser.add_argument("--min-start-distance-m", type=float, default=3.0)
    parser.add_argument("--min-separation-m", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    obstacle_distances = parse_distance_list(args.obstacle_distances_m)
    summary = write_obstacle_map(
        base_yaml=args.base_yaml.resolve(),
        trajectory_csv=args.trajectory_csv.resolve(),
        output_prefix=args.output_prefix.resolve(),
        obstacle_count=max(1, int(args.obstacle_count)),
        obstacle_size_m=max(0.05, float(args.obstacle_size_m)),
        seed=int(args.seed),
        min_start_distance_m=max(0.0, float(args.min_start_distance_m)),
        min_separation_m=max(0.0, float(args.min_separation_m)),
        obstacle_distances_m=obstacle_distances,
    )
    print(f"output_yaml={summary['output_yaml']}")
    for idx, obstacle in enumerate(summary["obstacles"], start=1):
        print(
            "obstacle_"
            f"{idx}=({obstacle['center_x_m']:.3f}, {obstacle['center_y_m']:.3f}) "
            f"size={obstacle['size_m']:.2f}m"
        )


def parse_distance_list(text: str) -> list[float] | None:
    text = text.strip()
    if not text:
        return None
    return [float(item.strip()) for item in text.split(",") if item.strip()]


if __name__ == "__main__":
    main()

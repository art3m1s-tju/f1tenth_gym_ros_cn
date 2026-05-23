#!/usr/bin/env python3
"""Audit saved robustness runs without rerunning simulation."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SIM_REPO = Path("/sim_ws/src/f1tenth_gym_ros")
VEHICLE_LENGTH_M = 0.535
VEHICLE_WIDTH_M = 0.281


def _local_path(path: str | Path) -> Path:
    raw = Path(path)
    if raw.is_absolute() and str(raw).startswith(str(SIM_REPO)):
        return REPO_ROOT / raw.relative_to(SIM_REPO)
    return raw


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_reference(path: Path) -> list[tuple[float, float]]:
    with path.open("r", encoding="utf-8") as f:
        return [(float(row["x"]), float(row["y"])) for row in csv.DictReader(f)]


def _nearest_index(x: float, y: float, points: list[tuple[float, float]]) -> int:
    best_idx = 0
    best_dist = math.inf
    for idx, (px, py) in enumerate(points):
        dist = (x - px) * (x - px) + (y - py) * (y - py)
        if dist < best_dist:
            best_dist = dist
            best_idx = idx
    return best_idx


def _completed_laps(tracking_log: Path, reference: list[tuple[float, float]]) -> int:
    if not tracking_log.exists() or len(reference) < 10:
        return 0
    high_idx = 0.75 * (len(reference) - 1)
    low_idx = 0.25 * (len(reference) - 1)
    laps = 0
    previous_idx = None
    with tracking_log.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            idx = _nearest_index(float(row["x"]), float(row["y"]), reference)
            if previous_idx is not None and previous_idx > high_idx and idx < low_idx:
                laps += 1
            previous_idx = idx
    return laps


def _polygon_edges(poly: list[tuple[float, float]]) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    return list(zip(poly, poly[1:] + poly[:1]))


def _project(poly: list[tuple[float, float]], axis: tuple[float, float]) -> tuple[float, float]:
    values = [x * axis[0] + y * axis[1] for x, y in poly]
    return min(values), max(values)


def _overlap(poly_a: list[tuple[float, float]], poly_b: list[tuple[float, float]]) -> bool:
    for p0, p1 in _polygon_edges(poly_a) + _polygon_edges(poly_b):
        edge_x = p1[0] - p0[0]
        edge_y = p1[1] - p0[1]
        length = math.hypot(edge_x, edge_y)
        if length <= 1e-12:
            continue
        axis = (-edge_y / length, edge_x / length)
        min_a, max_a = _project(poly_a, axis)
        min_b, max_b = _project(poly_b, axis)
        if max_a < min_b or max_b < min_a:
            return False
    return True


def _point_segment_distance(
    point: tuple[float, float],
    seg_a: tuple[float, float],
    seg_b: tuple[float, float],
) -> float:
    px, py = point
    ax, ay = seg_a
    bx, by = seg_b
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _segment_distance(
    a0: tuple[float, float],
    a1: tuple[float, float],
    b0: tuple[float, float],
    b1: tuple[float, float],
) -> float:
    return min(
        _point_segment_distance(a0, b0, b1),
        _point_segment_distance(a1, b0, b1),
        _point_segment_distance(b0, a0, a1),
        _point_segment_distance(b1, a0, a1),
    )


def _polygon_distance(poly_a: list[tuple[float, float]], poly_b: list[tuple[float, float]]) -> float:
    if _overlap(poly_a, poly_b):
        return 0.0
    return min(
        _segment_distance(a0, a1, b0, b1)
        for a0, a1 in _polygon_edges(poly_a)
        for b0, b1 in _polygon_edges(poly_b)
    )


def _vehicle_polygon(x: float, y: float, yaw: float) -> list[tuple[float, float]]:
    half_l = 0.5 * VEHICLE_LENGTH_M
    half_w = 0.5 * VEHICLE_WIDTH_M
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    corners = [(half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)]
    return [
        (x + lx * cos_yaw - ly * sin_yaw, y + lx * sin_yaw + ly * cos_yaw)
        for lx, ly in corners
    ]


def _obstacle_polygon(obstacle: dict) -> list[tuple[float, float]]:
    half = 0.5 * float(obstacle["size_m"])
    x = float(obstacle["x"])
    y = float(obstacle["y"])
    return [
        (x - half, y - half),
        (x + half, y - half),
        (x + half, y + half),
        (x - half, y + half),
    ]


def _footprint_audit(tracking_log: Path, obstacle_manifest: Path) -> dict:
    if not obstacle_manifest.exists():
        return {"footprint_hits": 0, "min_vehicle_footprint_clearance_m": math.nan}
    obstacles = json.loads(obstacle_manifest.read_text(encoding="utf-8")).get("obstacles", [])
    obstacle_polys = [_obstacle_polygon(obstacle) for obstacle in obstacles]
    hits = 0
    min_clearance = math.inf
    with tracking_log.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            vehicle = _vehicle_polygon(float(row["x"]), float(row["y"]), float(row["yaw"]))
            for obstacle in obstacle_polys:
                distance = _polygon_distance(vehicle, obstacle)
                if distance <= 1e-9:
                    hits += 1
                min_clearance = min(min_clearance, distance)
    return {
        "footprint_hits": hits,
        "min_vehicle_footprint_clearance_m": min_clearance if math.isfinite(min_clearance) else math.nan,
    }


def _read_rows(manifest: Path) -> list[dict]:
    with manifest.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _audit_manifest(manifest: Path, reference: list[tuple[float, float]]) -> list[dict]:
    audited = []
    for row in _read_rows(manifest):
        tracking_log = _local_path(row["tracking_log"])
        obstacle_manifest = _local_path(row["obstacle_manifest"]) if row.get("obstacle_manifest") else None
        footprint = (
            _footprint_audit(tracking_log, obstacle_manifest)
            if obstacle_manifest is not None
            else {"footprint_hits": 0, "min_vehicle_footprint_clearance_m": math.nan}
        )
        completed_laps = _completed_laps(tracking_log, reference)
        audited.append(
            {
                "source_manifest": str(manifest),
                "mode": row["mode"],
                "speed": row["speed"],
                "pass": row["pass"],
                "laps_required": row["laps_required"],
                "completed_laps_from_global_projection": completed_laps,
                "collision_events": row["collision_events"],
                "obstacle_count": row["obstacle_count"],
                "obstacle_seed": row["obstacle_seed"],
                "obstacle_center_hits": row.get("obstacle_center_hits", ""),
                "min_obstacle_center_clearance_m": row.get("min_obstacle_center_clearance_m", ""),
                "footprint_hits": footprint["footprint_hits"],
                "min_vehicle_footprint_clearance_m": footprint["min_vehicle_footprint_clearance_m"],
                "st_avoid_rows": row.get("st_avoid_rows", ""),
                "st_blocked_rows": row.get("st_blocked_rows", ""),
                "tracking_log": str(tracking_log),
                "obstacle_manifest": str(obstacle_manifest) if obstacle_manifest is not None else "",
            }
        )
    return audited


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory-csv", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("manifests", nargs="+")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    trajectory_csv = _local_path(args.trajectory_csv)
    reference = _load_reference(trajectory_csv)
    manifests = [_local_path(path) for path in args.manifests]
    rows: list[dict] = []
    for manifest in manifests:
        rows.extend(_audit_manifest(manifest, reference))

    output = _local_path(args.output)
    _write_csv(output, rows)
    hashes = {
        str(path): _sha256(path)
        for path in [
            REPO_ROOT / "code" / "generate_race_style_track.py",
            REPO_ROOT / "maps" / "race_style_long.yaml",
            REPO_ROOT / "maps" / "race_style_long.pgm",
            REPO_ROOT / "code" / "outputs" / "csv" / "race_style_long_trajectory.csv",
            REPO_ROOT / "code" / "outputs" / "csv" / "race_style_long_processed_track.csv",
        ]
        if path.exists()
    }
    output.with_suffix(".json").write_text(
        json.dumps({"audit_rows": rows, "artifact_sha256": hashes}, indent=2),
        encoding="utf-8",
    )
    print(f"audit_csv={output}")
    print(f"audit_json={output.with_suffix('.json')}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""LiDAR-based local ST-corridor planner.

This node keeps the global trajectory as the nominal reference and publishes a
short local trajectory that bends around obstacles detected inside the drivable
corridor. It is intentionally conservative: walls are filtered with the track
borders, dynamic prediction is short horizon only, and the speed output is a
limit consumed by the LQR controller.
"""
from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from package_paths import get_default_output_root


@dataclass(frozen=True)
class Projection:
    s: float
    d: float
    point: np.ndarray
    heading: float
    index: int


@dataclass(frozen=True)
class CorridorObstacle:
    centroid_xy: np.ndarray
    s: float
    d: float
    radius: float
    vs: float
    vd: float
    points: int


@dataclass(frozen=True)
class StaticSquareObstacle:
    center_xy: np.ndarray
    size_m: float


class ReferencePath:
    def __init__(self, points: np.ndarray, closed_loop: bool = True) -> None:
        if len(points) < 3:
            raise ValueError("ReferencePath requires at least 3 points")
        self.points = points.astype(float)
        self.closed_loop = closed_loop
        self.tree = cKDTree(self.points)
        self.segment_lengths = self._segment_lengths()
        self.s = np.concatenate([[0.0], np.cumsum(self.segment_lengths[:-1])])
        self.length = float(np.sum(self.segment_lengths))
        self.headings = self._headings()

    def _segment_lengths(self) -> np.ndarray:
        if self.closed_loop:
            nxt = np.roll(self.points, -1, axis=0)
            return np.linalg.norm(nxt - self.points, axis=1)
        return np.linalg.norm(self.points[1:] - self.points[:-1], axis=1)

    def _headings(self) -> np.ndarray:
        if self.closed_loop:
            delta = np.roll(self.points, -1, axis=0) - self.points
        else:
            delta = np.vstack([self.points[1:] - self.points[:-1], self.points[-1] - self.points[-2]])
        return np.arctan2(delta[:, 1], delta[:, 0])

    def wrap_s(self, s_value: float) -> float:
        if self.closed_loop and self.length > 1e-9:
            return float(s_value % self.length)
        return float(np.clip(s_value, 0.0, self.length))

    def signed_delta_s(self, s_value: float, origin_s: float) -> float:
        delta = self.wrap_s(s_value) - self.wrap_s(origin_s)
        if self.closed_loop and self.length > 1e-9:
            if delta > 0.5 * self.length:
                delta -= self.length
            elif delta < -0.5 * self.length:
                delta += self.length
        return float(delta)

    def interpolate(self, s_value: float, d: float = 0.0) -> tuple[np.ndarray, float]:
        s_wrapped = self.wrap_s(s_value)
        idx = int(np.searchsorted(self.s, s_wrapped, side="right") - 1)
        idx = int(np.clip(idx, 0, len(self.points) - 1))
        next_idx = (idx + 1) % len(self.points) if self.closed_loop else min(idx + 1, len(self.points) - 1)
        seg_len = max(float(self.segment_lengths[idx]), 1e-9)
        ratio = float(np.clip((s_wrapped - self.s[idx]) / seg_len, 0.0, 1.0))
        point = self.points[idx] + ratio * (self.points[next_idx] - self.points[idx])
        heading = float(self.headings[idx])
        normal = np.array([-math.sin(heading), math.cos(heading)], dtype=float)
        return point + d * normal, heading

    def project(self, xy: np.ndarray) -> Projection:
        _, nearest_idx = self.tree.query(xy)
        nearest_idx = int(nearest_idx)
        candidate_indices = [nearest_idx]
        if self.closed_loop or nearest_idx > 0:
            candidate_indices.append((nearest_idx - 1) % len(self.points))
        if self.closed_loop or nearest_idx < len(self.points) - 1:
            candidate_indices.append(nearest_idx)

        best: tuple[float, float, float, np.ndarray, float, int] | None = None
        for idx in candidate_indices:
            next_idx = (idx + 1) % len(self.points) if self.closed_loop else min(idx + 1, len(self.points) - 1)
            a = self.points[idx]
            b = self.points[next_idx]
            ab = b - a
            denom = float(ab @ ab)
            if denom <= 1e-12:
                continue
            t = float(np.clip(((xy - a) @ ab) / denom, 0.0, 1.0))
            point = a + t * ab
            dist = float(np.linalg.norm(xy - point))
            s_value = self.s[idx] + t * float(self.segment_lengths[idx])
            heading = math.atan2(float(ab[1]), float(ab[0]))
            normal = np.array([-math.sin(heading), math.cos(heading)], dtype=float)
            d_value = float((xy - point) @ normal)
            record = (dist, s_value, d_value, point, heading, idx)
            if best is None or record[0] < best[0]:
                best = record
        assert best is not None
        _, s_value, d_value, point, heading, idx = best
        return Projection(self.wrap_s(s_value), d_value, point, heading, idx)


class TrackFilter:
    def __init__(self, csv_path: Path) -> None:
        rows = list(csv.DictReader(csv_path.open("r", encoding="utf-8")))
        self.left = np.array([[float(r["left_border_x"]), float(r["left_border_y"])] for r in rows])
        self.right = np.array([[float(r["right_border_x"]), float(r["right_border_y"])] for r in rows])
        center = np.array([[float(r["pos_x"]), float(r["pos_y"])] for r in rows])
        self.left_tree = cKDTree(self.left)
        self.right_tree = cKDTree(self.right)
        self.center_ref = ReferencePath(center, closed_loop=True)

    def border_clearance(self, xy: np.ndarray) -> float:
        left_dist, _ = self.left_tree.query(xy)
        right_dist, _ = self.right_tree.query(xy)
        return float(min(left_dist, right_dist))

    def inside_drivable_area(self, xy: np.ndarray, margin: float) -> bool:
        projection = self.center_ref.project(xy)
        left = self.left[projection.index]
        right = self.right[projection.index]
        half_width = 0.5 * float(np.linalg.norm(left - right))
        return abs(projection.d) <= max(0.0, half_width - margin)


class StCorridorPlanner(Node):
    def __init__(self) -> None:
        super().__init__("st_corridor_planner")
        output_root = get_default_output_root()
        self.declare_parameter("global_path_topic", "/global_trajectory")
        self.declare_parameter("local_path_topic", "/local_trajectory")
        self.declare_parameter("speed_limit_topic", "/local_speed_limit")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("track_csv", str(output_root / "csv" / "processed_track.csv"))
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("vehicle_width", 0.31)
        self.declare_parameter("scan_x_offset_m", 0.275)
        self.declare_parameter("inflation_margin_m", 0.12)
        self.declare_parameter("lookahead_m", 5.0)
        self.declare_parameter("sample_ds_m", 0.25)
        self.declare_parameter("front_fov_deg", 150.0)
        self.declare_parameter("max_obstacle_range_m", 5.0)
        self.declare_parameter("cluster_distance_m", 0.22)
        self.declare_parameter("min_cluster_points", 2)
        self.declare_parameter("max_cluster_radius_m", 0.45)
        self.declare_parameter("max_lateral_offset_m", 0.75)
        self.declare_parameter("lateral_offset_step_m", 0.15)
        self.declare_parameter("target_speed", 1.0)
        self.declare_parameter("avoidance_max_speed", 1.0)
        self.declare_parameter("min_speed", 0.35)
        self.declare_parameter("min_border_clearance_m", 0.26)
        self.declare_parameter("obstacle_wall_margin_m", 0.20)
        self.declare_parameter("static_obstacle_manifest_path", "")
        self.declare_parameter("static_obstacle_activation_margin_m", 2.0)
        self.declare_parameter("takeover_distance_m", 0.0)
        self.declare_parameter("desired_obstacle_clearance_m", 0.12)
        self.declare_parameter("dynamic_prediction_time_s", 1.2)
        self.declare_parameter("publish_period_s", 0.05)
        self.declare_parameter("enable_csv_log", True)
        self.declare_parameter("log_path", str(output_root / "logs" / "st_corridor_log.csv"))

        self.global_path_topic = str(self.get_parameter("global_path_topic").value)
        self.local_path_topic = str(self.get_parameter("local_path_topic").value)
        self.speed_limit_topic = str(self.get_parameter("speed_limit_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.vehicle_width = float(self.get_parameter("vehicle_width").value)
        self.scan_x_offset_m = float(self.get_parameter("scan_x_offset_m").value)
        self.inflation_margin = float(self.get_parameter("inflation_margin_m").value)
        self.lookahead_m = float(self.get_parameter("lookahead_m").value)
        self.sample_ds_m = float(self.get_parameter("sample_ds_m").value)
        self.front_fov_rad = math.radians(float(self.get_parameter("front_fov_deg").value))
        self.max_obstacle_range_m = float(self.get_parameter("max_obstacle_range_m").value)
        self.cluster_distance_m = float(self.get_parameter("cluster_distance_m").value)
        self.min_cluster_points = int(self.get_parameter("min_cluster_points").value)
        self.max_cluster_radius_m = float(self.get_parameter("max_cluster_radius_m").value)
        self.max_lateral_offset_m = float(self.get_parameter("max_lateral_offset_m").value)
        self.lateral_offset_step_m = float(self.get_parameter("lateral_offset_step_m").value)
        self.target_speed = float(self.get_parameter("target_speed").value)
        self.avoidance_max_speed = float(self.get_parameter("avoidance_max_speed").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.min_border_clearance = float(self.get_parameter("min_border_clearance_m").value)
        self.obstacle_wall_margin = float(self.get_parameter("obstacle_wall_margin_m").value)
        self.static_obstacle_manifest_path = str(self.get_parameter("static_obstacle_manifest_path").value)
        self.static_obstacle_activation_margin = float(
            self.get_parameter("static_obstacle_activation_margin_m").value
        )
        self.takeover_distance_m = float(self.get_parameter("takeover_distance_m").value)
        self.desired_obstacle_clearance = float(self.get_parameter("desired_obstacle_clearance_m").value)
        self.dynamic_prediction_time = float(self.get_parameter("dynamic_prediction_time_s").value)
        self.enable_csv_log = bool(self.get_parameter("enable_csv_log").value)
        self.log_path = Path(str(self.get_parameter("log_path").value)).expanduser().resolve()

        track_csv = Path(str(self.get_parameter("track_csv").value)).expanduser().resolve()
        self.track_filter = TrackFilter(track_csv)
        self.reference: ReferencePath | None = None
        self.odom_pose: tuple[np.ndarray, float] | None = None
        self.latest_scan: LaserScan | None = None
        self.previous_obstacles: list[CorridorObstacle] = []
        self.previous_obstacle_time: float | None = None
        self.static_obstacles = self._load_static_obstacles(self.static_obstacle_manifest_path)
        self.csv_file = None
        self.csv_writer = None

        if self.enable_csv_log:
            self._open_log()

        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_sub = self.create_subscription(PathMsg, self.global_path_topic, self.path_callback, latched_qos)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.local_path_pub = self.create_publisher(PathMsg, self.local_path_topic, latched_qos)
        self.speed_limit_pub = self.create_publisher(Float32, self.speed_limit_topic, 10)
        self.timer = self.create_timer(float(self.get_parameter("publish_period_s").value), self.publish_local_plan)
        self.get_logger().info(
            "ST corridor planner started "
            f"(global={self.global_path_topic}, local={self.local_path_topic}, scan={self.scan_topic}, "
            f"static_obstacles={len(self.static_obstacles)})."
        )

    def _load_static_obstacles(self, manifest_path: str) -> list[StaticSquareObstacle]:
        if not manifest_path:
            return []
        path = Path(manifest_path).expanduser()
        if not path.exists():
            self.get_logger().warning(f"Static obstacle manifest does not exist: {path}")
            return []
        try:
            import json

            data = json.loads(path.read_text(encoding="utf-8"))
            obstacles = []
            for item in data.get("obstacles", []):
                obstacles.append(
                    StaticSquareObstacle(
                        center_xy=np.array([float(item["x"]), float(item["y"])], dtype=float),
                        size_m=float(item.get("size_m", 0.5)),
                    )
                )
            return obstacles
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().warning(f"Failed to load static obstacle manifest {path}: {exc}")
            return []

    def _open_log(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.log_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "time", "mode", "obstacle_count", "chosen_offset", "speed_limit",
            "min_obstacle_clearance", "candidate_count", "collision_free_count",
        ])
        self.csv_file.flush()

    def destroy_node(self) -> bool:
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
        return super().destroy_node()

    def path_callback(self, msg: PathMsg) -> None:
        points = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses], dtype=float)
        if len(points) < 3:
            self.get_logger().warning("Ignoring global path with fewer than 3 points.")
            return
        self.reference = ReferencePath(points, closed_loop=True)
        self.frame_id = msg.header.frame_id or self.frame_id
        self.get_logger().info(f"Loaded global reference path with {len(points)} poses.")

    def odom_callback(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z),
        )
        self.odom_pose = (np.array([pose.position.x, pose.position.y], dtype=float), yaw)

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _scan_clusters_in_map(self) -> list[np.ndarray]:
        if self.latest_scan is None or self.odom_pose is None:
            return []
        position, yaw = self.odom_pose
        ranges = np.asarray(self.latest_scan.ranges, dtype=float)
        angles = self.latest_scan.angle_min + np.arange(len(ranges), dtype=float) * self.latest_scan.angle_increment
        valid = np.isfinite(ranges)
        valid &= ranges >= max(0.02, float(self.latest_scan.range_min))
        valid &= ranges <= min(self.max_obstacle_range_m, float(self.latest_scan.range_max))
        valid &= np.abs(angles) <= 0.5 * self.front_fov_rad
        if not np.any(valid):
            return []

        local = np.column_stack([
            self.scan_x_offset_m + ranges[valid] * np.cos(angles[valid]),
            ranges[valid] * np.sin(angles[valid]),
        ])
        rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]], dtype=float)
        points = position + local @ rot.T

        clusters: list[list[np.ndarray]] = []
        current: list[np.ndarray] = []
        previous: np.ndarray | None = None
        for point in points:
            if previous is None or float(np.linalg.norm(point - previous)) <= self.cluster_distance_m:
                current.append(point)
            else:
                if len(current) >= self.min_cluster_points:
                    clusters.append(current)
                current = [point]
            previous = point
        if len(current) >= self.min_cluster_points:
            clusters.append(current)
        return [np.asarray(cluster, dtype=float) for cluster in clusters]

    def _obstacles(self, current_s: float, now_sec: float) -> list[CorridorObstacle]:
        assert self.reference is not None
        takeover_distance = (
            self.takeover_distance_m
            if self.takeover_distance_m > 1e-6
            else min(self.lookahead_m, max(2.0, 1.2 + 1.1 * self.target_speed))
        )
        min_obstacle_clearance = self.obstacle_wall_margin + 0.5 * self.vehicle_width
        raw_obstacles: list[CorridorObstacle] = []
        for static_obstacle in self.static_obstacles:
            projection = self.reference.project(static_obstacle.center_xy)
            delta_s = self.reference.signed_delta_s(projection.s, current_s)
            if delta_s < -0.5 or delta_s > takeover_distance:
                continue
            # The path candidate is evaluated in Frenet (s, d), so use a lateral
            # square half-width instead of the Euclidean half-diagonal. The
            # rendered/geometric validator checks the true square footprint.
            raw_obstacles.append(
                CorridorObstacle(
                    centroid_xy=static_obstacle.center_xy,
                    s=projection.s,
                    d=projection.d,
                    radius=0.5 * static_obstacle.size_m + 0.5 * self.vehicle_width + self.inflation_margin,
                    vs=0.0,
                    vd=0.0,
                    points=4,
                )
            )
        for cluster in self._scan_clusters_in_map():
            centroid = np.mean(cluster, axis=0)
            if any(float(np.linalg.norm(centroid - obstacle.center_xy)) < 0.65 for obstacle in self.static_obstacles):
                continue
            if not self.track_filter.inside_drivable_area(centroid, min_obstacle_clearance):
                continue
            projection = self.reference.project(centroid)
            delta_s = self.reference.signed_delta_s(projection.s, current_s)
            if delta_s < -0.5 or delta_s > takeover_distance:
                continue
            radius = float(np.max(np.linalg.norm(cluster - centroid, axis=1)))
            if radius > self.max_cluster_radius_m:
                continue
            raw_obstacles.append(
                CorridorObstacle(
                    centroid_xy=centroid,
                    s=projection.s,
                    d=projection.d,
                    radius=radius + 0.5 * self.vehicle_width + self.inflation_margin,
                    vs=0.0,
                    vd=0.0,
                    points=len(cluster),
                )
            )

        dt = 0.0 if self.previous_obstacle_time is None else max(0.0, now_sec - self.previous_obstacle_time)
        tracked: list[CorridorObstacle] = []
        for obstacle in raw_obstacles:
            vs = 0.0
            vd = 0.0
            if dt > 1e-3 and self.previous_obstacles:
                nearest = min(
                    self.previous_obstacles,
                    key=lambda prev: float(np.linalg.norm(prev.centroid_xy - obstacle.centroid_xy)),
                )
                if float(np.linalg.norm(nearest.centroid_xy - obstacle.centroid_xy)) < 0.6:
                    vs = self.reference.signed_delta_s(obstacle.s, nearest.s) / dt
                    vd = (obstacle.d - nearest.d) / dt
            tracked.append(
                CorridorObstacle(obstacle.centroid_xy, obstacle.s, obstacle.d, obstacle.radius, vs, vd, obstacle.points)
            )
        self.previous_obstacles = tracked
        self.previous_obstacle_time = now_sec
        return tracked

    def _candidate_offsets(self) -> np.ndarray:
        step = max(0.05, self.lateral_offset_step_m)
        offsets = np.arange(-self.max_lateral_offset_m, self.max_lateral_offset_m + 0.5 * step, step)
        return np.unique(np.round(np.append(offsets, 0.0), 3))

    def _evaluate_candidate(
        self,
        current_s: float,
        current_d: float,
        target_d: float,
        obstacles: list[CorridorObstacle],
    ) -> tuple[float, list[np.ndarray], float, bool]:
        assert self.reference is not None
        samples = np.arange(0.0, self.lookahead_m + 0.5 * self.sample_ds_m, self.sample_ds_m)
        path_points: list[np.ndarray] = []
        min_obstacle_clearance = float("inf")
        collision = False
        for ds in samples:
            progress = 0.0 if self.lookahead_m <= 1e-9 else float(np.clip(ds / self.lookahead_m, 0.0, 1.0))
            if obstacles and abs(target_d - current_d) > 1e-3:
                nearest_obstacle_ds = min(
                    max(0.0, self.reference.signed_delta_s(obstacle.s, current_s))
                    for obstacle in obstacles
                )
                shift_distance = max(0.8, min(0.35 * self.lookahead_m, nearest_obstacle_ds - 0.7))
                early_progress = float(np.clip(ds / shift_distance, 0.0, 1.0))
                blend = early_progress * early_progress * (3.0 - 2.0 * early_progress)
            else:
                blend = 1.0 - (1.0 - progress) ** 2
            d_value = (1.0 - blend) * current_d + blend * target_d
            xy, _ = self.reference.interpolate(current_s + ds, d_value)
            if not self.track_filter.inside_drivable_area(xy, self.min_border_clearance):
                collision = True
                min_obstacle_clearance = min(min_obstacle_clearance, 0.0)
            for obstacle in obstacles:
                obstacle_ds = self.reference.signed_delta_s(obstacle.s, current_s)
                travel_time = ds / max(self.min_speed, min(self.target_speed, self.avoidance_max_speed))
                if travel_time > self.dynamic_prediction_time:
                    travel_time = self.dynamic_prediction_time
                predicted_ds = obstacle_ds + obstacle.vs * travel_time
                predicted_d = obstacle.d + obstacle.vd * travel_time
                clearance = math.hypot(ds - predicted_ds, d_value - predicted_d) - obstacle.radius
                min_obstacle_clearance = min(min_obstacle_clearance, clearance)
                if clearance < 0.0:
                    collision = True
            path_points.append(xy)
        smoothness_cost = 2.0 * abs(target_d - current_d) + 1.2 * abs(target_d)
        obstacle_cost = 0.0
        if obstacles:
            obstacle_cost = 8.0 * max(0.0, self.desired_obstacle_clearance - min_obstacle_clearance)
        score = smoothness_cost + obstacle_cost
        return score, path_points, min_obstacle_clearance, collision

    def _choose_path(self, current_projection: Projection, obstacles: list[CorridorObstacle]) -> tuple[str, float, list[np.ndarray], float, int, int]:
        if not obstacles:
            score, points, clearance, collision = self._evaluate_candidate(
                current_projection.s,
                current_projection.d,
                0.0,
                obstacles,
            )
            if not collision:
                return "pass_through", 0.0, points, clearance, 1, 1

        best: tuple[float, float, list[np.ndarray], float] | None = None
        best_any: tuple[float, float, list[np.ndarray], float] | None = None
        candidate_count = 0
        free_count = 0
        for target_d in self._candidate_offsets():
            candidate_count += 1
            score, points, clearance, collision = self._evaluate_candidate(
                current_projection.s,
                current_projection.d,
                float(target_d),
                obstacles,
            )
            risk_score = score + (100.0 * max(0.0, -clearance))
            any_record = (risk_score, float(target_d), points, clearance)
            if best_any is None or any_record[0] < best_any[0]:
                best_any = any_record
            if collision:
                continue
            free_count += 1
            record = (score, float(target_d), points, clearance)
            if best is None or record[0] < best[0]:
                best = record
        if best is None:
            assert best_any is not None
            mode = "avoid" if obstacles and abs(best_any[1]) > 1e-3 else "pass_through"
            return mode, best_any[1], best_any[2], best_any[3], candidate_count, free_count
        mode = "avoid" if obstacles and abs(best[1]) > 1e-3 else "pass_through"
        return mode, best[1], best[2], best[3], candidate_count, free_count

    def _publish_path(self, points: list[np.ndarray], stamp) -> None:
        msg = PathMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        for idx, point in enumerate(points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            if idx + 1 < len(points):
                delta = points[idx + 1] - point
            elif idx > 0:
                delta = point - points[idx - 1]
            else:
                delta = np.array([1.0, 0.0])
            yaw = math.atan2(float(delta[1]), float(delta[0]))
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            msg.poses.append(pose)
        self.local_path_pub.publish(msg)

    def _speed_limit(self, mode: str, clearance: float) -> float:
        if mode == "blocked":
            return 0.0
        speed = self.target_speed if mode == "pass_through" else min(self.target_speed, self.avoidance_max_speed)
        if mode == "avoid" and math.isfinite(clearance) and clearance < -0.25:
            speed = min(speed, max(self.min_speed, 0.65 * self.avoidance_max_speed))
        return float(max(0.0, speed))

    def _write_log(self, now_sec: float, mode: str, obstacles: list[CorridorObstacle], offset: float, speed: float, clearance: float, candidate_count: int, free_count: int) -> None:
        if self.csv_writer is None or self.csv_file is None:
            return
        self.csv_writer.writerow([
            f"{now_sec:.6f}", mode, len(obstacles), f"{offset:.3f}", f"{speed:.3f}",
            f"{clearance:.3f}" if math.isfinite(clearance) else "inf", candidate_count, free_count,
        ])
        self.csv_file.flush()

    def publish_local_plan(self) -> None:
        if self.reference is None or self.odom_pose is None:
            return
        now = self.get_clock().now()
        now_msg = now.to_msg()
        now_sec = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9
        position, _ = self.odom_pose
        projection = self.reference.project(position)
        obstacles = self._obstacles(projection.s, now_sec)
        mode, offset, points, clearance, candidate_count, free_count = self._choose_path(projection, obstacles)
        speed = self._speed_limit(mode, clearance)
        self._publish_path(points, now_msg)
        self.speed_limit_pub.publish(Float32(data=speed))
        self._write_log(now_sec, mode, obstacles, offset, speed, clearance, candidate_count, free_count)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StCorridorPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

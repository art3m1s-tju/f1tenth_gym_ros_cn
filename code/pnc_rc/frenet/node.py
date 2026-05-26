"""ROS 2 node for Frenet static obstacle avoidance."""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from pnc_rc.frenet.planner import (
    FrenetPlanStats,
    FrenetPlannerConfig,
    LocalGridConfig,
    ReferencePath,
    build_occupancy_grid,
    initial_frenet_state,
    local_static_map_occupancy,
    plan_frenet_path,
    polyline_length,
    sample_reference_segment,
    speed_based_activation_lookahead,
    trim_path_to_position,
)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class FrenetStaticObstaclePlanner(Node):
    """Publish a local Frenet path that avoids inflated LaserScan obstacles."""

    def __init__(self) -> None:
        super().__init__("frenet_static_obstacle_planner")

        self.declare_parameter("global_path_topic", "/global_trajectory")
        self.declare_parameter("local_path_topic", "/local_trajectory")
        self.declare_parameter("speed_limit_topic", "/local_trajectory_speed_limit")
        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("target_speed", 1.5)
        self.declare_parameter("d_min", -1.0)
        self.declare_parameter("d_max", 1.0)
        self.declare_parameter("d_step", 0.1)
        self.declare_parameter("t_min", 2.0)
        self.declare_parameter("t_max", 3.0)
        self.declare_parameter("t_step", 0.5)
        self.declare_parameter("v_min", 0.6)
        self.declare_parameter("v_max", 2.5)
        self.declare_parameter("v_step", 0.3)
        self.declare_parameter("trajectory_dt", 0.1)
        self.declare_parameter("max_curvature", 1.1)
        self.declare_parameter("safe_clearance_m", 0.35)
        self.declare_parameter("min_clearance_m", 0.05)
        self.declare_parameter("corridor_radius_m", 0.14)
        self.declare_parameter("corridor_sample_step_m", 0.10)
        self.declare_parameter("path_collision_sample_step_m", 0.05)
        self.declare_parameter("footprint_front_m", 0.38)
        self.declare_parameter("footprint_rear_m", 0.05)
        self.declare_parameter("max_heading_jump", 0.65)
        self.declare_parameter("min_progress_step_m", 0.20)
        self.declare_parameter("grid_forward_m", 7.0)
        self.declare_parameter("grid_rear_m", 1.0)
        self.declare_parameter("grid_half_width_m", 3.0)
        self.declare_parameter("grid_resolution_m", 0.05)
        self.declare_parameter("grid_inflation_radius_m", 0.28)
        self.declare_parameter("scan_offset_x_m", 0.275)
        self.declare_parameter("debug_marker_topic", "/frenet/debug/candidates")
        self.declare_parameter("debug_max_safe_candidates", 12)
        self.declare_parameter("reuse_last_candidate_timeout_s", 1.0)
        self.declare_parameter("projection_search_window_m", 6.0)
        self.declare_parameter("stop_path_length_m", 0.25)
        self.declare_parameter("min_published_path_length_m", 0.75)
        self.declare_parameter("published_path_lookahead_m", 0.25)
        self.declare_parameter("min_path_publish_interval_s", 0.25)
        self.declare_parameter("path_republish_distance_m", 0.50)
        self.declare_parameter("path_republish_min_remaining_m", 2.0)
        self.declare_parameter("centerline_return_lookahead_m", 5.0)
        self.declare_parameter("centerline_return_step_m", 0.08)
        self.declare_parameter("centerline_threat_corridor_radius_m", 0.32)
        self.declare_parameter("centerline_threat_lookahead_m", 6.0)
        self.declare_parameter("reference_closed_loop", True)
        self.declare_parameter("cruise_speed_mps", 1.0)
        self.declare_parameter("activation_min_lookahead_m", 3.0)
        self.declare_parameter("activation_max_lookahead_m", 8.0)
        self.declare_parameter("activation_base_lookahead_m", 2.2)
        self.declare_parameter("activation_reaction_time_s", 1.0)
        self.declare_parameter("activation_decel_mps2", 2.0)
        self.declare_parameter("max_observed_speed_mps", 0.0)
        self.declare_parameter("centerline_speed_limit_mps", -1.0)
        self.declare_parameter("avoidance_speed_limit_mps", 0.70)
        self.declare_parameter("stop_speed_limit_mps", 0.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.planner_config = FrenetPlannerConfig(
            d_min=float(self.get_parameter("d_min").value),
            d_max=float(self.get_parameter("d_max").value),
            d_step=float(self.get_parameter("d_step").value),
            t_min=float(self.get_parameter("t_min").value),
            t_max=float(self.get_parameter("t_max").value),
            t_step=float(self.get_parameter("t_step").value),
            v_min=float(self.get_parameter("v_min").value),
            v_max=float(self.get_parameter("v_max").value),
            v_step=float(self.get_parameter("v_step").value),
            trajectory_dt=float(self.get_parameter("trajectory_dt").value),
            target_speed=float(self.get_parameter("target_speed").value),
            max_curvature=float(self.get_parameter("max_curvature").value),
            safe_clearance=float(self.get_parameter("safe_clearance_m").value),
            min_clearance_m=float(self.get_parameter("min_clearance_m").value),
            corridor_radius_m=float(self.get_parameter("corridor_radius_m").value),
            corridor_sample_step_m=float(
                self.get_parameter("corridor_sample_step_m").value
            ),
            path_collision_sample_step_m=float(
                self.get_parameter("path_collision_sample_step_m").value
            ),
            footprint_front_m=float(self.get_parameter("footprint_front_m").value),
            footprint_rear_m=float(self.get_parameter("footprint_rear_m").value),
            max_heading_jump=float(self.get_parameter("max_heading_jump").value),
            min_progress_step_m=float(self.get_parameter("min_progress_step_m").value),
        )
        self.grid_config = LocalGridConfig(
            forward_m=float(self.get_parameter("grid_forward_m").value),
            rear_m=float(self.get_parameter("grid_rear_m").value),
            half_width_m=float(self.get_parameter("grid_half_width_m").value),
            resolution_m=float(self.get_parameter("grid_resolution_m").value),
            inflation_radius_m=float(self.get_parameter("grid_inflation_radius_m").value),
            scan_offset_x_m=float(self.get_parameter("scan_offset_x_m").value),
        )

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos_profile = QoSProfile(depth=1)
        map_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(
            PathMsg,
            str(self.get_parameter("local_path_topic").value),
            qos_profile,
        )
        self.speed_limit_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("speed_limit_topic").value),
            qos_profile,
        )
        self.debug_marker_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("debug_marker_topic").value),
            10,
        )
        self.global_path_sub = self.create_subscription(
            PathMsg,
            str(self.get_parameter("global_path_topic").value),
            self.global_path_callback,
            qos_profile,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            10,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self.scan_callback,
            10,
        )
        self.map_sub = self.create_subscription(
            OccupancyGridMsg,
            str(self.get_parameter("map_topic").value),
            self.map_callback,
            map_qos_profile,
        )

        self.reference: ReferencePath | None = None
        self.latest_odom: Odometry | None = None
        self.latest_scan: LaserScan | None = None
        self.map_occupied: np.ndarray | None = None
        self.map_resolution: float | None = None
        self.map_origin_xy: tuple[float, float] | None = None
        self.map_logged = False
        self.previous_s_dot: float | None = None
        self.previous_plan_time: float | None = None
        self.last_no_candidate_log_time = 0.0
        self.debug_max_safe_candidates = max(
            1,
            int(self.get_parameter("debug_max_safe_candidates").value),
        )
        self.reuse_last_candidate_timeout_s = max(
            0.0,
            float(self.get_parameter("reuse_last_candidate_timeout_s").value),
        )
        self.projection_search_window_m = max(
            0.5,
            float(self.get_parameter("projection_search_window_m").value),
        )
        self.stop_path_length_m = max(
            0.05,
            float(self.get_parameter("stop_path_length_m").value),
        )
        self.min_published_path_length_m = max(
            0.0,
            float(self.get_parameter("min_published_path_length_m").value),
        )
        self.published_path_lookahead_m = max(
            0.0,
            float(self.get_parameter("published_path_lookahead_m").value),
        )
        self.min_path_publish_interval_s = max(
            0.0,
            float(self.get_parameter("min_path_publish_interval_s").value),
        )
        self.path_republish_distance_m = max(
            0.0,
            float(self.get_parameter("path_republish_distance_m").value),
        )
        self.path_republish_min_remaining_m = max(
            0.0,
            float(self.get_parameter("path_republish_min_remaining_m").value),
        )
        self.centerline_return_lookahead_m = max(
            0.5,
            float(self.get_parameter("centerline_return_lookahead_m").value),
        )
        self.centerline_return_step_m = max(
            0.02,
            float(self.get_parameter("centerline_return_step_m").value),
        )
        self.centerline_threat_corridor_radius_m = max(
            0.0,
            float(self.get_parameter("centerline_threat_corridor_radius_m").value),
        )
        self.centerline_threat_lookahead_m = max(
            0.5,
            float(self.get_parameter("centerline_threat_lookahead_m").value),
        )
        self.cruise_speed_mps = max(
            0.0,
            float(self.get_parameter("cruise_speed_mps").value),
        )
        self.activation_min_lookahead_m = max(
            0.0,
            float(self.get_parameter("activation_min_lookahead_m").value),
        )
        self.activation_max_lookahead_m = max(
            self.activation_min_lookahead_m,
            float(self.get_parameter("activation_max_lookahead_m").value),
        )
        self.activation_base_lookahead_m = max(
            0.0,
            float(self.get_parameter("activation_base_lookahead_m").value),
        )
        self.activation_reaction_time_s = max(
            0.0,
            float(self.get_parameter("activation_reaction_time_s").value),
        )
        self.activation_decel_mps2 = max(
            1e-6,
            float(self.get_parameter("activation_decel_mps2").value),
        )
        max_observed_speed = float(self.get_parameter("max_observed_speed_mps").value)
        if max_observed_speed <= 0.0:
            max_observed_speed = max(3.0, 1.5 * self.cruise_speed_mps + 0.75)
        self.max_observed_speed_mps = max(0.5, max_observed_speed)
        self.centerline_speed_limit_mps = float(
            self.get_parameter("centerline_speed_limit_mps").value
        )
        self.avoidance_speed_limit_mps = float(
            self.get_parameter("avoidance_speed_limit_mps").value
        )
        self.stop_speed_limit_mps = float(
            self.get_parameter("stop_speed_limit_mps").value
        )
        publish_rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / publish_rate, self.plan_once)
        self.last_safe_candidate_xy: np.ndarray | None = None
        self.last_safe_candidate_time: float | None = None
        self.last_reuse_log_time = 0.0
        self.previous_s: float | None = None
        self.reference_closed_loop = bool(self.get_parameter("reference_closed_loop").value)
        self.last_plan_elapsed_sec = 0.0
        self.last_plan_timing_log_time = 0.0
        self.last_path_drop_log_time = 0.0
        self.last_mode_log_time = 0.0
        self.last_far_threat_log_time = 0.0
        self.last_velocity_filter_log_time = 0.0
        self.current_mode = "centerline"
        self.last_published_path_xy: np.ndarray | None = None
        self.last_published_path_time: float | None = None
        self.last_published_path_mode: str | None = None
        self.last_debug_candidates = []
        self.last_debug_best_candidate = None
        self.previous_odom_position: np.ndarray | None = None
        self.previous_odom_time: float | None = None
        self.previous_odom_wall_time: float | None = None
        self.previous_velocity_xy = np.zeros(2, dtype=float)
        self.get_logger().info(
            "Frenet static obstacle planner started "
            f"(local_path={self.path_pub.topic_name}, "
            f"speed_limit_topic={self.speed_limit_pub.topic_name}, "
            f"debug_markers={self.debug_marker_pub.topic_name}, "
            f"rate={publish_rate:.1f}Hz)."
        )

    def global_path_callback(self, msg: PathMsg) -> None:
        points = np.array(
            [[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses],
            dtype=float,
        )
        if len(points) < 3:
            self.get_logger().warning("Ignoring global path with fewer than 3 points.")
            return
        self.reference = ReferencePath.from_points(
            points,
            closed_loop=self.reference_closed_loop,
        )
        self.frame_id = msg.header.frame_id or self.frame_id

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def map_callback(self, msg: OccupancyGridMsg) -> None:
        data = np.asarray(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )
        self.map_occupied = np.flipud(data >= 50)
        self.map_resolution = float(msg.info.resolution)
        self.map_origin_xy = (
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
        )
        if not self.map_logged:
            self.map_logged = True
            occupied_cells = int(np.count_nonzero(self.map_occupied))
            self.get_logger().info(
                "Loaded static occupancy map "
                f"({msg.info.width}x{msg.info.height}, "
                f"resolution={self.map_resolution:.3f}m, "
                f"occupied_cells={occupied_cells})."
            )

    def plan_once(self) -> None:
        if self.reference is None or self.latest_odom is None:
            return
        plan_start = time.perf_counter()
        stamp = self.get_clock().now().to_msg()
        position, yaw, velocity_xy = self._odom_state(self.latest_odom)
        now = time.monotonic()
        dt = None if self.previous_plan_time is None else now - self.previous_plan_time
        state = initial_frenet_state(
            self.reference,
            position,
            yaw,
            velocity_xy,
            self.previous_s,
            self.previous_s_dot,
            dt,
            self.projection_search_window_m,
        )
        self.previous_s = state.s
        self.previous_s_dot = state.s_dot
        self.previous_plan_time = now

        if self.latest_scan is None:
            self.publish_debug_markers(stamp, [], None)
            self.publish_stop_path(stamp, position, yaw)
            return

        scan = self.latest_scan
        vehicle_pose = (float(position[0]), float(position[1]), yaw)
        static_occupancy = self.local_map_occupancy(vehicle_pose)
        occupancy = build_occupancy_grid(
            np.asarray(scan.ranges, dtype=float),
            float(scan.angle_min),
            float(scan.angle_increment),
            float(scan.range_min),
            float(scan.range_max),
            self.grid_config,
            static_occupied=static_occupancy,
        )
        activation_lookahead = self._activation_lookahead_m()
        threat_lookahead = max(
            self.centerline_threat_lookahead_m,
            activation_lookahead,
        )
        centerline_threat_distance = self._centerline_threat_distance(
            occupancy,
            vehicle_pose,
            state.s,
            threat_lookahead,
        )
        if centerline_threat_distance is None:
            self.current_mode = "centerline"
            self.last_safe_candidate_xy = None
            self.last_safe_candidate_time = None
            self._publish_centerline_path(stamp, position, yaw, state.s)
            self._log_mode(
                now,
                "Publishing centerline return path; no obstacle threat ahead.",
            )
            return

        if centerline_threat_distance > activation_lookahead:
            self.current_mode = "centerline"
            self.last_safe_candidate_xy = None
            self.last_safe_candidate_time = None
            self._publish_centerline_path(stamp, position, yaw, state.s)
            self._log_far_threat(
                now,
                centerline_threat_distance,
                activation_lookahead,
            )
            return

        if self._publish_held_path_if_safe(stamp, now, occupancy, vehicle_pose):
            self.publish_debug_markers(stamp, [], None, keep_previous=True)
            return

        self.current_mode = "frenet"
        stats = FrenetPlanStats()
        debug_candidates = []
        candidate = plan_frenet_path(
            self.reference,
            state,
            occupancy,
            vehicle_pose,
            self.planner_config,
            stats,
            debug_candidates=debug_candidates,
        )
        publish_position, publish_yaw = self._odom_pose(self.latest_odom)
        fresh_s, _, _, _ = self.reference.project_near(
            publish_position,
            state.s,
            self.projection_search_window_m,
        )
        self.previous_s = fresh_s
        if candidate is None:
            self.last_plan_elapsed_sec = time.perf_counter() - plan_start
            self._log_plan_timing(now, stats, self.last_plan_elapsed_sec)
            if self._reuse_last_candidate_if_fresh(stamp, now, occupancy, vehicle_pose):
                self.publish_debug_markers(stamp, [], None, keep_previous=True)
                return
            self.log_no_candidate(
                stats,
                occupancy,
                static_occupancy is not None,
                state,
                velocity_xy,
                scan,
            )
            self.publish_debug_markers(stamp, [], None, keep_previous=True)
            self.publish_stop_path(stamp, publish_position, publish_yaw)
            return
        self.last_plan_elapsed_sec = time.perf_counter() - plan_start
        self._log_plan_timing(now, stats, self.last_plan_elapsed_sec)
        anchored_candidate = trim_path_to_position(
            candidate.xy,
            publish_position,
            self.min_published_path_length_m,
            self.published_path_lookahead_m,
        )
        if len(anchored_candidate) < 2:
            if now - self.last_path_drop_log_time >= 0.5:
                self.last_path_drop_log_time = now
                self.get_logger().warning(
                    "Discarding Frenet candidate because the anchored path is too short "
                    f"(plan_time={self.last_plan_elapsed_sec:.3f}s)."
                )
            self.publish_debug_markers(stamp, debug_candidates, candidate)
            self.publish_stop_path(stamp, publish_position, publish_yaw)
            return
        self.last_safe_candidate_xy = np.asarray(candidate.xy, dtype=float).copy()
        self.last_safe_candidate_time = now
        self.publish_debug_markers(stamp, debug_candidates, candidate)
        self.publish_path(stamp, anchored_candidate, mode="avoidance", force=True)

    def _centerline_threat_distance(
        self,
        occupancy,
        vehicle_pose: tuple[float, float, float],
        start_s: float,
        lookahead_m: float,
    ) -> float | None:
        if self.reference is None:
            return 0.0
        centerline = sample_reference_segment(
            self.reference,
            start_s,
            lookahead_m,
            self.centerline_return_step_m,
            d=0.0,
        )
        collision, min_clearance = occupancy.query_path(
            centerline,
            vehicle_pose,
            self.centerline_threat_corridor_radius_m,
            self.planner_config.corridor_sample_step_m,
            self.planner_config.footprint_front_m,
            self.planner_config.footprint_rear_m,
            self.planner_config.path_collision_sample_step_m,
        )
        if not collision and min_clearance >= self.planner_config.min_clearance_m:
            return None

        if len(centerline) < 2:
            return 0.0
        cumulative = np.concatenate(
            [
                np.array([0.0], dtype=float),
                np.cumsum(np.linalg.norm(np.diff(centerline, axis=0), axis=1)),
            ]
        )
        stride = max(1, int(math.ceil(0.25 / self.centerline_return_step_m)))
        for end_idx in range(1, len(centerline), stride):
            prefix = centerline[: end_idx + 1]
            prefix_collision, prefix_clearance = occupancy.query_path(
                prefix,
                vehicle_pose,
                self.centerline_threat_corridor_radius_m,
                self.planner_config.corridor_sample_step_m,
                self.planner_config.footprint_front_m,
                self.planner_config.footprint_rear_m,
                self.planner_config.path_collision_sample_step_m,
            )
            if (
                prefix_collision
                or prefix_clearance < self.planner_config.min_clearance_m
            ):
                return float(cumulative[end_idx])
        return float(cumulative[-1])

    def _activation_lookahead_m(self) -> float:
        return speed_based_activation_lookahead(
            self.cruise_speed_mps,
            base_lookahead_m=self.activation_base_lookahead_m,
            reaction_time_s=self.activation_reaction_time_s,
            decel_mps2=self.activation_decel_mps2,
            min_lookahead_m=self.activation_min_lookahead_m,
            max_lookahead_m=self.activation_max_lookahead_m,
        )

    def _publish_centerline_path(
        self,
        stamp,
        position: np.ndarray,
        yaw: float,
        start_s: float,
    ) -> None:
        if self.reference is None:
            self.publish_stop_path(stamp, position, yaw)
            return
        centerline = sample_reference_segment(
            self.reference,
            start_s,
            self.centerline_return_lookahead_m,
            self.centerline_return_step_m,
            d=0.0,
        )
        anchored = trim_path_to_position(
            centerline,
            position,
            self.min_published_path_length_m,
            self.published_path_lookahead_m,
        )
        if len(anchored) < 2:
            self.publish_stop_path(stamp, position, yaw)
            return
        self.publish_debug_markers(stamp, [], None)
        self.publish_path(stamp, anchored, mode="centerline")

    def _publish_held_path_if_safe(
        self,
        stamp,
        now: float,
        occupancy,
        vehicle_pose: tuple[float, float, float],
    ) -> bool:
        if (
            self.last_safe_candidate_xy is None
            or self.last_safe_candidate_time is None
        ):
            return False

        collision, min_clearance = occupancy.query_path(
            self.last_safe_candidate_xy,
            vehicle_pose,
            self.planner_config.corridor_radius_m,
            self.planner_config.corridor_sample_step_m,
            self.planner_config.footprint_front_m,
            self.planner_config.footprint_rear_m,
            self.planner_config.path_collision_sample_step_m,
        )
        if collision or min_clearance < self.planner_config.min_clearance_m:
            return False

        if self.latest_odom is not None:
            publish_position, _ = self._odom_pose(self.latest_odom)
        else:
            publish_position = np.array([vehicle_pose[0], vehicle_pose[1]], dtype=float)
        anchored = trim_path_to_position(
            self.last_safe_candidate_xy,
            publish_position,
            self.min_published_path_length_m,
            self.published_path_lookahead_m,
        )
        if len(anchored) < 2:
            return False

        self.current_mode = "hold"
        self._log_mode(
            now,
            f"Holding safe Frenet path (clearance={min_clearance:.2f}m).",
        )
        self.publish_path(stamp, anchored, mode="avoidance")
        return True

    def log_no_candidate(
        self,
        stats: FrenetPlanStats,
        occupancy,
        static_map_ready: bool,
        state,
        velocity_xy: np.ndarray,
        scan: LaserScan,
    ) -> None:
        now = time.monotonic()
        if now - self.last_no_candidate_log_time < 0.5:
            return
        self.last_no_candidate_log_time = now
        occupied_cells = int(np.count_nonzero(occupancy.occupied))
        scan_ranges = np.asarray(scan.ranges, dtype=float)
        finite_mask = np.isfinite(scan_ranges)
        zero_like_count = int(np.count_nonzero(finite_mask & (scan_ranges < 1e-3)))
        finite_count = int(np.count_nonzero(finite_mask))
        speed_mps = float(np.linalg.norm(velocity_xy))
        self.get_logger().warning(
            "No safe Frenet candidate; publishing stop path "
            f"(total={stats.total_candidates}, safe={stats.safe_candidates}, "
            f"progress_reject={stats.progress_rejections}, "
            f"heading_reject={stats.heading_rejections}, "
            f"curvature_reject={stats.curvature_rejections}, "
            f"clearance_reject={stats.clearance_rejections}, "
            f"collision_reject={stats.collision_rejections}, "
            f"best_clearance={stats.best_clearance_m:.2f}m, "
            f"occupied_cells={occupied_cells}, static_map_ready={static_map_ready}, "
            f"s={state.s:.2f}, d={state.d:.2f}, s_dot={state.s_dot:.2f}, "
            f"s_ddot={state.s_ddot:.2f}, d_dot={state.d_dot:.2f}, speed={speed_mps:.2f}, "
            f"scan_zero_like={zero_like_count}/{finite_count})."
        )

    def publish_stop_path(self, stamp, position: np.ndarray, yaw: float) -> None:
        forward = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
        offsets = np.array(
            [0.0, 0.5 * self.stop_path_length_m, self.stop_path_length_m],
            dtype=float,
        )
        points = position.reshape(1, 2) + offsets.reshape(-1, 1) * forward.reshape(1, 2)
        self.publish_path(stamp, points, mode="stop", force=True)

    def publish_path(
        self,
        stamp,
        points: np.ndarray,
        mode: str = "path",
        force: bool = False,
    ) -> bool:
        now = time.monotonic()
        points = np.asarray(points, dtype=float)
        if not force and not self._should_publish_path(now, points, mode):
            self._publish_speed_limit(mode)
            return False
        path_msg = PathMsg()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.frame_id
        for idx, point in enumerate(points):
            prev_point = points[max(0, idx - 1)]
            next_point = points[min(len(points) - 1, idx + 1)]
            yaw = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
            qz, qw = yaw_to_quaternion(yaw)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self._publish_speed_limit(mode)
        self.last_published_path_xy = points.copy()
        self.last_published_path_time = now
        self.last_published_path_mode = mode
        return True

    def _publish_speed_limit(self, mode: str) -> None:
        limit = self._speed_limit_for_mode(mode)
        msg = Float32()
        msg.data = float(limit)
        self.speed_limit_pub.publish(msg)

    def _speed_limit_for_mode(self, mode: str) -> float:
        if mode == "stop":
            return max(0.0, self.stop_speed_limit_mps)
        if mode == "avoidance":
            return self.avoidance_speed_limit_mps
        if mode == "centerline":
            return self.centerline_speed_limit_mps
        return math.nan

    def _should_publish_path(self, now: float, points: np.ndarray, mode: str) -> bool:
        if (
            self.last_published_path_xy is None
            or self.last_published_path_time is None
            or self.last_published_path_mode != mode
        ):
            return True

        if now - self.last_published_path_time < self.min_path_publish_interval_s:
            return False

        if self.latest_odom is not None:
            position, _ = self._odom_pose(self.latest_odom)
            if self._published_path_remaining_length(position) < self.path_republish_min_remaining_m:
                return True

        if self.path_republish_distance_m <= 0.0:
            return True

        endpoint_shift = float(
            np.linalg.norm(points[-1] - self.last_published_path_xy[-1])
        )
        return endpoint_shift >= self.path_republish_distance_m

    def _published_path_remaining_length(self, position: np.ndarray) -> float:
        if self.last_published_path_xy is None:
            return 0.0
        remaining = trim_path_to_position(
            self.last_published_path_xy,
            position,
            min_remaining_length_m=0.0,
        )
        return polyline_length(remaining)

    def publish_debug_markers(
        self,
        stamp,
        candidates,
        best_candidate,
        *,
        keep_previous: bool = False,
    ) -> None:
        if candidates or best_candidate is not None:
            self.last_debug_candidates = list(candidates)
            self.last_debug_best_candidate = best_candidate
        elif keep_previous:
            candidates = self.last_debug_candidates
            best_candidate = self.last_debug_best_candidate
        else:
            self.last_debug_candidates = []
            self.last_debug_best_candidate = None

        marker_array = MarkerArray()
        marker_array.markers.append(self._delete_all_marker(stamp))

        marker_id = 0
        for candidate in candidates[: self.debug_max_safe_candidates]:
            if best_candidate is not None and candidate is best_candidate:
                continue
            marker = self._candidate_marker(
                stamp,
                candidate.xy,
                marker_id,
                is_best=False,
            )
            marker_array.markers.append(marker)
            marker_id += 1

        if best_candidate is not None:
            marker_array.markers.append(
                self._candidate_marker(
                    stamp,
                    best_candidate.xy,
                    10000,
                    is_best=True,
                )
            )

        self.debug_marker_pub.publish(marker_array)

    def _reuse_last_candidate_if_fresh(
        self,
        stamp,
        now: float,
        occupancy,
        vehicle_pose: tuple[float, float, float],
    ) -> bool:
        if self.last_safe_candidate_xy is None or self.last_safe_candidate_time is None:
            return False
        age = now - self.last_safe_candidate_time
        if age > self.reuse_last_candidate_timeout_s:
            return False
        collision, min_clearance = occupancy.query_path(
            self.last_safe_candidate_xy,
            vehicle_pose,
            self.planner_config.corridor_radius_m,
            self.planner_config.corridor_sample_step_m,
            self.planner_config.footprint_front_m,
            self.planner_config.footprint_rear_m,
            self.planner_config.path_collision_sample_step_m,
        )
        if collision or min_clearance < self.planner_config.min_clearance_m:
            if now - self.last_reuse_log_time >= 0.5:
                self.last_reuse_log_time = now
                self.get_logger().warning(
                    "Discarding stale Frenet candidate because it is no longer safe "
                    f"(age={age:.2f}s, collision={collision}, "
                    f"clearance={min_clearance:.2f}m)."
                )
            return False
        if self.latest_odom is not None:
            publish_position, publish_yaw = self._odom_pose(self.latest_odom)
        else:
            publish_position = np.array([vehicle_pose[0], vehicle_pose[1]], dtype=float)
            publish_yaw = float(vehicle_pose[2])
        published_path = trim_path_to_position(
            self.last_safe_candidate_xy,
            publish_position,
            self.min_published_path_length_m,
            self.published_path_lookahead_m,
        )
        if len(published_path) < 2:
            if now - self.last_path_drop_log_time >= 0.5:
                self.last_path_drop_log_time = now
                self.get_logger().warning(
                    "Discarding reused Frenet candidate because the anchored path is too short "
                    f"(age={age:.2f}s)."
                )
            return False
        if now - self.last_reuse_log_time >= 0.5:
            self.last_reuse_log_time = now
            self.get_logger().warning(
                "No safe Frenet candidate in current cycle; reusing last safe path "
                f"(age={age:.2f}s, clearance={min_clearance:.2f}m)."
            )
        self.publish_path(stamp, published_path, mode="avoidance")
        return True

    def _delete_all_marker(self, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def _candidate_marker(self, stamp, points: np.ndarray, marker_id: int, *, is_best: bool) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "frenet_best_candidate" if is_best else "frenet_candidate_set"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.085 if is_best else 0.03
        marker.color.a = 1.0 if is_best else 0.42
        marker.color.r = 1.0 if is_best else 0.05
        marker.color.g = 0.08 if is_best else 0.65
        marker.color.b = 0.08 if is_best else 1.0
        for x_value, y_value in points:
            point = Point()
            point.x = float(x_value)
            point.y = float(y_value)
            point.z = 0.09 if is_best else 0.035
            marker.points.append(point)
        return marker

    def local_map_occupancy(self, vehicle_pose: tuple[float, float, float]) -> np.ndarray | None:
        if (
            self.map_occupied is None
            or self.map_resolution is None
            or self.map_origin_xy is None
        ):
            return None
        return local_static_map_occupancy(
            self.map_occupied,
            self.map_resolution,
            self.map_origin_xy,
            vehicle_pose,
            self.grid_config,
        )

    def _odom_pose(self, msg: Odometry) -> tuple[np.ndarray, float]:
        pose = msg.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        position = np.array([pose.position.x, pose.position.y], dtype=float)
        return position, yaw

    def _odom_state(self, msg: Odometry) -> tuple[np.ndarray, float, np.ndarray]:
        pose = msg.pose.pose
        twist = msg.twist.twist
        position, yaw = self._odom_pose(msg)
        velocity_xy = self.previous_velocity_xy.copy()
        twist_velocity_xy = np.array(
            [
                math.cos(yaw) * float(twist.linear.x)
                - math.sin(yaw) * float(twist.linear.y),
                math.sin(yaw) * float(twist.linear.x)
                + math.cos(yaw) * float(twist.linear.y),
            ],
            dtype=float,
        )
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        wall_time = time.monotonic()
        if self.previous_odom_position is not None and self.previous_odom_time is not None:
            dt = stamp_sec - self.previous_odom_time
            if 1e-4 <= dt <= 0.5:
                velocity_xy = (position - self.previous_odom_position) / dt
            elif self.previous_odom_wall_time is not None:
                wall_dt = wall_time - self.previous_odom_wall_time
                displacement = position - self.previous_odom_position
                if 1e-4 <= wall_dt <= 0.5 and np.linalg.norm(displacement) > 1e-5:
                    velocity_xy = displacement / wall_dt
        elif not np.any(velocity_xy):
            velocity_xy = twist_velocity_xy

        speed = float(np.linalg.norm(velocity_xy))
        if speed > self.max_observed_speed_mps:
            twist_speed = float(np.linalg.norm(twist_velocity_xy))
            if math.isfinite(twist_speed) and twist_speed <= self.max_observed_speed_mps:
                velocity_xy = twist_velocity_xy
            elif speed > 1e-9:
                velocity_xy = velocity_xy * (self.max_observed_speed_mps / speed)
            if wall_time - self.last_velocity_filter_log_time >= 0.5:
                self.last_velocity_filter_log_time = wall_time
                self.get_logger().warning(
                    "Filtered implausible Frenet odom speed estimate "
                    f"(raw={speed:.2f}m/s, twist={twist_speed:.2f}m/s, "
                    f"limit={self.max_observed_speed_mps:.2f}m/s)."
                )
        self.previous_velocity_xy = velocity_xy.copy()
        self.previous_odom_position = position.copy()
        self.previous_odom_time = stamp_sec
        self.previous_odom_wall_time = wall_time
        return position, yaw, velocity_xy

    def _log_plan_timing(
        self,
        now: float,
        stats: FrenetPlanStats,
        elapsed_sec: float,
    ) -> None:
        if now - self.last_plan_timing_log_time < 0.5:
            return
        self.last_plan_timing_log_time = now
        self.get_logger().info(
            "Frenet planning cycle "
            f"elapsed={elapsed_sec:.3f}s, total={stats.total_candidates}, "
            f"safe={stats.safe_candidates}, collision_reject={stats.collision_rejections}, "
            f"clearance_reject={stats.clearance_rejections}, heading_reject={stats.heading_rejections}, "
            f"curvature_reject={stats.curvature_rejections}."
        )

    def _log_mode(self, now: float, message: str) -> None:
        if now - self.last_mode_log_time < 1.0:
            return
        self.last_mode_log_time = now
        self.get_logger().info(message)

    def _log_far_threat(
        self,
        now: float,
        threat_distance_m: float,
        activation_lookahead_m: float,
    ) -> None:
        if now - self.last_far_threat_log_time < 1.0:
            return
        self.last_far_threat_log_time = now
        self.get_logger().info(
            "Centerline threat detected but keeping centerline "
            f"(threat_distance={threat_distance_m:.2f}m, "
            f"activation_lookahead={activation_lookahead_m:.2f}m, "
            f"cruise_speed={self.cruise_speed_mps:.2f}m/s)."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FrenetStaticObstaclePlanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

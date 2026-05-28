"""Frenet 静态障碍避障 ROS 2 节点。"""
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
    path_pose_error,
    plan_frenet_path,
    polyline_length,
    sample_reference_segment,
    sample_return_to_centerline_segment,
    select_temporally_consistent_candidate,
    speed_based_activation_lookahead,
    trim_path_to_position,
)
from pnc_rc.frenet.preset import FRENET_NODE_DEFAULTS


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """将 ROS 四元数转换为平面 yaw 角。

    Args:
        x: 四元数 x 分量。
        y: 四元数 y 分量。
        z: 四元数 z 分量。
        w: 四元数 w 分量。

    Returns:
        绕 z 轴的航向角，单位 rad。
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    """将平面 yaw 角转换为 ROS Pose 需要的 z/w 四元数分量。

    Args:
        yaw: 绕 z 轴的航向角，单位 rad。

    Returns:
        `(qz, qw)`，其余 x/y 分量在平面车模型中为 0。
    """
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class FrenetStaticObstaclePlanner(Node):
    """发布用于静态障碍避障的 Frenet 局部轨迹。

    节点订阅全局参考线、里程计、LaserScan 和静态地图，把 LaserScan/地图投影到
    车体局部占据栅格后枚举 Frenet 候选轨迹。输出 `/local_trajectory` 给 LQR
    控制器，同时发布局部速度限制和 RViz candidate marker。
    """

    def __init__(self) -> None:
        """声明参数、初始化状态缓存并创建 ROS 通信接口。

        参数默认值来自 `FRENET_NODE_DEFAULTS`。实际仿真通常会通过
        `pnc_sim_launch.py` 覆盖这些默认值；直接运行 `run_frenet_planner.py`
        时才会使用这里的 fallback。这样可以把“默认值在哪里定义”固定到一个
        表中，避免 node、launch、测试脚本继续各写一套。
        """
        super().__init__("frenet_static_obstacle_planner")

        for name, default_value in FRENET_NODE_DEFAULTS.items():
            self.declare_parameter(name, default_value)

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
            weight_curvature=float(self.get_parameter("weight_curvature").value),
            weight_curvature_rate=float(
                self.get_parameter("weight_curvature_rate").value
            ),
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
            max_initial_s_accel_mps2=float(
                self.get_parameter("max_initial_s_accel_mps2").value
            ),
            max_reliable_initial_s_accel_mps2=float(
                self.get_parameter("max_reliable_initial_s_accel_mps2").value
            ),
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
        self.held_path_replan_clearance_m = max(
            self.planner_config.min_clearance_m,
            float(self.get_parameter("held_path_replan_clearance_m").value),
        )
        self.held_path_min_remaining_m = max(
            0.0,
            float(self.get_parameter("held_path_min_remaining_m").value),
        )
        self.held_path_max_lateral_error_m = max(
            0.0,
            float(self.get_parameter("held_path_max_lateral_error_m").value),
        )
        self.held_path_max_heading_error_rad = max(
            0.0,
            float(self.get_parameter("held_path_max_heading_error_rad").value),
        )
        self.candidate_profile_consistency_weight = max(
            0.0,
            float(self.get_parameter("candidate_profile_consistency_weight").value),
        )
        self.candidate_profile_max_jump_m = max(
            0.0,
            float(self.get_parameter("candidate_profile_max_jump_m").value),
        )
        self.candidate_profile_lookahead_m = max(
            0.0,
            float(self.get_parameter("candidate_profile_lookahead_m").value),
        )
        self.candidate_profile_unlock_clearance_gain_m = max(
            0.0,
            float(
                self.get_parameter(
                    "candidate_profile_unlock_clearance_gain_m"
                ).value
            ),
        )
        self.candidate_channel_memory_timeout_s = max(
            0.0,
            float(self.get_parameter("candidate_channel_memory_timeout_s").value),
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
        self.max_published_path_length_m = max(
            0.0,
            float(self.get_parameter("max_published_path_length_m").value),
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
        self.centerline_return_direct_d_threshold_m = max(
            0.0,
            float(self.get_parameter("centerline_return_direct_d_threshold_m").value),
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
        self.activation_path_margin_m = max(
            0.0,
            float(self.get_parameter("activation_path_margin_m").value),
        )
        self.approach_slowdown_extra_m = float(
            self.get_parameter("approach_slowdown_extra_m").value
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
        self.last_hold_replan_log_time = 0.0
        self.last_candidate_selection_log_time = 0.0
        self.last_velocity_filter_log_time = 0.0
        self.current_mode = "centerline"
        self.last_selected_candidate_s_profile: np.ndarray | None = None
        self.last_selected_candidate_d_profile: np.ndarray | None = None
        self.last_selected_candidate_time: float | None = None
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
        """接收全局参考线并构建 Frenet 投影缓存。

        Args:
            msg: planner 发布的全局 `nav_msgs/Path`。

        Returns:
            None。
        """
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
        """缓存最新里程计消息。

        Args:
            msg: F1TENTH 仿真器输出的 ego odometry。

        Returns:
            None。
        """
        self.latest_odom = msg

    def scan_callback(self, msg: LaserScan) -> None:
        """缓存最新 LaserScan。

        Args:
            msg: 车载激光雷达扫描。

        Returns:
            None。
        """
        self.latest_scan = msg

    def map_callback(self, msg: OccupancyGridMsg) -> None:
        """接收静态地图并转换为布尔占据栅格。

        Args:
            msg: `/map` 发布的 `nav_msgs/OccupancyGrid`。

        Returns:
            None。地图只在首次加载时打印一次统计日志。
        """
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
        """执行一次 Frenet 避障状态机。

        流程:
            1. 从里程计和全局参考线估计当前 Frenet 状态。
            2. 用 LaserScan 与静态地图生成局部占据栅格。
            3. 若上一条 Frenet 路径仍安全且未接近终点，继续执行它。
            4. 沿中心线检测前方威胁距离。
            5. 无威胁时发布中心线；进入预激活区间时只提前限速；
               进入激活区间后才生成 Frenet 候选或 stop path。

        Returns:
            None。该函数通过 ROS topic 发布局部轨迹、速度限制和 RViz marker。
        """
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
        raw_activation_lookahead = self._activation_lookahead_m()
        activation_lookahead = self._effective_activation_lookahead_m(
            raw_activation_lookahead
        )
        slowdown_lookahead = self._slowdown_lookahead_m(activation_lookahead)
        threat_lookahead = max(
            self.centerline_threat_lookahead_m,
            slowdown_lookahead,
        )
        centerline_threat_distance = self._centerline_threat_distance(
            occupancy,
            vehicle_pose,
            state.s,
            threat_lookahead,
        )
        if centerline_threat_distance is None:
            self.current_mode = "centerline"
            self._clear_candidate_memory(clear_channel=False, now=now)
            self._publish_centerline_path(stamp, position, yaw, state.s, state.d)
            self._log_mode(
                now,
                "Publishing centerline return path; no obstacle threat ahead.",
            )
            return

        if centerline_threat_distance > slowdown_lookahead:
            self.current_mode = "centerline"
            self._clear_candidate_memory(clear_channel=False, now=now)
            self._publish_centerline_path(
                stamp,
                position,
                yaw,
                state.s,
                state.d,
                mode=self.current_mode,
            )
            self._log_far_threat(
                now,
                centerline_threat_distance,
                activation_lookahead,
                slowdown_lookahead,
                False,
            )
            return

        preactivation_only = centerline_threat_distance > activation_lookahead
        if preactivation_only:
            self.current_mode = "approach"
            self._clear_candidate_memory(clear_channel=False, now=now)
            self._publish_centerline_path(
                stamp,
                position,
                yaw,
                state.s,
                state.d,
                mode=self.current_mode,
            )
            self._log_far_threat(
                now,
                centerline_threat_distance,
                activation_lookahead,
                slowdown_lookahead,
                True,
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
        fresh_s, fresh_d, _, _ = self.reference.project_near(
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
            self._publish_approach_or_stop(
                stamp,
                publish_position,
                publish_yaw,
                fresh_s,
                fresh_d,
                preactivation_only,
                keep_debug=True,
            )
            return
        self.last_plan_elapsed_sec = time.perf_counter() - plan_start
        self._log_plan_timing(now, stats, self.last_plan_elapsed_sec)
        candidate = self._select_consistent_candidate(candidate, debug_candidates, now)
        anchored_candidate = trim_path_to_position(
            candidate.xy,
            publish_position,
            self.min_published_path_length_m,
            self.published_path_lookahead_m,
            self.max_published_path_length_m,
        )
        if len(anchored_candidate) < 2:
            if now - self.last_path_drop_log_time >= 0.5:
                self.last_path_drop_log_time = now
                self.get_logger().warning(
                    "Discarding Frenet candidate because the anchored path is too short "
                    f"(plan_time={self.last_plan_elapsed_sec:.3f}s)."
                )
            self.publish_debug_markers(stamp, debug_candidates, candidate)
            self._publish_approach_or_stop(
                stamp,
                publish_position,
                publish_yaw,
                fresh_s,
                fresh_d,
                preactivation_only,
                keep_debug=False,
            )
            return
        self.last_safe_candidate_xy = np.asarray(candidate.xy, dtype=float).copy()
        self.last_safe_candidate_time = now
        self.last_selected_candidate_s_profile = np.asarray(candidate.s, dtype=float).copy()
        self.last_selected_candidate_d_profile = np.asarray(candidate.d, dtype=float).copy()
        self.last_selected_candidate_time = now
        self.publish_debug_markers(stamp, debug_candidates, candidate)
        self.publish_path(stamp, anchored_candidate, mode="avoidance", force=True)

    def _clear_candidate_memory(
        self,
        *,
        clear_channel: bool = True,
        now: float | None = None,
    ) -> None:
        """清空与上一条 Frenet 候选相关的状态记忆。

        Args:
            clear_channel: 是否同时清空用于防蛇形的横向通道记忆。中心线短暂安全
                时只应清掉 path cache，不应立刻忘记刚选过的避障通道。
            now: `time.monotonic()` 当前时间。`clear_channel=False` 时用于判断
                通道记忆是否超过 timeout。

        Returns:
            None。
        """
        self.last_safe_candidate_xy = None
        self.last_safe_candidate_time = None
        if (
            not clear_channel
            and now is not None
            and self.last_selected_candidate_time is not None
            and now - self.last_selected_candidate_time
            <= self.candidate_channel_memory_timeout_s
        ):
            return
        self.last_selected_candidate_s_profile = None
        self.last_selected_candidate_d_profile = None
        self.last_selected_candidate_time = None

    def _publish_approach_or_stop(
        self,
        stamp,
        position: np.ndarray,
        yaw: float,
        start_s: float,
        start_d: float,
        preactivation_only: bool,
        *,
        keep_debug: bool,
    ) -> None:
        """按预激活状态发布 approach 中心线或 stop path。

        Args:
            stamp: ROS 时间戳。
            position: 发布轨迹使用的车辆世界坐标。
            yaw: 发布 stop path 时使用的车辆航向，单位 rad。
            start_s: approach 中心线采样起点弧长。
            start_d: 当前车辆相对中心线的横向偏移，单位 m。
            preactivation_only: `True` 表示还没进入必须避障的 activation 区间；
                此时无解只限速并继续中心线，不立即停车。
            keep_debug: 发布前是否保留上一帧 debug marker。

        Returns:
            None。
        """
        if keep_debug:
            self.publish_debug_markers(stamp, [], None, keep_previous=True)
        if preactivation_only:
            self._publish_centerline_path(
                stamp,
                position,
                yaw,
                start_s,
                start_d,
                mode="approach",
            )
            return
        self.publish_stop_path(stamp, position, yaw)

    def _centerline_threat_distance(
        self,
        occupancy,
        vehicle_pose: tuple[float, float, float],
        start_s: float,
        lookahead_m: float,
    ) -> float | None:
        """检测中心线前方最近威胁距离。

        Args:
            occupancy: 当前局部占据栅格。
            vehicle_pose: 车辆世界位姿 `(x, y, yaw)`。
            start_s: 从参考线弧长 `start_s` 开始向前检测。
            lookahead_m: 最大检测距离，单位 m。

        Returns:
            如果中心线通道安全，返回 `None`；否则返回首次碰撞或低 clearance
            出现位置相对当前点的沿线距离，单位 m。
        """
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
        """计算真正切入 Frenet 避障的速度相关距离。

        Returns:
            激活距离，单位 m。速度越高，距离越长。
        """
        return speed_based_activation_lookahead(
            self.cruise_speed_mps,
            base_lookahead_m=self.activation_base_lookahead_m,
            reaction_time_s=self.activation_reaction_time_s,
            decel_mps2=self.activation_decel_mps2,
            min_lookahead_m=self.activation_min_lookahead_m,
            max_lookahead_m=self.activation_max_lookahead_m,
        )

    def _effective_activation_lookahead_m(self, raw_activation_m: float) -> float:
        """把 Frenet 激活距离限制到当前可执行路径长度附近。

        速度公式给的是制动/反应距离，但如果它远大于实际发布给控制器的路径长度，
        Frenet 会在障碍很远时先规划出一条近似中心线的 avoidance path。这里把
        真正规划距离压到“发布路径长度 + 小余量”，让避障发生在障碍进入有效局部
        路径窗口后。

        Args:
            raw_activation_m: 速度公式计算出的原始激活距离，单位 m。

        Returns:
            实际用于状态机的 Frenet 激活距离，单位 m。
        """
        raw_activation = max(0.0, float(raw_activation_m))
        if self.max_published_path_length_m <= 0.0:
            return raw_activation
        path_limited_activation = (
            self.max_published_path_length_m + self.activation_path_margin_m
        )
        lower_bound = max(self.activation_min_lookahead_m, self.min_published_path_length_m)
        path_limited_activation = max(lower_bound, path_limited_activation)
        return min(raw_activation, path_limited_activation)

    def _slowdown_lookahead_m(self, activation_lookahead_m: float) -> float:
        """计算预激活/提前限速距离。

        Args:
            activation_lookahead_m: 真正切入 Frenet 的距离，单位 m。

        Returns:
            预激活距离，单位 m。障碍进入该距离后会提前尝试生成 Frenet 候选，
            但预激活阶段若暂时无解不会立即停车。
        """
        if self.approach_slowdown_extra_m >= 0.0:
            extra = self.approach_slowdown_extra_m
        else:
            extra = max(
                1.0,
                self.cruise_speed_mps * self.cruise_speed_mps
                / (2.0 * self.activation_decel_mps2),
            )
        return activation_lookahead_m + extra

    def _publish_centerline_path(
        self,
        stamp,
        position: np.ndarray,
        yaw: float,
        start_s: float,
        start_d: float = 0.0,
        mode: str = "centerline",
    ) -> None:
        """发布中心线或平滑回中局部轨迹并同步速度限制。

        Args:
            stamp: ROS 时间戳。
            position: 当前车辆世界坐标。
            yaw: 当前车辆航向，单位 rad。
            start_s: 局部中心线采样起点弧长。
            start_d: 当前车辆相对中心线的横向偏移，单位 m。偏移较大时会先
                发布从当前 `d` 平滑收敛到 0 的 return path，避免直接横跳。
            mode: 发布模式；`centerline` 表示正常跟线，`approach` 表示前方
                有威胁但尚处于预激活区间。

        Returns:
            None。
        """
        if self.reference is None:
            self.publish_stop_path(stamp, position, yaw)
            return
        returning_to_centerline = (
            abs(float(start_d)) > self.centerline_return_direct_d_threshold_m
        )
        if returning_to_centerline:
            centerline = sample_return_to_centerline_segment(
                self.reference,
                start_s,
                start_d,
                self.centerline_return_lookahead_m,
                self.centerline_return_step_m,
            )
        else:
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
            self.max_published_path_length_m,
        )
        if len(anchored) < 2:
            self.publish_stop_path(stamp, position, yaw)
            return
        self.publish_debug_markers(stamp, [], None)
        self.publish_path(stamp, anchored, mode=mode)

    def _publish_held_path_if_safe(
        self,
        stamp,
        now: float,
        occupancy,
        vehicle_pose: tuple[float, float, float],
    ) -> bool:
        """在上一条 Frenet 轨迹仍安全时继续复用。

        Args:
            stamp: ROS 时间戳。
            now: `time.monotonic()` 当前时间。
            occupancy: 当前局部占据栅格。
            vehicle_pose: 车辆世界位姿 `(x, y, yaw)`。

        Returns:
            如果成功复用并已发布轨迹，返回 `True`；否则返回 `False`，调用方
            需要重新规划。
        """
        cached = self._safe_cached_candidate(now, occupancy, vehicle_pose)
        if cached is None:
            return False
        candidate_xy, age, min_clearance = cached
        pose_error_m, heading_error_rad = path_pose_error(
            candidate_xy,
            np.array([vehicle_pose[0], vehicle_pose[1]], dtype=float),
            vehicle_pose[2],
        )
        if pose_error_m > self.held_path_max_lateral_error_m:
            self._log_hold_replan(
                now,
                "held path lateral error "
                f"{pose_error_m:.2f}m exceeds "
                f"{self.held_path_max_lateral_error_m:.2f}m",
            )
            return False
        if heading_error_rad > self.held_path_max_heading_error_rad:
            self._log_hold_replan(
                now,
                "held path heading error "
                f"{heading_error_rad:.2f}rad exceeds "
                f"{self.held_path_max_heading_error_rad:.2f}rad",
            )
            return False
        if min_clearance < self.held_path_replan_clearance_m:
            self._log_hold_replan(
                now,
                "held path clearance "
                f"{min_clearance:.2f}m below replan threshold "
                f"{self.held_path_replan_clearance_m:.2f}m",
            )
            return False

        anchored, _ = self._trim_cached_candidate(candidate_xy, vehicle_pose)
        if len(anchored) < 2:
            return False
        remaining_length = polyline_length(anchored)
        if remaining_length < self.held_path_min_remaining_m:
            self._log_hold_replan(
                now,
                "held path remaining length "
                f"{remaining_length:.2f}m below {self.held_path_min_remaining_m:.2f}m",
            )
            return False

        self.current_mode = "hold"
        self._log_mode(
            now,
            "Holding safe Frenet path "
            f"(age={age:.2f}s, clearance={min_clearance:.2f}m, "
            f"remaining={remaining_length:.2f}m, "
            f"pose_error={pose_error_m:.2f}m, "
            f"heading_error={heading_error_rad:.2f}rad).",
        )
        self.publish_path(stamp, anchored, mode="avoidance")
        return True

    def _safe_cached_candidate(
        self,
        now: float,
        occupancy,
        vehicle_pose: tuple[float, float, float],
    ) -> tuple[np.ndarray, float, float] | None:
        """读取并复检上一条安全候选轨迹。

        Args:
            now: `time.monotonic()` 当前时间。
            occupancy: 当前局部占据栅格。
            vehicle_pose: 车辆世界位姿 `(x, y, yaw)`。

        Returns:
            若缓存存在且剩余轨迹仍满足硬碰撞约束，返回
            `(remaining_candidate_xy, age_s, min_clearance_m)`；否则返回 `None`。
        """
        if self.last_safe_candidate_xy is None or self.last_safe_candidate_time is None:
            return None
        age = now - self.last_safe_candidate_time
        if self.latest_odom is not None:
            position, _ = self._odom_pose(self.latest_odom)
        else:
            position = np.array([vehicle_pose[0], vehicle_pose[1]], dtype=float)
        remaining_candidate = trim_path_to_position(
            self.last_safe_candidate_xy,
            position,
            min_remaining_length_m=0.0,
            anchor_lookahead_m=0.0,
            max_remaining_length_m=0.0,
        )
        if len(remaining_candidate) < 2:
            return None
        collision, min_clearance = occupancy.query_path(
            remaining_candidate,
            vehicle_pose,
            self.planner_config.corridor_radius_m,
            self.planner_config.corridor_sample_step_m,
            self.planner_config.footprint_front_m,
            self.planner_config.footprint_rear_m,
            self.planner_config.path_collision_sample_step_m,
        )
        if collision or min_clearance < self.planner_config.min_clearance_m:
            return None
        return remaining_candidate, age, float(min_clearance)

    def _trim_cached_candidate(
        self,
        candidate_xy: np.ndarray,
        vehicle_pose: tuple[float, float, float],
    ) -> tuple[np.ndarray, float]:
        """按车辆当前位置重锚定缓存候选轨迹。

        Args:
            candidate_xy: 上一条通过安全检查的候选轨迹世界坐标点。
            vehicle_pose: 车辆世界位姿 `(x, y, yaw)`，仅在 odom 暂不可用时兜底。

        Returns:
            `(anchored_path, publish_yaw)`。`anchored_path` 已从当前车辆附近裁剪，
            `publish_yaw` 用于后续需要发布 stop path 的场景。
        """
        if self.latest_odom is not None:
            publish_position, publish_yaw = self._odom_pose(self.latest_odom)
        else:
            publish_position = np.array([vehicle_pose[0], vehicle_pose[1]], dtype=float)
            publish_yaw = float(vehicle_pose[2])
        anchored = trim_path_to_position(
            candidate_xy,
            publish_position,
            self.min_published_path_length_m,
            self.published_path_lookahead_m,
            self.max_published_path_length_m,
        )
        return anchored, publish_yaw

    def _select_consistent_candidate(
        self,
        best_candidate,
        safe_candidates,
        now: float,
    ):
        """在原始最佳候选和近距离 profile 连续性之间做最终选择。

        Args:
            best_candidate: `plan_frenet_path()` 返回的原始最低 cost 候选。
            safe_candidates: 当前周期所有通过硬约束的候选，用于 RViz 和二次筛选。
            now: `time.monotonic()` 当前时间，用于日志限频。

        Returns:
            最终发布的候选轨迹。选择只在 hard-safe 候选内比较近距离
            `d(s)` profile，不再使用远端终点 `d` 伪造连续性。
        """
        # 下面这些情况无法或不需要做二次选择，直接使用 plan_frenet_path 的 raw best。
        if (
            # 没有 raw best，调用方后续会进入 stop/reuse 逻辑。
            best_candidate is None
            # 没有 safe candidate 集合，就没有可替换对象。
            or not safe_candidates
            # 没有上一条 profile，说明这是首次选择或记忆已清空。
            or self.last_selected_candidate_s_profile is None
            or self.last_selected_candidate_d_profile is None
            # profile 连续性被关闭时，不做额外选择。
            or self.candidate_profile_consistency_weight <= 0.0
            or self.candidate_profile_lookahead_m <= 0.0
        ):
            return best_candidate

        # 在所有 hard-safe candidates 里做二次选择，降低轨迹帧间跳变。
        selected, selection_reason = select_temporally_consistent_candidate(
            # plan_frenet_path 按原始 cost 选出的最低代价候选。
            best_candidate,
            # 当前周期所有通过硬约束的候选，来自 debug_candidates。
            safe_candidates,
            # 上一条最终发布候选的 s profile，用于和当前候选对齐比较。
            previous_s_profile=self.last_selected_candidate_s_profile,
            # 上一条最终发布候选的 d profile，用于判断整条横向通道是否连续。
            previous_d_profile=self.last_selected_candidate_d_profile,
            # 整条 d(s) profile 平均误差惩罚权重。
            profile_consistency_weight=self.candidate_profile_consistency_weight,
            # profile 最大跳变阈值，超过它认为跳出旧通道。
            profile_max_jump_m=self.candidate_profile_max_jump_m,
            # 只比较未来这段距离内的 d(s)，避免远端尾巴影响当前选择。
            profile_lookahead_m=self.candidate_profile_lookahead_m,
            # raw best 至少多出这么多 clearance，才允许打破 profile 连续性。
            profile_unlock_clearance_gain_m=(
                self.candidate_profile_unlock_clearance_gain_m
            ),
            # 期望安全裕度；raw best 达到该裕度才允许用 higher_clearance 解锁。
            safe_clearance_m=self.planner_config.safe_clearance,
        )
        # raw_cost 是不含 temporal/profile 惩罚的原始代价。
        raw_cost = float(best_candidate.cost)
        # selected_cost 同样是原始代价，用于日志解释为了连续性牺牲了多少基础 cost。
        selected_cost = float(selected.cost)
        # lower_cost 表示连续性候选原始 cost 太高，所以最终保留 raw best。
        if selection_reason == "lower_cost":
            # 候选选择日志限频，避免 20Hz 规划时刷屏。
            if now - self.last_candidate_selection_log_time >= 0.5:
                # 更新日志时间戳。
                self.last_candidate_selection_log_time = now
                # 打印 raw/selected cost，方便判断 temporal 惩罚是否过强。
                self.get_logger().info(
                    "Keeping lower-cost Frenet candidate instead of "
                    "temporally consistent candidate "
                    f"(raw_cost={raw_cost:.2f}, selected_cost={selected_cost:.2f}, "
                    f"cost_gap={selected_cost - raw_cost:.2f})."
                )
            # 明确返回 raw best，而不是 selected。
            return best_candidate
        # higher_clearance 表示 raw best 的安全裕度明显更好，所以打破连续性锁定。
        if selection_reason == "higher_clearance":
            # 同样做日志限频。
            if now - self.last_candidate_selection_log_time >= 0.5:
                # 更新日志时间戳。
                self.last_candidate_selection_log_time = now
                # 打印 raw clearance 和期望 safe_clearance，说明为什么保留 raw best。
                self.get_logger().info(
                    "Keeping higher-clearance Frenet candidate instead of "
                    "channel-consistent candidate "
                    f"(raw_clearance={float(best_candidate.min_clearance_m):.2f}m, "
                    f"safe_clearance={float(self.planner_config.safe_clearance):.2f}m)."
                )
            # 明确返回 raw best，而不是 selected。
            return best_candidate
        # 如果二次选择真的替换了 raw best，就打印一次解释日志。
        if selected is not best_candidate and now - self.last_candidate_selection_log_time >= 0.5:
            # 更新日志时间戳。
            self.last_candidate_selection_log_time = now
            # 打印 raw/selected 的 d、cost、clearance，方便 RViz 现象和代码选择对应起来。
            self.get_logger().info(
                "Selecting channel-consistent Frenet candidate "
                f"(raw_d={float(best_candidate.d[-1]):.2f}, "
                f"selected_d={float(selected.d[-1]):.2f}, "
                f"reason={selection_reason}, "
                f"raw_cost={raw_cost:.2f}, "
                f"selected_cost={selected_cost:.2f}, "
                f"raw_clearance={float(best_candidate.min_clearance_m):.2f}m, "
                f"selected_clearance={float(selected.min_clearance_m):.2f}m)."
            )
        # 返回最终发布的候选；可能是 raw best，也可能是 temporal/profile 更连续的候选。
        return selected

    def log_no_candidate(
        self,
        stats: FrenetPlanStats,
        occupancy,
        static_map_ready: bool,
        state,
        velocity_xy: np.ndarray,
        scan: LaserScan,
    ) -> None:
        """在当前周期没有安全 Frenet 候选时打印诊断日志。

        Args:
            stats: `plan_frenet_path()` 汇总的候选筛选计数。
            occupancy: 当前局部占据栅格。
            static_map_ready: 静态地图是否已经参与融合。
            state: 当前 Frenet 初始状态。
            velocity_xy: 车辆世界坐标速度向量。
            scan: 当前 LaserScan。

        Returns:
            None。日志按 0.5s 限频，避免无解时刷屏。
        """
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
        """发布沿当前车头方向的停车保持路径并同步 0 速限制。

        停车动作由 `stop` 模式的 0 速限制完成。这里仍发布一条非退化路径，是
        为了让 LQR 在停车期间有稳定的投影几何，避免 0.25m 级短路径触发
        open-loop endpoint 并导致横向误差在终点附近来回跳。

        Args:
            stamp: ROS 时间戳。
            position: 当前车辆世界坐标。
            yaw: 当前车辆航向，单位 rad。

        Returns:
            None。
        """
        forward = np.array([math.cos(yaw), math.sin(yaw)], dtype=float)
        sample_count = max(
            3,
            int(math.ceil(self.stop_path_length_m / self.centerline_return_step_m)) + 1,
        )
        offsets = np.linspace(0.0, self.stop_path_length_m, sample_count, dtype=float)
        points = position.reshape(1, 2) + offsets.reshape(-1, 1) * forward.reshape(1, 2)
        self.publish_path(stamp, points, mode="stop", force=True)

    def publish_path(
        self,
        stamp,
        points: np.ndarray,
        mode: str = "path",
        force: bool = False,
    ) -> bool:
        """发布局部路径并根据模式同步速度限制。

        Args:
            stamp: ROS 时间戳。
            points: 世界坐标路径点，形状为 `(N, 2)`。
            mode: 路径模式，决定速度限制和重发布节流策略。
            force: 是否跳过 `_should_publish_path()` 强制发布。

        Returns:
            实际发布了新 Path 返回 `True`；被节流时返回 `False`，但仍会刷新速度限制。
        """
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
        """按当前路径模式发布局部速度限制。

        Args:
            mode: `stop`、`avoidance`、`approach` 或 `centerline`。

        Returns:
            None。
        """
        limit = self._speed_limit_for_mode(mode)
        msg = Float32()
        msg.data = float(limit)
        self.speed_limit_pub.publish(msg)

    def _speed_limit_for_mode(self, mode: str) -> float:
        """查询路径模式对应的局部速度限制。

        Args:
            mode: 当前局部路径模式。

        Returns:
            局部限速，单位 m/s；未知模式返回 NaN 表示清除 LQR 局部限速。
        """
        if mode == "stop":
            return max(0.0, self.stop_speed_limit_mps)
        if mode == "avoidance":
            return self.avoidance_speed_limit_mps
        if mode == "approach":
            return self.avoidance_speed_limit_mps
        if mode == "centerline":
            return self.centerline_speed_limit_mps
        return math.nan

    def _should_publish_path(self, now: float, points: np.ndarray, mode: str) -> bool:
        """判断是否需要真正发布新的 Path 消息。

        Args:
            now: `time.monotonic()` 当前时间。
            points: 准备发布的局部路径点。
            mode: 准备发布的路径模式。

        Returns:
            `True` 表示应发布；`False` 表示路径变化不大且仍在节流时间内。
        """
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
        """估计上一条已发布路径从当前位置开始的剩余长度。

        Args:
            position: 当前车辆世界坐标。

        Returns:
            剩余路径长度，单位 m。没有已发布路径时返回 0。
        """
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
        """发布 RViz 候选轨迹 MarkerArray。

        Args:
            stamp: ROS 时间戳。
            candidates: 当前周期的 safe candidate 列表。
            best_candidate: 当前最终选择的候选；会用更粗、更亮颜色显示。
            keep_previous: 当前周期无新候选时是否沿用上一帧 marker，避免 RViz
                在短暂无解/复用阶段闪烁。

        Returns:
            None。
        """
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
        """当前周期无解时短时复用上一条安全候选。

        与 `_publish_held_path_if_safe()` 的区别是：held path 是正常规划前的
        主动保持；reuse 是当前周期已经规划失败后的兜底。复用前仍会重新做
        collision/clearance 检查，避免盲目沿用过期轨迹。

        Args:
            stamp: ROS 时间戳。
            now: `time.monotonic()` 当前时间。
            occupancy: 当前局部占据栅格。
            vehicle_pose: 车辆世界位姿 `(x, y, yaw)`。

        Returns:
            如果成功复用并发布上一条候选，返回 `True`；否则返回 `False`。
        """
        cached = self._safe_cached_candidate(now, occupancy, vehicle_pose)
        if cached is None:
            if (
                self.last_safe_candidate_xy is not None
                and self.last_safe_candidate_time is not None
                and now - self.last_reuse_log_time >= 0.5
            ):
                age = now - self.last_safe_candidate_time
                self.last_reuse_log_time = now
                self.get_logger().warning(
                    "Discarding stale Frenet candidate because it is no longer safe "
                    f"(age={age:.2f}s)."
                )
            return False
        candidate_xy, age, min_clearance = cached
        if age > self.reuse_last_candidate_timeout_s:
            return False
        published_path, _ = self._trim_cached_candidate(
            candidate_xy,
            vehicle_pose,
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
        """构造清空上一帧 RViz marker 的 DELETEALL 消息。

        Args:
            stamp: ROS 时间戳。

        Returns:
            `visualization_msgs/Marker`。
        """
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def _candidate_marker(self, stamp, points: np.ndarray, marker_id: int, *, is_best: bool) -> Marker:
        """把一条 Frenet 候选轨迹转换成 RViz line strip marker。

        Args:
            stamp: ROS 时间戳。
            points: 候选轨迹世界坐标点。
            marker_id: marker ID。
            is_best: 是否为最终选择的最低代价/同侧稳定候选。

        Returns:
            可发布到 MarkerArray 的 line strip marker。
        """
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
        """生成当前车辆位姿下的静态地图局部占据栅格。

        Args:
            vehicle_pose: 车辆世界位姿 `(x, y, yaw)`。

        Returns:
            与 LaserScan 局部栅格同形状的布尔占据数组；静态地图尚未加载时返回
            `None`，调用方会只使用 LaserScan 障碍。
        """
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
        """从 odom 消息中提取二维位置和 yaw。

        Args:
            msg: 最新 `nav_msgs/Odometry`。

        Returns:
            `(position_xy, yaw)`，其中位置单位为 m，yaw 单位为 rad。
        """
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
        """从 odom 中估计规划使用的位置、航向和世界坐标速度。

        速度优先由相邻 odom 位置差分得到，因为仿真中 twist 有时会在路径切换
        或碰撞反弹附近短时异常。若差分时间戳不可用，则退回 twist 速度；若
        `max_observed_speed_mps` 设置为正数，会对明显不合理的速度估计限幅。

        Args:
            msg: 最新 `nav_msgs/Odometry`。

        Returns:
            `(position_xy, yaw, velocity_xy)`。
        """
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
        """限频打印单次 Frenet 规划耗时和候选筛选统计。

        Args:
            now: `time.monotonic()` 当前时间。
            stats: 当前周期候选生成和拒绝原因统计。
            elapsed_sec: 当前规划周期耗时，单位 s。

        Returns:
            None。
        """
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
        """限频打印模式切换/中心线回退日志。

        Args:
            now: `time.monotonic()` 当前时间。
            message: 要输出的日志文本。

        Returns:
            None。
        """
        if now - self.last_mode_log_time < 1.0:
            return
        self.last_mode_log_time = now
        self.get_logger().info(message)

    def _log_far_threat(
        self,
        now: float,
        threat_distance_m: float,
        activation_lookahead_m: float,
        slowdown_lookahead_m: float,
        approach_mode: bool,
    ) -> None:
        """限频打印“发现远处中心线威胁”的决策日志。

        Args:
            now: `time.monotonic()` 当前时间。
            threat_distance_m: 中心线前方最近威胁距离，单位 m。
            activation_lookahead_m: 真正切入 Frenet 避障的距离，单位 m。
            slowdown_lookahead_m: 提前限速/预激活距离，单位 m。
            approach_mode: `True` 表示已进入预激活区间并提前尝试 Frenet。

        Returns:
            None。
        """
        if now - self.last_far_threat_log_time < 1.0:
            return
        self.last_far_threat_log_time = now
        action = "pre-activating Frenet planning" if approach_mode else "keeping centerline"
        self.get_logger().info(
            "Centerline threat detected; "
            f"{action} "
            f"(threat_distance={threat_distance_m:.2f}m, "
            f"activation_lookahead={activation_lookahead_m:.2f}m, "
            f"slowdown_lookahead={slowdown_lookahead_m:.2f}m, "
            f"cruise_speed={self.cruise_speed_mps:.2f}m/s)."
        )

    def _log_hold_replan(self, now: float, reason: str) -> None:
        """限频打印不再 hold 上一条轨迹而选择重规划的原因。

        Args:
            now: `time.monotonic()` 当前时间。
            reason: 触发重规划的原因说明。

        Returns:
            None。
        """
        if now - self.last_hold_replan_log_time < 0.5:
            return
        self.last_hold_replan_log_time = now
        self.get_logger().info(f"Replanning instead of holding Frenet path ({reason}).")


def main(args=None) -> None:
    """ROS 2 节点入口。

    Args:
        args: 传给 `rclpy.init()` 的可选命令行参数。

    Returns:
        None。函数会阻塞 spin，直到节点退出或收到 Ctrl-C。
    """
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

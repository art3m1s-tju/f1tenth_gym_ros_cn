#!/usr/bin/env python3
"""基于离散 LQR 的路径跟踪控制器。

本模块实现了一个适用于 F1TENTH 平台的 LQR（线性二次调节器）路径跟踪控制器。
控制器订阅全局参考轨迹和车辆里程计，计算最优转向指令并发布 Ackermann 控制命令。

核心算法流程：
    1. 通过 TF 或里程计获取车辆在地图坐标系下的位姿。
    2. 将车辆位置投影到参考路径上，计算横向误差和航向误差。
    3. 基于当前速度构建离散横向误差动力学模型，求解离散代数 Riccati 方程
       得到最优反馈增益 K。
    4. 反馈项：delta_fb = -K @ [e_lateral, e_heading]。
    5. 前馈项：delta_ff = feedforward_gain * atan(wheelbase * curvature_ref)。
    6. 最终转向指令 = 反馈 + 前馈，经最大转向角限幅后发布。
    7. 速度指令基于曲率和横向加速度约束进行限速。
"""
from __future__ import annotations

import csv
import math
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from scipy.spatial import KDTree
from tf2_ros import Buffer, TransformException, TransformListener

from pnc_rc.lqr.math import (
    compute_lqr_steering,
    compute_path_curvatures,
    compute_path_headings,
    wrap_angle,
)
from pnc_rc.lqr.geometry import (
    compute_curvature_limited_speed,
    interpolate_angle,
    project_to_path,
)
from package_paths import get_default_output_root


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """从四元数中提取平面偏航角。

    Args:
        x: 四元数 x 分量。
        y: 四元数 y 分量。
        z: 四元数 z 分量。
        w: 四元数 w 分量。

    Returns:
        偏航角（弧度），范围 (-pi, pi]。
    """
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


@dataclass(frozen=True)
class ReferenceSample:
    """参考路径上的连续投影采样点。

    Attributes:
        point: 投影点的二维坐标 [x, y]。
        heading: 投影点处的参考航向角（弧度）。
        curvature: 投影点处的参考曲率（1/m），正值表示左转。
        segment_idx: 投影所在路径段的起点索引。
        segment_t: 段内插值参数，范围 [0, 1]。
        closest_idx: KD-tree 查询得到的最近路径点索引。
    """

    point: np.ndarray
    heading: float
    curvature: float
    segment_idx: int
    segment_t: float
    closest_idx: int


class LqrController(Node):
    """基于离散 LQR 的 ROS 2 路径跟踪控制节点。

    该控制器通过求解离散代数 Riccati 方程（DARE）获得状态反馈增益，
    结合曲率前馈实现对参考路径的精确跟踪。状态向量为 [横向误差, 航向误差]，
    控制输入为前轮转向角。

    主要特性：
        - 支持通过 TF 树或直接里程计获取车辆位姿。
        - 基于曲率的自适应限速，防止弯道侧滑。
        - 加减速斜坡限幅，避免速度阶跃。
        - 实时 CSV 日志记录，便于离线分析和调参。
        - 发布实际行驶轨迹供 RViz 可视化。
    """

    def __init__(self) -> None:
        """初始化 LQR 控制器节点，声明参数并创建订阅/发布器。"""
        super().__init__("lqr_controller")

        default_log_path = get_default_output_root() / "logs" / "lqr_tracking_log.csv"

        # ---- ROS 话题参数 ----
        self.declare_parameter("path_topic", "/global_trajectory")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("drive_topic", "/drive")
        self.declare_parameter("tracked_frame", "")
        self.declare_parameter("use_tf_pose", True)
        self.declare_parameter("vehicle_frame", "base_link")
        self.declare_parameter("tf_lookup_timeout_sec", 0.02)
        self.declare_parameter("path_closed_loop", True)
        self.declare_parameter("open_loop_finish_distance", 0.25)

        # ---- 车辆模型参数 ----
        self.declare_parameter("wheelbase", 0.3302)
        self.declare_parameter("target_speed", 0.60)
        self.declare_parameter("min_speed", 0.25)
        self.declare_parameter("max_steering_angle", 0.61)
        self.declare_parameter("max_lateral_accel", 3.0)
        self.declare_parameter("enable_curvature_speed_limit", True)
        self.declare_parameter("enable_speed_ramp", True)
        self.declare_parameter("max_accel", 0.8)
        self.declare_parameter("max_decel", 1.5)
        self.declare_parameter("validation_position_noise_std", 0.0)
        self.declare_parameter("validation_heading_noise_std_deg", 0.0)
        self.declare_parameter("validation_pose_delay_ms", 0.0)
        self.declare_parameter("validation_noise_seed", 42)

        # ---- LQR 核心参数 ----
        self.declare_parameter("control_dt", 0.05)
        self.declare_parameter("lqr_min_model_speed", 0.25)
        self.declare_parameter("lqr_q_lateral", 3.0)
        self.declare_parameter("lqr_q_heading", 1.2)
        self.declare_parameter("lqr_r_steering", 8.0)
        self.declare_parameter("lqr_feedforward_gain", 1.0)

        # ---- 日志与可视化参数 ----
        self.declare_parameter("enable_tracking_csv_log", True)
        self.declare_parameter("tracking_log_path", str(default_log_path))
        self.declare_parameter("tracked_path_topic", "/tracked_path_lqr")
        self.declare_parameter("max_tracked_path_points", 2000)

        # ---- 速度分段增益表 ----
        self.declare_parameter("lqr_gain_table_path", "")

        # ---- 读取参数值 ----
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.drive_topic = str(self.get_parameter("drive_topic").value)
        self.tracked_frame = str(self.get_parameter("tracked_frame").value)
        self.use_tf_pose = bool(self.get_parameter("use_tf_pose").value)
        self.vehicle_frame = str(self.get_parameter("vehicle_frame").value)
        self.tf_lookup_timeout = Duration(
            seconds=max(0.0, float(self.get_parameter("tf_lookup_timeout_sec").value))
        )
        self.path_closed_loop = bool(self.get_parameter("path_closed_loop").value)
        self.open_loop_finish_distance = max(
            1e-3,
            float(self.get_parameter("open_loop_finish_distance").value),
        )

        self.wheelbase = max(1e-6, float(self.get_parameter("wheelbase").value))
        self.target_speed = max(0.0, float(self.get_parameter("target_speed").value))
        self.min_speed = max(0.0, float(self.get_parameter("min_speed").value))
        self.max_steering_angle = max(
            1e-6,
            float(self.get_parameter("max_steering_angle").value),
        )
        self.max_lateral_accel = max(
            1e-6,
            float(self.get_parameter("max_lateral_accel").value),
        )
        self.enable_curvature_speed_limit = bool(
            self.get_parameter("enable_curvature_speed_limit").value
        )
        self.enable_speed_ramp = bool(self.get_parameter("enable_speed_ramp").value)
        self.max_accel = max(0.0, float(self.get_parameter("max_accel").value))
        self.max_decel = max(0.0, float(self.get_parameter("max_decel").value))
        self.validation_position_noise_std = max(
            0.0,
            float(self.get_parameter("validation_position_noise_std").value),
        )
        self.validation_heading_noise_std = math.radians(
            max(0.0, float(self.get_parameter("validation_heading_noise_std_deg").value))
        )
        self.validation_pose_delay_sec = (
            max(0.0, float(self.get_parameter("validation_pose_delay_ms").value)) / 1000.0
        )
        self.validation_noise_seed = int(self.get_parameter("validation_noise_seed").value)

        self.control_dt = max(1e-4, float(self.get_parameter("control_dt").value))
        self.lqr_min_model_speed = max(
            1e-4,
            float(self.get_parameter("lqr_min_model_speed").value),
        )
        self.lqr_q_lateral = max(1e-9, float(self.get_parameter("lqr_q_lateral").value))
        self.lqr_q_heading = max(1e-9, float(self.get_parameter("lqr_q_heading").value))
        self.lqr_r_steering = max(
            1e-9,
            float(self.get_parameter("lqr_r_steering").value),
        )
        self.lqr_feedforward_gain = float(
            self.get_parameter("lqr_feedforward_gain").value
        )

        self.enable_tracking_csv_log = bool(
            self.get_parameter("enable_tracking_csv_log").value
        )
        self.tracking_log_path = Path(
            str(self.get_parameter("tracking_log_path").value)
        ).expanduser().resolve()

        # ---- 加载速度分段增益表 ----
        self.gain_table = None
        gain_table_path_str = str(self.get_parameter("lqr_gain_table_path").value)
        if gain_table_path_str:
            table_path = Path(gain_table_path_str).expanduser().resolve()
            if table_path.exists():
                from lqr_sweep.lookup_table import LqrLookupTable
                self.gain_table = LqrLookupTable.load(table_path)
                self.get_logger().info(
                    f"Loaded LQR gain table with {len(self.gain_table)} speed points."
                )

        # ---- 内部状态初始化 ----
        self.points: np.ndarray | None = None  # 参考路径点数组 [N, 2]
        self.headings: np.ndarray | None = None  # 每个路径点的切向航向角
        self.curvatures: np.ndarray | None = None  # 每个路径点的有符号曲率
        self.kdtree: KDTree | None = None  # 路径点空间索引
        self.path_frame_id = "map"
        self.previous_stamp_sec: float | None = None
        self.current_speed_cmd = 0.0  # 速度斜坡当前值
        self.open_loop_finished = False
        self.pose_history = deque()
        self.noise_rng = np.random.default_rng(self.validation_noise_seed)
        self.csv_file = None
        self.csv_writer = None
        self.csv_rows_since_flush = 0

        # ---- TF 基础设施 ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- 实际轨迹可视化 ----
        self.tracked_path_topic = str(self.get_parameter("tracked_path_topic").value)
        self.max_tracked_path_points = int(self.get_parameter("max_tracked_path_points").value)
        self.tracked_path_msg = PathMsg()
        self.tracked_path_msg.header.frame_id = "map"

        if self.enable_tracking_csv_log:
            self._open_tracking_log()

        # ---- 创建订阅器和发布器 ----
        # 使用 TRANSIENT_LOCAL QoS 确保晚启动也能收到最近一次发布的路径
        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.path_sub = self.create_subscription(
            PathMsg,
            self.path_topic,
            self.path_callback,
            path_qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10,
        )
        self.tracked_path_pub = self.create_publisher(
            PathMsg,
            self.tracked_path_topic,
            10,
        )

        self.get_logger().info(
            "LQR controller started "
            f"(path_topic={self.path_topic}, odom_topic={self.odom_topic}, "
            f"drive_topic={self.drive_topic}, use_tf_pose={self.use_tf_pose}, "
            f"target_speed={self.target_speed:.2f}, "
            f"validation_noise=({self.validation_position_noise_std:.3f}m, "
            f"{math.degrees(self.validation_heading_noise_std):.2f}deg, "
            f"{self.validation_pose_delay_sec * 1000.0:.0f}ms), "
            f"Q=({self.lqr_q_lateral:.2f}, {self.lqr_q_heading:.2f}), "
            f"R={self.lqr_r_steering:.2f})."
        )

    def _open_tracking_log(self) -> None:
        """创建并初始化跟踪日志 CSV 文件。

        日志记录每个控制周期的完整状态，包括位姿、误差、控制量和 LQR 增益，
        供 tracker_evaluate.py 进行离线误差分析。
        """
        self.tracking_log_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.tracking_log_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "time",
                "x",
                "y",
                "yaw",
                "v_actual",
                "v_cmd",
                "e_y",
                "e_psi",
                "curvature_ref",
                "delta_feedback",
                "delta_feedforward",
                "delta_cmd",
                "lqr_k_y",
                "lqr_k_psi",
                "closest_idx",
                "segment_idx",
                "segment_t",
                "compute_time_ms",
            ]
        )
        self.csv_file.flush()

    def destroy_node(self) -> bool:
        """节点销毁前刷新并关闭 CSV 日志文件。"""
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        return super().destroy_node()

    def path_callback(self, msg: PathMsg) -> None:
        """接收参考路径并重建内部数据结构。

        收到新路径后，预计算每个路径点的切向航向角和离散曲率，
        并构建 KD-tree 空间索引用于快速最近邻查询。

        Args:
            msg: nav_msgs/Path 消息，包含参考轨迹的 PoseStamped 序列。
        """
        points = np.array(
            [
                [pose.pose.position.x, pose.pose.position.y]
                for pose in msg.poses
            ],
            dtype=float,
        )
        if len(points) < 2:
            self.get_logger().warning("Ignoring path with fewer than 2 poses.")
            return

        self.points = points
        self.headings = compute_path_headings(points, self.path_closed_loop)
        self.curvatures = compute_path_curvatures(points, self.path_closed_loop)
        self.kdtree = KDTree(points)
        self.path_frame_id = msg.header.frame_id or self.tracked_frame or "map"
        self.open_loop_finished = False
        self.get_logger().info(
            f"Loaded LQR reference path with {len(points)} points "
            f"(frame={self.path_frame_id}, closed_loop={self.path_closed_loop})."
        )

    def _resolve_pose(self, msg: Odometry, stamp) -> tuple[np.ndarray, float] | None:
        """解析车辆在路径坐标系下的位姿。

        优先通过 TF 树查询 (path_frame → vehicle_frame) 获取位姿；
        若 TF 不可用且里程计坐标系与路径坐标系一致，则退回到直接读取里程计。

        Args:
            msg: 当前里程计消息。
            stamp: 用于 TF 查询的时间戳。

        Returns:
            (position, yaw) 元组，若无法解析则返回 None。
        """
        if self.use_tf_pose:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.path_frame_id,
                    self.vehicle_frame,
                    Time(),
                    self.tf_lookup_timeout,
                )
            except TransformException as exc:
                self.get_logger().warning(
                    f"Skipping LQR update because TF pose is unavailable: {exc}",
                    throttle_duration_sec=1.0,
                )
                return None

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return (
                np.array([translation.x, translation.y], dtype=float),
                quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w),
            )

        pose = msg.pose.pose
        odom_frame = msg.header.frame_id or ""
        expected_frame = self.tracked_frame or self.path_frame_id
        if odom_frame and expected_frame and odom_frame != expected_frame:
            self.get_logger().warning(
                "Using odometry pose even though odom frame '%s' differs from path frame '%s'."
                % (odom_frame, expected_frame),
                throttle_duration_sec=2.0,
            )
        return (
            np.array([pose.position.x, pose.position.y], dtype=float),
            quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    def _sample_reference(self, position: np.ndarray) -> ReferenceSample:
        """将车辆位置投影到最近的路径段上，获取连续参考量。"""
        assert self.points is not None
        assert self.headings is not None
        assert self.curvatures is not None
        assert self.kdtree is not None

        proj = project_to_path(
            position, self.points, self.kdtree,
            self.headings, self.curvatures, self.path_closed_loop,
        )
        return ReferenceSample(
            point=proj.point,
            heading=proj.heading,
            curvature=proj.curvature,
            segment_idx=proj.segment_idx,
            segment_t=proj.segment_t,
            closest_idx=proj.closest_idx,
        )

    def _validation_pose_for_control(
        self,
        position: np.ndarray,
        yaw: float,
        stamp_sec: float,
    ) -> tuple[np.ndarray, float]:
        """Return the delayed/noisy pose used only by validation control."""
        self.pose_history.append((stamp_sec, position.copy(), float(yaw)))

        delayed_position = position
        delayed_yaw = yaw
        if self.validation_pose_delay_sec > 0.0:
            cutoff = stamp_sec - self.validation_pose_delay_sec
            while len(self.pose_history) > 1 and self.pose_history[1][0] <= cutoff:
                self.pose_history.popleft()
            delayed_position = self.pose_history[0][1]
            delayed_yaw = self.pose_history[0][2]
        else:
            self.pose_history.clear()

        control_position = delayed_position.copy()
        control_yaw = float(delayed_yaw)
        if self.validation_position_noise_std > 0.0:
            control_position += self.noise_rng.normal(
                0.0,
                self.validation_position_noise_std,
                size=2,
            )
        if self.validation_heading_noise_std > 0.0:
            control_yaw = wrap_angle(
                control_yaw
                + float(self.noise_rng.normal(0.0, self.validation_heading_noise_std))
            )
        return control_position, control_yaw

    def _compute_speed_command(self, delta_cmd: float, curvature_ref: float) -> float:
        """根据转向指令和参考曲率计算安全速度。"""
        if self.enable_curvature_speed_limit:
            return compute_curvature_limited_speed(
                self.target_speed, curvature_ref, delta_cmd,
                self.wheelbase, self.max_lateral_accel, self.min_speed,
            )
        return self.target_speed

    def _apply_speed_ramp(self, desired_speed: float, stamp_sec: float) -> float:
        """对速度指令施加加减速斜坡限幅，避免起步阶跃。

        Args:
            desired_speed: 期望目标速度（m/s）。
            stamp_sec: 当前时间戳（秒）。

        Returns:
            经过加减速限幅后的实际速度指令（m/s）。
        """
        if not self.enable_speed_ramp:
            self.current_speed_cmd = desired_speed
            return desired_speed

        if self.previous_stamp_sec is None:
            self.current_speed_cmd = min(self.current_speed_cmd, desired_speed)
            return self.current_speed_cmd

        dt = max(0.0, stamp_sec - self.previous_stamp_sec)
        if desired_speed >= self.current_speed_cmd:
            step = self.max_accel * dt
            self.current_speed_cmd = min(desired_speed, self.current_speed_cmd + step)
        else:
            step = self.max_decel * dt
            self.current_speed_cmd = max(desired_speed, self.current_speed_cmd - step)
        return self.current_speed_cmd

    def _publish_drive(self, stamp, steering: float, speed: float) -> None:
        """发布一条 Ackermann 控制指令。"""
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = stamp
        drive_msg.drive.steering_angle = float(steering)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)

    def _write_tracking_log(
        self,
        stamp_sec: float,
        position: np.ndarray,
        yaw: float,
        v_actual: float,
        v_cmd: float,
        lateral_error: float,
        heading_error: float,
        sample: ReferenceSample,
        delta_feedback: float,
        delta_feedforward: float,
        delta_cmd: float,
        lqr_gain: np.ndarray,
        compute_time_ms: float,
    ) -> None:
        """向跟踪日志 CSV 写入一行控制周期数据。"""
        if self.csv_writer is None or self.csv_file is None:
            return

        self.csv_writer.writerow(
            [
                f"{stamp_sec:.6f}",
                f"{position[0]:.6f}",
                f"{position[1]:.6f}",
                f"{yaw:.6f}",
                f"{v_actual:.6f}",
                f"{v_cmd:.6f}",
                f"{lateral_error:.6f}",
                f"{heading_error:.6f}",
                f"{sample.curvature:.6f}",
                f"{delta_feedback:.6f}",
                f"{delta_feedforward:.6f}",
                f"{delta_cmd:.6f}",
                f"{lqr_gain[0]:.6f}",
                f"{lqr_gain[1]:.6f}",
                sample.closest_idx,
                sample.segment_idx,
                f"{sample.segment_t:.6f}",
                f"{compute_time_ms:.6f}",
            ]
        )
        self.csv_rows_since_flush += 1
        if self.csv_rows_since_flush >= 20:
            self.csv_file.flush()
            self.csv_rows_since_flush = 0

    def _append_tracked_pose(self, position: np.ndarray, yaw: float, stamp) -> None:
        """将当前位姿追加到实际轨迹并发布供 RViz 可视化。

        累计点数超过 max_tracked_path_points 时丢弃最旧的点。
        """
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.path_frame_id
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)

        self.tracked_path_msg.header.stamp = stamp
        self.tracked_path_msg.header.frame_id = self.path_frame_id
        self.tracked_path_msg.poses.append(pose)
        if len(self.tracked_path_msg.poses) > self.max_tracked_path_points:
            self.tracked_path_msg.poses = self.tracked_path_msg.poses[-self.max_tracked_path_points:]
        self.tracked_path_pub.publish(self.tracked_path_msg)

    def odom_callback(self, msg: Odometry) -> None:
        """里程计回调：执行一次完整的 LQR 控制更新。

        处理流程：
            1. 解析车辆位姿（TF 或里程计）。
            2. 投影到参考路径，计算横向误差 e_y 和航向误差 e_psi。
            3. 求解 LQR 增益并计算反馈转向量。
            4. 加上曲率前馈项，经转向角限幅后发布。
            5. 计算基于曲率的安全速度并施加加减速斜坡。
            6. 发布控制指令、更新轨迹可视化、写入日志。

        Args:
            msg: nav_msgs/Odometry 消息。
        """
        callback_start = time.perf_counter()
        if self.points is None or self.kdtree is None:
            return

        stamp = (
            msg.header.stamp
            if msg.header.stamp.sec or msg.header.stamp.nanosec
            else self.get_clock().now().to_msg()
        )
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        pose = self._resolve_pose(msg, stamp)
        if pose is None:
            return
        position, yaw = pose

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        v_actual = float(math.hypot(vx, vy))
        control_position, control_yaw = self._validation_pose_for_control(
            position,
            yaw,
            stamp_sec,
        )
        sample = self._sample_reference(control_position)
        truth_sample = self._sample_reference(position)

        normal = np.array([-math.sin(sample.heading), math.cos(sample.heading)])
        lateral_error = float((control_position - sample.point) @ normal)
        heading_error = wrap_angle(control_yaw - sample.heading)
        truth_normal = np.array([-math.sin(truth_sample.heading), math.cos(truth_sample.heading)])
        truth_lateral_error = float((position - truth_sample.point) @ truth_normal)
        truth_heading_error = wrap_angle(yaw - truth_sample.heading)

        dt = self.control_dt
        if self.previous_stamp_sec is not None:
            measured_dt = stamp_sec - self.previous_stamp_sec
            if 1e-4 <= measured_dt <= 0.5:
                dt = measured_dt

        if self.gain_table is not None:
            interp = self.gain_table.interpolate(max(v_actual, self.target_speed))
            q_lat = interp.q_lateral
            q_head = interp.q_heading
            r_steer = interp.r_steering
            ff_gain = interp.feedforward_gain
        else:
            q_lat = self.lqr_q_lateral
            q_head = self.lqr_q_heading
            r_steer = self.lqr_r_steering
            ff_gain = self.lqr_feedforward_gain

        delta_raw, delta_feedback, delta_feedforward, lqr_gain = compute_lqr_steering(
            lateral_error=lateral_error,
            heading_error=heading_error,
            curvature_ref=sample.curvature,
            speed=max(v_actual, self.target_speed),
            wheelbase=self.wheelbase,
            dt=dt,
            min_model_speed=self.lqr_min_model_speed,
            q_lateral=q_lat,
            q_heading=q_head,
            r_steering=r_steer,
            feedforward_gain=ff_gain,
        )
        delta_cmd = float(
            np.clip(delta_raw, -self.max_steering_angle, self.max_steering_angle)
        )
        desired_speed = self._compute_speed_command(delta_cmd, sample.curvature)
        v_cmd = self._apply_speed_ramp(desired_speed, stamp_sec)

        if self._open_loop_finished(position):
            if not self.open_loop_finished:
                self.get_logger().info("LQR open-loop endpoint reached; stopping.")
            self.open_loop_finished = True
            delta_cmd = 0.0
            v_cmd = 0.0
            self.current_speed_cmd = 0.0

        self._publish_drive(stamp, delta_cmd, v_cmd)
        self._append_tracked_pose(position, yaw, stamp)
        compute_time_ms = (time.perf_counter() - callback_start) * 1000.0
        self._write_tracking_log(
            stamp_sec=stamp_sec,
            position=position,
            yaw=yaw,
            v_actual=v_actual,
            v_cmd=v_cmd,
            lateral_error=truth_lateral_error,
            heading_error=truth_heading_error,
            sample=truth_sample,
            delta_feedback=delta_feedback,
            delta_feedforward=delta_feedforward,
            delta_cmd=delta_cmd,
            lqr_gain=lqr_gain,
            compute_time_ms=compute_time_ms,
        )
        self.previous_stamp_sec = stamp_sec

    def _open_loop_finished(self, position: np.ndarray) -> bool:
        """判断开环路径是否已到达终点。"""
        if self.path_closed_loop or self.points is None:
            return False
        endpoint = self.points[-1]
        return float(np.linalg.norm(position - endpoint)) <= self.open_loop_finish_distance


def main(args=None) -> None:
    """ROS 2 节点入口：启动 LQR 控制器并持续运行直到关闭。"""
    rclpy.init(args=args)
    node = LqrController()
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

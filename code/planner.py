#!/usr/bin/env python3
"""ROS 2 全局轨迹规划节点。

该节点读取处理后的赛道边界数据，运行 SQP 轨迹优化算法，
并发布一个经过平滑处理且 latched 的全局路径供控制器跟踪。
"""
from __future__ import annotations
import math
import os
import hashlib
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scipy.interpolate import splev, splprep
from scipy.spatial import cKDTree
from std_msgs.msg import String

# 导入预先定义的优化工具
from optimizer import (
    analyze_trajectory_candidate,
    estimate_closed_curve_curvature,
    load_track_data,
    optimize_trajectory,
)
from package_paths import get_default_output_root
from ros_pipeline_utils import metadata_from_json


CONTROL_FRIENDLY_MAX_CURVATURE = 1.0
CURVATURE_TOLERANCE = 1e-9


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    """将偏航角 (Yaw) 转换为 2D 平面简化的四元数 (z, w) 分量。
    四元数q = [x, y, z, w]四个数字平方和必须为1

    Args:
        yaw: 偏航角（弧度）。

    Returns:
        (qz, qw) 四元数分量元组。
    """
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def resample_closed_curve(points_xy: np.ndarray, target_count: int) -> np.ndarray:
    """Resample a closed curve to approximately equal arc-length spacing."""
    if len(points_xy) < 3 or target_count < 3:
        return points_xy
    closed = np.vstack([points_xy, points_xy[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    total = float(seg.sum())
    if total <= 1e-9:
        return points_xy
    cumulative = np.concatenate([[0.0], np.cumsum(seg)])
    targets = np.linspace(0.0, total, target_count + 1)[:-1]
    sampled = np.column_stack(
        [
            np.interp(targets, cumulative, closed[:, 0]),
            np.interp(targets, cumulative, closed[:, 1]),
        ]
    )
    return sampled


def smooth_closed_trajectory(points: np.ndarray, smoothing: float) -> np.ndarray:
    """Apply periodic B-spline smoothing and equal-distance resampling."""
    if smoothing <= 0.0 or len(points) < 8:
        return points
    try:
        tck, _ = splprep(
            [points[:, 0], points[:, 1]],
            s=float(smoothing),
            per=True,
            k=min(3, len(points) - 1),
        )
        u = np.linspace(0.0, 1.0, len(points), endpoint=False)
        x_s, y_s = splev(u, tck)
        return resample_closed_curve(np.column_stack([x_s, y_s]), len(points))
    except Exception as exc:
        print(f"[planner] Warning: trajectory smoothing failed ({exc}); using raw optimizer output.")
        return points


def build_control_friendly_trajectory(
    inner_xy: np.ndarray,
    outer_xy: np.ndarray,
    *,
    alpha: float,
    smoothing: float,
    auto_alpha: bool,
    target_max_curvature: float,
    min_clearance: float,
) -> tuple[np.ndarray, dict[str, float]]:
    """Build a smooth, conservative center-biased trajectory for controller tuning.

    ``alpha=0.5`` is the geometric centerline. Larger values move the trajectory
    toward the outer boundary, which can increase turning radius around a center
    island but also reduces recovery room near the outside wall. The automatic
    mode searches a bounded alpha/smoothing grid and requires the selected path
    to satisfy the configured curvature cap.
    """
    if len(inner_xy) != len(outer_xy) or len(inner_xy) < 8:
        raise ValueError("control_friendly trajectory requires paired inner/outer boundaries")

    inner_tree = cKDTree(inner_xy)
    outer_tree = cKDTree(outer_xy)

    def evaluate(candidate: np.ndarray) -> dict[str, float]:
        curvature = np.abs(estimate_closed_curve_curvature(candidate))
        dist_inner, _ = inner_tree.query(candidate)
        dist_outer, _ = outer_tree.query(candidate)
        clearance = np.minimum(dist_inner, dist_outer)
        return {
            "max_curvature": float(np.max(curvature)),
            "p95_curvature": float(np.percentile(curvature, 95)),
            "min_clearance": float(np.min(clearance)),
            "p05_clearance": float(np.percentile(clearance, 5)),
            "mean_clearance": float(np.mean(clearance)),
        }

    alpha = float(np.clip(alpha, 0.05, 0.95))
    smoothing = max(0.0, float(smoothing))
    target_max_curvature = max(0.0, float(target_max_curvature))
    if target_max_curvature <= 0.0:
        curvature_limit = CONTROL_FRIENDLY_MAX_CURVATURE
    else:
        curvature_limit = min(target_max_curvature, CONTROL_FRIENDLY_MAX_CURVATURE)
    min_clearance = max(0.0, float(min_clearance))

    if auto_alpha:
        # This is still a control-test line, not an aggressive racing line.
        # The upper alpha bound gives the generator enough room to satisfy the
        # hard curvature cap on tight indoor loops.
        alpha_candidates = np.unique(
            np.round(
                np.concatenate(
                    [
                        np.linspace(0.50, 0.70, 11),
                        np.array([alpha], dtype=float),
                    ]
                ),
                3,
            )
        )
        smoothing_candidates = np.unique(
            np.round(
                np.array([0.0, 1.0, smoothing, 3.0, 5.0, 8.0, 10.0], dtype=float),
                3,
            )
        )
    else:
        alpha_candidates = np.array([alpha], dtype=float)
        smoothing_candidates = np.array([smoothing], dtype=float)

    candidates: list[tuple[float, float, np.ndarray, dict[str, float]]] = []
    for alpha_candidate in alpha_candidates:
        base = inner_xy + float(alpha_candidate) * (outer_xy - inner_xy)
        for smoothing_candidate in smoothing_candidates:
            path = smooth_closed_trajectory(base, float(smoothing_candidate))
            metrics = evaluate(path)
            candidates.append(
                (float(alpha_candidate), float(smoothing_candidate), path, metrics)
            )

    curvature_valid = [
        candidate
        for candidate in candidates
        if candidate[3]["max_curvature"] <= curvature_limit + CURVATURE_TOLERANCE
    ]
    if not curvature_valid:
        best = min(candidates, key=lambda candidate: candidate[3]["max_curvature"])
        raise ValueError(
            "control_friendly trajectory cannot satisfy max curvature "
            f"{curvature_limit:.3f} 1/m; best candidate has "
            f"{best[3]['max_curvature']:.4f} 1/m"
        )

    valid = [
        candidate
        for candidate in curvature_valid
        if candidate[3]["min_clearance"] >= min_clearance
    ]
    pool = valid if valid else curvature_valid

    def candidate_score(
        candidate: tuple[float, float, np.ndarray, dict[str, float]],
    ) -> tuple[float, float, float]:
        alpha_candidate, smoothing_candidate, _, metrics = candidate
        clearance_penalty = max(0.0, min_clearance - metrics["min_clearance"])
        # Prefer modest outer bias after satisfying feasibility; too much outer
        # bias leaves less room for controller recovery at lap wrap.
        return (
            10.0 * clearance_penalty + metrics["max_curvature"],
            abs(alpha_candidate - alpha),
            smoothing_candidate,
        )

    chosen_alpha, chosen_smoothing, chosen_path, chosen_metrics = min(pool, key=candidate_score)
    chosen_metrics = dict(chosen_metrics)
    chosen_metrics["alpha"] = float(chosen_alpha)
    chosen_metrics["spline_smoothing"] = float(chosen_smoothing)
    chosen_metrics["auto_alpha"] = float(1.0 if auto_alpha else 0.0)
    chosen_metrics["curvature_limit"] = float(curvature_limit)
    return chosen_path, chosen_metrics


class GlobalTrajectoryPlanner(Node):
    """ROS 2 全局轨迹规划器，计算并锁存 (Latch) 轨迹路径。"""

    def __init__(self):
        """初始化节点、声明参数并执行轨迹规划。"""
        super().__init__("global_trajectory_planner")

        # 声明 ROS 2 参数
        output_root = get_default_output_root()
        default_track = str(output_root / "csv" / "processed_track.csv")
        self.declare_parameter("track_csv", default_track)
        self.declare_parameter("use_csv_topic", True)
        self.declare_parameter("csv_topic", "/pipeline/track_csv")
        self.declare_parameter("epsilon", 1.0)
        self.declare_parameter("vehicle_length", 0.535)
        self.declare_parameter("vehicle_width", 0.281)
        self.declare_parameter("wheelbase", 0.324)
        self.declare_parameter("safety_margin", 0.10)
        self.declare_parameter("max_iter", 50)
        self.declare_parameter("gamma_normal", 0.5)
        self.declare_parameter("gamma_inaccurate", 0.1)
        self.declare_parameter("normalize_objective", True)
        self.declare_parameter("trajectory_mode", "min_curvature")
        self.declare_parameter("trajectory_spline_smoothing", 4.0)
        self.declare_parameter("centerline_spline_smoothing", 5.0)
        self.declare_parameter("control_friendly_alpha", 0.56)
        self.declare_parameter("control_friendly_auto_alpha", True)
        self.declare_parameter("control_friendly_spline_smoothing", 2.0)
        self.declare_parameter("control_friendly_target_max_curvature", 1.0)
        self.declare_parameter("control_friendly_min_clearance", 0.35)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("path_topic", "/global_trajectory")
        self.declare_parameter("path_republish_period", 0.5)
        self.declare_parameter(
            "output_trajectory_csv",
            str(output_root / "csv" / "global_trajectory.csv"),
        )

        # 获取参数值
        track_csv = self.get_parameter("track_csv").value
        use_csv_topic = bool(self.get_parameter("use_csv_topic").value)
        csv_topic = str(self.get_parameter("csv_topic").value)
        epsilon = float(self.get_parameter("epsilon").value)
        vehicle_length = float(self.get_parameter("vehicle_length").value)
        vehicle_width = float(self.get_parameter("vehicle_width").value)
        wheelbase = float(self.get_parameter("wheelbase").value)
        safety_margin = float(self.get_parameter("safety_margin").value)
        max_iter = int(self.get_parameter("max_iter").value)
        gamma_normal = float(self.get_parameter("gamma_normal").value)
        gamma_inaccurate = float(self.get_parameter("gamma_inaccurate").value)
        normalize_objective = bool(self.get_parameter("normalize_objective").value)
        trajectory_mode = str(self.get_parameter("trajectory_mode").value)
        trajectory_spline_smoothing = float(
            self.get_parameter("trajectory_spline_smoothing").value
        )
        centerline_spline_smoothing = float(
            self.get_parameter("centerline_spline_smoothing").value
        )
        control_friendly_alpha = float(
            self.get_parameter("control_friendly_alpha").value
        )
        control_friendly_auto_alpha = bool(
            self.get_parameter("control_friendly_auto_alpha").value
        )
        control_friendly_spline_smoothing = float(
            self.get_parameter("control_friendly_spline_smoothing").value
        )
        control_friendly_target_max_curvature = float(
            self.get_parameter("control_friendly_target_max_curvature").value
        )
        control_friendly_min_clearance = float(
            self.get_parameter("control_friendly_min_clearance").value
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
        path_topic = str(self.get_parameter("path_topic").value)
        path_republish_period = float(self.get_parameter("path_republish_period").value)
        self.output_trajectory_csv = Path(
            str(self.get_parameter("output_trajectory_csv").value)
        ).resolve()
        self.path_msg: PathMsg | None = None
        self.last_csv_signature: tuple[str, str] | None = None
        self.epsilon = epsilon
        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width
        self.wheelbase = wheelbase
        self.safety_margin = safety_margin
        self.max_iter = max_iter
        self.gamma_normal = gamma_normal
        self.gamma_inaccurate = gamma_inaccurate
        self.normalize_objective = normalize_objective
        if trajectory_mode not in ("min_curvature", "centerline", "control_friendly"):
            raise ValueError(
                "trajectory_mode must be 'min_curvature', 'centerline', or "
                "'control_friendly', "
                f"got {trajectory_mode!r}"
            )
        self.trajectory_mode = trajectory_mode
        self.trajectory_spline_smoothing = trajectory_spline_smoothing
        self.centerline_spline_smoothing = centerline_spline_smoothing
        self.control_friendly_alpha = control_friendly_alpha
        self.control_friendly_auto_alpha = control_friendly_auto_alpha
        self.control_friendly_spline_smoothing = control_friendly_spline_smoothing
        self.control_friendly_target_max_curvature = control_friendly_target_max_curvature
        self.control_friendly_min_clearance = control_friendly_min_clearance

        # 设置 QoS 以支持 Transient Local (Latched)
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(PathMsg, path_topic, qos_profile)
        self.csv_sub = None
        if path_republish_period > 0.0:
            self.path_timer = self.create_timer(path_republish_period, self.republish_trajectory)
        else:
            self.path_timer = None

        if use_csv_topic:
            self.csv_sub = self.create_subscription(
                String,
                csv_topic,
                self.csv_callback,
                qos_profile,
            )
            self.get_logger().info(f"等待赛道 CSV 话题: {csv_topic}")
        elif os.path.exists(track_csv):
            self.plan_from_csv(Path(track_csv).resolve())
        else:
            self.get_logger().error(f"赛道 CSV 不存在: {track_csv}")
            raise FileNotFoundError(track_csv)

    def publish_trajectory(self, r_opt: np.ndarray) -> None:
        """将优化的轨迹点封装为 Path 消息并发布。

        计算每个点的切向角度，并填充到 PoseStamped 中。

        Args:
            r_opt: 优化后的轨迹点数组 [N, 2]。
        """
        path_msg = PathMsg()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id

        for idx, pt in enumerate(r_opt):
            # 使用前后点计算切向角
            next_pt = r_opt[(idx + 1) % len(r_opt)]
            prev_pt = r_opt[idx - 1]
            yaw = math.atan2(next_pt[1] - prev_pt[1], next_pt[0] - prev_pt[0])
            qz, qw = yaw_to_quaternion(yaw)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)

        self.path_msg = path_msg
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"已发布 {len(r_opt)} 个轨迹点到主题: {self.path_pub.topic_name}")

    def save_trajectory_csv(self, r_opt: np.ndarray) -> None:
        """Save the optimized trajectory to disk for inspection."""
        self.output_trajectory_csv.parent.mkdir(parents=True, exist_ok=True)
        with self.output_trajectory_csv.open("w", encoding="utf-8") as f:
            f.write("x,y,yaw\n")
            for idx, pt in enumerate(r_opt):
                next_pt = r_opt[(idx + 1) % len(r_opt)]
                prev_pt = r_opt[idx - 1]
                yaw = math.atan2(next_pt[1] - prev_pt[1], next_pt[0] - prev_pt[0])
                f.write(f"{float(pt[0]):.7f},{float(pt[1]):.7f},{yaw:.7f}\n")

    def csv_callback(self, msg: String) -> None:
        """Trigger planning when the CSV-export stage announces a new file."""
        csv_path: str | None = None
        try:
            payload = metadata_from_json(msg.data)
            csv_value = payload.get("csv_path")
            if isinstance(csv_value, str):
                csv_path = csv_value
        except Exception:
            csv_path = msg.data.strip()

        if not csv_path:
            self.get_logger().warning("收到空的 CSV 消息，已忽略。")
            return
        self.plan_from_csv(Path(csv_path).resolve())

    def plan_from_csv(self, track_csv: Path) -> None:
        """Run the optimizer from the provided track CSV and publish a path."""
        track_csv = track_csv.resolve()
        if not track_csv.exists():
            self.get_logger().error(f"赛道 CSV 不存在: {track_csv}")
            return
        csv_signature = (
            str(track_csv),
            hashlib.sha1(track_csv.read_bytes()).hexdigest(),
        )
        if csv_signature == self.last_csv_signature:
            return

        self.last_csv_signature = csv_signature
        self.get_logger().info(f"正在读取赛道 CSV: {track_csv}")
        p, _, v = load_track_data(track_csv)
        inner_xy = p
        outer_xy = p + v

        if self.trajectory_mode == "centerline":
            centerline = inner_xy + 0.5 * (outer_xy - inner_xy)
            r_publish = smooth_closed_trajectory(centerline, self.centerline_spline_smoothing)
            published_curvature = estimate_closed_curve_curvature(r_publish)
            self.get_logger().info(
                "使用高度平滑中心线作为控制测试轨迹: "
                f"centerline_spline_smoothing={self.centerline_spline_smoothing:g}, "
                f"发布曲率max={float(np.max(published_curvature)):.4f}"
            )
            self.save_trajectory_csv(r_publish)
            self.get_logger().info("中心线轨迹已生成，正在发布全局轨迹数据...")
            self.publish_trajectory(r_publish)
            return

        if self.trajectory_mode == "control_friendly":
            try:
                r_publish, metrics = build_control_friendly_trajectory(
                    inner_xy,
                    outer_xy,
                    alpha=self.control_friendly_alpha,
                    smoothing=self.control_friendly_spline_smoothing,
                    auto_alpha=self.control_friendly_auto_alpha,
                    target_max_curvature=self.control_friendly_target_max_curvature,
                    min_clearance=self.control_friendly_min_clearance,
                )
            except ValueError as exc:
                self.get_logger().error(str(exc))
                return
            self.get_logger().info(
                "使用控制友好平滑中心线轨迹: "
                f"alpha={metrics['alpha']:.3f}, "
                f"spline_smoothing={metrics['spline_smoothing']:g}, "
                f"auto_alpha={bool(metrics['auto_alpha'])}, "
                f"curvature_limit={metrics['curvature_limit']:.3f}, "
                f"max_curvature={metrics['max_curvature']:.4f}, "
                f"p95_curvature={metrics['p95_curvature']:.4f}, "
                f"min_clearance={metrics['min_clearance']:.4f}, "
                f"p05_clearance={metrics['p05_clearance']:.4f}"
            )
            self.save_trajectory_csv(r_publish)
            self.get_logger().info("控制友好轨迹已生成，正在发布全局轨迹数据...")
            self.publish_trajectory(r_publish)
            return

        self.get_logger().info(f"开始计算最小曲率轨迹 (epsilon={self.epsilon:g})...")
        r_opt, alpha_opt = optimize_trajectory(
            p,
            v,
            self.vehicle_length,
            self.vehicle_width,
            self.safety_margin,
            epsilon=self.epsilon,
            max_iter=self.max_iter,
            gamma_normal=self.gamma_normal,
            gamma_inaccurate=self.gamma_inaccurate,
            normalize_objective=self.normalize_objective,
        )
        metrics = analyze_trajectory_candidate(
            p,
            v,
            r_opt,
            alpha_opt,
            self.vehicle_length,
            self.vehicle_width,
            self.safety_margin,
            wheelbase=self.wheelbase,
        )
        r_publish = smooth_closed_trajectory(r_opt, self.trajectory_spline_smoothing)
        smoothed_curvature = estimate_closed_curve_curvature(r_publish)

        steering = metrics["max_required_steering_deg"]
        steering_text = "" if steering is None else f", max_steering={float(steering):.2f}deg"
        self.get_logger().info(
            f"轨迹优化完成: epsilon={self.epsilon:g}, 最小边距={float(metrics['min_body_to_wall']):.4f}m, "
            f"raw最大曲率={float(metrics['max_curvature']):.4f}, "
            f"发布曲率max={float(np.max(smoothed_curvature)):.4f}, "
            f"spline_smoothing={self.trajectory_spline_smoothing:g}{steering_text}"
        )
        self.save_trajectory_csv(r_publish)
        self.get_logger().info("优化完成，正在发布全局轨迹数据...")
        self.publish_trajectory(r_publish)

    def republish_trajectory(self) -> None:
        """周期性重发全局轨迹，方便 RViz 等后启动订阅者可见。"""
        if self.path_msg is None:
            return
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        for pose in self.path_msg.poses:
            pose.header = self.path_msg.header
        self.path_pub.publish(self.path_msg)


def main(args=None):
    """主函数。"""
    rclpy.init(args=args)
    planner = None
    try:
        planner = GlobalTrajectoryPlanner()
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        if planner is not None:
            planner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

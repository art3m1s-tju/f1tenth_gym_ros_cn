"""用于 LQR 参数评估的纯 Python 仿真封装，不依赖 ROS。"""
from __future__ import annotations

import csv
import math
import sys
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.spatial import KDTree

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pnc_rc.lqr.math import (
    compute_lqr_steering,
    compute_path_headings,
    compute_path_curvatures,
    wrap_angle,
)
from pnc_rc.lqr.geometry import (
    advance_projection_along_path,
    compute_curvature_limited_speed,
    project_to_path,
)
from lqr_sweep.lookup_table import LqrParams


@dataclass
class SimConfig:
    """单次仿真的基础配置。

    Attributes:
        map_path: F1TENTH Gym 地图路径，不包含扩展名。
        trajectory_csv: 全局参考轨迹 CSV 路径。
        map_ext: 地图文件扩展名。
        wheelbase: 车辆轴距，单位为米。
        max_steering_angle: 最大前轮转角，单位为弧度。
        max_lateral_accel: 限速计算使用的最大横向加速度。
        min_speed: 控制器允许的最低速度。
        enable_curvature_speed_limit: 是否启用曲率限速。
        enable_speed_ramp: 是否对速度命令施加加减速斜坡。
        max_accel: 最大加速度，单位 m/s^2。
        max_decel: 最大减速度，单位 m/s^2。
        physics_dt: 仿真积分时间步长。
        max_sim_time: 单次仿真的最大允许时间。
        lap_count: 需要连续完成的圈数。
        lqr_min_model_speed: LQR 线性模型使用的最低速度。
        lqr_lookahead_distance_m: LQR 控制误差使用的前向预瞄距离。
        steering_delay_steps: 转向执行延迟步数；1 表示当前周期执行上一周期的转向命令。
        position_noise_std: 位置观测高斯噪声标准差，单位为米。
        heading_noise_std: 航向观测高斯噪声标准差，单位为弧度。
    """

    map_path: str
    trajectory_csv: str
    map_ext: str = ".pgm"
    wheelbase: float = 0.3302
    max_steering_angle: float = 0.36
    max_lateral_accel: float = 2.0
    min_speed: float = 0.25
    enable_curvature_speed_limit: bool = True
    enable_speed_ramp: bool = True
    max_accel: float = 1.0
    max_decel: float = 2.0
    physics_dt: float = 0.02
    max_sim_time: float = 120.0
    lap_count: int = 5
    lqr_min_model_speed: float = 0.25
    lqr_lookahead_distance_m: float = 0.0
    steering_delay_steps: int = 1
    position_noise_std: float = 0.005
    heading_noise_std: float = 0.005


@dataclass
class SimResult:
    """单次仿真的结果。

    Attributes:
        success: 是否成功完成一圈。
        lap_time: 完成目标圈数所用仿真时间，单位为秒。
        metrics: 评估目标函数所需的性能指标。
        failure_reason: 失败时的人类可读原因。
    """

    success: bool
    lap_time: float = 0.0
    metrics: dict = field(default_factory=dict)
    failure_reason: str = ""


@dataclass
class TrajectoryCache:
    """预计算轨迹数据，用于减少重复 I/O 和几何计算。

    Attributes:
        points: 参考轨迹二维点数组，形状为 ``(N, 2)``。
        yaws: CSV 中读取的参考航向角数组。
        headings: 由轨迹点计算出的路径切向航向角数组。
        curvatures: 由轨迹点计算出的路径曲率数组。
        segment_lengths: 每个闭环路径段的长度。
        kdtree: 用于最近邻投影查询的 KDTree。
        path_length: 轨迹总长度估计值。
    """

    points: np.ndarray
    yaws: np.ndarray
    headings: np.ndarray
    curvatures: np.ndarray
    segment_lengths: np.ndarray
    kdtree: KDTree
    path_length: float


def load_trajectory_cache(csv_path: str) -> TrajectoryCache:
    """读取参考轨迹 CSV 并构建可复用缓存。

    Args:
        csv_path: 包含 ``x``、``y`` 和 ``yaw`` 列的参考轨迹 CSV 路径。

    Returns:
        包含轨迹点、航向、曲率和 KDTree 的缓存对象。
    """

    points = []
    yaws = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            points.append([float(row["x"]), float(row["y"])])
            yaws.append(float(row["yaw"]))
    pts = np.array(points, dtype=float)
    yw = np.array(yaws, dtype=float)
    segment_lengths = np.linalg.norm(np.roll(pts, -1, axis=0) - pts, axis=1)
    path_length = float(np.sum(segment_lengths))
    return TrajectoryCache(
        points=pts,
        yaws=yw,
        headings=compute_path_headings(pts, closed_loop=True),
        curvatures=compute_path_curvatures(pts, closed_loop=True),
        segment_lengths=segment_lengths,
        kdtree=KDTree(pts),
        path_length=path_length,
    )


def run_single_sim(
    config: SimConfig,
    params: LqrParams,
    target_speed: float,
    trajectory_cache: TrajectoryCache | None = None,
) -> SimResult:
    """在 f1tenth_gym 环境中运行一次 LQR 多圈跟踪仿真。

    Args:
        config: 仿真环境和车辆参数配置。
        params: 本次仿真使用的 LQR 参数。
        target_speed: 目标巡航速度，单位为米/秒。
        trajectory_cache: 可选的预计算轨迹缓存；未提供时从配置路径加载。

    Returns:
        成功时包含目标圈数总耗时和性能指标；失败时包含失败原因。
    """

    try:
        import gym
    except ImportError:
        return SimResult(success=False, failure_reason="f1tenth_gym not installed")

    if trajectory_cache is None:
        trajectory_cache = load_trajectory_cache(config.trajectory_csv)

    points = trajectory_cache.points
    headings = trajectory_cache.headings
    curvatures = trajectory_cache.curvatures
    segment_lengths = trajectory_cache.segment_lengths
    kdtree = trajectory_cache.kdtree
    path_length = trajectory_cache.path_length

    start_x, start_y = float(points[0, 0]), float(points[0, 1])
    start_yaw = float(trajectory_cache.yaws[0])

    try:
        env = gym.make(
            "f110_gym:f110-v0",
            map=config.map_path,
            map_ext=config.map_ext,
            num_agents=1,
        )
        obs, _, done, _ = env.reset(np.array([[start_x, start_y, start_yaw]]))
    except Exception as e:
        return SimResult(success=False, failure_reason=f"env init failed: {e}")

    dt = config.physics_dt
    cumulative_dist = 0.0
    completed_laps = 0
    required_laps = max(1, int(config.lap_count))
    prev_pos = np.array([start_x, start_y])
    lap_threshold = path_length * 0.80
    finish_radius = 0.5

    # 转向延迟缓存：近似模拟 CAN 总线和舵机执行延迟。
    # 延迟 N 步表示实际执行 N 个控制周期之前发出的转向命令。
    steering_buffer = deque(
        [0.0] * config.steering_delay_steps,
        maxlen=config.steering_delay_steps,
    )

    # 固定随机种子，使观测噪声在参数扫描中可复现。
    rng = np.random.default_rng(seed=42)

    lateral_errors = []
    heading_errors_deg = []
    steering_cmds = []
    speed_errors = []
    sim_time = 0.0
    current_speed_cmd = 0.0

    try:
        while sim_time < config.max_sim_time:
            x = float(obs["poses_x"][0])
            y = float(obs["poses_y"][0])
            theta = float(obs["poses_theta"][0])
            vx = float(obs["linear_vels_x"][0])
            vy = float(obs["linear_vels_y"][0])
            v_actual = math.hypot(vx, vy)

            # 给控制器观测加入噪声，但保持仿真器真实状态不变。
            x_noisy = x + rng.normal(0.0, config.position_noise_std)
            y_noisy = y + rng.normal(0.0, config.position_noise_std)
            theta_noisy = theta + rng.normal(0.0, config.heading_noise_std)

            position = np.array([x, y])
            step_dist = float(np.linalg.norm(position - prev_pos))
            cumulative_dist += step_dist
            prev_pos = position

            next_lap_distance = (completed_laps + 1) * lap_threshold
            if cumulative_dist > next_lap_distance:
                if math.hypot(x - start_x, y - start_y) < finish_radius:
                    completed_laps += 1
                    if completed_laps >= required_laps:
                        break

            noisy_position = np.array([x_noisy, y_noisy])
            control_proj = project_to_path(noisy_position, points, kdtree, headings, curvatures)
            control_proj = advance_projection_along_path(
                position=noisy_position,
                projection=control_proj,
                lookahead_distance=config.lqr_lookahead_distance_m,
                points=points,
                headings=headings,
                curvatures=curvatures,
                segment_lengths=segment_lengths,
            )
            control_heading_error = wrap_angle(theta_noisy - control_proj.heading)

            truth_proj = project_to_path(position, points, kdtree, headings, curvatures)
            truth_heading_error = wrap_angle(theta - truth_proj.heading)

            try:
                delta_raw, _, _, _ = compute_lqr_steering(
                    lateral_error=control_proj.lateral_error,
                    heading_error=control_heading_error,
                    curvature_ref=control_proj.curvature,
                    speed=max(v_actual, target_speed),
                    wheelbase=config.wheelbase,
                    dt=dt,
                    min_model_speed=config.lqr_min_model_speed,
                    q_lateral=params.q_lateral,
                    q_heading=params.q_heading,
                    r_steering=params.r_steering,
                    feedforward_gain=params.feedforward_gain,
                )
            except Exception:
                return SimResult(success=False, failure_reason="DARE solver failed")

            delta_cmd = float(np.clip(delta_raw, -config.max_steering_angle, config.max_steering_angle))
            if config.enable_curvature_speed_limit:
                speed_cmd = compute_curvature_limited_speed(
                    target_speed, control_proj.curvature, delta_cmd,
                    config.wheelbase, config.max_lateral_accel, config.min_speed,
                )
            else:
                speed_cmd = target_speed

            if config.enable_speed_ramp:
                if speed_cmd >= current_speed_cmd:
                    current_speed_cmd = min(speed_cmd, current_speed_cmd + config.max_accel * dt)
                else:
                    current_speed_cmd = max(speed_cmd, current_speed_cmd - config.max_decel * dt)
                speed_cmd = current_speed_cmd

            # 应用转向执行延迟。
            if config.steering_delay_steps > 0:
                delta_delayed = steering_buffer[0]
                steering_buffer.append(delta_cmd)
            else:
                delta_delayed = delta_cmd

            lateral_errors.append(abs(truth_proj.lateral_error))
            heading_errors_deg.append(abs(math.degrees(truth_heading_error)))
            steering_cmds.append(math.degrees(delta_cmd))
            speed_errors.append(abs(v_actual - speed_cmd))

            obs, _, done, _ = env.step(np.array([[delta_delayed, speed_cmd]]))
            sim_time += dt

            if done:
                expected_lap_time = path_length / max(target_speed, config.min_speed, 1e-3)
                enough_time_for_laps = sim_time >= 0.75 * required_laps * expected_lap_time
                if (
                    enough_time_for_laps
                    and math.hypot(x - start_x, y - start_y) < finish_radius
                    and abs(truth_proj.lateral_error) < 0.10
                ):
                    completed_laps = required_laps
                    break
                return SimResult(
                    success=False,
                    failure_reason=(
                        "collision "
                        f"t={sim_time:.2f}s x={x:.2f} y={y:.2f} "
                        f"e_y={truth_proj.lateral_error:.3f} "
                        f"e_psi={math.degrees(truth_heading_error):.2f}deg "
                        f"idx={truth_proj.closest_idx}"
                    ),
                )
    finally:
        env.close()

    if sim_time >= config.max_sim_time:
        return SimResult(success=False, failure_reason="timeout")

    lat_arr = np.array(lateral_errors)
    head_arr = np.array(heading_errors_deg)
    steer_arr = np.array(steering_cmds)
    speed_arr = np.array(speed_errors)
    steer_rate = np.abs(np.diff(steer_arr)) / dt
    steering_limit_deg = math.degrees(config.max_steering_angle)
    steering_sat_mask = np.abs(steer_arr) >= 0.9 * steering_limit_deg

    metrics = {
        "completed_laps": float(completed_laps),
        "mean_abs_e_y": float(np.mean(lat_arr)),
        "p95_abs_e_y": float(np.percentile(lat_arr, 95)),
        "max_abs_e_y": float(np.max(lat_arr)),
        "mean_abs_e_psi_deg": float(np.mean(head_arr)),
        "p95_abs_e_psi_deg": float(np.percentile(head_arr, 95)),
        "steering_rms_deg": float(np.sqrt(np.mean(steer_arr ** 2))),
        "steering_rate_rms_deg_s": float(np.sqrt(np.mean(steer_rate ** 2))) if len(steer_rate) > 0 else 0.0,
        "steering_saturation_count": int(np.count_nonzero(steering_sat_mask)),
        "steering_saturation_ratio": float(np.mean(steering_sat_mask)) if len(steering_sat_mask) > 0 else 0.0,
        "speed_rms_error": float(np.sqrt(np.mean(speed_arr ** 2))),
    }

    return SimResult(success=True, lap_time=sim_time, metrics=metrics)

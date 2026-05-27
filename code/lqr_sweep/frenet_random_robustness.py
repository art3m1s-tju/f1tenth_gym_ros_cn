#!/usr/bin/env python3
"""Frenet 静态障碍避障批量鲁棒性测试入口。

该脚本在同一条赛道上按 seed 随机生成障碍物地图，启动仿真并从 LQR tracking
日志与 launch 日志中提取碰撞、卡住、完成圈数等指标。`--mode batch` 用于
自动统计成功率，`--mode rviz` 用于打开同一套参数的可视化单 seed 调试。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lqr_sweep.generate_static_obstacle_test_map import write_obstacle_map
from pnc_rc.frenet.preset import FrenetPreset
from pnc_rc.frenet.preset import compute_frenet_preset
from pnc_rc.frenet.preset import frenet_static_test_launch_args


CONTAINER_PKG = Path("/sim_ws/src/f1tenth_gym_ros")


@dataclass
class TrackingSummary:
    """从 LQR tracking CSV 中提取的单次试验摘要。

    Attributes:
        completed_laps: 根据车辆实际位置或路径索引估计的完成圈数。
        zero_limit_runs: 局部限速为 0 且持续超过阈值的次数。
        low_speed_stuck_runs: 实际速度和指令速度都很低的卡住次数。
        max_zero_limit_duration_s: 最长 0 限速持续时间，单位 s。
        max_low_speed_duration_s: 最长低速卡住持续时间，单位 s。
        negative_v_path_count: 规划路径速度突然变负的日志行数，通常代表碰撞反弹。
        row_count: tracking CSV 有效行数。
        duration_s: 日志覆盖的仿真时间，单位 s。
        max_v_actual: 实际速度最大值，单位 m/s。
        max_v_cmd: 控制器速度指令最大值，单位 m/s。
        min_local_speed_limit: Frenet 局部限速最小值，单位 m/s。
        max_local_speed_limit: Frenet 局部限速最大值，单位 m/s。
    """

    completed_laps: int = 0
    zero_limit_runs: int = 0
    low_speed_stuck_runs: int = 0
    max_zero_limit_duration_s: float = 0.0
    max_low_speed_duration_s: float = 0.0
    negative_v_path_count: int = 0
    row_count: int = 0
    duration_s: float = 0.0
    max_v_actual: float = 0.0
    max_v_cmd: float = 0.0
    min_local_speed_limit: float = math.nan
    max_local_speed_limit: float = math.nan


@dataclass
class LaunchSummary:
    """从 ROS launch 日志中提取的避障故障计数。

    Attributes:
        ego_collision_count: 仿真器报告 ego collision 的次数。
        no_safe_stop_count: Frenet 无安全候选并发布 stop path 的次数。
        zero_limit_log_count: LQR 收到 0 局部限速日志的次数。
    """

    ego_collision_count: int = 0
    no_safe_stop_count: int = 0
    zero_limit_log_count: int = 0


@dataclass
class TrialResult:
    """单个随机 seed 的完整测试结果。

    Attributes:
        seed: 随机障碍 seed。
        success: 是否在无碰撞/不卡住条件下完成目标圈数。
        failure_reason: 失败原因；成功时为空字符串。
        completed_laps: 本 trial 完成圈数。
        obstacle_count: 本 trial 障碍物数量。
        map_prefix: 生成地图路径前缀，不带扩展名。
        tracking_log: LQR tracking CSV 路径。
        launch_log: ROS launch 日志路径。
        duration_s: tracking 日志覆盖时间，单位 s。
        zero_limit_runs: 0 限速卡住次数。
        low_speed_stuck_runs: 低速卡住次数。
        max_zero_limit_duration_s: 最长 0 限速持续时间，单位 s。
        max_low_speed_duration_s: 最长低速卡住持续时间，单位 s。
        negative_v_path_count: 速度符号异常行数。
        ego_collision_count: 仿真碰撞次数。
        no_safe_stop_count: Frenet stop fallback 次数。
        max_v_actual: 实际速度最大值，单位 m/s。
        max_v_cmd: 指令速度最大值，单位 m/s。
    """

    seed: int
    success: bool
    failure_reason: str
    completed_laps: int
    obstacle_count: int
    map_prefix: str
    tracking_log: str
    launch_log: str
    duration_s: float
    zero_limit_runs: int
    low_speed_stuck_runs: int
    max_zero_limit_duration_s: float
    max_low_speed_duration_s: float
    negative_v_path_count: int
    ego_collision_count: int
    no_safe_stop_count: int
    max_v_actual: float
    max_v_cmd: float


def _float_value(row: dict[str, str], key: str, default: float = math.nan) -> float:
    """安全读取 CSV 行中的浮点数。

    Args:
        row: `csv.DictReader` 读出的单行。
        key: 目标字段名。
        default: 字段缺失、空字符串或无法转换时返回的默认值。

    Returns:
        解析出的浮点值，失败时返回 `default`。
    """
    try:
        value = row.get(key, "")
        return float(value) if value != "" else default
    except (TypeError, ValueError):
        return default


def count_laps_from_tracking(
    rows: list[dict[str, str]],
    start_x: float,
    start_y: float,
    threshold_m: float = 0.5,
) -> int:
    """基于 tracking CSV 的 `closest_idx` 粗略估计完成圈数。

    Args:
        rows: tracking CSV 全部行。
        start_x: 起点 x 坐标，单位 m。
        start_y: 起点 y 坐标，单位 m。
        threshold_m: 通过起点附近才计入换圈的距离阈值，单位 m。

    Returns:
        估计完成圈数。日志太短或缺少索引时返回 0。
    """
    if len(rows) < 500:
        return 0
    indices: list[int] = []
    for row in rows:
        idx = _float_value(row, "closest_idx")
        if math.isfinite(idx):
            indices.append(int(idx))
    if not indices:
        return 0

    max_path_idx = max(indices)
    high_idx = 0.75 * max_path_idx
    low_idx = 0.25 * max_path_idx
    laps = 0
    previous_idx: int | None = None
    for row, idx in zip(rows, indices):
        if previous_idx is not None and previous_idx > high_idx and idx < low_idx:
            x = _float_value(row, "x")
            y = _float_value(row, "y")
            if math.isfinite(x) and math.isfinite(y):
                if math.hypot(x - start_x, y - start_y) < threshold_m:
                    laps += 1
        previous_idx = idx
    return laps


def load_reference_xy(path: Path) -> np.ndarray:
    """读取参考轨迹 CSV 的二维坐标。

    Args:
        path: 参考轨迹 CSV 路径，支持 `x/y` 或 `pos_x/pos_y` 字段。

    Returns:
        形状为 `(N, 2)` 的参考轨迹点数组。

    Raises:
        ValueError: 参考点数量少于 3。
    """
    points: list[tuple[float, float]] = []
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if "x" in row and "y" in row:
                points.append((float(row["x"]), float(row["y"])))
            else:
                points.append((float(row["pos_x"]), float(row["pos_y"])))
    if len(points) < 3:
        raise ValueError(f"reference path needs at least 3 points: {path}")
    return np.asarray(points, dtype=float)


def count_laps_from_positions(rows: list[dict[str, str]], reference_xy: np.ndarray) -> int:
    """基于车辆实际位置和全局参考线估计完成圈数。

    Args:
        rows: tracking CSV 全部行。
        reference_xy: 全局参考线二维坐标数组。

    Returns:
        按参考线最近点 unwrap 后得到的完成圈数。
    """
    if len(rows) < 100 or len(reference_xy) < 3:
        return 0
    positions: list[tuple[float, float]] = []
    for row in rows:
        x = _float_value(row, "x")
        y = _float_value(row, "y")
        if math.isfinite(x) and math.isfinite(y):
            positions.append((x, y))
    if len(positions) < 100:
        return 0

    ref = np.asarray(reference_xy, dtype=float)
    ref_count = len(ref)
    nearest_indices: list[int] = []
    for position in positions:
        delta = ref - np.asarray(position, dtype=float)
        nearest_indices.append(int(np.argmin(np.einsum("ij,ij->i", delta, delta))))

    wraps = 0
    previous_idx = nearest_indices[0]
    start_unwrapped = previous_idx
    max_unwrapped = start_unwrapped
    half_count = 0.5 * ref_count
    for idx in nearest_indices[1:]:
        delta_idx = idx - previous_idx
        if delta_idx < -half_count:
            wraps += 1
        elif delta_idx > half_count:
            wraps -= 1
        unwrapped_idx = idx + wraps * ref_count
        max_unwrapped = max(max_unwrapped, unwrapped_idx)
        previous_idx = idx

    progress_laps = (max_unwrapped - start_unwrapped) / float(ref_count)
    return max(0, int(math.floor(progress_laps + 1e-6)))


def _count_runs(
    rows: list[dict[str, str]],
    predicate,
    min_duration_s: float,
) -> int:
    """统计满足谓词且持续超过阈值的连续片段数量。

    Args:
        rows: tracking CSV 全部行。
        predicate: 接收 `(row, t, start_time)` 并返回是否处于故障状态的函数。
        min_duration_s: 计为一次 run 的最短持续时间，单位 s。

    Returns:
        连续故障片段数量。
    """
    if not rows:
        return 0
    start_time = _float_value(rows[0], "time", 0.0)
    active_start: float | None = None
    active_end = start_time
    runs = 0
    for row in rows:
        t = _float_value(row, "time")
        if not math.isfinite(t):
            continue
        is_active = predicate(row, t, start_time)
        if is_active and active_start is None:
            active_start = t
        if is_active:
            active_end = t
        if not is_active and active_start is not None:
            if active_end - active_start >= min_duration_s:
                runs += 1
            active_start = None
    if active_start is not None and active_end - active_start >= min_duration_s:
        runs += 1
    return runs


def _max_run_duration(rows: list[dict[str, str]], predicate) -> float:
    """计算满足谓词的最长连续片段时长。

    Args:
        rows: tracking CSV 全部行。
        predicate: 接收 `(row, t, start_time)` 并返回是否处于故障状态的函数。

    Returns:
        最长连续时长，单位 s。
    """
    if not rows:
        return 0.0
    start_time = _float_value(rows[0], "time", 0.0)
    active_start: float | None = None
    active_end = start_time
    max_duration = 0.0
    for row in rows:
        t = _float_value(row, "time")
        if not math.isfinite(t):
            continue
        is_active = predicate(row, t, start_time)
        if is_active and active_start is None:
            active_start = t
        if is_active:
            active_end = t
        if not is_active and active_start is not None:
            max_duration = max(max_duration, active_end - active_start)
            active_start = None
    if active_start is not None:
        max_duration = max(max_duration, active_end - active_start)
    return max_duration


def summarize_tracking_log(
    log_path: Path,
    start_x: float,
    start_y: float,
    reference_xy: np.ndarray | None = None,
) -> TrackingSummary:
    """汇总单个 trial 的 tracking CSV。

    Args:
        log_path: tracking CSV 路径。
        start_x: 起点 x 坐标，单位 m。
        start_y: 起点 y 坐标，单位 m。
        reference_xy: 可选全局参考线；提供时优先用实际位置估计圈数。

    Returns:
        `TrackingSummary`。日志不存在或为空时返回默认空摘要。
    """
    if not log_path.exists():
        return TrackingSummary()
    with log_path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return TrackingSummary()

    times = [_float_value(row, "time") for row in rows]
    finite_times = [value for value in times if math.isfinite(value)]
    v_actual = [_float_value(row, "v_actual", 0.0) for row in rows]
    v_cmd = [_float_value(row, "v_cmd", 0.0) for row in rows]
    speed_limits = [
        _float_value(row, "local_speed_limit")
        for row in rows
        if math.isfinite(_float_value(row, "local_speed_limit"))
    ]
    zero_limit_predicate = lambda row, t, start: (
        t > start + 1.0 and _float_value(row, "local_speed_limit") == 0.0
    )
    low_speed_predicate = lambda row, t, start: (
        t > start + 2.0
        and abs(_float_value(row, "v_actual", 0.0)) < 0.25
        and abs(_float_value(row, "v_cmd", 0.0)) < 0.35
    )
    zero_limit_runs = _count_runs(rows, zero_limit_predicate, min_duration_s=0.5)
    low_speed_stuck_runs = _count_runs(rows, low_speed_predicate, min_duration_s=1.0)
    negative_v_path_count = sum(
        1 for row in rows if _float_value(row, "v_path_signed", 0.0) < -0.05
    )
    return TrackingSummary(
        completed_laps=(
            count_laps_from_positions(rows, reference_xy)
            if reference_xy is not None
            else count_laps_from_tracking(rows, start_x, start_y)
        ),
        zero_limit_runs=zero_limit_runs,
        low_speed_stuck_runs=low_speed_stuck_runs,
        max_zero_limit_duration_s=_max_run_duration(rows, zero_limit_predicate),
        max_low_speed_duration_s=_max_run_duration(rows, low_speed_predicate),
        negative_v_path_count=negative_v_path_count,
        row_count=len(rows),
        duration_s=(max(finite_times) - min(finite_times)) if finite_times else 0.0,
        max_v_actual=max(v_actual) if v_actual else 0.0,
        max_v_cmd=max(v_cmd) if v_cmd else 0.0,
        min_local_speed_limit=min(speed_limits) if speed_limits else math.nan,
        max_local_speed_limit=max(speed_limits) if speed_limits else math.nan,
    )


def summarize_launch_log(log_path: Path) -> LaunchSummary:
    """统计 launch 日志中的关键故障字符串。

    Args:
        log_path: ROS launch 标准输出日志路径。

    Returns:
        `LaunchSummary`。日志不存在时返回默认空摘要。
    """
    if not log_path.exists():
        return LaunchSummary()
    text = log_path.read_text(encoding="utf-8", errors="replace")
    return LaunchSummary(
        ego_collision_count=text.count("Ego collision detected"),
        no_safe_stop_count=text.count("No safe Frenet candidate; publishing stop path"),
        zero_limit_log_count=text.count("Local trajectory speed limit set to 0.00m/s"),
    )


def build_launch_cmd(
    *,
    repo_root: Path,
    map_prefix: Path,
    track_csv: Path,
    trajectory_csv: Path,
    tracking_log: Path,
    target_speed: float,
    avoidance_speed: float,
    start_x: float,
    start_y: float,
    start_theta: float,
    preset: FrenetPreset,
    enable_rviz: bool = False,
) -> list[str]:
    """构造单次随机障碍测试的 ROS launch 命令。

    Args:
        repo_root: 容器内 package 根目录。
        map_prefix: 随机障碍地图前缀，不带 `.yaml/.pgm` 后缀。
        track_csv: 全局参考赛道 CSV。
        trajectory_csv: 本轮导出的全局轨迹 CSV 路径。
        tracking_log: LQR tracking CSV 日志路径。
        target_speed: LQR 全局巡航目标速度，单位 m/s。
        avoidance_speed: Frenet 避障阶段局部限速，单位 m/s。
        start_x: 初始 x 坐标，单位 m。
        start_y: 初始 y 坐标，单位 m。
        start_theta: 初始航向，单位 rad。
        preset: 由 `compute_frenet_preset()` 生成的 Frenet 参数预设。
        enable_rviz: 是否启动 RViz。

    Returns:
        可直接传给 `subprocess.Popen()` 或 `subprocess.run()` 的命令数组。
    """
    launch_file = repo_root / "launch" / "pnc_sim_launch.py"
    return [
        "ros2",
        "launch",
        str(launch_file),
        f"enable_rviz:={'true' if enable_rviz else 'false'}",
        "enable_frenet_planner:=true",
        f"map_path:={map_prefix}",
        f"track_csv:={track_csv}",
        f"trajectory_csv:={trajectory_csv}",
        "trajectory_mode:=centerline",
        f"sx:={start_x}",
        f"sy:={start_y}",
        f"stheta:={start_theta}",
        f"target_speed:={target_speed}",
        "min_speed:=0.20",
        "max_lateral_accel:=1.5",
        "max_accel:=1.0",
        "max_decel:=2.0",
        "curvature_speed_lookahead_m:=1.5",
        "local_speed_limit_timeout_s:=1.0",
        *frenet_static_test_launch_args(preset, avoidance_speed),
        f"log_path:={tracking_log}",
    ]


def generate_trial_assets(
    args: argparse.Namespace,
    batch_dir: Path,
    seed: int,
) -> tuple[str, Path, Path, Path, Path, dict]:
    """生成单个 seed 的地图、日志目录和输出路径。

    Args:
        args: CLI 参数命名空间。
        batch_dir: 当前批次输出目录。
        seed: 随机障碍 seed。

    Returns:
        `(trial_name, map_prefix, tracking_log, launch_log, trajectory_csv,
        obstacle_summary)`。
    """
    trial_name = f"seed_{seed:03d}"
    trial_dir = batch_dir / "trials" / trial_name
    maps_dir = trial_dir / "map"
    logs_dir = trial_dir / "logs"
    maps_dir.mkdir(parents=True, exist_ok=True)
    logs_dir.mkdir(parents=True, exist_ok=True)

    map_prefix = maps_dir / trial_name
    tracking_log = logs_dir / "tracking.csv"
    launch_log = logs_dir / "launch.log"
    trajectory_csv = logs_dir / "global_trajectory.csv"
    for path in (tracking_log, launch_log, trajectory_csv):
        if path.exists():
            path.unlink()

    obstacle_summary = write_obstacle_map(
        base_yaml=args.base_yaml.resolve(),
        trajectory_csv=args.obstacle_trajectory_csv.resolve(),
        output_prefix=map_prefix.resolve(),
        obstacle_count=max(1, int(args.obstacle_count)),
        obstacle_size_m=max(0.05, float(args.obstacle_size_m)),
        seed=int(seed),
        min_start_distance_m=max(0.0, float(args.min_start_distance_m)),
        min_separation_m=max(0.0, float(args.min_separation_m)),
        obstacle_distances_m=None,
    )
    return trial_name, map_prefix, tracking_log, launch_log, trajectory_csv, obstacle_summary


def terminate_process_group(proc: subprocess.Popen) -> None:
    """停止 ros2 launch 进程组。

    Args:
        proc: 由 `subprocess.Popen()` 启动的 launch 进程。

    Returns:
        None。优先发送 SIGINT，超时后再发送 SIGKILL。
    """
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        proc.wait(timeout=5)
    except ProcessLookupError:
        pass


def failure_reason(
    tracking: TrackingSummary,
    launch: LaunchSummary,
    required_laps: int,
    timed_out: bool,
    tracking_log: Path,
    stuck_duration_s: float,
) -> str:
    """根据日志摘要判定 trial 失败原因。

    Args:
        tracking: tracking CSV 摘要。
        launch: launch 日志摘要。
        required_laps: 成功所需完成圈数。
        timed_out: 是否因超时结束 trial。
        tracking_log: tracking CSV 路径，用于识别无日志失败。
        stuck_duration_s: 判定卡住的最长允许持续时间，单位 s。

    Returns:
        失败原因字符串；成功时返回空字符串。
    """
    if not tracking_log.exists() or tracking.row_count == 0:
        return "no_log"
    if launch.ego_collision_count > 0:
        return "collision"
    if tracking.negative_v_path_count > 0:
        return "negative_velocity"
    if tracking.max_zero_limit_duration_s >= stuck_duration_s:
        return "stuck_zero_speed_limit"
    if tracking.max_low_speed_duration_s >= stuck_duration_s:
        return "low_speed_stuck"
    if tracking.completed_laps < required_laps:
        prefix = "timeout" if timed_out else "incomplete"
        return f"{prefix}_laps_{tracking.completed_laps}_of_{required_laps}"
    return ""


def run_trial(args: argparse.Namespace, batch_dir: Path, seed: int, preset: FrenetPreset) -> TrialResult:
    """运行一个 headless 随机障碍鲁棒性 trial。

    Args:
        args: CLI 参数命名空间。
        batch_dir: 当前批次输出目录。
        seed: 随机障碍 seed。
        preset: Frenet 参数预设。

    Returns:
        `TrialResult`，包含成功/失败原因、圈数和诊断统计。
    """
    trial_name, map_prefix, tracking_log, launch_log, trajectory_csv, obstacle_summary = generate_trial_assets(
        args,
        batch_dir,
        seed,
    )
    reference_xy = load_reference_xy(args.obstacle_trajectory_csv.resolve())

    launch_cmd = build_launch_cmd(
        repo_root=args.repo_root,
        map_prefix=map_prefix.resolve(),
        track_csv=args.track_csv.resolve(),
        trajectory_csv=trajectory_csv.resolve(),
        tracking_log=tracking_log.resolve(),
        target_speed=args.target_speed,
        avoidance_speed=args.avoidance_speed,
        start_x=args.sx,
        start_y=args.sy,
        start_theta=args.stheta,
        preset=preset,
        enable_rviz=False,
    )

    print("\n" + "=" * 72)
    print(f"Trial {trial_name}: map={map_prefix}")
    print(
        "  obstacles="
        + ", ".join(
            f"({obs['center_x_m']:.2f},{obs['center_y_m']:.2f})"
            for obs in obstacle_summary["obstacles"]
        )
    )
    start_time = time.time()
    timed_out = False
    with launch_log.open("w", encoding="utf-8") as log_file:
        proc = subprocess.Popen(
            launch_cmd,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        try:
            while time.time() - start_time < args.timeout:
                time.sleep(args.poll_interval)
                launch_summary = summarize_launch_log(launch_log)
                if launch_summary.ego_collision_count > 0:
                    print("  collision detected; stopping trial")
                    break
                tracking_summary = summarize_tracking_log(
                    tracking_log,
                    args.sx,
                    args.sy,
                    reference_xy,
                )
                if tracking_summary.completed_laps >= args.laps:
                    print(f"  completed {tracking_summary.completed_laps} laps; stopping trial")
                    break
                if proc.poll() is not None:
                    print(f"  launch exited early with code {proc.returncode}")
                    break
            else:
                timed_out = True
                print("  timeout reached; stopping trial")
        finally:
            terminate_process_group(proc)

    tracking = summarize_tracking_log(tracking_log, args.sx, args.sy, reference_xy)
    launch = summarize_launch_log(launch_log)
    reason = failure_reason(
        tracking,
        launch,
        max(1, int(args.laps)),
        timed_out,
        tracking_log,
        args.stuck_duration,
    )
    success = reason == ""
    print(
        f"  result={'PASS' if success else 'FAIL'} "
        f"laps={tracking.completed_laps}/{args.laps} reason={reason or 'ok'}"
    )
    return TrialResult(
        seed=seed,
        success=success,
        failure_reason=reason,
        completed_laps=tracking.completed_laps,
        obstacle_count=int(obstacle_summary["obstacle_count"]),
        map_prefix=str(map_prefix),
        tracking_log=str(tracking_log),
        launch_log=str(launch_log),
        duration_s=tracking.duration_s,
        zero_limit_runs=tracking.zero_limit_runs,
        low_speed_stuck_runs=tracking.low_speed_stuck_runs,
        max_zero_limit_duration_s=tracking.max_zero_limit_duration_s,
        max_low_speed_duration_s=tracking.max_low_speed_duration_s,
        negative_v_path_count=tracking.negative_v_path_count,
        ego_collision_count=launch.ego_collision_count,
        no_safe_stop_count=launch.no_safe_stop_count,
        max_v_actual=tracking.max_v_actual,
        max_v_cmd=tracking.max_v_cmd,
    )


def run_rviz_seed(args: argparse.Namespace, batch_dir: Path, seed: int, preset: FrenetPreset) -> int:
    """生成一个随机 seed 并以前台 RViz 模式启动仿真。

    RViz 模式用于人工观察，不按圈数判定成功/失败。`args.timeout > 0` 时，到时
    自动向 ros2 launch 进程组发送 SIGINT，避免用户每次手动关闭一组 ROS 进程。

    Args:
        args: CLI 参数命名空间。
        batch_dir: 当前批次输出目录。
        seed: 随机障碍 seed。
        preset: Frenet 参数预设。

    Returns:
        `ros2 launch` 进程退出码；若按 timeout 正常停止，返回 0。
    """
    trial_name, map_prefix, tracking_log, _launch_log, trajectory_csv, obstacle_summary = generate_trial_assets(
        args,
        batch_dir,
        seed,
    )
    launch_cmd = build_launch_cmd(
        repo_root=args.repo_root,
        map_prefix=map_prefix.resolve(),
        track_csv=args.track_csv.resolve(),
        trajectory_csv=trajectory_csv.resolve(),
        tracking_log=tracking_log.resolve(),
        target_speed=args.target_speed,
        avoidance_speed=args.avoidance_speed,
        start_x=args.sx,
        start_y=args.sy,
        start_theta=args.stheta,
        preset=preset,
        enable_rviz=True,
    )

    print("\n" + "=" * 72)
    print(f"RViz trial {trial_name}: map={map_prefix}")
    print(f"Tracking log: {tracking_log}")
    print(
        "Obstacles: "
        + ", ".join(
            f"({obs['center_x_m']:.2f},{obs['center_y_m']:.2f})"
            for obs in obstacle_summary["obstacles"]
        )
    )
    print("Launch command:")
    print(" ".join(launch_cmd))
    proc = subprocess.Popen(launch_cmd, preexec_fn=os.setsid)
    if args.timeout <= 0.0:
        return proc.wait()
    start_time = time.time()
    try:
        while time.time() - start_time < args.timeout:
            if proc.poll() is not None:
                return int(proc.returncode)
            time.sleep(args.poll_interval)
        print(f"RViz timeout reached ({args.timeout:.1f}s); stopping trial.")
        terminate_process_group(proc)
        return 0
    except KeyboardInterrupt:
        terminate_process_group(proc)
        return 130


def write_outputs(batch_dir: Path, args: argparse.Namespace, preset: FrenetPreset, results: list[TrialResult]) -> None:
    """写出批量测试的 CSV/JSON 汇总。

    Args:
        batch_dir: 当前批次输出目录。
        args: CLI 参数命名空间。
        preset: 本批次使用的 Frenet 参数预设。
        results: 已完成 trial 的结果列表。

    Returns:
        None。每个 trial 后都会覆盖写出最新汇总，方便长任务中途查看。
    """
    summary_csv = batch_dir / "summary.csv"
    fieldnames = list(asdict(results[0]).keys()) if results else list(TrialResult.__dataclass_fields__.keys())
    with summary_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for result in results:
            writer.writerow(asdict(result))

    successes = sum(1 for result in results if result.success)
    summary_json = batch_dir / "summary.json"
    summary = {
        "success_rate": successes / len(results) if results else 0.0,
        "successful_trials": successes,
        "total_trials": len(results),
        "args": vars(args),
        "preset": asdict(preset),
        "results": [asdict(result) for result in results],
    }
    serializable = json.loads(json.dumps(summary, default=str))
    summary_json.write_text(json.dumps(serializable, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print("\n" + "=" * 72)
    print(f"Robustness summary: {successes}/{len(results)} passed")
    print(f"Success rate: {summary['success_rate']:.3f}")
    print(f"Summary CSV: {summary_csv}")
    print(f"Summary JSON: {summary_json}")


def parse_args() -> argparse.Namespace:
    """解析随机障碍鲁棒性测试 CLI 参数。

    Returns:
        `argparse.Namespace`，包含 batch/rviz 模式、速度、障碍数量、输出路径等。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("batch", "rviz"),
        default="batch",
        help="batch runs scored headless trials; rviz launches one generated seed with RViz",
    )
    parser.add_argument("--repo-root", type=Path, default=CONTAINER_PKG)
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--laps", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--poll-interval", type=float, default=2.0)
    parser.add_argument("--stuck-duration", type=float, default=3.0)
    parser.add_argument("--target-speed", type=float, default=3.0)
    parser.add_argument("--avoidance-speed", type=float, default=1.2)
    parser.add_argument("--obstacle-count", type=int, default=2)
    parser.add_argument("--obstacle-size-m", type=float, default=0.4)
    parser.add_argument("--min-start-distance-m", type=float, default=8.0)
    parser.add_argument("--min-separation-m", type=float, default=5.0)
    parser.add_argument("--sx", type=float, default=3.0)
    parser.add_argument("--sy", type=float, default=5.0)
    parser.add_argument("--stheta", type=float, default=3.1416)
    parser.add_argument(
        "--base-yaml",
        type=Path,
        default=CONTAINER_PKG / "maps" / "frenet_test_loop.yaml",
    )
    parser.add_argument(
        "--track-csv",
        type=Path,
        default=CONTAINER_PKG / "code" / "outputs" / "generated_tracks" / "frenet_test_loop_processed_track.csv",
    )
    parser.add_argument(
        "--obstacle-trajectory-csv",
        type=Path,
        default=CONTAINER_PKG / "code" / "outputs" / "generated_tracks" / "frenet_test_loop_trajectory.csv",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=CONTAINER_PKG / "code" / "outputs" / "frenet_random_obstacle_robustness",
    )
    parser.add_argument("--batch-name", default="")
    return parser.parse_args()


def main() -> int:
    """脚本入口。

    Returns:
        进程退出码。batch 模式下全部 trial 成功返回 0，否则返回 1；
        rviz 模式透传 `ros2 launch` 的退出码。
    """
    args = parse_args()
    args.repo_root = args.repo_root.resolve()
    batch_name = args.batch_name or datetime.now().strftime("%Y%m%d_%H%M%S")
    batch_dir = args.output_root.resolve() / batch_name
    batch_dir.mkdir(parents=True, exist_ok=True)

    preset = compute_frenet_preset(args.target_speed, args.avoidance_speed)
    print(f"Batch dir: {batch_dir}")
    print(
        "Config: "
        f"trials={args.trials}, laps={args.laps}, "
        f"target={args.target_speed}, avoidance={args.avoidance_speed}, "
        f"obstacles={args.obstacle_count}"
    )
    print(f"Preset: {json.dumps(asdict(preset), sort_keys=True)}")

    if args.mode == "rviz":
        return run_rviz_seed(args, batch_dir, int(args.seed_start), preset)

    results: list[TrialResult] = []
    for offset in range(max(1, int(args.trials))):
        seed = int(args.seed_start) + offset
        result = run_trial(args, batch_dir, seed, preset)
        results.append(result)
        write_outputs(batch_dir, args, preset, results)

    return 0 if all(result.success for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())

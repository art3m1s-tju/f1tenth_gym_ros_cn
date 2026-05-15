#!/usr/bin/env python3
"""ROS 验证脚本：使用增益表运行完整仿真并自动评估。"""
from __future__ import annotations

import csv
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def _count_laps_from_log(log_path: Path, start_x: float, start_y: float, threshold: float = 0.5) -> int:
    """通过日志中路径索引回绕次数估算完成圈数。

    Args:
        log_path: LQR 跟踪日志 CSV 路径。
        start_x: 参考轨迹起点 x 坐标。
        start_y: 参考轨迹起点 y 坐标。
        threshold: 回绕发生时车辆距离起点小于该阈值才计为完成一圈。

    Returns:
        估算完成圈数。
    """

    if not log_path.exists():
        return 0
    try:
        with open(log_path, "r") as f:
            rows = list(csv.DictReader(f))
        if len(rows) < 500:
            return 0

        max_path_idx = max(int(float(row["closest_idx"])) for row in rows)
        high_idx = 0.75 * max_path_idx
        low_idx = 0.25 * max_path_idx
        laps = 0
        previous_idx = None
        for row in rows:
            idx = int(float(row["closest_idx"]))
            if previous_idx is not None and previous_idx > high_idx and idx < low_idx:
                x = float(row["x"])
                y = float(row["y"])
                if math.hypot(x - start_x, y - start_y) < threshold:
                    laps += 1
            previous_idx = idx
        return laps
    except (ValueError, IndexError, OSError):
        return 0


def run_ros_validation(
    table_path: Path,
    target_speed: float = 1.0,
    output_dir: Path | None = None,
    trajectory_csv: str = "/sim_ws/src/f1tenth_gym_ros/code/outputs/csv/global_trajectory.csv",
    log_path: str = "/sim_ws/src/f1tenth_gym_ros/code/outputs/logs/lqr_tracking_log.csv",
    timeout_seconds: float = 90.0,
    lap_count: int = 5,
) -> Path | None:
    """启动完整 ROS 仿真，等待完成目标圈数后运行评估脚本。

    Args:
        table_path: 待验证的 LQR 增益表路径，用于打印追踪当前验证对象。
        target_speed: ROS 仿真节点使用的目标速度。
        output_dir: 评估结果输出目录；未提供时使用默认容器路径。
        trajectory_csv: 全局参考轨迹 CSV 路径。
        log_path: ROS 仿真产生的 LQR 跟踪日志路径。
        timeout_seconds: 等待目标圈数完成的最长时间。
        lap_count: 需要连续完成的圈数。

    Returns:
        成功产生日志后返回评估输出目录；若未产生日志则返回 ``None``。
    """

    if output_dir is None:
        output_dir = Path("/sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros")
    output_dir.mkdir(parents=True, exist_ok=True)

    log_file = Path(log_path)
    if log_file.exists():
        log_file.unlink()

    # 轮询前只读取一次起点位置，后续用于判断是否回到起点附近。
    with open(trajectory_csv) as f:
        reader = csv.DictReader(f)
        first = next(reader)
        start_x, start_y = float(first["x"]), float(first["y"])

    launch_file = "/sim_ws/src/f1tenth_gym_ros/launch/pnc_sim_launch.py"
    launch_cmd = [
        "ros2", "launch", launch_file,
        f"target_speed:={target_speed}",
        f"log_path:={log_path}",
        f"lqr_gain_table_path:={table_path}",
    ]

    required_laps = max(1, int(lap_count))
    print(f"Launching ROS simulation (timeout={timeout_seconds}s, laps={required_laps})...")
    print(f"  Table: {table_path}")
    print(f"  Target speed: {target_speed}")

    proc = subprocess.Popen(
        launch_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )

    start_time = time.time()
    try:
        while time.time() - start_time < timeout_seconds:
            time.sleep(2.0)
            completed_laps = _count_laps_from_log(log_file, start_x, start_y)
            if completed_laps >= required_laps:
                print(f"  {completed_laps} laps detected, stopping simulation...")
                break
        else:
            print("  Timeout reached, stopping simulation...")
    finally:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=10)

    if not log_file.exists():
        print("  ERROR: No log file produced.")
        return None

    eval_cmd = [
        "python3", "/sim_ws/src/f1tenth_gym_ros/code/tracker_evaluate.py",
        "--log", str(log_file),
        "--reference-trajectory-csv", trajectory_csv,
        "--output-dir", str(output_dir),
    ]
    print("  Running evaluation...")
    subprocess.run(eval_cmd, check=False)
    print(f"  Results saved to: {output_dir}")
    return output_dir


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--table", required=True)
    p.add_argument("--speed", type=float, default=1.0)
    p.add_argument("--timeout", type=float, default=90.0)
    p.add_argument("--laps", type=int, default=5)
    p.add_argument("--output-dir", default=None)
    args = p.parse_args()

    out = Path(args.output_dir) if args.output_dir else None
    run_ros_validation(
        Path(args.table),
        args.speed,
        out,
        timeout_seconds=args.timeout,
        lap_count=args.laps,
    )

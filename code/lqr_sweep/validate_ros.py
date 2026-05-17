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
from datetime import datetime

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
    track_csv: str = "/sim_ws/src/f1tenth_gym_ros/code/outputs/csv/processed_track.csv",
    trajectory_csv: str = "/sim_ws/src/f1tenth_gym_ros/code/outputs/csv/global_trajectory.csv",
    log_path: str = "/sim_ws/src/f1tenth_gym_ros/code/outputs/logs/lqr_tracking_log.csv",
    timeout_seconds: float = 90.0,
    lap_count: int = 5,
    min_speed: float = 0.4,
    max_lateral_accel: float = 4.0,
    max_steering_angle: float = 0.36,
    use_tf_pose: bool = False,
    enable_curvature_speed_limit: bool = True,
    curvature_speed_lookahead_m: float = 1.0,
    enable_speed_ramp: bool = True,
    max_accel: float = 1.0,
    max_decel: float = 2.0,
    enable_steering_rate_limit: bool = True,
    max_steering_rate: float = 2.0,
    position_noise_std: float = 0.0,
    heading_noise_std_deg: float = 0.0,
    pose_delay_ms: float = 0.0,
    noise_seed: int = 42,
    launch_log_path: Path | None = None,
    evaluator_log_path: Path | None = None,
    enable_rviz: bool = True,
) -> Path | None:
    """启动完整 ROS 仿真，等待完成目标圈数后运行评估脚本。

    Args:
        table_path: 待验证的 LQR 增益表路径，用于打印追踪当前验证对象。
        target_speed: ROS 仿真节点使用的目标速度。
        output_dir: 评估结果输出目录；未提供时使用默认容器路径。
        track_csv: 规划器读取的赛道 CSV 路径。
        trajectory_csv: 规划器输出并用于评估的全局参考轨迹 CSV 路径。
        log_path: ROS 仿真产生的 LQR 跟踪日志路径。
        timeout_seconds: 等待目标圈数完成的最长时间。
        lap_count: 需要连续完成的圈数。
        min_speed: 曲率限速后的最低速度。
        max_lateral_accel: 曲率限速使用的横向加速度上限。
        max_steering_angle: 最大前轮转角。
        use_tf_pose: 是否用 TF 查询车辆位姿；默认关闭，直接用 odom 位姿。
        enable_curvature_speed_limit: 是否启用曲率限速。
        enable_speed_ramp: 是否启用速度斜坡。
        max_accel: 速度斜坡最大加速度。
        max_decel: 速度斜坡最大减速度。

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
        f"enable_rviz:={str(enable_rviz).lower()}",
        f"target_speed:={target_speed}",
        f"min_speed:={min_speed}",
        f"max_lateral_accel:={max_lateral_accel}",
        f"max_steering_angle:={max_steering_angle}",
        f"use_tf_pose:={str(use_tf_pose).lower()}",
        f"enable_curvature_speed_limit:={str(enable_curvature_speed_limit).lower()}",
        f"curvature_speed_lookahead_m:={curvature_speed_lookahead_m}",
        f"enable_speed_ramp:={str(enable_speed_ramp).lower()}",
        f"max_accel:={max_accel}",
        f"max_decel:={max_decel}",
        f"enable_steering_rate_limit:={str(enable_steering_rate_limit).lower()}",
        f"max_steering_rate:={max_steering_rate}",
        f"validation_position_noise_std:={position_noise_std}",
        f"validation_heading_noise_std_deg:={heading_noise_std_deg}",
        f"validation_pose_delay_ms:={pose_delay_ms}",
        f"validation_noise_seed:={noise_seed}",
        f"track_csv:={track_csv}",
        f"trajectory_csv:={trajectory_csv}",
        f"log_path:={log_path}",
        f"lqr_gain_table_path:={table_path}",
    ]

    required_laps = max(1, int(lap_count))
    print(f"Launching ROS simulation (timeout={timeout_seconds}s, laps={required_laps})...")
    print(f"  Table: {table_path}")
    print(f"  Target speed: {target_speed}")
    print(
        "  Speed handling: "
        f"curvature_limit={enable_curvature_speed_limit}, "
        f"lookahead={curvature_speed_lookahead_m:.2f}m, "
        f"speed_ramp={enable_speed_ramp}, "
        f"min_speed={min_speed}, max_lat_accel={max_lateral_accel}"
    )
    print(
        "  Steering handling: "
        f"rate_limit={enable_steering_rate_limit}, "
        f"max_rate={max_steering_rate:.2f}rad/s"
    )
    print(
        "  Validation noise: "
        f"pos={position_noise_std:.3f}m, "
        f"heading={heading_noise_std_deg:.2f}deg, "
        f"delay={pose_delay_ms:.0f}ms, seed={noise_seed}"
    )

    launch_log_file = None
    stdout_target = subprocess.DEVNULL
    if launch_log_path is not None:
        launch_log_path.parent.mkdir(parents=True, exist_ok=True)
        launch_log_file = launch_log_path.open("w", encoding="utf-8")
        stdout_target = launch_log_file

    proc = subprocess.Popen(
        launch_cmd,
        stdout=stdout_target,
        stderr=subprocess.STDOUT,
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
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=5)
            except ProcessLookupError:
                pass
        if launch_log_file is not None:
            launch_log_file.flush()
            launch_log_file.close()

    if not log_file.exists():
        print("  ERROR: No log file produced.")
        return None

    eval_cmd = [
        "python3", "/sim_ws/src/f1tenth_gym_ros/code/tracker_evaluate.py",
        "--log", str(log_file),
        "--reference-trajectory-csv", trajectory_csv,
        "--output-dir", str(output_dir),
        "--steering-limit-rad", str(max_steering_angle),
    ]
    print("  Running evaluation...")
    evaluator_log_file = None
    eval_stdout = None
    if evaluator_log_path is not None:
        evaluator_log_path.parent.mkdir(parents=True, exist_ok=True)
        evaluator_log_file = evaluator_log_path.open("w", encoding="utf-8")
        eval_stdout = evaluator_log_file

    eval_result = subprocess.run(
        eval_cmd,
        check=False,
        stdout=eval_stdout,
        stderr=subprocess.STDOUT if eval_stdout is not None else None,
    )
    if evaluator_log_file is not None:
        evaluator_log_file.flush()
        evaluator_log_file.close()

    if eval_result.returncode != 0:
        failure_path = output_dir / "evaluation_failed.txt"
        failure_path.write_text(
            "tracker_evaluate.py failed\n"
            f"returncode={eval_result.returncode}\n"
            f"log={log_file}\n"
            f"evaluator_log={evaluator_log_path or ''}\n",
            encoding="utf-8",
        )
        print(f"  WARNING: evaluation failed, see: {failure_path}")
    print(f"  Results saved to: {output_dir}")
    return output_dir


def _speed_label(speed: float) -> str:
    return _number_label(speed)


def _number_label(value: float, digits: int = 2) -> str:
    text = f"{value:.{digits}f}".rstrip("0").rstrip(".")
    return text.replace(".", "p")


def _default_batch_name() -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"original_map_rviz_table_curvlimit_ramp_{stamp}"


def _noise_profile_values(
    profile: str,
    position_noise_std: float | None,
    heading_noise_std_deg: float | None,
    pose_delay_ms: float | None,
) -> tuple[float, float, float]:
    defaults = {
        "clean": (0.0, 0.0, 0.0),
        "light": (0.02, 1.0, 60.0),
    }
    pos, heading, delay = defaults[profile]
    if position_noise_std is not None:
        pos = position_noise_std
    if heading_noise_std_deg is not None:
        heading = heading_noise_std_deg
    if pose_delay_ms is not None:
        delay = pose_delay_ms
    return max(0.0, pos), max(0.0, heading), max(0.0, delay)


def _noise_folder_name(
    profile: str,
    position_noise_std: float,
    heading_noise_std_deg: float,
    pose_delay_ms: float,
) -> str:
    is_clean = (
        profile == "clean"
        and position_noise_std == 0.0
        and heading_noise_std_deg == 0.0
        and pose_delay_ms == 0.0
    )
    if is_clean:
        return "clean"
    pos_cm = position_noise_std * 100.0
    return (
        f"noisy_{profile}_"
        f"pos{_number_label(pos_cm)}cm_"
        f"yaw{_number_label(heading_noise_std_deg)}deg_"
        f"delay{int(round(pose_delay_ms))}ms"
    )


def _run_label(
    speed: float,
    enable_curvature_speed_limit: bool,
    enable_speed_ramp: bool,
    max_lateral_accel: float,
    noise_folder: str,
) -> str:
    return (
        f"v{_speed_label(speed)}_table_stadium_"
        f"curv{int(enable_curvature_speed_limit)}_"
        f"ramp{int(enable_speed_ramp)}_"
        f"alat{str(max_lateral_accel).replace('.', 'p')}_"
        f"{noise_folder}_"
        f"{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )


def run_batch_ros_validation(
    table_path: Path,
    speeds: list[float],
    batch_root: Path,
    track_csv: str,
    trajectory_csv: str,
    timeout_seconds: float,
    lap_count: int,
    min_speed: float,
    max_lateral_accel: float,
    max_steering_angle: float,
    use_tf_pose: bool,
    enable_curvature_speed_limit: bool,
    curvature_speed_lookahead_m: float,
    enable_speed_ramp: bool,
    max_accel: float,
    max_decel: float,
    enable_steering_rate_limit: bool,
    max_steering_rate: float,
    enable_rviz: bool,
    noise_profile: str,
    position_noise_std: float,
    heading_noise_std_deg: float,
    pose_delay_ms: float,
    noise_seed: int,
) -> None:
    """Run ROS/RViz validation for multiple speeds and archive each run."""
    noise_folder = _noise_folder_name(
        noise_profile,
        position_noise_std,
        heading_noise_std_deg,
        pose_delay_ms,
    )
    batch_root = batch_root / noise_folder
    batch_root.mkdir(parents=True, exist_ok=True)
    print(f"Batch output root: {batch_root}")

    manifest_path = batch_root / "manifest.csv"
    with manifest_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "speed",
                "run_dir",
                "tracking_log",
                "evaluation_dir",
                "launch_log",
                "evaluator_log",
                "timeout_s",
                "laps",
                "curvature_limit",
                "curvature_speed_lookahead_m",
                "speed_ramp",
                "max_lateral_accel",
                "max_accel",
                "max_decel",
                "steering_rate_limit",
                "max_steering_rate",
                "noise_profile",
                "position_noise_std",
                "heading_noise_std_deg",
                "pose_delay_ms",
                "noise_seed",
            ]
        )

        for speed in speeds:
            label = _run_label(
                speed,
                enable_curvature_speed_limit,
                enable_speed_ramp,
                max_lateral_accel,
                noise_folder,
            )
            run_dir = batch_root / label
            logs_dir = run_dir / "logs"
            eval_dir = run_dir / "evaluation"
            logs_dir.mkdir(parents=True, exist_ok=True)
            eval_dir.mkdir(parents=True, exist_ok=True)

            tracking_log = logs_dir / f"{label}_tracking.csv"
            launch_log = logs_dir / f"{label}_launch.log"
            evaluator_log = logs_dir / f"{label}_evaluator.log"

            print("\n" + "=" * 72)
            print(f"Running ROS/RViz validation: speed={speed:.1f} m/s")
            print(f"Run dir: {run_dir}")
            result_dir = run_ros_validation(
                table_path=table_path,
                target_speed=speed,
                output_dir=eval_dir,
                track_csv=track_csv,
                trajectory_csv=trajectory_csv,
                log_path=str(tracking_log),
                timeout_seconds=timeout_seconds,
                lap_count=lap_count,
                min_speed=min_speed,
                max_lateral_accel=max_lateral_accel,
                max_steering_angle=max_steering_angle,
                use_tf_pose=use_tf_pose,
                enable_curvature_speed_limit=enable_curvature_speed_limit,
                curvature_speed_lookahead_m=curvature_speed_lookahead_m,
                enable_speed_ramp=enable_speed_ramp,
                max_accel=max_accel,
                max_decel=max_decel,
                enable_steering_rate_limit=enable_steering_rate_limit,
                max_steering_rate=max_steering_rate,
                position_noise_std=position_noise_std,
                heading_noise_std_deg=heading_noise_std_deg,
                pose_delay_ms=pose_delay_ms,
                noise_seed=noise_seed,
                launch_log_path=launch_log,
                evaluator_log_path=evaluator_log,
                enable_rviz=enable_rviz,
            )
            writer.writerow(
                [
                    f"{speed:.2f}",
                    str(run_dir),
                    str(tracking_log),
                    str(result_dir or ""),
                    str(launch_log),
                    str(evaluator_log),
                    f"{timeout_seconds:.1f}",
                    lap_count,
                    int(enable_curvature_speed_limit),
                    f"{curvature_speed_lookahead_m:.3f}",
                    int(enable_speed_ramp),
                    f"{max_lateral_accel:.3f}",
                    f"{max_accel:.3f}",
                    f"{max_decel:.3f}",
                    int(enable_steering_rate_limit),
                    f"{max_steering_rate:.3f}",
                    noise_profile,
                    f"{position_noise_std:.4f}",
                    f"{heading_noise_std_deg:.4f}",
                    f"{pose_delay_ms:.1f}",
                    noise_seed,
                ]
            )
            f.flush()

    print(f"\nBatch manifest saved to: {manifest_path}")


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--table", required=True)
    p.add_argument("--mode", choices=["single", "batch"], default="single")
    p.add_argument("--speed", type=float, default=1.0)
    p.add_argument(
        "--speeds",
        nargs="+",
        type=float,
        default=[0.5, 1.0, 1.5, 2.0, 2.5, 3.0],
        help="Speed list used by --mode batch.",
    )
    p.add_argument("--timeout", type=float, default=90.0)
    p.add_argument("--laps", type=int, default=5)
    p.add_argument("--output-dir", default=None)
    p.add_argument(
        "--batch-name",
        default=None,
        help="Batch folder name under --output-dir for --mode batch.",
    )
    p.add_argument(
        "--track-csv",
        default="/sim_ws/src/f1tenth_gym_ros/code/outputs/csv/processed_track.csv",
    )
    p.add_argument(
        "--trajectory-csv",
        default="/sim_ws/src/f1tenth_gym_ros/code/outputs/csv/global_trajectory.csv",
    )
    p.add_argument(
        "--log-path",
        default="/sim_ws/src/f1tenth_gym_ros/code/outputs/logs/lqr_tracking_log.csv",
    )
    p.add_argument("--min-speed", type=float, default=0.4)
    p.add_argument("--max-lateral-accel", type=float, default=4.0)
    p.add_argument("--max-steering-angle", type=float, default=0.36)
    p.add_argument("--use-tf-pose", action="store_true")
    p.add_argument(
        "--curvature-speed-lookahead-m",
        type=float,
        default=1.0,
        help="Forward path distance used to preview max curvature for speed limiting.",
    )
    p.add_argument(
        "--disable-steering-rate-limit",
        action="store_true",
        help="Disable steering command rate limiting during ROS validation.",
    )
    p.add_argument(
        "--max-steering-rate",
        type=float,
        default=2.0,
        help="Maximum steering command rate in rad/s when rate limiting is enabled.",
    )
    p.add_argument(
        "--noise-profile",
        choices=["clean", "light"],
        default="clean",
        help=(
            "Validation noise preset. clean disables injected pose noise; "
            "light uses 2cm position noise, 1deg heading noise, and 60ms delay."
        ),
    )
    p.add_argument(
        "--position-noise-std",
        type=float,
        default=None,
        help="Override validation position noise standard deviation in meters.",
    )
    p.add_argument(
        "--heading-noise-std-deg",
        type=float,
        default=None,
        help="Override validation heading noise standard deviation in degrees.",
    )
    p.add_argument(
        "--pose-delay-ms",
        type=float,
        default=None,
        help="Override validation pose delay in milliseconds.",
    )
    p.add_argument("--noise-seed", type=int, default=42)
    p.add_argument(
        "--disable-curvature-speed-limit",
        action="store_true",
        help="Disable curvature-based speed limiting during ROS validation.",
    )
    p.add_argument(
        "--disable-speed-ramp",
        action="store_true",
        help="Disable acceleration/deceleration ramp during ROS validation.",
    )
    p.add_argument(
        "--disable-rviz",
        action="store_true",
        help="Run ROS validation without launching RViz.",
    )
    p.add_argument("--max-accel", type=float, default=1.0)
    p.add_argument("--max-decel", type=float, default=2.0)
    args = p.parse_args()

    position_noise_std, heading_noise_std_deg, pose_delay_ms = _noise_profile_values(
        args.noise_profile,
        args.position_noise_std,
        args.heading_noise_std_deg,
        args.pose_delay_ms,
    )

    if args.mode == "batch":
        output_root = Path(args.output_dir) if args.output_dir else Path(
            "/sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros"
        )
        batch_name = args.batch_name or _default_batch_name()
        run_batch_ros_validation(
            table_path=Path(args.table),
            speeds=args.speeds,
            batch_root=output_root / batch_name,
            track_csv=args.track_csv,
            trajectory_csv=args.trajectory_csv,
            timeout_seconds=args.timeout,
            lap_count=args.laps,
            min_speed=args.min_speed,
            max_lateral_accel=args.max_lateral_accel,
            max_steering_angle=args.max_steering_angle,
            use_tf_pose=args.use_tf_pose,
            enable_curvature_speed_limit=not args.disable_curvature_speed_limit,
            curvature_speed_lookahead_m=args.curvature_speed_lookahead_m,
            enable_speed_ramp=not args.disable_speed_ramp,
            max_accel=args.max_accel,
            max_decel=args.max_decel,
            enable_steering_rate_limit=not args.disable_steering_rate_limit,
            max_steering_rate=args.max_steering_rate,
            enable_rviz=not args.disable_rviz,
            noise_profile=args.noise_profile,
            position_noise_std=position_noise_std,
            heading_noise_std_deg=heading_noise_std_deg,
            pose_delay_ms=pose_delay_ms,
            noise_seed=args.noise_seed,
        )
    else:
        out = Path(args.output_dir) if args.output_dir else None
        run_ros_validation(
            Path(args.table),
            args.speed,
            out,
            track_csv=args.track_csv,
            trajectory_csv=args.trajectory_csv,
            log_path=args.log_path,
            timeout_seconds=args.timeout,
            lap_count=args.laps,
            min_speed=args.min_speed,
            max_lateral_accel=args.max_lateral_accel,
            max_steering_angle=args.max_steering_angle,
            use_tf_pose=args.use_tf_pose,
            enable_curvature_speed_limit=not args.disable_curvature_speed_limit,
            curvature_speed_lookahead_m=args.curvature_speed_lookahead_m,
            enable_speed_ramp=not args.disable_speed_ramp,
            max_accel=args.max_accel,
            max_decel=args.max_decel,
            enable_steering_rate_limit=not args.disable_steering_rate_limit,
            max_steering_rate=args.max_steering_rate,
            position_noise_std=position_noise_std,
            heading_noise_std_deg=heading_noise_std_deg,
            pose_delay_ms=pose_delay_ms,
            noise_seed=args.noise_seed,
            enable_rviz=not args.disable_rviz,
        )

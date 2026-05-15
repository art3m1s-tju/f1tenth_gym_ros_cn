#!/usr/bin/env python3
"""生成适合 3 m/s 跟踪的低曲率体育场形赛道和参考轨迹。"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def _stadium_points(
    straight_half_length: float,
    turn_radius: float,
    spacing: float,
) -> list[tuple[float, float, float]]:
    """生成闭环体育场中心线点及切向航向角。

    Args:
        straight_half_length: 直道半长，决定左右半圆圆心之间的距离。
        turn_radius: 两端半圆弯道半径。
        spacing: 相邻轨迹点的目标间距。

    Returns:
        ``(x, y, yaw)`` 元组列表，其中 ``yaw`` 为中心线切向方向。
    """

    points: list[tuple[float, float, float]] = []

    def add_point(x: float, y: float, yaw: float) -> None:
        """向中心线列表追加一个去重后的轨迹点。

        Args:
            x: 轨迹点 x 坐标。
            y: 轨迹点 y 坐标。
            yaw: 轨迹点切向航向角。
        """

        if points and math.hypot(x - points[-1][0], y - points[-1][1]) < 0.5 * spacing:
            return
        points.append((x, y, math.atan2(math.sin(yaw), math.cos(yaw))))

    n_straight = max(2, int((2.0 * straight_half_length) / spacing))
    n_half_straight = max(1, n_straight // 2)

    # 从上方直道中点出发，车辆朝向左侧。
    for i in range(n_half_straight):
        t = i / n_half_straight
        x = -straight_half_length * t
        add_point(x, turn_radius, math.pi)

    # 左侧弯道：从上方直道过渡到底部直道。
    n_turn = max(8, int((math.pi * turn_radius) / spacing))
    for i in range(n_turn):
        theta = math.pi / 2.0 + math.pi * i / n_turn
        x = -straight_half_length + turn_radius * math.cos(theta)
        y = turn_radius * math.sin(theta)
        add_point(x, y, theta + math.pi / 2.0)

    # 底部直道：从左向右。
    for i in range(n_straight):
        t = i / n_straight
        x = -straight_half_length + 2.0 * straight_half_length * t
        add_point(x, -turn_radius, 0.0)

    # 右侧弯道：从底部直道过渡回上方直道。
    for i in range(n_turn):
        theta = -math.pi / 2.0 + math.pi * i / n_turn
        x = straight_half_length + turn_radius * math.cos(theta)
        y = turn_radius * math.sin(theta)
        add_point(x, y, theta + math.pi / 2.0)

    # 补齐上方直道剩余半段，回到起点附近形成闭环。
    for i in range(n_half_straight):
        t = i / n_half_straight
        x = straight_half_length * (1.0 - t)
        add_point(x, turn_radius, math.pi)

    return points


def _write_trajectory(path: Path, points: list[tuple[float, float, float]]) -> None:
    """将中心线轨迹写入控制器使用的 CSV 文件。

    Args:
        path: 输出轨迹 CSV 路径。
        points: ``(x, y, yaw)`` 轨迹点列表。
    """

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "yaw"])
        for x, y, yaw in points:
            writer.writerow([f"{x:.7f}", f"{y:.7f}", f"{yaw:.7f}"])


def _write_processed_track(
    path: Path,
    points: list[tuple[float, float, float]],
    half_width: float,
) -> None:
    """写出包含赛道左右边界和累计里程的处理后赛道 CSV。

    Args:
        path: 输出处理后赛道 CSV 路径。
        points: 中心线 ``(x, y, yaw)`` 轨迹点列表。
        half_width: 赛道半宽。
    """

    path.parent.mkdir(parents=True, exist_ok=True)
    lapdist = 0.0
    previous: tuple[float, float] | None = None
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "left_border_x",
            "left_border_y",
            "right_border_x",
            "right_border_y",
            "pos_x",
            "pos_y",
            "Lapdist",
        ])
        for x, y, yaw in points:
            if previous is not None:
                lapdist += math.hypot(x - previous[0], y - previous[1])
            nx = -math.sin(yaw)
            ny = math.cos(yaw)
            writer.writerow([
                f"{x + half_width * nx:.7f}",
                f"{y + half_width * ny:.7f}",
                f"{x - half_width * nx:.7f}",
                f"{y - half_width * ny:.7f}",
                f"{x:.7f}",
                f"{y:.7f}",
                f"{lapdist:.7f}",
            ])
            previous = (x, y)


def _distance_to_stadium_centerline(
    x: float,
    y: float,
    straight_half_length: float,
    turn_radius: float,
) -> float:
    """计算任意点到体育场中心线的近似距离。

    Args:
        x: 查询点 x 坐标。
        y: 查询点 y 坐标。
        straight_half_length: 体育场直道半长。
        turn_radius: 体育场弯道半径。

    Returns:
        查询点到最近中心线段或圆弧的距离。
    """

    if -straight_half_length <= x <= straight_half_length:
        return min(abs(y - turn_radius), abs(y + turn_radius))
    center_x = -straight_half_length if x < 0.0 else straight_half_length
    return abs(math.hypot(x - center_x, y) - turn_radius)


def _write_map(
    pgm_path: Path,
    yaml_path: Path,
    straight_half_length: float,
    turn_radius: float,
    half_width: float,
    resolution: float,
    margin: float,
    open_map: bool,
) -> None:
    """生成 PGM 占据栅格地图及对应 YAML 元数据。

    Args:
        pgm_path: 输出 PGM 地图路径。
        yaml_path: 输出地图 YAML 路径。
        straight_half_length: 体育场直道半长。
        turn_radius: 体育场弯道半径。
        half_width: 可行驶区域半宽。
        resolution: 栅格分辨率，单位为米/像素。
        margin: 地图边界额外留白。
        open_map: 为 ``True`` 时生成全自由空间地图，便于控制器标定。
    """

    x_min = -straight_half_length - turn_radius - half_width - margin
    x_max = straight_half_length + turn_radius + half_width + margin
    y_min = -turn_radius - half_width - margin
    y_max = turn_radius + half_width + margin
    width = int(math.ceil((x_max - x_min) / resolution))
    height = int(math.ceil((y_max - y_min) / resolution))

    pgm_path.parent.mkdir(parents=True, exist_ok=True)
    with pgm_path.open("wb") as f:
        f.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        pixels = bytearray()
        for row in range(height):
            y = y_max - (row + 0.5) * resolution
            for col in range(width):
                x = x_min + (col + 0.5) * resolution
                if open_map:
                    pixels.append(254)
                else:
                    dist = _distance_to_stadium_centerline(
                        x, y, straight_half_length, turn_radius
                    )
                    pixels.append(254 if dist <= half_width else 0)
        f.write(pixels)

    yaml_path.write_text(
        "\n".join([
            f"image: {pgm_path.name}",
            "mode: trinary",
            f"resolution: {resolution}",
            f"origin: [{x_min:.6f}, {y_min:.6f}, 0]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.25",
            "",
        ]),
        encoding="ascii",
    )


def parse_args() -> argparse.Namespace:
    """解析赛道生成脚本的命令行参数。

    Returns:
        包含地图、轨迹输出路径和几何参数的命名空间。
    """

    parser = argparse.ArgumentParser()
    parser.add_argument("--map-prefix", default="maps/stadium_3ms")
    parser.add_argument("--trajectory-csv", default="code/outputs/csv/stadium_3ms_trajectory.csv")
    parser.add_argument("--track-csv", default="code/outputs/csv/stadium_3ms_processed_track.csv")
    parser.add_argument("--straight-half-length", type=float, default=4.0)
    parser.add_argument("--turn-radius", type=float, default=4.0)
    parser.add_argument("--track-width", type=float, default=2.0)
    parser.add_argument("--spacing", type=float, default=0.08)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--margin", type=float, default=0.8)
    parser.add_argument(
        "--open-map",
        action="store_true",
        help="Generate a fully free-space map for controller calibration.",
    )
    return parser.parse_args()


def main() -> None:
    """生成地图、参考轨迹和处理后赛道文件，并打印关键几何指标。"""

    args = parse_args()
    half_width = 0.5 * args.track_width
    points = _stadium_points(args.straight_half_length, args.turn_radius, args.spacing)

    map_prefix = Path(args.map_prefix)
    pgm_path = map_prefix.with_suffix(".pgm")
    yaml_path = map_prefix.with_suffix(".yaml")
    _write_map(
        pgm_path=pgm_path,
        yaml_path=yaml_path,
        straight_half_length=args.straight_half_length,
        turn_radius=args.turn_radius,
        half_width=half_width,
        resolution=args.resolution,
        margin=args.margin,
        open_map=args.open_map,
    )
    _write_trajectory(Path(args.trajectory_csv), points)
    _write_processed_track(Path(args.track_csv), points, half_width)

    max_curvature = 1.0 / args.turn_radius
    lateral_accel_at_3 = 3.0 * 3.0 * max_curvature
    print(f"Wrote {pgm_path}")
    print(f"Wrote {yaml_path}")
    print(f"Wrote {args.trajectory_csv}")
    print(f"Wrote {args.track_csv}")
    print(f"points={len(points)} max_curvature={max_curvature:.3f} 1/m")
    print(f"lateral_accel_at_3mps={lateral_accel_at_3:.3f} m/s^2")
    print("recommended start pose: sx=0.0000 sy=4.0000 stheta=3.1416")


if __name__ == "__main__":
    main()

"""Frenet 调参预设的唯一实现入口。"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from dataclasses import replace


FRENET_NODE_DEFAULTS = {
    "global_path_topic": "/global_trajectory",
    "local_path_topic": "/local_trajectory",
    "speed_limit_topic": "/local_trajectory_speed_limit",
    "odom_topic": "/ego_racecar/odom",
    "scan_topic": "/scan",
    "map_topic": "/map",
    "frame_id": "map",
    "publish_rate_hz": 20.0,
    "target_speed": 1.5,
    "d_min": -1.0,
    "d_max": 1.0,
    "d_step": 0.1,
    "t_min": 2.0,
    "t_max": 3.0,
    "t_step": 0.5,
    "v_min": 0.6,
    "v_max": 2.5,
    "v_step": 0.3,
    "trajectory_dt": 0.1,
    "max_curvature": 1.14,
    "weight_curvature": 0.4,
    "weight_curvature_rate": 8.0,
    "safe_clearance_m": 0.35,
    "min_clearance_m": 0.05,
    "corridor_radius_m": 0.20,
    "corridor_sample_step_m": 0.10,
    "path_collision_sample_step_m": 0.05,
    "footprint_front_m": 0.38,
    "footprint_rear_m": 0.05,
    "max_heading_jump": 0.65,
    "min_progress_step_m": 0.20,
    "max_initial_s_accel_mps2": 2.0,
    "max_reliable_initial_s_accel_mps2": 3.0,
    "grid_forward_m": 7.0,
    "grid_rear_m": 1.0,
    "grid_half_width_m": 3.0,
    "grid_resolution_m": 0.05,
    "grid_inflation_radius_m": 0.28,
    "scan_offset_x_m": 0.275,
    "debug_marker_topic": "/frenet/debug/candidates",
    "debug_max_safe_candidates": 12,
    "reuse_last_candidate_timeout_s": 1.0,
    "held_path_replan_clearance_m": 0.22,
    "held_path_min_remaining_m": 2.0,
    "held_path_max_lateral_error_m": 0.45,
    "held_path_max_heading_error_rad": 0.85,
    "candidate_profile_consistency_weight": 20.0,
    "candidate_profile_max_jump_m": 0.35,
    "candidate_profile_lookahead_m": 4.0,
    "candidate_profile_unlock_clearance_gain_m": 0.12,
    "candidate_channel_memory_timeout_s": 1.5,
    "projection_search_window_m": 6.0,
    "stop_path_length_m": 2.0,
    "min_published_path_length_m": 0.75,
    "published_path_lookahead_m": 0.25,
    "max_published_path_length_m": 4.0,
    "min_path_publish_interval_s": 0.25,
    "path_republish_distance_m": 0.50,
    "path_republish_min_remaining_m": 2.0,
    "centerline_return_lookahead_m": 5.0,
    "centerline_return_step_m": 0.08,
    "centerline_return_direct_d_threshold_m": 0.20,
    "centerline_threat_corridor_radius_m": 0.32,
    "centerline_threat_lookahead_m": 6.0,
    "reference_closed_loop": True,
    "cruise_speed_mps": 1.0,
    "activation_min_lookahead_m": 3.0,
    "activation_max_lookahead_m": 8.0,
    "activation_base_lookahead_m": 2.2,
    "activation_reaction_time_s": 1.0,
    "activation_decel_mps2": 2.0,
    "activation_path_margin_m": 0.75,
    "approach_slowdown_extra_m": -1.0,
    "max_observed_speed_mps": 0.0,
    "centerline_speed_limit_mps": -1.0,
    "avoidance_speed_limit_mps": 0.70,
    "stop_speed_limit_mps": 0.0,
}
"""Frenet node 直接运行时使用的参数默认值。

launch 文件和测试脚本通常会显式覆盖其中一部分参数。把 node 默认值集中到
这里，可以避免 `node.py` 中几十行 `declare_parameter()` 与 launch 配置继续
漂移，也方便排查“某个参数到底默认是多少”。
"""


FRENET_LAUNCH_ARGUMENT_DEFAULTS = [
    ("frenet_publish_rate_hz", "20.0"),
    ("frenet_target_speed", "1.5"),
    ("frenet_v_min", "0.6"),
    ("frenet_v_max", "2.5"),
    ("frenet_v_step", "0.3"),
    ("frenet_t_min", "2.0"),
    ("frenet_t_max", "3.0"),
    ("frenet_t_step", "0.5"),
    ("frenet_d_min", "-1.8"),
    ("frenet_d_max", "1.8"),
    ("frenet_d_step", "0.1"),
    ("frenet_trajectory_dt", "0.1"),
    ("frenet_max_curvature", "1.14"),
    ("frenet_weight_curvature", "0.4"),
    ("frenet_weight_curvature_rate", "8.0"),
    ("frenet_safe_clearance_m", "0.35"),
    ("frenet_min_clearance_m", "0.05"),
    ("frenet_corridor_radius_m", "0.20"),
    ("frenet_corridor_sample_step_m", "0.10"),
    ("frenet_path_collision_sample_step_m", "0.05"),
    ("frenet_footprint_front_m", "0.38"),
    ("frenet_footprint_rear_m", "0.05"),
    ("frenet_max_heading_jump", "0.85"),
    ("frenet_min_progress_step_m", "0.20"),
    ("frenet_max_initial_s_accel_mps2", "2.0"),
    ("frenet_max_reliable_initial_s_accel_mps2", "3.0"),
    ("frenet_reuse_last_candidate_timeout_s", "1.0"),
    ("frenet_held_path_replan_clearance_m", "0.22"),
    ("frenet_held_path_min_remaining_m", "2.0"),
    ("frenet_held_path_max_lateral_error_m", "0.45"),
    ("frenet_held_path_max_heading_error_rad", "0.85"),
    ("frenet_candidate_profile_consistency_weight", "20.0"),
    ("frenet_candidate_profile_max_jump_m", "0.35"),
    ("frenet_candidate_profile_lookahead_m", "4.0"),
    ("frenet_candidate_profile_unlock_clearance_gain_m", "0.12"),
    ("frenet_candidate_channel_memory_timeout_s", "1.5"),
    ("frenet_projection_search_window_m", "6.0"),
    ("frenet_stop_path_length_m", "2.0"),
    ("frenet_min_published_path_length_m", "0.75"),
    ("frenet_published_path_lookahead_m", "0.25"),
    ("frenet_max_published_path_length_m", "4.0"),
    ("frenet_min_path_publish_interval_s", "0.25"),
    ("frenet_path_republish_distance_m", "0.50"),
    ("frenet_path_republish_min_remaining_m", "2.0"),
    ("frenet_centerline_return_lookahead_m", "5.0"),
    ("frenet_centerline_return_step_m", "0.08"),
    ("frenet_centerline_return_direct_d_threshold_m", "0.20"),
    ("frenet_centerline_threat_corridor_radius_m", "0.32"),
    ("frenet_centerline_threat_lookahead_m", "6.0"),
    ("frenet_reference_closed_loop", "true"),
    ("frenet_activation_min_lookahead_m", "3.0"),
    ("frenet_activation_max_lookahead_m", "8.0"),
    ("frenet_activation_base_lookahead_m", "2.2"),
    ("frenet_activation_reaction_time_s", "1.0"),
    ("frenet_activation_decel_mps2", "2.0"),
    ("frenet_activation_path_margin_m", "0.75"),
    ("frenet_approach_slowdown_extra_m", "-1.0"),
    ("frenet_max_observed_speed_mps", "0.0"),
    ("frenet_centerline_speed_limit_mps", "-1.0"),
    ("frenet_avoidance_speed_limit_mps", "0.75"),
    ("frenet_stop_speed_limit_mps", "0.0"),
    ("frenet_grid_inflation_radius_m", "0.28"),
    ("frenet_grid_resolution_m", "0.05"),
    ("frenet_grid_forward_m", "10.0"),
    ("frenet_grid_rear_m", "1.0"),
    ("frenet_grid_half_width_m", "3.0"),
]
"""launch 层暴露给用户的 Frenet 参数及默认值。"""


FRENET_NODE_PARAM_MAP = [
    ("publish_rate_hz", "frenet_publish_rate_hz"),
    ("target_speed", "frenet_target_speed"),
    ("v_min", "frenet_v_min"),
    ("v_max", "frenet_v_max"),
    ("v_step", "frenet_v_step"),
    ("t_min", "frenet_t_min"),
    ("t_max", "frenet_t_max"),
    ("t_step", "frenet_t_step"),
    ("d_min", "frenet_d_min"),
    ("d_max", "frenet_d_max"),
    ("d_step", "frenet_d_step"),
    ("trajectory_dt", "frenet_trajectory_dt"),
    ("max_curvature", "frenet_max_curvature"),
    ("weight_curvature", "frenet_weight_curvature"),
    ("weight_curvature_rate", "frenet_weight_curvature_rate"),
    ("safe_clearance_m", "frenet_safe_clearance_m"),
    ("min_clearance_m", "frenet_min_clearance_m"),
    ("corridor_radius_m", "frenet_corridor_radius_m"),
    ("corridor_sample_step_m", "frenet_corridor_sample_step_m"),
    ("path_collision_sample_step_m", "frenet_path_collision_sample_step_m"),
    ("footprint_front_m", "frenet_footprint_front_m"),
    ("footprint_rear_m", "frenet_footprint_rear_m"),
    ("max_heading_jump", "frenet_max_heading_jump"),
    ("min_progress_step_m", "frenet_min_progress_step_m"),
    ("max_initial_s_accel_mps2", "frenet_max_initial_s_accel_mps2"),
    (
        "max_reliable_initial_s_accel_mps2",
        "frenet_max_reliable_initial_s_accel_mps2",
    ),
    ("reuse_last_candidate_timeout_s", "frenet_reuse_last_candidate_timeout_s"),
    ("held_path_replan_clearance_m", "frenet_held_path_replan_clearance_m"),
    ("held_path_min_remaining_m", "frenet_held_path_min_remaining_m"),
    ("held_path_max_lateral_error_m", "frenet_held_path_max_lateral_error_m"),
    (
        "held_path_max_heading_error_rad",
        "frenet_held_path_max_heading_error_rad",
    ),
    (
        "candidate_profile_consistency_weight",
        "frenet_candidate_profile_consistency_weight",
    ),
    ("candidate_profile_max_jump_m", "frenet_candidate_profile_max_jump_m"),
    ("candidate_profile_lookahead_m", "frenet_candidate_profile_lookahead_m"),
    (
        "candidate_profile_unlock_clearance_gain_m",
        "frenet_candidate_profile_unlock_clearance_gain_m",
    ),
    (
        "candidate_channel_memory_timeout_s",
        "frenet_candidate_channel_memory_timeout_s",
    ),
    ("projection_search_window_m", "frenet_projection_search_window_m"),
    ("stop_path_length_m", "frenet_stop_path_length_m"),
    ("min_published_path_length_m", "frenet_min_published_path_length_m"),
    ("published_path_lookahead_m", "frenet_published_path_lookahead_m"),
    ("max_published_path_length_m", "frenet_max_published_path_length_m"),
    ("min_path_publish_interval_s", "frenet_min_path_publish_interval_s"),
    ("path_republish_distance_m", "frenet_path_republish_distance_m"),
    ("path_republish_min_remaining_m", "frenet_path_republish_min_remaining_m"),
    ("centerline_return_lookahead_m", "frenet_centerline_return_lookahead_m"),
    ("centerline_return_step_m", "frenet_centerline_return_step_m"),
    (
        "centerline_return_direct_d_threshold_m",
        "frenet_centerline_return_direct_d_threshold_m",
    ),
    (
        "centerline_threat_corridor_radius_m",
        "frenet_centerline_threat_corridor_radius_m",
    ),
    ("centerline_threat_lookahead_m", "frenet_centerline_threat_lookahead_m"),
    ("reference_closed_loop", "frenet_reference_closed_loop"),
    ("activation_min_lookahead_m", "frenet_activation_min_lookahead_m"),
    ("activation_max_lookahead_m", "frenet_activation_max_lookahead_m"),
    ("activation_base_lookahead_m", "frenet_activation_base_lookahead_m"),
    ("activation_reaction_time_s", "frenet_activation_reaction_time_s"),
    ("activation_decel_mps2", "frenet_activation_decel_mps2"),
    ("activation_path_margin_m", "frenet_activation_path_margin_m"),
    ("approach_slowdown_extra_m", "frenet_approach_slowdown_extra_m"),
    ("max_observed_speed_mps", "frenet_max_observed_speed_mps"),
    ("centerline_speed_limit_mps", "frenet_centerline_speed_limit_mps"),
    ("avoidance_speed_limit_mps", "frenet_avoidance_speed_limit_mps"),
    ("stop_speed_limit_mps", "frenet_stop_speed_limit_mps"),
    ("grid_inflation_radius_m", "frenet_grid_inflation_radius_m"),
    ("grid_resolution_m", "frenet_grid_resolution_m"),
    ("grid_forward_m", "frenet_grid_forward_m"),
    ("grid_rear_m", "frenet_grid_rear_m"),
    ("grid_half_width_m", "frenet_grid_half_width_m"),
]
"""Frenet node 内部参数名到 launch 参数名的映射表。"""


@dataclass(frozen=True)
class FrenetPreset:
    """按目标速度派生出的 Frenet 规划参数。

    这些参数最初用于固定地图 RViz 测试，后来随机障碍鲁棒性测试也需要同一套
    规则。把公式集中到这里可以避免 shell 脚本、随机测试 runner 和 launch
    参数之间发生漂移。

    Attributes:
        geometry_target_speed: Frenet 多项式几何规划的目标速度，单位 m/s。
        v_min: 纵向终点速度采样下界，单位 m/s。
        v_max: 纵向终点速度采样上界，单位 m/s。
        v_step: 纵向终点速度采样间隔，单位 m/s。
        d_step: 横向终点采样间隔，单位 m。
        trajectory_dt: 候选轨迹离散时间步长，单位 s。
        grid_forward_m: 局部占据栅格前向距离，单位 m。
        hold_replan_clearance_m: hold 轨迹低于该 clearance 时强制重规划，单位 m。
        hold_min_remaining_m: hold 轨迹剩余长度低于该值时强制重规划，单位 m。
        reuse_timeout_s: 当前周期无解时允许复用上一条安全轨迹的最长时间，单位 s。
        activation_max_m: Frenet 激活距离上限，单位 m。
        activation_reaction_s: 速度相关激活距离中的反应时间项，单位 s。
        approach_extra_m: 预激活距离相对激活距离的额外前瞻，单位 m。
        activation_path_margin_m: 发布路径长度之外允许提前激活的余量，单位 m。
        centerline_return_lookahead_m: 绕障结束后平滑回中路径长度，单位 m。
        max_published_path_length_m: 实际发布给控制器的 Frenet 路径长度上限，单位 m。
    """

    geometry_target_speed: float
    v_min: float
    v_max: float
    v_step: float
    d_step: float
    trajectory_dt: float
    grid_forward_m: float
    hold_replan_clearance_m: float
    hold_min_remaining_m: float
    reuse_timeout_s: float
    activation_max_m: float
    activation_reaction_s: float
    approach_extra_m: float
    activation_path_margin_m: float
    centerline_return_lookahead_m: float
    max_published_path_length_m: float


def compute_frenet_preset(target_speed: float, avoidance_speed: float) -> FrenetPreset:
    """根据目标车速生成 Frenet 参数预设。

    Args:
        target_speed: LQR 全局巡航目标速度，单位 m/s。
        avoidance_speed: Frenet 避障阶段局部限速，单位 m/s。

    Returns:
        `FrenetPreset`。高速时扩大栅格、预激活距离和候选轨迹保持时间；低速
        时使用更短保持和更密轨迹采样，减少过度保守。
    """
    target = float(target_speed)
    avoidance = float(avoidance_speed)
    geom_target = max(1.2, target, avoidance)
    v_min = max(0.6, min(avoidance * 0.7, target * 0.5, geom_target))
    v_max = max(1.8, geom_target * 1.25, avoidance * 1.5)
    return FrenetPreset(
        geometry_target_speed=geom_target,
        v_min=v_min,
        v_max=v_max,
        v_step=0.75 if target >= 2.0 else 0.6,
        d_step=0.35 if target >= 2.0 else 0.3,
        trajectory_dt=0.10 if target >= 2.0 else 0.05,
        grid_forward_m=max(10.0, target * 6.0 + 2.0),
        hold_replan_clearance_m=0.10 if target >= 2.0 else 0.20,
        hold_min_remaining_m=max(1.20, min(2.20, target * 0.70)),
        reuse_timeout_s=2.0 if target >= 2.0 else 1.0,
        activation_max_m=max(8.0, 2.5 + target * 2.7),
        activation_reaction_s=1.2 if target >= 2.0 else 1.0,
        approach_extra_m=float(max(0.40, min(0.90, target * 0.25))),
        activation_path_margin_m=1.25,
        centerline_return_lookahead_m=float(
            min(3.5, max(2.0, 1.5 + 2.0 * target / 3.0))
        ),
        max_published_path_length_m=float(max(5.5, min(7.5, target * 2.2))),
    )


def frenet_static_test_launch_args(
    preset: FrenetPreset,
    avoidance_speed: float,
) -> list[str]:
    """生成固定地图测试和随机障碍测试共用的 Frenet launch 覆盖参数。

    Args:
        preset: 速度相关 Frenet preset。
        avoidance_speed: Frenet 避障阶段局部限速，单位 m/s。

    Returns:
        `ros2 launch` 可直接接收的 `name:=value` 参数列表。这里集中维护测试用
        Frenet 参数，避免 `run_frenet_test.sh` 与随机鲁棒性 runner 各写一份。
    """
    return [
        "frenet_reference_closed_loop:=true",
        "frenet_centerline_speed_limit_mps:=-1.0",
        f"frenet_avoidance_speed_limit_mps:={float(avoidance_speed):.3f}",
        "frenet_stop_speed_limit_mps:=0.0",
        f"frenet_target_speed:={preset.geometry_target_speed:.3f}",
        f"frenet_v_min:={preset.v_min:.3f}",
        f"frenet_v_max:={preset.v_max:.3f}",
        f"frenet_v_step:={preset.v_step:.3f}",
        "frenet_t_min:=4.0",
        "frenet_t_max:=6.0",
        "frenet_t_step:=2.0",
        f"frenet_trajectory_dt:={preset.trajectory_dt:.3f}",
        "frenet_d_min:=-1.8",
        "frenet_d_max:=1.8",
        f"frenet_d_step:={preset.d_step:.3f}",
        "frenet_max_heading_jump:=0.85",
        "frenet_max_initial_s_accel_mps2:=2.0",
        "frenet_max_reliable_initial_s_accel_mps2:=3.0",
        "frenet_grid_inflation_radius_m:=0.12",
        f"frenet_grid_forward_m:={preset.grid_forward_m:.3f}",
        "frenet_grid_half_width_m:=3.2",
        "frenet_max_curvature:=1.14",
        "frenet_weight_curvature:=0.4",
        "frenet_weight_curvature_rate:=8.0",
        "frenet_corridor_radius_m:=0.16",
        "frenet_corridor_sample_step_m:=0.05",
        "frenet_path_collision_sample_step_m:=0.05",
        "frenet_footprint_front_m:=0.45",
        "frenet_footprint_rear_m:=0.05",
        "frenet_safe_clearance_m:=0.30",
        "frenet_min_clearance_m:=0.04",
        "frenet_published_path_lookahead_m:=0.25",
        f"frenet_max_published_path_length_m:={preset.max_published_path_length_m:.3f}",
        "frenet_min_path_publish_interval_s:=0.25",
        "frenet_path_republish_distance_m:=0.50",
        "frenet_path_republish_min_remaining_m:=2.0",
        (
            "frenet_centerline_return_lookahead_m:="
            f"{preset.centerline_return_lookahead_m:.3f}"
        ),
        "frenet_centerline_threat_lookahead_m:=8.0",
        "frenet_centerline_threat_corridor_radius_m:=0.22",
        "frenet_activation_min_lookahead_m:=3.0",
        f"frenet_activation_max_lookahead_m:={preset.activation_max_m:.3f}",
        "frenet_activation_base_lookahead_m:=2.2",
        f"frenet_activation_reaction_time_s:={preset.activation_reaction_s:.3f}",
        "frenet_activation_decel_mps2:=2.0",
        f"frenet_activation_path_margin_m:={preset.activation_path_margin_m:.3f}",
        f"frenet_approach_slowdown_extra_m:={preset.approach_extra_m:.3f}",
        f"frenet_reuse_last_candidate_timeout_s:={preset.reuse_timeout_s:.3f}",
        f"frenet_held_path_replan_clearance_m:={preset.hold_replan_clearance_m:.3f}",
        f"frenet_held_path_min_remaining_m:={preset.hold_min_remaining_m:.3f}",
        "frenet_held_path_max_lateral_error_m:=0.45",
        "frenet_held_path_max_heading_error_rad:=0.85",
        "frenet_candidate_profile_consistency_weight:=20.0",
        "frenet_candidate_profile_max_jump_m:=0.35",
        "frenet_candidate_profile_lookahead_m:=4.0",
        "frenet_candidate_profile_unlock_clearance_gain_m:=0.12",
        "frenet_candidate_channel_memory_timeout_s:=1.5",
        "frenet_centerline_return_direct_d_threshold_m:=0.20",
    ]


def shell_launch_arg_lines(args: list[str]) -> str:
    """把 launch 参数列表输出成 shell heredoc 可直接拼接的多行文本。

    Args:
        args: `name:=value` 形式的 launch 参数列表。

    Returns:
        多行字符串。除最后一行外，每行都以反斜杠续行。
    """
    if not args:
        return ""
    return " \\\n".join(args)


def shell_default_assignments(preset: FrenetPreset) -> str:
    """输出 `run_frenet_test.sh` 可直接 `eval` 的默认变量赋值。

    Args:
        preset: 由 `compute_frenet_preset()` 生成的预设。

    Returns:
        多行 shell 变量赋值文本。变量名保持原脚本兼容，避免改动用户命令入口。
    """
    values = {
        "FRENET_GEOMETRY_TARGET_SPEED_DEFAULT": preset.geometry_target_speed,
        "FRENET_V_MIN_DEFAULT": preset.v_min,
        "FRENET_V_MAX_DEFAULT": preset.v_max,
        "FRENET_GRID_FORWARD_DEFAULT": preset.grid_forward_m,
        "FRENET_V_STEP_DEFAULT": preset.v_step,
        "FRENET_D_STEP_DEFAULT": preset.d_step,
        "FRENET_TRAJECTORY_DT_DEFAULT": preset.trajectory_dt,
        "FRENET_HOLD_REPLAN_CLEARANCE_DEFAULT": preset.hold_replan_clearance_m,
        "FRENET_HOLD_MIN_REMAINING_DEFAULT": preset.hold_min_remaining_m,
        "FRENET_REUSE_TIMEOUT_DEFAULT": preset.reuse_timeout_s,
        "FRENET_ACTIVATION_MAX_DEFAULT": preset.activation_max_m,
        "FRENET_ACTIVATION_REACTION_DEFAULT": preset.activation_reaction_s,
        "FRENET_APPROACH_EXTRA_DEFAULT": preset.approach_extra_m,
        "FRENET_ACTIVATION_PATH_MARGIN_DEFAULT": preset.activation_path_margin_m,
        "FRENET_CENTERLINE_RETURN_LOOKAHEAD_DEFAULT": (
            preset.centerline_return_lookahead_m
        ),
        "FRENET_MAX_PUBLISHED_PATH_LENGTH_DEFAULT": (
            preset.max_published_path_length_m
        ),
    }
    return "\n".join(f'{name}="{value:.3f}"' for name, value in values.items())


def _preset_with_cli_overrides(preset: FrenetPreset, args: argparse.Namespace) -> FrenetPreset:
    """把 `run_frenet_test.sh` 传入的环境变量覆盖值应用到 preset。

    Args:
        preset: 由目标速度推导出的基础 preset。
        args: CLI 参数命名空间。

    Returns:
        已覆盖字段的新 `FrenetPreset`。
    """
    overrides = {
        "geometry_target_speed": args.geometry_target_speed,
        "v_min": args.v_min,
        "v_max": args.v_max,
        "v_step": args.v_step,
        "d_step": args.d_step,
        "trajectory_dt": args.trajectory_dt,
        "grid_forward_m": args.grid_forward,
        "hold_replan_clearance_m": args.hold_replan_clearance,
        "hold_min_remaining_m": args.hold_min_remaining,
        "reuse_timeout_s": args.reuse_timeout,
        "activation_max_m": args.activation_max,
        "activation_reaction_s": args.activation_reaction,
        "approach_extra_m": args.approach_extra,
        "activation_path_margin_m": args.activation_path_margin,
        "centerline_return_lookahead_m": args.centerline_return_lookahead,
        "max_published_path_length_m": args.max_published_path_length,
    }
    clean_overrides = {
        name: value
        for name, value in overrides.items()
        if value is not None
    }
    return replace(preset, **clean_overrides)


def main() -> int:
    """命令行入口，供 shell 脚本获取共享 preset。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-speed", type=float, required=True)
    parser.add_argument("--avoidance-speed", type=float, required=True)
    parser.add_argument("--shell-defaults", action="store_true")
    parser.add_argument("--shell-launch-args", action="store_true")
    parser.add_argument("--geometry-target-speed", type=float)
    parser.add_argument("--v-min", type=float)
    parser.add_argument("--v-max", type=float)
    parser.add_argument("--v-step", type=float)
    parser.add_argument("--d-step", type=float)
    parser.add_argument("--trajectory-dt", type=float)
    parser.add_argument("--grid-forward", type=float)
    parser.add_argument("--hold-replan-clearance", type=float)
    parser.add_argument("--hold-min-remaining", type=float)
    parser.add_argument("--reuse-timeout", type=float)
    parser.add_argument("--activation-max", type=float)
    parser.add_argument("--activation-reaction", type=float)
    parser.add_argument("--approach-extra", type=float)
    parser.add_argument("--activation-path-margin", type=float)
    parser.add_argument("--centerline-return-lookahead", type=float)
    parser.add_argument("--max-published-path-length", type=float)
    args = parser.parse_args()

    preset = compute_frenet_preset(args.target_speed, args.avoidance_speed)
    if args.shell_defaults:
        print(shell_default_assignments(preset))
    if args.shell_launch_args:
        preset = _preset_with_cli_overrides(preset, args)
        launch_args = frenet_static_test_launch_args(preset, args.avoidance_speed)
        print(shell_launch_arg_lines(launch_args))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

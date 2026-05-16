"""用于 LQR 参数优化的约束式简化目标函数。"""
from __future__ import annotations

import math
from dataclasses import dataclass


# 用于归一化的参考尺度，使两个不同单位的指标可以放在一起平均。
_SCALE_LATERAL_MEAN = 0.05       # 5 厘米
_SCALE_HEADING_MEAN_DEG = 3.0    # 3 度
_SCALE_LATERAL_P95 = 0.08        # 8 厘米
_SCALE_HEADING_P95_DEG = 6.0     # 6 度

# 硬约束阈值：超过这些阈值说明参数不适合进入排序。
_MAX_LATERAL_ERROR = 0.25             # 25 厘米
_MAX_STEERING_SATURATION_RATIO = 0.10 # 超过 10% 时间接近转向限幅
_MAX_STEERING_RATE_RMS_DEG_S = 100.0  # 转向命令变化速度 RMS


@dataclass(frozen=True)
class ObjectiveConfig:
    """LQR sweep 目标函数权重和硬约束。"""

    lateral_mean_weight: float = 0.5
    heading_mean_weight: float = 0.5
    lateral_p95_weight: float = 0.0
    heading_p95_weight: float = 0.0
    lateral_mean_scale: float = _SCALE_LATERAL_MEAN
    heading_mean_scale_deg: float = _SCALE_HEADING_MEAN_DEG
    lateral_p95_scale: float = _SCALE_LATERAL_P95
    heading_p95_scale_deg: float = _SCALE_HEADING_P95_DEG
    max_lateral_error: float = _MAX_LATERAL_ERROR
    max_steering_saturation_ratio: float = _MAX_STEERING_SATURATION_RATIO
    max_steering_rate_rms_deg_s: float = _MAX_STEERING_RATE_RMS_DEG_S


DEFAULT_OBJECTIVE = ObjectiveConfig()

LOW_SPEED_HEADING_OBJECTIVE = ObjectiveConfig(
    lateral_mean_weight=0.30,
    heading_mean_weight=0.35,
    lateral_p95_weight=0.10,
    heading_p95_weight=0.25,
)


def compute_objective(
    metrics: dict[str, float],
    config: ObjectiveConfig = DEFAULT_OBJECTIVE,
) -> float:
    """根据仿真指标计算单个标量优化分数。

    先用硬约束淘汰明显不稳定或蛇形的参数，再用平均横向误差和平均航向
    误差排序。碰撞、超时等失败仿真仍由 sweep 引擎直接记为无穷大。

    Args:
        metrics: 仿真输出的指标字典，至少包含平均横向误差和平均航向误差。

    Returns:
        归一化后的平均跟踪误差分数。
    """

    mean_ey = metrics.get("mean_abs_e_y", math.inf)
    max_ey = metrics.get("max_abs_e_y", math.inf)
    p95_ey = metrics.get("p95_abs_e_y", math.inf)
    mean_heading = metrics.get("mean_abs_e_psi_deg", math.inf)
    p95_heading = metrics.get("p95_abs_e_psi_deg", math.inf)
    steering_rate = metrics.get("steering_rate_rms_deg_s", 0.0)
    steering_sat_ratio = metrics.get("steering_saturation_ratio", 0.0)

    required_values = [
        mean_ey,
        p95_ey,
        max_ey,
        mean_heading,
        p95_heading,
        steering_rate,
        steering_sat_ratio,
    ]
    if any(math.isinf(v) or math.isnan(v) for v in required_values):
        return math.inf

    if max_ey > config.max_lateral_error:
        return math.inf
    if steering_sat_ratio > config.max_steering_saturation_ratio:
        return math.inf
    if steering_rate > config.max_steering_rate_rms_deg_s:
        return math.inf

    weighted_scores = [
        config.lateral_mean_weight * mean_ey / config.lateral_mean_scale,
        config.heading_mean_weight * mean_heading / config.heading_mean_scale_deg,
        config.lateral_p95_weight * p95_ey / config.lateral_p95_scale,
        config.heading_p95_weight * p95_heading / config.heading_p95_scale_deg,
    ]
    total_weight = (
        config.lateral_mean_weight
        + config.heading_mean_weight
        + config.lateral_p95_weight
        + config.heading_p95_weight
    )
    return sum(weighted_scores) / total_weight

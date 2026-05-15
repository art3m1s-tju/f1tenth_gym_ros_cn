"""用于 LQR 参数优化的约束式简化目标函数。"""
from __future__ import annotations

import math


# 用于归一化的参考尺度，使两个不同单位的指标可以放在一起平均。
_SCALE_LATERAL_MEAN = 0.05       # 5 厘米
_SCALE_HEADING_MEAN_DEG = 3.0    # 3 度

# 硬约束阈值：超过这些阈值说明参数不适合进入排序。
_MAX_LATERAL_ERROR = 0.25             # 25 厘米
_MAX_STEERING_SATURATION_RATIO = 0.10 # 超过 10% 时间接近转向限幅
_MAX_STEERING_RATE_RMS_DEG_S = 100.0  # 转向命令变化速度 RMS


def compute_objective(metrics: dict[str, float]) -> float:
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
    mean_heading = metrics.get("mean_abs_e_psi_deg", math.inf)
    steering_rate = metrics.get("steering_rate_rms_deg_s", 0.0)
    steering_sat_ratio = metrics.get("steering_saturation_ratio", 0.0)

    required_values = [mean_ey, max_ey, mean_heading, steering_rate, steering_sat_ratio]
    if any(math.isinf(v) or math.isnan(v) for v in required_values):
        return math.inf

    if max_ey > _MAX_LATERAL_ERROR:
        return math.inf
    if steering_sat_ratio > _MAX_STEERING_SATURATION_RATIO:
        return math.inf
    if steering_rate > _MAX_STEERING_RATE_RMS_DEG_S:
        return math.inf

    lateral_score = mean_ey / _SCALE_LATERAL_MEAN
    heading_score = mean_heading / _SCALE_HEADING_MEAN_DEG
    return 0.5 * (lateral_score + heading_score)

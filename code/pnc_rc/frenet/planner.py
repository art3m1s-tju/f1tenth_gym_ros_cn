"""CPU Frenet local planner primitives for static obstacle avoidance."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage
from scipy.spatial import KDTree

from pnc_rc.lqr.geometry import (
    PathProjection,
    advance_projection_along_path,
    interpolate_angle,
    project_to_path,
)
from pnc_rc.lqr.math import compute_path_curvatures, compute_path_headings, wrap_angle


MIN_VALID_SCAN_RANGE_M = 1e-3
MIN_CURVATURE_SAMPLE_SPACING_M = 0.03


@dataclass(frozen=True)
class FrenetState:
    """车辆在参考线 Frenet 坐标系下的瞬时状态。

    Attributes:
        s: 沿参考线的弧长坐标，单位 m。
        d: 相对参考线的横向偏移，单位 m；正负号由参考线法向决定。
        s_dot: 参考线方向速度，单位 m/s。规划器会强制使用非负前进速度。
        d_dot: 横向速度，单位 m/s。
        s_ddot: 参考线方向加速度，单位 m/s^2。
        d_ddot: 横向加速度，单位 m/s^2。
    """

    s: float
    d: float
    s_dot: float
    d_dot: float
    s_ddot: float
    d_ddot: float


@dataclass(frozen=True)
class FrenetPlannerConfig:
    """Frenet 候选轨迹采样、碰撞判定和打分参数。

    Attributes:
        d_min: 横向终点采样下界，单位 m。
        d_max: 横向终点采样上界，单位 m。
        d_step: 横向终点采样间隔，单位 m。
        t_min: 候选轨迹最短时域，单位 s。
        t_max: 候选轨迹最长时域，单位 s。
        t_step: 时域采样间隔，单位 s。
        v_min: 纵向终点速度采样下界，单位 m/s。
        v_max: 纵向终点速度采样上界，单位 m/s。
        v_step: 纵向终点速度采样间隔，单位 m/s。
        trajectory_dt: 候选轨迹离散化时间步长，单位 s。
        target_speed: 候选轨迹速度误差项的目标速度，单位 m/s。
        max_curvature: 候选轨迹最大曲率硬约束，单位 1/m。
        safe_clearance: 期望安全裕度；低于该值会进入 soft cost，单位 m。
        min_clearance_m: 最小安全裕度硬约束，低于该值直接拒绝，单位 m。
        corridor_radius_m: 轨迹中心线扫掠半径，用于近似车体通道，单位 m。
        corridor_sample_step_m: 通道横向采样间隔，单位 m。
        path_collision_sample_step_m: 沿轨迹加密采样间隔，单位 m。
        footprint_front_m: 车体前向 footprint 额外检查距离，单位 m。
        footprint_rear_m: 车体后向 footprint 额外检查距离，单位 m。
        weight_*: 各 soft cost 项权重。
        max_heading_jump: 相邻轨迹点航向跳变硬约束，单位 rad。
        min_progress_step_m: 纵向进度最小单调前进约束，单位 m。
        max_initial_s_accel_mps2: 纵向多项式初始加速度限幅，单位 m/s^2；
            正负方向都会保留。
        max_reliable_initial_s_accel_mps2: 差分估计初始加速度的可信上界，
            超过该值认为估计离谱并回退到 0，单位 m/s^2。
    """

    d_min: float = -1.0
    d_max: float = 1.0
    d_step: float = 0.1
    t_min: float = 2.0
    t_max: float = 3.0
    t_step: float = 0.5
    v_min: float = 0.6
    v_max: float = 2.5
    v_step: float = 0.3
    trajectory_dt: float = 0.1
    target_speed: float = 1.5
    max_curvature: float = 1.14
    safe_clearance: float = 0.35
    min_clearance_m: float = 0.05
    corridor_radius_m: float = 0.14
    corridor_sample_step_m: float = 0.10
    path_collision_sample_step_m: float = 0.05
    footprint_front_m: float = 0.38
    footprint_rear_m: float = 0.05
    weight_lateral_jerk: float = 0.15
    weight_longitudinal_jerk: float = 0.08
    weight_time: float = 0.15
    weight_lateral_offset: float = 0.6
    weight_speed_error: float = 0.7
    weight_obstacle_clearance: float = 24.0
    weight_curvature: float = 0.4
    weight_curvature_rate: float = 8.0
    weight_lateral_shift: float = 1.2
    max_heading_jump: float = 0.65
    min_progress_step_m: float = 0.20
    max_initial_s_accel_mps2: float = 2.0
    max_reliable_initial_s_accel_mps2: float = 3.0


@dataclass(frozen=True)
class LocalGridConfig:
    """车辆局部坐标系下的占据栅格配置。

    Attributes:
        forward_m: 栅格向车前覆盖距离，单位 m。
        rear_m: 栅格向车后覆盖距离，单位 m。
        half_width_m: 栅格左右半宽，单位 m。
        resolution_m: 栅格分辨率，单位 m/cell。
        inflation_radius_m: 对障碍物边缘栅格做膨胀的半径，单位 m。
        scan_offset_x_m: 雷达相对车辆参考点的 x 方向偏移，单位 m。
    """

    forward_m: float = 7.0
    rear_m: float = 1.0
    half_width_m: float = 3.0
    resolution_m: float = 0.05
    inflation_radius_m: float = 0.28
    scan_offset_x_m: float = 0.275


@dataclass
class ReferencePath:
    """全局参考线及其 Frenet 投影/采样缓存。

    该类把全局中心线封装成可查询的几何对象：`project()` 用于把世界坐标投影
    到 `(s, d)`，`sample()` 用于从 `(s, d)` 还原世界坐标。Frenet 局部规划
    的所有候选轨迹都会通过这里从多项式坐标转回世界坐标。
    """

    points: np.ndarray
    headings: np.ndarray
    curvatures: np.ndarray
    segment_lengths: np.ndarray
    cumulative_s: np.ndarray
    kdtree: KDTree
    total_length: float
    closed_loop: bool = True

    @classmethod
    def from_points(cls, points: np.ndarray, closed_loop: bool = True) -> "ReferencePath":
        """从离散参考线点构造 Frenet 参考路径。

        Args:
            points: 形状为 `(N, 2)` 的世界坐标点序列。
            closed_loop: 是否把参考线视为闭环赛道。

        Returns:
            带 KDTree、弧长、航向和曲率缓存的 `ReferencePath`。

        Raises:
            ValueError: 当参考线点数少于 3 时抛出。
        """
        if len(points) < 3:
            raise ValueError("reference path needs at least 3 points")
        points = np.asarray(points, dtype=float)
        if closed_loop:
            path_for_segments = np.vstack([points, points[:1]])
        else:
            path_for_segments = points
        segment_lengths = np.linalg.norm(np.diff(path_for_segments, axis=0), axis=1)
        total_length = float(np.sum(segment_lengths))
        cumulative_s = np.concatenate([[0.0], np.cumsum(segment_lengths)])
        if closed_loop:
            cumulative_s = cumulative_s[:-1]
        return cls(
            points=points,
            headings=compute_path_headings(points, closed_loop=closed_loop),
            curvatures=compute_path_curvatures(points, closed_loop=closed_loop),
            segment_lengths=segment_lengths,
            cumulative_s=cumulative_s,
            kdtree=KDTree(points),
            total_length=total_length,
            closed_loop=closed_loop,
        )

    def project(self, position: np.ndarray) -> tuple[float, float, float, float]:
        """把世界坐标投影到整条参考线。

        Args:
            position: 世界坐标 `(x, y)`。

        Returns:
            `(s, d, heading, curvature)`，分别为参考线弧长、横向偏移、
            投影点航向和曲率。
        """
        projection = project_to_path(
            position,
            self.points,
            self.kdtree,
            self.headings,
            self.curvatures,
            closed_loop=self.closed_loop,
        )
        return self._projection_to_frenet(projection)

    def project_near(
        self,
        position: np.ndarray,
        s_hint: float,
        search_window_m: float,
    ) -> tuple[float, float, float, float]:
        """在上一帧弧长附近做局部投影，降低闭环赛道误投影概率。

        Args:
            position: 当前世界坐标 `(x, y)`。
            s_hint: 上一帧或预测的参考线弧长。
            search_window_m: 允许搜索的弧长窗口，单位 m；小于等于 0 时退化为
                全局投影。

        Returns:
            `(s, d, heading, curvature)`。
        """
        if search_window_m <= 0.0:
            return self.project(position)
        wrapped_hint = (
            float(s_hint % self.total_length)
            if self.closed_loop
            else float(np.clip(s_hint, 0.0, self.total_length))
        )
        candidate_segments = self._candidate_segments_near_s(
            wrapped_hint,
            search_window_m,
        )
        if not candidate_segments:
            return self.project(position)
        projection = self._project_to_candidate_segments(position, candidate_segments)
        return self._projection_to_frenet(projection)

    def sample(self, s: float, d: float) -> tuple[np.ndarray, float, float]:
        """从 Frenet 坐标采样世界坐标。

        Args:
            s: 参考线弧长坐标，单位 m。
            d: 横向偏移，单位 m。

        Returns:
            `(point, heading, curvature)`，其中 `point` 为世界坐标 `(x, y)`。
        """
        if self.closed_loop:
            s = float(s % self.total_length)
        else:
            s = float(np.clip(s, 0.0, self.total_length))
        segment_idx = int(np.searchsorted(self.cumulative_s, s, side="right") - 1)
        max_segment_idx = len(self.points) - 1 if self.closed_loop else len(self.points) - 2
        segment_idx = int(np.clip(segment_idx, 0, max_segment_idx))
        segment_length = float(self.segment_lengths[segment_idx])
        if segment_length <= 1e-9:
            t = 0.0
        else:
            t = float((s - self.cumulative_s[segment_idx]) / segment_length)
        next_idx = (
            (segment_idx + 1) % len(self.points)
            if self.closed_loop
            else min(segment_idx + 1, len(self.points) - 1)
        )
        point = (
            self.points[segment_idx]
            + t * (self.points[next_idx] - self.points[segment_idx])
        )
        heading = interpolate_angle(
            float(self.headings[segment_idx]),
            float(self.headings[next_idx]),
            t,
        )
        curvature = float(
            (1.0 - t) * self.curvatures[segment_idx]
            + t * self.curvatures[next_idx]
        )
        normal = np.array([-math.sin(heading), math.cos(heading)])
        return point + d * normal, heading, curvature

    def _projection_to_frenet(
        self,
        projection: PathProjection,
    ) -> tuple[float, float, float, float]:
        """把几何投影结果转换成 Frenet 状态量。

        Args:
            projection: `project_to_path()` 或近邻搜索得到的路径投影。

        Returns:
            `(s, d, heading, curvature)`。`s` 是沿参考线弧长，`d` 是带符号横向误差。
        """
        segment_length = float(self.segment_lengths[projection.segment_idx])
        s = float(
            self.cumulative_s[projection.segment_idx]
            + projection.segment_t * segment_length
        )
        return s, projection.lateral_error, projection.heading, projection.curvature

    def _candidate_segments_near_s(
        self,
        s_hint: float,
        search_window_m: float,
    ) -> list[int]:
        """筛选靠近上一帧 `s` 的参考线段。

        Args:
            s_hint: 上一帧或预测的参考线弧长，单位 m。
            search_window_m: 允许搜索的弧长窗口半径，单位 m。

        Returns:
            候选线段索引列表。闭环参考线会按环形距离处理，避免起终点跳变。
        """
        segment_mid_s = self.cumulative_s[: len(self.segment_lengths)] + 0.5 * self.segment_lengths
        if self.closed_loop:
            segment_mid_s = segment_mid_s % self.total_length
            arc_distance = np.abs(
                ((segment_mid_s - s_hint + 0.5 * self.total_length) % self.total_length)
                - 0.5 * self.total_length
            )
        else:
            arc_distance = np.abs(segment_mid_s - s_hint)
        candidate_segments = np.flatnonzero(
            arc_distance <= (search_window_m + 0.5 * self.segment_lengths)
        )
        return [int(segment_idx) for segment_idx in candidate_segments]

    def _project_to_candidate_segments(
        self,
        position: np.ndarray,
        candidate_segments: list[int],
    ) -> PathProjection:
        """只在给定线段集合上做点到参考线投影。

        Args:
            position: 待投影的世界坐标点。
            candidate_segments: 允许参与投影的参考线段索引。

        Returns:
            距离 `position` 最近的 `PathProjection`。
        """
        best_dist = float("inf")
        best_point = self.points[candidate_segments[0]].copy()
        best_seg_idx = candidate_segments[0]
        best_t = 0.0
        best_heading = float(self.headings[best_seg_idx])
        best_curvature = float(self.curvatures[best_seg_idx])

        for seg_idx in candidate_segments:
            start = self.points[seg_idx]
            next_idx = (
                (seg_idx + 1) % len(self.points)
                if self.closed_loop
                else min(seg_idx + 1, len(self.points) - 1)
            )
            end = self.points[next_idx]
            segment = end - start
            segment_length_sq = float(segment @ segment)
            if segment_length_sq <= 1e-12:
                continue
            t = float(
                np.clip(((position - start) @ segment) / segment_length_sq, 0.0, 1.0)
            )
            projected = start + t * segment
            distance = float(np.linalg.norm(position - projected))
            if distance >= best_dist:
                continue
            best_dist = distance
            best_point = projected
            best_seg_idx = seg_idx
            best_t = t
            best_heading = interpolate_angle(
                float(self.headings[seg_idx]),
                float(self.headings[next_idx]),
                t,
            )
            best_curvature = float(
                (1.0 - t) * self.curvatures[seg_idx]
                + t * self.curvatures[next_idx]
            )

        normal = np.array([-math.sin(best_heading), math.cos(best_heading)])
        lateral_error = float((position - best_point) @ normal)
        closest_idx = (
            best_seg_idx
            if best_t < 0.5
            else (
                (best_seg_idx + 1) % len(self.points)
                if self.closed_loop
                else min(best_seg_idx + 1, len(self.points) - 1)
            )
        )
        return PathProjection(
            point=best_point,
            heading=best_heading,
            curvature=best_curvature,
            lateral_error=lateral_error,
            segment_idx=best_seg_idx,
            segment_t=best_t,
            closest_idx=closest_idx,
        )


@dataclass(frozen=True)
class OccupancyGrid:
    """车辆局部占据栅格和距离场。

    Attributes:
        occupied: 膨胀后的占据栅格，`True` 表示该 cell 不可通行。
        distance_to_obstacle_m: 每个 cell 到膨胀障碍边界的距离，单位 m。
        config: 栅格分辨率、前后范围、半宽和膨胀半径配置。
    """

    occupied: np.ndarray
    distance_to_obstacle_m: np.ndarray
    config: LocalGridConfig

    @property
    def has_obstacles(self) -> bool:
        """判断局部栅格中是否存在障碍。

        Returns:
            至少一个 cell 被占据时返回 `True`。
        """
        return bool(np.any(self.occupied))

    def query_path(
        self,
        xy_points: np.ndarray,
        vehicle_pose: tuple[float, float, float],
        corridor_radius_m: float = 0.0,
        corridor_sample_step_m: float = 0.10,
        footprint_front_m: float = 0.0,
        footprint_rear_m: float = 0.0,
        path_sample_step_m: float = 0.05,
    ) -> tuple[bool, float]:
        """查询一条世界坐标路径的扫掠通道是否碰撞。

        Args:
            xy_points: 世界坐标路径点，形状为 `(N, 2)`。
            vehicle_pose: 车辆世界位姿 `(x, y, yaw)`。
            corridor_radius_m: 横向走廊半径，单位 m。
            corridor_sample_step_m: 走廊采样间隔，单位 m。
            footprint_front_m: 车辆前悬扫掠长度，单位 m。
            footprint_rear_m: 车辆后悬扫掠长度，单位 m。
            path_sample_step_m: 沿路径加密采样的最大间隔，单位 m。

        Returns:
            `(collision, min_clearance_m)`。`collision=True` 表示扫掠点落入占据
            cell；`min_clearance_m` 是扫掠点到障碍膨胀边界的最小距离。
        """
        if len(xy_points) == 0:
            return False, float("inf")
        dense_points = densify_path_points(xy_points, path_sample_step_m)
        query_points = swept_corridor_points(
            dense_points,
            corridor_radius_m,
            corridor_sample_step_m,
            footprint_front_m,
            footprint_rear_m,
        )
        local_xy = world_to_vehicle(query_points, vehicle_pose)
        rows, cols, inside = self.local_points_to_indices(local_xy)
        if not np.any(inside):
            return False, float("inf")
        inside_rows = rows[inside]
        inside_cols = cols[inside]
        collision = bool(np.any(self.occupied[inside_rows, inside_cols]))
        min_distance = float(np.min(self.distance_to_obstacle_m[inside_rows, inside_cols]))
        return collision, min_distance

    def local_points_to_indices(
        self,
        local_xy: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """将车体局部坐标点转换为栅格索引。

        Args:
            local_xy: 车体坐标系下的点，x 向前、y 向左，形状为 `(N, 2)`。

        Returns:
            `(rows, cols, inside)`。`inside` 标记对应点是否落在局部栅格内。
        """
        return _local_points_to_indices(local_xy, self.occupied.shape, self.config)


@dataclass(frozen=True)
class CandidatePath:
    """单条 Frenet 候选轨迹及其评价指标。

    Attributes:
        xy: 世界坐标轨迹点，形状为 `(N, 2)`。
        s: 轨迹点对应的参考线弧长序列。
        d: 轨迹点对应的横向偏移序列。
        s_dot: 纵向速度序列。
        d_dot: 横向速度序列。
        cost: 软约束综合代价，越小越优。
        min_clearance_m: 轨迹扫掠通道到障碍物的最小裕度，单位 m。
        max_curvature: 轨迹最大曲率绝对值，单位 1/m。
    """

    xy: np.ndarray
    s: np.ndarray
    d: np.ndarray
    s_dot: np.ndarray
    d_dot: np.ndarray
    cost: float
    min_clearance_m: float
    max_curvature: float


@dataclass
class FrenetPlanStats:
    """单次规划的候选统计，用于日志诊断。

    Attributes:
        total_candidates: 总采样候选数。
        collision_rejections: 因碰撞硬约束被拒绝的数量。
        progress_rejections: 因纵向进度非法被拒绝的数量。
        heading_rejections: 因航向跳变过大被拒绝的数量。
        curvature_rejections: 因曲率过大被拒绝的数量。
        clearance_rejections: 因最小裕度低于硬阈值被拒绝的数量。
        safe_candidates: 最终通过所有硬约束的候选数。
        best_clearance_m: 本轮候选中观测到的最大最小裕度，单位 m。
    """

    total_candidates: int = 0
    collision_rejections: int = 0
    progress_rejections: int = 0
    heading_rejections: int = 0
    curvature_rejections: int = 0
    clearance_rejections: int = 0
    safe_candidates: int = 0
    best_clearance_m: float = 0.0


def select_temporally_consistent_candidate(
    best_candidate: CandidatePath,
    safe_candidates: list[CandidatePath],
    *,
    previous_s_profile: np.ndarray | None = None,
    previous_d_profile: np.ndarray | None = None,
    profile_consistency_weight: float = 0.0,
    profile_max_jump_m: float = 0.0,
    profile_lookahead_m: float = 0.0,
    profile_unlock_clearance_gain_m: float = 0.0,
    safe_clearance_m: float,
    max_cost_gap: float = 3.0,
) -> tuple[CandidatePath, str]:
    """用单一 adjusted cost 在 safe candidates 中选择横向连续轨迹。

    `plan_frenet_path()` 已完成碰撞、clearance、曲率和前进性硬过滤；这里不再
    额外维护“中心线左/右侧”状态机，也不再比较远端终点 `d`。这里只比较上一条
    轨迹和当前候选在近距离重叠弧长内的 `d(s)` profile。若 raw best 的 clearance 已明显更好，则允许
    立即解锁，避免稳定性惩罚把车粘在安全裕度不足的旧通道。

    Args:
        best_candidate: 原始 cost 最低的候选。
        safe_candidates: 已通过硬碰撞/曲率/裕度检查的候选集合。
        previous_s_profile: 上一条选中轨迹的弧长序列，用于通道 profile 对齐。
        previous_d_profile: 上一条选中轨迹的横向偏移序列。
        profile_consistency_weight: profile 平均横向误差惩罚权重。
        profile_max_jump_m: raw best 超过该 profile 跳变时才考虑 clearance 解锁。
        profile_lookahead_m: 从当前候选起点向前比较的 profile 长度，单位 m。
        profile_unlock_clearance_gain_m: raw best 至少提升这么多 clearance 才允许
            忽略连续性惩罚，单位 m。
        safe_clearance_m: 期望安全裕度，单位 m。
        max_cost_gap: 连续性选择相对 raw best 允许增加的最大原始 cost。

    Returns:
        `(selected_candidate, reason)`。`reason` 用于日志解释选择来源。
    """
    # 没有 safe candidates 时无法二次选择，直接保留 raw best。
    if not safe_candidates:
        return best_candidate, "raw"

    # 整条 d(s) profile 连续性权重；用于抑制同侧内部左右摆动。
    profile_weight = max(0.0, float(profile_consistency_weight))
    # profile 最大跳变阈值；raw best 超过它时才认为 raw best 跳出了旧通道。
    profile_jump_limit = max(0.0, float(profile_max_jump_m))
    # profile 比较的前向距离；只比较近处轨迹，不比较整条远端尾巴。
    profile_lookahead = max(0.0, float(profile_lookahead_m))
    # raw best 至少多出这么多 clearance，才允许无视连续性惩罚直接切过去。
    unlock_clearance_gain = max(0.0, float(profile_unlock_clearance_gain_m))

    # key 是 safe_candidates 的下标，value 是 (平均 profile 误差, 最大 profile 误差)。
    profile_metrics: dict[int, tuple[float, float]] = {}
    # 只有上一条轨迹 profile 存在、lookahead 有效、profile 权重大于 0 时才计算 profile。
    use_profile = (
        previous_s_profile is not None
        and previous_d_profile is not None
        and profile_lookahead > 0.0
        and profile_weight > 0.0
    )
    # 没有可比较的上一条 profile 时，不做二次选择，避免用远端终点 d 伪造连续性。
    if not use_profile:
        return best_candidate, "raw"

    # 如果启用 profile 连续性，就逐条候选和上一条轨迹做 d(s) 对齐比较。
    # enumerate 保留候选下标，后面 adjusted_cost 可以 O(1) 查 profile 误差。
    for index, candidate in enumerate(safe_candidates):
        # lateral_profile_error 会在重叠 s 区间内插值比较两条 d(s) 曲线。
        metrics = lateral_profile_error(
            candidate.s,
            candidate.d,
            previous_s_profile,
            previous_d_profile,
            profile_lookahead,
        )
        # 没有足够重叠区间时 metrics 为 None，该候选不参与 profile 二次选择。
        if metrics is not None:
            profile_metrics[index] = metrics

    # 如果所有候选都和上一条轨迹没有重叠区间，保留 raw best。
    if not profile_metrics:
        return best_candidate, "raw"

    def adjusted_cost(index: int, candidate: CandidatePath) -> float:
        """计算原始 cost 和近距离 profile 连续性的统一排序值。"""
        # 候选必须有 profile_metrics 才会参与 min()；这里直接取平均 profile 误差。
        mean_error, _ = profile_metrics[index]
        # 从原始 cost 开始，只加入近距离 profile 平均误差二次惩罚。
        score = float(candidate.cost)
        # profile 平均误差二次惩罚，抑制同一侧内部 d(s) 大幅跳变。
        score += profile_weight * mean_error * mean_error
        # 返回统一排序分数，分数越小越优。
        return score

    # 只在可比较 profile 的候选中做二次选择，避免无重叠候选被误判为连续。
    indexed_candidates = [
        (index, candidate)
        for index, candidate in enumerate(safe_candidates)
        if index in profile_metrics
    ]
    # 在所有 safe candidates 中按 adjusted cost 选最优；并用原始 cost 做稳定 tie-break。
    selected_index, selected = min(
        indexed_candidates,
        key=lambda item: (adjusted_cost(item[0], item[1]), float(item[1].cost)),
    )
    # 找到 raw best 在 safe_candidates 中的下标，用于读取 raw best 的 profile 跳变。
    raw_index = next(
        (index for index, candidate in indexed_candidates if candidate is best_candidate),
        -1,
    )
    # 读取 raw best 的 profile 误差；如果 raw best 没有可比 profile，则返回 None。
    raw_metrics = profile_metrics.get(raw_index)
    # 判断 raw best 是否已经明显跳出上一条轨迹的横向通道。
    raw_outside_channel = (
        raw_metrics is not None
        and profile_jump_limit > 0.0
        and raw_metrics[1] > profile_jump_limit
    )
    # 正值表示 raw best 比当前 selected 有更大的最小 clearance。
    clearance_gain = float(best_candidate.min_clearance_m) - float(
        selected.min_clearance_m
    )
    # 如果 selected 不是 raw best，但 raw best 明显更安全，则允许打破连续性选择。
    if (
        selected is not best_candidate
        and raw_outside_channel
        and clearance_gain >= unlock_clearance_gain
        and float(best_candidate.min_clearance_m) >= float(safe_clearance_m)
    ):
        # 返回 higher_clearance，node 层会记录“为了安全裕度保留 raw best”。
        return best_candidate, "higher_clearance"

    # 原始 cost 是 plan_frenet_path() 算出的基础代价，不含 temporal/profile 惩罚。
    raw_cost = float(best_candidate.cost)
    # 如果连续性候选原始 cost 比 raw best 高太多，说明为了稳定牺牲过大。
    cost_gap = float(selected.cost) - raw_cost
    # 超过允许 cost gap 时，不接受连续性候选，退回 raw best。
    if selected is not best_candidate and cost_gap > float(max_cost_gap):
        return best_candidate, "lower_cost"
    # 如果 adjusted cost 最后还是选了 raw best，就直接返回 raw。
    if selected is best_candidate:
        return selected, "raw"
    # 取 selected 的最大 profile 跳变，用来区分“仍在同一通道”还是普通 temporal。
    _, selected_max_error = profile_metrics[selected_index]
    # selected 最大跳变在阈值内，标记为 same_profile，便于日志判断防蛇形生效。
    if profile_jump_limit <= 0.0 or selected_max_error <= profile_jump_limit:
        return selected, "same_profile"
    # 有 profile 但 selected 也超过阈值，只能说是 adjusted cost 的 temporal 选择。
    return selected, "temporal"


def lateral_profile_error(
    candidate_s: np.ndarray,
    candidate_d: np.ndarray,
    previous_s: np.ndarray,
    previous_d: np.ndarray,
    lookahead_m: float,
) -> tuple[float, float] | None:
    """计算候选轨迹和上一条轨迹在同一 `s` 区间内的横向 profile 差异。

    Args:
        candidate_s: 当前候选轨迹弧长序列。
        candidate_d: 当前候选轨迹横向偏移序列。
        previous_s: 上一条选中轨迹弧长序列。
        previous_d: 上一条选中轨迹横向偏移序列。
        lookahead_m: 从当前候选起点向前比较的最大弧长，单位 m。

    Returns:
        `(mean_abs_error, max_abs_error)`。如果两条轨迹没有足够重叠弧长，
        返回 `None`，调用方应退回到侧别/终点连续性逻辑。
    """
    candidate_s = np.asarray(candidate_s, dtype=float)
    candidate_d = np.asarray(candidate_d, dtype=float)
    previous_s = np.asarray(previous_s, dtype=float)
    previous_d = np.asarray(previous_d, dtype=float)
    if (
        len(candidate_s) < 2
        or len(candidate_d) != len(candidate_s)
        or len(previous_s) < 2
        or len(previous_d) != len(previous_s)
    ):
        return None

    candidate_s, candidate_d = _unique_monotonic_profile(candidate_s, candidate_d)
    previous_s, previous_d = _unique_monotonic_profile(previous_s, previous_d)
    if len(candidate_s) < 2 or len(previous_s) < 2:
        return None

    start_s = max(float(candidate_s[0]), float(previous_s[0]))
    end_s = min(
        float(candidate_s[-1]),
        float(previous_s[-1]),
        float(candidate_s[0]) + max(0.0, float(lookahead_m)),
    )
    if end_s <= start_s + 1e-6:
        return None

    sample_count = max(3, int(math.ceil((end_s - start_s) / 0.25)) + 1)
    sample_s = np.linspace(start_s, end_s, sample_count)
    candidate_profile = np.interp(sample_s, candidate_s, candidate_d)
    previous_profile = np.interp(sample_s, previous_s, previous_d)
    abs_error = np.abs(candidate_profile - previous_profile)
    return float(np.mean(abs_error)), float(np.max(abs_error))


def build_occupancy_grid(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    config: LocalGridConfig,
    static_occupied: np.ndarray | None = None,
) -> OccupancyGrid:
    """由 LaserScan 和静态地图构造车辆局部占据栅格。

    Args:
        ranges: LaserScan 距离数组。
        angle_min: 第一束激光角度，单位 rad。
        angle_increment: 相邻激光角度间隔，单位 rad。
        range_min: LaserScan 最小有效距离，单位 m。
        range_max: LaserScan 最大有效距离，单位 m。
        config: 局部栅格尺寸、分辨率和膨胀配置。
        static_occupied: 可选静态地图裁剪结果，坐标系与局部栅格一致。

    Returns:
        `OccupancyGrid`，其中 `occupied` 是膨胀后的障碍物，`distance_to_obstacle_m`
        是到膨胀边界的距离场。
    """
    height, width = local_grid_shape(config)
    raw = np.zeros((height, width), dtype=bool)
    if static_occupied is not None:
        raw |= static_occupied

    ranges = np.asarray(ranges, dtype=float)
    indices = np.arange(len(ranges), dtype=float)
    angles = angle_min + indices * angle_increment
    min_valid_range = max(float(range_min), MIN_VALID_SCAN_RANGE_M)
    valid = np.isfinite(ranges) & (ranges >= min_valid_range) & (ranges <= range_max)
    if np.any(valid):
        x = ranges[valid] * np.cos(angles[valid]) + config.scan_offset_x_m
        y = ranges[valid] * np.sin(angles[valid])
        rows, cols, inside = _local_points_to_indices(np.column_stack([x, y]), raw.shape, config)
        raw[rows[inside], cols[inside]] = True

    if not np.any(raw):
        occupied = np.zeros_like(raw, dtype=bool)
        clearance = np.full(raw.shape, float("inf"), dtype=float)
        return OccupancyGrid(
            occupied=occupied,
            distance_to_obstacle_m=clearance,
            config=config,
        )

    distance_to_raw = ndimage.distance_transform_edt(~raw) * config.resolution_m
    occupied = distance_to_raw <= config.inflation_radius_m
    clearance = np.maximum(0.0, distance_to_raw - config.inflation_radius_m)
    return OccupancyGrid(occupied=occupied, distance_to_obstacle_m=clearance, config=config)


def local_static_map_occupancy(
    map_occupied: np.ndarray,
    map_resolution: float,
    map_origin_xy: tuple[float, float],
    vehicle_pose: tuple[float, float, float],
    config: LocalGridConfig,
) -> np.ndarray:
    """把全局静态地图裁剪/旋转到车辆局部栅格。

    Args:
        map_occupied: 全局地图占据数组，`True` 表示障碍。
        map_resolution: 全局地图分辨率，单位 m/cell。
        map_origin_xy: 全局地图左下角世界坐标。
        vehicle_pose: 车辆世界位姿 `(x, y, yaw)`。
        config: 目标局部栅格配置。

    Returns:
        与 `build_occupancy_grid()` 原始局部栅格同形状的布尔数组。落在全局地图
        外的 cell 按占据处理，避免车辆规划到未知区域。
    """
    height, width = local_grid_shape(config)
    col_values = (np.arange(width, dtype=float) + 0.5) * config.resolution_m - config.rear_m
    row_values = (np.arange(height, dtype=float) + 0.5) * config.resolution_m - config.half_width_m
    local_x, local_y = np.meshgrid(col_values, row_values)
    world_x, world_y = vehicle_to_world(local_x, local_y, vehicle_pose)
    map_rows, map_cols, inside = world_points_to_map_indices(
        world_x,
        world_y,
        map_resolution,
        map_origin_xy,
        map_occupied.shape,
    )
    occupied = np.ones((height, width), dtype=bool)
    occupied[inside] = map_occupied[map_rows[inside], map_cols[inside]]
    return occupied


def local_grid_shape(config: LocalGridConfig) -> tuple[int, int]:
    """根据局部栅格配置计算数组形状。

    Args:
        config: 局部占据栅格配置。

    Returns:
        `(height, width)`。height 覆盖左右半宽，width 覆盖车后到车前的距离。
    """
    height = int(math.ceil((2.0 * config.half_width_m) / config.resolution_m))
    width = int(math.ceil((config.forward_m + config.rear_m) / config.resolution_m))
    return height, width


def world_to_vehicle(points_xy: np.ndarray, vehicle_pose: tuple[float, float, float]) -> np.ndarray:
    """把世界坐标点转换到车辆局部坐标系。

    Args:
        points_xy: 世界坐标点，形状为 `(N, 2)`。
        vehicle_pose: 车辆世界位姿 `(x, y, yaw)`。

    Returns:
        车体局部坐标点，x 向前、y 向左。
    """
    x, y, yaw = vehicle_pose
    shifted = np.asarray(points_xy, dtype=float) - np.array([x, y], dtype=float)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return np.column_stack(
        [
            cos_yaw * shifted[:, 0] + sin_yaw * shifted[:, 1],
            -sin_yaw * shifted[:, 0] + cos_yaw * shifted[:, 1],
        ]
    )


def vehicle_to_world(
    local_x: np.ndarray,
    local_y: np.ndarray,
    vehicle_pose: tuple[float, float, float],
) -> tuple[np.ndarray, np.ndarray]:
    """把车辆局部坐标网格转换到世界坐标。

    Args:
        local_x: 车体 x 坐标数组，x 向前。
        local_y: 车体 y 坐标数组，y 向左。
        vehicle_pose: 车辆世界位姿 `(x, y, yaw)`。

    Returns:
        `(world_x, world_y)`，形状与输入数组一致。
    """
    x, y, yaw = vehicle_pose
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    world_x = x + cos_yaw * local_x - sin_yaw * local_y
    world_y = y + sin_yaw * local_x + cos_yaw * local_y
    return world_x, world_y


def world_points_to_map_indices(
    world_x: np.ndarray,
    world_y: np.ndarray,
    map_resolution: float,
    map_origin_xy: tuple[float, float],
    map_shape: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """将世界坐标点转换为 OccupancyGrid 图像数组索引。

    Args:
        world_x: 世界 x 坐标数组。
        world_y: 世界 y 坐标数组。
        map_resolution: 地图分辨率，单位 m/cell。
        map_origin_xy: 地图左下角世界坐标。
        map_shape: 地图数组形状 `(height, width)`。

    Returns:
        `(rows, cols, inside)`。由于地图数组第 0 行在图像顶部，y 方向索引会做
        上下翻转。
    """
    cols = np.floor((world_x - map_origin_xy[0]) / map_resolution).astype(int)
    rows_from_bottom = np.floor((world_y - map_origin_xy[1]) / map_resolution).astype(int)
    rows = map_shape[0] - 1 - rows_from_bottom
    inside = (
        (rows >= 0)
        & (rows < map_shape[0])
        & (cols >= 0)
        & (cols < map_shape[1])
    )
    return rows, cols, inside


def initial_frenet_state(
    reference: ReferencePath,
    position: np.ndarray,
    yaw: float,
    velocity_xy: np.ndarray,
    previous_s: float | None,
    previous_s_dot: float | None,
    dt: float | None,
    projection_window_m: float = 6.0,
) -> FrenetState:
    """从里程计状态估计当前 Frenet 初始状态。

    Args:
        reference: 全局参考线。
        position: 当前世界坐标 `(x, y)`。
        yaw: 当前车身航向，单位 rad。
        velocity_xy: 世界坐标速度向量。
        previous_s: 上一次投影弧长，用于局部投影防跳点。
        previous_s_dot: 上一次纵向速度，用于估计纵向加速度。
        dt: 与上一规划周期的时间间隔，单位 s。
        projection_window_m: 局部投影搜索窗口，单位 m。

    Returns:
        当前 Frenet 状态。纵向速度会被截断为非负值，避免倒车噪声污染规划。
    """
    if previous_s is None:
        s, d, heading, _ = reference.project(position)
    else:
        s, d, heading, _ = reference.project_near(
            position,
            previous_s,
            projection_window_m,
        )
    tangent = np.array([math.cos(heading), math.sin(heading)])
    normal = np.array([-math.sin(heading), math.cos(heading)])
    s_dot = max(0.0, float(np.dot(velocity_xy, tangent)))
    d_dot = float(np.dot(velocity_xy, normal))
    if previous_s_dot is None or dt is None or dt <= 1e-6:
        s_ddot = 0.0
    else:
        s_ddot = float(np.clip((s_dot - previous_s_dot) / dt, -4.0, 4.0))
    return FrenetState(s=s, d=d, s_dot=s_dot, d_dot=d_dot, s_ddot=s_ddot, d_ddot=0.0)


def plan_frenet_path(
    reference: ReferencePath,
    state: FrenetState,
    occupancy: OccupancyGrid,
    vehicle_pose: tuple[float, float, float],
    config: FrenetPlannerConfig,
    stats: FrenetPlanStats | None = None,
    debug_candidates: list[CandidatePath] | None = None,
) -> CandidatePath | None:
    """枚举 Frenet 多项式候选并返回原始 cost 最低的安全轨迹。

    该函数只负责“生成候选”和“硬约束过滤”。同侧锁定、轨迹复用、预激活
    等 ROS 层状态机逻辑不在这里处理。

    Args:
        reference: 全局参考线。
        state: 当前 Frenet 初始状态。
        occupancy: 当前局部占据栅格。
        vehicle_pose: 车辆世界位姿 `(x, y, yaw)`。
        config: 采样、碰撞和 cost 配置。
        stats: 可选统计对象，用于记录拒绝原因。
        debug_candidates: 可选列表；传入后会填充所有 safe candidates 供 RViz 显示。

    Returns:
        原始 cost 最低的 `CandidatePath`；如果没有 safe candidate，则返回 `None`。
    """
    # 保存所有通过硬约束的候选轨迹，最后按 cost 排序选第一条。
    candidates: list[CandidatePath] = []
    # 过滤里程计/投影估计出来的异常纵向加速度，避免多项式初值不稳定。
    planning_state = _forward_progress_planning_state(state, config)
    # 第一层采样：枚举横向终点 d_final，也就是候选最终想停在中心线哪一侧。
    for d_final in _sample_range(config.d_min, config.d_max, config.d_step):
        # 第二层采样：枚举轨迹时长，时长越长通常越平滑但响应越慢。
        for duration in _sample_range(config.t_min, config.t_max, config.t_step):
            # 横向轨迹用五次多项式，约束起点 d/d_dot/d_ddot 和终点 d/d_dot=0/d_ddot=0。
            d_coeff = solve_quintic_lateral(planning_state, d_final, duration)
            # 用固定 dt 把连续多项式离散成候选轨迹点，末端加半个 dt 保证包含终点附近。
            time_values = np.arange(
                0.0,
                duration + 0.5 * config.trajectory_dt,
                config.trajectory_dt,
            )
            # 预先算横向位置、横向速度和横向 jerk；这些与终点速度无关，可复用。
            d_values, d_dot_values, _, d_jerk_values = evaluate_quintic(d_coeff, time_values)
            # 第三层采样：枚举纵向终点速度，决定这条候选快慢。
            for speed_final in _sample_range(config.v_min, config.v_max, config.v_step):
                # 统计总候选数，方便日志判断是采样太少还是过滤太严。
                if stats is not None:
                    stats.total_candidates += 1
                # 纵向轨迹用四次多项式，约束起点 s/s_dot/s_ddot 和终点 s_dot/s_ddot。
                s_coeff = solve_quartic_longitudinal(planning_state, speed_final, duration)
                # 计算纵向位置、速度和 jerk；速度用于前进性检查，jerk 用于 cost。
                s_values, s_dot_values, _, s_jerk_values = evaluate_quartic(s_coeff, time_values)
                # 硬过滤 1：候选必须一直前进，不能倒退，且总前进距离不能太短。
                progress_ok = _has_valid_progress_profile(
                    s_values,
                    s_dot_values,
                    config,
                )
                # 前进性不满足就直接丢弃，避免生成原地打转/倒车轨迹。
                if not progress_ok:
                    # 记录被前进性过滤掉的数量。
                    if stats is not None:
                        stats.progress_rejections += 1
                    continue
                # 把 Frenet 坐标 (s, d) 转成世界坐标点，后续所有几何检查都在 xy 上做。
                xy = np.array([reference.sample(s, d)[0] for s, d in zip(s_values, d_values)])
                # 计算相邻路径点的航向变化，用来发现离散点之间的尖锐折角。
                heading_jumps = estimate_heading_jumps(xy)
                # 没有足够点时认为航向跳变为 0；否则取最大绝对跳变。
                max_heading_jump = (
                    float(np.max(np.abs(heading_jumps)))
                    if len(heading_jumps)
                    else 0.0
                )
                # 硬过滤 2：航向跳变太大说明轨迹形状突兀，LQR 跟踪会不稳定。
                if max_heading_jump > config.max_heading_jump:
                    # 记录航向跳变过滤数量。
                    if stats is not None:
                        stats.heading_rejections += 1
                    continue
                # 硬过滤 3：用局部占据栅格检查整条轨迹扫掠走廊是否碰撞，并取最小 clearance。
                collision, min_clearance = occupancy.query_path(
                    xy,
                    vehicle_pose,
                    config.corridor_radius_m,
                    config.corridor_sample_step_m,
                    config.footprint_front_m,
                    config.footprint_rear_m,
                    config.path_collision_sample_step_m,
                )
                # 记录当前所有候选里见过的最大 clearance，用于无解日志诊断。
                if stats is not None:
                    stats.best_clearance_m = max(stats.best_clearance_m, min_clearance)
                # 只要扫掠走廊碰到占据栅格，直接丢弃。
                if collision:
                    # 记录碰撞过滤数量。
                    if stats is not None:
                        stats.collision_rejections += 1
                    continue
                # 硬过滤 4：没碰撞但离障碍太近，也直接丢弃。
                if min_clearance < config.min_clearance_m:
                    # 记录 clearance 过滤数量。
                    if stats is not None:
                        stats.clearance_rejections += 1
                    continue
                # 到这里说明候选已经通过安全硬约束，开始计算 soft cost。
                scored_candidate = score_candidate(
                    xy,
                    s_values,
                    d_values,
                    s_dot_values,
                    d_dot_values,
                    d_jerk_values,
                    s_jerk_values,
                    duration,
                    min_clearance,
                    config,
                )
                # 硬过滤 5：曲率超过车辆可跟踪上限，直接丢弃。
                if scored_candidate.max_curvature > config.max_curvature:
                    # 记录曲率过滤数量。
                    if stats is not None:
                        stats.curvature_rejections += 1
                    continue
                # 到这里才算真正 safe candidate。
                if stats is not None:
                    stats.safe_candidates += 1
                # 保存 safe candidate，后面按 cost 选 raw best。
                candidates.append(scored_candidate)
                # 如果调用方要 RViz debug，就把所有 safe candidates 也返回出去显示。
                if debug_candidates is not None:
                    debug_candidates.append(scored_candidate)
    # 没有任何 safe candidate 时返回 None，node 层会决定 stop 或复用旧轨迹。
    if not candidates:
        return None
    # 按 soft cost 从低到高排序；第一条就是原始最优候选。
    candidates.sort(key=lambda candidate: candidate.cost)
    # RViz 候选也按 cost 排序，方便 debug marker 前几个就是更优候选。
    if debug_candidates is not None:
        debug_candidates.sort(key=lambda candidate: candidate.cost)
    # 返回 raw best；node 层还可能用 temporal consistency 做二次选择。
    return candidates[0]


def score_candidate(
    xy: np.ndarray,
    s_values: np.ndarray,
    d_values: np.ndarray,
    s_dot_values: np.ndarray,
    d_dot_values: np.ndarray,
    d_jerk_values: np.ndarray,
    s_jerk_values: np.ndarray,
    duration: float,
    min_clearance: float,
    config: FrenetPlannerConfig,
) -> CandidatePath:
    """计算单条候选轨迹的 soft cost。

    碰撞、最小 clearance、航向跳变、曲率上限等硬约束已在调用前处理。
    这里的 cost 只用于 safe candidates 之间排序。

    Args:
        xy: 世界坐标轨迹点。
        s_values: 纵向弧长序列。
        d_values: 横向偏移序列。
        s_dot_values: 纵向速度序列。
        d_dot_values: 横向速度序列。
        d_jerk_values: 横向 jerk 序列。
        s_jerk_values: 纵向 jerk 序列。
        duration: 候选轨迹时长，单位 s。
        min_clearance: 轨迹最小障碍物裕度，单位 m。
        config: cost 权重和约束配置。

    Returns:
        带 cost、clearance 和曲率指标的 `CandidatePath`。
    """
    # 根据世界坐标路径点估计曲率，曲率越大越难跟踪。
    path_curvature = estimate_open_path_curvature(xy)
    # 有曲率数组时取最大绝对曲率；没有足够点时认为曲率为 0。
    if len(path_curvature):
        max_curvature = float(np.max(np.abs(path_curvature)))
    else:
        max_curvature = 0.0
    # 曲率变化率衡量路径是否局部弯弯绕绕；按弧长归一化后惩罚 dk/ds。
    curvature_rate = curvature_smoothness_cost(xy, path_curvature)
    # 横向相邻采样点最大变化量，用来惩罚局部横向突变。
    lateral_shift = float(np.max(np.abs(np.diff(d_values)))) if len(d_values) > 1 else 0.0
    # safe_clearance 是期望裕度；低于它但高于 min_clearance 的候选会被 soft penalty 惩罚。
    clearance_deficit = max(0.0, config.safe_clearance - min_clearance)
    # 终点速度越接近 target_speed，速度项 cost 越低。
    speed_error = float((config.target_speed - s_dot_values[-1]) ** 2)

    # 综合 soft cost：这里只在已经通过硬安全检查的候选之间排序。
    cost = (
        # 横向 jerk 越大，横向动作越急，惩罚越大。
        config.weight_lateral_jerk * float(np.sum(d_jerk_values**2))
        # 纵向 jerk 越大，加减速越不平顺，惩罚越大。
        + config.weight_longitudinal_jerk * float(np.sum(s_jerk_values**2))
        # 时间越长响应越慢，给一个小惩罚。
        + config.weight_time * duration
        # 终点离中心线越远，默认 cost 越高；障碍迫使绕行时仍可被 clearance/碰撞项接受。
        + config.weight_lateral_offset * float(d_values[-1] ** 2)
        # 终点速度偏离 target_speed 会被惩罚。
        + config.weight_speed_error * speed_error
        # clearance 低于期望安全裕度时二次惩罚；高于 safe_clearance 不再奖励。
        + config.weight_obstacle_clearance * clearance_deficit**2
        # 绝对曲率只给轻微偏好，避免把正常绕障弧线压回中心线。
        + config.weight_curvature * max_curvature**2
        # 曲率变化率是主要平滑项，抑制局部弯弯绕绕的候选。
        + config.weight_curvature_rate * curvature_rate
        # 横向采样跳变越大，路径越可能不好跟踪，惩罚越大。
        + config.weight_lateral_shift * lateral_shift
    )
    # 打包为 CandidatePath，后续 node 层和 RViz 都使用这个结构。
    return CandidatePath(
        # 世界坐标轨迹点，最终发布给 LQR 前还会按车辆当前位置重锚定。
        xy=xy,
        # 候选的纵向 Frenet 弧长序列，用于 profile 连续性比较。
        s=s_values,
        # 候选的横向 Frenet 偏移序列，用于 profile 连续性比较。
        d=d_values,
        # 纵向速度序列，用于调试和后续分析。
        s_dot=s_dot_values,
        # 横向速度序列，用于调试和后续分析。
        d_dot=d_dot_values,
        # safe candidate 排序用的最终 soft cost。
        cost=float(cost),
        # 该候选沿整条路径的最小障碍物裕度。
        min_clearance_m=float(min_clearance),
        # 该候选沿整条路径的最大曲率。
        max_curvature=max_curvature,
    )


def curvature_smoothness_cost(points: np.ndarray, path_curvature: np.ndarray) -> float:
    """计算曲率沿弧长变化的平滑性代价。

    这里惩罚的是 `dk/ds`，不是曲率绝对值。这样正常绕障所需的稳定弧线不会被
    过度压制，但局部左右弯绕、曲率突变的候选会得到更高 cost。

    Args:
        points: 世界坐标路径点，形状为 `(N, 2)`。
        path_curvature: 已估计的曲率序列；长度不匹配时会用压缩后的点重算。

    Returns:
        平均平方曲率变化率，单位约为 `1/m^4`。点数不足时返回 0。
    """
    filtered = _compress_path_samples(
        np.asarray(points, dtype=float),
        MIN_CURVATURE_SAMPLE_SPACING_M,
    )
    if len(filtered) < 3:
        return 0.0
    curvature = np.asarray(path_curvature, dtype=float)
    if len(curvature) != len(filtered):
        curvature = estimate_open_path_curvature(filtered)
    if len(curvature) < 3:
        return 0.0
    cumulative_s = _cumulative_arc_length(filtered)
    ds = np.diff(cumulative_s)
    valid = ds > 1e-6
    if np.count_nonzero(valid) == 0:
        return 0.0
    curvature_rate = np.diff(curvature)[valid] / ds[valid]
    return float(np.mean(curvature_rate**2))


def solve_quintic_lateral(state: FrenetState, d_final: float, duration: float) -> np.ndarray:
    """求解横向五次多项式 `d(t)`。

    Args:
        state: 当前 Frenet 初始状态，提供 `d/d_dot/d_ddot`。
        d_final: 目标终点横向偏移，单位 m。
        duration: 轨迹时长，单位 s。

    Returns:
        六个多项式系数 `[a0, a1, a2, a3, a4, a5]`，满足终点横向速度和加速度为 0。
    """
    a0 = state.d
    a1 = state.d_dot
    a2 = 0.5 * state.d_ddot
    t = duration
    matrix = np.array(
        [
            [t**3, t**4, t**5],
            [3.0 * t**2, 4.0 * t**3, 5.0 * t**4],
            [6.0 * t, 12.0 * t**2, 20.0 * t**3],
        ],
        dtype=float,
    )
    rhs = np.array(
        [
            d_final - (a0 + a1 * t + a2 * t**2),
            -(a1 + 2.0 * a2 * t),
            -(2.0 * a2),
        ],
        dtype=float,
    )
    a3, a4, a5 = np.linalg.solve(matrix, rhs)
    return np.array([a0, a1, a2, a3, a4, a5], dtype=float)


def solve_quartic_longitudinal(
    state: FrenetState,
    speed_final: float,
    duration: float,
) -> np.ndarray:
    """求解纵向四次多项式 `s(t)`。

    Args:
        state: 当前 Frenet 初始状态，提供 `s/s_dot/s_ddot`。
        speed_final: 目标终点纵向速度，单位 m/s。
        duration: 轨迹时长，单位 s。

    Returns:
        五个多项式系数 `[b0, b1, b2, b3, b4]`，满足终点加速度为 0。
    """
    b0 = state.s
    b1 = state.s_dot
    b2 = 0.5 * state.s_ddot
    t = duration
    matrix = np.array(
        [
            [3.0 * t**2, 4.0 * t**3],
            [6.0 * t, 12.0 * t**2],
        ],
        dtype=float,
    )
    rhs = np.array(
        [
            speed_final - (b1 + 2.0 * b2 * t),
            -(2.0 * b2),
        ],
        dtype=float,
    )
    b3, b4 = np.linalg.solve(matrix, rhs)
    return np.array([b0, b1, b2, b3, b4], dtype=float)


def evaluate_quintic(
    coefficients: np.ndarray,
    time_values: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """批量计算五次多项式的位置、速度、加速度和 jerk。

    Args:
        coefficients: `[a0, a1, a2, a3, a4, a5]`。
        time_values: 采样时间数组，单位 s。

    Returns:
        `(position, velocity, acceleration, jerk)`。
    """
    a0, a1, a2, a3, a4, a5 = coefficients
    t = time_values
    position = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
    velocity = a1 + 2.0 * a2 * t + 3.0 * a3 * t**2 + 4.0 * a4 * t**3 + 5.0 * a5 * t**4
    acceleration = 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t**2 + 20.0 * a5 * t**3
    jerk = 6.0 * a3 + 24.0 * a4 * t + 60.0 * a5 * t**2
    return position, velocity, acceleration, jerk


def evaluate_quartic(
    coefficients: np.ndarray,
    time_values: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """批量计算四次多项式的位置、速度、加速度和 jerk。

    Args:
        coefficients: `[b0, b1, b2, b3, b4]`。
        time_values: 采样时间数组，单位 s。

    Returns:
        `(position, velocity, acceleration, jerk)`。
    """
    b0, b1, b2, b3, b4 = coefficients
    t = time_values
    position = b0 + b1 * t + b2 * t**2 + b3 * t**3 + b4 * t**4
    velocity = b1 + 2.0 * b2 * t + 3.0 * b3 * t**2 + 4.0 * b4 * t**3
    acceleration = 2.0 * b2 + 6.0 * b3 * t + 12.0 * b4 * t**2
    jerk = 6.0 * b3 + 24.0 * b4 * t
    return position, velocity, acceleration, jerk


def estimate_open_path_curvature(points: np.ndarray) -> np.ndarray:
    """估计开放路径每个采样点的曲率。

    Args:
        points: 世界坐标路径点，形状为 `(N, 2)`。

    Returns:
        曲率数组，单位 1/m。点数不足或有效间距不足时返回 0 数组。
    """
    if len(points) < 3:
        return np.zeros(len(points), dtype=float)
    filtered = _compress_path_samples(points, MIN_CURVATURE_SAMPLE_SPACING_M)
    if len(filtered) < 3:
        return np.zeros(len(filtered), dtype=float)
    cumulative_s = _cumulative_arc_length(filtered)
    dx = np.gradient(filtered[:, 0], cumulative_s)
    dy = np.gradient(filtered[:, 1], cumulative_s)
    ddx = np.gradient(dx, cumulative_s)
    ddy = np.gradient(dy, cumulative_s)
    denominator = np.maximum((dx * dx + dy * dy) ** 1.5, 1e-6)
    return (dx * ddy - dy * ddx) / denominator


def estimate_heading_jumps(points: np.ndarray) -> np.ndarray:
    """估计相邻有效路径段之间的航向跳变。

    Args:
        points: 世界坐标路径点，形状为 `(N, 2)`。

    Returns:
        相邻段航向差数组，单位 rad，用于剔除突然掉头/折返的候选。
    """
    if len(points) < 3:
        return np.zeros(0, dtype=float)
    segment_vectors = np.diff(points, axis=0)
    segment_lengths = np.linalg.norm(segment_vectors, axis=1)
    valid = segment_lengths > 1e-6
    if np.count_nonzero(valid) < 2:
        return np.zeros(0, dtype=float)
    headings = np.arctan2(segment_vectors[valid, 1], segment_vectors[valid, 0])
    return np.array(
        [wrap_angle(float(curr - prev)) for prev, curr in zip(headings[:-1], headings[1:])],
        dtype=float,
    )


def swept_corridor_points(
    points: np.ndarray,
    corridor_radius_m: float,
    corridor_sample_step_m: float,
    footprint_front_m: float = 0.0,
    footprint_rear_m: float = 0.0,
) -> np.ndarray:
    """生成路径扫掠通道采样点。

    Args:
        points: 路径中心线点，形状为 `(N, 2)`。
        corridor_radius_m: 横向半宽，单位 m。
        corridor_sample_step_m: 横向/纵向 footprint 采样间隔，单位 m。
        footprint_front_m: 沿路径切向向前扩展距离，单位 m。
        footprint_rear_m: 沿路径切向向后扩展距离，单位 m。

    Returns:
        扩展后的采样点数组。碰撞检测会查询这些点所在的占据栅格 cell。
    """
    points = np.asarray(points, dtype=float)
    if len(points) == 0:
        return points.reshape(0, 2)
    radius = max(0.0, float(corridor_radius_m))
    front = max(0.0, float(footprint_front_m))
    rear = max(0.0, float(footprint_rear_m))
    if radius <= 1e-9 and front <= 1e-9 and rear <= 1e-9:
        return points
    step = max(1e-3, float(corridor_sample_step_m))
    offsets = np.arange(-radius, radius + 0.5 * step, step, dtype=float)
    if offsets[-1] < radius:
        offsets = np.append(offsets, radius)
    if not np.any(np.isclose(offsets, 0.0)):
        offsets = np.sort(np.append(offsets, 0.0))

    headings = estimate_path_headings(points)
    longitudinal_offsets = np.arange(-rear, front + 0.5 * step, step, dtype=float)
    if longitudinal_offsets[-1] < front:
        longitudinal_offsets = np.append(longitudinal_offsets, front)
    if not np.any(np.isclose(longitudinal_offsets, 0.0)):
        longitudinal_offsets = np.sort(np.append(longitudinal_offsets, 0.0))
    tangents = np.column_stack([np.cos(headings), np.sin(headings)])
    normals = np.column_stack([-np.sin(headings), np.cos(headings)])
    expanded = (
        points[:, None, None, :]
        + longitudinal_offsets[None, :, None, None] * tangents[:, None, None, :]
        + offsets[None, None, :, None] * normals[:, None, None, :]
    )
    return expanded.reshape(-1, 2)


def densify_path_points(points: np.ndarray, max_step_m: float) -> np.ndarray:
    """沿折线路径做空间加密。

    Args:
        points: 原始路径点，形状为 `(N, 2)`。
        max_step_m: 相邻输出点最大间距，单位 m。

    Returns:
        加密后的路径点。该步骤用于避免两个稀疏轨迹点之间穿过障碍物却漏检。
    """
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return points.reshape(-1, 2)
    max_step = float(max_step_m)
    if max_step <= 0.0:
        return points

    dense_points = [points[0]]
    for start, end in zip(points[:-1], points[1:]):
        delta = end - start
        distance = float(np.linalg.norm(delta))
        if distance <= 1e-12:
            continue
        subdivisions = max(1, int(math.ceil(distance / max_step)))
        for step_idx in range(1, subdivisions + 1):
            dense_points.append(start + (step_idx / subdivisions) * delta)
    return np.asarray(dense_points, dtype=float)


def trim_path_to_position(
    points: np.ndarray,
    position: np.ndarray,
    min_remaining_length_m: float = 0.0,
    anchor_lookahead_m: float = 0.0,
    max_remaining_length_m: float = 0.0,
) -> np.ndarray:
    """把局部路径重锚定到当前车辆位置附近。

    Args:
        points: 原始局部路径点。
        position: 当前车辆世界坐标。
        min_remaining_length_m: 裁剪后要求的最短剩余长度，单位 m。
        anchor_lookahead_m: 从最近投影点沿路径向前推进的锚点距离，单位 m。
        max_remaining_length_m: 裁剪后路径的最大长度，单位 m；非正数表示不截断。

    Returns:
        从当前车辆附近开始的新路径。若剩余长度不足，返回空数组。
    """
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return points.reshape(-1, 2)

    headings = estimate_path_headings(points)
    curvatures = compute_path_curvatures(points, closed_loop=False)
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    kdtree = KDTree(points)
    projection = project_to_path(
        np.asarray(position, dtype=float),
        points,
        kdtree,
        headings,
        curvatures,
        closed_loop=False,
    )
    if anchor_lookahead_m > 0.0:
        projection = advance_projection_along_path(
            np.asarray(position, dtype=float),
            projection,
            float(anchor_lookahead_m),
            points,
            headings,
            curvatures,
            segment_lengths,
            closed_loop=False,
        )

    anchor = np.asarray(projection.point, dtype=float).reshape(1, 2)
    remaining_points = points[min(projection.segment_idx + 1, len(points) - 1) :]
    trimmed = np.vstack([anchor, remaining_points])
    if polyline_length(trimmed) < max(0.0, float(min_remaining_length_m)):
        return np.empty((0, 2), dtype=float)
    trimmed = truncate_path_length(trimmed, max_remaining_length_m)
    return trimmed


def truncate_path_length(points: np.ndarray, max_length_m: float) -> np.ndarray:
    """按弧长截断路径，必要时在线段内插入精确终点。

    Args:
        points: 输入路径点，形状为 `(N, 2)`。
        max_length_m: 允许保留的最大弧长，单位 m；非正数表示不截断。

    Returns:
        截断后的路径。若原路径短于上限，则返回原路径副本。
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    max_length = float(max_length_m)
    if max_length <= 0.0 or len(points) < 2:
        return points.copy()

    kept = [points[0]]
    travelled = 0.0
    for start, end in zip(points[:-1], points[1:]):
        segment = end - start
        segment_length = float(np.linalg.norm(segment))
        if segment_length <= 1e-12:
            continue
        next_travelled = travelled + segment_length
        if next_travelled >= max_length:
            ratio = max(0.0, min(1.0, (max_length - travelled) / segment_length))
            kept.append(start + ratio * segment)
            return np.asarray(kept, dtype=float)
        kept.append(end)
        travelled = next_travelled
    return np.asarray(kept, dtype=float)


def sample_return_to_centerline_segment(
    reference: ReferencePath,
    start_s: float,
    start_d: float,
    length_m: float,
    step_m: float,
) -> np.ndarray:
    """采样一条从当前横向偏移平滑回到中心线的局部路径。

    Args:
        reference: Frenet 参考线。
        start_s: 当前车辆投影到参考线的弧长，单位 m。
        start_d: 当前车辆相对参考线的横向偏移，单位 m。
        length_m: 回中路径长度，单位 m。
        step_m: 采样间隔，单位 m。

    Returns:
        世界坐标路径点数组。第一点保持 `start_d`，终点横向偏移收敛到 0。
    """
    length = max(0.0, float(length_m))
    step = max(1e-3, float(step_m))
    sample_offsets = np.arange(0.0, length + 0.5 * step, step, dtype=float)
    if len(sample_offsets) < 2:
        sample_offsets = np.array([0.0, length], dtype=float)
    progress = np.clip(sample_offsets / max(length, 1e-6), 0.0, 1.0)
    smooth = progress * progress * (3.0 - 2.0 * progress)
    d_values = float(start_d) * (1.0 - smooth)
    return np.array(
        [
            reference.sample(float(start_s) + float(ds), float(d))[0]
            for ds, d in zip(sample_offsets, d_values)
        ],
        dtype=float,
    )


def path_pose_error(
    points: np.ndarray,
    position: np.ndarray,
    yaw: float,
) -> tuple[float, float]:
    """计算车辆位姿相对一条开放路径的横向距离和航向差。

    Args:
        points: 世界坐标路径点，形状为 `(N, 2)`。
        position: 当前车辆世界坐标。
        yaw: 当前车辆航向，单位 rad。

    Returns:
        `(distance_m, heading_error_rad)`。点数不足时返回无穷大，调用方应放弃
        复用该路径。
    """
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return float("inf"), float("inf")
    headings = estimate_path_headings(points)
    curvatures = compute_path_curvatures(points, closed_loop=False)
    projection = project_to_path(
        np.asarray(position, dtype=float),
        points,
        KDTree(points),
        headings,
        curvatures,
        closed_loop=False,
    )
    distance = float(np.linalg.norm(np.asarray(position, dtype=float) - projection.point))
    heading_error = abs(float(wrap_angle(float(yaw) - float(projection.heading))))
    return distance, heading_error


def sample_reference_segment(
    reference: ReferencePath,
    start_s: float,
    length_m: float,
    step_m: float,
    d: float = 0.0,
) -> np.ndarray:
    """从参考线采样一段固定横向偏移的局部路径。

    Args:
        reference: Frenet 参考线。
        start_s: 起始弧长，单位 m。
        length_m: 采样长度，单位 m。
        step_m: 采样间隔，单位 m。
        d: 横向偏移，单位 m。

    Returns:
        世界坐标路径点数组。
    """
    length = max(0.0, float(length_m))
    step = max(1e-3, float(step_m))
    sample_s = np.arange(0.0, length + 0.5 * step, step, dtype=float)
    if len(sample_s) == 0 or sample_s[-1] < length:
        sample_s = np.append(sample_s, length)
    return np.array(
        [reference.sample(start_s + offset, d)[0] for offset in sample_s],
        dtype=float,
    )


def polyline_length(points: np.ndarray) -> float:
    """计算折线路径长度。

    Args:
        points: 路径点数组。

    Returns:
        路径累计长度，单位 m。
    """
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def speed_based_activation_lookahead(
    speed_mps: float,
    *,
    base_lookahead_m: float,
    reaction_time_s: float,
    decel_mps2: float,
    min_lookahead_m: float,
    max_lookahead_m: float,
) -> float:
    """计算速度相关的 Frenet 激活距离。

    Args:
        speed_mps: 当前/目标速度，单位 m/s。
        base_lookahead_m: 基础前瞻距离，单位 m。
        reaction_time_s: 反应时间项，单位 s。
        decel_mps2: 期望减速度，单位 m/s^2。
        min_lookahead_m: 激活距离下限，单位 m。
        max_lookahead_m: 激活距离上限，单位 m。

    Returns:
        裁剪后的激活距离，单位 m。公式近似为
        `base + speed * reaction + speed^2 / (2 * decel)`。
    """
    speed = max(0.0, float(speed_mps))
    base = max(0.0, float(base_lookahead_m))
    reaction = max(0.0, float(reaction_time_s))
    decel = max(1e-6, float(decel_mps2))
    lower = max(0.0, float(min_lookahead_m))
    upper = max(lower, float(max_lookahead_m))
    raw = base + speed * reaction + speed * speed / (2.0 * decel)
    return float(np.clip(raw, lower, upper))


def estimate_path_headings(points: np.ndarray) -> np.ndarray:
    """估计路径每个点的切向航向。

    Args:
        points: 路径点数组，形状为 `(N, 2)`。

    Returns:
        每个点的航向角数组，单位 rad。
    """
    points = np.asarray(points, dtype=float)
    if len(points) < 2:
        return np.zeros(len(points), dtype=float)
    headings = np.zeros(len(points), dtype=float)
    for idx in range(len(points)):
        prev_point = points[max(0, idx - 1)]
        next_point = points[min(len(points) - 1, idx + 1)]
        delta = next_point - prev_point
        if float(delta @ delta) <= 1e-12:
            headings[idx] = headings[idx - 1] if idx > 0 else 0.0
        else:
            headings[idx] = math.atan2(float(delta[1]), float(delta[0]))
    return headings


def _has_valid_progress_profile(
    s_values: np.ndarray,
    s_dot_values: np.ndarray,
    config: FrenetPlannerConfig,
) -> bool:
    """检查纵向轨迹是否始终向前且总进度足够。

    Args:
        s_values: 候选轨迹纵向弧长序列。
        s_dot_values: 候选轨迹纵向速度序列。
        config: Frenet 规划配置。

    Returns:
        满足非负速度、非倒退、总进度超过阈值时返回 `True`。
    """
    if np.any(s_dot_values < -1e-6):
        return False
    progress_steps = np.diff(s_values)
    if np.any(progress_steps < -1e-6):
        return False
    total_progress = float(s_values[-1] - s_values[0]) if len(s_values) else 0.0
    if total_progress < config.min_progress_step_m:
        return False
    return True


def _forward_progress_planning_state(
    state: FrenetState,
    config: FrenetPlannerConfig | None = None,
) -> FrenetState:
    """过滤纵向多项式初始加速度中的离群估计。

    Args:
        state: 从里程计投影得到的原始 Frenet 状态。
        config: Frenet 规划配置。为空时使用 `FrenetPlannerConfig()` 默认阈值。

    Returns:
        适合生成前向轨迹的 Frenet 状态。可信范围内的正/负加速度都会保留；
        只有超过可靠上界的离群估计才会被拒绝为 0。
    """
    cfg = config or FrenetPlannerConfig()
    raw_s_ddot = float(state.s_ddot)
    reliable_limit = max(0.0, float(cfg.max_reliable_initial_s_accel_mps2))
    accel_limit = max(0.0, float(cfg.max_initial_s_accel_mps2))
    if not math.isfinite(raw_s_ddot):
        s_ddot = 0.0
    elif reliable_limit > 0.0 and abs(raw_s_ddot) > reliable_limit:
        s_ddot = 0.0
    elif accel_limit > 0.0:
        s_ddot = float(np.clip(raw_s_ddot, -accel_limit, accel_limit))
    else:
        s_ddot = 0.0
    if abs(s_ddot - raw_s_ddot) <= 1e-9:
        return state
    # Odom/projection-derived acceleration is noisy around path switches and
    # obstacle transitions. Keep plausible braking/acceleration direction, but
    # do not let an outlier become a hard quartic boundary condition.
    return FrenetState(
        s=state.s,
        d=state.d,
        s_dot=state.s_dot,
        d_dot=state.d_dot,
        s_ddot=s_ddot,
        d_ddot=state.d_ddot,
    )


def _sample_range(start: float, stop: float, step: float) -> np.ndarray:
    """生成包含端点附近值的一维采样序列。

    Args:
        start: 起始值。
        stop: 终止值。
        step: 采样间隔。

    Returns:
        采样数组。`step <= 0` 时只返回 `start`。
    """
    if step <= 0.0:
        return np.array([start], dtype=float)
    count = int(math.floor((stop - start) / step + 0.5)) + 1
    return start + step * np.arange(max(1, count), dtype=float)


def _compress_path_samples(points: np.ndarray, min_spacing_m: float) -> np.ndarray:
    """删除过近的路径采样点。

    Args:
        points: 原始路径点。
        min_spacing_m: 保留点之间的最小间距，单位 m。

    Returns:
        压缩后的路径点，始终尽量保留终点。
    """
    if len(points) < 2:
        return np.asarray(points, dtype=float)
    filtered = [np.asarray(points[0], dtype=float)]
    min_spacing_m = max(0.0, float(min_spacing_m))
    for point in np.asarray(points[1:], dtype=float):
        if np.linalg.norm(point - filtered[-1]) >= min_spacing_m:
            filtered.append(point)
    if np.linalg.norm(np.asarray(points[-1], dtype=float) - filtered[-1]) > 1e-9:
        filtered.append(np.asarray(points[-1], dtype=float))
    return np.asarray(filtered, dtype=float)


def _unique_monotonic_profile(
    s_values: np.ndarray,
    d_values: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """压缩 `s/d` profile，保证 `np.interp()` 需要的单调唯一弧长。

    Args:
        s_values: 原始弧长序列。
        d_values: 原始横向偏移序列。

    Returns:
        `(unique_s, unique_d)`。重复或倒退的弧长点会被跳过，保留第一次出现的点。
    """
    s_array = np.asarray(s_values, dtype=float)
    d_array = np.asarray(d_values, dtype=float)
    keep_s: list[float] = []
    keep_d: list[float] = []
    last_s = -float("inf")
    for s_value, d_value in zip(s_array, d_array):
        if not (math.isfinite(float(s_value)) and math.isfinite(float(d_value))):
            continue
        if float(s_value) <= last_s + 1e-9:
            continue
        keep_s.append(float(s_value))
        keep_d.append(float(d_value))
        last_s = float(s_value)
    return np.asarray(keep_s, dtype=float), np.asarray(keep_d, dtype=float)


def _cumulative_arc_length(points: np.ndarray) -> np.ndarray:
    """计算路径点对应的累计弧长。

    Args:
        points: 路径点数组。

    Returns:
        与输入等长的累计弧长数组，单位 m。
    """
    if len(points) < 2:
        return np.zeros(len(points), dtype=float)
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(segment_lengths)])


def _local_points_to_indices(
    local_xy: np.ndarray,
    shape: tuple[int, int],
    config: LocalGridConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """局部坐标到局部栅格索引的底层转换。

    Args:
        local_xy: 车体局部坐标点，形状为 `(N, 2)`。
        shape: 栅格形状 `(height, width)`。
        config: 局部栅格配置。

    Returns:
        `(rows, cols, inside)`。
    """
    cols = np.floor((local_xy[:, 0] + config.rear_m) / config.resolution_m).astype(int)
    rows = np.floor((local_xy[:, 1] + config.half_width_m) / config.resolution_m).astype(int)
    inside = (cols >= 0) & (cols < shape[1]) & (rows >= 0) & (rows < shape[0])
    return rows, cols, inside

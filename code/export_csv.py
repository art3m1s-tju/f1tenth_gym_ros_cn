#!/usr/bin/env python3
"""将处理后的赛道边界和中线导出为 CSV 赛道表。

输出格式遵循参考 `track.csv` 架构：
left_border_x,left_border_y,right_border_x,right_border_y,pos_x,pos_y,Lapdist
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.interpolate import splev, splprep
from scipy.spatial import cKDTree

import clean_map
from package_paths import get_default_map_yaml, get_default_output_root

DEFAULT_INPUT_YAML = get_default_map_yaml()
DEFAULT_OUTPUT_CSV = get_default_output_root() / "csv" / "processed_track.csv"
DEFAULT_OUTPUT_MAP_PREFIX = get_default_output_root() / "maps" / "processed_track_map"


def pixel_to_world(points_rc: np.ndarray, resolution: float, origin: list[float], image_height: int) -> np.ndarray:
    """将像素坐标 (行, 列) 转换为世界坐标 (x, y)。

    Args:
        points_rc: 像素坐标数组 [行, 列]。
        resolution: 地图分辨率（米/像素）。
        origin: 地图原点 [x, y, yaw]。
        image_height: 图像高度（像素）。

    Returns:
        世界坐标数组 [x, y]。
    """
    rows = points_rc[:, 0]
    cols = points_rc[:, 1]
    x = origin[0] + (cols + 0.5) * resolution
    y = origin[1] + (image_height - rows - 0.5) * resolution
    return np.column_stack([x, y])


def resample_closed_curve(points_xy: np.ndarray, target_count: int) -> np.ndarray:
    """将闭合曲线重采样为等距离分布的固定数量点。

    Args:
        points_xy: 输入的世界坐标点集。
        target_count: 目标点数。

    Returns:
        重采样后的点集。
    """
    if len(points_xy) < 3:
        return points_xy
    closed = np.vstack([points_xy, points_xy[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    total = float(seg.sum())
    if total <= 1e-9:
        return points_xy
    cumulative = np.concatenate([[0.0], np.cumsum(seg)])
    targets = np.linspace(0.0, total, target_count + 1)[:-1]

    sampled = []
    seg_idx = 0
    for dist in targets:
        while seg_idx < len(seg) - 1 and cumulative[seg_idx + 1] < dist:
            seg_idx += 1
        start = closed[seg_idx]
        end = closed[seg_idx + 1]
        seg_len = seg[seg_idx]
        if seg_len <= 1e-9:
            sampled.append(start)
            continue
        alpha = (dist - cumulative[seg_idx]) / seg_len
        sampled.append((1.0 - alpha) * start + alpha * end)
    return np.array(sampled, dtype=float)


def remove_near_duplicate_points(points_xy: np.ndarray, min_spacing: float) -> np.ndarray:
    """Remove consecutive points that are too close on a closed curve.

    Boundary extraction works on a raster map, so it can occasionally produce
    repeated or almost-repeated points. Those points create artificial curvature
    spikes later in the optimizer, even though the physical wall is smooth.
    """
    if len(points_xy) < 3:
        return points_xy

    min_spacing = max(0.0, float(min_spacing))
    if min_spacing <= 0.0:
        return points_xy

    kept = [points_xy[0]]
    for point in points_xy[1:]:
        if np.linalg.norm(point - kept[-1]) >= min_spacing:
            kept.append(point)

    if len(kept) > 2 and np.linalg.norm(kept[0] - kept[-1]) < min_spacing:
        kept[-1] = 0.5 * (kept[-1] + kept[0])
        kept.pop(0)

    if len(kept) < 3:
        return points_xy
    return np.asarray(kept, dtype=float)


def smooth_closed_curve_spline(
    points_xy: np.ndarray,
    smoothing: float,
    target_count: int,
) -> np.ndarray:
    """Fit a periodic B-spline and resample a closed curve."""
    if len(points_xy) < 8 or target_count < 8 or smoothing <= 0.0:
        return resample_closed_curve(points_xy, target_count)

    unique_points = remove_near_duplicate_points(points_xy, min_spacing=1e-4)
    if len(unique_points) < 8:
        return resample_closed_curve(points_xy, target_count)

    try:
        tck, _ = splprep(
            [unique_points[:, 0], unique_points[:, 1]],
            s=float(smoothing),
            per=True,
            k=min(3, len(unique_points) - 1),
        )
        u = np.linspace(0.0, 1.0, target_count, endpoint=False)
        x_s, y_s = splev(u, tck)
        smoothed = np.column_stack([x_s, y_s])
        return resample_closed_curve(smoothed, target_count)
    except Exception as exc:
        print(f"[export_csv] Warning: spline smoothing failed ({exc}); using arc-length resampling.")
        return resample_closed_curve(points_xy, target_count)


def resample_boundary_pairs_by_centerline(
    inner_xy: np.ndarray,
    outer_xy: np.ndarray,
    target_count: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Resample paired boundaries using the centerline arc length.

    Resampling the two walls independently can destroy point-to-point pairing.
    This function keeps the lateral vector paired with the centerline sample.
    """
    if len(inner_xy) != len(outer_xy) or len(inner_xy) < 3:
        return inner_xy, outer_xy

    center = 0.5 * (inner_xy + outer_xy)
    half_width = 0.5 * (outer_xy - inner_xy)
    center_closed = np.vstack([center, center[:1]])
    width_closed = np.vstack([half_width, half_width[:1]])
    seg = np.linalg.norm(np.diff(center_closed, axis=0), axis=1)
    total = float(seg.sum())
    if total <= 1e-9:
        return inner_xy, outer_xy

    cumulative = np.concatenate([[0.0], np.cumsum(seg)])
    targets = np.linspace(0.0, total, target_count + 1)[:-1]
    center_resampled = np.column_stack(
        [
            np.interp(targets, cumulative, center_closed[:, 0]),
            np.interp(targets, cumulative, center_closed[:, 1]),
        ]
    )
    width_resampled = np.column_stack(
        [
            np.interp(targets, cumulative, width_closed[:, 0]),
            np.interp(targets, cumulative, width_closed[:, 1]),
        ]
    )
    return center_resampled - width_resampled, center_resampled + width_resampled


def min_closed_segment_length(points_xy: np.ndarray) -> float:
    """Return the shortest segment length on a closed polyline."""
    if len(points_xy) < 2:
        return 0.0
    closed = np.vstack([points_xy, points_xy[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    return float(seg.min()) if len(seg) else 0.0


def postprocess_boundary_pairs(
    inner_xy: np.ndarray,
    outer_xy: np.ndarray,
    *,
    target_count: int,
    min_spacing_m: float = 0.03,
    savgol_window: int = 31,
    spline_smoothing: float = 0.10,
) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
    """Clean raster stair-steps before exporting boundary/centerline CSV rows."""
    before_inner_min = min_closed_segment_length(inner_xy)
    before_outer_min = min_closed_segment_length(outer_xy)

    inner, outer = sanitize_boundary_pairs(inner_xy, outer_xy)
    inner = clean_map.circular_savgol(inner, window=savgol_window, polyorder=2)
    outer = clean_map.circular_savgol(outer, window=savgol_window, polyorder=2)

    inner, outer = resample_boundary_pairs_by_centerline(inner, outer, target_count)

    center = 0.5 * (inner + outer)
    width = 0.5 * (outer - inner)
    center = remove_near_duplicate_points(center, min_spacing=min_spacing_m)
    center = smooth_closed_curve_spline(center, smoothing=spline_smoothing, target_count=target_count)
    width = clean_map.circular_savgol(width, window=savgol_window, polyorder=2)
    inner = center - width
    outer = center + width

    # A final per-wall spline pass removes residual pixel stair-steps on the
    # short inner obstacle boundary, where paired centerline resampling alone
    # can still leave nearly duplicated samples.
    inner = smooth_closed_curve_spline(inner, smoothing=spline_smoothing, target_count=target_count)
    outer = smooth_closed_curve_spline(outer, smoothing=spline_smoothing, target_count=target_count)
    inner, outer = sanitize_boundary_pairs(inner, outer, passes=2)

    stats = {
        "inner_min_segment_before_m": float(before_inner_min),
        "outer_min_segment_before_m": float(before_outer_min),
        "inner_min_segment_after_m": float(min_closed_segment_length(inner)),
        "outer_min_segment_after_m": float(min_closed_segment_length(outer)),
    }
    return inner, outer, stats


def estimate_boundary_sides(
    centerline_rc: np.ndarray,
    inner_rc: np.ndarray,
    outer_rc: np.ndarray,
) -> tuple[int, int]:
    """估算内外边界相对于中线前进方向的左右侧。

    Args:
        centerline_rc: 中线像素坐标。
        inner_rc: 内边界像素坐标云。
        outer_rc: 外边界像素坐标云。

    Returns:
        (inner_sign, outer_sign) 元组，1 代表左侧，-1 代表右侧。
    """
    inner_tree = cKDTree(inner_rc[:, ::-1])
    outer_tree = cKDTree(outer_rc[:, ::-1])

    inner_signs = []
    outer_signs = []
    for i, point in enumerate(centerline_rc):
        prev_pt = centerline_rc[i - 1]
        next_pt = centerline_rc[(i + 1) % len(centerline_rc)]
        tangent = next_pt - prev_pt
        norm = np.linalg.norm(tangent)
        if norm <= 1e-9:
            continue
        tangent = tangent / norm
        left_normal = np.array([-tangent[1], tangent[0]], dtype=float)

        _, inner_idx = inner_tree.query(point[::-1])
        _, outer_idx = outer_tree.query(point[::-1])
        inner_pt = inner_rc[int(inner_idx)]
        outer_pt = outer_rc[int(outer_idx)]

        inner_side = np.dot(inner_pt - point, left_normal)
        outer_side = np.dot(outer_pt - point, left_normal)
        inner_signs.append(1 if inner_side >= 0 else -1)
        outer_signs.append(1 if outer_side >= 0 else -1)

    inner_sign = 1 if np.sum(inner_signs) >= 0 else -1
    outer_sign = 1 if np.sum(outer_signs) >= 0 else -1
    if inner_sign == outer_sign:
        outer_sign *= -1
    return inner_sign, outer_sign


def sample_binary_mask(mask_float: np.ndarray, point_rc: np.ndarray) -> float:
    """在非整数像素坐标处采样连续二值掩码（使用双线性插值）。

    Args:
        mask_float: 浮点型掩码数组。
        point_rc: 要采样的像素坐标 [行, 列]。

    Returns:
        插值后的掩码值。
    """
    coords = np.array([[point_rc[0]], [point_rc[1]]], dtype=float)
    return float(ndimage.map_coordinates(mask_float, coords, order=1, mode="nearest")[0])


def march_mask_boundary(
    point_rc: np.ndarray,
    direction_rc: np.ndarray,
    mask_float: np.ndarray,
    target_inside: bool,
    max_dist_px: float,
    step_px: float = 0.25,
) -> np.ndarray | None:
    """沿指定方向步进，直到穿过二值掩码的边界。

    Args:
        point_rc: 起点像素坐标。
        direction_rc: 搜索方向向量。
        mask_float: 浮点型掩码。
        target_inside: 目标是在掩码内部 (True) 还是外部 (False)。
        max_dist_px: 最大搜索距离。
        step_px: 步进间隔。

    Returns:
        边界交点像素坐标，如果未找到则返回 None。
    """
    norm = float(np.linalg.norm(direction_rc))
    if norm <= 1e-9:
        return None
    direction = direction_rc / norm
    threshold = 0.5
    prev_t = 0.0
    prev_val = sample_binary_mask(mask_float, point_rc)

    for t in np.arange(step_px, max_dist_px + step_px, step_px):
        sample_point = point_rc + direction * t
        value = sample_binary_mask(mask_float, sample_point)
        crossed = (prev_val < threshold <= value) if target_inside else (prev_val > threshold >= value)
        if crossed:
            if abs(value - prev_val) <= 1e-8:
                alpha = 1.0
            else:
                alpha = (threshold - prev_val) / (value - prev_val)
            hit_t = prev_t + alpha * (t - prev_t)
            return point_rc + direction * hit_t
        prev_t = t
        prev_val = value
    return None


def search_boundary_hit(
    point_rc: np.ndarray,
    normal_rc: np.ndarray,
    preferred_sign: int,
    mask_float: np.ndarray,
    target_inside: bool,
    base_max_dist_px: float,
) -> np.ndarray | None:
    """Search a mask boundary along the normal using progressively wider ranges."""
    for scale in (1.0, 1.5, 2.0, 3.0):
        max_dist_px = base_max_dist_px * scale
        for sign in (preferred_sign, -preferred_sign):
            hit = march_mask_boundary(
                point_rc=point_rc,
                direction_rc=normal_rc * sign,
                mask_float=mask_float,
                target_inside=target_inside,
                max_dist_px=max_dist_px,
            )
            if hit is not None:
                return hit
    return None


def nearest_boundary_point_on_side(
    point_rc: np.ndarray,
    normal_rc: np.ndarray,
    boundary_rc_cloud: np.ndarray,
    preferred_sign: int,
) -> np.ndarray | None:
    """Fall back to the nearest boundary point that lies on the expected side."""
    if len(boundary_rc_cloud) == 0:
        return None

    offsets = boundary_rc_cloud - point_rc
    side_scores = offsets @ normal_rc
    candidates = boundary_rc_cloud[(side_scores * preferred_sign) > 0.0]
    if len(candidates) == 0:
        candidates = boundary_rc_cloud

    distances = np.linalg.norm(candidates - point_rc, axis=1)
    if len(distances) == 0:
        return None
    return candidates[int(np.argmin(distances))]


def interpolate_closed_curve_points(points: np.ndarray, invalid_mask: np.ndarray) -> np.ndarray:
    """Repair sparse invalid points on a closed curve by circular interpolation."""
    if len(points) == 0 or not invalid_mask.any():
        return points

    repaired = points.copy()
    n = len(points)
    valid_count = int((~invalid_mask).sum())
    if valid_count < 2:
        return repaired

    for idx in np.where(invalid_mask)[0]:
        prev_idx = (idx - 1) % n
        while invalid_mask[prev_idx] and prev_idx != idx:
            prev_idx = (prev_idx - 1) % n

        next_idx = (idx + 1) % n
        while invalid_mask[next_idx] and next_idx != idx:
            next_idx = (next_idx + 1) % n

        if prev_idx == idx and next_idx == idx:
            continue
        if prev_idx == idx:
            repaired[idx] = repaired[next_idx]
            continue
        if next_idx == idx:
            repaired[idx] = repaired[prev_idx]
            continue
        if prev_idx == next_idx:
            repaired[idx] = repaired[prev_idx]
            continue

        dist_prev = (idx - prev_idx) % n
        dist_next = (next_idx - idx) % n
        total = max(dist_prev + dist_next, 1)
        weight_prev = dist_next / total
        weight_next = dist_prev / total
        repaired[idx] = weight_prev * repaired[prev_idx] + weight_next * repaired[next_idx]

    return repaired


def intersect_boundaries_along_normals(
    centerline_rc: np.ndarray,
    inner_fill_mask: np.ndarray,
    outer_fill_mask: np.ndarray,
    inner_boundary_mask: np.ndarray,
    outer_boundary_mask: np.ndarray,
    width_guess_px: float,
) -> tuple[np.ndarray, np.ndarray]:
    """从中线点出发，沿法线方向寻找内外边界的精确交点。

    Args:
        centerline_rc: 中线点数组。
        inner_fill_mask: 内部填充掩码。
        outer_fill_mask: 外部填充掩码。
        inner_boundary_mask: 内部边界线掩码。
        outer_boundary_mask: 外部边界线掩码。
        width_guess_px: 预估赛道半宽（像素）。

    Returns:
        (inner_points, outer_points) 的像素坐标元组。
    """
    inner_rc_cloud = np.argwhere(inner_boundary_mask).astype(float)
    outer_rc_cloud = np.argwhere(outer_boundary_mask).astype(float)
    inner_sign, outer_sign = estimate_boundary_sides(centerline_rc, inner_rc_cloud, outer_rc_cloud)

    inner_points = np.zeros_like(centerline_rc, dtype=float)
    outer_points = np.zeros_like(centerline_rc, dtype=float)
    max_search_px = max(8.0, 1.8 * width_guess_px)
    inner_float = inner_fill_mask.astype(float)
    outer_float = outer_fill_mask.astype(float)
    fallback_count = 0
    inner_invalid = np.zeros(len(centerline_rc), dtype=bool)
    outer_invalid = np.zeros(len(centerline_rc), dtype=bool)

    for i, point in enumerate(centerline_rc):
        prev_pt = centerline_rc[i - 1]
        next_pt = centerline_rc[(i + 1) % len(centerline_rc)]
        tangent = next_pt - prev_pt
        tangent_norm = np.linalg.norm(tangent)
        if tangent_norm <= 1e-9:
            tangent = np.array([0.0, 1.0], dtype=float)
        else:
            tangent = tangent / tangent_norm

        normal = np.array([-tangent[1], tangent[0]], dtype=float)

        inner_hit = search_boundary_hit(
            point_rc=point,
            normal_rc=normal,
            preferred_sign=inner_sign,
            mask_float=inner_float,
            target_inside=True,
            base_max_dist_px=max_search_px,
        )
        outer_hit = search_boundary_hit(
            point_rc=point,
            normal_rc=normal,
            preferred_sign=outer_sign,
            mask_float=outer_float,
            target_inside=False,
            base_max_dist_px=max_search_px,
        )

        if inner_hit is None:
            inner_hit = nearest_boundary_point_on_side(
                point_rc=point,
                normal_rc=normal,
                boundary_rc_cloud=inner_rc_cloud,
                preferred_sign=inner_sign,
            )
            if inner_hit is not None:
                fallback_count += 1
                inner_invalid[i] = True
        if outer_hit is None:
            outer_hit = nearest_boundary_point_on_side(
                point_rc=point,
                normal_rc=normal,
                boundary_rc_cloud=outer_rc_cloud,
                preferred_sign=outer_sign,
            )
            if outer_hit is not None:
                fallback_count += 1
                outer_invalid[i] = True

        if inner_hit is None or outer_hit is None:
            raise RuntimeError(f"无法在采样点索引 {i} 处与双向边界相交。")

        inner_points[i] = inner_hit
        outer_points[i] = outer_hit

    if inner_invalid.any():
        inner_points = interpolate_closed_curve_points(inner_points, inner_invalid)
    if outer_invalid.any():
        outer_points = interpolate_closed_curve_points(outer_points, outer_invalid)

    # 平滑最终生成的边界点
    inner_points = clean_map.circular_savgol(inner_points, window=15, polyorder=2)
    outer_points = clean_map.circular_savgol(outer_points, window=15, polyorder=2)
    if fallback_count:
        print(f"[export_csv] Warning: used nearest-boundary fallback {fallback_count} time(s).")
    return inner_points, outer_points


def build_centerline_points(
    track_mask: np.ndarray,
    inner_fill: np.ndarray,
    outer_fill: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """通过三角剖分提取原始有序中线点及其宽度样本。

    Args:
        track_mask: 赛道区域掩码。
        inner_fill: 内部填充掩码。
        outer_fill: 外部填充掩码。

    Returns:
        (有序点数组, 宽度样本数组) 的元组。
    """
    inner_boundary = clean_map.boundary_from_fill(inner_fill)
    outer_boundary = clean_map.boundary_from_fill(outer_fill)
    if inner_boundary is None or outer_boundary is None:
        return np.empty((0, 2), dtype=float), np.empty((0,), dtype=float)

    inner_points = clean_map.sample_boundary_points(inner_boundary, step=3)
    outer_points = clean_map.sample_boundary_points(outer_boundary, step=3)
    if len(inner_points) < 8 or len(outer_points) < 8:
        return np.empty((0, 2), dtype=float), np.empty((0,), dtype=float)

    all_points = np.vstack([inner_points, outer_points])
    labels = np.concatenate([
        np.zeros(len(inner_points), dtype=np.int32),
        np.ones(len(outer_points), dtype=np.int32),
    ])

    tri = clean_map.Delaunay(all_points[:, ::-1])
    width_guess = float(clean_map.ndimage.distance_transform_edt(track_mask).max() * 2.0)
    area_threshold = max(12.0, 0.20 * width_guess * width_guess)
    isosceles_tol = 0.35
    pointedness_ratio = 1.4

    circumcenters: list[np.ndarray] = []
    width_samples: list[float] = []

    for simplex in tri.simplices:
        tri_points = all_points[simplex]
        tri_labels = labels[simplex]
        if len(np.unique(tri_labels)) < 2:
            continue

        p0, p1, p2 = tri_points
        lengths = np.array([
            np.linalg.norm(p1 - p0),
            np.linalg.norm(p2 - p1),
            np.linalg.norm(p0 - p2),
        ])
        lengths.sort()
        short, mid, long = lengths
        area = clean_map.triangle_area(p0, p1, p2)

        is_isosceles_like = (long - mid) / max(long, 1e-6) <= isosceles_tol
        is_pointed = long / max(short, 1e-6) >= pointedness_ratio
        is_large = area >= area_threshold
        if not ((is_isosceles_like and is_pointed) or is_large):
            continue

        center = clean_map.circumcenter(p0, p1, p2)
        if center is None:
            continue
        cy, cx = center
        if cy < 0 or cy >= track_mask.shape[0] or cx < 0 or cx >= track_mask.shape[1]:
            continue
        if not track_mask[int(round(cy)), int(round(cx))]:
            continue

        cross_widths = []
        for i in range(3):
            for j in range(i + 1, 3):
                if tri_labels[i] != tri_labels[j]:
                    cross_widths.append(float(np.linalg.norm(tri_points[i] - tri_points[j])))
        if not cross_widths:
            continue

        circumcenters.append(center)
        width_samples.append(float(np.mean(cross_widths)))

    if len(circumcenters) < 8:
        return np.empty((0, 2), dtype=float), np.empty((0,), dtype=float)

    circum_points = np.array(circumcenters, dtype=float)
    widths = np.array(width_samples, dtype=float)

    reference_center = np.argwhere(inner_fill).mean(axis=0)
    angles = np.arctan2(circum_points[:, 0] - reference_center[0], circum_points[:, 1] - reference_center[1])
    order = np.argsort(angles)
    ordered = circum_points[order]
    ordered_widths = widths[order]

    deduped = [ordered[0]]
    deduped_widths = [ordered_widths[0]]
    for point, width in zip(ordered[1:], ordered_widths[1:]):
        if np.linalg.norm(point - deduped[-1]) >= 1.0:
            deduped.append(point)
            deduped_widths.append(width)

    ordered = np.array(deduped, dtype=float)
    ordered_widths = np.array(deduped_widths, dtype=float)
    ordered = clean_map.smooth_centerline_points(ordered)
    ordered_widths = np.interp(
        np.linspace(0, len(ordered_widths), len(ordered), endpoint=False),
        np.arange(len(ordered_widths)),
        ordered_widths,
        period=len(ordered_widths),
    )
    return ordered, ordered_widths


def compute_lapdist(points_xy: np.ndarray) -> np.ndarray:
    """计算沿曲线点集的累计弧长（单圈距离）。

    Args:
        points_xy: 世界坐标点集 [x, y]。

    Returns:
        累计距离数组。
    """
    if len(points_xy) == 0:
        return np.empty((0,), dtype=float)
    diffs = np.diff(points_xy, axis=0)
    seg = np.linalg.norm(diffs, axis=1)
    lapdist = np.concatenate([[0.0], np.cumsum(seg)])
    return lapdist


def calculate_inward_normals(bound_pts: np.ndarray, v: np.ndarray) -> np.ndarray:
    """计算指向赛道内部的单位法向量。

    Args:
        bound_pts: 边界点 [x, y]。
        v: 横跨向量（内边界指向外边界）。

    Returns:
        向内单位法向量。
    """
    dp = (np.roll(bound_pts, -1, axis=0) - np.roll(bound_pts, 1, axis=0)) / 2.0
    dp_norm = np.linalg.norm(dp, axis=1, keepdims=True)
    tangent = dp / (dp_norm + 1e-8)
    n1 = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    n2 = np.column_stack([tangent[:, 1], -tangent[:, 0]])
    dot1 = np.sum(n1 * v, axis=1)
    normals = np.where(dot1[:, np.newaxis] > 0, n1, n2)
    return normals


def sanitize_boundary_pairs(inner_xy: np.ndarray, outer_xy: np.ndarray, passes: int = 3) -> tuple[np.ndarray, np.ndarray]:
    """通过法向检查清理和修复异常的内外边界对。

    Args:
        inner_xy: 内部边界点。
        outer_xy: 外部边界点。
        passes: 清理迭代次数。

    Returns:
        清理后的 (inner_xy, outer_xy) 元组。
    """
    inner = inner_xy.copy()
    outer = outer_xy.copy()
    for _ in range(passes):
        v = outer - inner
        n_inner = calculate_inward_normals(inner, v)
        n_outer = calculate_inward_normals(outer, v)
        d_inner = np.sum(n_inner * v, axis=1)
        d_outer = np.sum(n_outer * v, axis=1)
        threshold_inner = max(0.12, 0.25 * float(np.median(d_inner)))
        threshold_outer = max(0.12, 0.25 * float(np.median(d_outer)))
        bad = np.where((d_inner < threshold_inner) | (d_outer < threshold_outer))[0]
        if len(bad) == 0:
            break
        for idx in bad:
            # 使用相邻点进行平滑平衡
            inner[idx] = (
                0.15 * inner[idx - 2]
                + 0.35 * inner[idx - 1]
                + 0.35 * inner[(idx + 1) % len(inner)]
                + 0.15 * inner[(idx + 2) % len(inner)]
            )
            outer[idx] = (
                0.15 * outer[idx - 2]
                + 0.35 * outer[idx - 1]
                + 0.35 * outer[(idx + 1) % len(outer)]
                + 0.15 * outer[(idx + 2) % len(outer)]
            )
    return inner, outer


def derive_masks_from_cleaned_map(cleaned_image: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Recover track, inner-fill, and outer-fill masks from a cleaned track map."""
    free_mask = cleaned_image >= 250
    track_mask = clean_map.largest_component(free_mask)
    if not track_mask.any():
        raise RuntimeError("清洗后的地图中没有可用的赛道自由区域。")

    outer_fill = ndimage.binary_fill_holes(track_mask)
    inner_fill = outer_fill & ~track_mask
    if not inner_fill.any():
        raise RuntimeError("无法从清洗后的地图中恢复内边界区域。")
    return track_mask, inner_fill, outer_fill


def export_track_csv_from_cleaned_map(
    cleaned_image: np.ndarray,
    resolution: float,
    origin: list[float],
    output_csv: Path,
    output_map_prefix: Path | None = None,
    map_yaml_template: dict[str, object] | None = None,
    samples: int = 400,
    boundary_min_spacing_m: float = 0.03,
    boundary_savgol_window: int = 31,
    boundary_spline_smoothing: float = 0.10,
    track_mask: np.ndarray | None = None,
    inner_fill: np.ndarray | None = None,
    outer_fill: np.ndarray | None = None,
) -> dict[str, object]:
    """Export aligned track boundaries and centerline from a cleaned map image."""
    if track_mask is None or inner_fill is None or outer_fill is None:
        track_mask, inner_fill, outer_fill = derive_masks_from_cleaned_map(cleaned_image)
    else:
        track_mask = track_mask.astype(bool)
        inner_fill = inner_fill.astype(bool)
        outer_fill = outer_fill.astype(bool)
        if not track_mask.any():
            raise RuntimeError("清洗后的地图中没有可用的赛道自由区域。")
        if not inner_fill.any() or not outer_fill.any():
            raise RuntimeError("无法从清洗后的地图中恢复内外边界区域。")

    centerline_rc, width_samples = build_centerline_points(track_mask, inner_fill, outer_fill)
    if len(centerline_rc) < 8:
        raise RuntimeError("无法从清洗后的地图重构有序中线。")

    inner_boundary = clean_map.boundary_from_fill(inner_fill)
    outer_boundary = clean_map.boundary_from_fill(outer_fill)
    centerline_rc = resample_closed_curve(centerline_rc, target_count=samples)
    width_guess_px = float(width_samples.mean()) if len(width_samples) else 10.0
    inner_rc, outer_rc = intersect_boundaries_along_normals(
        centerline_rc=centerline_rc,
        inner_fill_mask=inner_fill,
        outer_fill_mask=outer_fill,
        inner_boundary_mask=inner_boundary,
        outer_boundary_mask=outer_boundary,
        width_guess_px=width_guess_px,
    )

    inner_xy = pixel_to_world(inner_rc, resolution, origin, cleaned_image.shape[0])
    outer_xy = pixel_to_world(outer_rc, resolution, origin, cleaned_image.shape[0])

    inner_xy, outer_xy, postprocess_stats = postprocess_boundary_pairs(
        inner_xy,
        outer_xy,
        target_count=samples,
        min_spacing_m=boundary_min_spacing_m,
        savgol_window=boundary_savgol_window,
        spline_smoothing=boundary_spline_smoothing,
    )

    centerline_xy = 0.5 * (inner_xy + outer_xy)
    lapdist = compute_lapdist(centerline_xy)

    output_csv = output_csv.resolve()
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "left_border_x",
                "left_border_y",
                "right_border_x",
                "right_border_y",
                "pos_x",
                "pos_y",
                "Lapdist",
            ]
        )
        for inner_pt, outer_pt, pos, dist in zip(inner_xy, outer_xy, centerline_xy, lapdist):
            writer.writerow(
                [
                    f"{inner_pt[0]:.7f}",
                    f"{inner_pt[1]:.7f}",
                    f"{outer_pt[0]:.7f}",
                    f"{outer_pt[1]:.7f}",
                    f"{pos[0]:.7f}",
                    f"{pos[1]:.7f}",
                    f"{dist:.7f}",
                ]
            )

    output_map_image: Path | None = None
    output_map_yaml: Path | None = None
    if output_map_prefix is not None:
        output_map_prefix = output_map_prefix.resolve()
        output_map_prefix.parent.mkdir(parents=True, exist_ok=True)
        output_map_image = output_map_prefix.with_suffix(".pgm")
        output_map_yaml = output_map_prefix.with_suffix(".yaml")
        Image.fromarray(cleaned_image, mode="L").save(output_map_image)

        yaml_out = dict(map_yaml_template or {})
        yaml_out.setdefault("mode", "trinary")
        yaml_out.setdefault("negate", 0)
        yaml_out.setdefault("occupied_thresh", 0.65)
        yaml_out.setdefault("free_thresh", 0.25)
        yaml_out["resolution"] = float(resolution)
        yaml_out["origin"] = list(origin)
        yaml_out["image"] = output_map_image.name
        clean_map.dump_simple_yaml(output_map_yaml, yaml_out)

    return {
        "output_csv": str(output_csv),
        "output_map_image": str(output_map_image) if output_map_image is not None else None,
        "output_map_yaml": str(output_map_yaml) if output_map_yaml is not None else None,
        "samples": int(len(centerline_xy)),
        "lap_length_m": float(lapdist[-1]) if len(lapdist) else 0.0,
        **postprocess_stats,
    }


def main() -> None:
    """运行 CSV 导出工具的入口函数。"""
    parser = argparse.ArgumentParser(description="将处理后的赛道边界和中线导出为 CSV。")
    parser.add_argument("--yaml", type=Path, default=DEFAULT_INPUT_YAML, help="输入的 ROS 地图 YAML 文件。")
    parser.add_argument("--output-csv", type=Path, default=DEFAULT_OUTPUT_CSV, help="输出 CSV 路径。")
    parser.add_argument(
        "--output-map-prefix",
        type=Path,
        default=DEFAULT_OUTPUT_MAP_PREFIX,
        help="模拟器使用的处理后地图前缀 (.pgm + .yaml)。",
    )
    parser.add_argument("--samples", type=int, default=400, help="沿中线等距离采样的数量。")
    parser.add_argument("--close-radius-m", type=float, default=0.20, help="与 clean_map.py 参数一致。")
    parser.add_argument("--coarse-smooth-m", type=float, default=0.18, help="与 clean_map.py 参数一致。")
    parser.add_argument("--boundary-offset-m", type=float, default=0.12, help="与 clean_map.py 参数一致。")
    parser.add_argument("--fine-smooth-m", type=float, default=0.06, help="与 clean_map.py 参数一致。")
    parser.add_argument(
        "--boundary-min-spacing-m",
        type=float,
        default=0.03,
        help="导出 CSV 前删除近重复中线点的最小间距。",
    )
    parser.add_argument(
        "--boundary-savgol-window",
        type=int,
        default=31,
        help="导出 CSV 前内外边界/宽度向量的周期 Savitzky-Golay 平滑窗口。",
    )
    parser.add_argument(
        "--boundary-spline-smoothing",
        type=float,
        default=0.10,
        help="导出 CSV 前中心线周期 B-spline 平滑强度；0 表示只等弧长重采样。",
    )
    args = parser.parse_args()

    yaml_path = args.yaml.resolve()
    yaml_data = clean_map.parse_simple_yaml(yaml_path)
    resolution = float(yaml_data["resolution"])
    origin = list(yaml_data["origin"])
    image_path = (yaml_path.parent / str(yaml_data["image"])).resolve()
    image = np.array(Image.open(image_path), dtype=np.uint8)

    # 1. 运行核心地图清理逻辑
    simulator_map, track_mask, _, _, boundary_groups = clean_map.clean_map(
        image=image,
        resolution=resolution,
        close_radius_m=args.close_radius_m,
        coarse_smooth_m=args.coarse_smooth_m,
        boundary_offset_m=args.boundary_offset_m,
        fine_smooth_m=args.fine_smooth_m,
    )
    inner_fill, outer_fill = boundary_groups["processed"]
    if inner_fill is None or outer_fill is None:
        raise RuntimeError("无法提取处理后的内外边界。")

    export_info = export_track_csv_from_cleaned_map(
        cleaned_image=simulator_map,
        resolution=resolution,
        origin=origin,
        output_csv=args.output_csv,
        output_map_prefix=args.output_map_prefix,
        map_yaml_template=yaml_data,
        samples=args.samples,
        boundary_min_spacing_m=args.boundary_min_spacing_m,
        boundary_savgol_window=args.boundary_savgol_window,
        boundary_spline_smoothing=args.boundary_spline_smoothing,
        track_mask=track_mask,
        inner_fill=inner_fill,
        outer_fill=outer_fill,
    )

    print(f"input_yaml={yaml_path}")
    print(f"output_csv={export_info['output_csv']}")
    print(f"output_map_image={export_info['output_map_image']}")
    print(f"output_map_yaml={export_info['output_map_yaml']}")
    print(f"samples={export_info['samples']}")
    print(f"lap_length_m={float(export_info['lap_length_m']):.4f}")
    print(f"inner_min_segment_before_m={float(export_info['inner_min_segment_before_m']):.4f}")
    print(f"inner_min_segment_after_m={float(export_info['inner_min_segment_after_m']):.4f}")
    print(f"outer_min_segment_before_m={float(export_info['outer_min_segment_before_m']):.4f}")
    print(f"outer_min_segment_after_m={float(export_info['outer_min_segment_after_m']):.4f}")


if __name__ == "__main__":
    main()

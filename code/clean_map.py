#!/usr/bin/env python3
"""从 ROS SLAM 占据网格地图中提取并闭合环形赛道。

策略：不进行全局边界闭合，而是首先识别出最像环形赛道的自由空间连通分量
（具有较大的内部空洞，且不过多接触图像边界），然后进行局部修复和闭合。
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage
from scipy import signal
from scipy.spatial import Delaunay

from package_paths import get_default_map_yaml, get_default_output_root

DEFAULT_INPUT_YAML = get_default_map_yaml()
DEFAULT_OUTPUT_PREFIX = get_default_output_root() / "maps" / "cleaned_map"
DEFAULT_PREVIEW_OUTPUT = get_default_output_root() / "figures" / "contour_preview.png"


# ---------------------------------------------------------------------------
# 实用辅助工具
# ---------------------------------------------------------------------------

def parse_simple_yaml(path: Path) -> dict[str, object]:
    """解析简单的 ROS 地图 YAML 文件。

    Args:
        path: YAML 文件的路径。

    Returns:
        包含 YAML 数据的字典。
    """
    data: dict[str, object] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip()
        if value.startswith("[") and value.endswith("]"):
            items = []
            for item in value[1:-1].split(","):
                item = item.strip()
                items.append(float(item) if "." in item else int(item))
            data[key] = items
        elif value.lower() in {"true", "false"}:
            data[key] = value.lower() == "true"
        elif value.replace(".", "", 1).replace("-", "", 1).isdigit():
            data[key] = float(value) if "." in value else int(value)
        else:
            data[key] = value
    return data


def dump_simple_yaml(path: Path, data: dict[str, object]) -> None:
    """将字典导出为简单的 ROS YAML 格式文件。

    Args:
        path: 输出文件的路径。
        data: 要写入的数据字典。
    """
    lines = []
    for key in ["image", "mode", "resolution", "origin", "negate", "occupied_thresh", "free_thresh"]:
        if key not in data:
            continue
        value = data[key]
        if isinstance(value, list):
            rendered = "[" + ", ".join(str(item) for item in value) + "]"
        else:
            rendered = str(value)
        lines.append(f"{key}: {rendered}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def disk(radius: int) -> np.ndarray:
    """生成一个二值圆形掩码（结构元素）。

    Args:
        radius: 圆形半径（像素）。

    Returns:
        代表圆形的布尔型 numpy 数组。
    """
    if radius <= 0:
        return np.ones((1, 1), dtype=bool)
    y, x = np.ogrid[-radius : radius + 1, -radius : radius + 1]
    return (x * x + y * y) <= radius * radius


def to_ros_trinary(occupied: np.ndarray, free: np.ndarray) -> np.ndarray:
    """将占据和自由区域转换为标准 ROS 三进制占据网格图像 (0, 205, 254)。

    Args:
        occupied: 占据区域的布尔掩码。
        free: 自由区域的布尔掩码。

    Returns:
        uint8 类型的图像数组。
    """
    image = np.full(occupied.shape, 205, dtype=np.uint8)
    image[free] = 254
    image[occupied] = 0
    return image


def smooth_mask(mask: np.ndarray, radius: int) -> np.ndarray:
    """平滑二值区域并保持其为填充状态。

    Args:
        mask: 输入的二值掩码。
        radius: 平滑半径（像素）。

    Returns:
        平滑后的掩码。
    """
    if mask is None or radius <= 0:
        return mask
    kernel = disk(radius)
    smoothed = ndimage.binary_closing(mask, structure=kernel)
    smoothed = ndimage.binary_opening(smoothed, structure=kernel)
    return ndimage.binary_fill_holes(smoothed)


def offset_track_by_boundary_distance(
    inner_fill: np.ndarray,
    outer_fill: np.ndarray,
    offset_px: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """通过欧几里得距离从提取的边界缩放赛道走廊。

    该函数使用边界距离阈值而不是直接的形态学操作，使外部边界向内收缩，内部边界向外扩张。

    Args:
        inner_fill: 内部填充掩码。
        outer_fill: 外部填充掩码。
        offset_px: 偏移距离（像素）。

    Returns:
        (新的内填充, 新的外填充, 赛道区域) 的元组。
    """
    corridor = outer_fill & ~inner_fill
    if offset_px <= 0 or not corridor.any():
        return inner_fill.copy(), outer_fill.copy(), corridor

    inner_boundary = boundary_from_fill(inner_fill)
    outer_boundary = boundary_from_fill(outer_fill)
    if inner_boundary is None or outer_boundary is None:
        return inner_fill.copy(), outer_fill.copy(), corridor

    dist_to_inner = ndimage.distance_transform_edt(~inner_boundary)
    dist_to_outer = ndimage.distance_transform_edt(~outer_boundary)

    outer_shrunk = outer_fill & (dist_to_outer >= float(offset_px))
    inner_expanded = inner_fill | (corridor & (dist_to_inner < float(offset_px)))

    # 保持嵌套关系有效，以便下游边界提取。
    outer_shrunk |= inner_expanded
    track_region = outer_shrunk & ~inner_expanded
    track_region = largest_component(track_region)
    outer_shrunk = track_region | inner_expanded
    return inner_expanded, outer_shrunk, track_region


def boundary_from_fill(mask: np.ndarray | None) -> np.ndarray | None:
    """从填充掩码中提取单像素宽度的边界线。

    Args:
        mask: 输入的填充掩码。

    Returns:
        边界线的掩码，如果输入为空则返回 None。
    """
    if mask is None:
        return None
    return mask & ~ndimage.binary_erosion(mask, structure=np.ones((3, 3), dtype=bool))


def largest_component(mask: np.ndarray) -> np.ndarray:
    """保留二值掩码中最大的连通分量。

    Args:
        mask: 输入掩码。

    Returns:
        仅包含最大分量的掩码。
    """
    if not mask.any():
        return mask
    labels, count = ndimage.label(mask, structure=ndimage.generate_binary_structure(2, 2))
    if count <= 1:
        return mask
    sizes = ndimage.sum(mask, labels, index=np.arange(1, count + 1))
    largest_label = int(np.argmax(sizes)) + 1
    return labels == largest_label


def morphological_skeleton(mask: np.ndarray) -> np.ndarray:
    """将二值掩码细化为单像素宽的骨架。

    Args:
        mask: 输入掩码。

    Returns:
        骨架掩码线。
    """
    work = mask.copy()
    skeleton = np.zeros_like(work, dtype=bool)
    structure = ndimage.generate_binary_structure(2, 1)

    while work.any():
        eroded = ndimage.binary_erosion(work, structure=structure)
        opened = ndimage.binary_dilation(eroded, structure=structure)
        skeleton |= work & ~opened
        work = eroded

    return largest_component(skeleton)


def neighbor_count(mask: np.ndarray) -> np.ndarray:
    """计算每个像素周围邻居的数量（3x3 窗口）。

    Args:
        mask: 输入二值掩码。

    Returns:
        邻居数量数组。
    """
    kernel = np.ones((3, 3), dtype=np.int32)
    counts = ndimage.convolve(mask.astype(np.int32), kernel, mode="constant", cval=0)
    return counts - mask.astype(np.int32)


def prune_skeleton(mask: np.ndarray, iterations: int = 40) -> np.ndarray:
    """通过迭代删除端点来移除骨架中的悬挂分支。

    Args:
        mask: 输入骨架掩码。
        iterations: 剪枝迭代次数。

    Returns:
        剪枝后的骨架掩码。
    """
    pruned = mask.copy()
    for _ in range(iterations):
        counts = neighbor_count(pruned)
        endpoints = pruned & (counts <= 1)
        if not endpoints.any():
            break
        pruned &= ~endpoints
    return largest_component(pruned)


def rasterize_polyline(points: np.ndarray, shape: tuple[int, int], closed: bool = True) -> np.ndarray:
    """将折线点集栅格化为二值图像。

    Args:
        points: 点坐标数组 (N, 2)。
        shape: 输出图像的形状 (H, W)。
        closed: 是否闭合折线。

    Returns:
        栅格化后的布尔掩码。
    """
    mask = np.zeros(shape, dtype=bool)
    if len(points) < 2:
        return mask
    limit = len(points) if closed else len(points) - 1
    for i in range(limit):
        p0 = points[i]
        p1 = points[(i + 1) % len(points)]
        steps = int(max(abs(p1[0] - p0[0]), abs(p1[1] - p0[1]))) + 1
        ys = np.rint(np.linspace(p0[0], p1[0], steps)).astype(int)
        xs = np.rint(np.linspace(p0[1], p1[1], steps)).astype(int)
        ys = np.clip(ys, 0, shape[0] - 1)
        xs = np.clip(xs, 0, shape[1] - 1)
        mask[ys, xs] = True
    return mask


def sample_boundary_points(mask: np.ndarray, step: int = 3) -> np.ndarray:
    """从边界掩码中按角度顺序采样点。

    Args:
        mask: 边界线掩码。
        step: 采样步长。

    Returns:
        有序的点坐标数组。
    """
    points = np.argwhere(mask)
    if len(points) == 0:
        return np.empty((0, 2), dtype=float)
    centroid = points.mean(axis=0)
    angles = np.arctan2(points[:, 0] - centroid[0], points[:, 1] - centroid[1])
    order = np.argsort(angles)
    ordered = points[order]
    sampled = ordered[:: max(1, step)]
    return sampled.astype(float)


def triangle_area(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    """计算三个点定义的三角形面积。

    Args:
        a, b, c: 三角形的三个顶点坐标。

    Returns:
        三角形面积。
    """
    ab = b - a
    ac = c - a
    return float(abs(ab[0] * ac[1] - ab[1] * ac[0]) * 0.5)


def circumcenter(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray | None:
    """计算三角形的外心。

    Args:
        a, b, c: 三个顶点的坐标 [y, x]。

    Returns:
        外心坐标 [y, x]，如果三点共线则返回 None。
    """
    ax, ay = a[1], a[0]
    bx, by = b[1], b[0]
    cx, cy = c[1], c[0]
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-6:
        return None
    ux = (
        (ax * ax + ay * ay) * (by - cy)
        + (bx * bx + by * by) * (cy - ay)
        + (cx * cx + cy * cy) * (ay - by)
    ) / d
    uy = (
        (ax * ax + ay * ay) * (cx - bx)
        + (bx * bx + by * by) * (ax - cx)
        + (cx * cx + cy * cy) * (bx - ax)
    ) / d
    return np.array([uy, ux], dtype=float)


def circular_savgol(points: np.ndarray, window: int = 9, polyorder: int = 2) -> np.ndarray:
    """对闭合曲线点集应用 Savitzky-Golay 平滑，并处理循环边界。

    Args:
        points: 原始点集坐标 (N, 2)。
        window: Savgol 窗口长度。
        polyorder: 多项式拟合阶数。

    Returns:
        平滑后的点集。
    """
    if len(points) < 5:
        return points
    window = min(window, len(points) if len(points) % 2 == 1 else len(points) - 1)
    if window < 5:
        return points
    pad = window // 2
    extended = np.vstack([points[-pad:], points, points[:pad]])
    y = signal.savgol_filter(extended[:, 0], window_length=window, polyorder=min(polyorder, window - 2), mode="interp")
    x = signal.savgol_filter(extended[:, 1], window_length=window, polyorder=min(polyorder, window - 2), mode="interp")
    return np.column_stack([y[pad:-pad], x[pad:-pad]])


def resample_closed_curve(points: np.ndarray, target_count: int) -> np.ndarray:
    """将闭合曲线重采样为等距离分布的固定数量点。

    Args:
        points: 输入点集。
        target_count: 目标点数。

    Returns:
        重采样后的点集。
    """
    if len(points) < 3 or target_count < 3:
        return points
    closed = np.vstack([points, points[:1]])
    seg_lengths = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    total = float(seg_lengths.sum())
    if total <= 1e-6:
        return points
    cumulative = np.concatenate([[0.0], np.cumsum(seg_lengths)])
    targets = np.linspace(0.0, total, target_count + 1)[:-1]

    sampled = []
    seg_idx = 0
    for t in targets:
        while seg_idx < len(seg_lengths) - 1 and cumulative[seg_idx + 1] < t:
            seg_idx += 1
        start = closed[seg_idx]
        end = closed[seg_idx + 1]
        seg_len = seg_lengths[seg_idx]
        if seg_len <= 1e-6:
            sampled.append(start)
            continue
        alpha = (t - cumulative[seg_idx]) / seg_len
        sampled.append((1.0 - alpha) * start + alpha * end)
    return np.array(sampled, dtype=float)


def smooth_centerline_points(points: np.ndarray) -> np.ndarray:
    """对中线点进行级联平滑处理。

    Args:
        points: 原始中线点。

    Returns:
        平滑后的点。
    """
    if len(points) < 8:
        return points
    target_count = max(64, len(points))
    smooth = resample_closed_curve(points, target_count=target_count)
    smooth = circular_savgol(smooth, window=15, polyorder=2)
    smooth = circular_savgol(smooth, window=11, polyorder=2)
    return smooth


def build_centerline(
    track_mask: np.ndarray,
    inner_fill: np.ndarray | None,
    outer_fill: np.ndarray | None,
) -> tuple[np.ndarray, dict[str, float]]:
    """使用中轴变换/Delaunay 三角剖分方法构建赛道中线。

    Args:
        track_mask: 赛道区域掩码。
        inner_fill: 内部填充掩码。
        outer_fill: 外部填充掩码。

    Returns:
        (中线掩码, 统计字典) 的元组。
    """
    empty_stats = {
        "average_width_px": 0.0,
        "min_width_px": 0.0,
        "max_width_px": 0.0,
        "centerline_pixels": 0,
        "triangulation_points": 0,
        "retained_triangles": 0,
    }

    if inner_fill is None or outer_fill is None or not track_mask.any():
        empty = np.zeros_like(track_mask, dtype=bool)
        return empty, empty_stats.copy()

    inner_boundary = boundary_from_fill(inner_fill)
    outer_boundary = boundary_from_fill(outer_fill)
    if inner_boundary is None or outer_boundary is None:
        empty = np.zeros_like(track_mask, dtype=bool)
        return empty, empty_stats.copy()
    inner_points = sample_boundary_points(inner_boundary, step=3)
    outer_points = sample_boundary_points(outer_boundary, step=3)
    if len(inner_points) < 8 or len(outer_points) < 8:
        empty = np.zeros_like(track_mask, dtype=bool)
        return empty, empty_stats.copy()

    # 合并内外点集进行 Delaunay 剖分
    all_points = np.vstack([inner_points, outer_points])
    labels = np.concatenate([
        np.zeros(len(inner_points), dtype=np.int32),
        np.ones(len(outer_points), dtype=np.int32),
    ])

    tri = Delaunay(all_points[:, ::-1])

    width_guess = float(ndimage.distance_transform_edt(track_mask).max() * 2.0)
    area_threshold = max(12.0, 0.20 * width_guess * width_guess)
    isosceles_tol = 0.35
    pointedness_ratio = 1.4

    circumcenters: list[np.ndarray] = []
    width_samples: list[float] = []

    for simplex in tri.simplices:
        tri_points = all_points[simplex]
        tri_labels = labels[simplex]

        # 核心逻辑：只保留跨越不同边界类别的三角形。
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
        area = triangle_area(p0, p1, p2)

        # 筛选符合赛道几何特征的三角形（等腰、尖锐或大面积）
        is_isosceles_like = (long - mid) / max(long, 1e-6) <= isosceles_tol
        is_pointed = long / max(short, 1e-6) >= pointedness_ratio
        is_large = area >= area_threshold

        if not ((is_isosceles_like and is_pointed) or is_large):
            continue

        center = circumcenter(p0, p1, p2)
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
        empty = np.zeros_like(track_mask, dtype=bool)
        return empty, empty_stats.copy()

    circum_points = np.array(circumcenters, dtype=float)
    widths = np.array(width_samples, dtype=float)

    # 离线地图适配：将保留的外心按环形顺序排列。
    reference_center = np.argwhere(inner_fill).mean(axis=0)
    angles = np.arctan2(circum_points[:, 0] - reference_center[0], circum_points[:, 1] - reference_center[1])
    order = np.argsort(angles)
    ordered = circum_points[order]
    ordered_widths = widths[order]

    # 去重
    deduped = [ordered[0]]
    deduped_widths = [ordered_widths[0]]
    for point, width in zip(ordered[1:], ordered_widths[1:]):
        if np.linalg.norm(point - deduped[-1]) >= 1.0:
            deduped.append(point)
            deduped_widths.append(width)

    ordered = np.array(deduped, dtype=float)
    ordered_widths = np.array(deduped_widths, dtype=float)
    ordered = smooth_centerline_points(ordered)
    ordered_widths = np.interp(
        np.linspace(0, len(ordered_widths), len(ordered), endpoint=False),
        np.arange(len(ordered_widths)),
        ordered_widths,
        period=len(ordered_widths),
    )

    # 栅格化并优化骨架
    centerline = rasterize_polyline(ordered, shape=track_mask.shape, closed=True)
    centerline &= track_mask
    centerline = ndimage.binary_dilation(centerline, structure=np.ones((3, 3), dtype=bool))
    centerline = ndimage.binary_closing(centerline, structure=disk(1))
    centerline = ndimage.binary_opening(centerline, structure=np.ones((2, 2), dtype=bool))
    centerline &= track_mask
    centerline = morphological_skeleton(centerline)
    centerline &= track_mask
    centerline = largest_component(centerline)
    centerline = prune_skeleton(centerline, iterations=6)

    center_stats = {
        "average_width_px": float(ordered_widths.mean()),
        "min_width_px": float(ordered_widths.min()),
        "max_width_px": float(ordered_widths.max()),
        "centerline_pixels": int(centerline.sum()),
        "triangulation_points": int(len(all_points)),
        "retained_triangles": int(len(circumcenters)),
    }
    return centerline, center_stats


# ---------------------------------------------------------------------------
# 第 4 步：在选定赛道上进行局部修复
# ---------------------------------------------------------------------------

def get_closed_regions(occupied: np.ndarray, close_radius: int = 2) -> list[np.ndarray]:
    """寻找被占据连通分量围成的所有填充“包络面”。

    Args:
        occupied: 占据区域掩码。
        close_radius: 闭合墙壁缝隙的半径。

    Returns:
        掩码列表，每个掩码代表一个环路的内部+边界。
    """
    if close_radius > 0:
        dilated = ndimage.binary_dilation(occupied, structure=disk(close_radius))
    else:
        dilated = occupied

    structure = ndimage.generate_binary_structure(2, 2)
    labels, count = ndimage.label(dilated, structure=structure)
    
    envelopes = []
    for i in range(1, count + 1):
        comp = labels == i
        # 寻找该特定分量的包络面
        filled = ndimage.binary_fill_holes(comp)
        
        # 只有在确实围住了某些东西（面积增加）时才保留
        if filled.sum() > comp.sum() + 10:
            envelopes.append(filled)
            
    return envelopes


def touches_image_border(mask: np.ndarray) -> bool:
    """判断掩码是否接触图像边界。"""
    return bool(
        mask[0, :].any()
        or mask[-1, :].any()
        or mask[:, 0].any()
        or mask[:, -1].any()
    )


def containment_fraction(inner: np.ndarray, outer: np.ndarray) -> float:
    """计算 inner 有多少比例被 outer 包含。"""
    inner_area = float(inner.sum())
    if inner_area <= 0.0:
        return 0.0
    return float((inner & outer).sum()) / inner_area


def select_track_envelopes(
    envelopes: list[np.ndarray],
    center: np.ndarray,
    min_hole_fraction: float,
) -> tuple[np.ndarray, np.ndarray, list[dict[str, object]]]:
    """从所有闭合包络面中选择最合理的一对内外边界。

    选择策略：
    1. 先优先挑选“不接触图像边界、面积更大、更靠近图像中心”的外边界候选。
    2. 再只在该外边界内部寻找内边界，而不是全图直接拿最小闭环。
    3. 内边界选择“被外边界高度包含、面积占比合理、面积最大的那个”。
    """
    scored: list[dict[str, object]] = []
    for mask in envelopes:
        area = int(mask.sum())
        coords = np.argwhere(mask)
        centroid = coords.mean(axis=0)
        dist_to_center = float(np.linalg.norm(centroid - center))
        scored.append(
            {
                "mask": mask,
                "area": area,
                "centroid": centroid,
                "dist": dist_to_center,
                "touches_border": touches_image_border(mask),
            }
        )

    outer_candidates = sorted(
        scored,
        key=lambda item: (item["touches_border"], -item["area"], item["dist"]),
    )

    for outer in outer_candidates:
        children: list[dict[str, object]] = []
        outer_mask = outer["mask"]
        outer_area = max(int(outer["area"]), 1)

        for inner in scored:
            if inner is outer or int(inner["area"]) >= outer_area:
                continue

            contain = containment_fraction(inner["mask"], outer_mask)
            area_ratio = float(inner["area"]) / float(outer_area)
            if contain < 0.98 or area_ratio < min_hole_fraction:
                continue

            children.append(
                {
                    **inner,
                    "containment": contain,
                    "area_ratio": area_ratio,
                    "centroid_gap": float(
                        np.linalg.norm(inner["centroid"] - outer["centroid"])
                    ),
                }
            )

        if not children:
            continue

        children.sort(key=lambda item: (-item["area"], item["centroid_gap"]))
        return children[0]["mask"], outer_mask, scored

    # 回退：如果没有找到严格嵌套的一对，就尽量选最像赛道的一组。
    fallback = sorted(scored, key=lambda item: (item["touches_border"], -item["area"], item["dist"]))
    outer_fill = fallback[0]["mask"]
    remaining = [item for item in fallback[1:] if not item["touches_border"]]
    inner_fill = (remaining[0] if remaining else fallback[-1])["mask"]
    return inner_fill, outer_fill, scored


def infer_inner_obstacle_fill(
    occupied: np.ndarray,
    outer_fill: np.ndarray,
    close_radius_px: int,
    min_area_px: int = 12,
) -> np.ndarray | None:
    """Infer an inner obstacle island from occupied pixels inside the outer loop.

    Some hand-cleaned or weak SLAM maps contain a thin or slightly open inner
    obstacle instead of a clean closed contour. The normal envelope detector
    cannot treat that as an inner boundary, so this fallback turns the largest
    interior occupied component into a conservative filled island.
    """
    if outer_fill is None or not outer_fill.any():
        return None

    labels, count = ndimage.label(occupied, structure=ndimage.generate_binary_structure(2, 2))
    if count <= 0:
        return None

    outer_area = max(int(outer_fill.sum()), 1)
    candidates: list[tuple[int, np.ndarray]] = []
    for label_idx in range(1, count + 1):
        comp = labels == label_idx
        area = int(comp.sum())
        if area < min_area_px:
            continue

        # Ignore the outer wall itself: filling it recreates the outer envelope.
        filled_comp = ndimage.binary_fill_holes(comp)
        filled_area = int(filled_comp.sum())
        if filled_area > 0.55 * outer_area:
            continue

        contain = containment_fraction(comp, outer_fill)
        if contain < 0.98:
            continue

        candidates.append((area, comp))

    if not candidates:
        return None

    candidates.sort(key=lambda item: item[0], reverse=True)
    inner = candidates[0][1]
    repair_radius = max(1, close_radius_px)
    inner = ndimage.binary_dilation(inner, structure=disk(repair_radius))
    inner = ndimage.binary_closing(inner, structure=disk(repair_radius))
    inner = ndimage.binary_fill_holes(inner)
    inner &= outer_fill
    if int(inner.sum()) < min_area_px:
        return None
    return inner


def clean_map(
    image: np.ndarray,
    resolution: float,
    close_radius_m: float = 0.20,
    coarse_smooth_m: float = 0.18,
    boundary_offset_m: float = 0.12,
    fine_smooth_m: float = 0.06,
    min_hole_fraction: float = 0.05,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict, dict[str, object]]:
    """通过内外边界从 ROS 占据网格中提取环形赛道。

    Args:
        image: 输入图像。
        resolution: 地图分辨率。
        close_radius_m: 墙壁缝隙闭合半径。
        coarse_smooth_m: 边界粗平滑半径。
        boundary_offset_m: 赛道向内收缩/扩张的偏移。
        fine_smooth_m: 偏移后的精细平滑半径。
        min_hole_fraction: 最小空洞比例（保留用于评分）。

    Returns:
        包含 (模拟器地图, 规划赛道掩码, 中线掩码, 统计, 边界组) 的元组。
    """
    occupied = image == 0
    free = image == 254
    h, w = image.shape
    center = np.array([h // 2, w // 2])

    close_radius_px = max(1, int(round(close_radius_m / resolution)))
    coarse_smooth_px = max(0, int(round(coarse_smooth_m / resolution)))
    boundary_offset_px = max(0, int(round(boundary_offset_m / resolution)))
    fine_smooth_px = max(0, int(round(fine_smooth_m / resolution)))

    envelopes = get_closed_regions(occupied, close_radius=close_radius_px)
    
    if not envelopes:
        empty = np.zeros_like(free, dtype=bool)
        return to_ros_trinary(occupied, free), free, empty, {"error": "no closed loops found"}, {
            "original": (None, None),
            "processed": (None, None),
        }

    inner_fill, outer_fill, scored = select_track_envelopes(
        envelopes=envelopes,
        center=center,
        min_hole_fraction=min_hole_fraction,
    )

    inferred_inner_used = False
    inferred_inner = infer_inner_obstacle_fill(
        occupied=occupied,
        outer_fill=outer_fill,
        close_radius_px=close_radius_px,
    )
    if inferred_inner is not None and (
        inner_fill is outer_fill
        or int(inner_fill.sum()) >= int(0.55 * max(int(outer_fill.sum()), 1))
    ):
        inner_fill = inferred_inner
        inferred_inner_used = True

    # 补偿膨胀移位：将结果腐蚀回去以对齐原始墙面
    if close_radius_px > 0:
        kernel = disk(close_radius_px)
        if not inferred_inner_used:
            inner_fill = ndimage.binary_erosion(inner_fill, structure=kernel)
        outer_fill = ndimage.binary_erosion(outer_fill, structure=kernel)

    original_inner_fill = inner_fill.copy()
    original_outer_fill = outer_fill.copy()

    # 对两个边界进行粗平滑
    inner_fill = smooth_mask(inner_fill, coarse_smooth_px)
    outer_fill = smooth_mask(outer_fill, coarse_smooth_px)

    simulator_inner_fill = inner_fill.copy()
    simulator_outer_fill = outer_fill.copy()
    simulator_track_region = simulator_outer_fill & ~simulator_inner_fill
    simulator_track_mask = simulator_track_region & free
    simulator_map = to_ros_trinary(~simulator_track_mask, simulator_track_mask)

    # 使用距离偏移将赛道向内收紧
    inner_fill, outer_fill, track_region = offset_track_by_boundary_distance(
        inner_fill=inner_fill,
        outer_fill=outer_fill,
        offset_px=boundary_offset_px,
    )

    # 偏移后的精细平滑
    if fine_smooth_px > 0:
        inner_fill = smooth_mask(inner_fill, fine_smooth_px)
        outer_fill = smooth_mask(outer_fill, fine_smooth_px)
        track_region = largest_component(outer_fill & ~inner_fill)

    # 结合原始自由空间限制
    track_mask = track_region & free
    centerline_mask, center_stats = build_centerline(track_mask, inner_fill, outer_fill)

    stats = {
        "track_pixels": int(track_region.sum()),
        "free_pixels": int(track_mask.sum()),
        "simulator_free_pixels": int(simulator_track_mask.sum()),
        "inner_area": int(inner_fill.sum()),
        "outer_area": int(outer_fill.sum()),
        "close_radius_px": close_radius_px,
        "coarse_smooth_px": coarse_smooth_px,
        "boundary_offset_px": boundary_offset_px,
        "fine_smooth_px": fine_smooth_px,
        "average_width_px": round(center_stats["average_width_px"], 3),
        "min_width_px": round(center_stats["min_width_px"], 3),
        "max_width_px": round(center_stats["max_width_px"], 3),
        "average_width_m": round(center_stats["average_width_px"] * resolution, 4),
        "min_width_m": round(center_stats["min_width_px"] * resolution, 4),
        "max_width_m": round(center_stats["max_width_px"] * resolution, 4),
        "centerline_pixels": center_stats["centerline_pixels"],
        "triangulation_points": center_stats.get("triangulation_points", 0),
        "retained_triangles": center_stats.get("retained_triangles", 0),
    }
    return simulator_map, track_mask, centerline_mask, stats, {
        "original": (original_inner_fill, original_outer_fill),
        "simulator": (simulator_inner_fill, simulator_outer_fill),
        "processed": (inner_fill, outer_fill),
        "simulator_track_mask": simulator_track_mask,
    }


# ---------------------------------------------------------------------------
# 命令行接口 (CLI)
# ---------------------------------------------------------------------------

def main() -> None:
    """运行地图清理工具的入口函数。"""
    parser = argparse.ArgumentParser(
        description="从 ROS SLAM 占据网格地图中提取环形赛道。")
    parser.add_argument("--yaml", type=Path, default=DEFAULT_INPUT_YAML,
                        help="输入的 ROS 地图 YAML 文件。")
    parser.add_argument("--output-prefix", type=Path, default=DEFAULT_OUTPUT_PREFIX,
                        help="输出文件的前缀。")
    parser.add_argument("--preview-output", type=Path, default=DEFAULT_PREVIEW_OUTPUT,
                        help="可视化预览图的路径。")
    parser.add_argument(
        "--debug-outputs",
        action="store_true",
        help="额外保存规划掩码、边界、中线和预览图等调试输出。",
    )
    parser.add_argument("--close-radius-m", type=float, default=0.20,
                        help="闭合墙缝的半径（米）。")
    parser.add_argument("--coarse-smooth-m", type=float, default=0.18,
                        help="提取边界的粗平滑半径（米）。")
    parser.add_argument("--boundary-offset-m", type=float, default=0.12,
                        help="赛道向内移动的偏移量：向内收缩外边界并向外扩张内边界（米）。")
    parser.add_argument("--fine-smooth-m", type=float, default=0.06,
                        help="偏移边界后的精细平滑半径（米）。")
    parser.add_argument("--min-hole-fraction", type=float, default=0.05,
                        help="最小空洞/面积比率。")
    args = parser.parse_args()

    yaml_path = args.yaml.resolve()
    yaml_data = parse_simple_yaml(yaml_path)
    resolution = float(yaml_data["resolution"])
    image_path = (yaml_path.parent / str(yaml_data["image"])).resolve()

    image = np.array(Image.open(image_path), dtype=np.uint8)
    simulator_map, track_mask, centerline_mask, stats, boundary_groups = clean_map(
        image=image,
        resolution=resolution,
        close_radius_m=args.close_radius_m,
        coarse_smooth_m=args.coarse_smooth_m,
        boundary_offset_m=args.boundary_offset_m,
        fine_smooth_m=args.fine_smooth_m,
        min_hole_fraction=args.min_hole_fraction,
    )
    original_inner, original_outer = boundary_groups["original"]
    simulator_inner, simulator_outer = boundary_groups["simulator"]
    inner_f, outer_f = boundary_groups["processed"]
    simulator_track_mask = boundary_groups["simulator_track_mask"]

    output_prefix = args.output_prefix.resolve()
    output_prefix.parent.mkdir(parents=True, exist_ok=True)

    # 保存模拟器适用的地图（闭合且平滑，但不收紧）
    output_image = output_prefix.with_suffix(".pgm")
    output_yaml = output_prefix.with_suffix(".yaml")
    Image.fromarray(simulator_map, mode="L").save(output_image)

    yaml_out = dict(yaml_data)
    yaml_out["image"] = output_image.name
    dump_simple_yaml(output_yaml, yaml_out)

    print(f"input_image={image_path}")
    print(f"output_image={output_image}")
    print(f"output_yaml={output_yaml}")
    if args.debug_outputs:
        # 保存规划用的赛道掩码（收紧后的走廊）
        track_path = output_prefix.parent / (output_prefix.stem + "_track.pgm")
        track_img = np.where(track_mask, np.uint8(254), np.uint8(0))
        Image.fromarray(track_img, mode="L").save(track_path)

        simulator_track_path = output_prefix.parent / (output_prefix.stem + "_sim_track.pgm")
        simulator_track_img = np.where(simulator_track_mask, np.uint8(254), np.uint8(0))
        Image.fromarray(simulator_track_img, mode="L").save(simulator_track_path)

        inner_boundary_path = output_prefix.parent / (output_prefix.stem + "_inner_boundary.pgm")
        outer_boundary_path = output_prefix.parent / (output_prefix.stem + "_outer_boundary.pgm")
        inner_boundary_img = np.where(boundary_from_fill(inner_f), np.uint8(254), np.uint8(0))
        outer_boundary_img = np.where(boundary_from_fill(outer_f), np.uint8(254), np.uint8(0))
        Image.fromarray(inner_boundary_img, mode="L").save(inner_boundary_path)
        Image.fromarray(outer_boundary_img, mode="L").save(outer_boundary_path)

        centerline_path = output_prefix.parent / (output_prefix.stem + "_centerline.pgm")
        centerline_img = np.where(centerline_mask, np.uint8(254), np.uint8(0))
        Image.fromarray(centerline_img, mode="L").save(centerline_path)

        # 可视化：保存边界检测预览图
        vis_path = args.preview_output.resolve()
        vis_path.parent.mkdir(parents=True, exist_ok=True)
        vis = Image.fromarray(image).convert("RGB")
        vis_data = np.array(vis)

        def draw_boundary(mask, color):
            """在可视化图像上绘制边界。"""
            edge = boundary_from_fill(mask)
            if edge is None:
                return
            vis_data[edge] = color

        track_overlay = np.zeros_like(vis_data)
        track_overlay[track_mask] = [80, 220, 120]
        vis_data = np.where(
            track_overlay > 0,
            (0.65 * vis_data + 0.35 * track_overlay).astype(np.uint8),
            vis_data,
        )

        draw_boundary(original_outer, [255, 180, 180])
        draw_boundary(original_inner, [170, 200, 255])
        draw_boundary(simulator_outer, [255, 120, 120])
        draw_boundary(simulator_inner, [80, 140, 255])
        draw_boundary(outer_f, [255, 0, 0])
        draw_boundary(inner_f, [0, 0, 255])
        vis_data[centerline_mask] = [255, 230, 0]

        Image.fromarray(vis_data).save(vis_path)

        print(f"track_mask={track_path}")
        print(f"simulator_track_mask={simulator_track_path}")
        print(f"inner_boundary_mask={inner_boundary_path}")
        print(f"outer_boundary_mask={outer_boundary_path}")
        print(f"centerline_mask={centerline_path}")
        print(f"visualization={vis_path}")
    for key, value in stats.items():
        print(f"{key}={value}")


if __name__ == "__main__":
    main()

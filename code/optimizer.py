#!/usr/bin/env python3
"""基于序列二次规划 (SQP) 的赛道轨迹优化函数库。"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import osqp
import pandas as pd
import scipy.linalg as la
import scipy.sparse as sp

from package_paths import get_default_output_root

DEFAULT_TRACK_CSV = get_default_output_root() / "csv" / "processed_track.csv"


def load_track_data(filepath: Path = DEFAULT_TRACK_CSV) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """加载赛道边界数据。

    读取 CSV 文件并提取内边界 (p)、外边界 (q) 以及横跨向量 (v = q - p)。

    Args:
        filepath: 赛道 CSV 文件的路径。

    Returns:
        (p, q, v) 的元组，均为 [N, 2] 的 numpy 数组。
    """
    df = pd.read_csv(filepath)
    p = df[['left_border_x', 'left_border_y']].values
    q = df[['right_border_x', 'right_border_y']].values
    v = q - p
    return p, q, v


def build_difference_matrix(N: int) -> sp.csc_matrix:
    """构建一阶差分矩阵 A (闭环)。

    用于计算相邻轨迹点之间的坐标差值，支持环形赛道闭合。

    Args:
        N: 控制点数量。

    Returns:
        [N, N] 的 CSC 格式稀疏差分矩阵。
    """
    diag = np.ones(N) * (-1)
    off_diag = np.ones(N - 1)
    A = sp.diags([diag, off_diag], [0, 1], shape=(N, N)).tolil()
    A[N - 1, 0] = 1  # 闭环边界：最后一个点减去第一个点
    return A.tocsc()


def calculate_distance_factor(A: sp.csc_matrix, p: np.ndarray, v: np.ndarray) -> tuple[sp.csc_matrix, np.ndarray]:
    """计算距离优化目标的 Hessian 矩阵 Hs 和一次项 fs。

    对应论文公式 (7.1) 和 (7.2)，用于最小化轨迹相对于中心线的距离。

    Args:
        A: 差分矩阵。
        p: 内边界坐标。
        v: 横跨向量。

    Returns:
        (Hs, fs) 元组。
    """
    N = len(p)
    px = p[:, 0]
    py = p[:, 1]
    Vx = sp.diags(v[:, 0])
    Vy = sp.diags(v[:, 1])
    ATA = A.T @ A

    Hs_x = Vx.T @ ATA @ Vx
    fs_x = Vx.T @ ATA @ px
    Hs_y = Vy.T @ ATA @ Vy
    fs_y = Vy.T @ ATA @ py

    Hs = 2 * (Hs_x + Hs_y)
    fs = 2 * (fs_x + fs_y)
    return Hs.tocsc(), fs


def calculate_derivative_matrices(r: np.ndarray) -> tuple[sp.diags, sp.diags, sp.diags]:
    """计算基于当前参考轨迹的曲率权重矩阵 Txx, Tyy, Txy。

    对应曲率公式中的一阶导数权重项。

    Args:
        r: 当前参考轨迹点 [N, 2]。

    Returns:
        (Txx, Tyy, Txy) 三个对角稀疏矩阵。
    """
    rx = r[:, 0]
    ry = r[:, 1]
    
    # 重新获取未归一化的中心差分导数
    rx_prime_unnorm = (np.roll(rx, -1) - np.roll(rx, 1)) / 2.0
    ry_prime_unnorm = (np.roll(ry, -1) - np.roll(ry, 1)) / 2.0
    
    denominator = (rx_prime_unnorm**2 + ry_prime_unnorm**2)**3 + 1e-8
    
    Txx_diag = (ry_prime_unnorm**2) / denominator
    Tyy_diag = (rx_prime_unnorm**2) / denominator
    Txy_diag = (-2 * rx_prime_unnorm * ry_prime_unnorm) / denominator
    
    return sp.diags(Txx_diag), sp.diags(Tyy_diag), sp.diags(Txy_diag)


def calculate_M_matrix(N: int) -> sp.csc_matrix:
    """构造三次样条的二阶导数映射矩阵 M (M = B^-1 * C)。

    用于将控制点比例 alpha 映射到轨迹的二阶导数空间。

    Args:
        N: 点的数量。

    Returns:
        [N, N] 的 CSC 格式映射矩阵 M。
    """
    B = sp.diags([1, 4, 1], [-1, 0, 1], shape=(N, N), dtype=float).tolil()
    C = sp.diags([6, -12, 6], [-1, 0, 1], shape=(N, N), dtype=float).tolil()
    B[0, N - 1] = 1
    B[N - 1, 0] = 1
    C[0, N - 1] = 6
    C[N - 1, 0] = 6
    
    M_dense = la.solve(B.toarray(), C.toarray(), assume_a='sym')
    # 截断极小值以恢复稀疏性
    M_dense[np.abs(M_dense) < 1e-4] = 0.0
    return sp.csc_matrix(M_dense)


def calculate_curvature_factor(
    M: sp.csc_matrix, 
    Txx: sp.diags, 
    Tyy: sp.diags, 
    Txy: sp.diags, 
    p: np.ndarray, 
    v: np.ndarray
) -> tuple[sp.csc_matrix, np.ndarray]:
    """计算曲率优化目标的 Hessian 矩阵 Hk 和一次项 fk。

    Args:
        M: 二阶导数映射矩阵。
        Txx, Tyy, Txy: 导数权重对角阵。
        p: 内边界。
        v: 横跨向量。

    Returns:
        (Hk, fk) 元组。
    """
    px = p[:, 0]
    py = p[:, 1]
    Vx = sp.diags(v[:, 0])
    Vy = sp.diags(v[:, 1])
    
    MT_Txx_M = M.T @ Txx @ M
    MT_Tyy_M = M.T @ Tyy @ M
    MT_Txy_M = M.T @ Txy @ M
    
    term1 = Vx.T @ MT_Txx_M @ Vx
    term_cross = Vx.T @ MT_Txy_M @ Vy
    term_cross_sym = term_cross + term_cross.T
    term2 = Vy.T @ MT_Tyy_M @ Vy
    
    Hk = 2 * (term1 + term2) + term_cross_sym
    fk = 2 * (Vx.T @ MT_Txx_M @ px + Vy.T @ MT_Tyy_M @ py) + Vy.T @ MT_Txy_M @ px + Vx.T @ MT_Txy_M @ py
    
    return Hk.tocsc(), fk


def calculate_boundary_normals(bound_pts: np.ndarray, v: np.ndarray) -> np.ndarray:
    """计算指向赛道内部的边界单位法向量。

    Args:
        bound_pts: 边界点坐标 [N, 2]。
        v: 内至外边界向量，用于确定法线指向赛道内部。

    Returns:
        单位法向量数组 [N, 2]。
    """
    dp = (np.roll(bound_pts, -1, axis=0) - np.roll(bound_pts, 1, axis=0)) / 2.0
    dp_norm = np.linalg.norm(dp, axis=1, keepdims=True)
    tangent = dp / (dp_norm + 1e-8)
    n1 = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    n2 = np.column_stack([tangent[:, 1], -tangent[:, 0]])
    dot1 = np.sum(n1 * v, axis=1)
    return np.where(dot1[:, np.newaxis] > 0, n1, n2)


def calculate_wv_per_point(r: np.ndarray, v: np.ndarray, l_v: float, b_v: float) -> np.ndarray:
    """计算车辆在当前轨迹点处的横向投影半宽 wv。

    Args:
        r: 轨迹点。
        v: 横跨向量。
        l_v: 车辆长度。
        b_v: 车辆宽度。

    Returns:
        每个点对应的 wv 数组。
    """
    d = np.roll(r, -1, axis=0) - r
    d_norm_safe = np.maximum(np.linalg.norm(d, axis=1), 1e-8)
    v_norm_safe = np.maximum(np.linalg.norm(v, axis=1), 1e-8)
    
    dot_vd = np.sum(v * d, axis=1)
    cos_val = np.clip(dot_vd / (v_norm_safe * d_norm_safe), -1.0, 1.0)
    angle_vd = np.arccos(cos_val)
    angle_vehicle = np.arctan2(b_v, l_v)
    diagonal = np.sqrt(l_v**2 + b_v**2) / 2.0
    return diagonal * np.abs(np.cos(angle_vd - angle_vehicle))


def evaluate_quadratic_objective(alpha: np.ndarray, H: sp.csc_matrix, f: np.ndarray) -> float:
    """评价二次代价函数 0.5 * x^T H x + f^T x。

    Args:
        alpha: 决策变量。
        H: Hessian 矩阵。
        f: 一次项向量。

    Returns:
        函数评价值。
    """
    return float(0.5 * (alpha @ (H @ alpha)) + (f @ alpha))


def compute_reference_objective_scales(
    p: np.ndarray,
    v: np.ndarray,
    M: sp.csc_matrix,
    Hs: sp.csc_matrix,
    fs: np.ndarray,
    alpha_ref: np.ndarray,
) -> dict[str, float]:
    """计算参考轨迹下的目标函数分量值，用于归一化权重。

    Args:
        p, v: 赛道几何。
        M: 映射矩阵。
        Hs, fs: 距离代价项。
        alpha_ref: 参考决策变量。

    Returns:
        包含分量值和缩放尺度的字典。
    """
    r_ref = p + v * alpha_ref[:, np.newaxis]
    Txx, Tyy, Txy = calculate_derivative_matrices(r_ref)
    Hk, fk = calculate_curvature_factor(M, Txx, Tyy, Txy, p, v)
    
    Jk = evaluate_quadratic_objective(alpha_ref, Hk, fk)
    Js = evaluate_quadratic_objective(alpha_ref, Hs, fs)
    return {
        "Jk_ref": Jk,
        "Js_ref": Js,
        "scale_k": max(abs(Jk), 1e-8),
        "scale_s": max(abs(Js), 1e-8),
    }


def optimize_trajectory(
    p: np.ndarray,
    v: np.ndarray,
    l_v: float,
    b_v: float,
    ws: float,
    epsilon: float = 0,
    max_iter: int = 100,
    convergence_tol: float = 1e-3,
    gamma_normal: float = 0.5,
    gamma_inaccurate: float = 0.1,
    normalize_objective: bool = True,
) -> tuple[np.ndarray, np.ndarray]:
    """使用 OSQP 进行序列二次规划 (SQP) 迭代，求解最优轨迹 alpha。

    Args:
        p: 内边界 [N, 2]。
        v: 横跨向量 [N, 2]。
        l_v, b_v: 车长车宽。
        ws: 安全距离裕度。
        epsilon: 距离项权重。
        max_iter: 最大迭代次数。
        convergence_tol: 收敛阈值，使用 max|Δalpha| 判断。
        gamma_normal: 正常情况的松弛步长。
        gamma_inaccurate: 求解不精确时的收缩步长。
        normalize_objective: 是否进行目标函数归一化。

    Returns:
        (最优轨迹点 [N, 2], 最优 alpha [N]) 的元组。
    """
    N = len(p)
    M = calculate_M_matrix(N)
    A_diff = build_difference_matrix(N)
    Hs, fs = calculate_distance_factor(A_diff, p, v)
    Hs = (Hs + Hs.T) / 2.0 + sp.diags(np.ones(N) * 1e-4)
    
    alpha_ref = np.full(N, 0.5, dtype=float)
    if normalize_objective:
        scale_info = compute_reference_objective_scales(p, v, M, Hs, fs, alpha_ref)
        scale_k, scale_s = scale_info["scale_k"], scale_info["scale_s"]
        print(f"归一化 | Jk={scale_info['Jk_ref']:.2e}, Js={scale_info['Js_ref']:.2e}, ratio={scale_k/scale_s:.2f}")
    else:
        scale_k, scale_s = 1.0, 1.0
    
    q_bound = p + v
    n_I = calculate_boundary_normals(p, v)
    n_O = calculate_boundary_normals(q_bound, v)
    v_dot_nI = np.maximum(np.sum(n_I * v, axis=1), 1e-4)
    v_dot_nO = np.maximum(np.sum(n_O * v, axis=1), 1e-4)
    
    A_constr = sp.eye(N).tocsc()
    reg = 1e-6 * sp.eye(N).tocsc()
    
    for iteration in range(max_iter):
        r_current = p + v * alpha_ref[:, np.newaxis]
        wv = calculate_wv_per_point(r_current, v, l_v, b_v)
        
        # 修正逻辑：alpha_i 为无量纲对比因子 [0, 1]
        alpha_min = np.clip((wv + ws) / v_dot_nI, 0.0, 1.0)
        alpha_max = np.clip(1.0 - (wv + ws) / v_dot_nO, 0.0, 1.0)
        
        # 修复不可行约束（局部噪声导致）
        infeasible = alpha_min > alpha_max
        if np.any(infeasible):
            alpha_min[infeasible] = np.maximum(0.0, alpha_ref[infeasible] - 0.02)
            alpha_max[infeasible] = np.minimum(1.0, alpha_ref[infeasible] + 0.02)
        
        Txx, Tyy, Txy = calculate_derivative_matrices(r_current)
        Hk, fk = calculate_curvature_factor(M, Txx, Tyy, Txy, p, v)
        Hk = (Hk + Hk.T) / 2.0 + sp.diags(np.ones(N) * 1e-4)

        P = sp.csc_matrix(Hk / scale_k + epsilon * (Hs / scale_s) + reg)
        q = fk / scale_k + epsilon * (fs / scale_s)
        
        prob = osqp.OSQP()
        prob.setup(P=P, q=q, A=A_constr, l=alpha_min, u=alpha_max, verbose=False, max_iter=20000)
        res = prob.solve()
        
        if res.info.status not in ['solved', 'solved inaccurate']:
            print(f"迭代 {iteration+1} 失败: {res.info.status}")
            break
            
        alpha_new = np.clip(res.x, 0.0, 1.0)
        diff = np.max(np.abs(alpha_new - alpha_ref))
        gamma = gamma_normal if res.info.status == 'solved' else gamma_inaccurate
        
        print(f"迭代 {iteration+1:2d} | max|Δα|: {diff:.6f} | 代价: {evaluate_quadratic_objective(alpha_new, P, q):.2e}")
        
        alpha_ref = (1 - gamma) * alpha_ref + gamma * alpha_new
        if diff < convergence_tol:
            print(f"收敛完成 (阈值 {convergence_tol:.2e})。")
            break
    else:
        print(f"达到最大迭代次数 {max_iter}，当前 max|Δα|={diff:.6f}。")

    return p + v * alpha_ref[:, np.newaxis], alpha_ref


def estimate_closed_curve_curvature(points: np.ndarray) -> np.ndarray:
    """计算闭合曲线各点处的曲率。

    Args:
        points: 点集 [N, 2]。

    Returns:
        曲率数组 [N]。
    """
    d1 = (np.roll(points, -1, axis=0) - np.roll(points, 1, axis=0)) / 2.0
    d2 = np.roll(points, -1, axis=0) - 2.0 * points + np.roll(points, 1, axis=0)
    denom = np.maximum(np.linalg.norm(d1, axis=1) ** 3, 1e-8)
    return np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / denom


def analyze_trajectory_candidate(
    p: np.ndarray,
    v: np.ndarray,
    r: np.ndarray,
    alpha: np.ndarray,
    l_v: float,
    b_v: float,
    ws: float,
    wheelbase: float | None = None,
) -> dict[str, object]:
    """分析轨迹候选项的各项性能指标。

    Args:
        p, v: 赛道几何。
        r: 轨迹坐标。
        alpha: 决策变量。
        l_v, b_v: 车长轴距等物理参数。
        ws: 设定的安全距离。
        wheelbase: 可选轴距，用于估算转角。

    Returns:
        指标字典。
    """
    q_bound = p + v
    n_I, n_O = calculate_boundary_normals(p, v), calculate_boundary_normals(q_bound, v)
    v_dot_nI, v_dot_nO = np.sum(n_I * v, axis=1), np.sum(n_O * v, axis=1)
    wv = calculate_wv_per_point(r, v, l_v, b_v)

    inner_clear = alpha * v_dot_nI - wv
    outer_clear = (1.0 - alpha) * v_dot_nO - wv
    min_clear = float(min(inner_clear.min(), outer_clear.min()))
    curvature = estimate_closed_curve_curvature(r)
    
    metrics = {
        "min_body_to_wall": min_clear,
        "max_curvature": float(np.max(curvature)),
        "curvature_p99": float(np.percentile(curvature, 99)),
        "max_required_steering_deg": float(np.degrees(np.arctan(wheelbase * np.max(curvature)))) if wheelbase else None
    }
    return metrics

def evaluate_trajectory_feasibility(
    metrics: dict[str, object],
    min_body_to_wall: float,
    max_steering_deg: float | None = None,
) -> tuple[bool, list[str]]:
    """根据几项关键指标判断轨迹是否通过门禁检查。

    Args:
        metrics: `analyze_trajectory_candidate` 返回的指标字典。
        min_body_to_wall: 允许的最小车身到墙体间距。
        max_steering_deg: 可选的最大允许转角。

    Returns:
        (是否通过, 失败原因列表)。
    """
    failures: list[str] = []

    body_clearance = float(metrics["min_body_to_wall"])
    if body_clearance < min_body_to_wall:
        failures.append(
            f"min_body_to_wall={body_clearance:.4f}m < {min_body_to_wall:.4f}m"
        )

    steering = metrics.get("max_required_steering_deg")
    if max_steering_deg is not None and steering is not None:
        steering_value = float(steering)
        if steering_value > max_steering_deg:
            failures.append(
                f"max_required_steering_deg={steering_value:.2f} > {max_steering_deg:.2f}"
            )

    return len(failures) == 0, failures

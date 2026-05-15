#!/usr/bin/env python3
"""LQR 控制器的纯数学计算模块。

本模块不依赖 ROS，包含以下功能：
    - 路径切向航向角和离散曲率的预计算。
    - 离散横向误差动力学模型的构建。
    - 离散代数 Riccati 方程（DARE）求解与最优增益计算。
    - LQR 反馈 + 曲率前馈的转向角合成。
"""
from __future__ import annotations

import math

import numpy as np
from scipy.linalg import solve_discrete_are


def wrap_angle(angle: float) -> float:
    """将角度归一化到 [-pi, pi] 区间。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_path_headings(points: np.ndarray, closed_loop: bool) -> np.ndarray:
    """计算每个路径点的切向航向角。

    使用前后相邻点的差分方向作为切向估计。

    Args:
        points: 路径点数组，形状 [N, 2]。
        closed_loop: 是否为闭环路径。

    Returns:
        每个路径点的航向角数组（弧度），形状 [N]。
    """
    headings = np.zeros(len(points), dtype=float)
    for idx in range(len(points)):
        if closed_loop:
            previous_point = points[idx - 1]
            next_point = points[(idx + 1) % len(points)]
        elif idx == 0:
            previous_point = points[idx]
            next_point = points[idx + 1]
        elif idx == len(points) - 1:
            previous_point = points[idx - 1]
            next_point = points[idx]
        else:
            previous_point = points[idx - 1]
            next_point = points[idx + 1]

        tangent = next_point - previous_point
        if float(np.linalg.norm(tangent)) <= 1e-9:
            tangent = np.array([1.0, 0.0], dtype=float)
        headings[idx] = math.atan2(float(tangent[1]), float(tangent[0]))
    return headings


def compute_path_curvatures(points: np.ndarray, closed_loop: bool) -> np.ndarray:
    """计算每个路径点的有符号 Menger 曲率。

    使用三点离散公式：kappa = 2 * cross(AB, BC) / (|AB| * |BC| * |AC|)。
    正值表示左转，负值表示右转。

    Args:
        points: 路径点数组，形状 [N, 2]。
        closed_loop: 是否为闭环路径。

    Returns:
        每个路径点的曲率数组（1/m），形状 [N]。
    """
    curvatures = np.zeros(len(points), dtype=float)
    if len(points) < 3:
        return curvatures

    for idx in range(len(points)):
        if not closed_loop and (idx == 0 or idx == len(points) - 1):
            continue

        previous_point = points[idx - 1]
        current_point = points[idx]
        next_point = points[(idx + 1) % len(points)]

        first_chord = current_point - previous_point
        second_chord = next_point - current_point
        full_chord = next_point - previous_point
        denominator = (
            float(np.linalg.norm(first_chord))
            * float(np.linalg.norm(second_chord))
            * float(np.linalg.norm(full_chord))
        )
        if denominator <= 1e-9:
            continue

        cross_z = float(
            first_chord[0] * second_chord[1]
            - first_chord[1] * second_chord[0]
        )
        curvatures[idx] = 2.0 * cross_z / denominator

    if not closed_loop:
        curvatures[0] = curvatures[1]
        curvatures[-1] = curvatures[-2]
    return curvatures


def build_lqr_model(
    speed: float,
    wheelbase: float,
    dt: float,
    min_model_speed: float,
) -> tuple[np.ndarray, np.ndarray]:
    """构建离散二阶横向误差动力学模型。

    状态向量 x = [e_lateral, e_heading]，控制输入 u = [delta_steering]。
    离散化模型：
        x[k+1] = A @ x[k] + B @ u[k]
    其中：
        A = [[1, v*dt], [0, 1]]
        B = [[0], [v*dt/L]]
    v*dt 表示一个控制周期内车辆前进的距离。小角度近似下：
        e_lateral[k+1] = e_lateral[k] + v*dt * e_heading[k]
        e_heading[k+1] = e_heading[k] + v*dt/L * delta_steering[k]

    Args:
        speed: 当前车速（m/s）。
        wheelbase: 前后轴距（m）。
        dt: 离散化时间步长（s）。
        min_model_speed: 模型最低速度下限，防止低速时矩阵退化。

    Returns:
        (A, B) 状态转移矩阵和输入矩阵的元组。
    """
    model_speed = max(abs(speed), min_model_speed)
    safe_wheelbase = max(wheelbase, 1e-6)
    safe_dt = max(dt, 1e-4)

    state_matrix = np.array(
        [
            [1.0, safe_dt * model_speed],
            [0.0, 1.0],
        ],
        dtype=float,
    )
    input_matrix = np.array(
        [
            [0.0],
            [safe_dt * model_speed / safe_wheelbase],
        ],
        dtype=float,
    )
    return state_matrix, input_matrix


def solve_lqr_gain(
    speed: float,
    wheelbase: float,
    dt: float,
    min_model_speed: float,
    q_lateral: float,
    q_heading: float,
    r_steering: float,
) -> np.ndarray:
    """求解当前速度下的离散 LQR 最优反馈增益。

    通过求解离散代数 Riccati 方程（DARE）得到最优代价矩阵 P，
    再计算反馈增益 K = (R + B'PB)^{-1} B'PA。
    solve_discrete_are(A, B, Q, R) 的输出就是 P。P 不是最终控制增益，
    而是描述当前状态误差会带来多少未来累计代价的矩阵：
        J = x'Px
    其中 x = [e_lateral, e_heading]。
    离散系统 x[k+1] = A*x[k] + B*u[k] 的最优反馈控制律为：
        u = -K*x
        K = (R + B'PB)^(-1) * B'PA
    本函数用 np.linalg.solve(left, right) 求解 left*K = right，避免显式求逆。

    Args:
        speed: 当前车速（m/s）。
        wheelbase: 前后轴距（m）。
        dt: 离散化时间步长（s）。
        min_model_speed: 模型最低速度下限。
        q_lateral: Q 矩阵中横向误差的权重。
        q_heading: Q 矩阵中航向误差的权重。
        r_steering: R 矩阵中转向输入的代价权重。

    Returns:
        形状为 [2] 的增益向量 [K_lateral, K_heading]。理论上 K 的形状是
        [1, 2]，reshape(2) 将其压成一维，方便后续直接与状态向量点乘。
    """
    state_matrix, input_matrix = build_lqr_model(
        speed=speed,
        wheelbase=wheelbase,
        dt=dt,
        min_model_speed=min_model_speed,
    )
    state_cost = np.diag([max(q_lateral, 1e-9), max(q_heading, 1e-9)])
    input_cost = np.array([[max(r_steering, 1e-9)]], dtype=float)
    riccati_solution = solve_discrete_are(
        state_matrix,
        input_matrix,
        state_cost,
        input_cost,
    )
    gain = np.linalg.solve(
        input_cost + input_matrix.T @ riccati_solution @ input_matrix,
        input_matrix.T @ riccati_solution @ state_matrix,
    )
    return np.asarray(gain, dtype=float).reshape(2)


def compute_lqr_steering(
    lateral_error: float,
    heading_error: float,
    curvature_ref: float,
    speed: float,
    wheelbase: float,
    dt: float,
    min_model_speed: float,
    q_lateral: float,
    q_heading: float,
    r_steering: float,
    feedforward_gain: float,
) -> tuple[float, float, float, np.ndarray]:
    """计算 LQR 反馈 + 曲率前馈的最终转向角。

    最终转向角 = 反馈项 + 前馈项：
        delta_fb = -K @ [e_lateral, e_heading]
        delta_ff = feedforward_gain * atan(wheelbase * curvature_ref)
        delta    = delta_fb + delta_ff

    Args:
        lateral_error: 横向误差（m），车辆在参考路径左侧为正。
        heading_error: 航向误差（rad），逆时针偏转为正。
        curvature_ref: 参考路径曲率（1/m）。
        speed: 当前车速（m/s）。
        wheelbase: 前后轴距（m）。
        dt: 控制周期（s）。
        min_model_speed: 模型最低速度。
        q_lateral: 横向误差权重。
        q_heading: 航向误差权重。
        r_steering: 转向代价权重。
        feedforward_gain: 前馈增益系数。

    Returns:
        (delta_total, delta_feedback, delta_feedforward, gain) 四元组。
    """
    gain = solve_lqr_gain(
        speed=speed,
        wheelbase=wheelbase,
        dt=dt,
        min_model_speed=min_model_speed,
        q_lateral=q_lateral,
        q_heading=q_heading,
        r_steering=r_steering,
    )
    state = np.array([lateral_error, heading_error], dtype=float)
    feedback = -float(gain @ state)
    feedforward = feedforward_gain * math.atan(wheelbase * curvature_ref)
    return feedback + feedforward, feedback, feedforward, gain

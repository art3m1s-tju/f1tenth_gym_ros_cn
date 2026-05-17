"""包含粗网格搜索与局部细化的 LQR 参数扫描引擎。"""
from __future__ import annotations

import itertools
import math
import os
import warnings
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field

import numpy as np
try:
    from tqdm import tqdm
except ImportError:
    def tqdm(iterable=None, **kwargs):
        """在未安装 tqdm 时提供最小兼容进度条。

        Args:
            iterable: 可选的可迭代对象；提供时直接原样返回。
            **kwargs: 兼容 tqdm 调用签名的关键字参数。

        Returns:
            原始可迭代对象，或具备 ``update`` 上下文管理接口的空实现。
        """

        class _Dummy:
            """无输出的进度条占位实现。"""

            def __init__(self, total=None, **kw):
                """初始化空进度条。

                Args:
                    total: 任务总数，仅用于兼容调用方。
                    **kw: 其他兼容参数。
                """

                self._n = 0
                self._total = total

            def update(self, n=1):
                """记录已完成任务数。

                Args:
                    n: 本次增加的完成数量。
                """

                self._n += n

            def __enter__(self):
                """进入上下文管理器。

                Returns:
                    当前空进度条实例。
                """

                return self

            def __exit__(self, *a):
                """退出上下文管理器。

                Args:
                    *a: 异常类型、异常实例和回溯对象。
                """

                pass
        if iterable is not None:
            return iterable
        return _Dummy(**kwargs)

from lqr_sweep.lookup_table import LqrParams, LqrLookupTable
from lqr_sweep.objective import DEFAULT_OBJECTIVE, ObjectiveConfig, compute_objective
from lqr_sweep.sim_harness import SimConfig, run_single_sim, load_trajectory_cache


@dataclass
class SweepConfig:
    """参数扫描流程的配置。

    Attributes:
        sim_config: 传递给每次仿真的基础配置。
        speed_points: 需要分别调参的目标速度采样点。
        q_lateral_range: 横向误差权重的搜索范围。
        q_heading_range: 航向误差权重的搜索范围。
        r_steering_range: 转向输入权重的搜索范围。
        ff_gain_range: 曲率前馈增益的搜索范围。
        coarse_grid_sizes: 四个参数维度的粗网格采样数量。
        refine_top_k: 对粗搜索排名前 K 的参数做局部细化。
        refine_grid_size: 每个维度的局部细化采样数量。
        refine_range_factor: 局部细化范围相对中心值的比例。
        objective_config: 目标函数配置。
        max_workers: 并行仿真的最大进程数；为 ``None`` 时自动使用 CPU 数量。
    """

    sim_config: SimConfig
    speed_points: list[float] = field(default_factory=lambda: [0.5, 1.0, 1.5, 2.0, 2.5, 3.0])
    q_lateral_range: tuple[float, float] = (1.0, 12.0)
    q_heading_range: tuple[float, float] = (0.5, 6.0)
    r_steering_range: tuple[float, float] = (3.0, 30.0)
    ff_gain_range: tuple[float, float] = (0.8, 1.2)
    coarse_grid_sizes: tuple[int, int, int, int] = (5, 4, 5, 3)
    refine_top_k: int = 3
    refine_grid_size: int = 3
    refine_range_factor: float = 0.3
    objective_config: ObjectiveConfig = DEFAULT_OBJECTIVE
    max_workers: int | None = None


@dataclass
class SpeedPointResult:
    """单个速度点的扫描结果。

    Attributes:
        speed: 本次扫描对应的目标速度。
        best_params: 该速度下得分最低的 LQR 参数。
        best_score: 最优参数对应的目标函数分数。
        all_results: 所有评估过的参数与分数列表。
    """

    speed: float
    best_params: LqrParams
    best_score: float
    all_results: list[tuple[LqrParams, float]] = field(default_factory=list)


def _build_coarse_grid(config: SweepConfig) -> list[LqrParams]:
    """根据配置生成粗搜索参数网格。

    Args:
        config: 参数扫描配置。

    Returns:
        粗网格中所有 LQR 参数组合。
    """

    n_ql, n_qh, n_rs, n_ff = config.coarse_grid_sizes
    q_lats = np.logspace(np.log10(config.q_lateral_range[0]), np.log10(config.q_lateral_range[1]), n_ql)
    q_heads = np.logspace(np.log10(config.q_heading_range[0]), np.log10(config.q_heading_range[1]), n_qh)
    r_steers = np.logspace(np.log10(config.r_steering_range[0]), np.log10(config.r_steering_range[1]), n_rs)
    ff_gains = np.linspace(config.ff_gain_range[0], config.ff_gain_range[1], n_ff)

    return [
        LqrParams(q_lateral=float(ql), q_heading=float(qh), r_steering=float(rs), feedforward_gain=float(ff))
        for ql, qh, rs, ff in itertools.product(q_lats, q_heads, r_steers, ff_gains)
    ]


def _build_refine_grid(center: LqrParams, config: SweepConfig) -> list[LqrParams]:
    """围绕一个中心参数生成局部细化网格。

    Args:
        center: 粗搜索阶段选出的中心参数。
        config: 参数扫描配置。

    Returns:
        以 ``center`` 为中心、按配置范围裁剪后的局部参数组合。
    """

    factor = config.refine_range_factor
    n = config.refine_grid_size

    def _local_range(val: float, lo: float, hi: float) -> np.ndarray:
        """生成单个参数维度的局部线性采样范围。

        Args:
            val: 中心值。
            lo: 全局搜索下界。
            hi: 全局搜索上界。

        Returns:
            限制在全局边界内的局部采样数组。
        """

        return np.linspace(max(lo, val * (1.0 - factor)), min(hi, val * (1.0 + factor)), n)

    q_lats = _local_range(center.q_lateral, *config.q_lateral_range)
    q_heads = _local_range(center.q_heading, *config.q_heading_range)
    r_steers = _local_range(center.r_steering, *config.r_steering_range)
    ff_gains = _local_range(center.feedforward_gain, *config.ff_gain_range)

    return [
        LqrParams(q_lateral=float(ql), q_heading=float(qh), r_steering=float(rs), feedforward_gain=float(ff))
        for ql, qh, rs, ff in itertools.product(q_lats, q_heads, r_steers, ff_gains)
    ]


def _evaluate_single(args: tuple) -> tuple[LqrParams, float, str, dict]:
    """评估单个参数组合的仿真分数。

    Args:
        args: ``(SimConfig, LqrParams, target_speed)`` 元组。

    Returns:
        ``(参数, 分数, 失败原因, 指标)``；仿真失败或约束淘汰时分数为无穷大。
    """

    sim_config, params, target_speed, objective_config = args
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Chosen integrator is RK4")
        result = run_single_sim(sim_config, params, target_speed)
    if not result.success:
        return params, math.inf, result.failure_reason, result.metrics
    score = compute_objective(result.metrics, objective_config)
    failure_reason = "" if math.isfinite(score) else "objective_constraints"
    return params, score, failure_reason, result.metrics


def _run_batch(
    args_list: list[tuple],
    executor: ProcessPoolExecutor,
    desc: str = "Evaluating",
) -> list[tuple[LqrParams, float, str, dict]]:
    """使用进程池并行评估一批参数组合。

    Args:
        args_list: 每个待评估任务的参数元组列表。
        executor: 外部创建的进程池执行器。
        desc: 进度条显示的任务描述。

    Returns:
        所有任务完成后的 ``(参数, 分数, 失败原因, 指标)`` 列表。
    """

    futures = [executor.submit(_evaluate_single, a) for a in args_list]
    results = []
    with tqdm(total=len(futures), desc=desc, unit="sim", leave=False) as pbar:
        for f in as_completed(futures):
            results.append(f.result())
            pbar.update(1)
    return results


def _summarize_failed_results(
    speed: float,
    results: list[tuple[LqrParams, float, str, dict]],
) -> str:
    """汇总某个速度点下所有候选参数失败的原因。

    Args:
        speed: 当前扫描的目标速度。
        results: 该速度下所有仿真评估结果。

    Returns:
        包含失败计数和关键指标范围的诊断字符串。
    """

    reason_counts: dict[str, int] = {}
    metrics_by_key: dict[str, list[float]] = {
        "max_abs_e_y": [],
        "steering_saturation_ratio": [],
        "steering_rate_rms_deg_s": [],
        "mean_abs_e_y": [],
        "mean_abs_e_psi_deg": [],
    }
    for _, _, reason, metrics in results:
        reason_counts[reason or "finite"] = reason_counts.get(reason or "finite", 0) + 1
        for key in metrics_by_key:
            value = metrics.get(key)
            if isinstance(value, (int, float)) and math.isfinite(float(value)):
                metrics_by_key[key].append(float(value))

    lines = [f"No finite sweep result for speed {speed:.1f} m/s."]
    lines.append(f"Failure counts: {reason_counts}")
    for key, values in metrics_by_key.items():
        if values:
            lines.append(
                f"{key}: min={min(values):.4g}, median={float(np.median(values)):.4g}, "
                f"max={max(values):.4g}"
            )
    return " ".join(lines)


def run_speed_point_sweep(
    speed: float,
    sweep_config: SweepConfig,
    verbose: bool = True,
) -> SpeedPointResult:
    """对单个速度点运行粗搜索和可选局部细化。

    Args:
        speed: 当前扫描的目标速度。
        sweep_config: 参数扫描配置。
        verbose: 是否打印扫描进度和最优结果。

    Returns:
        当前速度点的最优参数、最优分数和完整评估记录。
    """

    sim_config = sweep_config.sim_config
    max_workers = sweep_config.max_workers or os.cpu_count() or 1

    coarse_grid = _build_coarse_grid(sweep_config)
    if verbose:
        print(f"  [v={speed:.1f}] Coarse grid: {len(coarse_grid)} evaluations...")

    args_list = [
        (sim_config, p, speed, sweep_config.objective_config)
        for p in coarse_grid
    ]

    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        all_results = _run_batch(args_list, executor, desc=f"v={speed:.1f} coarse")
        all_results.sort(key=lambda x: x[1])

        finite_results = [item for item in all_results if math.isfinite(item[1])]
        if sweep_config.refine_top_k > 0 and finite_results:
            top_k = finite_results[: sweep_config.refine_top_k]
            if verbose:
                best = top_k[0]
                print(f"  [v={speed:.1f}] Coarse best: score={best[1]:.4f} "
                      f"Q=({best[0].q_lateral:.2f},{best[0].q_heading:.2f}) "
                      f"R={best[0].r_steering:.2f} ff={best[0].feedforward_gain:.2f}")

            refine_grid = []
            for params, _, _, _ in top_k:
                refine_grid.extend(_build_refine_grid(params, sweep_config))

            if verbose:
                print(f"  [v={speed:.1f}] Refine grid: {len(refine_grid)} evaluations...")

            refine_args = [
                (sim_config, p, speed, sweep_config.objective_config)
                for p in refine_grid
            ]
            all_results.extend(_run_batch(refine_args, executor, desc=f"v={speed:.1f} refine"))
            all_results.sort(key=lambda x: x[1])

    finite_results = [item for item in all_results if math.isfinite(item[1])]
    if not finite_results:
        raise RuntimeError(_summarize_failed_results(speed, all_results))

    best_params, best_score, _, _ = all_results[0]
    if verbose:
        print(f"  [v={speed:.1f}] Final best: score={best_score:.4f} "
              f"Q=({best_params.q_lateral:.2f},{best_params.q_heading:.2f}) "
              f"R={best_params.r_steering:.2f} ff={best_params.feedforward_gain:.2f}")

    compact_results = [(params, score) for params, score, _, _ in all_results]
    return SpeedPointResult(
        speed=speed,
        best_params=best_params,
        best_score=best_score,
        all_results=compact_results,
    )


def run_full_sweep(sweep_config: SweepConfig, verbose: bool = True) -> LqrLookupTable:
    """遍历所有速度点并构建 LQR 增益查找表。

    Args:
        sweep_config: 参数扫描配置。
        verbose: 是否打印每个速度点的扫描过程。

    Returns:
        根据各速度点最优参数生成的增益查找表。
    """

    results = []
    for speed in sweep_config.speed_points:
        if verbose:
            print(f"\n{'='*60}")
            print(f"Sweeping speed point: {speed:.1f} m/s")
            print(f"{'='*60}")
        sp_result = run_speed_point_sweep(speed, sweep_config, verbose=verbose)
        results.append((sp_result.speed, sp_result.best_params, sp_result.best_score))

    table = LqrLookupTable.from_sweep_results(results)

    if verbose:
        print(f"\n{'='*60}")
        print("Sweep complete. Lookup table:")
        print(f"{'='*60}")
        for entry in table.entries:
            print(f"  v={entry.speed:.1f}: Q=({entry.q_lateral:.3f},{entry.q_heading:.3f}) "
                  f"R={entry.r_steering:.3f} ff={entry.feedforward_gain:.3f} score={entry.score:.4f}")

    return table

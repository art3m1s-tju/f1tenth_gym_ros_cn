"""带线性插值的按速度分段 LQR 增益查找表。"""
from __future__ import annotations

from dataclasses import dataclass, asdict
from pathlib import Path

import yaml


@dataclass
class LqrParams:
    """LQR 控制器的一组可调参数。

    Attributes:
        q_lateral: 横向误差状态在 Q 矩阵中的权重。
        q_heading: 航向误差状态在 Q 矩阵中的权重。
        r_steering: 转向输入在 R 矩阵中的权重。
        feedforward_gain: 曲率前馈转向增益。
    """

    q_lateral: float = 3.0
    q_heading: float = 1.2
    r_steering: float = 8.0
    feedforward_gain: float = 1.0


@dataclass
class LqrGainEntry:
    """某个速度采样点对应的 LQR 增益表项。

    Attributes:
        speed: 表项对应的目标速度，单位为米/秒。
        q_lateral: 横向误差权重。
        q_heading: 航向误差权重。
        r_steering: 转向输入权重。
        feedforward_gain: 曲率前馈转向增益。
        score: 参数扫描得到的目标函数分数，越小越好。
    """

    speed: float
    q_lateral: float
    q_heading: float
    r_steering: float
    feedforward_gain: float
    score: float = float("inf")

    def to_params(self) -> LqrParams:
        """将表项转换为控制器可直接使用的参数对象。

        Returns:
            不包含速度和分数信息的 ``LqrParams`` 实例。
        """

        return LqrParams(
            q_lateral=self.q_lateral,
            q_heading=self.q_heading,
            r_steering=self.r_steering,
            feedforward_gain=self.feedforward_gain,
        )


class LqrLookupTable:
    """按速度排序的 LQR 增益查找表。

    查找表会在相邻速度表项之间做线性插值；当查询速度超出范围时，
    使用最近端点的参数。

    Attributes:
        entries: 按速度升序排列的增益表项列表。
    """

    def __init__(self, entries: list[LqrGainEntry]) -> None:
        """初始化查找表并按速度排序。

        Args:
            entries: 至少包含一个速度表项的列表。

        Raises:
            ValueError: 当 ``entries`` 为空时抛出。
        """

        if not entries:
            raise ValueError("Lookup table must have at least one entry.")
        self.entries = sorted(entries, key=lambda e: e.speed)

    def interpolate(self, speed: float) -> LqrParams:
        """按速度查询并插值得到 LQR 参数。

        Args:
            speed: 当前或目标车辆速度，单位为米/秒。

        Returns:
            对应速度下的 LQR 参数。
        """

        if speed <= self.entries[0].speed:
            return self.entries[0].to_params()
        if speed >= self.entries[-1].speed:
            return self.entries[-1].to_params()

        for i in range(len(self.entries) - 1):
            lo, hi = self.entries[i], self.entries[i + 1]
            if lo.speed <= speed <= hi.speed:
                t = (speed - lo.speed) / (hi.speed - lo.speed)
                return LqrParams(
                    q_lateral=lo.q_lateral + t * (hi.q_lateral - lo.q_lateral),
                    q_heading=lo.q_heading + t * (hi.q_heading - lo.q_heading),
                    r_steering=lo.r_steering + t * (hi.r_steering - lo.r_steering),
                    feedforward_gain=lo.feedforward_gain + t * (hi.feedforward_gain - lo.feedforward_gain),
                )

        return self.entries[-1].to_params()

    def save(self, path: Path) -> None:
        """将查找表保存为 YAML 文件。

        Args:
            path: 输出 YAML 文件路径。
        """

        data = {
            "version": 1,
            "entries": [asdict(e) for e in self.entries],
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)

    @classmethod
    def load(cls, path: Path) -> "LqrLookupTable":
        """从 YAML 文件加载 LQR 查找表。

        Args:
            path: 输入 YAML 文件路径。

        Returns:
            由文件内容构造出的 ``LqrLookupTable``。
        """

        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        entries = [LqrGainEntry(**e) for e in data["entries"]]
        return cls(entries)

    @classmethod
    def from_sweep_results(cls, results: list[tuple[float, LqrParams, float]]) -> "LqrLookupTable":
        """由参数扫描结果构造查找表。

        Args:
            results: ``(速度, 最优参数, 分数)`` 元组列表。

        Returns:
            包含所有速度采样点的查找表。
        """

        entries = [
            LqrGainEntry(
                speed=speed,
                q_lateral=params.q_lateral,
                q_heading=params.q_heading,
                r_steering=params.r_steering,
                feedforward_gain=params.feedforward_gain,
                score=score,
            )
            for speed, params, score in results
        ]
        return cls(entries)

    def __len__(self) -> int:
        """返回查找表中的表项数量。

        Returns:
            表项个数。
        """

        return len(self.entries)

    def __repr__(self) -> str:
        """生成便于调试查看的字符串表示。

        Returns:
            包含速度采样点列表的字符串。
        """

        speeds = [f"{e.speed:.1f}" for e in self.entries]
        return f"LqrLookupTable(speeds=[{', '.join(speeds)}])"

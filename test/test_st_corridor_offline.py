# Copyright 2026 OpenAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Offline regression tests for ST-corridor path choice logic."""

import importlib
import math
import sys
import types
from pathlib import Path

import pytest


np = pytest.importorskip("numpy")
pytest.importorskip("scipy.spatial")


def _module(name, **attrs):
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    return module


class _Node:
    def __init__(self, *args, **kwargs):
        pass


class _QoSProfile:
    def __init__(self, *args, **kwargs):
        self.durability = None


class _DurabilityPolicy:
    TRANSIENT_LOCAL = "transient_local"


class _Msg:
    pass


@pytest.fixture(name="st_module")
def fixture_st_module(monkeypatch):
    code_dir = Path(__file__).resolve().parents[1] / "code"
    monkeypatch.syspath_prepend(str(code_dir))

    rclpy = _module(
        "rclpy",
        init=lambda *args, **kwargs: None,
        ok=lambda: False,
        shutdown=lambda: None,
        spin=lambda node: None,
    )
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", _module("rclpy.node", Node=_Node))
    monkeypatch.setitem(
        sys.modules,
        "rclpy.qos",
        _module(
            "rclpy.qos",
            DurabilityPolicy=_DurabilityPolicy,
            QoSProfile=_QoSProfile,
        ),
    )
    monkeypatch.setitem(sys.modules, "geometry_msgs", _module("geometry_msgs"))
    monkeypatch.setitem(
        sys.modules,
        "geometry_msgs.msg",
        _module("geometry_msgs.msg", PoseStamped=_Msg),
    )
    monkeypatch.setitem(sys.modules, "nav_msgs", _module("nav_msgs"))
    monkeypatch.setitem(
        sys.modules,
        "nav_msgs.msg",
        _module("nav_msgs.msg", Odometry=_Msg, Path=_Msg),
    )
    monkeypatch.setitem(sys.modules, "sensor_msgs", _module("sensor_msgs"))
    monkeypatch.setitem(
        sys.modules,
        "sensor_msgs.msg",
        _module("sensor_msgs.msg", LaserScan=_Msg),
    )
    monkeypatch.setitem(sys.modules, "std_msgs", _module("std_msgs"))
    monkeypatch.setitem(
        sys.modules,
        "std_msgs.msg",
        _module("std_msgs.msg", Float32=_Msg),
    )
    sys.modules.pop("pnc_rc.corridor.st_corridor_planner", None)
    return importlib.import_module("pnc_rc.corridor.st_corridor_planner")


class _StraightTrackFilter:

    def __init__(self, half_width):
        self.half_width = half_width

    def inside_drivable_area(self, xy, margin):
        return abs(float(xy[1])) <= self.half_width - margin


def _straight_planner(st_module):
    planner = st_module.StCorridorPlanner.__new__(st_module.StCorridorPlanner)
    xs = np.linspace(0.0, 20.0, 81)
    planner.reference = st_module.ReferencePath(
        np.column_stack([xs, np.zeros_like(xs)]),
        closed_loop=False,
    )
    planner.track_filter = _StraightTrackFilter(half_width=1.3)
    planner.lookahead_m = 5.0
    planner.sample_ds_m = 0.25
    planner.min_border_clearance = 0.26
    planner.dynamic_prediction_time = 1.2
    planner.min_speed = 0.35
    planner.target_speed = 1.0
    planner.avoidance_max_speed = 1.0
    planner.max_lateral_offset_m = 0.75
    planner.lateral_offset_step_m = 0.15
    planner.desired_obstacle_clearance = 0.12
    return planner


def _minimum_clearance(points, obstacle_xy, obstacle_radius):
    distances = [
        float(np.linalg.norm(point - obstacle_xy)) - obstacle_radius
        for point in points
    ]
    return min(distances)


def test_static_half_meter_obstacle_selects_offset_with_clearance(st_module):
    planner = _straight_planner(st_module)
    projection = planner.reference.project(np.array([0.0, 0.0]))
    obstacle_xy = np.array([2.5, 0.0])
    obstacle = st_module.CorridorObstacle(
        centroid_xy=obstacle_xy,
        s=2.5,
        d=0.0,
        radius=0.5,
        vs=0.0,
        vd=0.0,
        points=20,
    )

    mode, offset, points, clearance, candidate_count, free_count = (
        planner._choose_path(projection, [obstacle])
    )

    assert mode == "avoid"
    assert abs(offset) >= planner.lateral_offset_step_m
    assert clearance > 0.0
    assert _minimum_clearance(points, obstacle_xy, 0.5) > 0.0
    assert candidate_count > free_count > 0


def test_without_obstacle_passes_through_near_zero_offset(st_module):
    planner = _straight_planner(st_module)
    projection = planner.reference.project(np.array([0.0, 0.0]))

    mode, offset, points, clearance, candidate_count, free_count = (
        planner._choose_path(projection, [])
    )

    lateral_offsets = [planner.reference.project(point).d for point in points]
    assert mode == "pass_through"
    assert math.isclose(offset, 0.0, abs_tol=1e-9)
    assert max(abs(d_value) for d_value in lateral_offsets) < 1e-9
    assert math.isinf(clearance)
    assert candidate_count == free_count

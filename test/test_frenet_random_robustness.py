import csv
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from lqr_sweep.frenet_random_robustness import (
    build_launch_cmd,
    compute_frenet_preset,
    count_laps_from_positions,
    count_laps_from_tracking,
    summarize_launch_log,
    summarize_tracking_log,
)
from pnc_rc.frenet.preset import frenet_static_test_launch_args


def test_compute_frenet_preset_scales_high_speed_parameters():
    preset = compute_frenet_preset(3.0, 1.2)

    assert math.isclose(preset.geometry_target_speed, 3.0)
    assert math.isclose(preset.v_min, 0.84)
    assert preset.grid_forward_m == 20.0
    assert preset.activation_max_m > 8.0
    assert math.isclose(preset.approach_extra_m, 0.75)
    assert math.isclose(preset.activation_path_margin_m, 1.25)
    assert math.isclose(preset.centerline_return_lookahead_m, 3.5)
    assert preset.reuse_timeout_s == 2.0
    assert math.isclose(preset.hold_min_remaining_m, 2.10)
    assert math.isclose(preset.max_published_path_length_m, 6.6)


def test_compute_frenet_preset_shortens_low_speed_centerline_return():
    preset = compute_frenet_preset(0.5, 0.5)

    assert math.isclose(preset.centerline_return_lookahead_m, 2.0)


def test_build_launch_cmd_can_enable_rviz(tmp_path):
    preset = compute_frenet_preset(3.0, 1.2)

    cmd = build_launch_cmd(
        repo_root=tmp_path,
        map_prefix=tmp_path / "map" / "seed_000",
        track_csv=tmp_path / "track.csv",
        trajectory_csv=tmp_path / "trajectory.csv",
        tracking_log=tmp_path / "tracking.csv",
        target_speed=3.0,
        avoidance_speed=1.2,
        start_x=3.0,
        start_y=5.0,
        start_theta=3.1416,
        preset=preset,
        enable_rviz=True,
    )

    assert "enable_rviz:=true" in cmd
    assert "enable_rviz:=false" not in cmd
    assert all(arg in cmd for arg in frenet_static_test_launch_args(preset, 1.2))
    assert "frenet_centerline_return_lookahead_m:=3.500" in cmd
    assert "frenet_max_published_path_length_m:=6.600" in cmd
    assert "frenet_candidate_profile_max_jump_m:=0.35" in cmd


def test_count_laps_from_tracking_uses_index_wraps_near_start():
    rows = []
    t = 0.0
    for _ in range(4):
        for idx in range(200):
            rows.append(
                {
                    "time": f"{t:.2f}",
                    "closest_idx": str(idx),
                    "x": "3.0" if idx < 5 else "0.0",
                    "y": "5.0" if idx < 5 else "0.0",
                }
            )
            t += 0.1

    assert count_laps_from_tracking(rows, 3.0, 5.0) == 3


def test_count_laps_from_positions_uses_global_reference_not_local_path_index():
    reference = []
    for idx in range(100):
        angle = 2.0 * math.pi * idx / 100.0
        reference.append((math.cos(angle), math.sin(angle)))

    rows = []
    for idx in range(260):
        ref_idx = (75 + idx) % 100
        x, y = reference[ref_idx]
        rows.append({"x": str(x), "y": str(y)})

    assert count_laps_from_positions(rows, reference) == 2


def test_summarize_launch_log_counts_failure_markers(tmp_path):
    log_path = tmp_path / "launch.log"
    log_path.write_text(
        "\n".join(
            [
                "Ego collision detected at (1.0, 2.0)",
                "No safe Frenet candidate; publishing stop path",
                "Local trajectory speed limit set to 0.00m/s",
            ]
        ),
        encoding="utf-8",
    )

    summary = summarize_launch_log(log_path)

    assert summary.ego_collision_count == 1
    assert summary.no_safe_stop_count == 1
    assert summary.zero_limit_log_count == 1


def test_summarize_tracking_log_detects_stuck_and_negative_velocity(tmp_path):
    log_path = tmp_path / "tracking.csv"
    fieldnames = [
        "time",
        "closest_idx",
        "x",
        "y",
        "v_actual",
        "v_cmd",
        "v_path_signed",
        "local_speed_limit",
    ]
    rows = []
    for idx in range(620):
        t = idx * 0.1
        rows.append(
            {
                "time": f"{t:.2f}",
                "closest_idx": str(idx % 200),
                "x": "3.0" if idx % 200 < 5 else "0.0",
                "y": "5.0" if idx % 200 < 5 else "0.0",
                "v_actual": "0.0" if 40 <= idx <= 60 else "1.0",
                "v_cmd": "0.0" if 40 <= idx <= 60 else "1.0",
                "v_path_signed": "-0.1" if idx == 100 else "1.0",
                "local_speed_limit": "0.0" if 20 <= idx <= 30 else "1.2",
            }
        )
    with log_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    summary = summarize_tracking_log(log_path, 3.0, 5.0)

    assert summary.completed_laps == 3
    assert summary.zero_limit_runs == 1
    assert summary.low_speed_stuck_runs == 1
    assert summary.max_zero_limit_duration_s > 0.9
    assert summary.max_low_speed_duration_s > 1.9
    assert summary.negative_v_path_count == 1

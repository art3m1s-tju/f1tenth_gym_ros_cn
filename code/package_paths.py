#!/usr/bin/env python3
"""Simplified path helpers for the simulator deployment."""
from __future__ import annotations

from pathlib import Path

SOURCE_ROOT = Path(__file__).resolve().parent
SIM_WS_ROOT = SOURCE_ROOT.parent


def get_package_share_dir() -> Path:
    return SOURCE_ROOT


def get_default_map_yaml() -> Path:
    return SIM_WS_ROOT / "maps" / "my_map.yaml"


def get_default_trajectory_csv() -> Path:
    return get_default_output_root() / "csv" / "processed_track.csv"


def get_default_output_root() -> Path:
    output = SOURCE_ROOT / "outputs"
    output.mkdir(parents=True, exist_ok=True)
    return output


def get_workspace_root() -> Path:
    return SIM_WS_ROOT

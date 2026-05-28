import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from pnc_rc.frenet.node import _occupancy_grid_data_to_map_occupied
from pnc_rc.frenet.planner import (
    LocalGridConfig,
    build_occupancy_grid,
    local_static_map_occupancy,
)


def test_map_data_conversion_preserves_static_obstacle_position():
    map_resolution = 0.1
    map_origin_xy = (-1.0, -1.0)
    height, width = 20, 30
    obstacle_xy = (0.8, -0.6)
    obstacle_col = int(np.floor((obstacle_xy[0] - map_origin_xy[0]) / map_resolution))
    obstacle_row_from_bottom = int(
        np.floor((obstacle_xy[1] - map_origin_xy[1]) / map_resolution)
    )
    obstacle_row = height - 1 - obstacle_row_from_bottom

    occupied_image_rows = np.zeros((height, width), dtype=bool)
    occupied_image_rows[obstacle_row, obstacle_col] = True
    map_data = np.flipud(np.where(occupied_image_rows, 100, 0)).reshape(-1).tolist()
    map_occupied = _occupancy_grid_data_to_map_occupied(map_data, height, width)

    grid_config = LocalGridConfig(
        forward_m=1.5,
        rear_m=0.2,
        half_width_m=0.8,
        resolution_m=0.05,
        inflation_radius_m=0.0,
        scan_offset_x_m=0.0,
    )
    vehicle_pose = (0.0, -0.6, 0.0)
    static_occupied = local_static_map_occupancy(
        map_occupied,
        map_resolution,
        map_origin_xy,
        vehicle_pose,
        grid_config,
    )
    occupancy = build_occupancy_grid(
        np.array([], dtype=float),
        angle_min=0.0,
        angle_increment=1.0,
        range_min=0.0,
        range_max=10.0,
        config=grid_config,
        static_occupied=static_occupied,
    )

    centerline_path = np.array([[0.0, -0.6], [1.2, -0.6]], dtype=float)
    collision, _ = occupancy.query_path(
        centerline_path,
        vehicle_pose,
        path_sample_step_m=0.05,
    )

    assert collision

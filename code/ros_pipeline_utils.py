#!/usr/bin/env python3
"""Shared helpers for the ROS 2 planning/control pipeline."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from nav_msgs.msg import OccupancyGrid
from PIL import Image

import clean_map
from package_paths import get_default_map_yaml


DEFAULT_MAP_YAML = get_default_map_yaml()


def load_map_from_yaml(yaml_path: Path) -> tuple[np.ndarray, dict[str, object], Path]:
    """Load a ROS map image and its lightweight YAML metadata."""
    yaml_path = yaml_path.resolve()
    yaml_data = clean_map.parse_simple_yaml(yaml_path)
    image_path = (yaml_path.parent / str(yaml_data["image"])).resolve()
    image = np.array(Image.open(image_path), dtype=np.uint8)
    return image, yaml_data, image_path


def metadata_to_json(metadata: dict[str, object]) -> str:
    """Serialize pipeline metadata for std_msgs/String transport."""
    return json.dumps(metadata, ensure_ascii=False, sort_keys=True)


def metadata_from_json(text: str) -> dict[str, object]:
    """Parse pipeline metadata transported in std_msgs/String."""
    if not text:
        return {}
    return json.loads(text)


def image_to_ros_occupancy(image: np.ndarray) -> np.ndarray:
    """Convert a trinary map image into standard OccupancyGrid values."""
    occupancy = np.full(image.shape, -1, dtype=np.int8)
    occupancy[image <= 50] = 100
    occupancy[image >= 250] = 0
    return occupancy


def ros_occupancy_to_image(occupancy: np.ndarray) -> np.ndarray:
    """Convert OccupancyGrid values back into a trinary map image."""
    image = np.full(occupancy.shape, 205, dtype=np.uint8)
    image[occupancy >= 50] = 0
    image[occupancy == 0] = 254
    return image


def occupancy_grid_from_image(
    image: np.ndarray,
    metadata: dict[str, object],
    frame_id: str,
) -> OccupancyGrid:
    """Build an OccupancyGrid from a map image and YAML-style metadata."""
    occupancy = image_to_ros_occupancy(image)
    height, width = occupancy.shape

    msg = OccupancyGrid()
    msg.header.frame_id = frame_id
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = float(metadata["resolution"])
    origin = list(metadata.get("origin", [0.0, 0.0, 0.0]))
    msg.info.origin.position.x = float(origin[0])
    msg.info.origin.position.y = float(origin[1])
    msg.info.origin.position.z = 0.0
    yaw = float(origin[2]) if len(origin) >= 3 else 0.0
    msg.info.origin.orientation.z = np.sin(yaw * 0.5)
    msg.info.origin.orientation.w = np.cos(yaw * 0.5)

    # OccupancyGrid is stored from bottom row to top row.
    msg.data = np.flipud(occupancy).reshape(-1).astype(int).tolist()
    return msg


def image_from_occupancy_grid(msg: OccupancyGrid) -> np.ndarray:
    """Recover the image-style map from an OccupancyGrid."""
    occupancy = np.asarray(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
    occupancy = np.flipud(occupancy)
    return ros_occupancy_to_image(occupancy)


def save_map_to_files(
    image: np.ndarray,
    metadata: dict[str, object],
    output_prefix: Path,
) -> tuple[Path, Path]:
    """Save a trinary map image and its YAML metadata to disk."""
    output_prefix = output_prefix.resolve()
    output_prefix.parent.mkdir(parents=True, exist_ok=True)

    output_image = output_prefix.with_suffix(".pgm")
    output_yaml = output_prefix.with_suffix(".yaml")
    Image.fromarray(image, mode="L").save(output_image)

    yaml_data = dict(metadata)
    yaml_data["image"] = output_image.name
    clean_map.dump_simple_yaml(output_yaml, yaml_data)
    return output_image, output_yaml


def occupancy_metadata_from_message(
    msg: OccupancyGrid,
    fallback: dict[str, object] | None = None,
) -> dict[str, object]:
    """Build YAML-style metadata from an OccupancyGrid, optionally preserving extras."""
    metadata = dict(fallback or {})
    fallback_origin = list(metadata.get("origin", [0.0, 0.0, 0.0]))
    yaw = float(fallback_origin[2]) if len(fallback_origin) >= 3 else 0.0
    metadata["resolution"] = float(msg.info.resolution)
    metadata["origin"] = [
        float(msg.info.origin.position.x),
        float(msg.info.origin.position.y),
        yaw,
    ]
    metadata.setdefault("mode", "trinary")
    metadata.setdefault("negate", 0)
    metadata.setdefault("occupied_thresh", 0.65)
    metadata.setdefault("free_thresh", 0.25)
    return metadata

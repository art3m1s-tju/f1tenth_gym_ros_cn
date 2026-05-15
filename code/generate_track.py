#!/usr/bin/env python3
"""Offline track boundary extraction for the simulator map.

Usage (from inside Docker container):
    python3 /sim_ws/src/f1tenth_gym_ros/code/generate_track.py

Outputs:
    code/outputs/csv/processed_track.csv
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
from PIL import Image

import clean_map
import export_csv
from package_paths import get_default_map_yaml, get_default_output_root


def main():
    yaml_path = get_default_map_yaml()
    output_root = get_default_output_root()
    output_csv = output_root / "csv" / "processed_track.csv"
    output_map_prefix = output_root / "maps" / "cleaned_map"

    print(f"Input map YAML: {yaml_path}")
    yaml_data = clean_map.parse_simple_yaml(yaml_path)
    resolution = float(yaml_data["resolution"])
    origin = list(yaml_data["origin"])
    image_path = (yaml_path.parent / str(yaml_data["image"])).resolve()
    image = np.array(Image.open(image_path), dtype=np.uint8)

    print(f"Image: {image_path} ({image.shape[1]}x{image.shape[0]})")
    print(f"Resolution: {resolution} m/px, Origin: {origin}")

    print("\n--- Step 1: Cleaning map ---")
    simulator_map, track_mask, _, stats, boundary_groups = clean_map.clean_map(
        image=image,
        resolution=resolution,
    )
    inner_fill, outer_fill = boundary_groups["processed"]
    if inner_fill is None or outer_fill is None:
        print("ERROR: Could not extract track boundaries from this map.")
        print("Stats:", stats)
        sys.exit(1)

    for k, v in stats.items():
        print(f"  {k} = {v}")

    print("\n--- Step 2: Exporting track CSV ---")
    export_info = export_csv.export_track_csv_from_cleaned_map(
        cleaned_image=simulator_map,
        resolution=resolution,
        origin=origin,
        output_csv=output_csv,
        output_map_prefix=output_map_prefix,
        map_yaml_template=yaml_data,
        samples=400,
        track_mask=track_mask,
        inner_fill=inner_fill,
        outer_fill=outer_fill,
    )

    print(f"\n--- Done ---")
    print(f"  CSV: {export_info['output_csv']}")
    print(f"  Samples: {export_info['samples']}")
    print(f"  Lap length: {export_info['lap_length_m']:.2f} m")

    # Print first trajectory point for initial pose reference
    import pandas as pd
    df = pd.read_csv(output_csv)
    cx = 0.5 * (df['left_border_x'].iloc[0] + df['right_border_x'].iloc[0])
    cy = 0.5 * (df['left_border_y'].iloc[0] + df['right_border_y'].iloc[0])
    cx2 = 0.5 * (df['left_border_x'].iloc[1] + df['right_border_x'].iloc[1])
    cy2 = 0.5 * (df['left_border_y'].iloc[1] + df['right_border_y'].iloc[1])
    import math
    theta = math.atan2(cy2 - cy, cx2 - cx)
    print(f"\n  Suggested initial pose for sim.yaml:")
    print(f"    sx: {cx:.4f}")
    print(f"    sy: {cy:.4f}")
    print(f"    stheta: {theta:.4f}")


if __name__ == "__main__":
    main()

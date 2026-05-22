from __future__ import annotations

import math

import numpy as np
from scipy.spatial import KDTree

from pnc_rc.lqr.geometry import advance_projection_along_path, project_to_path
from pnc_rc.lqr.math import compute_path_curvatures, compute_path_headings


def test_advance_projection_along_path_uses_forward_preview_point() -> None:
    points = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]], dtype=float)
    headings = compute_path_headings(points, closed_loop=False)
    curvatures = compute_path_curvatures(points, closed_loop=False)
    segment_lengths = np.array([1.0, 1.0], dtype=float)
    position = np.array([0.25, 0.2])
    projection = project_to_path(
        position=position,
        points=points,
        kdtree=KDTree(points),
        headings=headings,
        curvatures=curvatures,
        closed_loop=False,
    )

    preview = advance_projection_along_path(
        position=position,
        projection=projection,
        lookahead_distance=1.5,
        points=points,
        headings=headings,
        curvatures=curvatures,
        segment_lengths=segment_lengths,
        closed_loop=False,
    )

    assert np.allclose(preview.point, [1.75, 0.0])
    assert math.isclose(preview.lateral_error, 0.2)
    assert preview.segment_idx == 1
    assert math.isclose(preview.segment_t, 0.75)

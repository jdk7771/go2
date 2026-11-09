"""Shared body geometry and plotting helpers."""

import numpy as np
from .. import core
from ..settings import TARGET_CLEARANCE

BOX_EDGES = [
    (0, 1),
    (0, 2),
    (0, 4),
    (1, 3),
    (1, 5),
    (2, 3),
    (2, 6),
    (3, 7),
    (4, 5),
    (4, 6),
    (5, 7),
    (6, 7),
]

FACES = (
    [0, 1, 3, 2],
    [4, 5, 7, 6],
    [0, 1, 5, 4],
    [2, 3, 7, 6],
    [0, 2, 6, 4],
    [1, 3, 7, 5],
)


def obb_corners(pose: np.ndarray, half: np.ndarray) -> np.ndarray:
    local = np.array(
        [
            [sx * half[0], sy * half[1], sz * half[2]]
            for sx in (-1, 1)
            for sy in (-1, 1)
            for sz in (-1, 1)
        ]
    )
    return pose[:3] + local @ core.rpy_matrix(pose[3:]).T


def segments(poses: np.ndarray, half: np.ndarray) -> list:
    result = []
    for pose in poses:
        corners = obb_corners(pose, half)
        result.extend(corners[[i, j]] for i, j in BOX_EDGES)
    return result


def clearance_color(distance: float) -> str:
    if distance < 0:
        return "#dc2626"
    return "#d97706" if distance < TARGET_CLEARANCE else "#15803d"

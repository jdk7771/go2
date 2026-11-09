"""Body-center range queries and first-hit angular bins for collision input.

Only original point indices are returned: coordinates, area weights and the
point-to-OBB wrench law stay unchanged. Bins are fixed in world axes and cover
the whole sphere. Each bin approximates a finite-width ray; it is not a mesh
raycast. First-hit associations are recomputed at every objective evaluation.
"""

from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class PointSelection:
    method: str = "all"
    max_length: float = 0.6
    angle_deg: float = 4.0

    def __post_init__(self):
        if self.method not in ("all", "radius", "rays"):
            raise ValueError(f"Unknown point selection: {self.method}")
        if not math.isfinite(self.max_length) or self.max_length <= 0:
            raise ValueError("max_length must be finite and positive")
        if not math.isfinite(self.angle_deg) or not 0 < self.angle_deg <= 90:
            raise ValueError("angle_deg must be in (0, 90]")


class BodyCenterSelector:
    def __init__(self, points: np.ndarray, config: PointSelection):
        self.points = points
        self.config = config
        started = time.perf_counter()
        self.tree = cKDTree(points) if config.method != "all" else None
        self.setup_seconds = time.perf_counter() - started
        self.azimuth_bins = math.ceil(360 / config.angle_deg)
        self.elevation_bins = math.ceil(180 / config.angle_deg)
        self.calls = self.pose_queries = self.candidate_count = self.selected_count = 0
        self.query_seconds = 0.0

    def select(self, centers: np.ndarray) -> list[np.ndarray] | None:
        if self.tree is None:
            return None  # Original full-cloud path, with no indexing copies.
        started = time.perf_counter()
        neighbors = self.tree.query_ball_point(
            centers, self.config.max_length, return_sorted=False, workers=1
        )
        selections = []
        candidates = selected = 0
        for center, neighbor in zip(centers, neighbors):
            indices = np.asarray(neighbor, dtype=np.intp)
            candidates += len(indices)
            if self.config.method == "rays" and len(indices):
                delta = self.points[indices] - center
                radius_sq = np.einsum("ij,ij->i", delta, delta)
                azimuth = np.arctan2(delta[:, 1], delta[:, 0]) + np.pi
                elevation = (
                    np.arctan2(delta[:, 2], np.hypot(delta[:, 0], delta[:, 1]))
                    + np.pi / 2
                )
                az = (azimuth * (self.azimuth_bins / (2 * np.pi))).astype(int)
                az %= self.azimuth_bins
                el = (elevation * (self.elevation_bins / np.pi)).astype(int)
                el = np.minimum(el, self.elevation_bins - 1)
                bins = el * self.azimuth_bins + az
                # Primary key bin, then range, then original ID for exact ties.
                order = np.lexsort((indices, radius_sq, bins))
                ordered_bins = bins[order]
                first = np.r_[True, ordered_bins[1:] != ordered_bins[:-1]]
                indices = indices[order[first]]
            indices.sort()  # Preserve original summation order where possible.
            selections.append(indices)
            selected += len(indices)
        self.query_seconds += time.perf_counter() - started
        self.calls += 1
        self.pose_queries += len(centers)
        self.candidate_count += candidates
        self.selected_count += selected
        return selections

    def statistics(self) -> dict:
        queries = max(1, self.pose_queries)
        return {
            "selection_setup_seconds": self.setup_seconds,
            "selection_query_seconds": self.query_seconds,
            "selection_evaluations": self.calls,
            "mean_range_candidates_per_pose": self.candidate_count / queries,
            "mean_selected_points_per_pose": (
                len(self.points)
                if self.config.method == "all"
                else self.selected_count / queries
            ),
            "planning_cloud_points": len(self.points),
        }

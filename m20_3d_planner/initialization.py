"""Choose a smooth, nonstraight starting band using only the planning cloud.

This finite candidate search supplies a route reference, not a collision
certificate. Newton--Euler relaxation and held-out validation follow it.
"""

import time
import numpy as np
from scipy.spatial import cKDTree

from . import core
from .settings import POINT_VOXEL


def small_bend_initialization(reference):
    """An unoptimized 25 mm S-bend: no cloud scoring or obstacle avoidance."""
    poses = np.asarray(reference, dtype=float).copy()
    progress = np.linspace(0., 1., len(poses))
    poses[:, 1] += .025 * np.sin(2 * np.pi * progress) * np.sin(np.pi * progress)
    poses[[0, -1]] = reference[[0, -1]]
    return poses


def cloud_bow_initialization(cloud, reference, half, lateral_limit, clearance, allow_roll=True):
    started = time.perf_counter()
    cloud = np.asarray(cloud, dtype=float)
    reference = np.asarray(reference, dtype=float)
    half = np.asarray(half, dtype=float)
    if (reference.ndim != 2 or reference.shape[1] != 6 or len(reference) < 3
            or cloud.ndim != 2 or cloud.shape[1] != 3 or half.shape != (3,)
            or not all(np.isfinite(a).all() for a in (cloud, reference, half))
            or np.any(half <= 0) or not np.isfinite(lateral_limit)
            or lateral_limit <= 0 or not np.isfinite(clearance) or clearance <= 0):
        raise ValueError("Invalid point cloud, reference, geometry or initialization limits")
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(reference[:, :3], axis=0), axis=1))]
    if arc[-1] <= 0:
        raise ValueError("Initialization needs distinct trajectory endpoints")
    progress = arc / arc[-1]
    ramp = np.clip(np.minimum(progress, 1 - progress) / .17, 0, 1)
    # A broad central plateau lets the WHOLE finite body enter and leave a
    # constriction before the bend/height/roll returns to the endpoint pose.
    # Quintic ramps have zero first and second derivatives at both ends.
    bump = ramp ** 3 * (10 - 15 * ramp + 6 * ramp ** 2)
    bump[[0, -1]] = 0.
    # Use both sides with the same amplitudes; never infer a direction from a
    # scene ID. Include broad bends because millimetre noise cannot route round
    # an obstacle containing the original band.
    amplitudes = sorted(set(min(v, .95 * lateral_limit) for v in (.02, .06, .12, .22, .34, .46, .56)))
    offsets = [0.] + [sign * a for a in amplitudes for sign in (-1, 1)]
    heights = [0., -.04, -.08, -.12, .06, .12]
    tree = cKDTree(cloud)
    radius = np.linalg.norm(half) + clearance
    base_length = arc[-1]
    best = None
    records = []
    position_candidates = []

    def evaluate(poses, lateral, vertical, roll=0.):
        nonlocal best
        dense = core.densify_trajectory(poses, 4, "so3")
        selections = tree.query_ball_point(dense[:, :3], radius, workers=1)
        hinge_square = 0.
        minimum = np.inf
        for pose, indices in zip(dense, selections):
            if not indices:
                continue
            distance = core.obb_signed_distance(
                cloud[indices], pose[:3], core.rpy_matrix(pose[3:]), half
            )[0]
            minimum = min(minimum, float(distance.min()))
            hinge_square += float(np.sum(np.maximum(0., clearance - distance) ** 2))
        contact = .5 * 1.6e6 * POINT_VOXEL ** 2 * hinge_square / len(dense)
        offset = np.mean(np.sum((poses[:, :3] - reference[:, :3]) ** 2, axis=1))
        length = np.linalg.norm(np.diff(poses[:, :3], axis=0), axis=1).sum()
        effort = 10 * (.5 * offset + .1 * max(0., length - base_length)
                       + .02 * np.mean((poses[:, 3] - reference[:, 3]) ** 2))
        score = contact + effort
        entry = dict(lateral_amplitude_m=lateral, vertical_amplitude_m=vertical,
                     roll_amplitude_deg=roll, score=float(score), contact_score=float(contact),
                     shape_score=float(effort),
                     sampled_cloud_minimum_m=float(minimum) if np.isfinite(minimum) else None)
        records.append(entry)
        if best is None or score < best[0]:
            best = (score, poses.copy(), len(records) - 1)
        return score

    for lateral in offsets:
        for vertical in heights:
            if lateral == 0 and vertical == 0:
                continue
            poses = reference.copy()
            poses[:, 1] += lateral * bump
            poses[:, 2] += vertical * bump
            poses[[0, -1]] = reference[[0, -1]]
            if (np.abs(poses[1:-1, 1]).max() > lateral_limit + 1e-12
                    or poses[1:-1, 2].min() < .28 - 1e-12
                    or poses[1:-1, 2].max() > .52 + 1e-12):
                continue
            # Score between nodes, on all nearby original points rather than
            # first ray hits. This stage must not hide a surface behind another.
            score = evaluate(poses, lateral, vertical)
            position_candidates.append((score, poses, lateral, vertical))
    if allow_roll:
        # Refine good bends and small centred bends with symmetric roll guesses.
        # Always retain centred candidates: an unrolled body may intersect both
        # sides of a narrow corridor even when a rolled body fits there.
        shortlist = sorted(range(len(position_candidates)), key=lambda i: position_candidates[i][0])[:12]
        shortlist = sorted(set(shortlist + [i for i, c in enumerate(position_candidates)
                                           if abs(c[2]) <= amplitudes[0] + 1e-12]))
        rotation_bump = bump
        for index in shortlist:
            _, poses, lateral, vertical = position_candidates[index]
            for roll in (-40., -30., -15., 15., 30., 40.):
                tilted = poses.copy()
                tilted[:, 3] += np.radians(roll) * rotation_bump
                tilted[[0, -1]] = reference[[0, -1]]
                if np.abs(tilted[:, 3]).max() <= np.pi / 4 + 1e-12:
                    evaluate(tilted, lateral, vertical, roll)
    if best is None:
        raise ValueError("No smooth bow fits the trajectory bounds")
    return best[1], dict(method="cloud_bow", selected_candidate=best[2],
                         selected=records[best[2]], candidates=records,
                         candidate_count=len(records), subdivisions=4,
                         clearance_m=clearance,
                         score_weights=dict(contact_stiffness=1.6e6, shape=10.,
                                            mean_position_squared=.5, extra_length=.1,
                                            mean_roll_squared=.02),
                         plateau_ramp_fraction=.17, allow_roll=allow_roll,
                         seconds=time.perf_counter() - started,
                         reference_role="selected bow anchors XY and attitude restoration",
                         truth_used=False, scene_metadata_used=False)

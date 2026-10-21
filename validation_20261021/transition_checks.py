"""Audit continuity of prior paths under explicit reduced contact primitives.

This is a kinematic/contact audit, not a robot controller or gait planner.
For every saved body path, re-create the previous greedy four-wheel contacts.
Test whether each next body node can be reached while keeping all four wheel
contacts fixed in the world. This primitive permits leg articulation but no
wheel rolling. Separately test whether the greedy wheel-center displacement
fits fixed-heading, x-direction pure rolling on a horizontal plane.
"""
from __future__ import annotations

import itertools
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

from run_checks import HALF, HIPS, OFFSETS, box_hit, sphere_box_clearance

OUT = Path(__file__).resolve().parent
DATA = json.loads((OUT / "results.json").read_text())


def support_margin(body_xy, contacts):
    """Signed distance to convex-hull sides, positive inside."""
    hull = ConvexHull(contacts[:, :2])
    normals = hull.equations[:, :2]
    signed = hull.equations[:, :2] @ body_xy + hull.equations[:, 2]
    return float(np.min(-signed / np.linalg.norm(normals, axis=1)))


def greedy_feet(body, scene):
    kind = scene["kind"]
    height = .15 if kind == "step" else 0.
    gap_x, gap_end, gap_y = scene["gap"]
    obstacles = [np.array(b) for b in scene["obstacles"]]
    feet = []
    for hip in body + HIPS:
        xy = hip[:2] + OFFSETS
        ground = np.where(xy[:, 0] >= 1.4, height, 0.)
        centers = np.column_stack((xy, ground + .06))
        reach = np.linalg.norm(centers - hip, axis=1)
        good = (reach >= .18) & (reach <= .48)
        if "gap" in kind:
            good &= ~((xy[:, 0] > gap_x-.03) & (xy[:, 0] < gap_end+.03)
                       & (np.abs(xy[:, 1]) < gap_y+.03))
        for obstacle in obstacles:
            good &= sphere_box_clearance(centers, obstacle, .06) >= -1e-9
        if not good.any():
            return None
        feet.append(centers[np.flatnonzero(good)[0]])
    feet = np.array(feet)
    return feet if support_margin(body[:2], feet) >= .01-1e-9 else None


def hold_stance_over_edge(start, end, feet, obstacles):
    """Sufficient checks for this reduced locked-wheel, quasi-static edge."""
    failures = set()
    min_reach_margin = float("inf")
    min_support_margin = float("inf")
    for alpha in np.linspace(0., 1., 21):
        body = (1-alpha)*start + alpha*end
        hip = body[None, :] + HIPS
        reach = np.linalg.norm(feet-hip, axis=1)
        min_reach_margin = min(min_reach_margin,
                               float(np.min(np.minimum(reach-.18, .48-reach))))
        min_support_margin = min(min_support_margin, support_margin(body[:2], feet))
        if np.any(reach < .18-1e-8) or np.any(reach > .48+1e-8):
            failures.add("reach")
        if support_margin(body[:2], feet) < .01-1e-8:
            failures.add("support")
        if any(box_hit(body, HALF, obstacle, .005) for obstacle in obstacles):
            failures.add("body_collision")
    return {
        "passes": not failures,
        "failures": sorted(failures),
        "min_reach_margin_m": min_reach_margin,
        "min_support_margin_m": min_support_margin,
    }


def analyze(scene):
    path = scene["methods"]["body_with_node_support"]["points"]
    body = np.asarray(path)
    feet = np.array([greedy_feet(p, scene) for p in body])
    obstacles = [np.array(b) for b in scene["obstacles"]]
    edges = []
    for i, (a, b) in enumerate(zip(body[:-1], body[1:])):
        delta = feet[i+1]-feet[i]
        changed = np.linalg.norm(delta, axis=1) > 1e-7
        side = np.max(np.abs(delta[:, 1]))
        vertical = np.max(np.abs(delta[:, 2]))
        locked = hold_stance_over_edge(a, b, feet[i], obstacles)
        edges.append({
            "index": i,
            "body_delta_m": (b-a).tolist(),
            "greedy_contacts_changed": int(changed.sum()),
            # A fixed heading, unsteered wheel may only roll in x on flat ground.
            "greedy_rolling_x_only_compatible": bool(side < 1e-7 and vertical < 1e-7),
            "locked_stance": locked,
        })
    return {"kind": scene["kind"], "ceiling": scene["ceiling"],
            "gap": scene["gap"], "edges": edges}


def main():
    runs = [analyze(scene) for scene in DATA["runs"]]
    summary = []
    for kind in ("flat", "table", "gap", "table_gap", "step"):
        group = [run for run in runs if run["kind"] == kind]
        edges = list(itertools.chain.from_iterable(run["edges"] for run in group))
        reasons = defaultdict(int)
        for edge in edges:
            for failure in edge["locked_stance"]["failures"]:
                reasons[failure] += 1
        summary.append({"kind": kind, "scenes": len(group), "path_edges": len(edges),
            "all_greedy_contacts_changed": sum(e["greedy_contacts_changed"] == 4 for e in edges),
            "greedy_x_rolling_compatible": sum(e["greedy_rolling_x_only_compatible"] for e in edges),
            "locked_stance_edges_passing": sum(e["locked_stance"]["passes"] for e in edges),
            "locked_stance_failure_reasons": dict(reasons)})
    result = {
        "scope": "Reduced-model audit of saved body paths; no foothold replanning, dynamic forces, joint IK, RL, or robot execution.",
        "primitives": {
            "greedy_reference": "Recreates one selected contact per leg at each body node; changes are not assigned a gait.",
            "fixed_heading_roll": "Four unsteered wheels at yaw=0, level ground, pure x rolling; a y or z wheel-center displacement fails this narrow primitive.",
            "locked_stance": "Four previous contacts fixed in world, 21 interpolated body states, spherical leg reach, 1 cm static support margin, fixed-orientation body obstacle sweep.",
        },
        "summary": summary,
        "runs": runs,
    }
    (OUT / "transition_results.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

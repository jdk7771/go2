"""Reproducible counterexamples and reduced-model planning checks, not locomotion.

No robot URDF, learned controller, joint limits or dynamic integration is used.
Body search: translation-only box; four independently selected stationary contacts
with a spherical leg workspace and planar static support. This is an approximate
geometric screen, NOT a necessary/sufficient test of actual robot feasibility.
It ignores joints but also restricts contacts to a finite set and four-leg stance.
"""
from __future__ import annotations

import heapq
import itertools
import json
import time
from functools import lru_cache
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull

OUT = Path(__file__).resolve().parent
HALF = np.array([0.25, 0.14, 0.08])
HIPS = np.array([[x, y, 0.] for x in (-0.2, 0.2) for y in (-0.18, 0.18)])
OFFSETS = np.array(list(itertools.product((-.10, 0., .10), repeat=2)))
OFFSETS = OFFSETS[np.argsort(np.linalg.norm(OFFSETS, axis=1), kind="stable")]


def box_hit(center, half, box, margin=0.):
    low, high = np.asarray(box)
    return bool(np.all(center + half >= low - margin) and
                np.all(center - half <= high + margin))


def sphere_box_clearance(points, box, radius):
    low, high = np.asarray(box)
    c, h = (low + high) / 2, (high - low) / 2
    d = np.abs(np.asarray(points) - c) - h
    sdf = np.linalg.norm(np.maximum(d, 0.), axis=-1) + np.minimum(np.max(d, axis=-1), 0.)
    return sdf - radius


def astar(start, goal, neighbors, admissible, metric):
    if not admissible(start) or not admissible(goal):
        return None, 0
    heap, costs, parent = [(metric(start, goal), 0., start)], {start: 0.}, {}
    expanded = 0
    while heap:
        _, cost, node = heapq.heappop(heap)
        if cost > costs[node] + 1e-10:
            continue
        expanded += 1
        if node == goal:
            path = [node]
            while node in parent:
                node = parent[node]
                path.append(node)
            return path[::-1], expanded
        for nxt in neighbors(node):
            if not admissible(nxt):
                continue
            new = cost + metric(node, nxt)
            if new + 1e-10 < costs.get(nxt, float("inf")):
                costs[nxt], parent[nxt] = new, node
                heapq.heappush(heap, (new + metric(nxt, goal), new, nxt))
    return None, expanded


def grid_neighbors(shape):
    def neighbors(node):
        for d in range(len(shape)):
            for sign in (-1, 1):
                nxt = list(node)
                nxt[d] += sign
                if 0 <= nxt[d] < shape[d]:
                    yield tuple(nxt)
    return neighbors


def body_experiment(kind, rng):
    axes = [np.arange(.4, 2.61, .1), np.arange(-.8, .81, .1), np.arange(.3, .651, .05)]
    shape = tuple(map(len, axes))
    def pos(node):
        return np.array([axes[d][node[d]] for d in range(3)])
    ceiling = float(rng.uniform(.46, .52))
    gap_x = float(rng.uniform(1.12, 1.2))
    gap_end = float(rng.uniform(1.55, 1.62))
    gap_y = float(rng.uniform(.36, .44))
    obstacles = []
    if "table" in kind:
        obstacles.append(np.array([[.95, -2., ceiling], [2.0, 2., ceiling+.15]]))
    height = .15 if kind == "step" else 0.
    if height:
        obstacles.append(np.array([[1.4, -2., -.2], [3.5, 2., height]]))

    @lru_cache(None)
    def body_ok(node):
        p = pos(node)
        return all(not box_hit(p, HALF, box, .005) for box in obstacles)

    @lru_cache(None)
    def contacts(node):
        p = pos(node)
        feet = []
        for hip in p + HIPS:
            xy = hip[:2] + OFFSETS
            ground = np.where(xy[:, 0] >= 1.4, height, 0.)
            centers = np.column_stack((xy, ground + .06))
            reach = np.linalg.norm(centers - hip, axis=1)
            good = (reach >= .18) & (reach <= .48)
            if "gap" in kind:
                # Erode supporting surface by a 3 cm contact-placement margin.
                bad = ((xy[:, 0] > gap_x-.03) & (xy[:, 0] < gap_end+.03)
                       & (np.abs(xy[:, 1]) < gap_y+.03))
                good &= ~bad
            for box in obstacles:
                # Intended ground contact at a step top is permitted.
                good &= sphere_box_clearance(centers, box, .06) >= -1e-9
            if not good.any():
                return None
            feet.append(centers[np.flatnonzero(good)[0]])
        feet = np.array(feet)
        hull = ConvexHull(feet[:, :2])
        if np.any(hull.equations[:, :2] @ p[:2] + hull.equations[:, 2] > -.01):
            return None
        return feet

    start = (0, 8, 3)
    goal = (shape[0]-1, 8, 6 if height else 3)
    def metric(a, b):
        return float(np.linalg.norm(pos(a)-pos(b)))
    outputs = {}
    for mode in ("body_only", "body_with_node_support"):
        body_ok.cache_clear()
        contacts.cache_clear()
        def valid(n):
            return body_ok(n) and (mode == "body_only" or contacts(n) is not None)
        t = time.perf_counter()
        path, expanded = astar(start, goal, grid_neighbors(shape), valid, metric)
        elapsed = time.perf_counter()-t
        points = np.array([pos(n) for n in path]) if path else np.empty((0, 3))
        # Independent reporting: does the returned path pass the extra node check?
        failures = sum(contacts(n) is None for n in path) if path else None
        # Axis-aligned grid moves: swept AABB is exact for this fixed-orientation box.
        sweep_ok = path is not None and all(
            not box_hit((a+b)/2, HALF+np.abs(a-b)/2, box, .005)
            for a, b in zip(points[:-1], points[1:]) for box in obstacles)
        outputs[mode] = dict(path_found=path is not None, unsupported_nodes=failures,
            node_screen_pass=path is not None and failures == 0,
            swept_body_clear=bool(sweep_ok), expanded=expanded, seconds=elapsed,
            length=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
            points=points.tolist())
    return dict(kind=kind, ceiling=ceiling, gap=[gap_x, gap_end, gap_y],
                obstacles=[b.tolist() for b in obstacles], methods=outputs)


def contact_force_feasible(slope_deg, mu):
    """Static Newton-Euler equilibrium with a conservative friction pyramid."""
    angle = np.deg2rad(slope_deg)
    n = np.array([-np.sin(angle), 0., np.cos(angle)])
    t1, t2 = np.array([np.cos(angle), 0., np.sin(angle)]), np.array([0., 1., 0.])
    basis = np.column_stack((n, t1, t2))
    com = np.array([0., 0., .5])
    A, inequalities = np.zeros((6, 12)), []
    for i, (x, y) in enumerate(itertools.product((-.3, .3), (-.2, .2))):
        r = np.array([x, y, x*np.tan(angle)]) - com
        skew = np.array([[0., -r[2], r[1]], [r[2], 0., -r[0]], [-r[1], r[0], 0.]])
        A[:3, 3*i:3*i+3], A[3:, 3*i:3*i+3] = basis, skew @ basis
        for sign1, sign2 in itertools.product((-1, 1), repeat=2):
            row = np.zeros(12)
            row[3*i:3*i+3] = [-mu, sign1, sign2]
            inequalities.append(row)
    result = linprog(np.zeros(12), A_ub=inequalities, b_ub=np.zeros(16),
        A_eq=A, b_eq=[0., 0., 30*9.81, 0., 0., 0.],
        bounds=[(0., 400.), (None, None), (None, None)]*4, method="highs")
    return bool(result.success)


def swing_experiment():
    radius = .06
    box = np.array([[-.05, -.12, 0.], [.05, .12, .30]])
    s = np.linspace(0, 1, 301)
    nominal = np.column_stack((-.3+.6*s, 0*s, .06+.18*4*s*(1-s)))
    axes = [np.arange(-.3, .301, .02), np.arange(-.3, .301, .02), np.arange(.06, .501, .02)]
    shape = tuple(map(len, axes))
    def pos(n):
        return np.array([axes[d][n[d]] for d in range(3)])
    def metric(a, b):
        return float(np.linalg.norm(pos(a)-pos(b)))
    # SDF is 1-Lipschitz. > h/2 + eps at all nodes certifies each length-h edge.
    def valid(n):
        # Except for vertical lift/touchdown columns, keep the wheel bottom
        # at least 6 cm above the plane; disallow a rolling detour as a swing.
        lift_or_touchdown = n[:2] in ((0, 15), (30, 15))
        return (n[2] >= 3 or lift_or_touchdown) and sphere_box_clearance(pos(n), box, radius) > .010001
    path, expanded = astar((0, 15, 0), (30, 15, 0), grid_neighbors(shape), valid, metric)
    points = np.array([pos(n) for n in path])
    dense = np.concatenate([np.linspace(a, b, 11) for a, b in zip(points[:-1], points[1:])])
    return dict(endpoint_clear=bool(np.all(sphere_box_clearance(nominal[[0,-1]], box, radius) > 0)),
        nominal_min_clearance=float(sphere_box_clearance(nominal, box, radius).min()),
        searched_min_clearance=float(sphere_box_clearance(dense, box, radius).min()),
        expanded=expanded, searched_mid_swing_min_center_height=.12,
        nominal=nominal.tolist(), searched=points.tolist(), box=box.tolist())


def diagnostics():
    # Body and foot endpoints do not determine link collision.
    hip = np.array([0., 0., .5]); foot = np.array([.15, 0., .06]); length = .27
    delta = foot-hip; d = np.linalg.norm(delta)
    perp = np.array([-delta[2], 0., delta[0]])/d
    knee = (hip+foot)/2 + np.sqrt(length**2-d**2/4)*perp
    alternate = (hip+foot)/2 - np.sqrt(length**2-d**2/4)*perp
    obstruction = [knee-.035, knee+.035]
    knee_case = dict(hip=hip.tolist(), foot=foot.tolist(), knee=knee.tolist(),
        alternate_knee=alternate.tolist(), obstacle=np.asarray(obstruction).tolist(),
        link_length=length, foot_clear=bool(sphere_box_clearance(foot, obstruction, .06)>0),
        torso_clear=not box_hit(np.array([0., 0., .57]), np.array([.25,.14,.07]), obstruction),
        knee_clear=bool(sphere_box_clearance(knee, obstruction, .025)>0))
    # Abstract witness: each layer has a state, but there is no connected path.
    # Edges below are stipulated, not computed from robot geometry or a gait.
    witness = {"s0": ["s1_left"], "s1_left": [], "s1_right": ["s2"], "s2": []}
    reachable = {"s0"}; frontier = ["s0"]
    while frontier:
        for node in witness[frontier.pop()]:
            if node not in reachable:
                reachable.add(node); frontier.append(node)
    return dict(knee=knee_case,
        orientation=dict(corridor_width=.4, body_width=.3, body_length=.6,
            required_width_yaw0=.3,
            required_width_yaw30=float(.6*np.sin(np.pi/6)+.3*np.cos(np.pi/6))),
        friction=[dict(slope_deg=a, mu=mu, geometric_support=True,
                       force_feasible=contact_force_feasible(a, mu),
                       analytic_friction_possible=bool(np.tan(np.deg2rad(a)) <= mu+1e-9))
                  for a in (0, 10, 20, 30) for mu in (.1, .3, .6)],
        disconnected_contacts=dict(all_body_layers_nonempty=True,
            start_to_goal_path_exists="s2" in reachable, witness_graph=witness,
            note="Abstract graph counterexample, not a robot simulation"),
        rolling=dict(reference_center_velocity=[0., .2, 0.], wheel_tangent=[1., 0., 0.],
                     lateral_no_slip_residual_m_s=.2,
                     note="Fixed yaw and fixed relative leg positions; side-stepping is a different motion"))


def main():
    rng = np.random.default_rng(20261021)
    runs = [body_experiment(kind, rng) for kind in ("flat", "table", "gap", "table_gap", "step")
            for _ in range(1 if kind in ("flat", "step") else 12)]
    diag, swing = diagnostics(), swing_experiment()
    summary = []
    for kind in ("flat", "table", "gap", "table_gap", "step"):
        for method in ("body_only", "body_with_node_support"):
            rows = [r["methods"][method] for r in runs if r["kind"] == kind]
            summary.append(dict(kind=kind, method=method, n=len(rows),
                paths=sum(r["path_found"] for r in rows),
                node_screen_pass=sum(r["node_screen_pass"] for r in rows),
                swept_body_clear=sum(r["swept_body_clear"] for r in rows),
                median_ms=round(1000*float(np.median([r["seconds"] for r in rows])), 2)))
    results = dict(scope="Reduced translation-only geometry and static contact tests. No RL or whole-body dynamics.",
        seed=20261021, body_half_extent=HALF.tolist(), wheel_radius=.06,
        leg_length_bounds=[.18,.48], candidate_offsets=OFFSETS.tolist(),
        important_limit="Stationary contact choices may change discontinuously between nodes; no executable gait claimed.",
        summary=summary, runs=runs, diagnostics=diag, swing=swing)
    (OUT/"results.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.6))
    sample = next(r for r in runs if r["kind"] == "table_gap")
    x0, x1, gy = sample["gap"]
    axs[0].add_patch(plt.Rectangle((x0,-gy), x1-x0, 2*gy, color="#ddccbb", label="No support"))
    for method, label in (("body_only", "Body collision only"), ("body_with_node_support", "+ stationary contacts")):
        points = np.array(sample["methods"][method]["points"])
        if len(points):
            axs[0].plot(points[:,0], points[:,1], label=label)
            axs[1].plot(points[:,0], points[:,2], label=label)
    axs[0].set(xlabel="x (m)", ylabel="y (m)", title="A. Table + missing support (top view)")
    axs[0].legend(fontsize=8); axs[0].set_aspect("equal")
    axs[1].add_patch(plt.Rectangle((.95,sample["ceiling"]),1.05,.15,color="gray",label="Overhang"))
    axs[1].set(xlabel="x (m)", ylabel="body center z (m)", title="B. Body center reference")
    axs[1].legend(fontsize=8)
    for key, label in (("nominal", "Fixed swing arc"), ("searched", "3D search")):
        pts=np.array(swing[key]); axs[2].plot(pts[:,0],pts[:,1],label=label)
    axs[2].add_patch(plt.Rectangle((-.05,-.12),.1,.24,color="gray"))
    axs[2].set(xlabel="x (m)", ylabel="y (m)", title="C. Wheel-center swing (top view)")
    axs[2].set_aspect("equal"); axs[2].legend(fontsize=8)
    fig.suptitle("Reduced-model checks only: no executed gait, dynamics or RL")
    fig.tight_layout(); fig.savefig(OUT/"checks.png",dpi=170)
    print(json.dumps({"summary": summary, "swing": {k:v for k,v in swing.items() if k not in ("nominal","searched","box")},
                      "diagnostics": diag},indent=2))


if __name__ == "__main__":
    main()

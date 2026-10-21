"""Minimal continuous-contact graph for a four-legged *toy* stance model.

Fixed-heading planar body translation along x. Four point contacts have fixed
y-coordinates (two rails). An action either moves the body with all previous
contacts locked, or repositions exactly one leg while the other three retain
static support. This tests contact continuity missing from the earlier node
screen; it is NOT a wheeled-quadruped model or physics simulation.
"""
from __future__ import annotations

import heapq
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull, QhullError
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
STEP = .1
START_INDEX = 4
GOAL_INDEX = 26
BODY_Z = .45
FOOT_Z = .06
Y = np.array([-.18, .18, -.18, .18])
HIP_X = np.array([-.2, -.2, .2, .2])
MIN_REACH, MAX_REACH = .18, .48
MIN_SUPPORT_MARGIN = .01
MAX_SWING_DISTANCE = .35


def margin(body_x, footholds, active=(0, 1, 2, 3)):
    pts = np.column_stack((np.array(footholds)[list(active)]*STEP, Y[list(active)]))
    try:
        hull = ConvexHull(pts)
    except QhullError:
        return float("-inf")
    normals = hull.equations[:, :2]
    d = hull.equations[:, :2] @ np.array([body_x, 0.]) + hull.equations[:, 2]
    return float(np.min(-d / np.linalg.norm(normals, axis=1)))


def reachable(body_x, leg, foot_index):
    dx = foot_index*STEP-(body_x+HIP_X[leg])
    distance = math.hypot(dx, BODY_Z-FOOT_Z)
    return MIN_REACH-1e-9 <= distance <= MAX_REACH+1e-9


def ground(foot_index, gap):
    x = foot_index*STEP
    return 0 <= foot_index <= 30 and (gap is None or not gap[0] < x < gap[1])


def valid_stance(body_index, stance, gap):
    body_x = body_index*STEP
    return (all(ground(f, gap) and reachable(body_x, leg, f)
                for leg, f in enumerate(stance))
            and margin(body_x, stance) >= MIN_SUPPORT_MARGIN-1e-9)


def neighbors(state, gap):
    body_index, stance = state
    body_x = body_index*STEP
    if body_index < GOAL_INDEX and valid_stance(body_index+1, stance, gap):
        yield (body_index+1, stance), ("body_advance", None)
    for leg in range(4):
        others = tuple(i for i in range(4) if i != leg)
        if margin(body_x, stance, others) < MIN_SUPPORT_MARGIN-1e-9:
            continue
        hip_x = body_x+HIP_X[leg]
        for target in range(max(0, math.floor((hip_x-.28)/STEP)),
                            min(30, math.ceil((hip_x+.28)/STEP))+1):
            if target == stance[leg] or not ground(target, gap):
                continue
            if abs(target-stance[leg])*STEP > MAX_SWING_DISTANCE+1e-9:
                continue
            if not reachable(body_x, leg, target):
                continue
            target_stance = list(stance)
            target_stance[leg] = target
            target_stance = tuple(target_stance)
            if valid_stance(body_index, target_stance, gap):
                yield (body_index, target_stance), ("single_leg_swing", leg)


def solve(gap, max_expansions=200000):
    start = (START_INDEX, (2, 2, 6, 6))
    assert valid_stance(*start, gap)
    # Unit-cost A*: straight-line body progress is an admissible heuristic.
    queue = [(GOAL_INDEX-START_INDEX, 0, start)]
    cost = {start: 0}
    parents = {}
    expanded = 0
    while queue and expanded < max_expansions:
        _, g, state = heapq.heappop(queue)
        if g != cost[state]:
            continue
        expanded += 1
        if state[0] == GOAL_INDEX:
            actions = []
            trace = [state]
            while state in parents:
                state, action = parents[state]
                actions.append(action)
                trace.append(state)
            trace.reverse(); actions.reverse()
            return {"found": True, "expanded": expanded, "actions": actions,
                    "trace": [[i*STEP, [f*STEP for f in feet]] for i, feet in trace]}
        for next_state, action in neighbors(state, gap):
            new_cost = g+1
            if new_cost < cost.get(next_state, 1 << 30):
                cost[next_state] = new_cost
                parents[next_state] = (state, action)
                priority = new_cost+(GOAL_INDEX-next_state[0])
                heapq.heappush(queue, (priority, new_cost, next_state))
    return {"found": False, "expanded": expanded,
            "search_exhausted": not queue, "expansion_limit_hit": bool(queue),
            "max_reached_body_x": max(s[0] for s in cost)*STEP}


def validate_solution(result, gap):
    if not result["found"]:
        return False
    trace = [(int(round(x/STEP)), tuple(int(round(f/STEP)) for f in feet))
             for x, feet in result["trace"]]
    assert len(trace) == len(result["actions"])+1
    for before, after, (action, leg) in zip(trace[:-1], trace[1:], result["actions"]):
        assert valid_stance(*before, gap)
        assert valid_stance(*after, gap)
        if action == "body_advance":
            assert after[0] == before[0]+1 and after[1] == before[1]
            for alpha in np.linspace(0., 1., 11):
                body_x = (before[0]+alpha)*STEP
                assert margin(body_x, before[1]) >= MIN_SUPPORT_MARGIN-1e-9
                assert all(reachable(body_x, i, f) for i, f in enumerate(before[1]))
        else:
            assert action == "single_leg_swing" and before[0] == after[0]
            assert sum(a != b for a, b in zip(before[1], after[1])) == 1
            assert margin(before[0]*STEP, before[1],
                          tuple(i for i in range(4) if i != leg)) >= MIN_SUPPORT_MARGIN-1e-9
            assert abs(before[1][leg]-after[1][leg])*STEP <= MAX_SWING_DISTANCE+1e-9
    return True


def main():
    cases = {"continuous_ground": None, "full_width_gap": (1.15, 1.65)}
    output = {"scope": "Four point-foot toy, fixed two-rail foot y, locked stance body advance or one-leg swing; no joint IK, collision, forces, wheels or RL.",
              "parameters": {"start_body_x": .4, "goal_body_x": 2.6,
                             "gap_ground_x": [1.15, 1.65], "max_swing_distance": MAX_SWING_DISTANCE,
                             "support_margin": MIN_SUPPORT_MARGIN,
                             "reach_interval": [MIN_REACH, MAX_REACH]},
              "cases": {}}
    for name, gap in cases.items():
        result = solve(gap)
        result["post_hoc_trace_validation"] = validate_solution(result, gap) if result["found"] else None
        output["cases"][name] = result
    flat = output["cases"]["continuous_ground"]
    trace = flat["trace"]
    action_index = np.arange(len(trace))
    body_x = np.array([state[0] for state in trace])
    feet_x = np.array([state[1] for state in trace])
    four_margin = [margin(state[0], tuple(int(round(f/STEP)) for f in state[1])) for state in trace]
    swing_margin = np.full(len(trace), np.nan)
    for k, (action, leg) in enumerate(flat["actions"]):
        if action == "single_leg_swing":
            stance = tuple(int(round(f/STEP)) for f in trace[k][1])
            swing_margin[k] = margin(trace[k][0], stance, tuple(i for i in range(4) if i != leg))
    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True,
                             gridspec_kw={"height_ratios": [2, 1]})
    axes[0].plot(action_index, body_x, color="black", linewidth=2, label="body x")
    for leg in range(4):
        axes[0].step(action_index, feet_x[:, leg], where="post", label=f"foot {leg+1} x")
    axes[0].set(ylabel="world x (m)", title="Contact-graph example: locked support and one-foot swings")
    axes[0].legend(ncol=5, fontsize=8, loc="upper left")
    axes[1].plot(action_index, four_margin, color="#287f8a", label="four-contact margin")
    axes[1].scatter(action_index, swing_margin, s=13, color="#c45b35", label="three-contact margin during swing")
    axes[1].axhline(MIN_SUPPORT_MARGIN, color="black", linestyle="--", linewidth=.8, label="required 1 cm")
    axes[1].set(xlabel="action index", ylabel="support margin (m)")
    axes[1].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(OUT/"contact_graph.png", dpi=170)
    plt.close(fig)
    (OUT/"contact_graph_results.json").write_text(json.dumps(output, indent=2), encoding="utf-8")
    print(json.dumps({name: {key: val for key, val in result.items() if key not in ("trace", "actions")}
                      | {"body_advances": sum(a[0] == "body_advance" for a in result.get("actions", [])),
                         "single_leg_swings": sum(a[0] == "single_leg_swing" for a in result.get("actions", []))}
                      for name, result in output["cases"].items()}, indent=2))


if __name__ == "__main__":
    main()

"""Receding local planning: immutable executed prefix, short SE(3) bands.

The bounded lattice is a local subgoal selector/recovery aid, not a route to the
global goal. Only points inside the current sensing sphere enter either search
or Newton--Euler relaxation. A measured point cloud is not a solid-volume map.
"""

from dataclasses import asdict, dataclass
import argparse
import hashlib
import heapq
import itertools
import json
import math
from pathlib import Path
import shutil
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

from . import core
from .point_selection import PointSelection
from .ray_dynamics import RayDynamicsConfig, relax
from .settings import planner_weights


@dataclass(frozen=True)
class RollingConfig:
    horizon: float = .65
    sensing_radius: float = 1.4
    advance: float = .10
    command_translation: float = .01
    command_rotation: float = math.radians(2)
    clearance: float = .035
    grid_xy: float = .10
    grid_z: float = .04
    grid_roll: float = math.radians(15)
    grid_pitch: float = math.radians(11)
    grid_yaw: float = math.radians(9)
    local_nodes: int = 9
    relaxation_steps: int = 100
    max_replans: int = 100
    search_expansions: int = 1400
    lateral_limit: float = .60
    goal_tolerance: float = .005

    def __post_init__(self):
        for name, value in asdict(self).items():
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ('local_nodes', 'relaxation_steps', 'max_replans', 'search_expansions'):
            if not isinstance(getattr(self, name), int):
                raise ValueError(f"{name} must be an integer")
        if self.local_nodes < 3 or self.advance > self.horizon:
            raise ValueError('Require at least 3 nodes and advance <= horizon')


def interpolate(a, b, fractions):
    fractions = np.asarray(fractions)
    rotations = Slerp([0, 1], Rotation.from_euler('xyz', [a[3:], b[3:]]))(fractions)
    return np.column_stack((a[:3] + fractions[:, None] * (b[:3] - a[:3]),
                            rotations.as_euler('xyz')))


def rotation_distance(a, b):
    return float((Rotation.from_euler('xyz', a[3:]).inv() *
                  Rotation.from_euler('xyz', b[3:])).magnitude())


class LocalCollision:
    """Full local-cloud gate, independent of ray hit selection.

    Translation sweeps use enclosing OBBs. Rotating sweeps use midpoint SDF
    bounds and adaptive subdivision, bounding motion of every body point.
    The guarantee is against sensed points; independent held-out surface
    validation is done after execution.
    """
    def __init__(self, points, half, clearance):
        self.points = points
        self.tree = cKDTree(points)
        self.half = np.asarray(half)
        self.clearance = clearance
        self.checks = 0

    def free(self, pose, padding=0.):
        half = self.half + padding
        ids = self.tree.query_ball_point(pose[:3], float(np.linalg.norm(half)+self.clearance))
        self.checks += 1
        if not ids:
            return True
        local = (self.points[ids] - pose[:3]) @ core.rpy_matrix(pose[3:])
        q = np.abs(local)-half
        distance = np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)
        return bool(distance.min() >= self.clearance)

    def edge(self, a, b):
        distance = float(np.linalg.norm(b[:3] - a[:3]))
        angle = rotation_distance(a, b)
        # Translation along a body axis has an exact single swept OBB.
        if angle < 1e-10:
            local_delta = (b[:3] - a[:3]) @ core.rpy_matrix(a[3:])
            mid = (a + b) / 2
            # The box encloses the sweep even for off-axis translations.
            return self.free(mid, .5 * np.abs(local_delta))
        midpoint = interpolate(a, b, [.5])[0]
        motion = .5*(distance+np.linalg.norm(self.half)*angle)
        # A Lipschitz SDF bound avoids rejecting tight but safe rotations merely
        # because an expanded axis-aligned envelope is too conservative.
        ids = self.tree.query_ball_point(midpoint[:3], np.linalg.norm(self.half)+self.clearance+motion)
        self.checks += 1
        if not ids:
            return True
        q = np.abs((self.points[ids]-midpoint[:3]) @ core.rpy_matrix(midpoint[3:]))-self.half
        clearance = (np.linalg.norm(np.maximum(q, 0), axis=1)+np.minimum(q.max(axis=1), 0)).min()
        if clearance >= self.clearance+motion:
            return True
        if clearance < self.clearance or motion < .00015:
            return False
        return self.edge(a, midpoint) and self.edge(midpoint, b)


class LocalBand(core.TrajectoryProblem):
    """Bounds move with the local band; there is no full-route node anchor."""
    def __init__(self, *args, workspace, **kwargs):
        super().__init__(*args, **kwargs)
        self.workspace = workspace

    def bounds(self):
        bounds = []
        for i in self.interior:
            pose = self.initial[i]
            box = [(max(self.workspace[0, 0], pose[0]-.16), min(self.workspace[0, 1], pose[0]+.16)),
                   (max(self.workspace[1, 0], pose[1]-.16), min(self.workspace[1, 1], pose[1]+.16)),
                   (max(.28, pose[2]-.08), min(.52, pose[2]+.08)), (-math.pi/4, math.pi/4),
                   (-math.radians(22), math.radians(22)), (-math.radians(18), math.radians(18))]
            bounds.extend(box[d] for d in self.free_dims)
        return bounds


def local_subgoal(current, goal, start, workspace, gate, config, visits):
    """Bounded A* to a local frontier, never searching to a distant goal.

    The distant goal supplies a direction/cost only. Search vertices are all
    within horizon of current. All attitude coordinates can change locally.
    """
    spacing = np.array([config.grid_xy, config.grid_xy, config.grid_z,
                        config.grid_roll, config.grid_pitch, config.grid_yaw])
    # All six coordinates start at the actual committed pose. Each edge
    # changes one coordinate, avoiding an implicit attitude reset or a snap
    # of the robot to a global lattice near an obstacle.
    origin, root = current.copy(), (0, 0, 0, 0, 0, 0)
    def pose(k):
        if k == root:
            return current.copy()
        return origin + spacing * k
    def within(p):
        return (np.all(p[:3] >= workspace[:, 0]-1e-8) and
                np.all(p[:3] <= workspace[:, 1]+1e-8) and
                abs(p[3]) <= math.pi/4+1e-8 and
                abs(p[4]) <= math.radians(22)+1e-8 and
                abs(p[5]) <= math.radians(18)+1e-8 and
                np.linalg.norm(p[:3]-current[:3]) <= config.horizon+1e-8)
    def heuristic(p):
        return np.linalg.norm(p[:3]-goal[:3]) + .08*rotation_distance(p, goal)
    def revisit(p):
        if not len(visits):
            return 0.
        return .025*float(np.exp(-np.sum((visits-p[:3])**2, axis=1)/.015).sum())
    queue, counter = [], itertools.count()
    heapq.heappush(queue, (heuristic(current), next(counter), root))
    costs, parent, closed = {root: 0.}, {}, set()
    best, best_score, reached = root, math.inf, False
    for _ in range(config.search_expansions):
        if not queue:
            break
        _, _, node = heapq.heappop(queue)
        if node in closed:
            continue
        closed.add(node)
        p = pose(node)
        distance = np.linalg.norm(p[:3]-current[:3])
        score = heuristic(p) + .12*costs[node] + revisit(p)
        if distance >= config.advance*.75 and score < best_score:
            best, best_score = node, score
        # Only connect the final goal once it lies inside this local horizon.
        if (np.linalg.norm(goal[:3]-current[:3]) <= config.horizon and
                np.linalg.norm(p[:3]-goal[:3]) <= config.grid_xy*1.5 and gate.edge(p, goal)):
            best, reached = node, True
            break
        if distance >= config.horizon-config.grid_xy*.55:
            best = node
            break
        for dimension, sign in itertools.product(range(6), (-1, 1)):
            adjacent = list(node)
            adjacent[dimension] += sign
            adjacent = tuple(adjacent)
            if adjacent in closed:
                continue
            q = pose(adjacent)
            if not within(q):
                continue
            length = np.linalg.norm(q[:3]-p[:3]) + .08*rotation_distance(p, q)
            proposed = costs[node] + length + revisit(q)
            if proposed >= costs.get(adjacent, math.inf):
                continue
            if not gate.edge(p, q):
                continue
            costs[adjacent], parent[adjacent] = proposed, node
            heapq.heappush(queue, (proposed + 1.3*heuristic(q), next(counter), adjacent))
    if best == root and not reached:
        return None, dict(search_expanded=len(closed), local_frontier_found=False)
    keys = [best]
    while keys[-1] != root:
        keys.append(parent[keys[-1]])
    route = np.array([pose(k) for k in reversed(keys)])
    if reached and (np.linalg.norm(route[-1]-goal) > 1e-10):
        route = np.vstack((route, goal))
    return route, dict(search_expanded=len(closed), local_frontier_found=True)


def checked_prefix(band, gate, config):
    """Issue bounded commands up to advance, or reject this entire prefix."""
    result, budget = [band[0]], config.advance
    angular_budget = math.radians(15)
    for a, b in zip(band[:-1], band[1:]):
        distance = float(np.linalg.norm(b[:3]-a[:3]))
        angle = rotation_distance(a, b)
        fraction = min(1., budget/max(distance, 1e-12), angular_budget/max(angle, 1e-12))
        if fraction <= 1e-8:
            break
        endpoint = interpolate(a, b, [fraction])[0]
        if not gate.edge(a, endpoint):
            return None
        n = max(1, math.ceil(distance*fraction/config.command_translation),
                math.ceil(angle*fraction/config.command_rotation))
        result.extend(interpolate(a, endpoint, np.linspace(0, 1, n+1)[1:]))
        budget -= distance*fraction
        angular_budget -= angle*fraction
        if fraction < 1-1e-10 or budget <= 1e-9 or angular_budget <= 1e-9:
            break
    return np.array(result) if len(result) > 1 else None


def plan(cloud, start, goal, half, config=RollingConfig()):
    """No ground truth or precomputed global route enters this boundary."""
    if config.sensing_radius < config.horizon + .24 + np.linalg.norm(half) + .06:
        raise ValueError('Sensing sphere must cover every allowed local band body')
    started = time.perf_counter()
    index = cKDTree(cloud)
    workspace = np.array([[min(start[0], goal[0]), max(start[0], goal[0])],
                          [-config.lateral_limit, config.lateral_limit], [.28, .52]])
    endpoint_gate = LocalCollision(cloud, half, config.clearance)
    if not endpoint_gate.free(start) or not endpoint_gate.free(goal):
        raise ValueError('Start/goal body lacks requested sensed-cloud clearance')
    weights = planner_weights()
    weights.path_xy, weights.height = 30., 100.
    dynamics = RayDynamicsConfig(time_step=.015, final_time_step=.003,
                                 schedule_steps=config.relaxation_steps, linear_damping=20., angular_damping=1.)
    executed, records, visits = [start.copy()], [], []
    status = 'replan_limit'
    for cycle in range(config.max_replans):
        tick = time.perf_counter()
        current = executed[-1]
        if np.linalg.norm(current[:3]-goal[:3]) < config.goal_tolerance and rotation_distance(current, goal) < math.radians(.5):
            status = 'goal_reached'
            break
        ids = index.query_ball_point(current[:3], config.sensing_radius)
        sensed = cloud[ids]
        gate = LocalCollision(sensed, half, config.clearance)
        route, search_info = local_subgoal(current, goal, start, workspace, gate, config, np.array(visits).reshape(-1, 3))
        record = dict(cycle=cycle, start_index=len(executed)-1, sensed_points=len(ids),
                      sensed_center=current[:3].tolist(), **search_info)
        if route is None:
            record.update(seconds=time.perf_counter()-tick, status='no_local_frontier')
            records.append(record)
            status = 'no_local_frontier'
            break
        # First try a straight local band, which may intersect obstacles.
        # If its immediate prefix is blocked, shorten to the first reachable
        # lattice edge; this recovery is explicitly recorded, never hidden.
        attempts, accepted = [], None
        for recovery in (False, True):
            target = route[1] if recovery else route[-1]
            initial = interpolate(current, target, np.linspace(0, 1, config.local_nodes))
            problem = LocalBand(sensed, np.full(len(sensed), .02**2), initial, half,
                                float(goal[2]), .055, 'full', weights,
                                point_selection=PointSelection('rays', .6, 2.), workspace=workspace)
            frames, statistics = [], []
            def callback(state, force, torque, linear, angular, total_force, total_torque_body, extra, potential):
                frames.append(dict(poses=state.poses().copy(),
                                   velocity_world=state.velocity_world.copy(), omega_body=state.omega_body.copy(),
                                   acceleration_world=linear.copy(), angular_acceleration_body=angular.copy(),
                                   total_force_world=total_force.copy(), total_torque_body=total_torque_body.copy(),
                                   **extra))
                statistics.append(extra)
            state, info = relax(problem, config.relaxation_steps, dynamics, callback)
            band = state.poses()
            prefix = checked_prefix(band, gate, config)
            attempts.append(dict(recovery=recovery, initial=initial.tolist(), final=band.tolist(),
                                 iterations=info['iterations'], formal_success=info['formal_success'],
                                 prefix_safe=prefix is not None,
                                 maximum_translation_step=max(s['maximum_node_translation_step'] for s in statistics),
                                 maximum_rotation_step=max(s['maximum_node_rotation_step'] for s in statistics),
                                 states=frames[::max(1, len(frames)//8)]))
            if prefix is not None:
                accepted = prefix
                break
        record.update(attempts=attempts, local_subgoal=route[-1].tolist(),
                      local_reachability_route=route.tolist(), collision_checks=gate.checks)
        if accepted is None:
            record.update(status='unsafe_prefix_rejected', seconds=time.perf_counter()-tick)
            records.append(record)
            status = 'unsafe_prefix_rejected'
            break
        executed.extend(accepted[1:])
        visits.append(current[:3].copy())
        record.update(end_index=len(executed)-1, seconds=time.perf_counter()-tick, status='advanced')
        records.append(record)
        if len(visits) >= 16 and np.linalg.norm(executed[-1][:3]-visits[-16]) < .03:
            status = 'stalled'
            break
    if np.linalg.norm(executed[-1][:3]-goal[:3]) < config.goal_tolerance and rotation_distance(executed[-1], goal) < math.radians(.5):
        status = 'goal_reached'
    return np.array(executed), records, dict(status=status, goal_reached=status=='goal_reached',
        goal_error_m=float(np.linalg.norm(executed[-1][:3]-goal[:3])),
        planning_seconds=time.perf_counter()-started, replans=len(records),
        command_count=len(executed)-1, recovery_replans=sum(any(a['recovery'] for a in r.get('attempts', [])) for r in records))


def run_case(source, output, name, config):
    from .experiment import dense_validation
    from .visualization.web import scene_label
    with np.load(source/'scenes'/f'{name}.npz') as data:
        cloud, truth, half = data['planning_point_cloud'], data['dense_truth_surface'], data['body_half_extents']
        reference = data['straight_reference_trajectory']
    params = json.loads((source/'scenes'/f'{name}.json').read_text())['params']
    endpoint_gate = LocalCollision(cloud, half, config.clearance)
    def select_endpoint(reference_pose, inward):
        candidates = []
        for dx, dy, dz in itertools.product((0., .1, -.1, .2, -.2),
                                            (0., .1, -.1, .2, -.2, .3, -.3), (0., -.04, .04)):
            pose = reference_pose.copy()
            pose[:3] += [inward*dx, dy, dz]
            if endpoint_gate.free(pose):
                candidates.append((dx*dx+dy*dy+2*dz*dz, pose))
        if not candidates:
            raise ValueError(f'{name}: no locally safe endpoint candidate')
        return min(candidates, key=lambda c:c[0])[1]
    start = select_endpoint(reference[0], 1)
    goal = select_endpoint(reference[-1], -1)
    # Keep endpoint selection independent of any trajectory or solver outcome.
    executed, records, metrics = plan(cloud, start, goal, half, config)
    # Held-out geometry is first used here, after the planner has stopped.
    checked = executed if len(executed)>1 else np.repeat(executed, 2, axis=0)
    cover = params.get('surface_cover_radius_m')
    if cover is None and params.get('source_repository'):
        cover = params.get('truth_surface_cover_radius_m', .012)
    validation = dense_validation(checked, truth, half, 2, 'so3', cover,
                                  'imported_collision_mesh' if params.get('source_repository') else 'synthetic')
    dense, clearance = validation.pop('dense_poses'), validation.pop('dense_clearance')
    metrics.update(validation)
    metrics.update(start=start.tolist(), goal=goal.tolist(),
                   original_start=reference[0].tolist(), original_goal=reference[-1].tolist())
    metrics['planning_success'] = bool(metrics['goal_reached'] and validation['continuous_lower_bound_against_surfaces_m'] >= 0)
    metrics['label'] = params.get('label', scene_label(name))
    np.savez_compressed(output/f'{name}.npz', executed=executed, start=start, goal=goal, half=half,
                        dense_poses=dense, dense_clearance=clearance)
    def default(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        raise TypeError(type(value).__name__)
    (output/f'{name}.json').write_text(json.dumps(dict(metrics=metrics, records=records), ensure_ascii=False, default=default))
    # Visualization cloud is never planner input on reload.
    np.savez_compressed(output/f'{name}.cloud.npz', points=cloud)
    print(name, metrics['status'], 'error', round(metrics['goal_error_m'], 3),
          'continuous', round(metrics['continuous_lower_bound_against_surfaces_m']*1000, 1),
          'mm', round(metrics['planning_seconds'], 2), 's', flush=True)
    return dict(name=name, params=params, **metrics)


def main(argv=None):
    parser = argparse.ArgumentParser(description='从起点开始的局部滚动射线动力学规划')
    parser.add_argument('--source', type=Path, default=Path(__file__).parent/'outputs/ray_open_world_collision_v1')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenes', nargs='+')
    parser.add_argument('--workers', type=int, default=1)
    parser.add_argument('--max-replans', type=int, default=100)
    parser.add_argument('--relaxation-steps', type=int, default=100)
    parser.add_argument('--viewer-root', type=Path)
    args = parser.parse_args(argv)
    config = RollingConfig(max_replans=args.max_replans, relaxation_steps=args.relaxation_steps)
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output/'protocol.json').exists():
        raise ValueError('Use a new output directory to preserve recorded results')
    protocol = dict(solver='rolling_ray_dynamics', source=str(args.source.resolve()), config=asdict(config),
                    global_route_computed=False, executed_prefix_immutable=True,
                    endpoint_selection='Nearest safe candidate within 0.2 m longitudinal / 0.3 m lateral / 0.04 m vertical of original task endpoints, using sensed cloud only; no solver-result selection',
                    local_search='Bounded position/attitude lattice selects local frontier; straight local band then one-edge recovery',
                    execution='Geometric pose commands, not a physical robot controller; virtual twists reset for each local solve')
    snapshots = args.output/'source'
    snapshots.mkdir()
    hashes = {}
    for filename in ('__main__.py', 'rolling_planner.py', 'ray_dynamics.py', 'core.py', 'point_selection.py',
                     'virtual_wrench.py', 'settings.py', 'experiment.py', 'rolling_report.py',
                     'visualization/web.py', 'visualization/rolling_web.py',
                     'visualization/assets/rolling.js', 'visualization/assets/rolling.html',
                     'tests/test_rolling_planner.py'):
        source = Path(__file__).parent/filename
        target = snapshots/filename
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        hashes[filename] = hashlib.sha256(source.read_bytes()).hexdigest()
    protocol['source_sha256'] = hashes
    protocol['source_protocol_sha256'] = hashlib.sha256((args.source/'protocol.json').read_bytes()).hexdigest()
    (args.output/'protocol.json').write_text(json.dumps(protocol, ensure_ascii=False, indent=2))
    names = args.scenes or [p.stem for p in sorted((args.source/'scenes').glob('*.npz'))]
    if args.workers > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(args.workers) as pool:
            futures = [pool.submit(run_case, args.source, args.output, name, config) for name in names]
            results = [f.result() for f in futures]
    else:
        results = [run_case(args.source, args.output, name, config) for name in names]
    (args.output/'results.json').write_text(json.dumps(results, ensure_ascii=False, indent=2))
    from .rolling_report import write_report
    write_report(args.output)
    from .visualization.rolling_web import export
    export(args.output, args.viewer_root)


if __name__ == '__main__':
    main()

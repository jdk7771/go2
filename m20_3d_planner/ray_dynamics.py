"""Ray-selected Newton--Euler relaxation of SE(3) trajectory nodes.

Positions/linear velocities are spatial; angular velocities, angular
accelerations and box inertia are body-local. Euler angles are only an adapter
for the existing bounds, diagnostics and viewer. No optimizer chooses a step.
"""

from dataclasses import dataclass, fields
import math

import numpy as np
from scipy.spatial.transform import Rotation

from . import core
from .virtual_wrench import packed_gradient, project_angular_velocity


@dataclass(frozen=True)
class RayDynamicsConfig:
    time_step: float = 0.01
    final_time_step: float = 0.001
    schedule_steps: int = 1000
    mass: float = 1.0
    linear_damping: float = 40.0
    angular_damping: float = 2.0
    minimum_time_step: float = 1e-7
    equilibrium_tolerance: float = 1e-4
    speed_tolerance: float = 1e-4
    settle_steps: int = 5
    max_translation_step: float | None = 0.01
    max_rotation_step: float | None = math.radians(2.)
    time_step_growth: float = 1.05

    def __post_init__(self):
        for name in ("schedule_steps", "settle_steps"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        for field in fields(self):
            if field.name in ("schedule_steps", "settle_steps"):
                continue
            value = getattr(self, field.name)
            if field.name in ("max_translation_step", "max_rotation_step") and value is None:
                continue  # Replaying archives produced before motion limits existed.
            minimum = 0 if field.name.endswith("damping") else None
            if not math.isfinite(value) or (value < 0 if minimum == 0 else value <= 0):
                raise ValueError(f"Invalid ray dynamics parameter: {field.name}")
        if not self.minimum_time_step <= self.final_time_step <= self.time_step:
            raise ValueError("Require minimum_time_step <= final_time_step <= time_step")
        if self.time_step_growth < 1:
            raise ValueError("time_step_growth must be at least 1")

    def scheduled_time_step(self, step):
        """Geometric coarse-to-fine ceiling, followed by a constant fine phase."""
        fraction = min(max(step, 0) / max(1, self.schedule_steps - 1), 1.)
        return self.time_step * (self.final_time_step / self.time_step) ** fraction

    def inertia(self, half):
        h2 = np.asarray(half) ** 2
        return self.mass / 3 * (h2.sum() - h2)


@dataclass
class NodeState:
    transforms: np.ndarray
    velocity_world: np.ndarray
    omega_body: np.ndarray

    @classmethod
    def from_poses(cls, poses):
        transforms = np.tile(np.eye(4), (len(poses), 1, 1))
        transforms[:, :3, :3] = Rotation.from_euler("xyz", poses[:, 3:]).as_matrix()
        transforms[:, :3, 3] = poses[:, :3]
        return cls(transforms, np.zeros((len(poses), 3)), np.zeros((len(poses), 3)))

    @property
    def positions(self):
        return self.transforms[:, :3, 3]

    @property
    def rotations(self):
        return self.transforms[:, :3, :3]

    def poses(self):
        r = self.rotations
        angles = np.column_stack((
            np.arctan2(r[:, 2, 1], r[:, 2, 2]),
            np.arctan2(-r[:, 2, 0], np.hypot(r[:, 2, 1], r[:, 2, 2])),
            np.arctan2(r[:, 1, 0], r[:, 0, 0]),
        ))
        return np.column_stack((self.positions, angles))


def log_jacobian_transpose(phi, effort, side):
    """Apply inverse left/right SO(3) log-Jacobian transpose to covectors."""
    theta2 = np.sum(phi * phi, axis=-1)
    coefficient = np.empty_like(theta2)
    small = theta2 < 1e-8
    coefficient[small] = 1 / 12 + theta2[small] / 720
    theta = np.sqrt(theta2[~small])
    coefficient[~small] = (1 - .5 * theta / np.tan(.5 * theta)) / theta2[~small]
    cross = np.cross(phi, effort)
    return effort + (.5 if side == "left" else -.5) * cross + coefficient[..., None] * np.cross(phi, cross)


def elastic_wrenches(problem, state, selections=None):
    """Potential, conservative spatial force/moment, and current ray hits.

    An explicit selections list freezes ray associations during a numerical
    retry. Fresh associations are computed at every accepted node state.
    Rotational springs use SO(3) logarithms, not Euler-angle differences.
    """
    if selections is None:
        selections = problem.selector.select(state.positions)
    force = np.zeros_like(state.positions)
    torque = np.zeros_like(force)
    energy = 0.
    w = problem.weights
    for j, (position, rotation) in enumerate(zip(state.positions, state.rotations)):
        indices = selections[j]
        value, force[j], torque[j], _, _ = core.collision_energy_wrench(
            problem.points[indices], problem.point_weights[indices], position,
            rotation, problem.half, problem.safety, w.collision_stiffness,
        )
        energy += value

    xy = state.positions[:, :2] - problem.initial[:, :2]
    height = state.positions[:, 2] - problem.nominal_height
    energy += .5 * w.path_xy * np.sum(xy ** 2) + .5 * w.height * np.sum(height ** 2)
    force[:, :2] -= w.path_xy * xy
    force[:, 2] -= w.height * height
    delta = np.diff(state.positions, axis=0)
    bend = np.diff(delta, axis=0)
    energy += .5 * w.velocity_translation * np.sum(delta ** 2)
    energy += .5 * w.acceleration_translation * np.sum(bend ** 2)
    force[:-1] += w.velocity_translation * delta
    force[1:] -= w.velocity_translation * delta
    force[:-2] -= w.acceleration_translation * bend
    force[1:-1] += 2 * w.acceleration_translation * bend
    force[2:] -= w.acceleration_translation * bend

    reference = Rotation.from_euler("xyz", problem.initial[:, 3:]).as_matrix()
    attitude = Rotation.from_matrix(reference.transpose(0, 2, 1) @ state.rotations).as_rotvec()
    attitude_effort = w.attitude * np.array([1., 1., .35]) * attitude
    energy += .5 * np.sum(attitude * attitude_effort)
    body_moment = -log_jacobian_transpose(attitude, attitude_effort, "right")

    relative = state.rotations[:-1].transpose(0, 2, 1) @ state.rotations[1:]
    phi = Rotation.from_matrix(relative).as_rotvec()
    phi_bend = np.diff(phi, axis=0)
    energy += .5 * w.velocity_rotation * np.sum(phi ** 2)
    energy += .5 * w.acceleration_rotation * np.sum(phi_bend ** 2)
    effort = w.velocity_rotation * phi
    effort[:-1] -= w.acceleration_rotation * phi_bend
    effort[1:] += w.acceleration_rotation * phi_bend
    body_moment[:-1] += log_jacobian_transpose(phi, effort, "left")
    body_moment[1:] -= log_jacobian_transpose(phi, effort, "right")
    torque += np.einsum("nij,nj->ni", state.rotations, body_moment)
    return float(energy), force, torque, selections


def accelerations(state, force_world, torque_world, config, half):
    """Explicit Newton--Euler RHS, including damping and gyroscopic term.

    These accelerations exclude hard-bound impulses. Fixed endpoints have no
    motion; their support reactions are not modeled as external commands.
    """
    inertia = config.inertia(half)
    total_force = force_world - config.linear_damping * state.velocity_world
    total_torque_body = np.einsum("nji,nj->ni", state.rotations, torque_world)
    total_torque_body -= config.angular_damping * state.omega_body
    linear = total_force / config.mass
    angular = (total_torque_body - np.cross(state.omega_body, inertia * state.omega_body)) / inertia
    linear[[0, -1]] = 0
    angular[[0, -1]] = 0
    return linear, angular, total_force, total_torque_body


class PoseConstraints:
    def __init__(self, problem):
        self.initial = NodeState.from_poses(problem.initial)
        self.lower = problem.initial.copy()
        self.upper = problem.initial.copy()
        bounds = np.asarray(problem.bounds()).reshape(len(problem.interior), -1, 2)
        self.lower[np.ix_(problem.interior, problem.free_dims)] = bounds[:, :, 0]
        self.upper[np.ix_(problem.interior, problem.free_dims)] = bounds[:, :, 1]
        self.free_angles = [int(d - 3) for d in problem.free_dims if d >= 3]
        self.interior = problem.interior


def predict_twist(state, linear_acceleration, angular_acceleration, dt, constraints, inertia):
    """Predict velocities before pose projection; enforce current tangent locks."""
    velocity = state.velocity_world + dt * linear_acceleration
    omega = state.omega_body + dt * angular_acceleration
    poses = state.poses()
    for j in constraints.interior:
        omega[j] = project_angular_velocity(
            omega[j], poses[j], constraints.lower[j], constraints.upper[j], constraints.free_angles, inertia,
        )
    velocity[[0, -1]] = 0
    omega[[0, -1]] = 0
    velocity[(state.positions <= constraints.lower[:, :3] + 1e-10) & (velocity < 0)] = 0
    velocity[(state.positions >= constraints.upper[:, :3] - 1e-10) & (velocity > 0)] = 0
    return velocity, omega


def proposed_motion(prediction, dt):
    """Unclipped displacement and unwrapped rotation angle, including >2*pi."""
    velocity, omega = prediction
    return (float(dt * np.linalg.norm(velocity, axis=1).max()),
            float(dt * np.linalg.norm(omega, axis=1).max()))


def actual_motion(before, after):
    relative = before.rotations.transpose(0, 2, 1) @ after.rotations
    return (float(np.linalg.norm(after.positions - before.positions, axis=1).max()),
            float(np.linalg.norm(Rotation.from_matrix(relative).as_rotvec(), axis=1).max()))


def motion_within_limits(motion, config):
    return all(limit is None or value <= limit * (1 + 1e-12)
               for value, limit in zip(motion, (config.max_translation_step, config.max_rotation_step)))


def semi_implicit_step(state, linear_acceleration, angular_acceleration, dt, constraints, inertia, prediction=None):
    """v+=dt*a, omega+=dt*alpha, p+=dt*v, R=R*Exp(dt*omega)."""
    velocity, omega = prediction if prediction is not None else predict_twist(
        state, linear_acceleration, angular_acceleration, dt, constraints, inertia,
    )
    transforms = state.transforms.copy()
    transforms[:, :3, 3] += dt * velocity
    transforms[:, :3, :3] = state.rotations @ Rotation.from_rotvec(dt * omega).as_matrix()
    candidate = NodeState(transforms, velocity, omega)
    proposed = candidate.poses()
    clipped = np.clip(proposed, constraints.lower, constraints.upper)
    candidate.transforms[:, :3, 3] = clipped[:, :3]
    rotation_clipped = np.any(np.abs(clipped[:, 3:] - proposed[:, 3:]) > 1e-12, axis=1)
    if rotation_clipped.any():
        candidate.transforms[rotation_clipped, :3, :3] = Rotation.from_euler("xyz", clipped[rotation_clipped, 3:]).as_matrix()
    candidate.transforms[[0, -1]] = constraints.initial.transforms[[0, -1]]
    velocity[(clipped[:, :3] <= constraints.lower[:, :3] + 1e-10) & (velocity < 0)] = 0
    velocity[(clipped[:, :3] >= constraints.upper[:, :3] - 1e-10) & (velocity > 0)] = 0
    for j in constraints.interior:
        omega[j] = project_angular_velocity(
            omega[j], clipped[j], constraints.lower[j], constraints.upper[j], constraints.free_angles, inertia,
        )
    return candidate


def kinetic_energy(state, config, half):
    return float(.5 * config.mass * np.sum(state.velocity_world ** 2)
                 + .5 * np.sum(config.inertia(half) * state.omega_body ** 2))


def relax(problem, maxiter, config, callback):
    """Ray selection -> surface wrench -> acceleration -> semi-implicit step.

    A retry uses the same selected points to avoid confusing changes in ray
    hits with integrator instability. Reselection can change the reported
    potential; no globally monotone potential is claimed for ray switching.
    """
    if problem.selector.config.method != "rays":
        raise ValueError("ray_dynamics requires center-origin ray point selection")
    if maxiter < 1:
        raise ValueError("maxiter must be positive")
    state = NodeState.from_poses(problem.initial)
    constraints = PoseConstraints(problem)
    inertia = config.inertia(problem.half)
    bounds = np.asarray(problem.bounds())
    potential, force, torque, selections = elastic_wrenches(problem, state)
    evaluations, retries, elapsed, settled, step = 1, 0, 0., 0, 0
    motion_retries = energy_retries = 0
    dt = config.time_step
    success, message = False, "Maximum ray dynamics steps reached"

    def record(used_dt, scheduled_dt, proposal=(0., 0.), accepted=(0., 0.)):
        linear, angular, total_force, total_torque_body = accelerations(state, force, torque, config, problem.half)
        poses = state.poses()
        gradient = packed_gradient(problem, poses, force, torque)
        x = problem.pack(poses)
        residual = float(np.max(np.abs(x - np.clip(x - gradient, bounds[:, 0], bounds[:, 1]))))
        speed = max(np.linalg.norm(state.velocity_world, axis=1).max(), np.linalg.norm(state.omega_body, axis=1).max())
        kinetic = kinetic_energy(state, config, problem.half)
        callback(state, force, torque, linear, angular, total_force, total_torque_body, {
            "relaxation_time": elapsed, "time_step": used_dt,
            "scheduled_time_step": scheduled_dt, "kinetic_energy": kinetic,
            "mechanical_energy": potential + kinetic, "equilibrium_residual": residual,
            "maximum_linear_speed": float(np.linalg.norm(state.velocity_world, axis=1).max()),
            "maximum_angular_speed": float(np.linalg.norm(state.omega_body, axis=1).max()),
            "maximum_linear_acceleration": float(np.linalg.norm(linear, axis=1).max()),
            "maximum_angular_acceleration": float(np.linalg.norm(angular, axis=1).max()),
            "maximum_proposed_translation_step": proposal[0],
            "maximum_proposed_rotation_step": proposal[1],
            "maximum_node_translation_step": accepted[0],
            "maximum_node_rotation_step": accepted[1],
        }, potential)
        return linear, angular, residual, speed

    linear, angular, residual, speed = record(0., config.time_step)
    for step in range(1, maxiter + 1):
        scheduled = config.scheduled_time_step(step - 1)
        dt = min(dt * config.time_step_growth if step > 1 else dt, scheduled)
        previous_energy = potential + kinetic_energy(state, config, problem.half)
        while True:
            prediction = predict_twist(state, linear, angular, dt, constraints, inertia)
            proposal = proposed_motion(prediction, dt)
            if not motion_within_limits(proposal, config):
                motion_retries += 1
            else:
                candidate = semi_implicit_step(state, linear, angular, dt, constraints, inertia, prediction)
                accepted = actual_motion(state, candidate)
                if not motion_within_limits(accepted, config):
                    motion_retries += 1
                else:
                    candidate_potential, _, _, _ = elastic_wrenches(problem, candidate, selections)
                    evaluations += 1
                    energy = candidate_potential + kinetic_energy(candidate, config, problem.half)
                    if np.isfinite(energy) and energy <= previous_energy + 1e-10 * max(1., abs(previous_energy)):
                        break
                    energy_retries += 1
            retries += 1
            dt *= .5
            if dt < config.minimum_time_step:
                return state, dict(formal_success=False, message="Integrator time step below minimum; no equilibrium certificate",
                                   iterations=step - 1, evaluations=evaluations, rejected_steps=retries, relaxation_time=elapsed,
                                   motion_limited_retries=motion_retries, energy_rejected_steps=energy_retries)
        state = candidate
        elapsed += dt
        potential, force, torque, selections = elastic_wrenches(problem, state)
        evaluations += 1
        linear, angular, residual, speed = record(dt, scheduled, proposal, accepted)
        settled = settled + 1 if residual <= config.equilibrium_tolerance and speed <= config.speed_tolerance else 0
        if settled >= config.settle_steps:
            success, message = True, "Constrained wrench equilibrium and speed tolerances sustained"
            break
    return state, dict(formal_success=success, message=message, iterations=step,
                       evaluations=evaluations, rejected_steps=retries, relaxation_time=elapsed,
                       motion_limited_retries=motion_retries, energy_rejected_steps=energy_retries)

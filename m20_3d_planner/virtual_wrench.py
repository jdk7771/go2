"""Damped rigid-body relaxation of a chain of body poses.

Collision wrenches act on the real body OBB. Translational springs and Euler
angle springs retain the baseline potential exactly; angular spring efforts
are mapped to spatial moments by virtual work. Each body has virtual mass,
box inertia, linear velocity and body-frame angular velocity. No optimizer or
cost_grad call determines the update. Pseudo-time is unrelated to execution.
"""

from dataclasses import dataclass, asdict
import math

import numpy as np

from . import core


@dataclass(frozen=True)
class DynamicsConfig:
    time_step: float = 0.01
    mass: float = 1.0
    linear_damping: float = 40.0
    angular_damping: float = 2.0
    minimum_time_step: float = 1e-7
    equilibrium_tolerance: float = 1e-4
    speed_tolerance: float = 1e-4

    def __post_init__(self):
        if any(not math.isfinite(x) or x <= 0 for x in asdict(self).values()):
            raise ValueError("Virtual dynamics parameters must be finite and positive")
        if self.minimum_time_step > self.time_step:
            raise ValueError("minimum_time_step cannot exceed time_step")

    def inertia(self, half):
        # Uniform box: Ixx = m/12 * (width**2 + height**2), etc.
        h2 = np.asarray(half) ** 2
        return self.mass / 3 * (h2.sum() - h2)


def euler_axes(rpy):
    """World angular velocity = A(rpy) @ Euler angle rates."""
    _, pitch, yaw = rpy
    return np.column_stack((
        core.rz(yaw) @ core.ry(pitch) @ np.array([1., 0., 0.]),
        core.rz(yaw) @ np.array([0., 1., 0.]),
        np.array([0., 0., 1.]),
    ))


def matrix_rpy(matrix):
    """Inverse of Rz(yaw) Ry(pitch) Rx(roll), away from pitch = +/-90 deg."""
    return np.array([
        math.atan2(matrix[2, 1], matrix[2, 2]),
        math.atan2(-matrix[2, 0], math.hypot(matrix[2, 1], matrix[2, 2])),
        math.atan2(matrix[1, 0], matrix[0, 0]),
    ])


def body_wrenches(problem, poses):
    """Return potential, spatial force and spatial moment for every body.

    The potential is reported for diagnostics and integrator stability. Forces
    and moments, rather than packed optimization gradients, drive the bodies.
    """
    force = np.zeros((len(poses), 3))
    torque = np.zeros_like(force)
    energy = 0.
    w = problem.weights
    selections = problem.selector.select(poses[:, :3])
    for j, pose in enumerate(poses):
        indices = selections[j] if selections is not None else slice(None)
        value, force[j], torque[j], _, _ = core.collision_energy_wrench(
            problem.points[indices], problem.point_weights[indices],
            pose[:3], core.rpy_matrix(pose[3:]), problem.half,
            problem.safety, w.collision_stiffness,
        )
        energy += value

    xy = poses[:, :2] - problem.initial[:, :2]
    z = poses[:, 2] - problem.nominal_height
    energy += .5 * w.path_xy * np.sum(xy ** 2) + .5 * w.height * np.sum(z ** 2)
    force[:, :2] -= w.path_xy * xy
    force[:, 2] -= w.height * z
    angle_scale = np.array([1., 1., .35])
    energy += .5 * w.attitude * np.sum(angle_scale * poses[:, 3:] ** 2)
    angular_effort = -w.attitude * angle_scale * poses[:, 3:]

    # Neighbor springs and bending springs; accumulate equal/opposite efforts.
    for values, effort, stiffness, bending in (
        (poses[:, :3], force, w.velocity_translation, w.acceleration_translation),
        (poses[:, 3:], angular_effort, w.velocity_rotation, w.acceleration_rotation),
    ):
        delta = values[1:] - values[:-1]
        second = values[:-2] - 2 * values[1:-1] + values[2:]
        energy += .5 * stiffness * np.sum(delta ** 2) + .5 * bending * np.sum(second ** 2)
        effort[:-1] += stiffness * delta
        effort[1:] -= stiffness * delta
        effort[:-2] -= bending * second
        effort[1:-1] += 2 * bending * second
        effort[2:] -= bending * second
    for j, pose in enumerate(poses):
        # A.T @ tau = generalized Euler spring effort, NOT tau = effort.
        torque[j] += np.linalg.solve(euler_axes(pose[3:]).T, angular_effort[j])
    return float(energy), force, torque


def packed_gradient(problem, poses, force, torque):
    """Equilibrium diagnostic only; never used to choose a motion step."""
    gradient = np.empty_like(poses)
    gradient[:, :3] = -force
    for j, pose in enumerate(poses):
        gradient[j, 3:] = -euler_axes(pose[3:]).T @ torque[j]
    return problem.pack(gradient)


def project_angular_velocity(omega, pose, lower, upper, free_angles, inertia):
    """Inertia-metric constraint impulse removes outward/locked angle rates.

    This cannot increase kinetic energy. In general simply zeroing a component
    of body omega would not lock the corresponding Euler angle.
    """
    rate_map = np.linalg.solve(euler_axes(pose[3:]), core.rpy_matrix(pose[3:]))
    locked = set(range(3)) - set(free_angles)
    original = omega.copy()
    for _ in range(4):
        rates = rate_map @ omega
        for axis in free_angles:
            if ((pose[3 + axis] <= lower[3 + axis] + 1e-10 and rates[axis] < -1e-12)
                    or (pose[3 + axis] >= upper[3 + axis] - 1e-10 and rates[axis] > 1e-12)):
                locked.add(axis)
        if not locked:
            break
        constraints = rate_map[sorted(locked)]
        inverse_mass = constraints.T / inertia[:, None]
        omega = original - inverse_mass @ np.linalg.solve(
            constraints @ inverse_mass, constraints @ original
        )
    return omega


def rigid_body_step(problem, poses, velocity, omega, force, torque, config, dt):
    """Damped semi-implicit translation and body-frame Euler rigid-body motion."""
    inertia = config.inertia(problem.half)
    linear_decay = math.exp(-config.linear_damping * dt / config.mass)
    angular_decay = np.exp(-config.angular_damping * dt / inertia)
    next_velocity = linear_decay * velocity + (1 - linear_decay) * force / config.linear_damping
    next_omega = np.zeros_like(omega)
    candidate = poses.copy()
    all_lower = problem.initial.copy()
    all_upper = problem.initial.copy()
    bounds = np.asarray(problem.bounds()).reshape(len(problem.interior), -1, 2)
    all_lower[np.ix_(problem.interior, problem.free_dims)] = bounds[:, :, 0]
    all_upper[np.ix_(problem.interior, problem.free_dims)] = bounds[:, :, 1]
    free_angles = [int(d - 3) for d in problem.free_dims if d >= 3]
    for j in problem.interior:
        rotation = core.rpy_matrix(poses[j, 3:])
        moment = rotation.T @ torque[j] - np.cross(omega[j], inertia * omega[j])
        angular = angular_decay * omega[j] + (1 - angular_decay) * moment / config.angular_damping
        angular = project_angular_velocity(
            angular, poses[j], all_lower[j], all_upper[j], free_angles, inertia
        )
        candidate[j, :3] += dt * next_velocity[j]
        speed = np.linalg.norm(angular)
        if speed > 1e-14:
            rotation = rotation @ core.axis_angle(angular.copy(), speed * dt)
            candidate[j, 3:] = matrix_rpy(rotation)
        candidate[j] = np.clip(candidate[j], all_lower[j], all_upper[j])
        next_omega[j] = project_angular_velocity(
            angular, candidate[j], all_lower[j], all_upper[j], free_angles, inertia
        )
    # Constraint reactions remove outward translational velocity; endpoints and
    # locked coordinates have zero velocity. No physical time is attached.
    next_velocity[(candidate[:, :3] <= all_lower[:, :3] + 1e-10) & (next_velocity < 0)] = 0
    next_velocity[(candidate[:, :3] >= all_upper[:, :3] - 1e-10) & (next_velocity > 0)] = 0
    next_velocity[[0, -1]] = 0
    return candidate, next_velocity, next_omega


def kinetic_energy(velocity, omega, config, half):
    return float(.5 * config.mass * np.sum(velocity ** 2)
                 + .5 * np.sum(config.inertia(half) * omega ** 2))


def relax(problem, maxiter, config, callback):
    """Integrate until constrained force balance AND negligible motion.

    Retry a step at half dt if total mechanical energy grows: this is a
    stability guard on the numerical integrator, not a potential line search.
    Potential alone is allowed to increase by exchanging kinetic energy.
    """
    poses = problem.initial.copy()
    velocity = np.zeros((len(poses), 3))
    omega = np.zeros_like(velocity)
    potential, force, torque = body_wrenches(problem, poses)
    energy = potential
    elapsed, dt, evaluations, retries = 0., config.time_step, 1, 0
    bounds = np.asarray(problem.bounds())
    success, message = False, "Maximum virtual dynamics steps reached"

    def record(step, used_dt):
        callback(poses, force, torque, velocity, omega, {
            "relaxation_time": elapsed, "time_step": used_dt,
            "kinetic_energy": kinetic_energy(velocity, omega, config, problem.half),
            "mechanical_energy": energy,
            "maximum_linear_speed": float(np.linalg.norm(velocity, axis=1).max()),
            "maximum_angular_speed": float(np.linalg.norm(omega, axis=1).max()),
        }, potential)

    record(0, 0.)
    step = 0
    for step in range(1, maxiter + 1):
        while True:
            candidate, next_v, next_w = rigid_body_step(
                problem, poses, velocity, omega, force, torque, config, dt
            )
            next_potential, next_force, next_torque = body_wrenches(problem, candidate)
            evaluations += 1
            next_energy = next_potential + kinetic_energy(next_v, next_w, config, problem.half)
            if np.isfinite(next_energy) and next_energy <= energy + 1e-10 * max(1., abs(energy)):
                break
            retries += 1
            dt *= .5
            if dt < config.minimum_time_step:
                message = "Integrator time step fell below minimum; no equilibrium certificate"
                return poses, dict(formal_success=False, message=message,
                                   iterations=step - 1, evaluations=evaluations,
                                   rejected_steps=retries, relaxation_time=elapsed)
        poses, velocity, omega = candidate, next_v, next_w
        potential, force, torque, energy = next_potential, next_force, next_torque, next_energy
        elapsed += dt
        record(step, dt)
        gradient = packed_gradient(problem, poses, force, torque)
        x = problem.pack(poses)
        equilibrium = np.max(np.abs(x - np.clip(x - gradient, bounds[:, 0], bounds[:, 1])))
        speed = max(np.linalg.norm(velocity, axis=1).max(), np.linalg.norm(omega, axis=1).max())
        if equilibrium <= config.equilibrium_tolerance and speed <= config.speed_tolerance:
            success, message = True, "Constrained wrench equilibrium and speed tolerances reached"
            break
        dt = min(config.time_step, dt * 1.05)
    return poses, dict(formal_success=success, message=message, iterations=step,
                       evaluations=evaluations, rejected_steps=retries, relaxation_time=elapsed)

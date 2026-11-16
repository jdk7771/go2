"""Force/torque consistency, inertial motion and constrained dissipation."""

import unittest
from unittest.mock import patch

import numpy as np

from m20_3d_planner import core, settings, experiment
from m20_3d_planner.virtual_wrench import (
    DynamicsConfig, body_wrenches, packed_gradient, rigid_body_step,
    kinetic_energy, project_angular_velocity, matrix_rpy, euler_axes,
)


class VirtualWrenchChecks(unittest.TestCase):
    def setUp(self):
        self.half = np.array([.375, .115, .07])
        self.initial = core.make_initial_trajectory(5, .4)
        self.cloud = np.random.default_rng(4).uniform([-.2, .12, .43], [.3, .16, .48], (30, 3))

    def problem(self, mode="full"):
        return core.TrajectoryProblem(
            self.cloud, np.full(len(self.cloud), .02 ** 2), self.initial,
            self.half, .4, .047, mode, settings.planner_weights(), lateral_limit=.6,
        )

    def test_wrenches_retain_baseline_potential_and_virtual_work(self):
        for mode in settings.MODES:
            problem = self.problem(mode)
            x = problem.pack(self.initial) + np.random.default_rng(14).normal(0, .015, len(problem.pack(self.initial)))
            poses = problem.unpack(x)
            energy, force, torque = body_wrenches(problem, poses)
            baseline, gradient = problem.cost_grad(x)
            self.assertAlmostEqual(energy, baseline, places=11)
            np.testing.assert_allclose(packed_gradient(problem, poses, force, torque), gradient, atol=1e-10)

    def test_body_inertia_and_motion_depend_on_mass_and_moment(self):
        problem = self.problem()
        zero = np.zeros((5, 3))
        force, moment = zero.copy(), zero.copy()
        force[2, 1], moment[2, 0] = 1., .1
        dt, config = 1e-4, DynamicsConfig()
        poses, velocity, omega = rigid_body_step(problem, self.initial, zero, zero, force, moment, config, dt)
        np.testing.assert_allclose(config.inertia(self.half), [(.23 ** 2 + .14 ** 2) / 12, (.75 ** 2 + .14 ** 2) / 12, (.75 ** 2 + .23 ** 2) / 12])
        expected_v = (1 - np.exp(-config.linear_damping * dt / config.mass)) / config.linear_damping
        self.assertAlmostEqual(velocity[2, 1], expected_v)
        expected_w = (1 - np.exp(-config.angular_damping * dt / config.inertia(self.half)[0])) * .1 / config.angular_damping
        self.assertAlmostEqual(omega[2, 0], expected_w)
        self.assertAlmostEqual(poses[2, 3], dt * expected_w)
        self.assertAlmostEqual(poses[2, 1], dt * expected_v)
        _, heavier_v, heavier_w = rigid_body_step(problem, self.initial, zero, zero, force, moment, DynamicsConfig(mass=2), dt)
        self.assertLess(heavier_v[2, 1], velocity[2, 1])
        self.assertLess(heavier_w[2, 0], omega[2, 0])
        # Momentum persists after removing the external wrench, with damping.
        _, coast_v, _ = rigid_body_step(problem, poses, velocity, omega, zero, zero, config, dt)
        self.assertGreater(coast_v[2, 1], 0)
        self.assertLess(coast_v[2, 1], velocity[2, 1])

    def test_constraint_impulse_removes_outward_rates_without_adding_energy(self):
        pose = np.array([0., 0., .4, .2, -.15, .1])
        lower, upper = pose.copy(), pose.copy()
        lower[3:], upper[3:] = [-.5, -.4, -.5], pose[3:]
        inertia = DynamicsConfig().inertia(self.half)
        omega = np.array([2., 3., 4.])
        clipped = project_angular_velocity(omega, pose, lower, upper, [0, 1, 2], inertia)
        rates = np.linalg.solve(euler_axes(pose[3:]), core.rpy_matrix(pose[3:]) @ clipped)
        self.assertTrue(np.all(rates <= 1e-10))
        self.assertLessEqual(np.sum(inertia * clipped ** 2), np.sum(inertia * omega ** 2) + 1e-12)
        np.testing.assert_allclose(matrix_rpy(core.rpy_matrix(pose[3:])), pose[3:], atol=1e-12)

    def test_solver_integrates_without_optimizer_or_cost_gradient(self):
        snapshots, dynamics = [], []
        with patch.object(experiment, "minimize", side_effect=AssertionError("optimizer called")), patch.object(core.TrajectoryProblem, "cost_grad", side_effect=AssertionError("gradient update called")):
            poses, stats, history, _ = experiment.optimize_from_cloud(
                self.cloud, self.initial, self.half, .4, "full", 12,
                solver="virtual_wrench", pose_history=snapshots,
                dynamics_history=dynamics, lateral_limit=.6,
            )
        self.assertEqual(stats["solver"], "virtual_wrench")
        self.assertEqual(len(history), stats["iterations"] + 1)
        self.assertGreater(history[1]["kinetic_energy"], 0)
        energy = np.array([h["mechanical_energy"] for h in history])
        self.assertTrue(np.all(np.diff(energy) <= 1e-8))
        np.testing.assert_array_equal(poses[[0, -1]], self.initial[[0, -1]])
        np.testing.assert_array_equal(dynamics[0][:, 6:], 0)
        for states in dynamics:
            np.testing.assert_array_equal(states[[0, -1], 6:], 0)
        self.assertGreater(stats["relaxation_time"], 0)
        self.assertGreater(stats["gradient_inf_norm"], 0)

    def test_locked_dofs_and_hard_bounds(self):
        for mode in settings.MODES:
            problem = self.problem(mode)
            zero = np.zeros((5, 3))
            force = np.full((5, 3), 1e5)
            poses, velocity, omega = rigid_body_step(problem, self.initial, zero, zero, force, force, DynamicsConfig(), .01)
            bounds = np.asarray(problem.bounds())
            self.assertTrue(np.all(problem.pack(poses) >= bounds[:, 0]))
            self.assertTrue(np.all(problem.pack(poses) <= bounds[:, 1]))
            locked = sorted(set(range(6)) - set(problem.free_dims))
            np.testing.assert_array_equal(poses[:, locked], self.initial[:, locked])
            np.testing.assert_array_equal(poses[[0, -1]], self.initial[[0, -1]])

    def test_invalid_parameters(self):
        for value in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                DynamicsConfig(mass=value)
        with self.assertRaises(ValueError):
            DynamicsConfig(time_step=1e-8)

    def test_settling_requires_small_speed_and_wrench_and_keeps_modes_locked(self):
        from m20_3d_planner.virtual_wrench import relax

        problem = self.problem("translation")
        problem.points = np.empty((0, 3))
        problem.point_weights = np.empty(0)
        problem.weights = core.PlannerWeights(**{
            key: (32. if key == "height" else 0.)
            for key in core.PlannerWeights.__dataclass_fields__
        })
        problem.initial[:, 2] = .45
        records = []
        poses, stats = relax(
            problem, 3000, DynamicsConfig(linear_damping=12.),
            lambda q, f, t, v, omega, extra, cost: records.append((q.copy(), extra, cost)),
        )
        self.assertTrue(stats["formal_success"])
        np.testing.assert_allclose(poses[1:-1, 2], .4, atol=4e-6)
        np.testing.assert_array_equal(poses[:, 3:], 0)
        self.assertLess(records[-1][1]["maximum_linear_speed"], 1e-4)
        self.assertLess(records[-1][2], records[0][2])


if __name__ == "__main__":
    unittest.main()

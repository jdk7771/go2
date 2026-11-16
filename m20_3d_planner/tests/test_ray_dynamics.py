"""SE(3) virtual work, frame consistency, integration and validation checks."""

import unittest
from unittest.mock import patch
from dataclasses import asdict, replace
import json
from pathlib import Path
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation

from m20_3d_planner import core, experiment, settings
from m20_3d_planner.point_selection import PointSelection
from m20_3d_planner.ray_dynamics import (
    NodeState, PoseConstraints, RayDynamicsConfig, accelerations,
    elastic_wrenches, relax, semi_implicit_step, predict_twist,
    proposed_motion, actual_motion, motion_within_limits,
)


class RayDynamicsChecks(unittest.TestCase):
    def setUp(self):
        self.half = np.array([.375, .115, .07])
        self.initial = core.make_initial_trajectory(5, .4)
        self.cloud = np.random.default_rng(7).uniform([-.2, .13, .44], [.3, .17, .49], (30, 3))

    def problem(self, mode="full", empty=False):
        cloud = self.cloud[:0] if empty else self.cloud
        return core.TrajectoryProblem(
            cloud, np.full(len(cloud), .02 ** 2), self.initial,
            self.half, .4, .047, mode, settings.planner_weights(),
            PointSelection("rays"), lateral_limit=.6,
        )

    def test_newton_euler_uses_body_torque_inertia_and_gyroscopic_term(self):
        poses = self.initial.copy()
        poses[2, 3:] = [.2, -.1, .15]
        state = NodeState.from_poses(poses)
        state.velocity_world[2] = [.1, -.2, .05]
        state.omega_body[2] = [.3, -.4, .2]
        force, torque = np.zeros((5, 3)), np.zeros((5, 3))
        force[2], torque[2] = [2., -1., .5], [.1, .2, -.3]
        config = RayDynamicsConfig(linear_damping=3., angular_damping=.2)
        linear, angular, net_force, body_torque = accelerations(state, force, torque, config, self.half)
        inertia = config.inertia(self.half)
        expected_torque = state.rotations[2].T @ torque[2] - .2 * state.omega_body[2]
        np.testing.assert_allclose(body_torque[2], expected_torque)
        np.testing.assert_allclose(linear[2], (force[2] - 3 * state.velocity_world[2]) / config.mass)
        np.testing.assert_allclose(inertia * angular[2] + np.cross(state.omega_body[2], inertia * state.omega_body[2]), expected_torque)
        np.testing.assert_allclose(net_force[2], config.mass * linear[2])

    def test_semi_implicit_velocity_position_and_so3_exponential(self):
        problem = self.problem(empty=True)
        state = NodeState.from_poses(self.initial)
        state.velocity_world[2] = [.02, -.03, .01]
        state.omega_body[2] = [.1, -.2, .05]
        linear, angular = np.zeros((5, 3)), np.zeros((5, 3))
        linear[2], angular[2] = [1., 2., -.5], [.4, -.3, .2]
        dt = .001
        out = semi_implicit_step(state, linear, angular, dt, PoseConstraints(problem), RayDynamicsConfig().inertia(self.half))
        expected_v = state.velocity_world[2] + dt * linear[2]
        expected_w = state.omega_body[2] + dt * angular[2]
        np.testing.assert_allclose(out.velocity_world[2], expected_v)
        np.testing.assert_allclose(out.omega_body[2], expected_w)
        np.testing.assert_allclose(out.positions[2], state.positions[2] + dt * expected_v)
        np.testing.assert_allclose(out.rotations[2], state.rotations[2] @ Rotation.from_rotvec(dt * expected_w).as_matrix(), atol=1e-14)
        np.testing.assert_array_equal(state.positions, self.initial[:, :3])

    def test_so3_spring_and_collision_wrenches_match_virtual_work(self):
        problem = self.problem()
        state = NodeState.from_poses(self.initial)
        state.transforms[1:-1, :3, 3] += np.random.default_rng(10).normal(0, .01, (3, 3))
        state.transforms[1:-1, :3, :3] = Rotation.from_rotvec(np.random.default_rng(13).normal(0, .12, (3, 3))).as_matrix()
        _, force, torque, selection = elastic_wrenches(problem, state)
        eps = 1e-7
        for j in problem.interior:
            for axis in range(3):
                changes = []
                for sign in (-1, 1):
                    moved = NodeState(state.transforms.copy(), state.velocity_world.copy(), state.omega_body.copy())
                    moved.transforms[j, axis, 3] += sign * eps
                    changes.append(elastic_wrenches(problem, moved, selection)[0])
                self.assertAlmostEqual((changes[1] - changes[0]) / (2 * eps), -force[j, axis], delta=2e-5)
                changes = []
                for sign in (-1, 1):
                    moved = NodeState(state.transforms.copy(), state.velocity_world.copy(), state.omega_body.copy())
                    increment = np.zeros(3)
                    increment[axis] = sign * eps
                    moved.transforms[j, :3, :3] = state.rotations[j] @ Rotation.from_rotvec(increment).as_matrix()
                    changes.append(elastic_wrenches(problem, moved, selection)[0])
                expected = -(state.rotations[j].T @ torque[j])[axis]
                self.assertAlmostEqual((changes[1] - changes[0]) / (2 * eps), expected, delta=2e-5)

    def test_integrates_and_records_states_without_optimizer(self):
        poses, dynamics, states = [], [], []
        config = RayDynamicsConfig(time_step=.002, final_time_step=.0005, schedule_steps=6)
        with patch.object(experiment, "minimize", side_effect=AssertionError("optimizer called")), patch.object(core.TrajectoryProblem, "cost_grad", side_effect=AssertionError("gradient optimizer called")):
            final, stats, history, _ = experiment.optimize_from_cloud(
                self.cloud, self.initial, self.half, .4, "full", 12,
                solver="ray_dynamics", dynamics_config=config, pose_history=poses,
                dynamics_history=dynamics, state_history=states, lateral_limit=.6,
            )
        self.assertEqual(stats["point_selection"]["method"], "rays")
        self.assertEqual(len(states), stats["iterations"] + 1)
        self.assertEqual(len(poses), len(states))
        self.assertGreater(history[0]["maximum_linear_acceleration"], 0)
        self.assertGreater(history[1]["kinetic_energy"], 0)
        steps = np.array([h["time_step"] for h in history[1:]])
        self.assertTrue(all(h["time_step"] <= h["scheduled_time_step"] * (1 + 1e-12) for h in history[1:]))
        self.assertLessEqual(steps[-1], config.final_time_step * (1 + 1e-12))
        for pose, record, twist in zip(poses, states, dynamics):
            rotations = record["transforms"][:, :3, :3]
            np.testing.assert_allclose(rotations.transpose(0, 2, 1) @ rotations, np.tile(np.eye(3), (5, 1, 1)), atol=1e-12)
            np.testing.assert_allclose(np.linalg.det(rotations), 1, atol=1e-12)
            np.testing.assert_allclose(record["transforms"][:, :3, 3], pose[:, :3])
            np.testing.assert_array_equal(twist[[0, -1], 6:], 0)
            np.testing.assert_array_equal(record["accelerations"][[0, -1]], 0)
        np.testing.assert_array_equal(final[[0, -1]], self.initial[[0, -1]])
        for previous, current in zip(states, states[1:]):
            before = NodeState(previous["transforms"], np.zeros((5, 3)), np.zeros((5, 3)))
            after = NodeState(current["transforms"], np.zeros((5, 3)), np.zeros((5, 3)))
            self.assertTrue(motion_within_limits(actual_motion(before, after), config))

    def test_large_acceleration_is_retried_before_boundary_projection(self):
        problem = self.problem(empty=True)
        state = NodeState.from_poses(self.initial)
        config = RayDynamicsConfig(time_step=.02, final_time_step=.002)
        force = np.zeros((5, 3))
        torque = np.zeros_like(force)
        force[2, 1] = 6000.
        linear, angular, _, _ = accelerations(state, force, torque, config, self.half)
        constraints = PoseConstraints(problem)
        prediction = predict_twist(state, linear, angular, .02, constraints, config.inertia(self.half))
        self.assertAlmostEqual(proposed_motion(prediction, .02)[0], 2.4)
        uncapped = semi_implicit_step(state, linear, angular, .02, constraints, config.inertia(self.half))
        self.assertAlmostEqual(uncapped.positions[2, 1], .6)

        def conservative_field(problem, candidate, selections=None):
            return -float(np.sum(force * candidate.positions)), force.copy(), torque.copy(), [np.empty(0, dtype=int) for _ in range(5)]

        records = []
        with patch("m20_3d_planner.ray_dynamics.elastic_wrenches", side_effect=conservative_field):
            result, stats = relax(problem, 1, config, lambda *args: records.append(args[-2]))
        self.assertGreater(stats["motion_limited_retries"], 0)
        self.assertLessEqual(result.positions[2, 1], .01)
        self.assertLess(records[-1]["time_step"], .02)
        self.assertLessEqual(records[-1]["maximum_proposed_translation_step"], .01)
        self.assertLessEqual(records[-1]["maximum_node_translation_step"], .01)
        dt = records[-1]["time_step"]
        self.assertAlmostEqual(result.velocity_world[2, 1], dt * 6000.)
        self.assertAlmostEqual(result.positions[2, 1], dt * result.velocity_world[2, 1])

    def test_rotation_limit_uses_unwrapped_angle_not_only_matrix_difference(self):
        state = NodeState.from_poses(self.initial)
        velocity = np.zeros((5, 3))
        omega = np.zeros_like(velocity)
        dt = .02
        omega[2, 2] = 2 * np.pi / dt
        candidate = NodeState(state.transforms.copy(), velocity, omega)
        candidate.transforms[2, :3, :3] = Rotation.from_rotvec(dt * omega[2]).as_matrix()
        self.assertLess(actual_motion(state, candidate)[1], 1e-12)
        self.assertAlmostEqual(proposed_motion((velocity, omega), dt)[1], 2 * np.pi)
        self.assertFalse(motion_within_limits(proposed_motion((velocity, omega), dt), RayDynamicsConfig()))

    def test_large_angular_acceleration_retries_and_keeps_newton_euler_update(self):
        problem = self.problem(empty=True)
        torque = np.zeros((5, 3)); torque[2, 2] = 10.
        config = RayDynamicsConfig(time_step=.02, final_time_step=.002)
        def field(problem, state, selections=None):
            potential = -10. * state.poses()[2, 5]
            return potential, np.zeros_like(torque), torque.copy(), [np.empty(0, dtype=int) for _ in range(5)]
        records = []
        with patch("m20_3d_planner.ray_dynamics.elastic_wrenches", side_effect=field):
            result, stats = relax(problem, 1, config, lambda *args: records.append(args[-2]))
        self.assertGreater(stats["motion_limited_retries"], 0)
        dt = records[-1]["time_step"]
        expected_omega = dt * 10. / config.inertia(self.half)[2]
        self.assertAlmostEqual(result.omega_body[2, 2], expected_omega)
        np.testing.assert_allclose(result.rotations[2], Rotation.from_rotvec([0., 0., dt * expected_omega]).as_matrix(), atol=1e-12)
        self.assertLessEqual(records[-1]["maximum_node_rotation_step"], config.max_rotation_step)

    def test_failed_motion_limit_does_not_accept_an_oversized_step(self):
        problem = self.problem(empty=True)
        force = np.zeros((5, 3)); force[2, 1] = 6000.
        def field(problem, state, selections=None):
            return -float(np.sum(force * state.positions)), force.copy(), np.zeros_like(force), [np.empty(0, dtype=int) for _ in range(5)]
        config = RayDynamicsConfig(time_step=.02, final_time_step=.001, minimum_time_step=.001, max_translation_step=1e-12)
        records = []
        with patch("m20_3d_planner.ray_dynamics.elastic_wrenches", side_effect=field):
            result, stats = relax(problem, 5, config, lambda *args: records.append(args[-2]))
        self.assertFalse(stats["formal_success"])
        self.assertEqual(stats["iterations"], 0)
        self.assertEqual(len(records), 1)
        np.testing.assert_array_equal(result.positions, self.initial[:, :3])

    def test_step_recovers_after_stiff_transient_without_exceeding_motion_limits(self):
        problem = self.problem(empty=True)
        def field(problem, state, selections=None):
            decay = np.exp(-state.positions[2, 1] / .001)
            force = np.zeros((5, 3)); force[2, 1] = 6000. * decay
            return 6. * decay, force, np.zeros_like(force), [np.empty(0, dtype=int) for _ in range(5)]
        config = RayDynamicsConfig(time_step=.02, final_time_step=.002, linear_damping=4, time_step_growth=1.1)
        records = []
        with patch("m20_3d_planner.ray_dynamics.elastic_wrenches", side_effect=field):
            _, stats = relax(problem, 40, config, lambda *args: records.append(args[-2]))
        dt = np.array([r["time_step"] for r in records[1:]])
        self.assertLess(dt[0], config.time_step)
        self.assertTrue(np.any(np.diff(dt) > 1e-12))
        self.assertGreater(stats["motion_limited_retries"], 0)
        for record in records:
            self.assertLessEqual(record["time_step"], record["scheduled_time_step"] * (1 + 1e-12))
            self.assertLessEqual(record["maximum_node_translation_step"], config.max_translation_step * (1 + 1e-12))
            self.assertLessEqual(record["maximum_node_rotation_step"], config.max_rotation_step * (1 + 1e-12))

    def test_locked_modes_bounds_and_fixed_endpoint_transforms(self):
        for mode in settings.MODES:
            problem = self.problem(mode)
            state = NodeState.from_poses(self.initial)
            constraints = PoseConstraints(problem)
            inertia = RayDynamicsConfig().inertia(self.half)
            for _ in range(3):
                state = semi_implicit_step(state, np.full((5, 3), 1e3), np.full((5, 3), 1e3), .01, constraints, inertia)
            poses = state.poses()
            np.testing.assert_array_equal(state.transforms[[0, -1]], constraints.initial.transforms[[0, -1]])
            locked = sorted(set(range(6)) - set(problem.free_dims))
            np.testing.assert_allclose(poses[:, locked], self.initial[:, locked], atol=1e-12)
            bounds = np.asarray(problem.bounds())
            self.assertTrue(np.all(problem.pack(poses) >= bounds[:, 0] - 1e-12))
            self.assertTrue(np.all(problem.pack(poses) <= bounds[:, 1] + 1e-12))

    def test_empty_scene_height_recovery_converges_with_sustained_equilibrium(self):
        problem = self.problem("translation", empty=True)
        problem.initial[:, 2] = .45
        problem.weights = core.PlannerWeights(**{
            name: (32. if name == "height" else 0.) for name in core.PlannerWeights.__dataclass_fields__
        })
        records = []
        state, stats = relax(problem, 2000, RayDynamicsConfig(
            time_step=.01, final_time_step=.005, linear_damping=12., settle_steps=4,
        ), lambda *args: records.append(args[-2]))
        self.assertTrue(stats["formal_success"])
        np.testing.assert_allclose(state.positions[1:-1, 2], .4, atol=4e-6)
        for record in records[-4:]:
            self.assertLessEqual(record["equilibrium_residual"], 1e-4)
            self.assertLessEqual(record["maximum_linear_speed"], 1e-4)

    def test_continuous_validation_checks_so3_path_and_bounds_between_samples(self):
        poses = np.array([[-.2, 0, .4, -.3, .15, -.1], [.2, 0, .4, .3, -.15, .1]])
        dense = core.densify_trajectory(poses, 2, "so3")
        start, end = Rotation.from_euler("xyz", poses[:, 3:]).as_matrix()
        expected_mid = start @ Rotation.from_rotvec(.5 * Rotation.from_matrix(start.T @ end).as_rotvec()).as_matrix()
        np.testing.assert_allclose(core.rpy_matrix(dense[1, 3:]), expected_mid, atol=1e-12)
        truth = core.horizontal_plane((-1, 1), (-1, 1), 0., .022)[0]
        coarse = experiment.dense_validation(poses, truth, self.half, 4, "so3")
        fine = experiment.dense_validation(poses, truth, self.half, 100, "so3")
        self.assertLessEqual(coarse["continuous_lower_bound_against_truth_points_m"], fine["minimum_signed_distance_m"])
        self.assertEqual(coarse["rotation_interpolation"], "so3")

    def test_configuration_and_ray_requirement(self):
        config = RayDynamicsConfig(time_step=.02, final_time_step=.001, schedule_steps=5)
        values = [config.scheduled_time_step(i) for i in range(9)]
        self.assertAlmostEqual(values[0], .02)
        self.assertAlmostEqual(values[-1], .001)
        self.assertTrue(np.all(np.diff(values) <= 0))
        for kwargs in ({"mass": 0}, {"linear_damping": -1}, {"final_time_step": .1}, {"schedule_steps": 0}, {"settle_steps": True}, {"time_step": float("nan")}, {"max_translation_step": 0}, {"max_rotation_step": float("nan")}, {"time_step_growth": .9}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                RayDynamicsConfig(**kwargs)
        with self.assertRaisesRegex(ValueError, "requires"):
            experiment.optimize_from_cloud(self.cloud, self.initial, self.half, .4, "full", 1, solver="ray_dynamics", point_selection=PointSelection("all"))

    def test_tuning_weights_merge_baseline_and_reject_invalid_values(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "weights.json"
            path.write_text(json.dumps({"path_xy": 2., "attitude": 30.}))
            weights = experiment.load_planner_weights(path)
            self.assertEqual(weights.path_xy, 2.)
            self.assertEqual(weights.attitude, 30.)
            self.assertEqual(weights.collision_stiffness, settings.planner_weights().collision_stiffness)
            for invalid in ({"unknown": 1}, {"path_xy": -1}, {"attitude": True}, {"height": "32"}, []):
                path.write_text(json.dumps(invalid))
                with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                    experiment.load_planner_weights(path)

    def test_tuned_weights_and_activation_reach_solver_and_record(self):
        weights = replace(settings.planner_weights(), attitude=30., path_xy=2., acceleration_translation=300.)
        config = RayDynamicsConfig(time_step=.002, final_time_step=.001)
        _, stats, history, _ = experiment.optimize_from_cloud(
            self.cloud, self.initial, self.half, .4, "full", 2,
            solver="ray_dynamics", dynamics_config=config, weights=weights, activation_distance=.07,
        )
        self.assertEqual(stats["weights"], asdict(weights))
        self.assertEqual(stats["activation_distance_m"], .07)
        self.assertGreater(history[0]["maximum_linear_acceleration"], 0)
        with self.assertRaises(ValueError):
            experiment.optimize_from_cloud(self.cloud, self.initial, self.half, .4, "full", 1, activation_distance=-1)


if __name__ == "__main__":
    unittest.main()

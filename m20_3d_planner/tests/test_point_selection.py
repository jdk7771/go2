"""Selection geometry, unchanged wrench law and conservative range pruning."""

import unittest

from m20_3d_planner import experiment, core, settings
from m20_3d_planner.point_selection import BodyCenterSelector, PointSelection

import numpy as np


class PointSelectionChecks(unittest.TestCase):
    def test_center_origin_first_hit_full_sphere_and_max_length(self):
        center = np.array([1.7, -0.4, 0.8])
        azimuth = np.radians([1, 91, 181, 271, 37, 237])
        elevation = np.radians([1, 1, 1, 1, 81, -81])
        rays = np.column_stack(
            (
                np.cos(elevation) * np.cos(azimuth),
                np.cos(elevation) * np.sin(azimuth),
                np.sin(elevation),
            )
        )
        cloud = center + np.vstack((0.2 * rays, 0.4 * rays, 0.7 * rays))
        selector = BodyCenterSelector(cloud, PointSelection("rays", 0.6, 4))
        ids = selector.select(center[None])[0]
        np.testing.assert_array_equal(ids, np.arange(6))
        # Moving the origin changes the first-hit/range calculation.
        self.assertEqual(len(selector.select(np.array([[10.0, 10.0, 10.0]]))[0]), 0)

    def test_range_boundary_empty_cloud_and_ties(self):
        cloud = np.array([[0.6, 0.0, 0.0], [0.60001, 0.0, 0.0], [0.6, 0.0, 0.0]])
        selector = BodyCenterSelector(cloud, PointSelection("rays", 0.6, 4))
        np.testing.assert_array_equal(selector.select(np.zeros((1, 3)))[0], [0])
        empty = BodyCenterSelector(np.empty((0, 3)), PointSelection("rays"))
        self.assertEqual(len(empty.select(np.zeros((1, 3)))[0]), 0)
        energy, force, torque, distance, active = core.collision_energy_wrench(
            cloud[:0], np.empty(0), np.zeros(3), np.eye(3), np.ones(3), 0.047, 1e6
        )
        self.assertEqual((energy, active), (0, 0))
        self.assertEqual(distance, float("inf"))
        np.testing.assert_array_equal(np.r_[force, torque], 0.0)

    def test_radius_preserves_full_energy_and_gradient_for_all_rotations(self):
        rng = np.random.default_rng(803)
        cloud = rng.uniform([-1.8, -0.8, -0.2], [1.8, 0.8, 1.2], (12000, 3))
        initial = core.make_initial_trajectory(7, 0.4)
        half = np.array([0.375, 0.115, 0.07])
        self.assertGreater(0.6, np.linalg.norm(half) + settings.PLANNING_CLEARANCE)
        problems = [
            core.TrajectoryProblem(
                cloud,
                np.full(len(cloud), settings.POINT_VOXEL**2),
                initial,
                half,
                0.4,
                settings.PLANNING_CLEARANCE,
                "full",
                settings.planner_weights(),
                PointSelection(method),
            )
            for method in ("all", "radius")
        ]
        poses = initial.copy()
        poses[1:-1, :3] += rng.normal(0, 0.02, (5, 3))
        poses[1:-1, 3:] = rng.uniform(-0.3, 0.3, (5, 3))
        x = problems[0].pack(poses)
        e0, g0 = problems[0].cost_grad(x)
        e1, g1 = problems[1].cost_grad(x)
        np.testing.assert_allclose(e0, e1, rtol=1e-14)
        np.testing.assert_allclose(g0, g1, rtol=1e-13, atol=1e-10)

    def test_ray_wrench_keeps_surface_lever_arm_and_piecewise_gradient(self):
        # Two collinear points: select the nearer one, which applies an
        # off-center force on the top-right box face, yielding nonzero torque.
        point = np.array([0.223, 0.126, 0.483])
        center = np.array([0.0, 0.0, 0.4])
        cloud = np.array([point, center + 1.05 * (point - center)])
        initial = core.make_initial_trajectory(3, 0.4)
        problem = core.TrajectoryProblem(
            cloud,
            np.full(2, settings.POINT_VOXEL**2),
            initial,
            np.array([0.375, 0.115, 0.07]),
            0.4,
            settings.PLANNING_CLEARANCE,
            "full",
            settings.planner_weights(),
            PointSelection("rays", 0.6, 10),
        )
        x = problem.pack(initial)
        _, analytic = problem.cost_grad(x)
        numerical = np.zeros_like(x)
        for j in range(len(x)):
            delta = np.zeros_like(x)
            delta[j] = 1e-7
            numerical[j] = (
                problem.cost_grad(x + delta)[0] - problem.cost_grad(x - delta)[0]
            ) / 2e-7
        np.testing.assert_allclose(analytic, numerical, rtol=3e-5, atol=2e-5)
        self.assertGreater(np.linalg.norm(analytic[3:]), 1.0)
        snapshots = []
        _, stats, _, _ = experiment.optimize_from_cloud(
            cloud,
            initial,
            problem.half,
            0.4,
            "full",
            1,
            snapshots,
            point_selection=PointSelection("rays", 0.6, 10),
        )
        self.assertGreater(stats["selection_query_seconds"], 0.0)
        self.assertGreaterEqual(stats["total_planning_seconds"], stats["wall_seconds"])

    def test_invalid_configuration(self):
        for kwargs in (
            {"method": "bad"},
            {"max_length": 0},
            {"max_length": float("nan")},
            {"angle_deg": 0},
        ):
            with self.assertRaises(ValueError):
                PointSelection(**kwargs)


if __name__ == "__main__":
    unittest.main()

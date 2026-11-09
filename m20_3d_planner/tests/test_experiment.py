"""Numerical checks for the experiment's gradients and scoring conventions."""

import math
import unittest

import numpy as np

from m20_3d_planner import core, scenes, settings as suite, experiment as ablation


class AblationChecks(unittest.TestCase):
    def setUp(self):
        self.half = np.array([0.375, 0.115, 0.07])
        self.initial = core.make_initial_trajectory(5, 0.4)
        rng = np.random.default_rng(42)
        # Asymmetric points near the upper-right body edge avoid exact closest-
        # face ties while exercising translation and all three angle gradients.
        self.cloud = rng.uniform([0.05, 0.12, 0.44], [0.32, 0.15, 0.49], (40, 3))

    def problem(self, mode):
        return core.TrajectoryProblem(
            self.cloud,
            np.full(len(self.cloud), suite.POINT_VOXEL**2),
            self.initial,
            self.half,
            0.4,
            suite.PLANNING_CLEARANCE,
            mode,
            suite.planner_weights(),
        )

    def test_all_mode_gradients_match_finite_differences(self):
        for mode in ablation.MODES:
            with self.subTest(mode=mode):
                problem = self.problem(mode)
                rng = np.random.default_rng(13)
                x = problem.pack(self.initial) + rng.normal(
                    0, 0.001, size=problem.pack(self.initial).shape
                )
                _, analytic = problem.cost_grad(x)
                numerical = np.zeros_like(x)
                for j in range(len(x)):
                    delta = np.zeros_like(x)
                    delta[j] = 1e-7
                    numerical[j] = (
                        problem.cost_grad(x + delta)[0]
                        - problem.cost_grad(x - delta)[0]
                    ) / 2e-7
                np.testing.assert_allclose(analytic, numerical, rtol=3e-5, atol=2e-5)

    def test_fixed_dimensions_and_endpoints(self):
        for mode, fixed in (
            ("translation", [3, 4, 5]),
            ("translation_yaw", [3, 4]),
            ("full", []),
        ):
            problem = self.problem(mode)
            poses = problem.unpack(problem.pack(self.initial) + 0.001)
            np.testing.assert_array_equal(poses[[0, -1]], self.initial[[0, -1]])
            np.testing.assert_array_equal(poses[:, fixed], self.initial[:, fixed])
        with self.assertRaises(ValueError):
            self.problem("typo")

    def test_projected_gradient_at_active_bounds(self):
        bounds = np.tile([0.0, 1.0], (4, 1))
        actual = ablation.projected_gradient(
            np.array([0.0, 1.0, 0.0, 1.0]), np.array([2.0, -2.0, -0.2, 0.3]), bounds
        )
        np.testing.assert_allclose(actual, [0.0, 0.0, -0.2, 0.3])

    def test_uniform_start_does_not_call_scene_seed(self):
        from unittest.mock import patch

        with patch.object(
            ablation.suite,
            "build_scenes",
            side_effect=AssertionError("scene metadata used"),
        ):
            poses, opt, history, _ = ablation.optimize_from_cloud(
                self.cloud,
                self.initial,
                self.half,
                0.4,
                "translation_yaw",
                maxiter=1,
            )
        self.assertEqual(history[0]["minimum_base_z_m"], 0.4)
        for name in ("roll", "pitch", "yaw"):
            self.assertEqual(history[0][f"max_abs_{name}_deg"], 0.0)
        np.testing.assert_array_equal(poses[:, 3:5], 0.0)
        self.assertEqual(opt["free_dimensions"], [0, 1, 2, 5])

    def test_strict_20mm_does_not_use_legacy_tolerance(self):
        poses = core.make_initial_trajectory(2, 0.4)
        # A horizontal plane exactly 19 mm above the body.
        plane = core.horizontal_plane((-2, 2), (-0.5, 0.5), 0.489, 0.01)[0]
        valid = ablation.dense_validation(poses, plane, self.half, subdivisions=2)
        self.assertTrue(valid["collision_free_strict"])
        self.assertFalse(valid["clearance_20mm_strict"])
        self.assertTrue(valid["legacy_clearance_with_1p5mm_tolerance"])

    def test_continuous_point_bound_is_conservative(self):
        poses = np.array(
            [[-0.2, 0.0, 0.39, -0.1, 0.02, -0.02], [0.2, 0.01, 0.41, 0.15, -0.03, 0.03]]
        )
        plane = core.horizontal_plane((-1, 1), (-1, 1), 0.0, 0.022)[0]
        coarse = ablation.dense_validation(poses, plane, self.half, subdivisions=4)
        dense = ablation.dense_validation(poses, plane, self.half, subdivisions=150)
        self.assertLessEqual(
            coarse["continuous_lower_bound_against_truth_points_m"],
            dense["minimum_signed_distance_m"],
        )
        self.assertAlmostEqual(
            coarse["synthetic_surface_cover_radius_m"], math.sqrt(2) * 0.011
        )

    def test_bent_floor_is_transverse_continuous_and_nonparallel(self):
        scene = scenes.nonparallel_scenes(0.4)[1]
        profile = np.array(scene.params["floor_profile_yz_m"])
        slopes = np.diff(profile[:, 1]) / np.diff(profile[:, 0])
        np.testing.assert_allclose(slopes, [0.0, math.tan(math.pi / 6)])
        self.assertAlmostEqual(scene.params["join_interior_angle_deg"], 150.0)
        roof = np.array(scene.params["cross_section"][0]["yz"])
        self.assertEqual(roof[0, 1], roof[1, 1])
        # At both x ends the raised terrain has caps, not an open underside.
        for x in (-0.62, 0.62):
            cap = scene.truth_surface[np.isclose(scene.truth_surface[:, 0], x)]
            self.assertTrue(np.any((cap[:, 2] > 0.05) & (cap[:, 2] < 0.25)))
        # The same join is present on the top at an interior x station.
        join = scene.truth_surface[
            np.isclose(scene.truth_surface[:, 1], profile[1, 0])
            & np.isclose(scene.truth_surface[:, 2], profile[1, 1])
        ]
        self.assertTrue(np.any(np.abs(join[:, 0]) < 0.02))

    def test_closed_floor_sampling_respects_validation_cover_radius(self):
        from scipy.spatial import cKDTree

        profile = np.array(
            [[-0.62, 0.29], [-0.03, 0.29], [0.3, 0.29 + 0.33 * math.tan(math.pi / 6)]]
        )
        cloud = scenes.extruded_floor_profile(profile, (-0.62, 0.62), 0, 0.0075)
        rng = np.random.default_rng(23)
        y = rng.uniform(profile[0, 0], profile[-1, 0], 3000)
        height = np.interp(y, profile[:, 0], profile[:, 1])
        top = np.column_stack((rng.uniform(-0.62, 0.62, len(y)), y, height))
        cap = np.column_stack(
            (rng.choice([-0.62, 0.62], len(y)), y, rng.uniform(0, height))
        )
        distance = cKDTree(cloud).query(np.vstack((top, cap)))[0]
        self.assertLess(distance.max(), math.sqrt(2) * 0.011)

    def test_open_entry_channel_has_clear_start_and_goal_and_a_level_floor(self):
        scene = scenes.open_entry_channel(0.4)
        self.assertEqual(scene.params["ground_height_m"], 0.0)
        self.assertEqual(scene.params["roof_slope_deg"], 30.0)
        self.assertEqual(scene.params["side_inner_y_m"], [-0.22, 0.22])
        initial = scenes.initial_trajectory(0.4)
        for pose in initial[[0, -1]]:
            distance = core.obb_signed_distance(
                scene.truth_surface, pose[:3], np.eye(3), self.half
            )[0].min()
            self.assertGreater(distance, 0.02)
        center = initial[len(initial) // 2]
        distance = core.obb_signed_distance(
            scene.truth_surface, center[:3], np.eye(3), self.half
        )[0].min()
        self.assertLess(distance, 0.0)


if __name__ == "__main__":
    unittest.main()

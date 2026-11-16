import unittest
import numpy as np

from m20_3d_planner import core, scenes
from m20_3d_planner.initialization import cloud_bow_initialization


class CurvedInitializationChecks(unittest.TestCase):
    def setUp(self):
        self.reference = core.make_initial_trajectory(23, .4)
        self.half = np.array([.375, .115, .07])

    def test_asymmetric_obstacle_selects_clear_detour_and_mirrors_direction(self):
        cloud = scenes.box_surface(np.array([0., .08, .7]),
                                   np.array([.34, .44, 1.4]), np.zeros(3), .04)
        for sign in (1, -1):
            mirrored = cloud.copy()
            mirrored[:, 1] *= sign
            poses, info = cloud_bow_initialization(
                mirrored, self.reference, self.half, .6, .035, allow_roll=False
            )
            self.assertLess(sign * info["selected"]["lateral_amplitude_m"], 0)
            np.testing.assert_array_equal(poses[[0, -1]], self.reference[[0, -1]])
            np.testing.assert_array_equal(poses[:, 3:], 0.)
            self.assertLessEqual(np.abs(poses[:, 1]).max(), .6)
            self.assertGreaterEqual(poses[:, 2].min(), .28)
            self.assertLessEqual(poses[:, 2].max(), .52)
            distances = [core.obb_signed_distance(mirrored, q[:3], np.eye(3), self.half)[0].min()
                         for q in core.densify_trajectory(poses, 20, "so3")]
            self.assertGreater(min(distances), .02)

    def test_empty_cloud_still_produces_finite_nonstraight_band(self):
        poses, info = cloud_bow_initialization(
            np.empty((0, 3)), self.reference, self.half, .16, .035, allow_roll=False
        )
        self.assertTrue(np.isfinite(poses).all())
        self.assertFalse(np.array_equal(poses, self.reference))
        self.assertFalse(info["truth_used"])
        self.assertFalse(info["scene_metadata_used"])
        self.assertIsNone(info["selected"]["sampled_cloud_minimum_m"])

    def test_rejects_invalid_inputs(self):
        for limit in (0, -1, np.inf):
            with self.assertRaises(ValueError):
                cloud_bow_initialization(np.empty((0, 3)), self.reference, self.half, limit, .035)

    def test_narrow_tilted_passage_keeps_small_bend_and_uses_roll(self):
        cloud = core.tilted_corridor(np.pi / 6, .102, .04)[0]
        poses, info = cloud_bow_initialization(cloud, self.reference, self.half, .6, .02)
        self.assertLessEqual(abs(info["selected"]["lateral_amplitude_m"]), .06)
        self.assertLessEqual(abs(info["selected"]["vertical_amplitude_m"]), .04)
        self.assertGreater(poses[:, 3].max(), np.radians(15))
        clearance = [core.obb_signed_distance(cloud, q[:3], core.rpy_matrix(q[3:]), self.half)[0].min()
                     for q in core.densify_trajectory(poses, 20, "so3")]
        # This finite candidate is a starting route, not a 20 mm certificate.
        self.assertGreater(min(clearance), 0.)


if __name__ == "__main__":
    unittest.main()

"""Independent geometric/numerical checks and temporal registration regressions."""

from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import numpy as np

from fsgp_bgk_repro.core import (Config, SparseTerrainGP, TraversabilityMapper,
                                BGKHistory, bgk_update, extract_features, smooth_known)
from fsgp_bgk_repro.io import read_cloud


def plane(slope=(0.0, 0.0), spacing=0.15):
    x, y = np.meshgrid(np.arange(-3, 3.01, spacing), np.arange(-3, 3.01, spacing))
    return np.column_stack((x.ravel(), y.ravel(), (slope[0] * x + slope[1] * y).ravel()))


class FeatureTests(unittest.TestCase):
    def test_inclined_plane_has_zero_curvature_and_nonzero_height_variation(self):
        features = extract_features(plane((0.25, 0.10)))
        self.assertLess(np.max(np.abs(features[:, 3])), 1e-12)
        self.assertGreater(np.median(features[:, 4]), 0.01)

    def test_translation_and_rotation_about_gravity_preserve_features(self):
        points = plane((0.2, 0.1), spacing=0.19)
        # Use irregular points to avoid equidistant KNN ties after rotation.
        rng = np.random.default_rng(2)
        points[:, :2] += rng.normal(0, 0.01, (len(points), 2))
        points[:, 2] = points[:, 0] * 0.2 + points[:, 1] * 0.1
        rotation = np.array([[0.8, -0.6, 0], [0.6, 0.8, 0], [0, 0, 1]])
        first = extract_features(points)
        second = extract_features(points @ rotation.T + [3, -7, 1])
        np.testing.assert_allclose(first[:, 3:], second[:, 3:], atol=1e-12)

    def test_three_points_and_nonfinite_input(self):
        data = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [np.nan, 0, 0]])
        self.assertEqual(extract_features(data).shape, (3, 5))
        with self.assertRaises(ValueError):
            extract_features(data[:2])


class GPTests(unittest.TestCase):
    def test_analytic_derivative_matches_finite_difference_in_world_xy(self):
        rng = np.random.default_rng(9)
        train = rng.normal(size=(30, 4))
        height = np.sin(train[:, 0]) + 0.3 * train[:, 1]
        model = SparseTerrainGP(Config()).fit(train, height)
        query = rng.normal(size=(7, 4))
        _, variance, gradient = model.predict(query)
        for axis in (0, 1):
            delta = np.zeros_like(query)
            delta[:, axis] = 1e-5
            plus = model.predict(query + delta)[0]
            minus = model.predict(query - delta)[0]
            np.testing.assert_allclose(gradient[:, axis], (plus - minus) / 2e-5,
                                       atol=1e-7, rtol=1e-5)
        self.assertTrue(np.all(variance >= 0))

    def test_pca_rotation_does_not_change_isotropic_kernel_predictions(self):
        rng = np.random.default_rng(4)
        train, query = rng.normal(size=(20, 4)), rng.normal(size=(5, 4))
        heights = rng.normal(size=20)
        rotated = SparseTerrainGP(Config(pca=True)).fit(train, heights).predict(query)
        original = SparseTerrainGP(Config(pca=False)).fit(train, heights).predict(query)
        for expected, actual in zip(original, rotated):
            np.testing.assert_allclose(expected, actual, atol=1e-10)

    def test_observations_recovered_with_low_noise(self):
        rng = np.random.default_rng(5)
        train = rng.normal(size=(20, 2))
        heights = 0.2 * train[:, 0] - 0.1 * train[:, 1]
        config = Config(noise_variance=1e-8)
        predicted, variance, _ = SparseTerrainGP(config).fit(train, heights).predict(train)
        np.testing.assert_allclose(predicted, heights, atol=1e-5)
        self.assertLess(variance.max(), 3e-8)

    def test_flat_map_is_zero_cost_and_has_bounded_inducing_count(self):
        config = Config(inducing_points=50)
        mapper = TraversabilityMapper(config)
        result = mapper.fuse(mapper.assess(plane()))
        np.testing.assert_allclose(result["cost"][np.isfinite(result["cost"])], 0, atol=1e-12)
        self.assertLessEqual(len(result["inducing"]), 50)

    def test_analytic_ramp_height_and_slope_in_interior(self):
        config = Config(inducing_points=500, noise_variance=1e-6, lengthscale=1.0)
        result = TraversabilityMapper(config).assess(plane((0.25, 0.10)))
        interior = np.max(np.abs(result["xy"]), axis=1) < 1.3
        expected_height = result["xy"][:, 0] * 0.25 + result["xy"][:, 1] * 0.10
        self.assertLess(np.mean(np.abs(result["height"][interior] - expected_height[interior])), 0.01)
        self.assertLess(np.mean(np.abs(result["slope"][interior] - np.hypot(0.25, 0.10))), 0.025)


class HistoryTests(unittest.TestCase):
    def test_fusion_confidence_and_long_time_limit(self):
        recent, _ = bgk_update(0.8, 0.01, 0, 0.2, 0.1, 1.0)
        old, _ = bgk_update(0.8, 0.01, 100, 0.2, 0.1, 1.0)
        self.assertGreater(recent, 0.79)
        self.assertAlmostEqual(old, 0.2)

    def test_unknown_has_nan_cost_and_does_not_refresh_history(self):
        history = BGKHistory(Config(history_ttl=1.0))
        ids = np.array([[0, 0], [1, 0]])
        value, _, known = history.fuse(ids, [0.3, 0.7], [0.1, 0.1], [True, False], 0)
        self.assertTrue(known[0])
        self.assertTrue(np.isnan(value[1]))
        value, _, known = history.fuse(ids, [0.9, 0.9], [0.1, 0.1], [False, False], 0.5)
        self.assertAlmostEqual(value[0], 0.3)
        self.assertEqual(history.cells[(0, 0)][2], 0)
        _, _, known = history.fuse(ids, [0, 0], [0.1, 0.1], [False, False], 1.1)
        self.assertFalse(np.any(known))

    def test_world_cells_follow_negative_coordinates_and_rolling_window(self):
        mapper = TraversabilityMapper(Config(length=1.0))
        first_xy, first_ids, _ = mapper.grid((-0.05, 0))
        _, second_ids, _ = mapper.grid((0.15, 0))
        selected = np.flatnonzero(np.all(first_ids == [-1, 0], axis=1))[0]
        cost = np.zeros(len(first_ids)); cost[selected] = 0.8
        observed = np.zeros(len(first_ids), dtype=bool); observed[selected] = True
        mapper.history.fuse(first_ids, cost, np.full(len(cost), 0.1), observed, 0)
        fused, _, known = mapper.history.fuse(second_ids, cost, np.full(len(cost), 0.1),
                                             np.zeros(len(cost), dtype=bool), 0.1)
        moved = np.flatnonzero(np.all(second_ids == [-1, 0], axis=1))[0]
        self.assertTrue(known[moved])
        self.assertAlmostEqual(fused[moved], 0.8)
        np.testing.assert_allclose(first_xy[selected], [-0.1, 0.1])
        far_xy, far_ids, _ = mapper.grid((100, 100))
        mapper.history.fuse(far_ids, cost, np.full(len(cost), 0.1), np.zeros(len(cost), dtype=bool), 1)
        self.assertFalse(mapper.history.cells)

    def test_time_reversal_is_rejected(self):
        history = BGKHistory(Config())
        history.fuse(np.array([[0, 0]]), [1], [1], [True], 2)
        with self.assertRaises(ValueError):
            history.fuse(np.array([[0, 0]]), [1], [1], [True], 1)

    def test_masked_spatial_filter_preserves_constant_and_unknown(self):
        values = np.full((5, 7), 0.4)
        known = np.ones_like(values, dtype=bool); known[2, 3] = False
        values[2, 3] = np.nan
        result = smooth_known(values, known, 1.0)
        np.testing.assert_allclose(result[known], 0.4)
        self.assertTrue(np.isnan(result[2, 3]))


class ReaderTests(unittest.TestCase):
    def test_binary_pcd_with_extra_fields(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "test.pcd"
            header = ("VERSION 0.7\nFIELDS intensity z x y\nSIZE 4 4 4 4\nTYPE F F F F\n"
                      "COUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n")
            payload = np.array([[7, 3, 1, 2], [8, 6, 4, 5]], dtype="<f4").tobytes()
            path.write_bytes(header.encode() + payload)
            np.testing.assert_array_equal(read_cloud(path), [[1, 2, 3], [4, 5, 6]])
            path.write_bytes(header.encode() + payload[:-1])
            with self.assertRaises(ValueError):
                read_cloud(path)

    def test_ascii_and_npz(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "test.pcd"
            path.write_text("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                            "POINTS 2\nDATA ascii\n1 2 3\n4 5 6\n")
            points = read_cloud(path)
            target = Path(temp) / "cloud.npz"
            np.savez(target, points=points)
            np.testing.assert_array_equal(points, read_cloud(target))


if __name__ == "__main__":
    unittest.main()

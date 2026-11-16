"""Regressions for neighbour contamination and stacked height graph geometry."""
import unittest
import numpy as np
from fsgp_bgk_repro.table_multilayer import (
    SETTINGS, assign_layers, fit_layer, fit_multilayer, grid_triangles)


class MultilayerTests(unittest.TestCase):
    def test_three_overlapping_noisy_layers_are_preserved(self):
        rng = np.random.default_rng(53)
        xy = rng.uniform(-.45, .45, (250, 2))
        points = np.concatenate([np.column_stack((xy, z+rng.normal(0, .003, len(xy))))
                                 for z in (0., 1.1, 1.2)])
        centres, labels, fits = fit_multilayer(points, np.array([[0., 0.]]))
        self.assertEqual(len(fits), 3)
        np.testing.assert_allclose(centres, [0., 1.1, 1.2], atol=.002)
        for i, fit in enumerate(fits):
            self.assertTrue(np.isfinite(fit["height"][0]))
            self.assertEqual(len(fit["neighbor_ids"][0]), 30)
            self.assertTrue(np.all(labels[fit["neighbor_ids"][0]] == i))
            self.assertLess(abs(fit["height"][0]-centres[i]), .005)

    def test_height_filter_happens_before_knn_and_recovers_quadratic(self):
        rng = np.random.default_rng(71)
        xy = rng.uniform(-.3, .3, (150, 2))
        z = .01+.02*xy[:, 0]-.015*xy[:, 1]+.04*xy[:, 0]**2+.02*xy[:, 0]*xy[:, 1]
        correct = np.column_stack((xy, z))
        # A nearer, wrong-height group would exhaust an ungated KNN query.
        wrong = np.column_stack((rng.uniform(-.02, .02, (200, 2)), np.full(200, .1)))
        points = np.concatenate((correct, wrong))
        fit = fit_layer(points, np.array([[0., 0.]]), np.arange(len(points)), .01, .04)
        self.assertTrue(np.all(fit["neighbor_ids"][0] < len(correct)))
        self.assertAlmostEqual(fit["height"][0], .01, places=8)

    def test_29_points_cannot_borrow_from_another_layer(self):
        rng = np.random.default_rng(5)
        xy = rng.uniform(-.2, .2, (29, 2))
        points = np.concatenate([np.column_stack((xy, np.full(29, z))) for z in (0., 1.1, 1.2)])
        labels = assign_layers(points, np.array([0., 1.1, 1.2]), .04)
        for i, z in enumerate((0., 1.1, 1.2)):
            fit = fit_layer(points, np.array([[0., 0.]]), np.flatnonzero(labels == i), z, .04)
            self.assertEqual(fit["neighbor_count"][0], 29)
            self.assertTrue(np.isnan(fit["height"][0]))

    def test_radius_and_hull_prevent_false_surface_outside_observations(self):
        rng = np.random.default_rng(14)
        xy = rng.uniform([.03, -.3], [.3, .3], (100, 2))
        points = np.column_stack((xy, np.zeros(len(xy))))
        fit = fit_layer(points, np.array([[0., 0.], [1., 1.], [.15, 0.]]), np.arange(len(points)), 0., .04)
        self.assertEqual(fit["neighbor_count"][0], 30)
        self.assertFalse(fit["inside_hull"][0])
        self.assertTrue(np.isnan(fit["height"][0]))
        self.assertEqual(fit["neighbor_count"][1], 0)
        self.assertTrue(np.isnan(fit["height"][1]))
        self.assertTrue(np.isfinite(fit["height"][2]))

    def test_grid_does_not_connect_missing_values_or_height_jumps(self):
        for z in (0., 1.1, 1.2):
            self.assertEqual(len(grid_triangles(np.full(9, z), 3, 3)), 8)
        z = np.array([0., 0., 1.1, 0., np.nan, 1.1, 0., 0., 1.1])
        triangles = grid_triangles(z, 3, 3)
        self.assertTrue(np.all(np.isfinite(z[triangles])))
        self.assertTrue(np.all(np.ptp(z[triangles], axis=1) <= SETTINGS["max_triangle_height_jump_m"]))


if __name__ == "__main__":
    unittest.main()

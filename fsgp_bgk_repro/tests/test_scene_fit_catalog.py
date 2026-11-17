import unittest
import numpy as np

from fsgp_bgk_repro.scene_fit_catalog import exact_neighbors, fit_centers
from fsgp_bgk_repro.table_local_surfaces import LocalConfig, fit_seeds, evaluate_models


class CatalogFitTests(unittest.TestCase):
    def test_exact_neighbor_and_coefficient_parity_with_existing_method(self):
        # Tied distances, overlapping surfaces and insufficient neighborhoods.
        x, y = np.meshgrid(np.linspace(-.8, .8, 19), np.linspace(-.8, .8, 19))
        xy = np.column_stack((x.ravel(), y.ravel()))
        points = np.concatenate([np.column_stack((xy, np.full(len(xy), z))) for z in (0., .4, .52)])
        config = LocalConfig()
        old = fit_seeds(points, config)
        ids = exact_neighbors(points, points, config, batch_size=97)
        np.testing.assert_array_equal(ids, old["neighbor_ids"])
        new = fit_centers(points, points, ids, config)
        for key in ("coefficient", "height", "rmse", "full_rank"):
            np.testing.assert_allclose(new[key], old[key], atol=1e-12, equal_nan=True)

    def test_polynomial_recovery_and_full_support_outside_seed_subset(self):
        x, y = np.meshgrid(np.linspace(-.5, .5, 41), np.linspace(-.5, .5, 41))
        xy = np.column_stack((x.ravel(), y.ravel()))
        z = .1*xy[:, 0]**2 + .04*xy[:, 0]*xy[:, 1] + .2*xy[:, 1]**2 + .05*xy[:, 0]
        points = np.column_stack((xy, z))
        seed_ids = np.array([0, 300, 840, 1200, 1680])
        seeds = points[seed_ids]
        config = LocalConfig()
        ids = exact_neighbors(points, seeds, config)
        self.assertTrue((ids >= 0).all())
        self.assertTrue(np.any(~np.isin(ids, seed_ids)))
        fit = fit_centers(points, seeds, ids, config)
        prediction = evaluate_models(seeds, fit["coefficient"], np.arange(5), seeds[:, :2])
        np.testing.assert_allclose(prediction, seeds[:, 2], atol=1e-8)
        self.assertLess(np.nanmax(fit["rmse"]), 1e-8)

    def test_height_gate_does_not_borrow_points_from_other_roof(self):
        rng = np.random.default_rng(2026)
        xy = rng.uniform(-.1, .1, (29, 2))
        points = np.concatenate([np.column_stack((xy, np.full(29, z))) for z in (0., .4, .52)])
        ids = exact_neighbors(points, points[[0, 29, 58]], LocalConfig())
        self.assertTrue((ids == -1).all())


if __name__ == "__main__":
    unittest.main()

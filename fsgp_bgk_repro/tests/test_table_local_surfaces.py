import unittest
import numpy as np
from fsgp_bgk_repro.table_local_surfaces import LocalConfig, reconstruct


class LocalSurfaceTests(unittest.TestCase):
    def plane_cloud(self, levels, tilt=(0., 0.), n=24):
        rng = np.random.default_rng(42)
        x, y = np.meshgrid(np.linspace(-.8, .8, n), np.linspace(-.8, .8, n))
        xy = np.column_stack((x.ravel(), y.ravel()))
        return np.concatenate([np.column_stack((xy, z+xy@np.asarray(tilt)+rng.normal(0, .0005, len(xy)))) for z in levels])

    def test_surface_count_is_not_fixed_to_three(self):
        for levels in ([.23], [.23, .87], [.23, .45, .67, .89]):
            points = self.plane_cloud(levels)
            result = reconstruct(points)
            self.assertEqual(len(result["components"]), len(levels))
            inferred = sorted(np.median(result["fits"]["height"][c["point_ids"]]) for c in result["components"])
            np.testing.assert_allclose(inferred, levels, atol=.001)

    def test_overlapping_tilted_surfaces_without_horizontal_height_modes(self):
        points = self.plane_cloud([.13, .28], tilt=(.22, .1), n=29)
        result = reconstruct(points)
        self.assertEqual(len(result["components"]), 2)
        for component in result["components"]:
            ids = component["point_ids"]
            intercept = result["fits"]["height"][ids]-points[ids, :2]@np.array([.22, .1])
            self.assertLess(np.std(intercept), .001)
            faces = component["triangles"]
            truth_plane = (faces >= len(points)//2).astype(int)
            self.assertTrue(np.all(np.ptp(truth_plane, axis=1) == 0))

    def test_insufficient_points_cannot_borrow_from_another_height(self):
        rng = np.random.default_rng(9)
        xy = rng.uniform(-.15, .15, (29, 2))
        points = np.concatenate([np.column_stack((xy, np.full(29, z))) for z in (.12, .62)])
        result = reconstruct(points)
        self.assertEqual(len(result["components"]), 0)
        self.assertTrue(np.isnan(result["fits"]["height"]).all())
        self.assertTrue((result["fits"]["candidate_counts"] == 29).all())

    def test_relative_height_gates_and_independent_seed_predictions(self):
        points = self.plane_cloud([.13, .28])
        result = reconstruct(points)
        fits = result["fits"]
        seeds = np.flatnonzero(np.isfinite(fits["height"]))
        ids = fits["neighbor_ids"][seeds]
        self.assertEqual(ids.shape[1], 30)
        self.assertTrue(np.all(np.abs(points[ids, 2]-points[seeds, None, 2]) <= LocalConfig().seed_height_gate_m))
        # Equal XY observations at different heights both survive independently.
        q = 100
        self.assertTrue(np.array_equal(points[q, :2], points[q+len(points)//2, :2]))
        self.assertGreater(fits["height"][q+len(points)//2]-fits["height"][q], .14)


if __name__ == "__main__":
    unittest.main()

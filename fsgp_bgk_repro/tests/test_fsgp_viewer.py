"""Full-scene tiling, geometry support and independent physical checks."""
import unittest

import numpy as np

from fsgp_bgk_repro.core import Config, TraversabilityMapper
from fsgp_bgk_repro.export_fsgp_viewer import compute, FIELDS, grid_faces


class FSGPViewerTests(unittest.TestCase):
    def test_negative_world_grid_has_unique_owners_and_recovers_flat_zero(self):
        x, y = np.meshgrid(np.linspace(-3, 3, 81), np.linspace(-1, 1, 31))
        result = compute(np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size))))
        models = result['models']
        self.assertEqual(result['shape'], (10, 30))
        self.assertEqual(len(np.unique(result['cell_ids'], axis=0)), len(models))
        self.assertTrue(np.all(models[:, FIELDS.index('tile_id')] >= 0))
        self.assertTrue(np.all(models[:, FIELDS.index('valid')] == 1))
        np.testing.assert_allclose(models[:, FIELDS.index('gp_cost')], 0, atol=1e-12)
        self.assertEqual(len(result['triangles']), 2 * 9 * 29)

    def test_missing_observations_stay_unknown_and_faces_never_bridge_them(self):
        x, y = np.meshgrid(np.linspace(-3, 3, 61), np.linspace(-1, 1, 21))
        keep = np.abs(x.ravel()) > 1
        result = compute(np.column_stack((x.ravel()[keep], y.ravel()[keep], np.zeros(keep.sum()))))
        models = result['models']
        missing = np.abs(models[:, 0]) < .5
        self.assertTrue(missing.any())
        self.assertTrue(np.all(models[missing, FIELDS.index('valid')] == 0))
        self.assertTrue(np.isnan(models[missing, FIELDS.index('gp_cost')]).all())
        self.assertFalse(missing[result['triangles']].any())

    def test_owned_queries_match_direct_core_gp_and_raw_weighted_score(self):
        x, y = np.meshgrid(np.linspace(-1, 1, 41), np.linspace(-1, 1, 41))
        points = np.column_stack((x.ravel(), y.ravel(), .25*x.ravel()+.1*y.ravel())).astype('<f4')
        config = Config(inducing_points=125)
        result = compute(points, config)
        tile = result['tiles'][0]
        direct = TraversabilityMapper(config).assess(points, center=tile['center'])
        lookup = {tuple(cell): i for i, cell in enumerate(direct['cell_ids'])}
        ids = [lookup[tuple(cell)] for cell in result['cell_ids']]
        np.testing.assert_allclose(result['models'][:, FIELDS.index('height')], direct['height'][ids])
        np.testing.assert_allclose(result['models'][:, FIELDS.index('gp_cost')], direct['cost'].ravel()[ids])
        score = (direct['curvature'] + direct['height_variation'] + direct['slope']) / 3
        np.testing.assert_allclose(result['models'][:, FIELDS.index('gp_preliminary')], score[ids])

    def test_faces_accept_one_valid_half_cell_without_filling_missing_corner(self):
        faces = grid_faces((2, 2), np.array([True, True, True, False]))
        np.testing.assert_array_equal(faces, [[0, 1, 2]])


if __name__ == '__main__':
    unittest.main()

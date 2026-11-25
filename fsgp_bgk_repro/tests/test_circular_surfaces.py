"""Circular neighbourhood and observed-height separation regressions."""
import unittest

import numpy as np

from fsgp_bgk_repro.circular_surfaces import CircularConfig, CircularSurface, circular_pools
from fsgp_bgk_repro.terrain_assessment import (
    circular_column_areas, clearance_metrics, fit_pools, local_metrics)


class CircularSurfaceTests(unittest.TestCase):
    def test_circle_rejects_square_corners_and_uses_all_eligible_returns(self):
        axis = np.linspace(-.065, .065, 17)
        x, y = np.meshgrid(axis, axis)
        points = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
        pool = circular_pools(points, np.zeros((1, 3)), CircularConfig())[0]
        expected = np.flatnonzero(np.sum(points[:, :2]**2, axis=1) <= .07**2+1e-10)
        np.testing.assert_array_equal(pool, expected)
        self.assertGreater(len(pool), 15)
        self.assertTrue(np.all(np.linalg.norm(points[pool, :2], axis=1) <= .07000001))

    def test_observed_height_modes_do_not_assume_a_fixed_layer_count(self):
        axis = np.arange(-.3, .3001, .015)
        x, y = np.meshgrid(axis, axis)
        heights = (0., .2, .5, .85)
        points = np.concatenate([np.c_[x.ravel(), y.ravel(), np.full(x.size, z)]
                                 for z in heights])
        surface = CircularSurface(points)
        _, z = surface.candidates([[0., 0.]])
        values = z[np.isfinite(z)]
        for height in heights:
            self.assertTrue(np.any(np.abs(values-height) < 1e-5))
        self.assertLess(len(surface.seeds), len(points)/20)

    def test_local_height_discontinuity_is_not_smoothed_into_a_ramp(self):
        x, y = np.meshgrid(np.arange(-.4, .4001, .015), np.arange(-.25, .2501, .015))
        points = np.c_[x.ravel(), y.ravel(), np.where(x.ravel() < 0, 0., .03)]
        surface = CircularSurface(points)
        for xy, expected in [([-.12, 0.], 0.), ([.12, 0.], .03)]:
            ids, z = surface.query([xy], expected, z_tolerance=.01)
            self.assertGreaterEqual(ids[0], 0)
            self.assertAlmostEqual(z[0], expected, places=6)
        for model in surface.active:
            self.assertLess(np.ptp(points[surface.pools[model], 2]), .001)

    def test_clearance_clips_circle_corners_and_weights_partial_columns(self):
        axis = np.linspace(-.065, .065, 17)
        x, y = np.meshgrid(axis, axis)
        ground = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
        seeds = np.zeros((1, 3));cfg = CircularConfig()
        pools = circular_pools(ground, seeds, cfg)
        fits = fit_pools(ground, seeds, pools, cfg)
        areas = circular_column_areas(.07, .05)
        self.assertAlmostEqual(areas.sum(), np.pi*.07**2, places=11)
        empty_vertices, empty_faces = np.empty((0,3)), np.empty((0,3),int)
        outside = clearance_metrics(np.r_[ground, [[.065,.065,.4]]], seeds, fits,
                                    empty_vertices, empty_faces, cfg)
        self.assertEqual(outside['cost'][0], 0.)
        inside = clearance_metrics(np.r_[ground, [[0.,0.,.4]]], seeds, fits,
                                   empty_vertices, empty_faces, cfg)
        self.assertAlmostEqual(inside['cost'][0], .05**2/(np.pi*.07**2), places=10)
        roof = ground.copy();roof[:,2] = .4
        full = clearance_metrics(np.r_[ground,roof], seeds, fits,
                                 empty_vertices, empty_faces, cfg)
        self.assertAlmostEqual(full['cost'][0], 1.)

    def test_circle_normal_change_and_common_uncertainty_scale(self):
        seeds = np.zeros((1,3));c = np.zeros((1,6));c[0,[3,5]] = 1.
        fits = dict(coefficient=c,height=np.zeros(1),normal=np.array([[0.,0.,1.]]),
                    rmse=np.zeros(1),max_error=np.array([.004]))
        metrics = local_metrics(seeds,fits,np.array([.001]),dict(upper_m=np.zeros(1)),
                                dict(cost=np.zeros(1)),CircularConfig())
        self.assertAlmostEqual(metrics['normal_change_deg'][0],np.rad2deg(np.arctan(.14)),places=10)
        self.assertAlmostEqual(metrics['error_uncertainty_cost'][0],.3)

    def test_all_eleven_metrics_are_computed_and_unknown_body_is_not_zero(self):
        from fsgp_bgk_repro.assessment_export import FIELDS
        axis = np.arange(-.6,.6001,.025);x,y = np.meshgrid(axis,axis)
        surface = CircularSurface(np.c_[x.ravel(),y.ravel(),np.zeros(x.size)])
        vertices,faces,_ = surface.geometry()
        assessment = surface.compute_metrics(vertices,faces)
        packed = surface.packed();cols = {key:i for i,key in enumerate(FIELDS)}
        self.assertEqual(packed.shape,(len(surface.seeds),40))
        self.assertEqual(len(surface.config.supported_fields),11)
        for field in surface.config.supported_fields:
            self.assertTrue(np.isfinite(packed[:,cols[field]]).any(),field)
        inside = np.linalg.norm(surface.seeds[:,:2],axis=1)<.1
        self.assertTrue(np.isfinite(assessment['body']['risk'][inside]).any())
        outside = np.abs(surface.seeds[:,0])>.5
        self.assertTrue(np.isnan(assessment['body']['risk'][outside]).all())
        np.testing.assert_allclose(assessment['jumps']['bilateral_m'][
            np.isfinite(assessment['jumps']['bilateral_m'])],0.,atol=1e-10)


if __name__ == '__main__':
    unittest.main()

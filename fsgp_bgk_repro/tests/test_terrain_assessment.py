"""Analytical references independent of the exporter and rendering code."""
import unittest
from dataclasses import replace
import numpy as np

from fsgp_bgk_repro.terrain_assessment import (
    AssessmentConfig, basis, window_pools, fit_pools, variance, jump_metrics,
    clearance_metrics, local_metrics, body_metrics)


def two_sides(jump=0., background=True, noise=0., seed=42):
    axis = np.linspace(-.08, .08, 9)
    x, y = np.meshgrid(axis, axis)
    xy = np.column_stack((x.ravel(), y.ravel()))
    clouds, anchors = [], []
    rng = np.random.default_rng(seed)
    for center in (-.12, .12):
        p = xy + [center, 0]
        z = .15*p[:, 0]+.25*p[:, 0]**2+.1*p[:, 1]**2 if background else np.zeros(len(p))
        z = z + jump*(center > 0)
        clouds.append(np.column_stack((p, z+rng.normal(0, noise, len(p)))))
        anchors.append([center, 0, (.15*center+.25*center**2 if background else 0)+jump*(center > 0)])
    points, seeds = np.concatenate(clouds), np.array(anchors)
    pools = window_pools(points, seeds)
    fits = fit_pools(points, seeds, pools)
    return points, seeds, pools, fits


class TerrainAssessmentTests(unittest.TestCase):
    def test_quadratic_and_covariance_against_explicit_linear_algebra(self):
        points, seeds, pools, fits = two_sides()
        np.testing.assert_allclose(fits['coefficient'][:, 3:6], [[.25, 0, .1]]*2, atol=2e-8)
        i = 0
        delta = points[pools[i]]-seeds[i]
        scale = np.sqrt(np.mean(np.sum(delta[:, :2]**2, axis=1)))
        a = np.column_stack((np.ones(len(delta)), delta[:, 0]/scale, delta[:, 1]/scale,
                             (delta[:, 0]/scale)**2, delta[:, 0]*delta[:, 1]/scale**2,
                             (delta[:, 1]/scale)**2))
        gram = a.T @ a
        inverse = np.linalg.inv(gram+np.eye(6)*1e-10*np.trace(gram)/6)
        transform = np.diag([1, 1/scale, 1/scale, 1/scale**2, 1/scale**2, 1/scale**2])
        reference = .003**2*transform @ inverse @ gram @ inverse.T @ transform
        np.testing.assert_allclose(fits['covariance'][i], reference, rtol=1e-10, atol=1e-14)
        p = np.array([[.01, .02]])
        phi = np.array([1, .13, .02, .13**2, .13*.02, .02**2])
        np.testing.assert_allclose(variance(seeds, fits['covariance'], [i], p), [phi @ reference @ phi], rtol=1e-10)
        self.assertGreaterEqual(np.linalg.eigvalsh(reference).min(), 0)

    def test_background_invariance_and_step(self):
        for jump in (0, .12):
            values = []
            for background in (False, True):
                points, seeds, pools, fits = two_sides(jump, background)
                result = jump_metrics(points, seeds, pools, fits)
                self.assertEqual(len(result['pairs']), 1)
                self.assertEqual(np.intersect1d(*pools).size, 0)
                np.testing.assert_allclose(result['bilateral_m'], jump, atol=2e-8)
                np.testing.assert_allclose(result['one_sided_m'], jump, atol=2e-8)
                np.testing.assert_allclose(result['upper_m'], result['bilateral_m']+2*result['jump_sigma_m'])
                values.append(result['bilateral_m'])
            np.testing.assert_allclose(*values, atol=2e-8)

    def test_missing_side_is_unknown(self):
        points, seeds, pools, fits = two_sides()
        result = jump_metrics(points, seeds[:1], pools[:1], {k:v[:1] for k,v in fits.items()})
        self.assertTrue(np.isnan(result['bilateral_m'][0]))
        self.assertEqual(result['known_directions'][0], 0)

    def test_strict_spatial_gate(self):
        points, seeds, pools, fits = two_sides()
        for seed, ids in zip(seeds, pools):
            self.assertTrue(np.all(np.abs(points[ids, :2]-seed[:2]) <= .10000001))
            self.assertTrue(np.all(np.abs(points[ids, 2]-seed[2]) <= .04000001))

    def test_clearance_raw_and_fitted_surface_relative_to_sloped_ground(self):
        x,y = np.meshgrid(np.linspace(-.08,.08,9), np.linspace(-.08,.08,9))
        ground = np.column_stack((x.ravel(),y.ravel(),.2*x.ravel()))
        seeds = np.zeros((1,3));pools = window_pools(ground,seeds);fits = fit_pools(ground,seeds,pools)
        for height in (.4,.6,.8):
            roof=ground.copy();roof[:,2]+=height
            result=clearance_metrics(np.r_[ground,roof],seeds,fits,np.empty((0,3)),np.empty((0,3),int))
            self.assertEqual(result['cost'][0], 1 if height<=.6 else 0)
            # Surface evidence works even when raw obstacle observations are absent.
            result=clearance_metrics(ground,seeds,fits,roof,np.array([[0,8,80],[0,80,72]]))
            self.assertEqual(result['cost'][0]>0, height<=.6)

    def test_slope_hard_limit_and_flat_body_margin(self):
        axis=np.linspace(-1,1,21);x,y=np.meshgrid(axis,axis)
        seeds=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)));n=len(seeds)
        fits=dict(coefficient=np.zeros((n,6)),height=np.zeros(n),rmse=np.zeros(n),
                  max_error=np.zeros(n),normal=np.tile([0,0,1.],(n,1)))
        jumps=dict(upper_m=np.zeros(n));clearance=dict(cost=np.zeros(n))
        metrics=local_metrics(seeds,fits,np.zeros(n),jumps,clearance)
        body=body_metrics(seeds,fits,metrics,jumps,clearance)
        center=np.argmin(np.linalg.norm(seeds[:,:2],axis=1))
        self.assertAlmostEqual(body['support_margin_m'][center],.20)
        self.assertAlmostEqual(body['risk'][center],0)
        self.assertTrue(np.isnan(body['risk'][0]))
        fits['coefficient'][:,1]=1.2;fits['normal'][:]=[-1.2,0,1]/np.linalg.norm([-1.2,0,1])
        metrics=local_metrics(seeds,fits,np.zeros(n),jumps,clearance)
        np.testing.assert_array_equal(metrics['foot_cost'],np.ones(n))


if __name__=='__main__':
    unittest.main()

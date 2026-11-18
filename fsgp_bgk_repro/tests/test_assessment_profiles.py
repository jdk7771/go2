"""Numerical checks for 15 cm domains and the minimum-15 inclusion rule."""
from dataclasses import replace
import unittest
import numpy as np

from fsgp_bgk_repro.terrain_assessment import (
    AssessmentConfig, choose_support, clearance_metrics, fit_pools,
    local_metrics, window_pools,
)


class AssessmentProfileTests(unittest.TestCase):
    cfg=replace(AssessmentConfig(),patch_size_m=.15,minimum_points=15)

    def test_all_eligible_points_not_nearest_fifteen_and_strict_window(self):
        rng=np.random.default_rng(1515)
        xy=rng.uniform(-.07,.07,(23,2))
        z=.1*xy[:,0]+.2*xy[:,0]**2+.3*xy[:,1]**2
        points=np.r_[np.column_stack((xy,z)),[[.076,0,0],[0,0,.041]]]
        seeds=np.zeros((1,3));pools=window_pools(points,seeds,self.cfg)
        np.testing.assert_array_equal(pools[0],np.arange(23))
        fits=fit_pools(points,seeds,pools,self.cfg)
        self.assertEqual(fits['counts'][0],23)
        a=np.column_stack((np.ones(23),xy,xy[:,0]**2,xy[:,0]*xy[:,1],xy[:,1]**2))
        expected=np.linalg.lstsq(a,z,rcond=None)[0]
        np.testing.assert_allclose(fits['coefficient'][0],expected,atol=2e-8)
        for n in (14,15):
            f=fit_pools(points,seeds,[np.arange(n)],self.cfg)
            self.assertEqual(np.isfinite(f['height'][0]),n==15)

    def test_clearance_normalizes_nine_columns_not_sixteen(self):
        x,y=np.meshgrid(np.linspace(-.07,.07,5),np.linspace(-.07,.07,5))
        ground=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))
        seeds=np.zeros((1,3));pools=window_pools(ground,seeds,self.cfg);fits=fit_pools(ground,seeds,pools,self.cfg)
        roof=ground.copy();roof[:,2]=.4
        result=clearance_metrics(np.r_[ground,roof],seeds,fits,np.empty((0,3)),np.empty((0,3),int),self.cfg)
        self.assertEqual(result['raw_occupied'].shape,(1,9))
        self.assertEqual(result['cost'][0],1.)

    def test_body_support_respects_selected_patch_width(self):
        seeds=np.zeros((1,3));fits=dict(height=np.zeros(1),coefficient=np.zeros((1,6)))
        query=np.array([[[.074,0],[.076,0]]]);expected=np.zeros((1,2))
        ids,_=choose_support(seeds,fits,query,expected,config=self.cfg)
        np.testing.assert_array_equal(ids,[[0,-1]])
        ids,_=choose_support(seeds,fits,query,expected)
        np.testing.assert_array_equal(ids,[[0,0]])

    def test_normal_change_is_evaluated_at_fifteen_cm_corners(self):
        seeds=np.zeros((1,3));c=np.zeros((1,6));c[0,3]=1
        fits=dict(coefficient=c,height=np.zeros(1),normal=np.array([[0,0,1.]]),rmse=np.zeros(1),max_error=np.zeros(1))
        m=local_metrics(seeds,fits,np.zeros(1),dict(upper_m=np.zeros(1)),dict(cost=np.zeros(1)),self.cfg)
        self.assertAlmostEqual(m['normal_change_deg'][0],np.rad2deg(np.arctan(.15)),places=10)


if __name__=='__main__':unittest.main()

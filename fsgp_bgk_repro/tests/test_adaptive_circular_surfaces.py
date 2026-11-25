"""Bounded density adaptation, discontinuity preservation and observation gaps."""
import unittest

import numpy as np

from fsgp_bgk_repro.adaptive_circular_surfaces import AdaptiveCircularSurface


class AdaptiveCircularTests(unittest.TestCase):
    def test_sparse_plane_expands_radius_and_remains_accurate(self):
        x,y=np.meshgrid(np.arange(-.5,.5001,.035),np.arange(-.5,.5001,.035))
        p=np.c_[x.ravel(),y.ravel(),.5*x.ravel()+.15*y.ravel()]
        s=AdaptiveCircularSurface(p)
        self.assertGreater(len(s.active),10)
        self.assertTrue(np.any(s.model_radii_m[s.active]>.07))
        self.assertLessEqual(s.model_radii_m.max(),.15)
        self.assertTrue(np.all(s.fits['counts'][s.active]>=15))
        self.assertLess(s.fits['rmse'][s.active].max(),1e-7)
        ids,z=s.query([[0.,0.],[.1,0.]],expected=[0.,.05])
        self.assertTrue(np.all(ids>=0))
        np.testing.assert_allclose(z,[0.,.05],atol=1e-7)
        self.assertLess(len(s.seeds),len(p)/3)

    def test_flat_step_modes_do_not_become_a_smooth_bridge(self):
        x,y=np.meshgrid(np.arange(-.5,.5001,.025),np.arange(-.3,.3001,.025))
        p=np.c_[x.ravel(),y.ravel(),np.where(x.ravel()<0.,0.,.03)]
        s=AdaptiveCircularSurface(p)
        for i in s.active:
            self.assertLess(np.ptp(p[s.pools[i],2]),.001)
        ids,z=s.query([[-.15,0.],[.15,0.]],expected=[0.,.03])
        self.assertTrue(np.all(ids>=0));np.testing.assert_allclose(z,[0.,.03],atol=1e-7)

    def test_missing_strip_is_not_filled_by_a_larger_circle(self):
        x,y=np.meshgrid(np.arange(-.6,.6001,.03),np.arange(-.3,.3001,.03))
        p=np.c_[x.ravel(),y.ravel(),np.zeros(x.size)]
        p=p[np.abs(p[:,0])>.12]
        s=AdaptiveCircularSurface(p)
        ids,z=s.query([[0.,0.]])
        self.assertEqual(ids[0],-1);self.assertTrue(np.isnan(z[0]))
        vertices,faces,_=s.geometry()
        if len(faces):
            centers=vertices[faces].mean(axis=1)
            self.assertFalse(np.any(np.abs(centers[:,0])<.07))

    def test_each_rendered_triangle_stays_on_one_valid_function(self):
        x,y=np.meshgrid(np.arange(-.3,.3001,.03),np.arange(-.3,.3001,.03))
        p=np.c_[x.ravel(),y.ravel(),.2*x.ravel()**2+.1*y.ravel()**2]
        s=AdaptiveCircularSurface(p);v,f,owners=s.geometry()
        self.assertGreater(len(f),0)
        self.assertTrue(np.all(owners[f]==owners[f[:,0]][:,None]))
        for model in s.active:
            points=v[owners==model,:2]-s.seeds[model,:2]
            self.assertTrue(np.all(np.linalg.norm(points,axis=1)<=s.model_radii_m[model]+1e-6))

    def test_curved_region_can_keep_a_smaller_valid_domain(self):
        x,y=np.meshgrid(np.arange(-.5,.5001,.032),np.arange(-.3,.3001,.032))
        p=np.c_[x.ravel(),y.ravel(),.12*np.sin(28*x.ravel())]
        s=AdaptiveCircularSurface(p)
        preferred=s.config.center_pitch_m/np.sqrt(3)+s.config.coverage_margin_m
        self.assertGreater(len(s.active),0)
        self.assertTrue(np.any(s.model_radii_m[s.active]<preferred))
        self.assertTrue(np.all(s.fits['counts'][s.active]>=15))
        self.assertTrue(np.all(s.fits['rmse'][s.active]<=.008))


if __name__=='__main__':unittest.main()

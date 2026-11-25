"""Convex-only display, analytic-normal summaries and independent raw occupancy."""
import unittest
import numpy as np
from scipy.spatial import cKDTree
from fsgp_bgk_repro.convex_surface_assessment import (area,inside,display_patch,derivative_metrics,occupancy)


class ConvexAssessmentTests(unittest.TestCase):
    def setUp(self):
        self.polygon=np.array([[-.08,-.06],[.09,-.04],[.07,.08],[-.06,.09]])
        self.seed=np.array([1.,2.,.3])

    def test_render_covers_entire_polygon_and_stays_on_function(self):
        c=np.array([.01,.3,-.2,.5,.2,-.3])
        for rings in (1,3,5):
            v,f=display_patch(self.seed,c,self.polygon,rings=rings)
            xy=v[:,:2]-self.seed[:2]
            self.assertTrue(np.all(inside(xy,self.polygon)))
            a=xy[f[:,1]]-xy[f[:,0]];b=xy[f[:,2]]-xy[f[:,0]]
            total=np.sum(np.abs(a[:,0]*b[:,1]-a[:,1]*b[:,0]))/2
            self.assertAlmostEqual(total,area(self.polygon),places=12)
            u,w=xy.T;z=self.seed[2]+c[0]+c[1]*u+c[2]*w+c[3]*u*u+c[4]*u*w+c[5]*w*w
            np.testing.assert_allclose(v[:,2],z,atol=1e-14)

    def test_plane_normal_mean_and_smoothness(self):
        c=np.array([0.,.4,-.3,0.,0.,0.]);cov=np.zeros((6,6));cov[0,0]=.002**2
        m=derivative_metrics(c,self.polygon,cov)
        normal=np.array([-.4,.3,1.]);normal/=np.linalg.norm(normal)
        np.testing.assert_allclose(m['mean_normal'],normal,atol=1e-13)
        self.assertAlmostEqual(m['slope_deg'],np.rad2deg(np.arctan(.5)),places=10)
        self.assertEqual(m['normal_spread_deg'],0.)
        self.assertAlmostEqual(m['std_patch_m'],.002,places=12)

    def test_symmetric_bowl_has_small_mean_slope_and_nonzero_normal_spread(self):
        polygon=np.array([[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]])
        m=derivative_metrics(np.array([0.,0.,0.,2.,0.,2.]),polygon,np.eye(6)*1e-8)
        self.assertLess(m['slope_deg'],1e-6)
        self.assertGreater(m['normal_spread_deg'],10.)

    def test_occupancy_exact_clipped_column_area_and_polygon_gate(self):
        polygon=np.array([[-.075,-.05],[.075,-.05],[.075,.05],[-.075,.05]])
        p=np.array([[.012,.012,.2],[.09,.012,.2],[.012,.012,.7],[.012,.012,.05]])
        value,hits,columns=occupancy(p,cKDTree(p[:,:2]),np.zeros(3),np.zeros(6),polygon)
        self.assertAlmostEqual(value,1/6,places=12)
        self.assertEqual(hits,1);self.assertEqual(columns,1)

    def test_occupancy_evaluates_function_at_each_raw_xy(self):
        p=np.array([[.05,0.,.05+.2],[.04,0.,.04+.08]])
        c=np.array([0.,1.,0.,0.,0.,0.])
        value,hits,columns=occupancy(p,cKDTree(p[:,:2]),np.zeros(3),c,self.polygon)
        self.assertEqual(hits,1);self.assertGreater(value,0.)


if __name__=='__main__':unittest.main()

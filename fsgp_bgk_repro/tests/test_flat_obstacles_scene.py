"""Check the actual empty corridor and solid-platform geometry."""
import unittest
import numpy as np
from fsgp_bgk_repro.flat_obstacles_scene import build
from fsgp_bgk_repro.convex_surface_assessment import fresh_fixed_fit, occupancy
from scipy.spatial import cKDTree


class FlatObstaclesSceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.points, cls.labels, cls.meta = build(.025)

    def test_open_corridor_retains_floor_and_both_roof_surfaces(self):
        p=self.points
        middle=(np.abs(p[:,0])<.1)&(np.abs(p[:,1]-1.5)<.1)
        np.testing.assert_allclose(np.unique(p[middle,2]),[0.,.6,.72],atol=1e-6)
        free=(np.abs(p[:,0])<2-1e-6)&(p[:,1]>.5+1e-6)&(p[:,1]<2.5-1e-6)&(p[:,2]>1e-6)&(p[:,2]<.6-1e-6)
        self.assertFalse(free.any())
        self.assertEqual(len(p),len(np.unique(p,axis=0)))

    def test_platforms_are_solid_with_only_exposed_surfaces(self):
        p=self.points
        for box in self.meta['platforms']:
            lo,hi=np.array(box['min']),np.array(box['max'])
            interior=np.all((p[:,:2]>lo[:2]+1e-6)&(p[:,:2]<hi[:2]-1e-6),axis=1)
            self.assertGreater(interior.sum(),0)
            np.testing.assert_allclose(p[interior,2],hi[2],atol=1e-6)
            q=p[self.labels==box['label']]
            self.assertTrue(np.all((q>=lo-1e-6)&(q<=hi+1e-6)))
            on_surface=np.isclose(q[:,0],lo[0])|np.isclose(q[:,0],hi[0])|np.isclose(q[:,1],lo[1])|np.isclose(q[:,1],hi[1])|np.isclose(q[:,2],hi[2])
            self.assertTrue(on_surface.all())

    def test_new_square_fits_use_xyz_and_keep_superposed_surfaces_separate(self):
        # No part labels or known heights are supplied to this fit.
        seeds,pools,fits,_,_,info=fresh_fixed_fit(self.points.astype(float),'q15-n15')
        local=(np.abs(seeds[:,0])<.3)&(np.abs(seeds[:,1]-1.5)<.3)&np.isfinite(fits['height'])
        np.testing.assert_allclose(np.unique(fits['height'][local]),[0.,.6,.72],atol=1e-6)
        for model in np.flatnonzero(local):
            self.assertLess(np.ptp(self.points[pools[model],2]),1e-6)
        self.assertEqual(info['actual_main_solves'],np.isfinite(fits['rmse']).sum())

    def test_float32_clearance_boundary_is_occupied_but_taller_returns_are_not(self):
        polygon=np.array([[-.05,-.05],[.05,-.05],[.05,.05],[-.05,.05]])
        p=np.array([[.01,.01,.6],[-.01,-.01,.60001]],dtype='<f4').astype(float)
        value,hits,columns=occupancy(p,cKDTree(p[:,:2]),np.zeros(3),np.zeros(6),polygon)
        self.assertEqual(hits,1);self.assertEqual(columns,1);self.assertAlmostEqual(value,.25)


if __name__=='__main__':unittest.main()

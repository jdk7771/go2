import unittest
import numpy as np

from quadric_lidar_repro.core import Config, Moments, fit_patch
from quadric_lidar_repro.terrain import SegmentationConfig, analytic_map, local_height_map, segment_unorganized, vertical_geometry


class TerrainTests(unittest.TestCase):
    def test_segmentation_separates_overlapping_height_layers(self):
        x,y=np.meshgrid(np.linspace(-.6,.6,25),np.linspace(-.6,.6,25))
        lower=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))
        upper=lower+[0,0,.8]
        points=np.ascontiguousarray(np.vstack((lower,upper)))
        labels,groups,_=segment_unorganized(points,SegmentationConfig(cell_size=3.))
        self.assertTrue((labels>=0).all())
        self.assertTrue(set(labels[:len(lower)]).isdisjoint(set(labels[len(lower):])))
        for group in groups:
            self.assertLess(np.ptp(points[group,2]),1e-10)

    def test_vertical_root_selects_observed_sphere_layer(self):
        rng=np.random.default_rng(8)
        p=rng.normal(size=(1000,3));p*=2/np.linalg.norm(p,axis=1,keepdims=True)
        patch=fit_patch(Moments.from_points(p),Config(fitting="taubin"))
        xy=np.array([[.2,.3],[.2,.3],[3,0]])
        r=vertical_geometry(patch,xy,np.array([2.,-2.,0.]))
        self.assertTrue(r["valid"][:2].all());self.assertFalse(r["valid"][2])
        np.testing.assert_allclose(r["height"][:2],[np.sqrt(4-.13),-np.sqrt(4-.13)],atol=1e-9)

    def test_raw_plane_pipeline_and_local_polynomial(self):
        x,y=np.meshgrid(np.linspace(-1.9,1.9,50),np.linspace(-1.9,1.9,50))
        points=np.column_stack((x.ravel(),y.ravel(),(.2*x-.1*y).ravel()))
        xy=np.array([[.23,.31],[-.24,.17],[5,5]])
        for r in [analytic_map(points,xy),local_height_map(points,xy)]:
            self.assertTrue(r["valid"][:2].all());self.assertFalse(r["valid"][2])
            np.testing.assert_allclose(r["height"][:2],.2*xy[:2,0]-.1*xy[:2,1],atol=1e-8)
            np.testing.assert_allclose(r["slope_deg"][:2],np.degrees(np.arctan(np.hypot(.2,.1))),atol=1e-7)
            np.testing.assert_allclose(r["principal"][:2],0,atol=1e-7)


if __name__=="__main__":
    unittest.main()

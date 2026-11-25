"""Strict square-centimetre threshold and display-only triangle subdivision."""
import unittest
import numpy as np
from fsgp_bgk_repro.hill_small_gaps import is_small_gap,subdivide,boundary_compatible
from fsgp_bgk_repro.convex_surface_assessment import FIELDS


class HillSmallGapTests(unittest.TestCase):
    def test_nine_square_centimetres_is_not_small(self):
        self.assertTrue(is_small_gap(8.99/10000))
        self.assertFalse(is_small_gap(9/10000))
        self.assertFalse(is_small_gap(9.01/10000))
        self.assertFalse(is_small_gap(0.))

    def test_subdivision_preserves_area_and_bounds(self):
        triangle=np.array([[[0.,0.],[.4,0.],[0.,.004]]])
        split=subdivide(triangle,.04)
        lengths=np.linalg.norm(np.roll(split,-1,axis=1)-split,axis=2)
        self.assertLessEqual(lengths.max(),.04+1e-12)
        a,b=split[:,1]-split[:,0],split[:,2]-split[:,0]
        self.assertAlmostEqual(np.sum(abs(a[:,0]*b[:,1]-a[:,1]*b[:,0]))/2,.0008,places=14)
        self.assertTrue(np.all(split[:,:,0]>=0));self.assertTrue(np.all(split[:,:,1]>=0))
        self.assertTrue(np.all(split[:,:,0]/.4+split[:,:,1]/.004<=1+1e-12))

    def boundary_case(self,step=0.,slope=0.,extra_level=False):
        triangle=np.array([[[0.,0.],[.02,0.],[.01,.01]]])
        polygons=np.array([[-1.,-1.],[0.,-1.],[0.,1.],[-1.,1.],
                           [.02,-1.],[1.,-1.],[1.,1.],[.02,1.]])
        model=np.zeros((3 if extra_level else 2,len(FIELDS)))
        model[:,4]=slope;model[1,3]=step;offset=np.array([0,4,8])
        if extra_level:
            model[2,3]=1.;polygons=np.r_[polygons,polygons[:4]];offset=np.r_[offset,12]
        return boundary_compatible(triangle,np.arange(len(model))[None,:],model,polygons,offset)

    def test_distinct_platform_heights_are_not_bridged(self):
        self.assertFalse(self.boundary_case(step=.1).any())

    def test_small_height_disagreement_is_allowed(self):
        self.assertTrue(self.boundary_case(step=.02).all())

    def test_continuous_steep_slope_is_not_a_step(self):
        # Six centimetres rise across the gap, but both functions agree.
        self.assertTrue(self.boundary_case(slope=3.).all())

    def test_other_height_does_not_veto_same_level_support(self):
        np.testing.assert_array_equal(self.boundary_case(extra_level=True),[[True,True,False]])


if __name__=='__main__':unittest.main()

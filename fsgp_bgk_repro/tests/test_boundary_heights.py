"""Same-boundary predictions distinguish a step from a continuous steep slope."""
import unittest
import numpy as np
from fsgp_bgk_repro.boundary_heights import extract,predict
from fsgp_bgk_repro.convex_surface_assessment import FIELDS
from fsgp_bgk_repro.polygon_topology import PolygonTopology


def two_regions(step=0.,slope=0.):
    model=np.zeros((6,len(FIELDS)));polygons=[]
    for side,(low,high) in enumerate(((-.5,0.),(.04,.54))):
        for j in range(3):
            i=side*3+j;model[i,:2]=[(low+high)/2,(j-1)*.01]
            model[i,2]=slope*model[i,0]+side*step;model[i,4]=slope
            polygons.append(np.array([[low,-.2],[high,-.2],[high,.2],[low,.2]]))
    return model,np.arange(6),polygons


class BoundaryHeightTests(unittest.TestCase):
    def test_parallel_platforms_recover_eighty_centimetres(self):
        m,active,p=two_regions(.8);records,_=extract(m,active,p)
        self.assertGreater(len(records),10)
        for r in records:
            self.assertAlmostEqual(r['delta_m'],.8,places=12)
            self.assertAlmostEqual(r['xy'][0],.02,places=12)
            self.assertAlmostEqual(r['boundary_xy_gap_m'],.04,places=12)

    def test_continuous_steep_slope_has_no_jump(self):
        m,active,p=two_regions(slope=3.)
        self.assertGreater(abs(predict(m,0,m[0,:2])-predict(m,3,m[3,:2])),1.)
        records,_=extract(m,active,p);self.assertEqual(records,[])

    def test_step_superimposed_on_slope_uses_same_xy(self):
        m,active,p=two_regions(.15,3.);records,_=extract(m,active,p)
        self.assertGreater(len(records),10)
        np.testing.assert_allclose([r['delta_m'] for r in records],.15,atol=1e-12)

    def test_nearest_polygon_points_are_actual_boundary_positions(self):
        with PolygonTopology() as geo:
            a=geo.polygon(np.array([[0,0],[1,0],[1,1],[0,1]]));b=geo.point([1.04,.5])
            self.assertAlmostEqual(geo.distance(a,b),.04,places=12)
            np.testing.assert_allclose(geo.nearest(a,b),[[1.,.5],[1.04,.5]])


if __name__=='__main__':unittest.main()

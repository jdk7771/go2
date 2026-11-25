"""Connected-hole areas and exact constrained rendering of concave regions."""
import unittest
import numpy as np
from fsgp_bgk_repro.polygon_topology import PolygonTopology


def rectangle(x0,y0,x1,y1):return np.array([[x0,y0],[x1,y0],[x1,y1],[x0,y1]])


class PolygonTopologyTests(unittest.TestCase):
    def test_connected_gap_is_measured_as_one_region(self):
        with PolygonTopology() as geo:
            # Two small squares connected by a neck form one >9 cm² gap.
            hole=np.array([[0,0],[.02,0],[.02,.006],[.04,.006],[.04,0],[.06,0],
                           [.06,.02],[.04,.02],[.04,.014],[.02,.014],[.02,.02],[0,.02]])
            surface=geo.polygon(rectangle(-1,-1,1,1),[hole])
            gaps=geo.enclosed_gaps(surface)
            self.assertEqual(len(gaps),1)
            self.assertAlmostEqual(gaps[0]['area']*10000,9.6,places=10)
            # Verify strict area units: 9 cm² = 0.0009 m².
            self.assertGreater(gaps[0]['area'],9e-4)

    def test_concave_hole_triangulation_has_exact_area(self):
        with PolygonTopology() as geo:
            region=dict(exterior=np.array([[0,0],[.04,0],[.04,.01],[.01,.01],[.01,.04],[0,.04]]),holes=[])
            polygon=geo.polygon(region['exterior']);tri=geo.triangles(region)
            a,b=tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]
            area=np.sum(abs(a[:,0]*b[:,1]-a[:,1]*b[:,0]))/2
            self.assertAlmostEqual(area,geo.area(polygon),places=14)
            self.assertAlmostEqual(area*10000,7.,places=10)

    def test_union_uses_whole_polygon_holes_and_leaves_exterior_open(self):
        with PolygonTopology() as geo:
            ring=[rectangle(0,0,.1,.04),rectangle(0,.06,.1,.1),
                  rectangle(0,.04,.04,.06),rectangle(.06,.04,.1,.06)]
            union=geo.union(ring);gaps=geo.enclosed_gaps(union)
            self.assertEqual(len(gaps),1);self.assertAlmostEqual(gaps[0]['area']*10000,4.,places=10)
            opened=geo.union(ring[:3]);self.assertEqual(geo.enclosed_gaps(opened),[])

    def test_nested_coverage_island_does_not_duplicate_inner_hole(self):
        with PolygonTopology() as geo:
            outer=geo.polygon(rectangle(-1,-1,1,1),[rectangle(-.5,-.5,.5,.5)])
            island=geo.polygon(rectangle(-.2,-.2,.2,.2),[rectangle(-.05,-.05,.05,.05)])
            combined=geo._union_children([outer,island]);gaps=geo.enclosed_gaps(combined)
            self.assertEqual(len(gaps),2)
            np.testing.assert_allclose(sorted(g['area'] for g in gaps),[.01,.84],atol=1e-12)

    def test_filling_around_island_preserves_its_inner_gap(self):
        with PolygonTopology() as geo:
            outer=geo.polygon(rectangle(-1,-1,1,1),[rectangle(-.5,-.5,.5,.5)])
            island=geo.polygon(rectangle(-.2,-.2,.2,.2),[rectangle(-.05,-.05,.05,.05)])
            combined=geo._union_children([outer,island]);gaps=geo.enclosed_gaps(combined)
            annulus=max(gaps,key=lambda g:g['area'])
            completed=geo.union_with(combined,[annulus]);remaining=geo.enclosed_gaps(completed)
            self.assertEqual(len(remaining),1)
            self.assertAlmostEqual(remaining[0]['area'],.01,places=12)
            self.assertAlmostEqual(geo.area(completed)-geo.area(combined),.84,places=12)


if __name__=='__main__':unittest.main()

import unittest
from types import SimpleNamespace
import numpy as np
from fsgp_bgk_repro.convex_surface_assessment import FIELDS
from fsgp_bgk_repro.layer_projection_planner import LayerGraph,ProjectionPlanner,crossing_allowed,segment_distance


def two_planes(height,gap=0.):
    m=np.zeros((6,len(FIELDS)));m[:3,:2]=[.5,.5];m[3:,:2]=[1.5+gap,.5];m[3:,2]=height
    square=np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]])
    return SimpleNamespace(m=m,active=np.arange(6),polygon=lambda i:square)


class LayerProjectionTests(unittest.TestCase):
    def test_39cm_boundary_connects_but_40cm_does_not(self):
        below=LayerGraph(two_planes(.39));at=LayerGraph(two_planes(.40));above=LayerGraph(two_planes(.41))
        self.assertEqual(len(below.reachable(int(below.labels[0]))),2)
        self.assertEqual(len(at.reachable(int(at.labels[0]))),1)
        self.assertEqual(len(above.reachable(int(above.labels[0]))),1)

    def test_unknown_horizontal_gap_is_not_filled_by_projection(self):
        graph=LayerGraph(two_planes(.15,.08))
        self.assertEqual(len(graph.reachable(int(graph.labels[0]))),1)

    def test_continuous_slope_is_one_layer_even_with_large_height_change(self):
        surface=two_planes(0.);surface.m[:,4]=.5;surface.m[:,2]=surface.m[:,0]*.5
        graph=LayerGraph(surface)
        self.assertEqual(graph.labels[0],graph.labels[-1])
        self.assertEqual(graph.portals,[])

    def test_overlay_does_not_flatten_real_height(self):
        surface=two_planes(.3);before=surface.m.copy();graph=LayerGraph(surface)
        np.testing.assert_array_equal(surface.m,before)
        self.assertAlmostEqual(max(p['delta_m'] for p in graph.portals),.3)

    def test_flat_meeting_ramp_is_continuous_at_actual_boundary(self):
        surface=two_planes(0.);surface.m[3:,4]=.5;surface.m[3:,2]=.25
        graph=LayerGraph(surface)
        self.assertEqual(graph.labels[0],graph.labels[-1])
        self.assertEqual(graph.portals,[])

    def test_xy_overlap_does_not_authorize_switching_away_from_boundary(self):
        graph=LayerGraph(two_planes(.15));a,b=int(graph.labels[0]),int(graph.labels[-1])
        self.assertIsNotNone(graph.match(a,b,np.array([.8,.5]),np.array([1.2,.5])))
        self.assertIsNone(graph.match(a,b,np.array([.2,.5]),np.array([.3,.5])))

    def test_large_endpoint_height_change_on_same_layer_remains_rolling(self):
        planner=object.__new__(ProjectionPlanner);planner.graph=SimpleNamespace(labels=np.array([0,0]))
        a=dict(owners=[0]*4,feet=np.zeros((4,3)));b=dict(owners=[1]*4,feet=np.tile([.2,0,.12],(4,1)))
        self.assertEqual(planner.edge_kind(a,b)['kind'],'rolling')

    def test_failed_rolling_is_rejected_without_swing_fallback(self):
        planner=object.__new__(ProjectionPlanner);a=dict(key=(0,),body=np.zeros(3));b=dict(key=(1,))
        planner.edge_info={((0,),(1,)):dict(kind='rolling')};planner.rolling_edge=lambda a,b:None
        planner.lift_edge=lambda *a:(_ for _ in ()).throw(AssertionError('No swing fallback allowed'))
        result,bad=planner.validate_projection([a,b]);self.assertIsNone(result);self.assertEqual(bad,[((0,),(1,))])


if __name__=='__main__':unittest.main()

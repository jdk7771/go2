import unittest
from unittest.mock import patch
import numpy as np
from fsgp_bgk_repro.surface_robot import SurfaceRobot,support_margin
from fsgp_bgk_repro.m20_policy import DEFAULT_Q
from fsgp_bgk_repro.surface_astar import Config,SurfaceMap,Planner,jumps
from fsgp_bgk_repro.surface_obstacles import facing_wall,refine
from dataclasses import replace
from fsgp_bgk_repro.surface_layers import classify_stack
from fsgp_bgk_repro.convex_surface_assessment import FIELDS


class RobotGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.robot=SurfaceRobot()

    def test_xml_fk_for_random_reachable_joints(self):
        r=self.robot;rng=np.random.default_rng(8)
        ids=[r.mj.mj_name2id(r.model,r.mj.mjtObj.mjOBJ_BODY,name+'_wheel') for name in ('fl','fr','hl','hr')]
        for _ in range(12):
            q=DEFAULT_Q[:12].reshape(4,3)+rng.uniform(-.25,.25,(4,3));r.com(np.zeros(3),np.eye(3),q)
            np.testing.assert_allclose(r.fk(q),r.data.xpos[ids],atol=1e-12)

    def test_inverse_forward_round_trip(self):
        r=self.robot;body=np.array([0,0,.4]);feet=r.nominal_feet.copy();result=r.ik(body,np.eye(3),feet)
        self.assertLess(result['maximum_residual_m'],1e-10)
        np.testing.assert_allclose(result['wheels'],feet+[0,0,r.radius],atol=1e-12)

    def test_unreachable_contact_has_nonzero_soft_cost(self):
        r=self.robot;feet=r.nominal_feet.copy();feet[0,0]+=1.;result=r.ik(np.array([0,0,.4]),np.eye(3),feet)
        self.assertGreater(result['maximum_residual_m'],.1);self.assertGreater(result['soft_cost'],1.)

    def test_support_polygon_requires_com_inside(self):
        feet=np.array([[1,1,0],[1,-1,0],[-1,-1,0],[-1,1,0]])
        self.assertAlmostEqual(support_margin([0,0,1],feet),1.)
        self.assertLess(support_margin([2,0,0],feet),0)
        self.assertLess(support_margin([0,0,0],feet[:2]),0)


class ObservedSurfaceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.surface=SurfaceMap('surface-route-test');cls.robot=SurfaceRobot();cls.planner=Planner(cls.surface,cls.robot)

    def test_unknown_location_cannot_supply_contact(self):
        self.assertEqual(self.surface.query([20.,20.]),[]);self.assertEqual(self.surface.near([20.,20.]),[])

    def test_occupancy_does_not_change_cost_or_binary_headroom(self):
        original=SurfaceMap('table-low-042');read=np.fromfile
        def changed_occupancy(file,*args,**kwargs):
            data=read(file,*args,**kwargs)
            if str(file).endswith('/models.bin'):
                models=data.reshape(-1,len(FIELDS));column=FIELDS.index('clearance_cost')
                models[:,column]=1.-models[:,column]
            return data
        with patch('fsgp_bgk_repro.surface_astar.np.fromfile',side_effect=changed_occupancy):
            changed=SurfaceMap('table-low-042')
        self.assertNotIn('clearance_cost',changed.names)
        self.assertFalse(np.array_equal(original.m[:,FIELDS.index('clearance_cost')],changed.m[:,FIELDS.index('clearance_cost')]))
        np.testing.assert_array_equal(original.costs,changed.costs)
        for xy in ([0.,0.],[-2.24,0.]):
            a=original.query(xy,occupied=True);b=changed.query(xy,occupied=True)
            self.assertEqual([(o['owner'],o['overhead_passable'],o['cost']) for o in a],
                [(o['owner'],o['overhead_passable'],o['cost']) for o in b])

    def test_low_deck_and_overhead_are_distinct(self):
        ground=self.surface.query([0,0]);self.assertTrue(ground);self.assertAlmostEqual(ground[0]['point'][2],.03,places=6)
        options=self.surface.query([0,1.35],occupied=True);self.assertTrue(any(o['point'][2]<.01 for o in options))
        self.assertTrue(any(o['point'][2]<.01 and o['overhead_passable']==1 for o in self.surface.query([0,1.35])))

    def test_facing_wall_is_yz_to_forward(self):
        pose=dict(body=np.array([.9,0.,.4]),yaw=0.)
        wall=facing_wall(self.surface,pose);self.assertIsNotNone(wall);self.assertLess(wall['rmse_m'],1e-6)
        self.assertAlmostEqual(wall['nearest_world'][0],1.5,places=5)
        self.assertLess(wall['normal'][0],-.99)

    def test_virtual_force_preserves_contacts_and_ik(self):
        p=self.planner;pose=p.evaluate(np.array([.8,0.]),0,self.surface.query([.8,0])[0]['owner']);self.assertIsNotNone(pose)
        pose.update(phase='rolling',duration_s=.1,time_s=.1);routes=[dict(frames=[pose])]
        report=refine(p,routes);self.assertEqual(report['adjusted_frames'],1)
        after=routes[0]['frames'][0];np.testing.assert_allclose(after['feet'],pose['feet']);self.assertLess(after['body'][0],pose['body'][0])
        self.assertLessEqual(after['ik_max_m'],Config().ik_accept_m);self.assertGreaterEqual(after['zmp_margin_m'],Config().zmp_margin_m)

    def test_virtual_force_rejects_failed_pose(self):
        p=self.planner;pose=p.evaluate(np.array([.8,0.]),0,self.surface.query([.8,0])[0]['owner']);old=p.fixed_pose
        try:
            p.fixed_pose=lambda *a,**k:None;routes=[dict(frames=[pose])];report=refine(p,routes)
            self.assertEqual(report['adjusted_frames'],0);self.assertEqual(report['rejected_adjustments'],1);self.assertIs(routes[0]['frames'][0],pose)
        finally:p.fixed_pose=old

    def test_height_flag_and_same_xy_jump_differ_on_ramp(self):
        m=np.zeros((2,22));m[:,4]=.5;a=dict(feet=np.array([[0,0,0]]*4,float),owners=[0]*4);b=dict(feet=np.array([[.2,0,.1]]*4),owners=[1]*4)
        change,boundary=jumps(a,b,m);np.testing.assert_allclose(change,.1);np.testing.assert_allclose(boundary,0)

    def test_goal_height_cannot_be_satisfied_by_floor(self):
        planner=Planner(self.surface,self.robot,replace(Config(),maximum_expansions=25))
        yes,_=planner.search([-2.4,0],[-2.4,0],start_z=0.,goal_z=0.)
        no,_=planner.search([-2.4,0],[-2.4,0],start_z=0.,goal_z=1.2)
        self.assertEqual(len(yes),1);self.assertEqual(no,[])

    def test_wheel_cannot_cut_wall_when_body_is_clear(self):
        p=self.planner
        far=np.array([1.1,.85]);near=np.array([1.15,.85])
        self.assertTrue(self.surface.body_free(np.r_[near,.4],np.eye(3),self.robot))
        self.assertIsNotNone(p.evaluate(far,0,self.surface.query(far)[0]['owner']))
        self.assertIsNone(p.evaluate(near,0,self.surface.query(near)[0]['owner']))

    def test_table_endpoint_selects_requested_observed_height(self):
        surface=SurfaceMap('table');planner=Planner(surface,self.robot)
        floor=planner.nearby_endpoint([0,0],0,0.)
        top=planner.nearby_endpoint([0,0],0,1.2)
        np.testing.assert_allclose(floor['feet'][:,2],0.,atol=1e-8)
        np.testing.assert_allclose(top['feet'][:,2],1.2,atol=1e-7)
        self.assertLess(floor['body'][2],1.1)
        self.assertGreater(top['body'][2],1.2)

    def test_separated_table_legs_do_not_make_a_solid_wall(self):
        surface=SurfaceMap('table')
        self.assertIsNone(facing_wall(surface,dict(body=np.array([-2.,0.,.4]),yaw=0.)))

    def test_unsupported_initial_height_is_rejected(self):
        planner=Planner(self.surface,self.robot)
        with self.assertRaisesRegex(RuntimeError,'No observed'):
            planner.nearby_endpoint([-2.4,0],0,2.)

    def test_low_table_occupancy_requires_height_aware_policy(self):
        strict=SurfaceMap('table-low-042',replace(Config(),binary_overhead=False))
        aware=SurfaceMap('table-low-042',replace(Config(),reject_overhead_occupancy=False))
        self.assertFalse(any(abs(o['point'][2])<.01 for o in strict.query([0,0])))
        self.assertTrue(any(abs(o['point'][2])<.01 for o in aware.query([0,0])))
        planner=Planner(aware,self.robot,aware.config)
        floor=next(o for o in aware.query([0,0]) if abs(o['point'][2])<.01)
        pose=planner.evaluate(np.zeros(2),0.,floor['owner'])
        self.assertIsNotNone(pose);self.assertAlmostEqual(pose['body_height_m'],.32)
        self.assertGreaterEqual(pose['ceiling_z_m']-pose['body'][2]-.07,.025)
        self.assertIsNone(planner.fixed_pose(np.zeros(2),0.,pose['feet'],pose['normals'],pose['owners'],height=.4))

    def test_too_low_ceiling_rejects_floor_even_without_occupancy_filter(self):
        cfg=replace(Config(),reject_overhead_occupancy=False);surface=SurfaceMap('table-low-040',cfg)
        self.assertFalse(any(abs(o['point'][2])<.01 for o in surface.query([0,0])))
        floor=next(o for o in surface.query([0,0],occupied=True) if abs(o['point'][2])<.01)
        self.assertIsNone(Planner(surface,self.robot,cfg).evaluate(np.zeros(2),0.,floor['owner']))

    def test_body_lowers_before_horizontal_entry(self):
        cfg=replace(Config(),reject_overhead_occupancy=False);surface=SurfaceMap('table-low-042',cfg);planner=Planner(surface,self.robot,cfg)
        def pose(x):
            xy=np.array([x,0.]);floor=next(o for o in surface.query(xy) if abs(o['point'][2])<.01)
            return planner.evaluate(xy,0.,floor['owner'])
        a,b=pose(-2.),pose(-1.6);self.assertIsNotNone(a);self.assertIsNotNone(b)
        frames=planner.rolling_edge(a,b);self.assertIsNotNone(frames)
        all_frames=[a,*frames];lowered=[(i,p) for i,p in enumerate(all_frames) if p.get('phase')=='body_lowering']
        self.assertTrue(lowered)
        for i,p in lowered:
            np.testing.assert_allclose(p['feet'],all_frames[i-1]['feet'])
            np.testing.assert_allclose(p['body'][:2],all_frames[i-1]['body'][:2])
        self.assertLessEqual(max(abs(b['body_height_m']-a['body_height_m']) for a,b in zip(all_frames[:-1],all_frames[1:])),.0100001)
        reverse=planner.rolling_edge(b,a);self.assertIsNotNone(reverse)
        raised=[(i,p) for i,p in enumerate([b,*reverse]) if p.get('phase')=='body_raising']
        self.assertTrue(raised)
        for i,p in raised:
            np.testing.assert_allclose(p['feet'],[b,*reverse][i-1]['feet'])
            np.testing.assert_allclose(p['body'][:2],[b,*reverse][i-1]['body'][:2])

    def test_upward_scan_continues_above_blocked_intermediate_layer(self):
        options=[dict(owner=k,point=np.array([0.,0.,z])) for k,z in enumerate((0.,.30,.60))]
        layers=classify_stack(options,.415,.04)
        self.assertEqual([p['overhead_passable'] for p in layers],[0,0,1])
        self.assertEqual([c['owner'] for c in layers[0]['upward_candidates']],[1,2])
        self.assertEqual([c['switchable'] for c in layers[0]['upward_candidates']],[0,0])

    def test_climb_candidate_requires_both_step_capability_and_free_headroom(self):
        low,high=classify_stack([dict(owner=k,point=np.array([0.,0.,z])) for k,z in enumerate((0.,.04))],.415,.04)
        self.assertEqual(low['overhead_passable'],0)
        self.assertEqual(low['upward_candidates'][0]['switchable'],1)
        self.assertEqual(high['overhead_passable'],1)

    def test_existing_support_tread_is_not_an_overhead_ceiling(self):
        cfg=replace(Config(),ordered_switches=True,reject_overhead_occupancy=False)
        surface=SurfaceMap('layer-choice',cfg);planner=Planner(surface,self.robot,cfg)
        grid=tuple(np.round((np.array([-1.4,.74])-planner.low)/cfg.grid_m).astype(int))
        top=next(o for o in planner.center_options(grid) if o['point'][2]>.4)
        pose=planner.node(grid,0,top['owner']);self.assertIsNotNone(pose)
        self.assertTrue(np.isinf(pose['ceiling_z_m']))

    def test_descending_stair_uses_nearby_contact_to_keep_wheel_clear(self):
        cfg=replace(Config(),ordered_switches=True,reject_overhead_occupancy=False)
        surface=SurfaceMap('layer-choice',cfg);planner=Planner(surface,self.robot,cfg)
        grid=tuple(np.round((np.array([1.16,.74])-planner.low)/cfg.grid_m).astype(int))
        top=next(o for o in planner.center_options(grid) if o['point'][2]>.4)
        pose=planner.node(grid,0,top['owner']);self.assertIsNotNone(pose)
        self.assertGreaterEqual(surface.raw.query(pose['wheels'])[0].min(),cfg.wheel_clearance_m)

    def test_sparse_fitted_ceiling_still_blocks_but_cannot_supply_a_foot(self):
        low,high=classify_stack([dict(owner=0,point=np.array([0.,0.,0.]),raw_distance_m=.01),
            dict(owner=1,point=np.array([0.,0.,.03]),raw_distance_m=.10)],.415,.04)
        self.assertEqual(low['overhead_passable'],0)
        self.assertEqual(high['raw_supported'],0)
        self.assertEqual(low['upward_candidates'][0]['switchable'],0)

    def test_soft_ik_cannot_move_actual_contact_outside_observed_convex_domain(self):
        surface=SurfaceMap('flat');planner=Planner(surface,self.robot);xy=np.zeros(2)
        pose=planner.evaluate(xy,0.,surface.query(xy)[0]['owner']);self.assertIsNotNone(pose)
        feet=pose['feet'].copy();owner=pose['owners'][0];polygon=surface.polygon(owner)+surface.m[owner,:2]
        edge=polygon[1]-polygon[0];outward=np.array([edge[1],-edge[0]])/np.linalg.norm(edge)
        feet[0,:2]=(polygon[0]+polygon[1])/2-outward*1e-5
        self.assertIsNotNone(planner.fixed_pose(xy,0.,feet,pose['normals'],pose['owners']))
        old=self.robot.ik
        def shifted(*args,**kwargs):
            result=old(*args,**kwargs);result['wheels'][0,:2]=feet[0,:2]+outward*.005
            result['maximum_residual_m']=max(result['maximum_residual_m'],.005)
            return result
        try:
            self.robot.ik=shifted
            self.assertIsNone(planner.fixed_pose(xy,0.,feet,pose['normals'],pose['owners']))
        finally:self.robot.ik=old


if __name__=='__main__':unittest.main()

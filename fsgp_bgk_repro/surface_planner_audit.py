"""Independent checks of exported route poses against raw data and robot XML."""
import json
import time
import numpy as np
from scipy.spatial import ConvexHull
from .surface_astar import RESULT,SurfaceMap,Config,CASES
from .surface_robot import SurfaceRobot,LEG_NAMES
from .boundary_heights import predict
from .convex_surface_assessment import FIELDS,inside
from .assessment_export import sha,finite_json


def audit_scene(scene,robot=None):
    started=time.perf_counter();robot=robot or SurfaceRobot();path=RESULT/scene/'result.json';result=json.loads(path.read_text())
    if result.get('schema_version',1)>=2:
        from .layer_projection_audit import audit_scene as audit_projection
        return audit_projection(scene,robot)
    surface=SurfaceMap(scene,Config(**result['config']));assert result['status']=='success'
    cost_terms={'slope_deg':(.30,45.),'normal_spread_deg':(.20,15.),'rmse_m':(.15,.02),
        'max_error_m':(.10,.02),'std_patch_m':(.10,.01)}
    assert result['metric_weights']=={name:weight for name,(weight,scale) in cost_terms.items()}
    assert result['routes'];rows=[]
    for route in result['routes']:
        max_raw=0.;max_actual_raw=0.;max_ik=0.;min_margin=np.inf;min_wheel=np.inf;max_fk=0.;max_com=0.;count=0;ceiling_gaps=[]
        for p in route['frames']:
            feet=np.array(p['feet']);body=np.array(p['body']);R=np.array(p['rotation']);normals=np.array(p['normals']);q=np.array(p['joints']);support=np.array(p['support'],bool)
            assert support.sum() in (3,4)
            assert (p['swing_leg'] is not None)==(support.sum()==3)
            np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-10)
            assert np.all(q>=robot.limits[:,:,0]-1e-9) and np.all(q<=robot.limits[:,:,1]+1e-9)
            for leg in np.flatnonzero(support):
                owner=p['owners'][leg];uv=feet[leg,:2]-surface.m[owner,:2]
                assert surface.m[owner,FIELDS.index('valid')]>0
                if not surface.config.binary_overhead and surface.config.reject_overhead_occupancy:assert surface.m[owner,FIELDS.index('clearance_cost')]<=1e-10
                if result['config'].get('binary_overhead',False):
                    stack=surface.query(feet[leg,:2],occupied=True)
                    match=[o for o in stack if abs(o['point'][2]-feet[leg,2])<.012]
                    assert match and all(o['overhead_passable']==1 for o in match)
                assert inside(uv[None,:],surface.polygon(owner))[0]
                assert abs(predict(surface.m,owner,feet[leg,:2])-feet[leg,2])<1e-8
                distance=surface.raw.query(feet[leg])[0];max_raw=max(max_raw,float(distance));assert distance<=result['config']['raw_support_m']+1e-8
                count+=1
            com=robot.com(body,R,q);max_com=max(max_com,float(np.linalg.norm(com-p['com'])))
            wheel_ids=[robot.mj.mj_name2id(robot.model,robot.mj.mjtObj.mjOBJ_BODY,f'{name}_wheel') for name in LEG_NAMES]
            actual=robot.data.xpos[wheel_ids];fk_error=np.linalg.norm(actual-np.array(p['wheels']),axis=1).max();max_fk=max(max_fk,float(fk_error));assert fk_error<1e-8
            wheel_gap=float(surface.raw.query(actual)[0].min());min_wheel=min(min_wheel,wheel_gap)
            assert wheel_gap>=result['config'].get('wheel_clearance_m',.078)-1e-9
            error=np.linalg.norm(actual-(feet+robot.radius*normals),axis=1).max();max_ik=max(max_ik,float(error));assert error<=result['config']['ik_accept_m']+1e-8
            actual_contacts=actual-robot.radius*normals
            for leg in np.flatnonzero(support):
                owner=p['owners'][leg]
                assert inside((actual_contacts[leg,:2]-surface.m[owner,:2])[None,:],surface.polygon(owner))[0]
                distance=surface.raw.query(actual_contacts[leg])[0];max_actual_raw=max(max_actual_raw,float(distance))
                assert distance<=result['config']['raw_support_m']+1e-8
            hull=ConvexHull(feet[support,:2]);margin=np.min(-(hull.equations[:,:2]@com[:2]+hull.equations[:,2]));min_margin=min(min_margin,float(margin));assert margin>=result['config']['zmp_margin_m']-1e-9
            np.testing.assert_allclose(p['zmp'][:2],com[:2],atol=1e-8)
            hull_actual=ConvexHull(actual_contacts[support,:2])
            actual_margin=float(np.min(-(hull_actual.equations[:,:2]@com[:2]+hull_actual.equations[:,2])))
            min_margin=min(min_margin,actual_margin)
            assert actual_margin>=result['config']['zmp_margin_m']-1e-9
            plane=np.linalg.lstsq(np.c_[feet[support,:2],np.ones(support.sum())],feet[support,2],rcond=None)[0]
            assert abs(p['zmp'][2]-np.r_[com[:2],1.]@plane)<1e-9
            # Re-evaluate the oriented body box from the complete raw cloud.
            local=(surface.points-body)@R;assert not np.any(np.all(np.abs(local)<=robot.half_body+result['config']['clearance_m'],axis=1))
            if p.get('ceiling_z_m') is not None:
                gap=p['ceiling_z_m']-body[2]-np.abs(R[2])@robot.half_body
                assert gap>=result['config']['clearance_m']-1e-8;ceiling_gaps.append(float(gap))
            expected=np.mean([sum(weight*min(1.,max(0.,float(surface.m[i,FIELDS.index(name)])/scale))
                for name,(weight,scale) in cost_terms.items()) for i in p['owners']])
            assert abs(expected-p['terrain_cost'])<1e-10
        assert max_com<1e-8
        route_statistics=route.get('planning_statistics',result['statistics'])
        search=route_statistics['searches'][-1]
        assert np.linalg.norm(np.array(route['nodes'][-1]['body'])[:2]-search['goal_requested'])<=result['config']['grid_m']*.8+1e-8
        for node,key in ((route['nodes'][0],'start_support_z_m'),(route['nodes'][-1],'goal_support_z_m')):
            if search.get(key) is not None:
                assert abs(predict(surface.m,node['center_owner'],np.array(node['body'])[:2])-search[key])<=.04+1e-8
        changes=[];jumps=[]
        for a,b in zip(route['nodes'][:-1],route['nodes'][1:]):
            fa=np.array(a['feet']);fb=np.array(b['feet']);mid=(fa[:,:2]+fb[:,:2])/2
            gap=np.abs(predict(surface.m,np.array(a['owners']),mid)-predict(surface.m,np.array(b['owners']),mid))
            assert gap.max()<=result['config']['step_limit_m']+1e-6
            changes.append(np.abs(fa[:,2]-fb[:,2]));jumps.append(gap)
        flagged=sum(np.any(c>result['config']['lift_flag_m']+1e-6) for c in changes)
        assert flagged==sum(any(e['lift_flag']) for e in route['events'])
        support_heights=[p['feet'][leg][2] for p in route['frames'] for leg in range(4) if p['support'][leg]]
        under=[]
        if scene=='layer-choice':
            spec=next(c for c in result['test_cases'] if c['id']==route['case_id'])
            if spec['route_mode']=='upper':
                crossing=spec['crossing']
                assert any(crossing['x_min']<=p['body'][0]<=crossing['x_max'] and crossing['y_min']<=p['body'][1]<=crossing['y_max']
                    and min(f[2] for f in p['feet'])>=crossing['minimum_support_z_m'] for p in route['frames'])
                order=[p['swing_leg'] for p in route['frames'] if p['phase']=='prepare' and any(p.get('change_flag',[]))]
                assert order and len(order)%4==0
                assert order==list(result['config']['foot_change_order'])*(len(order)//4)
            else:assert max(abs(z) for z in support_heights)<.015
            for p in route['frames']:
                flag=p.get('change_flag',[False]*4);assert sum(flag)<=1
                if any(flag):assert flag[p['swing_leg']] and p['transition_reason']=='layer_switch'
                assert abs(p['body'][1])<1.2
        if scene.startswith('table-low-'):
            under=[p for p in route['frames'] if abs(p['body'][0])<1.0 and abs(p['body'][1])<.5 and p['body'][2]<.65]
            spec=next(c for c in result['test_cases'] if c['id']==route['case_id'])
            assert bool(under)==spec['expected_underpass']
            assert max(abs(b['body_height_m']-a['body_height_m']) for a,b in zip(route['frames'][:-1],route['frames'][1:]))<=.0100001
            assert max(abs(z) for z in support_heights)<1e-6
        rows.append(dict(case_id=route.get('case_id',scene),label=route['label'],branch=route['branch'],nodes=len(route['nodes']),frames=len(route['frames']),
            supported_contacts_checked=count,maximum_raw_distance_m=max_raw,maximum_ik_residual_m=max_ik,
            maximum_actual_fk_contact_raw_distance_m=max_actual_raw,
            maximum_mujoco_fk_error_m=max_fk,maximum_mujoco_com_error_m=max_com,minimum_zmp_margin_m=min_margin,
            support_z_range_m=[min(support_heights),max(support_heights)],
            body_z_range_m=[min(p['body'][2] for p in route['frames']),max(p['body'][2] for p in route['frames'])],
            minimum_observed_ceiling_gap_m=min(ceiling_gaps) if ceiling_gaps else None,
            minimum_raw_wheel_center_distance_m=min_wheel,
            under_table_frames=len(under),body_lowering_frames=sum(p['phase']=='body_lowering' for p in route['frames']),
            sequential_change_frames=sum(any(p.get('change_flag',[])) for p in route['frames']),
            lift_flagged_edges=int(flagged),single_leg_swing_frames=sum(p['phase']=='swing' for p in route['frames']),
            maximum_same_xy_function_jump_m=float(np.max(jumps,initial=0)),maximum_target_height_change_m=float(np.max(changes,initial=0)),
            route_length_m=float(sum(np.linalg.norm(np.array(b['body'])[:2]-np.array(a['body'])[:2]) for a,b in zip(route['nodes'][:-1],route['nodes'][1:])))))
    assert all(sha(surface.folder/name)==digest for name,digest in result['original_files_sha256'].items())
    for case in result.get('test_cases',[]):
        assert (case['routes']>0)==case.get('expected_route',True),(case['id'],case['status'])
    audit=dict(passed=True,scene=scene,elapsed_s=time.perf_counter()-started,result_sha256=sha(path),routes=rows,
        scope='Discrete poses and explicit intermediate samples only; not continuous collision or dynamic tracking certification.',
        unknown_support_contacts=0,raw_or_model_files_modified=False)
    (path.parent/'audit.json').write_text(json.dumps(finite_json(audit),ensure_ascii=False,indent=2)+'\n');print(scene,rows,flush=True)
    return audit


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument('--scenes',nargs='+',choices=list(CASES),default=list(CASES));args=parser.parse_args()
    r=SurfaceRobot()
    for s in args.scenes:audit_scene(s,r)

"""Independent support/geometry audit; crossing motion remains unvalidated."""
import json
import time
import numpy as np
from .assessment_export import sha,finite_json
from .convex_surface_assessment import FIELDS,inside
from .boundary_heights import predict
from .surface_astar import RESULT,Config,SurfaceMap,Planner
from .surface_robot import SurfaceRobot,support_margin,LEG_NAMES
from .layer_projection_planner import LayerGraph,crossing_allowed


def audit_scene(scene,robot=None):
    started=time.perf_counter();robot=robot or SurfaceRobot();path=RESULT/scene/'result.json';result=json.loads(path.read_text())
    surface=SurfaceMap(scene,Config(**result['config']));graph=LayerGraph(surface);rows=[]
    terms={'slope_deg':(.30,45.),'normal_spread_deg':(.20,15.),'rmse_m':(.15,.02),'max_error_m':(.10,.02),'std_patch_m':(.10,.01)}
    assert result['metric_weights']=={key:v[0] for key,v in terms.items()}
    for p in result['layer_graph']['portals']:
        q=np.asarray(p['xy']);delta=abs(float(predict(surface.m,p['left'],q)-predict(surface.m,p['right'],q)))
        assert abs(delta-p['delta_m'])<1e-10 and crossing_allowed(delta,p['boundary_xy_gap_m'])
    maps={m['id']:m for m in result['projection_maps']}
    for route in result['routes']:
        mapping=maps[route['map_id']];allowed=set(mapping['allowed_layers']);max_raw=0.;max_ik=0.;min_zmp=np.inf;min_wheel=np.inf;count=0;gaps=[]
        for frame in route['frames']:
            feet=np.asarray(frame['feet']);normals=np.asarray(frame['normals']);body=np.asarray(frame['body']);R=np.asarray(frame['rotation']);joints=np.asarray(frame['joints'])
            assert all(frame['support']) and frame['swing_leg'] is None
            assert not any(frame['lift_flag']) and not any(frame['change_flag'])
            np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-10)
            assert np.all(joints>=robot.limits[:,:,0]-1e-9) and np.all(joints<=robot.limits[:,:,1]+1e-9)
            com=robot.com(body,R,joints)
            wheel_ids=[robot.mj.mj_name2id(robot.model,robot.mj.mjtObj.mjOBJ_BODY,name+'_wheel') for name in LEG_NAMES]
            wheels=robot.data.xpos[wheel_ids].copy();actual=wheels-robot.radius*normals
            np.testing.assert_allclose(wheels,frame['wheels'],atol=1e-8);np.testing.assert_allclose(com,frame['com'],atol=1e-8)
            error=float(np.linalg.norm(wheels-(feet+robot.radius*normals),axis=1).max());max_ik=max(max_ik,error)
            assert error<=surface.config.ik_accept_m+1e-8
            for leg,owner in enumerate(frame['owners']):
                assert int(graph.labels[owner]) in allowed and surface.m[owner,FIELDS.index('valid')]>0
                assert abs(float(predict(surface.m,owner,feet[leg,:2]))-feet[leg,2])<1e-8
                for contact in (feet[leg],actual[leg]):
                    assert inside((contact[:2]-surface.m[owner,:2])[None,:],surface.polygon(owner))[0]
                    distance=float(surface.raw.query(contact)[0]);max_raw=max(max_raw,distance);assert distance<=surface.config.raw_support_m+1e-8
                count+=1
                options=surface.query(feet[leg,:2],occupied=True)
                matching=[p for p in options if abs(p['point'][2]-feet[leg,2])<.012]
                assert matching and all(p['overhead_passable'] for p in matching)
            margin=min(support_margin(com,feet),support_margin(com,actual));min_zmp=min(min_zmp,margin)
            assert margin>=surface.config.zmp_margin_m-1e-9
            np.testing.assert_allclose(frame['zmp'][:2],com[:2],atol=1e-8)
            wheel_gap=float(surface.raw.query(wheels)[0].min());min_wheel=min(min_wheel,wheel_gap);assert wheel_gap>=surface.config.wheel_clearance_m-1e-9
            local=(surface.points-body)@R;assert not np.any(np.all(np.abs(local)<=robot.half_body+surface.config.clearance_m,axis=1))
            if frame.get('ceiling_z_m') is not None:
                gap=frame['ceiling_z_m']-body[2]-np.abs(R[2])@robot.half_body;gaps.append(float(gap));assert gap>=surface.config.clearance_m-1e-8
            cost=np.mean([sum(weight*min(1.,max(0.,surface.m[i,FIELDS.index(name)]/scale)) for name,(weight,scale) in terms.items()) for i in frame['owners']])
            assert abs(cost-frame['terrain_cost'])<1e-10
        crossings=0
        for a,b in zip(route['nodes'][:-1],route['nodes'][1:]):
            old=graph.labels[a['owners']];new=graph.labels[b['owners']]
            if np.any(old!=new):
                crossings+=1
                for leg in np.flatnonzero(old!=new):
                    assert graph.match(old[leg],new[leg],np.asarray(a['feet'])[leg,:2],np.asarray(b['feet'])[leg,:2]) is not None
        assert crossings==len(route['events'])
        if route['map_mode']=='base':assert crossings==0 and allowed=={mapping['root_layer']}
        for event in route['events']:assert event['motion_validated'] is False and event['requires_layer_transition'] and not any(event['lift_flag'])
        # All rolling frames stay on the same four layer IDs until a declared crossing.
        for a,b in zip(route['frames'][:-1],route['frames'][1:]):
            if b['phase']=='layer_entry':continue
            np.testing.assert_array_equal(graph.labels[a['owners']],graph.labels[b['owners']])
        length=sum(np.linalg.norm(np.asarray(b['body'])[:2]-np.asarray(a['body'])[:2]) for a,b in zip(route['nodes'][:-1],route['nodes'][1:]))
        underpass=dict(underpass_frames=None,underpass_clearance_m=None,underpass_minimum_body_height_m=None,
            underpass_minimum_ceiling_gap_m=None,body_lowering_frames=None,underpass_normal_height_feasible=None)
        if scene.startswith('platform-underpass-'):
            from .scene_fit_catalog import WEB
            context=json.loads((WEB/scene/'scene.json').read_text());slab=context['tabletop']
            low,high=np.array(slab['min']),np.array(slab['max'])
            under=[p for p in route['frames'] if np.all(np.array(p['body'][:2])>low[:2]+.4)
                and np.all(np.array(p['body'][:2])<high[:2]-.4)
                and all(abs(f[2])<.015 for f in p['feet'])]
            assert under, 'Path never entered the second platform underpass'
            assert np.linalg.norm(np.array(route['frames'][-1]['body'][:2])-context['planning_test']['goal_xy_m'])<.16
            assert all(abs(f[2])<.015 for f in route['frames'][-1]['feet'])
            if context['planning_test']['exits_platform']:
                final=route['frames'][-1]
                assert final['body'][0]-robot.half_body[0]-surface.config.clearance_m>high[0]
                assert abs(final['body_height_m']-.40)<1e-6
            physical_gaps=[float(low[2]-p['body'][2]-np.abs(np.array(p['rotation'])[2])@robot.half_body) for p in under]
            assert min(physical_gaps)>=surface.config.clearance_m-1e-8
            h=[p['body_height_m'] for p in under]
            middle=under[len(under)//2];control=Planner(surface,robot,surface.config)
            normal_pose=control.fixed_pose(np.array(middle['body'][:2]),middle['yaw'],np.array(middle['feet']),
                np.array(middle['normals']),middle['owners'],height=.40)
            assert (normal_pose is not None)==(context['clearance_m']>=.5)
            if context['clearance_m']<.5:
                assert max(h)<=.32+1e-6 and any(p['phase']=='body_lowering' for p in route['frames'])
                assert any(p['phase']=='body_raising' for p in route['frames'])
                # The box, including its safety margin, must be below the slab
                # as soon as its XY footprint starts overlapping the slab.
                for p in route['frames']:
                    body=np.array(p['body']);R=np.array(p['rotation']);extent=np.abs(R)@robot.half_body+surface.config.clearance_m
                    overlap=np.all(body[:2]+extent[:2]>low[:2]) and np.all(body[:2]-extent[:2]<high[:2])
                    if overlap and all(abs(f[2])<.015 for f in p['feet']):
                        assert body[2]+extent[2]<low[2]+1e-8
            else:np.testing.assert_allclose(h,.40,atol=1e-6)
            underpass=dict(underpass_frames=len(under),underpass_clearance_m=context['clearance_m'],
                underpass_minimum_body_height_m=min(h),underpass_minimum_ceiling_gap_m=min(physical_gaps),
                body_lowering_frames=sum(p['phase']=='body_lowering' for p in route['frames']),
                underpass_normal_height_feasible=normal_pose is not None)
        rows.append(dict(case_id=route['case_id'],map_id=route['map_id'],map_mode=route['map_mode'],label=route['label'],
            nodes=len(route['nodes']),frames=len(route['frames']),route_length_m=float(length),route_cost=route['cost'],
            supported_contacts_checked=count,maximum_raw_distance_m=max_raw,maximum_ik_residual_m=max_ik,minimum_zmp_margin_m=float(min_zmp),
            minimum_raw_wheel_center_distance_m=min_wheel,minimum_observed_ceiling_gap_m=min(gaps) if gaps else None,
            layer_crossings=crossings,rolling_frames=sum(p['phase']=='rolling' for p in route['frames']),
            generated_swing_frames=0,crossing_motion_validated=False if crossings else None,**underpass))
    assert all(sha(surface.folder/name)==digest for name,digest in result['original_files_sha256'].items())
    for spec in result['test_cases']:
        if spec.get('expected_route') is False:assert spec['routes']==0
    audit=dict(passed=True,scene=scene,elapsed_s=time.perf_counter()-started,result_sha256=sha(path),routes=rows,
        scope='Observed rolling samples and supported crossing endpoints. Boundary crossings below 40cm are planning candidates; no swing/dynamic crossing verification.',
        occupancy_in_cost=False,endpoint_height_lift_rule=False)
    if scene.startswith('platform-underpass-'):
        centers=surface.m[surface.active];interior=np.all((centers[:,:2]>low[:2]+.1)&(centers[:,:2]<high[:2]-.1),axis=1)
        counts={name:int(np.sum(interior&(np.abs(centers[:,FIELDS.index('height')]-z)<.005)))
                for name,z in [('floor_functions',0.),('underside_functions',low[2]),('top_functions',high[2])]}
        assert all(counts.values()),counts
        audit['platform_fitting']=counts
    if scene=='four-platforms':
        from .four_platforms_audit import evaluate
        audit['four_platforms']=evaluate(surface,robot,result,rows)
    (path.parent/'audit.json').write_text(json.dumps(finite_json(audit),ensure_ascii=False,indent=2)+'\n')
    print(scene,'audited routes',len(rows),'contacts',sum(r['supported_contacts_checked'] for r in rows),flush=True)
    return audit

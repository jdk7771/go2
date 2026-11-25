"""Measure the single five-platform run, without requiring an ideal outcome."""
import json
import numpy as np
from .scene_fit_catalog import WEB


def body_intersects_solid(frame,box,half_body,margin=0.):
    """Independent OBB/AABB volume check using constructed-scene truth only."""
    rotation=np.asarray(frame['rotation']);body=np.asarray(frame['body'])
    lo,hi=np.asarray(box['min']),np.asarray(box['max']);delta=body-(lo+hi)/2
    axes=[*rotation.T,*np.eye(3)]
    axes.extend(np.cross(a,b) for a in rotation.T for b in np.eye(3))
    for axis in axes:
        if np.linalg.norm(axis)<1e-10:continue
        a=np.abs(axis@rotation)@(np.asarray(half_body)+margin)
        b=np.abs(axis)@((hi-lo)/2)
        if abs(axis@delta)>=a+b-1e-9:return False
    return True


def evaluate(surface,robot,result,rows):
    meta=json.loads((WEB/'four-platforms/scene.json').read_text());records=[]
    preferred=next((r for r in result['routes'] if r['map_mode']=='overlay'),None)
    if preferred is None:preferred=next(iter(result['routes']),None)
    volume_checks=0
    for route in result['routes']:
        for frame in route['frames']:
            for box in meta['boxes']:
                assert not body_intersects_solid(frame,box,robot.half_body,surface.config.clearance_m), (frame['body'],box['name'])
                volume_checks+=1
    for spec in meta['platforms']:
        k=spec['index'];lo=np.array(spec['tabletop']['min']);hi=np.array(spec['tabletop']['max'])
        floor=top=None;query=np.array(spec['center_xy_m'])
        for dx,dy in [(0.,0.)]+[(x,y) for x in (-.05,-.025,0.,.025,.05) for y in (-.05,-.025,0.,.025,.05)]:
            query=np.array(spec['center_xy_m'])+[dx,dy];stack=surface.query(query,occupied=True)
            floor=next((p for p in stack if abs(p['point'][2])<.015 and p['raw_supported']),None)
            top=next((p for p in stack if abs(p['point'][2]-hi[2])<.015 and p['raw_supported']),None)
            if top is not None and (spec['solid'] or floor is not None):break
        assert top is not None and (spec['solid'] or floor is not None)
        under=[];upper=[];cross_section=[]
        if preferred:
            for frame in preferred['frames']:
                body=np.array(frame['body']);feet=np.array(frame['feet'])
                if not lo[0]+.35<body[0]<hi[0]-.35:continue
                cross_section.append(frame)
                if lo[1]+.35<body[1]<hi[1]-.35:
                    if np.all(np.abs(feet[:,2])<.015):under.append(frame)
                    if np.all(np.abs(feet[:,2]-hi[2])<.015):upper.append(frame)
        if preferred is None:outcome='未找到完整路径'
        elif upper:outcome='跨层上台候选（动作未验证）'
        elif under:outcome='降身下穿' if min(p['body_height_m'] for p in under)<.39 else '正常下穿'
        elif cross_section:outcome='沿小路绕行' if k==5 else '绕行'
        else:outcome='未识别完整穿越'
        gaps=[float(lo[2]-p['body'][2]-np.abs(np.array(p['rotation'])[2])@robot.half_body) for p in under]
        records.append(dict(platform=k,query_xy_m=query.tolist(),top_height_m=float(top['point'][2]),
            lower_raw_supported=floor is not None,clearance_m=float(floor['overhead_clearance_m']) if floor else None,
            overhead_passable=floor['overhead_passable'] if floor else None,
            direct_climb_candidate=int(abs(top['point'][2])<result['config']['step_limit_m']-1e-9),
            full_width=spec['full_width'],actual_outcome=outcome,
            underpass_frames=len(under),upper_support_frames=len(upper),
            minimum_underpass_body_height_m=min((p['body_height_m'] for p in under),default=None),
            minimum_underpass_roof_gap_m=min(gaps) if gaps else None,
            passage_body_y_range_m=[min(p['body'][1] for p in cross_section),max(p['body'][1] for p in cross_section)] if cross_section else None))
    assert records[3]['direct_climb_candidate']==1 and records[3]['overhead_passable']==0
    assert records[4]['direct_climb_candidate']==0 and records[4]['overhead_passable']==0
    maps=[]
    for mapping in result['projection_maps']:
        stats=mapping['statistics'];search=stats['searches'][-1];tree=search.get('tree',[])
        max_x=max((surface.meta['bounds'][0][0]+.6+n['grid'][0]*result['config']['grid_m'] for n in tree),default=None)
        limit=mapping.get('expansion_limit',result['config']['maximum_expansions'])
        maps.append(dict(map_id=mapping['id'],status=mapping['status'],elapsed_s=stats['elapsed_s'],expansion_limit=limit,
            previous_attempts=len(mapping.get('previous_attempts',[])),
            previous_attempt_elapsed_s=sum(a['statistics']['elapsed_s'] for a in mapping.get('previous_attempts',[])),
            a_star_searches=len(stats['searches']),total_expansions=sum(s['expansions'] for s in stats['searches']),
            expansions=search['expansions'],search_limit_reached=search['expansions']>=limit,
            open_exhausted=search.get('open_exhausted'),furthest_search_candidate_x_m=max_x))
    return dict(platforms=records,planning_maps=maps,thresholds=meta['thresholds'],
        full_route_found=preferred is not None,selected_map_id=preferred['map_id'] if preferred else None,
        fifth_bypass=meta['fifth_bypass'],solid_volume_body_checks=volume_checks,
        scope='One full-chain query only. Outcomes measured from returned frames; constructed solid boxes independently audit body endpoints/samples, never guide planning. No forced waypoints. Cross-layer motion unvalidated.')

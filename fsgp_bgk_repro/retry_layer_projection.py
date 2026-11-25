"""Retry one exhausted projection map with a larger A* node budget.

The same saved XYZ, quadratic models, costs and constraints are reused. Other
scenes and the first layer-only map are preserved; the failed attempt is logged.
"""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import time
from .assessment_export import finite_json,sha
from .layer_projection_planner import Config,LayerSurfaceMap,LayerGraph,ProjectionPlanner,SurfaceRobot,RESULT


def run(scene,map_id,budget):
    folder=RESULT/scene;path=folder/'result.json';result=json.loads(path.read_text())
    mapping=next(m for m in result['projection_maps'] if m['id']==map_id)
    assert mapping['status']!='success', 'Successful maps need no retry'
    assert budget>mapping.get('expansion_limit',result['config']['maximum_expansions'])
    config=replace(Config(**result['config']),maximum_expansions=budget)
    started=time.perf_counter();surface=LayerSurfaceMap(scene,config);graph=LayerGraph(surface);surface.graph=graph
    assert surface.hashes==result['original_files_sha256']
    robot=SurfaceRobot();setup=time.perf_counter()-started
    allowed=set(mapping['allowed_layers']);planner=ProjectionPlanner(surface,robot,config,graph,allowed)
    spec=next(s for s in result['test_cases'] if s['id']==mapping['case_id'])
    route,stats=planner.plan_projection(spec)
    mapping.setdefault('previous_attempts',[]).append(dict(status=mapping['status'],
        expansion_limit=mapping.get('expansion_limit',result['config']['maximum_expansions']),statistics=mapping['statistics']))
    mapping.update(status='success' if route else 'no_validated_route',statistics=stats,expansion_limit=budget,retry_setup_s=setup)
    if route:
        route.update(case_id=spec['id'],map_id=map_id,map_mode=mapping['mode'],
            label=spec['label']+' · 叠加可跨层后重规划',branch=int(mapping['mode']=='overlay'),planning_statistics=stats)
        elapsed=0.
        for frame in route['frames']:
            elapsed+=frame['duration_s'];frame['time_s']=elapsed
        route['duration_s']=elapsed;result['routes'].append(route)
    spec.update(status='success' if any(r['case_id']==spec['id'] for r in result['routes']) else 'no_validated_route',
        routes=sum(r['case_id']==spec['id'] for r in result['routes']))
    result.update(status='success' if result['routes'] else 'no_validated_route',
        recommended_map_id=min(result['routes'],key=lambda r:r['cost'])['map_id'] if result['routes'] else None)
    result['statistics'].update(elapsed_s=sum(m['statistics']['elapsed_s'] for m in result['projection_maps']),
        node_evaluations=sum(m['statistics']['node_evaluations'] for m in result['projection_maps']),
        retry_setup_s=setup,previous_attempt_elapsed_s=sum(a['statistics']['elapsed_s'] for m in result['projection_maps'] for a in m.get('previous_attempts',[])))
    result['experiment_sources_sha256']={'retry_layer_projection.py':sha(Path(__file__))}
    result['reproduction_retry_command']=f'python -m fsgp_bgk_repro.retry_layer_projection --scene {scene} --map {map_id} --budget {budget}'
    result['notes'].append(f'{map_id} retried at {budget} expansions; no cost/constraint change; previous exhausted search is retained in previous_attempts.')
    assert all(sha(surface.folder/n)==h for n,h in surface.hashes.items())
    path.write_text(json.dumps(finite_json(result),ensure_ascii=False)+'\n')
    (folder/'layer-tree.json').write_text(json.dumps(finite_json(dict(graph=result['layer_graph'],maps=[{k:v for k,v in m.items() if k not in ('statistics','previous_attempts')} for m in result['projection_maps']])),ensure_ascii=False)+'\n')
    print(scene,map_id,'routes',int(route is not None),'s',round(stats['elapsed_s'],3),'budget',budget,flush=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',required=True);parser.add_argument('--map',required=True);parser.add_argument('--budget',type=int,required=True)
    args=parser.parse_args();run(args.scene,args.map,args.budget)

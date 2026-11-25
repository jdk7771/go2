"""A* on one continuous fitted layer, then its boundary-connected XY overlay.

Layer crossings are high-level connections, not generated swing-leg motions.
The user-set 0.40 m threshold is distinct from the old blind-policy experiment.
"""
from dataclasses import asdict,replace
from pathlib import Path
import argparse
import heapq
import json
import time
import numpy as np
from scipy.spatial import cKDTree
from .assessment_export import sha,finite_json
from .boundary_heights import extract,predict,GROUP_GAP,GROUP_HEIGHT
from .surface_astar import SurfaceMap,Planner,Config,CASES,RESULT
from .surface_robot import SurfaceRobot

CROSSING_LIMIT_M=.40


def crossing_allowed(height_m,xy_gap_m):
    return abs(height_m)<CROSSING_LIMIT_M-1e-9 and xy_gap_m<=GROUP_GAP+1e-9


def segment_distance(q,a,b):
    delta=np.asarray(b)-a
    t=np.clip(np.dot(np.asarray(q)-a,delta)/max(np.dot(delta,delta),1e-20),0,1)
    return float(np.linalg.norm(np.asarray(q)-(a+t*delta)))


class LayerGraph:
    def __init__(self,surface):
        polygons=[surface.polygon(i)+surface.m[i,:2] for i in surface.active]
        records,self.info=extract(surface.m,surface.active,polygons,include_membership=True)
        # The diagnostic extractor first groups models by predictions at both
        # centres. A flat/ramp junction can disagree there yet meet continuously
        # at its actual boundary. Merge such regions before planning.
        parent={r['id']:r['id'] for r in self.info['regions']};pair_max={}
        def root(k):
            while parent[k]!=k:parent[k]=parent[parent[k]];k=parent[k]
            return k
        for p in records:
            if p['boundary_xy_gap_m']>GROUP_GAP+1e-9:continue
            pair=tuple(sorted((p['left_region'],p['right_region'])))
            pair_max[pair]=max(pair_max.get(pair,0.),p['delta_m'])
        for (a,b),height in pair_max.items():
            if height<=GROUP_HEIGHT+1e-9:parent[root(b)]=root(a)
        remap={k:root(k) for k in parent}
        merged=np.array([remap.get(int(k),int(k)) for k in self.info['function_regions']])
        groups=[]
        for k in sorted(set(remap.values())):
            members=self.info['function_ids'][merged==k]
            groups.append(dict(id=k,functions=len(members),median_center_height_m=float(np.median(predict(surface.m,members,surface.m[members,:2])))))
        self.info.update(regions=groups,function_regions=merged,retained_components=len(groups))
        for p in records:p['left_region']=remap[p['left_region']];p['right_region']=remap[p['right_region']]
        records=[p for p in records if p['left_region']!=p['right_region']]
        self.regions={r['id']:r for r in self.info['regions']}
        self.labels=np.full(len(surface.m),-1,int)
        self.labels[self.info['function_ids']]=self.info['function_regions']
        self.links={r:set() for r in self.regions};unique={}
        for r in records:
            if not crossing_allowed(r['delta_m'],r['boundary_xy_gap_m']):continue
            key=(*sorted((r['left_region'],r['right_region'])),*np.round(r['xy']/.04).astype(int))
            if key not in unique or r['boundary_xy_gap_m']<unique[key]['boundary_xy_gap_m']:unique[key]=r
        self.portals=list(unique.values());self.by_pair={}
        for k,p in enumerate(self.portals):
            a,b=p['left_region'],p['right_region'];self.links[a].add(b);self.links[b].add(a)
            self.by_pair.setdefault(tuple(sorted((a,b))),[]).append(k)
        self.trees={pair:cKDTree([self.portals[k]['xy'] for k in ids]) for pair,ids in self.by_pair.items()}
        self.tree=cKDTree([p['xy'] for p in self.portals]) if self.portals else None

    def reachable(self,root):
        parent={int(root):None};queue=[int(root)]
        for a in queue:
            for b in sorted(self.links.get(a,())):
                if b not in parent:parent[b]=a;queue.append(b)
        return parent

    def match(self,a,b,old_xy,new_xy):
        pair=tuple(sorted((int(a),int(b))))
        if pair not in self.by_pair:return None
        middle=(np.asarray(old_xy)+new_xy)/2
        ids=self.trees[pair].query_ball_point(middle,np.linalg.norm(np.asarray(new_xy)-old_xy)/2+.08)
        options=[self.by_pair[pair][j] for j in ids if segment_distance(self.portals[self.by_pair[pair][j]]['xy'],np.asarray(old_xy),np.asarray(new_xy))<=.08]
        return min(options,key=lambda k:np.linalg.norm(self.portals[k]['xy']-middle)) if options else None

    def metadata(self):
        return dict(regions=list(self.regions.values()),portals=self.portals,
            continuous_prediction_tolerance_m=GROUP_HEIGHT,maximum_domain_gap_m=GROUP_GAP,
            crossing_height_exclusive_m=CROSSING_LIMIT_M,threshold_source='User-set boundary height difference; not measured blind-policy ability.')


class LayerSurfaceMap(SurfaceMap):
    def __init__(self,scene,config,graph=None):
        super().__init__(scene,config);self.graph=graph;self.allowed_layers=None;self.forced_region=None

    def query(self,xy,occupied=False):
        options=super().query(xy,occupied)
        if occupied or self.graph is None:return options
        return [o for o in options if self.graph.labels[o['owner']] in self.allowed_layers
            and (self.forced_region is None or self.graph.labels[o['owner']]==self.forced_region)]


class ProjectionPlanner(Planner):
    def __init__(self,surface,robot,config,graph,allowed):
        super().__init__(surface,robot,config);self.graph=graph;surface.allowed_layers=set(allowed)
        self.edge_info={};self.map_samples={};self.macro_grid_cache={};self.edge_type_cache={}

    def center_options(self,grid):
        result=super().center_options(grid)
        for o in result:
            self.map_samples[(*grid,int(self.graph.labels[o['owner']]))]=dict(xy_m=list(map(float,o['point'][:2])),
                z_m=float(o['point'][2]),layer_id=int(self.graph.labels[o['owner']]),owner=o['owner'],foot_cost=o['cost'],
                overhead_passable=o['overhead_passable'])
        return result

    def edge_kind(self,a,b):
        old=self.graph.labels[a['owners']];new=self.graph.labels[b['owners']];portals=[]
        for leg in np.flatnonzero(old!=new):
            match=self.graph.match(old[leg],new[leg],a['feet'][leg,:2],b['feet'][leg,:2])
            if match is None:return None
            portals.append(dict(leg=int(leg),portal_id=int(match),from_layer=int(old[leg]),to_layer=int(new[leg])))
        return dict(kind='layer_crossing' if portals else 'rolling',contacts=portals)

    def macro_targets(self,pose):
        if self.graph.tree is None or len(self.surface.allowed_layers)<2:return []
        own=int(self.graph.labels[pose['center_owner']]);cache_key=(*pose['key'][:2],own)
        if cache_key in self.macro_grid_cache:
            return [n for grid,owner in self.macro_grid_cache[cache_key] if (n:=self.node(grid,pose['key'][2],owner)) is not None]
        near={}
        for k in self.graph.tree.query_ball_point(pose['body'][:2],.65):
            p=self.graph.portals[k];a,b=p['left_region'],p['right_region']
            if own not in (a,b):continue
            other=b if a==own else a
            if other not in self.surface.allowed_layers:continue
            key=(other,*np.round(p['xy']/.16).astype(int))
            if key not in near:near[key]=k
        selected=sorted(near.values(),key=lambda k:np.linalg.norm(self.graph.portals[k]['xy']-pose['body'][:2]))[:12]
        targets=[];seen=set();geometry=[]
        for k in selected:
            p=self.graph.portals[k];other=p['right_region'] if own==p['left_region'] else p['left_region']
            normal=np.asarray(p['outward_normal_xy'])*(1 if own==p['left_region'] else -1)
            for distance in (.32,.48,.64):
                grid=tuple(np.round((np.asarray(p['xy'])+normal*distance-self.low)/self.cfg.grid_m).astype(int))
                for option in self.center_options(grid):
                    if self.graph.labels[option['owner']]!=other:continue
                    key=(*grid,option['owner'])
                    if key in seen:continue
                    seen.add(key);geometry.append((grid,option['owner']));nxt=self.node(grid,pose['key'][2],option['owner'])
                    if nxt is not None:targets.append(nxt)
        self.macro_grid_cache[cache_key]=geometry
        return targets

    def search_projection(self,start_xy,goal_xy,start_z=None,goal_z=None):
        try:
            start=self.nearby_endpoint(start_xy,0,start_z)
            self.nearby_endpoint(goal_xy,0,goal_z)
        except RuntimeError:return None,dict(expansions=0,reason='No supported stance on this projected map',tree=[])
        goal=np.asarray(goal_xy);queue=[(float(np.linalg.norm(start['body'][:2]-goal)),0.,0,start['key'])]
        costs={start['key']:0.};parents={start['key']:None};serial=0;expansions=0;last=None
        while queue and expansions<self.cfg.maximum_expansions:
            _,cost,_,key=heapq.heappop(queue)
            if cost!=costs.get(key):continue
            pose=self.nodes[key];expansions+=1;i,j,h,_=key
            at_height=goal_z is None or abs(float(predict(self.surface.m,pose['center_owner'],pose['body'][:2]))-goal_z)<=.04
            if np.linalg.norm(pose['body'][:2]-goal)<=self.cfg.grid_m*.8 and at_height:last=key;break
            candidates=[]
            for heading,direction in list(enumerate(self.directions))+[((h+d)%8,np.zeros(2,int)) for d in (-1,1)]:
                grid=(i+int(direction[0]),j+int(direction[1]))
                for option in self.center_options(grid):
                    nxt=self.node(grid,heading,option['owner'])
                    if nxt is not None:candidates.append(nxt)
            candidates+=self.macro_targets(pose)
            for nxt in candidates:
                edge=(key,nxt['key'])
                if edge in self.blocked:continue
                distance=float(np.linalg.norm(nxt['body'][:2]-pose['body'][:2]))
                heading=nxt['key'][2];turn=min(abs(heading-h),8-abs(heading-h))
                proposed=cost+distance*(1+2*nxt['cost'])+.015*turn
                if proposed>=costs.get(nxt['key'],np.inf):continue
                if edge not in self.edge_type_cache:self.edge_type_cache[edge]=self.edge_kind(pose,nxt)
                info=self.edge_type_cache[edge]
                if info is None:continue
                # A macro connection must actually cross a recorded boundary.
                if distance>self.cfg.grid_m*1.5 and info['kind']!='layer_crossing':continue
                costs[nxt['key']]=proposed;parents[nxt['key']]=key;self.edge_info[edge]=info;serial+=1
                heapq.heappush(queue,(proposed+np.linalg.norm(nxt['body'][:2]-goal),proposed,serial,nxt['key']))
        keys=[]
        if last is not None:
            while last is not None:keys.append(last);last=parents[last]
        index={key:k for k,key in enumerate(parents)}
        tree=[dict(id=k,parent=None if parents[key] is None else index[parents[key]],grid=list(key[:2]),heading=key[2],owner=key[3],cost=costs[key]) for k,key in enumerate(parents)]
        return (dict(nodes=[self.nodes[k] for k in reversed(keys)],cost=costs[keys[0]]) if keys else None),dict(expansions=expansions,tree=tree,open_exhausted=not queue)

    def rolling_edge(self,a,b):
        distance=np.linalg.norm(b['body'][:2]-a['body'][:2]);angle=(b['yaw']-a['yaw']+np.pi)%(2*np.pi)-np.pi
        count=max(1,int(np.ceil(distance/self.cfg.edge_sample_m)),int(np.ceil(abs(angle)/np.deg2rad(7.5))));frames=[]
        layers=self.graph.labels[a['owners']]
        for t in np.arange(1,count+1)/count:
            chosen=[]
            for leg in range(4):
                xy=a['feet'][leg,:2]*(1-t)+b['feet'][leg,:2]*t;expected=a['feet'][leg,2]*(1-t)+b['feet'][leg,2]*t
                self.surface.forced_region=int(layers[leg])
                try:options=self.surface.near(xy,expected)
                finally:self.surface.forced_region=None
                if not options:return None
                chosen.append(min(options,key=lambda o:abs(o['point'][2]-expected)+.01*o['cost']))
            frame=self.fixed_pose(a['body'][:2]*(1-t)+b['body'][:2]*t,a['yaw']+angle*t,
                np.array([o['point'] for o in chosen]),np.array([o['normal'] for o in chosen]),[o['owner'] for o in chosen])
            if frame is None:return None
            previous=frames[-1] if frames else a;change=frame['body_height_m']-previous['body_height_m']
            if abs(change)>1e-8:
                stance=previous if change<0 else frame
                heights=np.linspace(previous['body_height_m'],frame['body_height_m'],int(np.ceil(abs(change)/.01))+1)
                for height in (heights[1:] if change<0 else heights):
                    p=self.fixed_pose(stance['body'][:2],stance['yaw'],stance['feet'],stance['normals'],stance['owners'],height=height)
                    if p is None:return None
                    p.update(phase='body_lowering' if change<0 else 'body_raising',duration_s=.1);frames.append(p)
                if change>0:continue
            frame.update(phase='rolling',duration_s=max(.03,distance/count/self.cfg.speed_m_s));frames.append(frame)
        return frames

    def validate_projection(self,path):
        frames=[dict(path[0],phase='four_contact',duration_s=.3)];events=[];bad=[]
        for a,b in zip(path[:-1],path[1:]):
            info=self.edge_info[(a['key'],b['key'])]
            if info['kind']=='rolling':
                segment=self.rolling_edge(a,b)
                if segment is None:bad.append((a['key'],b['key']));continue
                frames+=segment
            else:
                # Show the two supported stances; do not invent a swing trajectory.
                start=len(frames);frames.extend([dict(a,phase='layer_exit',duration_s=.4),dict(b,phase='layer_entry',duration_s=.4)])
                events.append(dict(frame=start,frame_end=len(frames),kind='layer_crossing',contacts=info['contacts'],
                    lift_flag=[False]*4,requires_layer_transition=True,motion_validated=False))
        return (None,bad) if bad else (dict(frames=frames,events=events),[])

    def plan_projection(self,spec):
        started=time.perf_counter();searches=[];route=None
        for _ in range(self.cfg.maximum_replans):
            candidate,stats=self.search_projection(spec['start'],spec['goal'],spec.get('start_z'),spec.get('goal_z'));searches.append(stats)
            if candidate is None:break
            checked,bad=self.validate_projection(candidate['nodes'])
            if bad:self.blocked.update(bad);continue
            route=dict(candidate,**checked);break
        stats=dict(elapsed_s=time.perf_counter()-started,node_evaluations=self.evaluations,invalid_edges=len(self.blocked),
            searches=searches,tree=searches[-1]['tree'],projection_samples=list(self.map_samples.values()),
            layer_tree=list(self.layer_tree.values()))
        return route,stats


def specifications(scene):
    if scene=='layer-choice':return [dict(id='bridge',label='封闭通道',start=[-5.28,-.6],goal=[5.28,-.6],start_z=0.,goal_z=0.)]
    return [{k:v for k,v in spec.items() if k not in ('maximum_routes','route_mode','crossing','expected_underpass')} for spec in CASES[scene]]


def run_scene(scene,robot):
    config=replace(Config(),step_limit_m=CROSSING_LIMIT_M,lift_flag_m=None,ordered_switches=False,maximum_expansions=22000)
    started=time.perf_counter();surface=LayerSurfaceMap(scene,config);graph=LayerGraph(surface);surface.graph=graph
    setup_s=time.perf_counter()-started;routes=[];cases=[];maps=[];initial_stance=None
    for spec in specifications(scene):
        surface.allowed_layers=set(graph.regions);finder=ProjectionPlanner(surface,robot,config,graph,graph.regions)
        try:initial=finder.nearby_endpoint(spec['start'],0,spec.get('start_z'));root=int(graph.labels[initial['center_owner']])
        except RuntimeError:
            cases.append(dict(**spec,status='no_supported_start',routes=0));continue
        if initial_stance is None:initial_stance=dict(initial,phase='four_contact',duration_s=0.,time_s=0.)
        parent=graph.reachable(root);passes=[('base',{root})]
        if len(parent)>1:passes.append(('overlay',set(parent)))
        for mode,allowed in passes:
            planner=ProjectionPlanner(surface,robot,config,graph,allowed);route,stats=planner.plan_projection(spec)
            map_id=spec['id']+'-'+mode
            maps.append(dict(id=map_id,case_id=spec['id'],mode=mode,root_layer=root,allowed_layers=sorted(allowed),parent_layers=parent,
                projection_z_m=graph.regions[root]['median_center_height_m'],status='success' if route else 'no_validated_route',statistics=stats))
            if route:
                route.update(case_id=spec['id'],map_id=map_id,map_mode=mode,label=(spec['label'] or '路径')+(' · 本层滚动' if mode=='base' else ' · 叠加可跨层后重规划'),branch=int(mode=='overlay'),planning_statistics=stats)
                t=0.
                for frame in route['frames']:
                    t+=frame['duration_s'];frame['time_s']=t
                route['duration_s']=t;routes.append(route)
            print(scene,map_id,'routes',int(route is not None),'s',round(stats['elapsed_s'],3),flush=True)
        cases.append(dict(**spec,status='success' if any(r['case_id']==spec['id'] for r in routes) else 'no_validated_route',routes=sum(r['case_id']==spec['id'] for r in routes)))
    weights=dict(zip(surface.names,map(float,surface.weights)))
    result=dict(schema_version=2,planner='layer-projection-astar',scene=scene,name=surface.meta['name'],profile='qcircle-adaptive',
        status='success' if routes else 'no_validated_route',config=asdict(config),robot=robot.metadata(),routes=routes,test_cases=cases,initial_stance=initial_stance,
        projection_maps=maps,layer_graph=graph.metadata(),metric_weights=weights,
        statistics=dict(elapsed_s=sum(m['statistics']['elapsed_s'] for m in maps),surface_setup_s=setup_s,node_evaluations=sum(m['statistics']['node_evaluations'] for m in maps)),
        obstacles=dict(fitted_wall_patches=[],adjusted_frames=0,maximum_shift_m=0.),
        original_files_sha256=surface.hashes,recommended_map_id=min(routes,key=lambda r:r['cost'])['map_id'] if routes else None,
        sources_sha256={n:sha(Path(__file__).parent/n) for n in ('layer_projection_planner.py','boundary_heights.py','surface_astar.py','surface_robot.py','surface_layers.py')},
        notes=['Same continuous fitted layer is rolling. No endpoint-height lift rule and no rolling-to-swing fallback.',
            'Project only XY and terrain cost; keep real z, original support and layer identity. Cross-layer edges exist only at observed boundaries below 0.40m.',
            'Layer-crossing edges are high-level candidates. Endpoint IK/ZMP is checked; no swing trajectory or dynamic execution is certified.'])
    folder=RESULT/scene;folder.mkdir(parents=True,exist_ok=True)
    (folder/'result.json').write_text(json.dumps(finite_json(result),ensure_ascii=False)+'\n')
    (folder/'layer-tree.json').write_text(json.dumps(finite_json(dict(graph=result['layer_graph'],maps=[{k:v for k,v in m.items() if k!='statistics'} for m in maps])),ensure_ascii=False)+'\n')
    assert all(sha(surface.folder/n)==digest for n,digest in surface.hashes.items())
    return result


def run():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--scenes',nargs='+',choices=list(CASES),default=list(CASES));args=parser.parse_args()
    robot=SurfaceRobot()
    return [run_scene(scene,robot) for scene in args.scenes]


if __name__=='__main__':run()

"""A* on one continuous fitted layer, then its boundary-connected XY overlay.

Layer crossings are high-level connections, not generated swing-leg motions.
The user-set 0.40 m threshold is distinct from the old blind-policy experiment.
"""
from dataclasses import asdict,replace
from pathlib import Path
import argparse
import hashlib
import heapq
import json
import math
import pickle
import time
import numpy as np
from scipy.spatial import cKDTree
from .assessment_export import sha,finite_json
from .boundary_heights import extract,predict,GROUP_GAP,GROUP_HEIGHT
from .surface_astar import SurfaceMap,Planner,Config,CASES,RESULT
from .surface_robot import SurfaceRobot

CROSSING_LIMIT_M=.40
LAYER_GRAPH_CACHE=Path(__file__).resolve().parent/'outputs'/'layer_graph_cache'


def crossing_allowed(height_m,xy_gap_m):
    return abs(height_m)<CROSSING_LIMIT_M-1e-9 and xy_gap_m<=GROUP_GAP+1e-9


def segment_distance(q,a,b):
    delta=np.asarray(b)-a
    t=np.clip(np.dot(np.asarray(q)-a,delta)/max(np.dot(delta,delta),1e-20),0,1)
    return float(np.linalg.norm(np.asarray(q)-(a+t*delta)))


def _graph_cache_path(surface):
    hashes=getattr(surface,'hashes',None)
    if not isinstance(hashes,dict):return None
    keys=('models.bin','polygons.bin','polygon-offsets.bin')
    if not all(k in hashes for k in keys):return None
    sig='|'.join([*(str(hashes[k]) for k in keys),f'{GROUP_GAP:.9f}',f'{GROUP_HEIGHT:.9f}',f'{CROSSING_LIMIT_M:.9f}','v1'])
    digest=hashlib.sha256(sig.encode('ascii')).hexdigest()[:24]
    return LAYER_GRAPH_CACHE/f'{digest}.pkl'


class LayerGraph:
    def __init__(self,surface):
        cache_path=_graph_cache_path(surface)
        loaded=None
        if cache_path is not None and cache_path.exists():
            try:loaded=pickle.loads(cache_path.read_bytes())
            except Exception:loaded=None
        if loaded is not None:
            self.info,self.regions,self.labels,self.links,self.portals,self.by_pair=loaded
        else:
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
            if cache_path is not None:
                try:
                    cache_path.parent.mkdir(parents=True,exist_ok=True)
                    tmp=cache_path.with_suffix('.tmp')
                    tmp.write_bytes(pickle.dumps((self.info,self.regions,self.labels,self.links,self.portals,self.by_pair),protocol=pickle.HIGHEST_PROTOCOL))
                    tmp.replace(cache_path)
                except Exception:pass
        self.trees={pair:cKDTree([self.portals[k]['xy'] for k in ids]) for pair,ids in self.by_pair.items()}
        self.pair_coords={pair:np.array([self.portals[k]['xy'] for k in ids],dtype=float) for pair,ids in self.by_pair.items()}
        self.pair_ids={pair:np.asarray(ids,dtype=int) for pair,ids in self.by_pair.items()}
        self.tree=cKDTree([p['xy'] for p in self.portals]) if self.portals else None

    def reachable(self,root):
        parent={int(root):None};queue=[int(root)]
        for a in queue:
            for b in sorted(self.links.get(a,())):
                if b not in parent:parent[b]=a;queue.append(b)
        return parent

    def match(self,a,b,old_xy,new_xy):
        ia,ib=int(a),int(b)
        pair=(ia,ib) if ia<=ib else (ib,ia)
        coords=self.pair_coords.get(pair)
        if coords is None:return None
        ox,oy=float(old_xy[0]),float(old_xy[1]);nx,ny=float(new_xy[0]),float(new_xy[1])
        mx=.5*(ox+nx);my=.5*(oy+ny);dx=nx-ox;dy=ny-oy
        seg_len2=dx*dx+dy*dy
        ids=self.trees[pair].query_ball_point((mx,my),.5*math.sqrt(seg_len2)+.08)
        if not ids:return None
        pts=coords[ids]
        vx=pts[:,0]-ox;vy=pts[:,1]-oy
        inv_len2=1./(seg_len2 if seg_len2>1e-20 else 1e-20)
        t=np.clip((vx*dx+vy*dy)*inv_len2,0.,1.)
        ex=pts[:,0]-(ox+t*dx);ey=pts[:,1]-(oy+t*dy)
        ok=(ex*ex+ey*ey)<=.0064+1e-12
        if not np.any(ok):return None
        ok_pts=pts[ok];ok_idx=np.asarray(ids)[ok]
        d2=(ok_pts[:,0]-mx)**2+(ok_pts[:,1]-my)**2
        return int(self.pair_ids[pair][ok_idx[int(np.argmin(d2))]])

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
        labels=self.graph.labels;allowed=self.allowed_layers;forced=self.forced_region
        if forced is None:
            return [o for o in options if int(labels[o['owner']]) in allowed]
        return [o for o in options if int(labels[o['owner']])==forced and forced in allowed]


class ProjectionPlanner(Planner):
    def __init__(self,surface,robot,config,graph,allowed,heuristic_weight=1.0):
        super().__init__(surface,robot,config);self.graph=graph;surface.allowed_layers=set(allowed)
        self.heuristic_weight=float(heuristic_weight)
        self.edge_info={};self.map_samples={};self.macro_grid_cache={};self.edge_type_cache={}
        self._heuristic_cache={};self._moves_by_h=[
            [(hd,int(d[0]),int(d[1]),float(math.hypot(d[0],d[1])*config.grid_m)) for hd,d in enumerate(self.directions)]
            +[((h+d)%8,0,0,0.) for d in (-1,1)]
            for h in range(8)]

    def center_options(self,grid):
        cached=self.centers.get(grid)
        if cached is not None:return cached
        result=super().center_options(grid)
        labels=self.graph.labels
        for o in result:
            lay=int(labels[o['owner']])
            self.map_samples[(*grid,lay)]=dict(xy_m=list(map(float,o['point'][:2])),
                z_m=float(o['point'][2]),layer_id=lay,owner=o['owner'],foot_cost=o['cost'],
                overhead_passable=o['overhead_passable'])
        return result

    def edge_kind(self,a,b):
        old=self.graph.labels[a['owners']];new=self.graph.labels[b['owners']];portals=[]
        for leg in np.flatnonzero(old!=new):
            match=self.graph.match(old[leg],new[leg],a['feet'][leg,:2],b['feet'][leg,:2])
            if match is None:return None
            portals.append(dict(leg=int(leg),portal_id=int(match),from_layer=int(old[leg]),to_layer=int(new[leg])))
        return dict(kind='layer_crossing' if portals else 'rolling',contacts=portals)

    def _query_grid_options(self,grid):
        """Lightweight grid center query without populating layer_tree/map_samples."""
        cached=self.centers.get(grid)
        if cached is not None:return cached
        gx=float(self.low[0])+grid[0]*self.cfg.grid_m;gy=float(self.low[1])+grid[1]*self.cfg.grid_m
        if gx<self.low[0]-1e-12 or gy<self.low[1]-1e-12 or gx>self.high[0]+1e-12 or gy>self.high[1]+1e-12:return []
        return self.surface.query((gx,gy))

    def _macro_geometry(self,grid,own,use_center_options=True):
        if self.graph.tree is None or len(self.surface.allowed_layers)<2:return []
        cache_key=(grid[0],grid[1],own)
        cached=self.macro_grid_cache.get(cache_key)
        if cached is not None:return cached
        bx=float(self.low[0])+grid[0]*self.cfg.grid_m;by=float(self.low[1])+grid[1]*self.cfg.grid_m
        near={}
        allowed=self.surface.allowed_layers;portals=self.graph.portals
        for k in self.graph.tree.query_ball_point((bx,by),.65):
            p=portals[k];a,b=p['left_region'],p['right_region']
            if own!=a and own!=b:continue
            other=b if a==own else a
            if other not in allowed:continue
            px,py=float(p['xy'][0]),float(p['xy'][1])
            key=(other,int(round(px/.16)),int(round(py/.16)))
            if key not in near:near[key]=k
        if not near:
            self.macro_grid_cache[cache_key]=[]
            return []
        selected=sorted(near.values(),key=lambda k:math.hypot(float(portals[k]['xy'][0])-bx,float(portals[k]['xy'][1])-by))[:12]
        seen=set();geometry=[];labels=self.graph.labels;grid_m=self.cfg.grid_m
        lox,loy=float(self.low[0]),float(self.low[1])
        opt_fn=self.center_options if use_center_options else self._query_grid_options
        for k in selected:
            p=portals[k];other=p['right_region'] if own==p['left_region'] else p['left_region']
            sign=1. if own==p['left_region'] else -1.
            nx=float(p['outward_normal_xy'][0])*sign;ny=float(p['outward_normal_xy'][1])*sign
            px=float(p['xy'][0]);py=float(p['xy'][1])
            for distance in (.32,.48,.64):
                tg=(int(round((px+nx*distance-lox)/grid_m)),int(round((py+ny*distance-loy)/grid_m)))
                for option in opt_fn(tg):
                    if int(labels[option['owner']])!=other:continue
                    key=(tg[0],tg[1],option['owner'])
                    if key in seen:continue
                    seen.add(key);geometry.append((tg,option['owner']))
        self.macro_grid_cache[cache_key]=geometry
        return geometry

    def macro_targets(self,pose):
        if self.graph.tree is None or len(self.surface.allowed_layers)<2:return []
        own=int(self.graph.labels[pose['center_owner']]);grid=pose['key'][:2];h=pose['key'][2]
        return [n for tg,owner in self._macro_geometry(grid,own,use_center_options=True) if (n:=self.node(tg,h,owner)) is not None]

    def _build_heuristic_2d(self,goal_xy,goal_z=None):
        """Reverse 2D Dijkstra over passable projected (grid, layer) states.

        Strictly admissible lower bound for 4D A* edge costs:
        every 4D move (8-neighbor or macro portal jump) is present with its exact
        XY distance (<= distance*(1 + 2*nxt['cost']) + 0.015*turn).
        """
        gx,gy=float(goal_xy[0]),float(goal_xy[1])
        cache_key=(round(gx,6),round(gy,6),None if goal_z is None else round(float(goal_z),6))
        cached=self._heuristic_cache.get(cache_key)
        if cached is not None:return cached
        grid_m=self.cfg.grid_m;lox,loy=float(self.low[0]),float(self.low[1])
        hix,hiy=float(self.high[0]),float(self.high[1])
        nx=int(math.floor((hix-lox)/grid_m+1e-6))+1
        ny=int(math.floor((hiy-loy)/grid_m+1e-6))+1
        labels=self.graph.labels;allowed=self.surface.allowed_layers
        goal_rad=grid_m*.8+1e-9
        active_nodes=set();cell_layers={};goal_seeds=[]
        query_fn=self.surface.query
        for i in range(nx):
            x=lox+i*grid_m
            if x>hix+1e-12:continue
            dx_g=x-gx
            for j in range(ny):
                y=loy+j*grid_m
                if y>hiy+1e-12:continue
                opts=query_fn((x,y))
                if not opts:continue
                near_goal=(dx_g*dx_g+(y-gy)*(y-gy))<=goal_rad*goal_rad
                ls=set()
                for o in opts:
                    lay=int(labels[o['owner']])
                    ls.add(lay)
                    node=(i,j,lay)
                    active_nodes.add(node)
                    if near_goal and (goal_z is None or abs(float(o['point'][2])-goal_z)<=.04+1e-9):
                        goal_seeds.append(node)
                cell_layers[(i,j)]=ls
        if not goal_seeds:
            self._heuristic_cache[cache_key]={}
            return {}
        # Build reverse cross-layer edges (both 1-step boundary crossings and macro portal jumps).
        rev_cross={}
        if len(allowed)>=2 and self.graph.tree is not None:
            ptree=self.graph.tree;portals=self.graph.portals
            step_dirs=((1,0,grid_m),(1,1,grid_m*math.sqrt(2.)),(0,1,grid_m),(-1,1,grid_m*math.sqrt(2.)),
                (-1,0,grid_m),(-1,-1,grid_m*math.sqrt(2.)),(0,-1,grid_m),(1,-1,grid_m*math.sqrt(2.)),(0,0,0.))
            for (i,j),ls in cell_layers.items():
                x=lox+i*grid_m;y=loy+j*grid_m
                p_ids=ptree.query_ball_point((x,y),.72)
                if not p_ids:continue
                for lay in ls:
                    others=set()
                    for k in p_ids:
                        p=portals[k];a,b=p['left_region'],p['right_region']
                        if lay==a and b in allowed:others.add(b)
                        elif lay==b and a in allowed:others.add(a)
                    if not others:continue
                    src=(i,j,lay)
                    for dx,dy,w in step_dirs:
                        nl=cell_layers.get((i+dx,j+dy))
                        if not nl:continue
                        for other in others:
                            if other in nl:
                                rev_cross.setdefault((i+dx,j+dy,other),[]).append((src,w))
                    for tg,owner in self._macro_geometry((i,j),lay,use_center_options=False):
                        other=int(labels[owner]);dst=(tg[0],tg[1],other)
                        if dst in active_nodes:
                            mw=math.hypot((tg[0]-i)*grid_m,(tg[1]-j)*grid_m)
                            rev_cross.setdefault(dst,[]).append((src,mw))
        # Reverse Dijkstra from all goal seeds.
        dist_2d={};hq=[]
        for seed in set(goal_seeds):
            dist_2d[seed]=0.;heapq.heappush(hq,(0.,seed))
        diag=grid_m*math.sqrt(2.)
        nb_steps=((1,0,grid_m),(-1,0,grid_m),(0,1,grid_m),(0,-1,grid_m),
            (1,1,diag),(1,-1,diag),(-1,1,diag),(-1,-1,diag))
        while hq:
            d,cur=heapq.heappop(hq)
            if d!=dist_2d.get(cur):continue
            ci,cj,clay=cur
            for dx,dy,w in nb_steps:
                nxt=(ci+dx,cj+dy,clay)
                if nxt not in active_nodes:continue
                nd=d+w
                if nd<dist_2d.get(nxt,math.inf):
                    dist_2d[nxt]=nd;heapq.heappush(hq,(nd,nxt))
            cross=rev_cross.get(cur)
            if cross:
                for nxt,w in cross:
                    nd=d+w
                    if nd<dist_2d.get(nxt,math.inf):
                        dist_2d[nxt]=nd;heapq.heappush(hq,(nd,nxt))
        self._heuristic_cache[cache_key]=dist_2d
        return dist_2d

    def search_projection(self,start_xy,goal_xy,start_z=None,goal_z=None):
        try:
            start=self.nearby_endpoint(start_xy,0,start_z)
            self.nearby_endpoint(goal_xy,0,goal_z)
        except RuntimeError:return None,dict(expansions=0,reason='No supported stance on this projected map',tree=[])
        dist_2d=self._build_heuristic_2d(goal_xy,goal_z)
        labels=self.graph.labels;grid_m=self.cfg.grid_m
        start_key=start['key'];start_h=dist_2d.get((start_key[0],start_key[1],int(labels[start_key[3]])))
        if start_h is None:
            return None,dict(expansions=0,reason='Unreachable on 2D projected layer graph',tree=[],open_exhausted=True)
        goal=np.asarray(goal_xy,dtype=float);gx,gy=float(goal[0]),float(goal[1])
        hw=self.heuristic_weight;goal_rad=grid_m*.8;macro_lim=grid_m*1.5
        # Heap item: (f_score, g_score, serial, evaluated, parent_key, target_key, dist, turn, is_macro)
        queue=[(hw*start_h,0.,0,True,None,start_key,0.,0,False)]
        costs={start_key:0.};parents={start_key:None};closed=set() if hw>1.0 else None;serial=0;expansions=0;last=None
        nodes=self.nodes;blocked=self.blocked;edge_type_cache=self.edge_type_cache;edge_info=self.edge_info
        moves_by_h=self._moves_by_h;has_macro=self.graph.tree is not None and len(self.surface.allowed_layers)>=2
        while queue and expansions<self.cfg.maximum_expansions:
            _,g_val,_,evaluated,pkey,key,edist,eturn,is_macro=heapq.heappop(queue)
            if not evaluated:
                if closed is not None and key in closed:continue
                if g_val>=costs.get(key,math.inf):continue
                edge=(pkey,key)
                if edge in blocked:continue
                nxt=nodes.get(key)
                if key not in nodes:
                    nxt=self.node((key[0],key[1]),key[2],key[3])
                if nxt is None:continue
                pcost=costs.get(pkey,math.inf)
                proposed=pcost+edist*(1.+2.*nxt['cost'])+.015*eturn
                if proposed>=costs.get(key,math.inf):continue
                info=edge_type_cache.get(edge)
                if edge not in edge_type_cache:
                    info=self.edge_kind(nodes[pkey],nxt);edge_type_cache[edge]=info
                if info is None:continue
                if is_macro and edist>macro_lim and info['kind']!='layer_crossing':continue
                costs[key]=proposed;parents[key]=pkey;edge_info[edge]=info;serial+=1
                h_val=dist_2d[(key[0],key[1],int(labels[key[3]]))]
                heapq.heappush(queue,(proposed+hw*h_val,proposed,serial,True,None,key,0.,0,False))
                continue
            if g_val!=costs.get(key):continue
            if closed is not None:
                if key in closed:continue
                closed.add(key)
            pose=nodes[key];expansions+=1;i,j,h,own_id=key
            bx,by=float(pose['body'][0]),float(pose['body'][1])
            if math.hypot(bx-gx,by-gy)<=goal_rad:
                at_height=goal_z is None or abs(self.surface.predict_scalar(pose['center_owner'],bx,by)-goal_z)<=.04
                if at_height:last=key;break
            # Generate 8-neighbor + 2 in-place turn candidates lazily.
            for heading,di,dj,dist in moves_by_h[h]:
                gi=i+di;gj=j+dj;grid=(gi,gj)
                turn=abs(heading-h)
                if turn>4:turn=8-turn
                g_lb=g_val+dist+.015*turn
                for option in self.center_options(grid):
                    oid=option['owner'];lay=int(labels[oid])
                    h_val=dist_2d.get((gi,gj,lay))
                    if h_val is None:continue
                    tkey=(gi,gj,heading,oid)
                    if g_lb>=costs.get(tkey,math.inf):continue
                    edge=(key,tkey)
                    if edge in blocked:continue
                    if tkey in nodes:
                        nxt=nodes[tkey]
                        if nxt is None:continue
                        proposed=g_val+dist*(1.+2.*nxt['cost'])+.015*turn
                        if proposed>=costs.get(tkey,math.inf):continue
                        info=edge_type_cache.get(edge)
                        if edge not in edge_type_cache:
                            info=self.edge_kind(pose,nxt);edge_type_cache[edge]=info
                        if info is None:continue
                        costs[tkey]=proposed;parents[tkey]=key;edge_info[edge]=info;serial+=1
                        heapq.heappush(queue,(proposed+hw*h_val,proposed,serial,True,None,tkey,0.,0,False))
                    else:
                        serial+=1
                        heapq.heappush(queue,(g_lb+hw*h_val,g_lb,serial,False,key,tkey,dist,turn,False))
            # Generate macro cross-layer portal candidates lazily.
            if has_macro:
                own_layer=int(labels[own_id])
                for tg,oid in self._macro_geometry((i,j),own_layer,use_center_options=True):
                    gi,gj=tg;lay=int(labels[oid])
                    h_val=dist_2d.get((gi,gj,lay))
                    if h_val is None:continue
                    tkey=(gi,gj,h,oid)
                    dist=math.hypot((gi-i)*grid_m,(gj-j)*grid_m)
                    g_lb=g_val+dist
                    if g_lb>=costs.get(tkey,math.inf):continue
                    edge=(key,tkey)
                    if edge in blocked:continue
                    if tkey in nodes:
                        nxt=nodes[tkey]
                        if nxt is None:continue
                        proposed=g_val+dist*(1.+2.*nxt['cost'])
                        if proposed>=costs.get(tkey,math.inf):continue
                        info=edge_type_cache.get(edge)
                        if edge not in edge_type_cache:
                            info=self.edge_kind(pose,nxt);edge_type_cache[edge]=info
                        if info is None:continue
                        if dist>macro_lim and info['kind']!='layer_crossing':continue
                        costs[tkey]=proposed;parents[tkey]=key;edge_info[edge]=info;serial+=1
                        heapq.heappush(queue,(proposed+hw*h_val,proposed,serial,True,None,tkey,0.,0,False))
                    else:
                        serial+=1
                        heapq.heappush(queue,(g_lb+hw*h_val,g_lb,serial,False,key,tkey,dist,0,True))
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


def run_scene(scene,robot,heuristic_weight=1.0):
    config=replace(Config(),step_limit_m=CROSSING_LIMIT_M,lift_flag_m=None,ordered_switches=False,maximum_expansions=22000)
    started=time.perf_counter();surface=LayerSurfaceMap(scene,config);graph=LayerGraph(surface);surface.graph=graph
    setup_s=time.perf_counter()-started;routes=[];cases=[];maps=[];initial_stance=None
    for spec in specifications(scene):
        surface.allowed_layers=set(graph.regions);finder=ProjectionPlanner(surface,robot,config,graph,graph.regions,heuristic_weight=heuristic_weight)
        try:initial=finder.nearby_endpoint(spec['start'],0,spec.get('start_z'));root=int(graph.labels[initial['center_owner']])
        except RuntimeError:
            cases.append(dict(**spec,status='no_supported_start',routes=0));continue
        if initial_stance is None:initial_stance=dict(initial,phase='four_contact',duration_s=0.,time_s=0.)
        parent=graph.reachable(root);passes=[('base',{root})]
        if len(parent)>1:passes.append(('overlay',set(parent)))
        for mode,allowed in passes:
            planner=ProjectionPlanner(surface,robot,config,graph,allowed,heuristic_weight=heuristic_weight);route,stats=planner.plan_projection(spec)
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
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--scenes',nargs='+',choices=list(CASES),default=list(CASES))
    parser.add_argument('--heuristic-weight',type=float,default=1.0,help='A* heuristic weight (1.0=exact optimal, e.g. 1.15=fast near-optimal)')
    args=parser.parse_args()
    robot=SurfaceRobot()
    return [run_scene(scene,robot,heuristic_weight=args.heuristic_weight) for scene in args.scenes]


if __name__=='__main__':run()

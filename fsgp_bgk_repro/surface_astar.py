"""Initial A* over fitted surfaces with four supported contacts, soft IK and ZMP.

Only A* selects routes. Foot selection, body-height sampling and quasi-static
single-leg phases are explicit geometric checks; there is no second planner.
"""
from dataclasses import dataclass,asdict,replace
import heapq
import json
from pathlib import Path
import time
import numpy as np
from scipy.spatial import cKDTree
from .assessment_export import sha,finite_json
from .convex_surface_assessment import OUT,FIELDS,inside
from .boundary_heights import predict
from .surface_robot import SurfaceRobot,support_margin,body_rotation
from .m20_step_capability import OUT as CAPABILITY
from .scene_fit_catalog import WEB
from .surface_obstacles import refine
from .surface_layers import classify_stack,describe_stack

RESULT=WEB/'surface-planner/results'
CASES={
    'hill':[dict(id='hill',label='丘陵路径',start=[-3.8,-2.8],goal=[3.8,2.8],maximum_routes=1)],
    'surface-route-test':[dict(id='layered',label=None,start=[-2.4,0.],goal=[2.4,0.],maximum_routes=2)],
    'flat':[dict(id='flat',label='平地直行',start=[-3.2,0.],goal=[3.2,0.],start_z=0.,goal_z=0.)],
    'table':[
        dict(id='table-under',label='桌下穿行',start=[-2.24,0.],goal=[2.24,0.],start_z=0.,goal_z=0.),
        dict(id='table-top',label='桌面行走（初始在桌面）',start=[-.88,0.],goal=[.88,0.],start_z=1.2,goal_z=1.2),
        dict(id='ground-to-table',label='地面上桌能力反例',start=[-2.24,0.],goal=[0.,0.],start_z=0.,goal_z=1.2,expected_route=False)],
    'flat-corridor-platforms':[dict(id='platforms-bypass',label='高低台地面绕行',start=[-3.2,-1.6],goal=[3.2,-1.6],start_z=0.,goal_z=0.)]
}
for _scene,_height in [('table-low-040',.40),('table-low-042',.42),('table-low-060',.60)]:
    CASES[_scene]=[dict(id='low-table-under' if _height>.40 else 'low-table-bypass',
        label=f'净空 {_height*100:.0f} cm · '+('降身桌下穿行' if _height<.5 and _height>.4 else ('桌下穿行' if _height>.4 else '净空不足，地面绕行')),
        start=[-2.24,0.],goal=[2.24,0.],start_z=0.,goal_z=0.,expected_underpass=_height>.40)]
    if _height==.40:
        CASES[_scene].insert(0,dict(id='low-table-under-rejected',label='40 cm 净空下方目标不可达',
            start=[-2.24,0.],goal=[0.,0.],start_z=0.,goal_z=0.,expected_route=False))
CASES['layer-choice']=[
    dict(id='duck-under',label='方案一 · 压低身体穿桥下',start=[-5.28,-.60],goal=[5.28,-.60],start_z=0.,goal_z=0.,route_mode='lower'),
    dict(id='climb-over',label='方案二 · 逐腿上台阶走桥面',start=[-5.28,-.60],goal=[5.28,-.60],start_z=0.,goal_z=0.,route_mode='upper',
        crossing=dict(x_min=-.4,x_max=.4,y_min=.60,y_max=1.,minimum_support_z_m=.46))]
for _scene,_height in [('platform-underpass-070',.70),('platform-underpass-042',.42)]:
    CASES[_scene]=[dict(id='extended-platform-under',label=f'第二台下方净空 {_height*100:.0f} cm · '+
        ('降身穿过第二台下方' if _height<.5 else '正常高度穿过第二台下方'),
        start=[-3.2,-1.6],goal=[3.2,-1.6],start_z=0.,goal_z=0.,expected_underpass=True)]
CASES['four-platforms']=[dict(id='all-platforms',label='五平台链 · 全程自由规划',
    start=[-9.2,-2.4],goal=[9.2,-2.4],start_z=0.,goal_z=0.)]


@dataclass(frozen=True)
class Config:
    grid_m:float=.16
    raw_support_m:float=.03
    foot_adjust_m:float=.035
    ik_accept_m:float=.012
    zmp_margin_m:float=.01
    clearance_m:float=.025
    wheel_clearance_m:float=.078
    lift_flag_m:float=.04
    edge_sample_m:float=.02
    step_limit_m:float=.04
    speed_m_s:float=.12
    maximum_expansions:int=18000
    maximum_replans:int=16
    reject_overhead_occupancy:bool=True
    binary_overhead:bool=True
    minimum_passage_m:float=.415
    ordered_switches:bool=True
    foot_change_order:tuple=(0,1,2,3)


class SurfaceMap:
    def __init__(self,scene,config=Config()):
        self.scene=scene;self.config=config;self.folder=OUT/'qcircle-adaptive'/scene
        self.meta=json.loads((self.folder/'data.json').read_text());self.m=np.fromfile(self.folder/'models.bin','<f8').reshape(-1,len(FIELDS))
        self.polygons=np.fromfile(self.folder/'polygons.bin','<f8').reshape(-1,2);self.offsets=np.fromfile(self.folder/'polygon-offsets.bin','<i4')
        self.points=np.fromfile(self.folder/'points.bin','<f4').reshape(-1,3).astype(float);self.raw=cKDTree(self.points);self.rawxy=cKDTree(self.points[:,:2])
        self.active=np.flatnonzero(self.m[:,FIELDS.index('valid')]>0);self.tree=cKDTree(self.m[self.active,:2])
        self.radius=max(np.linalg.norm(self.polygons[self.offsets[i]:self.offsets[i+1]],axis=1).max() for i in self.active)
        # Occupancy remains a diagnostic metric; binary headroom and collision
        # checks decide feasibility. Keep the other five weights unchanged.
        self.cache={};self.weights=np.array([.30,.20,.15,.10,.10]);self.names=['slope_deg','normal_spread_deg','rmse_m','max_error_m','std_patch_m']
        self.scales=np.array([45.,15.,.02,.02,.01]);self.costs=np.clip(self.m[:,[FIELDS.index(k) for k in self.names]]/self.scales,0,1)@self.weights
        self.hashes={n:sha(self.folder/n) for n in ('models.bin','polygons.bin','polygon-offsets.bin','points.bin','data.json')}

    def polygon(self,i):return self.polygons[self.offsets[i]:self.offsets[i+1]]

    def near(self,xy,expected=None):
        """Project a missing target onto an observed convex domain, never fill it."""
        xy=np.asarray(xy);options=self.query(xy)
        if expected is not None:options=[o for o in options if abs(o['point'][2]-expected)<=.04+1e-6]
        if options:return options
        candidates=[]
        for i in self.active[self.tree.query_ball_point(xy,self.radius+self.config.foot_adjust_m)]:
            polygon=self.polygon(i)+self.m[i,:2];a=polygon;b=np.roll(polygon,-1,axis=0);ab=b-a
            t=np.clip(np.einsum('ij,ij->i',xy-a,ab)/np.einsum('ij,ij->i',ab,ab),0,1)
            edge=a+t[:,None]*ab;nearest=edge[np.argmin(np.linalg.norm(edge-xy,axis=1))]
            distance=np.linalg.norm(nearest-xy)
            if distance>self.config.foot_adjust_m:continue
            # Move one micrometre inward to avoid floating-point edge ambiguity.
            nearest=nearest+(self.m[i,:2]-nearest)*1e-5
            for o in self.query(nearest):
                if expected is None or abs(o['point'][2]-expected)<=.04+1e-6:candidates.append((distance,o))
        if not candidates:return []
        best=min(candidates,key=lambda pair:pair[0]+.001*pair[1]['cost'])
        return [best[1]]

    def query(self,xy,occupied=False):
        xy=np.asarray(xy);key=(*np.round(xy,6),occupied)
        if key in self.cache:return self.cache[key]
        ids=self.active[self.tree.query_ball_point(xy,self.radius+1e-8)];found=[]
        for i in ids:
            if not self.config.binary_overhead and not occupied and self.config.reject_overhead_occupancy and self.m[i,FIELDS.index('clearance_cost')]>1e-10:continue
            if not inside((xy-self.m[i,:2])[None,:],self.polygon(i))[0]:continue
            z=float(predict(self.m,i,xy));distance=float(self.raw.query([*xy,z])[0])
            u,v=xy-self.m[i,:2];c=self.m[i,3:9];n=np.array([-(c[1]+2*c[3]*u+c[4]*v),-(c[2]+c[4]*u+2*c[5]*v),1.]);n/=np.linalg.norm(n)
            found.append(dict(owner=int(i),point=np.r_[xy,z],normal=n,cost=float(self.costs[i]),raw_distance_m=distance))
        result=[]
        for option in sorted(found,key=lambda o:(o['raw_distance_m']>self.config.raw_support_m,o['cost'],o['raw_distance_m'])):
            if not any(abs(option['point'][2]-old['point'][2])<.012 for old in result):result.append(option)
        result=classify_stack(result,self.config.minimum_passage_m,self.config.step_limit_m,self.config.raw_support_m)
        if not occupied:result=[o for o in result if o['raw_supported'] and (not self.config.binary_overhead or o['overhead_passable'])]
        self.cache[key]=result;return result

    def contacts(self,xy,yaw,reference,robot):
        c,s=np.cos(yaw),np.sin(yaw);R=np.array([[c,-s],[s,c]])
        nominal=robot.nominal_feet[:,:2]@R.T+xy;chosen=[]
        offsets=np.array([[0,0],[1,0],[-1,0],[0,1],[0,-1],[1,1],[1,-1],[-1,1],[-1,-1]])*self.config.foot_adjust_m
        for location in nominal:
            options=[]
            for offset in offsets:
                q=location+offset
                for option in self.query(q):
                    # A supported nominal contact can still put the wheel into
                    # a stair riser. Try the existing nearby offsets first.
                    wheel=option['point']+robot.radius*option['normal']
                    if self.raw.query(wheel)[0]<self.config.wheel_clearance_m:continue
                    expected=float(predict(self.m,reference,q));difference=abs(option['point'][2]-expected)
                    if difference>.18:continue
                    options.append((option['cost']+.3*np.linalg.norm(offset)/.05+.4*difference/.15,option))
                if options and not np.any(offset):break
            if not options:return None
            chosen.append(min(options,key=lambda p:p[0])[1])
        return chosen

    def body_free(self,body,rotation,robot):
        ids=self.rawxy.query_ball_point(body[:2],np.linalg.norm(robot.half_body[:2])+.12)
        if not ids:return False
        local=(self.points[ids]-body)@rotation
        return not np.any(np.all(np.abs(local)<=robot.half_body+self.config.clearance_m,axis=1))


def jumps(a,b,m):
    q=(a['feet'][:,:2]+b['feet'][:,:2])/2
    boundary=np.abs(predict(m,np.array(a['owners']),q)-predict(m,np.array(b['owners']),q))
    change=np.abs(b['feet'][:,2]-a['feet'][:,2])
    return change,boundary


class Planner:
    def __init__(self,surface,robot,config=Config()):
        self.surface=surface;self.robot=robot;self.cfg=config;self.nodes={};self.centers={};self.blocked=set();self.evaluations=0
        self.route_mode='any';self.crossing=None;self.base_support_z=0.;self.layer_tree={}
        self.low=np.array(surface.meta['bounds'][0][:2])+.6;self.high=np.array(surface.meta['bounds'][1][:2])-.6
        self.directions=np.array([[1,0],[1,1],[0,1],[-1,1],[-1,0],[-1,-1],[0,-1],[1,-1]])

    def center_options(self,grid):
        grid=tuple(grid)
        if grid not in self.centers:
            xy=self.low+np.array(grid)*self.cfg.grid_m
            bounded=np.all(xy<=self.high) and np.all(xy>=self.low)
            self.centers[grid]=self.surface.query(xy) if bounded else []
            if bounded:self.layer_tree[grid]=describe_stack(xy,self.surface.query(xy,occupied=True))
        return self.centers[grid]

    def fixed_pose(self,xy,yaw,feet,normals,owners,support=None,height=None):
        mask=np.ones(4,bool) if support is None else np.asarray(support,bool)
        grounded=feet[mask]
        rotation=body_rotation(grounded,yaw);plane=np.linalg.lstsq(np.c_[grounded[:,:2],np.ones(len(grounded))],grounded[:,2],rcond=None)[0]
        ground=float(np.r_[xy,1.]@plane);options=[]
        upper=[]
        support_top=max(ground,float(grounded[:,2].max()))
        for o in self.surface.query(xy,occupied=True):
            # A tread already beneath a supporting foot is a support surface,
            # even if it lies slightly above the least-squares support plane.
            if o['point'][2]>(support_top+.02 if self.cfg.binary_overhead else ground+.12):upper.append(o['point'][2])
        ceiling=min(upper) if upper else np.inf
        for h in ((.32,.36,.40,.44,.48,.52) if height is None else (height,)):
            body=np.r_[xy,ground+h];ik=self.robot.ik(body,rotation,feet,normals)
            if ik['maximum_residual_m']>self.cfg.ik_accept_m:continue
            if body[2]+np.abs(rotation[2])@self.robot.half_body+self.cfg.clearance_m>ceiling:continue
            cost=ik['soft_cost']+.25*((h-self.robot.nominal_height)/.10)**2
            options.append((cost,body,h,ik))
        for cost,body,height,ik in sorted(options,key=lambda o:o[0]):
            if not self.surface.body_free(body,rotation,self.robot):continue
            if self.surface.raw.query(ik['wheels'])[0].min()<self.cfg.wheel_clearance_m:continue
            actual_contacts=ik['wheels']-self.robot.radius*normals
            if any(not inside((actual_contacts[k,:2]-self.surface.m[owners[k],:2])[None,:],self.surface.polygon(owners[k]))[0]
                   or self.surface.raw.query(actual_contacts[k])[0]>self.cfg.raw_support_m for k in np.flatnonzero(mask)):continue
            com=self.robot.com(body,rotation,ik['q'])
            margin=min(support_margin(com,feet[mask]),support_margin(com,actual_contacts[mask]))
            if margin<self.cfg.zmp_margin_m:continue
            foot_cost=np.array([self.surface.costs[i] for i in owners])
            return dict(body=body,rotation=rotation,yaw=float(yaw),feet=feet.copy(),normals=normals.copy(),owners=list(map(int,owners)),
                wheels=ik['wheels'],joints=ik['q'],ik_max_m=ik['maximum_residual_m'],ik_cost=ik['soft_cost'],
                com=com,zmp=np.r_[com[:2],float(np.r_[com[:2],1.]@plane)],zmp_margin_m=margin,
                body_height_m=height,body_height_cost=.25*((height-self.robot.nominal_height)/.1)**2,
                ceiling_z_m=ceiling,
                overhead_passable=1,overhead_clearance_m=float(ceiling-ground),change_flag=[False]*4,
                foot_costs=foot_cost,terrain_cost=float(foot_cost.mean()),cost=float(foot_cost.mean()+.15*cost+.05*max(0,(.05-margin)/.05)),
                support=mask.tolist(),swing_leg=None,lift_flag=[False]*4)
        return None

    def evaluate(self,xy,yaw,reference):
        chosen=self.surface.contacts(xy,yaw,reference,self.robot)
        if chosen is None:return None
        if self.route_mode=='lower' and any(abs(o['point'][2]-self.base_support_z)>.015 for o in chosen):return None
        pose=self.fixed_pose(xy,yaw,np.array([o['point'] for o in chosen]),np.array([o['normal'] for o in chosen]),[o['owner'] for o in chosen])
        if pose is not None:pose['raw_distances_m']=[o['raw_distance_m'] for o in chosen];pose['center_owner']=int(reference)
        return pose

    def node(self,grid,heading,owner):
        key=(*grid,int(heading),int(owner))
        if key not in self.nodes:
            self.evaluations+=1;pose=self.evaluate(self.low+np.array(grid)*self.cfg.grid_m,heading*np.pi/4,owner)
            if pose is not None:pose['key']=key
            self.nodes[key]=pose
        return self.nodes[key]

    def nearby_endpoint(self,xy,heading,support_z=None):
        center=np.round((np.array(xy)-self.low)/self.cfg.grid_m).astype(int)
        for radius in (0,1,2,3):
            for dx in range(-radius,radius+1):
                for dy in range(-radius,radius+1):
                    g=tuple(center+[dx,dy])
                    for option in self.center_options(g):
                        if support_z is not None and abs(option['point'][2]-support_z)>.04:continue
                        pose=self.node(g,heading,option['owner'])
                        if pose is not None:return pose
        raise RuntimeError('No observed four-contact IK/ZMP stance near endpoint')

    def search(self,start_xy,goal_xy,maximum_routes=1,start_z=None,goal_z=None):
        start=self.nearby_endpoint(start_xy,0,start_z);goal=np.array(goal_xy);reference=start['center_owner']
        def heuristic(pose,branch):
            if self.route_mode=='upper' and not branch:
                lo=np.array([self.crossing['x_min'],self.crossing['y_min']]);hi=np.array([self.crossing['x_max'],self.crossing['y_max']])
                return np.linalg.norm(pose['body'][:2]-np.clip(pose['body'][:2],lo,hi))+np.linalg.norm(goal-np.clip(goal,lo,hi))
            return np.linalg.norm(pose['body'][:2]-goal)
        queue=[];serial=0;initial=(*start['key'],0);gscore={initial:0.};parent={initial:None};heapq.heappush(queue,(heuristic(start,0),0.,serial,initial))
        results={};expansions=0;rejected_cap=0
        while queue and expansions<self.cfg.maximum_expansions:
            _,cost,_,key=heapq.heappop(queue)
            if cost!=gscore.get(key):continue
            i,j,h,owner,mask=key;pose=self.nodes[(i,j,h,owner)];expansions+=1
            at_height=goal_z is None or abs(float(predict(self.surface.m,owner,pose['body'][:2]))-goal_z)<=.04
            if np.linalg.norm(pose['body'][:2]-goal)<=self.cfg.grid_m*.8 and at_height and mask not in results and (self.route_mode!='upper' or mask==1):
                path=[];current=key
                while current is not None:path.append(self.nodes[current[:4]]);current=parent[current]
                results[mask]=dict(nodes=path[::-1],cost=cost,branch=int(mask))
                if len(results)>=maximum_routes:break
            moves=list(enumerate(self.directions))
            if self.cfg.ordered_switches:moves += [((h+d)%8,np.zeros(2,int)) for d in (-1,1)]
            for heading,direction in moves:
                grid=(i+int(direction[0]),j+int(direction[1]))
                if min(grid)<0:continue
                for option in self.center_options(grid):
                    nxt=self.node(grid,heading,option['owner'])
                    if nxt is None:continue
                    edge=(pose['key'],nxt['key'])
                    if edge in self.blocked:continue
                    change,boundary=jumps(pose,nxt,self.surface.m)
                    if boundary.max()>self.cfg.step_limit_m+1e-6:rejected_cap+=1;continue
                    # Align while supported on the same level. A prescribed
                    # four-leg layer switch keeps a fixed body heading.
                    if self.cfg.ordered_switches and (boundary>.015).any() and heading!=h:continue
                    lifted=change>self.cfg.lift_flag_m+1e-6
                    new_mask=mask
                    if self.route_mode=='upper':
                        if (self.crossing['x_min']<=nxt['body'][0]<=self.crossing['x_max'] and self.crossing['y_min']<=nxt['body'][1]<=self.crossing['y_max']
                                and nxt['feet'][:,2].min()>=self.crossing['minimum_support_z_m']):new_mask=1
                    elif maximum_routes>1:
                        q=nxt['body'][:2];level=float(predict(self.surface.m,nxt['center_owner'],q)-predict(self.surface.m,reference,q))
                        if level>.02:new_mask=1
                    target=(*nxt['key'],new_mask);distance=np.linalg.norm(nxt['body'][:2]-pose['body'][:2])
                    turning=min(abs(heading-h),8-abs(heading-h))
                    changes_required=np.logical_or(lifted,boundary>.015) if self.cfg.ordered_switches else lifted
                    proposed=cost+distance*(1.+2*nxt['cost'])+.015*turning+.08*changes_required.sum()
                    if proposed>=gscore.get(target,np.inf):continue
                    gscore[target]=proposed;parent[target]=key;serial+=1
                    heapq.heappush(queue,(proposed+heuristic(nxt,new_mask),proposed,serial,target))
        tree=[dict(id=n,parent=None if p is None else int(index[p]),grid=list(k[:2]),heading=k[2],owner=k[3],branch=k[4],cost=gscore[k])
              for index in [{k:n for n,k in enumerate(parent)}] for n,(k,p) in enumerate(parent.items())]
        return list(results.values()),dict(expansions=expansions,capability_rejected_edges=rejected_cap,tree=tree,
            start_requested=list(start_xy),start_used=start['body'][:2],goal_requested=list(goal_xy),
            start_support_z_m=start_z,goal_support_z_m=goal_z,open_exhausted=not bool(queue))

    def rolling_edge(self,a,b):
        distance=np.linalg.norm(b['body'][:2]-a['body'][:2]);angle=(b['yaw']-a['yaw']+np.pi)%(2*np.pi)-np.pi
        count=max(1,int(np.ceil(distance/self.cfg.edge_sample_m)),int(np.ceil(abs(angle)/np.deg2rad(7.5))))
        frames=[]
        for t in np.arange(1,count+1)/count:
            xy=a['body'][:2]*(1-t)+b['body'][:2]*t;yaw=a['yaw']+angle*t
            chosen=[]
            for leg in range(4):
                q=a['feet'][leg,:2]*(1-t)+b['feet'][leg,:2]*t;expected=a['feet'][leg,2]*(1-t)+b['feet'][leg,2]*t
                compatible=self.surface.near(q,expected)
                if not compatible:return None
                chosen.append(min(compatible,key=lambda o:abs(o['point'][2]-expected)+.01*o['cost']))
            frame=self.fixed_pose(xy,yaw,np.array([o['point'] for o in chosen]),np.array([o['normal'] for o in chosen]),[o['owner'] for o in chosen])
            if frame is None:return None
            if self.cfg.binary_overhead or not self.cfg.reject_overhead_occupancy:
                previous=frames[-1] if frames else a
                change=frame['body_height_m']-previous['body_height_m']
                if abs(change)>1e-8:
                    # Lower before horizontal entry, raise after horizontal exit.
                    stance=previous if change<0 else frame
                    heights=np.linspace(previous['body_height_m'],frame['body_height_m'],int(np.ceil(abs(change)/.01))+1)
                    transition=[]
                    for h in heights[1:] if change<0 else heights:
                        p=self.fixed_pose(stance['body'][:2],stance['yaw'],stance['feet'],stance['normals'],stance['owners'],height=h)
                        if p is None:return None
                        p.update(phase='body_lowering' if change<0 else 'body_raising',duration_s=.1)
                        transition.append(p)
                    if change<0:frames.extend(transition)
                    else:
                        transition[0].update(phase='rolling',duration_s=max(.03,distance/count/self.cfg.speed_m_s))
                        frames.extend(transition);continue
            frame['raw_distances_m']=[o['raw_distance_m'] for o in chosen];frame['phase']='rolling';frame['duration_s']=max(.03,distance/count/self.cfg.speed_m_s);frames.append(frame)
        return frames

    def lift_edge(self,a,b,flags,switch_flags=None):
        feet=a['feet'].copy();normals=a['normals'].copy();owners=list(a['owners']);frames=[]
        bodyxy=a['body'][:2].copy();yaw=a['yaw'];angle=(b['yaw']-a['yaw']+np.pi)%(2*np.pi)-np.pi
        switching=switch_flags is not None and np.any(switch_flags)
        remaining=list(self.cfg.foot_change_order) if switching and self.cfg.ordered_switches else list(range(4))
        # Reposition all four contacts in single-leg phases when a height flag
        # occurs. The threshold flag and the actual swing leg are separate data.
        while remaining:
            solution=None
            for leg in (remaining[:1] if switching and self.cfg.ordered_switches else remaining):
                mask=np.ones(4,bool);mask[leg]=False;centroid=feet[mask,:2].mean(0)
                nextyaw=a['yaw']+angle*(5-len(remaining))/4
                for fraction in (.2,.4,.6,.8,1.):
                    shifted=bodyxy+fraction*(centroid-bodyxy)
                    if np.linalg.norm(shifted-(a['body'][:2]+b['body'][:2])/2)>.24:continue
                    local=[];old=feet[leg].copy();target=b['feet'][leg].copy()
                    for t in (0.,.25,.5,.75,1.):
                        trial=feet.copy();trial[leg]=old*(1-t)+target*t
                        trial[leg,2]+=4*t*(1-t)*.06
                        trial_normals=normals.copy();trial_normals[leg]=(1-t)*normals[leg]+t*b['normals'][leg]
                        trial_normals[leg]/=np.linalg.norm(trial_normals[leg])
                        pose=self.fixed_pose(shifted,nextyaw,trial,trial_normals,owners,mask)
                        if pose is None:break
                        # A small spherical envelope checks swing-wheel clearance.
                        if 0<t<1 and self.surface.raw.query(pose['wheels'][leg])[0]<.07:break
                        pose.update(phase='prepare' if t==0 else ('land' if t==1 else 'swing'),
                            swing_leg=int(leg),lift_flag=flags.tolist(),duration_s=.25)
                        if switching:
                            pose['change_flag']=[k==leg for k in range(4)];pose['transition_reason']='layer_switch'
                        local.append(pose)
                    if len(local)!=5:continue
                    prep=[];ok=True
                    for t in (.25,.5,.75,1.):
                        qxy=bodyxy*(1-t)+shifted*t;qyaw=yaw+(nextyaw-yaw)*t
                        pose=self.fixed_pose(qxy,qyaw,feet,normals,owners)
                        if pose is None:ok=False;break
                        pose.update(phase='weight_shift',duration_s=.15,lift_flag=flags.tolist());prep.append(pose)
                    if not ok:continue
                    solution=(leg,shifted,nextyaw,prep+local);break
                if solution is not None:break
            if solution is None:return None
            leg,bodyxy,yaw,local=solution;frames.extend(local)
            feet[leg]=b['feet'][leg];normals[leg]=b['normals'][leg];owners[leg]=b['owners'][leg];remaining.remove(leg)
        for t in (.25,.5,.75,1.):
            last=self.fixed_pose(bodyxy*(1-t)+b['body'][:2]*t,yaw+((b['yaw']-yaw+np.pi)%(2*np.pi)-np.pi)*t,feet,normals,owners)
            if last is None:return None
            last.update(phase='four_contact',duration_s=.15,lift_flag=flags.tolist());frames.append(last)
        return frames

    def validate(self,path):
        frames=[dict(path[0],phase='four_contact',duration_s=.3)];events=[];bad=[]
        for a,b in zip(path[:-1],path[1:]):
            change,boundary=jumps(a,b,self.surface.m);flags=change>self.cfg.lift_flag_m+1e-6
            switches=boundary>.015 if self.cfg.ordered_switches else np.zeros(4,bool)
            segment=self.lift_edge(a,b,flags,switches) if flags.any() or switches.any() else self.rolling_edge(a,b)
            if segment is None and not flags.any() and not switches.any():
                segment=self.lift_edge(a,b,flags)
                if segment is not None:
                    for frame in segment:frame['transition_reason']='rolling_transition_infeasible'
            if segment is None:
                bad.append((a['key'],b['key']));continue
            if flags.any() or boundary.max()>.02:
                events.append(dict(frame=len(frames),frame_end=len(frames)+len(segment),feet_height_change_m=change,boundary_prediction_jump_m=boundary,
                    lift_flag=flags,change_required=switches,change_order=list(self.cfg.foot_change_order) if switches.any() else [],
                    capability_passed=bool(boundary.max()<=self.cfg.step_limit_m+1e-6)))
            frames.extend(segment)
        return (None,bad) if bad else (dict(frames=frames,events=events),None)

    def plan(self,start,goal,maximum_routes=1,start_z=None,goal_z=None,route_mode='any',crossing=None):
        self.route_mode=route_mode;self.crossing=crossing;self.base_support_z=0. if start_z is None else start_z
        begun=time.perf_counter();valid=[];searches=[]
        for attempt in range(self.cfg.maximum_replans):
            candidates,statistics=self.search(start,goal,maximum_routes,start_z,goal_z);searches.append({k:v for k,v in statistics.items() if k!='tree'})
            if self.cfg.ordered_switches:print('layer search',self.route_mode,attempt+1,'candidates',len(candidates),'expanded',statistics['expansions'],flush=True)
            failed=False;valid=[]
            for candidate in candidates:
                check,bad=self.validate(candidate['nodes'])
                if bad is not None:self.blocked.update(bad);failed=True;continue
                valid.append(dict(candidate,**check))
            if not failed:break
        return valid,dict(elapsed_s=time.perf_counter()-begun,node_evaluations=self.evaluations,cached_nodes=len(self.nodes),
            invalid_edges=len(self.blocked),searches=searches,tree=statistics['tree'])


def run_scene(scene,robot,config):
    if scene.startswith('table-low-'):config=replace(config,reject_overhead_occupancy=False)
    if scene=='layer-choice':config=replace(config,ordered_switches=True,reject_overhead_occupancy=False,maximum_expansions=30000)
    routes=[];cases=[];setup_s=0.;all_obstacles=[]
    for spec in CASES[scene]:
        begun=time.perf_counter();surface=SurfaceMap(scene,config);planner=Planner(surface,robot,config)
        setup_s+=time.perf_counter()-begun
        selected,statistics=planner.plan(spec['start'],spec['goal'],spec.get('maximum_routes',1),spec.get('start_z'),spec.get('goal_z'),spec.get('route_mode','any'),spec.get('crossing'))
        obstacle_result=refine(planner,selected);all_obstacles.append(obstacle_result)
        statistics['layer_tree']=list(planner.layer_tree.values())
        for route in selected:
            route['label']=spec['label'] or ('本层路径' if route['branch']==0 else '经过其他高度的路径')
            route['case_id']=spec['id'];route['planning_statistics']=statistics
        routes.extend(selected)
        cases.append(dict(**spec,status='success' if selected else 'no_validated_route',routes=len(selected),statistics=statistics))
        print(scene,spec['id'],'routes',len(selected),'seconds',round(statistics['elapsed_s'],3),flush=True)
    stats=dict(elapsed_s=sum(c['statistics']['elapsed_s'] for c in cases),
        node_evaluations=sum(c['statistics']['node_evaluations'] for c in cases),
        invalid_edges=sum(c['statistics']['invalid_edges'] for c in cases),
        searches=[s for c in cases for s in c['statistics']['searches']],
        tree=cases[0]['statistics']['tree'],surface_setup_s=setup_s,
        virtual_force_s=sum(o['elapsed_s'] for o in all_obstacles))
    obstacles=dict(all_obstacles[0],elapsed_s=stats['virtual_force_s'],
        fitted_wall_patches=[p for o in all_obstacles for p in o['fitted_wall_patches']],
        adjusted_frames=sum(o['adjusted_frames'] for o in all_obstacles),
        rejected_adjustments=sum(o['rejected_adjustments'] for o in all_obstacles),
        maximum_shift_m=max(o['maximum_shift_m'] for o in all_obstacles))
    folder=RESULT/scene;folder.mkdir(parents=True,exist_ok=True)
    for route in routes:
        route['nodes']=[{k:v for k,v in n.items() if k not in ('rotation','normals')} for n in route['nodes']]
        t=0.
        for frame in route['frames']:t+=frame['duration_s'];frame['time_s']=t
        route['duration_s']=t
    result=dict(scene=scene,name=surface.meta['name'],profile='qcircle-adaptive',status='success' if routes else 'no_validated_route',
        config=asdict(config),robot=robot.metadata(),routes=routes,statistics=stats,test_cases=cases,
        obstacles=obstacles,original_files_sha256=surface.hashes,metric_weights=dict(zip(surface.names,surface.weights)),
        capability_report_sha256=sha(CAPABILITY/'report.json'),
        capabilities=dict(single_step_m=config.step_limit_m,step_source='Existing blind actor forward-step rollouts, 3/3 at 4cm.',
            minimum_body_center_height_m=.32,minimum_passage_m=config.minimum_passage_m,
            crouch_source='Existing IK/body-height samples and geometric torso box, not a height command or crouch rollout of the blind actor.'),
        recommended_case_id=min(routes,key=lambda r:r['cost'])['case_id'] if routes else None,
        sources_sha256={name:sha(Path(__file__).parent/name) for name in
            ('surface_astar.py','surface_robot.py','surface_obstacles.py','m20_step_capability.py','surface_route_scene.py','low_table_scene.py','surface_layers.py','layer_choice_scene.py')},
        notes=['Body terrain cost is the mean of four supported contact costs; unknown contacts are rejected.',
            'Five terrain-cost terms retain weights 0.30/0.20/0.15/0.10/0.10 (sum 0.85). Occupancy is excluded; binary headroom and collision checks remain.',
            'Single-foot target height change >4cm sets lift flag. Same-XY surface jump is a separate capability gate.',
            'Quasi-static geometric planning only. The blind actor step test is not execution of these routes.',
            'Branch 1 records departure >2cm above the initial fitted function; this initial version returns at most one route per branch.',
            'Binary overhead rejects a support option below the minimum passage. Upward candidates also need the measured step capability and validated IK/ZMP transitions.',
            'Climb limit is actor-tested; minimum passage is geometric/IK from the existing height samples, not an actor-tested crouch command.'])
    (folder/'result.json').write_text(json.dumps(finite_json(result),ensure_ascii=False)+'\n')
    layers=dict(scene=scene,minimum_passage_m=config.minimum_passage_m,step_limit_m=config.step_limit_m,
        note='Pointwise upward candidates need the height capability and upper headroom; IK and sampled transitions are checked by A*. Parent links describe local height stacks, not a fixed global layer count.',
        cases=[dict(id=c['id'],stacks=c['statistics']['layer_tree'],search_tree=c['statistics']['tree']) for c in cases])
    (folder/'layer-tree.json').write_text(json.dumps(finite_json(layers),ensure_ascii=False)+'\n')
    print(scene,result['status'],'routes',len(routes),'seconds',stats['elapsed_s'],'nodes',stats['node_evaluations'],flush=True)
    assert all(sha(surface.folder/name)==digest for name,digest in surface.hashes.items())
    return result


def run_legacy():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenes',nargs='+',choices=list(CASES),default=list(CASES));args=parser.parse_args()
    robot=SurfaceRobot();cap=json.loads((CAPABILITY/'report.json').read_text());config=Config(step_limit_m=cap['conservative_step_limit_m'],minimum_passage_m=.32+robot.half_body[2]+Config().clearance_m)
    return [run_scene(scene,robot,config) for scene in args.scenes]


def run():
    from .layer_projection_planner import run as run_projection
    return run_projection()


if __name__=='__main__':run()

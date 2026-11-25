"""Five-platform chain; keep the historical URL, replace its local test variants."""
import json
import numpy as np
from .composite_scene import grid,write_pcd
from .scene_fit_catalog import WEB
from .table_scene import _face

SCENE='four-platforms'
CENTERS=(-7.2,-3.6,0.,3.6,7.2)
SPECS=((.80,None),(.80,.70),(.52,.42),(.30,.20),(.78,.38))
Y_SPANS=((-4.,-.9),(-4.,4.),(-4.,4.),(-4.,4.),(-4.,2.8))


def build(spacing=.025):
    boxes=[];platforms=[];clouds=[];groups=[]
    xy=grid(-10,10,-4,4,spacing);buried=np.zeros(len(xy),bool)
    for k,(cx,(top,clear),(y0,y1)) in enumerate(zip(CENTERS,SPECS,Y_SPANS),1):
        slab=dict(name=f'平台{k}',label=k+2,platform=k,
                  min=[cx-.8,y0,0. if clear is None else clear],max=[cx+.8,y1,top])
        legs=[] if clear is None else [dict(name=f'平台{k}支腿',label=7+k,platform=k,
            min=[x,y,0.],max=[x+.12,y+.12,clear])
            for x in (cx-.8,cx+.68) for y in (y0,y1-.12)]
        objects=[slab,*legs];boxes+=objects
        for box in ([slab] if clear is None else legs):
            buried|=np.all((xy>=np.array(box['min'][:2])-1e-7)&(xy<=np.array(box['max'][:2])+1e-7),axis=1)
        for box in objects:
            lo,hi=np.array(box['min']),np.array(box['max'])
            faces=[_face(lo,hi,axis,side,spacing) for axis in (0,1) for side in (0,1)]
            if box is slab:
                faces.append(_face(lo,hi,2,1,spacing))
                if clear is not None:faces.append(_face(lo,hi,2,0,spacing))
            p=np.concatenate(faces);clouds.append(p);groups.append(np.full(len(p),box['label'],np.uint8))
        platforms.append(dict(index=k,name=f'平台{k}',center_xy_m=[cx,-2.4],
            top_height_m=top,clearance_m=clear,solid=clear is None,tabletop=slab,legs=legs,
            y_range_m=[y0,y1],full_width=k in (2,3,4),right_boundary_y_m=y0,
            expected_behavior={1:'绕行',2:'正常下穿',3:'降身下穿',4:'跨层上台候选',5:'沿小路绕行'}[k]))
    floor=np.c_[xy[~buried],np.zeros((~buried).sum())]
    clouds.insert(0,floor);groups.insert(0,np.zeros(len(floor),np.uint8))
    points,first=np.unique(np.concatenate(clouds).astype('<f4'),axis=0,return_index=True)
    labels=np.concatenate(groups)[first]
    hidden=np.zeros(len(points),bool)
    for platform in platforms:
        for leg in platform['legs']:
            inside=np.all((points[:,:2]>=np.array(leg['min'][:2])-1e-7)&(points[:,:2]<=np.array(leg['max'][:2])+1e-7),axis=1)
            hidden|=inside&((np.abs(points[:,2])<1e-7)|(np.abs(points[:,2]-platform['clearance_m'])<1e-7))
    points,labels=points[~hidden],labels[~hidden]
    meta=dict(id=SCENE,name='五平台通行链 · 绕行、下穿、降身、上台、小路',units='m',
        kind='complete_synthetic_surface',simulated_lidar=False,spacing_m=spacing,
        bounds_m=[points.min(0).tolist(),points.max(0).tolist()],point_count=len(points),
        floor_size_m=[20.,8.],platforms=platforms,boxes=boxes,upper_display_threshold_m=.10,
        initial_forward_world=[1.,0.,0.],initial_right_world=[0.,-1.,0.],
        global_start_xy_m=[-9.2,-2.4],global_goal_xy_m=[9.2,-2.4],
        fifth_bypass=dict(y_range_m=[2.8,4.],width_m=1.2,side='initial robot left (+Y)'),
        removed_current_variants=True,extra_corridor_removed=True,
        fitting_input='XYZ only; labels and platform specifications are auditing/display references only',
        thresholds=dict(minimum_passage_m=.415,crossing_height_exclusive_m=.40,
            threshold_source='Current geometric planner settings; not measured 40cm blind-policy capability'))
    return points,labels,meta


def export():
    p,l,m=build();folder=WEB/SCENE;folder.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(folder/'scene.npz',points=p,labels=l);p.tofile(folder/'points.bin')
    write_pcd(folder/'scene.pcd',p,l,l,np.full(len(p),3),description='Five-platform chain with full-width barriers and a narrow bypass')
    (folder/'scene.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n')
    print(SCENE,'points',len(p),flush=True)
    return m


if __name__=='__main__':export()

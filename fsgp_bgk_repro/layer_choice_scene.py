"""Closed-sided course: duck under a bridge or ascend 4 cm steps over it."""
import json
import numpy as np
from .table_scene import _face,write_pcd
from .scene_fit_catalog import WEB

SCENE='layer-choice'


def build(spacing=.025):
    boxes=[dict(name='桥板',min=[-1.12,-1.5,.42],max=[1.12,1.5,.48])]
    # Twelve solid treads per approach. Ground underneath solids is removed.
    for side in (-1,1):
        for step in range(12):
            lo=-4.96+.32*step;hi=lo+.32;height=.04*(step+1)
            if side==1:lo,hi=-hi,-lo
            boxes.append(dict(name=f'{side}侧第{step+1}级',min=[lo,.10,0.],max=[hi,1.5,height]))
    boxes += [dict(name='侧墙',min=[-6.,y,0.],max=[6.,y+.12,1.3]) for y in (-1.62,1.5)]
    x=np.linspace(-6,6,481);y=np.linspace(-1.5,1.5,121);xy=np.array(np.meshgrid(x,y,indexing='ij')).reshape(2,-1).T
    floor=np.c_[xy,np.zeros(len(xy))];clouds=[floor];labels=[np.zeros(len(floor),np.uint8)]
    for k,box in enumerate(boxes):
        lo,hi=np.array(box['min']),np.array(box['max'])
        faces=[];probes=[]
        for axis in range(3):
            for side in (0,1):
                if axis==2 and side==0 and k!=0:continue
                p=_face(lo,hi,axis,side,spacing);normal=np.eye(3)[axis]*(2*side-1)
                faces.append(p);probes.append(p+normal*1e-5)
        p=np.concatenate(faces);probe=np.concatenate(probes)
        # Suppress coincident internal interfaces of neighbouring solid boxes.
        all_points=p;exposed=np.ones(len(p),bool)
        for j,other in enumerate(boxes):
            if j==k:continue
            low,high=np.array(other['min']),np.array(other['max'])
            exposed &= ~np.all((probe>=low-1e-9)&(probe<=high+1e-9),axis=1)
        clouds.append(all_points[exposed]);labels.append(np.full(exposed.sum(),k+1,np.uint8))
    keep=np.ones(len(floor),bool)
    for box in boxes[1:]:
        if box['min'][2]!=0:continue
        keep &= ~np.all((floor[:,:2]>=np.array(box['min'][:2])-1e-7)&(floor[:,:2]<=np.array(box['max'][:2])+1e-7),axis=1)
    clouds[0]=floor[keep];labels[0]=labels[0][keep]
    points,ids=np.unique(np.concatenate(clouds).astype('<f4'),axis=0,return_index=True)
    labels=np.concatenate(labels)[ids]
    meta=dict(id=SCENE,name='封闭通道 · 降身或逐级上桥',boxes=boxes,point_count=len(points),spacing_m=spacing,
        kind='synthetic_complete_surfaces',labels_used_for_planning=False,
        underside_m=.42,deck_top_m=.48,step_rise_m=.04,step_tread_m=.32,
        corridor_width_m=3.,side_bypass=False,start_xy_m=[-5.28,-.60],goal_xy_m=[5.28,-.60],
        upper_crossing=dict(x_min=-.4,x_max=.4,y_min=.60,y_max=1.,minimum_support_z_m=.46),
        note='The 48 cm deck is reached via twelve 4 cm steps, never a 48 cm vertical jump.')
    return points,labels,meta


def run():
    points,labels,meta=build();folder=WEB/SCENE;folder.mkdir(exist_ok=True)
    points.tofile(folder/'points.bin');np.savez_compressed(folder/'scene.npz',points=points,labels=labels)
    write_pcd(folder/'scene.pcd',points,labels)
    (folder/'scene.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n');print(meta['point_count'],flush=True)


if __name__=='__main__':run()

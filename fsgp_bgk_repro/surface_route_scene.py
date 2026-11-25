"""Small initial-planner test: 3 cm solid deck, front wall and low overhead slab."""
import json
import numpy as np
from .composite_scene import grid,write_pcd
from .table_scene import _face
from .scene_fit_catalog import WEB

SCENE='surface-route-test'


def build(spacing=.025):
    boxes=[dict(name='可切换矮台',min=[-1.2,-.65,0.],max=[1.2,.65,.03]),
           dict(name='迎面立墙',min=[1.5,-.7,0.],max=[1.55,.7,.75]),
           dict(name='低净空顶板',min=[-.5,1.0,.42],max=[.5,1.7,.50])]
    xy=grid(-3,3,-2,2,spacing);keep=np.ones(len(xy),bool)
    for box in boxes[:2]:keep&=~np.all((xy>=np.array(box['min'][:2])-1e-8)&(xy<=np.array(box['max'][:2])+1e-8),axis=1)
    clouds=[np.c_[xy[keep],np.zeros(keep.sum())]];labels=[np.zeros(keep.sum(),np.uint8)]
    for i,box in enumerate(boxes):
        lo,hi=np.array(box['min']),np.array(box['max']);faces=[_face(lo,hi,axis,side,spacing) for axis in (0,1) for side in (0,1)]
        faces.append(_face(lo,hi,2,1,spacing))
        if i==2:faces.append(_face(lo,hi,2,0,spacing))
        cloud=np.concatenate(faces);clouds.append(cloud);labels.append(np.full(len(cloud),i+1,np.uint8))
    points,ids=np.unique(np.concatenate(clouds).astype('<f4'),axis=0,return_index=True)
    return points,np.concatenate(labels)[ids],dict(id=SCENE,name='A* 初版 · 矮台、立墙与顶板',boxes=boxes,
        point_count=len(points),spacing_m=spacing,kind='synthetic_complete_surfaces',labels_used_for_planning=False,
        start_xy_m=[-2.4,0.],goal_xy_m=[2.4,0.],low_deck_height_m=.03)


def run():
    points,labels,meta=build();folder=WEB/SCENE;folder.mkdir(parents=True,exist_ok=True)
    points.tofile(folder/'points.bin');np.savez_compressed(folder/'scene.npz',points=points,labels=labels)
    write_pcd(folder/'scene.pcd',points,labels,labels,np.zeros(len(points)),description='Initial A-star surface/IK/ZMP test')
    (folder/'scene.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n');print(meta)


if __name__=='__main__':run()

"""Standalone low tables; identical XY geometry, only underside height changes."""
import json
import numpy as np
from .table_scene import geometry, _face, _inside_legs_xy, write_pcd
from .scene_fit_catalog import WEB, NAMES

HEIGHTS={'table-low-040':.40,'table-low-042':.42,'table-low-060':.60}


def build(scene,spacing=.025):
    clearance=HEIGHTS[scene];meta=geometry()
    meta['tabletop']['min'][2]=clearance;meta['tabletop']['max'][2]=clearance+.10
    for leg in meta['legs']:leg['max'][2]=clearance
    clouds=[];labels=[]
    for box in [meta['floor'],meta['tabletop'],*meta['legs']]:
        lo,hi=np.array(box['min']),np.array(box['max']);label=box['label']
        faces=[_face(lo,hi,2,0,spacing)] if label==0 else [
            _face(lo,hi,axis,side,spacing) for axis in range(3) for side in range(2)
            if not(label>=2 and axis==2)]
        cloud=np.concatenate(faces)
        if label in (0,1):
            contact=np.isclose(cloud[:,2],0. if label==0 else clearance)
            cloud=cloud[~(contact&_inside_legs_xy(cloud,meta['legs']))]
        clouds.append(cloud);labels.append(np.full(len(cloud),label,np.uint8))
    points,ids=np.unique(np.concatenate(clouds).astype('<f4'),axis=0,return_index=True)
    labels=np.concatenate(labels)[ids]
    empty=(points[:,2]>2e-6)&(points[:,2]<clearance-2e-6)&~_inside_legs_xy(points,meta['legs'],2e-6)
    assert not empty.any()
    meta.update(id=scene,name=NAMES[scene],clearance_m=clearance,point_count=len(points),spacing_m=spacing,
        kind='synthetic_complete_surfaces',labels_used_for_fitting_or_planning=False,
        upper_display_threshold_m=clearance/2,
        dimensions=dict(table_length=3.,table_width=2.,table_height=clearance+.10,clear_height=clearance,tabletop_thickness=.10,leg_width=.12),
        checks=dict(forbidden_points_under_table=int(empty.sum())))
    return points,labels,meta


def run():
    for scene in HEIGHTS:
        points,labels,meta=build(scene);folder=WEB/scene;folder.mkdir(exist_ok=True)
        points.tofile(folder/'points.bin');np.savez_compressed(folder/'scene.npz',points=points,labels=labels)
        write_pcd(folder/'scene.pcd',points,labels)
        (folder/'scene.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n')
        print(scene,len(points),flush=True)


if __name__=='__main__':run()

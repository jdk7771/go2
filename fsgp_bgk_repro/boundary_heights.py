"""Boundary height differences between neighboring fitted surface regions.

No scene labels or known platform heights are used in construction. Functions
are grouped by local geometric proximity and same-XY prediction agreement;
region outlines supply the boundary locations, never a center-height delta.
"""
import argparse
import csv
import json
from pathlib import Path
import time
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from .assessment_export import sha,finite_json
from .convex_surface_assessment import OUT,FIELDS,PROFILES,inside,area
from .hill_small_gaps import load_models
from .polygon_topology import PolygonTopology
from .terrain_assessment import basis

REPORT=OUT/'boundary-heights'
GROUP_GAP=.025
GROUP_HEIGHT=.02
MAX_BOUNDARY_GAP=.10
MIN_SHOWN_HEIGHT=.02
SAMPLE_STEP=.02
COLOR_MAX=.8


def predict(model,ids,xy):
    return model[ids,2]+np.sum(basis(np.asarray(xy)-model[ids,:2])*model[ids,3:9],axis=-1)


def compatible(model,a,b):
    # Agreement at three shared XY queries, including both centers. A smooth
    # slope may rise more than 2 cm; only the difference of predictions matters.
    q=np.stack((model[a,:2],(model[a,:2]+model[b,:2])/2,model[b,:2]),axis=1)
    return np.max(np.abs(predict(model,a[:,None],q)-predict(model,b[:,None],q)),axis=1)<=GROUP_HEIGHT


def ring_samples(ring,hole=False):
    ring=np.asarray(ring);signed=np.sum(ring[:-1,0]*ring[1:,1]-ring[:-1,1]*ring[1:,0])
    sign=(1 if signed>0 else -1)*(-1 if hole else 1)
    for a,b in zip(ring[:-1],ring[1:]):
        tangent=b-a;length=np.linalg.norm(tangent)
        if length<1e-9:continue
        normal=sign*np.array([tangent[1],-tangent[0]])/length
        count=max(1,int(np.ceil(length/SAMPLE_STEP)))
        for k in range(count):
            start=a+tangent*k/count;end=a+tangent*(k+1)/count
            yield (start+end)/2,normal,(end-start)/2


def extract(model,active,polygons,include_membership=False):
    tree=cKDTree(model[active,:2]);radius=max((np.linalg.norm(p-model[i,:2],axis=1).max() for i,p in zip(active,polygons)),default=0)
    candidates=tree.query_pairs(2*radius+GROUP_GAP,output_type='ndarray')
    candidates=candidates[compatible(model,active[candidates[:,0]],active[candidates[:,1]])]
    rows=[];cols=[];records=[]
    with PolygonTopology() as geo:
        geometries=[geo.polygon(p) for p in polygons]
        for a,b in candidates:
            if geo.distance(geometries[a],geometries[b])<=GROUP_GAP+1e-9:rows.append(a);cols.append(b)
        graph=coo_matrix((np.ones(len(rows)),(rows,cols)),shape=(len(active),len(active))).tocsr()
        count,labels=connected_components(graph,directed=False)
        sizes=np.bincount(labels,minlength=count);groups=[]
        for component in np.flatnonzero(sizes>=3):
            members=np.flatnonzero(labels==component);union=geo.union([polygons[i] for i in members])
            groups.append(dict(id=int(component),members=members,parts=list(geo.polygons(union))))
            geo.destroy(union)
        retained={g['id'] for g in groups};seen=set()
        for group in groups:
            component=group['id'];members=group['members'];own_tree=cKDTree(model[active[members],:2])
            for part in group['parts']:
                rings=[(part['exterior'],False)]+[(h,True) for h in part['holes'] if area(h[:-1])>=9e-4]
                for ring,hole in rings:
                    for midpoint,normal,half in ring_samples(ring,hole):
                        source=None
                        _,own=own_tree.query(midpoint,k=min(8,len(members)))
                        point=geo.point(midpoint)
                        for local in np.atleast_1d(own):
                            index=members[local]
                            if geo.distance(geometries[index],point)<=1e-7:source=int(index);break
                        if source is None:geo.destroy(point);continue
                        probe=midpoint+normal*.02;probe_point=geo.point(probe)
                        _,near=tree.query(probe,k=min(32,len(active)));options=[]
                        for target in np.atleast_1d(near):
                            if labels[target]==component or labels[target] not in retained:continue
                            if geo.distance(geometries[target],point)>MAX_BOUNDARY_GAP+1e-9:continue
                            outside=geo.nearest(geometries[target],probe_point)[0]
                            if np.dot(outside-midpoint,normal)<.002:continue
                            boundary=geo.nearest(geometries[target],point)[0]
                            q=(boundary+midpoint)/2;left,right=int(active[source]),int(active[target])
                            zi,zj=predict(model,left,q),predict(model,right,q)
                            options.append((np.linalg.norm(outside-probe),abs(zi-zj),int(target),q,zi,zj,boundary))
                        geo.destroy(point);geo.destroy(probe_point)
                        if not options:continue
                        nearest=min(o[0] for o in options)
                        # Among equally near supported regions, use the closest
                        # height layer, rather than all vertically stacked ones.
                        option=min((o for o in options if o[0]<=nearest+.005),key=lambda o:(o[1],o[0]))
                        _,difference,target,q,zi,zj,boundary=option
                        if difference<MIN_SHOWN_HEIGHT and not include_membership:continue
                        other=int(labels[target]);key=(*sorted((component,other)),*np.round(q/.02).astype(int))
                        if key in seen:continue
                        seen.add(key)
                        left,right=int(active[source]),int(active[target]);xy=np.stack((q-half,q+half))
                        za,zb=predict(model,left,xy),predict(model,right,xy)
                        records.append(dict(left=left,right=right,left_region=component,right_region=other,
                            xy=q,xy_ends=xy,left_z_m=float(zi),right_z_m=float(zj),delta_m=float(difference),
                            left_ends_z_m=za,right_ends_z_m=zb,
                            boundary_xy_gap_m=float(np.linalg.norm(boundary-midpoint)),length_m=float(np.linalg.norm(2*half))))
                        if include_membership:
                            records[-1].update(left_boundary_xy=midpoint,right_boundary_xy=boundary,outward_normal_xy=normal)
        group_info=[dict(id=g['id'],functions=len(g['members']),
            median_center_height_m=float(np.median(predict(model,active[g['members']],model[active[g['members']],:2])))) for g in groups]
    info=dict(total_components=int(count),retained_components=len(groups),regions=group_info,
        geometric_neighbor_pairs=len(rows))
    if include_membership:info.update(function_ids=active,function_regions=labels)
    return records,info


def build(scene,profile,label):
    started=time.perf_counter();folder,meta,model,active,polygons=load_models(profile,scene)
    hashes={name:sha(folder/name) for name in ('models.bin','polygons.bin','polygon-offsets.bin','points.bin','radii.bin','data.json')}
    records,group_info=extract(model,active,polygons)
    positions=[];values=[];line_records=[];line_kind=[]
    for index,r in enumerate(records):
        ends=r['xy_ends'];za=r['left_ends_z_m'];zb=r['right_ends_z_m'];difference=np.abs(za-zb)
        for z in (za,zb):
            positions.extend(np.c_[ends,z]);values.extend(difference);line_records.append(index);line_kind.append(0)
        # Sparse vertical rulers clarify the two heights without filling a wall.
        if index%8==0:
            positions.extend([[*r['xy'],r['left_z_m']],[*r['xy'],r['right_z_m']]])
            values.extend([r['delta_m']]*2);line_records.append(index);line_kind.append(1)
    positions=np.asarray(positions,dtype='<f4').reshape(-1,3);values=np.asarray(values,dtype='<f4')
    output=REPORT/profile/scene;output.mkdir(parents=True,exist_ok=True)
    positions.tofile(output/'lines.bin');values.tofile(output/'values.bin');np.asarray(line_records,'<u4').tofile(output/'line-records.bin')
    data=dict(scene=scene,name=meta['name'],profile=profile,label=label,raw_points=meta['raw_points'],
        valid_functions=meta['valid'],new_fits=0,records=records,grouping=group_info,
        boundary_samples=len(records),region_pairs=len({tuple(sorted((r['left_region'],r['right_region']))) for r in records}),
        mean_delta_m=float(np.mean([r['delta_m'] for r in records])) if records else None,
        maximum_delta_m=max((r['delta_m'] for r in records),default=None),
        line_segments=len(line_records),line_kind=line_kind,
        config=dict(sample_step_m=SAMPLE_STEP,group_max_xy_gap_m=GROUP_GAP,group_prediction_difference_m=GROUP_HEIGHT,
            minimum_region_functions=3,minimum_shown_difference_m=MIN_SHOWN_HEIGHT,
            maximum_boundary_xy_gap_m=MAX_BOUNDARY_GAP,color_max_m=COLOR_MAX,
            boundary='Midline between observed region boundary and nearest outward neighboring fitted domain',
            height='abs(f_i(q)-f_j(q)) at identical boundary XY q; not center height subtraction',
            scene_labels_used=False,no_surface_assembly=True),
        files=dict(lines='lines.bin',values='values.bin',line_records='line-records.bin'),
        original_files_sha256=hashes,elapsed_ms=(time.perf_counter()-started)*1000)
    (output/'data.json').write_text(json.dumps(finite_json(data),ensure_ascii=False)+'\n')
    with (output/'samples.csv').open('w') as stream:
        fields=['left','right','left_region','right_region','x','y','left_z_m','right_z_m','delta_m','boundary_xy_gap_m']
        writer=csv.DictWriter(stream,fieldnames=fields,extrasaction='ignore');writer.writeheader()
        for r in records:writer.writerow(dict(r,x=r['xy'][0],y=r['xy'][1]))
    assert all(sha(folder/name)==digest for name,digest in hashes.items())
    print(f"{scene}/{profile}: {len(records)} boundary samples, {data['region_pairs']} region pairs, max {data['maximum_delta_m']}",flush=True)
    return data


def run():
    parser=argparse.ArgumentParser();parser.add_argument('--scenes',nargs='+',default=['flat-corridor-platforms','steps'])
    parser.add_argument('--profiles',nargs='+',default=['qcircle-adaptive']);args=parser.parse_args()
    labels=dict(PROFILES);REPORT.mkdir(parents=True,exist_ok=True)
    records=[build(scene,profile,labels[profile]) for profile in args.profiles for scene in args.scenes]
    summary=[{key:r[key] for key in ('scene','profile','boundary_samples','region_pairs','mean_delta_m','maximum_delta_m','grouping','elapsed_ms')} for r in records]
    (REPORT/'summary.json').write_text(json.dumps(finite_json(summary),ensure_ascii=False,indent=2)+'\n')
    return summary


if __name__=='__main__':run()

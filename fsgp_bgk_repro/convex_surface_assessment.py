"""Local quadratic assessment: exact XY convex domains and six metrics.

Display tessellation has no observation-driven face deletion, inter-function
assembly or role in metric computation. Raw inputs and historical fits survive.
"""
import argparse
import csv
from dataclasses import asdict, replace
import json
import os
from pathlib import Path
import time

import numpy as np
from scipy.spatial import ConvexHull, cKDTree

from .adaptive_circular_surfaces import AdaptiveCircularSurface, AdaptiveCircularConfig
from .circular_surfaces import CircularSurface
from .terrain_assessment import AssessmentConfig, window_pools, fit_pools
from .assessment_export import OUT as HISTORY, finite_json, sha, stats
from .scene_fit_catalog import CURRENT_ASSESSMENT_SCENES as SCENES, NAMES, WEB

OUT=WEB/'convex-assessment'
METRICS=('clearance_cost','slope_deg','rmse_m','std_patch_m','normal_spread_deg','max_error_m')
FIELDS=['x','y','z','c0','c1','c2','c3','c4','c5','height','valid','point_count',
        'rmse_m','max_error_m','std_patch_m','slope_deg','normal_spread_deg','clearance_cost',
        'mean_normal_x','mean_normal_y','mean_normal_z','domain_area_m2']
PROFILES=(('qcircle-adaptive','自适应圆形 · 至少 15 点'),('qcircle-r07','圆形 R=7 cm · 至少 15 点'),
          ('q15-n15','15×15 cm · 至少 15 点'),('q20-n30','20×20 cm · 至少 30 点'))
OFFSET_PROFILE='qcircle-adaptive-no-offset'
EXPERIMENTAL_PROFILES=((OFFSET_PROFILE,'自适应圆形 · 无中心偏移限制'),)
PROFILE_SCENES={**{p:SCENES for p,_ in PROFILES},OFFSET_PROFILE:('forest',)}
SAMPLE_STEP=.01


def area(polygon):
    p=np.asarray(polygon)
    if len(p)<3:return 0.
    return float(abs(np.sum(p[:,0]*np.roll(p[:,1],-1)-p[:,1]*np.roll(p[:,0],-1)))/2)


def inside(xy,polygon,tolerance=1e-10):
    """ConvexHull returns CCW vertices; cross-product tests include boundary."""
    xy=np.asarray(xy).reshape(-1,2);p=np.asarray(polygon);edge=np.roll(p,-1,axis=0)-p
    delta=xy[:,None,:]-p
    return np.all(edge[None,:,0]*delta[:,:,1]-edge[None,:,1]*delta[:,:,0]>=-tolerance,axis=1)


def clip_rectangle(polygon,low,high):
    p=np.asarray(polygon,float)
    for axis,bound,sign in ((0,low[0],-1),(0,high[0],1),(1,low[1],-1),(1,high[1],1)):
        if len(p)<3:return np.empty((0,2))
        d=sign*(p[:,axis]-bound);prev=np.roll(p,1,axis=0);pd=np.roll(d,1)
        keep=d<=1e-12;cross=keep!=(pd<=1e-12)
        t=np.divide(pd,pd-d,out=np.zeros_like(pd),where=np.abs(pd-d)>1e-20)
        p=np.stack((prev+(p-prev)*t[:,None],p),axis=1).reshape(-1,2)[np.stack((cross,keep),axis=1).ravel()]
    return p


def domain_samples(polygon,step=SAMPLE_STEP):
    """Independent 1 cm XY grid, unrelated to display vertices or triangles."""
    lo,hi=polygon.min(0),polygon.max(0)
    axes=[(np.arange(np.floor(a/step),np.ceil(b/step))+.5)*step for a,b in zip(lo,hi)]
    x,y=np.meshgrid(*axes);q=np.c_[x.ravel(),y.ravel()]
    q=q[inside(q,polygon)]
    return q if len(q) else polygon.mean(0)[None,:]


def derivative_metrics(coefficient,polygon,covariance):
    q=domain_samples(polygon);u,v=q.T;c=coefficient
    normals=np.c_[-(c[1]+2*c[3]*u+c[4]*v),-(c[2]+c[4]*u+2*c[5]*v),np.ones(len(q))]
    normals/=np.linalg.norm(normals,axis=1)[:,None]
    average=normals.mean(0);average/=np.linalg.norm(average)
    slope=float(np.rad2deg(np.arccos(np.clip(average[2],-1,1))))
    angles=np.rad2deg(np.arccos(np.clip(normals@average,-1,1)))
    smooth=float(np.sqrt(np.mean(angles**2)))
    if smooth<1e-6:smooth=0.
    allq=np.r_[q,polygon];u,v=allq.T;phi=np.c_[np.ones(len(allq)),u,v,u*u,u*v,v*v]
    sigma=float(np.sqrt(np.maximum(0,np.einsum('ni,ij,nj->n',phi,covariance,phi))).max())
    return dict(slope_deg=slope,normal_spread_deg=smooth,std_patch_m=sigma,mean_normal=average,
        metric_sample_count=len(q),uncertainty_sample_count=len(allq))


def occupancy(points,tree,seed,coefficient,polygon,cell=.05,low=.10,high=.60):
    """Exact occupied-column XY areas clipped to this polygon; raw evidence only."""
    radius=float(np.linalg.norm(polygon,axis=1).max())
    ids=tree.query_ball_point(seed[:2],radius+1e-8)
    if not ids:return 0.,0,0
    raw=points[ids];uv=raw[:,:2]-seed[:2];raw=raw[inside(uv,polygon)]
    uv=raw[:,:2]-seed[:2];u,v=uv.T;c=coefficient
    predicted=seed[2]+c[0]+c[1]*u+c[2]*v+c[3]*u*u+c[4]*u*v+c[5]*v*v
    # Saved XYZ is float32: 0.60 m becomes 0.6000000238 m. Include such
    # quantization at the two closed interval boundaries (not a height margin).
    tolerance=max(1e-9,2*float(np.spacing(np.float32(max(abs(low),abs(high))))))
    dz=raw[:,2]-predicted;hits=raw[(dz>=low-tolerance)&(dz<=high+tolerance)]
    if not len(hits):return 0.,0,0
    columns=np.unique(np.floor(hits[:,:2]/cell).astype(np.int64),axis=0)
    occupied_area=0.
    for column in columns:
        lower=column*cell-seed[:2]
        occupied_area+=area(clip_rectangle(polygon,lower,lower+cell))
    return float(np.clip(occupied_area/area(polygon),0,1)),len(hits),len(columns)


def display_patch(seed,coefficient,polygon,edge_step=.04,rings=3):
    """Cover the entire convex domain. Every face belongs to this one function."""
    boundary=[]
    for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
        n=max(1,int(np.ceil(np.linalg.norm(b-a)/edge_step)))
        boundary.extend(a+(b-a)*t for t in np.arange(n)/n)
    boundary=np.asarray(boundary);center=polygon.mean(0);n=len(boundary)
    uv=np.concatenate([center[None,:]]+[center+(boundary-center)*r/rings for r in range(1,rings+1)])
    u,v=uv.T;c=coefficient
    z=seed[2]+c[0]+c[1]*u+c[2]*v+c[3]*u*u+c[4]*u*v+c[5]*v*v
    positions=np.c_[uv+seed[:2],z]
    a=np.arange(n);b=np.roll(a,-1)
    faces=[np.c_[np.zeros(n,int),1+a,1+b]]
    for r in range(1,rings):
        inner=1+(r-1)*n;outer=1+r*n
        faces.extend((np.c_[inner+a,outer+a,outer+b],np.c_[inner+a,outer+b,inner+b]))
    return positions,np.concatenate(faces)


def raw_points_path(name):
    historical=HISTORY/name/'points.bin'
    return historical if historical.exists() else WEB/name/'points.bin'


def fresh_fixed_fit(points,profile):
    """Run the existing fit rules for a new scene, without historical metrics."""
    if profile=='qcircle-r07':
        surface=CircularSurface(points)
        return surface.seeds,surface.pools,surface.fits,asdict(surface.config),np.full(len(surface.seeds),surface.config.radius_m),dict(
            source='computed_circle',actual_main_solves=surface.actual_solve_count,
            fit_pipeline_ms=surface.times['fit_pipeline_ms'],stages_ms=surface.times,extra_centers=surface.adaptive)
    if profile not in ('q15-n15','q20-n30'):raise ValueError(profile)
    cfg=replace(AssessmentConfig(),patch_size_m=.15,minimum_points=15) if profile=='q15-n15' else AssessmentConfig()
    start=time.perf_counter()
    _,ids=np.unique(np.floor(points/cfg.seed_voxel_m).astype(np.int64),axis=0,return_index=True)
    seeds=points[ids];selected=time.perf_counter()
    pools=window_pools(points,seeds,cfg);queried=time.perf_counter()
    fits=fit_pools(points,seeds,pools,cfg);solved=time.perf_counter()
    timings=dict(select_centers_ms=(selected-start)*1000,window_query_ms=(queried-selected)*1000,
        initial_fit_and_covariance_ms=(solved-queried)*1000,fit_pipeline_ms=(solved-start)*1000)
    return seeds,pools,fits,asdict(cfg),np.full(len(seeds),np.nan),dict(source='computed_square',
        actual_main_solves=int(np.isfinite(fits['rmse']).sum()),fit_pipeline_ms=timings['fit_pipeline_ms'],stages_ms=timings)


def load_fit(name,profile):
    original=raw_points_path(name)
    points=np.fromfile(original,'<f4').reshape(-1,3).astype(float)
    if profile in ('qcircle-adaptive',OFFSET_PROFILE):
        if name not in PROFILE_SCENES[profile]:raise ValueError(f'{profile} is unavailable for {name}')
        cfg=AdaptiveCircularConfig(center_offset_check=profile!=OFFSET_PROFILE)
        surface=AdaptiveCircularSurface(points,cfg)
        return points,surface.seeds,surface.pools,surface.fits,asdict(surface.config),surface.model_radii_m,dict(
            source='computed_adaptive',actual_main_solves=surface.actual_solve_count,
            fit_pipeline_ms=surface.times['fit_pipeline_ms'],stages_ms=surface.times,
            density_adaptation=surface.density,extra_centers=surface.adaptive)
    folder=HISTORY/name if profile=='q20-n30' else HISTORY/'profiles'/profile/name
    if not (folder/'data.json').exists():return (points,*fresh_fixed_fit(points,profile))
    meta=json.loads((folder/'data.json').read_text());fields=meta['fields']
    model=np.fromfile(folder/'models.bin','<f8').reshape(-1,len(fields));column={key:model[:,i] for i,key in enumerate(fields)}
    offsets=np.fromfile(folder/'neighbor-offsets.bin','<i4');neighbors=np.fromfile(folder/'neighbors.bin','<i4')
    pools=[neighbors[a:b] for a,b in zip(offsets[:-1],offsets[1:])]
    fits=dict(coefficient=model[:,3:9],height=column['height'],counts=column['point_count'].astype(int),
        rmse=column['rmse_m'],max_error=column['max_error_m'],
        covariance=np.fromfile(folder/'covariance.bin','<f8').reshape(-1,6,6))
    return points,model[:,:3],pools,fits,meta['config'],np.full(len(model),meta['config'].get('radius_m',np.nan)),dict(
        source='unchanged_saved_fit',actual_main_solves=meta.get('actual_solve_count',meta['solved']),fit_pipeline_ms=0.,
        reused_models_sha256=sha(folder/'models.bin'),historical_timings=meta['timings'])


def export_scene(name,profile,label):
    started=time.perf_counter()
    points,seeds,pools,fits,config,radii,fit_info=load_fit(name,profile)
    loaded=time.perf_counter();tree=cKDTree(points[:,:2]);active=np.flatnonzero(np.isfinite(fits['height']))
    n=len(seeds);derived={key:np.full(n,np.nan) for key in ('slope_deg','normal_spread_deg','std_patch_m','clearance_cost','domain_area_m2')}
    normals=np.full((n,3),np.nan);samples=np.zeros(n,np.int32);raw_hits=np.zeros(n,np.int32)
    polygons=[];poly_offsets=[0];polygon_by_model={};metric_ms=0.;geometry_ms=0.
    positions=[];faces=[];owners=[];vertex_offset=0
    for model in active:
        uv=points[pools[model],:2]-seeds[model,:2]
        hull=ConvexHull(uv);polygon=uv[hull.vertices];polygon_by_model[model]=polygon
    domains_done=time.perf_counter()
    for model in range(n):
        if model not in polygon_by_model:
            poly_offsets.append(poly_offsets[-1]);continue
        polygon=polygon_by_model[model];polygons.extend(polygon);poly_offsets.append(poly_offsets[-1]+len(polygon))
        last=time.perf_counter();m=derivative_metrics(fits['coefficient'][model],polygon,fits['covariance'][model])
        cost,hits,_=occupancy(points,tree,seeds[model],fits['coefficient'][model],polygon)
        for key in ('slope_deg','normal_spread_deg','std_patch_m'):derived[key][model]=m[key]
        derived['clearance_cost'][model]=cost;derived['domain_area_m2'][model]=area(polygon)
        normals[model]=m['mean_normal'];samples[model]=m['metric_sample_count'];raw_hits[model]=hits
        metric_ms+=(time.perf_counter()-last)*1000;last=time.perf_counter()
        v,f=display_patch(seeds[model],fits['coefficient'][model],polygon)
        positions.append(v);faces.append(f+vertex_offset);owners.append(np.full(len(v),model,dtype='<u4'));vertex_offset+=len(v)
        geometry_ms+=(time.perf_counter()-last)*1000
    vertices=np.concatenate(positions).astype('<f4') if positions else np.empty((0,3),'<f4')
    triangles=np.concatenate(faces).astype('<u4') if faces else np.empty((0,3),'<u4')
    owner=np.concatenate(owners) if owners else np.empty(0,'<u4')
    model=np.c_[seeds,fits['coefficient'],fits['height'],np.isfinite(fits['height']),fits['counts'],fits['rmse'],fits['max_error'],
        derived['std_patch_m'],derived['slope_deg'],derived['normal_spread_deg'],derived['clearance_cost'],normals,derived['domain_area_m2']].astype('<f8')
    assert model.shape==(n,len(FIELDS))
    folder=OUT/profile/name;folder.mkdir(parents=True,exist_ok=True)
    compute_ms=(time.perf_counter()-started)*1000
    for filename,array in [('models.bin',model),('patches.bin',vertices),('patch-faces.bin',triangles),('patch-owners.bin',owner),
        ('radii.bin',radii.astype('<f4')),('polygons.bin',np.asarray(polygons,dtype='<f8').reshape(-1,2)),
        ('polygon-offsets.bin',np.asarray(poly_offsets,dtype='<i4')),('metric-samples.bin',samples),('raw-obstacle-hits.bin',raw_hits),
        ('covariance.bin',fits['covariance'].astype('<f8'))]:array.tofile(folder/filename)
    offsets=np.r_[0,np.cumsum([len(p) for p in pools])].astype('<i4');offsets.tofile(folder/'neighbor-offsets.bin')
    (np.concatenate(pools).astype('<i4') if pools else np.empty(0,'<i4')).tofile(folder/'neighbors.bin')
    raw=folder/'points.bin'
    if not raw.exists():raw.symlink_to(os.path.relpath(raw_points_path(name),folder))
    with (folder/'per-model.csv').open('w') as stream:
        writer=csv.writer(stream);writer.writerow(['model',*FIELDS]);writer.writerows([i,*[float(x) if np.isfinite(x) else '' for x in row]] for i,row in enumerate(model))
    metadata=dict(id=name,name=NAMES[name],profile_id=profile,profile_label=label,raw_points=len(points),centers=n,
        solved=int(np.isfinite(fits['rmse']).sum()),actual_solve_count=fit_info['actual_main_solves'],valid=len(active),
        displayed_functions=int(len(np.unique(owner[triangles]))) if len(triangles) else 0,patch_triangles=len(triangles),
        fields=FIELDS,metrics=METRICS,config=config,bounds=[points.min(0).tolist(),points.max(0).tolist()],
        source=dict(kind='identical_saved_xyz',sha256=sha(raw_points_path(name))),fit=fit_info,
        radius_stats_m=dict(minimum=float(np.nanmin(radii[active])),median=float(np.nanmedian(radii[active])),maximum=float(np.nanmax(radii[active]))) if len(active) and np.isfinite(radii[active]).any() else None,
        geometry=dict(domain='exact convex hull of this function fitting-point XY',hull_tolerance_m=0.,observation_face_deletion=False,
            cross_function_assembly=False,display_rings=3,maximum_boundary_segment_m=.04,
            note='Every valid convex domain is entirely rendered; display triangulation is not used by any metric. Hulls do not detect internal data holes.'),
        metric_definition=dict(sample_step_m=SAMPLE_STEP,slope='Angle between +Z and normalized mean of analytic unit normals at independent 1 cm XY grid centers inside the hull.',
            smoothness='RMS angle in degrees of the same analytic unit normals around their normalized mean; larger means less smooth. No RMSE term.',
            uncertainty='Maximum sampled prediction-mean standard deviation on independent interior samples plus exact hull vertices; not calibrated test error.',
            occupancy='Raw returns only, 10..60 cm above the analytic function within its exact hull; 5 cm global XY columns, exact occupied-column/hull intersection-area fraction.',
            training_errors='RMSE and maximum absolute vertical residual on the original selected fitting points, not held-out accuracy.'),
        field_stats={key:stats(model[active,FIELDS.index(key)]) for key in METRICS},
        timings=dict(input_and_fit_ms=(loaded-started)*1000,convex_domains_ms=(domains_done-loaded)*1000,
            six_metrics_ms=metric_ms,display_geometry_ms=geometry_ms,total_compute_ms=compute_ms,
            note='One CPU run; fresh fitting or saved-model loading as specified by fit.source. Export IO and browser excluded; not comparable to fit-only timing.'))
    (folder/'data.json').write_text(json.dumps(finite_json(metadata),ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(scene=name,profile=profile,valid=len(active),displayed=metadata['displayed_functions'],triangles=len(triangles),seconds=round(compute_ms/1000,2))),flush=True)
    return metadata


def run(names=SCENES,profile_ids=None):
    OUT.mkdir(exist_ok=True);profiles=list(PROFILES) if profile_ids is None else [p for p in PROFILES+EXPERIMENTAL_PROFILES if p[0] in profile_ids]
    for profile,label in profiles:
        for name in names:
            if name in PROFILE_SCENES[profile]:export_scene(name,profile,label)
    catalog=[]
    for profile,label in PROFILES+EXPERIMENTAL_PROFILES:
        scope=PROFILE_SCENES[profile]
        cases=[name for name in scope if (OUT/profile/name/'data.json').exists()]
        if len(cases)==len(scope):catalog.append(dict(id=profile,label=label,path=profile+'/',scene_ids=list(scope)))
    (OUT/'profiles.json').write_text(json.dumps(dict(default_profile='qcircle-adaptive',profiles=catalog),ensure_ascii=False,indent=2)+'\n')
    (OUT/'catalog.json').write_text(json.dumps(dict(scenes=[dict(id=n,name=NAMES[n]) for n in SCENES],default_scene='hill',metrics=METRICS,fields=FIELDS),ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--scenes',nargs='+',choices=SCENES,default=SCENES)
    parser.add_argument('--profiles',nargs='+',choices=[p[0] for p in PROFILES+EXPERIMENTAL_PROFILES]);args=parser.parse_args();run(args.scenes,args.profiles)

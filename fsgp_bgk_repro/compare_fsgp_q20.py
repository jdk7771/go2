"""Paired geometry, native-risk ranking and temporal tests; no official scores.

Uses existing scenes and unchanged method implementations. Neither ground truth
nor surface labels enter a primary method. The explicitly named oracle control
is excluded from primary paired comparisons.
"""
import argparse
from dataclasses import asdict, replace
import gc
import hashlib
import json
from pathlib import Path
import time
import csv

import numpy as np
from scipy.ndimage import maximum_filter, convolve
from scipy.spatial import cKDTree
from scipy.stats import rankdata

from .core import (Config, TraversabilityMapper, SparseTerrainGP, extract_features,
                   select_inducing, interpolate_features, smooth_known)
from .terrain_assessment import (AssessmentConfig, window_pools, fit_pools, choose_support,
    variance, patches, jump_metrics, clearance_metrics, local_metrics)
from .assessment_export import assessment_source, finite_json, WEB, ROOT, NAMES
from .mixed_steps_scene import BREAKS, HEIGHTS
from .visualize_regression import local_quadratic

OUT=WEB/'fsgp-q20'
SCENES=['flat','ramp','flat_ramp','steps','steps-single','hill','corridor-040','corridor-060','corridor-080']
CONDITIONS=dict(dense_low=dict(keep=.85,z_noise_m=.003,xy_noise_m=.001),
                dense_noisy=dict(keep=.85,z_noise_m=.015,xy_noise_m=.001),
                sparse_noisy=dict(keep=.45,z_noise_m=.015,xy_noise_m=.001))
METHODS=['Q20','FSGP-125','FSGP-500']
SENSOR_SEEDS=[1101,1102,1103]
CFG=AssessmentConfig()


def write_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(finite_json(value),ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n')


def digest(array):return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()


def reference(scene,xy):
    """Analytic surface and 80x50 cm footprint constraints, not native costs."""
    x,y=xy.T;n=len(x);z=np.zeros(n);gradient=np.zeros((n,2))
    height_mask=np.ones(n,bool);risk_mask=np.ones(n,bool);hazard=np.zeros(n,bool)
    if scene=='ramp':z=.25*x+.1*y;gradient[:]=[.25,.1]
    elif scene=='flat_ramp':
        z=.3*np.clip(x+1,0,2);gradient[(x>-1)&(x<1),0]=.3
        height_mask &= np.minimum(np.abs(x+1),np.abs(x-1))>.04
    elif scene=='steps-single':
        bump=.15*np.exp(-((x+1.2)**2+(y-.5)**2)/.4)
        z=.45*(x>.15)+bump;gradient=np.column_stack((-5*(x+1.2)*bump,-5*(y-.5)*bump))
        height_mask &= np.abs(x-.15)>.04;hazard=np.abs(x-.15)<=.4
    elif scene=='steps':
        inside=np.abs(y)<1.5
        bands=np.searchsorted(BREAKS,x,side='right');z=np.where(inside,HEIGHTS[bands],0)
        distance=np.min(np.abs(x[:,None]-BREAKS),axis=1)
        height_mask &= ~((inside&(distance<=.04))|((np.abs(np.abs(y)-1.5)<=.04)&(x>-3)&(x<2.3)))
        for i,edge in enumerate(BREAKS):
            if abs(HEIGHTS[i+1]-HEIGHTS[i])>=.12-1e-8:
                hazard |= (np.abs(x-edge)<=.4)&(np.abs(y)<=1.75)
        # Body crossing a side wall with >=12 cm height is also a hazard.
        for band in range(1,len(HEIGHTS)-1):
            if HEIGHTS[band]>=.12:
                hazard |= (x>=BREAKS[band-1]-.4)&(x<=BREAKS[band]+.4)&(np.abs(np.abs(y)-1.5)<=.25)
    elif scene.startswith('corridor-'):
        h=int(scene[-3:])/100
        risk_mask &= np.abs(y)<=1.75  # Entire 50 cm body width has floor support.
        height_mask &= (np.abs(y)<=2)&~((np.abs(x)<=2.04)&(np.abs(np.abs(y)-1.06)<=.10))
        walls=(np.abs(x)<=2.4)&(np.abs(y)>=.75)&(np.abs(y)<=1.37)
        roof=(np.abs(x)<=2.4)&(np.abs(y)<=1.37)&(.1<=h<=.6)
        hazard=walls|roof
    elif scene not in ('flat','hill'):raise ValueError(scene)
    if scene=='hill':height_mask[:]=True;risk_mask[:]=False;gradient[:]=np.nan
    hazard |= np.arctan(np.linalg.norm(gradient,axis=1))>=np.pi/4
    risk_mask &= (np.abs(x)<=2.0+1e-8)&(np.abs(y)<=2.15+1e-8)
    return dict(height=z,gradient=gradient,height_mask=height_mask,risk_mask=risk_mask,hazard=hazard)


def source(scene):
    points,metadata=assessment_source(scene)
    keep=np.max(np.abs(points[:,:2]),axis=1)<=2.95+1e-7
    return points[keep],metadata


def sensor(scene,clean,condition,seed,xy):
    """Disjoint raw indices for hill; independent query XY for analytic tests."""
    rng=np.random.default_rng(seed);source_ids=np.arange(len(clean));heldout_ids=np.empty(0,int)
    query=xy.copy();truth=reference(scene,query)
    if scene=='hill':
        split=rng.permutation(len(clean));heldout_ids=split[:len(clean)//5]
        source_ids=split[len(clean)//5:]
        _,nearest=cKDTree(clean[heldout_ids,:2]).query(xy)
        heldout_ids=heldout_ids[nearest]
        assert len(np.unique(heldout_ids))==len(query)
        query=clean[heldout_ids,:2].astype(float);truth=reference(scene,query)
        truth['height']=clean[heldout_ids,2].astype(float)
        assert not np.intersect1d(source_ids,heldout_ids).size
    p=CONDITIONS[condition]
    selected=rng.choice(source_ids,int(len(source_ids)*p['keep']),replace=False)
    points=clean[selected].astype(float).copy()
    # Oracle is a separate labelled segmentation upper-bound control only.
    oracle_ground=np.isclose(clean[selected,2],0,atol=1e-8) if scene.startswith('corridor-') else np.zeros(len(selected),bool)
    points[:,:2]+=rng.normal(0,p['xy_noise_m'],(len(points),2))
    points[:,2]+=rng.normal(0,p['z_noise_m'],len(points))
    inside=np.max(np.abs(points[:,:2]),axis=1)<=2.95
    points=points[inside].astype('<f4');selected=selected[inside];oracle_ground=oracle_ground[inside]
    return points,query,truth,selected,heldout_ids,oracle_ground


def gp_predict(points,xy,config):
    begin=time.perf_counter();points=np.asarray(points,float)
    features=extract_features(points,config.neighbors)
    inducing=select_inducing(features,config,np.random.default_rng(config.seed),True)
    test,distance=interpolate_features(features,xy,config.interpolation_neighbors)
    gp=SparseTerrainGP(config).fit(inducing[:,[0,1,3,4]],inducing[:,2])
    height,var,gradient=gp.predict(test)
    observed=(distance<=config.support_radius)&(var<=config.max_variance)
    geometry_ms=(time.perf_counter()-begin)*1000
    preliminary=np.sum(test[:,2:4]*np.asarray(config.weights[:2]),axis=1)+config.weights[2]*np.linalg.norm(gradient,axis=1)
    cost=smooth_known(preliminary.reshape(25,25),observed.reshape(25,25),config.smoothing_sigma).ravel()
    result=dict(height=height,variance=var,gradient=gradient,known=observed,cost=cost,
                geometry_ms=geometry_ms,native_ms=(time.perf_counter()-begin)*1000,
                inducing=inducing,preliminary=preliminary,actual_models=len(inducing))
    return result


def q_predict(points,xy):
    begin=time.perf_counter();points=np.asarray(points,float)
    _,ids=np.unique(np.floor(points/CFG.seed_voxel_m).astype(np.int64),axis=0,return_index=True)
    seeds=points[ids];pools=window_pools(points,seeds,CFG);fits=fit_pools(points,seeds,pools,CFG)
    # Lowest observed nearby height associates the ground branch; no truth Z.
    _,raw=cKDTree(points[:,:2]).query(xy,k=min(16,len(points)))
    expected=np.min(points[raw,2],axis=1)
    support,height=choose_support(seeds,fits,xy,expected)
    known=support>=0;safe=np.maximum(support,0);d=xy-seeds[safe,:2];c=fits['coefficient'][safe]
    gradient=np.column_stack((c[:,1]+2*c[:,3]*d[:,0]+c[:,4]*d[:,1],c[:,2]+c[:,4]*d[:,0]+2*c[:,5]*d[:,1]))
    var=variance(seeds,fits['covariance'],safe,xy);var[~known]=np.nan;gradient[~known]=np.nan
    geometry_ms=(time.perf_counter()-begin)*1000
    positions,faces,owners,std=patches(points,seeds,pools,fits,CFG)
    jumps=jump_metrics(points,seeds,pools,fits,CFG)
    clear=clearance_metrics(points,seeds,fits,positions,faces,CFG)
    metrics=local_metrics(seeds,fits,std,jumps,clear,CFG)
    cost=metrics['foot_cost'][safe].copy();cost[~known]=np.nan
    result=dict(height=height,variance=var,gradient=gradient,known=known,cost=cost,
        geometry_ms=geometry_ms,native_ms=(time.perf_counter()-begin)*1000,
        actual_models=int(np.isfinite(fits['height']).sum()),centers=len(seeds),
        actual_primary_solves=int(np.isfinite(fits['rmse']).sum()),side_solves=jumps['solve_counts']['actual_side_solves'])
    evidence=dict(q_seeds=seeds,q_coefficients=fits['coefficient'],q_covariance=fits['covariance'],
        q_height=fits['height'],q_support=support,q_source_ids=ids,
        q_neighbor_offsets=np.r_[0,np.cumsum([len(p) for p in pools])],
        q_neighbor_ids=np.concatenate(pools),q_rmse=fits['rmse'])
    return result,evidence


def body_rank(cost):
    """Same discrete 80x50 cm footprint max for both native cost scales."""
    a=cost.reshape(25,25);known=np.isfinite(a)
    count=convolve(known.astype(float),np.ones((3,5)),mode='constant',cval=0)
    maximum=maximum_filter(np.where(known,a,-np.inf),size=(3,5),mode='constant',cval=-np.inf)
    valid=count>=12 # 80% of 15 grid centres inside the rectangle.
    return np.where(valid,maximum,np.nan).ravel()


def auc_ap(labels,scores):
    labels=np.asarray(labels,bool);scores=np.asarray(scores,float)
    pos=int(labels.sum());neg=len(labels)-pos
    if not pos or not neg:return dict(auc=None,ap=None,n=len(labels),positive=pos)
    auc=(rankdata(scores)[labels].sum()-pos*(pos+1)/2)/(pos*neg)
    order=np.argsort(-scores,kind='stable');s=scores[order];y=labels[order]
    ends=np.r_[np.flatnonzero(np.diff(s)!=0),len(s)-1]
    tp=np.cumsum(y)[ends];precision=tp/(ends+1);recall=tp/pos
    ap=np.sum(np.diff(np.r_[0,recall])*precision)
    return dict(auc=float(auc),ap=float(ap),n=len(labels),positive=pos)


def metric(result,truth,mask):
    mask=mask&result['known']&np.isfinite(result['height']);n=int(mask.sum())
    error=result['height'][mask]-truth['height'][mask]
    normal_mask=mask&np.isfinite(truth['gradient']).all(axis=1)&np.isfinite(result['gradient']).all(axis=1)
    p=np.column_stack((-result['gradient'][normal_mask],np.ones(normal_mask.sum())))
    t=np.column_stack((-truth['gradient'][normal_mask],np.ones(normal_mask.sum())))
    p/=np.linalg.norm(p,axis=1)[:,None];t/=np.linalg.norm(t,axis=1)[:,None]
    angle=np.rad2deg(np.arccos(np.clip(np.sum(p*t,axis=1),-1,1)))
    return dict(n=n,squared_error_sum=float(np.sum(error**2)),absolute_error_sum=float(np.sum(np.abs(error))),
        rmse_m=float(np.sqrt(np.mean(error**2))) if n else None,
        normal_n=len(angle),normal_error_sum_deg=float(angle.sum()),
        normal_mean_deg=float(angle.mean()) if len(angle) else None,
        variance_n=int(np.isfinite(result['variance'][mask]).sum()),
        within_2sigma=int(np.sum(np.abs(error)<=2*np.sqrt(np.maximum(0,result['variance'][mask])))))


def run_case(scene,condition,seed,clean,output):
    xy,_,_=TraversabilityMapper().grid((0,0))
    points,xy,truth,source_ids,test_ids,oracle=sensor(scene,clean,condition,seed,xy)
    methods={};q_evidence={}
    # Alternate order deterministically, without simultaneously running methods.
    sequence=['Q20','FSGP-125','FSGP-500']
    if seed%2:sequence=sequence[::-1]
    for method in sequence:
        if method=='Q20':methods[method],q_evidence=q_predict(points,xy)
        else:methods[method]=gp_predict(points,xy,Config(inducing_points=int(method.split('-')[1]),seed=seed))
    if oracle.any():
        methods['FSGP-125-oracle']=gp_predict(points[oracle],xy,Config(inducing_points=125,seed=seed))
    # Distinguish a representation advantage from using more original points.
    # This is the historical K=20 control, not the strict 20 cm Q20 algorithm.
    for count in (125,500):
        parent=methods[f'FSGP-{count}'];retained=parent['inducing'][:,:3]
        begin=time.perf_counter();height,known=local_quadratic(retained,xy,20)
        elapsed=(time.perf_counter()-begin)*1000
        methods[f'K20-on-M{count}']=dict(height=height,known=known&parent['known'],
            gradient=np.full((len(xy),2),np.nan),variance=np.full(len(xy),np.nan),cost=np.full(len(xy),np.nan),
            geometry_ms=elapsed,native_ms=elapsed,actual_models=int(known.sum()))
    common=truth['height_mask'].copy()
    for name in METHODS:common &= methods[name]['known']
    for result in methods.values():result['body_rank']=body_rank(result['cost'])
    common_risk=truth['risk_mask'].copy()
    for name in METHODS:common_risk &= np.isfinite(methods[name]['body_rank'])
    rows=[]
    for name,result in methods.items():
        own=truth['height_mask']&result['known']
        row=dict(scene=scene,condition=condition,seed=seed,method=name,scan_points=len(points),
            scan_sha256=digest(points),query_count=len(xy),height_eligible=int(truth['height_mask'].sum()),
            known_height=int(own.sum()),geometry_ms=result['geometry_ms'],native_ms=result['native_ms'],
            actual_models=result['actual_models'],common=metric(result,truth,common),own=metric(result,truth,own),
            risk_common=auc_ap(truth['hazard'][common_risk&np.isfinite(result['body_rank'])],result['body_rank'][common_risk&np.isfinite(result['body_rank'])]),
            risk_eligible=int(truth['risk_mask'].sum()),risk_known=int(np.sum(truth['risk_mask']&np.isfinite(result['body_rank']))),
            oracle_control=name.endswith('-oracle'),auxiliary_control=name not in METHODS,
            timing_scope='post-sampling height only' if name.startswith('K20-on-') else 'raw XYZ to common outputs')
        rows.append(row)
    case_id=f'{scene}_{condition}_{seed}'
    arrays=dict(points=points,query_xy=xy,truth_height=truth['height'],truth_gradient=truth['gradient'],
        height_mask=truth['height_mask'],risk_mask=truth['risk_mask'],hazard=truth['hazard'],
        common=common,common_risk=common_risk,source_ids=source_ids,test_ids=test_ids,oracle_ground=oracle,**q_evidence)
    serial={}
    for name,result in methods.items():
        serial[name]={k:v for k,v in result.items() if k not in ('inducing','preliminary')}
        for key in ('height','variance','gradient','known','cost','body_rank'):
            arrays[name+'__'+key]=result[key]
        if 'inducing' in result:arrays[name+'__inducing']=result['inducing']
    folder=output/'cases';folder.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(folder/(case_id+'.npz'),**arrays)
    display=np.linspace(0,len(points)-1,min(4000,len(points))).astype(int)
    write_json(folder/(case_id+'.json'),dict(id=case_id,scene=scene,name=NAMES[scene],condition=condition,seed=seed,
        points_shown=points[display],raw_points=len(points),query_xy=xy,truth=truth,methods=serial,
        rows=rows,common=common,common_risk=common_risk,reference_kind='withheld raw height, no labelled hazard/normal' if scene=='hill' else 'analytic geometry'))
    return dict(id=case_id,scene=scene,condition=condition,seed=seed,rows=rows,points=len(points),
        scan_sha256=digest(points),height_eligible=int(truth['height_mask'].sum()),common_n=int(common.sum())),arrays


def publish_records(output,cases):
    write_json(output/'results.json',dict(cases=cases,methods=METHODS,conditions=CONDITIONS,scenes=SCENES))
    rows=[]
    for case in cases:
        for row in case['rows']:
            flat={k:v for k,v in row.items() if not isinstance(v,dict)}
            for key in ('own','common','risk_common'):
                flat.update({key+'_'+k:v for k,v in row[key].items()})
            rows.append(flat)
    with (output/'per-run.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def temporal(output,frames=8):
    clean,_=source('flat_ramp');xy,cell_ids,shape=TraversabilityMapper().grid((0,0))
    cfg=Config(inducing_points=125);mapper=TraversabilityMapper(cfg)
    records=[];last={}
    for frame in range(frames):
        points,query,truth,*_=sensor('flat_ramp',clean,'dense_noisy',2200+frame,xy)
        if 2<=frame<=5:points=points[np.abs(points[:,0])>=.7]
        native=mapper.assess(points,timestamp=frame*.1);fused=mapper.fuse(native)
        q,_=q_predict(points,query)
        values={'FSGP':native['cost'].ravel(),'FSGP-BGK':fused['cost'].ravel(),'Q20':q['cost']}
        for name,cost in values.items():
            known=np.isfinite(cost);both=known&np.isfinite(last[name]) if name in last else np.zeros(len(cost),bool)
            records.append(dict(frame=frame,method=name,hole=2<=frame<=5,known=int(known.sum()),total=len(cost),
                coverage=float(known.mean()),flicker_common_cells=int(both.sum()),
                absolute_frame_change=float(np.mean(np.abs(cost[both]-last[name][both]))) if both.any() else None))
            last[name]=cost.copy()
        np.savez_compressed(output/f'temporal-{frame}.npz',points=points,xy=xy,
            fsgp=native['cost'],bgk=fused['cost'],q=q['cost'].reshape(shape))
    write_json(output/'temporal.json',dict(frames=frames,records=records,
        note='Static flat-ramp geometry, new noisy observations; frames 2..5 omit |x|<0.7 m. Same input for all methods; FSGP and BGK share the exact same single-frame prediction. BGK fuses cost only, not height. Flicker magnitudes are comparable between FSGP and BGK only. No changing-obstacle responsiveness or navigation success was tested.'))
    return records


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=OUT)
    parser.add_argument('--scenes',nargs='+',choices=SCENES,default=SCENES)
    parser.add_argument('--conditions',nargs='+',choices=list(CONDITIONS),default=list(CONDITIONS))
    parser.add_argument('--seeds',nargs='+',type=int,default=SENSOR_SEEDS)
    parser.add_argument('--skip-temporal',action='store_true');args=parser.parse_args()
    output=args.output;output.mkdir(parents=True,exist_ok=True)
    code_files=['core.py','terrain_assessment.py','compare_fsgp_q20.py','assessment_export.py']
    hashes={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in code_files}
    protocol=dict(experiment_date='2026-11-17',scenes=args.scenes,conditions={k:CONDITIONS[k] for k in args.conditions},
        sensor_seeds=args.seeds,query_count=625,map_length_m=5.,resolution_m=.2,
        q20_config=asdict(CFG),fsgp_config=asdict(Config()),fsgp_inducing=[125,500],code_sha256=hashes,
        hypotheses=['Dense local fits may preserve discontinuities and layered surfaces.',
            'GP smoothing and spatial support may improve noisy/sparse data coverage and derivatives.',
            'BGK may retain static-scene evidence and reduce native-cost flicker; it does not repair multilevel height representation.'],
        scope='Existing local CPU equation-reference FSGP, not the official GPU program or paper benchmark. No tuning to test labels. Primary methods share byte-identical noisy XYZ. Oracle ground control uses labels explicitly and is excluded from primary comparisons.',
        input_protocol='Use existing 2.5 cm analytic scene geometry, crop to +/-2.95 m, retain .85/.45 and add declared independent XYZ noise. Queries use analytic geometry, not noisy training residuals. Official hill uses 20% disjoint clean raw indices for withheld height targets; no hill normal or hazard labels claimed.',
        ranking='Native costs have different units/weights. Compare AUROC/AP rankings on the common supported 80x50cm footprint set; no cross-method cost MAE or test-tuned pass threshold. Native scores use the same 15-grid-centre rectangular maximum, >=80% known. This is not a dynamics/stability score.',
        controls='K20-on-M125/500 uses exactly the corresponding GP retained XYZ and the historical K=20 height-only quadratic function; it is not Q20, and its post-sampling height-only timing is excluded from primary speed comparisons. FSGP-125-oracle uses generated ground labels in corridors as an explicit representation/segmentation upper-bound control.',
        timing='Geometry runtime produces the same 625 height/gradient/variance/support outputs from XYZ. Q20 native extra includes patches/jumps/clearance/foot cost, excluding assembly/body score. GP native adds its cost smoothing. No display, disk IO or source generation counted. CPU single-thread BLAS.')
    write_json(output/'protocol.json',protocol)
    cases=[]
    for scene in args.scenes:
        clean,metadata=source(scene);write_json(output/(scene+'-source.json'),metadata)
        for condition in args.conditions:
            for seed in args.seeds:
                gc.collect();case,_=run_case(scene,condition,seed,clean,output)
                cases.append(case);publish_records(output,cases)
                print(json.dumps(dict(case=case['id'],points=case['points'],common=case['common_n'],
                    metrics={r['method']:dict(coverage=round(r['known_height']/max(r['height_eligible'],1),3),
                        rmse_m=r['own']['rmse_m'],normal_deg=r['own']['normal_mean_deg'],geometry_ms=round(r['geometry_ms'],1)) for r in case['rows']})),flush=True)
    if not args.skip_temporal:temporal(output)
    assert all(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==value for name,value in hashes.items())
    print('Completed',len(cases),'paired cases',flush=True)


if __name__=='__main__':main()

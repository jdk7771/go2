"""Density-adaptive, bounded circular fits for four sparse terrain examples.

Only observed XYZ is used. Each function retains its own radius, raw neighbours,
hull and finite observation support; no cross-function assembly or gap filling.
"""
import argparse
from dataclasses import dataclass, replace
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree

from .circular_surfaces import CircularConfig, CircularSurface, export_profile
from .terrain_assessment import (evaluate, variance, fit_pools, jump_metrics,
    clearance_metrics, local_metrics, body_metrics)

SCENES=('desert','mountain','snow','vocano')
PROFILE_ID='qcircle-adaptive'
PROFILE_LABEL='自适应圆形 · R≤15 cm · 至少 15 点'
RADIUS_LEVELS=np.array([.07,.085,.10,.12,.15])


@dataclass(frozen=True)
class AdaptiveCircularConfig(CircularConfig):
    adaptive_radius: bool = True
    maximum_radius_m: float = .15
    target_points: int = 24
    coverage_margin_m: float = 0.
    height_selection: str = 'observed modes, then residual to a local tangent plane'


def observed_modes(points, ids, minimum=15, gap=.012):
    """Retain populated 2-D height modes; do not mistake sloped rows for modes."""
    ids=np.asarray(ids,dtype=np.int32)
    if len(ids)<minimum:return []
    order=ids[np.argsort(points[ids,2],kind='stable')]
    groups=np.split(order,np.flatnonzero(np.diff(points[order,2])>gap)+1)
    supported=[]
    for group in groups:
        if len(group)<minimum:continue
        xy=points[group,:2];cov=np.cov(xy,rowvar=False)
        eig=np.linalg.eigvalsh(cov)
        if eig[0]>max(eig[-1],1e-20)*.005:supported.append(np.sort(group))
    return supported if supported else [np.sort(ids)]


def tangent_pool(points, seed, mode, radius, config):
    delta=points[mode]-seed
    ids=mode[np.sum(delta[:,:2]**2,axis=1)<=radius*radius+1e-12]
    if len(ids)<config.minimum_points:return ids
    # Select a height mode again when a small neighbourhood reveals a gap that
    # was hidden by more distant samples in the original maximum-radius pool.
    modes=observed_modes(points,ids,config.minimum_points,config.local_height_gap_m)
    if len(modes)>1:
        ids=min(modes,key=lambda g:np.min(np.abs(points[g,2]-seed[2])))
    delta=points[ids]-seed
    design=np.c_[np.ones(len(ids)),delta[:,:2]]
    theta=np.linalg.lstsq(design,delta[:,2],rcond=None)[0]
    keep=np.abs(delta[:,2]-design@theta)<=config.height_gate_m+1e-9
    if keep.sum()>=config.minimum_points and not keep.all():
        theta=np.linalg.lstsq(design[keep],delta[keep,2],rcond=None)[0]
        keep=np.abs(delta[:,2]-design@theta)<=config.height_gate_m+1e-9
    return ids[keep]


def fit_adaptive_pools(points,seeds,modes,config):
    count=len(seeds)
    # The raw-point hull lies inside the search disk. Reserve half a sample
    # spacing (less the existing hull tolerance) so a regular sloping grid does
    # not leave holes between three otherwise valid neighbouring functions.
    coverage_radius=config.center_pitch_m/np.sqrt(3)+config.coverage_margin_m
    levels=RADIUS_LEVELS
    chosen=fit_pools(points,seeds,[np.empty(0,np.int32) for _ in seeds],config)
    pools=[np.empty(0,np.int32) for _ in seeds]
    radii=np.full(count,levels[-1]);accepted=np.zeros(count,bool);fallback={}
    solve_count=0;attempted=0;attempts_by_radius={}
    for radius in levels:
        pending=np.flatnonzero(~accepted)
        if not len(pending):break
        current=[tangent_pool(points,seeds[i],modes[i],radius,config) for i in pending]
        fits=fit_pools(points,seeds[pending],current,config)
        actual=int(np.isfinite(fits['rmse']).sum());solve_count+=actual;attempted+=len(pending)
        attempts_by_radius[f'{radius:.3f}']=dict(candidates=len(pending),actual_solves=actual)
        for j,i in enumerate(pending):
            # Keep the final failed attempt too, so diagnostic centres/counts
            # are not silently removed from the denominator.
            for key in chosen:chosen[key][i]=fits[key][j]
            pools[i]=current[j];radii[i]=radius
            if not np.isfinite(fits['height'][j]):continue
            # Keep smaller high-quality functions when curvature makes a wider
            # domain invalid. Among fallback domains prefer the widest valid
            # one; coverage is a preference, never a reason to accept bad fits.
            fallback[i]=(radius,current[j],{key:value[j].copy() for key,value in fits.items()})
            if len(current[j])>=config.target_points and radius>=coverage_radius-1e-12:accepted[i]=True
    for i,(radius,pool,fit) in fallback.items():
        if accepted[i]:continue
        radii[i]=radius;pools[i]=pool
        for key,value in fit.items():chosen[key][i]=value
    return chosen,pools,radii,dict(actual_solves=solve_count,attempted_candidates=attempted,
        attempts_by_radius=attempts_by_radius)


class AdaptiveCircularSurface(CircularSurface):
    def __init__(self,points,config=AdaptiveCircularConfig()):
        started=time.perf_counter();self.points=np.asarray(points,dtype='<f4').astype(float)
        self.raw_tree=cKDTree(self.points[:,:2])
        rng=np.random.default_rng(211)
        sample=self.points[rng.choice(len(self.points),min(5000,len(self.points)),replace=False)]
        distances,_=self.raw_tree.query(sample[:,:2],k=min(config.target_points,len(self.points)))
        required=float(np.percentile(distances[:,-1],70))
        spacing,_=self.raw_tree.query(self.points[:,:2],k=2)
        self.raw_spacing=spacing[:,1]
        margin=max(0.,.5*float(np.median(self.raw_spacing))-config.support_hull_tolerance_m)
        nominal=float(RADIUS_LEVELS[np.flatnonzero(RADIUS_LEVELS>=np.clip(required,.07,.12)-1e-12)[0]])
        self.config=replace(config,radius_m=nominal,patch_size_m=2*nominal,
            center_pitch_m=1.6*nominal,seed_voxel_m=1.6*nominal,coverage_margin_m=margin)
        self.density=dict(sampled_points=len(sample),seed=211,required_target_radius_p70_m=required,
                          nominal_radius_m=nominal,center_pitch_m=self.config.center_pitch_m)
        low,high=self.points[:,:2].min(0),self.points[:,:2].max(0);pitch=self.config.center_pitch_m
        rows=np.arange(np.floor(low[1]/pitch)*pitch,high[1]+pitch*.5,pitch*np.sqrt(3)/2)
        locations=np.array([(x,y) for j,y in enumerate(rows) for x in
            np.arange(np.floor(low[0]/pitch)*pitch+(j%2)*pitch/2,high[0]+pitch*.5,pitch)])
        nearby=self.raw_tree.query_ball_point(locations,self.config.maximum_radius_m,return_sorted=True)
        seeds=[];source=[];modes=[]
        for xy,ids in zip(locations,nearby):
            for mode in observed_modes(self.points,ids,self.config.minimum_points,self.config.local_height_gap_m):
                anchor=mode[np.argmin(np.sum((self.points[mode,:2]-xy)**2,axis=1))]
                seeds.append([*xy,self.points[anchor,2]]);source.append(anchor);modes.append(mode)
        self.seeds=np.asarray(seeds).reshape(-1,3);self.source_ids=np.asarray(source,np.int32)
        selected=time.perf_counter()
        self.fits,self.pools,self.model_radii_m,initial=fit_adaptive_pools(self.points,self.seeds,modes,self.config)
        fitted=time.perf_counter();self.initial_attempts=initial
        self.actual_solve_count=initial['actual_solves']
        # Raw-spacing calibration is bounded to 22..35 mm. Compatibility is
        # checked at each raw point's own XY, removing normal slope height bias.
        self.support_radii_m=np.array([np.clip(.9*np.median(self.raw_spacing[ids]),.022,.035)
            if len(ids) else .022 for ids in self.pools])
        self._hulls={};self._reindex()
        self.adaptive=self._add_observed_gaps()
        end=time.perf_counter()
        self.times=dict(select_centers_ms=(selected-started)*1000,window_query_ms=0.,
            initial_fit_and_covariance_ms=(fitted-selected)*1000,
            support_domains_and_bounded_extra_centers_ms=(end-fitted)*1000,
            fit_pipeline_ms=(end-started)*1000)

    def _add_observed_gaps(self):
        budget=int(len(self.seeds)*self.config.max_extra_center_fraction)
        info=dict(budget=budget,centers_added=0,actual_candidate_solves=0,attempted_candidates=0)
        if not budget or not len(self.active):return info
        _,raw_ids=np.unique(np.floor(self.points/.04).astype(np.int64),axis=0,return_index=True)
        missing=[]
        for start in range(0,len(raw_ids),2048):
            ids=raw_ids[start:start+2048];p=self.points[ids]
            models,_=self.query(p[:,:2],p[:,2],z_tolerance=.012)
            missing.extend(ids[models<0])
        np.random.default_rng(707).shuffle(missing)
        candidates=np.asarray(missing[:3*budget],np.int32)
        if not len(candidates):return info
        seeds=self.points[candidates];nearby=self.raw_tree.query_ball_point(seeds[:,:2],self.config.maximum_radius_m)
        modes=[]
        for seed,ids in zip(seeds,nearby):
            groups=observed_modes(self.points,ids,self.config.minimum_points,self.config.local_height_gap_m)
            modes.append(min(groups,key=lambda g:np.min(np.abs(self.points[g,2]-seed[2]))) if groups else np.empty(0,np.int32))
        fits,pools,radii,attempt=fit_adaptive_pools(self.points,seeds,modes,self.config)
        info['attempted_candidates']=len(candidates);info['actual_candidate_solves']=attempt['actual_solves']
        self.actual_solve_count+=attempt['actual_solves']
        selected=[];distance=.6*self.config.radius_m;tree=cKDTree(self.seeds[self.active])
        for j in np.flatnonzero(np.isfinite(fits['height'])):
            p=seeds[j];near=tree.query_ball_point(p,distance)
            if near:
                old=self.seeds[self.active[near]]
                if np.any((np.abs(old[:,2]-p[2])<.012)&(np.linalg.norm(old[:,:2]-p[:2],axis=1)<distance)):continue
            if selected:
                old=seeds[selected]
                if np.any((np.abs(old[:,2]-p[2])<.012)&(np.linalg.norm(old[:,:2]-p[:2],axis=1)<distance)):continue
            selected.append(j)
            if len(selected)>=budget:break
        if selected:
            self.seeds=np.r_[self.seeds,seeds[selected]];self.source_ids=np.r_[self.source_ids,candidates[selected]]
            self.pools.extend(pools[j] for j in selected)
            self.model_radii_m=np.r_[self.model_radii_m,radii[selected]]
            self.support_radii_m=np.r_[self.support_radii_m,[np.clip(.9*np.median(self.raw_spacing[pools[j]]),.022,.035) for j in selected]]
            for key in self.fits:self.fits[key]=np.concatenate((self.fits[key],fits[key][selected]))
            self._reindex();info['centers_added']=len(selected)
        return info

    def _observed(self,xy,ids,valid,k=8):
        radius=float(self.support_radii_m.max(initial=.022))
        distance,raw_ids=self.raw_tree.query(xy,k=k,distance_upper_bound=radius)
        raw=self.points[np.minimum(raw_ids,len(self.points)-1)]
        z=evaluate(self.seeds,self.fits['coefficient'],ids[:,:,None],raw[:,None,:,:2])
        return np.any((distance[:,None,:]<=self.support_radii_m[ids,None])&
            (np.abs(raw[:,None,:,2]-z)<=self.config.raw_height_tolerance_m),axis=2)&valid

    def candidates(self,xy,k=None):
        xy=np.asarray(xy,float).reshape(-1,2)
        if not len(self.active):return np.full((len(xy),1),-1,int),np.full((len(xy),1),np.nan)
        near=self.tree.query_ball_point(xy,self.config.maximum_radius_m+1e-9)
        width=max(1,max(map(len,near),default=0))
        ids=np.full((len(xy),width),self.active[0],int);valid=np.zeros(ids.shape,bool)
        for j,row in enumerate(near):ids[j,:len(row)]=self.active[row];valid[j,:len(row)]=True
        uv=xy[:,None,:]-self.seeds[ids,:2]
        valid&=np.sum(uv**2,axis=2)<=self.model_radii_m[ids]**2+1e-12
        eq=self.hull_equations[ids]
        valid&=np.all(np.sum(uv[:,:,None,:]*eq[:,:,:,:2],axis=3)+eq[:,:,:,2]<=self.config.support_hull_tolerance_m+1e-9,axis=2)
        observed=self._observed(xy,ids,valid)
        more=np.flatnonzero(np.any(valid&~observed,axis=1))
        if len(more):observed[more]=self._observed(xy[more],ids[more],valid[more],k=32)
        valid&=observed
        z=evaluate(self.seeds,self.fits['coefficient'],ids,xy[:,None,:]);z[~valid]=np.nan
        ids[~valid]=-1
        return ids,z

    def geometry(self):
        theta=np.arange(24)*2*np.pi/24;unit=np.c_[np.cos(theta),np.sin(theta)]
        positions=[];faces=[];owners=[];offset=0;std=np.full(len(self.seeds),np.nan)
        for model in self.active:
            polygon=unit*self.model_radii_m[model]
            for eq in self.hull_equations[model]:
                if not np.any(eq[:2]) or len(polygon)<3:continue
                d=polygon@eq[:2]+eq[2]-self.config.support_hull_tolerance_m
                prev=np.roll(polygon,1,axis=0);pd=np.roll(d,1)
                inside=d<=1e-9;cross=inside!=(pd<=1e-9)
                t=np.divide(pd,pd-d,out=np.zeros_like(pd),where=np.abs(pd-d)>1e-20)
                polygon=np.stack((prev+(polygon-prev)*t[:,None],polygon),axis=1).reshape(-1,2)[np.stack((cross,inside),axis=1).ravel()]
            if len(polygon)<3:continue
            center=polygon.mean(0);inner=(polygon+center)/2;n=len(polygon)
            uv=np.r_[center[None,:],inner,polygon];xy=uv+self.seeds[model,:2]
            z=evaluate(self.seeds,self.fits['coefficient'],np.full(len(uv),model),xy)
            a=np.arange(1,n+1);b=np.r_[np.arange(2,n+1),1]
            tri=np.r_[np.c_[np.zeros(n,int),a,b],np.c_[a,a+n,b+n],np.c_[a,b+n,b]]
            mesh=np.c_[xy,z]
            checks=np.r_[xy,xy[tri].mean(axis=1),(xy[tri[:,0]]+xy[tri[:,1]])/2,
                         (xy[tri[:,1]]+xy[tri[:,2]])/2,(xy[tri[:,2]]+xy[tri[:,0]])/2]
            ids=np.full((len(checks),1),model,int)
            seen=self._observed(checks,ids,np.ones(ids.shape,bool),k=16)[:,0]
            nt=len(tri);seen_tri=seen[len(uv):].reshape(4,nt).all(axis=0)
            keep=tri[np.all(seen[:len(uv)][tri],axis=1)&seen_tri]
            positions.append(mesh);faces.append(keep+offset);owners.append(np.full(len(uv),model,dtype='<u4'));offset+=len(uv)
            good=seen[:len(uv)]
            if good.any():std[model]=np.max(np.sqrt(variance(self.seeds,self.fits['covariance'],np.full(good.sum(),model),xy[good])))
        self.std=std
        return (np.concatenate(positions).astype('<f4') if positions else np.empty((0,3),'<f4'),
            np.concatenate(faces).astype('<u4') if faces else np.empty((0,3),'<u4'),
            np.concatenate(owners).astype('<u4') if owners else np.empty(0,'<u4'))

    def compute_metrics(self,vertices,faces):
        times={};last=time.perf_counter()
        def mark(name):
            nonlocal last
            now=time.perf_counter();times[name]=(now-last)*1000;last=now
        jumps=jump_metrics(self.points,self.seeds,self.pools,self.fits,self.config);mark('three_jump_methods_ms')
        n=len(self.seeds);clearance=dict(cost=np.full(n,np.nan),raw_hits=np.zeros(n,int),surface_hits=np.zeros(n,int),
            column_areas_m2=np.zeros((n,36)))
        groups={r:np.flatnonzero(self.model_radii_m==r) for r in np.unique(self.model_radii_m)}
        for radius,ids in groups.items():
            cfg=replace(self.config,radius_m=float(radius),patch_size_m=2*float(radius))
            result=clearance_metrics(self.points,self.seeds[ids],{k:v[ids] for k,v in self.fits.items()},vertices,faces,cfg)
            for key in ('cost','raw_hits','surface_hits'):clearance[key][ids]=result[key]
            clearance['column_areas_m2'][ids,:len(result['column_areas_m2'])]=result['column_areas_m2']
        mark('clearance_ms');metrics={}
        for radius,ids in groups.items():
            cfg=replace(self.config,radius_m=float(radius),patch_size_m=2*float(radius))
            result=local_metrics(self.seeds[ids],{k:v[ids] for k,v in self.fits.items()},self.std[ids],
                dict(upper_m=jumps['upper_m'][ids]),dict(cost=clearance['cost'][ids]),cfg)
            for key,value in result.items():
                if key not in metrics:metrics[key]=np.full(n,np.nan)
                metrics[key][ids]=value
        if not metrics:
            metrics=local_metrics(self.seeds,self.fits,self.std,jumps,clearance,self.config)
        mark('local_costs_ms')
        body=body_metrics(self.seeds,self.fits,metrics,jumps,clearance,self.config,support_query=self.body_support)
        mark('body_50x80_ms')
        self.assessment_result=dict(jumps=jumps,clearance=clearance,metrics=metrics,body=body,times=times)
        return self.assessment_result

    def export_extra(self,folder):
        self.model_radii_m.astype('<f4').tofile(folder/'radii.bin')
        self.support_radii_m.astype('<f4').tofile(folder/'support-radii.bin')
        values=self.model_radii_m[self.active]
        stats=dict(minimum=float(values.min()),median=float(np.median(values)),maximum=float(values.max())) if len(values) else {}
        return dict(adaptive_radius=True,radius_stats_m=stats,density_adaptation=self.density,
            radius_histogram={f'{r:.3f}':int(np.sum(values==r)) for r in np.unique(values)},
            radius_selection_solves=self.initial_attempts,
            observation_rule='Own circular radius, observed hull and 22..35 mm spacing-calibrated raw XY distance; compare raw Z with this function evaluated at the raw point XY (12 mm residual tolerance).',
            center_note='Hexagonal pitch auto-selected from the 70th percentile radius for 24 raw XY neighbours. Same observed-height mode; no fixed layer count.',
            clearance='10..60 cm observed occupancy; 5 cm columns clipped to each function own circular radius, weighted by intersection area. Raw returns plus supported fitted-display vertices; zero does not certify free space.',
            geometry_note='Independent clipped circular functions with an extra radial ring; vertices, centroids and edge midpoints require original raw support. No cross-function triangles.',
            metric_note='All 11 metrics; normal/occupancy domains use each function own radius. Original fixed 7 cm and square profiles are preserved.')


def run(names=SCENES):
    return export_profile(names,surface_factory=AdaptiveCircularSurface,profile_id=PROFILE_ID,
        profile_label=PROFILE_LABEL,partial_profile=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--scenes',nargs='+',choices=SCENES,default=SCENES)
    run(parser.parse_args().scenes)

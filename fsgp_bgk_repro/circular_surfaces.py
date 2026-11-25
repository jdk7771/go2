"""Sparse, overlapping radius-7 cm polynomial domains with observed support.

The 11.5 cm hexagonal centre lattice limits model count. Domains share existing
raw observations; overlap does not create extra measurements. All downstream
queries require both a fitted circular domain and nearby compatible raw XYZ.
"""
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree, ConvexHull, QhullError

from .terrain_assessment import (AssessmentConfig, basis, evaluate, fit_pools, variance,
                                 jump_metrics, clearance_metrics, local_metrics, body_metrics)


@dataclass(frozen=True)
class CircularConfig(AssessmentConfig):
    patch_size_m: float = .14
    seed_voxel_m: float = .115
    minimum_points: int = 15
    max_rmse_m: float = .008
    domain_shape: str = 'circle'
    radius_m: float = .07
    center_pitch_m: float = .115
    raw_support_radius_m: float = .022
    raw_height_tolerance_m: float = .012
    support_hull_tolerance_m: float = .0075
    local_height_gap_m: float = .012
    max_extra_center_fraction: float = .05
    supported_fields: tuple = ('jump_b_m','slope_deg','roughness_cost','max_error_m',
        'rmse_m','std_patch_m','clearance_cost','foot_cost','body_cost','jump_a_m','jump_upper_m')


def centre_lattice(points, cfg):
    """Hexagonal XY locations; separate observed Z clusters, no global layers."""
    tree = cKDTree(points[:,:2])
    low, high = points[:,:2].min(0), points[:,:2].max(0)
    pitch = cfg.center_pitch_m
    rows = np.arange(np.floor(low[1]/pitch)*pitch, high[1]+pitch*.5, pitch*np.sqrt(3)/2)
    locations=[]
    for j,y in enumerate(rows):
        xs=np.arange(np.floor(low[0]/pitch)*pitch+(j%2)*pitch/2, high[0]+pitch*.5, pitch)
        locations.extend((x,y) for x in xs)
    locations=np.asarray(locations)
    nearby=tree.query_ball_point(locations,cfg.radius_m,return_sorted=True)
    seeds,source=[] ,[]
    for xy,pool in zip(locations,nearby):
        if not pool: continue
        ids=np.asarray(pool,dtype=np.int32)
        ids=ids[np.argsort(points[ids,2],kind='stable')]
        groups=np.split(ids,np.flatnonzero(np.diff(points[ids,2])>.02)+1)
        for group in groups:
            if len(group)<cfg.minimum_points:continue
            anchor=group[np.argmin(np.sum((points[group,:2]-xy)**2,axis=1))]
            seeds.append([*xy,points[anchor,2]]);source.append(anchor)
    return np.asarray(seeds).reshape(-1,3),np.asarray(source,dtype=np.int32)


def circular_pools(points,seeds,cfg):
    tree=cKDTree(points[:,:2])
    nearby=tree.query_ball_point(seeds[:,:2],cfg.radius_m+1e-9,return_sorted=True)
    pools=[]
    for seed,neighbors in zip(seeds,nearby):
        ids=np.asarray(neighbors,dtype=np.int32)
        ids=ids[np.abs(points[ids,2]-seed[2])<=cfg.height_gate_m+1e-9]
        if len(ids)>=cfg.minimum_points:
            ordered=ids[np.argsort(points[ids,2],kind='stable')]
            groups=np.split(ordered,np.flatnonzero(np.diff(points[ordered,2])>cfg.local_height_gap_m)+1)
            supported=[g for g in groups if len(g)>=cfg.minimum_points]
            # An under-supported side remains invalid, rather than borrowing
            # another populated height mode to turn a step into a smooth ramp.
            if len(groups)>1 and supported:
                ids=min(groups,key=lambda g:np.min(np.abs(points[g,2]-seed[2])))
                ids=np.sort(ids)
        pools.append(ids)
    return pools


class CircularSurface:
    def __init__(self,points,config=CircularConfig()):
        start=time.perf_counter();self.config=config
        self.points=np.asarray(points,dtype='<f4').astype(float)
        self.raw_tree=cKDTree(self.points[:,:2])
        self.seeds,self.source_ids=centre_lattice(self.points,config)
        selected=time.perf_counter()
        self.pools=circular_pools(self.points,self.seeds,config)
        queried=time.perf_counter()
        self.fits=fit_pools(self.points,self.seeds,self.pools,config)
        fitted=time.perf_counter()
        base_solves=int(np.isfinite(self.fits['rmse']).sum())
        self._hulls={};self._reindex()
        self.adaptive=self._add_observed_gaps()
        self.actual_solve_count=base_solves+self.adaptive['actual_candidate_solves']
        solved=time.perf_counter()
        self.times=dict(select_centers_ms=(selected-start)*1000,window_query_ms=(queried-selected)*1000,
            initial_fit_and_covariance_ms=(fitted-queried)*1000,
            support_domains_and_bounded_extra_centers_ms=(solved-fitted)*1000,fit_pipeline_ms=(solved-start)*1000)

    def _reindex(self):
        self.active=np.flatnonzero(np.isfinite(self.fits['height']))
        for i in self.active:
            if i in self._hulls:continue
            try:self._hulls[i]=ConvexHull(self.points[self.pools[i],:2]-self.seeds[i,:2]).equations
            except QhullError:continue
        width=max((len(e) for e in self._hulls.values()),default=1)
        self.hull_equations=np.zeros((len(self.seeds),width,3));self.hull_equations[:,:,2]=-1
        for i,e in self._hulls.items():self.hull_equations[i,:len(e)]=e
        self.tree=cKDTree(self.seeds[self.active,:2])

    def _add_observed_gaps(self):
        """Spend at most 5% more models, only at actual uncovered raw returns."""
        budget=int(len(self.seeds)*self.config.max_extra_center_fraction)
        info=dict(budget=budget,centers_added=0,actual_candidate_solves=0,attempted_candidates=0)
        if not budget or not len(self.active):return info
        _,raw_ids=np.unique(np.floor(self.points/.035).astype(np.int64),axis=0,return_index=True)
        p=self.points[raw_ids];lo=self.points[:,:2].min(0)+.07;hi=self.points[:,:2].max(0)-.07
        raw_ids=raw_ids[np.all((p[:,:2]>=lo)&(p[:,:2]<=hi),axis=1)]
        missing=[]
        for begin in range(0,len(raw_ids),2048):
            ids=raw_ids[begin:begin+2048];p=self.points[ids]
            models,_=self.query(p[:,:2],p[:,2],z_tolerance=.012)
            missing.extend(ids[models<0])
        if not missing:return info
        random=np.random.default_rng(707);random.shuffle(missing)
        candidates=np.asarray(missing[:3*budget],dtype=np.int32)
        seeds=self.points[candidates];pools=circular_pools(self.points,seeds,self.config)
        fits=fit_pools(self.points,seeds,pools,self.config)
        info['attempted_candidates']=len(candidates);info['actual_candidate_solves']=int(np.isfinite(fits['rmse']).sum())
        selected=[];tree=cKDTree(self.seeds[self.active])
        for j in np.flatnonzero(np.isfinite(fits['height'])):
            p=seeds[j];near=tree.query_ball_point(p,.06)
            if near:
                old=self.seeds[self.active[near]]
                if np.any((np.abs(old[:,2]-p[2])<.012)&(np.linalg.norm(old[:,:2]-p[:2],axis=1)<.06)):continue
            if selected:
                old=seeds[selected]
                if np.any((np.abs(old[:,2]-p[2])<.012)&(np.linalg.norm(old[:,:2]-p[:2],axis=1)<.06)):continue
            selected.append(j)
            if len(selected)>=budget:break
        if selected:
            self.seeds=np.r_[self.seeds,seeds[selected]];self.source_ids=np.r_[self.source_ids,candidates[selected]]
            self.pools.extend(pools[j] for j in selected)
            for name in self.fits:self.fits[name]=np.concatenate((self.fits[name],fits[name][selected]))
            self._reindex();info['centers_added']=len(selected)
        return info

    def candidates(self,xy,k=12):
        """One matrix row per XY; NaN means no measured support for that model."""
        xy=np.asarray(xy,dtype=float).reshape(-1,2)
        if not len(self.active):
            return np.full((len(xy),1),-1,int),np.full((len(xy),1),np.nan)
        distance,ix=self.tree.query(xy,k=k,distance_upper_bound=self.config.radius_m+1e-9)
        ix=np.atleast_2d(ix).reshape(len(xy),k);distance=np.asarray(distance).reshape(len(xy),k)
        ids=self.active[np.minimum(ix,len(self.active)-1)]
        z=evaluate(self.seeds,self.fits['coefficient'],ids,xy[:,None,:])
        valid=(ix<len(self.active))&(distance<=self.config.radius_m+1e-9)
        uv=xy[:,None,:]-self.seeds[ids,:2]
        eq=self.hull_equations[ids]
        valid &= np.all(np.sum(uv[:,:,None,:]*eq[:,:,:,:2],axis=3)+eq[:,:,:,2]
            <=self.config.support_hull_tolerance_m+1e-9,axis=2)
        # A raw support check is independent of the display triangles/hull.
        rd,ri=self.raw_tree.query(xy,k=8,distance_upper_bound=self.config.raw_support_radius_m)
        rd=np.asarray(rd).reshape(len(xy),8);ri=np.asarray(ri).reshape(len(xy),8)
        raw_z=self.points[np.minimum(ri,len(self.points)-1),2]
        observed=np.any((rd[:,None,:]<=self.config.raw_support_radius_m)&
            (np.abs(raw_z[:,None,:]-z[:,:,None])<=self.config.raw_height_tolerance_m),axis=2)
        # Query extra returns only when overlapping height branches need them.
        more=np.flatnonzero(np.any(valid&~observed,axis=1))
        if len(more):
            md,mi=self.raw_tree.query(xy[more],k=32,distance_upper_bound=self.config.raw_support_radius_m)
            mz=self.points[np.minimum(mi,len(self.points)-1),2]
            observed[more]=np.any((md[:,None,:]<=self.config.raw_support_radius_m)&
                (np.abs(mz[:,None,:]-z[more,:,None])<=self.config.raw_height_tolerance_m),axis=2)
        valid &= observed
        z[~valid]=np.nan
        ids=ids.copy();ids[~valid]=-1
        return ids,z

    def query(self,xy,expected=None,z_tolerance=.045):
        xy=np.asarray(xy,dtype=float).reshape(-1,2)
        if not len(self.active):return np.full(len(xy),-1,int),np.full(len(xy),np.nan)
        ids,z=self.candidates(xy)
        if expected is None:
            cost=z.copy()  # Lowest supported branch for a ground query.
        else:
            expected=np.broadcast_to(np.asarray(expected).reshape(-1,1),z.shape)
            cost=np.abs(z-expected)
            cost[cost>z_tolerance]=np.nan
            safe=np.maximum(ids,0)
            sigma=np.sqrt(variance(self.seeds,self.fits['covariance'],safe,xy[:,None,:]))
            distance=np.linalg.norm(xy[:,None,:]-self.seeds[safe,:2],axis=2)
            cost += 2*sigma+.0001*(distance/self.config.radius_m)**2
        cost=np.where(np.isfinite(cost),cost,np.inf)
        rank=np.argmin(cost,axis=1);row=np.arange(len(xy));known=np.isfinite(cost[row,rank])
        picked=ids[row,rank].copy();height=z[row,rank].copy()
        picked[~known]=-1;height[~known]=np.nan
        return picked,height

    def geometry(self):
        """Clip each circular boundary instead of dropping a whole outer ring."""
        theta=np.arange(24)*2*np.pi/24
        circle=np.column_stack((np.cos(theta),np.sin(theta)))*self.config.radius_m
        positions=[];faces=[];owners=[];offset=0
        std=np.full(len(self.seeds),np.nan)
        for model in self.active:
            polygon=circle.copy()
            for eq in self.hull_equations[model]:
                if not np.any(eq[:2]) or len(polygon)<3:continue
                d=polygon@eq[:2]+eq[2]-self.config.support_hull_tolerance_m
                prev=np.roll(polygon,1,axis=0);pd=np.roll(d,1)
                inside=d<=1e-9;cross=inside!=(pd<=1e-9)
                denominator=pd-d
                ratio=np.divide(pd,denominator,out=np.zeros_like(pd),where=np.abs(denominator)>1e-20)
                intersections=prev+(polygon-prev)*ratio[:,None]
                output=np.stack((intersections,polygon),axis=1).reshape(-1,2)
                polygon=output[np.stack((cross,inside),axis=1).ravel()]
            if len(polygon)<3:continue
            uv=np.r_[polygon.mean(0)[None,:],polygon]
            xy=uv+self.seeds[model,:2]
            z=evaluate(self.seeds,self.fits['coefficient'],np.full(len(uv),model),xy)
            count=len(polygon)
            tri=np.c_[np.zeros(count,int),np.arange(1,count+1),np.r_[np.arange(2,count+1),1]]
            # Vertices and triangle centroids require compatible observed XYZ.
            checks=np.r_[np.c_[xy,z],np.c_[xy,z][tri].mean(axis=1)]
            rd,ri=self.raw_tree.query(checks[:,:2],k=16,distance_upper_bound=self.config.raw_support_radius_m)
            rz=self.points[np.minimum(ri,len(self.points)-1),2]
            seen=np.any((rd<=self.config.raw_support_radius_m)&(np.abs(rz-checks[:,2,None])<=self.config.raw_height_tolerance_m),axis=1)
            keep=tri[np.all(seen[:len(uv)][tri],axis=1)&seen[len(uv):]]
            positions.append(np.c_[xy,z]);faces.append(keep+offset);owners.append(np.full(len(uv),model,dtype='<u4'));offset+=len(uv)
            good=seen[:len(uv)]
            if good.any():std[model]=np.max(np.sqrt(variance(self.seeds,self.fits['covariance'],np.full(good.sum(),model),xy[good])))
        self.std=std
        return (np.concatenate(positions).astype('<f4') if positions else np.empty((0,3),'<f4'),
            np.concatenate(faces).astype('<u4') if faces else np.empty((0,3),'<u4'),
            np.concatenate(owners).astype('<u4') if owners else np.empty(0,'<u4'))

    def body_support(self, query, expected):
        """Use the actual circle/hull/raw-point support rule for body samples."""
        shape=query.shape[:-1]
        ids,z=self.query(query.reshape(-1,2),expected.ravel(),z_tolerance=self.config.height_gate_m)
        return ids.reshape(shape),z.reshape(shape)

    def compute_metrics(self, vertices, faces):
        times={};last=time.perf_counter()
        def mark(name):
            nonlocal last
            now=time.perf_counter();times[name]=(now-last)*1000;last=now
        jumps=jump_metrics(self.points,self.seeds,self.pools,self.fits,self.config)
        mark('three_jump_methods_ms')
        clearance=clearance_metrics(self.points,self.seeds,self.fits,vertices,faces,self.config)
        mark('clearance_ms')
        metrics=local_metrics(self.seeds,self.fits,self.std,jumps,clearance,self.config)
        mark('local_costs_ms')
        body=body_metrics(self.seeds,self.fits,metrics,jumps,clearance,self.config,
                          support_query=self.body_support)
        mark('body_50x80_ms')
        self.assessment_result=dict(jumps=jumps,clearance=clearance,metrics=metrics,body=body,times=times)
        return self.assessment_result

    def packed(self):
        from .assessment_export import pack_assessment
        a=self.assessment_result
        return pack_assessment(self.seeds,self.fits,self.std,a['jumps'],a['clearance'],
            a['metrics'],a['body'],np.zeros(len(self.seeds)),self.config)


def export_profile(names=None, *, surface_factory=CircularSurface, profile_id='qcircle-r07',
                   profile_label='圆形 R=7 cm · 中心约 11.5 cm · 至少 15 点', partial_profile=False):
    from .assessment_export import OUT,FIELDS,finite_json,sha,stats
    from .scene_fit_catalog import ORDER,NAMES
    names=ORDER if names is None else names
    root=OUT/'profiles'/profile_id;root.mkdir(parents=True,exist_ok=True)
    records=[]
    for name in names:
        original=OUT/name
        if not (original/'points.bin').exists():continue
        started=time.perf_counter()
        points=np.fromfile(original/'points.bin',dtype='<f4').reshape(-1,3)
        surface=surface_factory(points)
        geometry_started=time.perf_counter();vertices,faces,owners=surface.geometry();geometry_ms=(time.perf_counter()-geometry_started)*1000
        assessment=surface.compute_metrics(vertices,faces)
        models=surface.packed()
        folder=root/name;folder.mkdir(exist_ok=True)
        for filename,array in [('models.bin',models),('patches.bin',vertices),('patch-faces.bin',faces),
                ('patch-owners.bin',owners),('mesh.bin',np.empty((0,3),'<u4')),
                ('body-contacts.bin',assessment['body']['contact_ids'].astype('<i4')),
                ('body-contact-z.bin',assessment['body']['contact_z'].astype('<f8'))]:
            array.tofile(folder/filename)
        offsets=np.r_[0,np.cumsum([len(p) for p in surface.pools])].astype('<i4')
        neighbors=np.concatenate(surface.pools).astype('<i4') if surface.pools else np.empty(0,'<i4')
        offsets.tofile(folder/'neighbor-offsets.bin');neighbors.tofile(folder/'neighbors.bin')
        surface.fits['covariance'].astype('<f8').tofile(folder/'covariance.bin')
        np.savez_compressed(folder/'result.npz',models=models,source_ids=surface.source_ids,
            covariance=surface.fits['covariance'],neighbor_offsets=offsets,neighbor_ids=neighbors,
            body_contacts=assessment['body']['contact_ids'],body_contact_z=assessment['body']['contact_z'],
            jump_pairs=assessment['jumps']['pairs'],clearance_column_areas_m2=assessment['clearance']['column_areas_m2'])
        raw=folder/'points.bin'
        if not raw.exists() and not raw.is_symlink():
            raw.symlink_to(Path('../../..')/name/'points.bin')
        with (folder/'per-model.csv').open('w') as stream:
            import csv
            writer=csv.writer(stream);writer.writerow(['seed',*FIELDS]);writer.writerows(
                [i,*[float(x) if np.isfinite(x) else '' for x in row]] for i,row in enumerate(models))
        with (folder/'jump-pairs.csv').open('w') as stream:
            writer=csv.writer(stream);writer.writerow(['left','right','boundary_x','boundary_y','method_a_m','method_b_m','sigma_delta_m','method_c_upper_m'])
            writer.writerows([[float(x) if np.isfinite(x) else '' for x in row] for row in assessment['jumps']['pairs']])
        std=surface.std[surface.active]
        referenced=np.concatenate([surface.pools[i] for i in surface.active]) if len(surface.active) else np.empty(0,int)
        usage=np.bincount(referenced,minlength=len(points))
        radius=surface.config.radius_m;pitch=surface.config.center_pitch_m
        overlap=(2*radius**2*np.arccos(pitch/(2*radius))-.5*pitch*np.sqrt(4*radius**2-pitch**2))/(np.pi*radius**2)
        meta=dict(id=name,name=NAMES[name],raw_points=len(points),centers=len(surface.seeds),
            solved=int(np.isfinite(surface.fits['rmse']).sum()),valid=len(surface.active),
            actual_solve_count=surface.actual_solve_count,adaptive=surface.adaptive,
            raw_points_referenced_by_valid_fits=int(np.count_nonzero(usage)),
            raw_points_shared_by_multiple_fits=int(np.count_nonzero(usage>1)),
            mean_models_per_used_raw_point=float(usage.sum()/max(np.count_nonzero(usage),1)),
            max_models_per_raw_point=int(usage.max()) if len(usage) else 0,
            nominal_pair_overlap_area_fraction=float(overlap),
            patch_triangles=len(faces),triangles=0,components=0,fields=FIELDS,
            bounds=[points.min(0).tolist(),points.max(0).tolist()],config=asdict(surface.config),
            profile_id=profile_id,profile_label=profile_label,
            source=dict(kind='identical_saved_xyz',sha256=sha(original/'points.bin')),
            field_stats={key:stats(models[:,i]) for i,key in enumerate(FIELDS) if i>=11},
            jump_solve_counts=assessment['jumps']['solve_counts'],metrics_version='shared-assessment-v1',
            metric_note='Same 11 metric definitions and 2 cm uncertainty-cost scale as square profiles; circle boundary normals, area-weighted clipped clearance columns and observed-domain body support.',
            clearance='10..60 cm observed occupancy; 5 cm columns clipped to the radius-7 cm circle, weighted by intersection area. Raw returns plus fitted-display vertices; zero does not certify free space.',
            body='Fixed 80x50 cm rectangle, four nominal contacts +/-35 and +/-20 cm, 25 support samples, >=80% grid coverage; circle/hull/compatible-raw-point support. Geometric proxy, no path planning or dynamics.',
            timings=dict(stages_ms={**surface.times, **assessment['times'], 'display_geometry_ms':geometry_ms,
                    'total_assessment_ms':surface.times['fit_pipeline_ms']+geometry_ms+sum(assessment['times'].values())},
                total_compute_ms=surface.times['fit_pipeline_ms']+geometry_ms+sum(assessment['times'].values()),
                export_wall_ms=(time.perf_counter()-started)*1000,
                note='Single CPU run. Includes fitting, display geometry, A/B/C, occupancy, local and body metrics; excludes IO. Additional side refits are counted separately.'),
            geometry_note='Independent circular polynomial domains. Only raw-supported vertices; no centre-to-centre mesh or invented gap fill.',
            center_note='Hexagonal XY lattice, 11.5 cm pitch. Z taken from nearby observed modes; not predefined surface counts.',
            observation_rule='Each queried XY requires a valid radius-7 cm polynomial and compatible raw XYZ within 2.2 cm. This finite sampling rule is not a free-space certificate.',
            displayed_functions=int(len(np.unique(owners[faces]))) if len(faces) else 0,
            sigma_median_m=float(np.nanmedian(std)) if len(std) else None)
        if hasattr(surface,'export_extra'):
            meta.update(surface.export_extra(folder))
        (folder/'data.json').write_text(json.dumps(finite_json(meta),ensure_ascii=False,indent=2)+'\n')
        records.append(meta)
        print(json.dumps(dict(scene=name,centers=meta['centers'],valid=meta['valid'],ms=round(meta['timings']['total_compute_ms'],1))),flush=True)
    available=[json.loads((root/name/'data.json').read_text()) for name in ORDER if (root/name/'data.json').exists()]
    (root/'catalog.json').write_text(json.dumps(dict(scenes=available,fields=FIELDS,
        partial_profile=partial_profile),ensure_ascii=False,indent=2)+'\n')
    return records


if __name__=='__main__':
    export_profile()

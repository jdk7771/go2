"""Independent numerical audits and controlled noise experiments for step.md."""
import argparse
import csv
import hashlib
import json
import time
from pathlib import Path
import numpy as np

from .assessment_export import OUT, FIELDS, compute, assessment_source, finite_json, stats, publish
from .terrain_assessment import window_pools, fit_pools, jump_metrics, evaluate


def write_json(path, value):
    path.write_text(json.dumps(finite_json(value), ensure_ascii=False, indent=2)+'\n')


def background(kind, xy):
    x,y=xy.T
    if kind=='flat': return np.zeros(len(x))
    if kind=='slope': return .15*x+.08*y
    if kind=='quadratic': return .15*x+.25*x*x+.1*y*y
    if kind=='kink': return .3*np.maximum(x,0)
    if kind=='cubic': return 5*x**3
    raise ValueError(kind)


def controlled(repeats=100):
    axis=np.linspace(-.08,.08,9);x,y=np.meshgrid(axis,axis)
    offsets=np.column_stack((x.ravel(),y.ravel()))
    xy=np.concatenate((offsets+[-.12,0],offsets+[.12,0]))
    anchors=np.array([[-.12,0],[.12,0]])
    rng=np.random.default_rng(20261117)
    rows=[]
    for kind in ('flat','slope','quadratic','kink','cubic'):
        for noise in (0.,.003,.015):
            for jump in (0.,.12):
                values=[];heldout=[];elapsed=[]
                truth=background(kind,xy)+jump*(xy[:,0]>0)
                for _ in range(repeats):
                    points=np.column_stack((xy,truth+rng.normal(0,noise,len(xy))))
                    # Use noisy observed centre returns, exactly as full-scene
                    # seeding does; analytic heights must not guide the gate.
                    seeds=points[[40,121]].copy()
                    start=time.perf_counter()
                    pools=window_pools(points,seeds)
                    fit=fit_pools(points,seeds,pools)
                    result=jump_metrics(points,seeds,pools,fit)
                    elapsed.append((time.perf_counter()-start)*1000)
                    if len(result['pairs']):
                        pair=result['pairs'][0];values.append([pair[4],pair[5],pair[6],pair[7]])
                    else: values.append([np.nan]*4)
                    # Independent coordinates, noiseless truth: mean-surface error,
                    # distinct from training residuals and a new noisy observation.
                    query=np.column_stack((rng.uniform(-.07,.07,100),rng.uniform(-.07,.07,100)))
                    for side,anchor in enumerate(anchors):
                        if not np.isfinite(fit['height'][side]):continue
                        p=query+anchor
                        pred=evaluate(seeds,fit['coefficient'],np.full(len(p),side),p)
                        err=pred-(background(kind,p)+jump*(side==1))
                        heldout.extend(err[np.isfinite(err)].tolist())
                a,b,sigma,c=np.array(values).T;known=np.isfinite(b)
                row=dict(background=kind,observation_noise_m=noise,true_jump_m=jump,repeats=repeats,
                    bilateral_known=int(known.sum()),a_rmse_m=float(np.sqrt(np.nanmean((a-jump)**2))),
                    b_rmse_m=float(np.sqrt(np.nanmean((b-jump)**2))),b_mean_m=float(np.nanmean(b)),
                    c_mean_m=float(np.nanmean(c)),absolute_jump_error_within_2sigma_fraction=float(np.mean((np.abs(b-jump)<=2*sigma)[known])) if known.any() else np.nan,
                    heldout_mean_surface_rmse_m=float(np.sqrt(np.mean(np.array(heldout)**2))),
                    pair_pipeline_median_ms=float(np.median(elapsed)))
                rows.append(row)
    with (OUT/'controlled-noise.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    report=dict(repeats_per_case=repeats,random_seed=20261117,cases=rows,
                note='Two fixed, explicitly separated 16 cm supports centred at +/-12 cm. Seeds use the noisy observed centre returns, not analytic heights. XYZ noise affects the height gate. Independent held-out XY coordinates use noiseless analytic truth and valid models only. Absolute jump is compared with a nonnegative true jump. 2-sigma fractions use bilateral-known trials only and are empirical diagnostics, not a calibrated confidence claim. Cubic violates the fitted quadratic model. Candidate boundary is correctly located by construction here, unlike full scenes.')
    write_json(OUT/'controlled-noise.json',report)
    return report


def audit():
    rows=[];columns={name:i for i,name in enumerate(FIELDS)}
    for path in sorted(OUT.glob('*/data.json')):
        meta=json.loads(path.read_text());folder=path.parent
        points=np.fromfile(folder/'points.bin',dtype='<f4').reshape(-1,3).astype(float)
        packed=np.fromfile(folder/'models.bin',dtype='<f8').reshape(-1,len(FIELDS))
        covariance=np.fromfile(folder/'covariance.bin',dtype='<f8').reshape(-1,6,6)
        offsets=np.fromfile(folder/'neighbor-offsets.bin',dtype='<i4');neighbors=np.fromfile(folder/'neighbors.bin',dtype='<i4')
        mesh=np.fromfile(folder/'mesh.bin',dtype='<u4').reshape(-1,3)
        patch=np.fromfile(folder/'patches.bin',dtype='<f4').reshape(-1,3)
        faces=np.fromfile(folder/'patch-faces.bin',dtype='<u4').reshape(-1,3)
        owners=np.fromfile(folder/'patch-owners.bin',dtype='<u4')
        n=len(packed);valid=packed[:,columns['valid']].astype(bool);solved=np.isfinite(packed[:,columns['rmse_m']])
        assert len(points)==meta['raw_points'] and n==meta['centers']
        assert valid.sum()==meta['valid'] and solved.sum()==meta['solved']
        assert offsets.shape==(n+1,) and offsets[-1]==len(neighbors)
        np.testing.assert_array_equal(np.diff(offsets),packed[:,columns['point_count']])
        used=np.zeros(n,bool);used[np.unique(mesh)]=True
        np.testing.assert_array_equal(used,packed[:,columns['assembled']].astype(bool))
        assert len(mesh)==meta['triangles'] and len(faces)==meta['patch_triangles']
        assert np.unique(owners).size==valid.sum()
        if len(faces):
            assert faces.max()<len(patch)
            assert np.all(owners[faces[:,0]]==owners[faces[:,1]]) and np.all(owners[faces[:,0]]==owners[faces[:,2]])
        checks=np.unique(np.r_[np.flatnonzero(valid)[::max(1,int(valid.sum())//100)],np.flatnonzero(solved)[::max(1,int(solved.sum())//100)]])
        residual_diff=0.;cov_diff=0.;minimum_eigenvalue=0.
        for i in checks:
            ids=neighbors[offsets[i]:offsets[i+1]]
            delta=points[ids]-packed[i,:3]
            assert np.max(np.abs(delta[:,:2]))<=.100001 and np.max(np.abs(delta[:,2]))<=.040001
            x,y=delta[:,:2].T;a=np.column_stack((np.ones(len(ids)),x,y,x*x,x*y,y*y))
            residual=a@packed[i,3:9]-delta[:,2]
            residual_diff=max(residual_diff,abs(np.max(np.abs(residual))-packed[i,columns['max_error_m']]))
            scale=np.sqrt(np.mean(x*x+y*y));u,v=x/scale,y/scale
            an=np.column_stack((np.ones(len(ids)),u,v,u*u,u*v,v*v));g=an.T@an;inv=np.linalg.inv(g+np.eye(6)*1e-10*np.trace(g)/6)
            sigma2=max(np.sum(residual**2)/(len(ids)-6),.003**2);t=np.diag([1,1/scale,1/scale,1/scale**2,1/scale**2,1/scale**2])
            reference=sigma2*t@inv@g@inv.T@t
            cov_diff=max(cov_diff,float(np.max(np.abs(reference-covariance[i]))))
            minimum_eigenvalue=min(minimum_eigenvalue,float(np.linalg.eigvalsh(covariance[i]).min()))
        assert residual_diff<1e-9 and cov_diff<1e-7 and minimum_eigenvalue>-1e-10
        body=packed[:,columns['body_cost']];known=np.isfinite(body)
        assert np.all((body[known]>=0)&(body[known]<=1))
        contacts=np.fromfile(folder/'body-contacts.bin',dtype='<i4').reshape(-1,4)
        assert np.all(contacts[known]>=0) and np.all(packed[known,columns['body_coverage']]>=.8)
        with (folder/'per-model.csv').open() as f:assert sum(1 for _ in csv.DictReader(f))==n
        central=0;best=np.inf
        for i in np.flatnonzero(valid):
            score=packed[i,0]**2+packed[i,1]**2+.03*(packed[i,9]-meta['bounds'][0][2])
            if score<best:best=score;central=int(i)
        # Audit actual repeated side solves as well as primary-window solves.
        # Supports depend on XYZ and a candidate midpoint, so counts can be
        # independently recovered without solving the polynomials again.
        side_solves=0;candidate_count=0;both_valid=0
        with (folder/'jump-pairs.csv').open() as f:
            for pair in csv.DictReader(f):
                left,right=int(float(pair['left'])),int(float(pair['right']))
                middle=(packed[left,:2]+packed[right,:2])/2
                direction=packed[right,:2]-packed[left,:2];direction/=np.linalg.norm(direction)
                left_ids=neighbors[offsets[left]:offsets[left+1]];right_ids=neighbors[offsets[right]:offsets[right+1]]
                lc=np.sum(np.sum((points[left_ids,:2]-middle)*direction,axis=1)<-1e-8)
                rc=np.sum(np.sum((points[right_ids,:2]-middle)*direction,axis=1)>1e-8)
                side_solves+=int(lc>=30)+int(rc>=30);candidate_count+=1;both_valid+=bool(pair['method_b_m'])
        side_counts=dict(candidate_pairs=candidate_count,side_windows=2*candidate_count,actual_side_solves=side_solves,both_sides_valid_pairs=both_valid)
        if 'jump_solve_counts' in meta:assert meta['jump_solve_counts']==side_counts
        else:
            meta['jump_solve_counts']=side_counts;write_json(path,meta)
        rows.append(dict(scene=meta['id'],raw=len(points),centers=n,solved=int(solved.sum()),valid=int(valid.sum()),
            unassembled_valid=int(np.sum(valid&~used)),functions_with_visible_patch=int(np.unique(owners[faces]).size) if len(faces) else 0,
            checked_models=len(checks),maximum_residual_difference_m=residual_diff,maximum_covariance_difference=cov_diff,
            body_known=int(known.sum()),jump_solve_counts=side_counts,central_seed=central,central_metrics={name:packed[central,col] for name,col in columns.items() if col>=11}))
    old=[]
    from .scene_fit_catalog import OUT as HISTORICAL
    for p in HISTORICAL.glob('*/max-error-audit.json'):
        record=json.loads(p.read_text());digest=hashlib.sha256((p.parent/'results.npz').read_bytes()).hexdigest()
        # Record the frozen reference comparison when the historical audit provides it.
        expected=record.get('result_sha256',record.get('result_npz_sha256',record.get('results_sha256')))
        if expected: assert digest==expected
        old.append(dict(scene=p.parent.name,current_sha256=digest,reference_sha256=expected))
    report=dict(passed=True,scenes=rows,historical_result_hashes=old)
    write_json(OUT/'numerical-audit.json',report)
    publish()
    return report


def timing(names,repeats):
    rows=[]
    for name in names:
        points,_=assessment_source(name);runs=[]
        for i in range(repeats):
            result=compute(points);runs.append(result['times'])
            print(json.dumps(dict(scene=name,run=i+1,total_ms=result['times']['total_assessment_ms'])),flush=True)
            del result
        rows.append(dict(scene=name,repeats=repeats,runs_ms=runs,medians_ms={key:float(np.median([r[key] for r in runs])) for key in runs[0]}))
    write_json(OUT/'repeat-timing.json',dict(scenes=rows,note='Same strict 20 cm pipeline. Each run includes all calculations and patch geometry; source construction and IO excluded. Single-thread BLAS. Not end-to-end latency with sensor/visualization.'))
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--repeats',type=int,default=100)
    parser.add_argument('--timing-scenes',nargs='*',default=[]);parser.add_argument('--timing-repeats',type=int,default=3)
    args=parser.parse_args();print('Audit:',audit()['passed'],flush=True)
    print('Controlled cases:',len(controlled(args.repeats)['cases']),flush=True)
    if args.timing_scenes:timing(args.timing_scenes,args.timing_repeats)


if __name__=='__main__':main()

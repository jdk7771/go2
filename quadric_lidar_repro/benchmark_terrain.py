"""Test raw-cloud segmentation/fitting on official FSGP-BGK terrain examples."""

import argparse
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
from time import perf_counter_ns

import numpy as np
from scipy.spatial import cKDTree

from fsgp_bgk_repro.core import Config as GPConfig, TraversabilityMapper
from fsgp_bgk_repro.experiment import environment
from .benchmark_curvature import write_csv
from .core import Config
from .curvature import tangent_basis
from .terrain import SegmentationConfig, analytic_map, estimate_normals, library, local_height_map

ROOT=Path(__file__).resolve().parent
DATA=ROOT/"data/fsgp_terrain"
SCENES=["hill","forest","quick","desert","mountain","snow","vocano"]


def load_cloud(record):
    data=Path(record["local_path"]).read_bytes()
    if hashlib.sha256(data).hexdigest()!=record["sha256"]:
        raise ValueError("PCD hash mismatch: "+record["local_path"])
    end=data.index(b"DATA binary\n")+len(b"DATA binary\n")
    fields=next(line.split()[1:] for line in data[:end].decode("ascii").splitlines() if line.startswith("FIELDS"))
    array=np.frombuffer(data[end:],dtype="<f4").reshape(-1,len(fields))
    points=array[:,:3].astype(float)
    normals=array[:,3:6].astype(float) if "normal_x" in fields else None
    if normals is not None:
        normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-30)
    else:
        tree=cKDTree(points);pieces=[]
        for start in range(0,len(points),4096):
            _,ids=tree.query(points[start:start+4096],k=20,workers=1)
            normal,_=estimate_normals(points,ids);normal*=np.where(normal[:,2:3]<0,-1,1)
            pieces.append(normal)
        normals=np.vstack(pieces)
    return points,normals,"PCD provided normals" if "normal_x" in fields else "dense clean-cloud PCA normals (estimated)"


def scan_from_cloud(points, representative_ids, center, rng, gp_config):
    ids=representative_ids[np.max(np.abs(points[representative_ids,:2]-center),axis=1)<=gp_config.length/2+gp_config.support_radius]
    if len(ids)>2500:
        ids=rng.choice(ids,2500,replace=False)
    ids=rng.choice(ids,int(np.floor(len(ids)*.85)),replace=False)
    scan=points[ids].copy();scan[:,2]+=rng.normal(0,.015,len(scan))
    if len(scan)<20:
        raise ValueError("too few observed points")
    return scan,ids


def dense_reference(points,normals,tree,xy,anchor_z,support=.45):
    """Diagnostic geometric reference, never the unpublished paper cost labels.

    Height uses clean point tangent planes. Physical curvature is a local linear
    fit of clean normal variation; it is an estimate, not analytic ground truth.
    """
    query=np.column_stack((xy,anchor_z))
    distance,ids=tree.query(query,k=48,workers=1)
    p,n=points[ids],normals[ids].copy()
    reference_n=n[:,0]
    sign=np.where(np.einsum("nki,ni->nk",n,reference_n)>=0,1.,-1.)
    n*=sign[:,:,None]
    keep=np.einsum("nki,ni->nk",n,reference_n)>=np.cos(np.radians(30))
    weight=np.where(keep,1/(distance+.001),0.)
    weight/=np.maximum(weight.sum(axis=1,keepdims=True),1e-30)
    normal=np.einsum("nk,nki->ni",weight,n)
    normal/=np.maximum(np.linalg.norm(normal,axis=1,keepdims=True),1e-30)
    safe=np.abs(n[:,:,2])>.2
    height=p[:,:,2]-np.divide(n[:,:,0]*(xy[:,None,0]-p[:,:,0])+n[:,:,1]*(xy[:,None,1]-p[:,:,1]),
                            n[:,:,2],out=np.zeros_like(weight),where=safe)
    height_weight=weight*safe
    height_weight/=np.maximum(height_weight.sum(axis=1,keepdims=True),1e-30)
    height=np.sum(height_weight*height,axis=1)
    center=np.einsum("nk,nki->ni",weight,p)
    first,second=tangent_basis(normal)
    u=np.stack([np.einsum("nki,ni->nk",p-center[:,None,:],axis) for axis in (first,second)],axis=2)
    dn=n-normal[:,None,:]
    gram=np.einsum("nk,nki,nkj->nij",weight,u,u)
    rhs=np.einsum("nk,nki,nkj->nij",weight,u,dn)
    derivative=np.linalg.solve(gram+np.eye(2)*1e-12,rhs)
    basis=np.stack((first,second),axis=2)
    shape=-np.einsum("nia,nbi->nab",basis,derivative)
    principal=np.linalg.eigvalsh(.5*(shape+shape.transpose(0,2,1)))
    valid=(distance[:,0]<=support)&(keep.sum(axis=1)>=8)&(np.abs(normal[:,2])>.25)&np.isfinite(principal).all(axis=1)
    return {"height":height,"normal":normal,"principal":principal,"valid":valid,
            "slope_deg":np.degrees(np.arccos(np.clip(np.abs(normal[:,2]),0,1)))}


def gp_map(points,center,inducing):
    start=perf_counter_ns()
    mapper=TraversabilityMapper(GPConfig(inducing_points=inducing))
    result=mapper.assess(points,center=center,timestamp=0.)
    # Single-frame comparison: no claim of a BGK temporal-history experiment.
    return {"height":result["height"],"slope_deg":np.degrees(np.arctan(result["slope"])),
            "valid":result["observed"],"xy":result["xy"],"pca_surface_variation":result["curvature"],
            "gp_variance":result["variance"],"timing_ms":{**result["timing_ms"],"total_ms":(perf_counter_ns()-start)/1e6}}


def timed(functions,repeats):
    for function in functions.values():
        function()
    names=list(functions);rng=np.random.default_rng(2026)
    samples={name:[] for name in names};stages={name:[] for name in names};last={}
    for _ in range(repeats):
        for index in rng.permutation(len(names)):
            name=names[index];start=perf_counter_ns();result=functions[name]()
            samples[name].append((perf_counter_ns()-start)/1e6)
            if "timing_ms" in result:stages[name].append(result["timing_ms"])
            last[name]=result;del result
    stats={name:{"median_ms":float(np.median(v)),"p95_ms":float(np.percentile(v,95)),"samples_ms":v,
                 "stage_samples_ms":stages[name]} for name,v in samples.items()}
    return last,stats


def metrics(estimate,reference,mask):
    count=int(mask.sum())
    result={"evaluation_cells":count,"height_mae_m":None,"slope_mae_deg":None,"curvature_rmse_per_m":None}
    if not count:return result
    result["height_mae_m"]=float(np.mean(np.abs(estimate["height"][mask]-reference["height"][mask])))
    result["slope_mae_deg"]=float(np.mean(np.abs(estimate["slope_deg"][mask]-reference["slope_deg"][mask])))
    if "principal" in estimate:
        principal=estimate["principal"][mask].copy()
        flip=np.sum(estimate["normal"][mask]*reference["normal"][mask],axis=1)<0
        principal[flip]=-principal[flip,::-1]
        result["curvature_rmse_per_m"]=float(np.sqrt(np.mean((principal-reference["principal"][mask])**2)))
    return result


def summarize(rows):
    summary=[]
    for scene in dict.fromkeys(r["scene"] for r in rows):
        for method in dict.fromkeys(r["method"] for r in rows):
            subset=[r for r in rows if r["scene"]==scene and r["method"]==method]
            count=sum(r["evaluation_cells"] for r in subset)
            values={}
            for key in ["height_mae_m","slope_mae_deg","curvature_rmse_per_m"]:
                selected=[r for r in subset if r[key] is not None]
                n=sum(r["evaluation_cells"] for r in selected)
                power=2 if key=="curvature_rmse_per_m" else 1
                values[key]=(sum(r[key]**power*r["evaluation_cells"] for r in selected)/n)**(1/power) if n else None
            summary.append({"scene":scene,"method":method,"frames":len(subset),"evaluation_cell_frames":count,**values,
                "median_runtime_ms":float(np.median([r["runtime_ms"] for r in subset])),
                "mean_coverage":float(np.mean([r["coverage"] for r in subset])),
                "mean_scan_points":float(np.mean([r["scan_points"] for r in subset])),
                **{key:float(np.median([r[key] for r in subset])) for key in
                   ["normal_knn_ms","region_growing_ms","fitting_ms","evaluation_ms"] if key in subset[0]}})
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=ROOT/"outputs/fsgp_terrain")
    parser.add_argument("--scenes",nargs="+",default=SCENES,choices=SCENES)
    parser.add_argument("--frames",type=int,default=8)
    parser.add_argument("--repeats",type=int,default=7)
    parser.add_argument("--raw-repeats",type=int,default=3)
    args=parser.parse_args()
    for key in ["OPENBLAS_NUM_THREADS","OMP_NUM_THREADS","MKL_NUM_THREADS"]:
        if os.environ.get(key)!="1":parser.error("set "+key+"=1 before starting Python")
    if min(args.frames,args.repeats,args.raw_repeats)<1:parser.error("counts must be positive")
    output=args.output;output.mkdir(parents=True,exist_ok=True);(output/"frames").mkdir(exist_ok=True)
    manifest=json.loads((DATA/"manifest.json").read_text())
    records={Path(r["local_path"]).stem:r for r in manifest["files"]}
    segmentation=SegmentationConfig();fit_config=Config(fitting="taubin")
    gp_config=GPConfig();library()
    rows,latencies,scene_info,raw_timings=[],[],[],[]
    for scene in args.scenes:
        points,normals,reference_kind=load_cloud(records[scene]);tree=cKDTree(points)
        _,representatives=np.unique(np.floor(points/.06).astype(np.int64),axis=0,return_index=True)
        base_center=np.median(points[representatives,:2],axis=0)
        info={"scene":scene,"raw_points":len(points),"voxel_representatives":len(representatives),
              "reference_normals":reference_kind,"base_center":base_center.tolist(),"source":records[scene]}
        scene_info.append(info);rng=np.random.default_rng(7+1000*SCENES.index(scene))
        for frame in range(args.frames):
            center=base_center+[-.8+1.6*frame/max(args.frames-1,1),0]
            scan,source_ids=scan_from_cloud(points,representatives,center,rng,gp_config)
            xy,_,shape=TraversabilityMapper(gp_config).grid(center)
            nearest=cKDTree(scan[:,:2]).query(xy,workers=1)[1]
            reference=dense_reference(points,normals,tree,xy,points[source_ids[nearest],2])
            functions={
                "Quadric":lambda:analytic_map(scan,xy,fit_config,segmentation,center),
                "Quadric-general":lambda:analytic_map(scan,xy,replace(fit_config,planar_ratio=0.),segmentation,center),
                "Local-quadratic":lambda:local_height_map(scan,xy),
                "FSGP125":lambda:gp_map(scan,center,125),
                "FSGP500":lambda:gp_map(scan,center,500),
            }
            estimates,timing=timed(functions,args.repeats)
            common=reference["valid"].copy()
            for method in ["Quadric","Local-quadratic","FSGP125","FSGP500"]:common&=estimates[method]["valid"]
            digest=hashlib.sha256(scan.tobytes()).hexdigest()
            evidence={"scan":scan,"source_ids":source_ids,"xy":xy,"shape":shape,"center":center,"common":common}
            for key,value in reference.items():evidence["reference_"+key]=value
            for method,estimate in estimates.items():
                mask=common if method!="Quadric-general" else reference["valid"]&estimate["valid"]
                row={"scene":scene,"frame":frame,"method":method,"scan_points":len(scan),"scan_sha256":digest,
                     "queries":len(xy),"coverage":float(np.mean(estimate["valid"])),**metrics(estimate,reference,mask),
                     "runtime_ms":timing[method]["median_ms"],"p95_ms":timing[method]["p95_ms"]}
                stage_samples=timing[method]["stage_samples_ms"]
                if "patch_counts" in estimate:
                    row.update({"patches":len(estimate["patches"]),"segmented_point_fraction":estimate["segmented_point_fraction"],
                                **{kind+"_patches":count for kind,count in estimate["patch_counts"].items()},
                                **{key:float(np.median([t[key] for t in stage_samples])) for key in
                                   ["normal_knn_ms","region_growing_ms","fitting_ms","evaluation_ms"]}})
                rows.append(row)
                for key in ["height","normal","principal","slope_deg","valid"]:
                    if key in estimate:evidence[method+"_"+key]=estimate[key]
            evidence["labels"]=estimates["Quadric"]["labels"]
            np.savez_compressed(output/"frames"/f"{scene}_{frame:03}.npz",**evidence)
            latencies.append({"scene":scene,"frame":frame,"methods":timing})
            print(f"{scene:8} {frame+1}/{args.frames}: N={len(scan)} patches={len(estimates['Quadric']['patches'])} "
                  f"coverage={np.mean(estimates['Quadric']['valid']):.1%}; "
                  +", ".join(f"{m}={timing[m]['median_ms']:.2f}ms" for m in ["Quadric","Local-quadratic","FSGP125"]),flush=True)
        # Dense original PCD input, with online local-window cropping INCLUDED.
        center=base_center;xy,_,_=TraversabilityMapper(gp_config).grid(center)
        def raw_local():
            local=points[np.max(np.abs(points[:,:2]-center),axis=1)<=gp_config.length/2+gp_config.support_radius]
            return local_height_map(local,xy)
        _,timing=timed({"Quadric":lambda:analytic_map(points,xy,fit_config,segmentation,center),
                       "Local-quadratic":raw_local,"FSGP125":lambda:gp_map(points,center,125)},args.raw_repeats)
        raw_timings.append({"scene":scene,"input_points":len(points),
            "local_points":int(np.sum(np.max(np.abs(points[:,:2]-center),axis=1)<=gp_config.length/2+gp_config.support_radius)),
            "methods":timing})
        print(f"Dense {scene}: "+", ".join(f"{m}={t['median_ms']:.1f}ms" for m,t in timing.items()),flush=True)
    summary=summarize(rows)
    source_names=["terrain.py","terrain_segment.cpp","benchmark_terrain.py","terrain_report.py","core.py","curvature.py"]
    protocol={"timestamp_utc":datetime.now(timezone.utc).isoformat(),"environment":environment(),
        "fit_config":fit_config.as_dict(),"segmentation":segmentation.as_dict(),"gp_config":gp_config.as_dict(),
        "scenes":scene_info,"frames":args.frames,"repeats":args.repeats,"raw_repeats":args.raw_repeats,
        "sensor":{"reference_voxel_m":.06,"cap_before_dropout":2500,"point_dropout":.15,"added_z_noise_std_m":.015,
                  "note":"Local diagnostic sensor model. Point dropout is not ray-traced occlusion; extra Gaussian noise is not specified by paper."},
        "timing":"Warm single-thread CPU; starts at XYZ input; includes segmentation normals/KNN/regions, fitting and grid geometry. Excludes PCD I/O, reference generation and visualization. Compilation occurs before timing.",
        "segmentation_method":"Unorganized 20-NN PCA normals; 1.5m XY blocks; 3D distance/normal/point-to-plane constrained connected components; no supplied labels or PCD normals.",
        "fsgp":"Independent CPU reference; 125/500 inducing points, single frame without BGK; also outputs uncertainty/cost. Not author GPU execution.",
        "metrics":"Height/slope against dense clean-cloud geometric reference. Curvature reference is estimated from normal variation, not author ground truth. PCA curvature index and geometric curvature units differ.",
        "common_mask":"Reference and current support intersection of Quadric, Local-quadratic and both FSGP variants. Quadric-general sensitivity uses its own reference intersection, reported separately.",
        "dataset_scope":"All seven official PCD examples at pinned commit. They are not the complete 2500 paper benchmark; Road/Indoor/Ruin original evaluation datasets are not in this release.",
        "raw_input_run":"Unmodified full PCD XYZ in memory, then local-window crop timed; no 0.06m thinning, cap, dropout or added noise.",
        "source_sha256":{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in source_names},
        "fsgp_source_sha256":{name:hashlib.sha256((ROOT.parent/"fsgp_bgk_repro"/name).read_bytes()).hexdigest() for name in ["core.py","io.py"]},
        "manifest_sha256":hashlib.sha256((DATA/"manifest.json").read_bytes()).hexdigest(),
        "sources":["https://arxiv.org/html/2503.04134v2","https://github.com/ZJU-FAST-Lab/FSGP_BGK"]}
    write_csv(output/"per_frame.csv",rows);write_csv(output/"summary.csv",summary)
    (output/"protocol.json").write_text(json.dumps(protocol,indent=2)+"\n")
    metrics_record={"summary":summary,"latencies":latencies,"dense_input_latencies":raw_timings}
    (output/"metrics.json").write_text(json.dumps(metrics_record,indent=2,allow_nan=False)+"\n")
    from .terrain_report import create_report
    create_report(output,protocol,metrics_record)
    print("Saved "+str(output),flush=True)


if __name__=="__main__":main()

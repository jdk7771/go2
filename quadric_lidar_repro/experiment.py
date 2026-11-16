"""Full-sequence raw LiDAR odometry, incremental mapping and saved-map localization."""

import csv
import hashlib
import json
from pathlib import Path
import platform
from time import perf_counter

import numpy as np
import scipy
from scipy.spatial.transform import Rotation

from .core import (Config, QuadricMap, SpatialHash, load_map, register, transform)
from .kitti import ground_truth, summarize, write_poses
from .segmentation import segment_scan

ROOT=Path(__file__).resolve().parent


def write_csv(path, rows):
    with Path(path).open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)


def localization(files, targets, truth, config, output):
    rng=np.random.default_rng(config.seed)
    index=SpatialHash(targets,config.hash_resolution)
    records=[]
    for frame in range(10,len(files),25):
        points=np.fromfile(files[frame],dtype='<f4').reshape(-1,4)[:,:3]
        patches,lists,_=segment_scan(points,config)
        initial=truth[frame].copy()
        noise=rng.uniform(-.2,.2,2);yaw=rng.uniform(-5,5)
        initial[:2,3]+=noise
        initial[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@initial[:3,:3]
        started=perf_counter()
        nearby=index.query(initial[:3,3],radius=35)
        lookup_ms=(perf_counter()-started)*1000
        estimate,info=register(patches,lists,nearby,initial,config)
        error=np.linalg.inv(truth[frame])@estimate
        records.append({'frame':frame,'initial_xy_noise_m':float(np.linalg.norm(noise)),
                        'initial_yaw_noise_deg':float(yaw),'translation_error_m':float(np.linalg.norm(error[:3,3])),
                        'rotation_error_deg':float(Rotation.from_matrix(error[:3,:3]).magnitude()*180/np.pi),
                        'runtime_ms':(perf_counter()-started)*1000,'hash_lookup_ms':lookup_ms,
                        'candidate_patches':len(nearby),'selected_points':info['selected_points'],
                        'accepted_steps':info['accepted_steps']})
    if records:
        write_csv(output/'localization.csv',records)
    return records


def benchmark(data,sequence,output,config=None,limit=None):
    data,output=Path(data),Path(output)
    output.mkdir(parents=True,exist_ok=True)
    config=config or Config()
    files=sorted((data/'sequences'/sequence/'velodyne').glob('*.bin'))
    if limit is not None: files=files[:limit]
    if len(files)<2: raise ValueError('need at least two LiDAR frames')
    # GT is read only after all odometry/mapping estimates have been generated.
    mapping=QuadricMap(config)
    odom_poses,map_poses,rows=[],[],[]
    relative=np.eye(4)
    odom_pose,map_pose=np.eye(4),np.eye(4)
    previous=None
    selected_clouds=[]
    start=perf_counter()
    for frame,file in enumerate(files):
        scan=np.fromfile(file,dtype='<f4').reshape(-1,4)[:,:3]
        started=perf_counter();patches,lists,stats=segment_scan(scan,config)
        feature_ms=(perf_counter()-started)*1000
        if not patches: raise RuntimeError(f'no patches at frame {frame}')
        odom_ms=match_ms=0.
        used=accepted=0
        if previous is not None:
            started=perf_counter()
            relative,info=register(patches,lists,previous,relative,config)
            odom_pose=odom_pose@relative
            odom_ms=(perf_counter()-started)*1000
            used,accepted=info['selected_points'],info['accepted_steps']
            started=perf_counter()
            map_pose,_=register(patches,lists,mapping.active,map_pose@relative,config)
            match_ms=(perf_counter()-started)*1000
        started=perf_counter();mapping.update(patches,lists,map_pose)
        update_ms=(perf_counter()-started)*1000
        odom_poses.append(odom_pose.copy());map_poses.append(map_pose.copy())
        rows.append({'frame':frame,'raw_points':len(scan),**stats,'feature_ms':feature_ms,'scan_to_scan_ms':odom_ms,
                     'scan_to_map_ms':match_ms,'map_update_ms':update_ms,
                     'odometry_total_ms':feature_ms+odom_ms,'mapping_total_ms':feature_ms+odom_ms+match_ms+update_ms,
                     'active_patches':len(mapping.active),'archived_patches':len(mapping.archive),
                     'selected_points':used,'accepted_steps':accepted})
        previous=patches
        if frame%10==0 or frame==len(files)-1:
            print(f'KITTI {sequence} frame {frame+1}/{len(files)}: patches={len(patches)} active={len(mapping.active)} '
                  f'odom={rows[-1]["odometry_total_ms"]:.0f}ms mapping={rows[-1]["mapping_total_ms"]:.0f}ms '
                  f'position={map_pose[:3,3].round(2).tolist()}',flush=True)
            np.savez_compressed(output/'progress.npz',odometry=odom_poses,mapping=map_poses)
        if frame in {0,len(files)//2,len(files)-1}:
            indices=np.linspace(0,len(scan)-1,min(6000,len(scan)),dtype=int)
            selected_clouds.append(transform(scan[indices],map_pose))
            if frame==0:
                labels=np.concatenate([np.full(len(p),{'plane':0,'quadric':1,'distribution':2}[patch.kind]) for p,patch in zip(lists,patches)])
                all_points=np.vstack(lists)
                selected=np.linspace(0,len(all_points)-1,min(18000,len(all_points)),dtype=int)
                np.savez_compressed(output/'first_scan_patches.npz',points=all_points[selected],labels=labels[selected],
                                    centers=[p.center for p in patches],kinds=[p.kind for p in patches])
    odom_poses,map_poses=np.asarray(odom_poses),np.asarray(map_poses)
    write_csv(output/'per_frame.csv',rows)
    map_count=mapping.save(output/'quadric_map.npz')
    reloaded=load_map(output/'quadric_map.npz',config)
    gt,camera_gt,tr=ground_truth(data,sequence)
    gt,camera_gt=gt[:len(files)],camera_gt[:len(files)]
    cam_odom=tr@odom_poses@np.linalg.inv(tr)
    cam_map=tr@map_poses@np.linalg.inv(tr)
    write_poses(output/f'{sequence}_gt_camera.txt',camera_gt)
    write_poses(output/f'{sequence}_odometry_camera.txt',cam_odom)
    write_poses(output/f'{sequence}_mapping_camera.txt',cam_map)
    metrics={}
    for name,predicted,camera_predicted in [('odometry',odom_poses,cam_odom),('mapping',map_poses,cam_map)]:
        metrics[name],segments=summarize(gt,predicted,camera_gt,camera_predicted,data,
                                       output/f'{sequence}_gt_camera.txt',output/f'{sequence}_{name}_camera.txt',ROOT/'build')
        write_csv(output/f'{name}_segment_errors.csv',segments)
    records=localization(files,reloaded,gt,config,output)
    if records:
        metrics['localization']={key:float(np.median([r[key] for r in records])) for key in
                                 ('translation_error_m','rotation_error_deg','runtime_ms','hash_lookup_ms')}
        metrics['localization']['queries']=len(records)
    np.savez_compressed(output/'trajectories.npz',ground_truth=gt,odometry=odom_poses,mapping=map_poses,
                        sample_map_points=np.vstack(selected_clouds))
    metrics['timing']={key:float(np.median([r[key] for r in rows[1:]])) for key in
                        ('feature_ms','scan_to_scan_ms','scan_to_map_ms','map_update_ms','odometry_total_ms','mapping_total_ms')}
    raw_xyz_bytes=sum(file.stat().st_size//4*3 for file in files)
    metrics['storage']={'raw_xyz_float32_bytes':raw_xyz_bytes,'serialized_map_bytes':(output/'quadric_map.npz').stat().st_size,
                        'serialized_map_patches':map_count,
                        'raw_xyz_to_npz_ratio':raw_xyz_bytes/(output/'quadric_map.npz').stat().st_size,
                        'note':'full input XYZ float32 payload vs compressed moments-only NPZ; includes geometric approximation and ZIP compression'}
    (output/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    implementation=['core.py','segmentation.py','segment.cpp','experiment.py','kitti.py','__main__.py']
    protocol={'paper':'https://arxiv.org/abs/2304.14190v1','scope':'independent CPU paper-equation reference, not author code',
              'sequence':sequence,'frames':len(files),'full_sequence':len(files)==len(np.loadtxt(data/'poses'/f'{sequence}.txt')),
              'config':config.as_dict(),'data_manifest_sha256':hashlib.sha256((data/f'sequence_{sequence}_manifest.json').read_bytes()).hexdigest(),
              'implementation_sha256':{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in implementation},
              'paper_pdf_sha256':hashlib.sha256((ROOT/'paper/paper.pdf').read_bytes()).hexdigest(),
              'coordinate_convention':'LiDAR poses expressed in first-LiDAR frame; official metric evaluated after conjugation into camera frame',
              'ground_truth_usage':'not used in odometry or mapping; localization initialization is GT + U(-0.2,0.2)m XY and U(-5,5)deg yaw, following paper Sec V-C',
              'localization_map':'saved map built from estimated scan-to-map trajectory; no ground-truth map correction',
              'deviations':['64-beam nominal HDL-64 ring projection, 2048 columns; extra 35-degree normal gate in range segmentation',
                            'quadric fitting uses the Schur-complement LS of ref [34] (or explicit --fitting taubin ablation); sensor-specific maximum-likelihood refinement is not implemented',
                            'PCA plane ratio, regularization, residual gates, sample budgets and LM stopping settings are explicit local choices',
                            'patch association and optimization use deterministic point budgets rather than every source point',
                            'default stability variant adds a 45-degree planar-normal association gate and Cauchy loss (scale 1); --literal disables both',
                            'integer tuple spatial hashing instead of binary GeoHash; same 1.5m cell size and lowest-MSE collision rule',
                            'no ROS, deskew/IMU fusion, loop closure, or official LOAM baseline; timings are Python/C++ reference on this CPU'],
              'numpy':np.__version__,'scipy':scipy.__version__,'platform':platform.platform(),'wall_seconds':perf_counter()-start}
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    from .report import create_report
    create_report(output,metrics,protocol)
    print(json.dumps(metrics,indent=2),flush=True)
    return metrics

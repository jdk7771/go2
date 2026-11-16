"""Verify data/source hashes and independently recompute reported KITTI metrics."""

import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from .core import Config, load_map
from .kitti import ground_truth, summarize

ROOT=Path(__file__).resolve().parent


def audit(output=ROOT/'outputs/kitti04',data=ROOT/'data/kitti'):
    output,data=Path(output),Path(data)
    protocol=json.loads((output/'protocol.json').read_text())
    metrics=json.loads((output/'metrics.json').read_text())
    manifest_path=data/f'sequence_{protocol["sequence"]}_manifest.json'
    assert hashlib.sha256(manifest_path.read_bytes()).hexdigest()==protocol['data_manifest_sha256']
    manifest=json.loads(manifest_path.read_text())
    for name,digest in protocol['implementation_sha256'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,f'code changed: {name}'
    assert hashlib.sha256((ROOT/'paper/paper.pdf').read_bytes()).hexdigest()==protocol['paper_pdf_sha256']
    checked=0
    for record in manifest['members'][:protocol['frames']]:
        path=data/record['name'].removeprefix('dataset/')
        assert path.stat().st_size==record['bytes']
        assert hashlib.sha256(path.read_bytes()).hexdigest()==record['sha256']
        assert record['bytes']%16==0
        checked+=1
    for name,record in manifest['auxiliary'].items():
        assert hashlib.sha256((data/name).read_bytes()).hexdigest()==record['sha256']
    truth,camera_truth,tr=ground_truth(data,protocol['sequence'])
    n=protocol['frames'];truth,camera_truth=truth[:n],camera_truth[:n]
    with np.load(output/'trajectories.npz') as arrays:
        np.testing.assert_allclose(arrays['ground_truth'],truth,atol=1e-12)
        for name in ['odometry','mapping']:
            prediction=arrays[name]
            assert prediction.shape==(n,4,4) and np.isfinite(prediction).all()
            np.testing.assert_allclose(prediction[0],np.eye(4),atol=1e-12)
            np.testing.assert_allclose(prediction[:,3],np.tile([0,0,0,1],(n,1)),atol=1e-12)
            rotation=prediction[:,:3,:3]
            np.testing.assert_allclose(rotation.transpose(0,2,1)@rotation,np.tile(np.eye(3),(n,1,1)),atol=1e-9)
            camera=tr@prediction@np.linalg.inv(tr)
            stored=np.loadtxt(output/f'{protocol["sequence"]}_{name}_camera.txt').reshape(-1,3,4)
            np.testing.assert_allclose(stored,camera[:,:3],atol=1e-8)
            result,_=summarize(truth,prediction,camera_truth,camera,data,
                               output/f'{protocol["sequence"]}_gt_camera.txt',output/f'{protocol["sequence"]}_{name}_camera.txt',ROOT/'build')
            for key in ['kitti_translation_error_percent','kitti_rotation_error_deg_per_100m','ape_rmse_m_no_alignment','final_position_error_m']:
                np.testing.assert_allclose(result[key],metrics[name][key],atol=1e-8)
    with (output/'per_frame.csv').open() as stream:rows=list(csv.DictReader(stream))
    assert len(rows)==n
    for key,value in metrics['timing'].items():
        np.testing.assert_allclose(np.median([float(r[key]) for r in rows[1:]]),value,atol=1e-10)
    with np.load(output/'quadric_map.npz') as saved:
        assert set(saved.files)=={'origins','moments','counts','resolution'}
        assert saved['moments'].shape[1]==55
        assert np.isfinite(saved['moments']).all() and np.all(saved['counts']>0)
        assert len(saved['counts'])==metrics['storage']['serialized_map_patches']
    patches=load_map(output/'quadric_map.npz',Config(**protocol['config']))
    assert len(patches)==metrics['storage']['serialized_map_patches']
    localization=[]
    if (output/'localization.csv').exists():
        with (output/'localization.csv').open() as stream:localization=list(csv.DictReader(stream))
        assert len(localization)==metrics['localization']['queries']
        for key in ['translation_error_m','rotation_error_deg','runtime_ms','hash_lookup_ms']:
            np.testing.assert_allclose(np.median([float(r[key]) for r in localization]),metrics['localization'][key],atol=1e-10)
    result={'status':'passed','input_frames_hashed':checked,'auxiliary_hashes_verified':len(manifest['auxiliary']),
            'algorithm_hashes_verified':True,'trajectory_se3_and_coordinates_valid':True,
            'official_kitti_metric_crosscheck':True,'timing_medians_recomputed':True,
            'saved_map_has_moments_and_no_historic_points':True,'localization_queries':len(localization),
            'accuracy_claim':'numerical consistency only; not a claim of matching paper accuracy'}
    (output/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/kitti04')
    parser.add_argument('--data',type=Path,default=ROOT/'data/kitti')
    args=parser.parse_args();audit(args.output,args.data)

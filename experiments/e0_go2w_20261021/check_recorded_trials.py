"""Check raw integrated-state integrity and actuator limits independently."""
import json
from pathlib import Path

import numpy as np

from model import ROOT, load_recorded_model


def check(path):
    p=Path(path);m=load_recorded_model(p/'model.xml')
    s=np.load(p/'searched_shift_states.npz');r=json.loads((p/'searched_shift.json').read_text())
    t=s['time'];q=s['qpos'];v=s['qvel'];ctrl=s['ctrl']
    assert q.shape==(len(t),m.nq) and v.shape==(len(t),m.nv)
    assert ctrl.shape==(len(t),m.nu)
    assert np.isfinite(q).all() and np.isfinite(v).all() and np.isfinite(ctrl).all()
    assert np.allclose(np.diff(t),r['dt'],atol=1e-9)
    assert np.max(abs(np.linalg.norm(q[:,3:7],axis=1)-1))<1e-6
    assert np.all(ctrl>=m.actuator_ctrlrange[:,0]-1e-9)
    assert np.all(ctrl<=m.actuator_ctrlrange[:,1]+1e-9)
    # Position increments must agree with the scale of integrated velocities.
    assert np.max(abs(np.diff(q[:,:3],axis=0))) <= r['dt']*np.max(abs(v[:,:3]))+1e-4
    if 'external_wrench' in s:
        impulse=float(s['external_wrench'][:,1].sum()*r['dt'])
        assert abs(impulse-r.get('push_impulse_y_ns',0.))<1e-8
    return dict(directory=str(p.relative_to(ROOT)),states=len(t),recorded_state_integrity_passed=True)


if __name__=='__main__':
    cases=[ROOT/'e1_height_sweep'/f'h{x:.3f}' for x in [.043,.086,.129,.172]]+[ROOT/'e1_dt001']
    cases += sorted((ROOT/'robustness_screen').glob('mu*'))
    result=[check(p) for p in cases]
    (ROOT/'recorded_trial_checks.json').write_text(json.dumps(result,indent=2))
    print(f'Raw-state integrity and torque limits checked in {len(result)} trials.')

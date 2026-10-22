"""Validate the new controller on the formerly failing 1.5-radius obstacle.

Nominal, finer integration, and two low-friction/push cells. This is a small
deterministic development screen, not statistical evaluation.
"""
import json
from model import ROOT
from e1_local import run
from check_recorded_trials import check
from validate_geometry_sat import validate


if __name__=='__main__':
    out=ROOT/'retarget_validation';out.mkdir(exist_ok=True)
    results=[];checks=[]
    for name,dt,mu,push in [('nominal',.002,.8,0.),('dt001',.001,.8,0.),
                           ('mu03_push_minus20',.002,.3,-20.),('mu03_push_plus20',.002,.3,20.)]:
        dest=out/name
        results.append(run('searched_shift',dt,dest,feedback=True,wheel_feedback=True,
            height=.129,retarget_swing=True,friction=mu,push_force=push))
        (out/'summary.json').write_text(json.dumps(results,indent=2))
        checks.append(dict(case=name,integrity=check(dest),geometry=validate(dest)))
        (out/'validation.json').write_text(json.dumps(checks,indent=2))

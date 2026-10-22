"""Return body laterally during descent, paired with/without online FL IK."""
import json
from model import ROOT
from e1_local import run
from check_recorded_trials import check
from validate_geometry_sat import validate


if __name__=='__main__':
    out=ROOT/'landing_reference_screen';out.mkdir(exist_ok=True)
    results=[];checks=[]
    for retarget in [False,True]:
        dest=out/f'retarget{int(retarget)}'
        result=run('searched_shift',.002,dest,feedback=True,wheel_feedback=True,
            height=.129,retarget_swing=retarget,relax_landing=True)
        results.append(result)
        (out/'summary.json').write_text(json.dumps(results,indent=2))
        checks.append(dict(retarget_swing=retarget,integrity=check(dest),geometry=validate(dest)))
        (out/'validation.json').write_text(json.dumps(checks,indent=2))

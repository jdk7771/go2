"""Test measured-base FL retargeting with the original execution gates."""
import json
from model import ROOT
from e1_local import run


if __name__=='__main__':
    out=ROOT/'retarget_screen';out.mkdir(exist_ok=True)
    results=[]
    for height in [.086,.129]:
        results.append(run('searched_shift',.002,out/f'h{height:.3f}',
            feedback=True,wheel_feedback=True,height=height,retarget_swing=True))
        (out/'summary.json').write_text(json.dumps(results,indent=2))

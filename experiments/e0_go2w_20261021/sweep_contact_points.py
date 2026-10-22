"""Paired screen of assumed versus measured support-force application points."""
import json
from model import ROOT
from e1_local import run


if __name__=='__main__':
    out=ROOT/'contact_point_screen';out.mkdir(exist_ok=True)
    results=[]
    for height in [.086,.129]:
        for measured in [False,True]:
            results.append(run('searched_shift',.002,out/f'h{height:.3f}_measured{int(measured)}',
                feedback=True,wheel_feedback=True,height=height,contact_points=measured))
            (out/'summary.json').write_text(json.dumps(results,indent=2))

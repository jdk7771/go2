"""Certify the geometry of each accepted perturbation trace independently."""
import json
from model import ROOT
from validate_geometry_sat import validate

out=ROOT/'robustness_screen'
rows=[]
for mu in [.3,.5,.8]:
    for push in [-20.,0.,20.]:
        directory=out/f'mu{mu:.1f}_push{push:+.0f}'
        result=validate(directory)
        rows.append(dict(actual_friction=mu,push_force_y_n=push,**result))
(out/'geometry_summary.json').write_text(json.dumps(rows,indent=2))

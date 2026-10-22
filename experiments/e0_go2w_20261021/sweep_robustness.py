"""E1 development perturbations: actual friction vs a known lateral push.

The controller still assumes mu=0.8. Actual injected wrench and impulse are
saved. Each cell is one deterministic case, not a statistical success rate.
"""
import json
from model import ROOT
from e1_local import run

out=ROOT/'robustness_screen';out.mkdir(exist_ok=True)
rows=[]
for mu in [.3,.5,.8]:
    for push in [-20.,0.,20.]:
        rows.append(run('searched_shift',.002,out/f'mu{mu:.1f}_push{push:+.0f}',feedback=True,
            height=.086,wheel_feedback=True,friction=mu,push_force=push))
(out/'summary.json').write_text(json.dumps(rows,indent=2))

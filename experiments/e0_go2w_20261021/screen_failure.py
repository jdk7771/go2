"""Diagnose the 12.9 cm failure using control gain vs swing reference changes.

These development cases are selected after observing a failure and are not an
independent test set. The same landing target and pass gates are retained.
"""
import json
from model import ROOT
from e1_local import run

out=ROOT/'failure_screen';out.mkdir(exist_ok=True)
rows=[]
for gain,clearance in [(50.,.07),(100.,.07),(25.,.04),(50.,.04),(100.,.04)]:
    rows.append(run('searched_shift',.002,out/f'gain{gain:.0f}_clear{clearance:.2f}',
        feedback=True,height=.129,wheel_feedback=True,body_gain=gain,swing_clearance=clearance))
(out/'summary.json').write_text(json.dumps(rows,indent=2))

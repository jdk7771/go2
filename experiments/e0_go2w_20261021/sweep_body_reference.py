"""Test a different motion family after high-swing folding/landing failures.

The body raises during swing and crouches again for touchdown. Same landing
target, controller gains and acceptance gates as the constant-height cases.
"""
import json
from model import ROOT
from e1_local import run

out=ROOT/'body_reference_screen';out.mkdir(exist_ok=True)
rows=[]
for h in [.129,.172]:
    for raise_by in [.03,.06,.09]:
        rows.append(run('searched_shift',.002,out/f'h{h:.3f}_raise{raise_by:.2f}',feedback=True,
            height=h,wheel_feedback=True,body_raise=raise_by))
(out/'summary.json').write_text(json.dumps(rows,indent=2))

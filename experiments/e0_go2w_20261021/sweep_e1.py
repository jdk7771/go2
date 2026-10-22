"""Single-factor development sweep. No trained policy, no statistical success rate."""
import json
from e1_local import run
from model import ROOT

out=ROOT/'e1_height_sweep';out.mkdir(exist_ok=True)
rows=[]
for height in [.043,.086,.129,.172]:
    for method in ['fixed','searched_shift']:
        row=run(method,.002,out/f'h{height:.3f}',feedback=True,height=height,wheel_feedback=True)
        rows.append(row)
(out/'summary.json').write_text(json.dumps(rows,indent=2))

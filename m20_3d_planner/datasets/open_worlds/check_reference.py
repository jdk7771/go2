import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import sys,json,time
from pathlib import Path
from collections import deque
sys.path.insert(0,str(Path(__file__).resolve().parents[3]))
import numpy as np
from m20_3d_planner.open_worlds import DATA,triangle_hits_box
out=DATA/'grid_references';out.mkdir(exist_ok=True)
tasks=json.loads((DATA/'tasks.json').read_text())['tasks']
axes=[np.linspace(-1.3,1.3,53),np.linspace(-.6,.6,25),np.array([.28,.4,.52])]
results=[]
for task in tasks:
 start=time.perf_counter()
 with np.load(DATA/'prepared'/(task['name']+'.npz')) as d:triangles=d['collision_triangles']
 half=np.array([.375,.115,.07000000254808628])+.02+task['analytic_tessellation_error_bound_m']
 begin=(0,12,1);goal=(52,12,1);queue=deque([begin]);parents={begin:None};checks=0
 def position(key):return np.array([axes[i][j] for i,j in enumerate(key)])
 while queue and goal not in parents:
  key=queue.popleft();a=position(key)
  for dimension in range(3):
   for sign in (-1,1):
    dest=list(key);dest[dimension]+=sign
    if not 0<=dest[dimension]<len(axes[dimension]):continue
    dest=tuple(dest)
    if dest in parents:continue
    b=position(dest);checks+=1
    if not triangle_hits_box(triangles,(a+b)/2,np.eye(3),half+abs(b-a)/2):
     parents[dest]=key;queue.append(dest)
 found=goal in parents
 result=dict(scene=task['name'],translation_reference_found=found,meaning='A found axis-aligned swept-box route establishes a >=20mm geometric path in the local bounds for zero attitude. Absence does not establish infeasibility. Reference may have more than 23 nodes and is never provided to the solver.',checks=checks,visited=len(parents),seconds=time.perf_counter()-start,used_for_initialization=False,used_for_planning=False)
 if found:
  path=[];key=goal
  while key is not None:path.append(position(key));key=parents[key]
  poses=np.zeros((len(path),6));poses[:,:3]=path[::-1]
  np.savez_compressed(out/(task['name']+'.npz'),poses=poses)
 (out/(task['name']+'.json')).write_text(json.dumps(result,indent=2));results.append(result);print(json.dumps(result),flush=True)
(out/'summary.json').write_text(json.dumps(results,indent=2))

"""Independent PhysX collision queries. NOT a robot locomotion evaluation.

Procedural static geometry only; no URDF, learned controller, dynamic tracking,
friction/contact integration or photorealistic sensor reconstruction is tested.
"""
import json
import math
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})

import carb
import numpy as np
import omni.usd
from omni.physx import get_physx_scene_query_interface
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.api import World

OUT = Path(__file__).resolve().parent
data = json.loads((OUT / "results.json").read_text())
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()


def cube(name, low, high):
    low, high = np.array(low, dtype=float), np.array(high, dtype=float)
    shape = UsdGeom.Cube.Define(stage, name)
    shape.CreateSizeAttr(1.)
    shape.AddTranslateOp().Set(Gf.Vec3d(*((low+high)/2).tolist()))
    shape.AddScaleOp().Set(Gf.Vec3d(*(high-low).tolist()))
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())


cube("/World/SwingObstacle", *data["swing"]["box"])
cube("/World/CorridorWallA", [2., .2, 0.], [4., .35, 1.])
cube("/World/CorridorWallB", [2., -.35, 0.], [4., -.2, 1.])
knee = data["diagnostics"]["knee"]
shift = np.array([6., 0., 0.])
cube("/World/KneeObstacle", np.array(knee["obstacle"])[0]+shift,
     np.array(knee["obstacle"])[1]+shift)
world.reset()
for _ in range(5):
    world.step(render=False)
query = get_physx_scene_query_interface()


def hits_sphere(center, radius):
    hits = []
    def hit_cb(hit):
        hits.append(str(hit.collision))
        return True
    query.overlap_sphere(float(radius), carb.Float3(*map(float, center)), hit_cb, False)
    return hits


def hits_box(center, half, yaw=0.):
    hits = []
    def hit_cb(hit):
        hits.append(str(hit.collision))
        return True
    query.overlap_box(carb.Float3(*map(float, half)), carb.Float3(*map(float, center)),
                      carb.Float4(0., 0., math.sin(yaw/2), math.cos(yaw/2)), hit_cb, False)
    return hits


searched = np.array(data["swing"]["searched"])
dense = np.concatenate([np.linspace(a, b, 11) for a, b in zip(searched[:-1], searched[1:])])
nominal = data["swing"]["nominal"]
result = {
    "scope": "Isaac Sim 5.1.0 static PhysX overlap queries, no locomotion dynamics or RL",
    "swing": {
        "nominal_sample_count": len(nominal),
        "nominal_colliding_samples": sum(bool(hits_sphere(p, .06)) for p in nominal),
        "searched_sample_count": len(dense),
        "searched_colliding_samples": sum(bool(hits_sphere(p, .06)) for p in dense),
    },
    "orientation": {
        "yaw0_collision": bool(hits_box([3., 0., .5], [.3, .15, .08])),
        "yaw30_collision": bool(hits_box([3., 0., .5], [.3, .15, .08], math.pi/6)),
    },
    "knee": {
        "torso_collision": bool(hits_box(shift+np.array([0., 0., .57]), [.25, .14, .07])),
        "wheel_collision": bool(hits_sphere(shift+np.array(knee["foot"]), .06)),
        "knee_collision": bool(hits_sphere(shift+np.array(knee["knee"]), .025)),
    },
}
result["agrees_with_reduced_geometry"] = (
    result["swing"]["nominal_colliding_samples"] > 0
    and result["swing"]["searched_colliding_samples"] == 0
    and result["orientation"] == {"yaw0_collision": False, "yaw30_collision": True}
    and result["knee"] == {"torso_collision": False, "wheel_collision": False, "knee_collision": True})
stage.GetRootLayer().Export(str(OUT/"diagnostic_scene.usda"))
(OUT/"isaac_results.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
print("ISAAC_COLLISION_RESULT=" + json.dumps(result), flush=True)
world.stop()
app.close()

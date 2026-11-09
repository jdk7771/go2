"""Frozen experiment settings and default paths; change explicitly for new runs."""

from pathlib import Path
from .core import PlannerWeights

HERE = Path(__file__).resolve().parent
DEFAULT_RUN = HERE / "outputs" / "unified_init"
MODES = ("translation", "translation_yaw", "full")
LABELS = {"translation": "XYZ", "translation_yaw": "XYZ + yaw", "full": "Full 6D"}
COLORS = dict(zip(MODES, ("tab:blue", "tab:orange", "tab:green")))
KEY_SCENES = (
    "beam_h430_yaw00_tilt+0",
    "tilted_corridor_30deg",
    "asymmetric_overhang_slalom",
)
NODES = 23
TARGET_CLEARANCE = 0.020
PLANNING_CLEARANCE = 0.047
POINT_VOXEL = 0.020
SOLVER_OPTIONS = {"ftol": 2.0e-9, "gtol": 1.0e-4, "maxls": 60}


def planner_weights() -> PlannerWeights:
    return PlannerWeights(
        collision_stiffness=1.6e6,
        path_xy=18.0,
        height=32.0,
        attitude=0.30,
        velocity_translation=8.0,
        acceleration_translation=110.0,
        velocity_rotation=1.5,
        acceleration_rotation=20.0,
    )

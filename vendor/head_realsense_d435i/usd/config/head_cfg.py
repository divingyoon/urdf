"""Isaac Lab asset configuration for the openarm D435i pan/tilt head.

The head is a 2-DOF ARTICULATION (not a single rigid body): two XC330 servos
drive `joint_pan` (Z) and `joint_tilt` (Y). Everything else is merged into three
rigid links (base / mid / camera). The USD already contains the links, masses,
inertias, convex-decomposition collision, and the two revolute drives, so this
Cfg mostly points at the file and declares the actuators.

Usage:
    from head_cfg import HEAD_CFG
    head = Articulation(HEAD_CFG.replace(prim_path="/World/Head"))

Import paths target Isaac Lab (isaaclab.*, formerly omni.isaac.lab.*).
"""
import os

try:
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
except ImportError:  # older Isaac Lab / Orbit
    import omni.isaac.lab.sim as sim_utils
    from omni.isaac.lab.actuators import ImplicitActuatorCfg
    from omni.isaac.lab.assets import ArticulationCfg

_HERE = os.path.dirname(os.path.abspath(__file__))
_USD_DIR = os.path.abspath(os.path.join(_HERE, ".."))

HEAD_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Head",
    spawn=sim_utils.UsdFileCfg(
        usd_path=os.path.join(_USD_DIR, "head.usda"),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=1,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            fix_root_link=True,   # base welded to world; set False if mounted on a moving arm
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        joint_pos={"joint_pan": 0.0, "joint_tilt": 0.0},
    ),
    actuators={
        # PLACEHOLDER gains — tune for your XC330 controller.
        "head_servos": ImplicitActuatorCfg(
            joint_names_expr=["joint_pan", "joint_tilt"],
            effort_limit=0.9,          # N*m (XC330-M288 ~ 0.93 stall)
            velocity_limit=6.0,        # rad/s
            stiffness=5.0,
            damping=0.5,
        ),
    },
)

# Convenience: joint order and default limits (radians) for reference.
JOINT_NAMES = ["joint_pan", "joint_tilt"]
JOINT_LIMITS_RAD = {
    "joint_pan": (-1.5708, 1.5708),     # +-90 deg (placeholder)
    "joint_tilt": (-1.570796, 1.570796),  # +-90 deg (from CAD, Revolute 10)
}

#!/usr/bin/env python3
"""
Isaac Sim standalone example: load the 2-DOF openarm D435i head articulation
(head.usda) and sweep the PAN and TILT joints so you can see the camera move.

Run inside an Isaac Sim python environment, e.g.:
    <isaac>/python.sh load_in_isaac.py            # Linux
    <isaac>\\python.bat load_in_isaac.py          # Windows

API note: written for omni.isaac.core (Isaac Sim 2023.1 - 4.x). On Isaac Sim 4.5+
the same classes live under `isaacsim.core.*`. Adjust imports if your version moved them.

USD vs URDF:
  * This script loads the prebuilt USD (usd/head.usda) — links, masses, collision,
    and the two revolute drives are already baked in.
  * To use the URDF instead: run the Isaac URDF Importer on urdf/head.urdf
    (Fix Base = true if the head is standalone; Merge Fixed Joints = on), then
    point HEAD_USD at the imported .usd.
"""
import os
import numpy as np
from isaacsim import SimulationApp

sim_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import GroundPlane
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation

HERE = os.path.dirname(os.path.abspath(__file__))
HEAD_USD = os.path.join(HERE, "usd", "head.usda")
HEAD_PRIM = "/World/Head"

# joint sweep amplitudes (radians)
PAN_AMP = np.deg2rad(60.0)
TILT_AMP = np.deg2rad(40.0)


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add(GroundPlane(prim_path="/World/ground", z_position=-0.05))

    # bring in the physics-complete head articulation
    add_reference_to_stage(usd_path=HEAD_USD, prim_path=HEAD_PRIM)
    head = world.scene.add(Articulation(prim_path=HEAD_PRIM, name="head"))

    world.reset()
    world.set_simulation_dt(physics_dt=1.0 / 120.0, rendering_dt=1.0 / 60.0)

    # resolve joint indices by name (order can vary after import)
    dof_names = head.dof_names
    print("DOF names:", dof_names)
    pan_i = dof_names.index("joint_pan")
    tilt_i = dof_names.index("joint_tilt")

    t = 0.0
    for step in range(4000):
        t += 1.0 / 120.0
        targets = head.get_joint_positions()
        targets[pan_i] = PAN_AMP * np.sin(2.0 * np.pi * 0.25 * t)     # 0.25 Hz pan
        targets[tilt_i] = TILT_AMP * np.sin(2.0 * np.pi * 0.4 * t)    # 0.40 Hz tilt
        head.set_joint_position_targets(targets)
        world.step(render=True)

    sim_app.close()


if __name__ == "__main__":
    main()

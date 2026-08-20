#!/usr/bin/env python3
"""Zero-action drift probe for one built RL USD.

Verifies that the phantom-motion symptom (robot moving with no action because
of resting self-collision contacts) is gone: the asset is spawned alone,
gravity disabled, position targets held at the initial pose. Any drift is
caused purely by internal contact forces.

One robot per process ON PURPOSE: spawning two articulations from the same
instanceable USD with different articulation flags in one stage aliases their
PhysX shapes (measured 1e11 N ghost contacts across a 3m gap), which is not a
training configuration - the cloner replicates identical flags.

Run per asset/mode (see the loop in the docstring of tools/build_usd.py):

    /home/user/rl_ws/IsaacLab/isaaclab.sh -p tools/probe_zero_action.py \
        --asset openarm_tesollo_bi_s_rl --self-collision on [--steps 200]

Prints one line:  RESULT <asset> selfcol=<on|off> max_drift=<rad> joint=<name>
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--asset", required=True, help="RL asset name, e.g. openarm_tesollo_bi_s_rl")
parser.add_argument("--self-collision", choices=["on", "off"], default="on")
parser.add_argument("--steps", type=int, default=200, help="Simulation steps (dt 1/120s).")
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(headless=True)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import sys  # noqa: E402
from pathlib import Path  # noqa: E402

import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
RL_DIR = ROOT / "generated" / "rl"

# The historical symptom pushed joints far past limits (up to ~5 rad); this
# threshold separates that from PD/solver settle noise.
DRIFT_LIMIT_RAD = 0.01


def main() -> int:
    usd_path = RL_DIR / args_cli.asset / f"{args_cli.asset}.usd"
    if not usd_path.is_file():
        raise SystemExit(f"missing USD (run tools/build_usd.py first): {usd_path}")

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1.0 / 120.0, gravity=(0.0, 0.0, 0.0))
    )
    robot = Articulation(ArticulationCfg(
        prim_path="/World/robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(usd_path),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=args_cli.self_collision == "on",
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=1,
            ),
        ),
        actuators={
            # stiffness/damping None -> keep the USD drive gains (uniform PD)
            "all": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=None, damping=None),
        },
    ))
    sim.reset()
    robot.update(sim.get_physics_dt())
    initial = robot.data.joint_pos.clone()
    drift = torch.zeros_like(initial)

    for _ in range(args_cli.steps):
        robot.set_joint_position_target(initial)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())
        drift = torch.maximum(drift, (robot.data.joint_pos - initial).abs())

    worst = drift.max().item()
    joint = robot.joint_names[int(drift.argmax().item()) % len(robot.joint_names)]
    verdict = "PHANTOM-MOTION" if worst > DRIFT_LIMIT_RAD else "ok"
    print(f"RESULT {args_cli.asset} selfcol={args_cli.self_collision} "
          f"max_drift={worst:.5f} joint={joint} {verdict}", flush=True)
    return 0 if verdict == "ok" else 1


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    sys.exit(code)

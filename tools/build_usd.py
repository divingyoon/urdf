#!/usr/bin/env python3
"""Headless URDF -> USD build for the generated RL assets.

Replaces the manual Isaac Sim GUI import. The import settings are pinned in
code so they can never drift from what the asset pipeline assumes:

- collider_type "convex_decomposition": required by the self-collision audit
  (tools/audit_self_collision.py) - a plain convex hull fills concave pockets
  (e.g. the palm's thumb pocket) and fabricates resting penetrations. The
  manifest records this as `requires_collision_approximation`.
- merge_fixed_joints False: the historical GUI imports did NOT merge. Measured
  on the previous openarm_tesollo_bi_s_rl USD: 77 rigid bodies including every
  `*_hl_palm` and `*_tip`. Merging drops 20 bodies, and hdgp addresses them by
  name - `palm_body` (the pose every fabric/IK/reward term is built on) and the
  fingertip contact sensors. With merge on the env dies at boot:
  "Sensor at path '.../r_hl_thumb_tip' could not find any bodies with contact
  reporter API". The body contract is verified below so this cannot regress
  silently again.
- fix_base True, import-time self_collision False (the training cfg decides
  `enabled_self_collisions` at runtime).
- joint drive: position targets with placeholder PD gains - hdgp actuator
  configs overwrite gains at spawn time.

The importer emits the same layered structure the GUI produced (top-level
`<asset>.usd` + `configuration/{base,physics,robot,sensor}.usd`), so the whole
asset directory is the artifact. After conversion the build verifies against
the manifest: every joint in `control_joint_order` must exist in the USD, and
every mesh collider must carry the required collision approximation.

Run (Isaac environment required):

    /home/user/rl_ws/IsaacLab/isaaclab.sh -p tools/build_usd.py [asset...] \
        [--sync-hdgp]

Note: apps/isaaclab.python.kit pins isaacsim.asset.importer.urdf (2.4.31);
changing that pin can change conversion output - keep it in mind when
comparing against GUI-imported USDs.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("names", nargs="*",
                    help="RL asset names, e.g. openarm_tesollo_bi_s_rl (default: all).")
parser.add_argument("--sync-hdgp", action="store_true",
                    help="Copy the USD layer stack and manifest into hdgp/assets/robot/<asset>/.")
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(headless=True)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import shutil  # noqa: E402
import sys  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402
from pathlib import Path  # noqa: E402

import yaml  # noqa: E402
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
RL_DIR = ROOT / "generated" / "rl"
HDGP_ROBOT_DIR = ROOT.parent / "hdgp" / "assets" / "robot"

# Placeholder drive gains (hdgp actuator cfgs overwrite them at spawn).
DRIVE_STIFFNESS = 100.0
DRIVE_DAMPING = 1.0

# Collider approximation: convexDecomposition on EVERY mesh collider.
# A convexHull build was attempted for spawn speed (GUI-era assets were 100%
# hull) and REVERTED: PhysX GPU limits convex hulls to 64 vertices, and the
# 64-vertex circumscribed hull of a large mesh inflates tens of millimetres
# past the exact hull - measured ghost contacts across a 9.6mm (l_al_5/l_al_7,
# 427kN) and even a 36mm gap (body/gripper finger). That is incompatible with
# enabled_self_collisions=True and cannot be audited offline. The GUI-era hull
# assets only worked because self-collision was always off. Slow first boot is
# decomposition cooking; it is cached per machine afterwards.


def convert(asset: str) -> Path:
    urdf_path = RL_DIR / f"{asset}.urdf"
    manifest_path = RL_DIR / f"{asset}_manifest.yaml"
    if not urdf_path.is_file() or not manifest_path.is_file():
        raise SystemExit(f"missing URDF or manifest for {asset}")
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    collider_type = manifest.get("requires_collision_approximation", "convex_decomposition")

    out_dir = RL_DIR / asset
    converter_cfg = UrdfConverterCfg(
        asset_path=str(urdf_path),
        usd_dir=str(out_dir),
        usd_file_name=f"{asset}.usd",
        force_usd_conversion=True,
        fix_base=True,
        merge_fixed_joints=False,
        self_collision=False,
        collider_type=collider_type,
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            target_type="position",
            gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
                stiffness=DRIVE_STIFFNESS, damping=DRIVE_DAMPING
            ),
        ),
    )
    if "self_collision_filtered_pairs" not in manifest:
        raise SystemExit(
            f"[{asset}] manifest has no self_collision_filtered_pairs - "
            "regenerate with `python3 tools/generate_rl_urdf.py` (audit enabled)"
        )
    usd_path = Path(UrdfConverter(converter_cfg).usd_path)
    verify_contract(usd_path, manifest, asset, urdf_path)
    apply_collision_filters(usd_path, urdf_path, asset,
                            [tuple(p) for p in manifest["self_collision_filtered_pairs"]])
    patch_visuals_prims(out_dir, asset, urdf_path)
    # make generated/rl/<asset>/ a self-contained bundle: usd layers + the
    # exact urdf/manifest the usd was built from
    for source in (urdf_path, manifest_path):
        shutil.copyfile(source, out_dir / source.name)
    return usd_path


def patch_visuals_prims(out_dir: Path, asset: str, urdf_path: Path) -> None:
    """Define empty /visuals/<link> prims the importer forgot.

    base.usd references /visuals/<link> in the physics/base layers for every
    link, but links without visual geometry (massless frames like *_hl_mount,
    palm_alias, head_cam_view) never get one - every stage open then spams
    'Unresolved reference' warnings (the old GUI pipeline had the same defect,
    patched ad hoc by IsaacLab scripts/tools/fix_openarm_physics_usd.py).
    """
    link_names = [link.get("name") or "" for link in ET.parse(urdf_path).getroot().findall("link")]
    for layer_name in (f"{asset}_physics.usd", f"{asset}_base.usd"):
        layer_path = out_dir / "configuration" / layer_name
        stage = Usd.Stage.Open(str(layer_path))
        patched = 0
        for link in link_names:
            path = f"/visuals/{link}"
            if not stage.GetPrimAtPath(path).IsValid():
                stage.DefinePrim(path, "Xform")
                patched += 1
        if patched:
            stage.GetRootLayer().Save()
            print(f"[{asset}] patched {patched} missing /visuals prims in {layer_name}")


def merged_body_resolver(urdf_path: Path, bodies: set[str]):
    """Map a URDF link to the USD rigid body it merged into (fixed-joint walk)."""
    fixed_parent: dict[str, str] = {}
    for joint in ET.parse(urdf_path).getroot().findall("joint"):
        if joint.get("type") == "fixed":
            parent, child = joint.find("parent"), joint.find("child")
            assert parent is not None and child is not None
            fixed_parent[child.get("link") or ""] = parent.get("link") or ""

    def resolve(link: str) -> str | None:
        current = link
        while current not in bodies:
            if current not in fixed_parent:
                return None
            current = fixed_parent[current]
        return current

    return resolve


def apply_collision_filters(usd_path: Path, urdf_path: Path, asset: str,
                            pairs: list[tuple[str, str]]) -> None:
    """Author PhysicsFilteredPairs for the manifest's audited pairs.

    The audit exports WARN pairs (documented in self_collision_allowlist.yaml)
    whose raw clearance is within convex-decomposition cooking inflation, plus
    unverified/nested vendor geometry, into the manifest. Those pairs must
    never generate self-collision contacts; everything else keeps full
    self-collision.
    """
    stage = Usd.Stage.Open(str(usd_path))
    root_path = stage.GetDefaultPrim().GetPath()
    bodies = {
        prim.GetName()
        for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    }
    resolve = merged_body_resolver(urdf_path, bodies)

    filtered = set()
    for link_a, link_b in pairs:
        body_a, body_b = resolve(link_a), resolve(link_b)
        if body_a is None or body_b is None:
            raise SystemExit(f"[{asset}] cannot map filtered pair to bodies: {link_a} <-> {link_b}")
        if body_a == body_b or frozenset((body_a, body_b)) in filtered:
            continue  # merged into one body, or already filtered
        filtered.add(frozenset((body_a, body_b)))
        prim = stage.GetPrimAtPath(root_path.AppendChild(body_a))
        assert prim.IsValid(), body_a
        api = UsdPhysics.FilteredPairsAPI.Apply(prim)
        api.GetFilteredPairsRel().AddTarget(root_path.AppendChild(body_b))
    stage.GetRootLayer().Save()
    print(f"[{asset}] collision filters authored: {len(filtered)} body pairs "
          f"(from {len(pairs)} audited link pairs)")


def verify_contract(usd_path: Path, manifest: dict, asset: str,
                    urdf_path: Path | None = None) -> None:
    """Manifest joints and every inertial link must exist; colliders must match collider_type.

    ★The body check is not decorative. hdgp addresses bodies **by name** -
    `palm_body` carries the pose that every fabric/IK/reward term is built on,
    and the fingertip contact sensors point at `*_tip`. A build that merges
    fixed joints silently drops 20 of them (77 -> 57 on bi_s) and this function
    still printed "contract ok", because it only ever looked at joints.
    """
    stage = Usd.Stage.Open(str(usd_path))
    usd_joints = set()
    usd_bodies = set()
    wrong_approximations = []
    counts: dict[str, int] = {}
    # instance proxies included: collision meshes live inside instanceable prims
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
            usd_joints.add(prim.GetName())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            usd_bodies.add(prim.GetName())
        if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            got = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
            counts[got] = counts.get(got, 0) + 1
            if got != "convexDecomposition":
                wrong_approximations.append((prim.GetPath().pathString, got, "convexDecomposition"))

    if urdf_path is not None:
        # Every link with an <inertial> is a body downstream code may address.
        want_bodies = {link.get("name")
                       for link in ET.parse(urdf_path).getroot().findall("link")
                       if link.find("inertial") is not None}
        lost = sorted(want_bodies - usd_bodies)
        if lost:
            raise SystemExit(
                f"[{asset}] USD body contract FAILED - {len(lost)} inertial links are not "
                f"rigid bodies (fixed-joint merging?): {', '.join(lost[:8])}"
                f"{' ...' if len(lost) > 8 else ''}"
            )

    expected = set(manifest["control_joint_order"])
    missing = sorted(expected - usd_joints)
    if missing:
        raise SystemExit(
            f"[{asset}] USD joint contract FAILED - {len(missing)} control joints missing: "
            f"{', '.join(missing[:8])}{' ...' if len(missing) > 8 else ''}"
        )
    if wrong_approximations:
        first = wrong_approximations[0]
        raise SystemExit(
            f"[{asset}] collider approximation FAILED for {len(wrong_approximations)} "
            f"colliders, e.g. {first[0]}: got {first[1]}, want {first[2]}"
        )
    extra = sorted(usd_joints - expected)
    print(f"[{asset}] contract ok: {len(expected)} control joints, "
          f"{len(usd_bodies)} rigid bodies, colliders {counts}"
          + (f"; extra articulated joints: {', '.join(extra)}" if extra else ""))


def sync_hdgp(asset: str, usd_path: Path) -> None:
    """Mirror the USD layer stack (+manifest) into hdgp's asset copy."""
    source_dir = usd_path.parent
    destination = HDGP_ROBOT_DIR / asset
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "configuration").mkdir(exist_ok=True)
    targets = [usd_path, RL_DIR / f"{asset}_manifest.yaml", RL_DIR / f"{asset}.urdf"]
    targets += sorted((source_dir / "configuration").glob("*.usd"))
    for source in targets:
        relative = source.relative_to(source_dir) if source.is_relative_to(source_dir) else Path(source.name)
        target = destination / relative
        shutil.copyfile(source, target)
        print(f"[{asset}] synced -> {target}")


def main() -> int:
    if args_cli.names:
        assets = args_cli.names
    else:
        assets = sorted(p.stem for p in RL_DIR.glob("*_rl.urdf"))
    for asset in assets:
        usd_path = convert(asset)
        print(f"[{asset}] wrote {usd_path}")
        if args_cli.sync_hdgp:
            sync_hdgp(asset, usd_path)
    return 0


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    sys.exit(code)

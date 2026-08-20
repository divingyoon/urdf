#!/usr/bin/env python3
"""Generate the Fabrics IK URDFs from the generated RL URDFs.

Fabrics (hdgp/source/FABRICS) loads its own URDF per robot variant whose
link/joint names, palm helper frames (palm ±0.25m axis points), and collision
sphere frames are hard-coded conventions of the fabric code and its
fabric_params YAMLs. Historically these URDFs were hand-regenerated inside
hdgp with four separate scripts every time an RL asset changed; this module
consolidates them so `generated/rl/*_rl.urdf` is the single kinematic source.

Variants (directory name == file name == robot name, the Fabrics convention):

- openarm_tesollo_bi_s            right arm+hand of openarm_tesollo_bi_s_rl
- openarm_tesollo_bi_s_left       left  arm+hand of openarm_tesollo_bi_s_rl
- openarm_tesollo_sensor_left_gripper
                                  left arm of openarm_tesollo_sensor_rl with
                                  the hand frozen (cspace = 7); palm = gripper
                                  TCP; frozen frames approximate the gripper
                                  volume for collision spheres
- openarm_rh56f1                  both arms of openarm_bi_rh56f1_rl
                                  (arm 7 + drive 6 per side, mimic -> fixed)

The tesollo/gripper variants patch structural templates vendored in
eef/fabric_templates/ (fabric-only frames and sphere layouts live there);
every kinematic quantity - arm joints (including the composite world->link1
transform), hand joints, palm offset, fingertip offsets - is overwritten from
the RL URDF, so the historical +8mm arm-base offset of the legacy fabric
URDFs is gone. Each variant is FK-verified against its RL URDF before it is
written; a variant that fails verification produces no output.

Run:  python3 tools/gen_fabric_urdfs.py [variant...] [--sync-hdgp]
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RL_DIR = ROOT / "generated" / "rl"
TEMPLATE_DIR = ROOT / "eef" / "fabric_templates"
OUT_DIR = ROOT / "generated" / "fabric"
HDGP_FABRIC_DIR = ROOT.parent / "hdgp" / "source" / "FABRICS" / "src" / "fabrics_sim" / "models" / "robots" / "urdf"

FINGERS = ("thumb", "index", "middle", "ring", "pinky")
FK_TOLERANCE_M = 2e-4
FK_TRIALS = 20

# palm helper frames are fabric-code conventions (±0.25m axis points), not
# geometry - never derived from the RL URDF (see hdgp generate_left_fabric_urdf).
PALM_HELPER_JOINTS = {
    "palm_x_joint", "palm_x_neg_joint",
    "palm_y_joint", "palm_y_neg_joint",
    "palm_z_joint", "palm_z_neg_joint",
}


# ---------------------------------------------------------------------------
# URDF parsing / FK (ported from hdgp assets_tools, FK-proven there)
# ---------------------------------------------------------------------------
def rpy_to_mat(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rot_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    rot_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rot_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rot_z @ rot_y @ rot_x


def mat_to_rpy(rot: np.ndarray) -> tuple[float, float, float]:
    p = math.asin(max(-1.0, min(1.0, -rot[2, 0])))
    if abs(math.cos(p)) > 1e-9:
        return math.atan2(rot[2, 1], rot[2, 2]), p, math.atan2(rot[1, 0], rot[0, 0])
    return math.atan2(-rot[1, 2], rot[1, 1]), p, 0.0


def axis_angle(axis: np.ndarray, q: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * (k @ k)


def make_transform(xyz: np.ndarray, rot: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rot
    transform[:3, 3] = xyz
    return transform


def parse_urdf(path: Path) -> dict[str, dict]:
    joints: dict[str, dict] = {}
    for joint in ET.parse(path).getroot().iter("joint"):
        origin = joint.find("origin")
        axis_el = joint.find("axis")
        limit = joint.find("limit")
        parent, child = joint.find("parent"), joint.find("child")
        assert parent is not None and child is not None
        joints[joint.get("name") or ""] = {
            "type": joint.get("type"),
            "parent": parent.get("link"),
            "child": child.get("link"),
            "xyz": np.array([float(v) for v in ((origin.get("xyz") if origin is not None else None) or "0 0 0").split()]),
            "rpy": np.array([float(v) for v in ((origin.get("rpy") if origin is not None else None) or "0 0 0").split()]),
            "axis": (np.array([float(v) for v in (axis_el.get("xyz") or "").split()]) if axis_el is not None else None),
            "limits": ((float(limit.get("lower") or 0), float(limit.get("upper") or 0)) if limit is not None else None),
        }
    return joints


def fk_link(joints: dict[str, dict], link: str, q: dict[str, float]) -> np.ndarray:
    """Accumulate the parent chain of ``link`` up to the root."""
    by_child = {j["child"]: (name, j) for name, j in joints.items()}
    chain = []
    current = link
    while current in by_child:
        chain.append(by_child[current])
        current = by_child[current][1]["parent"]
    transform = np.eye(4)
    for name, joint in reversed(chain):
        transform = transform @ make_transform(joint["xyz"], rpy_to_mat(*joint["rpy"]))
        if joint["type"] == "revolute":
            transform = transform @ make_transform(np.zeros(3), axis_angle(joint["axis"], q.get(name, 0.0)))
    return transform


# ---------------------------------------------------------------------------
# template editing helpers
# ---------------------------------------------------------------------------
def fmt(values) -> str:
    return " ".join(f"{v:.9g}" if abs(v) > 1e-12 else "0" for v in values)


def set_origin(joint: ET.Element, xyz, rpy) -> None:
    origin = joint.find("origin")
    if origin is None:
        origin = ET.SubElement(joint, "origin")
    origin.set("xyz", fmt(xyz))
    origin.set("rpy", fmt(rpy))


def set_axis_limits(joint: ET.Element, axis, limits) -> None:
    axis_el = joint.find("axis")
    assert axis_el is not None, joint.get("name")
    axis_el.set("xyz", fmt(axis))
    limit = joint.find("limit")
    assert limit is not None, joint.get("name")
    limit.set("lower", f"{limits[0]:.9g}")
    limit.set("upper", f"{limits[1]:.9g}")


def joints_by_name(root: ET.Element) -> dict[str, ET.Element]:
    return {j.get("name") or "": j for j in root.iter("joint")}


def patch_arm(joints: dict[str, ET.Element], rl: dict[str, dict], side: str) -> None:
    """Overwrite the template arm chain with the RL URDF's real values.

    joint1's origin is the composite body_root -> {side}_al_1 transform at
    q=0 (the fabric URDF has no body chain); joints 2..7 copy directly.
    """
    base = fk_link(rl, f"{side}_al_1", {})
    for i in range(1, 8):
        source = rl[f"{side}_aj_{i}"]
        target = joints[f"openarm_right_joint{i}"]
        if i == 1:
            set_origin(target, base[:3, 3], mat_to_rpy(base[:3, :3]))
        else:
            set_origin(target, source["xyz"], source["rpy"])
        set_axis_limits(target, source["axis"], source["limits"])


def patch_hand(joints: dict[str, ET.Element], rl: dict[str, dict], side: str) -> None:
    for index, finger in enumerate(FINGERS, start=1):
        for segment in range(1, 5):
            source = rl[f"{side}_hj_{finger}_{segment}"]
            target = joints[f"rj_dg_{index}_{segment}"]
            set_origin(target, source["xyz"], source["rpy"])
            set_axis_limits(target, source["axis"], source["limits"])
        tip = rl[f"{side}_hj_{finger}_tip"]
        set_origin(joints[f"rl_dg_{index}_tip_joint"], tip["xyz"], tip["rpy"])


def patch_palm(joints: dict[str, ET.Element], rl: dict[str, dict], side: str) -> None:
    """palm_link z = wrist->palm chain of the RL URDF (yaw-only chain, z sums).

    x/y keep the template values (fabric mount convention)."""
    chain = [f"{side}_hj_mount", f"{side}_hj_adapter", f"{side}_hj_base", f"{side}_hj_palm"]
    z_new = float(sum(rl[c]["xyz"][2] for c in chain))
    origin = joints["palm_link_joint"].find("origin")
    assert origin is not None
    xyz = [float(v) for v in (origin.get("xyz") or "0 0 0").split()]
    origin.set("xyz", fmt((xyz[0], xyz[1], z_new)))  # rpy keeps the template mount yaw


def write_variant(root: ET.Element, name: str, source_note: str) -> Path:
    root.set("name", name)
    out_dir = OUT_DIR / name
    out_dir.mkdir(parents=True, exist_ok=True)
    header = (
        f"<!-- GENERATED by tools/gen_fabric_urdfs.py from {source_note} - do not edit. -->\n"
        "<!-- Link/joint names, palm helper frames and sphere frames are fabric-code\n"
        "     conventions (fabric_params YAML frame lists); kinematics come from the\n"
        "     RL URDF and are FK-verified against it. -->\n"
    )
    path = out_dir / f"{name}.urdf"
    path.write_text('<?xml version="1.0" ?>\n' + header + ET.tostring(root, encoding="unicode"),
                    encoding="utf-8")
    return path


def write_manifest(name: str, source_rl: Path, urdf_path: Path) -> Path:
    joints = parse_urdf(urdf_path)
    cspace = [n for n, j in joints.items() if j["type"] == "revolute"]
    lines = [
        f"robot_name: {name}\n",
        f"source_rl_urdf: {source_rl.relative_to(ROOT)}\n",
        f"generated_urdf: {urdf_path.relative_to(ROOT)}\n",
        "palm_frame: palm_link\n" if name != "openarm_rh56f1" else "palm_frames: [r_hl_palm_sensor, l_hl_palm_sensor]\n",
        f"cspace_dim: {len(cspace)}\n",
        "cspace_joint_order:  # URDF document order == fabric cspace order\n",
    ]
    lines += [f"- {n}\n" for n in cspace]
    path = urdf_path.parent / f"{name}_manifest.yaml"
    path.write_text("".join(lines), encoding="utf-8")
    return path


# ---------------------------------------------------------------------------
# FK gates (a variant that fails produces no output)
# ---------------------------------------------------------------------------
def _fk_compare(fabric: dict, rl: dict, joint_map: dict[str, str],
                frame_pairs: list[tuple[str, str]], seed: int) -> float:
    rng = np.random.default_rng(seed)
    worst, comparisons = 0.0, 0
    for _ in range(FK_TRIALS):
        q_fabric, q_rl = {}, {}
        for fabric_name, rl_name in joint_map.items():
            low, high = rl[rl_name]["limits"]
            value = float(rng.uniform(low, high))
            q_fabric[fabric_name] = value
            q_rl[rl_name] = value
        for fabric_link, rl_link in frame_pairs:
            fabric_pos = fk_link(fabric, fabric_link, q_fabric)[:3, 3]
            rl_pos = fk_link(rl, rl_link, q_rl)[:3, 3]
            worst = max(worst, float(np.linalg.norm(fabric_pos - rl_pos)))
            comparisons += 1
    if comparisons == 0:
        # zero comparisons would read as "zero error" - a false pass
        raise SystemExit("FK gate compared nothing - frame names do not match")
    return worst


def tesollo_joint_map(side: str) -> dict[str, str]:
    mapping = {f"openarm_right_joint{i}": f"{side}_aj_{i}" for i in range(1, 8)}
    for index, finger in enumerate(FINGERS, start=1):
        for segment in range(1, 5):
            mapping[f"rj_dg_{index}_{segment}"] = f"{side}_hj_{finger}_{segment}"
    return mapping


def verify_tesollo(urdf_path: Path, rl: dict, side: str) -> float:
    frame_pairs = [("palm_link", f"{side}_hl_palm_alias")]
    frame_pairs += [(f"rl_dg_{i}_tip", f"{side}_hl_{finger}_tip")
                    for i, finger in enumerate(FINGERS, start=1)]
    return _fk_compare(parse_urdf(urdf_path), rl, tesollo_joint_map(side), frame_pairs, seed=7)


def verify_gripper(urdf_path: Path, rl: dict) -> float:
    joint_map = {f"openarm_right_joint{i}": f"l_aj_{i}" for i in range(1, 8)}
    return _fk_compare(parse_urdf(urdf_path), rl, joint_map,
                       [("palm_link", "l_hl_gripper_tcp")], seed=11)


def verify_rh56f1(urdf_path: Path, rl: dict) -> float:
    fabric = parse_urdf(urdf_path)
    joint_map = {n: n for n, j in fabric.items() if j["type"] == "revolute"}
    frame_pairs = []
    for side in ("r", "l"):
        frame_pairs.append((f"{side}_hl_palm_sensor", f"{side}_hl_palm_sensor"))
        frame_pairs += [(f"{side}_hl_{finger}_tip", f"{side}_hl_{finger}_tip") for finger in FINGERS]
    return _fk_compare(fabric, rl, joint_map, frame_pairs, seed=13)


# ---------------------------------------------------------------------------
# variant builders
# ---------------------------------------------------------------------------
def build_tesollo(name: str, template: str, rl_asset: str, side: str) -> Path:
    rl_path = RL_DIR / f"{rl_asset}.urdf"
    rl = parse_urdf(rl_path)
    tree = ET.parse(TEMPLATE_DIR / f"{template}.urdf")
    joints = joints_by_name(tree.getroot())
    patch_arm(joints, rl, side)
    patch_hand(joints, rl, side)
    patch_palm(joints, rl, side)
    for helper in PALM_HELPER_JOINTS:  # convention frames must exist untouched
        assert helper in joints, helper
    urdf_path = write_variant(tree.getroot(), name, f"{rl_asset}.urdf ({side} chain)")
    error = verify_tesollo(urdf_path, rl, side)
    if error > FK_TOLERANCE_M:
        urdf_path.unlink()
        raise SystemExit(f"[{name}] FK gate FAILED: {error * 1000:.3f}mm > {FK_TOLERANCE_M * 1000}mm")
    print(f"[{name}] FK gate ok ({error * 1e6:.1f}um, palm+5tips x{FK_TRIALS})")
    write_manifest(name, rl_path, urdf_path)
    return urdf_path


def build_gripper(name: str) -> Path:
    rl_path = RL_DIR / "openarm_tesollo_sensor_rl.urdf"
    rl = parse_urdf(rl_path)
    tree = ET.parse(TEMPLATE_DIR / f"{name}.urdf")
    joints = joints_by_name(tree.getroot())
    patch_arm(joints, rl, "l")
    # palm_link = gripper TCP: mount + tcp offsets from the RL URDF, no rotation
    z_tcp = float(rl["l_hj_gripper_mount"]["xyz"][2] + rl["l_hj_gripper_tcp"]["xyz"][2])
    set_origin(joints["palm_link_joint"], (0.0, 0.0, z_tcp), (0.0, 0.0, 0.0))
    # frozen hand frames (gripper-volume approximation) stay as templated
    urdf_path = write_variant(tree.getroot(), name, "openarm_tesollo_sensor_rl.urdf (left arm, hand frozen)")
    error = verify_gripper(urdf_path, rl)
    if error > FK_TOLERANCE_M:
        urdf_path.unlink()
        raise SystemExit(f"[{name}] FK gate FAILED: {error * 1000:.3f}mm")
    print(f"[{name}] FK gate ok ({error * 1e6:.1f}um, gripper TCP x{FK_TRIALS})")
    write_manifest(name, rl_path, urdf_path)
    return urdf_path


# --- openarm_rh56f1: built from scratch (ported from hdgp FABRICS generator) ---
RH_DRIVE = ("thumb_1", "thumb_2", "index_1", "middle_1", "ring_1", "pinky_1")
RH_MIMIC = ("thumb_3", "thumb_4", "index_2", "middle_2", "ring_2", "pinky_2")
RH_TIPS = ("thumb_tip", "index_tip", "middle_tip", "ring_tip", "pinky_tip")
RH_PALM_FIXED = ("mount", "palm_1", "palm_2", "palm_3", "palm_sensor")
RH_PALM_AXIS = {
    "x": "0.25 0 0", "x_neg": "-0.25 0 0",
    "y": "0 0.25 0", "y_neg": "0 -0.25 0",
    "z": "0 0 0.25", "z_neg": "0 0 -0.25",
}
RH_SPHERE_PARENT = ("thumb_2", "index_1", "middle_1", "ring_1", "pinky_1")


def _rh_massless_link(out: ET.Element, name: str) -> None:
    link = ET.SubElement(out, "link", {"name": name})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": "0.001"})
    ET.SubElement(inertial, "inertia", {"ixx": "1e-6", "ixy": "0", "ixz": "0",
                                        "iyy": "1e-6", "iyz": "0", "izz": "1e-6"})


def _rh_copy_joint(out: ET.Element, src: ET.Element, force_type: str) -> None:
    joint = ET.SubElement(out, "joint", {"name": src.get("name") or "", "type": force_type})
    origin = src.find("origin")
    ET.SubElement(joint, "origin", {
        "xyz": (origin.get("xyz") if origin is not None else None) or "0 0 0",
        "rpy": (origin.get("rpy") if origin is not None else None) or "0 0 0",
    })
    parent, child = src.find("parent"), src.find("child")
    assert parent is not None and child is not None
    ET.SubElement(joint, "parent", {"link": parent.get("link") or ""})
    ET.SubElement(joint, "child", {"link": child.get("link") or ""})
    if force_type == "revolute":
        axis = src.find("axis")
        ET.SubElement(joint, "axis", {"xyz": (axis.get("xyz") if axis is not None else None) or "0 0 1"})
        limit = src.find("limit")
        ET.SubElement(joint, "limit", {
            "lower": (limit.get("lower") if limit is not None else None) or "0",
            "upper": (limit.get("upper") if limit is not None else None) or "1.5",
            "effort": "5", "velocity": "2",
        })


def build_rh56f1(name: str = "openarm_rh56f1") -> Path:
    rl_path = RL_DIR / "openarm_bi_rh56f1_rl.urdf"
    src_joints = list(ET.parse(rl_path).getroot().findall("joint"))
    out = ET.Element("robot", {"name": name})
    _rh_massless_link(out, "body_link")

    def child_link(joint: ET.Element) -> str:
        child = joint.find("child")
        assert child is not None
        return child.get("link") or ""

    for side in ("r", "l"):
        prefix = f"{side}_"
        for joint in src_joints:
            joint_name = joint.get("name") or ""
            if joint_name == f"{prefix}aj_base":
                _rh_massless_link(out, child_link(joint))
                _rh_copy_joint(out, joint, "fixed")
            elif joint_name.startswith(f"{prefix}aj_") and joint_name[len(prefix) + 3:].isdigit():
                _rh_massless_link(out, child_link(joint))
                _rh_copy_joint(out, joint, "revolute")
        for joint in src_joints:
            joint_name = joint.get("name") or ""
            if not joint_name.startswith(f"{prefix}hj_"):
                continue
            suffix = joint_name[len(prefix) + 3:]
            if suffix in RH_PALM_FIXED or suffix in RH_MIMIC or suffix in RH_TIPS:
                _rh_massless_link(out, child_link(joint))
                _rh_copy_joint(out, joint, "fixed")
            elif suffix in RH_DRIVE:
                _rh_massless_link(out, child_link(joint))
                _rh_copy_joint(out, joint, "revolute")
            # *_sensor force-sensor frames: not needed for FK -> skipped
        for key, offset in RH_PALM_AXIS.items():
            point = f"ps_{side}_{key}"
            _rh_massless_link(out, point)
            joint = ET.SubElement(out, "joint", {"name": f"{point}_joint", "type": "fixed"})
            ET.SubElement(joint, "origin", {"xyz": offset, "rpy": "0 0 0"})
            ET.SubElement(joint, "parent", {"link": f"{side}_hl_palm_sensor"})
            ET.SubElement(joint, "child", {"link": point})
        for suffix in RH_SPHERE_PARENT:
            sphere = f"{side}_sphere_{suffix.split('_')[0]}"
            _rh_massless_link(out, sphere)
            joint = ET.SubElement(out, "joint", {"name": f"{sphere}_joint", "type": "fixed"})
            ET.SubElement(joint, "origin", {"xyz": "0 0.015 0", "rpy": "0 0 0"})
            ET.SubElement(joint, "parent", {"link": f"{side}_hl_{suffix}"})
            ET.SubElement(joint, "child", {"link": sphere})

    ET.indent(out, space="  ")
    urdf_path = write_variant(out, name, "openarm_bi_rh56f1_rl.urdf (both arms, mimic->fixed)")
    error = verify_rh56f1(urdf_path, parse_urdf(rl_path))
    if error > FK_TOLERANCE_M:
        urdf_path.unlink()
        raise SystemExit(f"[{name}] FK gate FAILED: {error * 1000:.3f}mm")
    print(f"[{name}] FK gate ok ({error * 1e6:.1f}um, palm_sensor+tips both sides x{FK_TRIALS})")
    write_manifest(name, rl_path, urdf_path)
    return urdf_path


VARIANTS = {
    "openarm_tesollo_bi_s": lambda: build_tesollo(
        "openarm_tesollo_bi_s", "openarm_tesollo_bi_s", "openarm_tesollo_bi_s_rl", "r"),
    "openarm_tesollo_bi_s_left": lambda: build_tesollo(
        "openarm_tesollo_bi_s_left", "openarm_tesollo_bi_s_left", "openarm_tesollo_bi_s_rl", "l"),
    "openarm_tesollo_sensor_left_gripper": lambda: build_gripper(
        "openarm_tesollo_sensor_left_gripper"),
    "openarm_rh56f1": build_rh56f1,
}


def sync_hdgp(urdf_path: Path) -> None:
    destination = HDGP_FABRIC_DIR / urdf_path.parent.name
    if not destination.is_dir():
        raise SystemExit(f"hdgp fabric dir missing (naming contract broken?): {destination}")
    for source in (urdf_path, urdf_path.parent / f"{urdf_path.parent.name}_manifest.yaml"):
        (destination / source.name).write_bytes(source.read_bytes())
        print(f"  synced -> {destination / source.name}")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("names", nargs="*", help="Variants to generate (default: all).")
    parser.add_argument("--sync-hdgp", action="store_true",
                        help="Copy outputs into hdgp/source/FABRICS/.../urdf/<name>/.")
    args = parser.parse_args(argv)

    invalid = [n for n in args.names if n not in VARIANTS]
    if invalid:
        parser.error(f"unknown variant(s): {', '.join(invalid)}; choose from {', '.join(VARIANTS)}")
    for name in args.names or list(VARIANTS):
        urdf_path = VARIANTS[name]()
        print(f"generated {urdf_path.relative_to(ROOT)}")
        if args.sync_hdgp:
            sync_hdgp(urdf_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

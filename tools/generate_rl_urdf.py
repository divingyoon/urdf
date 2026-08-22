#!/usr/bin/env python3
"""Generate RL-only canonical URDFs and joint-order manifests.

The source URDFs keep vendor/controller names. These generated URDFs rename
links and joints into a stable RL schema so Isaac/RL code can use one action
and observation convention across different end-effectors.
"""

from __future__ import annotations

import argparse
import copy
import re
import sys
import xml.etree.ElementTree as ET
from collections import OrderedDict
from pathlib import Path
from typing import Iterable

sys.path.insert(0, str(Path(__file__).resolve().parent))
from crop_body_plate import ensure_cropped_meshes  # noqa: E402
from crop_link7_flange import ensure_link7_flange_mesh  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "generated" / "rl"
ASSET_ROOTS = {
    "openarm_description": ROOT / "vendor" / "openarm_description",
    "dg_description": ROOT / "vendor" / "delto_m_ros2" / "dg_description",
    "tesollo_model": ROOT / "vendor" / "tesollo_model",
    "RH56F1": ROOT / "vendor" / "RH56F1",
}

# The vendor body mesh includes an 8mm mount plate below z=0 of the new robot
# origin (origin = plate top). Everything defined in the vendor frame moves
# down by this amount; cropped meshes from crop_body_plate.py are pre-shifted.
BASE_LIFT_M = 0.008

# The stock link7 meshes include the stock-gripper motor section that is
# physically removed when a hand replaces the stock gripper, so RL assets swap
# them for the pre-cropped variants. The hand mount then sits on the exposed
# flange plate plane (z=0.0495, the large flat surface); the flange bolts
# protrude 4mm further (to z=0.0535, the cropped mesh top) and insert into the
# adapter plate holes, so they are NOT the mounting plane.
STOCK_LINK7_MESH_SWAP = {
    "link7.dae": "link7_without_mat2_mat3_components00_03.dae",
    "link7_symp.stl": "link7_without_mat2_mat3_components00_03.stl",
}
ARM_FLANGE_LINKS = {"r_al_7", "l_al_7"}
LINK7_FLANGE_Z = 0.0495

# RH56F1 vendor collision meshes are unrepairably non-watertight (asymmetric
# across sides; e.g. plam_1 euler=-72), so their finger-chain collisions are
# replaced with primitives fitted to the scaled mesh AABB: elongated segments
# become inscribed cylinders (radius from the SMALLEST cross extent - the
# shells nest closely, a circumscribed fit interpenetrates the neighbours),
# pads/tips become boxes. palm_2/palm_3 are watertight and keep their meshes.
# palm_1 is the back cover whose volume palm_2/palm_3 already envelop; a box
# fit swallows the finger roots (>15mm), so its collision is dropped entirely.
# Visual geometry is untouched.
RH_PRIMITIVE_SUFFIXES = tuple(
    [f"thumb_{i}" for i in range(1, 5)]
    + [f"{finger}_{i}" for finger in ("index", "middle", "ring", "pinky") for i in (1, 2)]
    + [f"{finger}_{kind}" for finger in ("thumb", "index", "middle", "ring", "pinky")
       for kind in ("sensor", "tip")]
)
RH_PRIMITIVE_LINKS = {f"{s}_hl_{suffix}" for s in ("r", "l") for suffix in RH_PRIMITIVE_SUFFIXES}
RH_DROP_COLLISION_LINKS = {f"{s}_hl_palm_1" for s in ("r", "l")}
# Above this longest/second-longest extent ratio a cylinder fits better than a box.
RH_CYLINDER_ELONGATION = 1.6

HEAD_DIR = ROOT / "vendor" / "head_realsense_d435i"
HEAD_URDF = HEAD_DIR / "urdf" / "head.urdf"
# Head base mounts on body_link at z=+750mm from the robot origin (mount top).
HEAD_MOUNT_XYZ = "0 0 0.750"
HEAD_LINK_MAP = {
    "base_link": "head_base",
    "mid_link": "head_mid",
    "camera_link": "head_camera",
}
HEAD_JOINT_MAP = {
    "joint_pan": "head_j_pan",
    "joint_tilt": "head_j_tilt",
}
HEAD_LINK_ORDER = ["head_base", "head_mid", "head_camera", "head_cam_view"]
HEAD_JOINT_ORDER = ["head_j_mount", "head_j_pan", "head_j_tilt", "head_j_cam_view"]

# Camera view origin: center of the D435i front glass in the head_camera link
# frame (measured from the vendor collision mesh). +X is the viewing direction,
# +Z up. Tune here if calibration finds a different optical origin; the values
# are exported to each manifest (camera_view_frame) so sim code can attach a
# camera even when USD import merges the fixed frame away.
HEAD_CAM_VIEW_XYZ = (0.0147, 0.0145, 0.0365)
HEAD_CAM_VIEW_RPY = (0.0, 0.0, 0.0)

SOURCES = OrderedDict(
    [
        ("openarm_tesollo_sensor", ROOT / "generated" / "source" / "openarm_tesollo_sensor.urdf"),
        ("openarm_tesollo_bi", ROOT / "generated" / "source" / "openarm_tesollo_bi.urdf"),
        ("openarm_tesollo_bi_s", ROOT / "generated" / "source" / "openarm_tesollo_bi_s.urdf"),
        ("openarm_bi_rh56f1", ROOT / "generated" / "source" / "openarm_bi_rh56f1.urdf"),
    ]
)

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]
TESOLLO_FINGER_BY_INDEX = {
    "1": "thumb",
    "2": "index",
    "3": "middle",
    "4": "ring",
    "5": "pinky",
}
RH_FINGERS = {
    "thumb": "thumb",
    "index": "index",
    "middle": "middle",
    "ring": "ring",
    "little": "pinky",
}
MOVABLE_TYPES = {"revolute", "continuous", "prismatic"}


def side_prefix(side: str) -> str:
    return "r" if side == "right" else "l"


def add(mapping: dict[str, str], old: str, new: str) -> None:
    if old in mapping and mapping[old] != new:
        raise ValueError(f"conflicting mapping for {old}: {mapping[old]} vs {new}")
    mapping[old] = new


def normalize_mesh_uri(uri: str) -> str:
    """Convert package:// and legacy absolute mesh references into file:// URIs."""
    legacy_root_map = {
        ROOT / "openarm_description": ROOT / "vendor" / "openarm_description",
        ROOT / "RH56F1": ROOT / "vendor" / "RH56F1",
    }

    package_prefix = "package://"
    if not uri.startswith(package_prefix):
        file_prefix = "file://"
        if uri.startswith(file_prefix):
            path = Path(uri[len(file_prefix) :])
            for old_root, new_root in legacy_root_map.items():
                try:
                    relpath = path.relative_to(old_root)
                except ValueError:
                    continue
                return f"file://{(new_root / relpath).resolve()}"
        return uri

    remainder = uri[len(package_prefix) :]
    package, _, relpath = remainder.partition("/")
    asset_root = ASSET_ROOTS.get(package)
    if asset_root is None or not relpath:
        return uri
    return f"file://{(asset_root / relpath).resolve()}"


def hand_mount_parent_links(root: ET.Element) -> set[str]:
    """Arm links that carry a replacement hand (parent of a ``*_hj_mount`` joint)."""
    parents: set[str] = set()
    for joint in root.findall("joint"):
        name = joint.get("name") or ""
        parent = joint.find("parent")
        if name.endswith("_hj_mount") and parent is not None:
            parents.add(parent.get("link") or "")
    return parents


def strip_stock_gripper_motor(root: ET.Element) -> None:
    """Swap link7 meshes for the cropped variants without the stock-gripper motor.

    Only applies to link7s that carry a replacement hand mount; a link7 with
    the stock gripper attached keeps the stock mesh (the motor is present).
    """
    swap_links = ARM_FLANGE_LINKS & hand_mount_parent_links(root)
    for link in root.findall("link"):
        if link.get("name") not in swap_links:
            continue
        for mesh in link.iter("mesh"):
            uri = mesh.get("filename")
            if not uri:
                continue
            head, sep, base = uri.rpartition("/")
            replacement = STOCK_LINK7_MESH_SWAP.get(base)
            if replacement:
                mesh.set("filename", f"{head}{sep}{replacement}")


def make_self_collision_safe(root: ET.Element, flange_collision_mesh: Path) -> None:
    """Remove the resting-pose penetrations so self-collision can be enabled.

    With articulation self-collision on, PhysX collides all non-adjacent link
    pairs. Two pairs interpenetrate at rest by construction and generate
    phantom forces:
      - link7 <-> *_hl_adapter: the flange bolts insert 4mm into the adapter
        plate (collision approximations lose the bolt holes)
      - link7 <-> *_hl_base: the bolt tips touch the hand mount bottom
    Fix: the hand-mounted link7 collision uses the bolt-free flange-cut mesh
    (visual keeps the bolts), and the fully enclosed adapter plate loses its
    collision geometry (nothing external can ever reach it).
    """
    hand_links = ARM_FLANGE_LINKS & hand_mount_parent_links(root)
    adapter_links = {f"{name[0]}_hl_adapter" for name in hand_links}
    for link in root.findall("link"):
        name = link.get("name")
        if name in hand_links:
            for collision in link.findall("collision"):
                mesh = collision.find("geometry/mesh")
                if mesh is not None:
                    mesh.set("filename", f"file://{flange_collision_mesh}")
        elif name in adapter_links:
            for collision in link.findall("collision"):
                link.remove(collision)


def fit_rh56f1_primitive_collisions(root: ET.Element) -> None:
    """Swap RH56F1 finger-chain collision meshes for fitted primitives."""
    import math

    import numpy as np
    import trimesh

    from gen_fabric_urdfs import mat_to_rpy, rpy_to_mat

    # RH56F1-only: gate on palm_1, which no other asset has (tesollo hands
    # share finger link names like r_hl_thumb_1 and must keep their meshes)
    if not any(link.get("name") in RH_DROP_COLLISION_LINKS for link in root.findall("link")):
        return

    axis_rotation = {
        0: rpy_to_mat(0.0, math.pi / 2.0, 0.0),  # cylinder z -> x
        1: rpy_to_mat(math.pi / 2.0, 0.0, 0.0),  # cylinder z -> -y (symmetric)
        2: np.eye(3),
    }

    for link in root.findall("link"):
        if link.get("name") in RH_DROP_COLLISION_LINKS:
            for collision in link.findall("collision"):
                link.remove(collision)
            continue
        if link.get("name") not in RH_PRIMITIVE_LINKS:
            continue
        for collision in link.findall("collision"):
            mesh_el = collision.find("geometry/mesh")
            if mesh_el is None:
                continue
            path = (mesh_el.get("filename") or "").removeprefix("file://")
            scale = np.array([float(v) for v in (mesh_el.get("scale") or "1 1 1").split()])
            vertices = trimesh.load(path, force="mesh").vertices * scale
            low, high = vertices.min(axis=0), vertices.max(axis=0)
            center, extents = (low + high) / 2.0, high - low

            origin = collision.find("origin")
            old_xyz = np.array([float(v) for v in ((origin.get("xyz") if origin is not None else None) or "0 0 0").split()])
            old_rpy = [float(v) for v in ((origin.get("rpy") if origin is not None else None) or "0 0 0").split()]
            old_rot = rpy_to_mat(*old_rpy)

            geometry = collision.find("geometry")
            assert geometry is not None
            geometry.remove(mesh_el)
            order = np.argsort(extents)[::-1]
            if extents[order[0]] / max(extents[order[1]], 1e-9) > RH_CYLINDER_ELONGATION:
                axis = int(order[0])
                radius = float(extents[order[2]]) / 2.0  # inscribed: smallest cross extent
                ET.SubElement(geometry, "cylinder", {
                    "radius": f"{radius:.6g}", "length": f"{extents[axis]:.6g}",
                })
                new_rot = old_rot @ axis_rotation[axis]
            else:
                ET.SubElement(geometry, "box", {"size": " ".join(f"{v:.6g}" for v in extents)})
                new_rot = old_rot
            new_xyz = old_xyz + old_rot @ center
            if origin is None:
                origin = ET.SubElement(collision, "origin")
            origin.set("xyz", " ".join(f"{v:.6g}" for v in new_xyz))
            origin.set("rpy", " ".join(f"{v:.6g}" for v in mat_to_rpy(new_rot)))


def build_openarm_maps(link_names: set[str], joint_names: set[str]) -> tuple[dict[str, str], dict[str, str]]:
    link_map: dict[str, str] = {}
    joint_map: dict[str, str] = {}

    if "world" in link_names:
        add(link_map, "world", "body_root")
    if "openarm_body_link0" in link_names:
        add(link_map, "openarm_body_link0", "body_link")
    if "openarm_body_world_joint" in joint_names:
        add(joint_map, "openarm_body_world_joint", "body_j_base")

    for side in ("right", "left"):
        s = side_prefix(side)
        for i in range(8):
            old_link = f"openarm_{side}_link{i}"
            if old_link in link_names:
                add(link_map, old_link, f"{s}_al_{i}")
        fixed_joint = f"openarm_{side}_openarm_body_link0_joint"
        if fixed_joint in joint_names:
            add(joint_map, fixed_joint, f"{s}_aj_base")
        for i in range(1, 8):
            old_joint = f"openarm_{side}_joint{i}"
            if old_joint in joint_names:
                add(joint_map, old_joint, f"{s}_aj_{i}")

    return link_map, joint_map


def build_tesollo_maps(
    link_names: set[str], joint_names: set[str]
) -> tuple[dict[str, str], dict[str, str]]:
    link_map: dict[str, str] = {}
    joint_map: dict[str, str] = {}

    variants = [
        {
            "side": "right",
            "link_prefix": "rl",
            "joint_prefix": "rj",
            "base_link": "right_base_link",
            "adapter_joint": "dummy_joint",
            "mount_joint": "mount_right_link7_to_tesollo",
            "alias_link": "right_palm_link",
            "legacy_alias_link": "palm_link",
            "ee_link": "right_palm_ee",
            "legacy_ee_link": "palm_ee",
            "alias_joint": "right_upstream_palm_to_palm_link",
            "legacy_alias_joint": "upstream_palm_to_palm_link",
            "ee_joint": "right_palm_link_to_ee",
            "legacy_ee_joint": "palm_link_to_ee",
        },
        {
            "side": "left",
            "link_prefix": "ll",
            "joint_prefix": "lj",
            "base_link": "left_base_link",
            "adapter_joint": "base_joint2",
            "mount_joint": "mount_left_link7_to_tesollo",
            "alias_link": "left_palm_link",
            "legacy_alias_link": None,
            "ee_link": "left_palm_ee",
            "legacy_ee_link": None,
            "alias_joint": "left_upstream_palm_to_palm_link",
            "legacy_alias_joint": None,
            "ee_joint": "left_palm_link_to_ee",
            "legacy_ee_joint": None,
        },
    ]

    for cfg in variants:
        side = cfg["side"]
        s = side_prefix(side)
        lp = cfg["link_prefix"]
        jp = cfg["joint_prefix"]

        if cfg["base_link"] in link_names:
            add(link_map, cfg["base_link"], f"{s}_hl_mount")
        if f"{lp}_dg_mount" in link_names:
            add(link_map, f"{lp}_dg_mount", f"{s}_hl_adapter")
        if f"{lp}_dg_base" in link_names:
            add(link_map, f"{lp}_dg_base", f"{s}_hl_base")
        if f"{lp}_dg_palm" in link_names:
            add(link_map, f"{lp}_dg_palm", f"{s}_hl_palm")
        for key in ("alias_link", "legacy_alias_link"):
            old = cfg.get(key)
            if old and old in link_names:
                add(link_map, old, f"{s}_hl_palm_alias")
        for key in ("ee_link", "legacy_ee_link"):
            old = cfg.get(key)
            if old and old in link_names:
                add(link_map, old, f"{s}_hl_palm_ee")

        if cfg["mount_joint"] in joint_names:
            add(joint_map, cfg["mount_joint"], f"{s}_hj_mount")
        if cfg["adapter_joint"] in joint_names:
            add(joint_map, cfg["adapter_joint"], f"{s}_hj_adapter")
        if f"{jp}_dg_base" in joint_names:
            add(joint_map, f"{jp}_dg_base", f"{s}_hj_base")
        if f"{jp}_dg_palm" in joint_names:
            add(joint_map, f"{jp}_dg_palm", f"{s}_hj_palm")
        for key in ("alias_joint", "legacy_alias_joint"):
            old = cfg.get(key)
            if old and old in joint_names:
                add(joint_map, old, f"{s}_hj_palm_alias")
        for key in ("ee_joint", "legacy_ee_joint"):
            old = cfg.get(key)
            if old and old in joint_names:
                add(joint_map, old, f"{s}_hj_palm_ee")

        for dg_idx, finger in TESOLLO_FINGER_BY_INDEX.items():
            for segment in range(1, 5):
                old_link = f"{lp}_dg_{dg_idx}_{segment}"
                old_joint = f"{jp}_dg_{dg_idx}_{segment}"
                if old_link in link_names:
                    add(link_map, old_link, f"{s}_hl_{finger}_{segment}")
                if old_joint in joint_names:
                    add(joint_map, old_joint, f"{s}_hj_{finger}_{segment}")
            old_tip_link = f"{lp}_dg_{dg_idx}_tip"
            old_tip_joint = f"{jp}_dg_{dg_idx}_tip"
            if old_tip_link in link_names:
                add(link_map, old_tip_link, f"{s}_hl_{finger}_tip")
            if old_tip_joint in joint_names:
                add(joint_map, old_tip_joint, f"{s}_hj_{finger}_tip")

    return link_map, joint_map


def build_rh56f1_maps(link_names: set[str], joint_names: set[str]) -> tuple[dict[str, str], dict[str, str]]:
    link_map: dict[str, str] = {}
    joint_map: dict[str, str] = {}

    for side in ("right", "left"):
        s = side_prefix(side)
        base = f"rh56f1_{side}"
        old_base_link = f"{base}_base_link"
        if old_base_link in link_names:
            add(link_map, old_base_link, f"{s}_hl_base")
        mount_joint = f"mount_{side}_link7_to_rh56f1"
        if mount_joint in joint_names:
            add(joint_map, mount_joint, f"{s}_hj_mount")

        for i in range(1, 4):
            old_link = f"{base}_plam_{i}"
            old_joint = f"{base}_plam_{i}_joint"
            # The right hand source has a misspelled joint suffix: jont.
            old_joint_typo = f"{base}_plam_{i}_jont"
            if old_link in link_names:
                add(link_map, old_link, f"{s}_hl_palm_{i}")
            if old_joint in joint_names:
                add(joint_map, old_joint, f"{s}_hj_palm_{i}")
            if old_joint_typo in joint_names:
                add(joint_map, old_joint_typo, f"{s}_hj_palm_{i}")
        old_sensor_link = f"{base}_plam_force_sensor"
        old_sensor_joint = f"{base}_plam_force_sensor_joint"
        if old_sensor_link in link_names:
            add(link_map, old_sensor_link, f"{s}_hl_palm_sensor")
        if old_sensor_joint in joint_names:
            add(joint_map, old_sensor_joint, f"{s}_hj_palm_sensor")

        for src_finger, canonical_finger in RH_FINGERS.items():
            max_segment = 4 if src_finger == "thumb" else 2
            for segment in range(1, max_segment + 1):
                old_link = f"{base}_{side}_{src_finger}_{segment}"
                old_joint = f"{base}_{side}_{src_finger}_{segment}_joint"
                if old_link in link_names:
                    add(link_map, old_link, f"{s}_hl_{canonical_finger}_{segment}")
                if old_joint in joint_names:
                    add(joint_map, old_joint, f"{s}_hj_{canonical_finger}_{segment}")
            sensor_link = f"{base}_{src_finger}_force_sensor"
            sensor_joint = f"{base}_{src_finger}_force_sensor_joint"
            tip_link = f"{base}_{src_finger}_tip"
            tip_joint = f"{base}_{src_finger}_tip_joint"
            if sensor_link in link_names:
                add(link_map, sensor_link, f"{s}_hl_{canonical_finger}_sensor")
            if sensor_joint in joint_names:
                add(joint_map, sensor_joint, f"{s}_hj_{canonical_finger}_sensor")
            if tip_link in link_names:
                add(link_map, tip_link, f"{s}_hl_{canonical_finger}_tip")
            if tip_joint in joint_names:
                add(joint_map, tip_joint, f"{s}_hj_{canonical_finger}_tip")

    return link_map, joint_map


def build_stock_gripper_maps(link_names: set[str], joint_names: set[str]) -> tuple[dict[str, str], dict[str, str]]:
    link_map: dict[str, str] = {}
    joint_map: dict[str, str] = {}

    for side in ("right", "left"):
        s = side_prefix(side)
        entries = {
            f"openarm_{side}_hand": f"{s}_hl_gripper_base",
            f"openarm_{side}_hand_tcp": f"{s}_hl_gripper_tcp",
            f"openarm_{side}_left_finger": f"{s}_hl_gripper_left_finger",
            f"openarm_{side}_right_finger": f"{s}_hl_gripper_right_finger",
        }
        for old, new in entries.items():
            if old in link_names:
                add(link_map, old, new)
        joint_entries = {
            f"{side}_openarm_hand_joint": f"{s}_hj_gripper_mount",
            f"openarm_{side}_hand_tcp_joint": f"{s}_hj_gripper_tcp",
            f"openarm_{side}_finger_joint1": f"{s}_hj_gripper_1",
            f"openarm_{side}_finger_joint2": f"{s}_hj_gripper_2",
        }
        for old, new in joint_entries.items():
            if old in joint_names:
                add(joint_map, old, new)

    return link_map, joint_map


def merge_maps(*maps: tuple[dict[str, str], dict[str, str]]) -> tuple[dict[str, str], dict[str, str]]:
    link_map: dict[str, str] = {}
    joint_map: dict[str, str] = {}
    for lm, jm in maps:
        for old, new in lm.items():
            add(link_map, old, new)
        for old, new in jm.items():
            add(joint_map, old, new)
    return link_map, joint_map


def all_names(root: ET.Element) -> tuple[set[str], set[str]]:
    return (
        {elem.attrib["name"] for elem in root.findall("link") if "name" in elem.attrib},
        {elem.attrib["name"] for elem in root.findall("joint") if "name" in elem.attrib},
    )


def rename_tree(root: ET.Element, link_map: dict[str, str], joint_map: dict[str, str]) -> None:
    root.attrib["name"] = f"{root.attrib.get('name', 'robot')}_rl"

    for link in root.findall("link"):
        name = link.attrib.get("name")
        if name in link_map:
            link.attrib["name"] = link_map[name]

    for joint in root.findall("joint"):
        name = joint.attrib.get("name")
        if name in joint_map:
            joint.attrib["name"] = joint_map[name]

    for elem in root.iter():
        link_ref = elem.attrib.get("link")
        if link_ref in link_map:
            elem.attrib["link"] = link_map[link_ref]
        joint_ref = elem.attrib.get("joint")
        if joint_ref in joint_map:
            elem.attrib["joint"] = joint_map[joint_ref]
        reference = elem.attrib.get("reference")
        if reference in link_map:
            elem.attrib["reference"] = link_map[reference]
        filename = elem.attrib.get("filename")
        if filename:
            elem.attrib["filename"] = normalize_mesh_uri(filename)


def shift_origin_z(parent: ET.Element, dz: float) -> None:
    origin = parent.find("origin")
    if origin is None:
        origin = ET.SubElement(parent, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    xyz = origin.attrib.get("xyz", "0 0 0").split()
    if len(xyz) != 3:
        raise ValueError(f"malformed origin xyz: {origin.attrib.get('xyz')}")
    x, y, z = (float(v) for v in xyz)
    origin.attrib["xyz"] = f"{x:g} {y:g} {z + dz:g}"


def apply_base_origin_fix(root: ET.Element, cropped_meshes: dict[str, Path]) -> None:
    """Move the robot origin from the vendor plate bottom to the plate top.

    Uses pre-cropped/pre-shifted body meshes and lowers everything defined in
    the vendor body frame (inertial COM, arm base joints) by BASE_LIFT_M.
    """
    body = next((l for l in root.findall("link") if l.attrib.get("name") == "body_link"), None)
    if body is None:
        raise ValueError("body_link not found; cannot apply base origin fix")

    for tag, kind in (("visual", "visual"), ("collision", "collision")):
        for elem in body.findall(tag):
            mesh = elem.find("geometry/mesh")
            if mesh is None:
                raise ValueError(f"body_link {tag} has no mesh geometry")
            mesh.attrib["filename"] = f"file://{cropped_meshes[kind]}"

    inertial = body.find("inertial")
    if inertial is not None:
        shift_origin_z(inertial, -BASE_LIFT_M)

    arm_bases = [j for j in root.findall("joint") if j.attrib.get("name") in {"r_aj_base", "l_aj_base"}]
    if len(arm_bases) != 2:
        raise ValueError("expected both r_aj_base and l_aj_base joints")
    for joint in arm_bases:
        shift_origin_z(joint, -BASE_LIFT_M)


def attach_head(root: ET.Element) -> tuple[dict[str, str], dict[str, str]]:
    """Merge the D435i pan/tilt head into the robot, mounted on body_link."""
    head_root = ET.parse(HEAD_URDF).getroot()

    for link in head_root.findall("link"):
        new = copy.deepcopy(link)
        name = new.attrib.get("name")
        if name not in HEAD_LINK_MAP:
            raise ValueError(f"unexpected head link: {name}")
        new.attrib["name"] = HEAD_LINK_MAP[name]
        for mesh in new.iter("mesh"):
            filename = mesh.attrib.get("filename", "")
            if not filename.startswith("../"):
                raise ValueError(f"unexpected head mesh path: {filename}")
            mesh.attrib["filename"] = f"file://{(HEAD_URDF.parent / filename).resolve()}"
        root.append(new)

    for joint in head_root.findall("joint"):
        new = copy.deepcopy(joint)
        name = new.attrib.get("name")
        if name not in HEAD_JOINT_MAP:
            raise ValueError(f"unexpected head joint: {name}")
        new.attrib["name"] = HEAD_JOINT_MAP[name]
        for elem in new.iter():
            link_ref = elem.attrib.get("link")
            if link_ref in HEAD_LINK_MAP:
                elem.attrib["link"] = HEAD_LINK_MAP[link_ref]
        root.append(new)

    mount = ET.SubElement(root, "joint", {"name": "head_j_mount", "type": "fixed"})
    ET.SubElement(mount, "parent", {"link": "body_link"})
    ET.SubElement(mount, "child", {"link": "head_base"})
    ET.SubElement(mount, "origin", {"xyz": HEAD_MOUNT_XYZ, "rpy": "0 0 0"})

    ET.SubElement(root, "link", {"name": "head_cam_view"})
    view = ET.SubElement(root, "joint", {"name": "head_j_cam_view", "type": "fixed"})
    ET.SubElement(view, "parent", {"link": "head_camera"})
    ET.SubElement(view, "child", {"link": "head_cam_view"})
    ET.SubElement(
        view,
        "origin",
        {
            "xyz": " ".join(f"{v:g}" for v in HEAD_CAM_VIEW_XYZ),
            "rpy": " ".join(f"{v:g}" for v in HEAD_CAM_VIEW_RPY),
        },
    )

    return dict(HEAD_LINK_MAP), dict(HEAD_JOINT_MAP)


def sorted_joints(root: ET.Element) -> list[ET.Element]:
    joints = list(root.findall("joint"))
    by_name = {j.attrib["name"]: j for j in joints}
    ordered_names: list[str] = []

    for prefix in ("r", "l"):
        for name in [f"{prefix}_aj_base"] + [f"{prefix}_aj_{i}" for i in range(1, 8)]:
            if name in by_name:
                ordered_names.append(name)
        hand_names = hand_kinematic_order(prefix)
        ordered_names.extend([name for name in hand_names if name in by_name])

    for name in HEAD_JOINT_ORDER:
        if name in by_name:
            ordered_names.append(name)

    if "body_j_base" in by_name:
        ordered_names.insert(0, "body_j_base")

    seen = set(ordered_names)
    # Keep any unexpected joints deterministic and visible after the canonical blocks.
    ordered_names.extend(sorted(name for name in by_name if name not in seen))
    return [by_name[name] for name in ordered_names]


def sorted_links(root: ET.Element) -> list[ET.Element]:
    links = list(root.findall("link"))
    by_name = {l.attrib["name"]: l for l in links}
    ordered_names: list[str] = []

    for name in ["body_root", "body_link"]:
        if name in by_name:
            ordered_names.append(name)
    for prefix in ("r", "l"):
        ordered_names.extend([name for name in [f"{prefix}_al_{i}" for i in range(8)] if name in by_name])
        ordered_names.extend([name for name in hand_link_order(prefix) if name in by_name])
    ordered_names.extend([name for name in HEAD_LINK_ORDER if name in by_name])

    seen = set(ordered_names)
    ordered_names.extend(sorted(name for name in by_name if name not in seen))
    return [by_name[name] for name in ordered_names]


def hand_kinematic_order(prefix: str) -> list[str]:
    names = [
        f"{prefix}_hj_mount",
        f"{prefix}_hj_adapter",
        f"{prefix}_hj_base",
        f"{prefix}_hj_palm",
        f"{prefix}_hj_palm_alias",
        f"{prefix}_hj_palm_ee",
        f"{prefix}_hj_palm_1",
        f"{prefix}_hj_palm_2",
        f"{prefix}_hj_palm_3",
        f"{prefix}_hj_palm_sensor",
        f"{prefix}_hj_gripper_mount",
        f"{prefix}_hj_gripper_tcp",
        f"{prefix}_hj_gripper_1",
        f"{prefix}_hj_gripper_2",
    ]
    for finger in FINGER_NAMES:
        names.extend([f"{prefix}_hj_{finger}_{i}" for i in range(1, 5)])
        names.extend([f"{prefix}_hj_{finger}_sensor", f"{prefix}_hj_{finger}_tip"])
    return names


def hand_link_order(prefix: str) -> list[str]:
    names = [
        f"{prefix}_hl_mount",
        f"{prefix}_hl_adapter",
        f"{prefix}_hl_base",
        f"{prefix}_hl_palm",
        f"{prefix}_hl_palm_alias",
        f"{prefix}_hl_palm_ee",
        f"{prefix}_hl_palm_1",
        f"{prefix}_hl_palm_2",
        f"{prefix}_hl_palm_3",
        f"{prefix}_hl_palm_sensor",
        f"{prefix}_hl_gripper_base",
        f"{prefix}_hl_gripper_tcp",
        f"{prefix}_hl_gripper_left_finger",
        f"{prefix}_hl_gripper_right_finger",
    ]
    for finger in FINGER_NAMES:
        names.extend([f"{prefix}_hl_{finger}_{i}" for i in range(1, 5)])
        names.extend([f"{prefix}_hl_{finger}_sensor", f"{prefix}_hl_{finger}_tip"])
    return names


def reorder_top_level(root: ET.Element) -> None:
    non_links_joints = [copy.deepcopy(e) for e in list(root) if e.tag not in {"link", "joint"}]
    links = [copy.deepcopy(e) for e in sorted_links(root)]
    joints = [copy.deepcopy(e) for e in sorted_joints(root)]
    root[:] = non_links_joints + links + joints


def control_joint_order(root: ET.Element) -> list[str]:
    by_name = {j.attrib["name"]: j for j in root.findall("joint")}
    order: list[str] = []
    for prefix in ("r", "l"):
        order.extend([f"{prefix}_aj_{i}" for i in range(1, 8) if f"{prefix}_aj_{i}" in by_name])
        for finger in FINGER_NAMES:
            for i in range(1, 5):
                name = f"{prefix}_hj_{finger}_{i}"
                joint = by_name.get(name)
                if (
                    joint is not None
                    and joint.attrib.get("type") in MOVABLE_TYPES
                    and joint.find("mimic") is None
                ):
                    order.append(name)
        for name in [f"{prefix}_hj_gripper_1", f"{prefix}_hj_gripper_2"]:
            joint = by_name.get(name)
            if joint is not None and joint.attrib.get("type") in MOVABLE_TYPES and joint.find("mimic") is None:
                order.append(name)
    return order


def kinematic_joint_order(root: ET.Element) -> list[str]:
    return [j.attrib["name"] for j in sorted_joints(root)]


def link_order(root: ET.Element) -> list[str]:
    return [l.attrib["name"] for l in sorted_links(root)]


def validate(root: ET.Element, control_order: list[str]) -> None:
    link_names = [l.attrib["name"] for l in root.findall("link")]
    joint_names = [j.attrib["name"] for j in root.findall("joint")]
    if len(link_names) != len(set(link_names)):
        raise ValueError("duplicate link names after canonical rename")
    if len(joint_names) != len(set(joint_names)):
        raise ValueError("duplicate joint names after canonical rename")

    link_set = set(link_names)
    joint_by_name = {j.attrib["name"]: j for j in root.findall("joint")}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is not None and parent.attrib.get("link") not in link_set:
            raise ValueError(f"joint {joint.attrib['name']} references missing parent {parent.attrib.get('link')}")
        if child is not None and child.attrib.get("link") not in link_set:
            raise ValueError(f"joint {joint.attrib['name']} references missing child {child.attrib.get('link')}")
    for name in control_order:
        joint = joint_by_name.get(name)
        if joint is None:
            raise ValueError(f"control joint missing from URDF: {name}")
        if joint.attrib.get("type") not in MOVABLE_TYPES:
            raise ValueError(f"control joint is not movable: {name}")
        if joint.find("mimic") is not None:
            raise ValueError(f"control joint is a mimic joint: {name}")


def yaml_list(values: Iterable[str], indent: int = 2) -> str:
    pad = " " * indent
    values = list(values)
    if not values:
        return f"{pad}[]\n"
    return "".join(f"{pad}- {value}\n" for value in values)


def yaml_map(mapping: dict[str, str], indent: int = 2) -> str:
    pad = " " * indent
    if not mapping:
        return f"{pad}{{}}\n"
    return "".join(f"{pad}{old}: {new}\n" for old, new in sorted(mapping.items()))


def write_manifest(
    output_path: Path,
    source_path: Path,
    urdf_path: Path,
    root: ET.Element,
    link_map: dict[str, str],
    joint_map: dict[str, str],
) -> None:
    control = control_joint_order(root)
    kinematic = kinematic_joint_order(root)
    fixed = [
        j.attrib["name"]
        for j in root.findall("joint")
        if j.attrib.get("type") == "fixed"
    ]
    links = link_order(root)

    text = []
    text.append("# Generated by tools/generate_rl_urdf.py. Do not edit by hand.\n")
    text.append(f"source_urdf: {source_path.relative_to(ROOT)}\n")
    text.append(f"generated_urdf: {urdf_path.relative_to(ROOT)}\n")
    # USD import contract: hull-only self-collision clearances (see
    # tools/self_collision_allowlist.yaml) assume this collider approximation.
    text.append("requires_collision_approximation: convex_decomposition\n")
    text.append("schema:\n")
    text.append("  body_root: stage-level root link with no geometry\n")
    text.append("  body_link: openarm physical base link\n")
    text.append("  r_aj_N/l_aj_N: right/left OpenArm movable arm joints\n")
    text.append("  r_hj_*/l_hj_*: right/left hand joints, including fixed mount/base/palm/sensor joints\n")
    text.append("  r_al_N/l_al_N: right/left OpenArm arm links\n")
    text.append("  r_hl_*/l_hl_*: right/left hand links\n")
    text.append("  head_*: D435i pan/tilt head; head_j_pan/head_j_tilt are revolute but stay out of control_joint_order\n")
    text.append("  head_cam_view: camera view origin frame (+X = viewing direction, +Z up), fixed to head_camera\n")
    text.append("  origin note: robot origin sits at the mount plate TOP (vendor origin +8mm); head mounts at z=+0.750m\n")
    text.append("camera_view_frame:\n")
    text.append("  link: head_cam_view\n")
    text.append("  parent_link: head_camera\n")
    text.append(f"  xyz: [{', '.join(f'{v:g}' for v in HEAD_CAM_VIEW_XYZ)}]\n")
    text.append(f"  rpy: [{', '.join(f'{v:g}' for v in HEAD_CAM_VIEW_RPY)}]\n")
    text.append("  convention: +X viewing direction, +Z up (robot frame; not ROS optical)\n")
    text.append("control_joint_order:\n")
    text.append(yaml_list(control))
    text.append("kinematic_joint_order:\n")
    text.append(yaml_list(kinematic))
    text.append("fixed_joint_order:\n")
    text.append(yaml_list(fixed))
    text.append("link_order:\n")
    text.append(yaml_list(links))
    text.append("source_to_canonical_joints:\n")
    text.append(yaml_map(joint_map))
    text.append("source_to_canonical_links:\n")
    text.append(yaml_map(link_map))
    output_path.write_text("".join(text), encoding="utf-8")


def generate_one(name: str, source_path: Path) -> tuple[Path, Path]:
    tree = ET.parse(source_path)
    root = tree.getroot()
    link_names, joint_names = all_names(root)
    link_map, joint_map = merge_maps(
        build_openarm_maps(link_names, joint_names),
        build_stock_gripper_maps(link_names, joint_names),
        build_tesollo_maps(link_names, joint_names),
        build_rh56f1_maps(link_names, joint_names),
    )

    rename_tree(root, link_map, joint_map)
    strip_stock_gripper_motor(root)
    make_self_collision_safe(root, ensure_link7_flange_mesh())
    fit_rh56f1_primitive_collisions(root)
    apply_base_origin_fix(root, ensure_cropped_meshes())
    head_link_map, head_joint_map = attach_head(root)
    link_map, joint_map = merge_maps((link_map, joint_map), (head_link_map, head_joint_map))
    reorder_top_level(root)
    control = control_joint_order(root)
    validate(root, control)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    urdf_path = OUT_DIR / f"{name}_rl.urdf"
    manifest_path = OUT_DIR / f"{name}_rl_manifest.yaml"

    ET.indent(tree, space="  ")
    tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
    write_manifest(manifest_path, source_path, urdf_path, root, link_map, joint_map)
    return urdf_path, manifest_path


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "names",
        nargs="*",
        help="Optional source names to generate. Defaults to all stable source URDFs.",
    )
    parser.add_argument(
        "--skip-audit",
        action="store_true",
        help="Skip the self-collision audit after generation (used by fast tests).",
    )
    args = parser.parse_args(argv)

    invalid = [name for name in args.names if name not in SOURCES]
    if invalid:
        parser.error(f"unknown source name(s): {', '.join(invalid)}")
    names = args.names or list(SOURCES.keys())
    generated: list[tuple[Path, Path]] = []
    for name in names:
        urdf_path, manifest_path = generate_one(name, SOURCES[name])
        generated.append((urdf_path, manifest_path))
        print(f"generated {urdf_path.relative_to(ROOT)}")
        print(f"generated {manifest_path.relative_to(ROOT)}")

    if not args.skip_audit:
        import audit_self_collision

        ok = True
        for urdf_path, manifest_path in generated:
            # Zero pose plus registered task homes (tools/audit_poses.yaml):
            # near-contact pairs can appear only at a task home.
            pose_findings = audit_self_collision.audit_asset(urdf_path)
            all_findings: list = []
            for pose_name, findings in pose_findings.items():
                ok &= audit_self_collision.report(f"{urdf_path.stem}@{pose_name}", findings)
                all_findings.extend(findings)
            # tools/build_usd.py turns these into PhysX collision filters;
            # WARN pairs are unioned across poses and left/right symmetrized.
            pairs = audit_self_collision.filtered_pairs(
                all_findings, audit_self_collision.urdf_link_names(urdf_path))
            with manifest_path.open("a", encoding="utf-8") as f:
                f.write("self_collision_filtered_pairs:"
                        "  # audited near-contact/nested pairs -> USD collision filters\n")
                f.writelines(f"- [{a}, {b}]\n" for a, b in pairs)
        if not ok:
            print("self-collision audit FAILED - asset(s) interpenetrate at rest", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

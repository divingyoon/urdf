#!/usr/bin/env python3
"""Generate side-prefixed DG5F-S xacro fragments from the vendor URDFs.

The vendor DG5F-S release (`vendor/tesollo_model/dg5fs/`) ships plain URDFs
whose link/joint names are identical for the left and right hands
(``link_1_1``, ``joint_1_1``, ...), so they cannot be combined into one
bimanual robot as-is. This tool rewrites the ``*_w_mount`` vendor URDFs into
the same structure and naming convention the DG5F xacros use, and prepends the
``{side}_base_link`` adapter header expected by
``eef/tesollo_*_wrapper.xacro`` and ``tools/generate_rl_urdf.py``.

The DG5F arm-side adapter plate (``rl_dg_mount``/``ll_dg_mount`` from
``dg_description``) is inserted between the arm flange and the DG5F-S vendor
mount, because the same physical adapter is reused for the DG5F-S hands. The
resulting chain mirrors DG5F exactly:

    {side}_base_link
      -> adapter joint (z 0)      -> {lp}_dg_mount  (DG5F adapter plate mesh)
      -> {jp}_dg_base (z +0.004)  -> {lp}_dg_base   (DG5F-S vendor link_mount)
      -> {jp}_dg_palm (z +0.015)  -> {lp}_dg_palm   (DG5F-S vendor link_base)
      -> finger chains            -> {lp}_dg_N_M / {lp}_dg_N_tip

Mesh URIs use ``package://tesollo_model/...`` and ``package://dg_description/...``
which ``generate_rl_urdf.py`` resolves through ``ASSET_ROOTS``.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VENDOR_DIR = ROOT / "vendor" / "tesollo_model"
SOURCE_DIR = VENDOR_DIR / "dg5fs"
OUT_DIR = ROOT / "eef" / "dg5fs"

# Seat height of the DG5F adapter plate: the hand rests on the 4mm plate while
# the taller bosses insert into the hand-side mount (matches DG5F rj_dg_base).
ADAPTER_SEAT_Z = 0.004

SIDE_CONFIG = {
    "right": {
        "link_prefix": "rl",
        "joint_prefix": "rj",
        "adapter_joint": "dummy_joint",
        "adapter_inertial": {
            "origin_xyz": "-6.4721E-09 -2.1674E-05 0.0049997",
            "inertia": {
                "ixx": "1.0999E-05",
                "ixy": "-2.1734E-12",
                "ixz": "1.2441E-12",
                "iyy": "1.1018E-05",
                "iyz": "4.0435E-09",
                "izz": "2.0531E-05",
            },
        },
    },
    "left": {
        "link_prefix": "ll",
        "joint_prefix": "lj",
        "adapter_joint": "base_joint2",
        "adapter_inertial": {
            "origin_xyz": "6.4721E-09 2.1674E-05 0.0049997",
            "inertia": {
                "ixx": "1.0999E-05",
                "ixy": "-2.1734E-12",
                "ixz": "-1.2441E-12",
                "iyy": "1.1018E-05",
                "iyz": "-4.0435E-09",
                "izz": "2.0531E-05",
            },
        },
    },
}
ADAPTER_MASS = "0.05"

MESH_URI_RE = re.compile(r"^package://meshes/(?P<rel>.+)$")

# The vendor mount/base occupy the DG5F base/palm slots so the adapter plate
# can take the mount slot, mirroring the DG5F chain.
SPECIAL_LINK_RENAMES = {"link_mount": "{lp}_dg_base", "link_base": "{lp}_dg_palm"}
SPECIAL_JOINT_RENAMES = {"joint_base": "{jp}_dg_palm"}


def rename(name: str, link_prefix: str, joint_prefix: str) -> str:
    special_link = SPECIAL_LINK_RENAMES.get(name)
    if special_link:
        return special_link.format(lp=link_prefix)
    special_joint = SPECIAL_JOINT_RENAMES.get(name)
    if special_joint:
        return special_joint.format(jp=joint_prefix)
    if name.startswith("link_"):
        return f"{link_prefix}_dg_{name[len('link_') :]}"
    if name.startswith("joint_"):
        return f"{joint_prefix}_dg_{name[len('joint_') :]}"
    raise ValueError(f"unexpected vendor name: {name}")


def rewrite_mesh_uri(uri: str) -> str:
    match = MESH_URI_RE.match(uri)
    if match is None:
        raise ValueError(f"unexpected mesh uri: {uri}")
    return f"package://tesollo_model/dg5fs/meshes/{match.group('rel')}"


def build_adapter_link(side: str) -> tuple[ET.Element, ET.Element]:
    """DG5F adapter plate link + the fixed joint seating the vendor mount on it."""
    cfg = SIDE_CONFIG[side]
    lp, jp = cfg["link_prefix"], cfg["joint_prefix"]

    link = ET.Element("link", {"name": f"{lp}_dg_mount"})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": cfg["adapter_inertial"]["origin_xyz"], "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": ADAPTER_MASS})
    ET.SubElement(inertial, "inertia", cfg["adapter_inertial"]["inertia"])
    visual = ET.SubElement(link, "visual")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "mesh", {
        "filename": f"package://dg_description/meshes/dg5f_{side}/visual/{lp}_dg_mount.dae",
    })
    collision = ET.SubElement(link, "collision")
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "mesh", {
        "filename": f"package://dg_description/meshes/dg5f_{side}/collision/{lp}_dg_mount_c.STL",
    })

    seat = ET.Element("joint", {"name": f"{jp}_dg_base", "type": "fixed"})
    ET.SubElement(seat, "origin", {"xyz": f"0 0 {ADAPTER_SEAT_Z}", "rpy": "0 0 0"})
    ET.SubElement(seat, "parent", {"link": f"{lp}_dg_mount"})
    ET.SubElement(seat, "child", {"link": f"{lp}_dg_base"})
    return link, seat


def convert_side(side: str) -> Path:
    cfg = SIDE_CONFIG[side]
    lp, jp = cfg["link_prefix"], cfg["joint_prefix"]
    source_path = SOURCE_DIR / f"dg5fs_{side}_w_mount.urdf"
    root = ET.parse(source_path).getroot()

    for el in root.iter():
        name = el.get("name")
        link = el.get("link")
        filename = el.get("filename")
        if el.tag in {"link", "joint"} and name:
            el.set("name", rename(name, lp, jp))
        if el.tag in {"parent", "child"} and link:
            el.set("link", rename(link, lp, jp))
        if el.tag == "mesh" and filename:
            el.set("filename", rewrite_mesh_uri(filename))

    out_root = ET.Element("robot", {"name": f"dg5fs_{side}"})
    out_root.append(ET.Comment(
        f" GENERATED by tools/gen_dg5fs_xacro.py from "
        f"vendor/tesollo_model/dg5fs/dg5fs_{side}_w_mount.urdf - do not edit. "
    ))

    base_link = ET.SubElement(out_root, "link", {"name": f"{side}_base_link"})
    base_link.text = None
    adapter_joint = ET.SubElement(out_root, "joint", {"name": cfg["adapter_joint"], "type": "fixed"})
    ET.SubElement(adapter_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(adapter_joint, "parent", {"link": f"{side}_base_link"})
    ET.SubElement(adapter_joint, "child", {"link": f"{lp}_dg_mount"})

    adapter_link, seat_joint = build_adapter_link(side)
    out_root.append(adapter_link)
    out_root.append(seat_joint)

    for child in list(root):
        out_root.append(child)

    ET.indent(out_root, space="  ")
    out_path = OUT_DIR / f"dg5fs_{side}.xacro"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.ElementTree(out_root)
    tree.write(out_path, encoding="unicode", xml_declaration=True)
    return out_path


def generate_all() -> dict[str, Path]:
    return {side: convert_side(side) for side in SIDE_CONFIG}


def main() -> int:
    for side, path in generate_all().items():
        print(f"generated {path.relative_to(ROOT)} ({side})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Tests for the DG5F-S vendor URDF -> prefixed xacro fragment converter."""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import gen_dg5fs_xacro as gen  # noqa: E402

SIDES = ["left", "right"]


@pytest.fixture(scope="module")
def outputs() -> dict[str, ET.Element]:
    paths = gen.generate_all()
    return {side: ET.parse(paths[side]).getroot() for side in SIDES}


def names(root: ET.Element, tag: str) -> set[str]:
    return {name for el in root.findall(tag) if (name := el.get("name"))}


@pytest.mark.parametrize("side", SIDES)
def test_all_names_prefixed(outputs, side):
    root = outputs[side]
    lp = "rl" if side == "right" else "ll"
    jp = "rj" if side == "right" else "lj"
    adapter = "dummy_joint" if side == "right" else "base_joint2"

    link_names = names(root, "link")
    joint_names = names(root, "joint")
    assert f"{side}_base_link" in link_names
    for name in link_names - {f"{side}_base_link"}:
        assert name.startswith(f"{lp}_dg_"), name
    for name in joint_names - {adapter}:
        assert name.startswith(f"{jp}_dg_"), name


def test_left_right_names_disjoint(outputs):
    left = names(outputs["left"], "link") | names(outputs["left"], "joint")
    right = names(outputs["right"], "link") | names(outputs["right"], "joint")
    assert not left & right


@pytest.mark.parametrize("side", SIDES)
def test_twenty_revolute_joints(outputs, side):
    revolute = [j for j in outputs[side].findall("joint") if j.get("type") == "revolute"]
    assert len(revolute) == 20


@pytest.mark.parametrize("side", SIDES)
def test_mount_chain(outputs, side):
    """Chain mirrors DG5F: base_link -> adapter plate -> vendor mount -> vendor base."""
    lp = "rl" if side == "right" else "ll"
    jp = "rj" if side == "right" else "lj"
    adapter = "dummy_joint" if side == "right" else "base_joint2"
    joints = {j.get("name"): j for j in outputs[side].findall("joint")}

    adapter_joint = joints[adapter]
    assert adapter_joint.find("parent").get("link") == f"{side}_base_link"
    assert adapter_joint.find("child").get("link") == f"{lp}_dg_mount"

    seat_joint = joints[f"{jp}_dg_base"]
    assert seat_joint.find("parent").get("link") == f"{lp}_dg_mount"
    assert seat_joint.find("child").get("link") == f"{lp}_dg_base"
    z = float(seat_joint.find("origin").get("xyz").split()[2])
    assert abs(z - gen.ADAPTER_SEAT_Z) < 1e-12

    palm_joint = joints[f"{jp}_dg_palm"]
    assert palm_joint.find("parent").get("link") == f"{lp}_dg_base"
    assert palm_joint.find("child").get("link") == f"{lp}_dg_palm"
    z = float(palm_joint.find("origin").get("xyz").split()[2])
    assert abs(z - 0.015) < 1e-12


@pytest.mark.parametrize("side", SIDES)
def test_adapter_link_uses_dg5f_plate_mesh(outputs, side):
    lp = "rl" if side == "right" else "ll"
    links = {l.get("name"): l for l in outputs[side].findall("link")}
    meshes = [m.get("filename") for m in links[f"{lp}_dg_mount"].iter("mesh")]
    assert meshes == [
        f"package://dg_description/meshes/dg5f_{side}/visual/{lp}_dg_mount.dae",
        f"package://dg_description/meshes/dg5f_{side}/collision/{lp}_dg_mount_c.STL",
    ]


@pytest.mark.parametrize("side", SIDES)
def test_mesh_uris_use_vendor_package(outputs, side):
    package_roots = {
        "package://tesollo_model/": gen.VENDOR_DIR,
        "package://dg_description/": gen.ROOT / "vendor" / "delto_m_ros2" / "dg_description",
    }
    meshes = [m.get("filename") for m in outputs[side].iter("mesh")]
    assert meshes
    for uri in meshes:
        prefix = next((p for p in package_roots if uri.startswith(p)), None)
        assert prefix, uri
        assert (package_roots[prefix] / uri[len(prefix) :]).is_file(), uri


@pytest.mark.parametrize("side", SIDES)
def test_finger_links_complete(outputs, side):
    lp = "rl" if side == "right" else "ll"
    link_names = names(outputs[side], "link")
    for finger in range(1, 6):
        for segment in range(1, 5):
            assert f"{lp}_dg_{finger}_{segment}" in link_names
        assert f"{lp}_dg_{finger}_tip" in link_names

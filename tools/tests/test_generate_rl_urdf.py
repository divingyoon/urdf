"""Tests for the RL URDF generation pipeline (base origin fix + head attach)."""

from __future__ import annotations

import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

import generate_rl_urdf as gen  # noqa: E402

RL_NAMES = list(gen.SOURCES.keys())


@pytest.fixture(scope="session", autouse=True)
def generated_outputs() -> None:
    assert gen.main([]) == 0


def load_urdf(name: str) -> ET.Element:
    return ET.parse(gen.OUT_DIR / f"{name}_rl.urdf").getroot()


def load_manifest(name: str) -> dict:
    import yaml

    with open(gen.OUT_DIR / f"{name}_rl_manifest.yaml", encoding="utf-8") as f:
        return yaml.safe_load(f)


def joints_by_name(root: ET.Element) -> dict[str, ET.Element]:
    return {j.attrib["name"]: j for j in root.findall("joint")}


def origin_xyz(elem: ET.Element) -> tuple[float, float, float]:
    origin = elem.find("origin")
    assert origin is not None
    x, y, z = (float(v) for v in origin.attrib["xyz"].split())
    return x, y, z


def stl_z_range(path: Path) -> tuple[float, float]:
    data = path.read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    zmin, zmax = float("inf"), float("-inf")
    offset = 84
    for _ in range(count):
        record = data[offset : offset + 50]
        for v in range(3):
            z = struct.unpack("<f", record[20 + v * 12 : 24 + v * 12])[0]
            zmin, zmax = min(zmin, z), max(zmax, z)
        offset += 50
    return zmin, zmax


@pytest.mark.parametrize("name", RL_NAMES)
def test_arm_base_lifted_to_mount_top(name: str) -> None:
    joints = joints_by_name(load_urdf(name))
    for joint_name in ("r_aj_base", "l_aj_base"):
        _, _, z = origin_xyz(joints[joint_name])
        assert z == pytest.approx(0.690, abs=1e-9)


@pytest.mark.parametrize("name", RL_NAMES)
def test_body_link_uses_cropped_meshes(name: str) -> None:
    root = load_urdf(name)
    body = next(l for l in root.findall("link") if l.attrib["name"] == "body_link")
    for tag in ("visual", "collision"):
        mesh = body.find(f"{tag}/geometry/mesh")
        assert mesh is not None
        filename = mesh.attrib["filename"]
        assert filename.startswith("file://")
        assert "_cut.stl" in filename
        assert Path(filename[len("file://") :]).is_file()
    inertial = body.find("inertial")
    assert inertial is not None
    _, _, z = origin_xyz(inertial)
    assert z == pytest.approx(-0.008, abs=1e-9)


def test_cropped_collision_mesh_has_no_geometry_below_origin() -> None:
    zmin, zmax = stl_z_range(gen.ROOT / "generated" / "rl" / "meshes" / "body_link0_symp_cut.stl")
    assert zmin >= -1e-3
    assert zmax == pytest.approx(765.0, abs=0.5)


@pytest.mark.parametrize("name", RL_NAMES)
def test_head_attached(name: str) -> None:
    root = load_urdf(name)
    link_names = {l.attrib["name"] for l in root.findall("link")}
    assert {"head_base", "head_mid", "head_camera"} <= link_names

    joints = joints_by_name(root)
    mount = joints["head_j_mount"]
    assert mount.attrib["type"] == "fixed"
    parent, child = mount.find("parent"), mount.find("child")
    assert parent is not None and parent.attrib["link"] == "body_link"
    assert child is not None and child.attrib["link"] == "head_base"
    assert origin_xyz(mount) == pytest.approx((0.0, 0.0, 0.750))

    for joint_name in ("head_j_pan", "head_j_tilt"):
        assert joints[joint_name].attrib["type"] == "revolute"


@pytest.mark.parametrize("name", RL_NAMES)
def test_camera_view_frame(name: str) -> None:
    root = load_urdf(name)
    link_names = {l.attrib["name"] for l in root.findall("link")}
    assert "head_cam_view" in link_names

    joints = joints_by_name(root)
    view = joints["head_j_cam_view"]
    assert view.attrib["type"] == "fixed"
    parent, child = view.find("parent"), view.find("child")
    assert parent is not None and parent.attrib["link"] == "head_camera"
    assert child is not None and child.attrib["link"] == "head_cam_view"
    assert origin_xyz(view) == pytest.approx(gen.HEAD_CAM_VIEW_XYZ)

    manifest = load_manifest(name)
    frame = manifest["camera_view_frame"]
    assert frame["link"] == "head_cam_view"
    assert frame["parent_link"] == "head_camera"
    assert tuple(frame["xyz"]) == pytest.approx(gen.HEAD_CAM_VIEW_XYZ)
    assert tuple(frame["rpy"]) == pytest.approx(gen.HEAD_CAM_VIEW_RPY)


@pytest.mark.parametrize("name", RL_NAMES)
def test_head_joints_kinematic_only(name: str) -> None:
    manifest = load_manifest(name)
    control = manifest["control_joint_order"]
    kinematic = manifest["kinematic_joint_order"]
    assert not any(j.startswith("head_") for j in control)
    assert {"head_j_mount", "head_j_pan", "head_j_tilt"} <= set(kinematic)
    assert "head_j_mount" in manifest["fixed_joint_order"]


@pytest.mark.parametrize("name", RL_NAMES)
def test_control_joint_order_preserved(name: str) -> None:
    manifest = load_manifest(name)
    control = manifest["control_joint_order"]
    assert control[:7] == [f"r_aj_{i}" for i in range(1, 8)]
    assert all(j.startswith(("r_aj_", "r_hj_", "l_aj_", "l_hj_")) for j in control)


@pytest.mark.parametrize("name", RL_NAMES)
def test_head_mesh_paths_resolve(name: str) -> None:
    root = load_urdf(name)
    for link_name in ("head_base", "head_mid", "head_camera"):
        link = next(l for l in root.findall("link") if l.attrib["name"] == link_name)
        for mesh in link.iter("mesh"):
            filename = mesh.attrib["filename"]
            assert filename.startswith("file://")
            assert Path(filename[len("file://") :]).is_file()

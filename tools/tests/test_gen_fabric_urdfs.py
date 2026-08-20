"""Tests for the consolidated Fabrics URDF generator.

Each builder already enforces its FK gate (raises on mismatch), so generation
succeeding is the core assertion; the rest checks the fabric-code contracts.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

import gen_fabric_urdfs as gen  # noqa: E402

EXPECTED_CSPACE = {
    "openarm_tesollo_bi_s": 27,
    "openarm_tesollo_bi_s_left": 27,
    "openarm_tesollo_sensor_left_gripper": 7,
    "openarm_rh56f1": 26,
}


@pytest.fixture(scope="module")
def outputs() -> dict[str, Path]:
    return {name: build() for name, build in gen.VARIANTS.items()}


def test_all_variants_generate_with_fk_gate(outputs):
    assert set(outputs) == set(EXPECTED_CSPACE)
    for path in outputs.values():
        assert path.is_file()


@pytest.mark.parametrize("name", list(EXPECTED_CSPACE))
def test_cspace_dimension(outputs, name):
    joints = gen.parse_urdf(outputs[name])
    revolute = [n for n, j in joints.items() if j["type"] == "revolute"]
    assert len(revolute) == EXPECTED_CSPACE[name], revolute


@pytest.mark.parametrize("name", ["openarm_tesollo_bi_s", "openarm_tesollo_bi_s_left",
                                  "openarm_tesollo_sensor_left_gripper"])
def test_fabric_convention_frames_present(outputs, name):
    """palm helpers, palm_link, and fingertip frames are fabric-code contracts."""
    root = ET.parse(outputs[name]).getroot()
    joint_names = {j.get("name") for j in root.iter("joint")}
    link_names = {l.get("name") for l in root.iter("link")}
    assert gen.PALM_HELPER_JOINTS <= joint_names
    assert "palm_link" in link_names
    for index in range(1, 6):
        assert f"rl_dg_{index}_tip" in link_names


def test_gripper_hand_is_frozen(outputs):
    joints = gen.parse_urdf(outputs["openarm_tesollo_sensor_left_gripper"])
    revolute = [n for n, j in joints.items() if j["type"] == "revolute"]
    assert all(n.startswith("openarm_right_joint") for n in revolute)


def test_rh56f1_frames(outputs):
    root = ET.parse(outputs["openarm_rh56f1"]).getroot()
    link_names = {l.get("name") for l in root.iter("link")}
    for side in ("r", "l"):
        for key in gen.RH_PALM_AXIS:
            assert f"ps_{side}_{key}" in link_names
        for finger in gen.FINGERS:
            assert f"{side}_hl_{finger}_tip" in link_names


@pytest.mark.parametrize("name", list(EXPECTED_CSPACE))
def test_manifest_matches_urdf(outputs, name):
    import yaml

    manifest = yaml.safe_load((outputs[name].parent / f"{name}_manifest.yaml").read_text())
    joints = gen.parse_urdf(outputs[name])
    revolute = [n for n, j in joints.items() if j["type"] == "revolute"]
    assert manifest["cspace_dim"] == len(revolute)
    assert manifest["cspace_joint_order"] == revolute
    assert manifest["robot_name"] == name


def test_directory_equals_filename_convention(outputs):
    """hdgp's get_robot_urdf_path requires <dir>/<dir>.urdf."""
    for name, path in outputs.items():
        assert path.parent.name == name and path.name == f"{name}.urdf"
        assert (gen.HDGP_FABRIC_DIR / name).is_dir(), "hdgp consumer dir missing"

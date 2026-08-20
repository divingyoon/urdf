"""Tests for the resting-pose self-collision audit.

Slow (~2 min for all assets): builds proximity indices for every collision
mesh. Deselect with `-k "not audit"` during quick iterations.
"""

from __future__ import annotations

import copy
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

import audit_self_collision as audit  # noqa: E402
import generate_rl_urdf as gen  # noqa: E402

RL_NAMES = list(gen.SOURCES.keys())


@pytest.fixture(scope="module", autouse=True)
def generated_outputs() -> None:
    assert gen.main(["--skip-audit"]) == 0


@pytest.mark.parametrize("name", RL_NAMES)
def test_audit_passes_for_generated_assets(name: str) -> None:
    findings = audit.audit_urdf(gen.OUT_DIR / f"{name}_rl.urdf")
    failures = [f for f in findings if f.verdict == "FAIL"]
    assert not failures, [
        (f.link_a, f.link_b, f.hull_depth_m, f.raw_depth_m, f.reason) for f in failures
    ]


def test_audit_detects_reintroduced_penetration(tmp_path: Path) -> None:
    """Regression guard: un-fixing the mount stack must FAIL the audit.

    Reverts both self-collision fixes on the right arm of the bi_s asset:
    stock link7 collision (gripper motor + bolts) and the adapter plate
    collision. The motor section then penetrates the hand mount again.
    """
    tree = ET.parse(gen.OUT_DIR / "openarm_tesollo_bi_s_rl.urdf")
    root = tree.getroot()
    stock_collision = str(
        gen.ASSET_ROOTS["openarm_description"] / "meshes" / "arm" / "v10" / "collision" / "link7_symp.stl"
    )
    for link in root.findall("link"):
        if link.get("name") == "r_al_7":
            mesh = link.find("collision/geometry/mesh")
            assert mesh is not None
            mesh.set("filename", f"file://{stock_collision}")
        if link.get("name") == "r_hl_adapter":
            visual = link.find("visual")
            assert visual is not None and link.find("collision") is None
            collision = copy.deepcopy(visual)
            collision.tag = "collision"
            mesh = collision.find("geometry/mesh")
            assert mesh is not None
            mesh.set(
                "filename",
                mesh.get("filename", "").replace("/visual/", "/collision/").replace(
                    "rl_dg_mount.dae", "rl_dg_mount_c.STL"
                ),
            )
            link.append(collision)

    broken = tmp_path / "broken.urdf"
    tree.write(broken)
    failures = [f for f in audit.audit_urdf(broken) if f.verdict == "FAIL"]
    assert failures, "audit missed the reintroduced mount-stack penetration"
    failing_links = {link for f in failures for link in (f.link_a, f.link_b)}
    assert "r_al_7" in failing_links

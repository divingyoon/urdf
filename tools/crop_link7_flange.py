#!/usr/bin/env python3
"""Crop the flange bolts off the link7 collision mesh.

The cropped link7 mesh (stock-gripper motor removed) still carries the three
flange bolts, which protrude 4mm above the flange plate (raw z 608..612mm,
link-frame z 0.0495..0.0535). Physically they insert into the hand adapter
plate holes, but collision approximations (convex hull/decomposition) lose
those holes, so with articulation self-collision enabled the bolts permanently
penetrate the adapter and generate phantom repulsion forces.

This tool slices the collision mesh at the flange plate top so the collision
shape ends where the mounting plane is. Visual meshes keep the bolts. The
mesh origin is unchanged (no translation).

Outputs land in generated/rl/meshes/ and are consumed by generate_rl_urdf.py.
"""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ARM_MESH_DIR = ROOT / "vendor" / "openarm_description" / "meshes" / "arm" / "v10"
OUT_MESH_DIR = ROOT / "generated" / "rl" / "meshes"

SOURCE = ARM_MESH_DIR / "collision" / "link7_without_mat2_mat3_components00_03.stl"
OUTPUT = OUT_MESH_DIR / "link7_flange_cut.stl"

# Flange plate top in raw mesh units (mm): link-frame 0.0495 = (raw*0.001 - 0.5585).
FLANGE_TOP_RAW_MM = 608.0


def crop_mesh(source: Path, output: Path) -> None:
    """Slice off z > FLANGE_TOP_RAW_MM (the bolts); keep the mesh origin."""
    import trimesh

    if not source.is_file():
        raise FileNotFoundError(f"missing vendor mesh: {source}")

    mesh = trimesh.load(source, force="mesh")
    sliced = mesh.slice_plane(
        plane_origin=[0.0, 0.0, FLANGE_TOP_RAW_MM],
        plane_normal=[0.0, 0.0, -1.0],
        cap=True,
    )
    if sliced is None or len(sliced.faces) == 0:
        raise ValueError(f"slicing produced an empty mesh for {source}")

    zmax = sliced.bounds[1][2]
    if zmax > FLANGE_TOP_RAW_MM + 1e-3:
        raise ValueError(f"cropped mesh still extends above flange: zmax={zmax}")

    output.parent.mkdir(parents=True, exist_ok=True)
    sliced.export(output)
    print(f"cropped {source.relative_to(ROOT)} -> {output.relative_to(ROOT)} "
          f"(zmax {zmax:.3f} mm)")


def ensure_link7_flange_mesh() -> Path:
    """Create the bolt-free collision mesh when missing or stale."""
    if not OUTPUT.is_file() or OUTPUT.stat().st_mtime < SOURCE.stat().st_mtime:
        crop_mesh(SOURCE, OUTPUT)
    return OUTPUT


def main() -> int:
    crop_mesh(SOURCE, OUTPUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

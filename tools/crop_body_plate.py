#!/usr/bin/env python3
"""Crop the vendor 8mm mount plate off the OpenArm body_link0 meshes.

The vendor meshes put z=0 at the bottom of an 8mm mount plate. The real robot
uses a different mount thickness, so the RL assets define the robot origin at
the plate TOP instead. This tool slices away everything below z=8mm and
translates the result down by 8mm so the new mesh origin sits at the plate top.

Outputs land in generated/rl/meshes/ and are consumed by generate_rl_urdf.py.
"""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BODY_MESH_DIR = ROOT / "vendor" / "openarm_description" / "meshes" / "body" / "v10"
OUT_MESH_DIR = ROOT / "generated" / "rl" / "meshes"

# Vendor plate: z in [0, 8] mm. New origin = plate top.
PLATE_TOP_MM = 8.0

CROP_JOBS = {
    "collision": (
        BODY_MESH_DIR / "collision" / "body_link0_symp.stl",
        OUT_MESH_DIR / "body_link0_symp_cut.stl",
    ),
    "visual": (
        BODY_MESH_DIR / "visual" / "body_link0.stl",
        OUT_MESH_DIR / "body_link0_visual_cut.stl",
    ),
}


def crop_mesh(source: Path, output: Path) -> None:
    """Slice off z < PLATE_TOP_MM and shift the mesh down by PLATE_TOP_MM."""
    import trimesh

    if not source.is_file():
        raise FileNotFoundError(f"missing vendor mesh: {source}")

    mesh = trimesh.load(source, force="mesh")
    sliced = mesh.slice_plane(
        plane_origin=[0.0, 0.0, PLATE_TOP_MM],
        plane_normal=[0.0, 0.0, 1.0],
        cap=True,
    )
    if sliced is None or len(sliced.faces) == 0:
        raise ValueError(f"slicing produced an empty mesh for {source}")

    sliced.apply_translation([0.0, 0.0, -PLATE_TOP_MM])

    zmin, zmax = sliced.bounds[0][2], sliced.bounds[1][2]
    if zmin < -1e-3:
        raise ValueError(f"cropped mesh still extends below origin: zmin={zmin}")

    output.parent.mkdir(parents=True, exist_ok=True)
    sliced.export(output)
    print(f"cropped {source.relative_to(ROOT)} -> {output.relative_to(ROOT)} "
          f"(z range [{zmin:.3f}, {zmax:.3f}] mm)")


def ensure_cropped_meshes() -> dict[str, Path]:
    """Create cropped meshes when missing or stale. Returns output paths by kind."""
    outputs: dict[str, Path] = {}
    for kind, (source, output) in CROP_JOBS.items():
        if not output.is_file() or output.stat().st_mtime < source.stat().st_mtime:
            crop_mesh(source, output)
        outputs[kind] = output
    return outputs


def main() -> int:
    for source, output in CROP_JOBS.values():
        crop_mesh(source, output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

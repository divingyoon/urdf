# OpenArm End-Effector URDF Workspace

This workspace is for composing OpenArm arms with multiple end-effectors and generating RL-only URDFs with a stable naming and action-order interface.

The important separation is:

- Source/vendor descriptions keep their original names for ROS2, drivers, and hardware bringup.
- RL URDFs are generated artifacts with canonical names for Isaac/RL training.
- The RL agent should use the generated RL URDF plus its manifest, not the hardware/source URDF directly.

## Current Structure

```text
/home/user/rl_ws/urdf
├── assemblies/                 # Local composition xacro files
├── config/                     # Local controller configs
├── docs/                       # Plans and notes
├── eef/                        # Local end-effector xacro wrappers
├── generated/
│   ├── rl/                     # RL-only canonical URDFs and manifests
│   └── source/                 # Stable generated/source URDF inputs
├── launch/                     # Local ROS2 launch files
├── previews/                   # Visual/debug URDF previews
├── scripts/                    # Shell helpers
├── tools/                      # Generation/maintenance tools
├── vendor/                     # Imported/reference robot packages
│   ├── openarm_description/    # OpenArm ROS description package
│   ├── delto_m_ros2/           # Tesollo/Delto ROS2 packages
│   ├── RH56F1/                 # RH56F1 source hand packages and assets
│   └── openarm_real/           # OpenArm real hardware packages
├── build/                      # colcon build output
├── install/                    # colcon install output
└── log/                        # colcon log output
```

### Why Vendor Packages Are Under `vendor/`

Imported/reference packages are grouped under `vendor/` so local composition files and generated training assets are not mixed with upstream source trees. Launch files and helper scripts use explicit `vendor/...` filesystem paths where needed. Xacro files still use ROS package names such as `$(find openarm_description)`, so source `/home/user/rl_ws/urdf/install/setup.bash` or rebuild the workspace before processing those xacros.

## Directory Details

### `assemblies/`

Local robot compositions live here.

- `openarm_modular_dual.xacro` - OpenArm dual-arm body with left stock gripper and right Tesollo hand.
- `openarm_modular_dual_tesollo.xacro` - OpenArm dual-arm body with Tesollo hands on both sides.
- `openarm_left_gripper_bimanual_real.xacro` - Real-hardware OpenArm control description with left stock gripper and right hand controlled separately.

These are composition sources. They are not the preferred RL training inputs.

### `eef/`

Local wrapper xacros for end-effectors.

- `tesollo_left_wrapper.xacro`
- `tesollo_right_wrapper.xacro`
- `gripper_left.xacro`

Wrappers add stable helper frames around vendor end-effector descriptions. They are useful for source composition, but final RL naming is handled by `tools/generate_rl_urdf.py`.

### `generated/source/`

Stable source/generated URDFs used as inputs to the RL canonical generator.

- `openarm_tesollo_sensor.urdf`
- `openarm_tesollo_bi.urdf`
- `openarm_bi_rh56f1.urdf`
- `openarm_modular_dual.urdf`
- `openarm_bimanual_no_mount.urdf`

The first three are the currently validated structures and are used by the RL generator.

### `generated/rl/`

RL-only canonical outputs.

- `openarm_tesollo_sensor_rl.urdf`
- `openarm_tesollo_sensor_rl_manifest.yaml`
- `openarm_tesollo_bi_rl.urdf`
- `openarm_tesollo_bi_rl_manifest.yaml`
- `openarm_bi_rh56f1_rl.urdf`
- `openarm_bi_rh56f1_rl_manifest.yaml`

Use these for training.

### `previews/`

Scratch/debug URDF files for visual inspection.

- `link7_material_preview.urdf`
- `link7_mat3_component_preview.urdf`
- `openarm_bimanual_link7_parts_preview.urdf`

Do not use these as training inputs.

## CLI Usage

Run commands from anywhere unless noted.

### Generate all RL URDFs

```bash
python3 /home/user/rl_ws/urdf/tools/generate_rl_urdf.py
```

This generates all validated RL assets under `generated/rl/`.

Expected outputs:

```text
generated/rl/openarm_tesollo_sensor_rl.urdf
generated/rl/openarm_tesollo_sensor_rl_manifest.yaml
generated/rl/openarm_tesollo_bi_rl.urdf
generated/rl/openarm_tesollo_bi_rl_manifest.yaml
generated/rl/openarm_bi_rh56f1_rl.urdf
generated/rl/openarm_bi_rh56f1_rl_manifest.yaml
```

### Generate one RL URDF

```bash
python3 /home/user/rl_ws/urdf/tools/generate_rl_urdf.py openarm_tesollo_bi
python3 /home/user/rl_ws/urdf/tools/generate_rl_urdf.py openarm_tesollo_sensor
python3 /home/user/rl_ws/urdf/tools/generate_rl_urdf.py openarm_bi_rh56f1
```

Valid source names:

```text
openarm_tesollo_sensor
openarm_tesollo_bi
openarm_bi_rh56f1
```

### Validate the generator syntax

```bash
python3 -m py_compile /home/user/rl_ws/urdf/tools/generate_rl_urdf.py
```

### Inspect action order

```bash
sed -n '/^control_joint_order:/,/^kinematic_joint_order:/p'   /home/user/rl_ws/urdf/generated/rl/openarm_tesollo_bi_rl_manifest.yaml
```

### Inspect full kinematic order

```bash
sed -n '/^kinematic_joint_order:/,/^fixed_joint_order:/p'   /home/user/rl_ws/urdf/generated/rl/openarm_bi_rh56f1_rl_manifest.yaml
```

### Visualize the modular source xacro

```bash
/home/user/rl_ws/urdf/scripts/run_openarm_modular_dual.sh
```

The script uses:

```text
/home/user/rl_ws/urdf/launch/display_openarm_modular_dual.launch.py
```

If `install/setup.bash` is missing, the helper builds packages from:

```text
/home/user/rl_ws/urdf/vendor/openarm_description
/home/user/rl_ws/urdf/vendor/delto_m_ros2/dg_description
```

That launch file points to:

```text
/home/user/rl_ws/urdf/assemblies/openarm_modular_dual.xacro
```

### Real hardware bringup source path

The real-hardware launch path is:

```text
/home/user/rl_ws/urdf/launch/openarm_left_gripper_right_dg5_real.launch.py
```

It uses:

```text
/home/user/rl_ws/urdf/assemblies/openarm_left_gripper_bimanual_real.xacro
```

## RL Naming Scheme

The canonical RL schema uses compact side/type prefixes.

### Side Prefix

```text
r_ = right
l_ = left
```

### OpenArm Body and Arms

```text
body_root      # stage-level root link with no geometry
body_link      # OpenArm physical base link
body_j_base    # body_root/body fixed joint

r_al_0..7      # right arm links
l_al_0..7      # left arm links

r_aj_base      # fixed body -> right arm base joint
l_aj_base      # fixed body -> left arm base joint

r_aj_1..7      # right arm actuated joints
l_aj_1..7      # left arm actuated joints
```

### End-Effector Links and Joints

```text
r_hl_*         # right hand links
l_hl_*         # left hand links

r_hj_*         # right hand joints
l_hj_*         # left hand joints
```

Examples:

```text
r_hj_mount
r_hj_base
r_hj_palm
r_hj_palm_sensor
r_hj_thumb_1
r_hj_thumb_2
r_hj_thumb_sensor
r_hj_thumb_tip
r_hj_pinky_1
r_hj_pinky_tip
```

The generator preserves fixed joints for mount/base/palm/sensor/tip structure. These are important for real sensor interpretation and consistent observation frames even when they are not action-controlled.

## Action Order

The RL action order is defined by each manifest's `control_joint_order`, not by relying on parser-specific URDF ordering.

The order is always:

```text
right arm -> right hand movable joints -> left arm -> left hand movable joints
```

For Tesollo bimanual:

```text
r_aj_1..7
r_hj_thumb_1..4
r_hj_index_1..4
r_hj_middle_1..4
r_hj_ring_1..4
r_hj_pinky_1..4
l_aj_1..7
l_hj_thumb_1..4
l_hj_index_1..4
l_hj_middle_1..4
l_hj_ring_1..4
l_hj_pinky_1..4
```

For RH56F1 bimanual, mimic joints are excluded from `control_joint_order` and retained in `kinematic_joint_order`.

Current RH56F1 control order is:

```text
r_aj_1..7
r_hj_thumb_1
r_hj_thumb_2
r_hj_index_1
r_hj_middle_1
r_hj_ring_1
r_hj_pinky_1
l_aj_1..7
l_hj_thumb_1
l_hj_thumb_2
l_hj_index_1
l_hj_middle_1
l_hj_ring_1
l_hj_pinky_1
```

## Generation Principle

The generator follows a source-preserving pipeline.

```text
vendor/source URDF
    -> parse XML
    -> build source-to-canonical link map
    -> build source-to-canonical joint map
    -> rename URDF into RL-only canonical schema
    -> reorder top-level links/joints for readability
    -> validate uniqueness and parent/child references
    -> write RL URDF
    -> write manifest with action and kinematic order
```

### What the Generator Changes

- Robot name gets `_rl` suffix.
- OpenArm body, arm links, and arm joints are renamed to canonical names.
- Tesollo and RH56F1 end-effector links/joints are renamed to canonical hand names.
- Fixed base, palm, sensor, and tip joints are kept and renamed.
- Mimic joints remain in the URDF and kinematic order.
- Mimic joints are excluded from action control order.

### What the Generator Does Not Change

- It does not modify source/vendor URDFs.
- It does not change meshes, inertials, limits, origins, axes, mimic tags, or geometry.
- It does not make hardware/control URDFs use RL names.
- It does not infer a single action size across hands with different actuation models.

## Manifests

Each manifest has these sections.

```yaml
source_urdf: generated/source/openarm_tesollo_bi.urdf
generated_urdf: generated/rl/openarm_tesollo_bi_rl.urdf
control_joint_order:
  - r_aj_1
  - r_aj_2
kinematic_joint_order:
  - body_j_base
  - r_aj_base
fixed_joint_order:
  - body_j_base
source_to_canonical_joints:
  openarm_right_joint1: r_aj_1
source_to_canonical_links:
  openarm_right_link1: r_al_1
```

Use `control_joint_order` for action vector indexing. Use `kinematic_joint_order`, `fixed_joint_order`, and `source_to_canonical_*` for debugging, observation mapping, and sensor/frame interpretation.

## Recommended Training Inputs

Use one of:

```text
/home/user/rl_ws/urdf/generated/rl/openarm_tesollo_sensor_rl.urdf
/home/user/rl_ws/urdf/generated/rl/openarm_tesollo_bi_rl.urdf
/home/user/rl_ws/urdf/generated/rl/openarm_bi_rh56f1_rl.urdf
```

And always load the matching manifest:

```text
/home/user/rl_ws/urdf/generated/rl/openarm_tesollo_sensor_rl_manifest.yaml
/home/user/rl_ws/urdf/generated/rl/openarm_tesollo_bi_rl_manifest.yaml
/home/user/rl_ws/urdf/generated/rl/openarm_bi_rh56f1_rl_manifest.yaml
```

## Adding a New End-Effector

1. Add or import the source end-effector URDF/xacro under its vendor/source package.
2. Create a local wrapper under `eef/` if helper frames are needed.
3. Create a composition xacro under `assemblies/`.
4. Generate or save the stable source URDF under `generated/source/`.
5. Add a source entry and mapping rules to `tools/generate_rl_urdf.py`.
6. Run the generator (`python3 tools/generate_rl_urdf.py`). It also runs the
   self-collision audit (`tools/audit_self_collision.py`) - a FAIL means the
   asset interpenetrates at rest and must be fixed, not skipped.
7. Verify the new manifest's `control_joint_order` matches the desired action vector.
8. Build the USD headlessly (no GUI import):
   `/home/user/rl_ws/IsaacLab/isaaclab.sh -p tools/build_usd.py <asset> [--sync-hdgp]`.
   Import settings (convexDecomposition colliders, unmerged fixed joints, fixed
   base) are pinned in the script and contract-checked against the manifest.
9. If Fabrics needs the robot, add a variant to `tools/gen_fabric_urdfs.py`
   and run it (`[--sync-hdgp]`); every variant is FK-gated against its RL URDF.

## Notes

- `pinky` is the canonical name. Source names like `little` are mapped to `pinky`.
- Source typos such as RH56F1 `plam` are mapped to canonical `palm` names.
- XML order is made readable, but the manifest is the authoritative action-order source.

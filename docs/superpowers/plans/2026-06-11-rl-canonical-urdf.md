# RL Canonical URDF Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate RL-only URDFs with canonical body, arm, hand, base, palm, sensor, and action joint names while preserving source URDFs for real hardware/control.

**Architecture:** Keep vendor/source URDFs unchanged. Add a generator that parses the stable source URDFs, renames links/joints into the canonical RL schema, writes generated URDFs under `generated/rl`, and writes a manifest with `control_joint_order` and `kinematic_joint_order`.

**Tech Stack:** Python 3 standard library, URDF XML, YAML-compatible text manifests.

---

### Task 1: Add Canonical RL URDF Generator

**Files:**
- Create: `/home/user/rl_ws/urdf/tools/generate_rl_urdf.py`
- Output: `/home/user/rl_ws/urdf/generated/rl/*_rl.urdf`
- Output: `/home/user/rl_ws/urdf/generated/rl/*_rl_manifest.yaml`

- [x] **Step 1: Implement XML renaming without changing source URDFs**

The generator maps OpenArm body/base/arm names, Tesollo hand names, RH56F1 hand names, and stock gripper names to canonical RL names.

- [x] **Step 2: Emit stable action order**

`control_joint_order` is always right arm, right hand movable joints, left arm, left hand movable joints. Fixed base/palm/sensor joints are excluded from action control.

- [x] **Step 3: Emit kinematic order**

`kinematic_joint_order` includes body/base, arms, hand mount/base/palm/sensor/tip fixed joints, and movable joints.

### Task 2: Validate Outputs

**Files:**
- Read: `/home/user/rl_ws/urdf/generated/rl/*_rl.urdf`
- Read: `/home/user/rl_ws/urdf/generated/rl/*_rl_manifest.yaml`

- [ ] **Step 1: Run generator**

Run: `python3 /home/user/rl_ws/urdf/tools/generate_rl_urdf.py`

Expected: three RL URDFs and three manifests are generated.

- [ ] **Step 2: Validate uniqueness and references**

Run: `python3 /home/user/rl_ws/urdf/tools/generate_rl_urdf.py` again.

Expected: no duplicate link/joint names, no missing parent/child links, no fixed or mimic joints in `control_joint_order`.

### Task 3: Document Folder Classification

**Files:**
- Create: `/home/user/rl_ws/urdf/URDF_LAYOUT.md`

- [ ] **Step 1: Document current categories**

Record which directories are source packages, local composition files, generated RL assets, previews, launches/config, and build outputs.

- [ ] **Step 2: Avoid breaking ROS package discovery**

Keep `openarm_description`, `delto_m_ros2`, `RH56F1`, and `openarm_real` in place unless a compatibility symlink/package path update is done later.

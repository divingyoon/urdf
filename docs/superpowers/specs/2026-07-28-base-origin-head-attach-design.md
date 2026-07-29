# Base Origin Fix (+8mm) & D435i Head Attachment — Design

Date: 2026-07-28
Status: Approved (사용자 승인)

## 배경

- vendor `body_link0` STL은 바닥(z=0)이 8mm 마운트 플레이트 하면. 실제 마운트는 15mm 두께.
- 로봇 원점을 **마운트 상면**으로 정의하면 마운트 두께와 무관하게 원점이 일정해짐.
- `vendor/head_realsense_d435i` (D435i pan/tilt head)를 모든 RL URDF에 부착 필요.

## 결정 사항 (사용자 확인 완료)

1. 원점 = 플레이트 상면 (vendor 원점 대비 +8mm). 팔 부착 z: 0.698 → 0.690 m.
2. 플레이트 지오메트리(z<8mm)는 메시에서 잘라냄 (원점 아래 충돌 지오메트리 없음).
3. head pan/tilt 조인트는 revolute 유지, `control_joint_order` 제외 (`kinematic_joint_order`에만).
4. head 부착: `body_link` 기준 xyz=(0, 0, 0.750), rpy=(0,0,0).

## 구현

구현 위치: RL 변환 단계 (`tools/generate_rl_urdf.py`). 소스 URDF/vendor는 무수정.

### 1. 메시 크롭 — `tools/crop_body_plate.py`

- trimesh로 z=8mm 평면에서 슬라이스 후 -8mm 평행이동, `generated/rl/meshes/`에 저장:
  - collision: `body_link0_symp.stl` → `body_link0_symp_cut.stl`
  - visual: `body_link0.stl` → `body_link0_visual_cut.stl` (vendor .dae 대신 사용, 재질 색 손실 허용)
- 결과 z 범위 ≈ [0, 765] mm.

### 2. `generate_rl_urdf.py` 확장

- `body_link`: visual/collision 메시를 크롭 메시(file:// 절대경로)로 교체, inertial origin z -= 0.008.
- `r_aj_base`/`l_aj_base` origin z -= 0.008 (0.698 → 0.690).
- head 병합 (3개 RL URDF 전부):
  - 링크: `head_base`, `head_mid`, `head_camera`
  - 조인트: `head_j_mount` (fixed, body_link→head_base, z=0.750), `head_j_pan`, `head_j_tilt` (revolute)
  - 메시 경로 file:// 절대경로 변환. manifest 스키마/매핑에 head 반영.

### 3. 테스트 — `tools/tests/test_generate_rl_urdf.py`

- 3개 URDF: 팔 부착 z=0.690, head 링크/조인트 존재, head_j_mount z=0.750.
- pan/tilt가 control_joint_order에 없고 kinematic_joint_order에 있음.
- 크롭 메시 z 범위 [0−ε, 765+ε], 원점 아래 지오메트리 없음.
- 기존 control_joint_order 불변 (action 공간 보존).

## 비고

- `urdf/.git`이 빈 디렉터리라 git 커밋 불가 (저장소 아님).
- 실배치 시 로봇 루트를 15mm 마운트 상면에 스폰하면 오차 없음.

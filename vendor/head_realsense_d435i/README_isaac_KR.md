# openarm D435i Head — Isaac Sim / Isaac Lab 에셋 가이드

Intel RealSense **D435i** 카메라를 **XC330 다이나믹셀 서보 2개**로 pan/tilt 하는 로봇 헤드입니다.
Fusion 360 원본: `rl_ws / d435i / head v5`.

핵심 원칙: **회전축(모터) 2개를 제외한 모든 부품은 3개의 강체(rigid link)로 병합**되어 있습니다.
"모터 빼고 하나의 덩어리"가 정확히는 **모터가 2개이므로 3덩어리**가 됩니다.

## 구조 (2-DOF 아티큘레이션)

```
base_link  ──[joint_pan: Z]──▶ mid_link  ──[joint_tilt: Y]──▶ camera_link
(팔에 고정)      XC330 #1                     XC330 #2            (D435i)
```

| 링크 | 병합된 부품 | 질량 | 움직임 |
|------|-------------|------|--------|
| **base_link** | neck1 + neck2_r×2 + fpx330-s101×2 + xc330#1 하우징 | 109.6 g | 고정 (루트) |
| **mid_link** | fpx330-s102 + xc330#2 하우징 | 17.1 g | `joint_pan` (Z축, 높이 38.5 mm)로 회전 |
| **camera_link** | fpx330-h101 + camera_fix + D435i | 65.1 g | `joint_tilt` (Y축, 높이 66 mm)로 회전 |

> 각 서보의 **하우징은 부모 링크에 붙고, 출력축이 자식 링크를 돌립니다** (물리적으로 정확).
> `neck2_r`, `fpx330-s101`, `xc330`의 수많은 작은 볼트/브래킷 바디는 링크당 하나의 solid 메시로 병합됩니다.

## 파일 구성

```
isaac/
├── README_isaac_KR.md
├── load_in_isaac.py                # USD 로딩 + pan/tilt 스윕 예제
├── head_data.json                  # 추출 원천값(질량/COM/관성/조인트) — 재생성용
├── meshes/
│   ├── visual/                     # 렌더용 OBJ (스무스 노멀 포함, mm, 링크로컬)
│   │   ├── head_base.obj  head_mid.obj  head_camera.obj
│   │   └── head_full.obj           # 3링크 병합(월드 좌표, 참고용)
│   └── collision/                  # 충돌용 STL (mm, 링크로컬)
│       └── head_base.stl  head_mid.stl  head_camera.stl
├── urdf/
│   └── head.urdf                   # 3링크 + 2 revolute (radian)
└── usd/                            # ★ physics 완비 USD
    ├── head_base.usda              # base 강체 (mesh + mass/inertia + convexDecomp collision)
    ├── head_mid.usda               # mid  강체
    ├── head_camera.usda            # camera 강체
    ├── head.usda                   # ★ 아티큘레이션: 3링크 참조 + pan/tilt 조인트 + 드라이브
    └── config/
        ├── physics.yaml            # 이식용 physics·조인트·드라이브 설정(원천값)
        └── head_cfg.py             # Isaac Lab ArticulationCfg
```

### 좌표/단위 규칙
- **링크 프레임**: `base`=월드 원점, `mid`=pan 조인트 위치, `camera`=tilt 조인트 위치.
  URDF와 USD가 **동일한 링크 프레임**을 씁니다. 메시는 각 링크 로컬 좌표로 baked.
- 메시(OBJ/STL) = **mm** → URDF `scale="0.001"` 로 m 변환. USD points는 이미 **m**.
- `upAxis = Z`, `metersPerUnit = 1`.
- **각도 단위 주의**: URDF 조인트 한계 = **radian**, UsdPhysics 조인트 한계 = **degree**.

## physics 정보는 어디에 저장되는가
| 파일 | 저장된 physics |
|------|----------------|
| Fusion 문서 / STL | 없음 (형상만) |
| URDF | 링크별 질량·관성 + 조인트 축/한계 (충돌 근사는 임포트 시 지정) |
| **`.usda`** | **전부** — RigidBody, Mass/COM/관성, convexDecomposition 충돌, 조인트 + 드라이브, articulation root |

→ Isaac에서 **`usd/head.usda`** 를 참조하면 별도 세팅 없이 2-DOF 헤드가 바로 나옵니다.

## Isaac 로딩

### A) USD 직접 (권장, 가장 간단)
```python
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation
add_reference_to_stage(usd_path="usd/head.usda", prim_path="/World/Head")
head = Articulation(prim_path="/World/Head")
# joint_pan / joint_tilt 를 position target 으로 제어. 전체 예제는 load_in_isaac.py.
```

### B) Isaac Lab (ArticulationCfg)
```python
from usd.config.head_cfg import HEAD_CFG
from isaaclab.assets import Articulation
head = Articulation(HEAD_CFG.replace(prim_path="/World/Head"))
```

### C) URDF 임포트 경로
1. Isaac URDF Importer 로 `urdf/head.urdf` → USD 변환
   (Fix Base = true[단독] / false[팔에 장착], Merge Fixed Joints = on).
2. 두 조인트에 drive(stiffness/damping) 설정 후 사용.

## 팔(openarm)에 장착하기
`head.usda` 안의 **`fix_base_to_world`** 고정 조인트가 `base` 를 월드에 붙여둔 상태입니다.
팔 말단에 붙이려면:
- 이 고정 조인트를 **삭제**하거나 `body0` 를 팔의 tool 링크로 **재지정**하고,
- 팔의 end/tool 링크 → `/Head/base` 로 fixed joint 를 추가.
- Isaac Lab 이면 `head_cfg.py` 의 `fix_root_link=False` 로 두고 팔 아티큘레이션에 결합.

## ⚠️ 반드시 직접 확인/수정할 값 (PLACEHOLDER)
Fusion 원본에 조인트 한계/모터 스펙이 설정돼 있지 않아 **기본값을 넣어 두었습니다.**
실제 값으로 바꾸세요:

| 항목 | 현재 기본값 | 위치 |
|------|-------------|------|
| pan 한계 | ±90° (±1.5708 rad) **← PLACEHOLDER** | urdf `joint_pan/limit`, usd `joint_pan lower/upperLimit`, physics.yaml |
| tilt 한계 | ±90° (±1.570796 rad) **← CAD 설정값 반영됨** | 〃 `joint_tilt` |
| 모터 토크 | 0.9 N·m (XC330-M288 stall 근사) | `effort`, `maxForce`, `head_cfg.py effort_limit` |
| 속도 | 6.0 rad/s | `velocity`, `head_cfg.py velocity_limit` |
| 드라이브 게인 | stiffness/damping (임의) | usd 드라이브, `head_cfg.py` |

### 축 방향 참고
- **pan**: Fusion 축 = `(0,0,-1)`. URDF는 그대로 `axis="0 0 -1"`. USD는 `"Z"`(+Z)라 **부호가 반대**로 들어갑니다 (대칭 한계라 기능상 무방, 필요하면 `localRot`로 뒤집기).
- **tilt**: 축 = `(0,1,0)` → URDF `0 1 0`, USD `"Y"` 로 일치.

## 물성 요약 (Fusion 실측, 속 빈 실제 형상)
| 링크 | 질량(kg) | COM(링크프레임, m) | 대각 관성(kg·m², COM) |
|------|----------|--------------------|------------------------|
| base | 0.109629 | (0.003423, 0.0, 0.003366) | (4.382e-5, 7.657e-5, 8.819e-5) |
| mid | 0.017073 | (0.0, 0.0002, 0.019222) | (2.817e-6, 2.429e-6, 1.447e-6) |
| camera | 0.065101 | (-0.000717, 0.014670, 0.032665) | (3.929e-5, 1.017e-5, 3.749e-5) |

- 관성은 **COM 기준 월드정렬 대각** 근사(비대각 곱관성은 작아 생략, `principalAxes=(1,0,0,0)`).
- 재질/질량이 다르면 관성은 질량비로 스케일.

## 충돌
- 링크는 컨테이너가 아니므로 **convexDecomposition** 사용 (SDF 불필요).
- 충돌 메시가 조밀(2만~3만 tri)합니다 → PhysX가 로드 시 볼록분해로 단순화. 더 가볍게 하려면 STL/OBJ를 decimate.

## 원본 편집이 필요하면
Fusion `head v5` 에서 형상/조인트 수정 후 동일 파이프라인으로 재-export
(`head_data.json` 의 값이 원천). 조인트를 추가/변경하면 링크 그룹 매핑도 함께 갱신하세요.

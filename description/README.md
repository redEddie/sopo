# description — sopo 로봇 모델 (URDF)

Onshape CAD에서 export한 로봇 기술(robot description)과 그 후처리 파이프라인.

**오프라인 우선**: 이 폴더의 산출물(URDF, 메시, 뷰어 모델)은 전부 커밋되어 있어
Onshape API 없이 그대로 사용할 수 있다. 재export는 CAD를 수정했을 때만 필요하다.

## 산출물

| 파일 | 내용 | 관절 |
|---|---|---|
| `robot.urdf` | 전체 모델 (팔 + 그리퍼). 시각화 기준 | joint_1~6 + 핑거 2 (핑거는 mimic) |
| `arm_no_ee.urdf` | 팔만. 빈 `flange` 프레임 링크로 종료 (Franka link8 컨벤션). 제어/중력보상 기준 | joint_1~6 |
| `sopo_viewer.xml` | MuJoCo simulate 뷰어용 MJCF (중력/접촉 off, position actuator 포함) | — |
| `assets/` | 링크별 STL. 시각용 원본 + `*_collision.stl`(축소 충돌용) | — |

모든 URDF는 **커밋된 생성물**이다. 재생성 가능하지만(아래 파이프라인) 그대로 써도 된다.

## 파이프라인

```
Onshape 어셈블리 ──onshape-to-robot──▶ robot.urdf(raw) ──postprocess.py──▶ robot.urdf
                                                          │                  arm_no_ee.urdf
                                                          │                  sopo_viewer.xml
                                                          └ assets/*.stl (시각/충돌 이원화)
```

### 재export (CAD 변경 시에만 — Onshape API 키 필요)

```bash
pip install -e ".[description]"
# configs/onshape.env에 API 키 기입 후
set -a && source ../configs/onshape.env && set +a
onshape-to-robot .          # export 후 postprocess.py가 자동 실행된다 (config.json 참조)
```

- `config.json`의 `url`은 어셈블리 탭 URL. 재현성을 위해 workspace(`/w/`)가 아닌
  버전(`/v/`) URL을 쓰는 것을 권장.
- Onshape 규칙: 관절 mate 이름은 `dof_xxx` (export되면 `xxx`가 조인트명이 됨).
  revolute mate에 리밋을 걸면 URDF `<limit>`에 반영된다.

### 후처리만 다시 실행 (오프라인, 커밋된 robot.urdf 기준)

```bash
python postprocess.py     # 멱등 — 여러 번 실행해도 같은 결과
```

`postprocess.py`가 하는 일(상세는 파일 docstring): joint_6 continuous 변환,
`package://` 제거, 메시 시각/충돌 이원화, 핑거 mimic 추가, 그리퍼 관성 주입,
모터별 effort/velocity 주입(`JOINT_SPECS` 상수), arm_no_ee/뷰어 모델 생성.

### 보정 포인트

- **모터 스펙 변경**: `postprocess.py`의 `JOINT_SPECS` (정격 토크 기준, 듀얼 관절은 2배)
- **질량 실측 후**: `GRIPPER_MASS` 및 팔 링크 질량 (현재 그리퍼만 robonine 실측 기반 추정치)
- **관절 리밋**: Onshape mate 리밋이 소스. J1은 리밋 없으면 ±180°로 export됨

## 시각화

### MuJoCo 뷰어 (조인트 슬라이더, 로컬)

```bash
python viewer.py     # simulate UI: 스페이스바로 시작, Control 패널 슬라이더로 포징
```

### 웹 뷰어 (설치 불필요, 빠른 확인용)

[urdf-viewer](https://urdf-viewer-pi.vercel.app/) — 브라우저에 `robot.urdf`와
`assets/`를 드래그&드롭하면 조인트 슬라이더로 바로 확인할 수 있다
([GitHub, MIT](https://github.com/qkrdkwl9090/urdf-viewer), Three.js + NASA JPL의
urdf-loader + xacro-parser 기반).

우리 파이프라인과의 궁합: 이 뷰어는 메시 경로를 **드롭한 URDF 기준 상대경로**로 해석하는데,
postprocess가 `package://`를 제거해 상대경로로 바꿔두었기 때문에 그대로 동작한다.
ROS 패키지 경로(`package://`)를 쓰는 URDF는 이 뷰어에서 메시가 깨진다 — 우리가
상대경로를 고수하는 이유 중 하나다.

## 중력보상 준비 상태

- 모델: `arm_no_ee.urdf` (6-DOF, joint_6은 continuous)
- 그리퍼 관성: 주입됨 (robonine 실측 기반 추정치, `GRIPPER_MASS`)
- 팔 링크 관성: **미측정** — 실측 후 postprocess에 주입 단계를 추가한다
- 다음 단계: pinocchio 등으로 G(q) 계산 → Feetech 전류 지령으로 변환
  (토크-전류 환산은 cookbook/06, 07의 실측 절차 참고)

## 참고 레퍼런스

- `../docs/reference/franka_fr3/` — Franka FR3 URDF 2종 (팔만/핸드 포함). 플랜지 프레임 컨벤션의 원조
- [onshape-to-robot 문서](https://onshape-to-robot.readthedocs.io/)

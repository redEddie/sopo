# 3_safety_torque — 안전 토크

사람 근처에서 돌리기 전에, 관절마다 "자중을 버티는 데 실제로 필요한 토크"를 재서 캡을 정하고,
캡을 모터 EPROM에 새겨 어떤 코드가 돌아도 캡이 지켜지게 한다. 마지막으로 호스트 리플렉스가
정말 멈추는지 손으로 검증한다.

## 안전 수칙

### 토크 제한 원리

- `Torque_Limit`을 낮추면 모터가 낼 수 있는 최대 토크가 줄어든다.
- 쿡북 기본값은 300‰(정격의 30%)이다. 처음엔 낮게 시작하고 점진적으로 올린다.

### 과부하 보호 3종 레지스터

1. `Overload_Torque`: 이 값 이상의 부하가 감지되면 카운트를 시작한다.
2. `Protection_Time`: 설정한 시간(×10ms) 동안 과부하가 지속되면 보호 동작이 발생한다.
3. `Protective_Torque`: 동작 후 `Torque_Limit`이 이 값으로 강제 낮아진다.

즉, 과도한 외력이 일정 시간 이상 가해지면 모터가 스스로 힘을 줄여 손상을 막는다.

### 사람 주변 운용 수칙

- 토크를 켜기 전 반드시 `apply_safety()`로 안전 레지스터를 설정한다.
- 메인 루프는 `try/finally`로 감싸고, 종료 시 토크를 끈다.
- 위치 명령은 현재 위치를 먼저 읽고 `clamp_goal()`으로 한 번에 움직일 거리를 제한한다.
- 사람 손가락이 관절 사이에 들어가지 않도록 주의한다.
- 관절별 위치 한계와 토크 캡은 `2_pose_calibration/200_find_limits.py`/`300_gravity_load.py`로 실측해 `configs/calibration.yaml`에 기록한다. `1_setup/140_move_joint.py` 등 실행 스크립트가 이 파일이 있으면 자동으로 적용한다.

## 스크립트

### 300_gravity_load.py — 자중 토크 측정

```bash
python cookbook/3_safety_torque/300_gravity_load.py --port /dev/ttyACM0 --ids 19
python cookbook/3_safety_torque/300_gravity_load.py --port /dev/ttyACM0 --ids 15,16 --save
```

`Present_Load`는 모터 출력 듀티(‰)이므로, 정지 자세를 유지할 때의 값이 곧 중력을 버티는 토크 비율이다. 토크를 끈 상태에서 관절을 **최악 자세**(레버 수평, 아래 링크 완전 신전)에 손으로 놓고 Enter → 현재 위치를 목표로 잡고 토크 ON(`--hold-torque`, 기본 600) → `--seconds` 동안 부하/전류/처짐 측정 → 토크 OFF. 자세를 바꿔 반복 측정할 수 있고, 최댓값 × `--margin`(기본 1.5)을 권장 `Torque_Limit`으로 출력한다. `--save`로 `calibration.yaml`의 `torque_limits`에 기록한다.

주의: 정지 마찰이 부하 일부를 대신 버티므로 측정값은 실제 들어올리는 데 필요한 토크보다 작게 나온다 — 마진이 필요한 이유. 권장값은 `cookbook/1_setup/130_move_position.py --torque-limit <값>`으로 실제 들어올려지는지 검증한다.

### 310_torque_limits.py — 토크 제한 데모

```bash
python cookbook/3_safety_torque/310_torque_limits.py --port /dev/ttyACM0 --id 1
```

낮은 토크 한계로 현재 위치를 유지하면서 부하/전류를 출력한다. 관절을 손으로 밀면 수치가 포화되는 것을 확인할 수 있다. `--persist`를 주면 EPROM에 영구 저장하며, `yes` 입력을 요구한다.

### 320_persist_caps.py — 캡·리밋 EPROM 영구화

```bash
python cookbook/3_safety_torque/320_persist_caps.py --config configs/arm.yaml --dry-run   # 현재 EPROM vs 캘리브레이션 비교
python cookbook/3_safety_torque/320_persist_caps.py --config configs/arm.yaml             # 'yes' 후 기록
```

`calibration.yaml`의 `torque_limits`를 `Max_Torque_Limit`(EPROM)에, `position_limits`를 `Min/Max_Position_Limit`(EPROM)에 기록한다. `Torque_Limit`(RAM)은 전원을 켤 때 `Max_Torque_Limit`에서 복원되므로, 이후로는 어떤 스크립트가 `apply_safety()`를 잊어도 서보가 스스로 캡을 지킨다. 연속 관절(J1)의 펌웨어 위치 한계는 건드리지 않는다. 실행 스크립트(`jog.py`, `run_waypoints.py`)는 시작할 때 EPROM이 캘리브레이션과 다르면 경고한다(드리프트 검사).

### 330_reflex_check.py — 리플렉스 합격 판정

```bash
python cookbook/3_safety_torque/330_reflex_check.py --config configs/arm.yaml --joint J4 --delta 300 --cycles 10
```

Phase A: 손 대지 않고 왕복 10회 → 리플렉스 0건이면 합격(오탐 없음). Phase B: 왕복 중 관절을 손으로 잡음 → `[REFLEX] COLLISION …`이 0.5초 안에 뜨고, 홀드 후 손을 놓아도 밀지 않으며, `r`로 복구되면 합격. 실패 시 `ReflexConfig`(sat_ratio, t_collision, t_accel)나 캡을 조정한다. 블랙박스는 `logs/`에 남는다.

## 페이로드 안전망 (340 → 350 → 360)

수동 캡(30%) 중심에서 **능동 감지 중심**으로 안전 철학을 전환하는 파이프라인이다
(설계: `docs/payload-safety-requirements.md`). 완료하면 sopod가 전력 모드(균일 상향 캡 +
자세 의존 동적 임계)로 올라간다. 사전 조건: 2_pose_calibration의 230/231 + 4_torque_model의 410 --scale.

### 340_estimator_limits.py — 추정기 오차 한계 실측 (~20분)

```bash
python cookbook/3_safety_torque/340_estimator_limits.py --phase floor          # 정지 잔차 바닥 (최소 3자세)
python cookbook/3_safety_torque/340_estimator_limits.py --phase k --mass 0.5   # 알려진 추 5회 탈부착 → 복원률 → k
```

floor는 정지 잔차(노이즈+히스테리시스)의 |값| mean+3σ, k는 1/최저 복원률. 둘 다 `calibration.yaml`의
`estimation` 섹션에 저장된다. Present_Load는 세션 간 감도 변동이 있으니 **월 1회 또는 하드웨어 변경 시 재실행**.

### 350_define_payload.py — 최대 페이로드 선언 + 실현성 검사

```bash
python cookbook/3_safety_torque/350_define_payload.py 0.5     # kg. 버스 불필요 (순수 모델 계산)
```

자세 그리드(J2/J3 스윕)에서 최악 필요 duty를 계산해 전력 캡(`safety.power_torque_limit`, EPROM 상한)과 비교한다.
초과면 거부(--force 없이 저장 안 함), 90% 넘으면 경고. 통과하면 `arm.yaml`의 `payload.max_kg`에 기록한다
(이 스크립트만이 이 값을 기록한다). 포락선 미리보기도 출력한다.

### 360_payload_check.py — 합격시험 (sopod 클라이언트)

```bash
python cookbook/3_safety_torque/360_payload_check.py     # sopod 실행 + power_ok 상태에서
```

Phase A: m_max 추를 매달고 유지 + 서서히 이동 → 트립 0이면 합격. Phase B: m_max×k 초과 추 →
EXTERNAL_FORCE 트립 + 부착→REFLEX 지연 측정. PASS/FAIL을 출력한다.

## 완료 기준

- 전 관절 `torque_limits`가 `calibration.yaml`에 기록되고 320으로 EPROM에 영구화됐다 (`--dry-run` 무경고).
- 330의 Phase A 오탐 0건, Phase B 정지·홀드·복구 합격.
- 사람 근처 운용 시 캡 ≤ 400‰를 지킨다. → `4_torque_model`로.
- (페이로드 안전망) `estimation.floor`/`k` 기록 + 350 통과 + 360 양 Phase PASS → sopod 전력 모드.

# 4_torque_model — 토크 모델: 중력 검증과 튜닝

안전 캡이 잡히면 모터 부하 숫자를 물리 토크[N·m]와 대조한다. URDF 중력 모델 G(q)가 실측과 맞는지
검증하고 ‰↔토크 스케일을 보정하면, 이 값이 `sopo/model/estimation.py`의 외력 관측기(τ_ext)와
sopod의 EXTERNAL_FORCE 리플렉스 입력이 된다. PID/흔들림 튜닝도 여기서 숫자로 비교한다.

사전 조건: `2_pose_calibration`의 `gravity.zero_ticks`/`dir`이 잡혀 있을 것.

## 스크립트

### 400_verify_torque.py — 토크 캡 검증 왕복 이동

```bash
python cookbook/4_torque_model/400_verify_torque.py --port /dev/ttyACM0 --ids 19 --torque-limit 150
python cookbook/4_torque_model/400_verify_torque.py --port /dev/ttyACM0 --ids 15,16 --invert 16 --torque-limit 300 --delta 100
```

`3_safety_torque/300_gravity_load.py`가 낸 권장 캡을 걸고 `--delta`(기본 200틱)만큼 왕복하며 도달 시간, 오차, 이동 중 max load/전류를 표로 낸다. max load가 캡의 85% 이상이면 "가속 여유 부족"으로 표시하니 캡을 올린다. 듀얼 관절은 반전 모터를 `--invert`로 지정해야 두 모터가 같은 방향으로 관절을 돌린다.

### 410_gravity_check.py — 중력 모델 검증 + 스케일 보정 + 외력 추정

```bash
python cookbook/4_torque_model/410_gravity_check.py            # 자세별 측정 vs 모델 예측 비교
python cookbook/4_torque_model/410_gravity_check.py --scale 0.5  # 플랜지에 0.5kg 매달아 ‰↔토크 스케일 보정
```

URDF(실측 질량) 기반 중력 토크 G(q)와 실제 모터 부하(Present_Load)를 자세별로 비교한다. zero 기준은 `2_pose_calibration/230_calibrate_zero.py`에서 먼저 캡처할 것. 결과는 `calibration.yaml`의 gravity 섹션(scale)에 저장.

### 420_set_pid.py — 서보 PID 읽기/쓰기 (강성·진동 튜닝)

```bash
python cookbook/4_torque_model/420_set_pid.py --port /dev/ttyACM0 --ids 19,20,21          # 현재값
python cookbook/4_torque_model/420_set_pid.py --port /dev/ttyACM0 --ids 19 --p 48 --d 48  # 한 관절씩 시험
```

흔들림 대응 순서: ① `arm.yaml`의 `max_relative_target` 80→40, `acceleration` 30→15 (출발·정지 충격 감소) ② 듀얼 관절 `preload_ticks: 4` (두 모터가 서로 밀어 백래시 제거) ③ P/D 상향 (캡 안에서만 효과) ④ 캡 상향은 최후 — 캡이 곧 충돌 시 사람이 받는 힘. 근본적으로는 외력 추정(관측기)이 있어야 강성과 안전을 동시에 얻는다.

### 430_wobble_test.py — 흔들림 측정 (전/후 비교용)

```bash
python cookbook/4_torque_model/430_wobble_test.py --config configs/arm.yaml --move J2:+300,J3:+300 --cycles 3
```

대기 자세에서 지정 관절을 들었다 내리는 왕복을 반복하며 모터별로 밀림(`disturb_max/rms`: 다른 관절이 움직이는 동안 |목표−실측|)과 도착 후 진동(`settle_pp/osc/t`: 1초 창의 피크-투-피크·반전 횟수·정착 시간)을 표로 낸다. `preload_ticks`·PID·가속을 바꾸기 전후에 같은 명령으로 돌려 숫자를 비교한다.

## 완료 기준

- 410에서 자세별 측정 ‰와 모델 예측 ‰가 스케일 보정 후 어긋나지 않는다 (외력 열이 0 근처).
- `calibration.yaml`의 `gravity.scale`이 기록됐다 → sopod의 외력 관측기가 이 값을 쓴다.
- PID/preload 변경 전후를 430으로 숫자 비교해 채택했다.

# Sopo Feetech 쿡북

이 문서는 HopeJr 부품으로 제작한 커스텀 중형 로봇 암을 Feetech STS/SMS 시리얼 버스 서보(STS3215/STS3250)로 제어하기 위한 기초 가이드이다.

---

## 1. 하드웨어 개요

### Feetech STS 버스 서보

- 프로토콜 0, 4096 ticks/rev, 중심 2048
- 기본 통신 속도 1,000,000 bps
- 토크 제한은 정격 토크의 0~1000‰(0~100%)로 설정

### 배선

- 모든 서보를 3선(신호/전원/GND)으로 데이지체인 연결한다.
- 전원은 서보 전용 BEC/배터리를 사용하고, USB-시리얼 어댑터와 GND는 반드시 공통 접지한다.
- 신호선만 공유하고 전원은 별도 공급하는 것이 안전하다.

### 시리얼 어댑터

- `/dev/ttyACM0` 또는 `/dev/ttyUSB0` 등을 사용한다.
- Linux에서는 사용자가 `dialout` 그룹에 속해야 권한 문제 없이 열 수 있다.

---

## 2. 레지스터 기초

### EPROM vs SRAM

- **EPROM**(주소 < 40): 전원을 꺼도 유지된다. 쓰기 전 `bus.eprom_unlocked(id)`로 잠금을 풀어야 한다.
- **SRAM**(주소 ≥ 40): 전원 재인가 시 초기화된다. `Torque_Limit`은 전원 ON 시 `Max_Torque_Limit` 값으로 복원된다.
- `Lock`(55) 레지스터를 설정하면 EPROM 영역을 잠글 수 있다.

### 레지스터 표

| 이름 | 주소 | 설명 |
|------|------|------|
| ID | 5 | 모터 식별자 |
| Baud_Rate | 6 | 통신 속도 (0=1M, 1=500k, 2=250k, 3=128k, 4=115200, 5=57600, 6=38400, 7=19200) |
| Min_Position_Limit | 9 | 최소 위치 제한 |
| Max_Position_Limit | 11 | 최대 위치 제한 |
| Max_Torque_Limit | 16 | 전원 ON 시 Torque_Limit 초기값 |
| P_Coefficient | 21 | 위치 PID P 게인 |
| D_Coefficient | 22 | 위치 PID D 게인 |
| I_Coefficient | 23 | 위치 PID I 게인 |
| Protection_Current | 28 | 과전류 보호 기준 (6.5mA 단위) |
| Homing_Offset | 31 | 위치 오프셋 |
| Operating_Mode | 33 | 동작 모드 (0=위치, 1=속도, 2=PWM, 3=스텝) |
| Protective_Torque | 34 | 과부하 발생 후 낮춰지는 토크 한계 |
| Protection_Time | 35 | 과부하 판정 시간 (×10ms) |
| Overload_Torque | 36 | 과부하 판정 토크 한계 |
| Torque_Enable | 40 | 토크 ON/OFF |
| Acceleration | 41 | 가속도 (약 8.7°/s² 단위) |
| Goal_Position | 42 | 목표 위치 |
| Goal_Time | 44 | 목표 도달 시간 |
| Goal_Velocity | 46 | 목표 속도 |
| Torque_Limit | 48 | 현재 토크 한계 |
| Lock | 55 | EPROM 잠금 |
| Present_Position | 56 | 현재 위치 |
| Present_Velocity | 58 | 현재 속도 |
| Present_Load | 60 | 현재 부하 (‰) |
| Present_Voltage | 62 | 전압 (0.1V 단위) |
| Present_Temperature | 63 | 온도 (°C) |
| Status | 65 | 상태 플래그 |
| Moving | 66 | 이동 중 플래그 |
| Present_Current | 69 | 전류 (6.5mA 단위) |

---

## 3. 안전 수칙

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
- 관절별 위치 한계와 토크 캡은 `05_find_limits.py`/`06_gravity_load.py`로 실측해 `configs/calibration.yaml`에 기록한다. `08_move_joint.py` 등 실행 스크립트가 이 파일이 있으면 자동으로 적용한다.

---

## 4. 스크립트 사용법

### 00_scan.py — 버스 스캔

```bash
python cookbook/00_scan.py --port /dev/ttyACM0
```

무응답 ID마다 타임아웃을 기다리므로 기본은 ID 0~30만 스캔한다. 전체 검색은 `--max-id 253`.

출력 예시:

```text
      Baud |   ID | Model
------------------------------
   1000000 |    1 | sts3215
   1000000 |    2 | sts3215
    500000 |    1 | sts3215

[경고] 다음 ID가 여러 보드레이트에서 응답했습니다. ID 충돌 가능성:
  ID 1 @ 500000 (sts3215)
```

### 01_read_state.py — 상태 읽기

```bash
python cookbook/01_read_state.py --port /dev/ttyACM0 --ids 1,2,3
```

토크를 켜지 않고 10Hz로 위치/속도/부하/전압/온도/상태를 출력한다. `Ctrl+C`로 종료한다.

### 02_move_position.py — 위치 이동

```bash
python cookbook/02_move_position.py --port /dev/ttyACM0 --id 1 --goal 2500
```

`apply_safety()` 후 토크를 켜고, 현재 위치에서 `clamp_goal()`로 한 걸음씩 접근한다. 50Hz 루프, 10초 타임아웃, 오차 20 tick 이내에서 멈춘다. 소프트 리밋 밖의 목표는 리밋 값으로 잘라서 이동한다.

### 03_torque_limits.py — 토크 제한 데모

```bash
python cookbook/03_torque_limits.py --port /dev/ttyACM0 --id 1
```

낮은 토크 한계로 현재 위치를 유지하면서 부하/전류를 출력한다. 관절을 손으로 밀면 수치가 포화되는 것을 확인할 수 있다. `--persist`를 주면 EPROM에 영구 저장하며, `yes` 입력을 요구한다.

### 04_set_motor_id.py — 모터 ID 변경

```bash
python cookbook/04_set_motor_id.py --port /dev/ttyACM0 --current-id 1 --new-id 2
```

현재 ID를 아는 모터의 ID만 바꾼다. ID는 EPROM에 있어 쓰기 전 `Lock`을 해제해야 하고, 쓰는 즉시 적용되므로 이후 통신은 새 ID로 한다. 잠금 해제 → ID 쓰기 → 새 ID로 재잠금 → `ping` 검증 순서를 따른다.

보드레이트까지 같이 바꾸거나 버스에 모터가 딱 1개만 붙어 있어야 안심되는 상황이라면 아래 `04_setup_motor.py`를 사용한다.

### 04_setup_motor.py — ID/보드레이트 변경 (버스에 모터 1개)

```bash
python cookbook/04_setup_motor.py --port /dev/ttyACM0 --new-id 2
python cookbook/04_setup_motor.py --port /dev/ttyACM0 --current-id 1 --new-id 2 --new-baud 500000
```

버스에 모터가 정확히 1개만 연결되어 있어야 하며, 여러 모터가 있으면 거부한다. 현재 보드레이트가 1M이 아니면 `--baud`로 지정한다. ID/Baud_Rate는 쓰는 즉시 적용되므로 스크립트가 잠금 해제 → 보드레이트 변경(재접속) → ID 변경 → 새 ID로 재잠금 순서를 지키며, 변경 후 `ping`으로 검증한다.

### 05_find_limits.py — 관절 최소/최대 위치 기록 (손으로)

```bash
python cookbook/05_find_limits.py --port /dev/ttyACM0 --ids 19
python cookbook/05_find_limits.py --port /dev/ttyACM0 --ids 15,16 --write-eprom
```

토크를 끄고 관절을 손으로 양끝까지 움직이면 min/max를 실시간으로 기록한다. `Ctrl+C`로 끝내면 안쪽으로 `--margin`(기본 50틱)을 줄인 값을 `configs/calibration.yaml`의 `position_limits`에 저장한다. `--write-eprom`을 주면 모터의 `Min/Max_Position_Limit`(EPROM)에도 써서 펌웨어가 범위 밖 목표를 거부하게 만든다(소프트웨어 버그에 대한 2차 방어선). 듀얼 모터 관절은 두 ID를 함께 지정한다. 0/4095 랩 지점 근처면 경고한다.

### 06_gravity_load.py — 자중 토크 측정

```bash
python cookbook/06_gravity_load.py --port /dev/ttyACM0 --ids 19
python cookbook/06_gravity_load.py --port /dev/ttyACM0 --ids 15,16 --save
```

`Present_Load`는 모터 출력 듀티(‰)이므로, 정지 자세를 유지할 때의 값이 곧 중력을 버티는 토크 비율이다. 토크를 끈 상태에서 관절을 **최악 자세**(레버 수평, 아래 링크 완전 신전)에 손으로 놓고 Enter → 현재 위치를 목표로 잡고 토크 ON(`--hold-torque`, 기본 600) → `--seconds` 동안 부하/전류/처짐 측정 → 토크 OFF. 자세를 바꿔 반복 측정할 수 있고, 최댓값 × `--margin`(기본 1.5)을 권장 `Torque_Limit`으로 출력한다. `--save`로 `calibration.yaml`의 `torque_limits`에 기록한다.

주의: 정지 마찰이 부하 일부를 대신 버티므로 측정값은 실제 들어올리는 데 필요한 토크보다 작게 나온다 — 마진이 필요한 이유. 권장값은 `02_move_position.py --torque-limit <값>`으로 실제 들어올려지는지 검증한다.

### 07_verify_torque.py — 토크 캡 검증 왕복 이동

```bash
python cookbook/07_verify_torque.py --port /dev/ttyACM0 --ids 19 --torque-limit 150
python cookbook/07_verify_torque.py --port /dev/ttyACM0 --ids 15,16 --invert 16 --torque-limit 300 --delta 100
```

06에서 나온 권장 캡을 걸고 `--delta`(기본 200틱)만큼 왕복하며 도달 시간, 오차, 이동 중 max load/전류를 표로 낸다. max load가 캡의 85% 이상이면 "가속 여유 부족"으로 표시하니 캡을 올린다. 듀얼 관절은 반전 모터를 `--invert`로 지정해야 두 모터가 같은 방향으로 관절을 돌린다.

### 08_move_joint.py — 관절 단위 이동 (듀얼 쌍 동작 확인)

```bash
python cookbook/08_move_joint.py --config configs/arm.yaml --joint J2 --goal 2000
```

설정의 `joints` 정의로 관절 하나를 안전하게 이동시킨다. 듀얼 모터 관절(J2/J3)이 `K - goal` 관계로 함께 움직이는지, 리플렉스가 동작하는지 확인할 때 쓴다.

### 09_continuous_angle.py — 연속 관절(J1) 허용 범위 정하기

```bash
python cookbook/09_continuous_angle.py --port /dev/ttyACM0 --id 1
```

전선이 풀린 자세에서 실행하면 그 자세를 0°로 잡고, 손으로 돌리는 동안 랩(4095→0)을 자동 처리한 연속 각도를 표시한다. 전선이 당기기 시작하는 각도를 양쪽에서 읽어 `joints` 설정의 `range_ticks`로 넣는다.

---

## 5. 트러블슈팅

### 포트 권한

`Permission denied: '/dev/ttyACM0'`이 뜨면 두 가지를 한 번만 해두면 영구 해결된다.

```bash
# 1) dialout 그룹 가입 (재로그인 후 적용; 당장 쓰려면 newgrp dialout)
sudo usermod -aG dialout $USER

# 2) udev 규칙: 꽂을 때마다 자동 권한 + 시리얼별 고정 이름 /dev/sopo_follower
sudo cp configs/99-sopo.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/sopo_follower /dev/ttyACM*     # crw-rw-rw- 와 심볼릭 링크 확인
```

임시 조치는 `sudo chmod 666 /dev/ttyACM0` (재연결 전까지만 유효). 규칙 파일의 시리얼 번호는
`udevadm info -a -n /dev/ttyACM0 | grep serial`로 확인한다. 어댑터를 추가하면
같은 방식으로 줄을 추가한다.

### sync_read가 "There is no status packet!"으로 실패

개별 `read`는 되는데 `sync_read`만 실패하고, 특정 모터가 리스트 **마지막에 있을 때만 성공**한다면 그 모터의 `Return_Delay_Time`(주소 7)을 확인한다. 이 값이 0이 아니면(일부 모델은 출하값 250 = 500µs) 해당 모터의 응답이 지연되면서 sync read 응답 체인에서 자기 뒤 모터들의 응답과 충돌한다. 해결:

```python
with bus.eprom_unlocked(motor_id):
    bus.write("Return_Delay_Time", motor_id, 0)
```

`apply_safety()`는 이 설정을 자동으로 적용한다. (실사례: sm8512bl 출하값 250 때문에 7모터 sync_read가 전멸 — 0으로 바꾸자 전 조합 성공)

### 모터 무응답

- 전원이 켜져 있는지, GND가 공통 접지되어 있는지 확인한다.
- 통신 속도가 맞는지 `00_scan.py`로 여러 보드레이트를 시도한다.
- 케이블이 끊어지거나 TX/RX가 반대로 연결되지는 않았는지 확인한다.

### ID 충돌

동일 ID가 여러 모터에 설정되면 버스가 혼란스러워진다. `00_scan.py`가 여러 보드레이트에서 같은 ID를 발견하면 경고한다(같은 보드레이트의 ID 충돌은 스캔으로 잡히지 않으니, 모터를 하나씩 연결해 확인한다). `04_setup_motor.py`로 하나씩 분리해서 ID를 바꾼다.

### 전원 부족 증상

- 모터가 간헐적으로 응답하거나 위치가 튀는 경우
- 부하를 줄이거나 전원 공급 능력을 높인다.
- USB 포트 전원만으로 여러 서보를 구동하지 않는다.

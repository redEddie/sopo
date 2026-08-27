# sopo

HopeJr를 분해해 제작한 자체 중형 로봇암을 위한 제어기.

Feetech STS/SMS 버스 서보(STS3215/STS3250 등)를 직접 레지스터 레벨에서 다루는 경량 드라이버와,
**사람이 다치지 않도록 모터 토크(effort)에 제한을 건** 안전 계층, 관절 추상화, 호스트 측 충돌 리플렉스를 담고 있다.
리더 암은 없다 — 명령 소스는 스크립트(웨이포인트)와 이후 정책(VLA)이다.

## 레퍼런스

| 출처 | 가져온 것 |
|---|---|
| [lerobot Feetech 드라이버](https://github.com/huggingface/lerobot/tree/main/src/lerobot/motors/feetech) | 레지스터 테이블, scservo_sdk 사용법·타임아웃 패치, sign-magnitude 인코딩, Lock 처리 |
| [lerobot HopeJr](https://github.com/huggingface/lerobot/tree/main/src/lerobot/robots/hope_jr) | sync_read → clamp → sync_write 제어 루프, `max_relative_target` 안전 캡, 가속도 제한 |
| [Trossen WidowX 250s puppet](https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet) | 고정 주기 position 제어 루프 구조 (리더 암은 없어 미러 자체는 미구현) |

## 설치

```bash
pip install -e .
# 시리얼 포트 권한 (재로그인 필요)
sudo usermod -aG dialout $USER
```

## 구성

```
sopo/            핵심 패키지
  registers.py     STS/SMS 컨트롤 테이블 + sign-magnitude 인코딩
  bus.py           FeetechBus: ping/scan/read/write/sync_read/sync_write/torque
  safety.py        SafetyLimits + 토크 제한/과부하 보호/소프트 리밋/스텝 클램핑
  joints.py        관절 추상화 (single / dual 반전쌍 / 케이블 제한 continuous)
  reflex.py        호스트 측 충돌 리플렉스 (포화+정체 → 홀드 래칭, recover)
  control.py       다관절 안전 제어 루프 (클램프 → 명령 → reflex → 홀드/복구)
  sources.py       명령 소스 경계 (lerobot Teleoperator 구조): WaypointSource + 리더 암/정책 플레이스홀더
  config.py        arm.yaml/calibration.yaml 로더
cookbook/        Feetech 기초 조작 쿡북 — README.md 참조
                   (스캔 → 상태 읽기 → 이동 → 토크 제한 → ID 설정 → 위치 한계 실측 → 자중 토크 실측)
examples/
  run_waypoints.py 다관절 웨이포인트 주행 (안전 루프의 첫 클라이언트)
configs/
  arm.yaml         이 암의 정의 (포트/관절/K/range_ticks/안전 기본값) — 커밋됨
  waypoints.example.yaml 웨이포인트 예시
  calibration.yaml 실측 관절 한계/토크 캡 (05/06 스크립트가 생성, 실행 스크립트가 자동 적용)
```

## 빠른 시작

```bash
# 1. 버스에 뭐가 붙어있는지 확인
python cookbook/00_scan.py --port /dev/ttyACM0

# 2. 토크 끈 채로 상태 모니터링
python cookbook/01_read_state.py --port /dev/ttyACM0 --ids 1,2,3,4,5,6,7

# 3. 토크 제한 걸고 한 관절 이동
python cookbook/02_move_position.py --port /dev/ttyACM0 --id 1 --goal 2048

# 4. (필요시) 모터 ID 변경
python cookbook/04_set_motor_id.py --port /dev/ttyACM0 --current-id 1 --new-id 2

# 5. 관절별 위치 한계·자중 토크 실측 (손으로 움직여 기록)
python cookbook/05_find_limits.py --port /dev/ttyACM0 --ids 19
python cookbook/06_gravity_load.py --port /dev/ttyACM0 --ids 19 --save

# 6. 관절 단위 이동 + 리플렉스 (설정: configs/arm.yaml — 이 암의 정의, 커밋됨)
python cookbook/08_move_joint.py --config configs/arm.yaml --joint J4 --goal 2300 --torque-limit 150

# 7. 다관절 웨이포인트 주행 (리플렉스 포함)
python examples/run_waypoints.py --config configs/arm.yaml --waypoints configs/waypoints.example.yaml --verbose
```

## 안전 설계

토크를 켜는 모든 코드는 반드시 `apply_safety()`를 먼저 호출한다. 3중 안전 계층:

1. **모터 내부 토크 캡** — `Torque_Limit`(RAM, 48) 기본 30%. `persist_torque_limit()`로
   `Max_Torque_Limit`(EPROM, 16)에 영구 저장하면 전원을 다시 켜도 캡이 유지된다.
2. **모터 내부 과부하 차단** — 허용 토크의 80%(`Overload_Torque`)로 500ms(`Protection_Time`)
   이상 밀면 20%(`Protective_Torque`)로 자동 강하. 사람이나 장애물에 끼었을 때 스스로 힘을 뺀다.
3. **호스트 측 클램핑** — 관절 소프트 리밋 + 제어 주기당 최대 이동량(`max_relative_target`)
   제한으로 목표치 점프를 차단. `reflex.py`가 부하 포화+정체(충돌), 추종 오차, 쌍 불일치, 통신 두절, 과열을 감시해 홀드/정지한다.

사람 옆에서 돌릴 때는 `torque_limit`을 400(40%) 이하로 유지할 것.

### 토크 제한 단위

`Torque_Limit`/`Max_Torque_Limit`은 **천분율(‰, 0~1000)** 이다. 150 = 15%. 모터에 토크 센서는 없고
이 값은 PWM 출력 듀티의 상한이라, 정지 상태에서는 스톨 토크 × 비율이 곧 최대 토크이고
움직일 때는 역기전력 때문에 그보다 작다(= 보수적인 상한).

| 모델 | 스톨 토크 @12V | 150‰일 때 |
|---|---|---|
| sts3215 (12V, C018) | 30 kg·cm (7.4V C001 버전은 19.5) | 4.5 kg·cm |
| sts3250 | 50 kg·cm | 7.5 kg·cm |
| sm8512bl | 85 kg·cm | 12.8 kg·cm |

절대 단위로 지정하려면 `torque_limit_from_kgcm("sts3250", 7.5)` → 150 처럼 변환하고, 진짜 물리량으로
자르려면 `SafetyLimits(protection_current_ma=...)`로 `Protection_Current`(6.5mA 단위)를 설정한다.
기어 효율까지 포함한 정확한 값이 필요하면 아는 무게를 아는 레버에 매달고 `Present_Load`를 읽어 관절별로
한 번 보정한다.

## 라이선스

Apache-2.0. 레지스터 테이블은 lerobot(Apache-2.0)에서 가져와 수정했다.

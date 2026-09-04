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
sopo/            핵심 패키지 — 층별 서브패키지 구조 (공개 API는 sopo/__init__.py가 재수출)
  hal/           하드웨어 추상화: registers.py(STS/SMS 컨트롤 테이블 + sign-magnitude 인코딩),
                 bus.py(FeetechBus: ping/scan/read/write/sync_read/sync_write/torque)
  motion/        joints.py(관절 추상화: single / dual 반전쌍 / 케이블 제한 continuous),
                 control.py(다관절 안전 제어 루프: 클램프 → 명령 → reflex → 홀드/복구)
  safety/        limits.py(SafetyLimits + 토크 제한/과부하 보호/소프트 리밋/스텝 클램핑),
                 reflex.py(호스트 측 충돌 리플렉스: 포화+정체 → 홀드 래칭, recover)
  model/         dynamics.py(URDF 기반 중력/외력 토크 GravityModel — 관절↔모터 매핑은 arm.yaml에서 유도)
  runtime/       daemon.py(**sopod** — 버스를 독점하는 하위 제어기 데몬: 50Hz 루프 + reflex + 워치독, ZMQ 상태/명령/액션),
                 client.py(SopoClient: 상태 구독 · 명령 · 액션 스트림 — 정책/조그/GUI는 이걸로만 접근),
                 cli.py(python -m sopo.runtime.cli status|watch|move|idle|guiding|recover|init|goto|shutdown),
                 startup.py(자가진단 루틴: init.py / sopod init),
                 sources.py(명령 소스 경계 (lerobot Teleoperator 구조): WaypointSource/JogSource + 리더 암/정책 플레이스홀더)
  config.py      arm.yaml/calibration.yaml 로더
  keys.py        터미널 키 입력 (jog)
cookbook/        Feetech 기초 조작 쿡북 — README.md 참조
                   (스캔 → 상태 읽기 → 이동 → 토크 제한 → ID 설정 → 위치 한계 실측 → 자중 토크 실측)
examples/
  init.py          초기화: 자가진단(관절별 2.6° 왕복) → 연속 관절 home 확정 → standby_pose 대기
  run_waypoints.py 다관절 웨이포인트 주행 (안전 루프의 첫 클라이언트)
  jog.py           키보드 조그 (버스 직접, 데몬 없이)
  jog_client.py    키보드 조그 — sopod 클라이언트 (액션 스트림)
logs/            블랙박스 CSV (리플렉스/정지 시 직전 60초 자동 저장, gitignore)
configs/
  arm.yaml         이 암의 정의 (포트/관절/K/range_ticks/안전 기본값) — 커밋됨
  waypoints.example.yaml 웨이포인트 예시
  calibration.yaml 실측 관절 한계/토크 캡 (05/06 스크립트가 생성, 실행 스크립트가 자동 적용)
description/     로봇 모델 (URDF) — Onshape export + 후처리 파이프라인
                   robot.urdf(전체) / arm_no_ee.urdf(팔만, 제어 기준) / sopo_viewer.xml
                   산출물은 전부 커밋 — Onshape API 없이 사용 가능. 자세한 건 description/README.md
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

# 7.5 초기화: 자가진단 후 standby_pose(arm.yaml)로 이동해 대기 — 제어 세션 시작 전 루틴
python cookbook/13_capture_pose.py --config configs/arm.yaml   # 토크 OFF로 자세 만든 뒤 1회: standby_pose 기록
python examples/init.py --config configs/arm.yaml

# 8. 키보드 조그 (←/→ 이동, ↑/↓ 관절, space 홀드, q 종료)
python examples/jog.py --config configs/arm.yaml

# 9. 캡·리밋을 모터 EPROM에 영구화 (전원 켜는 순간부터 캡 적용)
python cookbook/10_persist_caps.py --config configs/arm.yaml --dry-run
python cookbook/10_persist_caps.py --config configs/arm.yaml
```

## sopod — 하위 제어기 데몬 (Franka의 Control box + FCI 포지션)

```bash
# 터미널 1: 데몬 (IDLE, 토크 OFF로 시작. 버스는 데몬만 잡는다 — 쿡북 스크립트와 동시 실행 불가)
sopod --config configs/arm.yaml            # 또는 python -m sopo.runtime.daemon

# 터미널 2: 상태 / 명령
python -m sopo.runtime.cli watch                   # 50Hz 상태 스트림 (모드, 관절, 부하, 전압, 지터)
python -m sopo.runtime.cli init                    # 자가진단 → standby_pose, 토크 유지
python -m sopo.runtime.cli goto J4=2300 J6=2500    # 단발 목표 (MOVE 모드에서)
python -m sopo.runtime.cli recover                 # REFLEX 래칭 해제
python -m sopo.runtime.cli idle                    # 토크 OFF

# 터미널 3: 액션 스트림 클라이언트 (정책·조그). 0.5초 끊기면 데몬이 홀드
python examples/jog_client.py
```

파이썬(정책)에서 — Franka의 `control()` 세션에 해당하는 **제어 리스**를 잡아야 액션이 받아들여진다:

```python
from sopo.runtime.client import SopoClient
c = SopoClient()
c.command("move")            # 토크 ON (FCI 활성화에 해당)
c.acquire("policy")          # 배타적 제어권. 다른 클라이언트가 잡고 있으면 거부
while True:
    s = c.state()            # 최신 상태 (50Hz)
    if s["mode"] == "reflex":          # 리플렉스: 리스가 회수되어 액션이 버려진다
        c.command("recover")           # 명시적 복구 (Franka automaticErrorRecovery)
        c.acquire("policy")            # 새 세션
    c.send_action({"J4": 2300})        # 0.5초 안에 계속 보내야 함 (워치독)
```

모드: `idle`(토크 OFF) · `guiding`(토크 OFF, 가르치기) · `move` · `reflex`(래칭) · `stopped`. 토크는 명령으로만 켜지고, 켜기 전 항상 `apply_safety()`.
REFLEX 중에는 `move/init/goto`가 거부되고 `recover`만 MOVE로 돌아가는 길이다. `idle`/`guiding`(토크 OFF)은 항상 허용.
액션 스트림이 0.5초 끊기면 **그 순간의 위치를 목표로 고정**해 홀드한다(끌려가지 않음). 데몬은 시작할 때 토크를 건드리지 않는다 —
이전 세션이 홀드로 끝났으면 REFLEX(`INHERITED_HOLD`)로 이어받아 `recover`/`idle`을 기다린다.

## 안전 설계

토크를 켜는 모든 코드는 반드시 `apply_safety()`를 먼저 호출한다. 3중 안전 계층:

1. **모터 내부 토크 캡** — `Torque_Limit`(RAM, 48) 기본 30%. `persist_torque_limit()`로
   `Max_Torque_Limit`(EPROM, 16)에 영구 저장하면 전원을 다시 켜도 캡이 유지된다.
2. **모터 내부 과부하 차단** — 허용 토크의 80%(`Overload_Torque`)로 500ms(`Protection_Time`)
   이상 밀면 20%(`Protective_Torque`)로 자동 강하. 사람이나 장애물에 끼었을 때 스스로 힘을 뺀다.
3. **호스트 측 클램핑** — 관절 소프트 리밋 + 제어 주기당 최대 이동량(`max_relative_target`)
   제한으로 목표치 점프를 차단. `reflex.py`가 부하 포화+정체(충돌), 추종 오차, 쌍 불일치, 통신 두절, 과열을 감시해 홀드/정지한다.

사람 옆에서 돌릴 때는 `torque_limit`을 400(40%) 이하로 유지할 것.

### 정지 정책 — 결함 시 굳는다, 힘이 빠지지 않는다 (IEC 60204-1 Cat 2, [#10](https://github.com/redEddie/sopo/issues/10))

자가진단 실패·리플렉스·통신 오류·데몬/스크립트 종료 등 **모든 소프트웨어 결함은 홀드**다: 목표=현재로 굳히고
`Torque_Limit`을 홀드 캡(600‰)으로 올려 빳빳하게 유지한다 (ISO 10218-1 safety-rated monitored stop, Franka reflex와 동일).
토크를 끄는 것(Cat 0)은 `idle`/`guiding` 명령, `cookbook/11_torque_off.py`, 스크립트의 `--release`뿐이다 —
쥔 물건을 떨어뜨리거나 팔 아래의 사람·물체를 치는 "예상치 못한 낙하"를 없애기 위해서다.
EPROM `Max_Torque_Limit`은 상한 600‰(`10_persist_caps`), 운용 캡(200~400‰)은 `apply_safety()`가 RAM에 쓴다.
서보 홀드는 브레이크가 아니라 능동 토크라 발열이 있고 홀드 캡을 넘는 힘에는 밀린다; 물리 E-stop(#5)은 표준대로 전원 차단(Cat 0)이다.

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

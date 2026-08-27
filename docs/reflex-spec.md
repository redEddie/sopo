# sopo/reflex.py 설계 스펙 (Franka 충돌 리플렉스의 sopo판)

배경: `docs/franka-safety.md` 2절 #2, #4, #5, #8, #12, #15. 실측 근거는 07 실험 —
무부하 이동 시 max load 124~128‰(캡 150의 85%), 손으로 잡으면 150‰에 포화된 채 1.8~3.75초 지속.
모터는 막혀도 캡 토크로 계속 밀므로 호스트가 감지해서 멈춰야 한다.

## 1. 원칙

- **순수 로직 모듈**: 버스를 만지지 않는다. 매 사이클 측정값을 받아 이벤트/모드를 돌려준다 → 가짜 데이터로 단위 테스트.
- **래칭**: 한 번 REFLEX에 들어가면 `recover()`를 명시적으로 부를 때까지 명령을 거부한다 (Franka `automaticErrorRecovery`).
- **홀드가 기본, 토크 해제는 예외**: 충돌·추종 오차는 "현재 위치를 목표로 다시 쓰고 홀드"(미는 힘만 제거, 중력 낙하 없음). 토크 해제는 통신 두절·과열에만.
- J1 range 클램프, 소프트 리밋 클램프는 **이벤트가 아니다** (조용히 잘라내기만).

## 2. 상태와 이벤트

```python
class Mode(Enum): IDLE, MOVE, REFLEX, STOPPED          # STOPPED = 토크 해제됨, 재시작 필요
class Event(Enum): COLLISION, TRACKING_ERROR, PAIR_MISMATCH, COMM_LOSS, OVERTEMP, JOINT_LIMIT
```

## 3. 판정 규칙 (모터 단위, 기본값은 ReflexConfig로 조정)

| 이벤트 | 조건 | 기본값 | 근거 |
|---|---|---|---|
| COLLISION | `abs(load) >= sat_ratio * cap` 가 `t_collision` 창 동안 지속 **AND** 그 창에서 추종 오차 감소량 < `progress_ticks` (손에 잡혀 1~2틱씩 기어가는 것도 충돌) | sat_ratio 0.95, t_collision 0.3 s, progress_ticks 40 | 07: 무부하 85%, 잡으면 100% 포화, 잡힌 채 ~110틱/s 로 기어감 |
| (가속 유예) | 목표가 `accel_step` 이상 바뀐 직후 `t_accel` 동안은 COLLISION 판정 보류 | accel_step 40틱, t_accel 0.2 s | Franka의 acceleration 임계값 상향에 대응 |
| TRACKING_ERROR | `abs(goal-present) > err_ticks` 가 `t_error` 이상 지속 | err_ticks 150(≈13°), t_error 0.5 s | 캡 부족·걸림·미응답 통합 감지 |
| PAIR_MISMATCH | 듀얼 쌍 `abs(ref + mirror - K) > pair_tol` | pair_tol 20틱 | 쌍이 서로 싸움 |
| COMM_LOSS | 연속 통신 실패 ≥ `comm_fail_max` | 5 | 연속 오류 5회 |
| OVERTEMP | 온도 ≥ `temp_stop` (경고는 `temp_warn`) | 70 °C / 65 °C | 모터 Max_Temperature_Limit 기본 70 |
| JOINT_LIMIT | 캘리브레이션된 모터의 **측정** 위치가 소프트 리밋을 `limit_margin` 넘게 벗어남 (명령 클램프는 이벤트 아님 — 외력에 밀리거나 캡 부족으로 처진 경우) | limit_margin 30틱 | libfranka `joint_position_limits_violation` |

`cap`은 `SafetyLimits.torque_for(motor_id)`. 부하 부호는 미는 방향 → `backoff` 방향 결정에 쓴다.

## 4. 리플렉스 동작

1. COLLISION / TRACKING_ERROR / PAIR_MISMATCH → `Mode.REFLEX`.
   호출자는 `hold_targets(present)`를 받아 **1회 sync_write**(목표=현재 실측). 옵션 `backoff_ticks`(기본 0, 권장 30~50)면
   충돌 관절만 부하 반대 방향으로 그만큼 물러난 목표. 이후 `update()`는 계속 호출하되 명령은 거부.
2. COMM_LOSS / OVERTEMP → `Mode.STOPPED`. 호출자는 토크 해제("암이 내려올 수 있음" 경고). 복구 불가, 프로그램 재시작.
3. `recover(present, load)`: 전 모터 `abs(load) < 0.5*cap` 이고 오차 < 30틱일 때만 `Mode.MOVE`로 복귀, 아니면 False와 사유.
   사용자 입력('r')로만 호출. 자동 복구 없음.

## 5. API

```python
@dataclass
class ReflexConfig:
    sat_ratio: float = 0.95; t_collision: float = 0.3
    accel_step: int = 40;    t_accel: float = 0.2
    err_ticks: int = 150;    t_error: float = 0.5
    pair_tol: int = 20;      comm_fail_max: int = 5
    temp_warn: int = 65;     temp_stop: int = 70
    backoff_ticks: int = 0

@dataclass
class Trip:                      # 발생한 이벤트 상세
    event: Event; motor_id: int | None; detail: str

class Reflex:
    def __init__(self, limits: SafetyLimits, pairs: dict[str, tuple[int, int, int]], cfg: ReflexConfig = ReflexConfig()):
        """pairs: 관절이름 -> (reference_id, mirror_id, K)"""
    @property
    def mode(self) -> Mode
    def update(self, now: float, present: dict[int, int], goal: dict[int, int],
               load: dict[int, int], temps: dict[int, int] | None = None,
               comm_ok: bool = True) -> list[Trip]
        """매 사이클 호출. goal은 이번 사이클에 보낸(또는 보낼) 목표. temps는 2초마다만 넘겨도 됨."""
    def hold_targets(self, present: dict[int, int]) -> dict[int, int]
    def recover(self, present: dict[int, int], load: dict[int, int]) -> tuple[bool, str]
    def warnings(self) -> list[str]     # temp_warn 등 비치명 경고 (매 사이클 비움)
```

내부 상태(모터별): 포화 시작 시각, 포화 구간 중 최소 오차, 마지막 목표(가속 유예용), 오차 초과 시작 시각, 연속 통신 실패 수.

## 6. 통합 지점

- `examples/run_waypoints.py`(예정): 관절 read + Present_Load → `reflex.update` → REFLEX면 `joint.command()`로 홀드 후
  키 입력 대기(`r` 복구 / `q` 종료), STOPPED면 토크 해제 후 종료.
- `cookbook/08_move_joint.py`: 같은 방식. **리플렉스 1차 시험은 여기서** — 리더 암 불필요.
- pairs는 yaml `joints`의 dual 항목에서 만든다. mode 변화와 Trip은 stderr에 시각과 함께 출력.

## 7. 테스트 (tests/test_reflex.py, 가짜 데이터, pytest)

1. 무부하 왕복 시퀀스(load 0→128→0, 오차 200→0): 이벤트 없음.
2. 목표 변경 직후 150ms 동안 load 150(=캡): 가속 유예로 이벤트 없음.
3. load 150이 0.35 s 지속 + 오차 100으로 정체: COLLISION 1회, mode REFLEX, 이후 update 반복해도 중복 Trip 없음.
4. load 150이 0.35 s 지속이지만 오차가 계속 감소: 이벤트 없음(무거운 자세에서 느리게 진행 중).
5. 오차 200이 0.6 s 지속(load 낮음): TRACKING_ERROR.
6. 쌍 합이 K±40: PAIR_MISMATCH.
7. comm_ok=False 5회: STOPPED. temps {19: 71}: STOPPED. {19: 66}: warnings만.
8. recover: load 높으면 False, 낮고 오차 작으면 True → MOVE.

## 8. 하드웨어 튜닝 절차 (Kimi가 실행 후 TODO.md에 수치 기록)

1. `08_move_joint --joint J4 --torque-limit 150` 왕복 10회: 오탐 0이어야 함. 오탐 나면 sat_ratio↑ 또는 t_accel↑.
2. 같은 조건에서 손으로 잡기: 0.3~0.6 s 안에 COLLISION, 홀드 후 손을 놓아도 움직이지 않음, `r`로 복구.
3. J2(듀얼)에서 반복 + 미러 모터 케이블을 뽑아 PAIR_MISMATCH 확인(토크 OFF 상태에서 뽑을 것).
4. 사람 근처 시험은 캡 ≤ 300‰.

## 9. 검증 기록

- 2026-08-28 J4(캡 200): Phase A 오탐 0/10회, Phase B 잡기 → 0.32s 감지(포화 −200‰, 이동 +1틱), 홀드·복구 6회. 기본 임계값으로 합격.

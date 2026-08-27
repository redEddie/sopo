# Kimi 인수인계 (2026-08-25)

이 문서는 Claude가 하던 작업을 Kimi Code가 이어받기 위한 것이다. 레포 루트에서 `kimi`를 실행하고
"HANDOFF.md를 읽고 1번부터 순서대로 진행해"라고 지시하면 된다. **토크를 켜는 코드는 반드시
아래 '하드웨어 불변 규칙'을 지킬 것.** 참고 문서: `README.md`, `cookbook/README.md`, `TODO.md`.

## 현재 상태

- 8모터 전부 검증됨. J1=1(sm8512bl), J2=10+11, J3=15+16(sts3250 듀얼, 반전 장착), J4=19, J5=20, J6=21(sts3215). J7 그리퍼 미장착.
- 버스 /dev/ttyACM0 @1M. sync_read 8모터 정상 (sm8512bl의 Return_Delay_Time을 0으로 고쳐서 해결됨).
- hopejr 잔재 EPROM 설정 정리 완료 (TODO.md '롤백용 원래 값' 참조).
- `configs/calibration.yaml`: 위치 한계는 10,11,15,16,19,20 기록됨. **ID 1은 [51, 4044]로 잘못 기록됨** — J1 범위가 0/4095 랩 지점을 지나서 그렇다. ID 21은 미기록. 자중 토크는 전부 기록(10/11=200, 나머지 150=하한값).
- 듀얼 쌍 관계: `pos_b = K - pos_a`, 실측 K: J2≈4005~4011, J3≈4100. (4095가 아님)

## 남은 작업 (순서대로)

### 1. J1 랩 문제 — `05_find_limits.py`에 언랩 + 호밍 센터링 추가
- 스윕 중 연속 위치를 언랩한다: 이전 읽기 대비 변화가 +2048 초과면 -4096, -2048 미만이면 +4096 누적.
- `--center-homing` 옵션: 스윕 후 언랩된 min/max의 중앙(center)이 2048로 읽히도록 `Homing_Offset`(주소 31, sign-magnitude bit 11, 범위 -2047..2047)을 조정한다.
  - 관측된 관계: `Present = raw - Homing_Offset` (mod 4096). 따라서 `new_offset = old_offset + (center - 2048)`를 -2047..2047로 정규화(4096 mod).
  - 쓰기는 `with bus.eprom_unlocked(id):` 안에서. 쓴 뒤 Present_Position을 다시 읽어 예상값(이전값 - (center-2048))과 일치하는지 검증.
  - 저장하는 position_limits는 새 프레임 기준: `[min - (center-2048) + margin, max - (center-2048) - margin]`.
  - 오프셋을 바꾸면 그 모터의 기존 position_limits는 무효 — 덮어쓴다.
- 실행: `python cookbook/05_find_limits.py --port /dev/ttyACM0 --ids 1 --center-homing --write-eprom`
- 선택: 듀얼 쌍(10,11 / 15,16)도 `--center-homing`으로 각 모터를 2048 중심에 맞추면 K가 4096이 되어 관절 추상화가 단순해진다. 단, 그러면 기존 10/11/15/16 한계를 다시 재야 한다.

### 2. 남은 측정
- `05` ID 21. `06` 재측정: 19, 15+16 (J5/J6 장착 전 값이라서). 10+11은 200으로 재측정 완료.
- 검증: `07_verify_torque.py`로 각 관절이 캡에서 실제로 움직이는지.

### 3. 캡 영구화 — `cookbook/08_persist_caps.py` 신규
- calibration.yaml의 torque_limits를 읽어 `persist_torque_limit()`로 각 모터 `Max_Torque_Limit`(EPROM)에 기록. 전원 켤 때부터 캡이 걸리게.
- position_limits도 `Min/Max_Position_Limit`에 기록(05 --write-eprom과 동일). `--dry-run`으로 미리 보여주고 `yes` 입력 후 쓰기.

### 4. 관절 추상화 — `sopo/joints.py`
- `Joint(name, motor_ids, invert_ids, K)`: 모터 1개 또는 듀얼 쌍. 관절값은 주 모터(첫 ID) 프레임.
- `read(bus) -> int`, `goal_for_motors(value) -> {id: pos}`: 쌍의 반전 모터는 `K - value`. 쌍은 항상 함께 명령해 서로 싸우지 않게.
- 관절별 한계/토크 캡은 calibration.yaml에서. 듀얼 쌍은 두 모터 캡 동일.

### 5. Robot API — `sopo/robot.py` `SopoRobot`
**전체 구조는 `docs/architecture.md`를 따른다 (데몬 `sopod` + IPC + 클라이언트). SopoRobot은 클라이언트 위에 올린다.**
- `connect(calibrate=True) / get_observation() / send_action(action) / disconnect()`. lerobot `Robot` 인터페이스 형태.
- 내부: sync_read → 관절 변환 → `clamp_goal` → sync_write. 정규화: 틱 ↔ [-1, 1] (관절 한계 기준).
- 정책 워치독: send_action이 일정 시간 안 오면 현재 자세 홀드 후 토크 해제.
- 명령 보간: 정책 10~30Hz → 버스 50Hz 스무딩(선형 보간).

### 6. 호스트 측 안전장치 (Franka 벤치마킹) — `sopo/reflex.py`
**구현 스펙: `docs/reflex-spec.md` (API, 임계값, 테스트 8종, 하드웨어 튜닝 절차). 배경: `docs/franka-safety.md`.**
- 충돌 감지 규칙 (실측 근거: 무부하 이동은 캡의 85% 이하, 손으로 잡으면 캡에 포화):
  `Present_Load >= 0.95 * 캡`이 300ms 이상 지속 **그리고** 위치 오차가 줄지 않으면 충돌 → 정지(목표=현재) 또는 후퇴(반대 방향 50틱).
- 속도/가속 리미터: 사이클당 스텝(`max_relative_target`)에 더해 속도 프로파일 제한.
- 온도 감시(65°C 경고, 70°C 토크 해제), 통신 오류 5회 연속 시 토크 해제(이미 mirror.py에 있음 — 공통 모듈로 이동).

## 하드웨어 불변 규칙 (위반 금지)

1. 토크를 켜기 전 반드시 `apply_safety()`. 메인 루프는 `try/finally`로 `end_session()`(기본 홀드).
2. 사람 근처 테스트는 캡 ≤ 300‰. J1(85 kg·cm)은 특히 낮게.
3. 목표 위치는 항상 현재 위치를 읽고 `clamp_goal()`을 거쳐 보낸다. 점프 금지.
4. 듀얼 쌍(10+11, 15+16)은 반드시 함께, `K - pos` 관계로 명령. 한쪽만 토크 ON 금지.
5. EPROM(주소 < 40) 쓰기는 `bus.eprom_unlocked(id)` 안에서만. ID/Baud 변경은 04 스크립트 참고(즉시 적용됨).
6. `Return_Delay_Time`은 전 모터 0 유지 (apply_safety가 강제).
7. **정지 정책 (#10)**: 결함·종료 시 토크를 끄지 말고 홀드(freeze: 목표=현재, 홀드 캡 600‰). 토크 해제는 명시 명령(`idle`/`guiding`/`--release`/11_torque_off)뿐. 토크 OFF 시 관절이 내려오므로 해제 메시지에 경고 유지.

## 레지스터/단위 요약

- Torque_Limit(48, RAM)/Max_Torque_Limit(16, EPROM): 천분율. sts3215(12V) 30, sts3250 50, sm8512bl 85 kg·cm 스톨.
- Present_Load(60): 출력 듀티 ‰, 부호 bit 10. Present_Current(69): 6.5mA 단위.
- Present_Position(56)/Goal_Position(42): 0~4095, 부호 bit 15. Homing_Offset(31): 부호 bit 11.
- Min/Max_Position_Limit(9/11), Phase(18, bit4=멀티턴 피드백 — 전 모터 off 상태 유지).
- 전체 표: `sopo/registers.py`, `cookbook/README.md` 2절.

## 작업 방식

- 코드 주석·문서는 한국어, 식별자는 영어. 기존 스타일(argparse, sys.path 부트스트랩) 유지.
- 하드웨어 테스트는 한 관절씩, 낮은 캡으로. 결과 수치는 TODO.md에 기록.
- 커밋 메시지는 영어로 한 줄 요약 + 본문. 안전 기본값(torque_limit 300, overload 80/50/20) 임의 변경 금지.

## 리뷰 1 (Claude, 2026-08-25) — joints.py / mirror.py 1차 구현에 대한 피드백

**채택**: `SingleMotorJoint`, `DualMotorJoint`, `build_joints`, yaml `joints` 섹션, mirror.py 관절 단위 리팩터.
안전 불변 규칙은 잘 지켜졌음.

**정정(Claude): J1은 기계적 스토퍼가 없고 케이블이 회전을 제한하는 관절이며 범위가 360°를 넘을 수 있다.
따라서 `ContinuousJoint`(소프트웨어 언랩)는 올바른 접근이다. HANDOFF 1번의 Homing_Offset 센터링은 J1에 적용하지 않는다.**
단, 아래 두 가지를 반영할 것 (단순하게, 리플렉스 아님):
1. **논리각 범위 클램프 추가.** J1 목표를 `home ± range_ticks`(yaml `range_ticks`, 기본 2600 ≈ ±228°, 총 456°)로
   잘라낸다. 에러/정지/토크 해제 없이 목표만 클램프 — "360°를 좀 넘어도 reflex가 나지 않는" 요구사항 그대로.
   `clamp_joint_goal`에서 continuous 관절도 이 범위로 클램프한다. 나중에 reflex.py를 만들 때 이 클램프는 오류로 취급하지 않는다.
   - `home`은 프로그램 시작 시 첫 읽기값(turn_count=0). 운용 규칙: **케이블이 풀린 자세에서 프로그램을 시작한다** (mirror.py 시작 메시지로 안내).
   - 리더와 팔로워 J1 모두 같은 규칙. 리더 J1이 범위를 넘으면 팔로워는 경계에 머문다.
2. **turn_count 파일 저장(`state_path`, `*_state.yaml`, `state_dir`) 제거.** 토크 OFF 중 사람이 랩 너머로 돌리면 파일 값이 어긋나
   범위 클램프가 엉뚱한 곳에서 걸린다. 메모리에서만 세고 시작 시 0. `MAX_SINGLE_MOVE=4095`는 의미가 없으니 삭제(스텝 클램프가 이미 막는다).
   - **확정(2026-08-28)**: Phase bit4 ON + Min/Max_Position_Limit 0/0 = 펌웨어 멀티턴. 4095 초과 목표를 그대로 받고 넘는다. `firmware_multiturn: true`로 운용 중.
   - `calibration.yaml`의 ID 1 position_limits는 사용하지 않는다(삭제됨). J1 한계는 yaml `range_ticks`로만.
2. `DualMotorJoint`: 미러 모터도 읽어서 `ref + mirror`가 K ± 20틱 안인지 매 사이클 검사, 벗어나면 경고 후 정지(쌍이 싸우는 상태). 미러 목표도 자기 모터의 position_limits로 클램프. K는 yaml 상수 대신 05가 쌍 측정 시 `pos_a + pos_b` 중앙값을 calibration.yaml `pair_constants`에 기록하고 거기서 읽는다.
3. 성능/원자성: Joint가 `goals(logical) -> {motor_id: tick}` 와 `motor_ids`만 제공하고, 루프에서 전 모터 `sync_read` 1회 → 관절 변환 → 클램프 → `sync_write` 1회. 쌍의 두 목표가 같은 패킷에 실린다.
4. 오타: mirror.py "폴터"→"폴더", "낼어올"→"내려올"; arm.yaml "폭더"→"폴더". 예시의 `position_limits: {1: [0, 4095], ...}`는 삭제 (리밋 해제를 권장하는 모양이 됨).
5. 위 수정 후 `python -m py_compile`, `--help`, 그리고 하드웨어 없이 `build_joints` + 클램프 로직 단위 테스트를 추가한 뒤 커밋. `cookbook/04_set_motor_id.py`와 README 변경도 같은 커밋에 포함.

### 리뷰 1 처리 결과 (Claude가 직접 반영, 커밋 9f0c996)
- 반영됨: ContinuousJoint 파일 상태 제거 + `range_ticks` 클램프 + **command()가 turn_count를 건드리던 이중 카운트 버그 수정**,
  mirror.py 연속 관절 범위 클램프·시작 경고·오타, 08_move_joint 스텝 클램프(목표를 통째로 보내던 문제), yaml 예시.
  단위 테스트(가짜 버스)로 랩 카운트·클램프·듀얼 매핑 검증함 — `tests/`로 옮겨 pytest화할 것.
- **Kimi 몫으로 남음**: (a) DualMotorJoint가 미러 모터도 읽어 `ref + mirror ≈ K ± 20` 검사, 벗어나면 경고/정지;
  (b) K를 05 실측(`pair_constants`)에서 읽기; (c) Joint가 `{id: tick}`을 반환하고 루프는 sync_read/sync_write 1회씩;
  (d) HANDOFF 2~3번(남은 측정, 08_persist_caps), 5~6번(SopoRobot, reflex.py).
- 규칙 추가: **토크를 켜는 모든 스크립트는 목표를 clamp_goal/스텝 클램프 없이 보내지 않는다** (08에서 위반 발견됨).

### 리뷰 2 (Claude, 2026-08-27) — reflex.py 1차 구현 (33c140b) 점검 결과, 직접 수정해 커밋
- **미탐 버그**: 오차가 1틱이라도 줄면 포화 타이머를 리셋 → 07 실험처럼 손에 잡힌 채 기어가는 경우 COLLISION이 영원히 안 뜸.
  → 규칙 변경: "포화가 t_collision 창 동안 지속 **AND** 창 내 진행량 < `progress_ticks`(40)" (스펙 3절 갱신). 테스트 4b(기어가기) 추가.
- **복구 불가 버그**: 08/mirror의 recover가 리플렉스 발동 시점의 낡은 부하값을 재사용 → 항상 "load too high". → 복구 시 다시 읽도록.
- **J1 프레임 버그**: 08은 홀드 목표를 bus.write로 직접(논리각 4096 초과 가능), mirror는 raw를 joint.command에 전달(논리각과 불일치 → 반턴 이동 위험).
  → 홀드는 항상 `joint.command()` 경유, reflex present는 연속 관절만 논리각(`reflex_present_view`).
- 사소: 온도 검사에서 같은 사이클에 update()를 두 번 호출 — 동작엔 문제 없음, 나중에 temps를 본 update에 합칠 것.
- 다음: 스펙 8절 하드웨어 튜닝 (J4 캡150 왕복 10회 오탐 0 → 손으로 잡아 0.3~0.6s 내 COLLISION → r 복구).

### 변경 (2026-08-27): 리더 암 없음 — mirror.py 삭제
- `examples/mirror.py`와 yaml의 `leader/follower` 섹션 제거. 설정은 `arm: {port, baudrate}` 하나. 08은 이미 반영됨.
- ~~새 작업~~ **완료(Claude, 2026-08-27)**: `examples/run_waypoints.py` + `sopo/control.py`(안전 루프) + `sopo/sources.py`(ActionSource, 리더 암/정책 플레이스홀더) + `sopo/config.py`(구 스키마 호환 로더). 원래 메모: 삭제된 mirror_loop(커밋 71025a7의 `examples/mirror.py`)에서 리더 읽기만 빼고
  안전 루프(관절 read → clamp_joint_goal → command, reflex update/hold/recover, 온도, 통신)를 그대로 옮긴다.
  명령 소스는 `configs/waypoints.example.yaml`의 관절 목표 리스트(`- {J2: 2000, J4: 2300}` …)를 순서대로, 각 목표에 도달(오차 < 30틱)하면 다음으로.
  소프트스타트(첫 목표까지 스텝 20)는 유지. 리더 관련 `invert/offsets/map_leader_to_follower`는 버린다.
- 이후 `sopod` 데몬의 첫 클라이언트도 이 스크립트다 (architecture.md 4절).


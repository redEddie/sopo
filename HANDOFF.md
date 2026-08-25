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
- `connect(calibrate=True) / get_observation() / send_action(action) / disconnect()`. lerobot `Robot` 인터페이스 형태.
- 내부: sync_read → 관절 변환 → `clamp_goal` → sync_write. 정규화: 틱 ↔ [-1, 1] (관절 한계 기준).
- 정책 워치독: send_action이 일정 시간 안 오면 현재 자세 홀드 후 토크 해제.
- 명령 보간: 정책 10~30Hz → 버스 50Hz 스무딩(선형 보간).

### 6. 호스트 측 안전장치 (Franka 벤치마킹) — `sopo/reflex.py`
- 충돌 감지 규칙 (실측 근거: 무부하 이동은 캡의 85% 이하, 손으로 잡으면 캡에 포화):
  `Present_Load >= 0.95 * 캡`이 300ms 이상 지속 **그리고** 위치 오차가 줄지 않으면 충돌 → 정지(목표=현재) 또는 후퇴(반대 방향 50틱).
- 속도/가속 리미터: 사이클당 스텝(`max_relative_target`)에 더해 속도 프로파일 제한.
- 온도 감시(65°C 경고, 70°C 토크 해제), 통신 오류 5회 연속 시 토크 해제(이미 mirror.py에 있음 — 공통 모듈로 이동).

## 하드웨어 불변 규칙 (위반 금지)

1. 토크를 켜기 전 반드시 `apply_safety()`. 메인 루프는 `try/finally`로 토크 해제.
2. 사람 근처 테스트는 캡 ≤ 300‰. J1(85 kg·cm)은 특히 낮게.
3. 목표 위치는 항상 현재 위치를 읽고 `clamp_goal()`을 거쳐 보낸다. 점프 금지.
4. 듀얼 쌍(10+11, 15+16)은 반드시 함께, `K - pos` 관계로 명령. 한쪽만 토크 ON 금지.
5. EPROM(주소 < 40) 쓰기는 `bus.eprom_unlocked(id)` 안에서만. ID/Baud 변경은 04 스크립트 참고(즉시 적용됨).
6. `Return_Delay_Time`은 전 모터 0 유지 (apply_safety가 강제).
7. 토크 OFF 상태에서 관절은 중력에 내려온다 — 종료 메시지에 경고 유지.

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

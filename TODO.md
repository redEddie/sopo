# sopo 로드맵

## 실측 하드웨어 구성 (2026-08-25 확인)

| 관절 | 모터 ID | 모델 | 비고 |
|---|---|---|---|
| J1 | 1 | sm8512bl | Return_Delay_Time 250→0 (sync_read 수정). **펌웨어 멀티턴 모드로 운용**: Phase bit4 ON, Min/Max_Position_Limit 0/0, Homing_Offset −1988 (2026-08-28 확정, +415° 연속 판독·조그 통과 확인). 단일턴 모드는 4095/0에서 긴 길로 돌아 ±5° 떨림 |
| J2 | 10, 11 | sts3250 ×2 | 듀얼, 반전 장착. 실측 pos10+pos11 ≈ **4005** (4095 아님 — 관절 추상화에서 상수로 보정) |
| J3 | 15, 16 | sts3250 ×2 | 듀얼, 반전 장착. 실측 pos15+pos16 ≈ **4100** |
| J4 | 19 | sts3250 | |
| J5 | 20 | sts3215 | 2026-08-25 오후 장착 |
| J6 | 21 | sts3215 | 2026-08-25 오후 장착 (초기 스캔 때는 J5로 오인) |
| J7(그리퍼) | - | - | 미장착 |

- 버스: /dev/ttyACM0 (CDC-ACM), 1M baud. 실측 왕복: 7모터 전체 읽기 1.74ms(575Hz), 단일 0.27ms — USB 레이턴시 이슈 없음
- 전 모터 Max_Torque_Limit 공장값 1000(100%) — `cookbook/10_persist_caps.py`로 영구화 (스크립트 완료, 기록은 사용자 실행)
- **EPROM 정규화 (2026-08-25 16시경, hopejr 잔재 제거)** — 롤백용 원래 값:
  - ID1 Phase 0b00111100 → 0b00101100 (bit4 해제) — **2026-08-28 다시 ON으로 되돌림**: 연속 관절엔 멀티턴 모드가 정답
  - ID10/11 Min/Max_Position_Limit 1030/3065 → 0/4095, ID15 960/2970 → 0/4095, ID16 1100/3200 → 0/4095
    (hopejr K=4095 기준 한계라 우리 암(K≈4005/4100)과 불일치 — ID11·15가 한계 밖에 있어 토크 ON 시 쌍이 싸울 뻔함)
  - ID20/21 Homing_Offset 85 → 0 (읽기값 +86 이동)
  - ID1 Homing_Offset 2047은 유지 (J1 중립 자세가 2096으로 중앙 근처라 적절)

## 1. 하드웨어 검증 (진행 중)

- [x] 포트 권한 설정
- [x] 버스 스캔: 모터 모델/ID 확인 (`cookbook/00_scan.py`)
- [x] 전 모터 상태 읽기: 12.1~12.4V, 34~40°C, Status 0 — 정상
- [x] 말단 모터(ID 21) 소구간 이동 테스트 — 토크 20% 제한, +100틱 왕복 성공
- [ ] 토크 제한 체감 테스트: 손으로 밀어 포화 확인 (`cookbook/03_torque_limits.py`)
- [ ] 관절별 소프트 리밋 실측 (`05_find_limits.py`) — 완료: J4. 남음: J1, J2, J3, J5, J6
- [ ] 자중 토크 실측 (`06_gravity_load.py`) — J1/J5/J6은 J6 장착 후 측정, **J2/J3/J4는 J5·J6 장착 전 값이라 재측정 필요**
- [ ] 다관절 웨이포인트 주행 하드웨어 검증 (`examples/run_waypoints.py`) — 리더 암 없음, 소스는 ActionSource로 교체 가능
- [ ] 키보드 조그 하드웨어 검증 (`examples/jog.py`)
- [ ] 리플렉스 하드웨어 튜닝 (`docs/reflex-spec.md` 8절) — 블랙박스 CSV(`logs/`)로 근거 남기기

## 2. Robot API/SDK (Franka의 libfranka 포지션)

**목표 구조와 계층: [`docs/architecture.md`](docs/architecture.md)** — 버스는 `sopod` 데몬만, 상태 PUB/명령 REP로 프로세스 분리, GUI는 구독자

- [x] **관절 추상화** 1차: `sopo/joints.py` (single / dual K−goal / 케이블 제한 continuous + range_ticks 클램프).
      남음: 듀얼 쌍 K 일관성 검사, K 실측값 연동, sync_read/write 통합
- [x] sm8512bl Return_Delay_Time 250 → 0 설정 (EPROM) — sync_read 실패 근본 원인이었음.
      apply_safety()가 이제 전 모터에 0을 강제해 재발 방지
- [ ] `SopoRobot` 클래스: `connect() / get_observation() / send_action()` 경계 확립
- [ ] 캘리브레이션 층: homing offset + 관절 범위 실측 → 틱 ↔ 정규화 좌표([-1,1] 또는 rad) 변환
- [ ] 명령 보간: 정책 10~30Hz → 버스 50~100Hz 스무딩 (Franka의 1kHz 보간에 대응)
- [ ] 정책 워치독: 액션 수신 끊기면 자세 유지/토크 해제
- [ ] lerobot `Robot` 인터페이스 호환 래퍼 → record/ACT/pi0 파이프라인 직결
- [ ] (선택) ZMQ/gRPC 서버로 제어기-정책 프로세스 분리

## 3. Franka 안전장치 벤치마킹

**상세 대응표: [`docs/franka-safety.md`](docs/franka-safety.md)** (libfranka 헤더 기준 17개 항목, 우선순위 포함)

Franka Research 3의 안전 계층을 sopo 수준에서 재현:

- [x] **충돌 리플렉스** `sopo/reflex.py` 구현 + 테스트 9종 (Kimi 구현, Claude 리뷰 2 수정). **남음: 하드웨어 튜닝 (`docs/reflex-spec.md` 8절)**
- [ ] **명령 레이트 리미터**: Franka는 위치/속도/가속/저크 한계를 인터페이스에서 강제.
      → sopo: `clamp_goal`(위치·스텝)에 더해 속도/가속 프로파일 제한 추가
- [ ] **접촉 vs 충돌 2단 임계값**: 낮은 임계(접촉 감지→감속)와 높은 임계(충돌→정지) 구분
- [ ] **워크스페이스 제한**: 관절 소프트 리밋(완료) + 필요 시 URDF 기반 EE 워크스페이스 체크
- [ ] **모니터드 스톱**: 정지 시 토크 유지 홀드 vs 토크 해제 정책 정리 (중력에 낙하하는 관절 구분)
- [ ] 기존 완료분: 토크 30% 캡, 모터 과부하 자동 감쇠(34/35/36), 통신 워치독, 종료 시 토크 해제

## Future work

- [ ] **하드웨어 E-stop** — 현재 없음. 서보 전원 라인(12V)에 물리 비상정지 스위치(NC 접점, 래칭 버튼) 설치.
      소프트웨어로 대체 불가한 유일한 안전 계층 (Franka의 외부 활성화 장치/사용자 정지 버튼에 해당).
      설치 전까지는 사람 근처 운용 시 토크 캡 ≤ 300‰ + 전원 스위치 손 닿는 곳에 두기로 대체.
      Present_Voltage로 전원 차단을 감지해 소프트웨어 상태도 STOPPED로 동기화할 것.

## GitHub 이슈 (libfranka 수준으로 가기 위해 지금은 미룬 것 — 전제가 갖춰지면 착수)

- #1 외력 토크 추정 τ_ext (URDF/룩업) — https://github.com/redEddie/sopo/issues/1
- #2 URDF + 기구학 — https://github.com/redEddie/sopo/issues/2
- #3 sopod 하위 제어기 데몬 + IPC — https://github.com/redEddie/sopo/issues/3
- #4 reflex contact 단계 — https://github.com/redEddie/sopo/issues/4
- #5 물리 E-stop — https://github.com/redEddie/sopo/issues/5
- #6 레이트 리미터 (속도/가속 프로파일) — https://github.com/redEddie/sopo/issues/6
- #7 캡·리밋 EPROM 영구화 — https://github.com/redEddie/sopo/issues/7
- #8 J1 펌웨어 멀티턴 모드 시험 — https://github.com/redEddie/sopo/issues/8
- #9 SopoRobot + lerobot 어댑터 — https://github.com/redEddie/sopo/issues/9


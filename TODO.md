# sopo 로드맵

> **v0.1.0** (2026-08-28): 드라이버·관절 추상화·안전 계층·리플렉스·sopod 데몬까지. 다음 단계는 URDF(#2) → 외력 추정(#1) → SopoRobot/VLA(#9).

## 실측 하드웨어 구성 (2026-08-25 확인)

| 관절 | 모터 ID | 모델 | 비고 |
|---|---|---|---|
| J1 | 1 | sm8512bl | Return_Delay_Time 250→0 (sync_read 수정). **펌웨어 멀티턴 모드로 운용**: Phase bit4 ON, Min/Max_Position_Limit 0/0, Homing_Offset −1988 (2026-08-28 확정, +415° 연속 판독·조그 통과 확인). 단일턴 모드는 4095/0에서 긴 길로 돌아 ±5° 떨림 |
| J2 | 10, 11 | sts3250 ×2 | 듀얼, 반전 장착. 실측 pos10+pos11 ≈ **4005** (4095 아님 — 관절 추상화에서 상수로 보정) |
| J3 | 15, 16 | sts3250 ×2 | 듀얼, 반전 장착. 실측 pos15+pos16 ≈ **4100** |
| J4 | 19 | sts3250 | **연속 관절로 전환(2026-08-28)**: 12_continuous_setup --center 후 멀티턴, ±180° 케이블 보호. 하드웨어 설정은 사용자 실행 필요 |
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
- [x] 토크 제한 체감 테스트 — 07·jog에서 손으로 잡아 캡 포화 확인 (150‰ ≈ 7.5 kg·cm)
- [x] 관절별 소프트 리밋 실측 — J2, J3, J5 완료(EPROM). J1/J4는 연속 관절(±176° 클램프). J6은 EE 미장착이라 플레이스홀더(기본 200~3896), EE 장착 후 측정
- [x] 자중 토크 → 캡 확정: 정적 측정 ×1.5는 움직임에 부족(포화·리플렉스 오탐) → J1 200, J2 300, J3 250, J4/J5 200, J6 250. 자가진단 최대 부하 J2 224/300 등 여유 확인. EPROM 영구화 완료(2026-08-28, #7 닫힘)
- [x] 다관절 안전 루프 하드웨어 검증 — init.py(WaypointSource)로 6관절 대기 자세 도달 확인 (2026-08-28)
- [x] 키보드 조그 하드웨어 검증 (`examples/jog.py`) — J1 360° 통과 확인
- [x] 초기화 루틴 검증 (`examples/init.py`): 케이블 확인 → 자가진단 6관절 통과 → standby_pose 도달. standby는 13_capture_pose로 기록
- [x] 리플렉스 합격 (2026-08-28, `14_reflex_check --joint J4 --delta 300`): Phase A 왕복 10회 오탐 0, Phase B 손으로 잡기 → 포화 0.32s 후 COLLISION, 홀드, r 복구 6회 반복 성공. 기본값(sat 0.95 / 0.3s / progress 40) 유지

## 2. Robot API/SDK (Franka의 libfranka 포지션)

**목표 구조와 계층: [`docs/architecture.md`](docs/architecture.md)** — 버스는 `sopod` 데몬만, 상태 PUB/명령 REP로 프로세스 분리, GUI는 구독자

- [x] **관절 추상화** 1차: `sopo/joints.py` (single / dual K−goal / 케이블 제한 continuous + range_ticks 클램프).
      남음: 듀얼 쌍 K 일관성 검사, K 실측값 연동, sync_read/write 통합
- [x] sm8512bl Return_Delay_Time 250 → 0 설정 (EPROM) — sync_read 실패 근본 원인이었음.
      apply_safety()가 이제 전 모터에 0을 강제해 재발 방지
- [x] **sopod 데몬 v0** (2026-08-28, #3): 버스 독점(포트 락), 50Hz 루프, 모드 상태머신, ZMQ 상태 PUB/명령 REP/액션 SUB, 워치독 홀드, init/goto/recover, 클라이언트·CLI·jog_client. 가짜 버스 통합 테스트
- [ ] `SopoRobot` 클래스: `connect() / get_observation() / send_action()` — 이제 SopoClient 위에 올린다 (#9)
- [ ] 캘리브레이션 층: homing offset + 관절 범위 실측 → 틱 ↔ 정규화 좌표([-1,1] 또는 rad) 변환
- [ ] 명령 보간: 정책 10~30Hz → 버스 50~100Hz 스무딩 (Franka의 1kHz 보간에 대응)
- [x] 정책 워치독: 액션 0.5s 끊기면 홀드 (sopod StreamSource)
- [ ] lerobot `Robot` 인터페이스 호환 래퍼 → record/ACT/pi0 파이프라인 직결
- [x] ZMQ로 제어기-정책 프로세스 분리 (sopod)

## 3. Franka 안전장치 벤치마킹

**상세 대응표: [`docs/franka-safety.md`](docs/franka-safety.md)** (libfranka 헤더 기준 17개 항목, 우선순위 포함)

Franka Research 3의 안전 계층을 sopo 수준에서 재현:

- [x] **충돌 리플렉스** `sopo/reflex.py` 구현 + 테스트 9종 (Kimi 구현, Claude 리뷰 2 수정). **남음: 하드웨어 튜닝 (`docs/reflex-spec.md` 8절)**
- [ ] **명령 레이트 리미터**: Franka는 위치/속도/가속/저크 한계를 인터페이스에서 강제.
      → sopo: `clamp_goal`(위치·스텝)에 더해 속도/가속 프로파일 제한 추가
- [ ] **접촉 vs 충돌 2단 임계값**: 낮은 임계(접촉 감지→감속)와 높은 임계(충돌→정지) 구분
- [ ] **워크스페이스 제한**: 관절 소프트 리밋(완료) + 필요 시 URDF 기반 EE 워크스페이스 체크
- [x] **모니터드 스톱** (#10, 2026-08-28): 결함 시 Cat 2 홀드(freeze, 홀드 캡 600‰), 토크 해제는 명시 명령만. EPROM 상한 600
- [x] 완료분: 관절별 캡, 모터 과부하 감쇠, 통신 워치독, 브로드캐스트+검증 토크 해제, 리플렉스(충돌·추종·쌍·리밋·통신·과열), 리밋 브레이크 존, 블랙박스, 전압 감시

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

## 흔들림 측정 (2026-08-28, `16_wobble_test --move J2:+300,J3:-300`)

- J2/J3 왕복 중 J1/J4/J5/J6 출력축 이동 0~1틱, 도착 후 진동 0회 — **엔코더(출력축)에는 흔들림이 없음**
- 눈에 보이는 흔들림은 엔코더 뒤쪽 기계 유격/휨(혼-브래킷 결합, 프린트 링크, 베이스 고정)으로 판단. 토크·PID·프리로드 대상 아님
- 정정: 손으로 밀면 엔코더도 4틱 움직임 = 기어 백래시(스펙 0.43°). P 64는 효과 없음(틈 안에선 강성 0). 프리로드 8은 두 모터 스톨로 발열 + 물렁해짐 → **프리로드 0, P 32로 복귀**. 소프트웨어 해법 없음(엔코더가 출력축) — 받아들이거나 손목 서보 교체
- 다음: jog로 토크 유지 상태에서 링크를 손으로 흔들어 값이 안 변하는 관절 = 기계 보강 대상


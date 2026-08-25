# sopo 로드맵

## 1. 하드웨어 검증 (진행 중)

- [ ] 포트 권한 설정 (`dialout` 그룹)
- [ ] 버스 스캔: 모터 모델/ID 확인 (`cookbook/00_scan.py`)
- [ ] 전 모터 상태 읽기: 전압/온도/부하 정상 범위 확인 (`cookbook/01_read_state.py`)
- [ ] 말단 모터 1개 소구간 이동 테스트 — 낮은 토크 제한으로 (`cookbook/02_move_position.py`)
- [ ] 토크 제한 체감 테스트: 손으로 밀어 포화 확인 (`cookbook/03_torque_limits.py`)
- [ ] 관절별 소프트 리밋 실측 → `configs/arm.yaml` 작성
- [ ] 미러 텔레옵 검증 (`examples/mirror.py`)

## 2. Robot API/SDK (Franka의 libfranka 포지션)

- [ ] `SopoRobot` 클래스: `connect() / get_observation() / send_action()` 경계 확립
- [ ] 캘리브레이션 층: homing offset + 관절 범위 실측 → 틱 ↔ 정규화 좌표([-1,1] 또는 rad) 변환
- [ ] 명령 보간: 정책 10~30Hz → 버스 50~100Hz 스무딩 (Franka의 1kHz 보간에 대응)
- [ ] 정책 워치독: 액션 수신 끊기면 자세 유지/토크 해제
- [ ] lerobot `Robot` 인터페이스 호환 래퍼 → record/ACT/pi0 파이프라인 직결
- [ ] (선택) ZMQ/gRPC 서버로 제어기-정책 프로세스 분리

## 3. Franka 안전장치 벤치마킹

Franka Research 3의 안전 계층을 sopo 수준에서 재현:

- [ ] **충돌 리플렉스**: Franka는 외란 토크가 임계값을 넘으면 반사 정지.
      → sopo: `Present_Load`/`Present_Current` 상시 감시, 임계 초과 지속 시 정지 또는 후퇴(back-off)
- [ ] **명령 레이트 리미터**: Franka는 위치/속도/가속/저크 한계를 인터페이스에서 강제.
      → sopo: `clamp_goal`(위치·스텝)에 더해 속도/가속 프로파일 제한 추가
- [ ] **접촉 vs 충돌 2단 임계값**: 낮은 임계(접촉 감지→감속)와 높은 임계(충돌→정지) 구분
- [ ] **워크스페이스 제한**: 관절 소프트 리밋(완료) + 필요 시 URDF 기반 EE 워크스페이스 체크
- [ ] **모니터드 스톱**: 정지 시 토크 유지 홀드 vs 토크 해제 정책 정리 (중력에 낙하하는 관절 구분)
- [ ] 기존 완료분: 토크 30% 캡, 모터 과부하 자동 감쇠(34/35/36), 통신 워치독, 종료 시 토크 해제

# Franka(libfranka) 안전장치 벤치마킹 → sopo 대응표

출처: libfranka `include/franka/{robot.h, robot_state.h, errors.h, rate_limiting.h, lowpass_filter.h,
joint_velocity_limits.h, control_types.h}` (main, 2026-08). Desk/안전 PLC 쪽 기능은 문서 기준.

## 1. libfranka가 제공하는 안전 계층

### A. 접촉/충돌 감지 — `setCollisionBehavior()` + `RobotState`
- **외력 추정** `tau_ext_hat_filtered` = 측정 관절토크 − 동역학 모델 예측토크(중력·관성·마찰), 저역통과 필터 적용. 데카르트 버전 `O_F_ext_hat_K`/`K_F_ext_hat_K`.
- **2단 임계값**: lower = *contact*(비래칭, 외력이 빠지면 자동 해제, `joint_contact`), upper = *collision*(래칭, 즉시 **reflex stop** → `RobotMode::kReflex`, `automaticErrorRecovery()`로만 해제, `joint_collision`).
- **가속 구간 / 정속 구간 별도 임계값** (`*_acceleration` vs `*_nominal`): 가속 중엔 모델 오차가 커서 임계값을 높게 둔다.
- 관절 공간(7×Nm)과 데카르트 공간(6×N/Nm) 둘 다. 임계값은 재부팅 전까지 유지되고, 수동 안내(guiding) 중엔 내부적으로 상향.

### B. 명령 레이트 리미터 — `rate_limiting.h` (1 kHz 기준)
- 관절: 토크 변화율 1000 Nm/s, 저크 5000 rad/s³, 가속 10 rad/s², 속도는 **위치 의존 한계**(`joint_velocity_limits.h`: 관절 끝 근처에서 허용 속도가 줄고 `deceleration_limit` 적용).
- 데카르트: 병진 가속 9 m/s², 저크 4500; 회전 가속 17 rad/s², 저크 8500; 엘보 별도.
- `limitRate()`가 명령을 한계 안으로 자르고, 한계를 넘는 불연속 명령은 `*_discontinuity` 에러로 모션 중단.
- 1차 저역통과 필터(`kDefaultCutoffFrequency` 100 Hz)로 명령 스무딩 (`control(..., limit_rate, cutoff_frequency)`).

### C. 통신/실시간 감시
- 1 kHz 콜백 응답 필수. `control_command_success_rate` = 최근 100개 명령 성공 비율. 패킷 손실이 허용치를 넘으면 `communication_constraints_violation` → 정지. `RealtimeConfig::kEnforce`로 RT 커널 강제.

### D. 한계 위반 감시 — `errors.h` 플래그 (래칭)
`joint_position_limits_violation`, `cartesian_position_limits_violation`, `self_collision_avoidance_violation`,
`joint_velocity_violation`, `cartesian_velocity_violation`, `force_control_safety_violation`, `joint_reflex`,
`cartesian_reflex`, **`max_goal_pose_deviation_violation`, `max_path_pose_deviation_violation`**(추종 오차 한계),
`cartesian_velocity_profile_safety_violation`, `*_motion_generator_start_pose_invalid`(시작 자세 불일치 → 점프 방지),
`*_velocity_discontinuity`, `*_acceleration_discontinuity`, `controller_torque_discontinuity`,
`communication_constraints_violation`, `power_limit_violation`, `tau_j_range_violation`, `instability_detected`,
`joint_move_in_wrong_direction`, `joint_p2p_insufficient_torque_for_planning`, ...

### E. 상태 머신과 복구
- `RobotMode {Idle, Move, Guiding, Reflex, UserStopped, AutomaticErrorRecovery}`. 에러는 래칭(`current_errors`, `last_motion_errors`), `stop()`으로 모션 선점, `automaticErrorRecovery()`로 복구.

### F. libfranka 밖 (하드웨어/Desk 안전 PLC) — 소프트웨어로 대체 불가
- 외부 활성화 장치(enabling device), 사용자 정지 버튼, 안전 등급 모니터드 스톱, 속도 제한(SLS), 워크스페이스 안전 영역. 이중화된 관절 토크 센서·엔코더.

## 2. sopo 대응표

| # | Franka 메커니즘 | sopo 현재 | 대응 방안 | 우선순위 |
|---|---|---|---|---|
| 1 | 외력 추정 τ_ext (모델 기반) | 없음. `Present_Load`(PWM 듀티)·`Present_Current`만 있음 | 모델 대신 **예상 부하 룩업**: 06으로 자세별 정지 부하를 기록 → τ_ext ≈ 실측 − 예상. 1단계는 절대 부하로 시작 | 중 |
| 2 | 2단 임계값 contact(비래칭)/collision(래칭+reflex) | 없음 | `sopo/reflex.py`: contact = 부하 ≥ 캡 70% → 감속·경고, 자동 해제 / collision = 부하 ≥ 캡 95%가 300 ms 지속 **그리고** 위치 오차가 줄지 않음 → 정지(목표=현재) 래칭, `recover()` 필요. 실측 근거: 무부하 이동 ≤ 캡 85%, 손으로 잡으면 캡 포화 | **높음** |
| 3 | 가속/정속 구간 별도 임계값 | 없음 | 목표가 바뀐 직후 N ms(가속 구간)는 임계값 상향 | 중 |
| 4 | 추종 오차 한계 (`max_path_pose_deviation`) | 없음 | 목표−실측 오차가 N틱 이상 M ms 지속 → 정지. 충돌 보조 신호 + "모터가 못 따라감"(캡 부족·걸림) 감지. 구현 쉬움 | **높음** |
| 5 | 시작 자세 불일치 거부 (`start_pose_invalid`) | 부분 — mirror의 소프트스타트 | `SopoRobot.send_action`: 첫 명령이 현재 자세와 임계 이상 다르면 거부 또는 보간 | 높음 |
| 6 | 레이트 리미터 (속도·가속·저크, 위치 의존 속도 한계) | 부분 — `max_relative_target`(스텝=속도 상한), 모터 `Acceleration` 레지스터 | 호스트 리미터: 스텝을 **관절 한계 근접도에 따라 축소**(Franka의 position-based velocity limit), 스텝 변화량 제한(가속) | 중 |
| 7 | 명령 저역통과 필터 | 없음 | `robot.py` 보간기: 정책 10~30 Hz → 50 Hz 선형 보간 + EMA | 중 |
| 8 | 통신 감시 (`success_rate`, constraints violation) | 있음 — mirror 5회 연속 오류 시 토크 해제 | 공통 모듈로 이동, 최근 100사이클 성공률 노출, **정책 워치독**(명령 끊기면 홀드→토크 해제) | 높음 |
| 9 | 관절 위치 한계 | 있음 — 호스트 `clamp_goal` + 모터 EPROM `Min/Max_Position_Limit` | J1 랩 해결 후 전 관절 EPROM 기록 (`08_persist_caps`) | 진행 중 |
| 10 | 관절 속도 위반 감시 | 없음 | `Present_Velocity` 감시, 상한 초과 시 정지 | 낮음 |
| 11 | 토크 범위/전력 한계 (`tau_j_range`, `power_limit`) | 있음 — `Torque_Limit`, 과부하 3종(34/35/36), `Protection_Current` 옵션 | `Max_Torque_Limit` EPROM 영구화 | 진행 중 |
| 12 | 온도/전압 | Franka 내부 처리 | mirror의 65 °C 경고 → 공통 모듈, 70 °C 토크 해제, 저전압(<11 V) 경고 | 중 |
| 13 | 불안정 감지 (`instability_detected`) | 없음 | 위치 오차 부호가 짧은 주기로 교대하면 게인/캡 문제로 정지 | 낮음 |
| 14 | 데카르트 힘 임계값, 자기충돌 회피, 워크스페이스 영역 | 없음 (URDF 없음) | URDF + 자코비안 이후. 전까지는 관절 한계로 대체 | 후순위 |
| 15 | RobotMode 상태 머신 + 래칭 에러 + `automaticErrorRecovery` | 없음 | `SopoRobot.mode ∈ {IDLE, MOVE, GUIDING(토크 OFF), REFLEX, STOPPED}`, 에러 래칭, `recover()` | 높음 |
| 16 | 외부 활성화 장치 / 비상정지 (하드웨어) | 없음 | **소프트웨어로 대체 불가** — 서보 전원 라인에 물리 E-stop 스위치. 키보드 소프트 정지는 보조 수단 | **높음(하드웨어)** |
| 17 | Guiding(수동 안내) | 있음 — 토크 OFF 리더 모드 | 그대로 | 완료 |

## 3. Kimi 구현 순서 제안

1. `sopo/reflex.py` — 공통 감시 모듈: 추종 오차 한계(#4) + 부하 포화 충돌(#2) + 통신/온도(#8, #12). 결과는 래칭 이벤트, `recover()`로 해제. mirror.py의 워치독을 여기로 이전.
2. `SopoRobot` 상태 머신(#15)과 시작 자세 검사(#5), 정책 워치독(#8).
3. 레이트 리미터(#6)와 보간 필터(#7).
4. 가속 구간 임계값(#3), 예상 부하 룩업(#1).
5. URDF 작성 후 #14.

하드웨어 E-stop(#16)은 코드와 무관하게 지금 달 것.

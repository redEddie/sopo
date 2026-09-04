# 5_gripper — 그리퍼 추가 (작업 브리프, 플레이스홀더)

이 폴더는 그리퍼 추가 작업용이다. 새로 참여하는 사람이 맡을 수 있게 범위와 순서를 적어 둔다.
아직 스크립트는 없다 — 작업하면서 여기에 둔다.

## 작업 브리프

대상: **SO-ARM100/101 Parallel Gripper (robonine)**를 플랜지에 장착하고, 토크 모델·안전 캡·캘리브레이션을 갱신한다.

1. **기구 장착**: 그리퍼를 플랜지(URDF의 `flange` 프레임)에 체결하고 핑거 구동 서보를 버스에 데이지체인 연결한다.
   새 서보 ID는 `cookbook/1_setup/120_set_motor_id.py`로 잡고, `configs/arm.yaml`의 `joints`에 J7로 추가한다.
2. **질량 실측 주입**: `description/link_masses.yaml`의 ee/핑거 항목을 실측값으로 바꾼다
   (지금은 추정치 — `description/README.md` 참조).
3. **모델 재생성**: `description/postprocess.py`를 다시 실행해 URDF(`robot.urdf`/`arm_no_ee.urdf`)에 반영한다.
4. **리밋 측정**: 그리퍼 서보의 위치 한계를 `cookbook/2_pose_calibration/200_find_limits.py --ids <새ID>`로 실측한다.
5. **중력 재검증**: 무게가 늘었으므로 `cookbook/4_torque_model/410_gravity_check.py`로 모델-실측을 다시 대조하고
   필요하면 `--scale` 보정을 다시 돌린다.
6. **핑거 구동**: 핑거는 mimic 조인트(URDF상)다 — 실제 구동은 서보 1개, URDF의 두 핑거는 mimic으로 따라간다.
   조인트 정의/명령 변환 시 이 점에 주의할 것.

완료 기준: 그리퍼 포함 전 관절이 `examples/init.py` 자가진단을 통과하고, 410 비교표의 외력 열이 0 근처.

관련 파일: `configs/arm.yaml`, `configs/calibration.yaml`, `description/link_masses.yaml`,
`description/postprocess.py`, `description/README.md`, `sopo/model/dynamics.py`.

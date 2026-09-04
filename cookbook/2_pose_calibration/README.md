# 2_pose_calibration — 자세와 범위의 이해

팔이 움직이기 시작하면 다음은 "어디까지 움직여도 되는가"와 "모터 틱이 로봇 모델의 각도와 어떻게 대응하는가"다.
여기서 만든 `configs/calibration.yaml`(position_limits, gravity 섹션)과 `arm.yaml`(standby_pose, range_ticks)이
이후 안전 캡·토크 모델의 입력이 된다.

## 스크립트

### 200_find_limits.py — 관절 최소/최대 위치 기록 (손으로)

```bash
python cookbook/2_pose_calibration/200_find_limits.py --port /dev/ttyACM0 --ids 19
python cookbook/2_pose_calibration/200_find_limits.py --port /dev/ttyACM0 --ids 15,16 --write-eprom
```

토크를 끄고 관절을 손으로 양끝까지 움직이면 min/max를 실시간으로 기록한다. `Ctrl+C`로 끝내면 안쪽으로 `--margin`(기본 50틱)을 줄인 값을 `configs/calibration.yaml`의 `position_limits`에 저장한다. `--write-eprom`을 주면 모터의 `Min/Max_Position_Limit`(EPROM)에도 써서 펌웨어가 범위 밖 목표를 거부하게 만든다(소프트웨어 버그에 대한 2차 방어선). 듀얼 모터 관절은 두 ID를 함께 지정한다. 0/4095 랩 지점 근처면 경고한다.

### 210_continuous_angle.py — 연속 관절(J1) 허용 범위 정하기

```bash
python cookbook/2_pose_calibration/210_continuous_angle.py --port /dev/ttyACM0 --id 1
```

전선이 풀린 자세에서 실행하면 그 자세를 0°로 잡고, 손으로 돌리는 동안 랩(4095→0)을 자동 처리한 연속 각도를 표시한다. 전선이 당기기 시작하는 각도를 양쪽에서 읽어 `joints` 설정의 `range_ticks`로 넣는다.

### 211_continuous_setup.py — 연속 관절(J1/J4) 서보 설정

```bash
# 관절을 케이블 풀린 자세에 놓고 (토크 OFF)
python cookbook/2_pose_calibration/211_continuous_setup.py --port /dev/ttyACM0 --id 19 --center
python cookbook/2_pose_calibration/210_continuous_angle.py --port /dev/ttyACM0 --id 19     # 손으로 360° 넘게 돌려 검증
```

센터링(현재 자세 = 2048, 단일턴 모드에서) → 펌웨어 멀티턴(Phase bit4 ON, Min/Max_Position_Limit 0/0) 순서로 기록한다. 멀티턴 모드에서는 서보가 4095/0 경계를 스스로 넘어 위치를 연속으로 보고한다. 단일턴 모드는 경계에서 긴 길로 돌아 ±5° 떨림이 난다(실측). 멀티턴 모드에서 `Homing_Offset`을 바꾸면 예측 불가하므로 센터링은 반드시 먼저. 전원을 끄면 바퀴 수가 초기화되므로 `arm.yaml`의 `home_abs: 2048`과 `range_ticks: 2048`(±180°)로 가장 가까운 2048을 집으로 잡아 케이블을 보호한다.

### 220_capture_pose.py — 대기 자세 기록

```bash
# 토크 OFF 상태에서 손으로 원하는 자세를 만든 뒤
python cookbook/2_pose_calibration/220_capture_pose.py --config configs/arm.yaml            # arm.yaml의 standby_pose 갱신
python cookbook/2_pose_calibration/220_capture_pose.py --config configs/arm.yaml --key rest_pose
```

관절 값을 읽어 `arm.yaml`에 한 줄로 기록한다. 연속 관절은 `home_abs`(2048) 기준 프레임으로 저장하므로 전원을 껐다 켜서 바퀴 수가 달라져도 같은 물리 자세를 가리킨다. `examples/init.py`가 자가진단 후 이 자세로 이동한다.

### 230_calibrate_zero.py — URDF zero pose 관절별 캘리브레이션

```bash
python cookbook/2_pose_calibration/230_calibrate_zero.py             # J1~J6 순차
python cookbook/2_pose_calibration/230_calibrate_zero.py --joint J5  # 한 관절만 다시
```

토크 OFF. 관절마다 zero 방향 안내가 뜨고, 손으로 맞춘 뒤 Enter로 기록한다. `gravity.zero_ticks`만 갱신된다(dir/scale 유지). 기준 자세는 `description/viewer.py` 초기 화면(슬라이더 전부 0).

### 231_check_directions.py — 모터 방향 ↔ URDF +q 부호 확인 (손 가이드)

```bash
python cookbook/2_pose_calibration/231_check_directions.py              # 전 관절 순차
python cookbook/2_pose_calibration/231_check_directions.py --joint J2   # 한 관절만
```

토크 OFF. 안내된 +q 방향(컨벤션 표 참조)으로 손으로 움직이면 틱 부호로 `gravity.dir`을 자동 판정한다.

## 완료 기준

- `calibration.yaml`에 전 관절 `position_limits`가, J1/J4는 `range_ticks`/`home_abs`가 arm.yaml에 잡혀 있다.
- `standby_pose`가 기록돼 `examples/init.py`가 그 자세로 간다.
- `gravity.zero_ticks`/`gravity.dir`이 전 관절에 있다 (토크 모델 검증의 입력). → `3_safety_torque`로.

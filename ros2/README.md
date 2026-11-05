# SOPO 플랜지 MoveIt 2

`description/arm_no_ee.urdf`의 `base_link → flange` 6축 체인을 사용한다. MoveIt은
계획을 `/sopo_arm/follow_joint_trajectory`로 보내고, 브리지는 기존 `sopod` 제어 리스로
실행한다. 플랜지 자세를 그리퍼 없이 시험하는 설정이다.

## 실행

Ubuntu 22.04에서 ROS 2 Humble과 MoveIt 2, RViz, robot_state_publisher가 설치된 환경 기준.
SOPO 체크아웃의 루트에서 실행한다. ROS 2의 Python과 같은 버전으로 가상환경을 만든다.

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -e .
python "$(command -v ros2)" launch "$PWD/ros2/sopo_moveit.launch.py" mode:=mock log_dir:="$PWD/logs/tracking"
```

기본 모드는 `mock`이다. 실기에서는 별도 터미널에서 같은 SOPO 환경을 활성화한 다음
`sopod --config configs/arm.yaml`을 실행하고, 위 launch의 `mode:=hardware`를 사용한다.
브리지와 데몬은 같은 머신의 기본 ZMQ 포트를 사용하며 동일한 arm/calibration 설정을 사용해야 한다.

1. 토크 OFF 상태에서 RViz 자세와 실제 팔 자세를 비교한다.
2. 움직임 범위를 비우고 `python -m sopo.runtime.cli move`를 실행한다. 시작 엔코더 자세를
   유지하고, 실측 범위 밖에 있는 관절만 가까운 한계에서 최대 12틱 안쪽으로 보정한다.
   듀얼 모터는 기준·미러 위치 한계를 함께 적용한다. `cli status`에
   `ENTERING_RANGE`가 표시되는 동안 제어 리스를 발급하지 않는다.
   보정이 끝나기 전에 실패하면 REFLEX 상태와 최근 이벤트를 확인한다.
3. RViz에서 **Interact** 도구를 고른 뒤, **MotionPlanning → Planning**의 시작 자세를
   `<current>`로 두고 목표를 현재 자세 가까이에서 조절한다. Velocity/Accel Scaling은
   첫 시험에서 `0.10`을 사용한다. **Plan**을 눌러 경로를 확인하고 **Execute**를 누른다.
   MoveIt은 관절별 계획 속도 상한 1.0 rad/s에 RViz의 속도 스케일을 적용한다.
4. 종료하거나 토크를 끌 때는 팔을 지지한다. RViz **Stop**은 궤적을 취소하고 데몬이
   현재 자세를 유지하게 한다. `python -m sopo.runtime.cli idle`은 토크를 해제한다.

## 추종 기록

실행마다 CSV를 남긴다. PlotJuggler에서 `elapsed_s`를 시간으로 사용해 각
`joint_N_desired_rad`(계획), `joint_N_applied_rad`(sopod 적용 목표),
`joint_N_actual_rad`(엔코더), `joint_N_error_rad`(계획 − 엔코더)를 확인한다.
이동 중 오차는 기록한다. 궤적 시간이 끝나면 최종 목표를 계속 전송하며,
0.035 rad 이내를 0.2초 유지하면 실행 성공으로 표시한다. 이 기록은 관절 추종 시험용이다. 플랜지 절대 위치의 정확도는
외부 측정이 필요하다.

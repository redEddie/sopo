# sopo

HopeJr를 분해해 제작한 자체 중형 로봇암을 위한 제어기.

Feetech STS/SMS 버스 서보(STS3215/STS3250 등)를 직접 레지스터 레벨에서 다루는 경량 드라이버와,
**사람이 다치지 않도록 모터 토크(effort)에 제한을 건** 안전 계층, 그리고 리더-팔로워 미러
텔레오퍼레이션 예제를 담고 있다.

## 레퍼런스

| 출처 | 가져온 것 |
|---|---|
| [lerobot Feetech 드라이버](https://github.com/huggingface/lerobot/tree/main/src/lerobot/motors/feetech) | 레지스터 테이블, scservo_sdk 사용법·타임아웃 패치, sign-magnitude 인코딩, Lock 처리 |
| [lerobot HopeJr](https://github.com/huggingface/lerobot/tree/main/src/lerobot/robots/hope_jr) | sync_read → clamp → sync_write 제어 루프, `max_relative_target` 안전 캡, 가속도 제한 |
| [Trossen WidowX 250s puppet](https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/examples/interbotix_xsarm_puppet) | 미러 구조: 마스터 토크 OFF + 퍼펫 position 모드, 고정 주기 관절각 미러링 |

## 설치

```bash
pip install -e .
# 시리얼 포트 권한 (재로그인 필요)
sudo usermod -aG dialout $USER
```

## 구성

```
sopo/            핵심 패키지
  registers.py     STS/SMS 컨트롤 테이블 + sign-magnitude 인코딩
  bus.py           FeetechBus: ping/scan/read/write/sync_read/sync_write/torque
  safety.py        SafetyLimits + 토크 제한/과부하 보호/소프트 리밋/스텝 클램핑
cookbook/        Feetech 기초 조작 쿡북 (스캔 → 상태 읽기 → 이동 → 토크 제한 → ID 설정)
examples/
  mirror.py        리더-팔로워 미러 텔레오퍼레이션
configs/
  arm.example.yaml 암 정의(포트/ID/안전 제한) 예시
```

## 빠른 시작

```bash
# 1. 버스에 뭐가 붙어있는지 확인
python cookbook/00_scan.py --port /dev/ttyACM0

# 2. 토크 끈 채로 상태 모니터링
python cookbook/01_read_state.py --port /dev/ttyACM0 --ids 1,2,3,4,5,6,7

# 3. 토크 제한 걸고 한 관절 이동
python cookbook/02_move_position.py --port /dev/ttyACM0 --id 1 --goal 2048

# 4. 미러 텔레오퍼레이션
cp configs/arm.example.yaml configs/arm.yaml   # 포트/ID 수정
python examples/mirror.py --config configs/arm.yaml
```

## 안전 설계

토크를 켜는 모든 코드는 반드시 `apply_safety()`를 먼저 호출한다. 3중 안전 계층:

1. **모터 내부 토크 캡** — `Torque_Limit`(RAM, 48) 기본 30%. `persist_torque_limit()`로
   `Max_Torque_Limit`(EPROM, 16)에 영구 저장하면 전원을 다시 켜도 캡이 유지된다.
2. **모터 내부 과부하 차단** — 허용 토크의 80%(`Overload_Torque`)로 500ms(`Protection_Time`)
   이상 밀면 20%(`Protective_Torque`)로 자동 강하. 사람이나 장애물에 끼었을 때 스스로 힘을 뺀다.
3. **호스트 측 클램핑** — 관절 소프트 리밋 + 제어 주기당 최대 이동량(`max_relative_target`)
   제한으로 목표치 점프를 차단. 미러 루프는 통신 오류 연속 5회 시 토크를 해제하고 정지한다.

사람 옆에서 돌릴 때는 `torque_limit`을 400(40%) 이하로 유지할 것.

## 라이선스

Apache-2.0. 레지스터 테이블은 lerobot(Apache-2.0)에서 가져와 수정했다.

# 다이나믹셀 + Feetech 혼용 실험 메모 (2026-09-03)

sopo 본 코드와 무관한 하드웨어 실험 기록. 미래에 다이나믹셀을 도입할 때 참고용.

## 결론

- **Feetech 드라이버 보드(CH343, `/dev/sopo_follower`)에 다이나믹셀을 같이 올리는 혼용은 실패.**
  케이블 2종·단독 연결 모두 무응답. 보드 DATA 회로(5V TTL 구동)와 다이나믹셀 X 시리즈
  DATA(3.3V 로직)의 레벨/구동 호환성 문제로 추정. 케이블·설정으로 해결 불가, 보드 회로 특성.
- **U2D2 + 다이나믹셀은 정상.** 권장 구성은 어댑터 2개 분리:
  - `/dev/sopo_follower` (CH343) → Feetech 8개 (ID 1,10,11,15,16,19,20,21)
  - `/dev/sopo_dynamixel` (U2D2, FTDI FT232H 0403:6014 serial FTAAMMJV) → 다이나믹셀
  - udev 규칙은 `configs/99-sopo.rules`에 둘 다 등록됨.

## 확인된 하드웨어 사실

- **핀 순서는 같다**: 양쪽 모두 1=GND, 2=VDD, 3=DATA (다이나믹셀 JST EHR-03 / Feetech 5264-3P).
  배선 역접속 문제가 아니었음. 커넥터 하우징 모양만 다름.
- 프로토콜이 달라도(Feetech 자체 vs Dynamixel Protocol 2.0) 같은 TTL 선에 섞이면
  서로 상대 패킷을 무시하므로 **버스 공유 자체는 이론상 가능**. 보안/페어링 절차는 없음.
- Feetech 모터들이 응답하는 동안 같은 버스의 다이나믹셀만 무응답 → 전원·포트 문제 배제됨.

## 보유 다이나믹셀 인벤토리

- **XL430-W250-T × 6**, ID 1~6, **보드레이트 1 Mbps**(출고 기본 57600에서 이미 변경됨), FW v50,
  모델번호 1060 (Protocol 2.0 핑 응답 기준).
- 스펙: 12V, 스톨 토크 1.5 N·m @12V, 해상도 4096 (Feetech과 동일), TTL 3핀.
- **전류제어 불가** (전류 센서 없음, XL 시리즈). 지원 모드: 위치/확장위치/속도/PWM.
  전류제어 필요하면 XC330/XM430 이상 (XW/XH/XM/X330만 전류 기반 제어 지원).
- Feetech 버스와 ID 1이 겹치지만 어댑터 분리 구성에서는 무관.

## 다이나믹셀 스캔 코드 (재사용용)

Protocol 2.0 방송 핑 스캐너. pyserial만 필요 (`pip install pyserial`).
`python3 dxl_scan.py /dev/sopo_dynamixel` 처럼 실행.

```python
"""Broadcast-ping scan for Dynamixel Protocol 2.0 devices."""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/sopo_dynamixel"
BAUDS = [57600, 115200, 1000000, 2000000, 3000000, 4000000, 9600, 460800]


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def ping_packet(dev_id: int) -> bytes:
    pkt = b"\xff\xff\xfd\x00" + bytes([dev_id, 0x03, 0x00, 0x01])
    crc = crc16(pkt)
    return pkt + bytes([crc & 0xFF, crc >> 8])


for baud in BAUDS:
    ser = serial.Serial(PORT, baud, timeout=0.05)
    ser.reset_input_buffer()
    ser.write(ping_packet(0xFE))  # broadcast ping
    ser.flush()
    end = time.time() + 0.2
    buf = b""
    while time.time() < end:
        buf += ser.read(ser.in_waiting or 1)
    while True:
        i = buf.find(b"\xff\xff\xfd\x00")
        if i < 0 or len(buf) < i + 9:
            break
        plen = buf[i + 5] | (buf[i + 6] << 8)
        total = 7 + plen  # Length 필드에 CRC 2바이트가 이미 포함됨
        if len(buf) < i + total:
            break
        pkt, buf = buf[i : i + total], buf[i + total :]
        if crc16(pkt[:-2]) == (pkt[-2] | (pkt[-1] << 8)) and pkt[7] == 0x55 and pkt[8] == 0:
            model = pkt[9] | (pkt[10] << 8)
            print(f"baud={baud}  ID={pkt[4]}  model={model}  fw={pkt[11]}")
    ser.close()
print("scan done")
```

주의: Protocol 2.0의 Length 필드는 CRC 2바이트를 포함하므로 패킷 전체 길이는 `7 + Length`
(처음에 `7 + Length + 2`로 계산해서 응답을 놓친 적 있음).

Feetech 쪽 스캔은 기존 `cookbook/00_scan.py` 사용.

## 향후 도입 시 작업 후보

1. `sopo/hal/bus.py`의 FeetechBus에 대응하는 DynamixelBus (Protocol 2.0, 제어 테이블 주소 맵 신규).
   SDK는 `pip install dynamixel-sdk` 또는 위 스캔 코드처럼 raw 패킷 직접 구성.
2. `sopo/safety/limits.py`의 `MODEL_STALL_TORQUE_KGCM`에 xl430 추가 시 주의: XL430은 토크 리밋이
   아니라 PWM 리밋 방식 (주소 36 PWM_Limit, 0~885).
3. `configs/arm.yaml`에 포트를 버스별로 분리하는 설정 확장 필요 (현재 단일 `baudrate`/포트 구조).

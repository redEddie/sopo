#!/usr/bin/env python3
"""버스에 연결된 Feetech 서보를 여러 통신 속도로 검색한다.

무응답 ID마다 타임아웃을 기다리므로 범위가 넓으면 오래 걸린다.
기본은 ID 0~30만 스캔하며, 필요하면 --max-id 253으로 전체를 검색한다.

예시:
    python cookbook/1_setup/100_scan.py --port /dev/ttyACM0
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse

from sopo import FeetechBus


BAUDRATES = [1_000_000, 500_000, 250_000, 128_000, 115_200, 57_600]


def main() -> None:
    parser = argparse.ArgumentParser(description="Feetech 버스 서보 스캔")
    parser.add_argument("--port", default="/dev/ttyACM0", help="시리얼 포트")
    parser.add_argument("--max-id", type=int, default=30, help="스캔할 최대 ID (전체는 253)")
    args = parser.parse_args()

    found_by_id: dict[int, tuple[int, str]] = {}
    duplicates: list[tuple[int, int, str]] = []  # (id, baud, model)

    print(f"{'Baud':>10} | {'ID':>4} | Model")
    print("-" * 30)

    for baud in BAUDRATES:
        bus = FeetechBus(args.port, baudrate=baud)
        try:
            bus.connect()
            scan = bus.scan(range(0, args.max_id + 1))
        except Exception as exc:  # noqa: BLE001
            print(f"{baud:>10} | 연결 실패: {exc}")
            continue
        finally:
            try:
                bus.disconnect()
            except Exception:  # noqa: BLE001
                pass

        for motor_id, model in sorted(scan.items()):
            print(f"{baud:>10} | {motor_id:>4} | {model}")
            if motor_id in found_by_id:
                duplicates.append((motor_id, baud, model))
            else:
                found_by_id[motor_id] = (baud, model)

    if duplicates:
        print("\n[경고] 다음 ID가 여러 보드레이트에서 응답했습니다. ID 충돌 가능성:")
        for motor_id, baud, model in duplicates:
            print(f"  ID {motor_id} @ {baud} ({model})")

    if not found_by_id:
        print("\n발견된 모터가 없습니다. 전원/GND/포트를 확인하세요.")
    else:
        print(f"\n총 {len(found_by_id)}개 고유 ID 발견.")


if __name__ == "__main__":
    main()

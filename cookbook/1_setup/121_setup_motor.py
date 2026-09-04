#!/usr/bin/env python3
"""버스에 모터가 1개만 연결된 상태에서 ID와 보드레이트를 변경한다.

주의: ID와 Baud_Rate는 쓰는 즉시 적용된다. 따라서 잠금 해제/재잠금(Lock)은
반드시 "그 시점에 모터가 응답하는 ID/보드레이트"로 보내야 한다.

예시:
    python cookbook/1_setup/121_setup_motor.py --port /dev/ttyACM0 --new-id 2
    python cookbook/1_setup/121_setup_motor.py --port /dev/ttyACM0 --current-id 1 --new-id 2 --new-baud 500000
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse

from sopo import FeetechBus


# Baud_Rate 레지스터 값
BAUD_TO_REG = {
    1_000_000: 0,
    500_000: 1,
    250_000: 2,
    128_000: 3,
    115_200: 4,
    57_600: 5,
    38_400: 6,
    19_200: 7,
}


def main() -> None:
    parser = argparse.ArgumentParser(description="모터 ID/보드레이트 설정")
    parser.add_argument("--port", default="/dev/ttyACM0", help="시리얼 포트")
    parser.add_argument("--baud", type=int, default=1_000_000, help="현재 보드레이트")
    parser.add_argument("--current-id", type=int, default=None, help="현재 ID (미지정 시 스캔)")
    parser.add_argument("--new-id", type=int, required=True, help="새 ID")
    parser.add_argument("--new-baud", type=int, default=None, help="새 보드레이트 (정수 Hz)")
    parser.add_argument("--max-id", type=int, default=30, help="스캔할 최대 ID")
    args = parser.parse_args()

    if not (0 <= args.new_id <= 253):
        raise SystemExit("새 ID는 0~253 범위여야 합니다.")

    if args.new_baud is not None and args.new_baud not in BAUD_TO_REG:
        raise SystemExit(
            f"지원하지 않는 보드레이트입니다: {args.new_baud}. "
            f"가능한 값: {sorted(BAUD_TO_REG.keys())}"
        )

    bus = FeetechBus(args.port, baudrate=args.baud)
    bus.connect()

    try:
        scan = bus.scan(range(0, args.max_id + 1))
        if len(scan) != 1:
            raise SystemExit(
                f"버스에 연결된 모터가 1개여야 합니다. 발견: {len(scan)}개 {list(scan.keys())} "
                f"(ID {args.max_id} 초과 모터는 --max-id로 범위를 넓혀 확인)"
            )

        current_id = args.current_id
        detected_id = next(iter(scan))
        if current_id is None:
            current_id = detected_id
        elif current_id != detected_id:
            raise SystemExit(
                f"지정한 현재 ID({current_id})와 스캔 결과({detected_id})가 다릅니다."
            )

        change_baud = args.new_baud is not None and args.new_baud != args.baud
        if current_id == args.new_id and not change_baud:
            print("변경할 내용이 없습니다.")
            return

        print(f"모터 ID {current_id} -> {args.new_id} 설정 시작")

        # EPROM 쓰기 잠금 해제. 이후 단계마다 모터가 응답하는 주소가 바뀌므로
        # eprom_unlocked() 컨텍스트 대신 수동으로 Lock을 관리한다.
        bus.write("Lock", current_id, 0)
        active_id = current_id

        # 보드레이트는 쓰는 즉시 적용되므로, 먼저 바꾸고 새 속도로 재접속한다.
        if change_baud:
            bus.write("Baud_Rate", active_id, BAUD_TO_REG[args.new_baud])
            print(f"보드레이트 변경: {args.baud} -> {args.new_baud}, 재접속합니다.")
            bus.disconnect()
            bus = FeetechBus(args.port, baudrate=args.new_baud)
            bus.connect()

        # ID도 쓰는 즉시 적용된다. 이후 통신은 새 ID로 한다.
        if active_id != args.new_id:
            bus.write("ID", active_id, args.new_id)
            active_id = args.new_id

        bus.write("Lock", active_id, 1)

        model = bus.ping(active_id)
        if model is None:
            raise SystemExit("검증 실패: 새 ID로 응답이 없습니다.")
        print(f"검증 성공: ID {active_id} ({model}) 응답 확인")
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()

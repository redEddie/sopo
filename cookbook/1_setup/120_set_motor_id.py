#!/usr/bin/env python3
"""버스에서 특정 ID를 가진 모터의 ID를 변경한다.

ID는 EPROM 영역(주소 5)에 있어 쓰기 전 Lock을 해제해야 하며, 값을 쓰는 즉시 적용된다.
따라서 잠금 해제 → ID 쓰기 → 새 ID로 잠금 → 새 ID로 ping 검증 순서를 따른다.

이 스크립트는 현재 ID를 알고 있을 때 쓴다. 모터를 하나씩 분리해 ID를 찾아야 하는
상황이라면 00_scan.py + 04_setup_motor.py를 사용한다.

예시:
    python cookbook/1_setup/120_set_motor_id.py --port /dev/ttyACM0 --current-id 1 --new-id 2
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse

from sopo import FeetechBus


def main() -> None:
    parser = argparse.ArgumentParser(description="모터 ID 설정")
    parser.add_argument("--port", default="/dev/ttyACM0", help="시리얼 포트")
    parser.add_argument("--baud", type=int, default=1_000_000, help="통신 속도")
    parser.add_argument("--current-id", type=int, required=True, help="현재 모터 ID")
    parser.add_argument("--new-id", type=int, required=True, help="새 모터 ID")
    args = parser.parse_args()

    if not (0 <= args.current_id <= 253):
        raise SystemExit("현재 ID는 0~253 범위여야 합니다.")
    if not (0 <= args.new_id <= 253):
        raise SystemExit("새 ID는 0~253 범위여야 합니다.")
    if args.current_id == args.new_id:
        print("현재 ID와 새 ID가 같습니다. 변경할 내용이 없습니다.")
        return

    bus = FeetechBus(args.port, baudrate=args.baud)
    bus.connect()

    try:
        model = bus.ping(args.current_id)
        if model is None:
            raise SystemExit(f"ID {args.current_id}로 응답하는 모터가 없습니다.")
        print(f"감지: ID {args.current_id} ({model})")

        if bus.ping(args.new_id) is not None:
            raise SystemExit(
                f"ID {args.new_id}를 사용하는 모터가 이미 버스에 있습니다. "
                "중복 ID를 만들지 않도록 먼저 해결하세요."
            )

        # ID는 EPROM이므로 Lock을 해제해야 쓸 수 있다. ID는 쓰는 즉시 적용되므로
        # eprom_unlocked() 컨텍스트를 쓰면 finally에서 잘못된(옛) ID로 잠금을 시도한다.
        # 따라서 수동으로 Lock을 관리한다.
        bus.write("Lock", args.current_id, 0)
        bus.write("ID", args.current_id, args.new_id)
        bus.write("Lock", args.new_id, 1)

        model = bus.ping(args.new_id)
        if model is None:
            raise SystemExit("검증 실패: 새 ID로 응답이 없습니다.")
        print(f"완료: ID {args.current_id} -> {args.new_id} ({model})")
    finally:
        bus.disconnect()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""sopod 클라이언트 조그: 키보드 → 액션 스트림. 데몬이 클램프·리플렉스·토크를 책임진다.

터미널 1: sopod --config configs/arm.yaml
터미널 2: python examples/jog_client.py
키: ←/→(a/d) 이동  ↑/↓(w/s)·1~6 관절  +/- 스텝  space 홀드  m MOVE(토크 ON)  g GUIDING(토크 OFF)  r 복구  q 종료
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sopo.client import SopoClient
from sopo.keys import KeyReader
from sopo.sources import JogSource


def main() -> None:
    c = SopoClient()
    s = c.state(2.0)
    if not s:
        print("no state from sopod - start it first: sopod --config configs/arm.yaml"); return
    names = list(s["joints"].keys())
    reader = KeyReader()
    pending: list[str] = []
    src = JogSource(names, lambda: pending, step=40)
    print(f"mode {s['mode']} - press m to enable torque (MOVE), then jog. q quits (arm holds; torque stays as is)")
    with reader:
        try:
            while True:
                s = c.state(0.5)
                if not s:
                    print("\rlost sopod state", end="", flush=True); continue
                present = {n: v["pos"] for n, v in s["joints"].items() if v["pos"] is not None}
                keys = reader.poll()
                extra = [k for k in keys if k in ("m", "g", "r")]
                pending[:] = [k for k in keys if k not in ("m", "g", "r")]
                for k in extra:
                    reader.restore()
                    print("\n", c.command({"m": "move", "g": "guiding", "r": "recover"}[k]))
                    reader.reenter()
                    src.targets = None  # 모드 전환 후 목표를 현재로 리셋
                action = src.get_action(present, time.monotonic())
                if s["mode"] == "move":
                    c.send_action(action)
                print(f"\r[{s['mode']:>7}/{s['reflex']:<8}] {src.last_status}  {s['volt'][0]:.1f}V p99 {s['jitter_p99_ms']}ms".ljust(150), end="", flush=True)
                if src.is_done():
                    break
                time.sleep(0.03)
        except KeyboardInterrupt:
            pass
    print("\njog client exit (daemon keeps its mode; use `python -m sopo.cli idle` to drop torque)")


if __name__ == "__main__":
    main()

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

from sopo.runtime.client import SopoClient
from sopo.keys import KeyReader
from sopo.runtime.sources import JogSource


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
                    r = c.command({"m": "move", "g": "guiding", "r": "recover"}[k])
                    print("\n", r)
                    if k in ("m", "r") and r.get("ok"):
                        print(" ", c.acquire("jog"))   # Franka control(): 리스는 reflex 때 회수되므로 복구 후 다시 받는다
                    reader.reenter()
                    src.targets = None  # 모드 전환 후 목표를 현재로 리셋
                action = src.get_action(present, time.monotonic())
                if s["mode"] == "move":
                    c.send_action(action)
                print(f"\r[{s['mode']:>7}/{s['reflex']:<8} lease={s.get('lease') or '-'}] {src.last_status}  {s['volt'][0]:.1f}V p99 {s['jitter_p99_ms']}ms".ljust(150), end="", flush=True)
                if src.is_done():
                    break
                time.sleep(0.03)
        except KeyboardInterrupt:
            pass
    if c.lease:
        c.release()
    print("\njog client exit (lease released; daemon keeps its mode - `python -m sopo.runtime.cli idle` drops torque)")


if __name__ == "__main__":
    main()

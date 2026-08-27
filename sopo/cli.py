"""sopo CLI: python -m sopo.cli <status|watch|move|idle|guiding|recover|init|goto|shutdown>"""

from __future__ import annotations

import argparse
import sys
import time

from .client import SopoClient


def fmt_state(s: dict) -> str:
    js = "  ".join(f"{n}:{v['pos']}->{v['goal']}({v['load']})" for n, v in s["joints"].items())
    return (f"{s['mode']:>7} reflex={s['reflex']:<8} {'STALE ' if s['stale'] else ''}{js}  | {s['volt'][0]:.1f}-{s['volt'][1]:.1f}V "
            f"cycle {s['cycle_ms']}ms p99 {s['jitter_p99_ms']}ms")


def main() -> None:
    parser = argparse.ArgumentParser(description="sopo daemon client")
    parser.add_argument("--host", default="127.0.0.1")
    sub = parser.add_subparsers(dest="cmd", required=True)
    for c in ("status", "watch", "move", "idle", "guiding", "recover", "init", "shutdown"):
        sub.add_parser(c)
    g = sub.add_parser("goto"); g.add_argument("targets", nargs="+", help="J2=1200 J3=2500 ...")
    args = parser.parse_args()
    c = SopoClient(args.host)
    if args.cmd == "status":
        s = c.state(1.0)
        print(fmt_state(s) if s else "no state (is sopod running?)")
    elif args.cmd == "watch":
        try:
            while True:
                s = c.state(1.0)
                print(fmt_state(s) if s else "no state", flush=True)
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
    elif args.cmd == "goto":
        action = {k: int(v) for k, v in (t.split("=") for t in args.targets)}
        print(c.command("goto", action=action))
    else:
        print(c.command(args.cmd))


if __name__ == "__main__":
    main()

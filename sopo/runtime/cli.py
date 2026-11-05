"""sopo CLI: python -m sopo.runtime.cli <status|watch|move|idle|guiding|recover|init|goto|stop|reset|shutdown>"""

from __future__ import annotations

import argparse
import sys
import time

from .client import SopoClient


def fmt_state(s: dict) -> str:
    joints = []
    for name, state in s["joints"].items():
        command = state["goal"]
        goal = state.get("nominal_goal", command)
        summary = f"{name}:{state['pos']}->{goal}({state['load']})"
        if command != goal:
            summary += f"[cmd={command}]"
        joints.append(summary)
    js = "  ".join(joints)
    lease = f"lease={s.get('lease') or '-':<8} "
    phase = "ENTERING_RANGE " if s.get("entering_range") else "STALE " if s["stale"] else ""
    line = (f"{s['mode']:>7} reflex={s['reflex']:<8} {lease}{phase}{js}  | {s['volt'][0]:.1f}-{s['volt'][1]:.1f}V "
            f"cycle {s['cycle_ms']}ms p99 {s['jitter_p99_ms']}ms")
    if s.get("trips") and s["mode"] == "reflex":
        line += "\n         last: " + s["trips"][-1]
    return line


def main() -> None:
    parser = argparse.ArgumentParser(description="sopo daemon client")
    parser.add_argument("--host", default="127.0.0.1")
    sub = parser.add_subparsers(dest="cmd", required=True)
    for c in ("status", "watch", "move", "idle", "guiding", "recover", "init", "shutdown", "tare_ext", "stop", "damped", "reset"):
        sub.add_parser(c)
    g = sub.add_parser("goto"); g.add_argument("targets", nargs="+", help="J2=45 J3=-30 ... (degree, REP-103)")
    g.add_argument("--ticks", action="store_true", help="degree 대신 raw 모터 틱으로 해석 (J2=2300 ...)")
    g.add_argument("--no-wait", action="store_true", help="return right after the command is accepted")
    g.add_argument("--timeout", type=float, default=20.0)
    hotfix = sub.add_parser("hotfix", help="adjust or clear a joint's hold offset")
    hotfix.add_argument("joint")
    hotfix.add_argument("adjustment", metavar="TICKS|clear")
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
    elif args.cmd == "hotfix":
        if args.adjustment == "clear":
            value = {"clear": True}
        else:
            try:
                value = {"delta_ticks": int(args.adjustment)}
            except ValueError:
                hotfix.error("expected signed integer ticks or 'clear'")
        response = c.command("hotfix", joint=args.joint, **value)
        print(response)
        if not response.get("ok"):
            sys.exit(1)
    elif args.cmd == "goto":
        action = {k: float(v) if not args.ticks else int(v)
                  for k, v in (t.split("=") for t in args.targets)}
        r = c.command("goto", action=action, unit="ticks" if args.ticks else "deg")
        if not r.get("ok"):
            print("refused:", r.get("error")); sys.exit(1)
        for n, msg in r.get("clamped", {}).items():
            print(f"warn: {n} {msg}")
        target = r["accepted"]
        if args.no_wait:
            print("accepted", target); return
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.timeout:
            s = c.state(1.0)
            if not s:
                continue
            if s["mode"] == "reflex":
                print(f"\nREFLEX after {time.monotonic() - t0:.1f}s: {s['trips'][-1] if s['trips'] else ''}\n-> `python -m sopo.runtime.cli recover`"); sys.exit(2)
            if s["mode"] != "move":
                print(f"\nmode changed to {s['mode']}"); sys.exit(2)
            err = {n: s["joints"][n]["pos"] - v for n, v in target.items() if s["joints"][n]["pos"] is not None}
            print("\r" + "  ".join(f"{n}:{s['joints'][n]['pos']}->{v} ({e:+d})" for (n, v), e in zip(target.items(), err.values())).ljust(100), end="", flush=True)
            if all(abs(e) <= 30 for e in err.values()):
                print(f"\nreached in {time.monotonic() - t0:.1f}s"); return
            time.sleep(0.2)
        print(f"\ntimeout after {args.timeout}s (still moving or blocked)"); sys.exit(3)
    else:
        print(c.command(args.cmd))


if __name__ == "__main__":
    main()

"""sopod: the low-level controller daemon - sole owner of the servo bus (docs/architecture.md).

Fixed-rate loop: read -> action (stream, watchdog) -> clamp -> command -> reflex -> hold/stop -> publish.
ZMQ: state PUB (latest only), command REQ/REP, action SUB. Everything else (jog, policies, GUI) is a client.
Starts in IDLE with torque off; torque is enabled only by an explicit command.
"""

from __future__ import annotations

import argparse
import json
import queue
import secrets
import statistics
import sys
import threading
import time
from collections import deque
from enum import Enum

import zmq

from .bus import FeetechBus
from .config import all_motor_ids, load_arm_config, make_joint_limits, make_joints, make_limits, make_pairs
from .control import (ARRIVAL_TICKS, SOFT_START_STEP, Blackbox, clamp_joint_goals, command_joints, describe_trip,
                      hold_joint_goals, motor_goals, read_joints, reflex_present_view)
from .joints import ContinuousJoint, DualMotorJoint
from .reflex import Mode as ReflexMode, Reflex, ReflexConfig
from .safety import apply_safety, verify_eprom
from .sources import StreamSource
from .startup import self_test

DEFAULT_PORTS = {"state": 5555, "cmd": 5556, "action": 5557}


class ArmMode(str, Enum):
    IDLE = "idle"          # torque off
    GUIDING = "guiding"    # torque off, state published (teach by hand)
    MOVE = "move"          # torque on, following the action stream
    REFLEX = "reflex"      # latched by reflex; 'recover' to resume
    STOPPED = "stopped"    # comm loss / overtemp; torque off; restart the daemon


class Daemon:
    def __init__(self, cfg: dict, ports: dict | None = None, watchdog_s: float = 0.5):
        self.cfg = cfg
        self.ports = {**DEFAULT_PORTS, **(ports or {})}
        self.joints = make_joints(cfg)
        self.limits = make_limits(cfg)
        self.joint_limits = make_joint_limits(cfg, self.joints)
        self.ids = all_motor_ids(self.joints)
        self.rate = float(cfg.get("rate_hz", 50))
        self.bus = FeetechBus(cfg["arm"]["port"], cfg["arm"].get("baudrate", 1_000_000))
        self.stream = StreamSource(watchdog_s)
        self.reflex = self._new_reflex()
        self.mode = ArmMode.IDLE
        self.soft = True
        self.goal: dict[str, int] = {}
        self.running = False
        self.cmd_q: queue.Queue = queue.Queue()
        self.cycle_times: deque = deque(maxlen=200)
        self.last_trips: list[str] = []
        self.blackbox = Blackbox(self.ids, rate_hz=self.rate)
        self.volt = (0.0, 0.0)
        self.temps: dict[int, int] = {}
        self.present: dict[str, int] = {}
        self.load: dict[int, int] = {}
        self.rp: dict[int, int] = {}
        # Control lease: exclusive right to stream actions (Franka's control() session). Revoked on
        # REFLEX/STOPPED/idle so a client cannot keep driving without acknowledging the event.
        self.lease: dict | None = None
        self._lease_warned = 0.0

    # ------------------------------------------------------------------ setup
    def _new_reflex(self) -> Reflex:
        r = Reflex(self.limits, make_pairs(self.cfg), ReflexConfig())
        for j in self.joints:
            if isinstance(j, ContinuousJoint) and j.range_bounds():
                lo, hi = j.range_bounds()
                r.set_limit(j.motor_id, lo, hi)
        return r

    def start(self) -> None:
        self.bus.connect()
        for w in verify_eprom(self.bus, self.limits, self.ids):
            self._log(f"warn (EPROM drift): {w}")
        self.bus.torque_off_verified(self.ids)
        read_joints(self.bus, self.joints)  # establishes home for continuous joints
        self.reflex = self._new_reflex()
        for j in self.joints:
            if isinstance(j, ContinuousJoint):
                self._log(f"{j.name}: home {j.home} (abs {j.home_abs}), range {j.range_bounds()}")

        ctx = zmq.Context.instance()
        self.pub = ctx.socket(zmq.PUB); self.pub.setsockopt(zmq.SNDHWM, 2); self.pub.bind(f"tcp://*:{self.ports['state']}")
        self.sub = ctx.socket(zmq.SUB); self.sub.setsockopt(zmq.SUBSCRIBE, b""); self.sub.setsockopt(zmq.RCVHWM, 10)
        self.sub.bind(f"tcp://*:{self.ports['action']}")
        self.rep = ctx.socket(zmq.REP); self.rep.bind(f"tcp://*:{self.ports['cmd']}")
        self.running = True
        threading.Thread(target=self._rep_thread, daemon=True).start()
        self._log(f"sopod up: state :{self.ports['state']} cmd :{self.ports['cmd']} action :{self.ports['action']} | mode {self.mode.value}, torque off")
        try:
            self._loop()
        finally:
            self.running = False
            self._log("shutting down: torque off")
            self.bus.disconnect(disable_torque_ids=self.ids)

    # ---------------------------------------------------------------- threads
    def _rep_thread(self) -> None:
        while self.running:
            try:
                if not self.rep.poll(200):
                    continue
                msg = self.rep.recv_json()
            except Exception:
                continue
            reply_q: queue.Queue = queue.Queue(1)
            self.cmd_q.put((msg, reply_q))
            try:
                reply = reply_q.get(timeout=60.0)
            except queue.Empty:
                reply = {"ok": False, "error": "daemon busy"}
            try:
                self.rep.send_json(reply)
            except Exception:
                pass

    # ------------------------------------------------------------------- loop
    def _loop(self) -> None:
        period = 1.0 / self.rate
        last_temp = last_volt = 0.0
        flagged: dict[tuple[int, str], float] = {}
        while self.running:
            t0 = time.monotonic()
            self._drain_commands()
            self._drain_actions(t0)
            comm_ok = True
            try:
                self.present = read_joints(self.bus, self.joints)
                self.load = self.bus.sync_read("Present_Load", self.ids)
                self.rp = reflex_present_view(self.bus, self.joints, self.present)
                if self.mode is ArmMode.MOVE:
                    action = self.stream.get_action(self.present, t0)
                    step = SOFT_START_STEP if self.soft else self.limits.max_relative_target
                    self.goal = clamp_joint_goals(action, self.present, self.joint_limits, step, self.joints,
                                                  self.limits.brake_zone_ticks, self.limits.brake_min_step)
                    command_joints(self.bus, self.joints, self.goal)
                    if self.soft and all(abs(action[n] - self.present[n]) < ARRIVAL_TICKS for n in action):
                        self.soft = False
                elif self.mode is not ArmMode.REFLEX:
                    self.goal = dict(self.present)
            except KeyboardInterrupt:
                self.bus.port.clearPort()
                raise
            except Exception as e:
                comm_ok = False
                self._log(f"comm error: {e}")

            for mid, text in self.bus.pop_motor_errors().items():
                if t0 - flagged.get((mid, text), -9) > 5.0:
                    flagged[(mid, text)] = t0
                    self._log(f"motor {mid} status flag: {text}")
            temps = None
            if comm_ok and t0 - last_temp > 2.0:
                last_temp = t0
                try:
                    temps = self.bus.sync_read("Present_Temperature", self.ids); self.temps = temps
                except Exception:
                    pass
            if comm_ok and t0 - last_volt > 0.2:
                last_volt = t0
                try:
                    v = self.bus.sync_read("Present_Voltage", self.ids)
                    self.volt = (min(v.values()) / 10, max(v.values()) / 10)
                except Exception:
                    pass

            if self.mode in (ArmMode.MOVE, ArmMode.REFLEX) and self.goal:
                mg = motor_goals(self.joints, self.goal)
                trips = self.reflex.update(t0, self.rp, mg, self.load, temps=temps, comm_ok=comm_ok)
                for trip in trips:
                    line = describe_trip(trip, self.joints); self._log(line); self.last_trips = (self.last_trips + [line])[-5:]
                for w in self.reflex.warnings():
                    self._log(f"warn: {w}")
                self.blackbox.record(t0, self.reflex.mode.value, self.rp, mg, self.load, ";".join(t.event.value for t in trips))
                if self.reflex.mode is ReflexMode.STOPPED:
                    self._set_torque(False); self.mode = ArmMode.STOPPED; self._revoke("stopped")
                    self._log(f"STOPPED - torque off. blackbox: {self.blackbox.dump('stopped')}")
                elif self.reflex.mode is ReflexMode.REFLEX and self.mode is not ArmMode.REFLEX:
                    hold = self.reflex.hold_targets(self.rp)
                    try:
                        command_joints(self.bus, self.joints, hold_joint_goals(self.joints, hold))
                    except Exception as e:
                        self._log(f"hold send failed: {e}")
                    self.mode = ArmMode.REFLEX; self._revoke("reflex")
                    why = self.last_trips[-1].replace("[REFLEX] ", "") if self.last_trips else "?"
                    self._log(f"REFLEX latched ({why}) - hold sent. blackbox: {self.blackbox.dump('reflex')}. send 'recover' to resume")

            elapsed = time.monotonic() - t0
            self.cycle_times.append(elapsed)
            self._publish(t0, elapsed)
            if elapsed < period:
                time.sleep(period - elapsed)

    def _drain_actions(self, now: float) -> None:
        last = None
        while True:
            try:
                last = self.sub.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except Exception:
                break
        if last is not None and isinstance(last, dict):
            action = last.get("action")
            token = last.get("lease")
            if not isinstance(action, dict):
                return
            if self.lease is None or token != self.lease["token"]:
                if now - self._lease_warned > 5.0:
                    self._lease_warned = now
                    self._log("action dropped: no valid control lease (client must 'acquire' - after a reflex: 'recover' then 'acquire')")
                return
            self.stream.push(action, now)

    def _drain_commands(self) -> None:
        while True:
            try:
                msg, reply_q = self.cmd_q.get_nowait()
            except queue.Empty:
                return
            try:
                reply = self._handle(msg)
            except Exception as e:
                reply = {"ok": False, "error": str(e)}
            reply_q.put(reply)

    # ---------------------------------------------------------------- commands
    def _set_torque(self, on: bool) -> None:
        if on:
            apply_safety(self.bus, self.ids, self.limits)   # always before enabling
            self.bus.enable_torque(self.ids)
        else:
            still = self.bus.torque_off_verified(self.ids)
            if still:
                self._log(f"!!! torque-off not verified for {still}")

    def _enter_move(self) -> None:
        if self.reflex.mode in (ReflexMode.REFLEX, ReflexMode.STOPPED):
            self.reflex = self._new_reflex()
        self.stream.clear()
        self._set_torque(True)
        self.soft = True
        self.mode = ArmMode.MOVE

    def _revoke(self, reason: str) -> None:
        if self.lease:
            self._log(f"control lease of '{self.lease['name']}' revoked ({reason})")
            self.lease = None
            self.stream.clear()

    def _handle(self, msg: dict) -> dict:
        cmd = msg.get("cmd")
        if cmd == "status":
            return {"ok": True, "state": self._state(0.0, 0.0)}
        if cmd == "acquire":
            name = str(msg.get("name") or "client")
            if self.mode in (ArmMode.REFLEX, ArmMode.STOPPED):
                return {"ok": False, "error": f"cannot acquire in {self.mode.value}: recover first"}
            if self.lease and self.lease["name"] != name:
                return {"ok": False, "error": f"lease held by '{self.lease['name']}'"}
            self.lease = {"name": name, "token": secrets.token_hex(8), "since": time.time()}
            self.stream.clear()
            self._log(f"control lease -> '{name}'")
            return {"ok": True, "lease": self.lease["token"], "mode": self.mode.value}
        if cmd == "release":
            if self.lease and msg.get("lease") == self.lease["token"]:
                self._revoke("released"); return {"ok": True}
            return {"ok": False, "error": "not the lease holder"}
        if cmd in ("goto", "init") and self.lease is not None:
            return {"ok": False, "error": f"control lease held by '{self.lease['name']}' - release it first"}
        # Latched errors must be acknowledged: only 'recover' leads back to MOVE (Franka: automaticErrorRecovery).
        # Dropping torque (idle/guiding) is always allowed.
        if self.mode is ArmMode.REFLEX and cmd in ("move", "torque_on", "init", "goto"):
            why = self.last_trips[-1] if self.last_trips else "reflex"
            return {"ok": False, "error": f"REFLEX latched ({why}) - run 'recover' first, or 'idle' to drop torque"}
        if cmd in ("move", "torque_on"):
            if self.mode is ArmMode.STOPPED:
                return {"ok": False, "error": "STOPPED: restart the daemon"}
            self._enter_move(); return {"ok": True, "mode": self.mode.value}
        if cmd in ("idle", "torque_off", "guiding"):
            self._set_torque(False)
            self.mode = ArmMode.GUIDING if cmd == "guiding" else ArmMode.IDLE
            self._revoke(cmd); self.stream.clear()
            return {"ok": True, "mode": self.mode.value}
        if cmd == "recover":
            if self.mode is not ArmMode.REFLEX:
                return {"ok": False, "error": f"not in REFLEX (mode {self.mode.value})"}
            ok, reason = self.reflex.recover(self.rp, self.load)
            if not ok:
                return {"ok": False, "error": reason}
            self.stream.clear(); self.soft = True; self.mode = ArmMode.MOVE
            return {"ok": True, "mode": "move"}
        if cmd == "goto":
            target = msg.get("action") or {}
            unknown = set(target) - {j.name for j in self.joints}
            if unknown:
                return {"ok": False, "error": f"unknown joints {sorted(unknown)}"}
            if self.mode is not ArmMode.MOVE:
                return {"ok": False, "error": f"goto needs MOVE mode (now {self.mode.value}); send 'move' first"}
            by_name = {j.name: j for j in self.joints}
            clamped, notes = {}, {}
            for name, v in target.items():
                j = by_name[name]
                if isinstance(j, ContinuousJoint):
                    cv = j.clamp(int(v))
                else:
                    lo, hi = self.joint_limits.get(name, (0, 4095))
                    cv = max(lo, min(hi, int(v)))
                clamped[name] = cv
                if cv != int(v):
                    notes[name] = f"{v} -> {cv} (soft limit)"
            self.stream.push(clamped, time.monotonic(), sticky=True)
            return {"ok": True, "accepted": clamped, "clamped": notes, "note": "accepted, not yet reached - watch state"}
        if cmd == "init":
            if self.mode is ArmMode.STOPPED:
                return {"ok": False, "error": "STOPPED"}
            self._enter_move()
            try:
                report = self_test(self.bus, self.joints, self.limits, self.joint_limits)
            except RuntimeError as e:
                self._set_torque(False); self.mode = ArmMode.IDLE
                return {"ok": False, "error": f"self-test failed: {e}"}
            standby = {str(k): int(v) for k, v in (self.cfg.get("standby_pose") or {}).items()}
            for j in self.joints:
                if isinstance(j, ContinuousJoint) and j.home_abs is not None and j.name in standby:
                    standby[j.name] += j.home - j.home_abs
            if standby:
                self.stream.push(standby, time.monotonic(), sticky=True)
            self.soft = True
            return {"ok": True, "self_test": {k: v["max_load"] for k, v in report.items()}, "standby": standby}
        if cmd == "shutdown":
            self.running = False
            return {"ok": True}
        return {"ok": False, "error": f"unknown command {cmd!r}"}

    # ------------------------------------------------------------------ state
    def _state(self, t0: float, elapsed: float) -> dict:
        ct = list(self.cycle_times)
        p99 = statistics.quantiles(ct, n=100)[-1] * 1e3 if len(ct) >= 20 else 0.0
        joints = {}
        for j in self.joints:
            ref = j.reference_id if isinstance(j, DualMotorJoint) else j.motor_ids[0]
            joints[j.name] = {"pos": self.present.get(j.name), "goal": self.goal.get(j.name), "load": self.load.get(ref)}
        return {
            "t": time.time(), "mode": self.mode.value, "reflex": self.reflex.mode.value, "stale": self.stream.stale,
            "lease": self.lease["name"] if self.lease else None,
            "joints": joints, "motors": {str(i): {"pos": self.rp.get(i), "load": self.load.get(i), "temp": self.temps.get(i)} for i in self.ids},
            "volt": list(self.volt), "cycle_ms": round(elapsed * 1e3, 2), "jitter_p99_ms": round(p99, 2), "trips": self.last_trips,
        }

    def _publish(self, t0: float, elapsed: float) -> None:
        try:
            self.pub.send_json(self._state(t0, elapsed), flags=zmq.NOBLOCK)
        except Exception:
            pass

    def _log(self, text: str) -> None:
        print(f"[sopod {time.strftime('%H:%M:%S')}] {text}", file=sys.stderr, flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description="sopod - sopo arm controller daemon")
    parser.add_argument("--config", default="configs/arm.yaml")
    parser.add_argument("--state-port", type=int, default=DEFAULT_PORTS["state"])
    parser.add_argument("--cmd-port", type=int, default=DEFAULT_PORTS["cmd"])
    parser.add_argument("--action-port", type=int, default=DEFAULT_PORTS["action"])
    parser.add_argument("--watchdog", type=float, default=0.5, help="seconds without actions before holding")
    args = parser.parse_args()
    cfg = load_arm_config(args.config)
    d = Daemon(cfg, {"state": args.state_port, "cmd": args.cmd_port, "action": args.action_port}, args.watchdog)
    try:
        d.start()
    except KeyboardInterrupt:
        print("\n[sopod] interrupted", file=sys.stderr)


if __name__ == "__main__":
    main()

"""sopod: the low-level controller daemon - sole owner of the servo bus (docs/architecture.md).

Fixed-rate loop: read -> action (stream, watchdog) -> clamp -> command -> reflex -> hold/stop -> publish.
ZMQ: state PUB (latest only), command REQ/REP, action SUB. Everything else (jog, policies, GUI) is a client.
Starts in IDLE with torque off; torque is enabled only by an explicit command.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import queue
import secrets
import statistics
import sys
import threading
import time
from collections import deque
from enum import Enum
from pathlib import Path

import zmq

from ..hal.bus import FeetechBus
from ..config import all_motor_ids, load_arm_config, make_joint_limits, make_joints, make_limits, make_pairs
from ..motion.control import (ARRIVAL_TICKS, SOFT_START_STEP, Blackbox, clamp_joint_goals, command_joints, describe_trip,
                              hold_joint_goals, motor_goals, read_joints, reflex_present_view)
from ..motion.joints import ContinuousJoint, DualMotorJoint
from ..safety.reflex import Mode as ReflexMode, Reflex, ReflexConfig
from ..safety.limits import SafetyLimits, apply_safety, freeze, verify_eprom
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
    def __init__(self, cfg: dict, ports: dict | None = None, watchdog_s: float = 0.5, release_on_exit: bool = False,
                 config_path: str | None = None):
        self.cfg = cfg
        self.release_on_exit = release_on_exit
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
        self.raw_pos: dict[int, int] = {}   # 물리 raw 틱 (외력 관측기용 — 연속 관절 논리각이 아님)
        self.ext: dict[str, float] = {}     # 관절별 외력 토크 [N·m] (estimator 없으면 빈 dict)
        self._ref_ids = {j.name: (j.reference_id if isinstance(j, DualMotorJoint) else j.motor_ids[0])
                         for j in self.joints}
        # degree 변환(zero/dir)은 pinocchio 없이도 가능하므로 estimator와 분리해 항상 로드
        self.cal = None
        if config_path:
            try:
                from ..config import load_gravity_cal
                self.cal = load_gravity_cal(Path(config_path).parent / "calibration.yaml")
            except Exception as e:
                self._log(f"gravity calibration not loaded (degree goto disabled): {e}")
        self.estimator = self._new_estimator(config_path)
        # 페이로드 안전망 (docs/payload-safety-requirements.md): arm.yaml payload.max_kg +
        # calibration.yaml estimation{floor,k} + 추정기가 갖춰지면 자세 의존 동적 임계를 reflex에 준다.
        self.payload_max_kg = float((cfg.get("payload") or {}).get("max_kg") or 0.0)
        self.power_cap = int(self.cfg.get("safety", {}).get("power_torque_limit", 600))
        self.floor: dict[str, float] = {}
        self.k_est: float | None = None
        if config_path:
            try:
                from ..config import load_estimation_limits
                self.floor, self.k_est = load_estimation_limits(Path(config_path).parent / "calibration.yaml")
            except Exception as e:
                self._log(f"estimation limits not loaded (동적 임계 비활성): {e}")
        self._tare_done = False   # FR-7 인터록: 이 세션에서 tare 1회 이상 완료해야 전력 모드
        self.settled = False      # 최근 사이클의 정착 상태 (FR-8/FR-9가 참조)
        # Control lease: exclusive right to stream actions (Franka's control() session). Revoked on
        # REFLEX/STOPPED/idle so a client cannot keep driving without acknowledging the event.
        self.lease: dict | None = None
        self._lease_warned = 0.0

    # ------------------------------------------------------------------ setup
    def _new_reflex(self) -> Reflex:
        # 외력 임계값은 arm.yaml safety 섹션에서 덮어쓸 수 있다 (ext_warn_nm/ext_trip_nm/t_ext)
        safety = self.cfg.get("safety", {})
        rc = ReflexConfig(
            ext_warn_nm=float(safety.get("ext_warn_nm", ReflexConfig.ext_warn_nm)),
            ext_trip_nm=float(safety.get("ext_trip_nm", ReflexConfig.ext_trip_nm)),
            t_ext=float(safety.get("t_ext", ReflexConfig.t_ext)),
        )
        r = Reflex(self.limits, make_pairs(self.cfg), rc)
        for j in self.joints:
            if isinstance(j, ContinuousJoint) and j.range_bounds():
                lo, hi = j.range_bounds()
                r.set_limit(j.motor_id, lo, hi)
        return r

    def _new_estimator(self, config_path: str | None):
        """외력 관측기를 만든다. config_path가 없거나 모델/캘리브레이션을 못 읽으면 None (pin 미설치 등)."""
        if not config_path:
            return None
        try:
            from ..config import load_gravity_cal, make_gravity_model
            from ..model.estimation import ExternalTorqueEstimator

            # calibration 경로 규칙은 load_arm_config와 동일: arm.yaml 옆의 calibration.yaml
            cal = load_gravity_cal(Path(config_path).parent / "calibration.yaml")
            if not cal.zero_ticks:
                raise ValueError("gravity.zero_ticks 없음 — cookbook/4_torque_model/410_gravity_check.py --calibrate-vertical 먼저")
            est = ExternalTorqueEstimator(make_gravity_model(config_path), cal, ref_ids=self._ref_ids)
            self._log(f"external torque estimator on (calibration: {Path(config_path).parent / 'calibration.yaml'})")
            return est
        except Exception as e:
            self._log(f"external torque estimator disabled: {e}")
            return None

    @property
    def _payload_active(self) -> bool:
        """동적 임계(포락선) 활성 조건 (FR-2): 추정기 + floor/k 실측 + 페이로드 선언."""
        return (self.estimator is not None and self.payload_max_kg > 0.0
                and bool(self.floor) and self.k_est is not None)

    @property
    def power_ok(self) -> bool:
        """전력 모드 인터록 (FR-7): 추정기 + floor/k 실측 + 이 세션에서 tare 완료.
        안전망이 서 있을 때만 힘을 푼다. 상태 스트림에도 표시."""
        return self.estimator is not None and bool(self.floor) and self.k_est is not None and self._tare_done

    def _motion_limits(self) -> SafetyLimits:
        """토크 ON 때 적용할 캡: power_ok면 균일 전력 캡, 아니면 기존(순한) 설정.
        per-motor 캘리브레이션 오버라이드는 순한 모드 전용 — 전력 모드에서는 균일 캡."""
        if not self.power_ok:
            return self.limits
        cap = min(self.power_cap, self.limits.eprom_torque_ceiling)  # EPROM 상한을 넘기지 않는다
        return dataclasses.replace(self.limits, torque_limit=cap, torque_limits={})

    def start(self) -> None:
        self.bus.connect()
        for w in verify_eprom(self.bus, self.limits, self.ids):
            self._log(f"warn (EPROM drift): {w}")
        read_joints(self.bus, self.joints)  # establishes home for continuous joints
        self.reflex = self._new_reflex()
        # Never touch torque at start: a previous session may have left the arm holding (Cat 2, #10).
        try:
            on = [i for i, v in self.bus.sync_read("Torque_Enable", self.ids).items() if v]
        except Exception:
            on = []
        if on:
            self.mode = ArmMode.REFLEX
            self.last_trips = ["[REFLEX] INHERITED_HOLD: torque was on at daemon start"]
            self._log(f"torque already on for {on}: adopting as a held arm (REFLEX). 'recover' to move, 'idle' to release")
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
            if self.release_on_exit or self.mode in (ArmMode.IDLE, ArmMode.GUIDING):
                self._log("shutting down: torque off")
                self.bus.disconnect(disable_torque_ids=self.ids)
            else:
                self._freeze("daemon exit")
                self._log("shutting down: arm HOLDS (torque on). release: python cookbook/1_setup/150_torque_off.py")
                self.bus.disconnect()

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
        prev_settled = False
        flagged: dict[tuple[int, str], float] = {}
        while self.running:
            t0 = time.monotonic()
            self._drain_commands()
            self._drain_actions(t0)
            comm_ok = True
            try:
                self.present = read_joints(self.bus, self.joints)
                self.load = self.bus.sync_read("Present_Load", self.ids)
                self.raw_pos = self.bus.sync_read("Present_Position", self.ids)
                self.rp = reflex_present_view(self.bus, self.joints, self.present, self.raw_pos)
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

            if self.estimator is not None and comm_ok:
                try:
                    vel = self.bus.sync_read("Present_Velocity", self.ids)
                except Exception:
                    vel = None  # 읽기 실패 시 v=0 (정지 가정) — 위치 미분 폴백은 하지 않는다
                try:
                    self.ext = self.estimator.update(self.raw_pos, self.load, vel)
                except Exception as e:
                    self._log(f"ext estimator error: {e}")

            # 외력 판정은 정착(목표 도달) 상태에서만: 이동 중에는 서보 PID 과도응답과
            # 미모델 가속 항이 잔차를 오염시킨다. 상태 발행은 항상 한다.
            settled = (self.mode is ArmMode.MOVE and not self.soft and self.goal
                       and all(abs(self.goal[n] - self.present.get(n, 0)) < ARRIVAL_TICKS
                               for n in self.goal))
            self.settled = settled

            # FR-2: 페이로드 포락선 임계를 매 사이클 계산해 reflex에 넘긴다 (FK 비용 µs).
            # 비활성이면 None — reflex는 고정 임계로 폴백한다.
            ext_limit = None
            if self._payload_active and comm_ok and self.cal is not None:
                try:
                    q_now = {n: self.cal.q(n, self.raw_pos[r]) for n, r in self._ref_ids.items()
                             if r in self.raw_pos}
                    if q_now:
                        ext_limit = self.estimator.model.payload_envelope(q_now, self.payload_max_kg,
                                                                          self.k_est, self.floor)
                except Exception as e:
                    self._log(f"payload envelope error: {e}")

            # FR-8: 정착 진입 에지에서 자동 tare (히스테리시스 바이어스를 계속 따라간다)
            if settled and not prev_settled:
                self._auto_tare()
            prev_settled = settled

            if self.mode in (ArmMode.MOVE, ArmMode.REFLEX) and self.goal:
                mg = motor_goals(self.joints, self.goal)
                trips = self.reflex.update(t0, self.rp, mg, self.load, temps=temps, comm_ok=comm_ok,
                                           ext_torque=(self.ext or None) if settled else None,
                                           ext_joint_motor=self._ref_ids, ext_limit=ext_limit)
                for trip in trips:
                    line = describe_trip(trip, self.joints); self._log(line); self.last_trips = (self.last_trips + [line])[-5:]
                for w in self.reflex.warnings():
                    self._log(f"warn: {w}")
                ext_note = ";".join(f"{n}:{v:+.2f}" for n, v in self.ext.items())
                self.blackbox.record(t0, self.reflex.mode.value, self.rp, mg, self.load, ";".join(t.event.value for t in trips),
                                     ext=ext_note)
                if self.reflex.mode is ReflexMode.STOPPED:
                    self.mode = ArmMode.STOPPED; self._freeze("stopped")
                    self._log(f"STOPPED (comm loss / overtemp) - arm holds. blackbox: {self.blackbox.dump('stopped')}. 'idle' to release")
                elif self.reflex.mode is ReflexMode.REFLEX and self.mode is not ArmMode.REFLEX:
                    hold = self.reflex.hold_targets(self.rp)
                    try:
                        command_joints(self.bus, self.joints, hold_joint_goals(self.joints, hold))
                    except Exception as e:
                        self._log(f"hold send failed: {e}")
                    self.mode = ArmMode.REFLEX; self._stiffen(); self._revoke("reflex")
                    why = self.last_trips[-1].replace("[REFLEX] ", "") if self.last_trips else "?"
                    self._log(f"REFLEX latched ({why}) - holding stiff. blackbox: {self.blackbox.dump('reflex')}. 'recover' to resume")

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
            limits = self._motion_limits()              # power_ok면 균일 전력 캡 (FR-7)
            apply_safety(self.bus, self.ids, limits)   # always before enabling
            if limits is not self.limits:
                self._log(f"전력 모드: 균일 캡 {limits.torque_limit}‰ (payload safety net armed)")
            self.bus.enable_torque(self.ids)
        else:
            still = self.bus.torque_off_verified(self.ids)
            if still:
                self._log(f"!!! torque-off not verified for {still}")

    def _auto_tare(self) -> None:
        """정착 진입 시 잔차 0점 맞추기 (FR-8). 큰 외력이 걸려 있으면 건너뛴다 — 걸린 하중을 0점으로 삼는 사고 방지."""
        if self.estimator is None:
            return
        big = {n: round(v, 3) for n, v in self.ext.items() if abs(v) >= self.reflex._cfg.ext_warn_nm}
        if big:
            self._log(f"auto-tare skipped: 외력 감지됨 {big} (걸린 하중을 0점으로 삼지 않기 위해)")
            return
        self.estimator.tare()
        first = not self._tare_done
        self._tare_done = True
        suffix = " — payload safety net armed (다음 토크 ON부터 전력 캡)" if first and self.power_ok else ""
        self._log(f"auto-tare: biases { {n: round(b, 3) for n, b in self.estimator.biases.items()} }{suffix}")

    def _enter_move(self) -> None:
        if self.reflex.mode in (ReflexMode.REFLEX, ReflexMode.STOPPED):
            self.reflex = self._new_reflex()
        self.stream.clear()
        self._set_torque(True)
        self.soft = True
        self.mode = ArmMode.MOVE

    def _stiffen(self) -> None:
        """Raise Torque_Limit to the hold cap so the frozen arm stays rigid (Cat 2 stop)."""
        try:
            self.bus.sync_write("Torque_Limit", {i: self.limits.hold_torque_limit for i in self.ids})
        except Exception as e:
            self._log(f"stiffen failed: {e}")

    def _freeze(self, reason: str) -> None:
        """Category-2 stop: goal = present, hold cap, torque kept. Lease revoked."""
        try:
            freeze(self.bus, self.ids, self.limits)
        except Exception as e:
            self._log(f"freeze failed ({e}) - servos keep their last goal")
        self._revoke(reason)

    def _revoke(self, reason: str) -> None:
        if self.lease:
            self._log(f"control lease of '{self.lease['name']}' revoked ({reason})")
            self.lease = None
            self.stream.clear()

    def _handle(self, msg: dict) -> dict:
        cmd = msg.get("cmd")
        if cmd == "status":
            return {"ok": True, "state": self._state(0.0, 0.0)}
        if cmd == "tare_ext":
            # 외력 관측기 0점 맞추기. 토크 ON 정착(MOVE + 정지) 상태에서만 허용 (FR-9) —
            # 토크 OFF나 이동 중이면 잔차가 −G(q)나 과도응답으로 오염된다.
            # (cmd는 루프 스레드에서 드레인되므로 pinocchio 스레드 안전)
            if self.estimator is None:
                return {"ok": False, "error": "external torque estimator not available (config_path/pinocchio/calibration 확인)"}
            if not (self.mode is ArmMode.MOVE and self.settled):
                return {"ok": False, "error": f"tare_ext는 MOVE 정착 상태에서만 허용 (mode={self.mode.value}, settled={self.settled})"}
            self.estimator.tare()
            self._tare_done = True
            self._log(f"external torque tare: biases { {n: round(b, 3) for n, b in self.estimator.biases.items()} }")
            return {"ok": True, "biases": {n: round(b, 4) for n, b in self.estimator.biases.items()}}
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
        if self.mode in (ArmMode.REFLEX, ArmMode.STOPPED) and cmd in ("move", "torque_on", "init", "goto", "acquire"):
            why = self.last_trips[-1] if self.last_trips else "reflex"
            return {"ok": False, "error": f"REFLEX latched ({why}) - run 'recover' first, or 'idle' to drop torque"}
        if cmd in ("move", "torque_on"):
            if self.mode is ArmMode.STOPPED:
                return {"ok": False, "error": "STOPPED: run 'recover' (after the cause is cleared) or 'idle'"}
            self._enter_move(); return {"ok": True, "mode": self.mode.value}
        if cmd in ("idle", "torque_off", "guiding"):
            self._set_torque(False)
            self.mode = ArmMode.GUIDING if cmd == "guiding" else ArmMode.IDLE
            self._revoke(cmd); self.stream.clear()
            return {"ok": True, "mode": self.mode.value}
        if cmd == "recover":
            if self.mode not in (ArmMode.REFLEX, ArmMode.STOPPED):
                return {"ok": False, "error": f"not in REFLEX/STOPPED (mode {self.mode.value})"}
            hot = {i: t for i, t in self.temps.items() if t >= self.reflex._cfg.temp_warn}
            if hot:
                return {"ok": False, "error": f"still hot: {hot} (let it cool below {self.reflex._cfg.temp_warn}°C)"}
            if self.reflex.mode is ReflexMode.REFLEX:
                ok, reason = self.reflex.recover(self.rp, self.load)
                if not ok:
                    return {"ok": False, "error": reason}
            else:
                self.reflex = self._new_reflex()
            apply_safety(self.bus, self.ids, self._motion_limits())  # back to motion caps (power_ok면 전력 캡)
            self.stream.clear(); self.soft = True; self.mode = ArmMode.MOVE
            return {"ok": True, "mode": "move"}
        if cmd == "goto":
            target = msg.get("action") or {}
            unknown = set(target) - {j.name for j in self.joints}
            if unknown:
                return {"ok": False, "error": f"unknown joints {sorted(unknown)}"}
            if self.mode is not ArmMode.MOVE:
                return {"ok": False, "error": f"goto needs MOVE mode (now {self.mode.value}); send 'move' first"}
            if msg.get("unit") == "deg":
                # REP-103 규약의 각도[deg] → 관절 프레임 틱 (calibration zero_ticks/dir)
                if self.cal is None or not self.cal.zero_ticks:
                    return {"ok": False, "error": "degree goto needs calibration (gravity.zero_ticks) - run cookbook/2_pose_calibration/230_calibrate_zero.py first"}
                target = {n: self.cal.ticks(n, float(v)) for n, v in target.items()}
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
                self.last_trips = (self.last_trips + [f"[REFLEX] SELF_TEST: {e}"])[-5:]
                self.mode = ArmMode.REFLEX; self._freeze("self-test failed")
                self._log(f"self-test failed - arm holds stiff. 'recover' then retry, or 'idle' to release: {e}")
                return {"ok": False, "error": f"self-test failed: {e} (arm holds; 'recover' or 'idle')"}
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
            "ext_torque": {n: round(v, 3) for n, v in self.ext.items()},
            "power_ok": self.power_ok,
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
    parser.add_argument("--release-on-exit", action="store_true", help="drop torque when the daemon exits (default: hold, Cat 2)")
    args = parser.parse_args()
    cfg = load_arm_config(args.config)
    d = Daemon(cfg, {"state": args.state_port, "cmd": args.cmd_port, "action": args.action_port}, args.watchdog, args.release_on_exit,
               config_path=args.config)
    try:
        d.start()
    except KeyboardInterrupt:
        print("\n[sopod] interrupted", file=sys.stderr)


if __name__ == "__main__":
    main()

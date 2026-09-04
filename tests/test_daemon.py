"""sopod integration test without hardware: FakeBus + real ZMQ on alternate ports."""
import pathlib, sys, threading, time
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import pytest
import sopo.runtime.daemon as daemon_mod
from sopo.runtime.sources import StreamSource

CFG = {
    "arm": {"port": "/dev/fake"},
    "joints": [{"name": "J1", "type": "continuous", "motor_id": 1, "range_ticks": 2000, "firmware_multiturn": True, "home_abs": 2048},
               {"name": "J2", "type": "dual", "ids": [10, 11], "reference_id": 11, "K": 4005},
               {"name": "J4", "type": "single", "motor_id": 19}],
    "safety": {"torque_limit": 200, "position_limits": {19: [220, 3969]}},
    "standby_pose": {"J4": 2300},
    "rate_hz": 100,
}


class FakePort:
    def clearPort(self): pass
    def closePort(self): pass


class FakeBus:
    """Position servo model: motor moves 30 ticks/cycle toward its goal when torque is on."""
    def __init__(self, port, baudrate=1_000_000):
        self.port = FakePort()
        self.pos = {1: 2100, 10: 4005 - 1500, 11: 1500, 19: 2000}
        self.goal = dict(self.pos)
        self.torque = {i: 0 for i in self.pos}
        self.regs = {}
        self.motor_errors = {}
    def connect(self): pass
    def disconnect(self, disable_torque_ids=None):
        if disable_torque_ids: self.torque_off_verified(disable_torque_ids)
    def _step(self):
        for i in self.pos:
            if self.torque[i]:
                d = self.goal[i] - self.pos[i]
                self.pos[i] += max(-30, min(30, d))
    def read(self, reg, mid):
        if reg == "Present_Position": self._step(); return self.pos[mid]
        if reg == "Torque_Enable": return self.torque[mid]
        if reg == "Max_Torque_Limit": return 600
        if reg in ("Min_Position_Limit", "Max_Position_Limit"): return {"Min_Position_Limit": 220, "Max_Position_Limit": 3969}[reg]
        return self.regs.get((mid, reg), 0)
    def write(self, reg, mid, value):
        if reg == "Goal_Position": self.goal[mid] = value
        elif reg == "Torque_Enable": self.torque[mid] = value
        else: self.regs[(mid, reg)] = value
    def sync_read(self, reg, ids):
        if reg == "Present_Position": self._step(); return {i: self.pos[i] for i in ids}
        if reg == "Present_Load": return {i: (20 if self.torque[i] else 0) for i in ids}
        if reg == "Present_Temperature": return {i: 35 for i in ids}
        if reg == "Present_Voltage": return {i: 122 for i in ids}
        if reg == "Torque_Enable": return {i: self.torque[i] for i in ids}
        return {i: 0 for i in ids}
    def sync_write(self, reg, values):
        for i, v in values.items(): self.write(reg, i, v)
    def enable_torque(self, ids):
        for i in ids: self.torque[i] = 1
    def torque_off_verified(self, ids, attempts=3):
        for i in ids: self.torque[i] = 0
        return []
    def pop_motor_errors(self): return {}
    from contextlib import contextmanager
    @contextmanager
    def eprom_unlocked(self, mid): yield


def test_stream_source_watchdog_and_sticky():
    s = StreamSource(watchdog_s=0.5)
    present = {"J4": 2000}
    assert s.get_action(present, 0.0) == present and s.stale
    s.push({"J4": 2300}, 1.0)
    assert s.get_action(present, 1.2) == {"J4": 2300} and not s.stale
    assert s.get_action(present, 1.8) == present and s.stale          # watchdog expired -> hold
    s.push({"J4": 2300}, 2.0, sticky=True)
    assert s.get_action(present, 9.0) == {"J4": 2300} and not s.stale  # sticky survives


@pytest.fixture
def running_daemon(monkeypatch):
    monkeypatch.setattr(daemon_mod, "FeetechBus", FakeBus)
    ports = {"state": 6555, "cmd": 6556, "action": 6557}
    d = daemon_mod.Daemon(CFG, ports, watchdog_s=0.3)
    t = threading.Thread(target=d.start, daemon=True)
    t.start()
    time.sleep(0.5)
    from sopo.runtime.client import SopoClient
    c = SopoClient("127.0.0.1", ports)
    yield d, c
    try:
        c.command("shutdown")
    except Exception:
        pass
    t.join(timeout=3)


def test_daemon_modes_actions_and_watchdog(running_daemon):
    d, c = running_daemon
    s = c.state(1.0)
    assert s and s["mode"] == "idle" and d.bus.torque[19] == 0
    # goto refused in IDLE; move enables torque (apply_safety first)
    assert c.command("goto", action={"J4": 2300})["ok"] is False
    assert c.command("move")["ok"] and d.mode.value == "move" and d.bus.torque[19] == 1
    assert d.bus.regs[(19, "Torque_Limit")] == 200 and d.bus.regs[(19, "Return_Delay_Time")] == 0
    # actions without a lease are dropped
    for _ in range(15):
        c.send_action({"J4": 5000}); time.sleep(0.02)
    assert abs(c.state(1.0)["joints"]["J4"]["pos"] - 2000) < 40
    assert c.acquire("test")["ok"] and c.state(1.0)["lease"] == "test"
    # action stream drives the joint; clamp to soft limits
    for _ in range(40):
        c.send_action({"J4": 5000}); time.sleep(0.02)
    s = c.state(1.0)
    assert s["joints"]["J4"]["goal"] <= 3969 and s["joints"]["J4"]["pos"] > 2000
    # watchdog: stop sending -> stale, goal == present (hold)
    time.sleep(0.6)
    s = c.state(1.0)
    assert s["stale"] and abs(s["joints"]["J4"]["goal"] - s["joints"]["J4"]["pos"]) <= 30
    # dual joint mirror commanded as K - goal
    assert abs(d.bus.goal[10] + d.bus.goal[11] - 4005) <= 1
    # goto is refused while someone holds the lease; after release it works (sticky)
    assert c.command("goto", action={"J4": 2500})["ok"] is False
    assert c.release()["ok"]
    assert c.command("goto", action={"J4": 2500})["ok"]
    time.sleep(0.8)
    assert abs(c.state(1.0)["joints"]["J4"]["pos"] - 2500) <= 30
    # idle drops torque
    assert c.command("idle")["ok"] and d.bus.torque[19] == 0 and d.mode.value == "idle"


def test_daemon_init_runs_self_test_then_standby(running_daemon):
    d, c = running_daemon
    r = c.command("init")
    assert r["ok"], r
    assert set(r["self_test"]) == {"J1", "J2", "J4"} and r["standby"] == {"J4": 2300}
    time.sleep(1.0)
    s = c.state(1.0)
    assert s["mode"] == "move" and abs(s["joints"]["J4"]["pos"] - 2300) <= 30


def test_reflex_latch_requires_explicit_recover(running_daemon):
    d, c = running_daemon
    assert c.command("move")["ok"]
    d.mode = daemon_mod.ArmMode.REFLEX          # simulate a latched trip
    d.last_trips = ["[REFLEX] COLLISION J4 (ID 19): test"]
    for cmd in ("move", "init"):
        r = c.command(cmd)
        assert r["ok"] is False and "recover" in r["error"]
    assert c.command("goto", action={"J4": 2300})["ok"] is False
    assert c.command("idle")["ok"] and d.mode.value == "idle"   # dropping torque always allowed


def test_lease_is_exclusive_and_revoked_on_reflex(running_daemon):
    d, c = running_daemon
    from sopo.runtime.client import SopoClient
    c2 = SopoClient("127.0.0.1", {"state": 6555, "cmd": 6556, "action": 6557})
    assert c.command("move")["ok"] and c.acquire("policy")["ok"]
    assert c2.acquire("jog")["ok"] is False                       # exclusive
    d.mode = daemon_mod.ArmMode.REFLEX; d._revoke("reflex")        # simulate a latched trip
    time.sleep(0.3)
    assert c.state(1.0)["lease"] is None
    assert c.acquire("policy")["ok"] is False                     # must recover first
    assert c.command("idle")["ok"]


def test_self_test_failure_freezes_instead_of_dropping_torque(running_daemon, monkeypatch):
    d, c = running_daemon
    def boom(*a, **k): raise RuntimeError("J2 did not reach")
    monkeypatch.setattr(daemon_mod, "self_test", boom)
    r = c.command("init")
    assert r["ok"] is False and "holds" in r["error"]
    assert d.mode.value == "reflex" and d.bus.torque[19] == 1                  # torque kept (Cat 2)
    assert d.bus.regs[(19, "Torque_Limit")] == 600                             # hold cap
    assert c.state(1.0)["lease"] is None
    assert c.command("move")["ok"] is False                                    # must recover
    assert c.command("recover")["ok"] and d.mode.value == "move"
    assert d.bus.regs[(19, "Torque_Limit")] == 200                             # motion cap restored


def test_daemon_exit_holds_torque(monkeypatch):
    monkeypatch.setattr(daemon_mod, "FeetechBus", FakeBus)
    ports = {"state": 6565, "cmd": 6566, "action": 6567}
    d = daemon_mod.Daemon(CFG, ports)
    t = threading.Thread(target=d.start, daemon=True); t.start(); time.sleep(0.5)
    from sopo.runtime.client import SopoClient
    c = SopoClient("127.0.0.1", ports)
    assert c.command("move")["ok"]
    c.command("shutdown"); t.join(timeout=3)
    assert d.bus.torque[19] == 1 and d.bus.regs[(19, "Torque_Limit")] == 600  # holds, stiff


def test_daemon_adopts_held_arm_at_start(monkeypatch):
    class HeldBus(FakeBus):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self.torque = {i: 1 for i in self.torque}   # previous session left the arm holding
    monkeypatch.setattr(daemon_mod, "FeetechBus", HeldBus)
    ports = {"state": 6575, "cmd": 6576, "action": 6577}
    d = daemon_mod.Daemon(CFG, ports)
    t = threading.Thread(target=d.start, daemon=True); t.start(); time.sleep(0.5)
    from sopo.runtime.client import SopoClient
    c = SopoClient("127.0.0.1", ports)
    s = c.state(1.0)
    assert s["mode"] == "reflex" and d.bus.torque[19] == 1        # not dropped
    assert c.command("move")["ok"] is False
    assert c.command("recover")["ok"] and d.mode.value == "move"
    c.command("shutdown"); t.join(timeout=3)


def test_ext_torque_in_state_and_tare(monkeypatch):
    """config_path를 주면 외력 관측기가 붙는다: state에 ext_torque, tare_ext 명령 동작."""
    pytest.importorskip("pinocchio", reason="pinocchio 미설치")
    monkeypatch.setattr(daemon_mod, "FeetechBus", FakeBus)
    ports = {"state": 6585, "cmd": 6586, "action": 6587}
    arm = pathlib.Path(__file__).resolve().parents[1] / "configs" / "arm.yaml"
    d = daemon_mod.Daemon(CFG, ports, config_path=str(arm))
    assert d.estimator is not None
    t = threading.Thread(target=d.start, daemon=True); t.start(); time.sleep(0.5)
    from sopo.runtime.client import SopoClient
    c = SopoClient("127.0.0.1", ports)
    s = c.state(1.0)
    assert s and set(s["ext_torque"]) == {"J1", "J2", "J4"}  # CFG의 관절만 (모델은 6관절)
    # FR-9: 토크 OFF(IDLE)에서는 tare 거부 — MOVE 정착 상태에서만 허용
    assert c.command("tare_ext")["ok"] is False
    assert c.command("move")["ok"]
    time.sleep(0.3)  # 정착 대기 (soft start 해제 + goal==present)
    r = c.command("tare_ext")
    assert r["ok"] and "J2" in r["biases"]
    c.command("shutdown"); t.join(timeout=3)


def test_estimator_absent_without_config_path(monkeypatch):
    """config_path가 없으면 estimator=None이어도 데몬 정상 (기존 동작 회귀 확인)."""
    monkeypatch.setattr(daemon_mod, "FeetechBus", FakeBus)
    ports = {"state": 6595, "cmd": 6596, "action": 6597}
    d = daemon_mod.Daemon(CFG, ports)
    assert d.estimator is None
    t = threading.Thread(target=d.start, daemon=True); t.start(); time.sleep(0.5)
    from sopo.runtime.client import SopoClient
    c = SopoClient("127.0.0.1", ports)
    s = c.state(1.0)
    assert s["ext_torque"] == {}
    assert c.command("tare_ext")["ok"] is False
    assert c.command("move")["ok"] and d.mode.value == "move"
    c.command("shutdown"); t.join(timeout=3)



def test_goto_degree_unit(monkeypatch):
    """goto unit=deg: REP-103 각도를 calibration zero/dir로 틱에 변환한다."""
    monkeypatch.setattr(daemon_mod, "FeetechBus", FakeBus)
    ports = {"state": 6605, "cmd": 6606, "action": 6607}
    arm = pathlib.Path(__file__).resolve().parents[1] / "configs" / "arm.yaml"
    d = daemon_mod.Daemon(CFG, ports, config_path=str(arm))
    assert d.cal is not None and d.cal.zero_ticks, "calibration.yaml gravity.zero_ticks 필요"
    t = threading.Thread(target=d.start, daemon=True); t.start(); time.sleep(0.5)
    from sopo.runtime.client import SopoClient
    c = SopoClient("127.0.0.1", ports)
    assert c.command("move")["ok"]
    zero_j4 = d.cal.zero_ticks["J4"]
    r = c.command("goto", action={"J4": 0.0}, unit="deg")
    assert r["ok"] and r["accepted"]["J4"] == round(zero_j4)
    r = c.command("goto", action={"J4": 90.0}, unit="deg")
    assert r["ok"] and r["accepted"]["J4"] == round(zero_j4 + 4096 / 4)
    # ticks 경로는 그대로
    r = c.command("goto", action={"J4": 2500}, unit="ticks")
    assert r["ok"] and r["accepted"]["J4"] == 2500
    c.command("shutdown"); t.join(timeout=3)


def test_goto_degree_without_calibration(monkeypatch):
    """calibration이 없으면 degree goto는 명확한 오류를 반환한다."""
    monkeypatch.setattr(daemon_mod, "FeetechBus", FakeBus)
    ports = {"state": 6615, "cmd": 6616, "action": 6617}
    d = daemon_mod.Daemon(CFG, ports)  # config_path 없음 → cal=None
    t = threading.Thread(target=d.start, daemon=True); t.start(); time.sleep(0.5)
    from sopo.runtime.client import SopoClient
    c = SopoClient("127.0.0.1", ports)
    assert c.command("move")["ok"]
    r = c.command("goto", action={"J4": 90.0}, unit="deg")
    assert r["ok"] is False and "230_calibrate_zero" in r["error"]
    r = c.command("goto", action={"J4": 2500})  # ticks는 여전히 동작
    assert r["ok"]
    c.command("shutdown"); t.join(timeout=3)


def _payload_configs(tmp):
    """repo의 configs를 tmp로 복사하고 estimation 섹션(floor/k) + payload 선언을 추가한다."""
    import shutil
    import yaml
    repo = pathlib.Path(__file__).resolve().parents[1] / "configs"
    shutil.copy(repo / "arm.yaml", tmp / "arm.yaml")
    cal = yaml.safe_load((repo / "calibration.yaml").read_text())
    cal["estimation"] = {"floor": {"J1": 0.2, "J2": 0.4, "J4": 0.2}, "k": 1.6}
    (tmp / "calibration.yaml").write_text(yaml.safe_dump(cal))
    arm = yaml.safe_load((tmp / "arm.yaml").read_text())
    arm["payload"] = {"max_kg": 0.5}
    (tmp / "arm.yaml").write_text(yaml.safe_dump(arm))
    return tmp / "arm.yaml"


def test_payload_power_mode_interlock(monkeypatch, tmp_path):
    """FR-7: floor/k + tare 완료 후에만 전력 캡(균일) 적용. 그 전엔 순한 캡(per-motor 오버라이드 포함)."""
    pytest.importorskip("pinocchio", reason="pinocchio 미설치")
    monkeypatch.setattr(daemon_mod, "FeetechBus", FakeBus)
    ports = {"state": 6625, "cmd": 6626, "action": 6627}
    cfg = dict(CFG)
    cfg["safety"] = {**CFG["safety"], "torque_limits": {19: 150}}  # 순한 모드 per-motor 오버라이드
    cfg["payload"] = {"max_kg": 0.5}                                # arm.yaml의 payload 섹션에 해당
    arm = _payload_configs(tmp_path)
    d = daemon_mod.Daemon(cfg, ports, config_path=str(arm))
    assert d.estimator is not None and d._payload_active and not d.power_ok
    t = threading.Thread(target=d.start, daemon=True); t.start(); time.sleep(0.5)
    from sopo.runtime.client import SopoClient
    c = SopoClient("127.0.0.1", ports)
    assert c.command("move")["ok"]
    assert d.bus.regs[(19, "Torque_Limit")] == 150            # 순한 모드: 오버라이드 적용
    assert d.bus.regs[(11, "Torque_Limit")] == 200            # 나머지는 기본 캡
    assert c.state(1.0)["power_ok"] is False
    time.sleep(0.3)                                            # 정착
    assert c.command("tare_ext")["ok"]                         # tare → 인터록 충족
    assert d.power_ok and c.state(1.0)["power_ok"] is True
    assert d.bus.regs[(19, "Torque_Limit")] == 150            # 캡은 토크 ON 시점에만 바뀐다
    c.command("idle"); c.command("move")
    assert d.bus.regs[(19, "Torque_Limit")] == 600            # 전력 모드: 균일 캡
    assert d.bus.regs[(11, "Torque_Limit")] == 600
    c.command("shutdown"); t.join(timeout=3)


def test_payload_envelope_passed_to_reflex(monkeypatch, tmp_path):
    """FR-2: 페이로드 활성이면 매 사이클 ext_limit(포락선)이 reflex.update에 전달된다."""
    pytest.importorskip("pinocchio", reason="pinocchio 미설치")
    monkeypatch.setattr(daemon_mod, "FeetechBus", FakeBus)
    ports = {"state": 6635, "cmd": 6636, "action": 6637}
    cfg = dict(CFG)
    cfg["payload"] = {"max_kg": 0.5}
    arm = _payload_configs(tmp_path)
    d = daemon_mod.Daemon(cfg, ports, config_path=str(arm))
    t = threading.Thread(target=d.start, daemon=True); t.start(); time.sleep(0.5)
    from sopo.runtime.client import SopoClient
    c = SopoClient("127.0.0.1", ports)
    seen = []
    orig = d.reflex.update
    def spy(*a, **kw):
        seen.append(kw.get("ext_limit"))
        return orig(*a, **kw)
    d.reflex.update = spy  # start() 이후 reflex는 교체되지 않으므로 여기서 감싼다
    c.command("move"); time.sleep(0.4)
    c.command("shutdown"); t.join(timeout=3)
    delivered = [x for x in seen if x is not None]
    assert delivered, "ext_limit가 한 번도 전달되지 않음"
    last = delivered[-1]
    assert {"J1", "J2", "J4"} <= set(last)
    assert abs(last["J1"] - 0.2) < 1e-6    # 수직축은 extra≈0 → floor 그대로
    assert 0.4 < last["J2"]                # floor 0.4 + |extra(q)|×1.6 (자세 의존)


def test_auto_tare_on_settle_and_guard(monkeypatch):
    """FR-8: 정착 에지에서 자동 tare. 큰 외력이 걸려 있으면 건너뛴다."""
    monkeypatch.setattr(daemon_mod, "FeetechBus", FakeBus)
    ports = {"state": 6645, "cmd": 6646, "action": 6647}

    class StubEst:
        def __init__(self, ext): self._ext = ext; self.calls = 0
        def update(self, *a, **k): return dict(self._ext)
        def tare(self): self.calls += 1
        @property
        def biases(self): return {"J2": 0.1}

    d = daemon_mod.Daemon(CFG, ports)   # config_path 없음 → payload 비활성 (floor/k 없음)
    d.estimator = StubEst({})           # 잔차 0 → auto-tare 허용
    t = threading.Thread(target=d.start, daemon=True); t.start(); time.sleep(0.5)
    from sopo.runtime.client import SopoClient
    c = SopoClient("127.0.0.1", ports)
    c.command("move"); time.sleep(0.4)
    assert d.estimator.calls >= 1 and d._tare_done          # 정착 에지에서 tare 됨
    assert not d.power_ok                                    # floor/k 없으면 전력 모드 아님
    # 큰 외력이 걸린 상태에서 새 정착 에지(이동→도착)를 만들면 건너뛴다
    d.estimator._ext = {"J2": 5.0}
    c.command("goto", action={"J4": 2300})
    time.sleep(0.8)
    assert d.estimator.calls == 1                            # 추가 tare 없음 (가드)
    c.command("shutdown"); t.join(timeout=3)

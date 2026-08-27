"""명령 소스(ActionSource): 매 사이클 관절 목표 {관절이름: 틱}을 내놓는 것.

lerobot의 `Teleoperator`(connect / get_action / disconnect) 경계를 그대로 따른다. 지금 구현된 건
웨이포인트뿐이지만, 리더 암(SO-ARM/lerobot leader)이나 정책을 같은 자리에 끼울 수 있도록
아래에 플레이스홀더를 주석으로 남긴다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

import yaml


class ActionSource(Protocol):
    name: str

    def connect(self) -> None: ...

    def get_action(self, present: dict[str, int], now: float) -> dict[str, int]:
        """현재 관절 위치와 시각을 받아 관절 목표를 돌려준다. 빠진 관절은 현재 위치 유지."""
        ...

    def is_done(self) -> bool: ...

    def disconnect(self) -> None: ...


@dataclass
class WaypointSource:
    """관절 목표 리스트를 순서대로. 각 목표에 도달(지정 관절 모두 arrival_ticks 이내)하고
    dwell_s만큼 머문 뒤 다음으로 넘어간다."""

    waypoints: list[dict[str, int]]
    dwell_s: float = 1.0
    loop: bool = False
    arrival_ticks: int = 30
    name: str = "waypoints"
    _index: int = field(default=0, init=False)
    _arrived_at: float | None = field(default=None, init=False)
    _done: bool = field(default=False, init=False)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "WaypointSource":
        data = yaml.safe_load(Path(path).read_text()) or {}
        wps = [{str(k): int(v) for k, v in wp.items()} for wp in data["waypoints"]]
        return cls(wps, dwell_s=float(data.get("dwell_s", 1.0)), loop=bool(data.get("loop", False)))

    @property
    def index(self) -> int:
        return self._index

    def connect(self) -> None:
        if not self.waypoints:
            raise ValueError("웨이포인트가 비어 있습니다.")

    def get_action(self, present: dict[str, int], now: float) -> dict[str, int]:
        if self._done:
            return dict(present)
        wp = self.waypoints[self._index]
        unknown = set(wp) - set(present)
        if unknown:
            raise KeyError(f"웨이포인트 {self._index}에 없는 관절 이름: {sorted(unknown)}")
        if all(abs(present[n] - wp[n]) < self.arrival_ticks for n in wp):
            if self._arrived_at is None:
                self._arrived_at = now
            elif now - self._arrived_at >= self.dwell_s:
                self._advance()
                wp = self.waypoints[self._index] if not self._done else {}
        else:
            self._arrived_at = None
        return {n: wp.get(n, present[n]) for n in present}

    def _advance(self) -> None:
        self._arrived_at = None
        if self._index + 1 < len(self.waypoints):
            self._index += 1
        elif self.loop:
            self._index = 0
        else:
            self._done = True

    def is_done(self) -> bool:
        return self._done

    def disconnect(self) -> None:
        pass


class JogSource:
    """키보드 조그: 선택한 관절의 목표를 키 입력마다 step틱씩 옮긴다. 리더 암 없이 자세를 찾는 용도.

    터미널 I/O는 주입한다(`poll_keys()` → 키 이름 리스트). 키 이름:
      left/right: 선택 관절 -/+ step   up/down: 관절 선택 이동   '1'..'9': 관절 직접 선택
      '+'/'-': step 증감   ' ': 홀드(목표=현재)   'q': 종료
    목표는 현재 위치 ±lead 안으로 묶어, 키를 떼면 곧 멈춘다(막힌 채 키를 눌러도 목표가 도망가지 않음).
    """

    name = "jog"

    def __init__(self, joint_names: list[str], poll_keys, step: int = 40, lead: int = 200):
        self.joint_names = list(joint_names)
        self.poll_keys = poll_keys
        self.step = step
        self.lead = lead
        self.active = 0
        self.targets: dict[str, int] | None = None
        self._present: dict[str, int] = {}
        self._quit = False
        self.last_status = ""

    @property
    def active_joint(self) -> str:
        return self.joint_names[self.active]

    def connect(self) -> None:
        pass

    def handle_key(self, key: str) -> None:
        if self.targets is None:
            return
        n = len(self.joint_names)
        if key == "left":
            self.targets[self.active_joint] -= self.step
        elif key == "right":
            self.targets[self.active_joint] += self.step
        elif key == "up":
            self.active = (self.active - 1) % n
        elif key == "down":
            self.active = (self.active + 1) % n
        elif key.isdigit() and 1 <= int(key) <= n:
            self.active = int(key) - 1
        elif key in ("+", "="):
            self.step = min(200, self.step + 10)
        elif key == "-":
            self.step = max(5, self.step - 10)
        elif key == " ":
            self.targets = dict(self._present)
        elif key == "q":
            self._quit = True

    def get_action(self, present: dict[str, int], now: float) -> dict[str, int]:
        self._present = dict(present)
        if self.targets is None:
            self.targets = dict(present)
        for key in self.poll_keys():
            self.handle_key(key)
        for name in self.targets:  # 리드 제한
            lo, hi = present[name] - self.lead, present[name] + self.lead
            self.targets[name] = max(lo, min(hi, self.targets[name]))
        self.last_status = "  ".join(
            f"{'[' + n + ']' if i == self.active else n}:{present[n]}->{self.targets[n]}"
            for i, n in enumerate(self.joint_names)
        ) + f"  step={self.step}"
        return dict(self.targets)

    def is_done(self) -> bool:
        return self._quit

    def disconnect(self) -> None:
        pass


class StreamSource:
    """외부(정책·조그 클라이언트)가 보내는 액션 스트림. sopod가 쓴다.

    push(action, now)로 최신 액션을 갱신하고, get_action은 마지막 액션을 돌려준다.
    watchdog_s 동안 새 액션이 없으면 현재 위치(홀드)를 돌려준다 — 정책이 멈추면 팔도 멈춘다.
    """

    name = "stream"

    def __init__(self, watchdog_s: float = 0.5):
        self.watchdog_s = watchdog_s
        self._action: dict[str, int] | None = None
        self._last_push = -1e9
        self._sticky = False
        self.stale = True
        self._hold: dict[str, int] | None = None

    def push(self, action: dict[str, int], now: float, sticky: bool = False) -> None:
        """sticky=True: 다음 push까지 워치독 없이 유지 (goto 같은 단발 목표)."""
        self._action = {str(k): int(v) for k, v in action.items()}
        self._last_push = now
        self._sticky = sticky

    def clear(self) -> None:
        self._action = None

    def connect(self) -> None:
        pass

    def get_action(self, present: dict[str, int], now: float) -> dict[str, int]:
        stale = self._action is None or (not self._sticky and (now - self._last_push) > self.watchdog_s)
        if stale:
            # 홀드: 진입 순간의 위치를 목표로 고정한다. 매 사이클 present를 따라가면 밀리는 대로
            # 끌려가서 저항도 못 하고 외력 감지(DISTURBANCE)도 안 된다.
            if not self.stale or self._hold is None:
                self._hold = dict(present)
            self.stale = True
            return {n: self._hold.get(n, present[n]) for n in present}
        self.stale = False
        self._hold = None
        return {n: self._action.get(n, present[n]) for n in present}

    def is_done(self) -> bool:
        return False

    def disconnect(self) -> None:
        pass


# ---------------------------------------------------------------------------
# 플레이스홀더 1: 리더 암 (lerobot Teleoperator / SO-ARM leader 구조)
#
# lerobot의 SO100/SO101 leader는 팔로워와 같은 종류의 모터를 단 암을 손으로 움직여
# 그 관절각을 액션으로 내보낸다. 핵심 구조:
#   - connect(calibrate=True): 자기 버스 연결 → 캘리브레이션 없으면 calibrate() → configure()
#   - configure(): 리더는 항상 토크 OFF (bus.disable_torque) — Trossen puppet의 master_modes와 동일
#   - get_action(): bus.sync_read("Present_Position") → {"<motor>.pos": 값}
#   - 리더/팔로워 조립 차이는 캘리브레이션(homing offset + 관절 범위)으로 정규화([-100,100] 또는 [0,100])해서
#     흡수한다. 팔로워는 같은 정규화의 역변환으로 목표를 만든다. sopo는 틱 단위라 관절별 invert/offset을
#     두거나, SopoRobot 캘리브레이션 층(틱 ↔ 정규화)이 생기면 그걸 쓴다.
#
# class LeaderArmSource:
#     name = "leader_arm"
#
#     def __init__(self, port: str, joints_cfg: list[dict], invert: set[str] = (), offsets: dict[str, int] = None):
#         from .bus import FeetechBus
#         from .joints import build_joints
#         self.bus = FeetechBus(port)
#         self.joints = build_joints(joints_cfg)      # 리더도 같은 관절 구조(듀얼/연속 포함)라고 가정
#         self.invert, self.offsets = set(invert), dict(offsets or {})
#
#     def connect(self) -> None:
#         self.bus.connect()
#         self.bus.disable_torque([mid for j in self.joints for mid in j.motor_ids])   # 손으로 움직여야 하므로
#
#     def get_action(self, present, now):
#         leader = {j.name: j.read(self.bus) for j in self.joints}                     # 연속 관절은 논리각
#         goal = {}
#         for name, pos in leader.items():
#             if name in self.invert:
#                 pos = 4095 - pos
#             goal[name] = pos + self.offsets.get(name, 0)
#         return goal          # 클램프(관절 리밋·스텝·J1 범위)는 control 루프가 한다
#
#     def is_done(self) -> bool:
#         return False         # 사용자가 Ctrl+C 할 때까지
#
#     def disconnect(self) -> None:
#         self.bus.disconnect()
#
# 사용: run_waypoints.py의 WaypointSource 자리에 LeaderArmSource(port=..., joints_cfg=cfg["joints"])를 넣으면
#       그대로 리더-팔로워 미러가 된다. 소프트스타트(첫 목표까지 느린 스텝)도 그대로 적용된다.
#
# ---------------------------------------------------------------------------
# 플레이스홀더 2: 정책 (lerobot policy / VLA)
#
# lerobot 추론 루프: obs = robot.get_observation(); action = policy.select_action(obs); robot.send_action(action)
# 정책은 10~30 Hz로 액션(청크)을 내므로, 이 소스는 마지막 액션을 보간해 50 Hz 루프에 내보내야 한다
# (docs/architecture.md L4). 정책이 멈추면 get_action이 present(홀드)를 돌려주고 워치독이 처리한다.
#
# class PolicySource:
#     name = "policy"
#     def __init__(self, policy, observe, hz=30): ...
#     def connect(self): ...
#     def get_action(self, present, now):
#         if now - self._last >= 1 / self.hz:
#             self._target = denormalize(self.policy.select_action(self.observe()))
#             self._last = now
#         return interpolate(present, self._target, ...)
#     def is_done(self): return False
#     def disconnect(self): ...

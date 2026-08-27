"""sopod 클라이언트: 상태 구독, 명령, 액션 스트림. 정책·조그·GUI는 이걸로만 팔에 접근한다."""

from __future__ import annotations

import time

import zmq

from .daemon import DEFAULT_PORTS


class SopoClient:
    def __init__(self, host: str = "127.0.0.1", ports: dict | None = None):
        p = {**DEFAULT_PORTS, **(ports or {})}
        ctx = zmq.Context.instance()
        self.sub = ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.SUBSCRIBE, b""); self.sub.setsockopt(zmq.CONFLATE, 1)
        self.sub.connect(f"tcp://{host}:{p['state']}")
        self.req = ctx.socket(zmq.REQ)
        self.req.setsockopt(zmq.RCVTIMEO, 65000); self.req.setsockopt(zmq.LINGER, 0)
        self.req.connect(f"tcp://{host}:{p['cmd']}")
        self.pub = ctx.socket(zmq.PUB)
        self.pub.connect(f"tcp://{host}:{p['action']}")
        self.lease: str | None = None
        time.sleep(0.1)  # PUB/SUB 연결 대기

    def state(self, timeout_s: float = 1.0) -> dict | None:
        """최신 상태 1개 (CONFLATE). 없으면 None."""
        if self.sub.poll(int(timeout_s * 1000)):
            return self.sub.recv_json()
        return None

    def command(self, cmd: str, **kw) -> dict:
        self.req.send_json({"cmd": cmd, **kw})
        return self.req.recv_json()

    def acquire(self, name: str = "client") -> dict:
        """배타적 제어권(리스). reflex/stopped/idle 때 회수되므로 그 뒤엔 recover → acquire를 다시 해야 한다."""
        r = self.command("acquire", name=name)
        self.lease = r.get("lease") if r.get("ok") else None
        return r

    def release(self) -> dict:
        r = self.command("release", lease=self.lease)
        self.lease = None
        return r

    def send_action(self, action: dict[str, int]) -> None:
        """관절 목표 {name: tick}. acquire()한 리스가 있어야 데몬이 받는다. 워치독(0.5s) 안에 계속 보낼 것."""
        self.pub.send_json({"action": {k: int(v) for k, v in action.items()}, "lease": self.lease}, flags=zmq.NOBLOCK)

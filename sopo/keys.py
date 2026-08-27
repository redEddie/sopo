"""cbreak 모드 논블로킹 키 읽기 (jog 클라이언트용)."""

import os
import select
import sys
import termios
import tty

KEYMAP = {"\x1b[D": "left", "\x1b[C": "right", "\x1b[A": "up", "\x1b[B": "down"}
ALIASES = {"a": "left", "d": "right", "w": "up", "s": "down", "h": " "}  # 화살표가 안 먹는 터미널용


class KeyReader:
    """cbreak 모드 논블로킹 키 읽기. sys.stdin.read()는 파이썬 버퍼가 키를 삼키므로 os.read(fd)를 쓴다."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.saved = None
        self._buf = b""

    def __enter__(self):
        self.reenter()
        return self

    def __exit__(self, *exc):
        self.restore()

    def restore(self):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
            self.saved = None

    def reenter(self):
        if self.saved is None:
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def poll(self) -> list[str]:
        while select.select([self.fd], [], [], 0)[0]:
            chunk = os.read(self.fd, 64)
            if not chunk:
                break
            self._buf += chunk
        keys: list[str] = []
        buf = self._buf
        self._buf = b""
        i = 0
        while i < len(buf):
            if buf[i:i + 1] == b"\x1b":
                seq = buf[i:i + 3].decode(errors="ignore")
                if seq in KEYMAP:
                    keys.append(KEYMAP[seq]); i += 3; continue
                if len(buf) - i < 3:          # 시퀀스가 아직 덜 옴 → 다음 poll에서
                    self._buf = buf[i:]; break
                i += 1; continue
            ch = buf[i:i + 1].decode(errors="ignore")
            keys.append(ALIASES.get(ch, ch)); i += 1
        return keys



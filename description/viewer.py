#!/usr/bin/env python3
"""sopo MuJoCo 뷰어 — simulate UI의 Control 패널 슬라이더로 조인트를 움직인다.

sopo_viewer.xml은 postprocess.py가 생성한다 (중력 off + position actuator 포함).
실행 후 시뮬레이션을 시작(스페이스바)하면 Control 패널 슬라이더로 포징할 수 있다.

실행:
    cd urdf && ../.venv/bin/python viewer.py
"""

from pathlib import Path

import mujoco.viewer

XML = Path(__file__).resolve().parent / "sopo_viewer.xml"


def main() -> None:
    if not XML.exists():
        raise SystemExit(f"{XML}이 없습니다. 먼저 onshape-to-robot export(또는 postprocess.py)를 실행하세요.")
    mujoco.viewer.launch_from_path(str(XML))


if __name__ == "__main__":
    main()

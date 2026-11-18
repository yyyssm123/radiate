#!/usr/bin/env python3
"""仿真和真机共用的 DDS 扫描入口，只有 --network 不同。"""
import sys

from g1d_transport import ControlFault
from run_real import main


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ControlFault, ValueError, OSError) as exc:
        print(f"已停止轨迹与发送: {exc}", file=sys.stderr)
        raise SystemExit(2)

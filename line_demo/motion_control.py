#!/usr/bin/env python3
"""官方 MotionSwitcher 的状态查询与明确执行的退出操作，不发布电机指令。"""
import argparse
from contextlib import nullcontext
import json
import math
import sys
import time

from dds_network import initialize
from g1d_transport import ControlFault


def read_mode(client):
    code, data = client.CheckMode()
    if code != 0:
        raise ControlFault(f"CheckMode 失败，返回码={code}；未确认内置运控退出")
    if not isinstance(data, dict) or not isinstance(data.get("name"), str):
        raise ControlFault(f"CheckMode 返回格式无法确认: {data!r}")
    return data


def make_client(timeout=2.):
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    client = MotionSwitcherClient()
    client.SetTimeout(timeout)
    client.Init()
    return client


def require_released(network):
    if network == "lo":
        return {"scope": "local loopback simulator; no built-in motion controller"}
    mode = read_mode(make_client())
    if mode["name"]:
        raise ControlFault(f"内置运控仍为 {mode['name']!r}；先完成现场支撑，运行 "
                           f"python3 line_demo/motion_control.py release --network {network} "
                           "退出并确认后重新捕获姿态。本次未创建 lowcmd writer")
    return {"check_mode": mode, "scope": "MotionSwitcher reports no active mode; other writers must be excluded on site"}


def wait_start_button(robot, initial, limits, network):
    """G1-D 专用示例的 A 键启动；等待期间不创建电机指令 writer。"""
    if network == "lo":
        return initial
    from real_motion import Guard
    guard = Guard(limits)
    guard.state(initial, time.monotonic())
    previous = initial.buttons
    print("按 G1-D 官方示例等待遥控器 A 启动。保持机器人静止；按 B 或 Ctrl+C 取消。", flush=True)
    if previous & (1 << 8):
        print("检测到 A 已按住，请松开后重新按 A。", flush=True)
    try:
        while True:
            state = robot.read()
            guard.state(state, time.monotonic())
            if state.buttons & (1 << 9):
                raise ControlFault("启动已由遥控器 B 取消，尚未发送电机指令")
            pressed = state.buttons & ~previous
            previous = state.buttons
            if pressed & (1 << 8):
                print("收到 A：捕获当前姿态并进行轨迹预检，随后保持 2 秒再平滑接入。", flush=True)
                return state
            time.sleep(.005)
    except KeyboardInterrupt as exc:
        raise ControlFault("启动确认已取消，尚未发送电机指令") from exc


def release_mode(client, timeout=10.):
    before = read_mode(client)
    if not before["name"]:
        return {"before": before, "after": before, "release_calls": 0}
    code, _ = client.ReleaseMode()
    if code != 0:
        raise ControlFault(f"ReleaseMode 失败，返回码={code}；不继续发送运动指令")
    deadline = time.monotonic()+timeout
    while True:
        after = read_mode(client)
        if not after["name"]:
            return {"before": before, "after": after, "release_calls": 1}
        if time.monotonic() >= deadline:
            raise ControlFault(f"ReleaseMode 已返回成功，但运控仍为 {after['name']!r}，未确认退出")
        time.sleep(.2)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("status", "release"))
    parser.add_argument("--network", required=True)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=10.)
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("timeout 必须为有限正数")
    # 与扫描程序共用本地写入锁，避免在本程序的另一个发送会话中切换控制权。
    from run_real import writer_lock
    context = writer_lock(args.network, args.domain) if args.action == "release" else nullcontext()
    with context:
        initialize(args.network, args.domain)
        client = make_client(min(args.timeout, 2.))
        result = (release_mode(client, args.timeout) if args.action == "release"
                  else {"mode": read_mode(client), "release_calls": 0})
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if args.action == "release":
        print("已确认 MotionSwitcher 的 name 为空；未发布电机指令。请重新读取 LowState 捕获姿态。")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ControlFault, ValueError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2)

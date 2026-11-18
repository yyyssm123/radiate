#!/usr/bin/env python3
"""通过官方 robot_state RPC 查询、关闭或恢复一个明确指定的服务。"""
import argparse
import json
import math
import sys
import time

from dds_network import initialize


def service_list(client):
    code, services = client.ServiceList()
    if code != 0 or services is None:
        raise RuntimeError(f"ServiceList 失败，返回码={code}；检查真机网卡与设备状态服务")
    return services


def find_service(services, name):
    for service in services:
        if service.name == name:
            return service
    raise RuntimeError(f"服务列表中没有 {name!r}；先执行 list，使用设备返回的准确名称")


def status_codes(services):
    # switch 请求的 0/1 与 ServiceList.status 并非同一个枚举，不能 int(enabled)。
    # robot_state 正在响应此次 RPC，以其条目作为本固件的运行状态锚点。
    # 用户 G1-D 的锚点为 0，故此设备 0=运行、1=停止；不把这一对应硬套到其他固件。
    anchor = find_service(services, "robot_state")
    if anchor.status not in (0, 1):
        raise RuntimeError(f"robot_state 状态为 {anchor.status}，无法确认状态编码，没有切换服务")
    return anchor.status, 1-anchor.status


def switch_service(client, name, enabled, timeout=5.):
    services = service_list(client)
    current = find_service(services, name)
    running, stopped = status_codes(services)
    desired = running if enabled else stopped
    if current.status == desired:
        return current  # 已处于目标状态，不再调用切换接口。
    if current.status not in (0, 1):
        raise RuntimeError(f"{name}: 未识别的服务状态 {current.status}，没有切换")
    if current.protect:
        raise RuntimeError(f"{name}: protect={current.protect}，设备禁止切换受保护服务")
    code = client.ServiceSwitch(name, bool(enabled))
    if code != 0:
        reason = "服务受保护" if code == 5202 else "切换失败"
        raise RuntimeError(f"ServiceSwitch({name!r}, {enabled}) {reason}，返回码={code}")
    # RPC 成功不等于进程已完成退出/启动，以后续 ServiceList 为准。
    deadline = time.monotonic() + timeout
    while True:
        services = service_list(client)
        if status_codes(services) != (running, stopped):
            raise RuntimeError("robot_state 的状态编码在切换期间改变，未确认服务状态")
        current = find_service(services, name)
        if current.status == desired:
            return current
        if time.monotonic() >= deadline:
            raise RuntimeError(f"切换请求已接受，但 {name} 读回状态仍为 {current.status}；未确认完成")
        time.sleep(.2)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("list", "off", "on"))
    parser.add_argument("--network", required=True, help="实际连接机器人的网卡")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--name", help="off/on 必填：ServiceList 返回的准确名称")
    parser.add_argument("--timeout", type=float, default=5.)
    args = parser.parse_args(argv)
    if args.action != "list" and not args.name:
        parser.error("off/on 必须显式指定 --name，不猜测 G1-D 固件的服务名称")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("timeout 必须是有限正数")
    # 本机 SDK 的 b2 模块有失效的 client_internal 导入；go2 模块的这两个接口
    # 使用相同 robot_state 服务、API 版本 1.0.0.1 和 API ID 1001/1003。
    from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
    initialize(args.network, args.domain)
    client = RobotStateClient()
    client.SetTimeout(min(args.timeout, 3.))
    client.Init()
    if args.action == "list":
        services = service_list(client)
        try:
            running, stopped = status_codes(services)
            labels = {running: "running", stopped: "stopped"}
        except RuntimeError:
            labels = {}  # 仍保留原始列表，无法解释的状态显示 unknown。
    else:
        services = [switch_service(client, args.name, args.action == "on", args.timeout)]
        labels = {services[0].status: "running" if args.action == "on" else "stopped"}
    print(json.dumps([{"name": s.name, "status": s.status, "protect": s.protect}
                      | {"state_label": labels.get(s.status, "unknown")} for s in services],
                     ensure_ascii=False, indent=2))
    if args.action != "list":
        print(f"已读回确认：{args.name} {'运行' if args.action == 'on' else '停止'}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2)

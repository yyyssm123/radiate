#!/usr/bin/env python3
"""只读采集 G1-D 原始 LowState 和 CheckMode；不创建运动指令 writer，不切换运控。"""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "line_demo"))

from g1d_config import CONTROLLED_JOINTS, JOINT_SLOTS, URDF
from joint_limits import joint_limit_violations


def annotate_limits(report):
    rows = {int(row["slot"]): row for row in report["slots"]}
    missing = sorted(set(JOINT_SLOTS.values())-rows.keys())
    if missing:
        raise ValueError(f"原始报告缺少模型槽位: {missing}")
    q = [rows[JOINT_SLOTS[name]]["q"] for name in CONTROLLED_JOINTS]
    violations = joint_limit_violations(q)
    startup = joint_limit_violations(q, .02)
    report["joint_limit_check"] = {
        "urdf": str(URDF), "within_urdf": not violations,
        "violations": violations, "startup_margin_rad": .02,
        "within_startup_margin": not startup, "startup_margin_violations": startup,
        "scope": "仅比较原始 q 与本地 URDF；不判定硬件机械限位、零位标定或运动准备状态"}
    return report


def capture(network, domain):
    from dds_network import initialize
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.utils.crc import CRC
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    initialize(network, domain)
    reader = ChannelSubscriber("rt/lowstate", LowState_)
    reader.Init()
    crc, msg, good, bad = CRC(), None, 0, 0
    deadline = time.monotonic()+4.
    try:
        while time.monotonic() < deadline:
            candidate = reader.Read()
            if candidate is not None:
                if int(candidate.crc) == crc.Crc(candidate):
                    msg, good = candidate, good+1
                    if good >= 20:
                        break
                else:
                    bad += 1
            time.sleep(.002)
    finally:
        reader.Close()
    if msg is None:
        raise SystemExit(f"未收到有效状态：valid={good}, bad_crc={bad}；未发运动指令")
    rows = []
    for slot, motor in enumerate(msg.motor_state):
        fields = {name: getattr(motor, name) for name in
                  ("mode", "q", "dq", "ddq", "tau_est", "temperature", "vol", "motorstate", "sensor", "reserve")
                  if hasattr(motor, name)}
        rows.append({"slot": slot, **fields})
    report = {"scope": "READ ONLY; no rt/lowcmd writer; no mode/service change",
              "valid_packets": good, "bad_crc": bad, "tick": int(msg.tick),
              "mode_pr": int(msg.mode_pr), "mode_machine": int(msg.mode_machine), "slots": rows}
    client = MotionSwitcherClient()
    client.SetTimeout(2.)
    client.Init()
    code, mode = client.CheckMode()
    report["motion_check"] = {"code": code, "mode": mode}
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--network", help="网卡名；只读采集实时反馈")
    source.add_argument("--from-report", type=Path, help="分析已有原始报告，不连接 DDS")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.from_report:
        report = json.loads(args.from_report.read_text())
        report["analysis_source"] = str(args.from_report)
    else:
        report = capture(args.network, args.domain)
    annotate_limits(report)
    args.report = args.report or ROOT / "out" / (
        "device_limit_analysis.json" if args.from_report else "device_raw_lowstate.json")
    if args.from_report and args.report.resolve() == args.from_report.resolve():
        parser.error("分析报告请使用另一个文件名，保留原始采集数据")
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n")
    print(json.dumps(report, indent=2, ensure_ascii=False), flush=True)
    print(f"保存到 {args.report}；未发运动指令，未切换运控。", flush=True)
    if args.from_report:
        print("本次仅分析已有文件，未连接 DDS。", flush=True)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""仅绑定 localhost 的真实 DDS + MuJoCo 往返测试，不连接物理网卡。"""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def wait_text(process, path, needle, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        text = path.read_text(errors="replace") if path.exists() else ""
        if needle in text:
            return
        if process.poll() is not None:
            raise RuntimeError(f"进程提前退出 {process.returncode}:\n{text[-6000:]}")
        time.sleep(.05)
    raise RuntimeError(f"等待 {needle} 超时；日志 {path}")


def stop(process, twice=False):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    if twice:
        time.sleep(.05)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


def check_startup(out, physics):
    """用实际发送记录复核非零起点、连续运动和返回，避免零位夹具掩盖问题。"""
    plan = json.loads((out / "plan.json").read_text())
    captured = plan["captured_q_rad"]
    names = plan["joint_names"]
    with (out / "trajectory.csv").open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    targets = [[float(row[f"{name}.q_cmd_rad"]) for name in names] for row in rows]
    if len(rows) < 10 or max(abs(q) for q in captured) < .1:
        raise RuntimeError("测试必须使用非零垂臂姿态并记录实际控制过程")
    first_error = max(abs(a-b) for a, b in zip(targets[0], captured))
    final_error = max(abs(a-b) for a, b in zip(targets[-1], captured))
    initial_error = max(abs(a-b) for a, b in zip(physics["initial_q_rad"], captured))
    other = [i for i, name in enumerate(names) if not name.startswith("left_")]
    other_error = max(abs(q[i]-captured[i]) for q in targets for i in other)
    max_speed = max(max(abs(a-b) for a, b in zip(q1, q0)) /
                    (float(r1["elapsed_s"])-float(r0["elapsed_s"]))
                    for r0, r1, q0, q1 in zip(rows, rows[1:], targets, targets[1:]))
    startup_rows = [q for row, q in zip(rows, targets) if row["phase"] == "startup-hold"]
    if not startup_rows:
        raise RuntimeError("没有捕获姿态的启动保持阶段")
    hold_error = max(abs(a-b) for q in startup_rows for a, b in zip(q, captured))
    if max(first_error, final_error, initial_error, other_error, hold_error) > 1e-6 or max_speed > .251:
        raise RuntimeError(f"垂臂接入/返回或速度检查失败: {first_error=}, {final_error=}, "
                           f"{initial_error=}, {other_error=}, {hold_error=}, {max_speed=}")
    return {"captured_q_rad": captured, "first_target_error_rad": first_error,
            "startup_hold_error_rad": hold_error, "final_target_error_rad": final_error,
            "waist_and_right_target_change_rad": other_error,
            "max_logged_target_speed_rad_s": max_speed}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action", choices=("probe", "scan"), default="scan")
    parser.add_argument("--domain", type=int, default=211)
    parser.add_argument("--speed", type=float, default=.03, help="scan 请求的路径峰值 m/s")
    parser.add_argument("--pose-json", type=Path, help="只用于 lo 仿真复现已记录的初始姿态")
    parser.add_argument("--trace", action="store_true", help="记录 CycloneDDS 发现诊断")
    parser.add_argument("--read-test", action="store_true", help="直接 reader 诊断")
    args = parser.parse_args()
    if args.trace:
        os.environ["G1D_DDS_TRACE"] = "1"
    if not 1 <= args.domain <= 220:
        parser.error("测试域使用 1–220，避免使用真机默认域 0")
    out = ROOT / "out" / f"dds_{args.action}_{time.time_ns()}"
    out.mkdir(parents=True)
    server_log, client_log = out / "sim.log", out / "client.log"
    sim = client = None
    base = [sys.executable, "-u", str(ROOT / "line_demo" / "line_scan.py")]
    connection = ["--network", "lo", "--domain", str(args.domain)]
    motion_args = ["--speed", str(args.speed)]
    sim_pose_args = ["--pose-json", str(args.pose_json.resolve())] if args.pose_json else []
    print(f"真实 DDS 测试：loopback 域 {args.domain}，记录 {out}", flush=True)
    try:
        with server_log.open("w") as sf, client_log.open("w") as cf:
            sim = subprocess.Popen([sys.executable, "-u", str(ROOT / "line_demo" / "mujoco_sim.py"),
                                    *connection, *sim_pose_args, "--no-viewer", "--report", str(out / "sim.json")],
                                   cwd=ROOT, stdout=sf, stderr=subprocess.STDOUT)
            wait_text(sim, server_log, "DDS_SIM_READY", 15)
            print("MuJoCo DDS 仿真器已启动", flush=True)
            if args.read_test:
                diagnostic = subprocess.run([sys.executable, "-u", str(ROOT / "tools" / "dds_read_probe.py"),
                                             str(args.domain)], capture_output=True, text=True, timeout=10)
                (out / "read_probe.log").write_text(diagnostic.stdout + diagnostic.stderr)
                print(diagnostic.stdout[-1000:], flush=True)
                if diagnostic.returncode:
                    raise RuntimeError("直接 reader 诊断失败，详见 read_probe.log")
            for action, extra, name in (("inspect", [], "inspect"),
                                         (args.action, ["--dry-run"], "dry_run")):
                result = subprocess.run([*base, action, *connection, *motion_args, *extra,
                                         "--report", str(out / f"{name}.json")],
                                        cwd=ROOT, capture_output=True, text=True, timeout=45)
                (out / f"{name}.log").write_text(result.stdout + result.stderr)
                if result.returncode:
                    raise RuntimeError(result.stdout + result.stderr)
                report = json.loads((out / f"{name}.json").read_text())
                if not report["state_layout_compatible"] or report["published_commands"] != 0:
                    raise RuntimeError(f"只读入口检查失败: {report}")
            print("inspect 和 dry-run 均通过，开始真实 DDS 控制", flush=True)
            client = subprocess.Popen([*base, args.action, *connection, *motion_args, "--cycles", "1",
                                       "--report", str(out / "plan.json"),
                                       "--log", str(out / "trajectory.csv")],
                                      cwd=ROOT, stdout=cf, stderr=subprocess.STDOUT)
            wait_text(client, client_log, "finished-hold", 240)
            stop(client, twice=True)
            if client.returncode:
                raise RuntimeError(client_log.read_text(errors="replace")[-6000:])
            stop(sim)
            if sim.returncode:
                raise RuntimeError(server_log.read_text(errors="replace")[-6000:])
        physics = json.loads((out / "sim.json").read_text())
        status = json.loads((out / "trajectory.status.json").read_text())
        if (physics["commands_accepted"] < 100 or physics["commands_rejected"] != 0
                or physics["physics_steps"] <= 0 or physics["max_tip_tracking_error_mm"] < .1
                or status["outcome"] != "operator-exit"):
            raise RuntimeError(f"DDS/物理闭环检查失败: {physics}, {status}")
        startup = check_startup(out, physics)
        result = {"passed": True, "action": args.action, "requested_speed_m_s": args.speed,
                  "network": "lo", "domain": args.domain,
                  "transport": "actual SDK DDS, separate simulator and controller processes",
                  "feedback": "MuJoCo physical dynamics, not ideal following",
                  "records": str(out), "startup": startup, "physics": physics, "controller": status}
        (out / "result.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
        (ROOT / "out" / f"dds_{args.action}_e2e.json").write_text(
            json.dumps(result, indent=2, ensure_ascii=False) + "\n")
        print(json.dumps(result, indent=2, ensure_ascii=False), flush=True)
    finally:
        stop(client, twice=True)
        stop(sim)


if __name__ == "__main__":
    main()

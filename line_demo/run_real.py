#!/usr/bin/env python3
"""G1-D 统一 DDS 控制。仿真用 --network lo，真机只换物理网卡名。"""
import argparse
from contextlib import contextmanager
from datetime import datetime
import fcntl
import json
from pathlib import Path
import re
import signal
import sys
import time
from zoneinfo import ZoneInfo

import numpy as np

from g1d_config import (ROOT, CONTROLLED_JOINTS, ARM_JOINTS, initial_pose,
                        DEFAULT_LOCKED_WAIST_PITCH, WAIST_PITCH_INDEX, load_assembly)
from g1d_transport import ControlFault, DdsRobot, KP, KD
from fixed_scan import FixedScanSegments, fixed_scan_plan
from joint_limits import joint_limit_violations
from real_motion import GRAVITY_JOINTS, Geometry, Guard, Limits, scan_plan, probe_plan
from real_runtime import Controller, CsvLog, SETTLE_SECONDS
from motion_control import require_released, wait_start_button


def timestamp():
    return datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S")


def dump(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, default=float) + "\n")


def print_timing(report):
    plan = report.get("plan", report)
    if "requested_peak_speed_m_s" in plan:
        print(f"扫描规划：请求峰值 {plan['requested_peak_speed_m_s']*100:.2f} cm/s，"
              f"规划路径峰值 {plan['planned_peak_path_speed_m_s']*100:.2f} cm/s，"
              f"扫描往返 {plan['scan_s']:.2f} s；"
              f"关节峰值 {plan['planned_peak_joint_speed_rad_s']:.3f} rad/s、"
              f"{plan['planned_peak_joint_acceleration_rad_s2']:.3f} rad/s²。", flush=True)


@contextmanager
def writer_lock(network, domain):
    # 只防止本程序多实例竞争，不代表已经排除其他 SDK 控制器。
    if not re.fullmatch(r"[A-Za-z0-9_.:-]+", network or "") or not 0 <= domain <= 232:
        raise ControlFault("网卡名或 DDS domain 不合法")
    with open(f"/tmp/g1d_lowcmd_{network}_{domain}.lock", "a") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ControlFault("同一网卡/域已有本程序在发 lowcmd") from exc
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("action", choices=("check", "replay", "inspect", "hold", "probe", "scan"))
    p.add_argument("--network", help="DDS 网卡：本地仿真 lo；真机用物理网卡")
    p.add_argument("--domain", type=int, default=0)
    p.add_argument("--handoff", choices=("g1d", "release"), default="g1d",
                   help="默认沿用现场已验证的 G1-D A 键启动；release 额外要求 MotionSwitcher name 为空")
    p.add_argument("--waist-pitch-mode", choices=("locked", "pd"), default="locked",
                   help="默认适配现场锁定 pitch：不发送槽位 14；pd 要求该槽位有有效反馈")
    p.add_argument("--locked-waist-pitch-angle", type=float, default=DEFAULT_LOCKED_WAIST_PITCH,
                   help="现场已确认的固定俯仰角 rad，默认直立零位")
    p.add_argument("--timeout", type=float, default=10.0, help="首次等状态超时 s")
    p.add_argument("--state-json", type=Path, help="离线使用 inspect 导出的实际姿态")
    p.add_argument("--report", type=Path)
    p.add_argument("--log", type=Path)
    p.add_argument("--dry-run", action="store_true", help="连接并规划，不创建 lowcmd writer")
    p.add_argument("--length", type=float, default=.50)
    p.add_argument("--forward", type=float, default=.02)
    p.add_argument("--center-dz", type=float, default=.20)
    p.add_argument("--scan-mode", choices=("fixed", "adaptive"),
                   help="默认 fixed，复用固定 50 cm 扫描轨迹；adaptive 每次按捕获姿态求 IK")
    p.add_argument("--anchor", choices=("zero", "current"),
                   help="adaptive 的扫描线参考点，默认 zero；指定该选项会选择 adaptive")
    p.add_argument("--gravity-compensation", action="store_true",
                   help="固定扫描可选，为左肩俯仰和左肘添加模型重力前馈")
    p.add_argument("--speed", type=float, default=.03,
                   help="扫描路径速度上限 m/s；关节约束较紧的位置会局部减速")
    p.add_argument("--cycles", type=int, default=1)
    p.add_argument("--joint-speed", type=float, default=.25)
    p.add_argument("--joint-acceleration", type=float, default=.5)
    p.add_argument("--state-timeout", type=float, default=.20)
    p.add_argument("--tracking-error", type=float, default=.25)
    p.add_argument("--waist-error", type=float, default=.10)
    p.add_argument("--joint", choices=ARM_JOINTS, default=ARM_JOINTS[0])
    p.add_argument("--amplitude", type=float, default=.03, help="probe 幅度 rad，最大 0.05")
    return p


def build(args, q, limits, action=None):
    action = action or args.action
    if action == "probe":
        return probe_plan(q, args.joint, args.amplitude, limits)
    if action == "hold":
        geometry = Geometry(q)
        Guard(limits).position(q)
        geometry.collision(q)
        return [], geometry, {"action": "hold", "q_rad": q.tolist(),
                              "end_effectors": geometry.arm.end_effectors}
    if args.scan_mode == "fixed":
        return fixed_scan_plan(q, limits, args.speed, args.cycles)
    return scan_plan(q, limits, args.length, args.forward, args.center_dz,
                     args.speed, args.cycles, args.anchor or "zero")


def inspect_report(robot, state, limits):
    issue = None
    try:
        Guard(limits).state(state, time.monotonic())
        Guard(limits).position(state.q)
    except ControlFault as exc:
        issue = str(exc)
    return {"observed_at": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
            "state_layout_compatible": issue is None, "issue": issue,
            "mode_machine": state.mode_machine, "mode_pr": state.mode_pr,
            "tick": state.tick, "present_slots": state.present,
            "joint_names": CONTROLLED_JOINTS, "q_rad": state.q.tolist(),
            "joint_limit_violations": joint_limit_violations(state.q),
            "startup_margin_violations": joint_limit_violations(state.q, limits.margin),
            "dq_rad_s": state.dq.tolist(), "motor_errors": state.errors.tolist(),
            "temperature_C": state.temperature.tolist(),
            "frames_crc_valid": robot.cache.good, "frames_rejected": robot.cache.bad,
            "last_rejected_reason": robot.cache.last_error,
            "published_commands": robot.sent,
            "configured_end_effectors": load_assembly(),
            "locked_waist_pitch_rad": limits.locked_waist_pitch,
            "not_commanded_slots": [14] if limits.locked_waist_pitch is not None else [],
            "scope": "槽位/数值检查，不证明关节方向、零位或硬件身份"}


def build_preflight(args, q, limits, report_path, feedback=None):
    """失败也保存同一捕获姿态和各候选原因，供离线复现；此阶段尚无指令 writer。"""
    try:
        segments, geometry, report = build(args, q, limits)
        if limits.gravity_compensation:
            gravity_peak = np.max([motion.peak_gravity_nm for _, motion in segments], axis=0)
            report["gravity_compensation"] = {
                "ramp_s": 2.,
                "planned_peak_abs_nm": {CONTROLLED_JOINTS[i]: float(peak)
                                         for i, peak in zip(GRAVITY_JOINTS, gravity_peak)}}
        return segments, geometry, report
    except (ControlFault, ValueError) as exc:
        report = dict(feedback or {})
        report.update({"status": "planning-rejected", "failure_stage": "planning",
                       "requested_action": args.action, "joint_names": CONTROLLED_JOINTS,
                       "q_rad": np.asarray(q, float).tolist(),
                       "captured_q_rad": np.asarray(q, float).tolist(),
                       "network": args.network, "domain": args.domain,
                       "parameters": {name: getattr(args, name) for name in
                                      ("scan_mode", "gravity_compensation", "length", "forward", "center_dz",
                                       "anchor", "speed", "cycles",
                                       "joint_speed", "joint_acceleration", "joint", "amplitude")},
                       "planning_error": str(exc),
                       "planning_diagnostics": getattr(exc, "details", {}),
                       "published_commands": 0})
        dump(report_path, report)
        print(f"轨迹预检报告: {report_path}；未创建 lowcmd writer、未发送指令", flush=True)
        raise


def replay(segments, geometry, q, limits, log_path):
    from offline_robot import OfflineRobot
    robot = OfflineRobot(q, locked_waist_pitch=limits.locked_waist_pitch)
    robot.refresh(0.0)
    controller = Controller(robot.read(), segments, geometry, limits, 0.0)
    duration = 2.0 + sum(segment.duration for _, segment in segments) + .05
    if isinstance(segments, FixedScanSegments):
        duration += 2 * SETTLE_SECONDS + .05
    logger = CsvLog(log_path)
    phase = None
    try:
        for k in range(int(np.ceil(duration / .002)) + 1):
            now = k * .002
            robot.refresh(now)
            state = robot.read()
            result = controller.tick(state, now)
            robot.publish(result.q, state, result.feedforward)
            logger.write(now, result, state, now)
            if result.phase != phase:
                print(f"  {now:7.3f}s {result.phase}", flush=True)
                phase = result.phase
    finally:
        logger.close()
    return {"fixture": "ideal-following, no DDS network, no physical dynamics",
            "commands_crc_and_idl_roundtrip": robot.sent,
            "virtual_duration_s": duration, "complete": controller.complete,
            "log": str(log_path)}


def run_loop(robot, captured, segments, geometry, limits, log_path):
    period = .002
    signals = {"count": 0}
    def stop_handler(*_):
        signals["count"] += 1
        controller.stop()
    phase = None
    start = time.monotonic()
    controller = Controller(captured, segments, geometry, limits, start)
    logger = CsvLog(log_path, background=True)
    previous_handlers = {s: signal.signal(s, stop_handler) for s in (signal.SIGINT, signal.SIGTERM)}
    deadline = start
    count = 0
    outcome = "operator-exit"
    last_timing = {}
    max_timing = dict.fromkeys(("gap_ms", "read_ms", "control_ms", "publish_ms",
                               "log_ms", "sleep_overshoot_ms"), 0.)
    gaps_over_10_ms = 0
    try:
        while signals["count"] < 2:
            now = time.monotonic()
            gap_ms = (now - controller.previous)*1000
            max_timing["gap_ms"] = max(max_timing["gap_ms"], gap_ms)
            gaps_over_10_ms += gap_ms > 10.
            measured = time.perf_counter()
            state = robot.read()
            read_done = time.perf_counter()
            result = controller.tick(state, now)
            control_done = time.perf_counter()
            if result.feedforward is None:
                robot.publish(result.q, state)
            else:
                robot.publish(result.q, state, result.feedforward)
            publish_done = time.perf_counter()
            logger.write(now - start, result, state, now)
            log_done = time.perf_counter()
            count += 1
            if result.phase != phase:
                print(f"[{now-start:.1f}s] {result.phase}", flush=True)
                phase = result.phase
                if phase in ("finished-hold", "stopped-hold"):
                    print("持续发送保持目标；Ctrl+C 停轨迹，再次 Ctrl+C 退出发送。", flush=True)
            deadline += period
            slack = deadline - time.monotonic()
            overshoot_ms = 0.
            if slack > 0:
                sleep_started = time.perf_counter()
                time.sleep(slack)
                overshoot_ms = max(0., (time.perf_counter()-sleep_started-slack)*1000)
            else:
                deadline = time.monotonic()
            last_timing = {"read_ms": (read_done-measured)*1000,
                           "control_ms": (control_done-read_done)*1000,
                           "publish_ms": (publish_done-control_done)*1000,
                           "log_ms": (log_done-publish_done)*1000,
                           "sleep_overshoot_ms": overshoot_ms}
            for name, value in last_timing.items():
                max_timing[name] = max(max_timing[name], value)
    except BaseException as exc:
        outcome = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        logger.close()
        for s, handler in previous_handlers.items():
            signal.signal(s, handler)
        elapsed = time.monotonic() - start
        dump(log_path.with_suffix(".status.json"),
             {"outcome": outcome, "commands": count, "elapsed_s": elapsed,
              "average_rate_hz": count/max(elapsed, 1e-9), "last_phase": phase,
              "loop_timing": {"max": max_timing, "last_successful_iteration": last_timing,
                              "gaps_over_10_ms": gaps_over_10_ms},
              "csv_dropped_rows": logger.dropped, "csv_error": logger.error,
              "last_target_rad": controller.last_q.tolist(),
              "scope": "发送停止；硬件最后目标和超时行为由固件决定"})
        print(f"结束发送，共 {count} 帧，平均 {count/max(elapsed,1e-9):.1f} Hz；记录: {log_path}")


def main(argv=None):
    p = parser()
    a = p.parse_args(argv)
    custom_geometry = a.anchor is not None or (a.length, a.forward, a.center_dz) != (.50, .02, .20)
    if a.scan_mode == "fixed" and custom_geometry:
        p.error("固定扫描沿用标准 50 cm 路径；自定义扫描线时请用 --scan-mode adaptive")
    if a.scan_mode is None:
        a.scan_mode = "adaptive" if custom_geometry else "fixed"
    if a.gravity_compensation and (a.scan_mode != "fixed" or a.action not in ("scan", "check", "replay")):
        p.error("--gravity-compensation 仅用于固定扫描及其 check/replay")
    numeric = [a.length, a.forward, a.center_dz, a.speed, a.joint_speed,
               a.joint_acceleration, a.state_timeout, a.tracking_error, a.waist_error,
               a.timeout, a.amplitude, a.locked_waist_pitch_angle]
    if (not np.isfinite(numeric).all() or a.length <= 0 or a.speed <= 0 or a.cycles < 1
            or not 0 < a.joint_speed <= .5 or not 0 < a.joint_acceleration <= 1
            or not .02 <= a.state_timeout <= .5 or not 0 < a.timeout
            or not 0 < a.tracking_error <= .5 or not 0 < a.waist_error <= .15):
        p.error("参数不合法：角速度最高 0.5 rad/s、加速度最高 1 rad/s²，反馈超时 0.02–0.5 s")
    limits = Limits(state_timeout=a.state_timeout, joint_speed=a.joint_speed,
                    joint_acceleration=a.joint_acceleration,
                    following_error=a.tracking_error, torso_error=a.waist_error,
                    gravity_compensation=a.gravity_compensation,
                    locked_waist_pitch=(a.locked_waist_pitch_angle if a.waist_pitch_mode == "locked" else None))
    report_path = a.report or ROOT / "out" / f"{a.action}_{timestamp()}.json"
    if a.action in ("check", "replay"):
        q = np.array(initial_pose(), float)
        if limits.locked_waist_pitch is not None:
            q[WAIST_PITCH_INDEX] = limits.locked_waist_pitch
        if a.state_json:
            saved = json.loads(a.state_json.read_text())
            if saved["joint_names"] != CONTROLLED_JOINTS:
                raise ControlFault("state-json 关节顺序不匹配")
            q = np.array(saved["q_rad"], float)
        segments, geometry, report = build_preflight(a, q, limits, report_path)
        if a.action == "replay":
            report["replay"] = replay(segments, geometry, q, limits,
                                       a.log or ROOT / "out" / "replay.csv")
        dump(report_path, report)
        print_timing(report)
        print(json.dumps(report, ensure_ascii=False, indent=2, default=float))
        print(f"报告: {report_path}；没有连接 DDS")
        return 0

    if not a.network:
        p.error("DDS 入口必须显式提供 --network 网卡名")
    if a.state_json:
        p.error("DDS 控制使用实时反馈，--state-json 仅限 check/replay")
    robot = None
    try:
        if a.action == "inspect" or a.dry_run:
            robot = DdsRobot(a.network, a.domain, limits.locked_waist_pitch)
            state = robot.wait(a.timeout)
            report = inspect_report(robot, state, limits)
            if a.action != "inspect" and report["state_layout_compatible"]:
                _, _, plan = build_preflight(a, state.q, limits, report_path, report)
                report["plan"] = plan
            dump(report_path, report)
            print_timing(report)
            print(json.dumps(report, ensure_ascii=False, indent=2, default=float))
            print(f"报告: {report_path}；没有创建 lowcmd writer 或发送指令")
            return 0 if report["state_layout_compatible"] else 2

        with writer_lock(a.network, a.domain):
            robot = DdsRobot(a.network, a.domain, limits.locked_waist_pitch)
            captured = robot.wait(a.timeout)
            try:
                guard = Guard(limits)
                guard.state(captured, time.monotonic())
                guard.position(captured.q)
            except ControlFault:
                report = inspect_report(robot, captured, limits)
                report.update({"requested_action": a.action, "failure_stage": "startup-feedback"})
                dump(report_path, report)
                print(f"启动检查报告: {report_path}；未创建 lowcmd writer、未发送指令", flush=True)
                raise
            if a.handoff == "release":
                require_released(a.network)
            captured = wait_start_button(robot, captured, limits, a.network)
            if np.abs(captured.dq).max() > .15:
                raise ControlFault("机器人仍在运动，不能捕获启动姿态")
            feedback = inspect_report(robot, captured, limits)
            segments, geometry, report = build_preflight(a, captured.q, limits, report_path, feedback)
            mode_report = (require_released(a.network) if a.handoff == "release" else
                           {"handoff": "G1-D dedicated SDK example; remote A confirmed" if a.network != "lo"
                            else "local loopback simulator"})
            robot.prepare()  # writer 发现耗时发生在控制循环之前；不发送消息。
            state = robot.read()
            Guard(limits).state(state, time.monotonic())
            if state.mode_machine != captured.mode_machine or np.abs(state.q - captured.q).max() > .03:
                raise ControlFault("规划期间姿态或控制模式改变，需要重新捕获后规划")
            if np.abs(state.dq).max() > .15:
                raise ControlFault("下发前机器人仍在运动")
            kp, kd = KP.copy(), KD.copy()
            if limits.locked_waist_pitch is not None:
                kp[WAIST_PITCH_INDEX] = kd[WAIST_PITCH_INDEX] = 0.
            report.update({"joint_names": CONTROLLED_JOINTS,
                           "captured_q_rad": captured.q.tolist(), "pd_kp": kp.tolist(),
                           "pd_kd": kd.tolist(), "mode_machine": state.mode_machine,
                           "locked_waist_pitch_rad": limits.locked_waist_pitch,
                           "not_commanded_slots": [14] if limits.locked_waist_pitch is not None else [],
                           "motion_control_check": mode_report})
            dump(report_path, report)
            print_timing(report)
            print(json.dumps(report, ensure_ascii=False, indent=2, default=float))
            print("首帧及启动保持使用上述 captured_q_rad。", flush=True)
            if isinstance(segments, FixedScanSegments):
                print("左臂停稳后进入固定扫描段，扫描完成后复核反馈并收臂。", flush=True)
            run_loop(robot, captured, segments, geometry, limits,
                     a.log or ROOT / "out" / f"real_{timestamp()}.csv")
            return 0
    finally:
        if robot is not None:
            robot.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ControlFault, ValueError, OSError) as exc:
        print(f"已停止轨迹与发送: {exc}", file=sys.stderr)
        raise SystemExit(2)

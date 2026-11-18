#!/usr/bin/env python3
"""G1-D MuJoCo DDS 仿真器：订阅 rt/lowcmd，发布真实物理状态 rt/lowstate。"""
import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import signal
import threading
import time

import mujoco
import numpy as np

from dds_network import initialize
from g1d_config import MODEL, ROOT, CONTROLLED_JOINTS, actuator_name, initial_pose, model_assembly
from g1d_transport import Codec, ControlFault, SLOTS


@dataclass(frozen=True)
class Command:
    enabled: np.ndarray
    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    received: float


class PhysicsBridge:
    def __init__(self, model=MODEL, waist_mode="fixed", initial_q=None,
                 mode_machine=1, command_timeout=.2):
        self.model_path = Path(model).resolve()
        self.end_effectors = model_assembly(model)
        self.model = mujoco.MjModel.from_xml_path(str(model))
        self.data = mujoco.MjData(self.model)
        self.scratch = mujoco.MjData(self.model)
        self.codec = Codec()
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
        self.state_factory = unitree_hg_msg_dds__LowState_
        jids = np.array([mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                         for n in CONTROLLED_JOINTS])
        self.aids = np.array([mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                              actuator_name(n)) for n in CONTROLLED_JOINTS])
        if np.any(jids < 0) or np.any(self.aids < 0) or self.model.nu != 16:
            raise ControlFault("仿真场景必须包含 G1-D 的 16 个腰/臂执行器")
        if (np.any(self.model.actuator_trntype[self.aids] != 0)
                or not np.array_equal(self.model.actuator_trnid[self.aids, 0], jids)
                or not np.allclose(self.model.actuator_gear[self.aids], [1, 0, 0, 0, 0, 0])
                or np.any(self.model.actuator_biastype[self.aids] != 0)
                or not np.allclose(self.model.actuator_gainprm[self.aids, 0], 1)):
            raise ControlFault("仿真器需要单位传动比的力矩电机模型")
        self.qadr = self.model.jnt_qposadr[jids]
        self.dadr = self.model.jnt_dofadr[jids]
        self.initial_q = (np.array(initial_pose(), float) if initial_q is None
                          else np.asarray(initial_q, float))
        bounds = self.model.jnt_range[jids]
        if (self.initial_q.shape != (16,) or not np.isfinite(self.initial_q).all()
                or np.any(self.initial_q < bounds[:, 0]) or np.any(self.initial_q > bounds[:, 1])):
            raise ControlFault("初始姿态不符合 G1-D 模型限位")
        self.data.qpos[self.qadr] = self.initial_q
        for name in ("Yaw_Joint", "torso_Joint"):
            eq = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, f"hold_{name}")
            if eq < 0:
                raise ControlFault("模型缺少腰部台架约束，请重建场景")
            self.data.eq_active[eq] = waist_mode == "fixed"
            # 非零初始姿态时，台架约束也必须保持在相同角度。
            index = CONTROLLED_JOINTS.index(name)
            self.model.eq_data[eq, 0] = self.initial_q[index]
        mujoco.mj_forward(self.model, self.data)
        self.mode_machine = mode_machine
        self.waist_mode = waist_mode
        self.command_timeout = command_timeout
        self.lock = threading.Lock()
        self.command = None
        self.commands_accepted = 0
        self.commands_rejected = 0
        self.last_rejection = ""
        self.steps = 0
        self.active_steps = 0
        self.timed_out = False
        self.max_joint_error = 0.0
        self.max_tip_error = 0.0
        self.tip_error_sum = 0.0
        self.tip_error_count = 0
        self.tip = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_wrist_yaw_link")

    def accept(self, msg, now=None):
        try:
            if int(msg.crc) != self.codec.crc.Crc(msg):
                raise ControlFault("LowCmd CRC 错误")
            if int(msg.mode_pr) != 0 or int(msg.mode_machine) != self.mode_machine:
                raise ControlFault("LowCmd 控制模式或 mode_machine 不匹配")
            motors = [msg.motor_cmd[int(s)] for s in SLOTS]
            arrays = [np.array([getattr(m, field) for m in motors], float)
                      for field in ("q", "dq", "tau", "kp", "kd")]
            if (not np.isfinite(np.concatenate(arrays)).all()
                    or np.any(arrays[3] < 0) or np.any(arrays[4] < 0)
                    or any(int(m.mode) not in (0, 1) for m in motors)):
                raise ControlFault("LowCmd 包含非法控制参数")
            command = Command(np.array([int(m.mode) == 1 for m in motors]), *arrays,
                              time.monotonic() if now is None else now)
            with self.lock:
                self.command = command
                self.commands_accepted += 1
            return True
        except Exception as exc:
            with self.lock:
                self.commands_rejected += 1
                self.last_rejection = str(exc)
            return False

    def step(self, now):
        with self.lock:
            command = self.command
        self.steps += 1
        if command is None:
            # 等待控制器时暂停动力学，持续发布初始姿态；接指令后才积分物理。
            # 这明确模拟接管前的保持状态，不把未控自由下垂姿态当作 seed。
            self.data.time += self.model.opt.timestep
            mujoco.mj_forward(self.model, self.data)
            return
        self.timed_out = now - command.received > self.command_timeout
        tau = (command.tau + command.kp * (command.q - self.data.qpos[self.qadr])
               + command.kd * (command.dq - self.data.qvel[self.dadr]))
        lo = np.where(self.model.actuator_ctrllimited[self.aids],
                      self.model.actuator_ctrlrange[self.aids, 0], -np.inf)
        hi = np.where(self.model.actuator_ctrllimited[self.aids],
                      self.model.actuator_ctrlrange[self.aids, 1], np.inf)
        self.data.ctrl[self.aids] = np.where(command.enabled, np.clip(tau, lo, hi), 0.)
        mujoco.mj_step(self.model, self.data)
        self.active_steps += 1
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise ControlFault("MuJoCo 物理状态出现非有限数值")

    def state(self):
        mujoco.mj_forward(self.model, self.data)
        with self.lock:
            command = self.command
        state = self.state_factory()
        state.mode_pr = 0
        state.mode_machine = self.mode_machine
        state.tick = int(round(self.steps * self.model.opt.timestep * 1000)) % 2**32
        for k, slot in enumerate(SLOTS):
            motor = state.motor_state[int(slot)]
            motor.mode = 1 if command is not None and command.enabled[k] else 0
            motor.q = float(self.data.qpos[self.qadr[k]])
            motor.dq = float(self.data.qvel[self.dadr[k]])
            motor.ddq = float(self.data.qacc[self.dadr[k]])
            motor.tau_est = float(self.data.qfrc_actuator[self.dadr[k]])
            # 占位的模拟电压/温度用于标识实际存在的槽位，非电热模型。
            motor.vol, motor.temperature = 48.0, [30, 30]
        if command is not None:
            self.max_joint_error = max(self.max_joint_error,
                                       float(np.abs(command.q - self.data.qpos[self.qadr]).max()))
            self.scratch.qpos[:] = self.data.qpos
            self.scratch.qpos[self.qadr] = np.where(command.enabled, command.q,
                                                   self.data.qpos[self.qadr])
            mujoco.mj_kinematics(self.model, self.scratch)
            error = float(np.linalg.norm(self.data.xpos[self.tip] - self.scratch.xpos[self.tip]))
            self.max_tip_error = max(self.max_tip_error, error)
            self.tip_error_sum += error**2
            self.tip_error_count += 1
        state.crc = self.codec.crc.Crc(state)
        return state

    def report(self):
        return {"model": str(self.model_path), "end_effectors": self.end_effectors,
                "waist_mode": self.waist_mode,
                "initial_q_rad": self.initial_q.tolist(), "physics_steps": self.active_steps,
                "steps_including_idle": self.steps, "commands_accepted": self.commands_accepted,
                "commands_rejected": self.commands_rejected, "last_rejection": self.last_rejection,
                "command_timed_out": self.timed_out,
                "timeout_policy": "hold-last-target (simulation only; not a firmware claim)",
                "max_joint_tracking_error_rad": self.max_joint_error,
                "max_tip_tracking_error_mm": self.max_tip_error * 1000,
                "rms_tip_tracking_error_mm": 1000*np.sqrt(self.tip_error_sum/max(1, self.tip_error_count)),
                "final_q_rad": self.data.qpos[self.qadr].tolist()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", default="lo", help="本机仿真默认 lo")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--waist-mode", choices=("fixed", "pd"), default="fixed")
    parser.add_argument("--init", choices=("hang", "zero"), default="hang",
                        help="仿真初始姿态，默认垂臂；zero 仅用于对照")
    parser.add_argument("--pose-json", type=Path, help="inspect 导出的初始姿态")
    parser.add_argument("--duration", type=float, default=0., help="仿真器墙钟运行秒数，0=持续")
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "dds_sim.json")
    args = parser.parse_args(argv)
    if not np.isfinite(args.duration) or args.duration < 0:
        parser.error("duration 必须为非负有限数")
    initial_q = initial_pose(args.init)
    if args.pose_json:
        pose = json.loads(args.pose_json.read_text())
        if pose["joint_names"] != CONTROLLED_JOINTS:
            parser.error("初始姿态关节顺序不匹配")
        initial_q = pose["q_rad"]
    core = PhysicsBridge(waist_mode=args.waist_mode, initial_q=initial_q)
    initialize(args.network, args.domain)
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    publisher = ChannelPublisher("rt/lowstate", LowState_)
    publisher.Init()
    subscriber = ChannelSubscriber("rt/lowcmd", LowCmd_)
    subscriber.Init(core.accept, 1)
    stop = {"value": False}
    def stop_handler(*_):
        stop["value"] = True
    previous = {s: signal.signal(s, stop_handler) for s in (signal.SIGINT, signal.SIGTERM)}
    viewer = None
    if not args.no_viewer:
        import importlib
        viewer = importlib.import_module("mujoco.viewer").launch_passive(core.model, core.data)
        viewer.cam.lookat[:] = [.15, 0., .65]
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 2.4, 130, -8
    start = time.monotonic()
    physics_due = publish_due = viewer_due = start
    published = 0
    outcome = "stopped"
    print(f"DDS_SIM_READY network={args.network} domain={args.domain} waist={args.waist_mode}", flush=True)
    print(f"模型末端装配: {core.end_effectors}", flush=True)
    print(f"仿真初始姿态: {'pose-json' if args.pose_json else args.init}；等待捕获当前姿态后接近工作点", flush=True)
    try:
        while not stop["value"] and (viewer is None or viewer.is_running()):
            now = time.monotonic()
            if args.duration and now - start >= args.duration:
                break
            if now - physics_due > .25:
                raise ControlFault("仿真物理循环落后墙钟超过 0.25 s")
            count = 0
            while now >= physics_due and count < 20:
                core.step(now)
                physics_due += core.model.opt.timestep
                count += 1
            if now >= publish_due:
                if not publisher.Write(core.state()):
                    raise ControlFault("仿真 LowState 发送失败")
                published += 1
                publish_due = now + .002
            if viewer is not None and now >= viewer_due:
                viewer.sync()
                viewer_due = now + .02
            delay = min(physics_due, publish_due) - time.monotonic()
            if delay > 0:
                time.sleep(min(delay, .002))
    except BaseException as exc:
        outcome = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if viewer is not None:
            viewer.close()
        subscriber.Close()
        publisher.Close()
        for s, handler in previous.items():
            signal.signal(s, handler)
        result = core.report()
        result.update({"outcome": outcome, "network": args.network, "domain": args.domain,
                       "published_states": published, "wall_seconds": time.monotonic() - start})
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
        print(json.dumps(result, indent=2, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    try:
        main()
    except (ControlFault, ValueError, OSError) as exc:
        raise SystemExit(f"仿真器停止: {exc}")

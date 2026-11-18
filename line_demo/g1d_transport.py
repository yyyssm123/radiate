"""G1-D HG DDS 编解码与状态缓存。构造 Codec 不创建任何网络通道。"""
from dataclasses import dataclass
import threading
import time

import numpy as np

from g1d_config import CONTROLLED_JOINTS, JOINT_SLOTS, pd_for_joint, WAIST_PITCH_INDEX
from dds_network import initialize, validate_network

SLOTS = np.array([JOINT_SLOTS[n] for n in CONTROLLED_JOINTS])
LEFT = np.array([CONTROLLED_JOINTS.index(n) for n in CONTROLLED_JOINTS if n.startswith("left_")])
KP = np.array([pd_for_joint(n)[0] for n in CONTROLLED_JOINTS])
KD = np.array([pd_for_joint(n)[1] for n in CONTROLLED_JOINTS])


class ControlFault(RuntimeError):
    pass


@dataclass(frozen=True)
class Snapshot:
    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray
    errors: np.ndarray
    temperature: np.ndarray
    present: tuple
    mode_machine: int
    mode_pr: int
    tick: int
    buttons: int
    received: float
    progressed: float


class Codec:
    def __init__(self, locked_waist_pitch=None):
        try:
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
            from unitree_sdk2py.utils.crc import CRC
        except ImportError as exc:
            raise ControlFault("缺少 unitree_sdk2py；见 REAL_ROBOT.md 的安装命令") from exc
        self.default_command = unitree_hg_msg_dds__LowCmd_
        self.crc = CRC()
        if locked_waist_pitch is not None and not np.isfinite(locked_waist_pitch):
            raise ControlFault("锁定的腰部俯仰角必须是有限数值")
        self.locked_waist_pitch = locked_waist_pitch

    def decode(self, msg, now, previous=None):
        if len(msg.motor_state) < 29 or int(msg.crc) != self.crc.Crc(msg):
            raise ControlFault("LowState 长度或 CRC 错误")
        states = [msg.motor_state[i] for i in SLOTS]
        q = np.array([s.q for s in states], float)
        dq = np.array([s.dq for s in states], float)
        tau = np.array([s.tau_est for s in states], float)
        if not np.isfinite(np.concatenate((q, dq, tau))).all():
            raise ControlFault("LowState 包含非有限数值")
        # 零位静止不代表电机缺失；mode/故障字也是有效槽位信号。
        # 若连状态字段都全零，则保持“不确定”，交给 Guard 拒绝发送并要求只读核对。
        present = tuple(i for i, s in enumerate(msg.motor_state[:29])
                        if int(s.mode) != 0 or int(s.motorstate) != 0
                        or abs(s.q) > 1e-5 or abs(s.dq) > 1e-5
                        or abs(s.tau_est) > 1e-5 or s.vol > 1
                        or max(s.temperature) > 1)
        if self.locked_waist_pitch is not None and 14 not in present:
            # 固定模型角来自现场确认，不伪造此槽位存在，不把它加入 present。
            q[WAIST_PITCH_INDEX] = self.locked_waist_pitch
        if previous is not None:
            tick_delta = (int(msg.tick) - previous.tick) % 2**32
            if tick_delta >= 2**31:
                raise ControlFault("LowState tick 倒退或旧包乱序")
        progressed = (now if previous is None or int(msg.tick) != previous.tick
                      else previous.progressed)
        return Snapshot(q=q, dq=dq, tau=tau,
                        errors=np.array([s.motorstate for s in states], np.uint32),
                        temperature=np.array([max(s.temperature) for s in states]),
                        present=present, mode_machine=int(msg.mode_machine),
                        mode_pr=int(msg.mode_pr), tick=int(msg.tick),
                        buttons=int(msg.wireless_remote[2]) | int(msg.wireless_remote[3]) << 8,
                        received=now, progressed=progressed)

    def encode(self, q, state):
        q = np.asarray(q, float)
        if q.shape != (len(SLOTS),) or not np.isfinite(q).all():
            raise ControlFault("指令必须是 16 个有限关节角")
        if (self.locked_waist_pitch is not None
                and abs(q[WAIST_PITCH_INDEX]-self.locked_waist_pitch) > .001):
            raise ControlFault("目标腰部俯仰角与已确认的锁定角不一致")
        cmd = self.default_command()
        cmd.mode_pr = 0
        cmd.mode_machine = state.mode_machine
        for k, slot in enumerate(SLOTS):
            if slot == 14 and self.locked_waist_pitch is not None:
                continue  # 槽位 14 保持 mode=kp=kd=tau=0，不接管锁定的俯仰关节。
            motor = cmd.motor_cmd[int(slot)]
            motor.mode = 1
            motor.q = float(q[k])
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = float(KP[k])
            motor.kd = float(KD[k])
        # 无效槽位 0–11/13 和 29–34 保持默认 mode=0、kp=kd=tau=0。
        cmd.crc = self.crc.Crc(cmd)
        return cmd


class StateCache:
    def __init__(self, codec):
        self.codec = codec
        self.lock = threading.Lock()
        self.state = None
        self.good = 0
        self.bad = 0
        self.last_error = ""

    def update(self, msg, now=None):
        with self.lock:
            try:
                self.state = self.codec.decode(msg, time.monotonic() if now is None else now,
                                               self.state)
                self.good += 1
            except Exception as exc:
                self.bad += 1
                self.last_error = str(exc)

    def read(self):
        with self.lock:
            return self.state


class DdsRobot:
    """inspect 阶段只有 subscriber；publish 第一次调用时才创建 lowcmd writer。"""
    def __init__(self, network, domain=0, locked_waist_pitch=None):
        validate_network(network, domain)
        self.network, self.domain = network, domain
        self.codec = Codec(locked_waist_pitch)
        self.cache = StateCache(self.codec)
        from unitree_sdk2py.core.channel import (
            ChannelPublisher, ChannelSubscriber)
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        self._publisher_class, self._command_type = ChannelPublisher, LowCmd_
        self.publisher = None
        initialize(network, domain)
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.cache.update, 1)
        self.sent = 0

    def read(self):
        return self.cache.read()

    def wait(self, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = self.read()
            if state is not None and self.cache.good >= 10 and state.progressed > state.received - 0.1:
                return state
            time.sleep(0.01)
        hint = (f"\n本机仿真需先在另一个终端运行并保持窗口打开：\n"
                f"python3 line_demo/mujoco_sim.py --network lo --domain {self.domain}"
                if self.network == "lo" else "\n请检查设备连接、网卡和 DDS 域是否一致。")
        raise ControlFault(f"{timeout:g}s 内没有连续有效 LowState；有效包={self.cache.good}，"
                           f"坏包={self.cache.bad}，最近错误={self.cache.last_error or '无'}" + hint)

    def prepare(self):
        if self.publisher is None:
            self.publisher = self._publisher_class("rt/lowcmd", self._command_type)
            self.publisher.Init()
            # writer 初始化可能耗时；调用方会先用 seed 保持，再进行任何运动。

    def publish(self, q, state):
        self.prepare()
        if not self.publisher.Write(self.codec.encode(q, state)):
            raise ControlFault("LowCmd DDS 写入失败")
        self.sent += 1

    def close(self):
        if self.publisher is not None:
            self.publisher.Close()
        self.subscriber.Close()

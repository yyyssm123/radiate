"""可由真实 DDS 或离线夹具驱动的控制循环。"""
import csv
from dataclasses import dataclass

import numpy as np

from g1d_config import CONTROLLED_JOINTS
from g1d_transport import ControlFault, LEFT
from real_motion import Guard


@dataclass
class Tick:
    phase: str
    q: np.ndarray
    pd_torque: np.ndarray
    complete: bool


class Controller:
    def __init__(self, captured, segments, geometry, limits, start, period=.002,
                 startup_hold=2.0):
        # 控制入口只接受以实测角为首尾的连续轨迹，拒绝误传一个零位起点。
        previous = captured.q[LEFT]
        for name, segment in segments:
            if not np.allclose(segment.sample(0.), previous, atol=1e-7, rtol=0):
                raise ControlFault(f"轨迹起点不连续: {name}；必须从捕获姿态平滑接入")
            previous = segment.sample(segment.duration)
        if segments and not np.allclose(previous, captured.q[LEFT], atol=1e-7, rtol=0):
            raise ControlFault("轨迹末段没有返回捕获姿态")
        for (_, first), (_, second) in zip(segments, segments[1:]):
            if not np.allclose(first.sample(first.duration), second.sample(0), atol=1e-7, rtol=0):
                raise ControlFault("轨迹分段之间存在关节角跳变")
        self.captured = captured.q.copy()
        self.segments = segments
        self.geometry = geometry
        self.guard = Guard(limits)
        self.guard.mode_machine = captured.mode_machine
        self.guard.last_command = self.captured.copy()
        self.guard.last_velocity = np.zeros_like(self.captured)
        self.start = start + startup_hold
        self.previous = start - period
        self.period = period
        self.last_q = self.captured.copy()
        self.last_buttons = captured.buttons
        self.stopped = False
        self.last_collision = -np.inf
        self.complete = False
        self.sent = 0

    def stop(self):
        self.stopped = True
        self.guard.last_velocity = np.zeros_like(self.last_q)

    def tick(self, state, now):
        if state is None:
            raise ControlFault("没有 LowState")
        dt = now - self.previous
        if dt <= 0 or dt > .05:
            raise ControlFault(f"控制循环调度间隔异常: {dt:.4f} s")
        # SDK 遥控器的 A/B 新按下时停止轨迹，保持最后目标。
        pressed = state.buttons & ~self.last_buttons
        self.last_buttons = state.buttons
        if pressed & ((1 << 8) | (1 << 9)):
            self.stop()
        q = self.captured.copy()
        elapsed = now - self.start
        phase = "startup-hold"
        if self.stopped:
            q = self.last_q.copy()
            phase = "stopped-hold"
        elif elapsed >= 0:
            phase = "finished-hold"
            remaining = elapsed
            self.complete = True
            for name, segment in self.segments:
                if remaining < segment.duration:
                    phase = name
                    q[LEFT] = segment.sample(remaining)
                    self.complete = False
                    break
                remaining -= segment.duration
            # 所有扫描/单关节动作的末段都回到 captured。
        holding = phase.endswith("hold")
        torque = self.guard.command(q, state, now, dt, holding=holding)
        if now - self.last_collision >= .02:
            self.geometry.collision(state.q)
            self.last_collision = now
        self.last_q = q.copy()
        self.previous = now
        return Tick(phase, q, torque, self.complete)


class CsvLog:
    def __init__(self, path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("w", newline="", buffering=1)
        self.writer = csv.writer(self.file)
        header = ["elapsed_s", "phase", "state_age_s", "state_tick"]
        for field in ("q_cmd_rad", "q_rad", "dq_rad_s", "tau_est_Nm", "pd_est_Nm"):
            header.extend(f"{name}.{field}" for name in CONTROLLED_JOINTS)
        self.writer.writerow(header)
        self.last = -np.inf

    def write(self, elapsed, tick, state, now):
        if now - self.last < .02:
            return
        self.writer.writerow([elapsed, tick.phase, now - state.received, state.tick,
                              *tick.q, *state.q, *state.dq, *state.tau, *tick.pd_torque])
        self.last = now

    def close(self):
        self.file.close()

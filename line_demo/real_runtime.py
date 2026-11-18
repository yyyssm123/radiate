"""可由真实 DDS 或离线夹具驱动的控制循环。"""
import csv
from dataclasses import dataclass
from queue import Empty, Full, Queue
from threading import Event, Thread

import numpy as np

from g1d_config import CONTROLLED_JOINTS, HANG_LEFT
from g1d_transport import ControlFault, LEFT
from fixed_scan import FixedScanSegments
from real_motion import Guard


SETTLE_POSITION_RAD = .06
SETTLE_SPEED_RAD_S = .04
SETTLE_SECONDS = .25
SETTLE_TIMEOUT = 4.
RIGHT = np.array([i for i, name in enumerate(CONTROLLED_JOINTS) if name.startswith("right_")])


@dataclass
class Tick:
    phase: str
    q: np.ndarray
    pd_torque: np.ndarray
    complete: bool
    feedforward: np.ndarray | None = None


class Controller:
    def __init__(self, captured, segments, geometry, limits, start, period=.002,
                 startup_hold=2.0):
        # 所有轨迹从实测角开始，每一段都要与上一段连续。
        self.fixed = isinstance(segments, FixedScanSegments)
        previous = captured.q[LEFT]
        for name, segment in segments:
            if not np.allclose(segment.sample(0.), previous, atol=1e-7, rtol=0):
                raise ControlFault(f"轨迹起点不连续: {name}；必须从捕获姿态平滑接入")
            previous = segment.sample(segment.duration)
        destination = HANG_LEFT if self.fixed else captured.q[LEFT]
        if not np.allclose(previous, destination, atol=1e-7, rtol=0):
            raise ControlFault("轨迹末段没有到达预定的保持姿态")
        self.finish_left = previous.copy()
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
        self.segment_index = 0
        self.segment_start = self.start
        self.settling_started = self.stable_since = self.stable_tick = None
        self.feedforward_blend = 0.

    def stop(self):
        self.stopped = True
        self.guard.last_velocity = np.zeros_like(self.last_q)

    def fixed_target(self, state, now):
        name, segment = self.segments[self.segment_index]
        elapsed = now - self.segment_start
        if elapsed < segment.duration:
            return name, segment.sample(elapsed)
        if self.segment_index + 1 == len(self.segments):
            self.complete = True
            return "finished-hold", self.finish_left

        next_name, next_segment = self.segments[self.segment_index + 1]
        target = segment.sample(segment.duration)
        if self.settling_started is None:
            self.settling_started = now
        error = np.abs(state.q[LEFT] - target)
        speed = np.abs(state.dq).max()
        waist_error = np.abs(state.q[:2] - self.captured[:2]).max()
        right_error = np.abs(state.q[RIGHT] - self.captured[RIGHT]).max()
        if (error.max() <= SETTLE_POSITION_RAD and speed <= SETTLE_SPEED_RAD_S
                and waist_error <= .02 and right_error <= .12):
            if self.stable_since is None:
                self.stable_since, self.stable_tick = now, state.tick
            elif now-self.stable_since >= SETTLE_SECONDS and state.tick != self.stable_tick:
                self.segment_index += 1
                self.segment_start = now
                self.settling_started = self.stable_since = self.stable_tick = None
                self.last_collision = -np.inf  # 进入下一段的首帧也复核最新整机模型碰撞。
                return next_name, next_segment.sample(0.)
        else:
            self.stable_since = self.stable_tick = None
        if now-self.settling_started >= SETTLE_TIMEOUT:
            joint = CONTROLLED_JOINTS[LEFT[int(error.argmax())]]
            raise ControlFault(f"{next_name} 前未停稳，未进入该段。{joint} 误差 {error.max():.3f} rad，"
                               f"最大实测速度 {speed:.3f} rad/s，腰/右臂偏差 "
                               f"{waist_error:.3f}/{right_error:.3f} rad")
        return f"{next_name}-settle-hold", target

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
        elif self.fixed and elapsed >= 0:
            phase, q[LEFT] = self.fixed_target(state, now)
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
        feedforward = None
        if self.guard.limits.gravity_compensation:
            if not self.stopped:
                u = float(np.clip(elapsed / 2., 0., 1.))
                self.feedforward_blend = u**3 * (10 + u*(-15 + 6*u))
            # 停止轨迹时保持上一目标的补偿比例，避免力矩突变。
            feedforward = self.feedforward_blend * self.geometry.gravity_feedforward(q)
        torque = self.guard.command(q, state, now, dt, holding=holding, feedforward=feedforward)
        if now - self.last_collision >= .02:
            self.geometry.collision(state.q)
            if self.fixed and phase in ("approach", "scan", "return",
                                        "scan-settle-hold", "return-settle-hold"):
                # 用最新腰姿和右臂反馈复核当前一帧的左臂目标。
                projected = state.q.copy()
                projected[LEFT] = q[LEFT]
                self.geometry.collision(projected)
            self.last_collision = now
        self.last_q = q.copy()
        self.previous = now
        return Tick(phase, q, torque, self.complete, feedforward)


class CsvLog:
    def __init__(self, path, background=False):
        path.parent.mkdir(parents=True, exist_ok=True)
        # 按块写 CSV，避免控制线程每条样本都向文件系统刷新。
        self.file = path.open("w", newline="", buffering=64*1024)
        self.writer = csv.writer(self.file)
        header = ["elapsed_s", "phase", "state_age_s", "state_tick"]
        for field in ("q_cmd_rad", "q_rad", "dq_rad_s", "tau_est_Nm", "pd_est_Nm", "tau_ff_Nm"):
            header.extend(f"{name}.{field}" for name in CONTROLLED_JOINTS)
        self.writer.writerow(header)
        self.last = -np.inf
        self.dropped = 0
        self.error = None
        self.queue = Queue(maxsize=500) if background else None
        if background:
            self.closing = Event()
            self.worker = Thread(target=self.write_pending, name="g1d-csv")
            self.worker.start()

    def write_pending(self):
        try:
            while not self.closing.is_set() or not self.queue.empty():
                try:
                    row = self.queue.get(timeout=.05)
                except Empty:
                    continue
                self.writer.writerow(row)
        except OSError as exc:
            self.error = str(exc)

    def write(self, elapsed, tick, state, now):
        if now - self.last < .02:
            return
        ff = tick.feedforward if tick.feedforward is not None else np.zeros_like(tick.q)
        row = [elapsed, tick.phase, now - state.received, state.tick,
               *tick.q, *state.q, *state.dq, *state.tau, *tick.pd_torque, *ff]
        if self.queue is None:
            self.writer.writerow(row)
        else:
            try:
                self.queue.put_nowait(row)
            except Full:
                self.dropped += 1
        self.last = now

    def close(self):
        if self.queue is not None:
            self.closing.set()
            self.worker.join()
        try:
            self.file.close()
        except OSError as exc:
            self.error = str(exc)

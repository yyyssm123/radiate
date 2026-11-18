"""理想跟随的离线协议夹具；不创建 DDS 通道，不模拟真实物理。"""
import numpy as np

from g1d_transport import Codec, SLOTS, StateCache, ControlFault
from g1d_config import WAIST_PITCH_INDEX


class OfflineRobot:
    def __init__(self, q=None, mode_machine=1, locked_waist_pitch=None):
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
        self.codec = Codec(locked_waist_pitch)
        self.cache = StateCache(self.codec)
        self.message = unitree_hg_msg_dds__LowState_()
        self.message.mode_machine = mode_machine
        self.q = np.zeros(16) if q is None else np.array(q, float)
        if locked_waist_pitch is not None:
            self.q[WAIST_PITCH_INDEX] = locked_waist_pitch
        self.sent = 0
        self.last_command = None

    def refresh(self, now):
        self.message.tick = (self.message.tick + 2) % 2**32
        for k, slot in enumerate(SLOTS):
            motor = self.message.motor_state[int(slot)]
            motor.q = float(self.q[k])
            motor.dq = 0.0
            motor.vol = 48.0
            motor.temperature = [30, 30]
        self.message.crc = self.codec.crc.Crc(self.message)
        self.cache.update(self.message, now)

    def read(self):
        return self.cache.read()

    def publish(self, q, state):
        command = self.codec.encode(q, state)
        command = type(command).deserialize(command.serialize())
        if command.crc != self.codec.crc.Crc(command):
            raise ControlFault("离线 IDL 往返的 LowCmd CRC 错误")
        self.q = np.array([command.motor_cmd[int(s)].q for s in SLOTS])
        if self.codec.locked_waist_pitch is not None:
            self.q[WAIST_PITCH_INDEX] = self.codec.locked_waist_pitch
        self.last_command = command
        self.sent += 1

    def close(self):
        pass

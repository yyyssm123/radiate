"""使用用户提供的 G1-D 原始反馈回归锁定俯仰，不启动 DDS。"""
import json
from pathlib import Path
import unittest

import numpy as np

from g1d_config import WAIST_PITCH_INDEX
from g1d_transport import Codec, ControlFault, KP, KD, SLOTS
from offline_robot import OfflineRobot
from real_motion import Guard, Limits, scan_plan


def measured_message():
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
    sample = json.loads((Path(__file__).resolve().parents[1] /
                         "references/g1d_locked_pitch_lowstate_sample.json").read_text())
    msg = unitree_hg_msg_dds__LowState_()
    msg.mode_pr, msg.mode_machine, msg.tick = sample["mode_pr"], sample["mode_machine"], sample["tick"]
    for row in sample["slots"]:
        for name, value in row.items():
            if name != "slot":
                setattr(msg.motor_state[row["slot"]], name, value)
    msg.crc = Codec().crc.Crc(msg)
    return msg


class TestLockedWaistPitch(unittest.TestCase):
    def test_measured_start_pose_has_a_complete_valid_scan_branch(self):
        state = Codec(0.).decode(measured_message(), 10.)
        segments, _, report = scan_plan(state.q, Limits(locked_waist_pitch=0.), speed=.1)
        self.assertGreater(report["collision_branches_rejected"], 0)
        self.assertLess(report["scan_s"], 30.)
        from g1d_transport import LEFT
        np.testing.assert_allclose(segments[0][1].sample(0.), state.q[LEFT], atol=1e-7)
        np.testing.assert_allclose(segments[-1][1].sample(segments[-1][1].duration), state.q[LEFT], atol=1e-7)

    def test_actual_zero_feedback_requires_explicit_locked_profile(self):
        msg = measured_message()
        state = Codec().decode(msg, 10.)
        self.assertNotIn(14, state.present)
        with self.assertRaisesRegex(ControlFault, "14"):
            Guard().state(state, 10.)
        locked = Codec(0.).decode(msg, 10.)
        Guard(Limits(locked_waist_pitch=0.)).state(locked, 10.)
        self.assertNotIn(14, locked.present)  # 不伪造其反馈存在。

    def test_actual_start_pose_and_sdk_commands_keep_pitch_disabled(self):
        codec = Codec(0.)
        state = codec.decode(measured_message(), 10.)
        cmd = codec.encode(state.q, state)
        cmd = type(cmd).deserialize(cmd.serialize())
        self.assertEqual(cmd.crc, codec.crc.Crc(cmd))
        for k, slot in enumerate(SLOTS):
            m = cmd.motor_cmd[int(slot)]
            if slot == 14:
                self.assertEqual((m.mode, m.q, m.kp, m.kd, m.tau, m.dq), (0, 0, 0, 0, 0, 0))
            else:
                self.assertEqual((m.mode, m.kp, m.kd), (1, KP[k], KD[k]))
                self.assertAlmostEqual(m.q, state.q[k], places=6)

    def test_only_locked_slot_can_be_silent(self):
        msg = measured_message()
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__MotorState_
        msg.motor_state[15] = unitree_hg_msg_dds__MotorState_()
        msg.crc = Codec().crc.Crc(msg)
        with self.assertRaisesRegex(ControlFault, "15"):
            Guard(Limits(locked_waist_pitch=0.)).state(Codec(0.).decode(msg, 10.), 10.)

    def test_inconsistent_feedback_or_target_cannot_move_locked_pitch(self):
        msg = measured_message()
        msg.motor_state[14].q = .2
        msg.crc = Codec().crc.Crc(msg)
        with self.assertRaisesRegex(ControlFault, "锁定角"):
            Guard(Limits(locked_waist_pitch=0.)).state(Codec(0.).decode(msg, 10.), 10.)
        state = Codec(0.).decode(measured_message(), 10.)
        target = state.q.copy()
        target[WAIST_PITCH_INDEX] = .05
        with self.assertRaisesRegex(ControlFault, "锁定角"):
            Codec(0.).encode(target, state)

    def test_nonzero_confirmed_angle_is_virtual_and_fixture_keeps_it(self):
        state = Codec(.1).decode(measured_message(), 10.)
        self.assertEqual(state.q[WAIST_PITCH_INDEX], .1)
        robot = OfflineRobot(state.q, locked_waist_pitch=.1)
        robot.refresh(10.)
        robot.publish(state.q, robot.read())
        self.assertEqual(robot.q[WAIST_PITCH_INDEX], .1)
        self.assertEqual(robot.last_command.motor_cmd[14].mode, 0)


if __name__ == "__main__":
    unittest.main()

"""G1-D DDS 物理桥的离线检查；网络往返另由 tools/test_dds_e2e.py 验证。"""
import unittest

import numpy as np

from dds_network import validate_network
from g1d_config import CONTROLLED_JOINTS, initial_pose
from g1d_transport import Codec, KP, KD, SLOTS
from mujoco_sim import PhysicsBridge
from offline_robot import OfflineRobot
from real_motion import Guard


class TestPhysicsBridge(unittest.TestCase):
    def setUp(self):
        self.bridge = PhysicsBridge(initial_q=np.zeros(16))
        self.robot = OfflineRobot()
        self.robot.refresh(0.)

    def command(self, q=None):
        return self.robot.codec.encode(np.zeros(16) if q is None else q, self.robot.read())

    def test_feedback_crc_and_real_sdk_slot_layout(self):
        for k in range(10):
            self.bridge.step(k*.001)
        state = Codec().decode(self.bridge.state(), .01)
        self.assertEqual(state.tick, 10)
        self.assertEqual(set(state.present), set(SLOTS))
        Guard().state(state, .01)
        np.testing.assert_array_equal(state.q, 0.)
        self.assertEqual(self.bridge.active_steps, 0)

    def test_pd_maps_slots_to_actuators_by_joint_name(self):
        target = np.zeros(16)
        left = CONTROLLED_JOINTS.index("left_shoulder_pitch_joint")
        right = CONTROLLED_JOINTS.index("right_shoulder_pitch_joint")
        target[left], target[right] = .05, -.05
        self.assertTrue(self.bridge.accept(self.command(target), 0.))
        self.bridge.step(0.)
        self.assertAlmostEqual(self.bridge.data.ctrl[self.bridge.aids[left]], 2.)
        self.assertAlmostEqual(self.bridge.data.ctrl[self.bridge.aids[right]], -2.)

    def test_force_limits_and_disabled_actuator(self):
        target = np.full(16, 10.)
        cmd = self.command(target)
        cmd.motor_cmd[15].mode = 0
        cmd.crc = self.robot.codec.crc.Crc(cmd)
        self.assertTrue(self.bridge.accept(cmd, 0.))
        self.bridge.step(0.)
        wrist = CONTROLLED_JOINTS.index("left_wrist_pitch_joint")
        shoulder = CONTROLLED_JOINTS.index("left_shoulder_pitch_joint")
        self.assertEqual(self.bridge.data.ctrl[self.bridge.aids[shoulder]], 0.)
        self.assertEqual(self.bridge.data.ctrl[self.bridge.aids[wrist]], 5.)

    def test_bad_packets_cannot_replace_last_valid_command(self):
        self.assertTrue(self.bridge.accept(self.command(), 0.))
        old = self.bridge.command
        bad = self.command()
        bad.crc ^= 1
        self.assertFalse(self.bridge.accept(bad, .1))
        self.assertIs(self.bridge.command, old)
        wrong_mode = self.command()
        wrong_mode.mode_machine = 7
        wrong_mode.crc = self.robot.codec.crc.Crc(wrong_mode)
        self.assertFalse(self.bridge.accept(wrong_mode, .2))
        self.assertEqual(self.bridge.commands_rejected, 2)

    def test_active_feedback_comes_from_physics(self):
        self.assertTrue(self.bridge.accept(self.command(), 0.))
        for k in range(500):
            self.bridge.step(k*.001)
        state = Codec().decode(self.bridge.state(), .5)
        self.assertGreater(np.abs(state.q).max(), .001)
        self.assertEqual(self.bridge.active_steps, 500)
        self.assertTrue(self.bridge.timed_out)
        self.assertTrue(np.isfinite(state.q).all())
        # 超时保持上一目标的策略不会把动力学瞬移回 seed。
        self.assertGreater(self.bridge.report()["max_tip_tracking_error_mm"], .1)


class TestLoopbackConfiguration(unittest.TestCase):
    def test_loopback_and_invalid_domain(self):
        validate_network("lo", 0)
        with self.assertRaises(ValueError):
            validate_network("lo", 233)


class TestHangingStartup(unittest.TestCase):
    def test_default_simulation_starts_with_hanging_arms(self):
        from real_motion import Geometry
        bridge = PhysicsBridge()
        expected = np.array(initial_pose())
        np.testing.assert_allclose(bridge.initial_q, expected)
        np.testing.assert_allclose(bridge.data.qpos[bridge.qadr], expected)
        self.assertGreater(np.abs(expected).max(), .1)
        Geometry(expected).collision(expected)
        state = Codec().decode(bridge.state(), 0.)
        np.testing.assert_allclose(state.q, expected)


if __name__ == "__main__":
    unittest.main(verbosity=2)

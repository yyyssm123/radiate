"""只用 RPC 替身验证 MotionSwitcher 与下发前阻止活动运控。"""
from contextlib import nullcontext
import tempfile
import time
from dataclasses import replace
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

from g1d_config import initial_pose
from offline_robot import OfflineRobot
from real_motion import Limits
from g1d_transport import ControlFault
from offline_robot import OfflineRobot
import motion_control as control
import run_real


class TestMotionControl(unittest.TestCase):
    def test_active_ai_blocks_lowlevel_entry(self):
        client = Mock()
        client.CheckMode.return_value = (0, {"form": "0", "name": "ai"})
        with patch.object(control, "make_client", return_value=client):
            with self.assertRaisesRegex(ControlFault, "ai"):
                control.require_released("eth6")
        client.ReleaseMode.assert_not_called()

    def test_loopback_does_not_call_robot_service(self):
        with patch.object(control, "make_client") as factory:
            control.require_released("lo")
            factory.assert_not_called()

    def test_errors_and_malformed_status_do_not_mean_released(self):
        for response in ((3102, None), (0, {}), (0, {"name": None})):
            client = Mock()
            client.CheckMode.return_value = response
            with self.assertRaises(ControlFault):
                control.read_mode(client)

    def test_release_ai_and_verify_empty_name(self):
        client = Mock()
        client.CheckMode.side_effect = [(0, {"name": "ai"}), (0, {"name": ""})]
        client.ReleaseMode.return_value = (0, None)
        result = control.release_mode(client)
        self.assertEqual(result["after"]["name"], "")
        client.ReleaseMode.assert_called_once_with()
        client.SelectMode.assert_not_called()

    def test_already_released_needs_no_switch(self):
        client = Mock()
        client.CheckMode.return_value = (0, {"name": ""})
        self.assertEqual(control.release_mode(client)["release_calls"], 0)
        client.ReleaseMode.assert_not_called()

    def test_release_failure_or_unfinished_exit_is_not_success(self):
        for code in (3102, 0):
            client = Mock()
            client.CheckMode.return_value = (0, {"name": "ai"})
            client.ReleaseMode.return_value = (code, None)
            with self.assertRaises(ControlFault):
                control.release_mode(client, timeout=0.)

    def test_actual_sender_gate_rejects_before_writer_is_prepared(self):
        instances = []
        class Robot(OfflineRobot):
            def __init__(self, *_):
                super().__init__(initial_pose(), locked_waist_pitch=0.)
                self.prepared = False
                instances.append(self)
            def wait(self, *_):
                self.refresh(time.monotonic())
                return self.read()
            def prepare(self):
                self.prepared = True
            def close(self):
                pass
        client = Mock()
        client.CheckMode.return_value = (0, {"name": "ai"})
        with tempfile.TemporaryDirectory() as tmp, \
                patch.object(run_real, "DdsRobot", Robot), \
                patch.object(run_real, "writer_lock", return_value=nullcontext()), \
                patch.object(control, "make_client", return_value=client):
            with self.assertRaisesRegex(ControlFault, "ai"):
                run_real.main(["hold", "--network", "eth6", "--handoff", "release",
                               "--report", str(Path(tmp)/"plan.json")])
        self.assertEqual(instances[0].sent, 0)
        self.assertFalse(instances[0].prepared)

    def test_remote_start_uses_latest_pose_and_never_publishes_while_waiting(self):
        robot = Mock()
        fixture = OfflineRobot()
        fixture.refresh(time.monotonic())
        initial = fixture.read()
        confirmed = replace(initial, buttons=1 << 8, q=initial.q+.001)
        robot.read.return_value = confirmed
        with patch("builtins.print"):
            result = control.wait_start_button(robot, initial, Limits(), "eth6")
        self.assertIs(result, confirmed)
        robot.prepare.assert_not_called()
        robot.publish.assert_not_called()

    def test_already_held_a_must_be_released_and_pressed_again(self):
        fixture = OfflineRobot()
        fixture.refresh(time.monotonic())
        held = replace(fixture.read(), buttons=1 << 8)
        released = replace(held, buttons=0)
        pressed = replace(held)
        robot = Mock()
        robot.read.side_effect = [held, released, pressed]
        with patch("builtins.print"), patch.object(control.time, "sleep"):
            self.assertIs(control.wait_start_button(robot, held, Limits(), "eth6"), pressed)
        self.assertEqual(robot.read.call_count, 3)

    def test_b_cancels_before_any_command(self):
        fixture = OfflineRobot()
        fixture.refresh(time.monotonic())
        robot = Mock()
        robot.read.return_value = replace(fixture.read(), buttons=1 << 9)
        with patch("builtins.print"), self.assertRaisesRegex(ControlFault, "取消"):
            control.wait_start_button(robot, fixture.read(), Limits(), "eth6")
        robot.publish.assert_not_called()

    def test_waiting_with_no_state_still_faults(self):
        fixture = OfflineRobot()
        fixture.refresh(time.monotonic())
        robot = Mock()
        robot.read.return_value = None
        with patch("builtins.print"), self.assertRaises(ControlFault):
            control.wait_start_button(robot, fixture.read(), Limits(), "eth6")

    def test_loopback_start_needs_no_remote(self):
        robot = Mock()
        sentinel = object()
        self.assertIs(control.wait_start_button(robot, sentinel, Limits(), "lo"), sentinel)
        robot.read.assert_not_called()


if __name__ == "__main__":
    unittest.main()

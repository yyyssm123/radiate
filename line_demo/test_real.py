"""真机入口的离线回归，使用真实 HG IDL/CRC，不创建 DDS 网络通道。"""
from dataclasses import replace
import io
import json
from pathlib import Path
import signal
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

from g1d_config import ARM_JOINTS, CONTROLLED_JOINTS, initial_pose
from g1d_transport import Codec, ControlFault, DdsRobot, StateCache, KP, KD, LEFT, SLOTS
from offline_robot import OfflineRobot
from real_motion import Guard, Limits, transition
from real_runtime import Controller
import run_real


class NoCollision:
    def collision(self, q):
        pass


class TestProtocol(unittest.TestCase):
    def setUp(self):
        self.robot = OfflineRobot()
        self.robot.refresh(10.0)

    def test_all_command_fields_and_inactive_slots(self):
        state = self.robot.read()
        target = np.linspace(0, .01, 16)
        self.robot.publish(target, state)
        msg = self.robot.last_command
        self.assertEqual(msg.mode_machine, state.mode_machine)
        self.assertEqual(msg.mode_pr, 0)
        self.assertEqual(msg.crc, self.robot.codec.crc.Crc(msg))
        for k, slot in enumerate(SLOTS):
            motor = msg.motor_cmd[int(slot)]
            self.assertEqual((motor.mode, motor.kp, motor.kd, motor.dq, motor.tau),
                             (1, KP[k], KD[k], 0., 0.))
            self.assertAlmostEqual(motor.q, target[k], places=7)
        for slot in set(range(35)) - set(SLOTS):
            motor = msg.motor_cmd[slot]
            self.assertEqual((motor.mode, motor.q, motor.kp, motor.kd, motor.tau), (0, 0, 0, 0, 0))

    def test_bad_crc_cannot_refresh_watchdog(self):
        old = self.robot.read()
        self.robot.message.crc ^= 1
        self.robot.cache.update(self.robot.message, 10.3)
        self.assertIs(self.robot.read(), old)
        self.assertEqual(self.robot.cache.bad, 1)
        with self.assertRaisesRegex(ControlFault, "超时"):
            Guard().state(self.robot.read(), 10.3)

    def test_frozen_tick_detected_even_with_fresh_receipts(self):
        self.robot.cache.update(self.robot.message, 10.3)
        self.assertEqual(self.robot.read().received, 10.3)
        self.assertEqual(self.robot.read().progressed, 10.0)
        with self.assertRaisesRegex(ControlFault, "tick"):
            Guard().state(self.robot.read(), 10.3)

    def test_nonfinite_feedback_rejected(self):
        old = self.robot.read()
        self.robot.message.motor_state[15].q = float("nan")
        self.robot.message.crc = self.robot.codec.crc.Crc(self.robot.message)
        self.robot.cache.update(self.robot.message, 10.1)
        self.assertIs(self.robot.read(), old)

    def test_stationary_zero_waist_with_mode_is_not_missing(self):
        motor = self.robot.message.motor_state[14]
        motor.mode = 1
        motor.q = motor.dq = motor.ddq = motor.tau_est = motor.vol = 0.
        motor.temperature = [0, 0]
        motor.motorstate = 0
        self.robot.message.crc = self.robot.codec.crc.Crc(self.robot.message)
        state = self.robot.codec.decode(self.robot.message, 10.)
        self.assertIn(14, state.present)
        Guard().state(state, 10.)

    def test_all_zero_waist_without_status_still_blocks(self):
        motor = self.robot.message.motor_state[14]
        motor.mode = motor.motorstate = 0
        motor.q = motor.dq = motor.ddq = motor.tau_est = motor.vol = 0.
        motor.temperature = [0, 0]
        self.robot.message.crc = self.robot.codec.crc.Crc(self.robot.message)
        state = self.robot.codec.decode(self.robot.message, 10.)
        self.assertNotIn(14, state.present)
        with self.assertRaisesRegex(ControlFault, r"\[14\].*inspect_device"):
            Guard().state(state, 10.)

    def test_fault_only_waist_reports_fault_instead_of_missing(self):
        motor = self.robot.message.motor_state[14]
        motor.mode = 0
        motor.motorstate = 1
        motor.q = motor.dq = motor.ddq = motor.tau_est = motor.vol = 0.
        motor.temperature = [0, 0]
        self.robot.message.crc = self.robot.codec.crc.Crc(self.robot.message)
        state = self.robot.codec.decode(self.robot.message, 10.)
        self.assertIn(14, state.present)
        with self.assertRaisesRegex(ControlFault, "电机报告故障"):
            Guard().state(state, 10.)

    def test_prepare_writer_does_not_send(self):
        class Publisher:
            def __init__(self, topic, message_type):
                self.topic, self.written, self.initialized = topic, [], False
            def Init(self):
                self.initialized = True
            def Write(self, command):
                self.written.append(command)
                return True
        robot = object.__new__(DdsRobot)
        robot._publisher_class, robot._command_type = Publisher, object
        robot.publisher, robot.codec, robot.sent = None, self.robot.codec, 0
        robot.prepare()
        self.assertEqual(robot.publisher.topic, "rt/lowcmd")
        self.assertTrue(robot.publisher.initialized)
        self.assertEqual(robot.publisher.written, [])
        robot.publish(np.zeros(16), self.robot.read())
        self.assertEqual(len(robot.publisher.written), 1)
        self.assertEqual(robot.sent, 1)


class TestFeedbackGuard(unittest.TestCase):
    def setUp(self):
        self.robot = OfflineRobot()
        self.robot.refresh(10.)
        self.state = self.robot.read()

    def test_wrong_robot_layout_and_missing_motor(self):
        for present in (self.state.present + (0,), tuple(s for s in self.state.present if s != 21)):
            with self.assertRaisesRegex(ControlFault, "槽位"):
                Guard().state(replace(self.state, present=present), 10.)

    def test_mode_change_and_non_pr_feedback(self):
        guard = Guard()
        guard.state(self.state, 10.)
        with self.assertRaisesRegex(ControlFault, "mode_machine"):
            guard.state(replace(self.state, mode_machine=7), 10.)
        with self.assertRaisesRegex(ControlFault, "mode_pr"):
            Guard().state(replace(self.state, mode_pr=1), 10.)

    def test_motor_fault_and_temperature(self):
        errors = self.state.errors.copy()
        errors[LEFT[0]] = 1
        with self.assertRaisesRegex(ControlFault, "故障"):
            Guard().state(replace(self.state, errors=errors), 10.)
        with self.assertRaisesRegex(ControlFault, "温度"):
            Guard().state(replace(self.state, temperature=np.full(16, 80)), 10.)
        tau = self.state.tau.copy()
        tau[LEFT[5]] = 6.0
        with self.assertRaisesRegex(ControlFault, "实测力矩"):
            Guard().state(replace(self.state, tau=tau), 10.)

    def test_limits_and_nonfinite_commands(self):
        guard = Guard()
        for q in (guard.hi + 1, np.full(16, np.nan), np.zeros(7)):
            with self.assertRaises(ControlFault):
                guard.position(q)

    def test_velocity_and_acceleration(self):
        for distance, error in ((.01, "角速度"), (.00001, "角加速度")):
            guard = Guard()
            guard.last_command = np.zeros(16)
            guard.last_velocity = np.zeros(16)
            q = np.zeros(16)
            q[LEFT[0]] = distance
            with self.assertRaisesRegex(ControlFault, error):
                guard.command(q, self.state, 10., .002)

    def test_wrist_torque_budget(self):
        q = np.zeros(16)
        q[LEFT[5]] = .12
        with self.assertRaisesRegex(ControlFault, "力矩"):
            Guard().command(q, self.state, 10., .002)

    def test_waist_drift(self):
        q_actual = self.state.q.copy()
        q_actual[1] = .11
        with self.assertRaisesRegex(ControlFault, "腰部"):
            Guard().command(np.zeros(16), replace(self.state, q=q_actual), 10., .002)

    def test_persistent_following_error(self):
        guard = Guard()
        q_actual = self.state.q.copy()
        q_actual[LEFT[0]] = -.3
        state = replace(self.state, q=q_actual)
        guard.command(np.zeros(16), state, 10., .002)
        with self.assertRaisesRegex(ControlFault, "持续"):
            guard.command(np.zeros(16), replace(state, received=10.21, progressed=10.21), 10.21, .21)


class TestController(unittest.TestCase):
    def setUp(self):
        self.robot = OfflineRobot()
        self.robot.refresh(0.)
        self.limits = Limits()
        target = np.zeros(7)
        target[0] = .03
        self.segments = [("out", transition(np.zeros(7), target, self.limits)),
                         ("back", transition(target, np.zeros(7), self.limits))]

    def test_complete_probe_returns_to_capture_and_holds(self):
        controller = Controller(self.robot.read(), self.segments, NoCollision(), self.limits, 0.)
        duration = 2 + sum(s.duration for _, s in self.segments) + .02
        for k in range(int(duration / .002) + 1):
            now = k * .002
            self.robot.refresh(now)
            tick = controller.tick(self.robot.read(), now)
            self.robot.publish(tick.q, self.robot.read())
            np.testing.assert_array_equal(tick.q[np.setdiff1d(np.arange(16), LEFT)], 0)
        self.assertTrue(tick.complete)
        self.assertEqual(tick.phase, "finished-hold")
        np.testing.assert_allclose(tick.q, 0., atol=1e-8)

    def test_remote_stop_freezes_last_target(self):
        controller = Controller(self.robot.read(), self.segments, NoCollision(), self.limits, 0.,
                                startup_hold=0.)
        state = self.robot.read()
        for k in range(301):
            now = k * .002
            self.robot.refresh(now)
            tick = controller.tick(self.robot.read(), now)
            self.robot.publish(tick.q, self.robot.read())
        previous = tick.q.copy()
        self.robot.refresh(.602)
        tick = controller.tick(replace(self.robot.read(), buttons=1 << 8), .602)
        self.assertEqual(tick.phase, "stopped-hold")
        np.testing.assert_array_equal(tick.q, previous)
        self.assertGreater(previous[LEFT[0]], 0.)

    def test_scheduling_gap_is_not_caught_up_by_jumping(self):
        for gap in (.0628, .07):
            with self.subTest(allowed_gap=gap):
                self.robot.refresh(0.)
                controller = Controller(self.robot.read(), [], NoCollision(), self.limits, 0.)
                controller.tick(self.robot.read(), 0.)
                self.robot.refresh(gap)
                tick = controller.tick(self.robot.read(), gap)
                self.assertEqual(tick.phase, "startup-hold")
                np.testing.assert_array_equal(tick.q, np.zeros(16))
        for gap in (.070001, .08):
            with self.subTest(rejected_gap=gap):
                self.robot.refresh(0.)
                controller = Controller(self.robot.read(), [], NoCollision(), self.limits, 0.)
                controller.tick(self.robot.read(), 0.)
                self.robot.refresh(gap)
                with self.assertRaisesRegex(ControlFault, "调度"):
                    controller.tick(self.robot.read(), gap)
                self.assertEqual(controller.previous, 0.)

    def test_retimed_trajectory_respects_derivative_limits(self):
        motion = self.segments[0][1]
        times = np.linspace(0, motion.base_duration, 10001)
        self.assertLessEqual(np.abs(motion.spline(times, 1)).max() / motion.scale, .25)
        self.assertLessEqual(np.abs(motion.spline(times, 2)).max() / motion.scale**2, .5)


class TestReadOnlyEntrypoints(unittest.TestCase):
    def test_missing_simulator_error_explains_startup_command(self):
        robot = object.__new__(DdsRobot)
        robot.network, robot.domain = "lo", 0
        robot.cache = StateCache(Codec())
        with self.assertRaisesRegex(ControlFault, "mujoco_sim.py --network lo --domain 0"):
            robot.wait(timeout=0)

    def test_inspect_and_dry_run_never_prepare_writer(self):
        class ReadOnlyRobot(OfflineRobot):
            prepared = 0
            published = 0
            def __init__(self, *_):
                super().__init__()
                self.refresh(time.monotonic())
                self.cache.good = 10
            def wait(self, *_):
                return self.read()
            def prepare(self):
                ReadOnlyRobot.prepared += 1
                raise AssertionError("只读入口创建了 writer")
            def publish(self, *_):
                ReadOnlyRobot.published += 1
                raise AssertionError("只读入口发送了指令")
        with tempfile.TemporaryDirectory() as tmp:
            for action in ("inspect", "scan"):
                args = [action, "--network", "test0", "--report", str(Path(tmp) / f"{action}.json")]
                if action == "scan":
                    args.append("--dry-run")
                with patch.object(run_real, "DdsRobot", ReadOnlyRobot), \
                        patch.object(run_real, "build", return_value=([], NoCollision(), {})), \
                        patch("sys.stdout", new=io.StringIO()):
                    self.assertEqual(run_real.main(args), 0)
                report = json.loads((Path(tmp) / f"{action}.json").read_text())
                self.assertEqual(report["published_commands"], 0)
        self.assertEqual(ReadOnlyRobot.prepared, 0)
        self.assertEqual(ReadOnlyRobot.published, 0)

    def test_live_entrypoint_requires_explicit_interface(self):
        with patch("sys.stderr", new=io.StringIO()), patch.object(run_real, "DdsRobot") as robot:
            with self.assertRaises(SystemExit) as result:
                run_real.main(["hold"])
            self.assertEqual(result.exception.code, 2)
            robot.assert_not_called()


class TestSendingLoop(unittest.TestCase):
    def test_actual_loop_logs_and_restores_handlers_on_exit(self):
        class Clock:
            t = 10.0
            def now(self):
                return self.t
            def sleep(self, seconds):
                self.t += seconds
        clock = Clock()
        class Robot(OfflineRobot):
            def read(self):
                self.refresh(clock.now())
                return super().read()
            def publish(self, q, state):
                super().publish(q, state)
                if self.sent == 3:
                    handler = signal.getsignal(signal.SIGINT)
                    handler(signal.SIGINT, None)
                    handler(signal.SIGINT, None)
        robot = Robot()
        captured = robot.read()
        previous = signal.getsignal(signal.SIGINT)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "run.csv"
            with patch.object(run_real.time, "monotonic", side_effect=clock.now), \
                    patch.object(run_real.time, "sleep", side_effect=clock.sleep), \
                    patch("sys.stdout", new=io.StringIO()):
                run_real.run_loop(robot, captured, [], NoCollision(), Limits(), path)
            status = json.loads(path.with_suffix(".status.json").read_text())
            self.assertEqual(status["commands"], 3)
            self.assertEqual(status["outcome"], "operator-exit")
            self.assertEqual(status["csv_dropped_rows"], 0)
            self.assertIsNone(status["csv_error"])
            self.assertAlmostEqual(status["loop_timing"]["max"]["gap_ms"], 2.)
            self.assertGreater(len(path.read_text().splitlines()), 1)
        self.assertIs(signal.getsignal(signal.SIGINT), previous)


class TestCapturedStartup(unittest.TestCase):
    def setUp(self):
        q = np.array(initial_pose())
        q[0], q[1] = .02, .03
        q[-1] = .07
        self.robot = OfflineRobot(q)
        self.robot.refresh(0.)
        self.captured = self.robot.read()
        self.limits = Limits()
        target = self.captured.q[LEFT].copy()
        target[0] += .03
        self.segments = [("out", transition(self.captured.q[LEFT], target, self.limits)),
                         ("back", transition(target, self.captured.q[LEFT], self.limits))]

    def test_first_command_and_startup_hold_equal_measured_nonzero_pose(self):
        controller = Controller(self.captured, self.segments, NoCollision(), self.limits, 0.)
        for k in range(1000):
            now = k * .002
            self.robot.refresh(now)
            state = self.robot.read()
            tick = controller.tick(state, now)
            self.robot.publish(tick.q, state)
            np.testing.assert_array_equal(tick.q, self.captured.q)
            encoded = np.array([self.robot.last_command.motor_cmd[int(s)].q for s in SLOTS])
            np.testing.assert_allclose(encoded, self.captured.q, atol=1e-7)

    def test_nonzero_start_transition_return_and_other_joints_are_continuous(self):
        controller = Controller(self.captured, self.segments, NoCollision(), self.limits, 0.,
                                startup_hold=0.)
        duration = sum(s.duration for _, s in self.segments) + .02
        previous = self.captured.q.copy()
        untouched = np.setdiff1d(np.arange(16), LEFT)
        for k in range(int(duration / .002) + 1):
            now = k * .002
            self.robot.refresh(now)
            state = self.robot.read()
            tick = controller.tick(state, now)
            self.robot.publish(tick.q, state)
            self.assertLessEqual(np.abs(tick.q - previous).max(), .25*.002*1.05 + 1e-7)
            np.testing.assert_array_equal(tick.q[untouched], self.captured.q[untouched])
            previous = tick.q.copy()
        self.assertTrue(tick.complete)
        np.testing.assert_allclose(tick.q, self.captured.q, atol=1e-7)

    def test_zero_start_profile_is_rejected_before_any_command(self):
        wrong = [("bad", transition(np.zeros(7), self.captured.q[LEFT], self.limits))]
        with self.assertRaisesRegex(ControlFault, "起点"):
            Controller(self.captured, wrong, NoCollision(), self.limits, 0.)
        self.assertEqual(self.robot.sent, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)

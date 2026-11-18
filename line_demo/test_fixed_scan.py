"""固定扫描轨迹及反馈换段回归。"""
from dataclasses import replace
import unittest

import numpy as np

from fixed_scan import FixedScanSegments, fixed_scan_plan
from g1d_config import HANG_LEFT, initial_pose
from g1d_transport import ControlFault, LEFT, SLOTS
from offline_robot import OfflineRobot
from real_motion import GRAVITY_JOINTS, Geometry, Guard, Limits, Motion
from real_runtime import Controller


class TestFixedScan(unittest.TestCase):
    def test_different_capture_uses_collision_detour_and_same_scan(self):
        home = np.array(initial_pose())
        different = home.copy()
        different[0] = .1
        different[LEFT] = [.519919, -.043552, -.285609, .72944, -.605124, .080044, .732515]
        limits = Limits(locked_waist_pitch=0.)
        simple, _, simple_report = fixed_scan_plan(home, limits, .10)
        detour, _, detour_report = fixed_scan_plan(different, limits, .10)

        self.assertEqual(simple_report["approach_method"], "direct")
        self.assertEqual(detour_report["approach_method"], "rrt-connect")
        np.testing.assert_allclose(detour[0][1].sample(0.), different[LEFT])
        scan_a, scan_b = simple[1][1], detour[1][1]
        np.testing.assert_array_equal(scan_a.sample(np.linspace(0, scan_a.duration, 200)),
                                      scan_b.sample(np.linspace(0, scan_b.duration, 200)))
        for segments, report in ((simple, simple_report), (detour, detour_report)):
            self.assertLess(report["planned_wrist_orientation_error_deg"], .01)
            self.assertLess(report["planned_transverse_max_mm"], .01)
            back = segments[-1][1]
            np.testing.assert_allclose(back.sample(back.duration), HANG_LEFT, atol=1e-9)
        self.assertNotEqual(simple_report["target_bottom_m"], detour_report["target_bottom_m"])


class TestSettling(unittest.TestCase):
    def setUp(self):
        class NoCollision:
            def collision(self, q):
                pass

        self.q = np.array(initial_pose())
        rest = self.q[LEFT]
        fixture = OfflineRobot(self.q, locked_waist_pitch=0.)
        fixture.refresh(0.)
        self.state = fixture.read()
        hold = Motion([0., .02], np.stack((rest, rest)))
        self.controller = Controller(self.state, FixedScanSegments(hold, hold, hold),
                                     NoCollision(), Limits(locked_waist_pitch=0.), 0.,
                                     period=.01, startup_hold=0.)

    def tick(self, index, position_error=0., velocity=0.):
        q, dq = self.q.copy(), np.zeros_like(self.q)
        q[LEFT[0]] += position_error
        dq[LEFT[0]] = velocity
        now = index * .01
        state = replace(self.state, q=q, dq=dq, tick=2*index, received=now, progressed=now)
        return self.controller.tick(state, now)

    def test_scan_and_return_both_wait_for_measured_settling(self):
        phases = []
        for i in range(100):
            # 进入扫描前角度偏差过大；扫描结束后实测仍在运动。
            result = self.tick(i, position_error=.07 if i < 10 else 0.,
                               velocity=.06 if 37 <= i < 50 else 0.)
            if not phases or phases[-1][1] != result.phase:
                phases.append((i, result.phase))
        self.assertEqual([name for _, name in phases],
                         ["approach", "scan-settle-hold", "scan", "return-settle-hold",
                          "return", "finished-hold"])
        self.assertGreaterEqual(phases[2][0], 35)
        self.assertGreaterEqual(phases[4][0], 75)
        self.assertTrue(result.complete)

    def test_settling_timeout_never_enters_scan(self):
        with self.assertRaisesRegex(ControlFault, "scan 前未停稳"):
            for i in range(450):
                self.assertIn(self.tick(i, position_error=.07).phase, ("approach", "scan-settle-hold"))


class TestGravityFeedforward(unittest.TestCase):
    def test_ramp_and_dds_fields(self):
        q = np.array(initial_pose())
        robot = OfflineRobot(q, locked_waist_pitch=0.)
        robot.refresh(0.)
        hold = Motion([0., 3.], np.stack((q[LEFT], q[LEFT])))
        geometry = Geometry(q)
        limits = Limits(locked_waist_pitch=0., gravity_compensation=True)
        ctl = Controller(robot.read(), FixedScanSegments(hold, hold, hold), geometry,
                         limits, 0., period=.02)
        model_ff = geometry.gravity_feedforward(q)
        self.assertTrue(np.all(np.abs(model_ff[GRAVITY_JOINTS]) > .1))
        self.assertEqual(np.count_nonzero(model_ff), 2)
        for k in range(201):
            now = .02*k
            robot.refresh(now)
            state = robot.read()
            tick = ctl.tick(state, now)
            if k in (0, 100, 150, 200):
                expected = {0: 0., 100: 0., 150: .5, 200: 1.}[k] * model_ff
                robot.publish(tick.q, state, tick.feedforward)
                np.testing.assert_allclose([robot.last_command.motor_cmd[i].tau for i in SLOTS],
                                           expected, atol=1e-7)
                self.assertEqual(robot.last_command.motor_cmd[14].mode, 0)

    def test_total_torque_guard_includes_feedforward(self):
        robot = OfflineRobot(initial_pose(), locked_waist_pitch=0.)
        robot.refresh(0.)
        state = robot.read()
        guard = Guard(Limits(locked_waist_pitch=0.))
        q = state.q.copy()
        q[LEFT[0]] += .05
        ff = np.zeros_like(q)
        ff[LEFT[0]] = guard.effort[LEFT[0]] * guard.limits.torque_fraction - 1.
        with self.assertRaisesRegex(ControlFault, "预计 PD 与前馈合计力矩超预算"):
            guard.command(q, state, 0., .002, feedforward=ff)


if __name__ == "__main__":
    unittest.main()

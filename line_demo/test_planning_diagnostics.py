"""实测起点碰撞和预检失败报告回归；不连接机器人或 DDS。"""
import io
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

from g1d_config import ROOT, CONTROLLED_JOINTS, initial_pose
from g1d_transport import LEFT
from line_servo_demo import Arm
from offline_robot import OfflineRobot
from real_motion import (Geometry, Guard, Limits, PlanningFault, scan_plan,
                         transition, validate_motion)
import run_real


SAMPLE = ROOT / "references/g1d_right_shoulder_model_contact_sample.json"


class TestPlanningDiagnostics(unittest.TestCase):
    def setUp(self):
        self.sample = json.loads(SAMPLE.read_text())
        self.q = np.array(self.sample["q_rad"])
        self.limits = Limits(locked_waist_pitch=0.)

    def test_measured_capture_reports_right_shoulder_before_searching_left_ik(self):
        original = self.q.copy()
        with patch("real_motion.continuous_line_ik") as search:
            with self.assertRaises(PlanningFault) as caught:
                scan_plan(self.q, self.limits, speed=.1)
            search.assert_not_called()
        detail = caught.exception.details["failures"][0]
        self.assertEqual(detail["stage"], "captured")
        self.assertEqual(set(detail["contacts"][0]["bodies"]),
                         {"torso_link", "right_shoulder_yaw_link"})
        self.assertGreater(detail["contacts"][0]["penetration_mm"], 4.)
        self.assertLess(detail["contacts"][0]["penetration_mm"], 5.)
        np.testing.assert_array_equal(detail["q_rad"], original)
        np.testing.assert_array_equal(self.q, original)

    def test_motion_failure_records_phase_time_and_actual_test_pose(self):
        motion = transition(self.q[LEFT], self.q[LEFT]+.001, self.limits)
        with self.assertRaises(PlanningFault) as caught:
            validate_motion(motion, Guard(self.limits), Geometry(self.q), self.q, stage="approach")
        self.assertEqual(caught.exception.details["stage"], "approach")
        self.assertEqual(caught.exception.details["time_s"], 0.)
        np.testing.assert_array_equal(caught.exception.details["q_rad"], self.q)

    def test_all_ik_rejections_are_retained_instead_of_only_last_error(self):
        q = np.array(initial_pose())
        with patch.object(Arm, "ik", return_value=(np.zeros(7), .1)):
            with self.assertRaises(PlanningFault) as caught:
                scan_plan(q, self.limits, speed=.1)
        failures = caught.exception.details["failures"]
        self.assertEqual(len(failures), 8)
        self.assertEqual(len({entry["branch"] for entry in failures}), 8)
        self.assertTrue(all(entry["stage"] == "ik-first" for entry in failures))
        self.assertTrue(all("首点 IK 误差" in entry["error"] for entry in failures))

    def test_model_only_clearance_change_allows_capture_without_disabling_collision(self):
        clear = self.q.copy()
        clear[CONTROLLED_JOINTS.index("right_shoulder_roll_joint")] = -.05
        Geometry(clear).collision(clear)
        with self.assertRaises(PlanningFault):
            Geometry(self.q).collision(self.q)

    def test_rejected_check_saves_exact_pose_for_offline_reproduction(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"check.json"
            with patch("sys.stdout", new=io.StringIO()):
                with self.assertRaises(PlanningFault):
                    run_real.main(["check", "--state-json", str(SAMPLE), "--report", str(path)])
            report = json.loads(path.read_text())
            self.assertEqual(report["failure_stage"], "planning")
            self.assertEqual(report["status"], "planning-rejected")
            np.testing.assert_array_equal(report["q_rad"], self.q)
            self.assertEqual(report["published_commands"], 0)
            self.assertEqual(report["planning_diagnostics"]["failures"][0]["stage"], "captured")

    def test_live_and_dry_run_failures_save_feedback_without_creating_writer(self):
        q = self.q.copy()
        class Robot(OfflineRobot):
            def __init__(self, *_):
                super().__init__(q, locked_waist_pitch=0.)
                self.message.mode_machine = 5
                self.refresh(time.monotonic())
                self.cache.good = 23
            def wait(self, *_):
                return self.read()
            def prepare(self):
                raise AssertionError("预检失败创建了指令 writer")
            def publish(self, *_):
                raise AssertionError("预检失败发送了运动指令")
        with tempfile.TemporaryDirectory() as tmp:
            for dry_run in (False, True):
                path = Path(tmp)/f"failure_{dry_run}.json"
                args = ["scan", "--network", "model_contact_test", "--report", str(path)]
                if dry_run:
                    args.append("--dry-run")
                with self.subTest(dry_run=dry_run), \
                        patch.object(run_real, "DdsRobot", Robot), \
                        patch.object(run_real, "wait_start_button", side_effect=lambda _, state, *_args: state), \
                        patch.object(run_real, "run_loop") as loop, \
                        patch("sys.stdout", new=io.StringIO()):
                    with self.assertRaises(PlanningFault):
                        run_real.main(args)
                    loop.assert_not_called()
                report = json.loads(path.read_text())
                self.assertEqual(report["mode_machine"], 5)
                self.assertEqual(report["published_commands"], 0)
                self.assertEqual(report["planning_diagnostics"]["failures"][0]["stage"], "captured")
                np.testing.assert_allclose(report["q_rad"], q, atol=1e-7)


if __name__ == "__main__":
    unittest.main()

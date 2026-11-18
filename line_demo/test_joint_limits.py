"""现场左腕超限反馈回归；不连接 DDS，不发送运动指令。"""
from dataclasses import replace
import io
import json
from pathlib import Path
import runpy
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import numpy as np

from g1d_config import ROOT, CONTROLLED_JOINTS, initial_pose
from g1d_transport import Codec, ControlFault
from joint_limits import joint_limits, joint_limit_violations
from offline_robot import OfflineRobot
from real_motion import Guard, Limits
import run_real


SAMPLE = ROOT / "references/g1d_wrist_limit_lowstate_sample.json"


def measured_state(now):
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
    raw = json.loads(SAMPLE.read_text())
    msg = unitree_hg_msg_dds__LowState_()
    msg.mode_pr, msg.mode_machine, msg.tick = raw["mode_pr"], raw["mode_machine"], raw["tick"]
    for row in raw["slots"]:
        for name, value in row.items():
            if name != "slot":
                setattr(msg.motor_state[row["slot"]], name, value)
    codec = Codec(0.)
    msg.crc = codec.crc.Crc(msg)
    return codec.decode(msg, now)


class TestJointLimitDiagnostics(unittest.TestCase):
    def test_measured_report_identifies_only_left_wrist_yaw_without_changing_q(self):
        state = measured_state(10.)
        original = state.q.copy()
        rows = joint_limit_violations(state.q)
        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertEqual((row["slot"], row["joint"], row["side"]),
                         (21, "left_wrist_yaw_joint", "lower"))
        self.assertAlmostEqual(row["q_rad"], -1.7686200141906738)
        self.assertAlmostEqual(row["lower_rad"], -1.614429558)
        self.assertAlmostEqual(row["excess_rad"], .15419045619067373)
        np.testing.assert_array_equal(state.q, original)

    def test_guard_error_contains_joint_slot_angle_and_excess(self):
        with self.assertRaises(ControlFault) as caught:
            Guard(Limits(locked_waist_pitch=0.)).state(measured_state(10.), 10.)
        error = str(caught.exception)
        for detail in ("slot 21", "left_wrist_yaw_joint", "-1.768620", "-101.33°",
                       "-1.614430", "8.83°"):
            self.assertIn(detail, error)

    def test_feedback_tolerance_and_target_margin_are_preserved(self):
        guard = Guard(Limits(locked_waist_pitch=0.))
        state = measured_state(10.)
        i = CONTROLLED_JOINTS.index("left_wrist_yaw_joint")
        q = state.q.copy()
        q[i] = guard.lo[i]-.005
        guard.state(replace(state, q=q), 10.)
        q[i] = guard.lo[i]-.011
        with self.assertRaises(ControlFault):
            guard.state(replace(state, q=q), 10.)
        q[i] = guard.lo[i]+.01
        with self.assertRaisesRegex(ControlFault, "目标余量"):
            guard.position(q)

    def test_both_limit_directions_and_startup_margin_are_reported(self):
        lo, hi, _ = joint_limits()
        q = np.array(initial_pose())
        q[2], q[3] = lo[2]-.1, hi[3]+.2
        rows = joint_limit_violations(q)
        self.assertEqual([r["side"] for r in rows], ["lower", "upper"])
        np.testing.assert_allclose([r["excess_rad"] for r in rows], [.1, .2])
        q[2], q[3] = lo[2]+.01, hi[3]-.01
        self.assertEqual(joint_limit_violations(q), [])
        self.assertEqual(len(joint_limit_violations(q, .02)), 2)

    def test_bad_limit_input_is_rejected(self):
        for q in (np.zeros(7), np.full(16, np.nan)):
            with self.assertRaises(ValueError):
                joint_limit_violations(q)

    def test_live_scan_aborts_before_remote_start_planning_or_writer(self):
        class Robot(OfflineRobot):
            def __init__(self, *_):
                super().__init__(locked_waist_pitch=0.)
                self.cache.state = measured_state(time.monotonic())
                self.cache.good = 20
            def wait(self, *_):
                return self.read()
            def prepare(self):
                raise AssertionError("超限状态创建了 writer")
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"failure.json"
            with patch.object(run_real, "DdsRobot", Robot), \
                    patch.object(run_real, "wait_start_button") as start, \
                    patch.object(run_real, "build") as build, \
                    patch("sys.stdout", new=io.StringIO()):
                with self.assertRaisesRegex(ControlFault, "slot 21"):
                    run_real.main(["scan", "--network", "eth6", "--report", str(path)])
                start.assert_not_called()
                build.assert_not_called()
            report = json.loads(path.read_text())
            self.assertEqual(report["published_commands"], 0)
            self.assertEqual(report["requested_action"], "scan")
            self.assertEqual(report["joint_limit_violations"][0]["slot"], 21)
            self.assertAlmostEqual(report["q_rad"][8], -1.7686200141906738)

    def test_saved_raw_report_can_be_analyzed_without_dds_or_sdk(self):
        before = SAMPLE.read_bytes()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/"analysis.json"
            args = ["inspect_device.py", "--from-report", str(SAMPLE), "--report", str(path)]
            with patch.object(sys, "argv", args), \
                    patch.dict(sys.modules, {"dds_network": None, "unitree_sdk2py": None}), \
                    patch("sys.stdout", new=io.StringIO()):
                runpy.run_path(str(ROOT/"tools/inspect_device.py"), run_name="__main__")
            report = json.loads(path.read_text())
            self.assertFalse(report["joint_limit_check"]["within_urdf"])
            self.assertEqual(report["joint_limit_check"]["violations"][0]["slot"], 21)
            self.assertEqual(report["slots"], json.loads(before)["slots"])
        self.assertEqual(SAMPLE.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()

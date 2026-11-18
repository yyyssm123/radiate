"""独立复核局部时间规划的速度、加速度、转向、几何精度和非零启动接入。"""
import unittest
import numpy as np

from g1d_config import initial_pose
from g1d_transport import LEFT
from path_timing import LocalPathMotion
from real_motion import Limits, scan_plan


class TestLocalPathMotion(unittest.TestCase):
    def test_curved_path_respects_limits_between_grid_points(self):
        s = np.linspace(0, 1, 51)
        q = np.column_stack((s, .01*np.tanh((s-.5)/.03)))
        motion = LocalPathMotion(s, q, .3, .4, .6, cycles=2)
        t = np.linspace(0, motion.duration, 60001)
        velocity, acceleration = motion.sample(t, 1), motion.sample(t, 2)
        self.assertLessEqual(np.abs(velocity).max(), .4)
        self.assertLessEqual(np.abs(acceleration).max(), .6)
        self.assertLessEqual(np.abs(velocity).max(), motion.peak_joint_speed+1e-8)
        self.assertLessEqual(np.abs(acceleration).max(), motion.peak_joint_acceleration+1e-8)
        # 第一列 q=s，可以独立复核路径速度上限及转向位置。
        self.assertLessEqual(np.abs(velocity[:, 0]).max(), .3)
        for k in range(5):
            np.testing.assert_allclose(motion.sample(k*motion.one_way_duration), q[0 if k%2 == 0 else -1], atol=1e-9)
            np.testing.assert_allclose(motion.sample(k*motion.one_way_duration, 1), 0., atol=1e-9)

    def test_analytic_derivatives_match_position_differences(self):
        s = np.linspace(0, .5, 11)
        q = np.column_stack((s, s*s))
        motion = LocalPathMotion(s, q, .1, .25, .5)
        # 避开分格加速度切换点，位置的一二阶导数应与解析结果一致。
        t = (motion.times[:-1]+np.diff(motion.times)*.43)*motion.certification_scale
        for times in (t, motion.duration-t):
            eps = 1e-5
            center, plus, minus = motion.sample(times), motion.sample(times+eps), motion.sample(times-eps)
            np.testing.assert_allclose((plus-minus)/(2*eps), motion.sample(times, 1), atol=2e-8)
            np.testing.assert_allclose((plus-2*center+minus)/eps**2, motion.sample(times, 2), atol=2e-5)

    def test_invalid_inputs_do_not_produce_a_trajectory(self):
        for speed in (0., -1., np.nan):
            with self.assertRaises(ValueError):
                LocalPathMotion([0., .5, 1.], [[0.], [.5], [1.]], speed, .25, .5)


class TestContinuousScan(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.captured = np.array(initial_pose())
        cls.slow = scan_plan(cls.captured, Limits(), speed=.03)
        cls.fast = scan_plan(cls.captured, Limits(), speed=.10)

    def test_speed_setting_changes_scan_duration_without_raising_joint_limits(self):
        slow, fast = self.slow[2], self.fast[2]
        self.assertLess(fast["scan_s"], slow["scan_s"]*.75)
        self.assertLess(fast["scan_s"], 30.)  # 旧实现同一路径是 136.26 秒。
        self.assertEqual(fast["joint_speed_limit_rad_s"], .25)
        self.assertEqual(fast["joint_acceleration_limit_rad_s2"], .5)
        motion = self.fast[0][1][1]
        t = np.linspace(0, motion.duration, 50001)
        self.assertLessEqual(np.abs(motion.sample(t, 1)).max(), .25)
        self.assertLessEqual(np.abs(motion.sample(t, 2)).max(), .5)

    def test_hanging_start_segment_joins_and_return_are_continuous(self):
        segments = self.fast[0]
        np.testing.assert_allclose(segments[0][1].sample(0), self.captured[LEFT], atol=1e-9)
        for (_, first), (_, second) in zip(segments, segments[1:]):
            np.testing.assert_allclose(first.sample(first.duration), second.sample(0), atol=1e-9)
        np.testing.assert_allclose(segments[-1][1].sample(segments[-1][1].duration), self.captured[LEFT], atol=1e-9)

    def test_smoothed_joint_path_still_follows_original_cartesian_line(self):
        segments, geometry, report = self.fast
        motion = segments[1][1]
        s = np.linspace(0, .5, 1001)
        positions = np.array([geometry.arm.fk(q) for q in motion.path(s)])
        target = np.array(report["target_bottom_m"])+np.column_stack((s*0, s*0, s))
        self.assertLess(np.linalg.norm(positions-target, axis=1).max(), 1e-4)
        # 原尖峰来自腕 roll 的冗余解跳动；新路径的相邻 5 mm 节点没有该突变。
        coarse = motion.path(np.linspace(0, .5, 101))
        self.assertLess(np.abs(np.diff(coarse[:, 4])).max(), .005)
        # 模型 Jacobian 复核末端速度，而非仅检查路径参数 ds/dt。
        t = np.linspace(0, motion.duration, 2001)
        peak_tip_speed = max(np.linalg.norm(geometry.arm.position_jacobian(q) @ dq)
                             for q, dq in zip(motion.sample(t), motion.sample(t, 1)))
        self.assertLessEqual(peak_tip_speed, .1)
        self.assertLessEqual(report["checked_peak_tip_speed_m_s"], .1)


if __name__ == "__main__":
    unittest.main()

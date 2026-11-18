"""固定左腕扫描路径，以及随捕获姿态变化的两段转移。"""
from hashlib import sha256
import json

import numpy as np

from g1d_config import ARM_JOINTS, HANG_LEFT, MODEL
from g1d_transport import ControlFault, LEFT
from path_timing import LocalPathMotion
from real_motion import (Geometry, Guard, PlanningFault, check_capture, failure_detail,
                         transition, validate_motion)


PROFILE = MODEL.with_name("scan_profile.json")
LENGTH = .50


class FixedScanSegments(list):
    def __init__(self, approach, scan, back):
        super().__init__([("approach", approach), ("scan", scan), ("return", back)])


class Transfer:
    def __init__(self, waypoints, limits):
        self.sections = [transition(a, b, limits, minimum=.6)
                         for a, b in zip(waypoints, waypoints[1:])]
        self.ends = np.cumsum([step.duration for step in self.sections])
        self.duration = float(self.ends[-1])

    def sample(self, t):
        i = min(np.searchsorted(self.ends, t, side="right"), len(self.sections) - 1)
        return self.sections[i].sample(t - (self.ends[i-1] if i else 0.))


def search_waypoints(start, end, lo, hi, edge_clear):
    """有限次双向 RRT-Connect，只在左臂的关节空间中搜索。"""
    rng = np.random.default_rng(895)
    # 每棵树保存关节角和通向根节点的父索引。
    trees = [([start], [-1]), ([end], [-1])]

    def extend(tree, target):
        nodes, parents = tree
        parent = int(np.argmin(np.linalg.norm(np.asarray(nodes) - target, axis=1)))
        a = nodes[parent]
        gap = np.abs(target - a).max()
        if gap < 1e-12:
            return parent, True
        b = target if gap <= .20 else a + (.20 / gap) * (target - a)
        if not edge_clear(a, b):
            return None, False
        nodes.append(b)
        parents.append(parent)
        return len(nodes) - 1, gap <= .20

    def trace(tree, i):
        nodes, parents = tree
        path = []
        while i >= 0:
            path.append(nodes[i])
            i = parents[i]
        return path

    for iteration in range(240):
        side = iteration % 2
        a, b = trees[side], trees[1-side]
        if iteration % 5 == 0:
            target = b[0][0]
        elif iteration % 5 == 1:
            target = rng.uniform(lo, hi)
        else:
            target = np.clip(rng.uniform(np.minimum(start, end)-.7, np.maximum(start, end)+.7), lo, hi)
        i, _ = extend(a, target)
        if i is None:
            continue
        while True:
            j, joined = extend(b, a[0][i])
            if j is None or joined:
                break
        if not joined:
            continue
        path = trace(a, i)[::-1] + trace(b, j)[1:]
        if side:
            path.reverse()
        # 直接连接相隔最远且可达的节点，避免沿搜索树逐级停顿。
        simplified = [path[0]]
        i = 0
        while i < len(path)-1:
            for j in range(len(path)-1, i, -1):
                if j == i+1 or edge_clear(path[i], path[j]):
                    simplified.append(path[j])
                    i = j
                    break
        return simplified
    return None


def plan_transfer(start, end, limits, guard, geometry, captured, stage):
    direct = transition(start, end, limits)
    try:
        validate_motion(direct, guard, geometry, captured, stage=stage)
        return direct, "direct", 0
    except PlanningFault as direct_error:
        details = {"failures": [failure_detail(direct_error, stage)]}

    full = captured.copy()
    for name, angles in (("start", start), ("end", end)):
        full[LEFT] = angles
        try:
            guard.position(full)
            geometry.collision(full)
        except ControlFault as exc:
            raise PlanningFault(f"{stage} 端点未通过模型预检: {exc}",
                                {"failures": [failure_detail(exc, stage, full, endpoint=name)]}) from exc

    def edge_clear(a, b):
        n = max(1, int(np.ceil(np.abs(b-a).max() / .015)))
        for q in np.linspace(a, b, n+1)[1:]:
            full[LEFT] = q
            try:
                guard.position(full)
                geometry.collision(full)
            except ControlFault:
                return False
        return True

    path = search_waypoints(start, end, guard.lo[LEFT]+limits.margin,
                            guard.hi[LEFT]-limits.margin, edge_clear)
    if path is None:
        raise PlanningFault(f"{stage} 的直接路径受阻，搜索预算内未找到可行绕行", details)
    motion = Transfer(path, limits)
    validate_motion(motion, guard, geometry, captured, stage=stage)
    return motion, "rrt-connect", len(path)-2


def fixed_scan_plan(captured, limits, speed=.03, cycles=1):
    geometry, guard = Geometry(captured), Guard(limits)
    check_capture(captured, guard, geometry)
    raw_profile = PROFILE.read_bytes()
    profile = json.loads(raw_profile)
    if (not isinstance(profile, dict) or profile.get("joint_names") != ARM_JOINTS
            or profile.get("scene_sha256") != sha256(MODEL.read_bytes()).hexdigest()):
        raise ControlFault("固定扫描配置与当前 MuJoCo 模型不匹配；需要重新生成并复核轨迹")
    nodes = np.asarray(profile.get("left_q_rad"), float)
    if nodes.shape != (101, len(ARM_JOINTS)) or not np.isfinite(nodes).all():
        raise ControlFault("固定扫描配置的关节点位格式错误")
    rest = np.asarray(HANG_LEFT)
    arm = geometry.arm
    bottom, rotation = arm.pose(nodes[0])
    top = arm.fk(nodes[-1])
    if np.linalg.norm(top - bottom - [0., 0., LENGTH]) > .001:
        raise ControlFault("当前腰姿下固定轨迹无法保持竖直 50 cm 扫描线")
    motion = LocalPathMotion(np.linspace(0., LENGTH, len(nodes)), nodes, speed,
                             limits.joint_speed, limits.joint_acceleration, cycles)
    transverse = validate_motion(motion, guard, geometry, captured, bottom[:2], speed,
                                 stage="scan", tip_rotation=rotation)
    approach, approach_method, approach_vias = plan_transfer(
        captured[LEFT], nodes[0], limits, guard, geometry, captured, "approach")
    back, return_method, return_vias = plan_transfer(
        nodes[0], rest, limits, guard, geometry, captured, "return")
    report = {"scan_mode": "fixed", "profile_sha256": sha256(raw_profile).hexdigest(),
              "end_effectors": arm.end_effectors, "reference_frame": "torso-attached",
              "length_m": LENGTH, "target_bottom_m": bottom.tolist(), "target_top_m": top.tolist(),
              "scan_start_left_q_rad": nodes[0].tolist(), "rest_left_q_rad": rest.tolist(),
              "approach_s": approach.duration, "scan_s": motion.duration, "return_s": back.duration,
              "approach_method": approach_method, "return_method": return_method,
              "approach_waypoints": approach_vias, "return_waypoints": return_vias,
              "requested_peak_speed_m_s": speed, "planned_peak_path_speed_m_s": motion.peak_path_speed,
              "checked_peak_tip_speed_m_s": motion.checked_peak_tip_speed,
              "planned_mean_path_speed_m_s": 2*cycles*LENGTH/motion.duration,
              "planned_peak_joint_speed_rad_s": motion.peak_joint_speed,
              "planned_peak_joint_acceleration_rad_s2": motion.peak_joint_acceleration,
              "joint_speed_limit_rad_s": limits.joint_speed,
              "joint_acceleration_limit_rad_s2": limits.joint_acceleration,
              "timing_certification_scale": motion.certification_scale,
              "startup_q_rad": captured.tolist(), "planned_transverse_max_mm": transverse*1000,
              "planned_wrist_orientation_error_deg": motion.checked_max_rotation_deg}
    return FixedScanSegments(approach, motion, back), geometry, report

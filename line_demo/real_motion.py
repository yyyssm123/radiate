"""真机和离线夹具共用：轨迹预检、时间标定与反馈保护。"""
from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.integrate import solve_ivp

from g1d_config import MODEL, CONTROLLED_JOINTS, ARM_JOINTS, WAIST_PITCH_INDEX
from g1d_transport import ControlFault, LEFT, SLOTS, KP, KD
from joint_limits import joint_limits, joint_limit_violations, format_joint_limit_violations
from line_servo_demo import Arm, make_line
from path_timing import LocalPathMotion


class PlanningFault(ControlFault):
    def __init__(self, message, details):
        super().__init__(message)
        self.details = details


def failure_detail(exc, stage, q=None, **context):
    detail = dict(getattr(exc, "details", {}))
    detail.update({"stage": stage, "error": str(exc), **context})
    if q is not None:
        detail["q_rad"] = np.asarray(q, float).tolist()
    return detail


def reject_candidate(diagnostics, branch, exc, stage):
    stage = getattr(exc, "details", {}).get("stage", stage)
    diagnostics.setdefault("failures", []).append(failure_detail(exc, stage, branch=branch))
    diagnostics["last_error"] = str(exc)
    diagnostics["rejected"] += 1


@dataclass
class Limits:
    margin: float = 0.02
    state_timeout: float = 0.20
    joint_speed: float = 0.25
    joint_acceleration: float = 0.5
    following_error: float = 0.25
    following_seconds: float = 0.2
    torso_error: float = 0.10
    temperature: float = 70.0
    torque_fraction: float = 0.90
    locked_waist_pitch: float | None = None


class Guard:
    def __init__(self, limits=None):
        self.limits = limits or Limits()
        self.lo, self.hi, self.effort = joint_limits()
        self.mode_machine = None
        self.bad_following_since = None
        self.last_command = None
        self.last_velocity = None
        self.last_dt = None

    def state(self, state, now):
        if state is None:
            raise ControlFault("没有 LowState")
        if max(now - state.received, now - state.progressed) > self.limits.state_timeout:
            raise ControlFault("LowState 超时或 tick 停止推进")
        if state.mode_pr != 0:
            raise ControlFault("LowState mode_pr 不是关节 PR 模式 0")
        if self.mode_machine is None:
            self.mode_machine = state.mode_machine
        elif state.mode_machine != self.mode_machine:
            raise ControlFault("mode_machine 在控制过程中改变")
        invalid = set(map(int, state.present)).intersection({*range(12), 13})
        required = set(map(int, SLOTS))
        if self.limits.locked_waist_pitch is not None:
            required.remove(14)
            if abs(state.q[WAIST_PITCH_INDEX]-self.limits.locked_waist_pitch) > .02:
                raise ControlFault("腰部 pitch 反馈与已确认的锁定角不一致")
        missing = required.difference(state.present)
        if invalid or missing:
            raise ControlFault(f"反馈槽位无法确认 G1-D 布局：无效槽位有信号 {sorted(invalid)}，"
                               f"有效槽位缺少可识别反馈 {sorted(missing)}。"
                               "全零反馈不能直接判定电机不存在；请运行 "
                               "python3 tools/inspect_device.py --network 实际网卡 只读核对，未继续发送指令")
        if not np.isfinite(np.concatenate((state.q, state.dq, state.tau))).all():
            raise ControlFault("反馈中有非有限数值")
        if np.any(state.errors):
            bad = [(CONTROLLED_JOINTS[i], int(e)) for i, e in enumerate(state.errors) if e]
            raise ControlFault(f"电机报告故障: {bad}")
        if np.max(state.temperature) >= self.limits.temperature:
            raise ControlFault("电机温度超过设定值")
        if np.any(np.abs(state.tau) > self.effort * 1.1):
            raise ControlFault("实测力矩估计超过 URDF 预算")
        if np.any(state.q < self.lo - 0.01) or np.any(state.q > self.hi + 0.01):
            rows = joint_limit_violations(state.q, bounds=(self.lo, self.hi))
            raise ControlFault("实测关节角超出 G1-D URDF 限位: "
                               + format_joint_limit_violations(rows))

    def position(self, q):
        q = np.asarray(q, float)
        if q.shape != self.lo.shape or not np.isfinite(q).all():
            raise ControlFault("目标关节角格式错误")
        if (self.limits.locked_waist_pitch is not None
                and abs(q[WAIST_PITCH_INDEX]-self.limits.locked_waist_pitch) > .001):
            raise ControlFault("规划改变了已锁定的腰部 pitch 角度")
        bad = (q < self.lo + self.limits.margin) | (q > self.hi - self.limits.margin)
        if np.any(bad):
            rows = joint_limit_violations(q, self.limits.margin, (self.lo, self.hi))
            raise ControlFault("目标接近或超出限位: " + format_joint_limit_violations(rows))

    def command(self, q, state, now, dt, holding=False):
        self.state(state, now)
        self.position(q)
        q = np.asarray(q)
        if self.last_command is not None:
            velocity = (q - self.last_command) / dt
            if np.max(np.abs(velocity)) > self.limits.joint_speed * 1.05 + 1e-5:
                raise ControlFault("目标角速度超限")
            if not holding and self.last_velocity is not None:
                # 相邻有限差分速度分别对应两个采样区间的中点。
                # 使用中点间隔，避免调度抖动造成虚假的加速度超限。
                velocity_dt = .5 * (dt + (self.last_dt or dt))
                acceleration = (velocity - self.last_velocity) / velocity_dt
                if np.max(np.abs(acceleration)) > self.limits.joint_acceleration * 1.2 + 1e-3:
                    raise ControlFault("目标角加速度超限")
            self.last_velocity = velocity
        else:
            self.last_velocity = np.zeros_like(q)
        # 这是发送前的 PD 力矩估计，不能替代电机固件的硬限幅。
        predicted_torque = KP * (q - state.q) - KD * state.dq
        if np.any(np.abs(predicted_torque) > self.effort * self.limits.torque_fraction):
            j = int(np.argmax(np.abs(predicted_torque) / self.effort))
            raise ControlFault(f"预计 PD 力矩超预算: {CONTROLLED_JOINTS[j]} "
                               f"{predicted_torque[j]:+.2f} N·m")
        error = np.abs(q - state.q)
        if np.max(error[:2]) > self.limits.torso_error:
            raise ControlFault("腰部无法保持捕获姿态，停止轨迹")
        if np.max(error) > self.limits.following_error:
            if self.bad_following_since is None:
                self.bad_following_since = now
            elif now - self.bad_following_since >= self.limits.following_seconds:
                raise ControlFault("关节跟踪误差持续超限")
        else:
            self.bad_following_since = None
        self.last_command = q.copy()
        self.last_dt = dt
        return predicted_torque


class Geometry:
    def __init__(self, captured):
        self.arm = Arm(MODEL)
        self.data = mujoco.MjData(self.arm.m)
        ids = [mujoco.mj_name2id(self.arm.m, mujoco.mjtObj.mjOBJ_JOINT, n)
               for n in CONTROLLED_JOINTS]
        self.qadr = self.arm.m.jnt_qposadr[ids]
        self.arm._fd.qpos[self.qadr] = captured

    def collision(self, q):
        self.data.qpos[self.qadr] = q
        mujoco.mj_forward(self.arm.m, self.data)
        pairs = {}
        for contact in self.data.contact:
            if contact.dist < -0.002:
                names = [mujoco.mj_id2name(self.arm.m, mujoco.mjtObj.mjOBJ_BODY,
                                         int(self.arm.m.geom_bodyid[g]))
                         for g in (contact.geom1, contact.geom2)]
                key = tuple(sorted(names))
                entry = {"bodies": names, "penetration_mm": float(-contact.dist*1000),
                         "geom_ids": [int(contact.geom1), int(contact.geom2)],
                         "position_m": contact.pos.tolist()}
                if key not in pairs or entry["penetration_mm"] > pairs[key]["penetration_mm"]:
                    pairs[key] = entry
        if pairs:
            contacts = sorted(pairs.values(), key=lambda row: row["penetration_mm"], reverse=True)
            worst = contacts[0]
            raise PlanningFault(f"模型自碰撞: {worst['bodies']}, "
                                f"穿透 {worst['penetration_mm']:.1f} mm",
                                {"kind": "self_collision", "contacts": contacts,
                                 "q_rad": np.asarray(q, float).tolist()})


class Motion:
    def __init__(self, times, q, speed=0.25, acceleration=0.5):
        self.spline = CubicSpline(times, q, axis=0,
                                 bc_type=((1, np.zeros(q.shape[1])), (1, np.zeros(q.shape[1]))))
        self.base_duration = float(times[-1])
        # 三次样条在每段的速度是二次式、加速度是一次式，直接检查极值，
        # 避免仅稀疏采样漏掉尖峰。
        c0, c1, c2, _ = self.spline.c
        h = np.diff(times)[:, None]
        end_v = 3*c0*h*h + 2*c1*h + c2
        critical = np.divide(-c1, 3*c0, out=np.zeros_like(c0), where=c0 != 0)
        critical = np.clip(critical, 0, h)
        critical_v = 3*c0*critical**2 + 2*c1*critical + c2
        max_speed = float(max(np.abs(c2).max(), np.abs(end_v).max(), np.abs(critical_v).max()))
        max_acc = float(max(np.abs(2*c1).max(), np.abs(6*c0*h + 2*c1).max()))
        self.scale = max(1.0, max_speed / speed, np.sqrt(max_acc / acceleration)) * 1.02
        self.duration = self.base_duration * self.scale

    def sample(self, t):
        return self.spline(np.clip(t / self.scale, 0, self.base_duration))


def transition(start, end, limits, minimum=3.0):
    gap = float(np.abs(end - start).max())
    duration = max(minimum, gap * 1.875 / limits.joint_speed,
                   np.sqrt(gap * 5.78 / limits.joint_acceleration))
    u = np.linspace(0, 1, max(51, int(duration / 0.02) + 1))
    s = 10*u**3 - 15*u**4 + 6*u**5
    return Motion(u * duration, start + s[:, None] * (end - start),
                  limits.joint_speed, limits.joint_acceleration)


def validate_motion(motion, guard, geometry, captured, line_xy=None, tip_speed_limit=None,
                    stage="trajectory"):
    # 几何预检覆盖整个运动，不在发送期间计算 IK。
    n = max(2, int(np.ceil(motion.duration / 0.01)) + 1)
    transverse, peak_tip_speed = 0.0, 0.0
    for t in np.linspace(0, motion.duration, n):
        q = captured.copy()
        q[LEFT] = motion.sample(t)
        try:
            guard.position(q)
            geometry.collision(q)
        except ControlFault as exc:
            raise PlanningFault(str(exc), failure_detail(exc, stage, q, time_s=float(t))) from exc
        if line_xy is not None:
            p = geometry.data.xpos[geometry.arm.tip]
            transverse = max(transverse, float(np.linalg.norm(p[:2] - line_xy)))
        if tip_speed_limit is not None:
            velocity = geometry.arm.position_jacobian(q[LEFT]) @ motion.sample(t, 1)
            peak_tip_speed = max(peak_tip_speed, float(np.linalg.norm(velocity)))
    if line_xy is not None and transverse > .001:
        raise ControlFault(f"插值轨迹偏离目标直线 {transverse*1000:.2f} mm")
    if tip_speed_limit is not None:
        if peak_tip_speed > tip_speed_limit*(1+1e-4):
            raise ControlFault(f"FK 复核的末端速度超限: {peak_tip_speed:.4f} m/s")
        motion.checked_peak_tip_speed = peak_tip_speed
    return transverse


def continuous_line_ik(geometry, captured, points, limits, diagnostics):
    """依次产生连续 IK 分支，调用方还必须检查完整接近、扫描、返回。"""
    arm, guard = geometry.arm, Guard(limits)
    candidates = [("captured", captured[LEFT].copy(), 0.), ("neutral", arm.q0.copy(), 0.),
                  ("captured-margin", captured[LEFT].copy(), limits.margin+.001)]
    for yaw in (-1., -.5, -1.5, .5, 1.):
        seed = np.array([.3, .2, yaw, 1., 0., 0., captured[LEFT][-1]])
        candidates.append((f"shoulder-yaw-{yaw:g}", seed, limits.margin+.001))
    distances = np.linalg.norm(points-points[0], axis=1)
    direction = (points[-1]-points[0])/distances[-1]
    for label, seed, margin in candidates:
        stage = "ik-first"
        try:
            first, error = arm.ik(points[0], seed=seed, q_ref=seed, n_restart=40, margin=margin)
            if error > 1e-5 or not np.isfinite(first).all():
                raise ControlFault(f"首点 IK 误差 {error*1000:.2f} mm")
            full = captured.copy()
            full[LEFT] = first
            guard.position(full)
            geometry.collision(full)
            stage = "ik-line"
            evaluations = 0
            def slope(s, q):
                nonlocal evaluations
                evaluations += 1
                if evaluations > 20000:
                    raise ControlFault("连续 IK 超过迭代预算，需要调整扫描范围")
                jac = arm.position_jacobian(q)
                value = jac.T @ np.linalg.solve(jac @ jac.T + 1e-12*np.eye(3), direction)
                if not np.isfinite(value).all() or np.abs(value).max() > 1000:
                    raise ControlFault("连续 IK 接近位置奇异点，需要调整扫描范围")
                return value
            result = solve_ivp(slope, (0., distances[-1]), first, t_eval=distances,
                               max_step=min(.002, distances[-1]/100), rtol=1e-9, atol=1e-11)
            if not result.success or result.y.shape[1] != len(points):
                raise ControlFault(f"连续 IK 未完成: {result.message}")
            solutions = result.y.T
            errors = np.array([np.linalg.norm(arm.fk(q)-p) for q, p in zip(solutions, points)])
            if not np.isfinite(solutions).all() or errors.max() > 1e-5:
                raise ControlFault(f"连续 IK 位置漂移 {errors.max()*1000:.3f} mm")
            for angles in solutions:
                full = captured.copy()
                full[LEFT] = angles
                guard.position(full)
                geometry.collision(full)
            slopes = (slope(distances[0], solutions[0]), slope(distances[-1], solutions[-1]))
            yield distances, solutions, errors, slopes, label
        except (ControlFault, np.linalg.LinAlgError) as exc:
            reject_candidate(diagnostics, label, exc, stage)


def scan_plan(captured, limits, length=.50, forward=.02, center_dz=.20,
              speed=.03, cycles=1, anchor="zero"):
    geometry = Geometry(captured)
    arm = geometry.arm
    if anchor == "current":
        p0 = arm.fk(captured[LEFT])
        # current 模式起点就在实测左腕处，forward/center_dz 仍可显式给定。
        x, y = p0[0] + forward, p0[1]
        zc = p0[2] + center_dz
        points = np.column_stack((np.full(101, x), np.full(101, y),
                                  np.linspace(zc-length/2, zc+length/2, 101)))
    else:
        points, _ = make_line(arm, length, forward, center_dz, 101)
    points = np.asarray(points)
    guard = Guard(limits)
    try:
        guard.position(captured)
        geometry.collision(captured)
    except ControlFault as exc:
        raise PlanningFault("捕获起始姿态未通过模型预检: " + str(exc),
                            {"captured_q_rad": captured.tolist(),
                             "failures": [failure_detail(exc, "captured", captured)]}) from exc
    diagnostics = {"rejected": 0, "last_error": "没有可用分支", "failures": []}
    for distances, solutions, errors, slopes, branch in continuous_line_ik(
            geometry, captured, points, limits, diagnostics):
        try:
            motion = LocalPathMotion(distances, solutions, speed, limits.joint_speed,
                                     limits.joint_acceleration, cycles, slopes)
            approach = transition(captured[LEFT], solutions[0], limits)
            back = transition(solutions[0], captured[LEFT], limits)
            validate_motion(approach, guard, geometry, captured, stage="approach")
            transverse = validate_motion(motion, guard, geometry, captured, np.asarray(points[0])[:2],
                                         speed, stage="scan")
            validate_motion(back, guard, geometry, captured, stage="return")
            break
        except (ControlFault, ValueError) as exc:
            reject_candidate(diagnostics, branch, exc, "retiming")
    else:
        raise PlanningFault(f"所有 {diagnostics['rejected']} 个 IK 候选均未通过完整路径预检: "
                            f"{diagnostics['last_error']}",
                            {"captured_q_rad": captured.tolist(),
                             "failures": diagnostics["failures"]})
    report = {"ik_max_mm": float(max(errors) * 1000), "length_m": length,
              "end_effectors": geometry.arm.end_effectors,
              "target_bottom_m": np.asarray(points[0]).tolist(),
              "target_top_m": np.asarray(points[-1]).tolist(),
              "approach_s": approach.duration, "scan_s": motion.duration,
              "return_s": back.duration, "anchor": anchor,
              "max_start_gap_rad": float(np.abs(solutions[0] - captured[LEFT]).max()),
              "planner": "continuous Jacobian IK + local path-speed LP, exact joint extrema check",
              "requested_peak_speed_m_s": speed,
              "planned_peak_path_speed_m_s": motion.peak_path_speed,
              "checked_peak_tip_speed_m_s": motion.checked_peak_tip_speed,
              "planned_mean_path_speed_m_s": 2*cycles*length/motion.duration,
              "planned_peak_joint_speed_rad_s": motion.peak_joint_speed,
              "planned_peak_joint_acceleration_rad_s2": motion.peak_joint_acceleration,
              "joint_speed_limit_rad_s": limits.joint_speed,
              "joint_acceleration_limit_rad_s2": limits.joint_acceleration,
              "timing_certification_scale": motion.certification_scale,
              "startup_source": "captured LowState (offline uses example pose)",
              "startup_q_rad": captured.tolist(),
              "collision_branches_rejected": diagnostics["rejected"],
              "candidate_failures": diagnostics["failures"],
              "ik_branch": branch,
              "planned_transverse_max_mm": transverse * 1000}
    return [("approach", approach), ("scan", motion), ("return", back)], geometry, report


def probe_plan(captured, joint, amplitude, limits):
    if joint not in ARM_JOINTS:
        raise ControlFault("probe 只接受左臂关节名")
    if not 0 < abs(amplitude) <= .05:
        raise ControlFault("probe 幅度必须大于 0 且不超过 0.05 rad")
    target = captured[LEFT].copy()
    target[ARM_JOINTS.index(joint)] += amplitude
    segments = [("probe-out", transition(captured[LEFT], target, limits)),
                ("probe-back", transition(target, captured[LEFT], limits))]
    geometry, guard = Geometry(captured), Guard(limits)
    for _, segment in segments:
        validate_motion(segment, guard, geometry, captured)
    return segments, geometry, {"joint": joint, "amplitude_rad": amplitude,
                                "end_effectors": geometry.arm.end_effectors}

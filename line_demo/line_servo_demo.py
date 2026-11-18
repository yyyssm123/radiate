#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""G1-D 左手竖直线扫描：G1-D URDF + G1-D SDK 示例原值 PD。

这个版本在 MuJoCo 里跑完整物理，不需要额外启动仿真器，也不需要 DDS：

    python3 line_servo_demo.py                # 弹窗口看
    python3 line_servo_demo.py --check        # 只算轨迹
    python3 line_servo_demo.py --no-viewer    # 无窗口，只打印
    python3 line_servo_demo.py --no-viewer --fast  # 不等待墙钟时间

模型由项目内 models/g1_d.urdf 转换，不使用 G1 双足或 H2 模型。
底盘、轮子、升降柱和手指固定在 URDF 零位；默认约束腰部，用官方 PD 控臂。
--waist-mode pd 关闭腰部约束，对照全腰/臂 PD 的效果。
本程序不连接 DDS、不控制硬件。真机映射和增益集中在 g1d_config.py。
"""
import argparse
import importlib
import sys
import time

import numpy as np

try:
    import mujoco
except ImportError:
    sys.exit("需要 mujoco: pip install mujoco")
try:
    from scipy.optimize import least_squares
except ImportError:
    sys.exit("需要 scipy: pip install scipy")

from g1d_config import (MODEL, TIP_BODY, ARM_JOINTS, CONTROLLED_JOINTS,
                        JOINT_SLOTS, pd_for_joint, actuator_name, initial_pose, model_assembly)


# ==================================================================
class Arm:
    """MuJoCo 模型 + 运动学 + 仿真播放。"""

    def __init__(self, path):
        self.end_effectors = model_assembly(path)
        self.m = mujoco.MjModel.from_xml_path(str(path))
        self.d = mujoco.MjData(self.m)
        mid = lambda o, n: mujoco.mj_name2id(self.m, o, n)

        self.jids = [mid(mujoco.mjtObj.mjOBJ_JOINT, n) for n in ARM_JOINTS]
        miss = [n for n, j in zip(ARM_JOINTS, self.jids) if j < 0]
        if miss:
            raise RuntimeError(f"模型缺少关节: {miss}")

        self.qadr = np.array([self.m.jnt_qposadr[j] for j in self.jids])
        self.dadr = np.array([self.m.jnt_dofadr[j] for j in self.jids])
        self.lo = np.array([self.m.jnt_range[j][0] for j in self.jids])
        self.hi = np.array([self.m.jnt_range[j][1] for j in self.jids])
        self.tip = mid(mujoco.mjtObj.mjOBJ_BODY, TIP_BODY)
        if self.tip < 0:
            raise RuntimeError(f"模型缺少末端 {TIP_BODY}")

        self.aids = [mid(mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name(n))
                     for n in ARM_JOINTS]
        self.acts_ok = all(a >= 0 for a in self.aids)

        self.q0 = np.zeros(7)
        # 左臂顺序：肩 pitch/roll/yaw、肘、腕 roll/pitch/yaw。
        self.KP = np.array([pd_for_joint(n)[0] for n in ARM_JOINTS])
        self.KD = np.array([pd_for_joint(n)[1] for n in ARM_JOINTS])

        mujoco.mj_forward(self.m, self.d)
        self._fd = mujoco.MjData(self.m)
        self.R0 = self.pose(self.q0)[1].copy()

    # ---------- 运动学 ----------
    def pose(self, q):
        d = self._fd
        d.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.m, d)
        mujoco.mj_comPos(self.m, d)
        return d.xpos[self.tip].copy(), d.xmat[self.tip].reshape(3, 3).copy()

    def fk(self, q):
        return self.pose(q)[0]

    def position_jacobian(self, q):
        self.pose(q)
        position = np.zeros((3, self.m.nv))
        mujoco.mj_jacBody(self.m, self._fd, position, None, self.tip)
        return position[:, self.dadr]

    def tip_pos(self):
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)
        return self.d.xpos[self.tip].copy()

    def ik(self, target, seed=None, q_ref=None, w_ref=0.02, n_restart=200,
           tol=1e-5, margin=0.):
        """位置 IK + 参考关节角软正则，再修正位置误差；不约束末端朝向。"""
        q_ref = self.q0 if q_ref is None else q_ref
        rng = np.random.default_rng(12345)
        lo, hi = self.lo+margin, self.hi-margin
        if not np.isfinite(margin) or margin < 0 or np.any(lo >= hi):
            raise ValueError("IK 关节限位余量不合法")

        def pos_only(s):
            s = np.clip(s, lo + 1e-6, hi - 1e-6)
            sol = least_squares(lambda q: self.fk(q) - target, s,
                                bounds=(lo, hi), method="trf",
                                xtol=1e-13, ftol=1e-13, gtol=1e-13,
                                max_nfev=400)
            return sol.x, float(np.linalg.norm(self.fk(sol.x) - target))

        # ---- 阶段 1：先找能到达的解 ----
        base = seed if seed is not None else q_ref
        q, e = pos_only(base)
        if e > tol:
            best_q, best_e = q, e
            span = hi - lo
            for r in range(n_restart):
                # 在 seed 附近扰动，不用全域随机 —— 否则会跳到差 2 rad
                # 的另一个解分支，位置对但姿态完全翻转
                scale = 0.05 + 0.5 * (r / max(1, n_restart))
                s = np.clip(base + rng.normal(0, scale, len(base)) * span,
                            lo + 1e-6, hi - 1e-6)
                q2, e2 = pos_only(s)
                if e2 <= tol:
                    d_old = (np.abs(best_q - base).sum()
                             if best_e <= tol else 1e9)
                    if best_e > tol or np.abs(q2 - base).sum() < d_old:
                        best_q, best_e = q2, e2
                    if best_e <= tol and r > n_restart // 4:
                        break
                elif e2 < best_e:
                    best_q, best_e = q2, e2
            q, e = best_q, best_e

        if e > 1e-2:
            return q, e

        # ---- 阶段 2：零空间正则 ----
        try:
            sol = least_squares(
                lambda qq: np.concatenate([self.fk(qq) - target,
                                           w_ref * (qq - q_ref)]),
                q, bounds=(lo, hi), method="trf",
                xtol=1e-13, ftol=1e-13, gtol=1e-13, max_nfev=200)
            e2 = float(np.linalg.norm(self.fk(sol.x) - target))
            if e2 <= max(e * 3, 5e-4):
                q3, e3 = pos_only(sol.x)
                if e3 <= e2:
                    return q3, e3
                return sol.x, e2
        except Exception:
            pass
        return q, e

    # ---------- 仿真播放 ----------
    def _prepare_pd(self):
        """按关节名映射 G1-D SDK 增益；MuJoCo 编号与 DDS 槽位分开。"""
        if not self.acts_ok:
            raise RuntimeError("找不到左臂 actuator")
        if self.m.nu != len(CONTROLLED_JOINTS):
            raise RuntimeError("需要由 build_model.py 生成的 G1-D 腰部/双臂模型")
        jids = self.m.actuator_trnid[:, 0]
        actual = [mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_JOINT, int(j))
                  for j in jids]
        if (len(set(actual)) != len(actual) or set(actual) != set(CONTROLLED_JOINTS)
                or np.any(self.m.actuator_trntype != 0)
                or not np.allclose(self.m.actuator_gear, [1, 0, 0, 0, 0, 0])
                or np.any(self.m.actuator_dyntype != 0)
                or np.any(self.m.actuator_gaintype != 0)
                or not np.allclose(self.m.actuator_gainprm[:, 0], 1)
                or np.any(self.m.actuator_biastype != 0)):
            raise RuntimeError("执行器关节或类型与 G1-D 力矩电机模型不一致")
        self._act_qadr = self.m.jnt_qposadr[jids]
        self._act_dadr = self.m.jnt_dofadr[jids]
        self._kp_all = np.array([pd_for_joint(n)[0] for n in actual])
        self._kd_all = np.array([pd_for_joint(n)[1] for n in actual])
        self._act_names = actual

    def _pd_torque(self, q_target):
        # 对应 LowCmd 的 dq=0、tau=0：不额外引入速度或重力前馈。
        tau = (self._kp_all * (q_target - self.d.qpos[self._act_qadr])
               - self._kd_all * self.d.qvel[self._act_dadr])
        lo = np.where(self.m.actuator_ctrllimited,
                      self.m.actuator_ctrlrange[:, 0], -np.inf)
        hi = np.where(self.m.actuator_ctrllimited,
                      self.m.actuator_ctrlrange[:, 1], np.inf)
        return np.clip(tau, lo, hi)

    def play(self, traj, dt=0.02, viewer=True, speed=1.0, realtime=True,
             waist_mode="fixed", initial_q=None):
        self._prepare_pd()

        mujoco.mj_resetData(self.m, self.d)
        pose = np.asarray(initial_pose() if initial_q is None else initial_q, float)
        if pose.shape != (16,) or not np.isfinite(pose).all():
            raise ValueError("初始姿态必须是 16 个有限关节角")
        values = dict(zip(CONTROLLED_JOINTS, pose))
        self.d.qpos[self._act_qadr] = [values[n] for n in self._act_names]
        for name in ("Yaw_Joint", "torso_Joint"):
            eq = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_EQUALITY, f"hold_{name}")
            if eq < 0:
                raise RuntimeError("缺少腰部约束，请重新运行 build_model.py")
            self.d.eq_active[eq] = waist_mode == "fixed"
            self.m.eq_data[eq, 0] = values[name]
        mujoco.mj_forward(self.m, self.d)

        start = self.d.qpos[self.qadr].copy()

        # 非左臂的目标固定在初始化姿态；不能每步追随当前角度，
        # 否则位置误差始终为零，只剩阻尼，无法提供保持刚度。
        q_target = self.d.qpos[self._act_qadr].copy()
        substeps = max(1, int(round(dt / self.m.opt.timestep)))
        frame_dt = substeps * self.m.opt.timestep
        if not np.isclose(frame_dt, dt):
            raise ValueError("--dt 必须是模型物理步长的整数倍")

        v = None
        if viewer:
            mjv = importlib.import_module("mujoco.viewer")
            v = mjv.launch_passive(self.m, self.d)
            v.cam.distance = 2.4
            v.cam.azimuth = 130
            v.cam.elevation = -8
            v.cam.lookat[:] = [0.15, 0.0, 0.65]

        def step():
            self.d.ctrl[:] = self._pd_torque(q_target)
            mujoco.mj_step(self.m, self.d)

        def frame():
            for _ in range(substeps):
                step()
            if v is not None:
                v.sync()
            if realtime:
                time.sleep(frame_dt / speed)

        # 从实际仿真初始姿态接近工作点，至少 3 秒，峰值角速度不超过 0.25 rad/s。
        approach_s = max(3.0, np.abs(traj[0] - start).max() * np.pi / (2 * .25))
        warm = max(1, int(np.ceil(approach_s / frame_dt)))
        for i in range(warm):
            if v is not None and not v.is_running():
                break
            a = 0.5 * (1 - np.cos(np.pi * (i + 1) / warm))
            tgt = (1 - a) * start + a * traj[0]
            q_target[self.aids] = tgt
            frame()

        log = []
        for q_t in traj:
            if v is not None and not v.is_running():
                break
            q_target[self.aids] = q_t
            frame()
            log.append(self.tip_pos())

        if v is not None:
            v.close()
        return np.array(log)


# ==================================================================
def make_line(arm, length, forward, center_dz, n):
    p0 = arm.fk(arm.q0)
    x, y = p0[0] + forward, p0[1]
    zc = p0[2] + center_dz
    zs = np.linspace(zc - length / 2, zc + length / 2, n)
    return [np.array([x, y, z]) for z in zs], p0


def solve(arm, pts):
    qs, errs, prev = [], [], None
    for p in pts:
        q, e = arm.ik(p, seed=prev, q_ref=prev)
        qs.append(q)
        errs.append(e)
        prev = q
    return np.array(qs), np.array(errs)


def densify(qs, length, speed, dt):
    qs = np.asarray(qs, float)
    n = len(qs)
    if n < 2:
        return qs.copy()
    avg_v = max(speed, 1e-4) * 2.0 / np.pi
    steps = max(int(round((length / avg_v) / dt)), 2)
    s = 0.5 * (1 - np.cos(np.pi * np.linspace(0, 1, steps)))
    out = np.empty((steps, qs.shape[1]))
    for j in range(qs.shape[1]):
        out[:, j] = np.interp(s * (n - 1), np.arange(n), qs[:, j])
    return out


def main():
    ap = argparse.ArgumentParser(
        description="G1-D 左手竖直线扫描（G1-D URDF / 官方示例 PD）")
    ap.add_argument("--model", default=str(MODEL))
    ap.add_argument("--check", action="store_true", help="只算轨迹")
    ap.add_argument("--no-viewer", action="store_true", help="不开窗口")
    ap.add_argument("--fast", action="store_true", help="仿真不等待墙钟时间")
    ap.add_argument("--init", choices=("hang", "zero"), default="hang",
                    help="仿真初始姿态；默认垂臂，zero 仅用于诊断")
    ap.add_argument("--waist-mode", choices=("fixed", "pd"), default="fixed",
                    help="fixed=约束锁腰的手臂台架；pd=腰部也用 SDK 原值 PD")
    ap.add_argument("--length", type=float, default=0.50)
    ap.add_argument("--forward", type=float, default=0.02,
                    help="相对 G1-D 零位的前伸距离 m")
    ap.add_argument("--center-dz", type=float, default=0.20)
    ap.add_argument("--points", type=int, default=101)
    ap.add_argument("--speed", type=float, default=0.03, help="m/s")
    ap.add_argument("--cycles", type=int, default=2)
    ap.add_argument("--dt", type=float, default=0.02)
    args = ap.parse_args()
    if (args.length <= 0 or args.speed <= 0 or args.dt <= 0
            or args.points < 2 or args.cycles < 1):
        ap.error("length、speed、dt 必须为正，points >= 2，cycles >= 1")

    print("=" * 62)
    print("G1-D 左手竖直线扫描（G1-D URDF / SDK 原值 PD）")
    print("=" * 62)

    arm = Arm(args.model)
    p0 = arm.fk(arm.q0)
    print(f"模型: {args.model}")
    print(f"仿真初始姿态: {args.init}；从该姿态平滑接近工作点")
    print(f"腰部模式: {args.waist_mode}；底盘/升降柱/手指固定在 URDF 零位")
    print(f"左臂 PD（G1-D SDK 示例原值）: KP={arm.KP.tolist()}")
    print(f"                           KD={arm.KD.tolist()}")
    print(f"左臂 DDS 槽位: {[JOINT_SLOTS[n] for n in ARM_JOINTS]}")
    print(f"定义扫描线的零位末端参考: ({p0[0]:+.4f}, {p0[1]:+.4f}, {p0[2]:+.4f})")

    pts, _ = make_line(arm, args.length, args.forward, args.center_dz,
                       args.points)
    print(f"竖直线: x={pts[0][0]:+.4f} y={pts[0][1]:+.4f} "
          f"z={pts[0][2]:.4f} → {pts[-1][2]:.4f} ({args.length*100:.0f} cm)")

    qs, errs = solve(arm, pts)
    seg = float(np.linalg.norm(pts[-1] - pts[0]))
    traj = densify(qs, seg, args.speed, args.dt)
    print(f"IK: {args.points} 点, 最大误差 {errs.max()*1000:.3f} mm")
    print(f"轨迹: {len(traj)} 帧 @ {1/args.dt:.0f} Hz, "
          f"{args.speed*100:.1f} cm/s")

    dq = np.abs(np.diff(traj, axis=0)).max()
    print(f"单帧最大关节变化: {dq:.4f} rad")
    margin = np.minimum(traj - arm.lo, arm.hi - traj).min()
    print(f"最小关节限位余量: {np.degrees(margin):.2f}°")
    if not np.all(np.isfinite(traj)) or errs.max() > 1e-3 or dq > 0.08:
        print("轨迹检查失败：需要 IK 误差 <= 1 mm、单帧变化 <= 0.08 rad")
        return 1

    if args.check:
        print("\n[--check] 只算轨迹，不播放")
        return 0

    seq = []
    for _ in range(args.cycles):
        seq.extend(traj)
        seq.extend(traj[::-1])
    seq = np.array(seq)
    print(f"\n播放 {len(seq)} 帧, {args.cycles} 个往返")

    log = arm.play(seq, dt=args.dt, viewer=not args.no_viewer,
                   realtime=not args.fast, waist_mode=args.waist_mode,
                   initial_q=initial_pose(args.init))

    if len(log) > 5:
        L = log[5:]
        print("\n实际末端轨迹:")
        print(f"  x 范围 {L[:,0].min():+.4f} ~ {L[:,0].max():+.4f}  "
              f"漂移 {(L[:,0].max()-L[:,0].min())*1000:.2f} mm")
        print(f"  y 范围 {L[:,1].min():+.4f} ~ {L[:,1].max():+.4f}  "
              f"漂移 {(L[:,1].max()-L[:,1].min())*1000:.2f} mm")
        print(f"  z 范围 {L[:,2].min():+.4f} ~ {L[:,2].max():+.4f}  "
              f"行程 {(L[:,2].max()-L[:,2].min())*100:.2f} cm")
        planned = np.array([arm.fk(q) for q in seq[5:len(log)]])
        error = np.linalg.norm(L - planned, axis=1) * 1000
        print(f"  相对规划轨迹跟踪误差: RMS={np.sqrt(np.mean(error**2)):.2f} mm, "
              f"最大={error.max():.2f} mm")
        print(f"  相对目标直线 x/y 最大偏差: "
              f"{np.abs(L[:, 0] - pts[0][0]).max()*1000:.2f} / "
              f"{np.abs(L[:, 1] - pts[0][1]).max()*1000:.2f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())

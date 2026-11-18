"""沿固定关节路径的局部时间规划，不改变路径、不放宽关节限速。

变量 x=(ds/dt)^2；每格采用恒定路径加速度 u=(x[i+1]-x[i])/(2*ds)。
关节加速度 q_ss*x+q_s*u 对相邻 x 是线性的。通过稀疏 LP 选择局部速度，
再解析检查格内的关节速度/加速度极值。LP 最大化离散速度平方之和，
不宣称连续时间全局最优，也没有施加 jerk 约束。
"""
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import linprog
from scipy.sparse import coo_matrix


class LocalPathMotion:
    def __init__(self, positions, q, speed, joint_speed, joint_acceleration, cycles=1,
                 endpoint_slopes=None):
        positions, q = np.asarray(positions, float), np.asarray(q, float)
        if (positions.ndim != 1 or len(positions) < 3 or q.ndim != 2
                or q.shape[0] != len(positions) or np.any(np.diff(positions) <= 0)
                or not np.isfinite(positions).all() or not np.isfinite(q).all()
                or not np.isfinite([speed, joint_speed, joint_acceleration]).all()
                or min(speed, joint_speed, joint_acceleration) <= 0
                or not isinstance(cycles, int) or cycles < 1):
            raise ValueError("局部时间规划的路径、速度或循环数不合法")
        bc = ("not-a-knot" if endpoint_slopes is None else
              ((1, endpoint_slopes[0]), (1, endpoint_slopes[1])))
        self.path = CubicSpline(positions, q, bc_type=bc)
        self.cycles = cycles
        # 每个路径样条节点都在规划网格内，保证后续格内极值分析使用单个三次多项式。
        self.grid = np.concatenate([np.linspace(a, b, max(2, int(np.ceil((b-a)/.001))+1))[:-1]
                                    for a, b in zip(positions, positions[1:])]
                                   + [positions[-1:]])
        ds = np.diff(self.grid)
        count, joints = len(self.grid), q.shape[1]
        qs = self.path(self.grid, 1)
        caps = np.minimum((speed*.98)**2,
                          np.min(np.divide((joint_speed*.98)**2, qs**2,
                                           out=np.full_like(qs, np.inf), where=np.abs(qs)>1e-12), axis=1))
        left, right = [], []
        for fraction in (0., .5, 1.):
            s = self.grid[:-1] + fraction*ds
            slope = self.path(s, 1)/(2*ds[:, None])
            curve = self.path(s, 2)
            left.append(curve*(1-fraction)-slope)
            right.append(curve*fraction+slope)
        left, right = np.concatenate(left, axis=1).ravel(), np.concatenate(right, axis=1).ravel()
        cells = np.repeat(np.arange(count-1), joints*3)
        rows = np.arange(len(cells))
        matrix = coo_matrix((np.r_[left, right, -left, -right],
                             (np.r_[rows, rows, rows+len(rows), rows+len(rows)],
                              np.r_[cells, cells+1, cells, cells+1])),
                            shape=(2*len(rows), count)).tocsr()
        bounds = np.column_stack((np.zeros(count), caps))
        bounds[[0, -1], 1] = 0.  # 两端停稳，往返与接近/返回都保持位置和速度连续。
        result = linprog(-np.ones(count), A_ub=matrix,
                         b_ub=np.full(matrix.shape[0], joint_acceleration*.98),
                         bounds=bounds, method="highs")
        if not result.success or result.x is None or not np.isfinite(result.x).all():
            raise ValueError(f"局部时间规划失败: {result.message}")
        x = np.maximum(result.x, 0.)
        self.path_velocity = np.sqrt(x)
        speed_sum = self.path_velocity[:-1] + self.path_velocity[1:]
        if np.any(speed_sum <= 1e-12):
            raise ValueError("路径局部无法通过，需要调整扫描范围或 IK 分支")
        self.path_acceleration = np.diff(x)/(2*ds)
        self.times = np.r_[0., np.cumsum(2*ds/speed_sum)]
        raw_v, raw_a = self._exact_extrema(x)
        # 修正离散 LP 在格内可能产生的小超限；大倍率表示应细化几何/网格而非静默变慢。
        self.certification_scale = max(1., raw_v/joint_speed,
                                       np.sqrt(raw_a/joint_acceleration)) * 1.000001
        if self.certification_scale > 1.05:
            raise ValueError("格内极值验证要求超过 5% 的修正，需要细化路径，未生成运动指令")
        self.one_way_duration = float(self.times[-1] * self.certification_scale)
        self.duration = self.one_way_duration * 2 * cycles
        self.peak_joint_speed = raw_v/self.certification_scale
        self.peak_joint_acceleration = raw_a/self.certification_scale**2
        self.peak_path_speed = float(self.path_velocity.max()/self.certification_scale)

    def _exact_extrema(self, x):
        """q_s 为二次式，x 为一次式；q_tt 为二次式，可求出全部速度/加速度极值。"""
        s, h = self.grid[:-1], np.diff(self.grid)[:, None]
        p, c, d = self.path(s, 1), self.path(s, 2), self.path(s, 3)
        u, x0 = self.path_acceleration[:, None], x[:-1, None]
        aa, bb, cc = 2.5*d*u, d*x0+3*c*u, c*x0+p*u
        max_v = max_a = 0.
        def evaluate(offset):
            nonlocal max_v, max_a
            offset = np.clip(np.broadcast_to(offset, p.shape), 0., h)
            velocity = (p+c*offset+.5*d*offset**2)*np.sqrt(np.maximum(0., x0+2*u*offset))
            acceleration = aa*offset**2+bb*offset+cc
            max_v = max(max_v, float(np.abs(velocity).max()))
            max_a = max(max_a, float(np.abs(acceleration).max()))
        evaluate(0.)
        evaluate(h)
        evaluate(np.divide(-bb, 2*aa, out=np.zeros_like(bb), where=np.abs(aa)>1e-14))
        discriminant = bb**2-4*aa*cc
        root = np.sqrt(np.maximum(0., discriminant))
        stable = -.5*(bb+np.where(bb >= 0., root, -root))
        for candidate in (np.divide(stable, aa, out=np.zeros_like(bb), where=np.abs(aa)>1e-14),
                          np.divide(cc, stable, out=np.zeros_like(bb), where=np.abs(stable)>1e-14)):
            candidate = np.where(discriminant >= 0., candidate, 0.)
            evaluate(candidate)
        evaluate(np.divide(-cc, bb, out=np.zeros_like(bb), where=np.abs(bb)>1e-14))
        return max_v, max_a

    def sample(self, t, derivative=0):
        if derivative not in (0, 1, 2):
            raise ValueError("只支持位置、速度和加速度")
        elapsed = np.clip(np.asarray(t, float), 0., self.duration)
        phase = np.remainder(elapsed, 2*self.one_way_duration)
        reverse = phase > self.one_way_duration
        local = np.where(reverse, 2*self.one_way_duration-phase, phase)/self.certification_scale
        cell = np.clip(np.searchsorted(self.times, local, side="right")-1, 0, len(self.times)-2)
        dt = local-self.times[cell]
        u, v = self.path_acceleration[cell], self.path_velocity[cell]
        s = np.clip(self.grid[cell]+v*dt+.5*u*dt**2, self.grid[0], self.grid[-1])
        if derivative == 0:
            return self.path(s)
        ds = (v+u*dt)/self.certification_scale
        if derivative == 1:
            return self.path(s, 1)*(ds*np.where(reverse, -1., 1.))[..., None]
        return (self.path(s, 2)*ds[..., None]**2
                + self.path(s, 1)*(u/self.certification_scale**2)[..., None])

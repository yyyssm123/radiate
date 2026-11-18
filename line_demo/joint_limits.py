"""模型关节限位与只读诊断；不创建 DDS 通道。"""
import xml.etree.ElementTree as ET

import numpy as np

from g1d_config import CONTROLLED_JOINTS, JOINT_SLOTS, URDF


def joint_limits():
    joints = {j.get("name"): j for j in ET.parse(URDF).getroot().findall("joint")}
    return tuple(np.array([float(joints[n].find("limit").get(attr))
                           for n in CONTROLLED_JOINTS])
                 for attr in ("lower", "upper", "effort"))


def joint_limit_violations(q, margin=0., bounds=None):
    """保留实测角度，列出超出范围的关节；margin 是目标的限位余量。"""
    q = np.asarray(q, float)
    if q.shape != (len(CONTROLLED_JOINTS),) or not np.isfinite(q).all():
        raise ValueError("限位诊断需要 16 个有限关节角")
    lo, hi = joint_limits()[:2] if bounds is None else bounds
    lower, upper = lo + margin, hi - margin
    rows = []
    for i in np.flatnonzero((q < lower) | (q > upper)):
        below = q[i] < lower[i]
        excess = float(lower[i]-q[i] if below else q[i]-upper[i])
        rows.append({"slot": JOINT_SLOTS[CONTROLLED_JOINTS[i]],
                     "joint": CONTROLLED_JOINTS[i], "q_rad": float(q[i]),
                     "q_deg": float(np.rad2deg(q[i])),
                     "lower_rad": float(lo[i]), "upper_rad": float(hi[i]),
                     "lower_deg": float(np.rad2deg(lo[i])),
                     "upper_deg": float(np.rad2deg(hi[i])),
                     "required_margin_rad": margin,
                     "side": "lower" if below else "upper",
                     "excess_rad": excess, "excess_deg": float(np.rad2deg(excess))})
    return rows


def format_joint_limit_violations(rows):
    return "; ".join(
        f"slot {r['slot']} {r['joint']}: q={r['q_rad']:+.6f} rad ({r['q_deg']:+.2f}°), "
        f"URDF=[{r['lower_rad']:+.6f}, {r['upper_rad']:+.6f}] rad "
        f"([{r['lower_deg']:+.2f}°, {r['upper_deg']:+.2f}°]), "
        f"{'下' if r['side'] == 'lower' else '上'}限"
        f"{'（含目标余量）' if r['required_margin_rad'] else ''}"
        f"超出 {r['excess_rad']:.6f} rad ({r['excess_deg']:.2f}°)"
        for r in rows)

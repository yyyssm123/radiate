#!/usr/bin/env python3
"""离线重算固定扫描点位，输出待复核的配置，不连接 DDS。"""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "line_demo"))

from fixed_scan import LENGTH, plan_transfer
from g1d_config import ARM_JOINTS, HANG_LEFT, MODEL, initial_pose
from g1d_transport import LEFT
from path_timing import LocalPathMotion
from real_motion import Guard, Limits, scan_plan, validate_motion


def build_profile():
    captured = np.array(initial_pose())
    limits = Limits(locked_waist_pitch=0.)
    segments, geometry, line = scan_plan(captured, limits, speed=.10)
    arm, guard = geometry.arm, Guard(limits)
    distances = np.linspace(0., LENGTH, 101)
    points = np.linspace(line["target_bottom_m"], line["target_top_m"], len(distances))
    seed_path = segments[1][1].path(distances)
    # 固定采用参考路径离底端 10 cm 处的腕部朝向，肩 yaw 跟随参考路径以选定冗余解。
    rotation = Rotation.from_matrix(arm.pose(seed_path[20])[1])
    inverse_rotation = rotation.inv()
    lo, hi = arm.lo + .025, arm.hi - .025
    angles = seed_path[0]
    path = []
    for point, seed in zip(points, seed_path):
        def residual(q):
            position, orientation = arm.pose(q)
            error = (inverse_rotation * Rotation.from_matrix(orientation)).as_rotvec()
            return np.r_[position - point, .22 * error, .01 * (q[2] - seed[2])]

        solution = least_squares(residual, np.clip(angles, lo+1e-7, hi-1e-7), bounds=(lo, hi),
                                 gtol=1e-11, ftol=1e-11, xtol=1e-11, max_nfev=55)
        if not solution.success or np.linalg.norm(solution.fun[:6]) > 5e-5:
            raise ValueError(f"固定腕部朝向的 IK 失败，扫描点 {len(path)}")
        angles = solution.x
        path.append(angles)
    nodes = np.round(path, 12)
    motion = LocalPathMotion(distances, nodes, .10, limits.joint_speed, limits.joint_acceleration)
    validate_motion(motion, guard, geometry, captured, points[0, :2], .10,
                    stage="scan", tip_rotation=rotation.as_matrix())
    plan_transfer(captured[LEFT], nodes[0], limits, guard, geometry, captured, "approach")
    plan_transfer(nodes[0], np.asarray(HANG_LEFT), limits, guard, geometry, captured, "return")
    return nodes.tolist()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="待复核 JSON 文件，必须是新文件")
    args = parser.parse_args()
    rows = build_profile()
    header = {"joint_names": ARM_JOINTS, "scene_sha256": sha256(MODEL.read_bytes()).hexdigest()}
    with args.output.open("x") as output:
        output.write(json.dumps(header, indent=2)[:-2] + ',\n  "left_q_rad": [\n')
        output.write(",\n".join("    " + json.dumps(row) for row in rows))
        output.write("\n  ]\n}\n")


if __name__ == "__main__":
    main()

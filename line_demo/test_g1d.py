"""G1-D 离线回归：SDK 参数、URDF 几何、力矩限制和仿真约束。"""
import json
from pathlib import Path
import re
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from g1d_config import (MODEL, URDF, SDK_EXAMPLE, KP_29, KD_29, ARM_JOINTS,
                        JOINT_SLOTS, CONTROLLED_JOINTS, actuator_name)
from line_servo_demo import Arm


class TestG1D(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original = ET.parse(URDF).getroot()
        # 独立导入完整 URDF，比较修改后的仿真模型是否改变左臂几何。
        ref = ET.parse(URDF).getroot()
        ref.find("mujoco/compiler").set("fusestatic", "false")
        ref.find("mujoco/compiler").set("meshdir", str(URDF.parent / "meshes"))
        for mesh in ref.iter("mesh"):
            mesh.set("filename", str(URDF.parent / "meshes" / Path(mesh.get("filename")).name))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "reference.urdf"
            ET.ElementTree(ref).write(path, encoding="utf-8")
            cls.reference = mujoco.MjModel.from_xml_path(str(path))

    def setUp(self):
        self.arm = Arm(MODEL)
        self.arm._prepare_pd()

    def test_gains_and_slots_match_g1d_sdk(self):
        sdk = re.sub(r"//[^\n]*", "", SDK_EXAMPLE.read_text())
        for name, values in (("Kp", KP_29), ("Kd", KD_29)):
            body = re.search(r"\b" + name + r"\s*\{([^}]+)\}", sdk).group(1)
            parsed = [float(v.strip().rstrip("f")) for v in body.split(",") if v.strip()]
            self.assertEqual(parsed, values)
        self.assertEqual([JOINT_SLOTS[n] for n in ARM_JOINTS], list(range(15, 22)))
        self.assertEqual(set(JOINT_SLOTS.values()), {12, 14, *range(15, 29)})
        axes = {j.get("name"): j.find("axis").get("xyz")
                for j in self.original.findall("joint") if j.find("axis") is not None}
        self.assertEqual(axes["Yaw_Joint"], "0 1 0")
        self.assertEqual(JOINT_SLOTS["Yaw_Joint"], 14)
        self.assertEqual(axes["torso_Joint"], "0 0 1")
        self.assertEqual(JOINT_SLOTS["torso_Joint"], 12)

    def test_geometry_matches_original_urdf(self):
        ref = self.reference
        data = mujoco.MjData(ref)
        ids = [mujoco.mj_name2id(ref, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ARM_JOINTS]
        tip = mujoco.mj_name2id(ref, mujoco.mjtObj.mjOBJ_BODY, "left_wrist_yaw_link")
        for q in (np.zeros(7), [-0.4, 0.3, 0.5, 1.1, -0.3, 0.2, -0.2]):
            data.qpos[ref.jnt_qposadr[ids]] = q
            mujoco.mj_kinematics(ref, data)
            p, rotation = self.arm.pose(q)
            np.testing.assert_allclose(p, data.xpos[tip], atol=2e-6)
            np.testing.assert_allclose(rotation, data.xmat[tip].reshape(3, 3), atol=2e-6)

    def test_position_jacobian_matches_independent_finite_difference(self):
        arm = Arm(MODEL)
        q = np.array([-.4, .3, .5, 1.1, -.3, .2, -.2])
        eps = 1e-6
        numerical = np.column_stack([(arm.fk(q+eps*np.eye(7)[i])-arm.fk(q-eps*np.eye(7)[i]))/(2*eps)
                                     for i in range(7)])
        np.testing.assert_allclose(arm.position_jacobian(q), numerical, atol=1e-8)

    def test_torque_bounds_come_from_urdf(self):
        arm = self.arm
        limits = {j.get("name"): float(j.find("limit").get("effort"))
                  for j in self.original.findall("joint") if j.get("name") in CONTROLLED_JOINTS}
        for name, limit in limits.items():
            a = mujoco.mj_name2id(arm.m, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name(name))
            j = arm.m.actuator_trnid[a, 0]
            np.testing.assert_allclose(arm.m.actuator_ctrlrange[a], [-limit, limit])
            np.testing.assert_allclose(arm.m.actuator_forcerange[a], [-limit, limit])
            np.testing.assert_allclose(arm.m.jnt_actfrcrange[j], [-limit, limit])
        target = arm.d.qpos[arm._act_qadr].copy() + 1000
        tau = arm._pd_torque(target)
        self.assertEqual(tau[arm.aids[0]], 25.0)
        self.assertEqual(tau[arm.aids[5]], 5.0)

    def test_pd_uses_joint_mapping_and_damping(self):
        arm = self.arm
        target = arm.d.qpos[arm._act_qadr].copy()
        shoulder = arm.aids[0]
        target[shoulder] += 0.02
        arm.d.qvel[arm._act_dadr[shoulder]] = 0.1
        self.assertAlmostEqual(arm._pd_torque(target)[shoulder], 0.7)
        waist = mujoco.mj_name2id(arm.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "torso")
        target[waist] += 0.02
        self.assertAlmostEqual(arm._pd_torque(target)[waist], 1.2)

    def test_fixed_fixture_has_no_initial_penetration(self):
        arm = self.arm
        self.assertEqual(arm.m.nv, 16)
        self.assertFalse(np.any(arm.m.jnt_type == mujoco.mjtJoint.mjJNT_FREE))
        mujoco.mj_forward(arm.m, arm.d)
        self.assertEqual(arm.d.ncon, 0)
        self.assertEqual(arm.m.neq, 2)
        self.assertTrue(arm.d.eq_active.all())

    def test_play_keeps_torque_configuration(self):
        arm = self.arm
        fields = ("actuator_gainprm", "actuator_biastype", "actuator_ctrlrange",
                  "actuator_forcerange", "jnt_actfrcrange")
        before = {field: getattr(arm.m, field).copy() for field in fields}
        log = arm.play(np.zeros((2, 7)), viewer=False, realtime=False)
        self.assertTrue(np.isfinite(log).all())
        waist = [mujoco.mj_name2id(arm.m, mujoco.mjtObj.mjOBJ_JOINT, n)
                 for n in ("Yaw_Joint", "torso_Joint")]
        self.assertLess(np.abs(arm.d.qpos[arm.m.jnt_qposadr[waist]]).max(), 0.002)
        for field, expected in before.items():
            np.testing.assert_array_equal(getattr(arm.m, field), expected)


if __name__ == "__main__":
    unittest.main(verbosity=2)

"""现场拆除左灵巧手后的装配、碰撞及轨迹回归；不连接 DDS。"""
import hashlib
import io
import json
from contextlib import redirect_stdout
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from build_model import build
from g1d_config import ROOT, MODEL, URDF, CONTROLLED_JOINTS, load_assembly
from g1d_transport import ControlFault, LEFT
from line_servo_demo import Arm
from mujoco_sim import PhysicsBridge
from real_motion import Geometry, Guard, Limits, scan_plan


class TestEndEffectorAssembly(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        cls.tmp = Path(cls.temp.name)
        cls.source_hash = hashlib.sha256(URDF.read_bytes()).hexdigest()
        config = cls.tmp / "full.json"
        config.write_text(json.dumps({"left_hand": "dex3", "right_hand": "dex3"}))
        cls.full_path = cls.tmp / "full" / "scene.xml"
        with redirect_stdout(io.StringIO()):
            cls.full_manifest = build(assembly_path=config, output=cls.full_path)
        cls.full = mujoco.MjModel.from_xml_path(str(cls.full_path))
        cls.current = mujoco.MjModel.from_xml_path(str(MODEL))
        sample = json.loads((ROOT / "references/g1d_left_hand_removed_start_sample.json").read_text())
        if sample["joint_names"] != CONTROLLED_JOINTS:
            raise AssertionError("实测姿态关节顺序不匹配")
        cls.q = np.array(sample["q_rad"])

    def test_installed_parts_and_removed_mass_match_real_assembly(self):
        model, full = self.current, self.full
        self.assertEqual(load_assembly(), {"left_hand": "none", "right_hand": "dex3"})
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)]
        self.assertFalse(any(n and n.startswith("left_hand_") for n in names))
        self.assertIn("left_wrist_yaw_link", names)
        self.assertIn("right_hand_palm_link", names)
        self.assertEqual((model.nq, model.nv, model.nu), (16, 16, 16))
        source = ET.parse(URDF).getroot()
        removed_mass = sum(float(link.find("inertial/mass").get("value"))
                           for link in source.findall("link")
                           if link.get("name").startswith("left_hand_"))
        self.assertGreater(removed_mass, .5)
        self.assertAlmostEqual(float(full.body_mass.sum()-model.body_mass.sum()), removed_mass, places=5)
        for name in names:
            if name and name.startswith("right_hand_"):
                i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
                j = mujoco.mj_name2id(full, mujoco.mjtObj.mjOBJ_BODY, name)
                np.testing.assert_array_equal(model.body_inertia[i], full.body_inertia[j])

    def test_full_hand_profile_can_be_rebuilt_without_editing_source_urdf(self):
        self.assertGreaterEqual(mujoco.mj_name2id(self.full, mujoco.mjtObj.mjOBJ_BODY,
                                               "left_hand_palm_link"), 0)
        manifest = json.loads(self.full_path.with_name("simulation.json").read_text())
        self.assertEqual(manifest["omitted_links"], [])
        self.assertEqual(hashlib.sha256(URDF.read_bytes()).hexdigest(), self.source_hash)
        official_record = json.loads((ROOT / "models/source.json").read_text())
        self.assertEqual(self.source_hash, official_record["urdf_sha256"])

    def test_right_hand_can_be_removed_independently(self):
        config = self.tmp / "right_removed.json"
        config.write_text(json.dumps({"left_hand": "dex3", "right_hand": "none"}))
        output = self.tmp / "right_removed" / "scene.xml"
        with redirect_stdout(io.StringIO()):
            build(assembly_path=config, output=output)
        model = mujoco.MjModel.from_xml_path(str(output))
        self.assertEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_hand_palm_link"), -1)
        self.assertGreaterEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                               "left_hand_palm_link"), 0)
        self.assertGreaterEqual(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                               "right_wrist_yaw_link"), 0)

    def test_measured_pose_and_full_scan_pass_without_nonexistent_left_hand(self):
        limits = Limits(locked_waist_pitch=0.)
        Guard(limits).position(self.q)
        Geometry(self.q).collision(self.q)
        segments, _, report = scan_plan(self.q, limits, speed=.1)
        self.assertEqual(report["end_effectors"], load_assembly())
        self.assertLess(report["scan_s"], 25.)
        self.assertGreater(report["collision_branches_rejected"], 0)
        np.testing.assert_allclose(segments[0][1].sample(0), self.q[LEFT], atol=1e-7)
        np.testing.assert_allclose(segments[-1][1].sample(segments[-1][1].duration),
                                   self.q[LEFT], atol=1e-7)

    def test_original_hand_causes_the_reproduced_start_collision(self):
        with patch("real_motion.MODEL", self.full_path):
            geometry = Geometry(self.q)
        with self.assertRaisesRegex(ControlFault, "left_hand"):
            geometry.collision(self.q)

    def test_shoulder_collision_protection_is_still_active(self):
        q = self.q.copy()
        q[CONTROLLED_JOINTS.index("left_shoulder_roll_joint")] = -.5
        with self.assertRaisesRegex(ControlFault, "模型自碰撞"):
            Geometry(q).collision(q)

    def test_configuration_change_requires_model_rebuild(self):
        with patch("g1d_config.load_assembly", return_value={"left_hand": "dex3", "right_hand": "dex3"}):
            for constructor in (lambda: Arm(MODEL), PhysicsBridge):
                with self.assertRaisesRegex(ValueError, "build_model"):
                    constructor()

    def test_unknown_or_incomplete_assembly_is_rejected(self):
        path = self.tmp / "invalid.json"
        for config in ({"left_hand": "probe", "right_hand": "dex3"},
                       {"left_hand": "none"}, []):
            path.write_text(json.dumps(config))
            with self.assertRaises(ValueError):
                load_assembly(path)


if __name__ == "__main__":
    unittest.main()

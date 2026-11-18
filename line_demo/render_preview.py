#!/usr/bin/env python3
"""离屏渲染项目内的 G1-D 模型，不启动 DDS 或窗口。"""
import os
os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
from PIL import Image

from g1d_config import MODEL, ROOT, CONTROLLED_JOINTS, initial_pose


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    model.vis.global_.offwidth = 960
    model.vis.global_.offheight = 720
    data = mujoco.MjData(model)
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in CONTROLLED_JOINTS]
    data.qpos[model.jnt_qposadr[ids]] = initial_pose()
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.lookat[:] = [0.1, 0.0, 0.65]
    camera.distance = 2.2
    camera.azimuth = 135
    camera.elevation = -10
    with mujoco.Renderer(model, height=720, width=960) as renderer:
        renderer.update_scene(data, camera)
        path = ROOT / "out" / "g1d_preview.png"
        path.parent.mkdir(exist_ok=True)
        Image.fromarray(renderer.render()).save(path)
    print(path)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""从项目内 G1-D 原始 URDF 构建固定底座的手臂验证场景。"""
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from g1d_config import MODEL, URDF, CONTROLLED_JOINTS, actuator_name, load_assembly


def configure_end_effectors(root, assembly):
    """在转换前移除未安装的手部子树，同时移除其惯量与碰撞几何。"""
    links = {link.get("name") for link in root.findall("link")}
    children = {}
    for joint in root.findall("joint"):
        children.setdefault(joint.find("parent").get("link"), []).append(
            joint.find("child").get("link"))
    removed = set()
    for side in ("left", "right"):
        palm = f"{side}_hand_palm_link"
        if palm not in links:
            raise ValueError(f"源 URDF 缺少末端根连杆: {palm}")
        if assembly[f"{side}_hand"] == "none":
            pending = [palm]
            while pending:
                name = pending.pop()
                if name not in removed:
                    removed.add(name)
                    pending.extend(children.get(name, []))
    for element in list(root):
        if element.tag == "link" and element.get("name") in removed:
            root.remove(element)
        elif element.tag == "joint" and (
                element.find("parent").get("link") in removed
                or element.find("child").get("link") in removed):
            root.remove(element)
    return sorted(removed)


def build(assembly_path=None, output=MODEL):
    output = Path(output)
    assembly = load_assembly(assembly_path)
    root = ET.parse(URDF).getroot()
    removed_links = configure_end_effectors(root, assembly)
    compiler = root.find("mujoco/compiler")
    compiler.set("meshdir", str(URDF.parent / "meshes"))
    compiler.set("discardvisual", "false")
    compiler.set("fusestatic", "false")
    for mesh in root.iter("mesh"):
        mesh.set("filename", str(URDF.parent / "meshes" / Path(mesh.get("filename")).name))

    # 台架条件：底盘、车轮、升降柱和手指固定在 URDF 零位。
    # 只保留 SDK lowcmd 定义的 16 个腰/臂关节，避免给升降柱编造 PD。
    # 固定关节仍保留其连杆、网格和惯量；原始 URDF 文件保持原样。
    for joint in root.findall("joint"):
        if joint.get("name") not in CONTROLLED_JOINTS:
            joint.set("type", "fixed")

    with tempfile.TemporaryDirectory(prefix="g1d_model_") as tmp:
        prepared = Path(tmp) / "g1d_fixed.urdf"
        ET.ElementTree(root).write(prepared, encoding="utf-8", xml_declaration=True)
        model = mujoco.MjModel.from_xml_path(str(prepared))
        converted = Path(tmp) / "converted.xml"
        mujoco.mj_saveLastXML(str(converted), model)
        scene = ET.parse(converted).getroot()

    scene.set("model", "G1-D fixed-base arm validation")
    opt = scene.find("option")
    if opt is None:
        opt = ET.SubElement(scene, "option")
    opt.set("timestep", "0.001")
    opt.set("integrator", "implicitfast")

    # MuJoCo 默认把 mesh 坐标局部化。保留转换生成的位姿与惯量，只改文件路径。
    compiler = scene.find("compiler")
    meshdir = ("meshes" if output.parent.resolve() == URDF.parent.resolve()
               else str(URDF.parent / "meshes"))
    compiler.set("meshdir", meshdir)
    for mesh in scene.findall("asset/mesh"):
        mesh.set("file", Path(mesh.get("file")).name)

    limits = {j.get("name"): float(j.find("limit").get("effort"))
              for j in root.findall("joint") if j.get("name") in CONTROLLED_JOINTS}
    for joint in scene.findall(".//joint"):
        name = joint.get("name")
        if name in limits:
            joint.set("actuatorfrcrange", f"{-limits[name]} {limits[name]}")
            # G1 官方 MuJoCo 模型也使用 0.01；这是转子惯量的仿真假设，
            # 不属于 G1-D URDF 的连杆惯量或 SDK 的 KP/KD。
            joint.set("armature", "0.01")

    actuators = ET.SubElement(scene, "actuator")
    for name in CONTROLLED_JOINTS:
        bound = f"{-limits[name]} {limits[name]}"
        ET.SubElement(actuators, "motor", name=actuator_name(name), joint=name,
                      ctrllimited="true", ctrlrange=bound,
                      forcelimited="true", forcerange=bound)

    # 相邻连杆的关节壳体本来就相交。固定底座的 URDF 导入后有些连杆
    # 与 world 属于同一焊接组，MuJoCo 的默认父子过滤不覆盖它们，显式排除。
    # 非相邻连杆之间的自碰撞仍然启用。
    contact = ET.SubElement(scene, "contact")
    for joint in root.findall("joint"):
        ET.SubElement(contact, "exclude", body1=joint.find("parent").get("link"),
                      body2=joint.find("child").get("link"))

    # 默认手臂台架约束：锁定腰部，避免把尚未辨识的腰部动力学混进臂部误差。
    # 两个关节和执行器仍保留；播放器的 --waist-mode pd 会关闭这两项约束。
    equality = ET.SubElement(scene, "equality")
    for name in ("Yaw_Joint", "torso_Joint"):
        ET.SubElement(equality, "joint", name=f"hold_{name}", joint1=name,
                      polycoef="0 0 0 0 0", solref="0.002 1", solimp="0.999 0.999 0.001")

    world = scene.find("worldbody")
    ET.SubElement(world, "light", name="key", pos="1 -1 3", dir="-0.3 0.3 -1")
    # URDF 的零高度是底盘坐标原点。地面放在轮胎最低点下，不抬高或改写模型原点。
    # 用几何的真实顶点求最低点，避免用包围球半径误判底盘高度。
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    bottom = np.inf
    for gid in range(model.ngeom):
        if model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = model.geom_dataid[gid]
        adr, count = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        vertices = model.mesh_vert[adr:adr + count]
        rz = data.geom_xmat[gid].reshape(3, 3)[2]
        bottom = min(bottom, float((vertices @ rz + data.geom_xpos[gid, 2]).min()))
    ET.SubElement(world, "geom", name="floor", type="plane", size="3 3 0.1",
                  pos=f"0 0 {bottom - 0.001:.8f}", rgba="0.22 0.25 0.28 1")
    ET.SubElement(scene, "visual")

    ET.indent(scene, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(scene).write(output, encoding="utf-8", xml_declaration=True)
    compiled = mujoco.MjModel.from_xml_path(str(output))
    check = mujoco.MjData(compiled)
    mujoco.mj_forward(compiled, check)
    manifest = {
        "source_urdf": URDF.name,
        "end_effectors": assembly,
        "omitted_links": removed_links,
        "fixed": "base, wheels, two lift stages, installed fingers (URDF zero pose)",
        "controlled_joints": CONTROLLED_JOINTS,
        "torque_limits_Nm_from_urdf": limits,
        "armature_kg_m2_assumption": 0.01,
        "timestep_s": compiled.opt.timestep,
        "ground_z_m": bottom - 0.001,
        "nq": compiled.nq, "nv": compiled.nv, "nu": compiled.nu,
        "initial_contacts": check.ncon,
        "waist_constraint_default": "fixed; disable using --waist-mode pd",
    }
    output.with_name("simulation.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"G1-D 模型: {output}")
    print(f"末端装配: {assembly}；未安装连杆移除 {len(removed_links)} 个")
    print(f"nq={compiled.nq}, nv={compiled.nv}, nu={compiled.nu}, 起始接触={check.ncon}")
    print(f"力矩限幅来自 URDF，地面 z={bottom - 0.001:.4f} m；底座/升降柱固定")
    return manifest


if __name__ == "__main__":
    build()

"""G1-D 官方 SDK 槽位与 PD；模型关节名按 G1-D URDF 映射。"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "models" / "scene.xml"
URDF = ROOT / "models" / "g1_d.urdf"
SDK_EXAMPLE = ROOT / "references" / "g1d_arm_example.cpp"
ASSEMBLY = ROOT / "models" / "assembly.json"


def load_assembly(path=None):
    """现场末端装配；none 保留腕部，dex3 使用源 URDF 的三指手。"""
    config = json.loads(Path(path or ASSEMBLY).read_text())
    if (not isinstance(config, dict) or set(config) != {"left_hand", "right_hand"}
            or any(value not in ("none", "dex3") for value in config.values())):
        raise ValueError("assembly.json 必须指定 left_hand/right_hand，值为 none 或 dex3")
    return config


def model_assembly(path=MODEL):
    """正式场景必须与装配配置一致，避免改配置后漏掉重建。"""
    path = Path(path)
    if path.resolve() != MODEL.resolve():
        return None  # 独立参考模型由调用方指定，不套用现场配置。
    config = load_assembly()
    try:
        manifest = json.loads(path.with_name("simulation.json").read_text())
    except (OSError, ValueError) as exc:
        raise ValueError("缺少有效的模型装配记录；请运行 python3 line_demo/build_model.py") from exc
    if manifest.get("end_effectors") != config:
        raise ValueError("装配配置与已生成场景不一致；请运行 python3 line_demo/build_model.py")
    return config

ARM_SUFFIXES = (
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
    "wrist_roll", "wrist_pitch", "wrist_yaw",
)
ARM_JOINTS = [f"left_{n}_joint" for n in ARM_SUFFIXES]
RIGHT_ARM_JOINTS = [f"right_{n}_joint" for n in ARM_SUFFIXES]
TIP_BODY = "left_wrist_yaw_link"

# SDK 的 G1D_NUM_MOTOR=29 是协议槽位数。0–11 和 13 无效；
# 轮子、两级升降柱和手指均不在本表内，不能套用臂部 LowCmd。
KP_29 = [0.] * 12 + [60., 0., 40.] + [40.] * 14
KD_29 = [0.] * 12 + [1., 0., 1.] + [1.] * 14
JOINT_SLOTS = {
    "torso_Joint": 12,  # URDF 轴 Z，对应 SDK WaistYaw
    "Yaw_Joint": 14,    # URDF 虽名 Yaw，实际轴 Y，对应 SDK WaistPitch
    **{name: 15 + k for k, name in enumerate(ARM_JOINTS)},
    **{name: 22 + k for k, name in enumerate(RIGHT_ARM_JOINTS)},
}
CONTROLLED_JOINTS = list(JOINT_SLOTS)

# 现场用户确认：这台 G1-D 的腰部 pitch 锁定在直立零位，槽位 14 反馈全零。
# 模型保留此关节角，统一 DDS 入口默认不向该槽位发送控制指令。
DEFAULT_LOCKED_WAIST_PITCH = 0.0
WAIST_PITCH_INDEX = CONTROLLED_JOINTS.index("Yaw_Joint")

# 模型演示用的左右对称垂臂示例，单位 rad；不作为真机零位或标定值。
# 左/右肩 roll 符号相反；腕部低于肘部，姿态已经过本地 URDF 碰撞检查。
HANG_LEFT = (0.30, 0.15, 0.0, 1.0, 0.0, 0.0, 0.0)
HANG_RIGHT = (0.30, -0.15, 0.0, 1.0, 0.0, 0.0, 0.0)


def initial_pose(name="hang"):
    """仅用于仿真或离线检查；真机启动角度必须来自 LowState。"""
    if name not in ("hang", "zero"):
        raise ValueError(f"未知示例姿态: {name}")
    values = dict.fromkeys(CONTROLLED_JOINTS, 0.0)
    if name == "hang":
        values.update(zip(ARM_JOINTS, HANG_LEFT))
        values.update(zip(RIGHT_ARM_JOINTS, HANG_RIGHT))
    return [values[n] for n in CONTROLLED_JOINTS]


def pd_for_joint(name):
    slot = JOINT_SLOTS[name]
    return KP_29[slot], KD_29[slot]


def actuator_name(name):
    return name.removesuffix("_joint").removesuffix("_Joint")

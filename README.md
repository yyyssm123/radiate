# G1-D 左手竖直线扫描

由 H2 演示适配为真正的 G1-D 模型，使用 G1-D SDK 示例的原值 PD。
本目录自带 URDF、45 个 STL、MuJoCo 场景和示例参数，无需引用 H2 工程。

**仿真与真机统一入口见 [DDS_USAGE.md](DDS_USAGE.md)**：先启动独立 MuJoCo DDS
仿真器，控制器统一运行 `line_demo/line_scan.py`；本地用 `--network lo`，接机只换网卡名。

安装、只读检查、保持和单关节验证见 [REAL_ROBOT.md](REAL_ROBOT.md)。目前已通过
95 项回归测试和独立进程的真实 DDS + MuJoCo 完整扫描；用户已读回真机服务与 LowState。
现场已确认左灵巧手拆除，正式模型默认按此装配生成，真机扫描仍需现场复测。

默认扫描复用同一条 50 cm 竖直关节轨迹，腕部朝向固定。首帧和启动保持使用实时
`LowState` 捕获的关节角，左臂限速接近扫描起点后等待实测停稳，再开始扫描。
扫描结束后，左臂回到预设收臂姿态。腰和右臂目标保持在捕获姿态。
启动流程见 [启动姿态](DDS_USAGE.md#启动姿态)，速度和规划算法见 [PLANNING.md](PLANNING.md)。

## 运行

```bash
cd /home/lenovo/fushe-G1-D

# 离线运动学检查，不连接硬件
python3 line_demo/line_servo_demo.py --check

# 打开 MuJoCo 窗口，播放一个完整往返
python3 line_demo/line_servo_demo.py --cycles 1

# 不打开窗口，不等待墙钟时间，计算完整物理仿真
python3 line_demo/line_servo_demo.py --no-viewer --fast --cycles 1
```

依赖在 `requirements.txt`。需要重建场景时运行：

```bash
python3 line_demo/build_model.py
python3 -m unittest discover -s line_demo -p 'test_*.py' -v
python3 line_demo/render_preview.py
```

预览输出到 `out/g1d_preview.png`，物理验证记录在 `out/verification.log`。

## 末端装配

`models/assembly.json` 是现场装配配置，当前为：

```json
{
  "left_hand": "none",
  "right_hand": "dex3"
}
```

`none` 只移除对应侧的手掌、手指及其质量惯量和碰撞几何，保留完整手腕；
`dex3` 使用源 URDF 中的三指手。左右手可分别配置。当前不包含其他工具的模型。
重新安装左灵巧手时，将 `left_hand` 改为 `dex3`，然后运行
`python3 line_demo/build_model.py`，并重启已打开的 MuJoCo 仿真器。
控制命令、SDK 槽位和 PD 参数保持相同；原始 URDF 保留完整版本。
装配配置修改后若未重建，正式模型加载会提示重建，避免使用旧场景。

## 数据来源和模型处理

- 原始模型：本地 `/home/lenovo/g1d_ros2_ws/src/g1d_description/urdf/g1_d.urdf`。
  原样复制为 `models/g1_d.urdf`，使用该描述包中的实际 G1-D 底盘、立柱、腰部、
  双臂和三指手网格，不使用 G1 双足模型。来源与文件摘要在 `models/source.json`。
- 官方控制参考：本地 SDK 的 `example/g1/g1d/g1d_arm_example.cpp`，副本保存在
  `references/`。`line_demo/g1d_config.py` 集中定义其槽位和增益。
- `models/scene.xml` 由原始 URDF 和装配配置生成，保持腰/臂关节轴、连杆几何、质量、惯量、
  角度限位及力矩限制，移除未安装的手部子树。测试对比原始 URDF 的腕部 FK，防止转换改变几何。
- 底盘、车轮、升降柱和已安装的手指固定在 URDF 零位，不编造这些部件的控制参数。
  默认对两个腰关节加零位约束，作为手臂验证台架；双臂仍进行重力下的力矩 PD 仿真。
- 相邻连杆的关节壳体碰撞显式排除，非相邻连杆的碰撞保留。起始姿态无接触穿透。
- 转子等效惯量使用 `armature=0.01 kg·m²`，参考本地 G1 官方 MuJoCo 模型。
  这是仿真假设，不是 G1-D 厂商标定值。物理步长为 1 ms。

## G1-D 官方示例 PD 和协议槽位

| 部位 | SDK 槽位 | 本地 URDF 关节 | KP / KD |
|---|---:|---|---:|
| 腰 yaw | 12 | `torso_Joint` | 60 / 1 |
| 腰 pitch | 14 | `Yaw_Joint`，实际轴为 Y | 40 / 1 |
| 左臂 | 15–21 | 肩 pitch/roll/yaw、肘、腕 roll/pitch/yaw | 每个 40 / 1 |
| 右臂 | 22–28 | 同上 | 每个 40 / 1 |

SDK 的 29 是协议槽位数，G1-D 的 0–11 和 13 无效，不应按 29 个实际电机关节建模。
轮子和升降柱不在上述臂部控制表内，升降在 SDK 中有单独的 AGV 高度接口。
MuJoCo 的 16 个执行器编号与 DDS 槽位分开，按关节名关联，不能直接拿数组位置代替。

仿真保持力矩电机，使用：

```text
tau = KP * (q_target - q_actual) - KD * dq_actual
```

对应 SDK 示例的 `dq=0、tau_ff=0`。输入、执行器输出和关节总力矩都有来自 URDF
的限制；例如腕 pitch/yaw 为 ±5 N·m。没有放大 KP/KD，也没有偷偷加入重力前馈。

## 默认轨迹和验证结果

本节列出独立诊断工具 `line_servo_demo.py` 的参数和结果；统一 DDS 控制器
`line_scan.py` 的新速度规划与验证结果见 [PLANNING.md](PLANNING.md)。

保留 50 cm 竖直线，针对 G1-D 重新选择了位置，参数都以零位左腕为参考：

| 参数 | 默认值 |
|---|---:|
| `--length` | 0.50 m |
| `--forward` | 0.02 m |
| `--center-dz` | 0.20 m |
| `--points` | 101 |
| `--speed` | 0.03 m/s，中段峰值 |
| `--dt` | 0.02 s，50 Hz 目标轨迹 |
| `--cycles` | 2 |
| `--waist-mode` | `fixed`，手臂验证台架 |

世界坐标中的目标为 `x=0.2771 m、y=0.1488 m、z=0.7347→1.2347 m`。
纯位置 IK，不固定末端朝向。101 个采样点的最大位置误差显示为 0.000 mm，
最小关节限位余量 32.49°，插值后单帧最大角度变化 0.0079 rad，单程 1309 帧。
若位置不可达或关节轨迹跳变过大，检查失败并停止播放。

以下为保留双侧三指手时的历史物理结果；当前装配验证见 DDS_USAGE.md。
固定腰部、SDK 原值 PD 的一个完整往返测得：

| 物理跟踪指标 | 结果 |
|---|---:|
| 末端三维跟踪误差 RMS | 45.81 mm |
| 最大三维跟踪误差 | 63.12 mm |
| x/y 坐标变化范围 | 75.56 / 4.99 mm |
| z 实际行程 | 48.68 cm |

这些是物理跟踪结果，不等于 IK 精度。官方示例的纯 PD 在这个模型和姿态下存在重力
引起的跟踪偏差，目前尚未达到毫米级扫描精度。后续需结合实际负载、重力前馈或末端
反馈验证；不能用增大增益后的结果替代原值 PD 的验证。

## 腰部动力学对照与后续真机

```bash
python3 line_demo/line_servo_demo.py --waist-mode pd --no-viewer --fast --cycles 1
```

该模式关闭两项腰部零位约束，腰部也按 SDK 的 60/1、40/1 控制。当前模型在重力下会
明显前倾，整条世界坐标轨迹被带偏；静止目标下腰 pitch 可偏到约 1.06 rad。
默认锁腰明确表示台架条件，不能据此宣称全机动力学或真机已经通过验证。

`line_servo_demo.py` 仍是离线物理演示；`run_real.py` 已实现真实 DDS 发送、状态反馈、
按捕获姿态规划、平滑接入与返回、限幅和停止逻辑。`inspect` 与 `--dry-run` 不创建指令
writer；`hold/probe/scan` 会发送 `rt/lowcmd`。现场运行步骤和验证范围见 REAL_ROBOT.md。
原 `/home/lenovo/fushe-H2-D` 工程保持独立，不会被本项目覆盖。

# G1-D 左手竖直线扫描

由 H2 演示适配为真正的 G1-D 模型，使用 G1-D SDK 示例的原值 PD。
本目录自带 URDF、45 个 STL、MuJoCo 场景和示例参数，无需引用 H2 工程。

**仿真与真机统一入口见 [DDS_USAGE.md](DDS_USAGE.md)**：先启动独立 MuJoCo DDS
仿真器，控制器统一运行 `line_demo/line_scan.py`；本地用 `--network lo`，接机只换网卡名。

安装、只读检查、保持和单关节验证见 [REAL_ROBOT.md](REAL_ROBOT.md)。最近一次完整回归
通过 100 项测试，MuJoCo 已通过独立进程的 DDS 完整扫描，真机也已有单次扫描和收臂完成记录。
现场已确认左灵巧手拆除，右手保留 Dex3。用户已修正造成 21 号腕偏航受阻的反装螺丝，
目前报告真机启动后各阶段都有抖动，根因仍待本次真机日志验证。单次完成记录不代表
连续运行稳定性或末端精度验收通过。

默认扫描复用同一条 50 cm 竖直关节轨迹，腕部朝向固定。首帧和启动保持使用实时
`LowState` 捕获的关节角，左臂限速接近扫描起点后等待实测停稳，再开始扫描。
扫描结束后，左臂回到预设收臂姿态。腰和右臂目标保持在捕获姿态。
启动流程见 [启动姿态](DDS_USAGE.md#启动姿态)，速度和规划算法见 [PLANNING.md](PLANNING.md)。

## 运行

```bash
cd /home/lenovo/fushe-G1-D

# 离线检查当前默认固定扫描，不连接 DDS
python3 line_demo/line_scan.py check --speed 0.10

# 终端 1 启动 MuJoCo，保持窗口打开，等待 DDS_SIM_READY
python3 line_demo/mujoco_sim.py --network lo

# 终端 2 运行同一套扫描控制器
python3 line_demo/line_scan.py scan --network lo --cycles 1 --speed 0.10
```

真机网卡为 `eth6`。以下命令只读取状态并规划，报告中的 `published_commands` 应为 0。
去掉 `--dry-run` 会发送运动指令，并等待遥控器 A 启动。现场正在排查的抖动需先确认。

```bash
python3 line_demo/line_scan.py scan --network eth6 --cycles 1 --speed 0.10 \
  --gravity-compensation --dry-run --report out/real_scan_preflight.json
```

`--gravity-compensation` 默认关闭，可选用于固定扫描的左肩俯仰和左肘。
仿真也可使用同一选项。它在启动保持结束后用 2 秒平滑接入力矩前馈。
一次 `--cycles 1` 包括上行、下行各 50 cm；多次往返共用一次接近和一次收臂。
结束时持续保持收臂目标，第一次 Ctrl+C 停止轨迹，第二次退出发送。

## 速度与轨迹

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `--speed` | 0.03 m/s | 左腕参考点的路径速度上限 |
| `--joint-speed` | 0.25 rad/s | 各受控关节的目标角速度上限 |
| `--joint-acceleration` | 0.50 rad/s² | 目标角加速度上限 |
| `--cycles` | 1 | 扫描往返次数 |

固定轨迹保存在 `models/scan_profile.json`，含 101 组左臂关节角。改变速度参数时会为
同一路径重新计时并预检，路径无需重新生成。默认关节约束下，`--speed 0.10` 和 `0.20`
的扫描往返均约 31.78 秒，路径峰值约 4.75 cm/s。关节速度改为 0.30 rad/s、加速度保持
0.50 rad/s² 时，往返约 26.86 秒，已在本机仿真记录中验证。程序接受的参数上限为
0.50 rad/s 和 1.00 rad/s²，真机适用值需实测验证。

`--scan-mode adaptive` 会根据启动姿态重新求 IK。指定 `--anchor` 或更改扫描线几何参数
也会选择 adaptive；显式 fixed 与自定义扫描线参数同时使用会报错。

## 主要代码

| 文件 | 职责 |
|---|---|
| `line_demo/line_scan.py` | 统一命令入口 |
| `line_demo/run_real.py` | 参数、启动、预检和控制循环 |
| `line_demo/fixed_scan.py` | 固定扫描以及接近、收臂路线 |
| `line_demo/path_timing.py` | 路径局部速度规划 |
| `line_demo/real_runtime.py` | 分段执行、停稳判断、重力前馈和后台 CSV |
| `line_demo/real_motion.py` | IK、过渡轨迹、碰撞和反馈保护 |
| `line_demo/g1d_config.py` | 模型路径、关节槽位和 PD |
| `line_demo/g1d_transport.py` | HG LowCmd/LowState、CRC 和 DDS |
| `line_demo/mujoco_sim.py` | DDS 物理仿真器 |
| `tools/build_scan_profile.py` | 离线生成待复核扫描配置 |

运行结果写入 `out/`，包括规划 JSON、约 50 Hz 的轨迹 CSV 和退出状态 `.status.json`。
后者包含 `loop_timing`、日志丢行数和写盘错误。`out/` 和 Python 缓存已加入 `.gitignore`。
控制循环相邻两轮间隔超过 70 ms 时停止轨迹与发送，目标发送周期为 2 ms。

## 安装与验证

依赖在 `requirements.txt`，官方 Python SDK 路径按本机安装位置指定。

```bash
python3 -m pip install -r requirements.txt
python3 -m pip install -e /home/lenovo/unitree_sdk2_python
python3 -m unittest discover -s line_demo -p 'test_*.py' -v
python3 tools/test_dds_e2e.py --speed 0.10 --gravity-compensation
```

测试工具绑定本机 `lo` 和隔离 DDS 域。重建场景和生成预览的命令为

```bash
python3 line_demo/build_model.py
python3 line_demo/render_preview.py
```

预览输出到 `out/g1d_preview.png`。固定扫描配置记录场景摘要，重建后如果摘要变化，需生成
并复核新的扫描配置，见 [PLANNING.md](PLANNING.md)。

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
tau = tau_ff + KP * (q_target - q_actual) - KD * dq_actual
```

默认采用 SDK 示例的 `dq=0、tau_ff=0`，双臂 KP/KD 为 40/1。
显式指定 `--gravity-compensation` 时，仅左肩俯仰和左肘的 `tau_ff` 使用模型静态重力。
CSV 中 `pd_est_Nm` 和 `tau_ff_Nm` 分别记录 PD 估计与前馈。实际力矩由电机固件执行，
发送前检查 PD 和 PD 加前馈的预算。例如腕 pitch/yaw 的 URDF 力矩上限为 5 N·m，
默认发送前预算为其 90%，即 4.5 N·m。默认锁腰配置禁用槽位 14 的电机指令。

## 独立诊断演示及历史结果

本节列出独立诊断工具 `line_servo_demo.py` 的参数和结果；统一 DDS 控制器
`line_scan.py` 的速度规划与验证结果见 [PLANNING.md](PLANNING.md)。该工具可用
`python3 line_demo/line_servo_demo.py --check` 或 `--no-viewer --fast --cycles 1` 运行。

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

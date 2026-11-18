# G1-D 真机运行

统一入口是 `line_demo/line_scan.py`，仿真和真机只换 `--network`。
`line_demo/run_real.py` 保留兼容调用。DDS 仿真启动方式见 [DDS_USAGE.md](DDS_USAGE.md)。
发送协议依据本地 SDK 的
`example/g1/g1d/g1d_arm_example.cpp`：`rt/lowstate` → `rt/lowcmd`、HG IDL、
`mode_pr=0`、回显反馈的 `mode_machine`、SDK CRC。双臂 KP=40/KD=1，腰 yaw=60/1、
pitch=40/1，`dq=0 / tau_ff=0`。没有放大官方增益。

软件入口、离线验证和本机真实 DDS/MuJoCo 联调已经完成；用户已在 eth6 读回真机服务列表，
真机网卡通信、固件接管、关节方向和
任务精度仍需接机测量。`replay` 的理想跟随结果不能当作真机或物理仿真结果。

## 环境

本机已能导入 `unitree_sdk2py` 和 CycloneDDS，测试使用了实际 HG IDL 和原生 CRC。
迁移到新电脑时安装项目依赖与官方 Python SDK：

```bash
cd /home/lenovo/fushe-G1-D
python3 -m pip install -r requirements.txt
python3 -m pip install -e /home/lenovo/unitree_sdk2_python
```

默认沿用现场已验证的 G1-D 专用示例：真机等待遥控器 A 确认，再捕获姿态与预检，
不自动调用 MotionSwitcher.ReleaseMode，也不调整底盘或升降柱。
不要同时运行另一个会发送 `rt/lowcmd` 的程序（包括 `g1d_arm_example`）。
本程序的进程锁只覆盖自己的实例，不能检测其他 SDK 或 ROS 控制器。

## 这台设备的锁定俯仰与启动方式

用户确认这台 G1-D 的俯仰锁定在直立零位，原始 LowState 的 14 号槽位全部为零。
默认 `--waist-pitch-mode locked --locked-waist-pitch-angle 0`：模型保留俯仰角 0，
实际指令槽位 14 保持 `mode=kp=kd=tau=0`，不施加俯仰 PD 或支撑力矩；
腰 yaw 与双臂 15 个有反馈的槽位仍使用官方原值 PD。
原始 `present_slots` 不伪造 14 号存在，报告另列 `not_commanded_slots=[14]`。
其他槽位缺失、故障、CRC、超时、跟踪与力矩保护保持启用。

现场另已确认左灵巧手拆除。当前 `models/assembly.json` 使用
`left_hand: none`、`right_hand: dex3`；正式场景移除左手掌、手指及其惯量和碰撞体，
保留左腕三轴。此配置同时用于真机轨迹预检和本地物理仿真，不改变 DDS 协议或原值 PD。
`models/g1_d.urdf` 保留完整源模型。更换末端后应更新装配配置、重建场景并重启仿真器。
将来若解锁 pitch，需先确认其反馈恢复，再显式使用 `--waist-pitch-mode pd`。
若俯仰只靠其他运控支撑，不能在未安排支撑与接管时退出该运控。

本机专用参考是 `/home/lenovo/unitree_sdk2/example/g1/g1d/g1d_arm_example.cpp`。
它使用 `rt/lowcmd`，等待 A 启动，没有调用 CheckMode 或 ReleaseMode。
用户已确认 `./g1d_arm_example eth6` 在这台机器人上按 A 即可运行，因此默认采用
相同的 A 键启动路径；不把通用 G1 示例中的退出步骤强制套用到 G1-D。
对照细节见 [SDK_COMPARISON.md](SDK_COMPARISON.md)。

`inspect`、`--dry-run` 立即只读执行，不等 A。`hold/probe/scan` 在物理网卡上等待 A 的新按下，
初始已按住时必须先松开；等待时 B 或 Ctrl+C 取消，尚不创建电机指令 writer。
确认后先检查反馈静止，再规划全部动作、检查姿态未漂移，首帧才保持捕获姿态。
`lo` 上的独立 MuJoCo 仿真器没有此原生遥控确认，控制器仍直接运行。

如果现场明确要求通过 MotionSwitcher 退出，可以使用可选工具：

```bash
python3 line_demo/motion_control.py status --network eth6
# 仅在已完成现场支撑和接管准备、确实需要退出时执行：
python3 line_demo/motion_control.py release --network eth6
```

退出工具只调用官方 ReleaseMode 一次，然后限时查询，读回 `name=""` 才报告成功。
不发布电机指令，不自动恢复 AI；默认扫描入口不会调用该操作。
`--handoff release` 可额外要求扫描前 CheckMode 的 name 为空；默认 `--handoff g1d`
采用上述现场已验证的专用示例流程。模式名称仅用于 MotionSwitcher 的报告，
不能证明现场不存在其他 SDK/ROS 指令发送者。

## 控制话题与服务管理

本项目自主规划上肢扫描动作，指令发布到 `rt/lowcmd`，与 SDK 的 G1-D 示例相同；
没有使用 `rt/arm_sdk`。G1 文档中为 `/arm_sdk` 开发关闭 `g1_arm_example` 的说明，
不能直接作为本项目 G1-D 低层控制的服务切换步骤。
用户提供的这台 G1-D 服务列表中没有 `g1_arm_example`。仅凭 `move_service`、
`height_service` 等名称不能判定谁在控制手臂，也不能据此认定低层控制权已释放。
按 G1-D 的现场固件要求确认低层调试条件后再进行控制会话。

通过设备状态服务的 `RobotStateClient.ServiceList()` 查询名称，再用
`ServiceSwitch(name, False)` 关闭。项目提供 `line_demo/robot_service.py`，必须显式指定
网卡与待切换服务名。先查询（不会切换服务）：

```bash
cd /home/lenovo/fushe-G1-D
python3 line_demo/robot_service.py list --network eth6
```

只有在现场流程或厂商确认要切换的具体服务后，才使用下面的命令。
将 `实际服务名` 替换为该服务的准确名称，工具不选择默认目标：

```bash
python3 line_demo/robot_service.py off --network eth6 --name 实际服务名
```

状态纠正：`ServiceSwitch` 请求中的 `switch=0/1` 和列表中的 `status` 不是同一个枚举。
这台 G1-D 的 `robot_state` 正在响应查询，其 `status=0`，据此推断此固件中
`status=0` 为运行、`status=1` 为停止；之前反向解释有误，现已修正。
工具以能响应此次 RPC 的 `robot_state` 条目为运行状态锚点，输出 `state_label`；
锚点缺失、编码不明确或切换过程中改变时拒绝切换，保留原始状态值供核对。
服务存在且未受保护时才发送一次请求，并用后续 `ServiceList` 确认目标状态。
RPC 失败、受保护或读回未完成都会报错，
不会绕过保护或尝试关闭其他服务。已经停止的服务不重复切换。

先完成现场支撑与调试准备；关闭原控制器可能改变手臂和腰部的支撑状态。本项目使用
`rt/lowcmd` 低层接口，单独停止手臂动作服务不能证明整机控制权已释放，仍需满足
G1-D 固件对低层调试模式的要求。

结束调试、退出本项目发送进程后，按现场接管流程恢复相同服务：

```bash
python3 line_demo/robot_service.py on --network eth6 --name 实际服务名
```

不要在扫描或保持进程仍发送指令时恢复内置服务。此工具不会自动恢复或开机禁用服务。
本机 Python SDK 的 `b2.robot_state` 有失效导入，因此工具采用 SDK 的 `go2.robot_state`
实现；此处只使用相同的 `robot_state` RPC 服务、版本 `1.0.0.1`、API 1001/1003，
没有使用 Go2 运动控制接口。参考 [官方 Python SDK 实现](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/go2/robot_state/robot_state_client.py)。
用户已完成真机只读服务查询；服务工具的修复通过了 RPC 替身测试，本次未实际切换设备服务。

## 接机步骤

现场真机网卡使用用户确认的 `eth6`。运行前确认当前 Linux 环境中能看到该网卡且支持多播：

```bash
ip -br link show eth6
ip -br -4 addr
cd /home/lenovo/fushe-G1-D
```

先只读反馈，不建立指令 writer：

```bash
python3 line_demo/run_real.py inspect --network eth6 --report out/device.json
```

报告记录 16 项模型腰/臂角度（其中 pitch 是现场确认的固定角）、温度、故障、
`mode_machine`、CRC 接收计数和槽位兼容性。完整 G1-D 映射是 12、14、15–28；
当前锁定配置仅要求 12、15–28 有反馈，14 为固定角；0–11/13 无效。
`state_layout_compatible=true` 仅代表数值与布局检查通过，不证明槽位方向或机型身份。
机器人关节如果在限位附近、仍在运动或有故障，先在原控制系统中处理。

启动检查失败时会打印具体关节、DDS 槽位、实测角、URDF 范围和超出量，并把同一帧
反馈写入本次报告；此阶段没有创建 lowcmd writer。只读 `inspect` 报告也包含
`joint_limit_violations` 和 `startup_margin_violations`。

原始采集工具现在也会附带 `joint_limit_check`。已有采集文件可直接离线分析：

```bash
python3 tools/inspect_device.py --from-report out/device_limit_error.json --report out/device_limit_analysis.json
```

此前用户提供的反馈中，槽位 21 `left_wrist_yaw_joint` 为 −1.768620 rad（−101.33°），
本地 G1-D 原始 URDF 下限为 −1.614430 rad（−92.50°），超出约 8.83°。
模型中此限位与来源 URDF 一致，因此保留保护。启动目标另需留出 0.02 rad 余量，
该关节的启动范围约为 [−1.594430, +1.594430] rad（±91.35°）。
先通过现场已验证的原控制方式回到模型范围内并静止；若关节实际姿态与反馈明显不符，
需要核对零位标定或机型对应的限位，不能仅凭一次实测超限扩大模型范围。

后续 `out/shoulder_collision_state.json` 的实测起点已通过角度检查，但原装配模型仍包含
已拆除的左灵巧手，导致部分可用 IK 分支在接近动作起点被手/腰模型重叠拦住。
此前终端仅显示最后一个候选的肩部碰撞错误，不能据此认为所有候选都发生同一碰撞。
本次通过按实物移除左手模型解决该起点的预检问题，保留肩部自碰撞检查及原阈值。

扫描会先检查捕获起点的模型碰撞。预检失败也会保存 `--report` 指定的
文件（默认 `out/scan_<时间>.json`），其中 `q_rad` 是同一帧捕获角度，
`planning_diagnostics.failures` 记录起点或各 IK 候选的阶段、碰撞连杆与姿态；
接近、扫描、返回中的失败另有段内 `time_s`。此阶段不创建指令 writer。
可用 `check --state-json 报告路径` 离线复现，并沿用报告 `parameters` 中的线路参数。

`out/collision_latest.json` 的 2026-11-18 起点在模型中有右肩 yaw 连杆与躯干约 4.4 mm
的重叠。右臂目标在扫描中保持捕获值，因此改变左臂 IK 无法消除这处起始重叠。
这说明模型预检的阻塞位置，不等同于已确认实物穿透；需现场核对右上臂是否贴体。
对照分析见 `out/collision_latest_preflight.json`，碰撞阈值和场景几何保持原值。

使用实测姿态规划整条扫描，仍不发指令：

```bash
python3 line_demo/run_real.py scan --network eth6 --dry-run --report out/live_plan.json
```

真机发送入口有以下三个动作。当前锁定配置接管腰 yaw 与双臂的 15 个槽位；
俯仰保持禁用字段。它们是底层 `lowcmd` 控制；腰 yaw 和右臂目标保持捕获姿态。

```bash
# 保持捕获姿态，没有预设的关节移动
python3 line_demo/run_real.py hold --network eth6

# 左肩 pitch 小幅往返 0.03 rad（约 1.7°），验证方向与反馈
python3 line_demo/run_real.py probe --network eth6 --joint left_shoulder_pitch_joint --amplitude 0.03

# 映射和保持验证通过后，执行一个 50 cm 竖直扫描往返
python3 line_demo/run_real.py scan --network eth6 --cycles 1
```

`hold/probe/scan` 每次都重新捕获实时姿态。调试时双臂自然下垂即可，首帧目标就是采集到的
关节角。正式动作之前先保持 2 秒。默认扫描先平滑接近固定七关节起姿，停稳后沿固定路径
上下往返，最后按规划收臂。左臂收臂目标沿用仿真示例中的左臂角度，见报告 `rest_left_q_rad`。
`probe` 可改成左臂任一个完整关节名，幅度最多 0.05 rad。首次实机验证时应由现场
操作者掌握硬件急停，先确认小动作的实际方向和腰部保持情况。

## 停止和退出

- 正常动作结束后持续发送末段目标，默认扫描保持收臂姿态。
- 第一次 Ctrl+C，或遥控器 A/B 的新按下：立即停止轨迹推进，持续发送最后一个目标。
  这不是切断力矩，也不会自动返回起点。
- 再次 Ctrl+C：退出并停止发送。退出前由现场操作者完成原控制器的接管；程序不会
  编造一个 G1-D 固件接管 API，也不会用清零 KP/KD 的方式让机器人失去支撑。
- 反馈超时、CRC 后续无有效包、tick 停滞、电机故障、模式改变或保护阈值超限时，
  停止轨迹和发送，返回码 2。它不能在物理断网后远程制动；最后目标、掉线和程序
  被强杀后的硬件行为仍由固件决定，必须在实机确认固件失联保护。

三个动作是独立控制会话；上一会话仍保持时，进程锁会拒绝第二个写入实例。

## 内置检查和记录

下发前完成整条动作的模型几何预检。捕获到的腰角进入 FK，规划期间若姿态改变超过
0.03 rad，则不发指令，要求重新捕获和规划。固定扫描的进入和退出都要求连续 0.25 秒
满足左臂关节误差 ≤ 0.06 rad、实测速度 ≤ 0.04 rad/s、腰/右臂分别偏差 ≤ 0.02/0.12 rad。
等待 4 秒仍不满足就停止发送并报告误差。目标点是左腕 `left_wrist_yaw_link` 原点，
腕部目标朝向在固定扫描段内保持恒定。
升降高度不会由此程序改变，报告的 z 使用 URDF 的立柱零位；实际立柱高度的常量偏移
不影响相对竖直路径，但报告的绝对 z 不是实测世界高度。底盘姿态假设为水平静止。

`--gravity-compensation` 为固定扫描的左肩俯仰和左肘添加模型重力前馈，默认关闭。
捕获姿态保持结束后，前馈在 2 秒内从零平滑接入。预检报告记录前馈峰值，CSV 的
`tau_ff_Nm` 记录每个关节实际下发的前馈，`pd_est_Nm` 仍记录 PD 部分。发送前对 PD
分量和 PD 加前馈的合力矩分别执行原力矩预算检查。可在本机运行
`python3 tools/test_dds_e2e.py --speed 0.10 --gravity-compensation` 做闭环验证。

结束时生成的 `.status.json` 中，`loop_timing` 记录最大控制周期、超过 10 ms 的周期数，
以及读取反馈、计算、DDS 写入、日志入队和休眠超时的耗时。发生调度停止时，
`last_successful_iteration` 记录触发保护前最后一轮的分项耗时。CSV 由后台线程写盘，
报告也记录丢失的样本数 `csv_dropped_rows` 与写盘错误 `csv_error`。

| 检查 | 默认值 |
|---|---:|
| 目标下发周期 | 0.002 s，目标 500 Hz |
| LowState 有效包/推进 tick 超时 | 0.20 s |
| 控制循环最大调度间隔 | 0.05 s |
| 目标关节角速度 | ≤0.25 rad/s |
| 目标关节角加速度 | ≤0.5 rad/s² |
| 目标距 URDF 限位余量 | ≥0.02 rad |
| 腰部偏离保持目标 | ≤0.10 rad |
| 持续关节跟踪误差 | 0.25 rad / 0.2 s |
| 下发前估计 PD 力矩 | ≤URDF effort 的 90% |
| 反馈 tau_est | ≤URDF effort 的 110% |
| 温度 | <70°C |
| 模型非相邻连杆穿透 | ≤2 mm |

扫描采用连续 Jacobian IK 和局部路径速度规划，根据关节限制只在受限区段放慢。
默认垂臂示例的扫描往返在 `--speed 0.03` 时约 34.92 s，在 `--speed 0.10` 时约
20.28 s；接近与返回各约 4.46 s，另有 2 秒启动保持。`--speed` 是规划路径峰值上限，
实际耗时与规划速度见报告，完整算法与比较见 [PLANNING.md](PLANNING.md)。
格内关节速度与加速度进行解析极值检查，目标插值的横向偏差也检查在 1 mm 内。

PD 力矩检查是依据当前反馈计算的预测，无法替代电机固件硬限幅。模型碰撞检查只覆盖
模型内的几何，不包含现场障碍物；原值 PD 的物理误差仍见 README，不能宣称已经
达到毫米级实机扫描精度。

运行时在 `out/real_*.csv` 记录目标角、反馈角、速度、力矩估计、反馈年龄和阶段；
结束状态与原因保存在同名 `.status.json`。控制台打印实际平均下发频率。

## 已运行的离线验证

```bash
python3 -m unittest discover -s line_demo -p 'test_*.py' -v
python3 line_demo/run_real.py check --report out/real_plan_check.json
python3 line_demo/run_real.py replay --report out/real_replay.json --log out/real_replay.csv

# inspect 导出现场状态之后，还能离线复核该姿态的轨迹
python3 line_demo/run_real.py check --state-json out/device.json
```

已通过 95 项测试，覆盖模型、物理桥、官方增益、实际 IDL/CRC、无效槽位、坏包/NaN、冻结 tick、
错误机型布局、故障、温度、模式切换、限位、速度/加速度、力矩、腰部漂移、停止键、
只读入口不创建 writer，以及非零实测起点、保持、连续过渡和返回。默认垂臂示例的完整扫描
共回放 22,949 条经过 SDK IDL 序列化、反序列化与 CRC 校验的命令，最后回到捕获姿态
并进入保持。该离线回放不创建 DDS 网络通道。
服务管理测试覆盖只读查询、明确名称、缺失/受保护服务、RPC 失败及切换后状态读回。
新规划测试还独立复核位置 Jacobian、格内速度/加速度、转向停稳、几何误差和速度参数效果。
末端装配回归覆盖左手拆除后的几何与质量移除、腕部保留、左右手独立配置、重建一致性、
实测起点的完整规划、原模型碰撞复现及肩部碰撞保护保留，结果见 `out/assembly_regression_tests.txt`。
现场超限回归使用本次完整原始反馈，验证角度与槽位诊断、保护阈值保持、预检失败时不创建
writer，以及不依赖 DDS/SDK 连接的历史报告分析。结果见 `out/joint_limit_regression_tests.txt`。
另外已通过独立进程的真实 DDS + MuJoCo 小幅动作和完整扫描，结果见 DDS_USAGE.md。

完整报告见 `out/real_replay.json`。当前待接机验证的是实际链路、设备方向/零位、
固件控制权与失联行为、腰部保持能力和最终探头轨迹精度。

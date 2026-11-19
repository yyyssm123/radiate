# 仿真、真机共用一份控制程序

统一入口是 `line_demo/line_scan.py`。轨迹规划、官方 PD、反馈保护、SDK 槽位、CRC 和
停止逻辑全部共用，没有 `--backend sim/real` 分支。

当前 `models/assembly.json` 配置为左手 `none`、右手 `dex3`，匹配现场已拆除左灵巧手的状态。
IK、碰撞预检和仿真使用同一份正式场景，左腕仍保留全部三轴。装配配置更改后运行
`python3 line_demo/build_model.py` 并重启仿真器；详见 README.md 的末端装配说明。
仿真报告和扫描规划报告均记录 `end_effectors`，便于核对所用配置。

```text
line_scan.py  ── rt/lowcmd ──→  MuJoCo DDS 仿真器 / G1-D 真机
             ←─ rt/lowstate ──
```

## 本机仿真

终端 1 启动 G1-D 物理仿真器：

```bash
cd /home/lenovo/fushe-G1-D
python3 line_demo/mujoco_sim.py --network lo
```

终端 2 运行控制程序：

```bash
cd /home/lenovo/fushe-G1-D
python3 line_demo/line_scan.py scan --network lo --cycles 1
```

速度可用 `--speed 0.10` 调整，单位 m/s，表示规划路径峰值上限。
控制器会打印实际规划峰值及耗时。默认固定腕部朝向的 50 cm 路径在 `0.10` 下
扫描往返约 31.8 秒，另计启动保持、接近、停稳和收臂。算法与旧模式的比较见 [PLANNING.md](PLANNING.md)。

`lo` 自动配置为只在本机使用的 DDS 单播发现，无需改系统网卡多播标志。
状态包使用较小的 DDS UDP 分片，避免本机发现可用而较大状态包无法接收。
不需要窗口时，给仿真器加 `--no-viewer`，控制程序保持原样。
双方默认都使用 DDS domain 0；远端仿真器也可以使用相应物理网卡。

仿真器窗口需要保持打开。若控制器提示“10s 内没有连续有效 LowState”，先确认终端 1
已打印 `DDS_SIM_READY`，两边的网卡和 domain 相同；只启动控制器不会创建仿真器。

## 启动姿态

默认仿真使用双臂下垂的示例姿态：左右肩 pitch 为 0.30 rad、roll 分别为 ±0.15 rad、
肘为 1.00 rad，其余受控关节为零。这组角度已通过本地模型的自碰撞检查，
是仿真示例，不是厂商标定值或真机必须达到的姿态。`--init zero` 可用于零位对照。

真机启动使用实时 `LowState`，先保持捕获角度 2 秒，再限速接近固定扫描起姿。
实测停稳后执行固定 50 cm 扫描；扫描结束后再次确认停稳，再将左臂收到预设姿态。
腰和右臂始终以捕获角度为目标。接近、扫描和收臂均在发送前做关节限位和模型自碰撞预检。

扫描线和腕部朝向相对躯干固定，腰 yaw 改变会改变扫描线在底盘坐标系中的位置。
需要原来的按实测姿态选 IK 分支和返回启动姿态时，使用 `--scan-mode adaptive`。
指定 `--anchor` 或调整扫描线几何参数也会使用 adaptive 模式。

连接真机后还可以把实际垂臂姿态用于仿真复现：

```bash
python3 line_demo/line_scan.py inspect --network eth6 --report out/device.json
python3 line_demo/mujoco_sim.py --network lo --pose-json out/device.json
```

`--pose-json` 仅设定仿真初始角度，真机控制始终读取实时反馈。

## 接真机

现场真机网卡已确认为 `eth6`，控制器命令只替换网卡名：

```bash
cd /home/lenovo/fushe-G1-D
python3 line_demo/line_scan.py scan --network eth6 --cycles 1
```

正式运动之前，仍使用同一个入口完成只读反馈检查、规划检查和小幅关节验证：

内置手臂服务的查询、关闭与恢复见 [REAL_ROBOT.md](REAL_ROBOT.md#控制话题与服务管理)。
按已验证的 G1-D 专用示例完成现场准备，结束其他低层发送程序；下面的运动会话等待 A 键启动。
只有现场流程确实要求额外切换服务时，才使用对应工具；默认不强制退出 AI。

```bash
python3 line_demo/line_scan.py inspect --network eth6
python3 line_demo/line_scan.py scan --network eth6 --dry-run
python3 line_demo/line_scan.py hold --network eth6
python3 line_demo/line_scan.py probe --network eth6 --amplitude 0.03
```

`inspect`、`--dry-run` 不创建指令 writer。其余命令发送 SDK 低层指令，包含腰和双臂
当前锁定俯仰配置的 15 个有效控制槽位；模型俯仰角固定为 0，槽位 14 为禁用字段。
真机等待遥控器 A 的新按下再捕获、预检、启动；`lo` 仿真直接运行。
这是用户已在 `g1d_arm_example eth6` 上验证过的启动方式，默认不强制退出 AI。
右臂与腰 yaw 的目标保持启动时捕获的姿态。
动作结束后持续保持收臂目标；Ctrl+C 一次停轨迹、再次退出发送。
固件接管与失联行为等实机步骤见 [REAL_ROBOT.md](REAL_ROBOT.md)。

## 仿真器的行为

- 接收并校验真实 HG `LowCmd`，按 SDK 槽位映射到模型执行器，不假设数组编号相同。
- 使用 `tau_ff + KP*(q_target-q) + KD*(dq_target-dq)`，保留 URDF 的力矩限制，
  用 `mj_step` 积分物理；`LowState` 的 q/dq/力矩来自积分后的状态，不复制目标角度。
- 发布推进的 tick、模式与 CRC；模拟电压 48 V、温度 30°C 仅为槽位存在性占位值。
- 第一个有效指令前暂停动力学、发布初始姿态，让控制器完成捕获和规划。
- 0.2 秒没有新指令时继续保持上一目标。这是仿真器策略，不能当作真机固件掉线行为。

场景仍采用台架条件：底盘、轮子、升降柱、已安装的手指固定，默认约束腰部。给仿真器加
`--waist-mode pd` 可关闭腰约束，控制程序不用修改；模型腰部支撑不足时会触发同一套
反馈保护。协议一致不代表动力学、负载和真实固件已经完成校准。

## 验证

```bash
python3 -m unittest discover -s line_demo -p 'test_*.py' -v

# 单独两个进程，通过本机 lo 和隔离测试域 211 运行真实 DDS + MuJoCo
python3 tools/test_dds_e2e.py --action probe
python3 tools/test_dds_e2e.py --action scan
python3 tools/test_dds_e2e.py --action scan --speed 0.10
```

测试从非零垂臂姿态开始，依次执行 `inspect`、`dry-run` 和实际动作，检查 CRC、收发数量、
物理积分、跟踪误差、首帧与启动保持、目标速度、腰和右臂保持及返回姿态。
结果保存在 `out/dds_probe_e2e.json`、`out/dds_scan_e2e.json`，
详细日志位于对应报告的 `records` 目录；测试不会连接物理网卡。

最近一次完整回归通过 100 项测试，当前固定扫描、可选重力前馈和后台日志均已进入验证。
以下为历史自适应版本的复现记录，该阶段通过 95 项回归，并用 `out/shoulder_collision_state.json` 的现场角度
完成本机真实 DDS + MuJoCo 扫描，稳定报告为 `out/dds_scan_left_hand_removed_e2e.json`。
可复现同一姿态：

```bash
python3 tools/test_dds_e2e.py --action scan --speed 0.10 --pose-json out/shoulder_collision_state.json --domain 214
```

这组起点的接近、扫描往返、返回分别约 11.30、20.28、11.30 秒，加启动保持共约 44.87 秒。
验证覆盖只读检查、轨迹预检、首帧保持、物理反馈和返回捕获姿态；指令拒绝数为 0。
仿真器与规划报告均记录 `left_hand: none`、`right_hand: dex3`。本次未执行真机运动。

下表为保留双侧三指手时，`--speed 0.10` 新规划的历史完整物理测试，报告为
`out/dds_scan_speed_010_e2e.json`，其 `records` 指向详细日志：

| 项目 | 结果 |
|---|---:|
| 当时离线回归 | 81 项通过 |
| 小幅动作的真实 DDS 指令接收 | 3,984 条 |
| 完整 50 cm 目标扫描往返的 DDS 指令接收 | 15,335 条 |
| 完整扫描的指令拒绝数 | 0 |
| 完整扫描控制器平均发送频率 | 493.75 Hz，目标 500 Hz |
| 接管后的 MuJoCo 物理积分 | 31,389 步 |
| 包含启动、接近、扫描、返回和停止的控制时长 | 31.28 s |
| 首帧及启动保持目标与捕获角度的差 | 0 rad |
| 最终目标与捕获角度的差 | 0 rad |
| 腰和右臂目标变化 | 0 rad |
| 完整扫描记录的最大目标角速度 | 0.2451 rad/s |
| 物理末端跟踪误差 RMS / 最大 | 39.07 / 62.94 mm |

完整扫描进入 `finished-hold` 后，由测试发送停止信号退出；最终目标回到捕获姿态。
这里通过的是协议与控制流程，表中的物理误差仍然存在，不代表毫米级轨迹精度通过。
后续固定扫描已有真机完整单次往返和收臂记录。当前用户报告全程抖动，稳定性仍待诊断，
任务精度和固件接管行为需单独验收。项目最新运行说明见 README.md。

`run_real.py` 为兼容保留的实现入口，旧命令仍可用。`line_servo_demo.py` 是独立进程内
的物理诊断工具，`replay` 是理想跟随离线夹具，二者都不代替真实 DDS 联调。

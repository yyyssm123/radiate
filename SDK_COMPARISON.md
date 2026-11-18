# 与本机 G1-D 官方专用示例的对照

参考源码：`/home/lenovo/unitree_sdk2/example/g1/g1d/g1d_arm_example.cpp`。
本项目的 `references/g1d_arm_example.cpp` 与该文件逐字节一致，SHA256 为
`1046e9f5614adf1e88d0789afc59d3b1cd1d4cc0c639675233b9a362e37aba93`。
本机对应可执行文件是 `/home/lenovo/unitree_sdk2/build/bin/g1d_arm_example`。

| 项目 | 专用示例 | 当前扫描程序 |
|---|---|---|
| 指令 / 反馈 | `rt/lowcmd` / `rt/lowstate` | 相同 |
| 消息与校验 | HG LowCmd/LowState、CRC、回显 mode_machine | 相同协议 |
| PD | 双臂 40/1，腰 yaw 60/1，pitch 40/1 | 有反馈控制槽位保留原值 |
| 现场启动 | 等待遥控器 A | 物理网卡同样等待 A；lo 直接运行 |
| 退出内置运控 | 没有调用 CheckMode/ReleaseMode | 默认不强制退出；可选显式 release 工具 |
| 动作 | 双肩 ±30° 正弦摆动 | 左腕竖直线扫描，腰 yaw 与右臂保持 |
| 末端装配 | 不加载手部模型做预检 | 按 assembly.json 生成场景；当前左手 none、右手 dex3 |
| 接入 | 初始化用实测角，但循环首轮会把双肩目标覆盖为 sin(0)=0 | 实测角首帧、保持 2 秒、平滑接近和返回 |
| 14 号 pitch | 通用示例按有效关节下发 | 本机已确认锁定零位且反馈全零，禁用 PD 字段，模型保留固定角 |

用户已确认专用例子在这台设备上“按 A 即可运行”，因此使用同一启动机制；
之前把通用 G1 示例的 ReleaseMode 作为 G1-D 必需条件，依据不足，现已改为可选。
CheckMode 返回 `ai` 不再作为此专用默认流程的拒绝条件。
这不授权两个低层发送程序并行运行，启动扫描前仍需结束原 SDK 示例和其他发送会话。

保留本项目的 CRC、模式一致性、温度、故障、限位、速度/加速度、力矩和模型碰撞保护。
程序对实际初始姿态会尝试有限个 IK 候选，每个候选都检查完整接近、扫描、返回；
全部失败即拒绝下发，不因模仿示例而跳过预检。

验证范围包括回归测试、实际 SDK 序列化/CRC 和 lo 上真实 DDS + MuJoCo。
使用 `out/device_raw_lowstate.json` 记录的真实初始姿态，仅在本机仿真复现时，
扫描约 21.68 秒、接近和返回各约 12.01 秒；完整发送会话约 47.79 秒。
该次物理验证没有连接 eth6，尚未由本项目实测真机扫描。

# change_mini_v1.16.1 项目状态

- 基线：`testv3_v1.16.1`（起始提交 `e1da1439dc`）。
- 工作分支：`change_mini_v1.16.1`。
- 独立工作树：`/home/crocodile/PX4-Autopilot-change-mini`。
- 目标：适配无变形的小型 Quad/Rover 双模无人机。
- 硬件：云台舵机 MAIN5；左轮 MAIN7；右轮 MAIN8；车轮为 0%~100% 单向 PWM，负指令归零。
- 云台：普通 90 度 PWM 舵机，默认 1500 us；RC channel 10 与 MAVLink mount/gimbal 连续控制。
- 切换约束：Quad -> Rover 必须由 PX4 原生着陆检测确认 landed；删除变形执行器、位置检测及等待过程。
- 当前固件目标：`make hkust_nxt-dual_mini`；airframe ID 为 22002。
- 当前发布基线：`change_mini_v1.16.1 @ 9602c367e9be78f6d12971d0a019cf477517689f`；
  MAVLink stream 修复分支为 `fix/mini-rover-mavlink-stream-config`。
- NXT-Dual 定时器：MAIN1~4=TIM1，MAIN5/6=TIM2，MAIN7/8=TIM3。
- 轮 PWM 默认载波：1 kHz，由 `PWM_MAIN_TIM2` 参数化；MAIN7/8 使用 duty mask `0xC0`。
- 原生 `actuator_motors_rover.control[0]` 是右轮、`control[1]` 是左轮；mini 仲裁层
  将其路由为 MAIN8/右轮、MAIN7/左轮。
- 安全输出：切换后仅接受目标控制器在切换 epoch 之后发布且不超过 200 ms 的样本。
- `mini_vehicle_control` 提供锁存式输出诊断；`mini_vehicle_control status` 显示当前源年龄、
  解锁期最大年龄、输出问题/恢复/安全阻断/armed下降次数，以及最近一次原因、四路输入和
  arming快照。已解锁问题边沿同时通过MAVLink日志通知QGC，但不改变任何输出判定或赋值。
- 历史行为修复：已从 `325a9d07ba` 选择性移植 Quad-Rover 自动解除锁定抑制、MC
  控制器状态更新、Rover 外环所有权及 yaw-rate setpoint 低值保留；未移植调试探针。
- Rover -> Quad 时 MC 姿态/rate 控制器会清空旧目标、积分与滤波，并拒绝切换
  epoch 之前的 attitude/rate/RC/autotune 输入；`ManualControl` 保持原生参数化手势。
- Rover 实时调参现状和 QGC 第一阶段实施路线记录于
  `docs/mini_rover_realtime_tuning_guide.md`；目标 QGC 基线是
  `nanjia24/qgroundcontrol` 的 `codex/joystick-aux-px4-development` 分支提交
  `754135601a53d7650ddeb6562ca5a5cd2167880c`。
- QGC Agent 的可直接交付提示词位于 `docs/mini_rover_qgc_agent_prompt.md`；固定功能
  分支为 `codex/mini-rover-realtime-tuning`，独立工作树为
  `E:\workspace\QGC\qgroundcontrol-worktrees\mini-rover-realtime-tuning`，要求 Agent
  完成自动测试、commit 和 push。
- MAVLink 协议已发布到 `QQgdiw/mavlink:mini-rover-tuning-v1.16.1`，固定 commit 为
  `07c6964a8fcc364c49d394f0bf0275b9fc05857d`；PX4/QGC dialect 分别为
  `mini_rover`/`qgc_mini_rover`。
- 机载通信按原生PX4所有权分流：VIO/SLAM只向EKF2提供定位，物理RC在Position操控，
  Offboard接收机载setpoint，Mission/RTL由Navigator执行；`MANUAL_CONTROL`仅是可选虚拟
  摇杆。完整MAVLink契约位于`docs/mini_vehicle_mavlink_communication.md`。
- mini Rover正式Offboard接口限定为local NED二维位置Go-to；MAVLink使用
  `SET_POSITION_TARGET_LOCAL_NED`，DDS使用`OffboardControlMode.position +
  TrajectorySetpoint.position[0:1]`，不承诺前向速度+yaw-rate接口。
- VIO-only可支持local Position和Offboard，但不能满足标准RTL的global/Home要求；当前
  Rover Direct RTL还会因持续landed而直接IDLE，修复并实车验证前禁止作为安全功能。
- 本机无SD卡时Mission需设置`SYS_DM_BACKEND=1`使用RAM dataman，任务数据重启即丢失。
- 当前 uXRCE-DDS 能力、全部 topics、QoS、时间/坐标约定和已知缺口位于
  `docs/mini_vehicle_dds_communication.md`。DDS client 已编译但真机默认禁用；正式
  ROS 2 集成仍需要从最终固件消息定义生成并锁定配套 `px4_msgs`。官方 package 骨架
  固定为`PX4/px4_msgs release/1.16 @ 392e831c1f...`；自定义`VehicleStatus`尚需从
  version 1升级并提供translation，当前DDS不具备正式发布条件。
- Differential mini 已提供 Rate 50 Hz、Attitude 30 Hz、Velocity 25 Hz、
  Position/Path 10 Hz 的按需 stream；未加入默认遥测配置，Quad/inactive 和陈旧
  response 使用有效位/NaN 明确失效。
- 源码锚点 `2f5d1f003b...` 的目标构建 Flash image 为 1,699,944 B / 1,792 KiB
  （92.64%）；`.px4` SHA-256 为
  `c8fb8aaeebe40846292c2fddd21d919ce5a7f736d3e83347c0b31794e53c9bd0`，`.bin` 为
  `b06be017cfbdc706493e294f3759ebf5368e9bf944b79794cb88e6d1e23c6a97`。
- 该源码锚点已通过隔离 Linux PATH 的 PX4 CTest 146/146 和 MAVLink dialect
  测试 2/2；尚未进行 QGC、USB 实机或车辆硬件验收。
- future误判修复版本通过隔离Linux PATH的PX4 CTest 147/147（含
  `unit-MiniVehicleControlDiagnostics`）及`make hkust_nxt-dual_mini`；Flash image为
  1,702,752 B / 1,792 KiB（92.79%）。`.px4` SHA-256为
  `c76a95a286d96248ca76a39dae26641f65d240ac910624cd78da87583e5595a6`，`.bin`为
  `258930095ea2af119d9e81d90cac4592f895c87d22d61160b3e23dcbfd60eea0`；尚未刷机验收修复。
- MAVLink stream 配置现使用固定存储、generation 和单一 1 s monotonic deadline 在
  receiver/CLI 与 main thread 间交接；prepare 在 handoff 锁外完成，commit 只做有界
  非阻塞变更，真实配置成功才返回 command 511/512 Accepted。stream 删除采用
  reader/retired 生命周期保护，receiver 启动失败和 stop-all 注册表竞态也已加固。
- 当前修复树通过 PX4 CTest 148/148，其中 `unit-MavlinkStreamConfig` 22/22；普通并发压力
  300/300、关闭 ASLR 后 ThreadSanitizer 100/100、mini Rover dialect 2/2。本次受影响
  文件 AStyle 8/8 通过，`git diff --check` 通过；全仓库 AStyle 仍可能受未改动基线文件影响。
- 最近 GCC 9.3.1 目标构建 Flash 为 1,709,128 B / 1,792 KiB（93.14%）；`.px4` SHA-256
  为 `293dd2ef1e23eb88195c466a20b76103c69c6c3202d52386f507bf7fe24ee3cb`，`.bin` 为
  `8ded817c00d876116757f387a66892e9c634d011d8f28b809b2be79ac991c4c5`。
- USB command/ACK 无电池验收和带电实时波形验收均尚未执行；目标板构建和主机自动测试
  不能替代这两阶段实机验收。

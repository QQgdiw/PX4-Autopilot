# change_mini_v1.16.1 项目状态

- 基线：`testv3_v1.16.1`（起始提交 `e1da1439dc`）。
- 工作分支：`change_mini_v1.16.1`。
- 独立工作树：`/home/crocodile/PX4-Autopilot-change-mini`。
- 目标：适配无变形的小型 Quad/Rover 双模无人机。
- 硬件：云台舵机 MAIN5；左轮 MAIN7；右轮 MAIN8；车轮为 0%~100% 单向 PWM，负指令归零。
- 云台：普通 90 度 PWM 舵机，默认 1500 us；RC channel 10 与 MAVLink mount/gimbal 连续控制。
- 切换约束：Quad -> Rover 必须由 PX4 原生着陆检测确认 landed；删除变形执行器、位置检测及等待过程。
- 当前固件目标：`make hkust_nxt-dual_mini`；airframe ID 为 22002。
- 当前固件源码协议锚点：`2f5d1f003b3106060e70df012de59bfc3404837c`。
- NXT-Dual 定时器：MAIN1~4=TIM1，MAIN5/6=TIM2，MAIN7/8=TIM3。
- 轮 PWM 默认载波：1 kHz，由 `PWM_MAIN_TIM2` 参数化；MAIN7/8 使用 duty mask `0xC0`。
- 原生 `actuator_motors_rover.control[0]` 是右轮、`control[1]` 是左轮；mini 仲裁层
  将其路由为 MAIN8/右轮、MAIN7/左轮。
- 安全输出：切换后仅接受目标控制器在切换 epoch 之后发布且不超过 200 ms 的样本。
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
- 机载电脑正式车辆控制采用 MAVLink2 归一化 `MANUAL_CONTROL`；完整身份、链路、
  失联、双模切换、云台和调参消息契约位于
  `docs/mini_vehicle_mavlink_communication.md`。
- 当前 uXRCE-DDS 能力、全部 topics、QoS、时间/坐标约定和已知缺口位于
  `docs/mini_vehicle_dds_communication.md`。DDS client 已编译但真机默认禁用；正式
  ROS 2 集成仍需要从最终固件消息定义生成并锁定配套 `px4_msgs`。
- Differential mini 已提供 Rate 50 Hz、Attitude 30 Hz、Velocity 25 Hz、
  Position/Path 10 Hz 的按需 stream；未加入默认遥测配置，Quad/inactive 和陈旧
  response 使用有效位/NaN 明确失效。
- 源码锚点 `2f5d1f003b...` 的目标构建 Flash image 为 1,699,944 B / 1,792 KiB
  （92.64%）；`.px4` SHA-256 为
  `c8fb8aaeebe40846292c2fddd21d919ce5a7f736d3e83347c0b31794e53c9bd0`，`.bin` 为
  `b06be017cfbdc706493e294f3759ebf5368e9bf944b79794cb88e6d1e23c6a97`。
- 该源码锚点已通过隔离 Linux PATH 的 PX4 CTest 146/146 和 MAVLink dialect
  测试 2/2；尚未进行 QGC、USB 实机或车辆硬件验收。

# Local Project Instructions

【人类编写区域】

- 保留给项目所有者追加指令。

【客观事实区域】

- `change_mini_v1.16.1` 从 `testv3_v1.16.1` 创建，独立工作树为
  `/home/crocodile/PX4-Autopilot-change-mini`。
- mini 机型是不变形 Quad/Rover，当前飞控固件目标为 `hkust_nxt-dual_mini`；旧
  `zeroone_x6_mini` 配置保留但不再是当前硬件目标。
- MAIN5 接 90 度摄像头云台舵机，MAIN7 接左轮，MAIN8 接右轮。
- 旧 ZeroOne X6 的 MAIN7/8 共用 TIM12；轮输出默认以 1 kHz 产生 0%~100%
  单向 PWM 占空比，该频率可由 `PWM_MAIN_TIM2` 调整并必须匹配双路 H 桥规格。
- HKUST NXT-Dual 的 MAIN1~4、MAIN5/6、MAIN7/8 分别属于 TIM1、TIM2、TIM3，
  对应 `PWM_MAIN_TIM0`、`PWM_MAIN_TIM1`、`PWM_MAIN_TIM2` 三个独立协议组。
- 云台使用 PX4 原生 gimbal 模块：RC channel 10 映射至 AUX1，同时接受 MAVLink
  gimbal v2 控制；默认/失效 PWM 为 1500 us，范围为 1000~2000 us。
- 原生 `rover_differential` 的 `actuator_motors_rover.control[0]` 语义是右轮、
  `control[1]` 语义是左轮；mini 仲裁层必须将其分别路由到 MAIN8、MAIN7。
- mini 的 QGC 适配基线是 `https://github.com/nanjia24/qgroundcontrol.git` 分支
  `codex/joystick-aux-px4-development`，核验提交为
  `754135601a53d7650ddeb6562ca5a5cd2167880c`；上级文档中的 `898aee795c...` 属于
  另一个 `change1_v1.16.1` 项目，不得用于本任务。
- QGC 实时调参功能分支固定为 `codex/mini-rover-realtime-tuning`，独立工作树固定为
  `E:\workspace\QGC\qgroundcontrol-worktrees\mini-rover-realtime-tuning`；完整执行提示词
  位于 `docs/mini_rover_qgc_agent_prompt.md`。
- 该 QGC 基线没有 PX4 Rover PID Tuning 页面；mini 以
  `MAV_TYPE_VTOL_FIXEDROTOR`（值22）上报，Rover 页面还必须以 `HYBR_QUAD_ROV=1` 识别并与
  Multirotor 页面并存。
- mini Rover tuning 的 MAVLink 私有开发必须以
  `QQgdiw/mavlink` 的 `master @ 5bfd76d80281f6027134e854aafe6cb3dbfbe9e1`
  为精确父提交；不得从已前进的 master 或 `qgc-hybrid-*-r2` composite 线创建。
- mini Rover tuning 已固定为 `QQgdiw/mavlink` 分支
  `mini-rover-tuning-v1.16.1`、commit
  `07c6964a8fcc364c49d394f0bf0275b9fc05857d`。PX4 使用 `mini_rover` dialect，
  QGC 使用保留原 `all.xml` 的 `qgc_mini_rover` composite dialect。
- 四个 MAVLink2 消息为 60100 Rate（LEN 27/CRC 147）、60101 Attitude
  （23/85）、60102 Velocity（43/217）、60103 Position（44/90）；MAVLink1
  不支持这些大于 255 的消息 ID。
- 机载电脑正式车辆控制选用 MAVLink2 `MANUAL_CONTROL`；专用链路使用
  `COM_RC_IN_MODE=1`，需要物理RC失效接管时才使用整源切换模式2，不能把车辆轴与
  RC AUX1拆分到两个并行输入源。
- 当前DDS实际形态状态topic为`/fmu/out/vehicle_status_v1`；消息含自定义
  `is_quad_rover`字段，正式ROS 2集成必须锁定由本分支最终消息定义生成的
  `px4_msgs`，不得假定任意upstream v1.16定义兼容。
- DDS配套包可使用`PX4/px4_msgs release/1.16 @
  392e831c1f659429ca83902e66820d7094591410`作为骨架；当前自定义
  `VehicleStatus`增加字段后仍保留`MESSAGE_VERSION=1`，正式发布前必须升级消息版本并
  提供显式translation，不能用相同`_v1` topic承载不同wire schema。

【项目规范区域】

- mini 机型使用独立 `mini_vehicle_control`，不得改变旧
  `zeroone_x6_hybrid` 的变形状态机与硬件行为。
- NXT-Dual 的 `default.px4board` 默认启用 AS5600 和旧 hybrid 模块；mini overlay
  必须显式将两者设为 `n`，不能依赖省略配置来排除旧变形功能。
- 新增的机型启动脚本必须同时加入 `ROMFS/px4fmu_common/init.d/CMakeLists.txt`；仅在
  `rc.vehicle_setup` 中引用不会被打包进 NuttX ROMFS。
- Quad -> Rover 必须使用新鲜的 `vehicle_land_detected.landed` 作为门控，禁止恢复
  高度阈值、变形传感器或定时等待逻辑。
- 模式切换后，只有目标控制器在本次切换之后发布的新鲜执行器样本才可输出；该
  epoch 门控不得实现为对外可见的变形过渡态。
- 模式切换的 epoch 门控必须同时清除控制器内部积分、滤波与缓存，并拒绝切换
  epoch 之前的上游 setpoint/RC 输入；仅检查最终 actuator 消息时间戳不能阻止
  旧状态被重新发布为新时间戳输出。
- Rover 左右轮分别保留原生差速控制器的 control[0]/control[1] 所有权；负指令仅在
  最终输出仲裁层钳制为零，禁止在 Rover 控制器之外重新计算差速控制。
- 修改 mini 物理输出映射前，必须同时核对原生控制器的槽位语义和实物 MAIN 接线；
  不能仅依据数组下标名称推断左右轮，否则会导致右摇杆方向反转。
- MAIN7/8 的占空比输出必须复用 PX4 PWM 输出的解锁、failsafe 与 output-function
  安全链路，不得用直接写 GPIO/定时器的旁路驱动绕开安全状态。
- Rover 实时调参不得把 MC 的 `ATTITUDE_TARGET` 或
  `POSITION_TARGET_LOCAL_NED` 当作 Rover setpoint；正式页面必须消费 Rover 控制器
  实际使用的 response/setpoint、源时间戳和有效位，参数写入继续走标准 PARAM/Fact。
- 严谨 Rate 实时调参只以 USB 等高带宽链路验收；57,600 baud 数传不支持，
  115,200 baud 在叠加常规遥测后也不得作为内环曲线的可靠验收链路。
- 第一阶段 tuning producer 只实现并验收 Differential mini；MAVLink wire schema
  预留 `drive_type`、body-Y 和逐轴有效位，但禁止借此扩大到 Ackermann/Mecanum
  控制实现或宣称其已支持。
- Rover tuning stream 的 `CONTROLLER_ACTIVE` 必须来自控制器原子 status，并同时由
  当前 Rover 形态门控；inactive/Quad 不得把有限的复位值标成有效曲线。
- mini 形态门控要求 `HYBRID_STATE_DRIVING` 的 hybrid status 在 200 ms 内；
  `vehicle_status.vehicle_type==ROVER` 的辅助状态允许 1 s，因 Commander 正常仅在
  状态变化或约 500 ms 周期发布 vehicle status。
- Rate、Attitude、Velocity 的跨 topic 输出只允许与 status 使用完全相同的控制周期
  时间戳；禁止用宽松时间窗口拼接。response 原始数据超过 500 ms 未更新时必须失效。
- 四条 tuning stream 只能按 `MAV_CMD_SET_MESSAGE_INTERVAL` 请求启用，不得加入默认
  MAVLink stream 配置；严谨验收只使用 MAVLink2 USB 链路。
- 修改任何versioned DDS消息的字段或wire布局时必须同步递增`MESSAGE_VERSION`、提供旧版
  translation并发布匹配的`px4_msgs` commit；禁止在相同版本topic后缀下静默改变schema。

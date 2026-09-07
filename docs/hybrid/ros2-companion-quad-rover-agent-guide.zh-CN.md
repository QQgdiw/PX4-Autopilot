# ZeroOne Quad/Rover 机载电脑通信与控制协议

本文是 `zeroone_x6_hybrid` 机载电脑实现的规范性接口文档，主要面向负责开发
ROS 2/MAVLink 客户端的 Agent，同时保留足够的人类可读说明。

本文描述的是已经实现的接口和当前已知限制。机载电脑不得自行实现第二套车辆
控制器，不得把该机型伪装成 VTOL，也不得绕过 PX4 原生差速 Rover 控制链直接
计算左右轮命令。

## 1. 固定版本基准

| 项目 | 固定值 |
| --- | --- |
| PX4 仓库 | `https://github.com/QQgdiw/PX4-Autopilot.git` |
| PX4 分支 | `feature/testc4-rover-tuning` |
| 本文对应 PX4 commit | `7c4fb9e638787435351437c4dfc99e417adffd85` |
| 固件目标 | `zeroone_x6_hybrid` |
| MAVLink 版本 | MAVLink 2 |
| MAVLink dialect | `hybrid_vehicle` |
| MAVLink 仓库 | `https://github.com/QQgdiw/mavlink.git` |
| MAVLink 固定 tag | `qgc-hybrid-rover-tuning-v1.16.1-r1` |
| MAVLink peeled commit | `21922689c6fb113884df0f66582d8e602286fdc1` |
| ROS 2 消息仓库 | `https://github.com/QQgdiw/px4_msgs.git` |
| `px4_msgs` 固定 tag | `hybrid-rover-v1.16.1-r1` |
| `px4_msgs` peeled commit | `e0f41fb57ed9217ba15730854f6c7c92b0262134` |

机载端必须固定使用上述两个 tag，不得使用 stock `common.xml`、上游
`PX4/px4_msgs release/1.16` 或仅凭分支最新状态构建生产版本。

MAVLink tag 由 GitHub ruleset `22006870`保护；`px4_msgs` tag 由 ruleset
`22433564`保护。两条规则均禁止删除和 non-fast-forward 更新。

兼容性来源的优先级为：

1. 上表固定的 MAVLink 与 `px4_msgs` tag；
2. 本文规定的字段语义、安全门控和状态机；
3. PX4 固件对应 commit 中的消息定义和实现；
4. 普通 PX4/ROS 2 文档。

若这些来源发生冲突，停止集成并记录具体字段、版本和抓包证据，不得自行猜测。

## 2. 总体架构与职责边界

机载电脑同时使用两条相互独立的通信路径：

```text
MAVLink 2 Hybrid 客户端
  <- HEARTBEAT(type=200)
  <- HYBRID_VEHICLE_STATUS(60000)
  <- COMMAND_ACK(command=50000)
  -> MAV_CMD_DO_HYBRID_TRANSITION(50000)
  -> 可选：MAV_CMD_SET_MESSAGE_INTERVAL(511)

ROS 2 Hybrid Supervisor
  -> 根据 MAVLink 形态、故障和命令生命周期决定是否允许 Rover Offboard
  -> 形态变化或状态失效时清空缓存并执行停止策略

uXRCE-DDS Rover Offboard Publisher
  -> /fmu/in/offboard_control_mode
  -> /fmu/in/rover_velocity_setpoint
  <- /fmu/out/timesync_status
```

职责约束：

- MAVLink 负责机型识别、形态/故障监控、显式变形命令、ACK 和可选调参数据。
- DDS 只负责向 PX4 原生 Rover 控制器输入车体前向速度和偏航角速度。
- PX4 是形态、epoch、解锁、模式、失控保护和最终执行器输出的唯一权威。
- 机载端不得发布 `RoverThrottleSetpoint`、`RoverSteeringSetpoint`、直接执行器值
  或左右轮命令作为正常 `/cmd_vel` 路径。
- QGC 和机载端可以同时接收状态，但必须避免同时发起相互冲突的变形命令。
- command 50000 的 ACK 只能由 `hybrid_vehicle_control` 产生，客户端不得伪造。

PX4 保留的控制链为：

```text
speed_body_x + yaw_rate
  -> DifferentialVelControl + DifferentialRateControl
  -> RoverDifferential
  -> control allocation
  -> M2006/C610 轮速闭环
```

## 3. MAVLink 2 Hybrid 协议

### 3.1 机型识别

- `HEARTBEAT.type = MAV_TYPE_QUAD_ROVER = 200`。
- 该值在 Quad、Rover 和变形期间保持不变，只表示机型家族。
- 不得根据 `HEARTBEAT.type` 判断当前物理形态。
- `EXTENDED_SYS_STATE.vtol_state`保持未定义，不得用于形态判断。
- 当前动态形态来自 `HYBRID_VEHICLE_STATUS`，命令生命周期还必须结合
  `COMMAND_ACK` 和客户端自身保存的活动请求判断。

### 3.2 显式变形命令 50000

发送 `COMMAND_LONG`：

```text
command = MAV_CMD_DO_HYBRID_TRANSITION = 50000
param1  = 1.0  # 请求 Quad
       或 2.0  # 请求 Rover
param2..param7 = 0.0
```

直接 MAVLink 命令传输允许保留参数为零或 NaN，但为了与 Mission 的严格校验保持
一致，机载端必须始终发送有限的精确 `0.0`。不得发送
`MAV_CMD_DO_VTOL_TRANSITION`代替本命令。

主要 ACK 行为：

| 情况 | ACK | 客户端动作 |
| --- | --- | --- |
| 参数目标非法或保留参数非法 | `DENIED` | 失败，不自动重试 |
| 状态未知或暂时不满足安全条件 | `TEMPORARILY_REJECTED` | 等待新的人类/上层请求，不循环重试 |
| 已处于请求的稳定形态 | `ACCEPTED` | 核验新鲜稳定状态，不等待第二个 ACK |
| 接受新的变形请求 | `IN_PROGRESS` | 保存 `result_param2` sequence，停止 Rover 输出 |
| 重发当前同目标请求 | `IN_PROGRESS` | 仍属于原 sequence，不当作新请求 |
| 变形期间请求相反目标 | `TEMPORARILY_REJECTED` | 不排队，不自动重发 |
| 安全推进所有权建立 | 终态 `ACCEPTED` | 再等待新鲜目标形态状态 |
| 变形或协调序列失败 | 终态 `FAILED` | 进入故障处理，不自动清故障 |

所有 command 50000 ACK 的 `result_param2` 都携带当前
`transition_sequence`。新请求在首个 `IN_PROGRESS` 前递增 sequence；该请求的
进度和终态 ACK 使用相同 sequence。

客户端关联 ACK 时至少使用：

```text
source system + source component + command 50000 + result_param2
```

不得仅按接收顺序关联，也不得把重复 `IN_PROGRESS`解释成新变形。

### 3.3 当前 HX8/HX-65HM 协调序列

当前构型使用一个 HX8-U45H-M 起落架舵机和两个 HX-65HM 变形舵机，共享一条
1 Mbps 半双工 UART。机载端不直接控制这些舵机，但必须理解命令终态的含义。

Quad 转 Rover：

```text
保持 Quad 推进所有权
  -> 自动模式下放下起落架
  -> 等待 PX4 landed 确认
  -> 请求并等待上锁
  -> HX-65HM 执行 Quad->Rover 变形
  -> 起落架达到车轮安全间隙后建立 Rover 推进所有权
  -> 自动模式下继续收起起落架
```

Rover 转 Quad：

```text
自动模式下放下起落架
  -> 请求并等待上锁
  -> HX-65HM 执行 Rover->Quad 变形
  -> 建立 Quad 推进所有权并允许后续起飞
  -> 自动模式下检测离地后收起起落架
```

`LG_AUTO_EN=0`时，起落架角度由操作员独立控制，不参与上述阶段推进和 Ready
判断；但 HX8 在线、配置核验和保护健康仍属于安全门控。Quad 转 Rover 仍要求
着陆确认并上锁，Rover 转 Quad 仍要求上锁。

当前 message 60000 没有完整传输内部 `sequence_state`、
`propulsion_owner/ready` 和起落架字段。因此客户端必须采用保守规则：只要自己
跟踪的命令仍为 `IN_PROGRESS`、状态报告仍显示最近命令为 `IN_PROGRESS`，或收到
故障/未知状态，就按变形中处理，不得仅凭 `current_state`提前恢复 Rover 输出。

### 3.4 HYBRID_VEHICLE_STATUS（60000）

Normal 和 Onboard MAVLink 模式默认以 1 Hz 发送 message 60000。MAVLink 1 不
发送该私有消息，机载链路必须协商 MAVLink 2。

| 字段 | 含义 |
| --- | --- |
| `timestamp` | PX4 启动时间，微秒 |
| `transition_sequence` | 最近接受的变形 sequence |
| `transition_elapsed_ms` | 当前变形/故障计时，毫秒并饱和到 `UINT32_MAX` |
| `position_normalized` | 变形位置归一化值；不可用时 NaN |
| `current_state` | `0=Quad, 1=Transitioning, 2=Rover, 3=Unknown, 4=Fault` |
| `target_state` | `0=None, 1=Quad, 2=Rover` |
| `fault_reason` | 变形故障码，零表示无故障 |
| `command_result` | 最近 command 50000 的 `MAV_RESULT` |
| `sensor_source` | 位置反馈来源数值 |
| `actuator_backend` | 变形执行器后端数值 |
| `actuator_protection_flags` | 执行器保护标志 |
| `flags` | 传感器、位置、执行器、着陆状态位 |
| `command_timestamp` | MAVLink 2 extension；原始 PX4 `vehicle_command.timestamp` |

`flags`定义为：

| 位 | 值 | 含义 |
| ---: | ---: | --- |
| 0 | 1 | sensors enabled |
| 1 | 2 | position confirmed |
| 2 | 4 | position valid |
| 3 | 8 | actuator online |
| 4 | 16 | actuator healthy |
| 5 | 32 | actuator configuration verified |
| 6 | 64 | landed |
| 7 | 128 | land detection fresh |

建议机载端以本地单调时钟记录每次接收时间，并将超过 3 秒未更新的 message
60000 视为失效。MAVLink 状态失效时，即使 DDS 链路仍连接，也不得继续声明
Rover Offboard 有效。

### 3.5 HX-65HM 枚举兼容限制

当前 PX4 内部 uORB 已定义：

```text
SENSOR_HX65   = 4
ACTUATOR_HX65 = 2
```

但固定 MAVLink tag 的 `hybrid_vehicle.xml` 目前只为这些字段声明到
`SENSOR_HX8=3` 和 `ACTUATOR_HX8=1`。由于 message 60000 的两个字段都是
`uint8_t`，MAVLink 帧长度、CRC 和反序列化不会因此失败，客户端仍会收到数值
`4` 和 `2`；生成的 SDK 只是不具备对应的符号名。

当前客户端必须：

- 接受并保存未知的 `uint8_t` 枚举值；
- 不因数值 `sensor_source=4` 或 `actuator_backend=2` 丢弃整条状态消息；
- 不把这两个字段作为唯一的运动许可条件；
- 使用 `current_state`、`fault_reason`、`flags`、ACK 生命周期和状态新鲜度进行
  安全判断；
- 在 UI/日志中可显示为 `HX65(raw=4/2)` 或 `unknown(raw=N)`。

未来可以通过只新增 XML 枚举项发布新 MAVLink tag。该操作不改变 message 60000
的 LEN/CRC，但所有 PX4/QGC/机载生成绑定都应切换到同一个新 tag。新增内部
sequence/gear 线上字段则必须使用 MAVLink 2 extension 或新消息，并单独进行协议
版本设计，不能与简单枚举补全混为一项。

### 3.6 Rover 实时调参消息

当前 MAVLink tag 还包含四条按需调参消息：

| 消息 | ID | LEN | CRC Extra |
| --- | ---: | ---: | ---: |
| `ROVER_RATE_TUNING_STATUS` | 60100 | 27 | 147 |
| `ROVER_ATTITUDE_TUNING_STATUS` | 60101 | 23 | 85 |
| `ROVER_VELOCITY_TUNING_STATUS` | 60102 | 43 | 217 |
| `ROVER_POSITION_TUNING_STATUS` | 60103 | 44 | 90 |

这些 stream 不在任何默认速率表中。只有需要调参时才通过
`MAV_CMD_SET_MESSAGE_INTERVAL`（511）开启，结束后关闭。Quad、变形、Fault、
controller inactive 或状态过期时，客户端必须接受 `valid_flags=0`且浮点字段为
NaN 的终止帧。

## 4. uXRCE-DDS 与 px4_msgs 契约

### 4.1 固定 px4_msgs

机载 ROS 2 工作区必须检出：

```text
repository: https://github.com/QQgdiw/px4_msgs.git
tag:        hybrid-rover-v1.16.1-r1
commit:     e0f41fb57ed9217ba15730854f6c7c92b0262134
```

该 tag 的 236 个消息和 1 个服务由 PX4 commit `7c4fb9e638`完整同步。禁止用
上游 `PX4/px4_msgs release/1.16`替代，否则 `rover_velocity`字段和项目扩展消息
不匹配。

首次集成必须在目标 ROS 2 环境执行 `colcon build`并保存 ROS 发行版、RMW 实现、
构建日志和最终锁定 commit。本 PX4 WSL 环境没有 ROS 2/`colcon`，因此仓库发布
只完成了消息同一性和依赖检查，没有冒充完成机载端 ROS 构建。

### 4.2 DDS topic

当前控制所需映射来自 PX4：

| DDS topic | ROS 2 类型 | 方向 |
| --- | --- | --- |
| `/fmu/in/offboard_control_mode` | `px4_msgs::msg::OffboardControlMode` | 机载到 PX4 |
| `/fmu/in/rover_velocity_setpoint` | `px4_msgs::msg::RoverVelocitySetpoint` | 机载到 PX4 |
| `/fmu/out/timesync_status` | `px4_msgs::msg::TimesyncStatus` | PX4 到机载 |

`HybridVehicleStatus`虽然存在于 `px4_msgs`完整消息包中，但当前没有登记为
`/fmu/out/hybrid_vehicle_status`。仅存在 ROS 类型不代表 PX4 会通过 DDS 发布该
topic；形态状态仍必须走 MAVLink message 60000。

### 4.3 OffboardControlMode exact-one-bit

Rover Offboard 只接受 `OffboardControlMode.rover_velocity=true`。完整样本必须满足：

```text
rover_velocity    = true
position          = false
velocity          = false
acceleration      = false
attitude          = false
body_rate         = false
thrust_and_torque = false
direct_actuator   = false
```

这不是建议，而是独立 Quad/Rover 的协议门控。任何 legacy `velocity`、多个 bit
同时为真或所有 bit 均为假，都不能进入 Hybrid Rover Offboard 控制链。

普通非混合 Rover 的旧 Offboard 行为不代表本机型允许 legacy 输入。

### 4.4 RoverVelocitySetpoint

消息固定为：

```text
uint64  timestamp
float32 speed_body_x  # m/s，车体前向为正，倒车为负
float32 yaw_rate      # rad/s，PX4 FRD/NED 正方向
```

所有字段必须有限。倒车速度必须保持负号，不得取绝对值或转换为单独方向位。

### 4.5 `/cmd_vel`映射

上层输入按 ROS REP-103 FLU `geometry_msgs::msg::Twist`解释：

```text
RoverVelocitySetpoint.speed_body_x = cmd_vel.linear.x
RoverVelocitySetpoint.yaw_rate     = -cmd_vel.angular.z
```

ROS 正 `angular.z`表示左转/逆时针；PX4 FRD/NED 正偏航角速度表示右转/顺时针，
因此必须取负号。不要使用 `linear.y/z` 或 `angular.x/y`选择其他控制器。

必须测试以下映射：

| `/cmd_vel (linear.x, angular.z)` | PX4 `(speed_body_x, yaw_rate)` |
| --- | --- |
| `(0.3, 0.0)` | `(0.3, 0.0)` |
| `(-0.3, 0.0)` | `(-0.3, 0.0)` |
| `(0.0, 0.4)` | `(0.0, -0.4)` |
| `(0.3, 0.4)` | `(0.3, -0.4)` |

### 4.6 时间戳与发布频率

两条 DDS 输入消息的 `timestamp`必须：

- 非零；
- 不在 PX4 时间域的未来；
- 在 `COM_OF_LOSS_T`允许的新鲜度范围内；
- 对同一控制周期使用一致的 PX4 时间来源。

使用标准 PX4 ROS 2 时间同步机制，并监控 `/fmu/out/timesync_status`。不得直接把
Linux wall clock 微秒值当作 PX4 启动时间使用而不验证。

机载端应以至少 20 Hz 连续发布两条输入，且发布周期必须显著短于实际
`COM_OF_LOSS_T`。进入 Offboard 前按 PX4 标准要求先建立持续有效的 setpoint
stream；模式切换成功与否必须通过 PX4 状态确认，不能以“DDS publish 成功”代替。

## 5. 机载安全状态机

推荐实现以下状态：

| 状态 | 进入条件 | Rover 输出许可 |
| --- | --- | --- |
| `WAIT_LINK` | 未识别 type 200、MAVLink2/status 或 DDS 未就绪 | 禁止 |
| `QUAD` | 新鲜、无故障、可确认的 Quad 状态 | 禁止 |
| `TRANSITION` | 本机活动请求、ACK/status `IN_PROGRESS`、状态 1 或目标不稳定 | 禁止 |
| `ROVER_READY` | 新鲜、无故障、稳定 Rover，且没有活动变形生命周期 | 等待新的 `/cmd_vel` |
| `ROVER_ACTIVE` | `ROVER_READY`后收到新命令，DDS/时间戳持续有效且 PX4 接受 Offboard | 允许 |
| `FAULT` | 状态 3/4、非零故障、MAVLink 状态过期或协议关联失败 | 禁止 |

进入 `ROVER_READY`不得立即重放缓存。每次以下事件发生时必须清空 `/cmd_vel`
缓存和 DDS 控制 epoch：

- 启动或 MAVLink 重连；
- 收到新的变形 sequence；
- 离开稳定 Rover；
- command 50000 进入 `IN_PROGRESS`或失败；
- 状态未知、故障或过期；
- Offboard/DDS 时间戳失效。

恢复 Rover 输出需要全部满足：

1. MAVLink 2 状态新鲜且无故障；
2. 确认稳定 Rover 且没有活动 command 50000 生命周期；
3. 观察到该稳定状态之后收到严格更新的 `/cmd_vel`；
4. 在该观察之后生成新的 DDS 时间戳；
5. PX4 的模式、解锁和普通 Offboard 检查通过。

PX4 内部还会使用 `transition_completed_timestamp`执行最终 epoch 门控。该字段
当前不在 message 60000 中，外部机载端无法直接比较，所以必须执行上述“观察
稳定状态后清缓存并等待新命令”的规则。

## 6. 停止与故障处理

| 事件 | 必须动作 |
| --- | --- |
| `/cmd_vel`超时或非有限 | 停止声明有效 Rover 控制，触发配置的 PX4 Offboard-loss 行为 |
| MAVLink status 过期 | 立即退出 `ROVER_ACTIVE`，不得因 DDS 仍在线继续运动 |
| 变形开始或收到 `IN_PROGRESS` | 清缓存并停止 Rover Offboard |
| `FAULT`或非零 `fault_reason` | 停止输出，报告故障，不自动 clear fault |
| DDS endpoint 丢失 | 停止控制并等待重新建立完整状态机 |
| 时间同步异常或未来时间戳 | 丢弃样本，不修饰时间戳绕过 PX4 门控 |
| command 50000 被拒绝 | 保留当前安全形态，不自动循环重试 |

从 `ROVER_ACTIVE`退出时，只有在“新鲜、无故障、仍稳定 Rover”的短窗口内可以
发送有限零速度完成受控停止；一旦形态、故障或状态新鲜度无效，应停止声明
Rover Offboard，让 PX4 的 `COM_OF_LOSS_T`和失控动作接管。不得在变形或 Fault
期间持续发送能重新声明 Rover 控制权的 mode bit。

## 7. 建议启动流程

1. 启动 MAVLink 2 客户端，确认 `HEARTBEAT.type=200`。
2. 加载固定 `hybrid_vehicle` dialect，等待新鲜 message 60000。
3. 启动 uXRCE-DDS/ROS 2，并确认三个所需 endpoint 匹配。
4. 验证时间同步和本地单调时钟监控。
5. 初始化为 `WAIT_LINK`，清空所有 `/cmd_vel`和变形请求缓存。
6. 根据 MAVLink 状态进入 `QUAD`、`ROVER_READY`或 `FAULT`。
7. 只有 `ROVER_READY`之后的新命令才能建立至少 20 Hz exact-one-bit stream。
8. 请求 Offboard/解锁后，通过 PX4 状态确认结果；失败不得直接驱动执行器补偿。

## 8. 必须实现的自动化测试

机载软件至少需要覆盖：

1. 四组 FLU 到 FRD/NED 的速度和偏航符号映射。
2. exact-one-bit：每个错误 bit、mixed bits 和全 false 都被拒绝。
3. NaN、Inf、零时间戳、未来时间戳、超时输入全部失效。
4. Quad、Transition、Unknown、Fault 和过期 MAVLink 状态禁止 Rover 输出。
5. 变形前缓存和变形期间输入不能在稳定 Rover 后自动重放。
6. 稳定 Rover 后第一条新命令可以进入正常发布。
7. command 50000 的进度、重复请求、终态成功、已稳定、拒绝、失败和相反目标。
8. 未知 HX65 raw enum 不导致整条 message 60000 被丢弃。
9. `/cmd_vel`丢失、DDS 断开、MAVLink 断开和时间同步异常均进入停止策略。
10. 代码中不存在 `/cmd_vel -> throttle/steering/wheel/direct actuator`旁路。

## 9. 实机验收证据

交付时应保存：

- 机载软件 commit、ROS 2 发行版、RMW 实现；
- `px4_msgs` tag/peeled commit 和 `colcon build`日志；
- MAVLink tag/peeled commit 和生成器版本；
- uXRCE-DDS Agent 启动配置和 endpoint 发现记录；
- PX4 参数导出，特别是 `COM_OF_LOSS_T`及其 loss action；
- DDS 发布频率、时间戳和 timesync 日志；
- MAVLink 2 message 60000、command 50000/ACK 抓包；
- ROS bag、ULog 和实机视频。

实机项目至少包括：前进、倒车、左右偏航、组合运动、零速停止、topic 超时、
Offboard loss、变形期间禁止输出、变形后旧 epoch 拒绝、QGC/机载状态一致性和
HX 执行器故障路径。

## 10. 当前已知协议限制

1. `HybridVehicleStatus`及其 sequence/gear 详细字段不是 DDS 输出 topic。
2. message 60000 尚未传输内部 `sequence_state`、`propulsion_owner/ready`和逐项
   起落架状态。
3. HX65 sensor/backend 数值已经由固件发送，但当前 MAVLink XML 尚无符号枚举。
4. `px4_msgs`仓库已完成消息同一性检查，但尚未在本 WSL 环境完成 ROS 2
   `colcon build`；该测试必须在机载 ROS 工作区执行。
5. 软件构建和单元测试不等于 QGC、ROS 2、无线链路和实机互操作验收。

## 11. 源码与补充文档

- MAVLink/DDS 总体合同：`docs/hybrid/quad-rover-mavlink-dds-contract.md`
- Rover 调参协议：`docs/hybrid/rover_mavlink_tuning_integration.md`
- Rover PID/参数：`docs/hybrid/rover_pid_and_parameter_reference.md`
- DDS topic 清单：`src/modules/uxrce_dds_client/dds_topics.yaml`
- DDS 消息：`msg/OffboardControlMode.msg`、`msg/RoverVelocitySetpoint.msg`
- Hybrid MAVLink stream：`src/modules/mavlink/streams/HYBRID_VEHICLE_STATUS.hpp`
- Rover Offboard 门控：
  `src/modules/rover_differential/DifferentialVelControl/DifferentialVelControl.cpp`
- Hybrid 状态与输出所有权：
  `src/modules/hybrid_vehicle_control/hybrid_vehicle_control.cpp`
- HX 协调序列：`src/lib/hybrid_control/HybridSequenceCoordinator.cpp`

本文取代未提交的旧文件
`PX4-Autopilot-change1_v1.16.1/docs/hybrid/ros2-companion-quad-rover-agent-guide.md`
作为当前机载开发入口。旧文件仅可用于历史对照，不得继续使用其中的旧 PX4、
MAVLink 或未固定 `px4_msgs`版本锚点。

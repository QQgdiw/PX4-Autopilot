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
| 协议实现基准 PX4 commit | `fb1fb9bdeacf32e67a4cc168ed87373e6dcbcb7f` |
| 固件目标 | `zeroone_x6_hybrid` |
| MAVLink 版本 | MAVLink 2 |
| MAVLink dialect | `hybrid_vehicle` |
| MAVLink 仓库 | `https://github.com/QQgdiw/mavlink.git` |
| MAVLink 固定 tag | `qgc-hybrid-rover-tuning-v1.16.1-r2` |
| MAVLink peeled commit | `ec506d609e775035b7c8ed37f09ef05774409281` |
| ROS 2 消息仓库 | `https://github.com/QQgdiw/px4_msgs.git` |
| `px4_msgs` 固定 tag | `hybrid-rover-v1.16.1-r1` |
| `px4_msgs` peeled commit | `e0f41fb57ed9217ba15730854f6c7c92b0262134` |

机载端必须固定使用上述两个 tag，不得使用 stock `common.xml`、上游
`PX4/px4_msgs release/1.16` 或仅凭分支最新状态构建生产版本。

MAVLink tag 由 GitHub ruleset `22434667`保护；`px4_msgs` tag 由 ruleset
`22433564`保护。两条规则均禁止删除和 non-fast-forward 更新。

本次 PX4 更新只改变 MAVLink gitlink，没有修改 uORB `.msg` 或 DDS topic 清单，
因此从 PX4 `7c4fb9e638787435351437c4dfc99e417adffd85` 同步生成的上述
`px4_msgs` r1 tag 仍与当前固件兼容。

兼容性来源的优先级为：

1. 上表固定的 MAVLink 与 `px4_msgs` tag；
2. 本文规定的字段语义、安全门控和状态机；
3. PX4 固件对应 commit 中的消息定义和实现；
4. 普通 PX4/ROS 2 文档。

若这些来源发生冲突，停止集成并记录具体字段、版本和抓包证据，不得自行猜测。

## 2. 总体架构与职责边界

机载电脑的 Rover 控制与安全反馈可以统一使用 DDS；MAVLink 保留给 QGC、显式
变形命令和可选实时调参：

```text
MAVLink 2 Hybrid 客户端（按需）
  <- HEARTBEAT(type=200)
  <- HYBRID_VEHICLE_STATUS(60000)
  <- COMMAND_ACK(command=50000)
  -> MAV_CMD_DO_HYBRID_TRANSITION(50000)
  -> 可选：MAV_CMD_SET_MESSAGE_INTERVAL(511)

ROS 2 Hybrid Supervisor / uXRCE-DDS Rover Offboard
  <- /fmu/out/hybrid_vehicle_status
  <- /fmu/out/vehicle_status
  <- /fmu/out/vehicle_control_mode
  <- /fmu/out/timesync_status
  -> /fmu/in/offboard_control_mode
  -> /fmu/in/rover_velocity_setpoint
  -> 根据 DDS 形态、推进许可、故障、epoch 和模式状态决定是否允许 Rover Offboard
  -> 形态变化或状态失效时清空缓存并执行停止策略
```

职责约束：

- DDS 负责 Rover 形态/故障/推进许可/epoch 反馈、PX4 模式状态，以及向原生
  Rover 控制器输入车体前向速度和偏航角速度。
- MAVLink 继续负责 QGC 状态、显式变形命令、ACK 和可选调参数据；只做
  `/cmd_vel`转发的机载进程不必依赖 MAVLink。
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

使用 MAVLink 状态的客户端仍应以本地单调时钟记录接收时间，并将超过 3 秒未
更新的 message 60000 视为失效。DDS-only Rover 控制不以 message 60000 作为
许可输入，而应使用第 4、5 节规定的 `/fmu/out/hybrid_vehicle_status`新鲜度。

### 3.5 HX-65HM 枚举定义与兼容策略

当前 PX4 uORB 与固定 MAVLink r2 tag 均定义：

```text
SENSOR_HX65   = 4
ACTUATOR_HX65 = 2
```

使用 r2 tag 生成的 C/C++/Python 绑定会提供对应符号名。此次更新只向既有枚举
追加取值，message 60000 的字段和线格式未变化，Payload LEN 仍为 `37`，CRC
Extra 仍为 `57`。command 50000、message 60000 和 60100--60103 的 ID、长度与
CRC 均保持兼容。

机载客户端必须：

- 生产构建固定使用 r2 tag，不从浮动分支或 stock dialect 生成绑定；
- 优先使用 `HYBRID_VEHICLE_SENSOR_HX65` 和
  `HYBRID_VEHICLE_ACTUATOR_HX65` 符号；
- 接受并保存未知的 `uint8_t` 枚举值；
- 不因数值 `sensor_source=4` 或 `actuator_backend=2` 丢弃整条状态消息；
- 不把这两个字段作为唯一的运动许可条件；
- 使用 `current_state`、`fault_reason`、`flags`、ACK 生命周期和状态新鲜度进行
  安全判断；
- 在日志中同时保留符号名和原始值，例如 `HX65(raw=4)`；未来未知值显示为
  `unknown(raw=N)`。

旧 r1 tag 在数值层面仍能接收 `4/2`，但生成 SDK 没有 HX65 符号，仅作为历史
兼容说明，不再是机载端生产版本锚点。新增内部 sequence/gear 线上字段仍须使用
MAVLink 2 extension 或新消息，并单独进行协议版本设计，不能误认为本次枚举补全
已经传输了这些状态。

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
| `/fmu/out/hybrid_vehicle_status` | `px4_msgs::msg::HybridVehicleStatus` | PX4 到机载 |
| `/fmu/out/vehicle_status` | `px4_msgs::msg::VehicleStatus` | PX4 到机载 |
| `/fmu/out/vehicle_control_mode` | `px4_msgs::msg::VehicleControlMode` | PX4 到机载 |
| `/fmu/out/timesync_status` | `px4_msgs::msg::TimesyncStatus` | PX4 到机载 |

`/fmu/out/hybrid_vehicle_status`直接桥接 PX4 内部同名 uORB，包含完整的形态、
故障、transition sequence/完成时间、推进所有者/许可和起落架状态。当前生产者
以 20 ms 周期运行并发布状态，因此 DDS 更新率最高约为 50 Hz；机载实测时必须
记录实际频率和 XRCE 链路负载。

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
| `WAIT_LINK` | Hybrid/Vehicle/ControlMode/Timesync DDS 状态未就绪或过期 | 禁止 |
| `QUAD` | 新鲜、无故障、可确认的 Quad 状态 | 禁止 |
| `TRANSITION` | sequence 非稳定态、形态变化、目标不稳定或可选 MAVLink ACK 为 `IN_PROGRESS` | 禁止 |
| `ROVER_READY` | 新鲜、无故障、稳定 Rover，且 Rover propulsion ready | 等待新的 `/cmd_vel` |
| `ROVER_ACTIVE` | `ROVER_READY`后收到新命令，DDS/时间戳持续有效且 PX4 接受 Offboard | 允许 |
| `FAULT` | 状态 3/4、任一故障、执行器异常或 Hybrid DDS 状态过期 | 禁止 |

进入 `ROVER_READY`不得立即重放缓存。每次以下事件发生时必须清空 `/cmd_vel`
缓存和 DDS 控制 epoch：

- 启动、PX4 重启或 DDS 重连；
- 收到新的变形 sequence；
- `transition_completed_timestamp`变化；
- 离开稳定 Rover；
- sequence 离开 `SEQUENCE_STABLE_ROVER`，或可选 command 50000 进入
  `IN_PROGRESS`/失败；
- 状态未知、故障或过期；
- Offboard/DDS 时间戳失效。

恢复 Rover 输出需要全部满足：

1. Hybrid、Vehicle、ControlMode 和 Timesync DDS 状态新鲜；
2. `current_state=DRIVING`、`sequence_state=SEQUENCE_STABLE_ROVER`、
   `propulsion_owner=PROPULSION_ROVER`且`propulsion_ready=true`；
3. `fault_reason`、`sequence_fault`和执行器保护均为零，执行器在线、健康且配置
   已验证；
4. 观察到新的 `transition_completed_timestamp`后收到严格更新的 `/cmd_vel`；
5. 在该观察之后生成新的 PX4 时间域 DDS 时间戳；
6. `VehicleStatus`和`VehicleControlMode`共同确认 PX4 已解锁并进入 Offboard。

PX4 内部仍会使用 `transition_completed_timestamp`执行最终 epoch 门控。DDS
现在公开同一字段，机载端应重复执行上游缓存隔离；外部检查不能替代 PX4 内部
门控。

## 6. 停止与故障处理

| 事件 | 必须动作 |
| --- | --- |
| `/cmd_vel`超时或非有限 | 停止声明有效 Rover 控制，触发配置的 PX4 Offboard-loss 行为 |
| Hybrid DDS status 过期 | 立即退出 `ROVER_ACTIVE`，停止声明有效 Rover 控制 |
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

1. 启动 uXRCE-DDS/ROS 2，并确认第 4.2 节六个 endpoint 匹配。
2. 等待新鲜的 Hybrid、Vehicle、ControlMode 和 Timesync 状态。
3. 验证当前为 type 200、时间同步有效，并初始化本地单调时钟超时监控。
4. 若机载端还负责变形，再启动固定 r2 dialect 的 MAVLink 2 客户端；否则该步骤
   可以省略。
5. 初始化为 `WAIT_LINK`，清空所有 `/cmd_vel`和变形请求缓存。
6. 根据 DDS Hybrid 状态进入 `QUAD`、`ROVER_READY`或 `FAULT`。
7. 只有 `ROVER_READY`之后的新命令才能建立至少 20 Hz exact-one-bit stream。
8. 请求 Offboard/解锁后，通过 PX4 状态确认结果；失败不得直接驱动执行器补偿。

## 8. 必须实现的自动化测试

机载软件至少需要覆盖：

1. 四组 FLU 到 FRD/NED 的速度和偏航符号映射。
2. exact-one-bit：每个错误 bit、mixed bits 和全 false 都被拒绝。
3. NaN、Inf、零时间戳、未来时间戳、超时输入全部失效。
4. Quad、Transition、Unknown、Fault 和过期 Hybrid DDS 状态禁止 Rover 输出。
5. 变形前缓存和变形期间输入不能在稳定 Rover 后自动重放。
6. 稳定 Rover 后第一条新命令可以进入正常发布。
7. 如果机载端实现变形命令：覆盖 command 50000 的进度、重复请求、终态成功、
   已稳定、拒绝、失败和相反目标。
8. HX65 的 `4/2` 能解析为 r2 符号，未来未知 raw enum 也不导致整条 message
   60000 被丢弃。
9. `/cmd_vel`丢失、DDS 断开和时间同步异常均进入停止策略；使用 MAVLink 命令时
   还必须覆盖其断链。
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

1. message 60000 尚未传输内部 `sequence_state`、`propulsion_owner/ready`和逐项
   起落架状态。
2. MAVLink r2 只补充 HX65 sensor/backend 符号枚举，没有新增逐舵机、gear 或
   sequence 线上字段。
3. `px4_msgs`仓库已完成消息同一性检查，但尚未在本 WSL 环境完成 ROS 2
   `colcon build`；该测试必须在机载 ROS 工作区执行。
4. 软件构建和单元测试不等于 QGC、ROS 2、无线链路和实机互操作验收。

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

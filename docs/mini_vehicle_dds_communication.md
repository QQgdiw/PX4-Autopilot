# Mini Quad/Rover DDS 通信接口

## 1. 文档范围和当前决策

目标固件 `change_mini_v1.16.1` / `hkust_nxt-dual_mini` 已编译 PX4
`uxrce_dds_client`。本文记录该固件当前实际暴露的 uXRCE-DDS/ROS 2 接口，供机载电脑
团队评估和联调。固件源码锚点为
`2f5d1f003b3106060e70df012de59bfc3404837c`。

当前项目的正式车辆控制链路已经选定为 MAVLink2 `MANUAL_CONTROL`，不是 DDS。DDS
可用于标准状态、传感器和后续 ROS 2 集成，但目前不能替代全部 mini 功能，原因包括：

- 没有独立 `HybridVehicleStatus` DDS topic；
- 没有 `RoverVelocitySetpoint` 或 mini 专用 Rover 控制 topic；
- 没有四个 Rover tuning status DDS topic；
- 没有高频 gimbal manager DDS input；
- `/fmu/in/manual_control_input` 缺少语义正确的 DDS input source 枚举；
- mini 自定义 `VehicleStatus` 需要精确匹配的 `px4_msgs`，当前尚未发布该配套仓库。

因此本文描述“已有能力和边界”，不授权机载程序同时用 DDS 与 MAVLink 向同一控制功能
写入数据。

## 2. 架构和版本

```text
PX4 uORB
   <-> uxrce_dds_client (飞控)
   <-> Serial XRCE transport
   <-> MicroXRCEAgent v2.4.2 (机载电脑)
   <-> DDS / ROS 2 nodes
```

PX4 本分支内置的 Micro XRCE-DDS Client project version 为 2.4.0，与最新 v3.x Agent
不兼容。部署基线应固定 `eProsima/Micro-XRCE-DDS-Agent` tag `v2.4.2`，不能只写
“安装最新版”。

硬件目标默认 `UXRCE_DDS_CFG=0`，所以模块虽然已编译，但不会在真机自动启动。配置一个
串口并重启后，由自动生成的 `rc.serial` 唯一启动 client；不要再从 airframe 脚本重复
启动第二个实例。

## 3. px4_msgs 兼容性前置条件

DDS 类型必须与固件构建时的 `.msg` 定义匹配。当前固件的
`msg/versioned/VehicleStatus.msg` 为 `MESSAGE_VERSION=1`，并包含自定义字段
`is_quad_rover`。它在 DDS 中的实际 topic 名为：

```text
/fmu/out/vehicle_status_v1
```

官方包的可复现基线是 `PX4/px4_msgs` 的 `release/1.16`，提交
`392e831c1f659429ca83902e66820d7094591410`。静态对比当前 DDS 使用的 44 种消息类型，
其中 42 种与该提交逐字相同；`ActuatorMotors` 只多了不影响 wire schema 的 `# TOPICS`
注释，而 `VehicleStatus` 实际多了 `is_quad_rover` 字段。因此官方 release/1.16 只能作为
package 骨架，不能直接作为本固件的最终 `px4_msgs`。

更重要的是，自定义字段加入后 `VehicleStatus.MESSAGE_VERSION` 仍保持 1。这违反 versioned
message 的演进约定：当前 ROS 2 端既无法通过 topic 后缀识别这份自定义 schema，也不能
依赖官方 v1 translation。短期必须精确同步本固件的消息定义；长期应把自定义 schema
升级为新版本并提供 v1 到新版本的显式 translation。

DDS 正式集成前必须完成以下交付之一：

1. 从最终 `change_mini_v1.16.1` 固件提交导出完整消息定义，生成并发布一个固定 commit
   的配套 `px4_msgs`；或
2. 提供显式支持该自定义版本的 message translation package，并完成双向类型测试。

机载工程必须把该 `px4_msgs` commit 写入依赖锁定文件。不能按“同为 PX4 v1.16”推断
兼容，也不能混用 upstream `VehicleStatus` 和本项目的其他消息定义。在配套 artifact
交付前，DDS 可以做传输层检查，但不能宣称完整 ROS 2 接口已验收。

生成候选配套包时，先固定官方 package 骨架，再用固件源码覆盖消息定义：

```bash
mkdir -p ~/ros2_ws/src
git clone --branch release/1.16 https://github.com/PX4/px4_msgs.git ~/ros2_ws/src/px4_msgs
git -C ~/ros2_ws/src/px4_msgs checkout 392e831c1f659429ca83902e66820d7094591410
/home/crocodile/PX4-Autopilot-change-mini/Tools/copy_to_ros_ws.sh ~/ros2_ws
cd ~/ros2_ws
colcon build --packages-select px4_msgs
```

随后必须把覆盖结果提交到项目 fork 并锁定该 commit，不能把未提交的工作区当作发布物。
生成后应逐项检查本固件实际使用的 44 种消息类型；已注释的
`VehicleAngularVelocity` 不在有效 DDS 类型集合中。

## 4. 物理接口和参数

HKUST NXT-Dual 串口映射：

| PX4 端口 | 设备 | `UXRCE_DDS_CFG` 值 |
| --- | --- | ---: |
| TELEM1 | `/dev/ttyS1` | 101 |
| TELEM2 | `/dev/ttyS3` | 102 |
| TELEM3 | `/dev/ttyS6` | 103 |
| TELEM4 | `/dev/ttyS7` | 104 |
| GPS1 | `/dev/ttyS0` | 201 |
| GPS2 | `/dev/ttyS2` | 202 |
| Radio Controller | `/dev/ttyS4` | 300 |

推荐将 TELEM2 专用于 DDS，示例配置：

```text
param set MAV_1_CONFIG 0
param set UXRCE_DDS_CFG 102
param set SER_TEL2_BAUD 921600
param set UXRCE_DDS_DOM_ID 0
param set UXRCE_DDS_KEY 1
param set UXRCE_DDS_PTCFG 0
param set UXRCE_DDS_SYNCT 1
param set UXRCE_DDS_SYNCC 0
param set UXRCE_DDS_TX_TO 3
param set UXRCE_DDS_RX_TO -1
param save
reboot
```

`MAV_1_CONFIG=0` 只是释放示例中的 TELEM2。实际安装时要检查全部 `MAV_n_CONFIG` 和其他
串口驱动，确保没有任何模块占用同一设备。MAVLink USB 可以与 DDS TELEM2 并行使用。

参数定义和默认值：

| 参数 | 默认 | 语义 |
| --- | ---: | --- |
| `UXRCE_DDS_CFG` | 0 | 0=禁用；选择串口后重启生效 |
| `UXRCE_DDS_DOM_ID` | 0 | 必须与机载端 `ROS_DOMAIN_ID` 一致 |
| `UXRCE_DDS_KEY` | 1 | 非零；同一 Agent 下每个 client 唯一 |
| `UXRCE_DDS_PTCFG` | 0 | 0 默认、1 localhost-only、2 自定义 participant |
| `UXRCE_DDS_SYNCT` | 1 | DDS/PX4 消息时间戳同步 |
| `UXRCE_DDS_SYNCC` | 0 | 不修改飞控系统时钟 |
| `UXRCE_DDS_TX_TO` | 3 s | 无发送数据后重建连接；小于 1 禁用 |
| `UXRCE_DDS_RX_TO` | -1 | 无接收数据重建连接；小于 1 禁用 |

通用源码为 Ethernet 目标定义了 `UXRCE_DDS_PRT=8888` 和
`UXRCE_DDS_AG_IP=2130706433`（127.0.0.1），但二者带 `requires_ethernet` 条件，未进入
`hkust_nxt-dual_mini` 的生成参数。该目标的 NuttX 配置明确为 `CONFIG_NET is not set`，
因此 `UXRCE_DDS_CLIENT_UDP` 未定义，UDP transport 初始化代码被条件编译排除；CLI
帮助仍可能显示 `-t udp`，但该目标会初始化失败。`UXRCE_DDS_CFG` 也不提供 Ethernet
选项。当前真机 DDS 只支持串口，若未来需要 UDP，必须新增并重新构建带网络栈和实际
Ethernet 硬件支持的独立目标。

默认端口占用为 GPS1=`GPS_1_CONFIG=201`、TELEM1=`MAV_0_CONFIG=101`、RC=
`RC_PORT_CONFIG=300`。TELEM2 默认空闲且 `SER_TEL2_BAUD=921600`，因此是当前首选。
GPS1/GPS2/RC 的默认 baud 为 0，未经显式配置不能用于 client；TELEM3/4 默认仅 57600，
不适合作为高吞吐 DDS 的默认选择。

## 5. Agent 启动

机载 Linux 上先启动 Agent，再给飞控上电或重启 client：

```bash
export ROS_DOMAIN_ID=0
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

`/dev/ttyUSB0` 应替换为连接飞控 TELEM2 的稳定 udev symlink。Agent 日志应出现 client
session 和 DDS entities 创建成功。飞控 NSH 验证：

```text
uxrce_dds_client status
param show UXRCE_DDS_CFG
param show SER_TEL2_BAUD
```

手工 `uxrce_dds_client start -t serial ...` 仅用于诊断；持久部署以参数和 `rc.serial`
为准，避免重启行为与手工命令不一致。

## 6. Topic 名称、QoS 和队列

`dds_topics.yaml` 中的 `/fmu/out` 表示 PX4 发布到 DDS，`/fmu/in` 表示 PX4 从 DDS 接收。
XRCE wire entity 名以 `rt/` 为前缀；ROS 2 CLI 中显示为普通 `/fmu/...` 名称。
当前有效配置包含 24 个 `/fmu/out`、27 个 `/fmu/in`，`subscriptions_multi` 为 0。

当前实现的 QoS 是固定代码行为：

| 方向 | PX4 entity QoS | 应用端建议 |
| --- | --- | --- |
| `/fmu/out` | Best Effort、Transient Local、Keep Last、depth=0 | subscriber 使用 Best Effort；Volatile 可兼容，不假定连接后立即收到历史样本 |
| `/fmu/in` | Best Effort、Volatile、Keep Last、depth=2倍对应 uORB queue | publisher 使用 Best Effort/Volatile，队列保持很小并持续刷新控制 setpoint |

XRCE 创建 entity 使用可靠 control stream，不代表 topic payload 是 Reliable。ROS 2 默认
Reliable subscription 可能与 Best Effort publisher 不兼容；推荐显式使用
`rclcpp::SensorDataQoS()` 或等价的 Best Effort 配置，并按 topic 需要选择 durability。

所有 `/fmu/out` uORB subscription 的最小 poll interval 固定为 10 ms，所以单 topic 的
client 侧发送上限约为 100 Hz，实际频率还受源 topic 更新率、串口带宽和 Best Effort
丢包影响。
即使 PX4 writer 声明 Transient Local，当前 depth 为 0，业务启动仍必须等待一帧新的
有效状态并执行 freshness 检查，不能把“能发现 topic”当成已经获得初始状态。

除 `VehicleStatus` 外，当前列表内消息的 `MESSAGE_VERSION` 为 0，所以 topic 没有版本
后缀；`VehicleStatus` 的运行时名称必须使用 `/fmu/out/vehicle_status_v1`。

## 7. PX4 发布的 topics

| ROS 2 topic | 类型 `px4_msgs/msg/...` | 用途 |
| --- | --- | --- |
| `/fmu/out/register_ext_component_reply` | `RegisterExtComponentReply` | 外部组件注册回复 |
| `/fmu/out/arming_check_request` | `ArmingCheckRequest` | 外部解锁检查请求 |
| `/fmu/out/mode_completed` | `ModeCompleted` | 模式执行完成 |
| `/fmu/out/battery_status` | `BatteryStatus` | 电池状态 |
| `/fmu/out/collision_constraints` | `CollisionConstraints` | 避障约束 |
| `/fmu/out/estimator_status_flags` | `EstimatorStatusFlags` | 估计器有效性 |
| `/fmu/out/failsafe_flags` | `FailsafeFlags` | failsafe 原因位 |
| `/fmu/out/manual_control_setpoint` | `ManualControlSetpoint` | PX4 选中的手动输入，仅监视 |
| `/fmu/out/message_format_response` | `MessageFormatResponse` | 消息格式查询回复 |
| `/fmu/out/position_setpoint_triplet` | `PositionSetpointTriplet` | 导航位置目标 |
| `/fmu/out/sensor_combined` | `SensorCombined` | IMU 组合数据 |
| `/fmu/out/timesync_status` | `TimesyncStatus` | 时间同步质量 |
| `/fmu/out/vehicle_land_detected` | `VehicleLandDetected` | PX4 原生着陆检测 |
| `/fmu/out/vehicle_attitude` | `VehicleAttitude` | 姿态四元数 |
| `/fmu/out/vehicle_control_mode` | `VehicleControlMode` | 当前控制层使能位 |
| `/fmu/out/vehicle_command_ack` | `VehicleCommandAck` | `VehicleCommand` 结果 |
| `/fmu/out/vehicle_global_position` | `VehicleGlobalPosition` | WGS84/global 状态 |
| `/fmu/out/vehicle_gps_position` | `SensorGps` | GPS 原始/融合输入状态 |
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | local NED 状态 |
| `/fmu/out/vehicle_odometry` | `VehicleOdometry` | 里程计状态 |
| `/fmu/out/vehicle_status_v1` | `VehicleStatus` v1 | armed、nav state、vehicle type、mini 标识 |
| `/fmu/out/airspeed_validated` | `AirspeedValidated` | 空速状态 |
| `/fmu/out/vtol_vehicle_status` | `VtolVehicleStatus` | 原生 VTOL 状态；mini 形态不可依赖 |
| `/fmu/out/home_position` | `HomePosition` | Home |

`/fmu/out/vehicle_angular_velocity` 在 YAML 中被注释，当前固件不会创建该 DDS topic。

## 8. PX4 接收的 topics

| ROS 2 topic | 类型 `px4_msgs/msg/...` | 用途/限制 |
| --- | --- | --- |
| `/fmu/in/register_ext_component_request` | `RegisterExtComponentRequest` | 外部组件注册 |
| `/fmu/in/unregister_ext_component` | `UnregisterExtComponent` | 外部组件注销 |
| `/fmu/in/config_overrides_request` | `ConfigOverrides` | 临时配置 override |
| `/fmu/in/arming_check_reply` | `ArmingCheckReply` | 外部解锁检查回复 |
| `/fmu/in/message_format_request` | `MessageFormatRequest` | 消息格式查询 |
| `/fmu/in/mode_completed` | `ModeCompleted` | 外部模式完成通知 |
| `/fmu/in/config_control_setpoints` | `VehicleControlMode` | 外部 mode executor 配置 |
| `/fmu/in/distance_sensor` | `DistanceSensor` | 外部距离传感器 |
| `/fmu/in/manual_control_input` | `ManualControlSetpoint` | 原始手动输入；本项目不作为正式控制入口 |
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | Offboard heartbeat/控制位 |
| `/fmu/in/onboard_computer_status` | `OnboardComputerStatus` | 机载电脑健康状态 |
| `/fmu/in/obstacle_distance` | `ObstacleDistance` | 障碍距离 |
| `/fmu/in/sensor_optical_flow` | `SensorOpticalFlow` | 光流输入 |
| `/fmu/in/goto_setpoint` | `GotoSetpoint` | Go-to 目标 |
| `/fmu/in/telemetry_status` | `TelemetryStatus` | 外部遥测状态 |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | PX4 Offboard 轨迹目标 |
| `/fmu/in/vehicle_attitude_setpoint` | `VehicleAttitudeSetpoint` | 姿态目标 |
| `/fmu/in/vehicle_mocap_odometry` | `VehicleOdometry` | mocap 里程计 |
| `/fmu/in/vehicle_rates_setpoint` | `VehicleRatesSetpoint` | 角速度目标 |
| `/fmu/in/vehicle_visual_odometry` | `VehicleOdometry` | VIO 里程计 |
| `/fmu/in/vehicle_command` | `VehicleCommand` | 标准 PX4 命令 |
| `/fmu/in/vehicle_command_mode_executor` | `VehicleCommand` | mode executor 专用命令入口 |
| `/fmu/in/vehicle_thrust_setpoint` | `VehicleThrustSetpoint` | 低层推力目标 |
| `/fmu/in/vehicle_torque_setpoint` | `VehicleTorqueSetpoint` | 低层力矩目标 |
| `/fmu/in/actuator_motors` | `ActuatorMotors` | 直接电机目标；不属于本项目控制接口 |
| `/fmu/in/actuator_servos` | `ActuatorServos` | 直接舵机目标；不属于本项目控制接口 |
| `/fmu/in/aux_global_position` | `VehicleGlobalPosition` | 辅助 global position |

`actuator_motors/servos`、thrust、torque 等低层 topic 的存在不等于业务程序有权直接使用。
它们可能绕过本项目期望的 Rover/MC 控制层所有权、模式检查或 setpoint freshness 语义，
当前机载程序不得向这些 topic 发布。

## 9. Mini 特有状态与命令边界

### 9.1 形态状态

DDS 中没有 `HybridVehicleStatus`。匹配本项目 `px4_msgs` 后，可用
`/fmu/out/vehicle_status_v1` 判断：

| 条件 | 语义 |
| --- | --- |
| `is_quad_rover=true`, `vehicle_type=VEHICLE_TYPE_ROTARY_WING(1)` | Quad |
| `is_quad_rover=true`, `vehicle_type=VEHICLE_TYPE_ROVER(3)` | Rover |

必须检查消息 timestamp 新鲜度。`/fmu/out/vtol_vehicle_status` 是原生 VTOL topic；mini
控制器不负责把自己的 Quad/Rover 状态写入该 topic，所以不能把它作为当前形态真值。
MAVLink 正式链路使用 `EXTENDED_SYS_STATE`，详见 MAVLink 文档。

### 9.2 VehicleCommand

`/fmu/in/vehicle_command` 能传标准命令，并从 `/fmu/out/vehicle_command_ack` 得到结果。
若只做实验，消息至少正确填写：

- `timestamp`：当前同步时钟 us；
- `command`：如 `VEHICLE_CMD_DO_VTOL_TRANSITION=3000`；
- `param1`：Quad=3，Rover=4；
- `target_system=1`、`target_component=1`；
- 唯一的 `source_system/source_component`；
- `from_external=true`。

但当前产品命令也统一走 MAVLink，DDS 节点不得重复发布同一命令，否则 ACK 关联和控制
所有权会变得不确定。

### 9.3 ManualControlSetpoint

`/fmu/in/manual_control_input` 直接映射到 uORB。消息只定义
`SOURCE_RC` 和 `SOURCE_MAVLINK_0..5`，没有 `SOURCE_DDS`。填 `SOURCE_UNKNOWN` 会被
manual selector 拒绝；伪装成 MAVLink source 虽可能通过当前实现，但会制造错误的
来源语义和后续兼容风险。因此本项目不把它作为正式 DDS 车辆控制入口。

### 9.4 当前不存在的接口

下列名称不在 `dds_topics.yaml`，ROS 2 程序不能订阅/发布后假定其存在：

- `/fmu/out/hybrid_vehicle_status`
- `/fmu/in/rover_velocity_setpoint`
- `/fmu/in/differential_velocity_setpoint`
- `/fmu/in/rover_rate_setpoint`
- `/fmu/in/rover_attitude_setpoint`
- `/fmu/in/rover_position_setpoint`
- `/fmu/out/rover_rate_tuning_status`
- `/fmu/out/rover_attitude_tuning_status`
- `/fmu/out/rover_velocity_tuning_status`
- `/fmu/out/rover_position_tuning_status`
- `/fmu/in/gimbal_manager_set_manual_control`
- `/fmu/in/gimbal_manager_set_attitude`

若未来要正式使用 DDS 控制，应先新增有独立 source/epoch/freshness 语义的 mini 输入消息，
并同时交付匹配的 `px4_msgs`、failsafe 设计和硬件测试；不能通过复用低层 actuator topic
绕开该工作。

## 10. 时间戳和坐标系

`UXRCE_DDS_SYNCT=1` 时，client 测量 Agent OS 与 PX4 HRT 的偏移。PX4 发往 DDS 时，
只对名为 `timestamp`、`timestamp_sample` 的字段加上该偏移；DDS 发往 PX4 时，对非零
值减去偏移并钳制到当前 HRT。输入时间戳为 0 时，代码会直接替换成接收时的当前 HRT，
因此 0 是允许值，但会丢失源采样时刻和链路延迟信息。连续控制和严谨时效检查应填写
Agent/ROS 时钟域的当前微秒值，不能靠恒填 0 掩盖陈旧数据。使用仿真时间时要确保
Agent 和 node 的时间源设计一致。

上述转换不会改变 PX4 的 HRT。`UXRCE_DDS_SYNCC=0` 时也不会设置 PX4 系统 realtime
clock；即使显式启用 SYNCC，修改的也是 system clock，不是 HRT。

监视以下状态：

- 飞控 `uxrce_dds_client status` 中 session、同步和 payload 计数；
- `/fmu/out/timesync_status` 的 offset 和 round-trip 质量；
- 每个控制/命令消息的 timestamp 单调性和接收端新鲜度。

PX4 不自动把 ROS 常用坐标系转换成自己的坐标系：

- local world：NED，x=North、y=East、z=Down；
- body：FRD，x=Forward、y=Right、z=Down；
- `VehicleAttitude.q`：Hamilton 顺序 `[w,x,y,z]`，从 body FRD 旋转到 earth NED；
- `VehicleOdometry`：必须读取 `pose_frame` 和 `velocity_frame`；pose 可为 NED 或
  world-fixed FRD，velocity 还可为 body-fixed FRD，不能无条件按 NED 解读；
- `TrajectorySetpoint`：NED，yaw/yawspeed 按 NED z 轴约定；
- ROS ENU/FLU 数据必须在发布前显式转换，禁止只交换 x/y 或只对 z 取反。

## 11. 验证步骤

先从文档锚点源码构建目标；大段输出写入文件，只查看末尾结果：

```bash
cd /home/crocodile/PX4-Autopilot-change-mini
make hkust_nxt-dual_mini >/tmp/hkust_nxt-dual_mini.log 2>&1
tail -20 /tmp/hkust_nxt-dual_mini.log
```

飞控启动后检查自动配置和连接状态：

```text
param show -q UXRCE_DDS_CFG
param show -q SER_TEL2_BAUD
uxrce_dds_client status
```

在匹配的 `px4_msgs` 已交付并启动 Agent 后执行：

```bash
export ROS_DOMAIN_ID=0
ros2 topic list | sort
ros2 topic info -v /fmu/out/vehicle_status_v1
ros2 topic echo --qos-reliability best_effort /fmu/out/vehicle_status_v1
ros2 topic hz /fmu/out/vehicle_local_position --qos-reliability best_effort
```

保持飞控未解锁，可用全 false 的 `OffboardControlMode` 验证 `/fmu/in` 和零时间戳转换；
随后在 NSH 执行 `listener offboard_control_mode 1`，不得用 actuator topic 做连通性测试：

```bash
ros2 topic pub --once --qos-reliability best_effort --qos-durability volatile \
  /fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode \
  '{timestamp: 0, position: false, velocity: false, acceleration: false, attitude: false, body_rate: false, thrust_and_torque: false, direct_actuator: false}'
```

最低验收项：

- 飞控 client 和 Agent 断开/重连后能够自动恢复，key/domain 不冲突；
- topic list 与第 7、8 节一致，且出现 `vehicle_status_v1` 而非未版本化名称；
- 已知 armed/nav/landed 状态在 ROS 2 侧字段和值一致；
- Quad/Rover 切换时 `vehicle_status_v1` 的 `vehicle_type` 正确变化；
- 坐标和时间戳通过静止、前进、右移/右转等已知动作核对；
- 停止 Agent、拔串口、重启飞控后均不会产生陈旧控制输出；
- 与 MAVLink 并行监视时只读状态一致，没有两个车辆控制 writer。

## 12. 常见故障

| 现象 | 优先检查 |
| --- | --- |
| `uxrce_dds_client not running` | `UXRCE_DDS_CFG` 是否为 0；是否保存并重启 |
| Agent 无 client | 串口设备/交叉接线/地线/波特率；端口是否被 MAVLink 占用 |
| Agent 有 session，ROS 无 topic | `ROS_DOMAIN_ID`、`UXRCE_DDS_DOM_ID`、participant 配置 |
| 找不到 `vehicle_status` | 正确名称是 `/fmu/out/vehicle_status_v1` |
| topic 存在但 echo 无数据 | subscription 是否错误使用 Reliable-only QoS |
| 字段乱码或反序列化失败 | `px4_msgs` 与最终固件消息定义不匹配 |
| 命令有发布无 ACK | target/source/from_external/timestamp、QoS 和 topic 名 |
| 连接周期性重建 | 带宽、TX timeout、供电/地线、Agent v2/v3 不兼容 |

所有“通过”声明都应记录固件 commit、`px4_msgs` commit、Agent 版本、ROS 2 版本、串口
波特率和实际执行的测试项，不能只记录“能看到 topic”。

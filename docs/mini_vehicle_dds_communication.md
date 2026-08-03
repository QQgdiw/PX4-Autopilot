# Mini Quad/Rover DDS 通信接口

## 1. 文档范围与结论

本文记录 `change_mini_v1.16.1` / `hkust_nxt-dual_mini` 当前实际编译的
uXRCE-DDS/ROS 2 接口。固件源码锚点为
`2f5d1f003b3106060e70df012de59bfc3404837c`。

DDS 和 MAVLink 可以完成相似的数据流，但它们不是同一套 wire protocol：

| 需求 | DDS topic | 控制权 |
| --- | --- | --- |
| VIO/SLAM 提供 local NED 位姿/速度 | `/fmu/in/vehicle_visual_odometry` | 不改变；进入 EKF2 |
| 物理 RC 在 Position 模式操控 | VIO 输入 + 物理 RC | 物理 RC/PX4 |
| Quad/Rover local position Offboard | `offboard_control_mode` + `trajectory_setpoint` | ROS 2 节点 |
| 模式、arm、RTL、形态命令 | `/fmu/vehicle_command` service 或 command topic | 按目标模式转交 PX4 |
| 完整 Mission 上传 | 当前 DDS topic 集合不提供 | 使用 MAVLink Mission Protocol |

本项目不把 `/fmu/in/manual_control_input` 当作正式 DDS 控制入口。它是手动输入数据，不是
外部定位或 Offboard setpoint，而且当前消息没有语义正确的 DDS source 枚举。

本文对 Rover 的正式 Offboard 范围只描述 local NED 二维位置 Go-to。当前原生控制器内部
仍有其他 `TrajectorySetpoint` 分支，但 mini 没有专用、经过本项目验收的
`RoverVelocitySetpoint` 或“前向速度 + yaw-rate”接口，本文不把它们列为产品契约。

## 2. 架构、传输与版本

```text
PX4 uORB
   <-> uxrce_dds_client (飞控)
   <-> Serial XRCE transport
   <-> MicroXRCEAgent v2.4.2 (机载电脑)
   <-> DDS / ROS 2 nodes
```

当前硬件目标已编译 `uxrce_dds_client`，但默认 `UXRCE_DDS_CFG=0`，真机不会自动启动。
该目标未编译 NuttX network stack，UDP transport 被条件编译排除；CLI 即使显示
`-t udp` 也不能让当前固件获得网络 DDS。真机 DDS 只支持串口。

PX4 内置 Micro XRCE-DDS Client project version 为 2.4.0，部署固定使用
`eProsima/Micro-XRCE-DDS-Agent` tag `v2.4.2`，不要直接替换为不兼容的 v3.x Agent。

### 2.1 `px4_msgs` 前置条件

DDS 类型必须与固件构建时的 `.msg` wire schema 一致。当前
`msg/versioned/VehicleStatus.msg` 是 `MESSAGE_VERSION=1`，但含项目自定义字段
`is_quad_rover`，运行时 topic 为：

```text
/fmu/out/vehicle_status_v1
```

官方骨架固定为 `PX4/px4_msgs release/1.16 @
392e831c1f659429ca83902e66820d7094591410`。它的 `VehicleStatus` 没有该字段，不能直接
作为本固件最终接口包。短期联调必须从本分支最终消息定义生成并锁定项目
`px4_msgs` commit；正式发布前还必须递增 `MESSAGE_VERSION` 并提供 v1 translation，
避免同一个 `_v1` topic 名承载不同 schema。

候选包生成流程：

```bash
mkdir -p ~/ros2_ws/src
git clone --branch release/1.16 https://github.com/PX4/px4_msgs.git ~/ros2_ws/src/px4_msgs
git -C ~/ros2_ws/src/px4_msgs checkout 392e831c1f659429ca83902e66820d7094591410
/home/crocodile/PX4-Autopilot-change-mini/Tools/copy_to_ros_ws.sh ~/ros2_ws
cd ~/ros2_ws
colcon build --packages-select px4_msgs
```

覆盖结果必须提交到项目 fork 并锁定 commit，不能把未提交工作区当成发布物。

## 3. 串口配置与 Agent

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

TELEM2 默认空闲且默认 baud 为 921600，是当前首选：

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

`MAV_1_CONFIG=0` 只是释放示例中的 TELEM2；还要检查全部串口驱动没有占用同一设备。
MAVLink USB 可与 DDS TELEM2 并行。一个物理串口不能同时运行 MAVLink 和 DDS。

机载 Linux 启动 Agent：

```bash
export ROS_DOMAIN_ID=0
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

实际部署使用稳定 udev symlink。飞控用以下命令确认连接：

```text
uxrce_dds_client status
param show UXRCE_DDS_CFG
param show SER_TEL2_BAUD
```

持久启动由参数和 `rc.serial` 负责。手工 `uxrce_dds_client start` 只用于诊断，不能与
自动实例并行。

## 4. Topic 方向、QoS 与时间

`/fmu/out` 表示 PX4 发布到 DDS，`/fmu/in` 表示 PX4 从 DDS 接收。当前配置包含 24 个
`/fmu/out`、27 个 `/fmu/in`。

`dds_topics.yaml` 中的配置名是 `/fmu/out/vehicle_status`；代码生成器根据
`MESSAGE_VERSION=1` 把运行时 ROS 2 topic 改为 `/fmu/out/vehicle_status_v1`。应用只能使用
运行时带版本后缀的名称。

| 方向 | PX4 QoS | ROS 2 端建议 |
| --- | --- | --- |
| `/fmu/out` | Best Effort、Transient Local、Keep Last、depth=0 | Best Effort；不依赖历史样本 |
| `/fmu/in` | Best Effort、Volatile、Keep Last | Best Effort/Volatile，小队列、持续刷新 |

推荐使用 `rclcpp::SensorDataQoS()` 或等价 Best Effort 配置。XRCE control stream 可靠不
代表 topic payload 是 Reliable。PX4 `/fmu/out` 单 topic 的 client poll 上限约 100 Hz，
实际还受源 topic、串口带宽和 Best Effort 丢包影响。

`UXRCE_DDS_SYNCT=1` 时，client 在 ROS/Agent 时钟与 PX4 HRT 之间转换
`timestamp`、`timestamp_sample`。外部输入应使用 ROS 时钟域的采样微秒时间，并监视
`/fmu/out/timesync_status`。输入 timestamp 为 0 会被替换成接收时刻，虽可工作但会丢失
源采样延迟信息，不用于正式 VIO 或控制验收。

坐标约定：

- world：NED，x=North、y=East、z=Down；
- body：FRD，x=Forward、y=Right、z=Down；
- quaternion：Hamilton `[w,x,y,z]`，body FRD 到 world frame；
- ROS ENU/FLU 必须完整转换位置、速度、姿态、角速度和 covariance。

## 5. VIO/SLAM 提供 local NED 定位

### 5.1 输入 topic

ROS 2 节点以传感器实际频率发布：

```text
/fmu/in/vehicle_visual_odometry
px4_msgs/msg/VehicleOdometry
```

建议 30--50 Hz，并正确填写：

| 字段 | local NED VIO 约定 |
| --- | --- |
| `timestamp_sample` | VIO 采样时刻；不得使用未来时间 |
| `timestamp` | ROS 节点发布时刻 |
| `pose_frame` | `POSE_FRAME_NED` |
| `position[3]` | North/East/Down，m |
| `q[4]` | body FRD 到 NED |
| `velocity_frame` | NED 速度用 `VELOCITY_FRAME_NED` |
| `velocity[3]` | North/East/Down，m/s |
| `angular_velocity[3]` | body FRD，rad/s |
| variance 数组 | 真实方差；未知量用 NaN，不伪造高精度 |
| `reset_counter` | 地图重置/重定位时递增 |
| `quality` | `-1` 失败，`0` 未知/未设置，`1..100` 为质量 |

只有真正与 North/East 对齐的 VIO world frame 才使用 `POSE_FRAME_NED`。任意初始航向
的 SLAM world frame 应使用 `POSE_FRAME_FRD`，并与 EKF yaw 配置一致。

虽然 YAML 中存在 `/fmu/in/vehicle_mocap_odometry`，但当前 mini 目标的 EKF2 只订阅
`vehicle_visual_odometry`；会消费 mocap topic 的 LPE 和 `attitude_estimator_q` 在该目标
均被禁用。因此 VIO、SLAM 或 motion-capture 只要目的是给当前 EKF2 提供外部里程计，
都必须发布到 `/fmu/in/vehicle_visual_odometry`。

### 5.2 EKF2 与状态确认

`EKF2_EV_CTRL` bits 与 MAVLink 文档一致：水平位置=1、垂直位置=2、三维速度=4、yaw=8。
例如水平位置 + 速度为 `5`，再融合视觉 yaw 为 `13`。是否融合高度/yaw、
`EKF2_EV_DELAY`、noise 和 `EKF2_HGT_REF` 必须按传感器质量和实测延迟确定。

默认 `EKF2_EV_QMIN=0` 不执行 quality 门控。即使消息为 `quality=-1`，有限测量仍可能
被融合。VIO 能提供可靠质量时建议设置 `EKF2_EV_QMIN>=1` 并实测阈值；VIO 故障时节点
必须停止发布有限测量，不能只修改 quality 后继续发送。

外部定位只提供测量，不转移控制权。确认以下输出后再允许 Position/Offboard：

- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_odometry`
- `/fmu/out/estimator_status_flags`
- `/fmu/out/failsafe_flags`

VIO-only local NED 不天然提供纬经度或 global Home。标准 PX4 RTL 仍要求有效 global
position 和 Home。

`/fmu/in/aux_global_position` 是当前 DDS 的辅助 global position 输入，需配合
`EKF2_AGP_CTRL` 和真实 WGS84/uncertainty 使用。当前实现只持续融合水平经纬度；参数元数据
中的 vertical bit 尚未形成持续高度融合。该输入也没有完整 GNSS fix、卫星、速度或航向
语义，代码不会替发送端拒绝所有 invalid/stale 组合。发布端必须先校验有效性和新鲜度，
不能把它当成完整外置 GPS，也不能靠固定 global origin 伪造持续有效定位。

## 6. 物理 RC + Position 模式

“机载电脑定位、遥控器操控”不需要 DDS 控制 setpoint：

1. 设置 `COM_RC_IN_MODE=0`，只使用物理 RC。
2. ROS 2 节点持续发布 `vehicle_visual_odometry`。
3. 确认 EKF2 local position/velocity 有效。
4. 用 RC 模式开关进入 Position，或经 `vehicle_command` 请求 Position。
5. 不发布 `/fmu/in/manual_control_input`、`offboard_control_mode` 或
   `trajectory_setpoint`。

经 DDS 请求 Position 时，发送 `VEHICLE_CMD_DO_SET_MODE`：`param1=1`、`param2=3`、
`param3=0`。一次性命令优先使用第 8 节的 `/fmu/vehicle_command` service；也可发布
`/fmu/in/vehicle_command` 并订阅 `/fmu/out/vehicle_command_ack`。最终模式以新的
`/fmu/out/vehicle_status_v1.nav_state` 为准。

## 7. Offboard local position

### 7.1 公共流程

ROS 2 节点必须同时以 10--20 Hz 发布：

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
```

先连续发布至少 1 s，再通过 `/fmu/vehicle_command` service 或
`/fmu/in/vehicle_command` topic 请求 Offboard：

- command：`VEHICLE_CMD_DO_SET_MODE`
- `param1=1`、`param2=6`、`param3=0`
- `target_system=1`、`target_component=1`
- 独立 `source_system/source_component`
- `from_external=true`

`OffboardControlMode` 只设置 `position=true`，其余控制位为 false。每帧 timestamp 必须
更新；`TrajectorySetpoint` 未使用的所有字段填 NaN。模式切换会清除旧目标，因此进入
Offboard 后必须收到一帧新的 `TrajectorySetpoint` 才能产生有效 Rover 输出。ACK 后还要
确认 `vehicle_status_v1.nav_state` 已进入 Offboard。

当前默认 `COM_OF_LOSS_T=1 s`，丢失动作由 `COM_OBL_RC_ACT` 决定。停止节点、暂停 Agent、
拔串口和 VIO 失效必须分别测试，不能只验证正常轨迹。

### 7.2 Quad 与 Rover 字段

| 字段 | Quad position | Rover position Go-to |
| --- | --- | --- |
| `position[0]` | North 目标 | North 目标 |
| `position[1]` | East 目标 | East 目标 |
| `position[2]` | Down 目标 | NaN |
| `velocity[]` | 全 NaN | 全 NaN |
| `acceleration[]` | 全 NaN | 全 NaN |
| `yaw` | 有限目标或 NaN | NaN |
| `yawspeed` | NaN | NaN |

Rover 的 `DifferentialPosControl` 消费 `OffboardControlMode.position` 和
`TrajectorySetpoint.position[0:1]`，再进入原生二维 Go-to/path 控制；巡航速度上限来自
`RO_SPEED_LIM`。

`/fmu/in/goto_setpoint` 只被 Multicopter `GotoControl` 消费，Rover 不订阅它。不要因为
topic 名为 Go-to 就把它用于 Rover。Rover 的正确 DDS Go-to 入口是本节的
`OffboardControlMode + TrajectorySetpoint`。

本项目不把 velocity/attitude/body-rate 分支列为 mini Rover 产品接口。不得用低层
`actuator_motors`、thrust 或 torque topic 绕过原生控制器和 mini 输出安全门控。

## 8. Mission、RTL 与命令

当前 client 编译了可靠请求/响应的 ROS 2 service：

```text
/fmu/vehicle_command
px4_msgs/srv/VehicleCommand
```

Arm、模式和 RTL 等一次性命令优先用该 service，response 为 `VehicleCommandAck`。当前
实现只保存一个 pending command，并仅以 command ID 和 ACK 时间匹配；调用端必须严格
串行、同一时间只保留一个请求并设置超时。另有一个尚未通过本项目实机验收的源码风险：
service pending flag 没有显式初始化。因此正式部署前必须做首次调用、拒绝、超时和重连
测试；若 service 未通过验收，使用 `/fmu/in/vehicle_command` topic，并从
`/fmu/out/vehicle_command_ack` 按 command、source/target 和 timestamp 关联结果。

常用 `VehicleCommand` payload：

| 操作 | command | 关键参数 |
| --- | --- | --- |
| Arm/disarm | `VEHICLE_CMD_COMPONENT_ARM_DISARM` (400) | `param1=1/0`, `param2=0` |
| Position | `VEHICLE_CMD_DO_SET_MODE` (176) | `param1=1`, `param2=3`, `param3=0` |
| Offboard | `VEHICLE_CMD_DO_SET_MODE` (176) | `param1=1`, `param2=6`, `param3=0` |
| Auto Mission | `VEHICLE_CMD_DO_SET_MODE` (176) | `param1=1`, `param2=4`, `param3=4` |
| Quad RTL | `VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` (20) | 其余参数为 0 |
| Quad/Rover | `VEHICLE_CMD_DO_VTOL_TRANSITION` (3000) | `param1=3/4` |

正常 disarm 不使用强制 magic value。ACK 后分别用 `vehicle_status_v1.arming_state`、
`nav_state` 和 `vehicle_type` 确认最终状态，不能只依赖 service 返回成功。

当前 `dds_topics.yaml` 没有 Mission item 上传/下载 topics，因此 DDS 节点不能实现完整
MAVLink Mission Protocol。推荐流程：

1. 用 QGC 或 MAVLink 程序上传并确认 mission；
2. DDS 节点可用 `VehicleCommand` 请求 Auto Mission；
3. 由 PX4 Navigator 生成目标并执行；
4. 从 `vehicle_status_v1`、`position_setpoint_triplet` 和 ACK 监视状态。

本机没有 SD 卡时，默认 dataman 文件后端无法可靠保存任务。设置
`SYS_DM_BACKEND=1`、保存并重启，再用 `dataman status` 确认 RAM backend；Mission、
Fence 和 Rally 会在重启后丢失，因此每次启动都要重新上传并确认。

Auto Mission 模式命令为 `VEHICLE_CMD_DO_SET_MODE`，`param1=1`、`param2=4`、
`param3=4`。Quad 主动 RTL 可发送 `VEHICLE_CMD_NAV_RETURN_TO_LAUNCH`（20）。两者都必须
先确认所需定位有效，并在 ACK 后确认实际 nav state。Mission 执行期间禁止切换
Quad/Rover；Rover 到达最后一项后可能继续 AUTO_MISSION 且 armed，业务端必须显式退出
Auto、停车并按安全条件 disarm。

当前 mini Rover 的 Direct RTL 会因 Rover 持续 `landed=true` 而直接进入 IDLE，不生成
返航轨迹，即使 ACK Accepted 且 nav state 显示 AUTO_RTL。Mission 中的 RTL item 同样
受影响。修复并实车验证前，Rover RTL 不得作为命令或 failsafe 安全功能。

标准 RTL 要求 local/global position、local altitude 和 Home。只有 local VIO 时，不应
请求标准 RTL；如需回到本地起点，应由 Offboard 节点发布 local position Go-to，并明确
它不是 PX4 RTL。

Quad/Rover 切换使用 `VEHICLE_CMD_DO_VTOL_TRANSITION`（3000），`param1=3` 为 Quad、
`param1=4` 为 Rover。Quad -> Rover 仍受新鲜 landed 状态门控。任何命令都应填写
timestamp、target/source、`from_external=true`，并关联
`/fmu/out/vehicle_command_ack`。

## 9. 当前状态与目标状态

| 信息 | DDS output | 说明 |
| --- | --- | --- |
| armed、nav state、机型 | `/fmu/out/vehicle_status_v1` | 含 mini 自定义字段 |
| current local position/velocity | `/fmu/out/vehicle_local_position` | EKF2 NED |
| current odometry | `/fmu/out/vehicle_odometry` | 位姿、速度、frame |
| current global position | `/fmu/out/vehicle_global_position` | RTL 前检查 |
| Home | `/fmu/out/home_position` | RTL 前检查 |
| attitude | `/fmu/out/vehicle_attitude` | body FRD 到 NED |
| landing state | `/fmu/out/vehicle_land_detected` | Quad -> Rover 门控来源 |
| control mode bits | `/fmu/out/vehicle_control_mode` | 当前控制层 |
| Mission/RTL global target | `/fmu/out/position_setpoint_triplet` | Navigator triplet |
| command result | `/fmu/out/vehicle_command_ack` | 还需状态确认 |

当前 DDS 输出没有 `trajectory_setpoint`、`vehicle_local_position_setpoint`、
`rover_position_setpoint` 或四个 Rover controller status。发送 Offboard 的节点必须保留
自己最后一次有效 local 目标；DDS 不能回读一个权威 Rover local target。若需要实时
Rover response/setpoint 调参，当前接口仍是 USB MAVLink 私有 tuning streams。

DDS 中没有 `HybridVehicleStatus`。匹配本项目 `px4_msgs` 后，用
`vehicle_status_v1` 判断形态：

| 条件 | mini 形态 |
| --- | --- |
| `is_quad_rover=true`, `vehicle_type=VEHICLE_TYPE_ROTARY_WING` | Quad |
| `is_quad_rover=true`, `vehicle_type=VEHICLE_TYPE_ROVER` | Rover |

`/fmu/out/vtol_vehicle_status` 是原生 VTOL topic，mini 不把自己的形态真值写入该 topic，
不能依赖它判断 Quad/Rover。

## 10. 当前全部 `/fmu/out` topics

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
| `/fmu/out/position_setpoint_triplet` | `PositionSetpointTriplet` | Navigator global 目标 |
| `/fmu/out/sensor_combined` | `SensorCombined` | IMU 组合数据 |
| `/fmu/out/timesync_status` | `TimesyncStatus` | 时间同步质量 |
| `/fmu/out/vehicle_land_detected` | `VehicleLandDetected` | PX4 原生着陆检测 |
| `/fmu/out/vehicle_attitude` | `VehicleAttitude` | 姿态四元数 |
| `/fmu/out/vehicle_control_mode` | `VehicleControlMode` | 当前控制层使能位 |
| `/fmu/out/vehicle_command_ack` | `VehicleCommandAck` | `VehicleCommand` 结果 |
| `/fmu/out/vehicle_global_position` | `VehicleGlobalPosition` | WGS84/global 状态 |
| `/fmu/out/vehicle_gps_position` | `SensorGps` | GPS 状态 |
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | local NED 状态 |
| `/fmu/out/vehicle_odometry` | `VehicleOdometry` | 融合里程计状态 |
| `/fmu/out/vehicle_status_v1` | `VehicleStatus` v1 | armed、nav state、mini 标识 |
| `/fmu/out/airspeed_validated` | `AirspeedValidated` | 空速状态 |
| `/fmu/out/vtol_vehicle_status` | `VtolVehicleStatus` | 原生 VTOL 状态；mini 不依赖 |
| `/fmu/out/home_position` | `HomePosition` | Home |

`/fmu/out/vehicle_angular_velocity` 在 YAML 中被注释，当前不会创建。

## 11. 当前全部 `/fmu/in` topics

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
| `/fmu/in/manual_control_input` | `ManualControlSetpoint` | 非本项目正式控制入口 |
| `/fmu/in/offboard_control_mode` | `OffboardControlMode` | Offboard heartbeat/控制位 |
| `/fmu/in/onboard_computer_status` | `OnboardComputerStatus` | 机载电脑健康状态 |
| `/fmu/in/obstacle_distance` | `ObstacleDistance` | 障碍距离 |
| `/fmu/in/sensor_optical_flow` | `SensorOpticalFlow` | 光流输入 |
| `/fmu/in/goto_setpoint` | `GotoSetpoint` | 仅 Quad/MC GotoControl |
| `/fmu/in/telemetry_status` | `TelemetryStatus` | 外部遥测状态 |
| `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | Offboard 轨迹目标 |
| `/fmu/in/vehicle_attitude_setpoint` | `VehicleAttitudeSetpoint` | 姿态目标；非 Rover 正式契约 |
| `/fmu/in/vehicle_mocap_odometry` | `VehicleOdometry` | 当前 mini EKF2 不消费 |
| `/fmu/in/vehicle_rates_setpoint` | `VehicleRatesSetpoint` | 角速度目标；非 Rover 正式契约 |
| `/fmu/in/vehicle_visual_odometry` | `VehicleOdometry` | VIO 里程计 |
| `/fmu/in/vehicle_command` | `VehicleCommand` | 标准 PX4 命令 |
| `/fmu/in/vehicle_command_mode_executor` | `VehicleCommand` | mode executor 专用入口 |
| `/fmu/in/vehicle_thrust_setpoint` | `VehicleThrustSetpoint` | 低层推力；业务端禁用 |
| `/fmu/in/vehicle_torque_setpoint` | `VehicleTorqueSetpoint` | 低层力矩；业务端禁用 |
| `/fmu/in/actuator_motors` | `ActuatorMotors` | 直接电机；业务端禁用 |
| `/fmu/in/actuator_servos` | `ActuatorServos` | 直接舵机；业务端禁用 |
| `/fmu/in/aux_global_position` | `VehicleGlobalPosition` | 辅助 global position |

低层 actuator、thrust、torque topics 的存在不代表机载业务程序有权使用。它们可能绕过
Quad/Rover 原生控制器、模式检查、freshness 和 mini 形态输出门控。

## 12. 当前不存在的接口

以下名称不在 `dds_topics.yaml`，不能假定存在：

- `/fmu/out/hybrid_vehicle_status`
- `/fmu/in/rover_velocity_setpoint`
- `/fmu/in/differential_velocity_setpoint`
- `/fmu/in/rover_rate_setpoint`
- `/fmu/in/rover_attitude_setpoint`
- `/fmu/in/rover_position_setpoint`
- `/fmu/out/rover_rate_status`
- `/fmu/out/rover_attitude_status`
- `/fmu/out/rover_velocity_status`
- `/fmu/out/rover_position_status`
- `/fmu/out/trajectory_setpoint`
- `/fmu/in/gimbal_manager_set_manual_control`
- `/fmu/in/gimbal_manager_set_attitude`

上述四个 `rover_*_status` 是控制器实际 uORB 名称；私有
`ROVER_*_TUNING_STATUS` 是 MAVLink 消息名，不是 DDS/uORB topic。云台连续控制和 Rover
实时 tuning 仍使用 MAVLink。完整 Mission 上传同样使用 MAVLink；DDS `VehicleCommand`
只能请求执行已存在的任务或模式。

## 13. 联调顺序

1. 固定 Agent 和项目 `px4_msgs` 版本，确认串口 session、entities、QoS 和 timesync。
2. 只发布 VIO，核对 NED/FRD、尺度、variance、reset 和 EKF2 输出。
3. `COM_RC_IN_MODE=0`，验证物理 RC Position；确认 VIO topic 不改变手动输入源。
4. 以 exact-one-bit `position` 验证 Quad 三维、Rover 二维 Offboard，并注入 topic/Agent 断流。
5. 无 SD 时确认 RAM dataman，再通过 MAVLink 上传 mission；有 global/Home 后只验收 Quad
   RTL，当前 Rover RTL 记录为已知不可用。
6. 验证 `VehicleCommandAck`、最终 nav state、形态状态和所有消息 freshness。
7. 记录 DDS 当前缺失的 target/tuning/gimbal 接口，不用低层 topic 临时绕过。

本文只证明当前源码中的 topic 和数据流。尚未完成 ROS 2 Agent、VIO、串口、模式、实车
运动或 failsafe 硬件验收时，不能宣称 DDS 控制链路已经通过整机测试。

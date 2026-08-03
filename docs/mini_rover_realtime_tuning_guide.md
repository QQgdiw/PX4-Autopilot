# Mini Rover 实时调参功能开发指南

## 1. 任务边界与固定基线

本指南交接给负责 QGroundControl 的 Agent。当前 PX4 Agent 不修改 QGC 仓库。

- QGC 仓库：`https://github.com/nanjia24/qgroundcontrol.git`
- QGC 分支：`codex/joystick-aux-px4-development`
- 已核对 HEAD：`754135601a53d7650ddeb6562ca5a5cd2167880c`
- QGC 功能分支：`codex/mini-rover-realtime-tuning`
- QGC 独立工作树：
  `E:\workspace\QGC\qgroundcontrol-worktrees\mini-rover-realtime-tuning`
- PX4 固件工作树：`/home/crocodile/PX4-Autopilot-change-mini`
- PX4 分支：`change_mini_v1.16.1`
- PX4 固件源码锚点：`2f5d1f003b3106060e70df012de59bfc3404837c`
- PX4 构建目标：`make hkust_nxt-dual_mini`
- mini 机型识别参数：`HYBR_QUAD_ROV=1`
- mini 心跳机型：`MAV_TYPE=22`，即 `MAV_TYPE_VTOL_FIXEDROTOR`

`898aee795c...` 是另一个 `change1_v1.16.1` 项目写入上级 `AGENTS.md` 的 QGC
适配基准，不是本任务从目标 QGC 仓库核验得到的提交。本任务不得用它替代上述
`754135601a...` 基线。

第一阶段只实现：

1. Rate、Attitude、Velocity、Position/Path 四组实时 Response 与 Setpoint 曲线；
2. 每组对应的 PX4 参数查看和编辑；
3. USB 链路上的正确启停、失效显示和重连恢复。

第一阶段不实现自动调参、参数推荐、自动驾驶模式切换、日志分析器或低带宽数传优化。
QGC Agent 必须自行完成 commit 和 push；仓库初始化、执行纪律和最终报告格式见随本
GUIDE 交付的 `mini_rover_qgc_agent_prompt.md`。

## 2. 已确认的现状

目标 QGC 没有 PX4 Rover PID Tuning 页面。

- `src/AutoPilotPlugins/PX4/PX4TuningComponent.cc` 只路由 Fixed Wing、
  Multirotor、VTOL 和 Spacecraft。普通 `MAV_TYPE_GROUND_ROVER` 返回空页面。
- `PX4TuningComponentVTOL.qml` 只有 Multirotor 标签；mini 虽然包含 Rover，仍会
  被识别为 VTOL，因此只增加 Ground Rover case 不能覆盖 mini。
- 现有 Multirotor Rate/Attitude 页面请求 `ATTITUDE_QUATERNION` 和
  `ATTITUDE_TARGET`；Velocity/Position 页面请求 `LOCAL_POSITION_NED` 和
  `POSITION_TARGET_LOCAL_NED`。

不能只复制现有 Multirotor QML。PX4 的标准 target stream 订阅的是
`vehicle_attitude_setpoint`、`vehicle_rates_setpoint` 和
`vehicle_local_position_setpoint`，而差速 Rover 实际控制链使用 `rover_*` topic。
直接复用会显示空值、旧值或 Quad 控制器的目标值。

## 3. 第一阶段架构决定

采用四个专用、紧凑的 MAVLink2 tuning status 消息，参数编辑继续使用 QGC 已有的
PARAM/Fact 通道。四个消息只传实时控制状态，不重复传参数值。

协议已经固定在：

- fork：`https://github.com/QQgdiw/mavlink.git`
- branch：`mini-rover-tuning-v1.16.1`
- parent baseline：official MAVLink `master @ 5bfd76d80281f6027134e854aafe6cb3dbfbe9e1`
- commit：`07c6964a8fcc364c49d394f0bf0275b9fc05857d`
- PX4 dialect：`mini_rover`（只包含 `common + mini Rover`）
- QGC dialect：`qgc_mini_rover`（包含原 `all + mini Rover`）

| 消息 | ID | Payload | CRC extra | 完整未签名 MAVLink2 帧 |
| --- | ---: | ---: | ---: | ---: |
| `ROVER_RATE_TUNING_STATUS` | 60100 | 27 B | 147 | 39 B |
| `ROVER_ATTITUDE_TUNING_STATUS` | 60101 | 23 B | 85 | 35 B |
| `ROVER_VELOCITY_TUNING_STATUS` | 60102 | 43 B | 217 | 55 B |
| `ROVER_POSITION_TUNING_STATUS` | 60103 | 44 B | 90 | 56 B |

这些 ID 已扫描当前 composite 中的全部 dialect，无冲突。ID 大于 255，只能通过
MAVLink2 发送；MAVLink1 不是本功能支持或验收链路。启用 MAVLink2 signing 时，每帧
还要增加 13 B。协议固定测试命令为 `python3 tests/test_mini_rover_dialect.py`。

不采用以下方案：

- `DEBUG_FLOAT_ARRAY` / `NAMED_VALUE_FLOAT`：缺少稳定类型和原子样本语义，正式
  MAVLink 定义也不建议将 debug message 用作生产接口。
- ArduPilot `PID_TUNING`：当前 mini 使用基于 `common.xml` 的 `mini_rover`
  dialect，PX4 没有该 stream，其 axis 也不能正确表达四层
  Rover/Pure Pursuit 控制链。
- 条件改写标准 target message：无法完整表达 Rover 滤波后 body-X 速度、活动路径
  目标、有效位和控制层状态，也容易污染 Quad 模式的标准语义。

## 4. 实时消息契约

所有消息必须包含 PX4 HRT 时间戳 `time_usec` 和 `valid_flags`。活动控制器帧的
`time_usec` 是对应 status 的控制周期时间戳；Quad/inactive 终止帧使用最新的
`hybrid_vehicle_status.timestamp` 与 `vehicle_status.timestamp` 中的较大值，不能把
它当作控制器采样。QGC 只对有效活动帧按源时间戳追加曲线，终止帧只用于
立即清除有效状态；不能把 QGC 的 10 ms Timer 当作样本时钟。

### 4.1 Rate

| 字段 | PX4 数据源 | 单位/语义 |
| --- | --- | --- |
| `time_usec` | `rover_rate_status.timestamp` | us，HRT |
| `yaw_rate_response` | `measured_yaw_rate` | rad/s，已应用 `RO_YAW_RATE_TH` |
| `yaw_rate_setpoint` | `adjusted_yaw_rate_setpoint` | rad/s，斜坡限制后的实际目标 |
| `integral` | `pid_yaw_rate_integral` | 控制器积分贡献 |
| `control_output` | `rover_steering_setpoint.normalized_speed_diff` | 归一化差速输出 |

QGC 曲线显示时将 rad/s 转为 deg/s。`RO_YAW_RATE_TH` 只过滤实测噪声，绝不能
拿它过滤或隐藏低幅 yaw-rate setpoint。

### 4.2 Attitude

| 字段 | PX4 数据源 | 单位/语义 |
| --- | --- | --- |
| `time_usec` | `rover_attitude_status.timestamp` | us，HRT |
| `yaw_response` | `measured_yaw` | rad，实际航向 |
| `yaw_setpoint` | `adjusted_yaw_setpoint` | rad，斜坡限制后的航向目标 |
| `yaw_rate_setpoint` | `rover_rate_setpoint.yaw_rate_setpoint` | rad/s，外环输出诊断 |

PX4 的 `RoverAttitudeStatus.msg` 已统一标为 rad。QGC 显示 deg，并对 yaw 做连续解包裹：
response 按上一采样连续展开，setpoint 选择与当前 response 最接近的 `2*pi` 等价值。
时间戳回退、失效或重连时重置解包裹状态。`yaw_rate_setpoint` 是本层输出，使用
`OUTPUT_PRIMARY_VALID`，不是第二个 response 轴。

### 4.3 Velocity

| 字段 | PX4 数据源 | 单位/语义 |
| --- | --- | --- |
| `time_usec` | `rover_velocity_status.timestamp` | us，HRT |
| `speed_body_x_response` | `measured_speed_body_x` | m/s，控制器实际使用的 body-X 速度 |
| `speed_body_x_setpoint` | `adjusted_speed_body_x_setpoint` | m/s，斜坡限制后的实际目标 |
| `integral_body_x` | `pid_throttle_body_x_integral` | 控制器积分贡献 |
| `throttle_body_x` | `rover_throttle_setpoint.throttle_body_x` | 归一化输出 |

不要用 `LOCAL_POSITION_NED.vx/vy` 在 QGC 临时投影来冒充 response；Rover 控制器使用
姿态变换及 `RO_SPEED_TH` 处理后的 body-X 值。

PX4 producer 已修正状态零初始化并填写 raw body-X setpoint。wire schema 同时预留
body-Y response/setpoint/integral/output 和对应 secondary 有效位；Differential 第一阶段
必须保持这些字段为 NaN、有效位为 0，QGC 不显示 Lateral 轴。

### 4.4 Position/Path

Position 层是 Pure Pursuit 路径跟踪，不是 Position PID。页面标题和说明必须使用
`Position / Path` 或 `Path Tracking`，不能声称存在位置 PID。

PX4 已新增原子 `rover_position_status`。它在 Manual course、Auto 和 Go-to 分支记录
控制器实际使用的活动目标；spot-turn、Auto IDLE、无效 target 和无路径停止状态明确
保持 target/path invalid。

| 字段 | PX4 数据源 | 单位/语义 |
| --- | --- | --- |
| `time_usec` | 新 `rover_position_status.timestamp` | us，HRT |
| `position_north/east` | 当前 `_curr_pos_ned` | m，同一 local NED frame |
| `target_north/east` | 当前分支实际传给 Pure Pursuit 的 target | m，同一 frame |
| `crosstrack_error` | `pure_pursuit_status` | m |
| `lookahead_distance` | `pure_pursuit_status` | m |
| `target_bearing` | `pure_pursuit_status` | rad |
| `distance_to_waypoint` | `pure_pursuit_status` | m |
| `xy_reset_counter` | `vehicle_local_position.xy_reset_counter` | EKF local-frame reset检测 |

页面至少提供 North、East 两个 Response/Setpoint 轴和 Cross-track 诊断。收到新的
`xy_reset_counter` 时清空 Position 图，不能把坐标系跳变画成控制误差。没有实际路径
目标的 spot-turn 等状态应把 target 标为 invalid/NaN。

### 4.5 有效位和陈旧数据

`valid_flags` 是固定的 16-bit bitmask：

| 值 | 名称 | 用途 |
| ---: | --- | --- |
| 1 | `CONTROLLER_ACTIVE` | 对应 producer 通过 sanity check 并实际执行 |
| 2 / 4 | `RESPONSE_PRIMARY_VALID` / `SETPOINT_PRIMARY_VALID` | Rate、Attitude、body-X |
| 8 / 16 | `RESPONSE_SECONDARY_VALID` / `SETPOINT_SECONDARY_VALID` | 预留 body-Y |
| 32 / 64 | `INTEGRAL_PRIMARY_VALID` / `INTEGRAL_SECONDARY_VALID` | X/Y 积分 |
| 128 / 256 | `OUTPUT_PRIMARY_VALID` / `OUTPUT_SECONDARY_VALID` | 本层输出；Attitude 也使用 primary |
| 512 / 1024 / 2048 | `POSITION_VALID` / `TARGET_VALID` / `PATH_VALID` | Position/Path |

四条消息都带 `drive_type`。本阶段 Rover 形态为 `DIFFERENTIAL=1`；Quad 形态为
`UNKNOWN=0` 且所有 flags 清零。QGC 必须拒绝把其他 `drive_type` 冒充为 Differential
支持。每条 stream 的 Rover 形态门控同时要求：

1. `vehicle_status.timestamp>0`、新鲜度小于 1 s，且
   `vehicle_status.vehicle_type==ROVER`；
2. `hybrid_vehicle_status.timestamp>0`、新鲜度小于 200 ms，且状态为
   `DRIVING`。

形态门控失败时，stream 发送 `drive_type=UNKNOWN`、flags=0 和 NaN 曲线字段的
终止帧。形态有效但本调度周期没有新 controller status 时不发帧，由 QGC 的
500 ms 超时处理断流。只有取到新 status 且 producer 原子发布的
`status.active=true` 时才发送有效活动数据；status 已更新但 producer inactive 时发送
`drive_type=DIFFERENTIAL`、flags=0 和 NaN 曲线字段的终止帧。Position 使用相同的
形态/status/active 语义。

三层 output 只在 output 与 status 时间戳完全相等时置位，不存在原 ±200 ms 拼接窗口。
IMU/姿态/本地速度 response 超过 500 ms 未更新时置 NaN；Position 还要求
`xy_valid`。QGC 收到 invalid 后立即停止追加相应曲线并显示数据不可用。超过 500 ms
没有新 MAVLink 源时间戳、时间戳回退或飞控重启时，FactGroup 清为 NaN 并重置图表
状态，禁止保持最后一个值继续绘制。

## 5. PX4 producer 交付状态

QGC-only 修改无法完成正确曲线；以下固件接口已经交付：

1. `hkust_nxt-dual_mini` 固定编译 `mini_rover` dialect；QGC 使用同一 commit 的
   `qgc_mini_rover` composite。
2. Differential Rate、Attitude、Velocity、Position/Path 四层状态和四条 stream 已实现。
3. Position 覆盖 Manual course、Auto、Go-to；spot-turn、IDLE 和 invalid target 不发布
   旧路径。Manual/Auto/Offboard/Go-to source 或 `nav_state` 变化时，producer 更新
   `_source_epoch`、清除已锁存目标并消费非当前源的待处理输入；只有时间戳晚于
   epoch 的当前源样本才能重新建立路径。EKF XY reset 通过 counter 通知 QGC 清图。
4. 四条 stream 已注册但未加入任何默认 stream 配置，只响应
   `MAV_CMD_SET_MESSAGE_INTERVAL` 请求。
5. Quad 或对应控制层 inactive 时发送 flags=0 的终止状态；response 陈旧时逐字段失效。
6. 固件源码锚点 `2f5d1f003b3106060e70df012de59bfc3404837c` 的 Flash image 为
   `1,699,944 B / 1,792 KiB`（92.64%）；PX4 tests `146/146`、MAVLink dialect tests
   `2/2` 通过。产物 SHA-256：`.px4`
   `c8fb8aaeebe40846292c2fddd21d919ce5a7f736d3e83347c0b31794e53c9bd0`，`.bin`
   `b06be017cfbdc706493e294f3759ebf5368e9bf944b79794cb88e6d1e23c6a97`。这些结果是
   构建/自动测试证据，不替代 USB 与实车验收；任何后续固件源码或
   MAVLink gitlink 修改都会使该尺寸、哈希和测试证据失效，必须重新验证。

核心 Rover 控制器只发布 uORB 状态；MAVLink stream 负责序列化，不能把 MAVLink
发送逻辑写进控制器。这一边界保持控制逻辑与传输层解耦。

## 6. QGC 实施步骤

### 6.1 固定 MAVLink 依赖

目标 QGC 分支当前在 `cmake/CustomOptions.cmake` 使用 stock `mavlink/mavlink`、commit
`b1fb5a1a32c41c6e46fea70600d626a0b5a8edbe`、dialect `all`。QGC Agent 应将其改为：

- repository：`https://github.com/QQgdiw/mavlink.git`
- commit：`07c6964a8fcc364c49d394f0bf0275b9fc05857d`，禁止只 pin 可移动 branch HEAD
- dialect：`qgc_mini_rover`

`qgc_mini_rover.xml` 已组合原 `all.xml` 与 `mini_rover.xml`，因此不能把 QGC 直接改成
PX4 使用的精简 `mini_rover`。QGC 构建仍须验证现有全部 MAVLink 消息可生成，并为上表
四个 ID、payload length 和 CRC extra 加静态断言或单测。

修改 pin 后必须重新配置或清理 QGC 对应的 CMake dependency cache，并从生成日志或
产物路径确认实际使用 `07c6964a...`；禁止让旧 `b1fb5a1a...` 下载缓存造成假通过。

不要借用上级项目的 `qgc-hybrid-*` tag；它们服务于另一套 message 60000 契约。

### 6.2 每 Vehicle 独立 FactGroup

新增：

- `src/Vehicle/FactGroups/RoverTuningFactGroup.h`
- `src/Vehicle/FactGroups/RoverTuningFactGroup.cc`
- `src/Vehicle/FactGroups/RoverTuningFactGroup.json`

并更新 `src/Vehicle/FactGroups/CMakeLists.txt`、`Vehicle.h` 和 `Vehicle.cc`。

`Vehicle` 必须各自拥有一个 `RoverTuningFactGroup`，通过现有
`FactGroup::handleMessage` 分发解码四个消息，并以 Q_PROPERTY 暴露给 QML。不要把
有状态实例放进 `PX4FirmwarePlugin::factGroups()`；PX4 FirmwarePlugin 是多 Vehicle
共享单例，会导致多机数据串线。

FactGroup 在 C++ 中统一做 rad -> deg、valid/NaN、超时和 yaw unwrap 所需的状态
管理，QML 只绑定显示值。消息来自非当前 Vehicle 时必须拒绝；时间戳倒退按飞控重启/
数据 epoch 变化清理状态。`flags=0` 是必须消费的状态清除事件，不能静默丢弃；
`drive_type=UNKNOWN && flags=0` 通常表示正常 Quad/inactive，而不是协议错误。某字段未置
有效位时应把对应 Fact 清为 NaN；只有字段被标为 valid 却不是有限值等自相矛盾样本，
才拒绝该字段并记录诊断。Ackermann/Mecanum 是第一阶段不支持，不能套用 Differential
参数页。

### 6.3 独立的 stream mode

扩展 `Vehicle::PIDTuningTelemetryMode` 和 `MAVLinkStreamConfig`：

| 页面 | 请求消息 | 建议 USB 频率 | interval 参数 |
| --- | --- | --- | --- |
| Rate | `ROVER_RATE_TUNING_STATUS` | 50 Hz | 20000 us |
| Attitude | `ROVER_ATTITUDE_TUNING_STATUS` | 30 Hz | 33333 us |
| Velocity | `ROVER_VELOCITY_TUNING_STATUS` | 25 Hz | 40000 us |
| Position/Path | `ROVER_POSITION_TUNING_STATUS` | 10 Hz | 100000 us |

沿用 `MAVLinkStreamConfig` 现有 ACK/中断状态机。进入页面时只请求当前消息；切页先
恢复上一消息默认值，再请求下一消息；退出 PID Tuning、断链或销毁 Vehicle 时恢复
默认值。不要同时常开四个高频 stream。链路协商为 MAVLink1 时不得进入请求循环，应
直接报告实时 Rover tuning 需要 MAVLink2。

### 6.4 页面路由

第一阶段必需的新文件：

- `PX4TuningComponentRoverAll.qml`
- `PX4TuningComponentRoverRate.qml`
- `PX4TuningComponentRoverAttitude.qml`
- `PX4TuningComponentRoverVelocity.qml`
- `PX4TuningComponentRoverPosition.qml`

并加入 `src/AutoPilotPlugins/PX4/CMakeLists.txt`。

第一阶段路由规则：

- `MAV_TYPE_VTOL_FIXEDROTOR` 且参数 `HYBR_QUAD_ROV==1`：现有 VTOL 页面同时显示
  Multirotor 和 Rover 两个顶层标签，不能覆盖或删除 Multirotor 调参。
- 其他 VTOL：行为保持不变。

Rover 顶层入口由 mini 能力决定，不随当前 Quad/Rover 形态隐藏。mini 处于 Quad 时
仍保留该入口并显示 inactive/无实时数据；形态切换不得销毁页面或丢失参数编辑入口。
基线分支已有的 joystick/AUX 行为也必须保持不变。

普通 `MAV_TYPE_GROUND_ROVER` 路由不是 mini 第一阶段的验收项。只有在相应 PX4
target 也交付相同四个 producer 后，才新增 `PX4TuningComponentRover.qml` 并在
`PX4TuningComponent.cc` 路由；不能给不支持协议的 stock Rover 暴露永久空页面。

不要只用 `SYS_AUTOSTART=22002` 作为长期能力判断；优先使用明确的
`HYBR_QUAD_ROV` 参数。组件模型必须在参数下载完成后刷新，不能在 Fact 尚不可用时
永久判定为非 hybrid。

### 6.5 复用图表而不伪造采样率

复用 `PIDTuning.qml` 的曲线、暂停、缩放和 `FactSlider`。给它增加向后兼容的可选
source-timestamp 采样入口：Rover 页面在 FactGroup 的新时间戳到达时追加一次样本，
横轴使用 PX4 源时间；原 Multirotor 页面继续使用现有 Timer 路径。

不要让 10 ms QML Timer 把一个 25/50 Hz MAVLink 样本重复追加成 100 Hz 数据。
参数滑块继续通过 `FactPanelController::getParameterFact()` 和现有 PARAM_SET 回读
工作，不在 Rover FactGroup 自建参数协议。

## 7. 页面与参数分组

### Rate

- 曲线：Yaw Rate Response / Setpoint；可选附加 Integral、Steering Output。
- 主调参数：`RO_YAW_RATE_P`、`RO_YAW_RATE_I`、`RD_MAX_THR_YAW_R`。
- 限制参数：`RO_YAW_RATE_LIM`、`RO_YAW_ACCEL_LIM`、`RO_YAW_DECEL_LIM`。
- 高级噪声参数：`RO_YAW_RATE_TH`，明确标注仅作用于 measurement。

### Attitude

- 曲线：Yaw Response / Setpoint；可选附加 yaw-rate setpoint。
- 主调参数：`RO_YAW_P`。
- 相关限制：`RO_YAW_RATE_LIM`、`RO_YAW_ACCEL_LIM`、`RO_YAW_DECEL_LIM`。

### Velocity

- 曲线：Body-X Speed Response / Setpoint；可选附加 Integral、Throttle Output。
- 主调参数：`RO_SPEED_P`、`RO_SPEED_I`、`RO_MAX_THR_SPEED`。
- 限制参数：`RO_SPEED_LIM`、`RO_ACCEL_LIM`、`RO_DECEL_LIM`、`RO_JERK_LIM`。
- 高级噪声参数：`RO_SPEED_TH`，明确标注仅作用于 measurement。

### Position / Path

- 曲线：North Response / Target、East Response / Target、Cross-track Error。
- 主调参数：`PP_LOOKAHD_GAIN`、`PP_LOOKAHD_MIN`、`PP_LOOKAHD_MAX`。
- 路径状态机参数：`RD_TRANS_TRN_DRV`、`RD_TRANS_DRV_TRN`、
  `RD_MISS_SPD_GAIN`。

滑块的合法 min/max/increment 取 PX4 参数 metadata。为提高可调性可以提供较窄的
默认显示窗口，但必须保留精确文本输入，不能硬编码一个比 metadata 更窄的可写范围。
修改后应显示飞控回读值；超时或拒绝时恢复并提示，不能只更新本地 UI。

## 8. USB 与数传边界

主要调试连接为 USB，以上频率适用。按完整未签名 MAVLink2 帧计算，单页上限分别为：
Rate 39 B x 50 Hz = 1,950 B/s；Attitude 35 B x 30 Hz = 1,050 B/s；Velocity
55 B x 25 Hz = 1,375 B/s；Position 56 B x 10 Hz = 560 B/s。四页一次只启用一条
stream，USB 有足够余量；仍需用 QGC Link Status 或抓包核对实际丢包和调度抖动。

典型低速数传不支持严谨的 Rate 实时调参：

- 57,600 baud 的 PX4 默认预算约 2,880 B/s，叠加常规遥测后明确不足。
- 115,200 baud 默认预算约 5,760 B/s，叠加常规流后余量有限，丢包和抖动会破坏
  内环曲线的时间一致性。
- 如只做低速观察，可把 Attitude/Velocity/Position 降到 10 至 20 Hz；这不能作为
  Rate 参数验收依据。

无 SD 卡时，PX4 可通过 MAVLink2 把 ULog 流到地面端，官方建议链路约
`>=50 KB/s`，USB 适用。默认 Rover logging topic 只有 10 Hz，因此它能作为现有
Python 离线分析的应急恢复方案，但不能替代本指南的 Rate 实时页面。低速数传不适合
完整 ULog streaming。

## 9. 测试与验收

### 自动测试

1. MAVLink fork 已有永久测试 `tests/test_mini_rover_dialect.py`：固定四个
   ID/CRC/payload，生成 `qgc_mini_rover`，并逐消息 pack/decode。
2. 当前最终源码已通过 146 项现有测试和目标固件构建；尚缺控制器级注入测试。后续修改
   producer 时应补 active/sanity/inactive/stale、同周期 output、Position 各路径/epoch/EKF reset。
3. QGC FactGroup 必须新增：四消息解码、drive type、rad/deg、yaw wrap、NaN、500 ms timeout、重启时间戳
   回退、`xy_reset_counter` 和多 Vehicle 隔离测试。
4. 扩展 `test/MAVLink/MAVLinkStreamConfigTest.cc`：四种请求、ACK、切页中断、退出
   恢复默认和断链恢复。
5. 普通 Quad/VTOL 回归：原 Multirotor PID 页面、标准高频 stream 与参数编辑不变。

### USB 实机验收

1. `HYBR_QUAD_ROV=1` 的 mini 同时显示 Multirotor、Rover 顶层入口；普通 VTOL 不
   出现 Rover。普通 Ground Rover 保持现状，除非同一任务明确加入其 PX4 producer。
2. 四页打开时只启用对应自定义消息，关闭/切页后上一消息恢复默认速率。
3. 曲线方向、单位和时间对齐与 NSH `listener rover_*_status` 同步核对。
4. Rate 50 Hz 不重复采样；断链、模式切换、controller inactive 后不延续旧曲线。
5. yaw 跨越正负 180 度无整屏跳变；local position reset 会清空 Position 图。
6. 每个参数修改均产生标准 PARAM_SET、收到飞控回读，重连后值仍正确。
7. 记录 USB 实测带宽、消息丢包率、QGC CPU 占用和 `hkust_nxt-dual_mini` Flash
   增量，分别报告构建通过与实车通过。
8. 最终报告 QGC 可执行构建产物的路径、大小和 SHA-256；没有 USB 飞控时明确列出
   未执行的实机步骤，不得把自动测试通过写成 USB 验收通过。

## 10. 工作量评估与交接顺序

- MAVLink 契约/fork 与 PX4 producer：约 2 至 3 个工程日。
- QGC FactGroup、stream lifecycle、四页和参数编辑：约 3 至 5 个工程日。
- 自动测试、USB 联调与实车验收：约 1 至 2 个工程日。
- 合计：约 6 至 10 个工程日。

交接顺序必须是：先固定消息 XML、ID、单位和 commit，再实现 PX4 producer，再实现
QGC decoder/UI，最后 USB 联调。QGC 可以先搭页面结构，但在真实 producer 就绪前
不得以模拟数据宣称实时调参完成。

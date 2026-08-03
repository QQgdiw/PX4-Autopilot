# Mini Quad/Rover MAVLink 通信接口

## 1. 文档范围与结论

本文面向与 `change_mini_v1.16.1` 固件联调的机载电脑程序：

- 固件源码锚点：`2f5d1f003b3106060e70df012de59bfc3404837c`
- 构建目标：`make hkust_nxt-dual_mini`
- airframe：`SYS_AUTOSTART=22002`
- 机型：无变形机构的 Quad/Rover 双模载具

PX4 与机载电脑之间没有一个包办所有工作的“车辆控制消息”。应按数据用途选择接口：

| 需求 | MAVLink 入口 | 控制权 |
| --- | --- | --- |
| VIO/SLAM 提供位置和速度 | `ODOMETRY` | 不改变；数据进入 EKF2 |
| 物理 RC 在 Position 模式操控 | RC + EKF2 融合结果 | 物理 RC/PX4 |
| 机载电脑 local NED 目标控制 | `SET_POSITION_TARGET_LOCAL_NED` + Offboard | 机载电脑 |
| 航点任务 | MAVLink Mission Protocol + Auto Mission | PX4 Navigator |
| 主动返航 | `MAV_CMD_NAV_RETURN_TO_LAUNCH` | PX4 Navigator |
| 模拟遥控器 | `MANUAL_CONTROL` | MAVLink 虚拟摇杆 |

`MANUAL_CONTROL` 只是可选的虚拟摇杆，不是外部定位或 Offboard 的替代品。物理 RC 需要
继续控制车辆时，不应由机载电脑持续发送 `MANUAL_CONTROL`。

本项目对 Rover 机载 Offboard 的正式文档范围仅为 local NED 二维位置 Go-to。本分支
继承的控制器内部还有 velocity/attitude/rate 分支，但没有把“前向速度 + yaw-rate”定义
成 mini 的稳定外部契约；本文不指导或承诺该接口，也不移植其他 worktree 的私有消息。

## 2. 定位数据与控制权

外部定位和控制 setpoint 是两条独立链路：

```text
VIO/SLAM --ODOMETRY--> EKF2 --> vehicle local position
                                  |
              +-------------------+--------------------+
              |                   |                    |
       RC + Position        Offboard setpoint     Mission / RTL
       物理 RC 控制          机载电脑控制           PX4 控制
```

### 2.1 VIO/SLAM local NED 与 GPS/global 的区别

| 项目 | VIO/SLAM local NED | GPS/global position |
| --- | --- | --- |
| 坐标 | 相对本地原点，单位 m | WGS84 纬度/经度/高度 |
| 优点 | 室内可用、更新快、近距离连续 | 有绝对地理参考，可建立全局 Home |
| 主要误差 | 长时间漂移、重定位跳变 | 遮挡、多径、更新率和局部精度受环境影响 |
| Position/本地 Offboard | 可以 | 可以，经 EKF2 生成 local NED |
| 全局 Mission/标准 RTL | 单独使用时不保证可用 | 有效 global position 和 Home 时可用 |

本项目机载视觉定位应优先使用 `ODOMETRY` 传 local NED 位置、速度、姿态、协方差和
reset 信息。`VISION_POSITION_ESTIMATE` 是兼容性入口，但没有完整速度、frame、quality
等字段，不作为新程序首选。

标准 RTL 在当前 Commander 中要求有效 local position、global position、local altitude
和 Home。未地理配准的 VIO-only 系统即使能进入 Position，也不能据此假定标准 RTL
可用。若产品要求 RTL，必须另行提供并验证可靠的 global position 与 Home。

当前分支没有普通 `GPS_INPUT` 接收路径；`HIL_GPS` 仅在 `MAV_USEHILGPS=1` 时作为特殊
仿真式入口接收。产品外部 GNSS 优先直接接飞控 GPS 端口，不能把设置 global origin
等同于持续提供有效 GPS 测量。

## 3. MAVLink 版本、身份与链路

### 3.1 固定 dialect

| 项目 | 固定值 |
| --- | --- |
| MAVLink fork | `https://github.com/QQgdiw/mavlink.git` |
| commit | `07c6964a8fcc364c49d394f0bf0275b9fc05857d` |
| PX4/机载端 dialect | `mini_rover` |
| QGC composite dialect | `qgc_mini_rover` |

部署建议设置 `MAV_PROTO_VER=2`。车辆的标准消息来自 `common.xml`；MAVLink2 还用于
message ID 大于 255 的 gimbal v2 和四个私有 Rover tuning 消息。

### 3.2 身份和心跳

默认飞控为 `MAV_SYS_ID=1`、`MAV_COMP_ID=1`。机载电脑建议使用相同 system ID、独立
component ID `MAV_COMP_ID_ONBOARD_COMPUTER=191`，并以 1 Hz 发送：

- `HEARTBEAT.type=MAV_TYPE_ONBOARD_CONTROLLER`
- `HEARTBEAT.autopilot=MAV_AUTOPILOT_INVALID`

命令发送到精确的 `target_system=MAV_SYS_ID`、`target_component=MAV_COMP_ID`。
`COMMAND_ACK` 只表示命令处理结果；模式、armed、形态还要通过后续状态消息确认。

### 3.3 物理链路

USB 默认使用 Onboard MAVLink，Linux 通常为 `/dev/ttyACM*`。设备号可能变化，产品程序
应使用稳定 udev symlink。QGC 和机载程序需要共享 USB 时，应由 `mavlink-routerd` 等
单一进程持有串口，再转发给不同 endpoint，不能同时直接打开设备。

TELEM2 可作为机载串口：

```text
MAV_1_CONFIG    = 102
SER_TEL2_BAUD   = 921600
MAV_1_MODE      = 2       # Onboard
MAV_1_RATE      = 0
MAV_1_FORWARD   = 0
MAV_1_RADIO_CTL = 0
```

TELEM2 同一时间只能分配给 MAVLink 或 uXRCE-DDS。使用 MAVLink 时保持
`UXRCE_DDS_CFG!=102`。连接后用 `mavlink status` 确认设备、MAVLink2、模式和丢包率。

## 4. 机载 VIO/SLAM 输入

### 4.1 `ODOMETRY` 字段约定

建议以 30--50 Hz 发送，最低频率和最大延迟应由实测确定：

| 字段 | local NED VIO 要求 |
| --- | --- |
| `time_usec` | VIO 采样时间；使用 MAVLink `TIMESYNC` 对齐，不能用未来时间 |
| `frame_id` | `MAV_FRAME_LOCAL_NED` |
| `child_frame_id` | 速度为 NED 时同样用 `MAV_FRAME_LOCAL_NED` |
| `x/y/z` | North/East/Down，m |
| `q` | body FRD 到 local NED 的 Hamilton 四元数 `[w,x,y,z]` |
| `vx/vy/vz` | NED 速度，m/s |
| `rollspeed/pitchspeed/yawspeed` | body FRD 角速度，rad/s；未知时填 NaN |
| 两组 covariance | 填真实方差；未知时首元素为 NaN，不能伪造很小噪声 |
| `reset_counter` | VIO 地图重置或重定位跳变时递增 |
| `estimator_type` | `MAV_ESTIMATOR_TYPE_VIO` 或 `VISION` |
| `quality` | `-1` 失败，`0` 未知/未设置，`1..100` 表示质量 |

只有真正与 North/East 对齐的 VIO world frame 才使用 `MAV_FRAME_LOCAL_NED`。任意初始
航向的 SLAM world frame 应使用 `MAV_FRAME_LOCAL_FRD`，并与 EKF yaw 配置一致。如果
原始 ROS 数据是 ENU/FLU，必须在发送前完整转换位置、速度、姿态、角速度和 covariance；
不能只交换 x/y 或只对 z 取反。

PX4 在 `TIMESYNC` 尚未收敛时会暂用接收时刻作为 `timestamp_sample`，这能避免错误时钟
立即污染 EKF，但会隐藏链路延迟。正式联调仍应检查时间同步和端到端延迟。

### 4.2 EKF2 配置

`EKF2_EV_CTRL` 按实际可用量选择，不要机械复制一个固定值：

| bit | 值 | 融合内容 |
| ---: | ---: | --- |
| 0 | 1 | 水平位置 |
| 1 | 2 | 垂直位置 |
| 2 | 4 | 三维速度 |
| 3 | 8 | yaw |

例如水平位置 + 速度为 `5`，再融合视觉 yaw 为 `13`。只有在 VIO 高度和坐标对齐已验证
时才增加垂直位置 bit。`EKF2_EV_DELAY` 应按测得延迟设置；默认
`EKF2_EV_NOISE_MD=0` 会使用消息 covariance 作为观测噪声并受参数下限约束。
`EKF2_HGT_REF` 是否使用 barometer、GPS、range 或 vision 是整机估计器决策，不能仅因
发送了 VIO 就盲目改成 vision。

如果 VIO 能提供可靠 quality，建议设置 `EKF2_EV_QMIN>=1` 并按实测选择阈值。默认
`EKF2_EV_QMIN=0` 会绕过 quality 门控；此时即使消息标记 `quality=-1`，有限的位置/速度
仍可能参与融合。VIO 故障时发送端必须停止发布有限测量，不能只改 quality 后继续发送。

在允许解锁或切换 Position/Offboard 前，至少确认：

- `ESTIMATOR_STATUS` 没有对应的水平位置/速度故障；
- `LOCAL_POSITION_NED` 与 VIO 运动方向、尺度和静止噪声一致；
- PX4 回传 `ODOMETRY` 连续且没有非预期 reset；
- 停止 VIO、制造延迟和触发重定位时，EKF/failsafe 行为符合预期。

## 5. 物理 RC + Position 模式

这是“机载电脑提供定位，飞手用遥控器操控”的标准流程：

1. 设置 `COM_RC_IN_MODE=0`，只选择物理 RC 作为手动输入。
2. 机载电脑持续发送 `ODOMETRY`，但不发送 `MANUAL_CONTROL` 和 Offboard setpoint。
3. 确认 EKF2 local position/velocity 有效后，进入 Position 模式。
4. RC stick 进入 PX4 原生 Quad 或 Differential Rover Position 控制器。
5. 机载电脑只监视状态；VIO 停止时由 PX4 的定位丢失和 RC failsafe 处理。

进入 Position 可发送 `MAV_CMD_DO_SET_MODE`（176）：`param1=1`、`param2=3`、
`param3=0`。ACK 后必须用新的 `HEARTBEAT.custom_mode` 确认实际模式。

VIO 数据不会“抢摇杆”。只有持续发送 `MANUAL_CONTROL` 并把 `COM_RC_IN_MODE` 配成允许
joystick 时，才会参与手动输入源选择。

## 6. Offboard local NED 控制

Offboard 表示机载电脑取得 setpoint 所有权。物理 RC 仍建议保留模式开关、上锁/急停等
安全能力，但 stick 是否自动退出 Offboard 受机型和参数限制，不能把它当成通用接管机制。

### 6.1 通用进入流程

1. 保持 `MAV_FWDEXTSP=1`。
2. 以 10--20 Hz 连续发送合法 setpoint；先发送至少 1 s，再请求 Offboard。
3. 用 `MAV_CMD_DO_SET_MODE`：`param1=1`、`param2=6`、`param3=0`。
4. ACK 后确认 `HEARTBEAT.custom_mode` 已为 Offboard，并继续发送 setpoint。
5. 退出时先请求一个可运行的 PX4 模式并确认，再停止 setpoint 流。

接收频率必须高于 2 Hz；建议值留有链路抖动余量。当前默认
`COM_OF_LOSS_T=1 s`，超时动作由 `COM_OBL_RC_ACT` 决定，默认进入 Position。必须分别在
Quad/Rover、有/无有效 RC 和定位的条件下实测，不能只验证参数值。

当前 MAVLink receiver 在尚未进入 Offboard 时只更新 `OffboardControlMode`，不会发布
`TrajectorySetpoint`；所以预发送用于满足模式可用性，真正目标必须在模式切换后继续发出
新帧。不能进入 Offboard 后立即停止发送并期待 PX4 锁存预发送目标。

### 6.2 Quad local position

Quad 使用 `SET_POSITION_TARGET_LOCAL_NED`，`coordinate_frame=MAV_FRAME_LOCAL_NED`：

- `x/y/z` 为目标 North/East/Down；
- 不控制的 velocity、acceleration、yaw/yaw-rate 必须在 `type_mask` 中 ignore；
- 需要控制 yaw 时，只取消 yaw ignore 并填弧度值；
- 每一帧都发送完整、有限且相互一致的目标。

### 6.3 Rover 二维位置 Go-to

Rover 同样发送 `SET_POSITION_TARGET_LOCAL_NED`，但正式契约只有：

- `x`、`y`：有限的 local NED 目标位置；
- `z`、velocity、acceleration、yaw、yaw-rate：全部 ignore；
- PX4 使用 `RO_SPEED_LIM` 作为该 Offboard 目标的巡航速度上限；
- 原生 Differential Rover 将二维位置转成 path、速度、航向和轮输出。

不要把 `yaw_rate` 填进这条消息后假定 Rover 会执行“速度 + yaw-rate 直接驾驶”。MAVLink
接收器由 position/velocity/acceleration 字段生成 Offboard 控制位，不会因单独的
`yaw_rate` 建立一个 mini Rover body-rate 契约。

单向 H 桥仍会在最终 mini 输出层把负轮指令钳为 0，所以紧转弯时可能是一轮停、另一轮
前转；这不改变上层 Go-to 接口。

### 6.4 pymavlink Rover Go-to 示例

以下函数必须由固定频率线程持续调用，不能只发一次：

```python
import math
import time
from pymavlink import mavutil


def send_rover_local_position(link, target_system, target_component, north_m, east_m):
    m = mavutil.mavlink
    type_mask = (
        m.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | m.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | m.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | m.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | m.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | m.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | m.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    link.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        target_system,
        target_component,
        m.MAV_FRAME_LOCAL_NED,
        type_mask,
        float(north_m), float(east_m), 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0,
    )
```

发送线程应检查目标有限、连接状态和模式；程序异常时不能继续重发最后一个旧目标。

## 7. Mission、Go-to 与 RTL

三者不要混用：

| 功能 | 接口 | 是否需持续 setpoint | 定位要求 |
| --- | --- | --- | --- |
| 临时 local Go-to | Offboard + `SET_POSITION_TARGET_LOCAL_NED` | 是 | 有效 local position |
| 多航点任务 | MAVLink Mission Protocol，进入 Auto Mission | 否 | 通常需要 global position |
| 标准 RTL | `MAV_CMD_NAV_RETURN_TO_LAUNCH` | 否 | global/local position、altitude、Home |

任务上传使用标准 `MISSION_COUNT`、`MISSION_REQUEST_INT`、`MISSION_ITEM_INT`、
`MISSION_ACK` 握手。`MISSION_ACK=ACCEPTED` 只表示传输、解析和存储完成，不证明
Navigator feasibility 或实车可执行。进入 Auto Mission 使用
`MAV_CMD_DO_SET_MODE`，`param1=1`、`param2=4`、`param3=4`，再由
`HEARTBEAT.custom_mode` 和 `MISSION_CURRENT` 确认。

本机没有 SD 卡时，默认 dataman 文件后端无法可靠保存 Mission。设置
`SYS_DM_BACKEND=1`、保存并重启，再用 `dataman status` 确认 RAM backend；Mission、
Fence 和 Rally 数据会在重启时丢失，机载程序每次启动后都要重新上传并确认。

Quad 主动 RTL 可发送 `MAV_CMD_NAV_RETURN_TO_LAUNCH`（20），等待 ACK 并确认模式变为
Auto RTL。也可设置 Auto 主模式 `param2=4`、RTL 子模式 `param3=5`，但直接 RTL command
更清楚。

当前 mini Rover 的 Direct RTL 存在已确认的行为缺口：Rover 通常持续
`landed=true`，RTL 激活后会直接进入 IDLE，不生成返航轨迹；Commander 仍可能返回
Accepted 且显示 AUTO_RTL。Mission 中的 RTL item 同样受影响。修复代码并完成实车测试
前，Rover RTL 不得作为人工命令或 failsafe 安全功能。Rover 返回应暂用经过验证的
Mission waypoint 或 Offboard local Go-to，并分别承担其定位与失联边界。

Mission/RTL 的轨迹由 PX4 Navigator 和目标形态的原生控制器执行。任务项必须适合当前
Quad 或 Rover 形态；Mission/RTL 执行期间禁止切换 Quad/Rover，模式命令也不会替程序
自动执行形态切换。VIO-only 时如需
“回到本地起点”，应由机载程序在 Offboard 中发送 local Go-to；这不是 PX4 标准 RTL，
不能复用 RTL 的 global/Home 安全承诺。

任务进度使用 `MISSION_CURRENT.seq` 和 `MISSION_ITEM_REACHED`；当前不要依赖
`MISSION_CURRENT.mission_state` 判断完成。Rover 到达最后一项后可能仍保持 AUTO_MISSION
且继续 armed，业务状态机必须显式退出 Auto、停车并按安全条件 disarm。

自动失联返航属于 failsafe，而不是同一个命令：

- RC 丢失：`COM_RC_LOSS_T`、`NAV_RCL_ACT`
- Offboard 丢失：`COM_OF_LOSS_T`、`COM_OBL_RC_ACT`
- datalink 丢失：`COM_DL_LOSS_T`、`NAV_DLL_ACT`

这些动作只有在对应模式要求和定位条件满足时才可能进入 RTL。当前 Rover 必须避免把
任何失联动作配置为未经修复验证的 RTL，并对最终替代动作做整机故障注入测试。

## 8. 当前位置、速度与目标状态

Onboard MAVLink 默认已发送主要状态；带宽不足时仍可能自动降频。业务端应按需要用
`MAV_CMD_SET_MESSAGE_INTERVAL` 请求，并对每条消息做 timestamp/freshness 检查。

| 信息 | MAVLink 消息 | 适用说明 |
| --- | --- | --- |
| armed、模式 | `HEARTBEAT` | command 后的最终模式判据 |
| Quad/Rover 形态 | `EXTENDED_SYS_STATE.vtol_state` | MC=Quad，FW=Rover |
| 当前 local 位置/速度 | `LOCAL_POSITION_NED` | EKF2 NED 输出 |
| 当前融合位姿/速度 | `ODOMETRY` | 注意与机载发给 PX4 的同名输入区分方向 |
| 当前 global 位置 | `GLOBAL_POSITION_INT` | global 无效时不能用于 RTL |
| 当前姿态 | `ATTITUDE` / `ATTITUDE_QUATERNION` | NED/FRD 约定 |
| Home | `HOME_POSITION` | RTL 前必须有效 |
| EKF 状态 | `ESTIMATOR_STATUS` | 检查估计器有效性和故障 |
| MC local 控制目标 | `POSITION_TARGET_LOCAL_NED` | 来自 MC local setpoint，不是 Rover 权威目标 |
| Mission/RTL global 目标 | `POSITION_TARGET_GLOBAL_INT` | 来自 Navigator position triplet |
| 当前 mission item | `MISSION_CURRENT` | 任务进度 |
| 命令结果 | `COMMAND_ACK` | 仍需状态确认 |

标准 `POSITION_TARGET_LOCAL_NED` stream 订阅 `vehicle_local_position_setpoint`；当前
Differential Rover 不发布该 topic，所以不能用它验证 Rover Offboard 目标。Rover
发送端应保留自己最后一次有效目标，并可按需请求私有
`ROVER_POSITION_TUNING_STATUS`、`ROVER_VELOCITY_TUNING_STATUS` 监视控制器实际目标与响应。

## 9. 通用命令与双模状态

### 9.1 Arm/disarm

使用 `MAV_CMD_COMPONENT_ARM_DISARM`（400）：`param1=1` arm、`param1=0` disarm，
正常操作 `param2=0`。禁止在产品逻辑中使用强制 disarm magic value `21196`。ACK 后还要
确认 `HEARTBEAT.base_mode` 的 `MAV_MODE_FLAG_SAFETY_ARMED` 位。

### 9.2 Quad/Rover 切换

复用 `MAV_CMD_DO_VTOL_TRANSITION`（3000）：

| 目标形态 | param1 | ACK 后状态 |
| --- | ---: | --- |
| Quad | `MAV_VTOL_STATE_MC` (3) | `EXTENDED_SYS_STATE.vtol_state=MC` |
| Rover | `MAV_VTOL_STATE_FW` (4) | `EXTENDED_SYS_STATE.vtol_state=FW` |

Quad -> Rover 只有在 1.5 s 内的新鲜 `landed=true` 时接受；Rover -> Quad 当前直接接受。
形态切换不会改变 armed 状态。切换前应停车/最低推力，ACK 后等待新的形态状态，再进入
适合目标形态的 Position、Offboard、Mission 或其他模式。

## 10. `MANUAL_CONTROL` 可选虚拟摇杆

只有明确需要机载电脑模拟遥控器时才使用该消息：

| 字段 | PX4 内部字段 | Rover Manual 语义 |
| --- | --- | --- |
| `x/1000` | pitch | 前进/后退请求 |
| `y/1000` | roll | 差速转向请求 |
| `z/500-1` | throttle | Rover 车轮不用；仍须填合法 0--1000 |
| `r/1000` | yaw | Rover 车轮不用；仍须填合法 -1000--1000 |

建议 20--50 Hz。它会进入 `ManualControlSelector`：

- `COM_RC_IN_MODE=0`：只接受物理 RC；
- `COM_RC_IN_MODE=1`：只接受 MAVLink joystick；
- `COM_RC_IN_MODE=2`：RC/MAVLink 整套输入失效后切换；
- 不支持车辆轴来自 MAVLink、AUX1 同时来自物理 RC 的按轴混合。

因此 RC Position 场景保持模式 0 且不发送该消息。Offboard 场景也不靠它提供 setpoint。

## 11. MAIN5 云台与 Rover 调参消息

MAIN5 为 50 Hz 普通 PWM 舵机，1000--2000 us，默认/disarmed/failsafe 为 1500 us，
机械总行程 90 度。机载端使用 gimbal v2：

1. `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`（1001）取得 primary control；
2. `GIMBAL_MANAGER_SET_PITCHYAW`（287）以 10--20 Hz 连续发送；
3. pitch 限幅约 `[-pi/4,+pi/4]`，yaw=0，未使用 rate 填 NaN；
4. 单次低频命令可用 `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`（1000）。

物理 RC channel 10 -> AUX1 只有在物理 RC 被 selector 选中时才有效。

四条私有 Rover 消息只用于实时调参，不能控制车辆：

| 消息 | ID | 建议上限 |
| --- | ---: | ---: |
| `ROVER_RATE_TUNING_STATUS` | 60100 | 50 Hz |
| `ROVER_ATTITUDE_TUNING_STATUS` | 60101 | 30 Hz |
| `ROVER_VELOCITY_TUNING_STATUS` | 60102 | 25 Hz |
| `ROVER_POSITION_TUNING_STATUS` | 60103 | 10 Hz |

它们默认不发送，由 `MAV_CMD_SET_MESSAGE_INTERVAL` 按需启用。字段定义见
`docs/mini_rover_realtime_tuning_guide.md`。

## 12. 联调顺序

1. 拆桨、架空车轮，确认 MAVLink2、身份、心跳和命令 ACK/状态确认。
2. 只接 VIO，核对 NED/FRD、尺度、延迟、covariance、reset 和 EKF2 融合状态。
3. `COM_RC_IN_MODE=0`，验证物理 RC 在 Quad/Rover Position 下控制且机载 VIO 不抢输入。
4. 分别验证 Quad 三维 local Offboard 和 Rover 二维 local Go-to，注入 setpoint 断流。
5. 无 SD 时确认 RAM dataman，再验证 Mission；有可靠 global/Home 后只先验收 Quad RTL。
   当前 Rover RTL 记录为已知不可用，不作为安全功能。
6. 验证 landed 时 Quad -> Rover、两个方向的形态状态和模式重新确认。
7. 最后验证云台、可选虚拟摇杆和按需 Rover tuning；它们不能替代前述控制链路验收。

文档描述的是当前源码接口。USB、VIO、ROS/MAVLink 程序、实车运动和 failsafe 仍需分别
验收，不能把静态接口核对等同于整机测试通过。

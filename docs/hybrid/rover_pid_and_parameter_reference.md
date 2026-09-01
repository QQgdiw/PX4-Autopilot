# Quad-Rover 混合载具 Rover 控制环与参数参考

本文档以 `debug/testc1-v1.16.1` 当前源码为准，适用于
`zeroone_x6_hybrid` 差速 Rover。内容按控制链由简单到复杂排列。

## 1. 总体控制结构

Rover 控制不是一个单独的“大 PID”，而是纵向速度控制和横向航向控制并联，
随后经过差速混合；每个 M2006 又各自带一个转子速度闭环。

```text
纵向支路：
位置/任务规划（可选） -> 车体前向速度设定 -> 速度 PI + 速度前馈
                                         -> 归一化油门 ┐

横向支路：
路径/航向设定（可选） -> 航向 P -> 偏航角速度 PI + 差速前馈
                                         -> 归一化差速 ├-> 差速混合
                                                          -> 左右轮归一化命令
                                                          -> 每轮 M2006 转速 PID+FF
                                                          -> C610 电流命令
```

纵向速度环和横向航向环是并联关系；航向 P 与偏航角速度 PI 是串级关系；
M2006 转子速度 PID 是两个车轮共同拥有的最内层执行器闭环。

差速混合公式为：

```text
left  = throttle_body_x - normalized_speed_diff
right = throttle_body_x + normalized_speed_diff
```

当油门与差速叠加超过 `[-1, 1]` 时，混合器优先保留转向差速并削减纵向油门。
`hybrid_vehicle_control` 再将 Rover 左右轮输出路由到最终 Motor 6/5；M2006
适配器读取 Motor 6 作为左轮、Motor 5 作为右轮。

## 2. 所有模式共有的最内层：M2006 转子速度闭环

该闭环位于 `m2006_can` 驱动中，左右电机各运行一套相同的控制器。

| 项目 | 内容 |
|---|---|
| 控制器 | PID + 目标转速前馈，带积分抗饱和 |
| 输入设定值 | 最终左右轮归一化命令 `[-1,1]` 乘以 `M2K_MAX_RPM` |
| 输入反馈 | C610 反馈的 M2006 转子转速，单位 rpm；这是减速箱前转子转速 |
| 控制误差 | `target_rpm - measured_rpm` |
| 输出 | C610 有符号电流命令，受 `M2K_CUR_LIM` 限制 |
| P 参数 | `M2K_SPD_P` |
| I 参数 | `M2K_SPD_I` |
| D 参数 | `M2K_SPD_D`；当前实现对测量值求负微分，避免设定值突变造成微分冲击 |
| 前馈参数 | `M2K_SPD_FF`，直接乘目标转速 |
| 设定值斜率 | `M2K_RPM_SLEW`，单位 rpm/s |
| 输出限幅 | `M2K_CUR_LIM` |
| 日志主题 | `m2006_motor_status` |

控制器形式可概括为：

```text
current = P * speed_error
        + I * integral(speed_error)
        - D * derivative(measured_rpm)
        + FF * target_rpm
```

该闭环始终是 Rover 推进链的最后一级。因此即使上层处于 Manual 模式、没有
车体速度闭环，电机侧仍然是闭环转速控制。

## 3. Manual：开环车体控制 + M2006 内环

Manual 是最简单的 Rover 模式，上层不使用车体速度、航向或偏航角速度 PID。

```text
pitch -> rover_throttle_setpoint -> 油门加减速斜率限制 -> 差速混合
roll  -> rover_steering_setpoint -----------------------> 差速混合
                                                        -> M2006 转速 PID
```

| 通道 | 输入 | 输出 | 相关参数 |
|---|---|---|---|
| 前后 | `manual_control_setpoint.pitch` | `rover_throttle_setpoint.throttle_body_x` | `RO_ACCEL_LIM`、`RO_DECEL_LIM`、`RO_MAX_THR_SPEED` 用于最终归一化油门斜率限制 |
| 转向 | 反号后的 `manual_control_setpoint.roll` | `rover_steering_setpoint.normalized_speed_diff` | 无 PID 参数 |
| 混合 | 归一化油门和差速 | `actuator_motors_rover.control[0/1]` | `CA_R_REV` 仅形成 reversible 标志；实际 M2006 方向由 `M2K_L_REV/M2K_R_REV` 处理 |

## 4. Acro：偏航角速度 PI

Acro 在 Manual 基础上增加偏航角速度闭环。纵向油门仍然是开环归一化命令，
横向转向则闭环跟踪偏航角速度。

```text
roll -> 目标偏航角速度 -> 设定值加/减速度限制 -> 偏航角速度 PI + 几何前馈
                                                   -> normalized_speed_diff
pitch --------------------------------------------> normalized throttle
```

| 项目 | 内容 |
|---|---|
| 输入设定值 | 遥控 roll 映射到 `[-RO_YAW_RATE_LIM, +RO_YAW_RATE_LIM]` |
| 输入反馈 | `vehicle_angular_velocity.xyz[2]`，即机体系 Z 轴角速度 |
| 控制误差 | 调整后的目标偏航角速度减实测偏航角速度 |
| 输出 | `rover_steering_setpoint.normalized_speed_diff`，范围 `[-1,1]` |
| P 参数 | `RO_YAW_RATE_P` |
| I 参数 | `RO_YAW_RATE_I` |
| D 参数 | 无，固定为 0 |
| 前馈 | 使用 `RD_WHEEL_TRACK` 和 `RD_MAX_THR_YAW_R` 将目标角速度换算成归一化轮速差 |
| 设定值限制 | `RO_YAW_RATE_LIM`、`RO_YAW_ACCEL_LIM`、`RO_YAW_DECEL_LIM` |
| 测量死区 | `RO_YAW_RATE_TH` |
| 日志主题 | `rover_rate_setpoint`、`rover_rate_status`、`rover_steering_setpoint` |

`RO_YAW_RATE_TH` 只用于过滤实测角速度噪声，不过滤目标角速度；低角速度外环
或 Offboard 指令仍是有效控制量。调参时应以静止 gyro Z 噪声确定该阈值，而不
能用它制造遥控或控制目标死区。

## 5. Stabilized：航向 P 串联偏航角速度 PI

Stabilized 根据转向摇杆状态选择两条横向路径：

- 转向摇杆不在中位，或者车辆纵向油门为零：与 Acro 相同，直接控制偏航角速度。
- 车辆正在前后运动且转向摇杆回中：锁存当时航向，启用航向保持外环。

航向保持时的串级结构为：

```text
锁存航向 -> 航向误差 -> 航向 P -> 偏航角速度设定
                              -> 偏航角速度 PI -> 轮速差
```

### 航向外环

| 项目 | 内容 |
|---|---|
| 输入设定值 | `rover_attitude_setpoint.yaw_setpoint` |
| 输入反馈 | `vehicle_attitude` 解算出的 yaw |
| 控制误差 | 包络到 `[-π,π]` 的航向误差 |
| 输出 | `rover_rate_setpoint.yaw_rate_setpoint` |
| P 参数 | `RO_YAW_P` |
| I/D 参数 | 无 |
| 输出限幅 | `RO_YAW_RATE_LIM` |
| 设定值转动速率 | 同样使用 `RO_YAW_RATE_LIM` 限制航向设定值变化速率 |
| 日志主题 | `rover_attitude_setpoint`、`rover_attitude_status`、`rover_rate_setpoint` |

航向外环输出进入上一节的偏航角速度 PI，因此应先整定角速度内环，再整定
`RO_YAW_P`。

## 6. Position：速度 PI 与航向串级环并联

手动 Position 模式比 Stabilized 多出车体前向速度闭环和航迹保持逻辑。

```text
pitch -> 车体速度设定 ---------------------------> 速度 PI + 前馈 -> 油门
roll  -> 航向增量；回中时由 Pure Pursuit 保持直线 -> 航向 P -> 角速度 PI -> 差速
```

### 车体前向速度环

| 项目 | 内容 |
|---|---|
| 输入设定值 | `differential_velocity_setpoint.speed`，受 `RO_SPEED_LIM` 限制 |
| 输入反馈 | `vehicle_local_position.vx/vy` 经姿态旋转后的机体系 X 速度 |
| 控制误差 | 调整后的车体 X 速度设定值减实测车体 X 速度 |
| 输出 | `rover_throttle_setpoint.throttle_body_x`，范围 `[-1,1]` |
| P 参数 | `RO_SPEED_P` |
| I 参数 | `RO_SPEED_I` |
| D 参数 | 无，固定为 0 |
| 前馈 | 用 `RO_MAX_THR_SPEED` 将 m/s 线性映射为归一化油门 |
| 设定值限制 | `RO_ACCEL_LIM`、`RO_DECEL_LIM`、`RO_SPEED_LIM` |
| 测量死区 | `RO_SPEED_TH` |
| 日志主题 | `differential_velocity_setpoint`、`rover_velocity_status`、`rover_throttle_setpoint` |

速度 PI 与“航向 P → 偏航角速度 PI”横向串级环并联。两条支路最终在差速混合
处汇合；若转向所需差速太大，速度控制给出的纵向命令会被削减。

### 手动 Position 的路径生成

- pitch 映射到 `[-RO_SPEED_LIM, +RO_SPEED_LIM]`。
- roll 经过 `RO_YAW_STICK_DZ` 后生成有限航向增量。
- roll 回中且车辆在运动时，使用 Pure Pursuit 沿锁存直线生成航向设定。
- 需要较大转向时，速度控制器会根据 `RD_TRANS_DRV_TRN`、
  `RD_TRANS_TRN_DRV` 在原地转向和正常行驶之间切换。

Pure Pursuit 是几何路径跟踪器，不是 PID；其输出是目标 bearing，随后仍由航向
P 和偏航角速度 PI 执行。

## 7. Mission/Auto：位置与路径规划叠加在 Position 控制链外层

Mission、Auto、RTL 等使用相同的内层：

```text
航点/任务 -> Pure Pursuit -> bearing -> 航向 P -> 偏航角速度 PI -> 差速
          -> 速度规划 ------> 速度 PI -------------------------> 油门
```

位置层本身没有位置误差 PID。它负责：

- 将全局航点转换为本地 NED 坐标；
- 根据 `PP_LOOKAHD_*` 计算目标 bearing；
- 根据剩余距离、减速度和 jerk 规划允许速度；
- 根据航点夹角和 `RD_MISS_SPD_GAIN` 降低过弯速度；
- 在 `NAV_ACC_RAD` 范围内判定到达/停车。

因此任务模式调参必须在速度 PI、偏航角速度 PI 和航向 P 已经稳定后进行。

## 8. Offboard 模式

独立 Quad-Rover 在 Rover 形态只接受项目专用的 body-speed + yaw-rate 接口：

```text
OffboardControlMode.rover_velocity = true（其余控制 bit 必须全为 false）
RoverVelocitySetpoint.speed_body_x -> 车体前向速度 PI -> throttle
RoverVelocitySetpoint.yaw_rate     -> 偏航角速度 PI -> steering
```

DDS 输入 topic 为 `/fmu/in/offboard_control_mode` 和
`/fmu/in/rover_velocity_setpoint`。`speed_body_x` 是带符号的车体 X 轴速度，
`yaw_rate` 是 PX4 FRD/NED 正方向角速度；若上游是 ROS REP-103 FLU
`geometry_msgs/Twist`，映射为 `speed_body_x=linear.x`、
`yaw_rate=-angular.z`。

控制器仅在新鲜、无故障、稳定 `DRIVING` 状态接受该输入。mode 与 setpoint 都
必须非零时间戳、不过期、字段有限；setpoint 时间戳还必须严格晚于最近一次
`transition_completed_timestamp`。legacy、多个控制 bit、错误形态、变形中、
Fault、陈旧状态和旧 epoch 输入都会安全归零。普通非混合 Rover 仍保留标准 PX4
legacy Offboard 路径，不受该混合机型专用门控影响。

## 9. 控制环汇总

| 控制环 | 串并联位置 | 设定值输入 | 反馈输入 | 输出 | 参数 | 使用模式 |
|---|---|---|---|---|---|---|
| M2006 左/右转子速度 PID+FF | 每轮最内层，两轮并联 | 归一化轮命令 × `M2K_MAX_RPM` | C610 rotor rpm | C610 current | `M2K_SPD_P/I/D/FF` | 所有 Rover 推进模式 |
| 偏航角速度 PI+FF | 横向内环 | `rover_rate_setpoint.yaw_rate_setpoint` | gyro Z yaw rate | normalized speed difference | `RO_YAW_RATE_P/I`、`RD_WHEEL_TRACK`、`RD_MAX_THR_YAW_R` | Acro、Stabilized、Position、Auto/Mission、专用 Rover Velocity Offboard |
| 航向 P | 串联在角速度环外 | yaw/bearing setpoint | vehicle yaw | yaw-rate setpoint | `RO_YAW_P` | Stabilized 航向保持、Position、Auto/Mission |
| 车体前向速度 PI+FF | 与横向环并联 | body-X speed setpoint | body-X measured speed | normalized throttle | `RO_SPEED_P/I`、`RO_MAX_THR_SPEED` | Position、Auto/Mission、专用 Rover Velocity Offboard |
| Pure Pursuit | 最外层几何控制，非 PID | 路径/航点、当前位置、速度 | 位置和航迹 | bearing setpoint | `PP_LOOKAHD_GAIN/MIN/MAX` | Position 直线保持、Auto/Mission |

## 10. Rover 专用及强相关参数表

表中的“元数据默认值”来自参数定义；机型启动脚本可能覆盖它。当前
`22001_quad_rover` 明确将 `RO_MAX_THR_SPEED` 设为 `2.47 m/s`、
`NAV_ACC_RAD` 设为 `2 m`，并把 M2006 最大转速和斜率设为 `18000`。
实机保存值仍应以 `param show` 为准。

下表的排列顺序就是建议调参顺序。开始后一阶段前，应确认前一阶段在有效样本
区间内已经稳定；尤其不能在 M2006 转速环尚未跟随、车轮仍处于静摩擦死区时
整定车体偏航或速度环。P、I、D 和前馈初调时应一次只改变一项，并用 ULog
同时核对设定值、实测值、误差和执行器输出。

| 参数 | 单位 | 元数据默认值 | 作用 | 调参方法或技巧 |
|---|---:|---:|---|---|
| `M2K_EN` | bool | 1 | M2006/C610 私有 CAN 驱动使能 | 首先确认启用且两电机持续在线；该项是功能开关，不用于改善动态性能。修改后按驱动要求重启验证。 |
| `M2K_L_ID` | CAN ID | 1 | 左轮 C610 ID；当前实现只接受 1 | 架空车轮，分别只给左、右轮小指令并核对反馈来源；ID 错误必须先修正，不能用反向参数补偿轮位映射错误。 |
| `M2K_R_ID` | CAN ID | 2 | 右轮 C610 ID；当前实现只接受 2 | 与左轮相同，确认右轮命令只改变右侧目标转速和反馈；当前硬件应保持左 1、右 2。 |
| `M2K_L_REV` | bool | 0 | 左轮归一化命令反相 | 架空低速前进测试：正前进命令应使左轮产生车辆前进方向的运动。只在该轮方向错误时切换。 |
| `M2K_R_REV` | bool | 0 | 右轮归一化命令反相 | 同样用低速前进测试确认右轮方向；前进正常但转向相反时，应先检查转向符号链路，不要随意改变单轮方向。 |
| `CA_R_REV` | bitmask | 0 | Rover 输出 reversible 标志；M2006 实际转向反相仍由独立参数控制 | 确认控制分配允许 Rover 通道输出反向命令。它不是实际轮向校准旋钮；M2006 左右轮方向仍以 `M2K_L_REV/R_REV` 为准。 |
| `M2K_MAX_RPM` | rotor rpm | 18000 | 归一化轮命令对应的 M2006 转子最大目标转速 | 根据安全最高车速、总减速比和轮径计算，再用架空/低载测速核对。过大会放大上层命令并更早触及电流限幅，过小会限制最高车速。 |
| `M2K_CUR_LIM` | C610 current unit | 10000 | 每轮速度控制器输出电流限幅 | 先按电机、电调、电源和机构允许值设保守上限。调环时检查 `current_command` 是否长期顶住限幅；若已饱和，应先降低目标/负载或确认硬件，不能仅继续增大 P/I。 |
| `M2K_RPM_SLEW` | rotor rpm/s | 18000 | 每轮目标转速变化斜率 | 初调使用能看清阶跃且不会冲击机构的保守斜率；内环稳定后再逐步提高。过小会掩盖内环响应，过大可能引起电流尖峰或打滑。 |
| `M2K_SPD_P` | - | 0 | M2006 转子速度 P | 先将 I/D/FF 置零，从小逐步增加，做正反向阶跃；以响应足够快且无持续振荡、尖叫和频繁电流饱和为准，出现高频抖动或超调明显时回退。 |
| `M2K_SPD_FF` | - | 0 | M2006 目标转速前馈 | P 环稳定后，在不同稳态转速下增加 FF，使维持目标所需的 P 误差减小；过大表现为同方向超调或轻载速度偏高。正反向差异大时先排查摩擦和机械不对称。 |
| `M2K_SPD_I` | - | 0 | M2006 转子速度 I | P/FF 确定后小步增加，用持续负载测试消除稳态转速误差。解除负载后若长时间超调、反向拖拽或积分贴限，说明 I 过大。 |
| `M2K_SPD_D` | - | 0 | M2006 转子速度 D（测量微分） | 通常最后调且可保持 0；仅在反馈噪声足够低、P 调整后仍有可重复超调时小量增加。电流高频抖动或噪声放大即应减小。 |
| `RD_WHEEL_TRACK` | m | 0 | 左右轮中心间距，用于角速度前馈换算 | 直接测量左右轮接地点中心距并填入，不把它当作自由增益。实测有效轮距受轮胎侧滑影响时，可在低速稳态原地转向数据上做小幅等效修正。 |
| `RD_MAX_THR_YAW_R` | m/s | 0 | 原地最大差速命令对应的单侧轮线速度标定值 | 在 M2006 环稳定后做低到中等差速测试，用轮速/车体角速度核对几何前馈。过小会给出过强前馈，过大则主要依赖 PI 才能转到目标。 |
| `RO_YAW_RATE_LIM` | deg/s | 0 | 角速度设定上限、遥控映射范围和航向环输出上限 | 先按车辆不会侧翻、严重打滑或冲击机构的最大实测角速度设定；调内环时使用低于该上限的多个阶跃，不要一开始就满量程。 |
| `RO_YAW_ACCEL_LIM` | deg/s² | -1 | 偏航角速度设定增大速率限制；-1 禁用 | 初调设为可重复且不打滑的保守值；若设定值本身爬升过慢，先提高该项再评价 P。过大会造成轮速/电流突变。 |
| `RO_YAW_DECEL_LIM` | deg/s² | -1 | 偏航角速度设定减小速率限制；-1 禁用 | 用回中和正反向切换检查停车/换向。过小会拖尾，过大可能急停打滑；通常根据减速能力独立于加速限制设置。 |
| `RO_YAW_RATE_TH` | deg/s | 3 | 偏航角速度测量死区，不过滤目标角速度 | 静止记录 gyro Z 噪声，将阈值设在噪声包络稍上方。不能为消噪而设得过大，否则低速实测反馈被持续归零，容易掩盖小角速度动态。 |
| `RO_YAW_RATE_P` | - | 0 | 偏航角速度 PI 的比例增益 | 先令 I 为 0，用 Acro 或隔离的角速度阶跃逐步增大 P；观察 `rover_rate_status`。响应迟缓可增加，持续摆振、轮命令交替或明显超调时减小。 |
| `RO_YAW_RATE_I` | - | 0 | 偏航角速度 PI 的积分增益 | P 稳定后，在地面摩擦一致的持续角速度指令下增加 I 以消除稳态误差。松杆后拖尾、低频摆动或积分长期贴限说明过大。 |
| `RO_YAW_STICK_DZ` | 归一化 | 0.1 | Stabilized/Position 的转向摇杆死区 | 查看遥控 roll 中位噪声，将死区设为能可靠回中的最小值。过大导致中位附近转向不连续，过小会使航向保持频繁退出。 |
| `RO_YAW_P` | - | 0 | 航向外环比例增益 | 仅在角速度内环稳定后调。使用小到中等航向阶跃，从小增加；航向收敛慢可增加，来回摆头或角速度长期触限则减小。 |
| `RD_TRANS_TRN_DRV` | rad | 0.0872665 | 航向误差低于该值时，从原地转向切回行驶 | 与 `RD_TRANS_DRV_TRN` 配成有间隔的回差，下阈值必须更小。车辆对准后迟迟不前进可适当增大；尚未对准就前进则减小。 |
| `RD_TRANS_DRV_TRN` | rad | 0.174533 | 航向误差高于该值时，从行驶切换为原地转向；也参与航点过弯减速判定 | 在 Position 下逐步增大路径航向误差测试。过小会频繁原地转向，过大则车辆带着较大朝向误差前冲；始终保持大于 `RD_TRANS_TRN_DRV`。 |
| `RO_MAX_THR_SPEED` | m/s | 0；机型覆盖为 2.47 | 最大归一化油门对应的实车速度，也是速度环前馈标定值 | 在平直地面、车体速度估计有效时，用开环稳定速度标定“满归一化油门对应速度”。过小会造成速度前馈偏大，过大会使前馈不足。不要用架空轮速代替带载车速。 |
| `RO_SPEED_LIM` | m/s | -1 | Position/Auto 等模式的速度设定上限；必须配置为正值才能通过检查 | 先设为低于已验证安全速度的正值，速度环和路径稳定后再逐步提高。若目标被截顶，应先检查此项而不是增大 P/I。 |
| `RO_SPEED_TH` | m/s | 0.1 | 实测车体速度低于此值时按零处理 | 静止记录 body-X 速度噪声，将阈值设在噪声包络稍上方且低于可靠起步速度。过大会造成低速反馈消失和停车附近积分积累。 |
| `RO_SPEED_P` | - | 0 | 车体前向速度 PI 的比例增益 | 先令 I 为 0，在直线、低速、无转向条件下做速度阶跃。响应慢可增大；油门振荡、轮胎反复打滑或速度明显超调时减小。 |
| `RO_SPEED_I` | - | 0 | 车体前向速度 PI 的积分增益 | P 和 `RO_MAX_THR_SPEED` 前馈稳定后，用缓坡或可重复负载增加 I 消除稳态误差。停车拖尾、解除负载后超调或油门长期饱和说明过大。 |
| `RO_ACCEL_LIM` | m/s² | -1 | 速度设定和最终油门上升斜率限制；-1 禁用 | 速度环稳定后按抓地力和机构冲击逐步提高。过小表现为设定/油门爬升慢，过大表现为起步打滑、电流尖峰或俯仰冲击。 |
| `RO_DECEL_LIM` | m/s² | -1 | 速度设定和最终油门下降斜率限制；Auto 减速规划也使用；-1 禁用 | 用松杆、停车和正反向测试确定。过小会制动距离长，过大可能打滑或机械冲击；任务停车规划依赖它，修改后要重测航点停车。 |
| `PP_LOOKAHD_MIN` | m | 1.0 | Pure Pursuit 最小前视距离 | 先根据车辆尺寸和最低调试速度设定，确保低速不会追逐过近目标点。过小易蛇形，过大则贴线和转弯响应迟缓。 |
| `PP_LOOKAHD_MAX` | m | 10.0 | Pure Pursuit 最大前视距离 | 根据最高任务速度和场地曲率限制上限。过大会切弯，过小会在高速下频繁修正；必须不小于 `PP_LOOKAHD_MIN`。 |
| `PP_LOOKAHD_GAIN` | - | 1.0 | Pure Pursuit 前视距离随速度变化的增益；越小通常转向越激进 | 内层稳定后在直线和固定半径弯道逐步调整。蛇形振荡时通常增大，切弯/转向迟钝时减小，并同时检查 MIN/MAX 是否已限幅。 |
| `RD_MISS_SPD_GAIN` | - | -1 | 航点夹角导致的速度削减增益；-1 禁用 | 最后在不同夹角连续航点上调。过弯冲出路径时增加减速作用；直线路段或小转角被无谓降速时减小。先确认路径误差不是内层造成。 |
| `RO_JERK_LIM` | m/s³ | -1 | Auto/航点停车速度规划的 jerk 限制；-1 禁用相关平滑规划 | 在加减速度限制确定后调平顺性。减小可降低加速度突变但会更早、更缓慢地制动；过大接近无平滑。修改后核对任务停车距离。 |
| `NAV_ACC_RAD` | m | 10；机型覆盖为 2 | 航点接受/停车半径 | 最后按定位精度、车身尺寸和实际制动距离设定。过小可能在航点附近徘徊或无法完成，过大会提前判定到达和切角。 |

## 11. ULog 调参脚本

当前工作树根目录提供 `rover_ulog_plot.py`。脚本通过 pyulog 的 topic 白名单只
加载 Rover 调参所需数据，生成六张图和一个结构化 `summary.md`，不会导出整份
日志的逐样本原始数据。

```sh
python3 rover_ulog_plot.py <log.ulg> --out <output-directory>
python3 rover_ulog_plot.py <log.ulg> --start 30 --end 90
python3 rover_ulog_plot.py <log.ulg> --list
```

输出包括角速度内环、航向串级环、车体速度环、Pure Pursuit 路径、M2006
左右轮内环以及模式/安全状态图。摘要同时列出 ULog 保存的 Rover 参数快照、
RMS 误差和缺失字段。

当前默认 Logger topic 集合包含 Rover 上层控制状态和最终 `actuator_motors`，
但不保证包含 `m2006_motor_status`、`actuator_motors_rover` 和
`hybrid_vehicle_status`。缺失时脚本会明确报告，而不会用其他字段冒充。若后续
需要在 ULog 中完整分析 M2006 最内层，应先单独确认日志配置或再增加默认日志
topic；仅执行 `listener` 不会把该 topic 自动写入 ULog。

## 12. 推荐调参顺序

1. 先整定两轮 `M2K_SPD_*`，确认左右轮目标 rotor rpm、实测 rpm 和电流响应一致。
2. 冻结航向外环影响，在 Acro 中整定 `RO_YAW_RATE_P/I` 及角速度前馈标定。
3. 在 Stabilized 中整定 `RO_YAW_P`。
4. 在 Position 中整定 `RO_SPEED_P/I` 和 `RO_MAX_THR_SPEED`。
5. 最后调 `RO_ACCEL_LIM`、`RO_DECEL_LIM`、`RO_JERK_LIM`、Pure Pursuit 和任务过弯参数。

每次结论必须同时检查设定值、实测值、误差、执行器输出和有效样本条件；带载
启动落入静摩擦区的样本不能用于辨识闭环增益。

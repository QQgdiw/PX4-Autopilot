# Hybrid Differential Rover 实时调参协议与固件集成

本文说明普通可变形 Quad/Rover 固件中的 Differential Rover 实时调参链路。该功能只涉及 PX4、MAVLink 协议和固件侧；QGC 客户端实现不在本仓库范围内。

## 1. 协议版本

- PX4 MAVLink dialect：`hybrid_vehicle`
- MAVLink 仓库分支：`feature/hybrid-rover-tuning-v1.16.1`
- MAVLink commit：`21922689c6fb113884df0f66582d8e602286fdc1`
- 组合协议标签：`qgc-hybrid-rover-tuning-v1.16.1-r1`

`hybrid_vehicle.xml` 引入独立的 `rover_tuning.xml`。因此 PX4 的 `hybrid_vehicle` 与 QGC 的 `qgc_hybrid` composite 同时包含原有 command 50000、message 60000 和四条 Rover 调参消息；`qgc_hybrid` 继续排除会与 message 60000 冲突的 Storm32 dialect。

| 消息 | ID | Payload LEN | CRC Extra | 内容 |
|---|---:|---:|---:|---|
| `ROVER_RATE_TUNING_STATUS` | 60100 | 27 | 147 | 偏航角速度响应、目标、积分和差速输出 |
| `ROVER_ATTITUDE_TUNING_STATUS` | 60101 | 23 | 85 | 航向响应、目标和角速度外环输出 |
| `ROVER_VELOCITY_TUNING_STATUS` | 60102 | 43 | 217 | 车体系速度响应、目标、积分和油门输出 |
| `ROVER_POSITION_TUNING_STATUS` | 60103 | 44 | 90 | 本地位置、目标、Pure Pursuit 路径误差和 XY reset counter |

## 2. 固件数据链路

```text
Differential Rover 控制器
  ├─ rover_rate_status + rover_steering_setpoint
  │    └─ ROVER_RATE_TUNING_STATUS (60100)
  ├─ rover_attitude_status + rover_rate_setpoint
  │    └─ ROVER_ATTITUDE_TUNING_STATUS (60101)
  ├─ rover_velocity_status + rover_throttle_setpoint
  │    └─ ROVER_VELOCITY_TUNING_STATUS (60102)
  └─ rover_position_status
       └─ ROVER_POSITION_TUNING_STATUS (60103)
```

Rate、Attitude、Velocity 状态都带有明确的 `active`。Position 使用单个原子状态同时发布当前位置、活动目标、路径诊断、有效位和 EKF XY reset counter，避免上位机把不同控制周期的数据拼成一个样本。

Rate、Attitude、Velocity 的执行器/下一级输出只有在其 uORB 时间戳与 controller status 时间戳完全相等时才标为有效。控制器状态超过 200 ms 未更新时，stream 发送 `valid_flags=0`、数值为 NaN 的终止帧。

## 3. Hybrid 形态门控

四条 stream 只在以下条件同时成立时发布有效调参字段：

1. `vehicle_status.vehicle_type == VEHICLE_TYPE_ROVER`；
2. `vehicle_status` 新鲜度小于 1 s；
3. `hybrid_vehicle_status` 新鲜度小于 200 ms；
4. `current_state == HYBRID_STATE_DRIVING`；
5. `fault_reason == TRANSFORM_FAULT_NONE`；
6. 对应 controller status 新鲜且 `active=true`。

Quad、Transitioning、Fault、controller inactive 或 controller status stale 时发送终止帧：`drive_type=UNKNOWN` 或 Differential、`valid_flags=0`，所有不可用浮点字段为 NaN。stream 不引用 `mini_vehicle_control`，也不引入 mini 固件、执行器或板级配置。

## 4. 启停方式

四条调参 stream 只注册到 MAVLink stream registry，不进入 Normal、Onboard 或其他默认 rate 表。QGC 必须使用 `MAV_CMD_SET_MESSAGE_INTERVAL`（command 511）按需启用、停用或恢复默认值。

通用 stream 配置链路采用有界 handoff：

- 请求带固定存储和 generation；
- 排队、执行和提交共享一个 1 s 单调 deadline；
- 超时后晚到的回调不能提交；
- main thread 独占 stream registry；
- prepare 与 commit 分离，commit 不分配、不析构且不持有阻塞锁；
- reader 保护期间，替换或删除的 stream 延迟析构；
- receiver stop/join 和 shutdown 会唤醒等待者；
- command 511/512 只有真实提交成功才返回 `MAV_RESULT_ACCEPTED`。

## 5. 调试与验收

推荐从内环到外环依次启用 60100、60101、60102、60103，并与 `docs/hybrid/rover_pid_and_parameter_reference.md` 的调参顺序配合。

自动验证包括：

```sh
make tests
make zeroone_x6_hybrid
```

还需实机/QGC完成以下验收：

- command 511 按需启停四条 stream，ACK 与实际生效一致；
- Quad、变形中、Fault 和 Rover 控制器 inactive 时收到终止帧；
- Rover 各模式中响应、目标、积分和输出与 ULog 同周期数据一致；
- 高频启停与链路断开重连不会产生晚到生效或退出死锁。

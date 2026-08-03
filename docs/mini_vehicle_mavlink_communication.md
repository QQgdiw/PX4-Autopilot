# Mini Quad/Rover MAVLink 通信接口

## 1. 文档范围

本文面向与 `change_mini_v1.16.1` 固件联调的机载电脑程序，目标固件为：

- PX4 分支：`change_mini_v1.16.1`
- 固件源码锚点：`2f5d1f003b3106060e70df012de59bfc3404837c`
- 构建目标：`make hkust_nxt-dual_mini`
- 机型：无变形机构的 Quad/Rover 双模载具
- airframe：`SYS_AUTOSTART=22002`
- 正式车辆控制入口：MAVLink2 `MANUAL_CONTROL`

车辆控制、解锁、模式、双模切换和云台均使用标准 MAVLink 消息。私有消息仅用于
Differential Rover 实时调参，不参与车辆控制。

## 2. 协议版本和 dialect

车辆轴和模式控制只依赖 `common.xml`，但正式机载构建仍建议统一使用下列固定 dialect；
它同时保证 gimbal v2 和私有 Rover 消息版本一致，不能从可移动的 MAVLink `master`
重新生成：

| 项目 | 固定值 |
| --- | --- |
| MAVLink fork | `https://github.com/QQgdiw/mavlink.git` |
| commit | `07c6964a8fcc364c49d394f0bf0275b9fc05857d` |
| PX4/机载端 dialect | `mini_rover` |
| QGC composite dialect | `qgc_mini_rover` |

部署时建议设置 `MAV_PROTO_VER=2`。四个私有消息及高频
`GIMBAL_MANAGER_SET_PITCHYAW`（287）的 ID 大于 255，MAVLink1 无法传输；只做基础
车辆轴控制时虽然 MAVLink1 可以编码相关消息，也不应在同一产品中维护两套链路行为。
某些系统预装的旧 `pymavlink` 尚无 message 287，机载构建必须在启动测试中确认所用
dialect 实际生成了该 API，不能只检查 Python 包可以 import。

本机 `HEARTBEAT.type` 固定为数值 22（`MAV_TYPE_VTOL_FIXEDROTOR`），这是兼容性机型
标识，不代表当前是 Quad 还是 Rover。mini 没有私有 MAVLink
`HYBRID_VEHICLE_STATUS`，形态必须通过 `EXTENDED_SYS_STATE` 确认。

## 3. 物理链路

### 3.1 USB，当前推荐链路

飞控默认参数 `SYS_USB_AUTO=2`、`USB_MAV_MODE=2`，USB 枚举后 PX4 在
`/dev/ttyACM0` 上启动 Onboard 模式 MAVLink。Linux 设备号可能随插拔变化，产品程序应
通过 USB VID/PID 或稳定的 udev symlink 选择设备，不要永久写死 ACM 序号。

USB Onboard 模式默认发送预算为 100,000 B/s，足以承载 20--50 Hz
`MANUAL_CONTROL` 和按需 Rover tuning streams。一个串口
设备只能由一个进程持有；若 QGC 和机载程序必须同时通过同一 USB 访问飞控，应由
`mavlink-routerd` 等单一进程持有 `/dev/ttyACM*`，再向两个 UDP endpoint 转发。禁止
QGC 与业务程序同时直接打开同一设备。

### 3.2 TELEM2 备用链路

HKUST NXT-Dual 的 TELEM2 对应 `/dev/ttyS3`。推荐参数如下，全部保存后重启生效：

```text
MAV_1_CONFIG   = 102       # TELEM2
SER_TEL2_BAUD  = 921600
MAV_1_MODE     = 2         # Onboard
MAV_1_RATE     = 0         # 自动使用约 baud/20 B/s
MAV_1_FORWARD  = 0
MAV_1_RADIO_CTL= 0
```

同一物理串口不能同时分配给 MAVLink 和 uXRCE-DDS。使用上述配置时必须保持
`UXRCE_DDS_CFG!=102`。57,600/115,200 baud 可承载车辆控制和低频状态，但不能作为四层
Rover 实时调参曲线的验收链路；实时调参以 USB 为准。

连接后在 NSH 执行 `mavlink status`，确认目标实例、设备、MAVLink2、模式和发送丢包率。

## 4. MAVLink 身份和心跳

默认飞控身份是：

- `MAV_SYS_ID=1`
- `MAV_COMP_ID=1`（Autopilot）

机载电脑属于同一 vehicle，推荐使用相同 system ID 和独立 component ID：

- source system：与 `MAV_SYS_ID` 相同
- source component：`MAV_COMP_ID_ONBOARD_COMPUTER=191`
- `HEARTBEAT.type=MAV_TYPE_ONBOARD_CONTROLLER`
- `HEARTBEAT.autopilot=MAV_AUTOPILOT_INVALID`
- 发送频率：1 Hz

所有 `COMMAND_LONG`/`COMMAND_INT` 应精确发送到 `target_system=MAV_SYS_ID`、
`target_component=MAV_COMP_ID`。PX4 对普通命令不把 `target_system=0` 当成可靠的产品
接口。`MANUAL_CONTROL.target` 可为 0 或本机 system ID，正式程序仍应填精确 ID。

同一时刻只允许一个程序持续发送 `MANUAL_CONTROL`。QGC joystick、测试脚本和正式机载
程序不能并行争用；PX4 按 MAVLink instance 标识输入源，不能可靠区分同一 instance 上
两个发送者交错的数据。

## 5. 上机前参数

| 参数 | 源码默认 | 本项目建议 | 说明 |
| --- | ---: | ---: | --- |
| `COM_RC_IN_MODE` | 3 | 1 或 2 | 1=只用 MAVLink joystick；2=RC/MAVLink 失效后整源切换；禁止保留默认 3 |
| `COM_RC_LOSS_T` | 0.5 s | 0.5 s 起测 | 最后一帧手动输入可继续生效到此超时 |
| `COM_FAIL_ACT_T` | 5 s | 必须实车评审 | 进入配置 failsafe 动作前的默认延时 |
| `NAV_RCL_ACT` | 2 | 必须实车评审 | 默认 2=Return，室内或无定位时未必安全 |
| `COM_RCL_EXCEPT` | 0 | 0 | 不建议忽略手动输入失联 |
| `MAN_ARM_GESTURE` | 1 | 0 | API 控制只使用显式 arm/disarm，避免轴组合触发手势 |
| `MAV_PROTO_VER` | 0 | 2 | 固定 MAVLink2，尤其是启用私有 tuning 时 |

`COM_RC_IN_MODE=1` 会禁用物理接收机作为手动输入，因此物理 RC channel 10 也不能同时
控制云台。需要 RC 备用时使用模式 2，但它切换的是整套 `roll/pitch/yaw/throttle/aux`
输入源，不支持“车辆轴来自机载电脑、AUX1 同时来自物理 RC”。当前输入失效超过
`COM_RC_LOSS_T` 后，另一个来源才可接管。

默认 `NAV_RCL_ACT=2` 是 Return。必须分别在 Quad、Rover 形态验证所选失联动作；不能
只验证地面停车后就假定空中行为安全。`MANUAL_CONTROL` 失联和 GCS HEARTBEAT 失联是
两套检测，车辆主控制安全不得只依赖 GCS datalink failsafe。测试记录必须包含从最后
一帧输入到 `COM_RC_LOSS_T`、`COM_FAIL_ACT_T` 以及最终动作的完整时间线，不能只记录
最后进入了什么模式。

## 6. 命令和状态确认

### 6.1 解锁与上锁

使用 `MAV_CMD_COMPONENT_ARM_DISARM`（400）：

| 操作 | param1 | param2 |
| --- | ---: | ---: |
| Arm | 1 | 0 |
| Disarm | 0 | 0 |

禁止在产品逻辑中使用强制 disarm magic value `param2=21196`。收到
`COMMAND_ACK.result=MAV_RESULT_ACCEPTED` 后，还要确认后续 `HEARTBEAT.base_mode` 的
`MAV_MODE_FLAG_SAFETY_ARMED` 位达到目标状态。ACK 表示命令处理结果，不替代状态确认。

### 6.2 PX4 控制模式

使用 `MAV_CMD_DO_SET_MODE`（176），`param1=1`
（`MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`），`param2` 使用 PX4 custom main mode：

| 模式 | param2 | param3 |
| --- | ---: | ---: |
| Manual | 1 | 0 |
| Altitude | 2 | 0 |
| Position | 3 | 0 |
| Auto | 4 | 对应 submode |
| Acro | 5 | 0 |
| Offboard | 6 | 0 |
| Stabilized | 7 | 0 |

Rover 用 `MANUAL_CONTROL` 直接驾驶前必须进入 Manual；Quad 应按任务选择 Stabilized、
Altitude 或 Position。双模切换不会替机载程序自动选择一个适合两种形态的控制模式。
ACK 后用新的 `HEARTBEAT.custom_mode` 确认模式。

`MAV_CMD_DO_SET_MODE` 接收层可能先发送 Accepted，Commander 随后才根据模式条件发送
拒绝结果。发送端可能看到同一 command 的多个 ACK，不能把第一个 Accepted 当作最终
模式状态；以命令之后收到的 `HEARTBEAT.custom_mode` 为最终判据。

### 6.3 Quad/Rover 切换

复用标准 `MAV_CMD_DO_VTOL_TRANSITION`（3000）：

| 目标形态 | param1 | MAVLink 名称 |
| --- | ---: | --- |
| Quad | 3 | `MAV_VTOL_STATE_MC` |
| Rover | 4 | `MAV_VTOL_STATE_FW` |

`param2` 至 `param7` 填 0。固件没有变形等待态，接受后直接改变控制器所有权：

- Quad -> Rover：仅当 `vehicle_land_detected.landed=true` 且样本不超过 1.5 s 时接受；
  否则 ACK 为 `MAV_RESULT_TEMPORARILY_REJECTED`。
- Rover -> Quad：当前固件直接接受，不额外要求 landed 或 disarmed。
- 同形态重复请求：返回 `MAV_RESULT_ACCEPTED`，是幂等操作。
- 非 3/4 值：返回 `MAV_RESULT_UNSUPPORTED`。

ACK 只表示内部形态请求已接受。切换后输出层还会等待目标控制器在切换 epoch 之后产生
且不超过 200 ms 的新执行器样本。最终形态用新的 `EXTENDED_SYS_STATE.vtol_state` 确认：

| `vtol_state` | mini 语义 |
| ---: | --- |
| `MAV_VTOL_STATE_MC` (3) | Quad |
| `MAV_VTOL_STATE_FW` (4) | Rover |

不要用 `HEARTBEAT.type` 判断形态，也不要等待
`MAV_VTOL_STATE_TRANSITION_TO_MC/FW`；mini 正常切换不会对外暴露变形过渡态。

推荐切换状态机：

1. 持续发送全零/最低油门的中立 `MANUAL_CONTROL`，车辆停止后再发切换命令。
2. 不暂停控制发送线程，等待与 command 3000 匹配的 `COMMAND_ACK`。
3. ACK 非 Accepted 时保持原形态和中立输入；暂时拒绝只能在 landed 条件满足后重试。
4. ACK Accepted 后等待一帧新的目标 `EXTENDED_SYS_STATE`。
5. 继续发送中立值，进入目标控制模式；确认后再缓升控制量。
6. 任一步超时都保持中立并报故障，禁止根据固定延时猜测切换成功。

形态切换不改变 armed 状态。Rover -> Quad 也可在 armed 时接受，因此机载程序必须把
“停车、中立输入、状态确认”作为自己的操作前置条件。

## 7. 归一化 MANUAL_CONTROL

建议 20--50 Hz 连续发送，示例实现使用 20 Hz。低于 10 Hz 时，相对默认 0.5 s 失联
阈值的丢帧余量太小。消息没有逐帧 ACK。

| MAVLink 字段 | 合法范围 | PX4 内部字段 | Quad 语义 | Rover Manual 语义 |
| --- | ---: | --- | --- | --- |
| `x` | -1000..1000 | `pitch=x/1000` | pitch stick | 前进/后退请求 |
| `y` | -1000..1000 | `roll=y/1000` | roll stick | 差速转向，正值为右转请求 |
| `z` | 0..1000 | `throttle=z/500-1` | 0%..100% throttle | 不用于车轮控制 |
| `r` | -1000..1000 | `yaw=r/1000` | yaw stick | 不用于车轮控制 |
| `buttons` | bitmask | `buttons` | 可选 | 当前不用，填 0 |

Rover 安全中立帧为 `x=0,y=0,z=0,r=0,buttons=0`。在 Rover 中也要发送合法的 `z/r`，
不要使用协议 invalid sentinel。当前 PX4 对越界轴不是整帧拒绝，而是把该字段保留为
零初始化值并仍将消息标记为 valid；对 `z` 而言，内部零等价于 MAVLink `z=500` 的
中油门，不是 `z=0` 的最低油门。机载程序必须在发送前自行限幅和检查有限值。

车轮是单向 PWM。Rover 控制器产生的任一负轮指令会在最终 mini 输出层钳为 0：

- `x<0` 不会得到正常倒车，两轮通常都停止；
- 原地转向时，反向的一轮停止，另一轮仍可正转；
- 上层不得把“请求为负”解释成“实车一定反转”。

切换 epoch 会丢弃切换前的缓存输入。控制发送线程必须持续运行，并在形态确认后提供新的
中立帧，再逐步增加命令。程序退出前应先中立、显式 disarm 并确认状态；不能在 armed
状态直接停止发送线程，把停车完全留给超时动作。

## 8. MAIN5 摄像头云台

MAIN5 是普通 50 Hz PWM 舵机，参数范围 1000--2000 us，默认、disarmed 和 failsafe 均
为 1500 us。`MNT_RANGE_PITCH=90` 表示总机械行程 90 度，即当前零偏下约 -45 到 +45 度。

固件配置 `MNT_MODE_IN=0`，支持 RC 和 MAVLink gimbal v2，最后主动的输入取得控制权。
机载端推荐流程：

1. 用 `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`（1001）取得 primary control；param1/2
   填机载端 source sysid/compid，param3/4 填 0，param7 填 0。
2. 连续角度控制使用 `GIMBAL_MANAGER_SET_PITCHYAW`（287），建议 10--20 Hz。
3. `pitch` 使用弧度并限幅到 `[-pi/4,+pi/4]`，`yaw=0`；未使用的
   `pitch_rate/yaw_rate` 填 NaN，`gimbal_device_id=0`，flags 使用 follow 模式 0。
4. 低频、需要 ACK 的单次设置可使用
   `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`（1000），其角度单位是度。

不要用已废弃的 gimbal v1 mount 命令作为新接口。物理 RC 只有在其整套手动输入源被
PX4 selector 选中时，channel 10 -> AUX1 才可控制云台；RC AUX 发生超过约 0.25 的明显
移动后会重新取得 Auto input 的控制权。

## 9. Rover 私有实时调参消息

这些消息只传控制器 response/setpoint/status，不能解锁、切换或驱动车辆：

| 消息 | ID | 建议上限 | Payload / CRC extra |
| --- | ---: | ---: | --- |
| `ROVER_RATE_TUNING_STATUS` | 60100 | 50 Hz | 27 B / 147 |
| `ROVER_ATTITUDE_TUNING_STATUS` | 60101 | 30 Hz | 23 B / 85 |
| `ROVER_VELOCITY_TUNING_STATUS` | 60102 | 25 Hz | 43 B / 217 |
| `ROVER_POSITION_TUNING_STATUS` | 60103 | 10 Hz | 44 B / 90 |

它们默认不发送。使用 `MAV_CMD_SET_MESSAGE_INTERVAL` 请求，param1 为 message ID，
param2 为周期 us；禁用时 param2=-1。仅在 Differential Rover 活动且状态新鲜时有效。
`drive_type=0, valid_flags=0` 是 Quad/inactive 清除帧，不是协议错误。完整字段和有效位
定义见 `docs/mini_rover_realtime_tuning_guide.md`。

## 10. pymavlink 最小骨架

下例展示控制发送线程和命令 ACK 必须并行。它省略业务状态机和异常恢复，不能直接替代
产品安全逻辑。若 `MAV_SYS_ID` 不是 1，应同时修改机载端 source system。

```python
import math
import threading
import time

from pymavlink import mavutil

AP_SYSID = 1
AP_COMPID = 1
SOURCE_COMPID = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER

link = mavutil.mavlink_connection(
    "/dev/ttyACM0", source_system=AP_SYSID, source_component=SOURCE_COMPID
)
link.wait_heartbeat(timeout=10)

send_lock = threading.Lock()
stop_event = threading.Event()
axes = {"x": 0, "y": 0, "z": 0, "r": 0}


def control_tx():
    next_heartbeat = 0.0
    while not stop_event.is_set():
        now = time.monotonic()
        with send_lock:
            link.mav.manual_control_send(
                AP_SYSID, axes["x"], axes["y"], axes["z"], axes["r"], 0
            )
            if now >= next_heartbeat:
                link.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                next_heartbeat = now + 1.0
        stop_event.wait(0.05)  # 20 Hz MANUAL_CONTROL


threading.Thread(target=control_tx, daemon=True).start()


def command_long(command, params, timeout_s=3.0):
    while link.recv_match(type="COMMAND_ACK", blocking=False):
        pass
    with send_lock:
        link.mav.command_long_send(
            AP_SYSID, AP_COMPID, command, 0, *(list(params) + [0.0] * (7-len(params)))
        )
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        ack = link.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.1)
        if ack and ack.command == command:
            return ack.result
    raise TimeoutError(f"no COMMAND_ACK for {command}")


def wait_main_mode(expected, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        hb = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.1)
        if hb and ((hb.custom_mode >> 16) & 0xff) == expected:
            return
    raise TimeoutError(f"PX4 main mode did not become {expected}")


def wait_vtol_state(expected, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = link.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=0.1)
        if state and state.vtol_state == expected:
            return
    raise TimeoutError(f"VTOL state did not become {expected}")


# Rover Manual mode, then request Rover shape. Keep axes neutral while waiting.
result = command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, [1, 1, 0])
assert result == mavutil.mavlink.MAV_RESULT_ACCEPTED
wait_main_mode(1)
result = command_long(
    mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
    [mavutil.mavlink.MAV_VTOL_STATE_FW],
)
assert result == mavutil.mavlink.MAV_RESULT_ACCEPTED
wait_vtol_state(mavutil.mavlink.MAV_VTOL_STATE_FW)

# Drive forward/right only after a new EXTENDED_SYS_STATE confirms FW/Rover.
# axes.update(x=300, y=150, z=0, r=0)
```

生产程序还必须实现：目标状态新鲜度、ACK 去重、断线重连、输出限幅/斜坡、线程故障
监控，以及 disarm 确认后才能退出。

## 11. 联调验收清单

在拆桨、架空车轮条件下先完成台架测试：

- `param show` 确认第 5 节参数，`mavlink status` 确认正确设备和 MAVLink2。
- 机载端 1 Hz HEARTBEAT 可持续收发，sysid/compid 无冲突。
- Arm/disarm 的 ACK 与后续 HEARTBEAT armed 位一致，拒绝原因可记录。
- Rover Manual 下 `x>0` 前进，`y>0` 右转，`y<0` 左转；负轮命令按预期归零。
- Quad 控制先从最低 `z` 开始，确认四个轴方向和所选 PX4 模式。
- airborne Quad -> Rover 得到 Temporarily Rejected；landed 后得到 Accepted。
- 两个方向切换后都收到新 `EXTENDED_SYS_STATE`，且旧输入不会驱动新控制器。
- 主动拔线分别验证 Quad/Rover 的 `COM_RC_LOSS_T`、`COM_FAIL_ACT_T` 和
  `NAV_RCL_ACT` 实际时间线与动作。
- 云台上电 1500 us，角度两端无机械顶死，RC/MAVLink 控制权切换符合预期。
- 需要 tuning 时，USB 上四个按需 stream 的频率、inactive 清除帧和断线恢复正确。

只有以上硬件项目实际通过后，才能把“协议已实现”升级为“整机控制链路已验收”。

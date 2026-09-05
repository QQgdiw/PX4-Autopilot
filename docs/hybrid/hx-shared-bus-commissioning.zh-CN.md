# HX 三舵机共享总线调试手册

适用对象：一个华馨京 HX8-U45H-M 起落架舵机和两个幻尔 HX-65HM 变形舵机，
三者共用一条半双工 UART，总线默认 `1000000 baud, 8N1`。

> 本手册中的 `<实测值>` 必须替换为本机测量结果。禁止带桨调试，首次动作时应卸载机构或限制行程，并准备可直接切断舵机电源的急停。

## 1. 接线和上位机预配置

- 飞控默认使用 EXT2；`HX8_SER_CFG` 只选择端口，`HX_BAUD` 是三台舵机共用的唯一速率参数。
- 三台舵机、电源和飞控必须共地；确认电源能承受三台舵机同时启动和堵转时的峰值。
- 厂家上位机与飞控不得同时连接或驱动总线。
- 两个 HX-65HM 出厂 ID 都可能是 1，必须一次只接一台完成配置，之后才能共同上电。
- 两台 HX-65HM 共线时禁止发送 Ping。实机确认其 Ping 会忽略目标 ID，两台舵机会
  在约 43.2 us 后同时回包并造成推挽争用；PX4 使用单播身份寄存器 Read 完成在线与ID核验。

逐台使用厂家上位机设置并回读：

| 舵机 | ID | 波特率 | 其他要求 |
|---|---:|---:|---|
| HX8-U45H-M | 0 | 1 Mbps | 开启响应、堵转释放；保护值与下文 `HX8_CFG_*` 一致 |
| 左 HX-65HM | 1 | 1 Mbps（baud code 0） | 位置模式、响应级别 1、保护掩码 44 |
| 右 HX-65HM | 2 | 1 Mbps（baud code 0） | 位置模式、响应级别 1、保护掩码 44 |

PX4 不回读比较舵机波特率；速率错误的舵机不会产生有效响应。

## 2. 首次 PX4 参数配置

先设置基础参数和 HX8 保护期望值。端点参数保持无效默认值，此时驱动可以读取位置，
但禁止机构动作和解锁。

```sh
param set HYB_ACT_TYPE 2
param set HX8_SER_CFG 401
param set HX_BAUD 1000000

param set PWM_MAIN_FUNC5 0
param set PWM_MAIN_FUNC6 0
param set PWM_MAIN_FUNC8 0
param set PWM_MAIN_DIS8 0
param set PWM_MAIN_FAIL8 0
param set M2K_EN 1

param set HX8_ID 0
param set H65_L_ID 1
param set H65_R_ID 2
param set H65_PROT 44

param set HX8_CFG_RSP 1
param set HX8_CFG_STL 1
param set HX8_CFG_SPWR 6000
param set HX8_CFG_TEMP 70
param set HX8_CFG_PWR 20000
param set HX8_CFG_CUR 4000
param set HX8_CFG_VMIN 9000
param set HX8_CFG_VMAX 12600
param set HX8_CFG_BOOT 0
reboot
```

`HX8_SER_CFG=401` 表示当前机型的 EXT2。若改用其他端口，应在 QGC 中选择端口，
不要猜测数值。Motor 5/6 已由 M2006 CAN 使用，因此对应 PWM 功能必须为 0。
上面的保护值是当前已批准的起始配置，仍需与实际电源和机构能力核对。

重启后检查：

```sh
hx8_uart_servo status
listener hx8_servo_status 1
listener hx65_servo_status 1
```

此阶段因端点尚未标定，`motion_config=0` 和 `config check` 失败属于预期结果；
三台舵机应在线、健康，listener 中的单舵机配置核验应通过。CLI 的 HX65 pair
`verified` 会与 `motion_config` 一起保持为 0，直到七个端点有效。如果 HX8 保护配置
不一致，必须在完全未解锁且机构卸载时执行：

```sh
hx8_uart_servo config write
hx8_uart_servo config check
```

`config write` 只写 HX8 持久保护配置，不写 ID、波特率，也不配置 HX-65HM。

## 3. 标定七个机械位置

断开推进动力并固定机体。用厂家工具低速移动，或在确认允许反驱后手动移动机构；
每到一个位置，重新执行对应的 `listener` 并记录稳定值。

| 参数 | 位置 | 从哪里读取 |
|---|---|---|
| `LG_ANG_DN` | 起落架完全放下 | `hx8_servo_status.angle_deg` |
| `LG_ANG_CLR` | 起落架与车轮同高/刚好避让 | `hx8_servo_status.angle_deg` |
| `LG_ANG_STW` | 起落架完全收起 | `hx8_servo_status.angle_deg` |
| `H65_L_QUD` | 左侧 Quad 端点 | `hx65_servo_status.left_position_steps` |
| `H65_R_QUD` | 右侧 Quad 端点 | `hx65_servo_status.right_position_steps` |
| `H65_L_ROV` | 左侧 Rover 端点 | `hx65_servo_status.left_position_steps` |
| `H65_R_ROV` | 右侧 Rover 端点 | `hx65_servo_status.right_position_steps` |

要求：`LG_ANG_CLR` 必须严格位于放下与收起角度之间；左右 HX-65HM 可反向运动，
分别填写实测端点，不要强行改成相同符号。HX-65HM 有效端点范围为
`-30719..30719`。

写入实测值和动作参数：

```sh
param set LG_ANG_DN <实测值>
param set LG_ANG_CLR <实测值>
param set LG_ANG_STW <实测值>
param set LG_PWR_LIM <实测安全值_mW>

param set H65_L_QUD <实测值>
param set H65_R_QUD <实测值>
param set H65_L_ROV <实测值>
param set H65_R_ROV <实测值>

param set LG_MOVE_T 5000
param set LG_ACC_T 200
param set LG_DEC_T 200
param set LG_ANG_TOL 5
param set LG_TIMEOUT 8
param set LG_LAND_T 1
param set LG_AIR_T 1

param set H65_SPEED 1000
param set H65_ACC 10
param set H65_TOL 100
param set H65_SKEW 0.15
param set HYBRID_TRANS_T 6
param set HYB_STALL_T 0.8
param set HYB_STALL_D 0.02
reboot
```

`LG_PWR_LIM` 必须非零且不大于 `HX8_CFG_PWR`。`LG_MOVE_T` 必须大于
`LG_ACC_T + LG_DEC_T`，`LG_TIMEOUT` 应留出大于实际全行程时间的余量。

## 4. 最终检查和状态判断

```sh
param show HX_BAUD
param show HYB_ACT_TYPE
hx8_uart_servo status
hx8_uart_servo config check
echo $?
listener hx8_servo_status 3
listener hx65_servo_status 3
listener hybrid_vehicle_status 3
```

调试固件可执行 `hx8_uart_servo trace`。它以环形缓存保留最近16条HX8/HX65收发事件，
在首个HX-65HM Monitor超时时冻结；`outcome=4`表示该触发条件，`wrapped=1`仅表示更早的
历史已被覆盖，输出仍按时间顺序排列。该命令不会实时打印总线数据。

混合协议调度在最后一个HX8发送或接收字节后为HX-65HM保留固定40 ms解析恢复时间；
恢复时间到达后优先发送待处理HX-65HM事务。该值是实机协议兼容约束而非调参项，
HX8紧急卸力仍可抢占并重新开始恢复计时。

验收条件：

- `config check` 返回 `0`。
- HX8：`online=1`、`healthy=1`、`config_verified=1`，角度和电压/电流/温度合理。
- HX-65HM：左右均 online/healthy/verified、位置有效，`motion_config_valid=1`。
- `timeout_count`、`protocol_error_count` 和保护标志不持续增长。
- 稳定状态下 `sequence_fault=0`、`fault_reason=0`；只有 Ready 状态才允许对应推进系统解锁。

## 5. 无桨动作验收

自动模式先执行 `param set LG_AUTO_EN 1` 并重启：

1. Quad 空中请求 Rover：先放起落架，落地稳定且完全放下后卸载 Quad；双 HX-65HM
   变形到位，再收起落架。到达 `LG_ANG_CLR` 后 Rover Ready，重新解锁后才能行走。
2. Rover 地面请求 Quad：Rover 停止并放下起落架；完全放下后变形。Quad Ready 后
   重新解锁，确认持续离地后再收起落架。

手动模式执行 `param set LG_AUTO_EN 0`、`param set LG_MAN_CH <1..6>` 后重启：
通道小于 `-0.5` 放下，中位保持，大于 `0.5` 收起。起落架与变形动作允许同时进行，
其角度不参与变形阶段或Ready判定；HX8在线、配置核验和保护健康仍参与Ready与解锁门控。
Quad转Rover仍要求着陆确认并上锁，Rover转Quad仍要求上锁。

每次完整往返至少观察一次三个 listener，确认左右变形同步、无碰撞、无异常电流和温升。

## 6. 必做故障测试

- 分别断开 HX8、左 HX-65HM、右 HX-65HM：状态必须转为不健康/故障，禁止继续危险动作或解锁。
- 轻微限制一侧 HX-65HM：超过 `H65_SKEW` 或无进展超时后必须进入变形故障，另一侧不得继续硬拉。
- 自动模式限制起落架：超过 `LG_TIMEOUT` 必须产生起落架超时，Quad→Rover 的空中过渡不得提前关闭 Quad。
- 手动模式触发HX8失联或保护：不得因取消角度门控而变为Ready或允许解锁。
- 手动模式断开遥控输入：起落架必须保持，不得自动前往新端点。

发生故障后先断开舵机动力并排除机械/供电原因，再检查 `hx8_servo_status`、
`hx65_servo_status` 和 `hybrid_vehicle_status`。不得通过放宽容差或增大功率掩盖卡滞、方向错误或端点错误。

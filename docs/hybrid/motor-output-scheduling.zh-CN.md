# Hybrid 电机输出调度与 Bluejay 台架验收

## 核实基准与根因边界

2026-10-17 基于 `feature/testc4-rover-tuning` 的 `4feadd93f15fa0c09040769009250b2d414cabfe`
工作树实施。故障参数 `/mnt/e/PX4/zeroone_hybird_1016_vaild1.params` 记录的是
`154c614a20000000`；两个版本的 Hybrid 电机仲裁均采用固定 20 ms 轮询。
不需要回退到故障提交。

现场示波器证据：DShot300 位周期约 3.4 μs，帧间隔约 20 ms。位速率和帧率是
两个不同量，提升位速率不能消除仲裁的 20 ms 等待。

代码路径为：多旋翼 ControlAllocator 发布 `actuator_motors_mc`，原生差速控制器
发布 `actuator_motors_rover`，Hybrid 仲裁发布最终 `actuator_motors`；
`FunctionMotors` 的订阅回调唤醒 MixingOutput/DShot，`updateOutputs()` 调用
`up_dshot_trigger()`，底层按定时器启动一次 DMA 帧发送。DMA 不在两次触发之间
自动按控制频率循环发送。原 MixingOutput 回调缺失的备用调度为 50 ms。

因此代码中的 50 Hz 最终输出与现场观测吻合，且同时限制了新控制值更新和发帧。
[Bluejay 上游源码](https://github.com/bird-sanctuary/bluejay/blob/main/src/Bluejay.asm)
的 DShot 协议检测有计数初值 10 和 100 ms 检测等待；50 Hz 在这个窗口内通常只能
提供约 5 帧。此证据支持启动不兼容的解释，但现场具体 Bluejay 发布版本尚未知，
“只提高帧率即可恢复”仍需实测，不能把源码推断写成硬件恢复结论。

参数核实：`DSHOT_BIDIR_EN=0`、`DSHOT_3D_ENABLE=0`、`HYB_ACT_TYPE=2`。
`PWM_AUX_FUNC1..4=104/101/102/103`，第五路以后禁用；
`PWM_AUX_TIM0=-4` 表示 DShot300，`PWM_AUX_TIM1=-3` 表示 DShot600。
当前 DShot 驱动仅支持单一位速率，会将后续冲突定时器参数调整为先读到的速率。
MAIN 另有四路电机映射与 400 Hz 参数，不能把 MAIN 的设置当作 AUX 的实际波形。
本修复不导入或修改现场参数，验收需保存启动后的实际参数和 `dshot status`。

## 调度设计

| 层次 | 修复后的行为 |
|---|---|
| Hybrid 快路径 | 两个电机来源的 uORB 回调唤醒；立即取最新值、检查许可并发布最终输出 |
| Hybrid 看门狗 | 无回调时 4 ms 后再检查并发布；来源中断仍会执行失效判断 |
| 机构管理 | 20 ms 到期才更新参数、变形状态机、起落架协调、舵机命令及状态发布 |
| DShot | 新电机消息回调立即运行；无消息时 4 ms 备用运行，名义空闲重复频率 250 Hz |
| 来源时效 | Hybrid 和 DShot 电机提供者均使用 100 ms 命令年龄上限，拒绝零时间戳和未来时间戳 |

4 ms 是软件备用调度间隔，不是硬实时帧间隔保证。正常新指令更快时，回调继续
提高更新频率；工作队列拥塞可能产生抖动，须通过示波器和工作队列统计验收。
机构管理不随每次电机回调提速；管理周期受工作队列唤醒相位和负载影响，不保证
精确 20.000 ms。

有效转发保留来源的 `timestamp` 与 `timestamp_sample`。重复发送同一控制值
不会刷新其来源年龄。被禁止的输出为全 NaN，携带当前安全判断时间；DShot 将
其转换为停止值。若整个 Hybrid 模块不再发布，DShot 专用的 100 ms 提供者超时
仍会将缓存转为 NaN。其他输出驱动保留原备用调度与命令保持默认值。

HX65 必须同时具有匹配的推进所有权和 `propulsion_ready`；变形故障、位置许可、
commissioning 和变形完成时间检查保留。快路径每次读取 `actuator_armed`，急停、
lockdown 和 force_failsafe 会禁止 Hybrid 推进输出；输出驱动仍执行自身安全逻辑。

未解锁时 MixingOutput 仍将普通电机控制映射为停止输出。QGC 电机测试使用其
独立的显式测试指令、原有测试超时和安全检查；不需要伪造解锁或制造高频油门。

删除的 `i == 4 || i == 5` 特判实际限制的是 **DShot 3D 正反转油门映射**，不是
`DSHOT_BIDIR_EN` 控制的 eRPM 双向遥测。恢复为全局 `DSHOT_3D_ENABLE` 或
各电机 reversible 标志控制所有物理通道；现场两项参数为 0，不会因此开启 3D。

## 自动验证范围

`functional-HybridOutput` 直接运行生产仲裁函数，覆盖冷启动无输入、1 kHz 输入
与 50 Hz 管理分离、时间戳保留、100 ms 来源失效、Rover 轮映射、变形 epoch、
Fault、错误所有权、推进未就绪、位置丢失、commissioning 和急停。

`functional-mixer_module_tests` 新增 DShot 配置下的命令保持上限测试，覆盖
未解锁停止、无来源时限时电机测试、测试到期、正常指令中断、恢复和急停；
既有测试也验证其他驱动默认行为。这里只执行实际混控代码，没有模拟成物理
DShot DMA 成功，更不代表电调已经完成冷启动识别。

SITL test 配置增加实际 Hybrid 模块及 HX 参数提供模块，以编译生产类测试；
没有构建或修改 `zeroone_x6_default`。

## 拆桨台架验收

1. 保存当前参数；确认烧录文件 SHA-256 与交付一致。旋翼全部拆桨，固定机体，
   轮子架空。记录四个电调的实际 Bluejay 发布版本。
2. 冷上电，保持未解锁且不打开电机测试。分别测四路 DShot：位周期保持所选
   DShot300 对应值；帧间隔应主要不超过约 4 ms（或随新控制值更快），记录
   最大间隔和抖动。解码确认发送停止命令而不是转动油门。重复至少 10 次冷启动，
   观察是否完成协议识别及准备提示音。
3. 逐路 QGC 电机测试；确认映射、方向、响应和停止。退出页面/等待测试超时，
   确认停转。测试过程中触发急停，确认停转及持续停止帧。
4. 稳定 Quad 下观察 `uorb top` 中 `actuator_motors_mc` 和 `actuator_motors`，
   新控制值更新应随上游，不能仍停留在 50 Hz。结合 ULog/时间戳检查端到端延迟。
5. 执行 Quad/Rover 双向变形、起落架联动及可控故障测试。变形中和 Fault 时检查
   推进停止；稳定 Rover 时旋翼停止、轮映射正确；完成切换前的缓存不得重新生效。
6. 在受控台架上中断上游控制器发布，确认最后有效命令超过 100 ms 后，在下一次
   软件检查时输出停止；另外停止 Hybrid 发布者，验证 DShot 侧超时兜底。该步骤
   会导致载具故障/失控保护，不在带桨或飞行中执行。
7. 比较 `work_queue status`、`top`、`dshot status` 和 `uorb top`，记录 CPU 负载、
   work queue 周期、最终输出频率与调度抖动。检查 HX8/HX65 超时、重试、错误计数
   在同样动作下是否增长，机构状态发布应仍约 50 Hz。

仅完成上述实测后，才可确认 Bluejay 电机恢复和时序/负载验收通过。

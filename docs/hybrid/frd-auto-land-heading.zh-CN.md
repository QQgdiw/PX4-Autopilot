# FRD 外部视觉与公共 AUTO_LAND 固定航向

## 适用版本与接口升级

目标：`feature/testc4-rover-tuning`，`zeroone_x6_hybrid`。
修复前基线为 `a672224f4e2571ad386fa988de6c5de8e7e7d53b`；日志中的
`75f6a8081e9631b3679739ae7c3e21d185ae7ca5` 同样包含此问题。
具体固件提交以本次发布记录及 `ver all` 为准，不以分支名称代替版本锁定。

配套消息：`QQgdiw/px4_msgs` 分支 `hybrid-rover-local-heading-v1.16.1`，
固定 tag `hybrid-rover-v1.16.1-r2`，提交
`3b569db54de24ff3e7311ccb407693975f5601f6`。

`VehicleLocalPosition.MESSAGE_VERSION` 从 0 升至 1：

| 接口 | 本次含义 |
| --- | --- |
| uORB `vehicle_local_position` / `estimator_local_position` | 内部名称不变，布局增加一位 |
| DDS `/fmu/out/vehicle_local_position_v1` | 新版本输出；由生成器自动加 `_v1`，YAML 保留基础名称 |
| `heading_good_for_control` | 保留原有绝对航向对准完成判据，不强置为真 |
| `heading_good_for_local_control` | 当前世界固定局部航向可用于保持，不保证北向对准 |
| MAVLink、QGC | 本次不改消息 XML、ID、CRC 或绑定；并不通过 MAVLink 新增该标志 |

旧 r1 的 `VehicleLocalPosition` 与新话题不自动兼容；必须重编译 px4_msgs
及依赖节点，按类型版本选择话题。不能把 FRD 数据改标 NED，也不能用该标志
代替位置、姿态、时间同步、新鲜度、形态、Offboard 或解锁检查。

## 根因与复现

FRD 视觉是具有任意固定航向参考的世界坐标系，并非随载体旋转的机体系。
EKF 开启该航向融合时，`ev_yaw=true`、`yaw_align=false` 可以同时成立。
旧 `isYawFinalAlignComplete()` 因而返回 false。Auto 在处理显式 yaw 和
`MPC_YAW_MODE` 之前输出 `yaw=NaN, yawspeed=0`，降落将 NaN 缓存到
`_land_heading`。由于角速度为有限零值，后续不会生成固定角度目标。
PositionControl 最终逐帧使用实测 yaw 填补 NaN，丢失角度偏差纠正。

先在未修复的生产 FlightTaskAuto 路径运行测试：期望 0.7 rad 的两个断言
实际均为 NaN。独立 EKF 传感器仿真确认 FRD 视觉融合有效时绝对资格仍为假。
没有根据该机制调整任何 PID，也未声称它解释了全部偏航或触地冲击。

## 局部资格判定与多实例一致性

EKF 每次发布时计算新标志：

1. 滤波器已初始化、倾斜对准完成、四元数有限、航向方差有限且非负，
   不存在 `bad_hdg` 数值故障；发布的 heading 本身也必须有限。
2. 满足原有绝对航向资格，或满足以下外部视觉局部分支：
   EV yaw 融合开启、无 EV yaw fault、存在成功融合或接受的航向重置，
   且 `time_last_fuse` 在 EKF 延迟融合时间域内小于 400 ms，不能来自未来。

400 ms 复用 `2 * EV_MAX_INTERVAL`，不新增参数，不等于较长的融合失败重置
超时，也不是收到任意视觉包就刷新。它容忍短时丢包，但连续拒绝/低质量、
丢失、停止融合后不能无限保持局部资格；恢复需再次接受融合或重置。
时间范围按 EKF 融合时间域计算，包含系统既有测量延迟设计，不是零延迟保证。
已有绝对航向的机型继续按原有绝对资格处理，不额外要求持续 EV 输入。

每个 EKF 实例将标志写入自身的完整位置消息。EKF2Selector 直接复制选中
实例的结构体，保留采样时间，并统一处理位置/航向重置；不会拼接另一实例
的状态位。切换到无资格实例立即随新消息报告 false。选择器拒绝倒序和
过期样本。FlightTask 对未初始化、过期或未来时间戳清除资格。

其他既有位置生产者（LPE、VectorNav、HIL、SIH）同时填充新字段，保持原有
正常航向来源的行为；没有给它们新增“FRD 配置即有效”的捷径。

## 降落生命周期

本修复位于公共 `FlightTaskAuto` 的 LAND 分支，不依赖 Offboard 来源：
模式切到 AUTO_LAND、命令触发降落、failsafe 进入同一公共降落路径，以及
任务/返航最终生成 LAND setpoint 的路径均使用此逻辑。没有改 Navigator
命令入口、定位失败处置、人工接管、降落检测或上锁逻辑。

- 首次执行 LAND、局部资格有效且非 Weathervane 时捕获目标。
  已绝对对准时保留现有显式 yaw/航向规划语义；只有局部资格时捕获当前
  实测局部 yaw，不能解释北向 triplet yaw 为局部目标。
- 捕获后保持目标；实测 yaw 扰动不使目标每帧跟随。
  原有航向速率限制保留：若交接时上一任务目标与捕获目标不同，最终发布目标
  可先按既有限速收敛到捕获值，这不等于每帧追随实测 yaw。
- 资格丢失时清除固定目标，保持原有角速度/失效保护通道；不得用 RC 辅助
  的旧目标恢复无效角度锁定。恢复后捕获当时有效的新目标，不拉回失效前目标。
- EKF heading reset 时，同步平移并包络 `_land_heading` 与 `_yaw_sp_prev`；
  多 EKF 切换产生的合成 reset 走同一通道。
- 离开 LAND 再进入、或任务重新 activate，重新捕获；不是沿用上次降落目标。
- `MPC_LAND_RC_HELP` 的偏航摇杆仍可请求角速度，横向/油门辅助保留；
  Weathervane 仍拥有自己的角速度控制。其他 Auto 模式不因新标志放开绝对
  航向检查；本改动不保证任意 FRD 全球航点任务可用。

## 可选旧版转换：不是自动兼容

按仓库版本机制保留 `msg/px4_msgs_old/msg/VehicleLocalPositionV0.msg`，并在
`msg/translation_node/translations/translation_vehicle_local_position_v1.h`
提供可选转换源：v1 → v0 丢弃新字段；v0 → v1 保守置新字段为 false，
其他字段逐一复制。不伪造旧固件无法证明的局部航向资格。

转换节点不在飞控上运行，也不会因更新 px4_msgs 自动部署。需要另行在 ROS 2
环境编译、启动并联调；本环境只有静态字段完整性检查，没有 ROS 运行验证。
推荐直接升级依赖和话题，不把未经验证的转换节点当作本次上线前提。

## 验证与待完成实机项目

已完成：修复前失败复现；EKF 合成传感器测试；Auto 与选择器功能测试；
148 个单元/功能 CTest 套件全部通过；消息布局及双向转换静态检查；消息仓库
逐文件同一性检查。测试覆盖局部/绝对资格、丢失/恢复、拒绝融合、重置、
实例切换、重新进入、显式 yaw、全部 MPC_YAW_MODE、摇杆辅助及 Weathervane。

完整 `make tests` 仍被既有 HX8 Linux `EAGAIN == EWOULDBLOCK` 判断触发的
`-Werror=logical-op` 阻断；未关闭告警、未删除测试。上述 148 项为单独构建
全部 unit/functional 目标后的结果，不冒充完整 make tests 成功。

仍须机载/实机验证：

1. 新 tag 执行 colcon 构建，确认实际订阅 `_v1`，时间戳与同步正常。
2. 安全条件下 FRD Offboard → AUTO_LAND：记录实际 yaw、trajectory yaw、
   yawspeed、两个资格位和 heading reset。固定目标应有限且不随外界扰动漂移。
3. 检查视觉中断/恢复、航向重置、重新进入降落和人工接管；定位失效与
   failsafe 仍须正常响应，不通过注入“有效”标志绕过安全逻辑。
4. 回归 NED 对准、手动/自动降落及着陆上锁。命令入口覆盖为源码检查与公共
   任务测试，尚未完成各入口的整机闭环飞行仿真或真实飞行。

本改动只能确认固定航向目标链路得到修复；实际偏航响应、角速度环性能和
触地冲击需后续日志与实机评估，不能由软件测试代替。

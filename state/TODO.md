# TODO

- [x] 确认需求和硬件输出分配。
- [x] 从 `testv3_v1.16.1` 创建独立 worktree。
- [x] 梳理原混合状态机与机型配置。
- [x] 设计并实现无变形安全切换。
- [x] 配置单向 PWM 左右轮输出。
- [x] 配置 MAIN5 云台及 RC/MAVLink 控制。
- [x] 完成脚本、参数生成、映射与静态安全核验。
- [x] 两次构建目标固件并审查差异。
- [x] 迁移至隔离目标 `hkust_nxt-dual_mini` 并排除其默认继承的旧变形模块。
- [x] 修复 mini 启动脚本未进入 NuttX ROMFS 的问题并重新构建。
- [x] 审计 `e1da143..325a9d0` 并选择性移植适用于 mini 的控制行为修复。
- [x] 加固 Rover -> Quad 的 MC 控制器状态清理与上游输入 epoch 门控。
- [x] 在隔离 Linux PATH 下运行 PX4 SITL 全量测试（146/146）。
- [x] 修正 rover 差速槽位到 MAIN7/MAIN8 的左右轮映射。
- [x] 审计目标 QGC 的 Rover 调参能力并编写第一阶段实时曲线/参数编辑交接指南。
- [x] 编写 QGC Agent 可直接执行的完整提示词并固定其功能分支、独立 worktree、测试、
  commit 和 push 要求。
- [x] 从 `mavlink master @ 5bfd76d80281f6027134e854aafe6cb3dbfbe9e1` 创建并发布
  mini Rover tuning 私有 dialect 分支。
- [x] 新增 Differential Rate、Attitude、Velocity、Position/Path 四层实时状态协议。
- [x] 实现四条按需 MAVLink stream、有效位、源时间戳和 mini target dialect 隔离。
- [x] 完成消息生成测试、PX4 tests 与 `make hkust_nxt-dual_mini` 构建验证。
- [x] 完成 mini 形态 epoch reset、Rate/Attitude/Velocity 缓存清理、Position source/nav-state
  epoch 和 vehicle/hybrid 双重形态门控复审。
- [x] 按原生PX4流程重写机载通信：外部定位、RC Position、Offboard、Mission/RTL与
  可选`MANUAL_CONTROL`分离，并限定mini Rover正式Offboard为二维位置Go-to。
- [x] 编写当前 uXRCE-DDS topics、QoS、时间/坐标和兼容性边界文档。
- [x] 为四桨瞬时归零增加mini输出原因锁存、arming安全边沿、QGC告警和判定等价单测。
- [ ] 由 QGC Agent 按 GUIDE 固定 `qgc_mini_rover` commit，完成四页实时曲线和参数编辑。
- [ ] 机载程序按 MAVLink 文档实现VIO `ODOMETRY`、RC Position协同、Offboard Go-to、
  命令ACK/最终状态确认和断线恢复。
- [ ] 无SD卡时设置`SYS_DM_BACKEND=1`，验证RAM dataman、Mission每次启动重传及任务完成后
  Rover显式退出Auto/停车/disarm流程。
- [ ] 修复并实车验证Rover Direct RTL在持续landed时直接IDLE的问题；完成前禁止把
  Rover RTL或Mission内RTL item配置成安全动作。
- [ ] 实测VIO的NED/FRD坐标、延迟、covariance、reset、quality门控及停止发布故障场景。
- [ ] 分别在 Quad/Rover 形态实测 `COM_RC_LOSS_T`、`COM_FAIL_ACT_T` 与所选
  `NAV_RCL_ACT`，确定产品失联时间线和动作。
- [ ] 若启用正式 DDS/ROS 2 集成，从最终固件消息定义发布并锁定配套 `px4_msgs` commit。
- [ ] 正式发布DDS前为含`is_quad_rover`的`VehicleStatus`递增`MESSAGE_VERSION`，实现
  upstream v1到新版本的translation并验证topic后缀与wire schema。
- [ ] 实机示波器确认 H 桥 PWM 频率、0%/100% 占空比与失效低电平。
- [ ] 刷入新固件后实机复测右摇杆右转、左摇杆左转及直行时两轮方向。
- [ ] 实机确认 airborne 拒绝 Quad -> Rover、landed 接受及两方向控制器接管。
- [ ] 实机验证 Rover -> Quad 前存在非零姿态目标/推力/积分时，旧输入不会重新驱动 MAIN1~4。
- [ ] 实机确认 RC channel 10 与 MAVLink gimbal v2 的控制权切换和 90 度机械范围。
- [ ] 刷入输出诊断固件，拆桨复现降速后立即执行`mini_vehicle_control status`，记录QGC
  `Mini output issue`/`Mini safety block`边沿消息并据锁存原因定位，不再人工盯高速listener。

# LOG

- 2026-07-30：用户确认 mini 机型不含变形机构；旧变形检测、等待状态机与高度门控应移除。
- 2026-07-30：轮驱动硬件只能接受单向占空比；Rover 负轮指令按需求钳制为零，因此倒车和原地反向轮转不可用，这是已知运动能力限制。
- 2026-07-30：只允许在独立 worktree 工作，不触碰其他 Agent 的 worktree。
- 2026-07-30：新增 `zeroone_x6_mini` target 和 airframe 22002；旧 hybrid target 未修改。
- 2026-07-30：`make zeroone_x6_mini` 完整构建成功（1181/1181），增量安全修订后再次
  构建成功（19/19）；最终 Flash 1,827,740 B / 1,920 KiB（92.96%）。
- 2026-07-30：生成配置确认 mini 编入 gimbal 与 mini_vehicle_control，未编入旧
  hybrid_vehicle_control、AS5600 或 TMAG5273。
- 2026-07-30：PWM 参数映射核验为 1000/1250/1500/1750/2000 ->
  0/250/500/750/1000 us；TIM12 在 1 kHz 下对应约 0/25/50/75/100% duty。
- 2026-07-30：尚未实机测试。1 kHz 是可改参数默认值而非已确认的 H 桥要求；首次
  上电前必须查数据手册并用示波器验证，不能把固件构建成功当作硬件验证成功。
- 2026-07-31：当前飞控改为 HKUST NXT-Dual，新增隔离目标
  `make hkust_nxt-dual_mini`。板级确认 MAIN1~4/TIM1、MAIN5~6/TIM2、
  MAIN7~8/TIM3，原三组输出协议可保持不变。
- 2026-07-31：NXT-Dual default 配置会被 variant 继承，首次构建因此夹带 AS5600
  和旧 `hybrid_vehicle_control`；mini 配置已显式设为 `n`。修正后构建成功
  （1126/1126），生成 builtin 表确认两者均不存在。
- 2026-07-31：最终 HKUST 固件 Flash 1,691,240 B / 1,792 KiB（92.17%），仍未
  进行实机 PWM、着陆切换或云台验收。
- 2026-08-02：现场 `SYS_AUTOSTART=22002` 正确，但 `rover_differential`、
  `rover_pos_control`、`mini_vehicle_control` 均未运行。根因是
  `rc.mini_hybrid_apps` 未加入公共 init.d 的 ROMFS CMake 清单，导致启动脚本引用
  了不存在的文件；已补入并重新构建。
- 2026-08-02：修复后 HKUST 构建完成（1127/1127），生成 ROMFS 明确包含
  `etc/init.d/rc.mini_hybrid_apps` 和 `mini_vehicle_control start`。
- 2026-08-02：实机确认 MAIN7 为左轮、MAIN8 为右轮；右摇杆前进时右轮加速且车身
  向左转。审查原生 `RoverDifferential` 接口注释和公式后确认其输出顺序为
  `control[0]=右轮`、`control[1]=左轮`。mini 仲裁层此前反向路由，已改为
  `control[0]->MAIN8`、`control[1]->MAIN7`；原生差速控制和负值归零逻辑未改动。
- 2026-08-02：现场刷入修复固件后确认 `rover_differential`、`rover_pos_control`、
  `mini_vehicle_control` 均为 running；`RO_YAW_P` 显示 `x +` 且值为 0.8500，
  参数 used 数由 721 增至 998，证明 Rover 参数已被加载并可供 QGC 请求。
- 2026-08-02：历史审计确认 `e1da143`（2026-06-04）到空窗后的首个提交
  `325a9d0`（2026-06-28）之间没有其他 commit、branch reflog 或 stash 快照；
  6 月 18 日附近的未提交工作区无法由 Git 复原。`325a9d0` 是唯一可证实的比较终点。
- 2026-08-02：选择性移植 `325a9d0` 的行为修复：Quad-Rover Rover 状态不再触发
  landed/preflight 自动解除锁定；MC attitude/rate 使用持久 vehicle status 正确停机；
  Position/Velocity 保持 attitude-to-rate 所有权；`RO_YAW_RATE_TH` 只过滤实测噪声，
  不再吞掉低角速度设定值。普通 Rover 的 Commander 行为未改变。
- 2026-08-02：拒绝移植 `ManualControl` 的 Rover 全手势屏蔽，因为它会同时禁用
  disarm 和用户显式启用的 kill；同时跳过 `RD_STAB_DBG`、`RD_ATT_DBG`、
  `RD_POS_DBG`、`RD_VEL_DBG` 及仅日志频率变化。
- 2026-08-02：发现 `325a9d0` 的 MC 早退会保留旧姿态、推力、rate 积分和 yaw
  滤波，最终 actuator 时间门无法识别这些旧状态重新生成的新消息。现已在 Rover
  清空状态，并以 Rover -> Quad 的 vehicle status timestamp 拒绝旧 attitude/rate、
  RC 和 autotune 输入；普通非混合机型不启用该门控。
- 2026-08-02：`vehicle_rates_setpoint_s` 在此分支没有 `timestamp_sample` 字段；首次
  增量构建据此失败，删除错误字段赋值后重建成功。最终 HKUST 固件 Flash
  1,692,312 B / 1,792 KiB（92.22%），仍未进行上述实机切换测试。
- 2026-08-02：使用仅含 Linux 工具的临时 `PATH` 执行 `make tests`，配置、编译和
  运行均成功；CTest 汇总为 146/146 通过，总耗时 49.61 s。该测试不替代 HKUST
  实机的 PWM、着陆门控和模式接管验收。
- 2026-08-02：最终固件 SHA-256：`hkust_nxt-dual_mini.px4`
  `7754a9e4768a9718af49a1a6c1c5b700ffac9dd67fe7c15da38e6f2660f863b2`；
  `hkust_nxt-dual_mini.bin`
  `d4b79a087ad91b7c5f842758392cb818438e2600c86486a5ef487c400fe1f70b`。
- 2026-08-02：左右轮槽位修正后重新执行 `make hkust_nxt-dual_mini`，21/21 个增量
  步骤成功；Flash 使用量仍为 1,692,312 B / 1,792 KiB（92.22%）。新
  `hkust_nxt-dual_mini.px4` SHA-256 为
  `000b7805c7a5520445e9311ac48b43fbf0f1f8d0c61d7f1589173913f74e3dd9`。
- 2026-08-02：映射修正后再次执行隔离 Linux PATH 的 `make tests`，CTest 汇总
  `100% tests passed, 0 tests failed out of 146`，总耗时 48.37 s。
- 2026-08-02：只读核验 `nanjia24/qgroundcontrol` 的
  `codex/joystick-aux-px4-development`，HEAD 为 `754135601a53d7650ddeb6562ca5a5cd2167880c`。
  该版本没有 PX4 Rover PID Tuning 页面；mini 以 `MAV_TYPE_VTOL_FIXEDROTOR` 连接，
  因此普通 Ground Rover 路由也不足以识别它。
- 2026-08-02：标准 `ATTITUDE_TARGET`/`POSITION_TARGET_LOCAL_NED` 不承载差速 Rover
  控制器实际使用的 setpoint。第一阶段正式路线确定为四个按页面请求的专用
  MAVLink2 tuning status 消息，参数编辑继续使用标准 PARAM/Fact；QGC 只读审计未
  产生仓库修改，实施指南见 `docs/mini_rover_realtime_tuning_guide.md`。
- 2026-08-02：USB 可支持建议的 Rate 50 Hz、Attitude 30 Hz、Velocity 25 Hz、
  Position 10 Hz 单页数据流；57,600 baud 数传明确不足，115,200 baud 叠加普通
  遥测后不具备严谨 Rate 调参余量。无 SD 可经 USB 使用 MAVLink2 ULog 应急，但
  默认 Rover logging topic 仅 10 Hz，不能替代实时内环页面。
- 2026-08-02：从精确 MAVLink 父提交 `5bfd76d80281f6027134e854aafe6cb3dbfbe9e1`
  发布 `mini-rover-tuning-v1.16.1`，实现 commit 为
  `07c6964a8fcc364c49d394f0bf0275b9fc05857d`。`mini_rover` 服务 PX4，
  `qgc_mini_rover` 组合 `all + mini` 服务 QGC；远端 branch HEAD 已核对一致。
- 2026-08-02：固定四条 MAVLink2 消息 60100--60103，payload/CRC 分别为
  27/147、23/85、43/217、44/90。新增永久测试覆盖 composite 生成、固定 wire
  常量及四消息 pack/decode，2/2 通过。
- 2026-08-02：Rate/Attitude/Velocity status 新增 producer 原子 `active`；所有 Rover
  producer 均零初始化并填写该字段。mini stream 还以 Rover vehicle type 门控，
  inactive/Quad 清 flags；output 只接受与 status 完全相同的周期时间戳。
- 2026-08-02：Differential response 保存原始采样时间；角速度、姿态或本地速度超过
  500 ms 未更新时写 NaN，速度还要求 `v_xy_valid`。Position 在 inactive 时不再发布
  有效坐标；Auto IDLE/invalid target 安全停止，模式/投影 epoch 清旧目标，Manual
  course latch 不跨模式保留，本地 Go-to 在 EKF XY reset 后按 `delta_xy` 调整。
- 2026-08-02：最终 `make hkust_nxt-dual_mini` 成功，Flash 1,696,536 B / 1,792 KiB
  （92.45%）；隔离 Linux PATH 的 `make tests` 为 146/146 通过，MAVLink dialect
  测试 2/2 通过。固件 SHA-256：`.px4`
  `ecd12832608953288fee467946b7b1d49aadc362fc3955495f1b004d6422e369`，`.bin`
  `56e18d041c65702ff580f8e10bca33e02cef789e6f2b3a7d8e57eed160035d15`。
- 2026-08-03：终审发现并修复形态切换缓存风险。`RoverDifferential`、Rate、Attitude、
  Velocity 清除 PID/slew、控制器缓存、传感器源时间戳和待处理订阅；父级在新鲜 steering
  到达前保持零 throttle fallback，避免 Rate/Attitude 直通路径因没有 throttle topic 而永久
  阻塞。Position 新增 Manual/Auto/Offboard/Go-to source epoch，按 `vehicle_status.nav_state`
  识别同层 Auto 切换，旧输入按时间戳丢弃并发布零速。
- 2026-08-03：mini Rover 入口和四条 tuning stream 同时要求 `vehicle_type==ROVER`、
  vehicle status 不超过 1 s、`HYBRID_STATE_DRIVING` 不超过 200 ms；修复 SITL 非 mini
  条件编译下的 `driving_shape` 未使用错误。最终父级 mode key/steering output epoch 也已
  加入，防止同一 Rover 形态内旧控制器输出重放。最终构建 Flash image 1,700,184 B /
  1,792 KiB（92.65%），`.px4` SHA-256 `fcea2c46817b3bfdf7e90cd832491a19127e2635c92d1aa4a02ab2f743a3e364`，
  `.bin` SHA-256 `d7a658319675b5b983e9229c44c102028ef1dc3916c47ee52972a863517ffba4`；
  最终 PX4 CTest 146/146、MAVLink dialect 2/2、git diff --check 和新增文件 AStyle 检查通过。
  QGC、USB、PWM、模式切换和实车验收仍未完成。
- 2026-08-03：收尾时再次在当前源码快照执行隔离 Linux PATH 的 `make tests`，
  CTest 为 146/146 通过（48.39 s）；MAVLink dialect 为 2/2 通过，目标固件增量构建
  成功。父仓库 gitlink 精确从 `5bfd76d80281f6027134e854aafe6cb3dbfbe9e1` 更新到
  干净的 `07c6964a8fcc364c49d394f0bf0275b9fc05857d`，ELF 包含四个 tuning stream；
  固件尺寸与 SHA-256 均未变化。相关 C/C++ AStyle、全部改动文件尾随空白检查和
  `git diff --check` 通过，且没有遗留构建/测试进程。
- 2026-08-03：为 QGC Agent 新增完整执行提示词，固定功能分支
  `codex/mini-rover-realtime-tuning` 和工作树
  `E:\workspace\QGC\qgroundcontrol-worktrees\mini-rover-realtime-tuning`，覆盖精确基线、
  worktree 隔离、MAVLink pin、功能边界、实现结构、自动测试、USB 验收、结构化提交和
  push。远端再次核验 QGC baseline 仍为 `754135601a...`，MAVLink branch 仍为
  `07c6964a...` 且直接父提交为 `5bfd76d...`；本步骤未修改任何 QGC worktree。
- 2026-08-03：机载电脑车辆控制最终选用 MAVLink2 归一化 `MANUAL_CONTROL`。确认其
  `x/y/z/r` 分别进入 PX4 `pitch/roll/throttle/yaw`，Differential Rover 只使用
  `pitch` 前进和 `roll` 转向；建议 20--50 Hz，输入超时由 `COM_RC_LOSS_T` 和
  `NAV_RCL_ACT` 共同处理。`COM_RC_IN_MODE=1` 不允许物理 RC channel 10 同时控制
  AUX1，需要 RC 备用时只能用模式2做整套输入源失效接管。
- 2026-08-03：纠正旧 QGC 文档的 MAV_TYPE 枚举名。airframe 设置数值22，在固定
  MAVLink commit `07c6964a...` 中它是 `MAV_TYPE_VTOL_FIXEDROTOR`；
  `MAV_TYPE_VTOL_RESERVED5` 的实际值是25。QGC routing 仍需同时检查
  `HYBR_QUAD_ROV=1`。
- 2026-08-03：DDS审计确认硬件目标编译 `uxrce_dds_client` 但默认
  `UXRCE_DDS_CFG=0`。当前实际形态topic是版本化的
  `/fmu/out/vehicle_status_v1`，其消息含自定义 `is_quad_rover`；查询
  `https://github.com/QQgdiw/px4_msgs.git` 返回 repository not found，因此正式DDS
  集成仍缺一个从最终固件定义生成并固定的 `px4_msgs` artifact。当前DDS也没有
  HybridVehicleStatus、RoverVelocitySetpoint、四条Rover tuning或高频gimbal输入。
- 2026-08-03：新增 `docs/mini_vehicle_mavlink_communication.md` 和
  `docs/mini_vehicle_dds_communication.md`，分别作为机载电脑的正式车辆控制契约和
  当前DDS能力/缺口说明。
- 2026-08-03：固件源码以 commit
  `2f5d1f003b3106060e70df012de59bfc3404837c` 固定。该提交重新执行
  `make hkust_nxt-dual_mini` 成功，Flash 1,699,944 B / 1,792 KiB（92.64%）；
  `.px4` SHA-256 `c8fb8aaeebe40846292c2fddd21d919ce5a7f736d3e83347c0b31794e53c9bd0`，
  `.bin` SHA-256 `b06be017cfbdc706493e294f3759ebf5368e9bf944b79794cb88e6d1e23c6a97`。
  隔离Linux PATH的PX4 CTest为146/146（52.46 s），MAVLink dialect为2/2；
  实机验收仍待完成。
- 2026-08-03：通信文档静态终审完成：`dds_topics.yaml`中的全部启用topic均在DDS文档
  覆盖，pymavlink示例通过Python语法编译，固定MAVLink枚举/消息常量与源码一致，
  `git diff --check`通过。上述检查不包含ROS 2 Agent、USB、RC接管、云台或整车实测。
- 2026-08-03：首次创建并推送远端分支`origin/change_mini_v1.16.1`，upstream tracking
  配置成功；已推送固件源码提交`2f5d1f003b`和通信文档提交`c8fa199435`。

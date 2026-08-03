# QGC Mini Rover 实时调参开发 Agent 提示词

以下内容应与 `mini_rover_realtime_tuning_guide.md` 一起原样交付给负责
QGroundControl 的开发 Agent。

---

你是本任务的 QGroundControl 开发 Agent。你的目标是在指定 QGC 精确基线上，完整
实现 Mini Quad/Rover 的 Differential Rover 第一阶段实时调参界面，并完成自动测试、
构建、提交和推送。不要只输出方案或停留在页面骨架；应持续工作到代码、测试和交付
记录全部完成，或者遇到无法在当前权限和环境内解决的真实阻塞。

## 1. 必读材料与权限边界

开始任何修改前，必须完整阅读随本提示词交付的
`mini_rover_realtime_tuning_guide.md`。该 GUIDE 是功能、协议、数据语义和验收标准的
详细契约，本提示词补充仓库初始化、执行纪律和最终交付要求。

进入 QGC 仓库后，还必须从仓库根目录开始查找并完整阅读所有适用于修改路径的
`AGENTS.md` 或同类本地指令。若 QGC 仓库的局部指令与本提示词在代码风格、构建命令
或测试入口上冲突，以更局部的仓库指令为准；以下精确 Git/MAVLink 基线、协议常量和
功能边界不得自行改变。

你的写入范围仅限本任务新建的 QGC worktree。禁止修改：

- 其他 Agent 的任何 worktree；
- `/home/crocodile/PX4-Autopilot-change-mini` 中的 PX4 源码；
- `QQgdiw/mavlink` 仓库或已发布的协议 commit；
- QGC 主仓库中不属于本任务 worktree 的工作区、分支或未提交改动。

如果发现 PX4 producer 或固定 MAVLink XML 存在问题，应给出文件、字段、消息样本和
可复现证据，然后暂停相关协议扩展并向任务所有者报告。不得在 QGC 中伪造数据、改写
消息含义或用标准 Multirotor target message 掩盖上游问题。

PX4 固件源码协议锚点固定为
`2f5d1f003b3106060e70df012de59bfc3404837c`（分支
`change_mini_v1.16.1`、目标 `hkust_nxt-dual_mini`）。不要用可移动的 PX4 branch HEAD
代替该兼容性锚点。

## 2. 固定仓库、分支与 worktree

QGC 开发基线必须精确固定为：

- repository：`https://github.com/nanjia24/qgroundcontrol.git`
- baseline branch：`codex/joystick-aux-px4-development`
- baseline commit：`754135601a53d7650ddeb6562ca5a5cd2167880c`
- feature branch：`codex/mini-rover-realtime-tuning`
- worktree：`E:\workspace\QGC\qgroundcontrol-worktrees\mini-rover-realtime-tuning`

不得从当前 `main`、移动后的远端分支 HEAD、其他 QGC 功能分支或已有 Agent worktree
创建本分支。即使远端 baseline branch 将来前进，也必须从上述 40 位 commit 创建。

先在 `E:\workspace\QGC` 下找到已经管理该仓库 worktree、且 remote 指向上述 URL 的
主 clone。若不存在，可将新的控制仓库 clone 到
`E:\workspace\QGC\qgroundcontrol-source`，但最终开发仍必须位于指定 worktree。
不要覆盖、删除或复用一个来源不明的同名目录或分支。

仅当该控制仓库确实不存在时，使用：

```powershell
git clone --no-checkout https://github.com/nanjia24/qgroundcontrol.git `
  "E:\workspace\QGC\qgroundcontrol-source"
```

下面是 PowerShell 初始化流程示例，其中 `$OwnerRepo` 应替换为核验过的主 clone：

```powershell
$OwnerRepo = "E:\workspace\QGC\qgroundcontrol-source"
$Worktree = "E:\workspace\QGC\qgroundcontrol-worktrees\mini-rover-realtime-tuning"
$Base = "754135601a53d7650ddeb6562ca5a5cd2167880c"
$Branch = "codex/mini-rover-realtime-tuning"

git -C $OwnerRepo remote -v
git -C $OwnerRepo fetch origin codex/joystick-aux-px4-development
git -C $OwnerRepo cat-file -e "$Base^{commit}"
git -C $OwnerRepo worktree list --porcelain
git -C $OwnerRepo worktree add -b $Branch $Worktree $Base

git -C $Worktree rev-parse HEAD
git -C $Worktree status --short --branch
git -C $Worktree merge-base --is-ancestor $Base HEAD
```

创建后 `HEAD` 必须等于 `$Base` 且工作区必须干净。若 feature branch、目标目录或远端
同名分支已经存在，不得删除、reset、force push 或擅自接管；先核验其所有者和提交父链，
有冲突时向任务所有者报告。

若网络确实需要代理，可只为当前终端临时设置 `HTTP_PROXY`/`HTTPS_PROXY` 为
`http://127.0.0.1:7897`。禁止为此修改全局 Git 配置。

## 3. 固定 MAVLink 依赖

QGC 必须使用以下已经发布并核验的 MAVLink 依赖：

- repository：`https://github.com/QQgdiw/mavlink.git`
- branch reference：`mini-rover-tuning-v1.16.1`
- exact commit：`07c6964a8fcc364c49d394f0bf0275b9fc05857d`
- direct parent：`5bfd76d80281f6027134e854aafe6cb3dbfbe9e1`
- QGC dialect：`qgc_mini_rover`

必须把 QGC 的依赖固定到 40 位 commit，而不是可移动 branch。QGC 不能使用精简的
`mini_rover` dialect；`qgc_mini_rover` 才是保留 QGC 原 `all.xml` 消息集合并加入 mini
消息的 composite dialect。不得使用 `qgc-hybrid-*` tag，也不得继续使用 stock
`mavlink/mavlink` pin。

按照 QGC 已有 CMake 依赖机制修改 repository、commit 和 dialect。生成文件必须通过
仓库现有生成流程产生，不得手工编辑生成的 MAVLink header。修改 pin 后必须按 QGC
现有构建方式重新配置或清理对应的 CMake dependency cache，并用生成日志/路径证明
实际使用的是 `07c6964a...`，不能复用旧 `b1fb5a1a...` 的下载或生成结果。构建或测试
必须验证：

- `ROVER_RATE_TUNING_STATUS`：ID 60100、payload 27、CRC extra 147；
- `ROVER_ATTITUDE_TUNING_STATUS`：ID 60101、payload 23、CRC extra 85；
- `ROVER_VELOCITY_TUNING_STATUS`：ID 60102、payload 43、CRC extra 217；
- `ROVER_POSITION_TUNING_STATUS`：ID 60103、payload 44、CRC extra 90。

这四个消息只支持 MAVLink2。不得为 MAVLink1 添加降级解释或替代 debug message。

需要 USB 联调时，已验证 PX4 参考固件位于
`/home/crocodile/PX4-Autopilot-change-mini/build/hkust_nxt-dual_mini/`
（Windows 可通过本机 WSL 文件共享访问）。应使用 GUIDE 中给出的 `.px4` SHA-256
核验刷写文件；无法访问该文件或没有实机时，不影响继续完成 QGC 软件实现和自动测试。

## 4. 实现范围

只实现 GUIDE 定义的第一阶段：

1. Rate 实时 Response/Setpoint 曲线和对应参数编辑；
2. Attitude 实时 Response/Setpoint 曲线和对应参数编辑；
3. Velocity 实时 Response/Setpoint 曲线和对应参数编辑；
4. Position/Path 实时 Response/Target 曲线、Cross-track 诊断和对应参数编辑；
5. USB/MAVLink2 链路上的 stream 请求、切页停止、断链恢复和数据失效处理。

明确不做：自动调参、参数推荐、日志分析器、飞行/行驶模式切换、普通 stock Rover
支持、Ackermann/Mecanum producer、低带宽数传优化，以及 PX4/MAVLink 协议改动。

页面只对 `MAV_TYPE_VTOL_FIXEDROTOR`（值 22）且参数 `HYBR_QUAD_ROV==1` 的 mini
机型启用。参数尚未下载完成时不能永久缓存为“不支持”；参数到达后必须刷新能力判断。
保留原 VTOL/Multirotor PID Tuning 页面，并增加并列的 Rover 入口。其他 VTOL、普通
Multirotor 和普通 Ground Rover 的现有行为不得改变。这里的页面能力由机型和参数决定，
不是由当前 Quad/Rover 形态决定：确认是 mini 后，即使当前处于 Quad 形态，Rover 入口
也必须保留并显示 inactive/无实时数据，不能在形态切换时销毁页面。基线分支已有的
joystick/AUX 功能也必须保持不变。

## 5. 必须遵守的实现结构

以 GUIDE 第 4 至第 7 节为完整字段和页面契约，并满足以下不可删减要求：

- 每个 `Vehicle` 独立拥有 `RoverTuningFactGroup`；不得把有状态实例放到多 Vehicle
  共享的 `PX4FirmwarePlugin` 单例。
- 在 C++ FactGroup 中完成消息解码、有限值检查、`valid_flags`、`drive_type`、源时间戳、
  500 ms 超时、时间戳回退/重启清理、rad 到 deg 转换和 yaw unwrap。QML 只负责显示
  和交互，不复制协议状态机。
- 只有 `ROVER_DRIVE_TYPE_DIFFERENTIAL` 的活动帧可追加 Differential 曲线。UNKNOWN
  终止帧用于正常清除状态；Ackermann 和 Mecanum 必须显示当前不支持，不能误套
  Differential 参数和曲线。
- 活动帧严格按 PX4 `time_usec` 追加一次曲线样本。不能让 QML 10 ms Timer 把 25 Hz
  或 50 Hz 的同一消息重复绘制成 100 Hz。
- `flags=0` 的 inactive 帧是必须消费的状态清除事件，不能作为坏包静默丢弃。
  `drive_type=UNKNOWN && flags=0` 通常表示正常 Quad/inactive；应立即清空曲线有效状态，
  但不显示为协议错误。Ackermann/Mecanum 表示当前第一阶段不支持。只有 flags 宣称
  某字段 valid 但数值非有限、UNKNOWN 却携带活动有效位等自相矛盾样本，才拒绝对应
  字段并记录诊断。
- 终止帧、inactive、缺失字段有效位、超过 500 ms 无新源时间戳、时间戳倒退、飞控
  重启或断链时，立即停止追加并清除/失效对应数据。不得保持最后一个值伪装为实时数据。
- yaw response 连续解包裹；yaw setpoint 选择与 response 最近的 `2*pi` 等价值。失效、
  重连或时间戳回退时重置 unwrap 状态。
- Position 的 `xy_reset_counter` 变化时清空 Position 图；target/path 无效时不得继续
  绘制旧路径。
- 扩展现有 `PIDTuningTelemetryMode`/`MAVLinkStreamConfig`，页面一次只请求一条消息：
  Rate 50 Hz、Attitude 30 Hz、Velocity 25 Hz、Position 10 Hz。切页时先恢复上一条消息
  默认速率，退出、断链或销毁 Vehicle 时也必须恢复。复用现有 ACK/中断处理。
- MAVLink1 链路不得反复发送失败请求，应明确显示该功能需要 MAVLink2。
- 参数编辑继续使用 QGC 标准 `FactPanelController`、PARAM_SET 和飞控回读；min/max/
  increment 来自参数 metadata。禁止建立私有参数协议或仅更新本地 UI。
- 复用现有 PID Tuning 图表和 QGC 视觉规范，不做无关页面重构。新增接口必须向后兼容
  原 Multirotor Timer 采样路径。

## 6. 实施顺序

按以下顺序推进并在每一步保持可构建：

1. 核验精确 QGC HEAD、工作区和适用的本地开发指令；初始化仓库要求的 submodule。
2. 按 QGC 官方/仓库文档核验 Windows Qt、CMake、编译器和测试环境；记录版本。能运行
   时先做基线构建或最小相关测试，区分基线问题和本次回归。
3. 更新 MAVLink repository/commit/dialect，并先验证四个消息可生成和编译。
4. 实现每 Vehicle FactGroup、字段 metadata、有效性和超时状态机及单元测试。
5. 实现四种独立 stream mode、ACK/中断/恢复生命周期及测试。
6. 实现 mini 能力路由、Rover 总页、四个子页和标准参数编辑。
7. 完成普通 VTOL/Multirotor 回归、全目标构建、自动测试和可执行 UI 检查。
8. 有 mini 飞控和 USB 条件时执行 GUIDE 的 USB 实机验收；没有硬件时继续完成所有
   可执行的软件验证，并在最终报告中逐项标明哪些实机验收未执行。
9. 审查 diff，只保留本任务修改，按结构化消息 commit 并 push feature branch。

不得把模拟数据、静态 QML 占位或仅能编译的空 Fact 当作功能完成。不得因为缺少实机
而跳过可以在本地完成的 decoder、状态机、路由和 stream lifecycle 测试。

## 7. 最低测试与验收要求

除仓库本地指令要求的测试外，至少完成并报告：

- MAVLink dependency 生成成功，四个 ID/LEN/CRC 固定断言通过；
- 四类消息 pack/decode 和字段映射测试；
- 每 Vehicle 隔离与非当前 Vehicle 消息拒绝测试；
- Differential/UNKNOWN/非支持 drive type 测试；
- rad/deg、yaw wrap、NaN、各有效位、500 ms timeout、源时间戳重复/倒退和重启测试；
- Position `xy_reset_counter` 清图、target/path invalid 测试；
- 四种 `MAV_CMD_SET_MESSAGE_INTERVAL` 请求频率、ACK、切页中断、退出恢复默认、断链
  恢复和 MAVLink1 拒绝测试；
- mini 参数异步到达后的页面路由测试，以及普通 VTOL/Multirotor 不回归；
- 参数 Fact 查找、写入、飞控回读/拒绝处理测试；
- Windows 上 QGC 目标完整构建成功，并运行所有受影响的现有测试。

UI 检查至少覆盖 Rate、Attitude、Velocity、Position/Path 四页，确认文本不截断、曲线
不重叠、切页不残留上一 stream，且原 Multirotor 页面保持可用。可以生成截图作为证据，
但截图不能替代数据和状态机测试。

如果某项测试因缺少 USB 飞控、Qt 插件、签名证书或其他外部条件无法运行，必须报告
准确命令、错误、已完成的替代验证和残余风险。严格区分“代码已编写”“构建通过”
“自动测试通过”和“USB 实机通过”。

## 8. Commit 与 push

完成前执行 `git status --short`、`git diff --check`、完整 diff 审查和仓库要求的格式
检查。禁止提交 build 目录、下载缓存、IDE 配置、临时日志、凭据或与任务无关的格式化。
遵循仓库 `.gitattributes` 和现有文件行尾，禁止 Windows CRLF 转换造成全文件噪声 diff。

可以按依赖、后端/测试、UI 三个逻辑阶段拆分提交；提交消息遵循仓库局部规范。若仓库
没有更具体要求，使用例如：

```text
feat[px4-rover]: add mini rover realtime tuning
```

没有已知 GitHub Issue 编号时不得捏造 `Fixes #...`。不得 amend、reset 或 force push
他人的提交。最终推送：

```powershell
git push -u origin codex/mini-rover-realtime-tuning
```

如果 `origin` 不是具有推送权限的 `nanjia24/qgroundcontrol` remote，应先明确报告并
使用仓库已有、经核验的正确 push remote；不要静默改写其他 Agent 的 remote 配置。

## 9. 完成定义与最终报告

只有以下条件全部满足，才能把软件开发标记为完成：

- 工作基线、MAVLink commit 和 dialect 精确匹配本提示词；
- 四页实时曲线和参数编辑均为真实实现；
- stream 生命周期、失效、重连、多 Vehicle 和原页面回归有自动测试；
- QGC 目标构建成功；
- 本任务修改已按规范 commit 并成功 push；
- 工作树最终干净；
- 未完成的 USB/硬件验收被明确列为未验证，而不是隐含为通过。

最终报告必须包含：

1. worktree、feature branch、基线 commit、最终 commit 列表和远端分支；
2. MAVLink repository、40 位 commit、dialect 和四消息常量核验结果；
3. 主要架构和用户可见行为摘要；
4. 实际运行的构建/测试命令及通过、失败、跳过数量；
5. 最终 QGC 可执行构建产物的路径、大小和 SHA-256；
6. 四页 UI 检查或截图结果；
7. USB/实机是否执行，未执行项和原因；
8. `git status --short` 结果与已知残余风险。

如果阻塞，最终报告还必须给出最小可复现证据和继续工作所需的具体外部条件，不能只写
“环境问题”或“待测试”。

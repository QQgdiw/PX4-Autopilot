# Test Workspace State

- Hardware isolation proved that HX8 is healthy at a centered-switch cold boot
  and that either first landing-gear move triggers power protection. The root
  cause is now identified in code: command `0x0e` encodes its interval as 16
  bits instead of the vendor-defined 32 bits, shifting the remaining fields.
  The correction is built; target-hardware retest is still pending. Corrected
  artifact: `build/zeroone_x6_hybrid/zeroone_x6_hybrid.px4`, 1,773,676 bytes,
  SHA-256
  `21571592d52922060a727335e74f0a754825e94b210e9cdbcda286fc58d25e87`.
- With `LG_AUTO_EN=0`, landing-gear angle is operator-owned and does not gate
  transformation stages or Ready. HX8 online/config/protection health remains
  mandatory, Quad-to-Rover still requires landing plus disarm, Rover-to-Quad
  still requires disarm, and concurrent HX8/HX-65HM motion is permitted.
- Pre-`0x0e`-fix manual-gear firmware artifact:
  `build/zeroone_x6_hybrid/zeroone_x6_hybrid.px4`, 1,773,656 bytes, SHA-256
  `7c99c8242e24d2acd7001404b26aaaf5ac369e792d2ed85bc574d4c5a9ffbdea`.

## 2026-09-02 HX8 landing gear and dual HX-65HM transformation worktree

- Base: `origin/testc3_v1.16.1` at
  `285a9d5716e6f2935545532350645044a52ad11b`.
- Branch: `feat/hybrid-landing-gear-servos`.
- Worktree: `/home/crocodile/PX4-Autopilot-hx8-hx65hm`.
- Scope: one FashionStar HX8-U45H-M multi-turn landing-gear actuator and two
  Hiwonder HX-65HM transformation actuators on one half-duplex UART bus.
- Shared-bus UART rate: `HX_BAUD` in the `Hybrid Control` group, default
  1,000,000 baud; the setting applies to the flight-controller UART for all
  three servos.
- Concise field guide: `docs/hybrid/hx-shared-bus-commissioning.zh-CN.md` covers
  offline ID/rate setup, PX4 parameters, seven endpoint measurements, status
  commands, no-propeller acceptance, and fault injection.
- Current phase: implementation and host verification complete; target-hardware
  commissioning and no-propeller acceptance remain pending.
- Required build target: `make zeroone_x6_hybrid` only.
- Initial integrated artifact: `build/zeroone_x6_hybrid/zeroone_x6_hybrid.px4`,
  1,773,612 bytes, SHA-256
  `ae2adcd53983d87dd927809ea89a5cf01c16a088d9424eea301fc60bbba1b6e4`.
- Linker FLASH usage: 1,890,024 bytes of 1,920 KiB (96.13%).
- The HX8 boot verifier deliberately does not read parameter 36. The shared
  UART rate is applied from `HX_BAUD`; a servo-side baud mismatch is detected
  by absence of replies, not by comparing protocol-specific baud codes.
- An HX-65HM response timeout only invalidates online/healthy after the initial
  request plus both retries fail. A retryable miss keeps the last verified
  state; independent 500 ms feedback freshness still guards stale devices.
- The single mixed-protocol UART enforces 5 ms of bus quiet time after a valid
  response and after response-less HX-65HM broadcasts before either protocol
  may transmit another request.
- `hx8_uart_servo status` reports per-side HX-65HM timeout/retry counters using
  atomic snapshots, allowing field diagnosis without changing the uORB ABI.
- A malformed or wrong-ID HX-65HM frame increments the protocol-error counter
  and resets parser framing but no longer aborts the expected transaction;
  valid following bytes remain eligible until the normal response timeout.
- `hx8_uart_servo status` stores and prints only the first HX-65HM RX parser
  error, bounded to 16 wire-order bytes plus result/expected-ID/request-kind;
  it does not stream or repeatedly log bus traffic.
- The same bounded snapshot records a first ordinary response timeout as
  diagnostic result 5, preserving an all-header or truncated RX stream after
  repeated-header recovery.
- The HX-65HM stream parser treats an arbitrary run of `0xff` bytes as a
  repeated header preamble and retains the last two header bytes. Since valid
  response IDs are 0..253, this is unambiguous and preserves all other checks.
- During mixed-bus startup, HX-65HM discovery and configuration verification
  now run as an exclusive phase before any HX8 transaction. Normal interleaved
  scheduling resumes after both HX-65HM devices either verify or exhaust the
  boot attempts. This isolates startup protocol interaction without masking a
  genuine HX-65HM failure.
- The diagnostic firmware captures the HX-65HM boot transaction stream before
  the first UART read: exact TX and RX bytes, relative microsecond timestamps,
  parser results, and timeouts. It freezes after success, failure, or 512 data
  bytes and prints only on `hx8_uart_servo trace`; no live logging occurs in the
  timing-sensitive receive path.
- HX-65HM boot verification no longer sends Ping because both physical devices
  respond to a targeted Ping on a multidrop bus. It starts with a targeted
  identity-register Read. After a successful boot, the bounded trace repeatedly
  rearms around each Monitor Read and freezes only on the first monitor timeout.
- The Monitor diagnostic now uses a 16-entry circular pre-trigger buffer with
  at most 32 raw bytes per entry (512 bytes total). It continuously records
  HX8/HX-65HM TX and RX plus timeout/parser markers after boot, then freezes in
  chronological order on the first HX-65HM Monitor timeout.
- After any HX8 TX or RX, H65 transmission is held until a fixed 40 ms parser-
  recovery deadline. If H65 work is pending when the deadline expires, it is
  serviced before ordinary HX8 work; an HX8 emergency release can still
  preempt and restart the recovery interval.
- Hardware acceptance has not been performed.

This directory records the status of testing for `debug/testc1-v1.16.1`.

- Base: `origin/testc1_v1.16.1` at `d86bdd6a4a957704a7c4218a628d6007e2a4e1f9`
- Scope: build and debug the `zeroone_x6_hybrid` firmware only.
- Worktree: `/home/crocodile/PX4-Autopilot-debug-testc1-v1.16.1`
- USB diagnosis: the post-initialization FDCAN filter configuration path was the
  reproducible USB CDC blocker. M2006 now retains the driver's initial
  accept-to-FIFO0 policy and software-filters C610 IDs instead of re-entering
  FDCAN INIT at runtime.
- Implemented topology: PMU/DroneCAN owns physical CAN1; M2006/C610 owns physical
  CAN2 (FDCAN2, PB12 RX and PB13 TX). The H7 driver now maps each instance's
  physical mask to local logical interfaces and tracks ownership per physical
  bus. M2006 continues to use logical interface zero, which maps to FDCAN2 for a
  CAN2-only helper.
- Final host verification artifact:
  `build/zeroone_x6_hybrid/zeroone_x6_hybrid.px4`, 1,751,883 bytes, generated
  2026-07-23 21:24:26 +0800. SHA-256:
  `4e6e7788f02d180a7a3ab76df2862fb9be1170cbdf4a58e7f8bc65498bdc0369`.
- Target hardware acceptance for USB CDC, PMU DroneCAN on CAN1, simultaneous
  M2006 IDs 1/2 on CAN2, and reversed startup order has not yet been performed.
- First target report: USB is stable and M2006 CAN2 is healthy (`rx=395776`,
  `tx=196905`, `hw errors=0`, both motors online, `can_error_count=0`), but
  `uavcan status` reports `application not running`; PMU communication/LED is
  therefore not yet validated.
- DroneCAN-only isolation also fails with M2006 disabled across a full power
  cycle, so CAN2-first ownership is excluded as a necessary cause. A temporary
  staged startup diagnostic is available at
  `build/zeroone_x6_hybrid/zeroone_x6_hybrid_dronecan_start_diag.px4`, SHA-256
  `c6a500730d1cb9c2e1d538d25de999c55380e5b4bfd2031f26cba4080acf08fd`.
- Root cause: commit `b299183a29` initialized the STM32H7 driver's logical
  interface count to zero until asynchronous hardware init, but libuavcan
  requires at least one interface while constructing `UavcanNode`. Commits
  `7f1a479735` and `7014e3ce55` establish a tested pre-init `CanDriverView`
  used directly by production for logical count and CAN1/CAN2 mapping.
- Final production artifact:
  `build/zeroone_x6_hybrid/zeroone_x6_hybrid_dronecan_topology_fix.px4`,
  1,752,452 bytes, generated 2026-07-24 16:15:16 +0800. SHA-256:
  `5e34d479ce59f99f86b47e849fdfe3fc43323664ca5b0afd4a8e341f6c992dee`.
  Host verification passed `CanInterfaceMap`, `CanOwnership`, and
  `hybridCheck` 1/1 each plus `make zeroone_x6_hybrid`. Target acceptance is
  complete: DroneCAN/PMU runs on CAN1, both M2006/C610 nodes run on CAN2,
  USB/QGC remains stable, and a disarmed simultaneous C610 disconnect/reconnect
  recovered both nodes without a latched fault.
- HX8 commissioning inputs supplied so far: servo ID 0, provisional Quad/Rover
  endpoints 90/180 degrees, regulated 12 V supply, and 36 W allowed run power.
  Persistent protection fields use mW/mA/mV and a Celsius temperature limit;
  live temperature telemetry separately uses a raw thermistor ADC conversion.
- User-selected HX8 protection candidates were stall power 48000 mW, current
  limit 5000 mA, and 65 C temperature threshold. BEC output is
  confirmed sufficient. Persistent write remains blocked pending resolution of
  the 48 W stall threshold versus 36 W internal/run power-limit relationship.
- User subsequently chose to retain all HX8 manufacturer-default internal
  protection settings. Their numeric values are not represented by PX4's zero
  placeholders and must be read from the vendor tool/manual or protocol before
  PX4 expected-value parameters can be populated.
- Manufacturer defaults were supplied: response 0, ID 0, baud code 5
  (115200), stall protection 0, stall power 6000 mW, voltage 4000--12600 mV,
  temperature 70 C, power 20000 mW, current 4000 mA, power hysteresis 0,
  power-on lock 0, angle limit enabled at +/-180.0 degrees, soft start
  disabled, soft-start time 3000 ms, and midpoint offset 0.
- These defaults cannot be accepted unchanged: response and stall protection
  must be enabled; 4 V undervoltage conflicts with the intended 9.0--12.6 V
  safety envelope; and a 20 W persistent limit requires runtime motion power
  no greater than 20 W. Register 41 is documented in degrees C, exposing a
  now-fixed driver unit bug where expected configuration used raw ADC.
- Approved safe commissioning profile: response/stall protection enabled,
  stall power 6000 mW, temperature 70 C, internal and runtime power 20000 mW,
  current 4000 mA, voltage 9000--12600 mV, and power-on lock disabled.
- Implementation now uses `HX8_CFG_TEMP` in degrees C for register 41 while
  retaining ADC conversion only for live telemetry. HX8 config validation no
  longer consumes PWM endpoints; absent-driver CLI returns failure; unsafe
  response/stall/voltage settings and motion power above the persistent limit
  are rejected.
- Host tests pass all four `Hx8` targets and `TransformationStateMachine`.
  `make zeroone_x6_hybrid` passes. Final local artifact SHA-256:
  `0e24dd9d851c8f3651c1b8c7310a26cb9ef4a48dae62ce6cac009a953164244b`.
  Hardware validation remains pending.
- Fix committed locally as `ece4655271d71b7bc8f720685c0e2a460dfd1ae8`
  (`fix[hx8]: enforce safe servo commissioning`); branch is one commit ahead
  of `origin/testc1_v1.16.1` and has not been pushed.
- HX8 bench diagnosis: vendor-tool communication and all configured protection
  values are valid, but PX4 has zero valid RX frames. The original ZeroOne
  public PX4 repository and X6 V2 branch use the same serial mapping as this
  worktree. A local diagnostic build now reports the opened device and TX
  attempt/error counts from `hx8_uart_servo status`.
- Diagnostic commit: `9cc27c286073b614075332199a7e39daaa9e1734`
  (`test[hx8]: expose UART transmission diagnostics`). Current firmware SHA-256:
  `c57d58d7277c63b54f0e0e51fde2e81696bb78a578b6f33bacc8050c995ff1cc`.
- Root cause confirmed by target diagnostic: opening the UART in the NSH task
  left an invalid FD in the serial work queue (`EBADF=-9` on every send). Commit
  `5b35c8704e7894bc7259cfdbbaf7e729c8b50203` opens/configures it in `Run()`.
  Current firmware SHA-256 is
  `a18663a0d4e9e1cfdc85592432d37bde460a0406f6c1d38054f7e19088f9b148`.
- Hardware then confirmed clean Ping TX/RX. Vendor ParamRead responses echo the
  register number before its value (`05 1c 03 03 00 21 01 49` for register 33
  value 1), which the old controller rejected. Commit `8355722b3a` fixes the
  response shape and extraction. Current firmware SHA-256:
  `37f2b333539489d62ac7ddd2bab0df1999b80c503891e87a51f957c9c7cfa543`.
- Full register capture proved register 41 stores raw ADC 741 for the vendor
  UI's 70 C threshold. Commit `1c954ca375` keeps `HX8_CFG_TEMP=70` user-facing
  but converts it to ADC for protocol compare/write. Current firmware SHA-256:
  `52b14c39582a84dd1d0a81d6fcc3c7df01ffa184596f89dbb7f2f99bf837c963`.
- Current RC/HX8 diagnosis found two independent defects. The hybrid module
  interpreted `RC_MAP_TRANS_SW=7` as `manual_control_setpoint.aux3`, although
  AUX3 is controlled by `RC_MAP_AUX3`; it now consumes the canonical
  `manual_control_switches.transition_switch` output from `rc_update`.
- Stable HX8 HOLD previously issued a new timed-move command every 200 ms. An
  armed servo could reject the next command while the preceding 1000 ms move
  was still active. HOLD is now sent once when motion becomes enabled or the
  stable endpoint changes, and is re-armed only after motion is disabled.
- Host verification passes HX8 Protocol 10/10, Controller 23/23, CLI 1/1,
  BackendPolicy 7/7, TransformationStateMachine 44/44, and ManualControl 7/7.
  `make zeroone_x6_hybrid` passes. Current uncommitted artifact SHA-256:
  `d2a3a052934af0dfa7696540af983128498393fdf03421cac04456a6af37a6ba`.
- The latest cached-status MC spool-up correction was rebuilt as
  `build/zeroone_x6_hybrid/zeroone_x6_hybrid.px4`, SHA-256
  `c97433197a51f651bc778b8447600c071fa4feb795991366814a1857fa3fc471`; it
  remains unverified on hardware.
- Target reproduction then exposed a third HX8 defect: the vendor returned
  command response code `1` while the servo status reported
  `STATUS_COMMAND_EXECUTING` (`status_flags=1`) with no protection bits. PX4
  treated every nonzero command response as rejected, latched fault 11, and
  sent a secondary RELEASE. Command response `0` (accepted/idle) and `1`
  (accepted/executing) are now accepted; `2+` remains rejection/error.
- The response-code fix passes HX8 Protocol 10/10, Controller 24/24,
  CLI 1/1, and BackendPolicy 7/7. `make zeroone_x6_hybrid` passes. Current
  firmware SHA-256:
  `da4a1d0fd53a7e9c2e9ef9df916a488a0c8b867cef6d4d595426e24fba54737c`.
- Target validation of that artifact confirmed the response-code fix for
  RELEASE (`command_accepted=true`, `command_result=1`), but a Rover MOVE to
  exactly `HX8_ANG_ROV=180.0` still produces a real fault 11. Quad HOLD at
  90 degrees uses the same ID, timing, and 20 W power envelope successfully,
  leaving the exact upper-angle boundary as the leading hardware/protocol
  variable. A temporary 179-degree boundary test is pending.
- A fuller UART capture included `05 1C 0B 02 00 01 2F`, proving one TimedMove
  was accepted/executing. The surrounding status frames still reported angle
  about 89.7 degrees and moving, so this frame does not prove that the Rover
  179-degree MOVE was sent. The following `05 1C 18 02 00 01 3C` is an
  accepted Stop/RELEASE response.
- A diagnostic firmware now reports local sequence rejection and local motion
  acceptance, including sequence, type, target, cached endpoints, timing,
  power, and transition parameters. Current artifact SHA-256:
  `bd7d426be1cd242dd5902c28d45c2466532f1b0e3d58cf6986b727605286d5a4`.
- Hardware showed the outer motion validator accepts Rover sequences 4 and 8,
  and the captured TimedMove ACK is accepted/executing, while the servo angle
  remains near 89.7 degrees at only 14 mA/167 mW. The earlier diagnostic was
  emitted before `Controller::setTarget()` and therefore did not prove the
  controller's internal result. A corrected diagnostic now logs controller
  queue/rejection and the exact sequence/result tuple that triggers fault 11.
  Current artifact SHA-256:
  `c04cedb3fee00510b440ca8f893c0820beacd88958b7d55fe8e33ec796962ddc`.
- Rover tuning reference is maintained in
  `docs/hybrid/rover_pid_and_parameter_reference.md`. The worktree-local
  `rover_ulog_plot.py` loads a fixed relevant-topic whitelist and generates six
  control/mode plots plus a parameter-and-error summary. It does not dump raw
  ULog samples. Current default logging may omit `m2006_motor_status`,
  `actuator_motors_rover`, and `hybrid_vehicle_status`; the script reports such
  omissions explicitly.
- Section 10 of the Rover tuning reference orders all Rover/M2006 parameters
  by the required inside-out tuning sequence and records the test method,
  observation target, and too-high/too-low symptoms for each parameter.
- The 2026-09 integration line uses `testc3_v1.16.1` as an immutable checkpoint
  of the validated testc1 working state. `debug/testc4_v1.16.1` is the semantic
  integration branch for testc2; Rover tuning is migrated afterward from a
  separate worktree so the two integration boundaries remain auditable.
- The testc3 checkpoint candidate builds as `zeroone_x6_hybrid`: FLASH is
  1,872,672 / 1,966,080 bytes (95.25%). The `.px4` SHA-256 is
  `d0c8642d51886f13f75a07f0f911e1943a4a96dbe51b92befd16bc786cecc981` and
  the `.bin` SHA-256 is
  `88ce13b7eaf91f111e70f551336169d7b4145cf1c73f6512f7c5d507de4d7708`.
- The testc2 integration preserves the testc1 CAN/M2006/HX8 implementation and
  uses the tested `manual_control_switches` transition edge path. It adds the
  independent MAV type 200, command 50000, message 60000, Mission lifecycle,
  shape-specific mode routing, and exact-one-bit `rover_velocity` DDS input.
- The testc2-stage MAVLink gitlink is
  `3b84efb97a7c0b4767868e8725bd6902c0d884e8`. The merged hybrid target uses
  `hybrid_vehicle`, builds at 1,879,496 / 1,966,080 FLASH bytes (95.60%), and
  has `.px4` SHA-256
  `eccfb72fcabbb0b7aa56f115d8a83f1fd16d2282dd49922903d3faa29fb40c4a`;
  the `.bin` SHA-256 is
  `7c53292714b73184fd305f3e780cafd457e9b7560d1bd652e6966c763da1ff06`.
- All 166 CTest cases pass after correcting a pre-existing near-pi test
  expectation to the continuous normalized value `0.1 / 180`; the production
  normalization algorithm was unchanged. The Rover yaw-rate target is no
  longer filtered by `RO_YAW_RATE_TH`; only measured gyro noise is filtered.
- All 39 affected C/C++ files were run through project AStyle and then forced
  to rebuild despite AStyle's preserved mtimes. The final incremental hybrid
  rebuild and 166/166 test run both pass.
- Rover realtime tuning is integrated on the independent
  `feature/testc4-rover-tuning` worktree. The PX4 MAVLink gitlink is
  `21922689c6fb113884df0f66582d8e602286fdc1`; its published branch is
  `feature/hybrid-rover-tuning-v1.16.1` and its annotated composite tag is
  `qgc-hybrid-rover-tuning-v1.16.1-r1`.
- GitHub ruleset `22006870` actively protects that new tag from deletion and
  non-fast-forward updates; this was verified through the repository ruleset
  API after publication.
- The combined `hybrid_vehicle`/`qgc_hybrid` protocol retains command 50000 and
  message 60000 and adds messages 60100--60103. QGC composite generation still
  excludes the conflicting Storm32 message 60000.
- Differential Rate, Attitude, Velocity, and Position controllers now publish
  atomic tuning status with explicit `active` and validity semantics. Rover
  controller caches are invalidated on shape/control-source epoch changes;
  testc2 exact-one-bit `rover_velocity` and transition epoch checks remain in
  the Rate and Velocity controllers.
- The tuning streams are registry-only and have no default stream rate. They
  require command 511 and emit flags-zero/NaN termination frames outside a
  fresh, fault-free `HYBRID_STATE_DRIVING` controller sample.
- Isolated-PATH `make tests` passes 167/167, including 22/22 bounded stream
  configuration cases and 3/3 Differential Offboard policy cases. The first
  clean test build exposed and fixed a missing generated parameter/uORB/MAVLink
  header dependency in `unit-MavlinkStreamConfig`.
- The final `zeroone_x6_hybrid` build uses 1,894,168 / 1,966,080 FLASH bytes
  (96.34%). Artifact sizes are 1,774,812 bytes (`.px4`) and 1,894,168 bytes
  (`.bin`). SHA-256 is
  `9761cc02b95be3019089b6d8fc88004a155fb38de6f6419ae0e900174004ecf4`
  for `.px4` and
  `04624f585668ed3eaa7164819fca322831faa05cbb6ea35272b82972961c2f09`
  for `.bin`.
- PX4 migration is structured as protocol gitlink commit `0f7d4aed7c`, Rover
  tuning producer/stream commit `4bec06916e`, and bounded stream lifecycle
  commit `f95427ab56`; the final documentation/evidence commit follows them on
  `feature/testc4-rover-tuning`.
- The PX4 integration is published on remote branch
  `origin/feature/testc4-rover-tuning`; original `testc1_v1.16.1`,
  `testc2_v1.16.1`, `testc3_v1.16.1`, and `testc4_v1.16.1` refs were not
  rewritten by the tuning phase.
- `HYBR_QUAD_ROV` belonged to the retired MAV_TYPE 22 VTOL-facade startup
  path. Independent Hybrid identity is now derived solely from `MAV_TYPE=200`,
  and `rc.hybrid_apps` starts all multicopter controllers without the `vtol`
  argument. The obsolete parameter and its three controller overrides were
  removed on 2026-09-02; the old architecture remains described only in the
  historical design/plan documents.
- The post-removal Hybrid contract and 6/6 focused CTests pass, all four
  affected source files pass the project AStyle check, and
  `make zeroone_x6_hybrid` succeeds. Generated parameter XML/JSON/C++ metadata
  contains no `HYBR_QUAD_ROV`. FLASH is 1,893,952 / 1,966,080 bytes (96.33%);
  the `.px4` is 1,774,504 bytes with SHA-256
  `c974219f7c14491d53c5b9bc5be7f3b1855dc8925c10160c2158b30ef95b6aa4`,
  and the `.bin` SHA-256 is
  `e3a7681a7a3c68f681e12aa67b298729b461ad3dc78861e255a7b030039be250`.
- The 2026-09-05 HX semantic merge uses one driver-owned 1 Mbps half-duplex
  UART for one HX8-U45H-M landing-gear servo and two HX-65HM transformation
  servos. `HX_BAUD` is the sole shared-bus baud parameter and defaults to
  1,000,000; `LG_AUTO_EN=0` disables automatic gear-position sequencing but
  does not remove HX8 online/configuration/protection safety gating.
- The merge preserves the testc4 independent Hybrid protocol and Rover tuning
  implementation. The MAVLink submodule remains at combined protocol commit
  `21922689c6fb113884df0f66582d8e602286fdc1`; command 50000/message 60000 and
  messages 60100--60103 are unchanged. No MAVLink stream-rate or
  `dds_topics.yaml` change is part of the HX merge.
- Internal uORB is extended with `Hx65ServoCommand`, `Hx65ServoStatus`, HX8
  gear move/hold command types, and Hybrid sequence/propulsion/gear status
  fields. These additions are not exposed to QGC or `/fmu/out` DDS by this
  merge; QGC continues using the existing private MAVLink dialect and cannot
  display the new per-servo/gear details without a future versioned message.
- Final pre-commit verification passes 171/171 full CTests, 14/14 focused
  HX/Hybrid/Commander/Mission tests, the Hybrid source contract, affected-file
  AStyle and `git diff --check`. `zeroone_x6_hybrid` uses 1,869,480 / 1,966,080
  FLASH bytes (95.09%). Artifact sizes are 1,748,844 bytes (`.px4`) and
  1,869,480 bytes (`.bin`); SHA-256 values are
  `48f3980bace8f7df77c97e24bd38ba4d81b56729833db3fb11a10d499d89849b`
  and `8f13ce277f4ec2063bddea1777f2a1d76f93c8cfc0779bb4941da2415cc0f95f`.

# Mixed UART Servo and Landing-Gear Design Audit

Date: 2026-09-02

现场配置与验收步骤见
[`hx-shared-bus-commissioning.zh-CN.md`](hx-shared-bus-commissioning.zh-CN.md)。

## Scope and verified baseline

- Worktree: `/home/crocodile/PX4-Autopilot-hx8-hx65hm`
- Branch: `feat/hybrid-landing-gear-servos`
- Base: `origin/testc3_v1.16.1` at
  `285a9d5716e6f2935545532350645044a52ad11b`
- Firmware target: `make zeroone_x6_hybrid`
- Unmodified baseline build: passed; artifact size 1,757,312 bytes, SHA-256
  `06a74494864148b250dc277f03f269012d7036ddafbd04344ab0f1791b72fcf7`
- Linker evidence: FLASH usage is 1,872,640 bytes of 1,920 KiB (95.25%), so the
  implementation must remain compact and avoid a second general-purpose servo
  framework.

Evidence reviewed:

- Existing `src/lib/hx8_servo`, `src/drivers/actuators/hx8_uart_servo`,
  `src/lib/hybrid_control`, `src/modules/hybrid_vehicle_control`, and Commander
  hybrid checks.
- FashionStar public UART/RS-485 protocol v1.0.25 and product selection pages.
- Local HX-65HM protocol v1.0 PDF, user manual, register workbook V3.7, and
  Python/STM32 examples under `/mnt/e/PX4/HX-65HM`.

## FashionStar HX8-U45H-M capability matrix

| Capability | Product protocol | Current PX4 implementation |
| --- | --- | --- |
| Half-duplex UART, ID addressing | Supported | Supported for one configured ID |
| Ping | `0x01` | Supported |
| Simple single-turn time move | `0x08` | Not implemented |
| Damping mode | `0x09` | Not implemented |
| Single-turn angle read | `0x0a` | Supported |
| Advanced single-turn, time and accel/decel | `0x0b` | Supported; only motion command used |
| Advanced single-turn, speed and accel/decel | `0x0c` | Not implemented |
| Simple multi-turn time move | `0x0d` | Not implemented |
| Advanced multi-turn, time and accel/decel | `0x0e` | Implemented for the landing-gear role |
| Advanced multi-turn, speed and accel/decel | `0x0f` | Not implemented |
| Multi-turn angle read | `0x10` | Not sent directly |
| Reset turn count | `0x11` | Not implemented |
| Async write/execute | `0x12` / `0x13` | Not implemented |
| Status monitor | `0x16` | Supported |
| Set origin | `0x17` | Not implemented |
| Stop/release | `0x18` | Supported |
| Synchronous command | `0x19` | Not implemented |
| Configuration read/write | `0x03` / `0x04` | Selected safety parameters only |
| Voltage/current/power/temperature/status | Monitor response | Supported |
| Multi-turn position feedback | Signed 32-bit, 0.1 degree | Already decoded by monitor `0x16` |

The implemented HX8 extension adds command `0x0e`, encodes the target
as signed 32-bit tenths of a degree, remove the current +/-180 degree command
limit for the landing-gear role, and retain the existing monitor-based 32-bit
feedback. The single-turn transformation endpoint assumptions must not be reused
for the landing gear.

## Hiwonder HX-65HM capability matrix

| Capability | Supported | Limits or qualification |
| --- | --- | --- |
| Half-duplex UART and daisy chain | Yes | Different wire protocol from FashionStar |
| Ping | Unsafe on multidrop bus | Hardware ignores target ID; both tested servos reply simultaneously |
| Position mode | Yes | Absolute target about +/-7.5 turns, -30719..30719 steps |
| Closed-loop velocity mode | Yes | Direction encoded in sign bit; up to 3400 steps/s |
| Open-loop PWM velocity mode | Yes | -1000..1000; unsuitable for endpoint-safe transformation |
| Position acceleration | Yes | One acceleration-rate field, 0..254 x 100 steps/s^2 |
| Separate acceleration/deceleration times | No | Not equivalent to FashionStar trapezoid timing |
| Time-based position trajectory | No | Manual explicitly says Time is ineffective in position mode |
| Direct unicast write with response | Yes | Starts each servo immediately |
| Staged write plus simultaneous ACTION | Yes | Recommended for two-sided transformation |
| Broadcast synchronous write | Yes | No per-servo acknowledgement |
| Synchronous read | Documented | Protocol says it is available only on some products; must be bench-proved |
| Position/speed/load feedback | Yes | Load is motor drive duty, not measured output torque |
| Voltage/temperature/current feedback | Yes | Electrical power can be estimated; no direct power telemetry |
| Moving flag and error byte | Yes | Voltage/sensor/temp/current/angle/overload error bits |
| Torque enable/disable | Yes | Volatile register 0x28 |
| Persistent protection configuration | Yes | NVS writes require controlled commissioning |
| Firmware/model registers | Yes | Useful identity evidence but not a cryptographic model guarantee |
| Automatic stall recovery semantics | Not proven | Must not be assumed from marketing text |

The two HX-65HM units should run in position mode. A transition is staged to
both IDs using REG_WRITE and started only after both acknowledgements with one
broadcast ACTION. Completion requires both fresh positions inside their own
endpoint tolerances, low/zero moving flags, no protection bits, and left/right
agreement within a configured mechanical skew limit.

## Documentation conflicts that require commissioning evidence

1. The HX-65HM protocol PDF says 1 Mbps. Its register table says baud code 0,
   which is 1 Mbps, is the default. The user manual says 115200 in its overview
   but tells the PC tool to use 1 Mbps. Treat 1 Mbps as factory default and
   verify each HX-65HM individually at the selected `HX_BAUD` before joining the
   shared bus. PX4 does not compare the readable baud-code register: a servo at
   another rate cannot answer on the active UART in the first place.
2. Both HX-65HM units default to ID 1. Connecting both before assigning unique
   IDs creates response collisions. Configure and verify them one at a time.
3. The HX-65HM register workbook lists the overcurrent value as initially 6000
   mA but its numeric maximum column as 511 while the description again says
   6000 mA. Do not write this persistent limit from PX4 until bench readback or
   vendor confirmation resolves the conflict.
4. The supplied SDK still uses `HX_30HM` names and some negative-value helper
   usage is inconsistent. Production encoding must follow the HX-65HM protocol
   and be covered by fixed byte-vector tests rather than copied blindly.

## Required architecture

Only one module may open the physical UART. It contains:

1. A small bus transaction scheduler with exactly one outstanding request.
2. A FashionStar protocol/controller for the one HX8 landing-gear servo.
3. A Hiwonder protocol/controller for two HX-65HM transformation servos.
4. Per-device freshness, timeout, retry, protection, and command-sequence state.

The core mechanism logic remains independent of both packet formats:

- `TransformationPairStateMachine` consumes two generic position/health samples
  and emits paired Quad/Rover endpoint intents.
- `HybridSequenceCoordinator` consumes generic landing-gear position/health and
  shape state, and owns gear intent, sequencing, propulsion ownership,
  readiness, landing-detector dwell, and fault priority.

This preserves the existing rule that control logic must not depend directly on
hardware protocol details. Starting a separate HX-65HM driver on the same serial
device is rejected because it creates concurrent reads, response theft, and
unrecoverable parser ambiguity.

## State model correction

The current `Flying / Transition / Driving / Fault` state alone is insufficient.
The coordinator needs independent fields:

- physical shape: Quad, Rover, Transitioning, Unknown, Fault;
- propulsion owner: Quad, Rover, None;
- readiness: Ready or Not Ready;
- sequence phase: stable, gear deploying, waiting for landing, disarming,
  transforming, retracting to clearance, retracting to stow, or fault;
- landing gear: stowed, clear, fully down, moving, unknown, or fault.

The implementation keeps logical `FLYING` and `DRIVING` values compatible and
adds readiness, propulsion ownership, sequence phase/fault, and landing-gear
state to the internal uORB status used by Commander. MAVLink message 60000 remains wire-compatible
unless detailed external phase supervision is explicitly placed in scope; if it
is needed, add a separate versioned private message and update QGC/ROS consumers
together instead of changing message 60000's CRC and silently breaking them.

## Recommended automatic sequences

### Quad to Rover

1. A Rover request starts HX8 deployment while Quad propulsion and flight
   control remain active. It does not command descent.
2. Wait for a fresh `landed=true` continuously for a configurable dwell and for
   HX8 to confirm the full-down multi-turn endpoint.
3. Request disarm through Commander and wait for confirmed disarmed state. A
   transient landing indication must never disable Quad propulsion.
4. Set propulsion owner to None/Rover Not Ready and move both HX-65HM units to
   their Rover endpoints.
5. After both transformation endpoints are debounced, retract HX8.
6. When HX8 passes and debounces the wheel-clearance position, publish Rover
   Ready and permit a new arm using Rover stick semantics. Continue to stow.
7. Any bus, position, skew, protection, or no-progress failure before Ready
   inhibits both propulsion paths and latches a phase-specific fault.

### Rover to Quad

1. Require or request confirmed Rover disarm, stop wheel output, and publish Quad
   Not Ready with propulsion owner None.
2. Deploy HX8 and require the full-down endpoint.
3. Move both HX-65HM units to their Quad endpoints and require paired agreement.
4. Publish Quad Ready with landing gear still fully down and allow a new arm
   using normal multicopter throttle-low semantics.
5. Only after fresh `landed=false` continuously for a configurable takeoff dwell
   may HX8 retract to stow.
6. An HX8 fault after confirmed takeoff must not disable multicopter attitude or
   rate control in flight. It raises a latched maintenance/transition fault and
   blocks future shape changes; the established airborne Quad control path is
   preserved.

## Manual landing-gear mode safety decision

The owner-approved `LG_AUTO_EN=0` behavior gives the operator independent gear
position authority: gear-down, clearance, and stowed angles do not gate shape
transitions or Ready, and gear and shape actuators may move concurrently. HX8
communication, verified configuration, and protection health still gate Ready,
arming, and transition entry so a failed gear actuator cannot be ignored before
takeoff. Quad-to-Rover still requires a confirmed landing and disarm;
Rover-to-Quad still requires disarm. This explicitly accepts responsibility for
mechanical clearance and sequencing in the operator procedure.

Recommended channel semantics are three discrete zones rather than continuously
mapping stick value to a multi-turn target:

- below -0.5: deploy fully;
- -0.5 through +0.5: hold current position;
- above +0.5: stow fully.

Manual commands are not suppressed while landed or while the shape actuators are
moving. Loss or staleness of the RC channel commands hold/stop, never a new target.

## Parameter groups still requiring calibrated values

- Shared bus: serial port, one common baud, HX8 ID, left/right HX-65HM IDs.
- Landing gear: full-down, wheel-clearance, and stowed multi-turn angles;
  tolerance; movement time; acceleration/deceleration; power; no-progress limit;
  landing and takeoff dwell times.
- Transformation pair: each side's Quad and Rover step endpoints; speed;
  acceleration; tolerance; maximum left/right normalized skew; timeout and
  no-progress limits.
- Manual mode: enable and RC channel mapping.

No uncalibrated endpoint should receive a nonzero default motion command. Initial
defaults must fail configuration validation until commissioning records contain
all three HX8 positions and all four HX-65HM endpoints.

## Verification gates

1. Host fixed-frame tests for both protocols, including corrupted, interleaved,
   wrong-ID, stale, and sign-bit frames.
2. Host scheduler tests proving one outstanding request and bounded fairness for
   all three devices.
3. State-machine tests for every phase, dwell, timeout, feedback loss, unilateral
   HX-65HM movement, skew, and in-flight HX8 failure.
4. Commander tests for Quad/Rover propulsion ownership and different arming-stick
   semantics.
5. `make zeroone_x6_hybrid`, reported separately from hardware verification.
6. Bench commissioning at no load, then constrained load, before any propellers
   are fitted: IDs/rate, endpoint directions, current/temperature limits,
   simultaneous start, power-cycle position retention, stop/release behavior,
   UART collision tests, and power-rail worst case.

## Implemented parameters and commissioning order

Set `HYB_ACT_TYPE=2`, set `PWM_MAIN_FUNC8=0`, and keep `HX8_SER_CFG` on the single selected UART.
`HX_BAUD` is the one flight-controller UART rate for all three servos and
defaults to 1 Mbps. The IDs default to HX8 0, left HX-65HM 1, and right HX-65HM
2. Configure each HX-65HM by itself before joining the bus; both factory-default
ID 1 devices must never be powered together until one ID has been changed and
both devices communicate at the selected rate.

The endpoint defaults are deliberately invalid. With propulsion disabled and
the mechanism restrained, use `hx8_uart_servo status` to read the HX8 angle and
both HX-65HM positions, then set:

- `LG_ANG_DN`, `LG_ANG_CLR`, `LG_ANG_STW`, `LG_ANG_TOL`, `LG_MOVE_T`,
  `LG_ACC_T`, `LG_DEC_T`, and a nonzero `LG_PWR_LIM`;
- `H65_L_QUD`, `H65_L_ROV`, `H65_R_QUD`, `H65_R_ROV`, `H65_TOL`,
  `H65_SKEW`, `H65_SPEED`, and `H65_ACC`;
- `LG_AUTO_EN`, `LG_MAN_CH`, `LG_TIMEOUT`, `LG_LAND_T`, and `LG_AIR_T`.

After any ID, endpoint, timing, power, or protection change, reboot before
testing. Run `hx8_uart_servo config check`; it succeeds only when HX8 protection,
both HX-65HM identities/protection/mode, and all motion endpoints are valid.
`config write` commissions HX8 persistent protection only and must be invoked
fully disarmed. HX-65HM persistent configuration remains a vendor-tool bench
operation followed by PX4 readback verification.

The software tests and firmware build do not constitute hardware validation.
In particular, ACTION simultaneity, the exact HX8 stop/hold behavior, common-bus
electrical timing, under-load skew, and protection thresholds still require the
no-propeller bench sequence above.

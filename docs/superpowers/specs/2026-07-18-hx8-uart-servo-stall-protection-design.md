# HX8 UART Servo and Transformation Stall Protection Design

Date: 2026-07-18

Branch: `change1_v1.16.1`

Target: `zeroone_x6_hybrid`

## 1. Purpose

Improve transformation-mechanism stall protection in two supported hardware
configurations:

1. Preserve the existing PWM servo on M8, add continuous position progress
   monitoring using AS5600 or two TMAG5273 sensors, disable PWM pulses on a
   detected stall, and show a latched board-red-LED fault pattern.
2. Add a FashionStar HX8-U45H-M half-duplex UART servo driver, use the servo's
   internal angle and protection feedback, and integrate it with the same
   transformation state, stall, fault, output-disable, and LED semantics.

The implementation is limited to `zeroone_x6_hybrid`. It does not adapt
`zeroone_x6_default` or any other airframe.

## 2. Confirmed Hardware

- Flight controller software target: repository board `zeroone_x6`.
- The X6 V2 and X6+ V2 names do not require different software handling.
- PWM configuration:
  - transformation servo output: M8;
  - M8 disarmed and failsafe raw PWM values: zero;
  - position feedback: AS5600 or TMAG5273, selected automatically at runtime.
- TMAG5273 configuration:
  - two sensors, one near each endpoint;
  - N52 axial cylindrical magnet, 5 mm diameter and 10 mm length;
  - assume the installed geometry provides a sufficiently monotonic,
    non-blind XYZ signature over the complete travel;
  - retain endpoint thresholds as independent final-position confirmation;
  - hardware characterization remains mandatory.
- HX8 configuration:
  - FashionStar HX8-U45H-M;
  - 9.0--12.6 V independent servo supply, common ground with the flight
    controller;
  - peak current is 5.5 A;
  - normal full-duplex flight-controller UART connected through an external
    automatic-direction half-duplex TTL adapter;
  - no DIR/OE GPIO;
  - default flight-controller port: EXT2 (`/dev/ttyS3`);
  - default link: 115200 baud, 8 data bits, no parity, 1 stop bit;
  - no flight-controller-controlled servo power switch.
- Board LEDs:
  - red, green, and blue GPIO channels exist;
  - green remains the power indication;
  - blue remains the armed-state indication;
  - Commander owns red fault/overload indication.

## 3. Safety Boundary

Stopping PWM pulses does not guarantee that an arbitrary RC servo releases
torque. A PWM servo can only be described as protected after bench measurement
shows its current falls when pulses stop. Without that evidence, the PWM path
reduces risk but does not guarantee prevention of thermal damage.

On the HX8 path, a working UART allows PX4 to command stop-and-release. On a
complete UART failure PX4 cannot command release or cut power. Protection then
depends entirely on previously verified HX8 onboard stall, current, power,
temperature, and voltage configuration. This is an accepted residual hardware
risk and must be reported separately from software behavior.

## 4. Architecture

### 4.1 Continuous position adapter

All position providers expose a hardware-independent sample:

```text
position_normalized: Quad = 0.0, Rover = 1.0
position_valid
source
timestamp
```

Supported sources are AS5600, paired TMAG5273 sensors, and HX8 internal angle.
The adapter owns source-specific conversion. The transformation state machine
does not access sensor messages or servo protocol fields.

### 4.2 Progress and stall monitor

A pure library component accepts normalized position, target, command-active
state, and HRT time. It produces moving, target-reached, no-progress, and
sensor-fault results. It has no uORB, UART, PWM, board GPIO, or parameter
dependencies.

### 4.3 Actuator backends

A reboot-required `HYB_ACT_TYPE` parameter selects exactly one backend:

```text
0 = PWM servo (default)
1 = HX8 UART servo
```

Runtime fallback is prohibited. PWM is the compatibility default. HX8 mode
keeps M8 disabled. PWM mode does not open EXT2. The selector, module, startup,
and parameters are enabled only for `zeroone_x6_hybrid`.

### 4.4 Fault and indication boundary

The core state machine publishes transformation fault state and cause.
Commander remains the only owner of board status LEDs. Neither the hybrid
control module nor the HX8 driver directly toggles board GPIOs.

## 5. Position Providers

### 5.1 AS5600

- Use the existing configured Quad and Rover endpoint angles.
- Handle zero-to-two-pi wrap correctly.
- Map directed mechanical travel to normalized zero-to-one progress.
- Keep the existing angular endpoint tolerance and debounce for final endpoint
  confirmation.

### 5.2 TMAG5273

Read full XYZ vectors from both endpoint sensors and calculate magnitudes:

```text
B_quad  = sqrt(x_quad^2  + y_quad^2  + z_quad^2)
B_rover = sqrt(x_rover^2 + y_rover^2 + z_rover^2)
position_raw = B_rover / (B_quad + B_rover)
```

Requirements:

- use XYZ magnitude instead of only `abs(mag_z)` for progress;
- apply a bounded short low-pass filter to the normalized ratio;
- retain current per-endpoint magnetic thresholds for final endpoint
  confirmation;
- treat a source change as a new progress-monitor epoch;
- reject non-finite, stale, and physically invalid samples;
- do not infer continuous position if bench data is not monotonic or contains a
  middle blind region.

The current local driver assumes TMAG5273A1 +/-40 mT at 100 Hz and does not
configure range or averaging. With the confirmed ideal on-axis N52 magnet,
estimated field is 40 mT at 6.5 mm, 5 mT at 16.5 mm, 3 mT at 20.3 mm, and
1 mT at 31.0 mm from the magnet face. Bench data, not the estimate, determines
the final useful region.

### 5.3 Automatic external-sensor selection

- This selection applies to the PWM backend. The HX8 backend always requires
  fresh internal-angle feedback and does not require external position sensors.
- Preserve the existing `HYB_SENS_EN` switch.
- With sensors disabled, retain maximum-time open-loop completion.
- With sensors enabled, prefer fresh healthy AS5600 data.
- If AS5600 is invalid and both required TMAG streams are valid, use TMAG.
- Do not add a sensor-type selector parameter.
- Reset progress monitoring whenever the selected source changes.
- If no valid configured source remains for `HYB_SENS_TO`, enter a sensor
  fault; never degrade a runtime sensor failure to open-loop completion.

### 5.4 HX8 internal position

- Read the HX8 single-turn angle.
- Map configured Quad and Rover target angles to normalized progress.
- Require a valid checksum, matching servo ID, finite angle, and fresh HRT
  timestamp.
- External AS5600/TMAG data, if present, may confirm endpoints but is not fused
  numerically with HX8 angle and is not an HX8 health or arming requirement in
  this scope.

## 6. Stall Detection

Stall detection runs only in `TRANSITION_TO_QUAD` or
`TRANSITION_TO_ROVER`, after an actuator command is effective and before final
endpoint confirmation. Stable holding does not run position-only stall logic.

For target direction `direction = +1` toward Rover and `-1` toward Quad, the
monitor maintains:

```text
anchor_position
anchor_time
```

At monitor start, the current sample becomes the anchor. The anchor and its
time update only when `direction * (position - anchor_position) >=
HYB_STALL_D`. Reverse movement never moves the anchor, and sub-threshold
movement does not accumulate through oscillation. This makes the timer reset
only after one directed net displacement from the last accepted anchor.

Add:

| Parameter | Default | Meaning |
|---|---:|---|
| `HYB_STALL_T` | 0.8 s | Maximum interval without valid directed progress |
| `HYB_STALL_D` | 0.02 | Minimum progress, as a fraction of full travel |

Enter `STALL` when all conditions are true:

```text
transition active
actuator command effective
position fresh and valid
endpoint not confirmed
target error remains greater than HYB_STALL_D
directed progress remains less than HYB_STALL_D for HYB_STALL_T
```

`HYBRID_TRANS_T` remains the independent absolute transition deadline. The
stall timer detects an immobile mechanism early; the absolute deadline catches
motion that never reaches the endpoint.

## 7. PWM Backend

- Preserve normal output and stable endpoint holding behavior.
- Preserve sensor-disabled maximum-time behavior.
- On any latched transformation fault, publish NaN for the transformation
  servo.
- Keep `PWM_MAIN_DIS8=0` and `PWM_MAIN_FAIL8=0` so fault/disarmed operation is
  intended to suppress M8 pulses rather than command midpoint.
- `actuator_servos.control[0] = 0` must never be used as a stop request because
  normalized zero is servo midpoint.
- Fault, lockdown, manual lockdown, or force failsafe overrides RC
  commissioning and leaves the output invalid.

## 8. HX8 Driver

### 8.1 Module scope

Add an actuator driver under `src/drivers/actuators/hx8_uart_servo`. It owns
UART, protocol framing, request scheduling, telemetry, protection verification,
commissioning writes, and communication diagnostics. It does not own hybrid
state, external sensors, propulsion output, or LEDs.

### 8.2 Serial configuration

- Standard PX4 port parameter: `HX8_SER_CFG`, default EXT2.
- Baud parameter: `HX8_BAUD`, default 115200 and restricted to vendor-supported
  values.
- ID parameter: `HX8_ID`, default 0, range 0--254.
- The conditional start wrapper checks the selected actuator backend before
  opening `${SERIAL_DEV}`. PWM mode exits without opening the device.
- Use normal TX/RX UART mode. Do not call `TIOCSSINGLEWIRE`.
- Complete each transmit with a real transmission-complete/drain operation
  before waiting for a response.

### 8.3 Required protocol commands

Implement only the required commands:

| ID | Purpose |
|---:|---|
| `0x01` | ping/online detection |
| `0x03` | parameter read |
| `0x04` | persistent parameter write |
| `0x0A` | single-turn angle read |
| `0x0B` | timed advanced single-turn angle control |
| `0x16` | complete status monitor |
| `0x18` | stop/release, hold, or damping |

Command frames begin `0x12 0x4c`; response frames begin `0x05 0x1c`; checksum
is the modulo-256 sum of preceding frame bytes. Parsing must recover after
garbage, partial frames, concatenated frames, wrong headers, invalid lengths,
checksum failures, ID mismatches, unknown commands, timeout, and optional TX
echo.

### 8.4 Scheduling

Only one request may be outstanding. Priority is:

1. emergency stop-and-release;
2. new target command;
3. runtime status monitor;
4. angle read;
5. configuration verification;
6. diagnostics.

Maintain at least 20 ms between commands to the servo. Send a target only on a
new command or bounded retry, not continuously. Monitor at 20 Hz while moving
and 5 Hz while stable. All retries and response waits are bounded.

### 8.5 Motion parameters

Add configured Quad/Rover angles, move time, acceleration time, deceleration
time, and run-power limit. Require:

```text
move time > acceleration time + deceleration time
move time < HYBRID_TRANS_T
angles within supported single-turn range
power within HX8 model limits
```

Invalid configuration prevents the HX8 backend from becoming healthy.

### 8.6 uORB interface

Add driver-specific command and status topics. Command carries HRT timestamp,
ID, type, target angle, timing, power, and sequence. Status carries HRT sample
and publication timestamps, online/config-verified flags, angle, voltage,
current, power, temperature, raw protection flags, command result/sequence,
communication counters, and latest valid response time.

The pure transformation library never includes these messages. The module
adapter converts HX8 status into the common position and actuator-health input.

### 8.7 Safety gate

- Position commands require armed or explicitly permitted prearmed
  commissioning state, valid configuration, no lockdown/failsafe, and fresh
  command.
- Stop-and-release is always permitted.
- Persistent writes require fully disarmed state and explicit commissioning.
- Expired or out-of-order commands are rejected.
- Configuration mismatch or active protection rejects normal motion.

## 9. HX8 Protection Configuration

PX4 parameters contain expected stall-unlock, current, power, temperature,
voltage, response, and related safety settings. These are safety-calibration
inputs rather than guessed design constants: implementation must document the
protocol unit and supported range of every value, derive defaults from the
HX8-U45H-M documentation, and validate those defaults with the bench load
before the HX8 backend is accepted for operation.

Normal boot is read-only:

1. ping the configured ID;
2. read model/version and required protection settings;
3. compare every value with PX4 expectations;
4. publish `config_verified=true` only on a complete match;
5. fail preflight on mismatch; never repair automatically.

The explicit command `hx8_uart_servo config write` is the only persistent-write
path. It requires disarmed commissioning state and mandatory per-item readback.
No startup loop writes servo nonvolatile memory.

## 10. Fault Model

Preserve fault codes zero through six and append:

```text
7  STALL
8  ACTUATOR_COMMUNICATION
9  ACTUATOR_PROTECTION
10 ACTUATOR_CONFIG_MISMATCH
11 ACTUATOR_COMMAND_REJECTED
```

The first initiating cause remains the primary fault. A secondary failure, such
as inability to transmit release after a stall, is retained in HX8 status and
events and does not erase the stall root cause.

Extend `hybrid_vehicle_status` with actuator backend, position source including
HX8, normalized position/validity, actuator online/healthy/config-verified
summary, no-progress elapsed time, and protection summary. Detailed HX8 values
remain in `hx8_servo_status`.

On any transformation fault:

1. enter and latch `TRANSITION_FAULT`;
2. preserve the initiating cause;
3. clear RC commissioning ownership;
4. reject new transformation targets;
5. apply existing propulsion fault gating;
6. run the selected backend's safe action;
7. publish a critical event and LED state;
8. never retry transformation automatically.

PWM safe action is NaN output. HX8 safe action is bounded stop-and-release
attempts followed by prohibition of position commands and low-rate monitoring.
If communication is unavailable, internal protection is the only release
mechanism.

## 11. Stable-State Monitoring and Arming

External-sensor mode continues to verify the expected endpoint. A debounced
mismatch faults. Position-only logic cannot detect PWM holding current at a
correct endpoint.

HX8 stable monitoring continues at low rate. Angle mismatch, stale
communication, or any active protection condition faults even if the angle
still appears correct.

PWM preflight requires valid PWM configuration, a known confirmed shape when
sensors are enabled, and no latched fault.

HX8 preflight additionally requires conflict-free serial configuration, online
matching ID, fresh status, verified protection configuration, no active
protection, known endpoint angle, disabled M8, and no latched fault.

## 12. Fault Clearing

Only an explicit `hybrid_vehicle_control clear_fault` while fully disarmed may
clear a fault. Configuration must be valid and the selected backend's required
feedback must be fresh. The HX8 backend additionally requires restored
communication and cleared recoverable protection flags. A voltage protection
state requiring power cycle cannot be cleared in software while it remains
active.

After clearing, a confirmed endpoint restores its stable state. An intermediate
position becomes `UNKNOWN`, remains unarmable, and may be moved only through
permitted disarmed/prearmed commissioning. The failed target is never resumed
automatically.

## 13. Commander Red LED

Commander remains the sole board-red-LED owner:

| Condition | Pattern |
|---|---|
| healthy | off |
| CPU/RAM overload | existing continuous fast flash, approximately 10 Hz |
| other transformation fault | 1 Hz slow flash |
| transformation stall | two 150 ms flashes followed by 1 s pause |
| overload plus transformation fault | three flashes followed by pause |

Only fresh hybrid status affects the pattern. A stale status on the hybrid
airframe produces the generic transformation-fault pattern and prevents arming.
Green power and blue armed indications are unchanged.

## 14. Tests

### 14.1 Pure host tests

- AS5600 wrap, direction, normalization, and endpoint tolerance;
- TMAG XYZ magnitude, ratio, saturation, filtering, monotonic input, noise, and
  stale samples;
- automatic source selection and progress reset on source change;
- normal directed progress;
- immobility, reverse motion, noise, and oscillation stall cases;
- no stall in stable hold or after endpoint confirmation;
- independent sensor and absolute-transition timeout;
- every fault latch and disarmed clear condition;
- LED healthy, overload, generic fault, stall, combined, and stale patterns.

### 14.2 Protocol and driver tests

- official/golden frames for every implemented command;
- partial, concatenated, corrupt, oversized, mismatched, unknown, timeout, and
  echoed frames;
- one outstanding request, priority, 20 ms spacing, bounded retry, and parser
  resynchronization;
- armed/lockdown/failsafe/config gates;
- read-only boot verification;
- disarmed explicit write and mandatory readback;
- communication statistics and HRT freshness.

### 14.3 Integration tests

- default PWM backend and legacy sensor-disabled timing;
- AS5600 and TMAG auto-detection;
- PWM fault produces NaN and never midpoint;
- HX8 mode disables M8 and conditionally opens EXT2;
- PWM mode does not open EXT2;
- Commander/health checks consume expanded hybrid state correctly;
- only `make zeroone_x6_hybrid` is a required firmware build.

### 14.4 Bench tests

- record full-travel TMAG XYZ, saturation, normalized monotonicity, and blind
  regions;
- AS5600 wrap and repeatability;
- at least 20 normal transitions in each direction for each applicable sensor
  path with no false stalls;
- induce stalls at several positions and measure detection time;
- use an oscilloscope to prove M8 pulses stop after PWM fault;
- measure RC-servo current after pulse loss and report whether torque actually
  releases;
- logic-analyzer verification of adapter turnaround and UART timing;
- HX8 angle/status/electrical telemetry verification;
- HX8 onboard release under deliberate stall;
- UART removal during deliberate stall to verify the accepted internal-only
  protection boundary;
- voltage protection and required power-cycle behavior;
- every red LED pattern and overload coexistence.

Acceptance timing:

- stall detection no later than `HYB_STALL_T + 100 ms`;
- PWM NaN within one hybrid-control cycle after fault;
- communication fault within configured bounded request timeouts;
- hybrid fault publication within one hybrid-control cycle;
- red indication within 100 ms of published fault;
- no automatic fault recovery or actuator-backend fallback.

## 15. Development Sequence

1. Add pure common position/progress/stall logic, PWM integration, fault codes,
   and Commander LED behavior using tests first.
2. Add protocol parser, HX8 driver, uORB messages, and protected commissioning
   using tests first.
3. Add backend selection, EXT2 conditional ownership, M8 mutual exclusion,
   health checks, and full hybrid integration.
4. Run focused host tests and `make zeroone_x6_hybrid`, then perform the bench
   matrix. Firmware build success and physical protection evidence are reported
   separately.

## 16. Explicit Non-Goals

- no `zeroone_x6_default` or other-airframe adaptation;
- no DroneCAN/Cyphal encapsulation of the vendor UART protocol;
- no software UART on arbitrary GPIO;
- no manual DIR/OE GPIO;
- no controllable HX8 power switch;
- no runtime PWM/HX8 fallback;
- no AS5600/TMAG numeric fusion;
- no unrelated multi-turn, multi-servo synchronization, or asynchronous HX8
  protocol features;
- no claim that pulse loss or UART loss guarantees physical torque release
  without bench evidence.

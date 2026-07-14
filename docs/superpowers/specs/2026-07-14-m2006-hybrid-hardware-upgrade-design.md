# M2006 Hybrid Hardware Upgrade Design

Date: 2026-07-14

Status: Approved in interactive design review; pending review of this written specification.

## 1. Purpose

Upgrade the ZeroOne quad-rover hybrid vehicle from the previous hardware revision to:

- Two DJI M2006 P36 motors with C610 controllers on CAN1.
- One positional PWM servo on M8 for the transformation mechanism.
- AS5600 continuous magnetic angle feedback with TMAG5273 endpoint fallback.

The implementation must preserve PX4's native multicopter and differential-rover control chains. `hybrid_vehicle_control` remains an output and vehicle-state arbiter; it does not replace either controller.

## 2. Baseline and Scope

Create branch `change1_v1.16.1` from `testv3_v1.16.1` at `e1da1439dc`.

Hardware facts:

- Flight controller: ZeroOne Air X6 V2.
- Firmware target: `make zeroone_x6_hybrid`.
- CAN bus: CAN1 at 1 Mbps.
- Left C610: ID 1, feedback ID `0x201`.
- Right C610: ID 2, feedback ID `0x202`.
- Both current commands share standard CAN frame `0x200`.
- Wheel diameter: 340 mm.
- M2006 output shaft to wheel reduction: 3.6:1.
- Transformation servo: M8.
- No DroneCAN or Cyphal devices currently share CAN1.

Out of scope:

- Replacing `RoverDifferential` or changing its steering/throttle ownership.
- Making DJI's proprietary frames appear to be DroneCAN or Cyphal messages.
- Runtime coexistence between M2006, DroneCAN, and Cyphal on CAN1.
- Guaranteeing motor shutdown after flight-controller power loss or a total physical CAN failure without additional hardware.

## 3. System Architecture

```text
PX4 MC controllers ---- actuator_motors_mc ----+
                                                +--> hybrid_vehicle_control
RoverDifferential ----- actuator_motors_rover --+            |
                                                             +--> actuator_motors
                                                             |     control[0..3] -> M1-M4
                                                             |     control[4..5] -> m2006_can
                                                             |
                                                             +--> actuator_servos[0]
                                                                   Servo 1 -> M8
```

### 3.1 `hybrid_vehicle_control`

- Preserve the existing MC/Rover mode and output arbitration.
- In `FLYING`, publish only MC motor controls 0 through 3.
- In `DRIVING`, place the native Rover left and right outputs in final `actuator_motors.control[4]` and `[5]`.
- In transition, unknown, or fault states, publish no propulsion command.
- Replace the old two-output H-bridge transformation command with `actuator_servos.control[0]` only.
- Continuously publish the selected Quad or Rover servo target after normal completion.

Physical servo hold applies while PX4 actuator outputs are armed or prearmed. A full vehicle disarm intentionally applies `PWM_MAIN_DIS8=0` and releases the servo, as required by the selected disarmed behavior.

### 3.2 `m2006_can`

Add a standalone driver under the PX4 driver layer. It owns CAN framing, wheel-speed loops, watchdogs, and diagnostics. It must not calculate steering or differential wheel mixing.

The driver subscribes to:

- Final `actuator_motors`.
- `hybrid_vehicle_status`.
- Vehicle actuator armed state.

Nonzero commands require all of the following:

- Vehicle is armed.
- Hybrid state is `DRIVING`.
- Both motor feedback streams are healthy.
- Final actuator command is finite and fresh.
- No latched driver or CAN fault exists.

### 3.3 Physical output mapping

- M1-M4 retain multicopter motor functions.
- M5 and M6 physical output functions are disabled; logical controls `[4]` and `[5]` remain available to `m2006_can` and logging.
- M8 is assigned `Servo 1` and consumes `actuator_servos.control[0]`.
- M8 disarmed and failsafe values are both zero, so faults do not command the default 1500 us center position.

## 4. CAN Ownership and Protocol

`m2006_can` reuses PX4's STM32H7 UAVCAN low-level `CanInitHelper`/`ICanIface` infrastructure rather than copying STM32 HAL code. It initializes only CAN1 at 1 Mbps and accepts standard identifiers `0x201` and `0x202`.

Runtime ownership is mutually exclusive:

- Starting `m2006_can` requires `UAVCAN_ENABLE=0` and `CYPHAL_ENABLE=0`.
- Conflicting configuration causes `m2006_can` to refuse startup and publish a health error.
- DroneCAN and Cyphal remain compiled and available when M2006 is disabled.
- Changing CAN ownership requires a reboot because the STM32H7 low-level CAN driver is not safely deinitialized at runtime.

C610 command frame `0x200`:

| Bytes | Meaning |
| --- | --- |
| 0-1 | Left ID 1 current command, signed 16-bit big-endian |
| 2-3 | Right ID 2 current command, signed 16-bit big-endian |
| 4-7 | Zero |

Feedback frames decode encoder position, output-shaft rpm, and feedback torque-current. C610 documentation defines bytes 6 and 7 as unused; no temperature protection may be based on those bytes.

## 5. M2006 Speed Control

The driver runs at 500 Hz. Each wheel has an independent controller:

```text
normalized wheel command
    * M2K_MAX_RPM
    -> rpm setpoint slew limiter
    -> speed PID plus optional feedforward
    -> signed current limit
    -> C610 internal FOC torque control
```

Requirements:

- Constrain normalized commands to `[-1, 1]`.
- Default maximum output-shaft speed is 500 rpm, DJI's no-load rating.
- Keep left and right direction reversal independent and configurable.
- Apply anti-windup whenever the current command saturates.
- Reset integrators whenever drive is disabled, a fault occurs, or a stopped controller is re-enabled.
- Do not add a second software current loop; the speed controller directly sets the C610 torque-current command and the C610 performs its internal FOC control.
- `RO_SPEED_LIM` remains the upper requested vehicle-speed limit.
- Re-identify `RO_MAX_THR_SPEED`; do not reuse the previous DShot value.

At 500 rpm, 3.6:1 additional reduction, and a 340 mm wheel, theoretical no-load vehicle speed is approximately 2.47 m/s. This is an initial `RO_MAX_THR_SPEED` estimate, not a loaded vehicle result.

## 6. M2006 Safety State

On startup, transmit zero current until both IDs provide continuous valid feedback for 100 ms. Default watchdogs are 50 ms for motor feedback and 100 ms for final actuator commands.

If either motor feedback is lost:

- Command both motors to zero.
- Reset both speed controllers.
- Latch a driver fault.
- Do not recover on returning feedback while armed.
- Clear only after the vehicle is disarmed and both feedback streams are healthy.

Use the same two-wheel shutdown for bus-off, sustained transmit failure, invalid actuator data, or stale actuator data. Add target-rpm slew limiting so recovery after disarm cannot apply an immediate full-scale step.

The official C610 documentation does not establish a guaranteed zero-torque watchdog after all command frames disappear. Software can command zero only while CAN transmission remains possible. A hardware emergency stop or C610 power disconnect is required if the safety case must cover flight-controller power loss, broken CAN wiring, or complete transceiver failure.

## 7. Transformation State Machine

Extend hybrid state reporting with `UNKNOWN` and `TRANSITION_FAULT` while retaining `FLYING`, `DRIVING`, and both directional transition states.

### 7.1 Stable states

- `FLYING`: continuously hold the configured Quad servo target and allow only M1-M4.
- `DRIVING`: continuously hold the configured Rover servo target and allow only M2006 wheel commands.
- A transition request immediately suppresses all propulsion before moving M8.
- Preserve the existing altitude check before a Quad-to-Rover transition.

### 7.2 Sensor-enabled behavior

AS5600 is the preferred continuous position source. TMAG5273 Quad/Rover endpoint sensors are the fallback.

At startup:

- A valid AS5600 angle within one endpoint tolerance determines the physical state.
- If AS5600 is unavailable, exactly one valid TMAG5273 endpoint determines the state.
- Neither endpoint, both TMAG endpoints, stale sensors, or conflicting valid sources produce `UNKNOWN` or `TRANSITION_FAULT`; they never default silently to Quad.

During transition:

- If AS5600 is valid, require the target angle to remain within tolerance for the configured debounce time.
- If AS5600 becomes unavailable, an online target TMAG5273 may complete the transition.
- If both source types are online, contradictory endpoint conclusions are a fault.
- Loss of all usable target feedback, simultaneous TMAG endpoints, or maximum-time expiry is a fault.
- A fault publishes a non-finite Servo 1 command; with M8 disarmed/failsafe set to zero, effective PWM pulses stop.
- Fault exit requires disarm followed by a new explicit Quad/Rover request, or a reboot that uniquely identifies a valid physical endpoint.

### 7.3 Sensor-disabled behavior

- Ignore AS5600 and TMAG5273 state.
- Drive to the configured target until `HYBRID_TRANS_T` expires.
- Treat time expiry as successful completion.
- Use the persistent operator-configured startup state after reboot.
- Runtime transitions must not rewrite the startup-state parameter.

This mode is explicitly open-loop: elapsed time is an assumption, not proof of physical completion.

### 7.4 Manual operation

Normal RC, MAVLink, and mission commands request a target state; they do not bypass completion checks.

Direct manual servo commissioning is allowed only while disarmed through PX4 actuator test or an explicit prearmed servo path. It keeps propulsion inhibited and does not declare `FLYING` or `DRIVING` automatically.

## 8. Parameters

### 8.1 M2006 parameters

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `M2K_EN` | 1 in this airframe | Enable standalone CAN driver |
| `M2K_L_ID` | 1 | Left C610 ID |
| `M2K_R_ID` | 2 | Right C610 ID |
| `M2K_L_REV` | 0 | Reverse left direction |
| `M2K_R_REV` | 0 | Reverse right direction |
| `M2K_MAX_RPM` | 500 | Full-scale output-shaft rpm |
| `M2K_CUR_LIM` | 10000 | Absolute C610 command limit |
| `M2K_SPD_P` | 0 | Speed proportional gain; bench configuration required |
| `M2K_SPD_I` | 0 | Speed integral gain; bench configuration required |
| `M2K_SPD_D` | 0 | Speed derivative gain |
| `M2K_SPD_FF` | 0 | Speed feedforward gain |
| `M2K_RPM_SLEW` | 500 rpm/s | Target-rpm rate limit |
| `M2K_FB_TO` | 0.05 s | Motor feedback timeout |
| `M2K_CMD_TO` | 0.10 s | Final actuator command timeout |

IDs must be distinct and valid for the C610 `0x200` group. Zero controller gains are intentionally non-driving defaults; driving-mode health checks reject an untuned all-zero controller.

### 8.2 Transformation parameters

Retain existing altitude, manual channel, AS5600 target-angle, TMAG device-ID, and TMAG threshold parameters.

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `HYBRID_TRANS_T` | Existing airframe value | Maximum transformation time |
| `HYB_SENS_EN` | 1 | Enable AS5600/TMAG completion checks |
| `HYB_BOOT_ST` | Quad | Configured startup assumption when sensors are disabled |
| `HYB_SV_QUD` | 0 | Quad normalized Servo 1 target |
| `HYB_SV_ROV` | 0 | Rover normalized Servo 1 target |
| `HYB_ANG_TOL` | 0.05 rad | AS5600 endpoint tolerance |
| `HYB_SENS_TO` | 0.30 s | Position feedback timeout |
| `HYB_DBNC_T` | 0.10 s | Continuous endpoint confirmation time |

Both servo targets default to center to avoid an uncalibrated full-travel movement. A health check rejects transformation or arming until the two configured targets differ by a safe minimum and both lie in `[-1, 1]`.

## 9. Status, Logging, and Health Checks

Add `m2006_motor_status` with two-element fields for:

- Target rpm, measured rpm, and error.
- Current command and feedback torque-current.
- Encoder position and feedback age.
- Online state and fault bits.
- RX, TX, timeout, and bus-off counters.

Extend `hybrid_vehicle_status` with:

- Current and target state.
- Active source: AS5600, TMAG5273, or none.
- Source validity.
- Transition elapsed time.
- Fault reason.

Publish status topics at bounded rates suitable for ULog. Text warnings must be rate-limited and raw CAN frames must not be printed continuously.

Arming rules:

- Reject arming in `UNKNOWN`, either transition state, or `TRANSITION_FAULT`.
- In `FLYING`, M2006 availability does not block arming.
- In `DRIVING`, require M2006 enabled, both controllers online, nonzero configured controller behavior, and no latched M2006 fault.
- Reject arming if M8 mapping, Servo targets, disarmed value, or failsafe value is unsafe.
- Reject M2006 initialization if DroneCAN or Cyphal is enabled on the same bus.

## 10. Verification

### 10.1 Host tests

- Encode signed current commands into the correct `0x200` bytes.
- Decode positive/negative rpm and torque-current, encoder boundaries, invalid IDs, and invalid DLC.
- Verify reversal, saturation, slew limits, anti-windup, and integrator reset.
- Exercise AS5600 primary, TMAG fallback, conflicting sources, sensor-disabled timing, fault latching, and fault clearing.

Hardware-independent CAN codec, controller, and transformation logic must be small testable C++ units. PX4 CAN and uORB integration remain thin adapters.

### 10.2 Firmware verification

- Run `make zeroone_x6_hybrid` with output redirected to a log.
- Verify generated parameters and uORB definitions.
- Verify M5/M6 are disabled and M8 is Servo 1 with zero disarmed/failsafe values.
- Verify M2006 and UAVCAN configurations build separately.
- Track firmware size because the approved baseline already uses 1,833,372 bytes of 1,920 KB Flash (93.25%).

### 10.3 Bench verification

- Begin with wheels unloaded and a temporarily reduced current limit.
- Confirm 1 Mbps and IDs `0x200`, `0x201`, `0x202` with a CAN analyzer.
- Confirm wheel direction and target/measured rpm signs.
- Inject command timeout, one-wheel feedback loss, and CAN errors; verify both current commands go to zero and faults latch.
- Record C610 behavior after controller power loss or a physical CAN disconnect.
- Calibrate M8 without mechanism load first.
- Verify normal endpoint hold, AS5600 completion, TMAG fallback, conflict/timeout faults, and zero pulses on fault with an oscilloscope.
- Verify sensor-disabled time completion and configured startup state.

### 10.4 Vehicle verification

Tune from inner to outer loops:

1. Independent M2006 wheel-speed loops.
2. Rover yaw-rate loop.
3. Rover yaw-attitude loop.
4. Vehicle speed loop.
5. Position and mission loops.

ULog evidence must include setpoint, measurement, error, actuator command, feedback torque-current, hybrid state, and valid sample conditions. Validate Stability, Position, Offboard, Mission, and both transformation directions. Update `RO_MAX_THR_SPEED` only from loaded measurements.

Build success and physical vehicle validation are separate results.

## 11. References

- DJI RoboMaster M2006 product specification: https://www.robomaster.com/en-US/products/components/detail/1277
- DJI RoboMaster C610 user guide: https://cdn-hz.robomaster.com/tem/RM%20C610%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E8%B0%83%E9%80%9F%E5%99%A8%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%20%E5%8F%91%E5%B8%83%E7%89%88.pdf
- Reference implementation inspected at HNUYueLuRM/basic_framework commit `c1394f3`.

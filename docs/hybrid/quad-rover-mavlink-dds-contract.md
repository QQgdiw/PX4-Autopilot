# Independent Quad/Rover integration contract

This document is the external integration contract for the `zeroone_x6_hybrid`
target. It describes the interfaces implemented by this firmware revision; it
does not turn the vehicle into a MAVLink VTOL and it does not replace normal PX4
flight or differential-Rover controllers.

## Compatibility boundary

- Dialect: `hybrid_vehicle`.
- Vehicle type: `MAV_TYPE_QUAD_ROVER` (`200`).
- Transition command: `MAV_CMD_DO_HYBRID_TRANSITION` (`50000`).
- Status message: `HYBRID_VEHICLE_STATUS` (`60000`).
- `hybrid_vehicle.xml` includes `development.xml`. The private enum entries and
  message exist at the generated `hybrid_vehicle` dialect boundary, not in a
  stock `common`-only MAVLink library.
- QGC and companion implementations must use MAVLink 2 and generate their
  bindings from this project's `hybrid_vehicle.xml`. `HYBRID_VEHICLE_STATUS`
  is not sent on a MAVLink 1 output link.
- MAVLink Normal and Onboard modes configure `HYBRID_VEHICLE_STATUS` at 1 Hz
  by default. Other MAVLink modes do not enable it by default.

There is no on-wire contract-version field or capability negotiation. The
numeric values above are therefore compatibility identifiers and must not be
renumbered. Client releases must pin a known-compatible firmware dialect and,
for DDS, matching `px4_msgs` definitions. Additive MAVLink fields belong after
`<extensions/>`; incompatible semantic or base-field changes require a new
explicitly versioned contract rather than silent reinterpretation.

`HEARTBEAT.type` is statically `MAV_TYPE_QUAD_ROVER` (`200`) in both physical
shapes. It never changes to multicopter or ground-rover type as the mechanism
moves. The dynamic shape, transition, and fault state comes only from
`HYBRID_VEHICLE_STATUS.current_state`. `EXTENDED_SYS_STATE.vtol_state` remains `MAV_VTOL_STATE_UNDEFINED`; clients must not infer shape from VTOL state.

## Transition command and ACK lifecycle

Send `COMMAND_LONG` or the equivalent vehicle command with:

- `command = MAV_CMD_DO_HYBRID_TRANSITION` (`50000`)
- `param1`: `1` = Quad, `2` = Rover; other values are invalid
- `param2` through `param7`: reserved and must each be zero or NaN

Transformation is landed-only. A fresh `vehicle_land_detected` sample with
`landed=true` is required; there is deliberately no height estimate or
height-threshold substitute. Stale land detection and not-landed conditions are
temporary rejections. A mode request never implicitly transforms the vehicle,
and a mode requested during transition, unknown state, or fault is not queued
for later execution. Request the shape, wait for its terminal result and stable
status, then request the mode.

`hybrid_vehicle_control` is the sole publisher/owner of ACKs for command 50000.
No QGC, Commander, Navigator, or companion component should synthesize a second
ACK. Its `COMMAND_ACK` behavior is:

| Condition | Immediate/progress ACK | Terminal ACK | Sequence behavior |
| --- | --- | --- | --- |
| Invalid `param1` | `MAV_RESULT_DENIED` | none | current sequence |
| Nonzero/non-NaN reserved parameter | `MAV_RESULT_DENIED` | none | current sequence |
| Transformation fault or fault state | `MAV_RESULT_DENIED` | none | current sequence |
| Land detector stale | `MAV_RESULT_TEMPORARILY_REJECTED` | none | current sequence |
| Not landed | `MAV_RESULT_TEMPORARILY_REJECTED` | none | current sequence |
| Unknown stable/transition state | `MAV_RESULT_DENIED` | none | current sequence |
| Already in requested stable shape | `MAV_RESULT_ACCEPTED` | none | current sequence, no increment |
| New valid transition | `MAV_RESULT_IN_PROGRESS` | `MAV_RESULT_ACCEPTED` on stable completion, or `MAV_RESULT_FAILED` on transformation fault | increment before `IN_PROGRESS`; both ACKs carry the accepted sequence |
| Retransmit the active target | `MAV_RESULT_IN_PROGRESS` | original request receives the terminal ACK | current sequence, no increment |
| Request opposite target while transitioning | `MAV_RESULT_TEMPORARILY_REJECTED` | none | current sequence; request is not queued |
| State machine fails to enter transition | `MAV_RESULT_FAILED` | none | current sequence |

Every hybrid ACK sets `result_param2 = transition_sequence`. A newly started
request increments `transition_sequence` before its `IN_PROGRESS` ACK, so its
progress and terminal ACKs have the same non-changing correlation value.
Already-stable acceptance, retransmission, and immediate rejection carry the
current value without incrementing it. Clients should correlate on command,
target system/component, and `result_param2`; they must tolerate retransmitted
`IN_PROGRESS` ACKs and must not interpret them as new transitions.

## Hybrid status message

`HYBRID_VEHICLE_STATUS` (`60000`) contains these wire fields:

| Field | Meaning and units |
| --- | --- |
| `timestamp` (`uint64`) | PX4 boot time in microseconds |
| `transition_sequence` (`uint32`) | most recently accepted/newly started transition sequence |
| `transition_elapsed_ms` (`uint32`) | active transition elapsed time in milliseconds; zero outside timed transition/fault reporting |
| `position_normalized` (`float`) | normalized mechanism position, NaN when unavailable |
| `current_state` (`uint8`) | dynamic state enum below |
| `target_state` (`uint8`) | requested shape during transition, or none/stable target as published |
| `fault_reason` (`uint8`) | transformation fault enum below |
| `command_result` (`uint8`) | most recent command result using `MAV_RESULT`; initial/no-result is `255` |
| `sensor_source` (`uint8`) | source currently confirming mechanism position |
| `actuator_backend` (`uint8`) | active transformation actuator backend |
| `actuator_protection_flags` (`uint8`) | backend-provided protection bit field; backend-specific interpretation |
| `flags` (`uint16`) | independent status booleans listed below |
| `command_timestamp` (`uint64`, MAVLink 2 extension) | originating PX4 `vehicle_command.timestamp`, in microseconds |

`command_timestamp` identifies the exact originating `vehicle_command.timestamp`
whose result is in `command_result`; mission execution uses this to reject stale
or unrelated results. For a transition terminal result it remains the timestamp
of the command that started that transition. The uORB `transition_elapsed` is
in microseconds. MAVLink `transition_elapsed_ms` is its integer floor division
by 1000 and saturates at `UINT32_MAX` rather than wrapping. In normative terms,
`transition_elapsed_ms` is a floor conversion with `UINT32_MAX` saturation.

State values are `QUAD=0`, `TRANSITIONING=1`, `ROVER=2`, `UNKNOWN=3`, and
`TRANSITION_FAULT=4`. Target/shape values are `NONE=0`, `QUAD=1`, and `ROVER=2`.
Sensor sources are `NONE=0`, `AS5600=1`, `TMAG5273=2`, and `HX8=3`. Actuator
backends are `PWM=0` and `HX8=1`.

Fault values are:

| Value | Fault |
| ---: | --- |
| 0 | none |
| 1 | no sensor |
| 2 | sensor conflict |
| 3 | sensor timeout |
| 4 | transition timeout |
| 5 | invalid servo configuration |
| 6 | invalid transformation configuration |
| 7 | stall/no progress |
| 8 | actuator communication |
| 9 | actuator protection |
| 10 | actuator configuration mismatch |
| 11 | actuator command rejected |

Status flag bits are independent and may be combined:

| Bit | Value | Meaning |
| ---: | ---: | --- |
| 0 | 1 | sensors enabled |
| 1 | 2 | position confirmed |
| 2 | 4 | normalized position valid |
| 3 | 8 | actuator online |
| 4 | 16 | actuator healthy |
| 5 | 32 | actuator configuration verified |
| 6 | 64 | landed |
| 7 | 128 | land-detection sample fresh |

The internal `HybridVehicleStatus` uORB type additionally exposes the individual
AS5600/TMAG validity booleans, sensor/position booleans, `transition_elapsed`
and `no_progress_elapsed` in microseconds, transition-completed timestamp,
command reject reason, landed state, and land freshness. These extra fields are
not fields of message 60000 unless explicitly mapped above.

## Mission command behavior

Mission items use command 50000. Mission feasibility accepts only exact
`param1` values `1` or `2` and requires every reserved `param2` through `param7`
value to be finite zero; nonzero, NaN, infinity, and fractional target values
are rejected before flight. This is stricter than the direct command transport,
where NaN remains an accepted placeholder for a reserved parameter.

Navigator issues a given hybrid item exactly once per `(mission_id,
mission-sequence)` activation. It snapshots `transition_sequence`, records the
published `vehicle_command.timestamp`, and does not reissue the command on each
mission loop. A status result is related only when `command_timestamp` exactly
matches that issued timestamp. `TEMPORARILY_REJECTED`, `DENIED`, or `FAILED`, or
a fresh `TRANSITION_FAULT`, fails the mission result. `IN_PROGRESS`/no result
waits. `ACCEPTED` still waits for fresh status (maximum age one second), the
requested stable state and target, and either an already-stable snapshot or a
strictly newer sequence. This prevents an old stable status from completing a
new item and prevents duplicate command execution.

This describes execution after a mission is present on the vehicle; standard
MAVLink mission upload/download support for private command 50000 still has to
be implemented and verified by the selected QGC/mission-client build.

## Mode matrix

Mode availability is shape-specific. A stable Quad uses the normal PX4
rotary-wing support matrix: every mode otherwise supported and allowed by PX4
is allowed by the hybrid shape gate. A stable Rover permits only the following
navigation states through the hybrid shape gate:

| User-facing mode | PX4 navigation state | Quad | Rover |
| --- | --- | :---: | :---: |
| Manual | `MANUAL` | allowed | allowed |
| Position | `POSCTL` | allowed | allowed |
| Mission | `AUTO_MISSION` | allowed | allowed |
| Hold/Loiter | `AUTO_LOITER` | allowed | allowed |
| Return | `AUTO_RTL` | allowed | allowed |
| Acro | `ACRO` | allowed | allowed |
| Offboard | `OFFBOARD` | allowed | allowed |
| Stabilized | `STAB` | allowed | allowed |
| Altitude | `ALTCTL` | allowed if normal Quad checks pass | denied |
| Every other PX4 navigation state | corresponding PX4 state | allowed only if normal Quad checks pass | denied |

Unsupported modes in a stable shape are denied. All mode changes during
`TRANSITIONING`, `UNKNOWN`, stale status, or `TRANSITION_FAULT` are temporarily
rejected, not queued. A transition request never implies a subsequent mode
change. Existing generic PX4 arming, estimator, position, mission, and mode
requirements remain in force after this shape gate.

## DDS and Rover Offboard contract

The uXRCE-DDS input topics are:

| Topic | ROS 2 type |
| --- | --- |
| `/fmu/in/offboard_control_mode` | `px4_msgs::msg::OffboardControlMode` |
| `/fmu/in/rover_velocity_setpoint` | `px4_msgs::msg::RoverVelocitySetpoint` |

Rover Offboard requires `OffboardControlMode.rover_velocity=true` and `RoverVelocitySetpoint`. `rover_velocity` must be the exact one enabled control
bit: `position`, `velocity`, `acceleration`, `attitude`, `body_rate`,
`thrust_and_torque`, and `direct_actuator` must all be false. Both the mode and
setpoint timestamps must be nonzero, not in the future, and fresh under
`COM_OF_LOSS_T`. Both setpoint fields must be finite.

The controller consumes Rover Offboard only in fresh, fault-free, stable
`DRIVING` state. Hybrid status must be no older than one second. After every
transition to Rover, the setpoint timestamp must be strictly newer than
`transition_completed_timestamp`; a pre-transition or same-timestamp setpoint
is stale and cannot move the rover. Transition, unknown, fault, stale status,
wrong/multiple Offboard bits, and stale/invalid input all gate the velocity
input off.

`OffboardControlMode.rover_velocity` and `RoverVelocitySetpoint` are project
extensions. Companion workspaces must regenerate/use `px4_msgs` from this exact
firmware message set; an upstream or older `px4_msgs` package does not satisfy
the type contract. `HybridVehicleStatus` is currently internal uORB/MAVLink
status, not a `/fmu/out/hybrid_vehicle_status` DDS publication.

For a ROS REP-103 FLU `geometry_msgs/Twist` input, the required mapping is:

```text
speed_body_x = linear.x
yaw_rate = -angular.z
```

Thus forward is positive `linear.x`; reverse remains a signed negative speed
and must not be clamped, absolutized, or converted into a separate direction
flag. ROS FLU positive yaw is left/counter-clockwise, while PX4 body FRD/NED
positive yaw rate is right/clockwise, hence the minus sign. Unused Twist axes
do not select other PX4 controllers and should be zero.

The setpoint is an input to the existing PX4 differential-Rover control chain.
It does not bypass or replace PX4 speed/yaw-rate limits, acceleration limiting,
PID controllers, steering/throttle generation, control allocation, arming, or
failsafe behavior. Companions should command physical m/s and rad/s and let the
configured PX4 limits and PIDs enforce the vehicle envelope.

## External deliverables and remaining validation

QGC integration must deliver generated MAVLink 2 bindings for the private
dialect; recognition of type 200 without treating it as VTOL; command-50000 UI
and ACK lifecycle handling using `result_param2`; status/fault presentation;
the mode matrix and no-queue behavior; and mission editor/transfer support for
the exact command parameters. A companion integration must deliver the matching
generated dialect if it uses MAVLink, regenerated matching `px4_msgs`, the
exact-one-bit Rover Offboard publisher, the FLU-to-FRD sign conversion, signed
reverse preservation, freshness/republication behavior, and fault/transition
gating.

Software unit tests and firmware compilation do not constitute QGC, ROS,
mission-transfer, live-link, or physical validation. Remaining physical
validation includes landed-only rejection on the installed land detector;
Quad/Rover mechanism completion and every fault path on the installed PWM/HX8
and sensor hardware; wheel direction including signed reverse; yaw sign;
configured speed/yaw/acceleration limits and PID response; Offboard timeout and
post-transition freshness; full mode and mission behavior; MAVLink 2 ACK/status
correlation over the real telemetry link; and end-to-end QGC and companion
interoperability with the pinned generated artifacts.

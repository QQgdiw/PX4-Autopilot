# Independent Quad-Rover Integration Design

**Date:** 2026-07-21

## Purpose

Replace the current VTOL facade with an independent Quad-Rover vehicle model.
The vehicle must expose its real hybrid semantics to QGroundControl (QGC) and
the companion computer, preserve PX4's native multicopter and differential
Rover control paths, and add a Rover Offboard velocity interface equivalent to
ROS 2 `/cmd_vel` forward speed plus yaw rate.

This design applies only to `zeroone_x6_hybrid`.

## Non-goals

- Do not support fixed-wing or VTOL flight modes.
- Do not make a mode request implicitly move the transformation mechanism.
- Do not bypass PX4's Rover speed and yaw-rate closed loops with direct wheel
  commands.
- Do not introduce ROS message types into PX4 firmware.

## Vehicle Identity And MAVLink

### Project dialect

Add a MAVLink 2 project dialect named `hybrid_vehicle`, derived from the
current `development` dialect. The PX4 hybrid board, QGC, and companion
computer use the same dialect-generated bindings.

The dialect owns these project-private definitions:

| Definition | Value | Meaning |
| --- | ---: | --- |
| `MAV_TYPE_QUAD_ROVER` | 200 | Static vehicle identity in `HEARTBEAT.system_type` |
| `MAV_CMD_DO_HYBRID_TRANSITION` | 50000 | Explicit mechanism transition command and mission item |
| `HYBRID_VEHICLE_STATUS` | project MAVLink 2 message | Machine-readable hybrid state and diagnostics |

These values are intentionally private. They must not be added to upstream
`common.xml` as unregistered standard values. The dialect changes the generated
`MAV_TYPE_ENUM_END`, so PX4 accepts `MAV_TYPE=200` rather than resetting it to
zero. This requires maintaining the dialect in the MAVLink dependency and
regenerating the QGC and companion bindings. Generic ground stations that do
not know the dialect may identify the vehicle as unknown.

The airframe default changes from standard VTOL type 22 to
`MAV_TYPE_QUAD_ROVER`. `HEARTBEAT.system_type` stays static through all shape
changes. `EXTENDED_SYS_STATE` must no longer report a VTOL or fixed-wing state;
it retains the real landed indication only.

### Transition command

`MAV_CMD_DO_HYBRID_TRANSITION` is the only external or mission path that can
request a shape change.

| Parameter | Value |
| --- | --- |
| `param1` | `1`: target Quad, `2`: target Rover |
| `param2..7` | Reserved; no force, quad-chute, or VTOL semantics |

The command has asynchronous acknowledgement semantics:

| Condition | `COMMAND_ACK` result |
| --- | --- |
| Invalid target or reserved parameter use | `DENIED` |
| Land detector false/stale, or opposite transition active | `TEMPORARILY_REJECTED` |
| Existing transformation fault | `DENIED` |
| Target already stable | `ACCEPTED` |
| Transition started | `IN_PROGRESS` |
| Target confirmed | `ACCEPTED` |
| Timeout, sensor, actuator, or stall failure | `FAILED` |

Each accepted request receives a monotonically increasing transformation
sequence in `result_param2`. The sequence also appears in the status message.
This prevents stale acknowledgements or status samples from being associated
with a newer request.

### Hybrid status message

`HYBRID_VEHICLE_STATUS` mirrors the public parts of
`hybrid_vehicle_status`: sequence, current and target shapes, transition
elapsed time, fault reason, sensor source, actuator backend and health,
protection flags, normalized position, and state flags.

State flags include: sensor enabled, position valid, endpoint confirmed,
actuator online, healthy, and configuration verified, `landed`, and land
detection freshness. The stream sends immediately on a state change and at a
low steady-state rate. It is the authority for QGC and companion shape display;
neither endpoint infers shape from `vehicle_type` or VTOL fields.

## PX4 Identity And Mode Routing

### Remove VTOL coupling

The current `MAV_TYPE=22`, `HYBR_QUAD_ROV && is_vtol()` inference,
`vtol_vehicle_status` dependency, and standard
`VEHICLE_CMD_DO_VTOL_TRANSITION` listener are replaced. A dedicated hybrid
identity check derives from `MAV_TYPE_QUAD_ROVER`, not from `is_vtol()`.

`vehicle_status.is_vtol` is always false for this vehicle. `vehicle_type` is
set from fresh `hybrid_vehicle_status` only when the shape is stable:

- Quad -> `VEHICLE_TYPE_ROTARY_WING`
- Rover -> `VEHICLE_TYPE_ROVER`
- Transitioning, unknown, or fault -> no active physical control type

The hybrid output gate remains the only selector between native multicopter
and native Rover actuator paths.

### Mode capability matrix

QGC displays only modes appropriate to the confirmed stable shape. Commander
enforces the same table independently.

| Mode | Quad | Rover |
| --- | --- | --- |
| Manual, Acro, Stabilized, Position, Hold, Mission, RTL, Offboard | Allowed | Allowed |
| Altitude, Position Slow, Takeoff, Land, Precision Land, Orbit, Follow Me | Allowed | Rejected |
| Fixed-wing and VTOL modes | Rejected | Rejected |

Rover Hold stops and holds at the current location only with valid positioning.
Rover RTL drives to Home and stops. Neither mode changes shape or initiates a
takeoff or landing.

### Transition gate

Every transition request, regardless of source, requires a fresh
`vehicle_land_detected` sample with `landed=true`. Local altitude is not used.
The land detector normally publishes at 1 Hz; a missing timestamp or a stale
sample is a denial, not a fallback to height-based protection.

Transitions preserve the current navigation state but impose an actuator gate:

- During transition all propulsion and wheel outputs are invalid/disabled.
- New mode requests are rejected while transitioning, unknown, or faulted.
- Requests are not queued.
- On stable completion, the retained mode resumes on the target shape's native
  controller chain.
- Manual input must be fresher than the completion timestamp.
- Offboard input must be fresher than the completion timestamp.
- Auto, Mission, and RTL retain their internal context and resume only after
  the confirmed stable shape is available.
- A transformation fault disables outputs and prevents recovery until the
  existing explicit fault-clear path succeeds and normal mode requirements are
  met again.

## Mission

The MAVLink mission parser accepts `MAV_CMD_DO_HYBRID_TRANSITION` as a project
mission item. Navigator and MissionBlock publish the corresponding vehicle
command and wait for a fresh matching `transition_sequence` whose stable shape
matches the requested target.

The mission does not advance for stale status, an opposite target, or a
transition fault. It never skips the item or attempts an automatic alternate
shape. Fault handling remains visible through the command acknowledgement and
`HYBRID_VEHICLE_STATUS` for operator intervention.

Mission waypoints before and after an explicit transition keep using the
existing Navigator outputs. The stable `vehicle_type` selects the native
multicopter or Rover control path; no VTOL mission item is used.

## Rover Offboard Velocity Interface

### Inputs

Add the following uORB messages and DDS inputs:

```text
OffboardControlMode.rover_velocity
/fmu/in/rover_velocity_setpoint (RoverVelocitySetpoint)
  speed_body_x  # m/s, positive forward, negative reverse
  yaw_rate      # rad/s, PX4 body FRD/NED convention
```

The companion keeps ROS 2 `/cmd_vel` as its public interface. It converts the
standard ROS body FLU request as follows:

```text
speed_body_x = cmd_vel.linear.x
yaw_rate     = -cmd_vel.angular.z
```

The sign inversion converts ROS positive left/CCW yaw around up to PX4 positive
yaw around down. `linear.y`, `linear.z`, `angular.x`, and `angular.y` are not
supported by a differential Rover and are not forwarded.

### Control ownership

`rover_velocity=true` is a dedicated Offboard control selection. Commander
sets Offboard, velocity, rates, and allocation control flags, but not position,
altitude, or attitude flags. It is accepted only in a stable Rover shape.

The two native differential loops run concurrently with disjoint output
ownership:

```text
RoverVelocitySetpoint.speed_body_x
  -> DifferentialVelControl
  -> existing speed PID, limits, and acceleration shaping
  -> RoverThrottleSetpoint

RoverVelocitySetpoint.yaw_rate
  -> DifferentialRateControl
  -> existing yaw-rate PID, yaw acceleration limits, and wheel-track mapping
  -> RoverSteeringSetpoint
```

The existing world-frame `TrajectorySetpoint.velocity` path remains available
only for its current legacy velocity semantics. It is not combined with the
new mode. The new mode does not create an attitude or bearing setpoint and does
not let any other controller publish the active throttle or steering command.

### Freshness And loss handling

`OffboardControlMode.rover_velocity` and `RoverVelocitySetpoint` both need
fresh timestamps. The controller checks the command timestamp locally and
publishes zero throttle and zero steering while resetting its integrators on a
stale, invalid, wrong-shape, or pre-transition-epoch sample. This gives an
immediate stop before Commander finishes its normal Offboard-loss state
transition.

Commander adds `rover_velocity` to Offboard availability and requires the
measurements used by the native speed and yaw-rate controllers. The existing
`COM_OF_LOSS_T` and configured Offboard loss action remain the sole global
watchdog policy.

## Verification

### Unit and source-contract tests

- Dialect generation and `MAV_TYPE` validation.
- Command parsing, acknowledgement result, sequence correlation, and land
  detector freshness gate.
- Shape-specific mode capability table and transition mode-request rejection.
- Mission transition completion only for the matching sequence and stable
  target; fault never advances the mission.
- ROS FLU to PX4 FRD yaw-rate sign conversion.
- Positive and negative speed, positive and negative yaw rate, simultaneous
  speed/yaw command, controller exclusivity, limits, stale input stop, and
  post-transition freshness epoch.

### Build and bench acceptance

Only run `make zeroone_x6_hybrid`. A successful build is not hardware
validation. Bench testing must separately cover QGC status/mode display,
explicit transition, cross-shape mission items, Rover Hold/RTL, `cmd_vel`
forward/reverse/turn sign, Offboard loss, and transition-fault inhibition.

## Residual risk

Private MAVLink definitions require synchronized PX4, QGC, and companion
updates. A generic GCS will not understand the project vehicle type or hybrid
status. Hardware testing remains required for all transformation and wheel
motion safety behavior.

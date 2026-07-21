# Independent Quad-Rover Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the hybrid vehicle VTOL facade with an independent Quad-Rover MAVLink identity and mode model, and add closed-loop Rover Offboard body speed plus yaw-rate control.

**Architecture:** The `hybrid_vehicle` MAVLink 2 dialect owns the private vehicle type, explicit transition command, and hybrid status message. `hybrid_vehicle_control` remains the transformation and actuator-output authority; Commander derives active locomotion and mode availability from fresh hybrid status, while Navigator waits for the custom mission transition item. A dedicated Rover velocity input selects only the native differential speed and yaw-rate loops, preserving their existing output ownership and safety limits.

**Tech Stack:** PX4 v1.16.1, C++17, uORB, MAVLink 2 C dialect generation, uXRCE-DDS, GoogleTest, CTest, NuttX `zeroone_x6_hybrid` build.

## Global Constraints

- Work only on `change1_v1.16.1` and only build `zeroone_x6_hybrid`.
- Do not build `zeroone_x6_default` or run it in parallel with the hybrid build.
- Preserve PX4 native multicopter and differential Rover controller chains; `hybrid_vehicle_control` only gates transformation and selects final actuator output.
- Do not use fixed-wing/VTOL command, status, or control semantics for the Quad-Rover vehicle. Do not remove upstream VTOL support for non-hybrid PX4 vehicles.
- Transformation requires a fresh `vehicle_land_detected.landed=true` sample. Altitude is not a fallback gate.
- A transition fault has priority over RC/debug input: final servo and propulsion output remain invalid, and no mode request is queued.
- QGC and the companion computer are external deliverables. PX4 supplies a versioned dialect and protocol document; do not add ROS message dependencies to firmware.
- ROS 2 `/cmd_vel` uses body FLU: `linear.x` maps directly to PX4 speed, and `angular.z` is negated for PX4 FRD/NED yaw rate.
- Use `make zeroone_x6_hybrid` for firmware verification. Report build and physical bench results separately.

---

## File Structure

| Path | Responsibility |
| --- | --- |
| `src/modules/mavlink/mavlink/message_definitions/v1.0/hybrid_vehicle.xml` (MAVLink submodule) | Private MAVLink 2 type, command, and status wire contract. |
| `boards/zeroone/x6/hybrid.px4board` | Select the private dialect for the hybrid target only. |
| `ROMFS/px4fmu_common/init.d/airframes/22001_quad_rover` | Use the private static MAV type and remove VTOL airframe description. |
| `msg/HybridVehicleStatus.msg` | Add transition sequence, command result, reject reason, and land-gate observability. |
| `msg/RoverVelocitySetpoint.msg` | New body-forward speed plus yaw-rate input. |
| `msg/OffboardControlMode.msg` | Dedicated `rover_velocity` mode selection bit. |
| `src/modules/uxrce_dds_client/dds_topics.yaml` | Expose the new uORB input to the companion computer. |
| `src/lib/hybrid_control/HybridTransitionPolicy.*` | Pure command, land-gate, mode-capability, and post-transition freshness policy. |
| `src/modules/hybrid_vehicle_control/*` | Consume the custom command, enforce land gating, publish sequence-aware status, and retire the height gate. |
| `src/modules/commander/*` | Identify the independent vehicle, publish its stable locomotion type, and deny unavailable modes. |
| `src/modules/navigator/*`, `src/modules/mavlink/mavlink_mission.cpp` | Parse and complete custom hybrid mission items without touching upstream VTOL behavior. |
| `src/modules/mavlink/streams/HYBRID_VEHICLE_STATUS.hpp` | Serialize the uORB hybrid status into the private MAVLink stream. |
| `src/modules/rover_differential/*` | Route the dedicated Offboard input into the existing velocity and rate PID loops. |
| `docs/hybrid/quad-rover-mavlink-dds-contract.md` | Companion/QGC dialect, command, state, `cmd_vel`, and mode contract. |

## Task 1: Add and Generate the Private MAVLink Dialect

**Files:**
- Create: `src/modules/mavlink/mavlink/message_definitions/v1.0/hybrid_vehicle.xml` (MAVLink submodule)
- Modify: `boards/zeroone/x6/hybrid.px4board:62`
- Modify: `ROMFS/px4fmu_common/init.d/airframes/22001_quad_rover:3-12`
- Modify: `src/modules/mavlink/mavlink_params.c:76-100`
- Test: generated `build/zeroone_x6_hybrid/mavlink/hybrid_vehicle/hybrid_vehicle.h`

**Interfaces:**
- Produces `MAV_TYPE_QUAD_ROVER = 200`, `MAV_CMD_DO_HYBRID_TRANSITION = 50000`, and `MAVLINK_MSG_ID_HYBRID_VEHICLE_STATUS = 60000`.
- Consumed by all later MAVLink, Commander, Mission, and airframe tasks.

- [ ] **Step 1: Add the dialect generation failure check**

Run from the MAVLink submodule after creating no file yet:

```bash
python3 -m pymavlink.tools.mavgen --lang C --wire-protocol 2.0 \
  --output /tmp/hybrid_vehicle_mavgen \
  message_definitions/v1.0/hybrid_vehicle.xml
```

Expected: FAIL because `hybrid_vehicle.xml` does not exist.

- [ ] **Step 2: Define the complete dialect contract**

Create `hybrid_vehicle.xml` with `development.xml` as its sole include and these exact extension entries:

```xml
<enum name="MAV_TYPE">
  <entry value="200" name="MAV_TYPE_QUAD_ROVER">
    <description>Vehicle with a multicopter flight shape and differential ground-rover shape.</description>
  </entry>
</enum>
<enum name="MAV_CMD">
  <entry value="50000" name="MAV_CMD_DO_HYBRID_TRANSITION" hasLocation="false" isDestination="false">
    <param index="1" label="Target Shape" enum="HYBRID_VEHICLE_SHAPE">1: Quad, 2: Rover.</param>
    <param index="2" reserved="true"/>
    <param index="3" reserved="true"/>
    <param index="4" reserved="true"/>
    <param index="5" reserved="true"/>
    <param index="6" reserved="true"/>
    <param index="7" reserved="true"/>
  </entry>
</enum>
<enum name="HYBRID_VEHICLE_SHAPE">
  <entry value="0" name="HYBRID_VEHICLE_SHAPE_NONE"/>
  <entry value="1" name="HYBRID_VEHICLE_SHAPE_QUAD"/>
  <entry value="2" name="HYBRID_VEHICLE_SHAPE_ROVER"/>
</enum>
```

Define these exact state and fault enum values in the same dialect:

```text
HYBRID_VEHICLE_STATE_QUAD=0, TRANSITIONING=1, ROVER=2, UNKNOWN=3, TRANSITION_FAULT=4
HYBRID_VEHICLE_FAULT_NONE=0, NO_SENSOR=1, SENSOR_CONFLICT=2, SENSOR_TIMEOUT=3,
TRANSITION_TIMEOUT=4, INVALID_SERVO_CONFIG=5, INVALID_CONFIGURATION=6, STALL=7,
ACTUATOR_COMMUNICATION=8, ACTUATOR_PROTECTION=9, ACTUATOR_CONFIG_MISMATCH=10,
ACTUATOR_COMMAND_REJECTED=11
```

Define `HYBRID_VEHICLE_STATUS_FLAGS` as a `uint16` bitmask:

```text
SENSORS_ENABLED=1, POSITION_CONFIRMED=2, POSITION_VALID=4, ACTUATOR_ONLINE=8,
ACTUATOR_HEALTHY=16, ACTUATOR_CONFIG_VERIFIED=32, LANDED=64,
LAND_DETECTION_FRESH=128
```

Add `HYBRID_VEHICLE_STATUS` with message id `60000`, fields in this wire order:

```xml
<field type="uint64_t" name="timestamp" units="us">PX4 boot time.</field>
<field type="uint32_t" name="transition_sequence">Accepted transition sequence.</field>
<field type="uint32_t" name="transition_elapsed_ms">Elapsed time for active transition.</field>
<field type="float" name="position_normalized">Mechanism position, NaN if unavailable.</field>
<field type="uint8_t" name="current_state" enum="HYBRID_VEHICLE_STATE"/>
<field type="uint8_t" name="target_state" enum="HYBRID_VEHICLE_SHAPE"/>
<field type="uint8_t" name="fault_reason" enum="HYBRID_VEHICLE_FAULT"/>
<field type="uint8_t" name="command_result" enum="MAV_RESULT"/>
<field type="uint8_t" name="sensor_source"/>
<field type="uint8_t" name="actuator_backend"/>
<field type="uint8_t" name="actuator_protection_flags"/>
<field type="uint16_t" name="flags" enum="HYBRID_VEHICLE_STATUS_FLAGS"/>
```

Define state, fault, backend, source, and bitmask values to exactly mirror the corresponding public uORB constants from `HybridVehicleStatus.msg`; do not create a second semantic mapping.

- [ ] **Step 3: Select the dialect and independent airframe type**

Set the hybrid board configuration to:

```text
CONFIG_MAVLINK_DIALECT="hybrid_vehicle"
```

Change the airframe header from VTOL wording to Quad-Rover wording and set:

```sh
param set-default MAV_TYPE 200
```

Add value `200` and its description to the `MAV_TYPE` parameter documentation. Do not modify the default board dialect or any non-hybrid airframe.

- [ ] **Step 4: Run the dialect generation check**

Run:

```bash
cd src/modules/mavlink/mavlink
python3 -m pymavlink.tools.mavgen --lang C --wire-protocol 2.0 \
  --output /tmp/hybrid_vehicle_mavgen \
  message_definitions/v1.0/hybrid_vehicle.xml
test -f /tmp/hybrid_vehicle_mavgen/hybrid_vehicle/hybrid_vehicle.h
```

Expected: exit 0 and generated constants for type 200, command 50000, and message 60000.

- [ ] **Step 5: Commit the dialect boundary**

Commit the MAVLink submodule change first, then stage its updated pointer in PX4 with the airframe and board changes:

```bash
git -C src/modules/mavlink/mavlink add message_definitions/v1.0/hybrid_vehicle.xml
git -C src/modules/mavlink/mavlink commit -m "feat[mavlink]: add hybrid vehicle dialect"
git add boards/zeroone/x6/hybrid.px4board ROMFS/px4fmu_common/init.d/airframes/22001_quad_rover src/modules/mavlink/mavlink_params.c src/modules/mavlink/mavlink
git commit -m "feat[hybrid]: select independent vehicle dialect"
```

## Task 2: Add uORB and DDS Contracts Before Consumers

**Files:**
- Create: `msg/RoverVelocitySetpoint.msg`
- Modify: `msg/OffboardControlMode.msg`
- Modify: `msg/HybridVehicleStatus.msg`
- Modify: `msg/CMakeLists.txt`
- Modify: `src/modules/uxrce_dds_client/dds_topics.yaml`
- Test: generated uORB headers and generated `px4_msgs` interface package

**Interfaces:**
- Produces `rover_velocity_setpoint_s` and `offboard_control_mode_s::rover_velocity`.
- Extends `hybrid_vehicle_status_s` with `transition_sequence`, `transition_completed_timestamp`, `command_result`, `command_reject_reason`, `landed`, and `land_detection_fresh`.

- [ ] **Step 1: Add the message-contract compile failure test**

Add a temporary include in the Rover differential unit target for:

```cpp
#include <uORB/topics/rover_velocity_setpoint.h>
```

Run its target configuration before adding the message. Expected: generated-header failure because the topic does not exist. Revert the temporary include before Step 2.

- [ ] **Step 2: Define exact uORB fields**

Create `RoverVelocitySetpoint.msg`:

```text
uint64 timestamp
float32 speed_body_x # m/s, body forward positive, body reverse negative
float32 yaw_rate     # rad/s, PX4 body FRD/NED positive convention
```

Append, never reorder, this field to `OffboardControlMode.msg`:

```text
bool rover_velocity # differential Rover body speed and yaw-rate offboard control
```

Append these fields and constants to `HybridVehicleStatus.msg`:

```text
uint8 COMMAND_RESULT_NONE = 255
uint8 REJECT_NONE = 0
uint8 REJECT_INVALID_TARGET = 1
uint8 REJECT_RESERVED_PARAMETER = 2
uint8 REJECT_NOT_LANDED = 3
uint8 REJECT_LAND_DETECTOR_STALE = 4
uint8 REJECT_OPPOSITE_TRANSITION = 5
uint8 REJECT_TRANSFORMATION_FAULT = 6
uint8 REJECT_UNKNOWN_STATE = 7
uint32 transition_sequence
uint64 transition_completed_timestamp
uint8 command_result
uint8 command_reject_reason
bool landed
bool land_detection_fresh
```

Add `RoverVelocitySetpoint.msg` to `msg/CMakeLists.txt`, then expose it as:

```yaml
- topic: /fmu/in/rover_velocity_setpoint
  type: px4_msgs::msg::RoverVelocitySetpoint
```

in the uXRCE subscriber list. Regenerate the companion `px4_msgs` package from this exact PX4 message set; do not hand-write a divergent IDL.

- [ ] **Step 3: Regenerate and inspect contracts**

Run the normal target configure/generate command and verify:

```bash
rg "rover_velocity" build/zeroone_x6_hybrid/uORB/topics/offboard_control_mode.h
test -f build/zeroone_x6_hybrid/uORB/topics/rover_velocity_setpoint.h
```

Expected: the bit and new topic header exist. Verify the generated `px4_msgs` package includes `RoverVelocitySetpoint` and the appended `rover_velocity` field.

- [ ] **Step 4: Commit the public uORB/DDS contract**

```bash
git add msg/RoverVelocitySetpoint.msg msg/OffboardControlMode.msg msg/HybridVehicleStatus.msg msg/CMakeLists.txt src/modules/uxrce_dds_client/dds_topics.yaml
git commit -m "feat[hybrid]: add rover velocity offboard contract"
```

## Task 3: Add Tested Hybrid Transition and Mode Policy

**Files:**
- Create: `src/lib/hybrid_control/HybridTransitionPolicy.hpp`
- Create: `src/lib/hybrid_control/HybridTransitionPolicy.cpp`
- Create: `src/lib/hybrid_control/HybridTransitionPolicyTest.cpp`
- Modify: `src/lib/hybrid_control/CMakeLists.txt`

**Interfaces:**
- Consumes scalar land status, hybrid state, requested target, current target, and timestamps.
- Produces `TransitionDecision { bool start; HybridTarget target; CommandResult ack_result; RejectReason reject_reason; }` and pure `modeAllowedForShape()` / `offboardInputFreshAfter()` predicates.
- Used by `hybrid_vehicle_control`, Commander, Mission tests, and Rover Offboard gating.

- [ ] **Step 1: Write failing policy tests**

Create tests with these exact assertions:

```cpp
TEST(HybridTransitionPolicy, DeniesStaleOrUnlandedRequests)
{
    EXPECT_EQ(decideTransition({true, false, false, HybridState::Flying, HybridTarget::None}).reject_reason,
              RejectReason::LandDetectorStale);
    EXPECT_EQ(decideTransition({true, true, false, HybridState::Flying, HybridTarget::None}).reject_reason,
              RejectReason::NotLanded);
}

TEST(HybridTransitionPolicy, AcceptsOnlyValidStableTarget)
{
    EXPECT_TRUE(decideTransition({true, true, false, HybridState::Flying, HybridTarget::Driving}).start);
    EXPECT_EQ(decideTransition({true, true, false, HybridState::Flying, HybridTarget::Flying}).ack_result,
              CommandResult::Accepted);
}

TEST(HybridTransitionPolicy, RejectsModesOutsideStableShape)
{
    EXPECT_FALSE(modeAllowedForShape(HybridState::Driving, NAVIGATION_STATE_ALTCTL));
    EXPECT_TRUE(modeAllowedForShape(HybridState::Driving, NAVIGATION_STATE_OFFBOARD));
    EXPECT_FALSE(modeAllowedForShape(HybridState::TransitionToRover, NAVIGATION_STATE_MANUAL));
}

TEST(HybridTransitionPolicy, RequiresInputAfterCompletionEpoch)
{
    EXPECT_FALSE(offboardInputFreshAfter(99, 100, 150));
    EXPECT_TRUE(offboardInputFreshAfter(101, 100, 150));
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run:

```bash
ctest --test-dir build/zeroone_x6_hybrid -R HybridTransitionPolicy --output-on-failure
```

Expected: FAIL because the policy library and test target do not exist.

- [ ] **Step 3: Implement the narrow policy API**

Use these definitions, keeping hardware subscriptions out of the library:

```cpp
enum class RejectReason : uint8_t { None, InvalidTarget, ReservedParameter, NotLanded, LandDetectorStale, OppositeTransition, TransformationFault, UnknownState };
enum class CommandResult : uint8_t { Accepted = 0, TemporarilyRejected = 1, Denied = 2, Failed = 4, InProgress = 5 };
struct TransitionRequest { bool land_sample_fresh; bool landed; bool faulted; HybridState state; HybridTarget requested; HybridTarget active_target; };
struct TransitionDecision { bool start; HybridTarget target; CommandResult ack_result; RejectReason reject_reason; };
TransitionDecision decideTransition(const TransitionRequest &request);
bool modeAllowedForShape(HybridState state, uint8_t nav_state);
bool offboardInputFreshAfter(uint64_t input_timestamp, uint64_t completion_timestamp, uint64_t now);
```

Use `MAV_RESULT_TEMPORARILY_REJECTED` for stale/unlanded/opposite-transition requests, `MAV_RESULT_DENIED` for faults/invalid requests, and `MAV_RESULT_ACCEPTED` for an already-stable target.

- [ ] **Step 4: Run the focused policy tests**

Run the command from Step 2. Expected: all `HybridTransitionPolicy` tests pass.

- [ ] **Step 5: Commit the policy boundary**

```bash
git add src/lib/hybrid_control/HybridTransitionPolicy.hpp src/lib/hybrid_control/HybridTransitionPolicy.cpp src/lib/hybrid_control/HybridTransitionPolicyTest.cpp src/lib/hybrid_control/CMakeLists.txt
git commit -m "feat[hybrid]: add transition and mode policy"
```

## Task 4: Convert Hybrid Control to the Independent Command and Land Gate

**Files:**
- Modify: `src/modules/hybrid_vehicle_control/hybrid_vehicle_control.hpp`
- Modify: `src/modules/hybrid_vehicle_control/hybrid_vehicle_control.cpp`
- Modify: `src/modules/hybrid_vehicle_control/hybrid_vehicle_control_params.c`
- Modify: `ROMFS/px4fmu_common/init.d/airframes/22001_quad_rover`
- Test: `src/lib/hybrid_control/HybridTransitionPolicyTest.cpp`

**Interfaces:**
- Consumes `vehicle_command`, `vehicle_land_detected`, and the Task 3 policy.
- Produces sequence-aware `hybrid_vehicle_status` and exactly one command acknowledgement per external transition request.

- [ ] **Step 1: Extend failing tests for command lifecycle**

Add policy-level lifecycle cases:

```cpp
TEST(HybridTransitionPolicy, OppositeTransitionIsTemporarilyRejected)
{
    const auto result = decideTransition({true, true, false, HybridState::TransitionToRover,
                                          HybridTarget::Flying, HybridTarget::Driving});
    EXPECT_FALSE(result.start);
    EXPECT_EQ(result.ack_result, MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_EQ(result.reject_reason, RejectReason::OppositeTransition);
}
```

Run the focused target; expected FAIL until the policy handles an active opposite target.

- [ ] **Step 2: Replace the old input and parameter**

In the module header, replace the `vehicle_local_position` subscription with:

```cpp
uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
vehicle_land_detected_s _vehicle_land_detected{};
```

Remove `HYBRID_MAX_Z` from the header parameter binding, parameter definition, and airframe defaults. Do not leave a deprecated parameter that has no effect.

- [ ] **Step 3: Implement custom-command lifecycle**

Filter only `vehicle_command_s::VEHICLE_CMD_DO_HYBRID_TRANSITION`. Validate `param1` as Quad/Rover and require `param2..7` to be zero or NaN. Use the Task 3 policy with a land sample freshness window of `2_s` (the land detector normally republishes at least every second). Route every existing request source--custom MAVLink command, Mission command, and RC transfer switch--through that same policy before it can call `_transformation.request()`. Both Quad and Rover targets require the fresh landed result; no source may retain the former height-only or Quad-bypass behavior.

Store this exact pending-request record on a started external request:

```cpp
struct PendingTransitionAck {
    bool active{false};
    uint32_t sequence{0};
    uint32_t command{0};
    uint8_t target_system{0};
    uint16_t target_component{0};
};
```

Publish `IN_PROGRESS` only when a new transition starts. Publish terminal `ACCEPTED` or `FAILED` once when the matching sequence reaches a stable target or fault. Publish immediate rejection for invalid/unsafe requests. Internal Mission commands must start the same state machine but do not require an external terminal acknowledgement.

- [ ] **Step 4: Publish observable gate and lifecycle state**

Set every newly added `hybrid_vehicle_status_s` field on every publication:

```cpp
status.transition_sequence = _transition_sequence;
status.transition_completed_timestamp = _transition_complete_time;
status.command_result = _last_command_result;
status.command_reject_reason = _last_command_reject_reason;
status.landed = _vehicle_land_detected.landed;
status.land_detection_fresh = timestamp_fresh(_vehicle_land_detected.timestamp, now, 2_s);
```

Increment `_transition_sequence` only after a request is accepted to start. Record `_transition_complete_time` only upon confirmed stable completion; later Offboard and manual gates compare their input timestamps against it. In `publish_motor_outputs()`, reject any cached multicopter or Rover actuator message whose `timestamp` is not later than `_transition_complete_time`; this prevents pre-transition manual, auto, or Offboard output from being selected after shape completion.

- [ ] **Step 5: Run focused tests and source checks**

Run:

```bash
ctest --test-dir build/zeroone_x6_hybrid -R HybridTransitionPolicy --output-on-failure
rg "VEHICLE_CMD_DO_VTOL_TRANSITION|HYBRID_MAX_Z" src/modules/hybrid_vehicle_control ROMFS/px4fmu_common/init.d/airframes/22001_quad_rover
```

Expected: policy tests pass; the search produces no active hybrid-control/airframe use of either obsolete symbol.

- [ ] **Step 6: Commit the transformation command migration**

```bash
git add src/modules/hybrid_vehicle_control src/lib/hybrid_control ROMFS/px4fmu_common/init.d/airframes/22001_quad_rover
git commit -m "feat[hybrid]: gate transformation by landed state"
```

## Task 5: Make Commander a True Quad-Rover Mode Authority

**Files:**
- Modify: `src/modules/commander/commander_helper.cpp`
- Modify: `src/modules/commander/commander_helper.h`
- Modify: `src/modules/commander/Commander.cpp`
- Modify: `src/modules/commander/Commander.hpp`
- Modify: `src/modules/commander/HybridStatusGuard.hpp`
- Modify: `src/modules/commander/CommanderHybridStatusTest.cpp`
- Modify: `src/modules/commander/ModeManagementTest.cpp`

**Interfaces:**
- Consumes private system type 200 and fresh `hybrid_vehicle_status`.
- Produces `is_quad_rover=true`, `is_vtol=false`, stable physical `vehicle_type`, and mode allow/deny decisions using Task 3 policy.

- [ ] **Step 1: Write failing Commander tests**

Add tests that construct `vehicle_status_s` with system type 200 and fresh status:

```cpp
TEST(CommanderHybridStatus, IndependentIdentityIsNotVtol)
{
    vehicle_status_s status{};
    status.system_type = MAV_TYPE_QUAD_ROVER;
    EXPECT_TRUE(commander::is_quad_rover(status));
    EXPECT_FALSE(commander::is_vtol(status));
}

TEST(CommanderHybridStatus, RoverRejectsAltitudeAndTransitionRejectsAllModes)
{
    EXPECT_FALSE(commander::hybridModeAllowed(HYBRID_STATE_DRIVING, NAVIGATION_STATE_ALTCTL));
    EXPECT_TRUE(commander::hybridModeAllowed(HYBRID_STATE_DRIVING, NAVIGATION_STATE_AUTO_RTL));
    EXPECT_FALSE(commander::hybridModeAllowed(HYBRID_STATE_TRANSITIONING, NAVIGATION_STATE_MANUAL));
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run:

```bash
ctest --test-dir build/zeroone_x6_hybrid -R CommanderHybridStatus --output-on-failure
```

Expected: FAIL because the independent type helper and mode guard do not exist.

- [ ] **Step 3: Remove hybrid dependence on VTOL state**

Define `is_quad_rover()` from `system_type == MAV_TYPE_QUAD_ROVER`. In `Commander::updateParameters()`, set `is_quad_rover` from that helper, retain the static system type, set `is_vtol=false`, and remove the custom `HYBR_QUAD_ROV && is_vtol()` condition. Do not alter generic `is_vtol()` behavior for standard vehicle types.

For a hybrid vehicle, do not call the custom VTOL remapping path in `vtolStatusUpdate()`. Instead use fresh `hybrid_vehicle_status` to set `vehicle_type` only for stable Flying/Driving states. In transition, unknown, or fault, do not report fixed-wing and do not enable a physical controller chain.

Add `VEHICLE_CMD_DO_HYBRID_TRANSITION` to Commander's existing "handled by other parts of the system" switch cases. Commander must not publish `UNSUPPORTED` or a duplicate acknowledgement: `hybrid_vehicle_control` is the sole acknowledgement owner for this command.

- [ ] **Step 4: Gate mode changes before UserModeIntention accepts them**

At the Commander command and action-request mode entry points, apply:

```cpp
if (_vehicle_status.is_quad_rover
    && !commander::hybridModeAllowed(_current_hybrid_state, desired_nav_state)) {
    answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED);
    return true;
}
```

Use `DENIED` for shape-stable but physically unsupported modes (for example Rover Altitude) and `TEMPORARILY_REJECTED` for transition, stale, unknown, or fault state. Apply the same guard to RC/action mode transitions so QGC cannot bypass it. Do not queue requests.

- [ ] **Step 5: Preserve resume gates**

Keep the requested navigation state through a valid transition, but expose `_transition_complete_time` through fresh hybrid status. The active output path must require post-completion manual or Offboard timestamps; autonomous Navigator modes retain their own context. Ensure fault state keeps the existing Commander LED and arming-check ownership.

- [ ] **Step 6: Run Commander tests and commit**

Run the test from Step 2 and `ctest --test-dir build/zeroone_x6_hybrid -R ModeManagement --output-on-failure`; expected: all selected tests pass.

```bash
git add src/modules/commander
git commit -m "feat[commander]: route independent quad-rover modes"
```

## Task 6: Add Custom Mission Transition Semantics

**Files:**
- Modify: `src/modules/navigator/navigation.h`
- Modify: `src/modules/mavlink/mavlink_mission.cpp`
- Modify: `src/modules/navigator/mission_block.h`
- Modify: `src/modules/navigator/mission_block.cpp`
- Modify: `src/modules/navigator/mission_base.cpp`
- Modify: `src/modules/navigator/mission.cpp`
- Modify: `src/modules/navigator/MissionFeasibility/FeasibilityChecker.cpp`
- Test: create `src/modules/navigator/HybridTransitionMissionTest.cpp`
- Modify: `src/modules/navigator/CMakeLists.txt`

**Interfaces:**
- Produces `NAV_CMD_DO_HYBRID_TRANSITION = 50000` and mission completion based on fresh matching hybrid sequence and target state.
- Retains upstream `NAV_CMD_DO_VTOL_TRANSITION` behavior for non-hybrid PX4 configurations.

- [ ] **Step 1: Write failing mission-item tests**

Create tests covering the pure item acceptance predicate:

```cpp
TEST(HybridTransitionMission, CompletesOnlyMatchingStableSequence)
{
    EXPECT_FALSE(hybridTransitionMissionReached(7, HYBRID_VEHICLE_SHAPE_ROVER,
                                                makeHybridStatus(6, HYBRID_STATE_DRIVING, TARGET_DRIVING)));
    EXPECT_FALSE(hybridTransitionMissionReached(7, HYBRID_VEHICLE_SHAPE_ROVER,
                                                makeHybridStatus(7, HYBRID_STATE_TRANSITIONING, TARGET_DRIVING)));
    EXPECT_TRUE(hybridTransitionMissionReached(7, HYBRID_VEHICLE_SHAPE_ROVER,
                                               makeHybridStatus(7, HYBRID_STATE_DRIVING, TARGET_DRIVING)));
}

TEST(HybridTransitionMission, FaultNeverCompletesItem)
{
    EXPECT_FALSE(hybridTransitionMissionReached(7, HYBRID_VEHICLE_SHAPE_QUAD,
                                                makeHybridStatus(7, HYBRID_STATE_TRANSITION_FAULT, TARGET_FLYING)));
}
```

- [ ] **Step 2: Run the new mission test to verify it fails**

Run:

```bash
ctest --test-dir build/zeroone_x6_hybrid -R HybridTransitionMission --output-on-failure
```

Expected: FAIL because no custom navigation command or helper exists.

- [ ] **Step 3: Parse and store the custom item**

Add `NAV_CMD_DO_HYBRID_TRANSITION = MAV_CMD_DO_HYBRID_TRANSITION` to `navigation.h`. Add it to both MAVLink mission upload and download switch cases alongside other non-positional `DO` items. Validate `param1` is exactly 1 or 2 and reject unsupported reserved parameters at upload time.

- [ ] **Step 4: Publish once and wait for matching status**

Add a `hybrid_vehicle_status` subscription and fields to MissionBlock:

```cpp
uORB::SubscriptionData<hybrid_vehicle_status_s> _hybrid_status_sub{ORB_ID(hybrid_vehicle_status)};
uint32_t _hybrid_transition_sequence{0};
```

When the item is activated, snapshot the most recently seen `hybrid_vehicle_status.transition_sequence`, publish `vehicle_command.command = NAV_CMD_DO_HYBRID_TRANSITION` once, and do not republish on every Navigator cycle. Mark the item reached only when fresh status has the requested stable target and either (a) its sequence is greater than the snapshot, or (b) the requested target was already stable at activation. On a fault keep the item active and report the Mission failure/status indication; never advance or substitute a VTOL command. Navigator does not depend on an external command-ack subscription.

- [ ] **Step 5: Remove hybrid references to standard VTOL mission behavior**

For the hybrid command, do not call `handleVtolTransition()`, `set_vtol_transition_item()`, or use fixed-wing vehicle-type completion conditions. Keep these upstream paths untouched for true VTOL vehicles. Add the hybrid command to the same non-position/terrain-exclusion branches needed to prevent it being treated as a waypoint.

- [ ] **Step 6: Run mission tests and commit**

Run the command from Step 2 plus existing `MissionFeasibility` tests. Expected: all pass.

```bash
git add src/modules/navigator src/modules/mavlink/mavlink_mission.cpp
git commit -m "feat[navigator]: support hybrid transition mission items"
```

## Task 7: Stream Public Hybrid Status Over MAVLink

**Files:**
- Create: `src/modules/mavlink/streams/HYBRID_VEHICLE_STATUS.hpp`
- Modify: `src/modules/mavlink/mavlink_messages.cpp`
- Modify: `src/modules/mavlink/streams/EXTENDED_SYS_STATE.hpp`
- Test: build-time generated MAVLink symbols and stream source contract

**Interfaces:**
- Consumes `hybrid_vehicle_status_s`.
- Produces MAVLink `HYBRID_VEHICLE_STATUS` only on MAVLink 2 links using the private dialect.

- [ ] **Step 1: Add the stream source-contract failure check**

Run before adding the stream:

```bash
rg "MavlinkStreamHybridVehicleStatus" src/modules/mavlink
```

Expected: no matches.

- [ ] **Step 2: Implement a sequence-aware stream**

Create a stream with this shape:

```cpp
class MavlinkStreamHybridVehicleStatus : public MavlinkStream
{
public:
    static constexpr const char *get_name_static() { return "HYBRID_VEHICLE_STATUS"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HYBRID_VEHICLE_STATUS; }
    explicit MavlinkStreamHybridVehicleStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}
    bool send() override;
private:
    uORB::Subscription _hybrid_status_sub{ORB_ID(hybrid_vehicle_status)};
};
```

`send()` copies one status sample, packs every wire field defined in Task 1 without unit changes, and sends on publication changes. It must never synthesize VTOL state. Register the stream in `mavlink_messages.cpp` under the generated message macro. Update `EXTENDED_SYS_STATE` so a `MAV_TYPE_QUAD_ROVER` system leaves `vtol_state` undefined rather than advertising FW/VTOL.

- [ ] **Step 3: Verify stream registration and generated symbols**

Run:

```bash
rg "MavlinkStreamHybridVehicleStatus" src/modules/mavlink/mavlink_messages.cpp src/modules/mavlink/streams/HYBRID_VEHICLE_STATUS.hpp
rg "MAVLINK_MSG_ID_HYBRID_VEHICLE_STATUS" build/zeroone_x6_hybrid/mavlink/hybrid_vehicle
```

Expected: both generated and registered symbols are present.

- [ ] **Step 4: Commit the telemetry path**

```bash
git add src/modules/mavlink
git commit -m "feat[mavlink]: publish hybrid vehicle status"
```

## Task 8: Implement Native Rover `speed_body_x + yaw_rate` Offboard Control

**Files:**
- Modify: `src/modules/commander/ModeUtil/control_mode.cpp`
- Modify: `src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp`
- Modify: `src/modules/rover_differential/DifferentialVelControl/DifferentialVelControl.hpp`
- Modify: `src/modules/rover_differential/DifferentialVelControl/DifferentialVelControl.cpp`
- Modify: `src/modules/rover_differential/DifferentialRateControl/DifferentialRateControl.hpp`
- Modify: `src/modules/rover_differential/DifferentialRateControl/DifferentialRateControl.cpp`
- Test: create `src/lib/rover_control/RoverVelocityOffboardPolicyTest.cpp`
- Modify: `src/lib/rover_control/CMakeLists.txt`

**Interfaces:**
- Consumes `offboard_control_mode_s::rover_velocity` and `rover_velocity_setpoint_s`.
- Produces the existing `rover_throttle_setpoint_s` from speed PID and `rover_steering_setpoint_s` from yaw-rate PID, with no attitude controller owner.

- [ ] **Step 1: Write failing combined-mode tests**

Test pure PX4-side input validation and transition epoch handling:

```cpp
TEST(RoverVelocityOffboardPolicy, RequiresFreshPostTransitionInput)
{
    EXPECT_FALSE(roverVelocityInputUsable({99, 0.3f, 0.2f}, 100, 150));
    EXPECT_FALSE(roverVelocityInputUsable({120, NAN, 0.2f}, 100, 150));
    EXPECT_TRUE(roverVelocityInputUsable({120, -0.3f, 0.2f}, 100, 150));
}
```

Run the new CTest target. Expected: FAIL because helpers and target are absent.

- [ ] **Step 2: Add dedicated Commander selection and health checks**

In `getVehicleControlMode()`, add the `rover_velocity` branch before generic `velocity`:

```cpp
if (offboard_control_mode.rover_velocity) {
    vehicle_control_mode.flag_control_velocity_enabled = true;
    vehicle_control_mode.flag_control_rates_enabled = true;
    vehicle_control_mode.flag_control_allocation_enabled = true;
}
```

Do not set position, altitude, climb-rate, or attitude flags in this branch. In `OffboardChecks`, include `rover_velocity` in signal availability and require the velocity and angular-velocity estimates used by the Rover loops. Commander must deny entering this selection unless fresh hybrid status reports stable Rover.

Reject any `OffboardControlMode` sample that sets `rover_velocity` together with `position`, `velocity`, `acceleration`, `attitude`, `body_rate`, `thrust_and_torque`, or `direct_actuator`. This dedicated selection has exactly one valid control bit and cannot be interpreted through the existing priority chain.

- [ ] **Step 3: Route speed without a bearing target**

Add this subscription and cache to `DifferentialVelControl`:

```cpp
uORB::Subscription _rover_velocity_setpoint_sub{ORB_ID(rover_velocity_setpoint)};
rover_velocity_setpoint_s _rover_velocity_setpoint{};
```

When `rover_velocity` is selected, copy the input and use only
`speed_body_x` as the speed-loop setpoint. Do not compute a norm/bearing, do
not publish `rover_attitude_setpoint`, and do not let the legacy
`trajectory_setpoint` path run. Keep `RoverControl::speedControl()`, its
negative-speed handling, `RO_SPEED_LIM`, acceleration/deceleration, and
feasibility limiting against the current steering demand.

- [ ] **Step 4: Route yaw rate to the existing differential rate loop**

Add the same input subscription to `DifferentialRateControl`. When
`rover_velocity` is selected, publish:

```cpp
rover_rate_setpoint_s rate{};
rate.timestamp = _timestamp;
rate.yaw_rate_setpoint = _rover_velocity_setpoint.yaw_rate;
_rover_rate_setpoint_pub.publish(rate);
```

Keep `RoverControl::rateControl()` as the only producer of the steering speed
difference. Do not publish a direct throttle in this branch. Because the
dedicated Commander selection leaves attitude and position disabled,
`DifferentialAttControl` and `DifferentialPosControl` cannot overwrite the
active speed/rate owners.

- [ ] **Step 5: Add immediate stale-input stop**

Add `ParamFloat<px4::params::COM_OF_LOSS_T>` and a `hybrid_vehicle_status`
subscription to both differential controllers. Use `COM_OF_LOSS_T` as the
local maximum age and require `current_state == HYBRID_STATE_DRIVING`, zero
fault reason, and `setpoint.timestamp > transition_completed_timestamp`. For
stale, invalid, wrong-shape, or pre-transition-epoch input, reset the
respective PID/slew state and publish both exact zero setpoints:

```cpp
rover_throttle_setpoint.throttle_body_x = 0.f;
rover_throttle_setpoint.throttle_body_y = 0.f;
rover_steering_setpoint.normalized_speed_diff = 0.f;
```

This local stop is in addition to, not a replacement for, Commander Offboard
loss handling.

- [ ] **Step 6: Run Rover tests and commit**

Run:

```bash
ctest --test-dir build/zeroone_x6_hybrid -R "RoverControl|RoverVelocityOffboardPolicy" --output-on-failure
```

Expected: speed sign, yaw sign, post-transition epoch, and existing Rover
control tests pass.

```bash
git add src/modules/commander/ModeUtil/control_mode.cpp src/modules/commander/HealthAndArmingChecks/checks/offboardCheck.cpp src/modules/rover_differential src/lib/rover_control
git commit -m "feat[rover]: add body velocity offboard control"
```

## Task 9: Publish the Integration Contract and Verify the Complete Target

**Files:**
- Create: `docs/hybrid/quad-rover-mavlink-dds-contract.md`
- Modify: `README.md` only if it has a hybrid protocol entry point
- Test: source-contract script under `test/` if one is not already present

**Interfaces:**
- Documents exact dialect name, MAV type, command parameters, ACK lifecycle,
  mission item behavior, status message, ROS FLU mapping, DDS topics, allowed
  mode matrix, and external compatibility boundary.

- [ ] **Step 1: Write the documentation contract test first**

Create `test/hybrid_quad_rover_contract.sh` with required literal checks:

```sh
#!/usr/bin/env bash
set -euo pipefail
rg -q 'MAV_TYPE_QUAD_ROVER' docs/hybrid/quad-rover-mavlink-dds-contract.md
rg -q 'MAV_CMD_DO_HYBRID_TRANSITION' docs/hybrid/quad-rover-mavlink-dds-contract.md
rg -q '/fmu/in/rover_velocity_setpoint' docs/hybrid/quad-rover-mavlink-dds-contract.md
rg -q 'yaw_rate.*-.*angular.z' docs/hybrid/quad-rover-mavlink-dds-contract.md
```

Run it before writing the document. Expected: FAIL because the document is absent.

- [ ] **Step 2: Document the external contract**

The document must state all of the following verbatim or equivalently:

```text
Dialect: hybrid_vehicle
Vehicle type: MAV_TYPE_QUAD_ROVER (200)
Transition command: MAV_CMD_DO_HYBRID_TRANSITION (50000), param1 1=Quad, 2=Rover
ROS mapping: speed_body_x = linear.x; yaw_rate = -angular.z
Rover Offboard: OffboardControlMode.rover_velocity=true and RoverVelocitySetpoint
```

Include the command result table, status fields/flags, state-specific mode
matrix, landed-only conversion gate, no queued mode request rule, and the fact
that QGC/companion must use MAVLink 2 and the project dialect.

- [ ] **Step 3: Run focused tests and full firmware build serially**

Use a Linux-only PATH if WSL Anaconda protobuf is inherited:

```bash
export PATH=/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:/home/crocodile/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
ctest --test-dir build/zeroone_x6_hybrid -R "HybridTransitionPolicy|CommanderHybridStatus|ModeManagement|HybridTransitionMission|RoverControl|RoverVelocityOffboardPolicy" --output-on-failure
bash test/hybrid_quad_rover_contract.sh
make zeroone_x6_hybrid > build/zeroone_x6_hybrid/independent_quad_rover_build.log 2>&1
tail -n 40 build/zeroone_x6_hybrid/independent_quad_rover_build.log
```

Expected: every selected CTest passes, documentation contract exits 0, and
`make zeroone_x6_hybrid` exits 0. Do not claim physical validation from these
commands.

- [ ] **Step 4: Commit documentation and verification contract**

```bash
git add docs/hybrid/quad-rover-mavlink-dds-contract.md test/hybrid_quad_rover_contract.sh README.md
git commit -m "docs[hybrid]: document independent vehicle protocol"
```

## Task 10: Bench Acceptance (No Firmware Claim Without It)

**Files:**
- Modify: `docs/hybrid/quad-rover-mavlink-dds-contract.md`
- Test: physical bench matrix recorded in the document

**Interfaces:**
- Consumes the built hybrid firmware, QGC/companion dialect builds, real land detector, mechanism, M2006 wheels, and UART/PWM transformation backend.
- Produces explicit pass/fail evidence; it does not change software interfaces.

- [ ] **Step 1: Verify protocol identity and state on a MAVLink 2 link**

Check that QGC and companion decode `MAV_TYPE_QUAD_ROVER` and
`HYBRID_VEHICLE_STATUS`; verify no `MAV_CMD_DO_VTOL_TRANSITION` is sent or
accepted for mechanism control.

- [ ] **Step 2: Verify landed gate and all transition terminals**

With a fresh landed sample, command Quad->Rover and Rover->Quad; verify
`IN_PROGRESS` then matching-sequence `ACCEPTED`. With landed false, stale land
data, opposite active request, and a deliberately induced sensor/actuator
fault, verify the exact rejection/failure results and zero/NaN output gate.

- [ ] **Step 3: Verify state-specific modes and Mission**

In each stable shape, select every mode in its capability table. Confirm
unsupported modes are rejected, no mode is queued during transition, Rover
Hold stops at valid position, Rover RTL drives Home then stops, and a Mission
with explicit hybrid transition item continues only after the matching target
is stable.

- [ ] **Step 4: Verify Rover Offboard behavior and loss handling**

At the companion publish `rover_velocity=true` and `cmd_vel` cases:

```text
(linear.x, angular.z): (0.3, 0), (-0.3, 0), (0, 0.4), (0.3, 0.4)
```

Verify direction, yaw sign, speed/yaw limits, reverse, combined motion,
zero-command stop, stale topic stop, standard `COM_OF_LOSS_T` action, and that
pre-transition setpoints cannot move the Rover after a completed transition.

- [ ] **Step 5: Record only observed hardware evidence**

Append dated pass/fail results, firmware hash, QGC/companion dialect hashes,
test setup, and outstanding failures to the contract document. Commit only the
observed record:

```bash
git add docs/hybrid/quad-rover-mavlink-dds-contract.md
git commit -m "test[hybrid]: record quad-rover bench acceptance"
```

## Plan Self-Review

### Spec coverage

- Independent type/dialect, explicit command, status telemetry, and no VTOL
  facade: Tasks 1, 4, 5, and 7.
- Landed-only transition, asynchronous ACK, sequence, and fault safety: Tasks
  3 and 4.
- Shape-specific modes, preserved navigation state, no queued mode, and Rover
  Hold/RTL: Task 5.
- Mission custom item and matching-sequence completion: Task 6.
- Native Rover body speed plus yaw-rate Offboard, FLU/FRD sign, DDS, limits,
  fresh epoch, and loss handling: Tasks 2 and 8.
- QGC/companion protocol handoff, software verification, and hardware
  evidence: Tasks 9 and 10.

### Placeholder scan

The plan contains no unfinished implementation placeholders. External QGC and
companion modifications are intentionally out of this workspace and are
specified through the dialect and protocol contract.

### Type consistency

The same names are used throughout: `MAV_TYPE_QUAD_ROVER`,
`MAV_CMD_DO_HYBRID_TRANSITION`, `HYBRID_VEHICLE_STATUS`,
`RoverVelocitySetpoint`, `rover_velocity`, `speed_body_x`, and `yaw_rate`.

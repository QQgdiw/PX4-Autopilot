# M2006 Hybrid Hardware Upgrade Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the hybrid rover's DShot wheel motors with two CAN1 M2006/C610 speed-controlled actuators and replace the H-bridge transformation actuator with one safety-monitored M8 positional servo.

**Architecture:** Put hardware-independent protocol, speed-control, fault-gating, and transformation logic in `src/lib/hybrid_control` with host tests. Keep `m2006_can` as a thin PX4 UAVCAN-low-level CAN/uORB adapter, keep `hybrid_vehicle_control` as the sole MC/Rover/servo arbiter, and make Commander consume explicit health topics for arming decisions.

**Tech Stack:** PX4 v1.16.1, C++17, PX4 ModuleBase/ModuleParams/ScheduledWorkItem, uORB, libuavcan STM32H7 low-level CAN driver, GoogleTest, NuttX, PX4 ROMFS shell.

## Global Constraints

- Work only in `/home/crocodile/PX4-Autopilot-change1_v1.16.1` on `change1_v1.16.1`, based on `testv3_v1.16.1` commit `e1da1439dc`.
- Preserve `position -> velocity -> attitude -> rate -> steering -> RoverDifferential`; no new left/right mixing outside `RoverDifferential`.
- Final logical wheel controls remain `actuator_motors.control[4]` for right and `control[5]` for left to preserve the project logging contract; the CAN adapter must explicitly map left ID 1 from index 5 and right ID 2 from index 4.
- CAN1 is 1 Mbps. M2006 mode is runtime-mutually-exclusive with DroneCAN and Cyphal and changing the owner requires reboot.
- C610 current commands use standard frame `0x200`; left ID 1 occupies bytes 0-1, right ID 2 bytes 2-3, and bytes 4-7 are zero.
- C610 feedback uses `0x201` and `0x202`; bytes 6-7 are not temperature data.
- M8 is Servo 1. M5/M6 physical functions are disabled. `PWM_MAIN_DIS8=0` and `PWM_MAIN_FAIL8=0`.
- Sensor-enabled transformation uses AS5600 first and TMAG5273 as fallback; sensor-disabled transformation completes only by configured elapsed time.
- A detected transition fault stops M8 pulses and all propulsion. A wheel feedback/CAN fault commands both C610s to zero and remains latched until disarm plus healthy feedback.
- No new third-party dependency.
- All build and test output goes to local log files; display no more than 50 relevant lines.
- Build success and hardware validation must be reported separately.
- Every commit uses `<type>[scope]: <description>`; add an issue footer only if the owner supplies an issue number.

---

## File Map

Create `src/lib/hybrid_control/`:

- `C610Protocol.hpp/.cpp`: pure standard-frame encoder/decoder.
- `M2006SpeedController.hpp/.cpp`: one-wheel speed PID, slew limiting, current saturation, anti-windup.
- `M2006DriveGate.hpp/.cpp`: two-wheel readiness and latched-fault policy.
- `TransformationStateMachine.hpp/.cpp`: sensor selection, transitions, timeout behavior, servo target, unknown/fault behavior.
- `*Test.cpp`: host tests for each pure unit.
- `CMakeLists.txt`: library and unit-test targets.

Create `src/drivers/uavcan/m2006_can/`:

- `M2006Can.hpp/.cpp`: PX4 module, CAN1 owner, uORB adapter, 500 Hz loop, status publication.
- `m2006_can_params.c`: M2006 parameter definitions.
- `module.yaml`: module and parameter metadata.
- `CMakeLists.txt`: module target linked to `hybrid_control`, `uavcan`, and the selected low-level UAVCAN driver.

Create/modify contracts and integration:

- `msg/M2006MotorStatus.msg`: bounded-rate wheel/CAN diagnostic status.
- `msg/HybridVehicleStatus.msg`: explicit unknown/fault, target, sensor source, validity, elapsed time, and reason.
- `src/modules/hybrid_vehicle_control/*`: consume the pure transformation state machine and publish one servo control.
- `src/modules/commander/HealthAndArmingChecks/checks/hybridCheck.hpp/.cpp`: hybrid-specific arming report.
- `ROMFS/px4fmu_common/init.d/airframes/22001_quad_rover`: safe output/parameter defaults.
- `ROMFS/px4fmu_common/init.d/rc.hybrid_apps`: start `m2006_can` before the hybrid arbiter.
- `boards/zeroone/x6/hybrid.px4board`: enable the new module while retaining UAVCAN low-level build support.
- `docs/hybrid/m2006_commissioning.md`: bench and vehicle verification instructions.

---

### Task 1: C610 Protocol Codec

**Files:**
- Create: `src/lib/hybrid_control/C610Protocol.hpp`
- Create: `src/lib/hybrid_control/C610Protocol.cpp`
- Create: `src/lib/hybrid_control/C610ProtocolTest.cpp`
- Create: `src/lib/hybrid_control/CMakeLists.txt`
- Modify: `src/lib/CMakeLists.txt`

**Interfaces:**
- Produces: `hybrid_control::makeC610Command(int16_t left, int16_t right) -> C610CommandFrame`.
- Produces: `hybrid_control::decodeC610Feedback(uint32_t id, uint8_t dlc, const uint8_t data[8], uint8_t left_id, uint8_t right_id, MotorIndex &, C610Feedback &) -> bool`.
- Consumed by: Task 5 `M2006Can`.

- [ ] **Step 1: Write the failing codec tests**

```cpp
#include <gtest/gtest.h>
#include <cstring>
#include "C610Protocol.hpp"

using namespace hybrid_control;

TEST(C610Protocol, EncodesGroupedCurrentBigEndian)
{
    const C610CommandFrame frame = makeC610Command(-10000, 10000);
    EXPECT_EQ(frame.id, 0x200u);
    EXPECT_EQ(frame.dlc, 8);
    const uint8_t expected[8] {0xD8, 0xF0, 0x27, 0x10, 0, 0, 0, 0};
    EXPECT_EQ(0, memcmp(frame.data, expected, sizeof(expected)));
}

TEST(C610Protocol, DecodesConfiguredFeedbackIds)
{
    const uint8_t raw[8] {0x12, 0x34, 0xFE, 0xD4, 0x03, 0xE8, 0, 0};
    MotorIndex index{};
    C610Feedback feedback{};
    ASSERT_TRUE(decodeC610Feedback(0x201, 8, raw, 1, 2, index, feedback));
    EXPECT_EQ(index, MotorIndex::Left);
    EXPECT_EQ(feedback.encoder, 0x1234);
    EXPECT_EQ(feedback.rpm, -300);
    EXPECT_EQ(feedback.torque_current, 1000);
    EXPECT_FALSE(decodeC610Feedback(0x201, 7, raw, 1, 2, index, feedback));
    EXPECT_FALSE(decodeC610Feedback(0x203, 8, raw, 1, 2, index, feedback));
}
```

- [ ] **Step 2: Register the library/test and verify the test fails**

```cmake
px4_add_library(hybrid_control
    C610Protocol.cpp
)
target_include_directories(hybrid_control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
px4_add_unit_gtest(SRC C610ProtocolTest.cpp LINKLIBS hybrid_control)
```

Add `add_subdirectory(hybrid_control)` to `src/lib/CMakeLists.txt`.

Run: `make tests TESTFILTER=C610Protocol > /tmp/m2k-task1-test.log 2>&1`

Expected: FAIL because `C610Protocol.hpp` and its functions do not exist yet.

- [ ] **Step 3: Add the exact codec API**

```cpp
namespace hybrid_control {
enum class MotorIndex : uint8_t { Left = 0, Right = 1 };
struct C610Feedback { uint16_t encoder; int16_t rpm; int16_t torque_current; };
struct C610CommandFrame { uint32_t id{0x200}; uint8_t data[8]{}; uint8_t dlc{8}; };
C610CommandFrame makeC610Command(int16_t left, int16_t right);
bool decodeC610Feedback(uint32_t id, uint8_t dlc, const uint8_t data[8],
                        uint8_t left_id, uint8_t right_id,
                        MotorIndex &index, C610Feedback &feedback);
}
```

Implement big-endian signed packing/unpacking. Reject DLC other than 8, extended/RTR/error flag bits, IDs outside the two configured `0x200 + id` values, duplicate IDs, and configured IDs outside 1-4.

- [ ] **Step 4: Run focused and aggregate unit tests**

Run: `make tests TESTFILTER=C610Protocol > /tmp/m2k-task1-test.log 2>&1`

Expected: `C610Protocol` tests PASS with zero failures.

- [ ] **Step 5: Commit the protocol unit**

```bash
git add src/lib/CMakeLists.txt src/lib/hybrid_control
git commit -m "feat[m2006]: add C610 frame codec"
```

---

### Task 2: Wheel Speed Controller and Two-Wheel Fault Gate

**Files:**
- Create: `src/lib/hybrid_control/M2006SpeedController.hpp`
- Create: `src/lib/hybrid_control/M2006SpeedController.cpp`
- Create: `src/lib/hybrid_control/M2006SpeedControllerTest.cpp`
- Create: `src/lib/hybrid_control/M2006DriveGate.hpp`
- Create: `src/lib/hybrid_control/M2006DriveGate.cpp`
- Create: `src/lib/hybrid_control/M2006DriveGateTest.cpp`
- Modify: `src/lib/hybrid_control/CMakeLists.txt`

**Interfaces:**
- Produces: `M2006SpeedController::update(float normalized, float measured_rpm, float dt, bool enabled) -> int16_t`.
- Produces: `M2006DriveGate::update(const DriveGateInput &) -> bool` and `faultBits() -> uint32_t`.
- Consumed by: Task 5 `M2006Can`.

- [ ] **Step 1: Write failing controller tests**

```cpp
TEST(M2006SpeedController, SaturatesAndResets)
{
    M2006SpeedController controller;
    controller.configure({500.f, 2.f, 4.f, 0.f, 0.f, 1000, 500.f});
    EXPECT_EQ(controller.update(1.f, 0.f, 0.01f, true), 1000);
    EXPECT_GT(controller.integral(), 0.f);
    EXPECT_EQ(controller.update(1.f, 0.f, 0.01f, false), 0);
    EXPECT_FLOAT_EQ(controller.integral(), 0.f);
}

TEST(M2006SpeedController, SlewsTarget)
{
    M2006SpeedController controller;
    controller.configure({500.f, 1.f, 0.f, 0.f, 0.f, 10000, 500.f});
    controller.update(1.f, 0.f, 0.1f, true);
    EXPECT_FLOAT_EQ(controller.targetRpm(), 50.f);
}
```

- [ ] **Step 2: Write failing gate tests**

```cpp
TEST(M2006DriveGate, RequiresBothFeedbackStreams)
{
    M2006DriveGate gate;
    DriveGateInput input{false, true, true, true, {true, true}, false, 0};
    EXPECT_FALSE(gate.update(input));
    input.now_us = 100000;
    EXPECT_FALSE(gate.update(input));
    input.armed = true;
    EXPECT_TRUE(gate.update(input));
}

TEST(M2006DriveGate, LatchesFeedbackLossUntilDisarmedHealthy)
{
    M2006DriveGate gate;
    DriveGateInput input{false, true, true, true, {true, true}, false, 0};
    gate.update(input);
    input.now_us = 100000;
    gate.update(input);
    input.armed = true;
    ASSERT_TRUE(gate.update(input));
    input.feedback_healthy[1] = false;
    EXPECT_FALSE(gate.update(input));
    input.feedback_healthy[1] = true;
    EXPECT_FALSE(gate.update(input));
    input.armed = false;
    input.now_us = 200000;
    EXPECT_FALSE(gate.update(input));
    input.now_us = 300000;
    EXPECT_FALSE(gate.update(input));
    input.armed = true;
    EXPECT_TRUE(gate.update(input));
}
```

- [ ] **Step 3: Verify both new suites fail**

Run: `make tests TESTFILTER='M2006(SpeedController|DriveGate)' > /tmp/m2k-task2-test.log 2>&1`

Expected: FAIL because the controller and gate types do not exist.

- [ ] **Step 4: Implement the controller API and control law**

```cpp
struct SpeedControllerConfig {
    float max_rpm;
    float kp;
    float ki;
    float kd;
    float kff;
    int16_t current_limit;
    float rpm_slew;
};

class M2006SpeedController {
public:
    void configure(const SpeedControllerConfig &config);
    void reset();
    int16_t update(float normalized, float measured_rpm, float dt, bool enabled);
    float targetRpm() const { return _target_rpm; }
    float integral() const { return _integral; }
private:
    SpeedControllerConfig _config{};
    float _target_rpm{0.f};
    float _integral{0.f};
    float _last_measurement{0.f};
};
```

Use derivative-on-measurement, constrain normalized input to `[-1, 1]`, limit target change to `rpm_slew * dt`, clamp current before converting to `int16_t`, and integrate only when the unclamped output would move out of saturation.

- [ ] **Step 5: Implement the fault gate**

```cpp
enum DriveFault : uint32_t {
    DriveFaultNone = 0,
    DriveFaultLeftFeedback = 1u << 0,
    DriveFaultRightFeedback = 1u << 1,
    DriveFaultCan = 1u << 2,
    DriveFaultCommand = 1u << 3,
};

struct DriveGateInput {
    bool armed;
    bool driving;
    bool command_fresh;
    bool command_finite;
    bool feedback_healthy[2];
    bool can_error;
    uint64_t now_us;
};

class M2006DriveGate {
public:
    bool update(const DriveGateInput &input);
    uint32_t faultBits() const { return _fault_bits; }
private:
    uint32_t _fault_bits{DriveFaultNone};
    uint64_t _both_healthy_since{0};
};
```

Require 100 ms continuous dual feedback before enabling. Latch loss/CAN/invalid-command faults while armed. Clear only while disarmed after the same 100 ms healthy interval. Merely leaving `DRIVING` disables output without latching.

- [ ] **Step 6: Run tests and commit**

Run: `make tests TESTFILTER='M2006(SpeedController|DriveGate)' > /tmp/m2k-task2-test.log 2>&1`

Expected: both suites PASS.

```bash
git add src/lib/hybrid_control
git commit -m "feat[m2006]: add wheel speed and fault control"
```

---

### Task 3: Pure Transformation State Machine

**Files:**
- Create: `src/lib/hybrid_control/TransformationStateMachine.hpp`
- Create: `src/lib/hybrid_control/TransformationStateMachine.cpp`
- Create: `src/lib/hybrid_control/TransformationStateMachineTest.cpp`
- Modify: `src/lib/hybrid_control/CMakeLists.txt`

**Interfaces:**
- Produces: `TransformationStateMachine::initialize`, `request`, `update`, and `clearFault`.
- Consumed by: Task 6 `HybridVehicleControl`.

- [ ] **Step 1: Write the transition-table tests first**

Cover these named tests with explicit inputs and outputs:

```cpp
TEST(TransformationStateMachine, As5600CompletesAfterDebounce);
TEST(TransformationStateMachine, FallsBackToTargetTmag);
TEST(TransformationStateMachine, SensorConflictFaultsAndReleasesServo);
TEST(TransformationStateMachine, EnabledSensorTimeoutFaults);
TEST(TransformationStateMachine, DisabledSensorsCompleteByTime);
TEST(TransformationStateMachine, StartupUsesConfiguredStateOnlyWhenSensorsDisabled);
TEST(TransformationStateMachine, FaultClearsOnlyDisarmedWithExplicitRequest);
```

The AS5600 test must hold the angle inside tolerance for 99 ms and expect transition, then advance to 100 ms and expect the target stable state. The sensor-disabled test must remain transitioning at `max_transition_us - 1` and complete exactly at `max_transition_us`.

- [ ] **Step 2: Verify the state tests fail**

Run: `make tests TESTFILTER=TransformationStateMachine > /tmp/m2k-task3-test.log 2>&1`

Expected: FAIL because the state-machine API is missing.

- [ ] **Step 3: Add the exact public types**

```cpp
enum class HybridState : uint8_t { Flying, TransitionToRover, Driving, TransitionToQuad, Unknown, Fault };
enum class HybridTarget : uint8_t { None, Flying, Driving };
enum class SensorSource : uint8_t { None, As5600, Tmag5273 };
enum class TransformFault : uint8_t { None, NoSensor, SensorConflict, SensorTimeout, TransitionTimeout, InvalidServoConfig };

struct TransformationConfig {
    bool sensors_enabled;
    HybridState configured_boot_state;
    float quad_servo;
    float rover_servo;
    float quad_angle;
    float rover_angle;
    float angle_tolerance;
    uint64_t sensor_timeout_us;
    uint64_t debounce_us;
    uint64_t max_transition_us;
};

struct TransformationInput {
    uint64_t now_us;
    bool as5600_valid;
    float as5600_angle;
    bool tmag_quad_valid;
    bool tmag_quad_active;
    bool tmag_rover_valid;
    bool tmag_rover_active;
};

struct TransformationOutput {
    HybridState state;
    HybridTarget target;
    SensorSource source;
    TransformFault fault;
    bool servo_enabled;
    float servo_value;
};
```

- [ ] **Step 4: Implement the deterministic transition rules**

Use AS5600 if fresh and within a configured endpoint tolerance. Use the target TMAG endpoint only when AS5600 is unavailable. Treat both active TMAG endpoints or a valid AS5600 endpoint opposite to a valid active TMAG endpoint as conflict. In sensor-enabled transitions, timeout is a fault. In sensor-disabled transitions, timeout is success. Stable states output their configured servo target; unknown/fault output `servo_enabled=false`.

Reject servo configuration when either value is outside `[-1, 1]` or `fabsf(quad_servo - rover_servo) < 0.1f`.

- [ ] **Step 5: Run all pure-library tests and commit**

Run: `make tests TESTFILTER='(C610Protocol|M2006|TransformationStateMachine)' > /tmp/m2k-task3-test.log 2>&1`

Expected: all focused suites PASS.

```bash
git add src/lib/hybrid_control
git commit -m "feat[hybrid]: add transformation state machine"
```

---

### Task 4: uORB Contracts and Parameters

**Files:**
- Create: `msg/M2006MotorStatus.msg`
- Modify: `msg/HybridVehicleStatus.msg`
- Modify: `msg/CMakeLists.txt`
- Create: `src/drivers/uavcan/m2006_can/m2006_can_params.c`
- Create: `src/drivers/uavcan/m2006_can/module.yaml`
- Modify: `src/modules/hybrid_vehicle_control/hybrid_vehicle_control_params.c`

**Interfaces:**
- Produces: `m2006_motor_status_s` and expanded `hybrid_vehicle_status_s`.
- Produces: all `M2K_*` and new `HYB_*` parameters approved in the design.
- Consumed by: Tasks 5-8.

- [ ] **Step 1: Define `M2006MotorStatus.msg`**

```text
uint64 timestamp
uint64[2] feedback_timestamp
float32[2] target_rpm
float32[2] measured_rpm
float32[2] speed_error
int16[2] current_command
int16[2] torque_current
uint16[2] encoder
bool[2] online
uint32 fault_flags
uint32 rx_count
uint32 tx_count
uint32 timeout_count
uint64 can_error_count
```

Document array index 0 as left/ID1 and index 1 as right/ID2.

- [ ] **Step 2: Expand `HybridVehicleStatus.msg` compatibly**

Keep existing state numeric values and add:

```text
uint8 HYBRID_STATE_UNKNOWN = 3
uint8 HYBRID_STATE_TRANSITION_FAULT = 4
uint8 TARGET_NONE = 0
uint8 TARGET_FLYING = 1
uint8 TARGET_DRIVING = 2
uint8 SENSOR_NONE = 0
uint8 SENSOR_AS5600 = 1
uint8 SENSOR_TMAG5273 = 2
uint8 current_state
uint8 target_state
uint8 sensor_source
uint8 fault_reason
bool as5600_valid
bool tmag_quad_valid
bool tmag_rover_valid
uint64 transition_elapsed
```

Retain `HYBRID_STATE_TRANSITIONING=1` so current Commander comparisons remain valid until Task 7 updates them.

- [ ] **Step 3: Add M2006 parameters with exact bounds**

Define `M2K_EN`, IDs 1-4, reversal flags, `M2K_MAX_RPM` 1-500, `M2K_CUR_LIM` 0-10000, `M2K_SPD_P/I/D/FF`, `M2K_RPM_SLEW`, `M2K_FB_TO`, and `M2K_CMD_TO`. Mark enable and IDs reboot-required. Defaults must match the approved design.

- [ ] **Step 4: Add transformation parameters**

Add `HYB_SENS_EN`, `HYB_BOOT_ST`, `HYB_SV_QUD`, `HYB_SV_ROV`, `HYB_ANG_TOL`, `HYB_SENS_TO`, and `HYB_DBNC_T`. Change `HYBRID_TRANS_T` documentation from fallback success to maximum duration whose result depends on sensor mode. Keep existing AS5600 target angles and TMAG IDs/thresholds.

- [ ] **Step 5: Generate contracts through a board build**

Run: `make zeroone_x6_hybrid > /tmp/m2k-task4-build.log 2>&1`

Expected: generated uORB headers and parameter metadata compile with no duplicate parameter names or message-field errors.

- [ ] **Step 6: Commit contracts and parameters**

```bash
git add msg src/drivers/uavcan/m2006_can/m2006_can_params.c \
  src/drivers/uavcan/m2006_can/module.yaml \
  src/modules/hybrid_vehicle_control/hybrid_vehicle_control_params.c
git commit -m "feat[hybrid]: define M2006 and transformation contracts"
```

---

### Task 5: Standalone `m2006_can` PX4 Driver

**Files:**
- Create: `src/drivers/uavcan/m2006_can/CMakeLists.txt`
- Create: `src/drivers/uavcan/m2006_can/M2006Can.hpp`
- Create: `src/drivers/uavcan/m2006_can/M2006Can.cpp`
- Modify: `src/drivers/uavcan/CMakeLists.txt`
- Modify: `src/drivers/uavcan/Kconfig`
- Modify: `boards/zeroone/x6/hybrid.px4board`

**Interfaces:**
- Consumes: Task 1 codec, Task 2 controller/gate, final `actuator_motors`, `actuator_armed`, and `hybrid_vehicle_status`.
- Produces: `m2006_motor_status` and zero/nonzero C610 frame `0x200`.

- [ ] **Step 1: Add build wiring and a compile-only module skeleton**

In `src/drivers/uavcan/CMakeLists.txt`, after the selected low-level driver target exists:

```cmake
if(CONFIG_DRIVERS_M2006_CAN)
    add_subdirectory(m2006_can)
endif()
```

The module CMake target must link `hybrid_control`, `uavcan`, and `uavcan_${UAVCAN_DRIVER}_driver`, and include `../uavcan_driver.hpp` plus the selected driver's public headers. Add `CONFIG_DRIVERS_M2006_CAN=y` to the hybrid board.

Run: `make zeroone_x6_hybrid > /tmp/m2k-task5-skeleton.log 2>&1`

Expected: FAIL until the module class and entry point are defined.

- [ ] **Step 2: Define the PX4 module class**

```cpp
class M2006Can final : public ModuleBase<M2006Can>,
                       public ModuleParams,
                       public px4::ScheduledWorkItem {
public:
    static int task_spawn(int argc, char *argv[]);
    static M2006Can *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    bool init();
    void Run() override;
    int print_status() override;
private:
    UAVCAN_DRIVER::CanInitHelper<16> _can{1u};
    uavcan::ICanIface *_iface{nullptr};
    M2006SpeedController _speed[2]{};
    M2006DriveGate _gate{};
};
```

Schedule at 2 ms. Check `UAVCAN_ENABLE` and `CYPHAL_ENABLE` before calling `_can.init(1000000)`. Configure two standard-data filters with ID `0x201/0x202` and mask `MaskStdID | FlagEFF | FlagRTR`.

- [ ] **Step 3: Add RX and freshness handling**

Drain `ICanIface::receive()` non-blockingly each cycle. Decode only through `decodeC610Feedback`. Set feedback timestamps from monotonic receive time, update ID-specific counters, and treat feedback older than `M2K_FB_TO` as offline. Never interpret bytes 6-7.

- [ ] **Step 4: Add command gating and control**

Copy final actuator indices exactly:

```cpp
const float normalized_left = motors.control[5];
const float normalized_right = motors.control[4];
```

Apply `M2K_L_REV` and `M2K_R_REV` before each speed controller. The gate receives armed state, `HYBRID_STATE_DRIVING`, command age, finiteness, both feedback states, and CAN-error deltas. If disabled or faulted, reset both controllers and send `makeC610Command(0, 0)`.

- [ ] **Step 5: Add TX, status, and CLI behavior**

Send one frame each 2 ms with a 2 ms monotonic deadline. Count TX-full separately from negative send errors; sustained failures and rising CAN hardware error count enter the CAN fault path. Publish `m2006_motor_status` at 50 Hz. `m2006_can status` prints one bounded snapshot; no raw-frame stream command is added.

- [ ] **Step 6: Verify pure tests and firmware build**

Run:

```bash
make tests TESTFILTER='(C610Protocol|M2006)' > /tmp/m2k-task5-tests.log 2>&1
make zeroone_x6_hybrid > /tmp/m2k-task5-build.log 2>&1
```

Expected: all focused tests PASS and firmware reaches `Creating ...zeroone_x6_hybrid.px4`.

- [ ] **Step 7: Commit the driver**

```bash
git add src/drivers/uavcan boards/zeroone/x6/hybrid.px4board
git commit -m "feat[m2006]: add standalone CAN wheel driver"
```

---

### Task 6: Integrate the Positional Servo State Machine

**Files:**
- Modify: `src/modules/hybrid_vehicle_control/hybrid_vehicle_control.hpp`
- Modify: `src/modules/hybrid_vehicle_control/hybrid_vehicle_control.cpp`
- Modify: `src/modules/hybrid_vehicle_control/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 3 `TransformationStateMachine`, AS5600 `sensor_encoder`, TMAG `magnetic_sensor`, vehicle commands, RC request, armed/prearmed state.
- Produces: one Servo 1 value, final MC/Rover motor arbitration, expanded `hybrid_vehicle_status`.

- [ ] **Step 1: Replace the local state enum with the pure state machine**

Add `hybrid_control` to module dependencies. Store one `TransformationStateMachine`, last sensor timestamps/values, and last output. Remove old H-bridge-specific completion flags and the second servo output.

- [ ] **Step 2: Build `TransformationInput` from uORB**

Mark AS5600 valid only when its message age is within `HYB_SENS_TO`. Route TMAG messages by configured device ID, calculate active state with `fabsf(mag_z) >= threshold`, and apply the same freshness limit. Do not silently use a stale source.

- [ ] **Step 3: Route requests without bypasses**

Existing RC, MAVLink, and mission requests call `TransformationStateMachine::request`. Preserve the Quad-to-Rover altitude guard. Direct RC servo commissioning is accepted only when `actuator_armed.armed == false` and `actuator_armed.prearmed == true`; it suppresses propulsion and never declares a stable state.

- [ ] **Step 4: Publish one servo and safe propulsion outputs**

Initialize all motor and servo controls to `NAN`. If `servo_enabled`, set only `servos.control[0]`. Stable `Flying` copies MC controls 0-3. Stable `Driving` copies Rover left output to final index 5 and Rover right output to index 4. Transition, unknown, fault, and manual commissioning copy no propulsion.

- [ ] **Step 5: Publish the expanded status**

Map both directional internal transition states to the compatible uORB `HYBRID_STATE_TRANSITIONING`, and use `target_state` to distinguish direction. Publish source validity, active source, elapsed time, and fault reason on every bounded module cycle.

- [ ] **Step 6: Run state tests and firmware build**

Run:

```bash
make tests TESTFILTER=TransformationStateMachine > /tmp/m2k-task6-tests.log 2>&1
make zeroone_x6_hybrid > /tmp/m2k-task6-build.log 2>&1
```

Expected: state tests PASS; no references remain to the old complementary `servos.control[1]` H-bridge output; firmware builds.

- [ ] **Step 7: Commit the servo/state integration**

```bash
git add src/modules/hybrid_vehicle_control
git commit -m "feat[hybrid]: drive transformation servo with sensor safety"
```

---

### Task 7: Hybrid Arming and Control-Mode Safety Checks

**Files:**
- Create: `src/modules/commander/HealthAndArmingChecks/checks/hybridCheck.hpp`
- Create: `src/modules/commander/HealthAndArmingChecks/checks/hybridCheck.cpp`
- Create: `src/modules/commander/HealthAndArmingChecks/checks/hybridCheckTest.cpp`
- Modify: `src/modules/commander/HealthAndArmingChecks/CMakeLists.txt`
- Modify: `src/modules/commander/HealthAndArmingChecks/HealthAndArmingChecks.hpp`
- Modify: `src/modules/commander/Commander.cpp`
- Modify: `src/modules/commander/Commander.hpp`

**Interfaces:**
- Consumes: `hybrid_vehicle_status`, `m2006_motor_status`, `M2K_EN`, controller gains, and M5/M6/M8 output parameters.
- Produces: arming failures under `health_component_t::system` and control-mode suppression outside stable states.

- [ ] **Step 1: Write the hybrid check truth-table test**

```cpp
TEST(HybridCheck, FlyingDoesNotRequireM2006);
TEST(HybridCheck, DrivingRequiresBothM2006Online);
TEST(HybridCheck, UnknownTransitionAndFaultRejectArming);
TEST(HybridCheck, UnsafeM8MappingRejectsArming);
TEST(HybridCheck, AllZeroControllerRejectsDriving);
```

Each test publishes the required uORB status and asserts `reporter.canArm(...)` or the emitted event ID.

- [ ] **Step 2: Verify the new functional test fails**

Run: `make tests TESTFILTER=HybridCheck > /tmp/m2k-task7-tests.log 2>&1`

Expected: FAIL because `HybridChecks` is not registered.

- [ ] **Step 3: Add and register `HybridChecks`**

Derive from `HealthAndArmingCheckBase`, subscribe to both status topics, and add one instance to `_checks`. Report these distinct rate-limited events:

```text
check_hybrid_state_unsafe
check_hybrid_m2006_unhealthy
check_hybrid_servo_mapping
check_hybrid_controller_unconfigured
```

Apply checks only when `vehicle_status.is_quad_rover` is true. Flying ignores M2006 health. Driving requires enabled driver, both online, no faults, and at least one nonzero `P/I/FF` gain.

- [ ] **Step 4: Validate output configuration**

Require M5/M6 functions disabled, M8 function Servo 1, M8 disarmed/failsafe zero, servo targets finite/separated, and no conflicting `UAVCAN_ENABLE`/`CYPHAL_ENABLE` when `M2K_EN=1`.

- [ ] **Step 5: Extend Commander state handling**

Only `FLYING` sets rotary-wing type and only `DRIVING` sets rover type. Transitioning, unknown, and fault retain the last stable type but force all position, velocity, attitude, rate, altitude, acceleration, and allocation control flags false. Handle stale `hybrid_vehicle_status` as unsafe instead of preserving an indefinitely valid state.

- [ ] **Step 6: Run checks and commit**

Run:

```bash
make tests TESTFILTER=HybridCheck > /tmp/m2k-task7-tests.log 2>&1
make zeroone_x6_hybrid > /tmp/m2k-task7-build.log 2>&1
```

Expected: all hybrid arming truth-table tests PASS and firmware builds.

```bash
git add src/modules/commander
git commit -m "feat[commander]: enforce hybrid actuator health"
```

---

### Task 8: Airframe Defaults, Startup Ordering, and CAN Exclusivity

**Files:**
- Modify: `ROMFS/px4fmu_common/init.d/airframes/22001_quad_rover`
- Modify: `ROMFS/px4fmu_common/init.d/rc.hybrid_apps`
- Modify: `boards/zeroone/x6/hybrid.px4board`

**Interfaces:**
- Consumes: all new parameters/modules.
- Produces: a safe default runtime configuration for the new hardware.

- [ ] **Step 1: Set exact output defaults**

Add these airframe defaults:

```sh
param set-default PWM_MAIN_FUNC1 101
param set-default PWM_MAIN_FUNC2 102
param set-default PWM_MAIN_FUNC3 103
param set-default PWM_MAIN_FUNC4 104
param set-default PWM_MAIN_FUNC5 0
param set-default PWM_MAIN_FUNC6 0
param set-default PWM_MAIN_FUNC8 201
param set-default PWM_MAIN_DIS8 0
param set-default PWM_MAIN_FAIL8 0
param set-default CA_ROTOR_COUNT 4
param set-default CA_SV_CS_COUNT 1
```

Do not assign M5/M6 to any motor function.

- [ ] **Step 2: Set CAN and initial control defaults**

```sh
param set-default UAVCAN_ENABLE 0
param set-default CYPHAL_ENABLE 0
param set-default M2K_EN 1
param set-default M2K_L_ID 1
param set-default M2K_R_ID 2
param set-default M2K_MAX_RPM 500
param set-default M2K_CUR_LIM 10000
param set-default RO_MAX_THR_SPEED 2.47
```

Keep speed gains at zero so driving arming remains blocked until bench tuning. Keep `HYB_SENS_EN=1`, both servo targets at zero, and rely on the servo-configuration health check to block uncalibrated transformation.

- [ ] **Step 3: Start modules in safe order**

In `rc.hybrid_apps`, start sensor drivers through the existing board sensor startup, then start `m2006_can`, native MC/Rover controllers, and finally `hybrid_vehicle_control`. Check return values; if `m2006_can` fails, print one error and leave wheel operation unavailable rather than continuing silently.

- [ ] **Step 4: Verify generated ROMFS and parameters**

Run: `make zeroone_x6_hybrid > /tmp/m2k-task8-build.log 2>&1`

Expected: build succeeds; `build/zeroone_x6_hybrid/etc/init.d/airframes/22001_quad_rover` contains all exact defaults and `rc.hybrid_apps` contains one M2006 start.

- [ ] **Step 5: Commit airframe integration**

```bash
git add ROMFS boards/zeroone/x6/hybrid.px4board
git commit -m "feat[airframe]: configure CAN wheels and M8 servo"
```

---

### Task 9: Integrated Verification and Firmware-Size Gate

**Files:**
- Modify only files required by failures found in this task; do not weaken tests or remove modules to force a pass.

**Interfaces:**
- Consumes: Tasks 1-8.
- Produces: verified host tests and both hybrid/default board artifacts.

- [ ] **Step 1: Run all focused host tests**

Run: `make tests TESTFILTER='(C610Protocol|M2006|TransformationStateMachine|HybridCheck)' > /tmp/m2k-task9-tests.log 2>&1`

Expected: zero failing tests. Record test count from the log.

- [ ] **Step 2: Build the hybrid target from a clean configuration**

Run:

```bash
rm -rf build/zeroone_x6_hybrid
make zeroone_x6_hybrid > /tmp/m2k-task9-hybrid-build.log 2>&1
```

Expected: firmware artifact exists and Flash does not overflow 1,920 KB. Record absolute bytes, percentage, and delta from the approved baseline 1,833,372 bytes. If usage exceeds 95%, report it as a release risk even if linking succeeds.

- [ ] **Step 3: Build the board's default DroneCAN-capable target**

Run: `make zeroone_x6_default > /tmp/m2k-task9-default-build.log 2>&1`

Expected: default firmware builds, proving the original DroneCAN code path was not removed or made uncompilable.

- [ ] **Step 4: Inspect configuration invariants**

Run bounded searches that verify:

```text
M5/M6 physical functions disabled
M8 Servo 1
M8 DIS8/FAIL8 zero
exactly one left index 5 and right index 4 mapping
no H-bridge write to actuator_servos.control[1]
no direct wheel mixing in m2006_can
```

- [ ] **Step 5: Run diff and static hygiene checks**

Run:

```bash
git diff --check
git status --short
git diff --stat testv3_v1.16.1...HEAD
```

Expected: no whitespace errors, no generated build artifacts tracked, and only scoped source/docs changes.

- [ ] **Step 6: Commit only genuine verification fixes**

If verification required source changes, commit them with the semantic scope matching the fix, for example:

```bash
git commit -m "fix[m2006]: preserve zero output on stale commands"
```

If no source change was required, do not create an empty commit.

---

### Task 10: Commissioning Guide and Handoff

**Files:**
- Create: `docs/hybrid/m2006_commissioning.md`
- Modify: `docs/superpowers/specs/2026-07-14-m2006-hybrid-hardware-upgrade-design.md` only if implementation names differ from the approved specification; explain every such correction.

**Interfaces:**
- Consumes: final parameter names, commands, topics, and fault behavior.
- Produces: a reproducible bench-to-vehicle procedure.

- [ ] **Step 1: Document the no-load CAN bench procedure**

Include exact commands:

```sh
param set M2K_CUR_LIM 1000
listener m2006_motor_status 5
m2006_can status
```

Require wheel suspension, CAN analyzer confirmation of `0x200/0x201/0x202`, left/right sign checks, one-wheel disconnect, command-timeout injection, disarm recovery, and explicit recording of behavior after physical CAN loss.

- [ ] **Step 2: Document servo calibration**

Require unloaded linkage first, M8 oscilloscope verification, safe `PWM_MAIN_MIN8/MAX8`, separated `HYB_SV_QUD/ROV`, AS5600 completion, TMAG fallback, conflict/timeout fault, zero pulse in fault/disarm, and sensor-disabled timed completion.

- [ ] **Step 3: Document tuning order and ULog evidence**

State the mandatory order: M2006 wheel speed, Rover yaw rate, yaw attitude, vehicle speed, then position/mission. Require target rpm, measured rpm, error, current command, torque-current feedback, hybrid state, and valid sample conditions. State that theoretical 2.47 m/s must be replaced by loaded measurement before finalizing `RO_MAX_THR_SPEED`.

- [ ] **Step 4: Verify documentation commands against metadata**

Run bounded `rg` checks to ensure every documented parameter, module command, and topic exists in source/generated metadata. Correct documentation rather than inventing aliases.

- [ ] **Step 5: Commit documentation**

```bash
git add docs/hybrid docs/superpowers/specs/2026-07-14-m2006-hybrid-hardware-upgrade-design.md
git commit -m "docs[hybrid]: add M2006 commissioning procedure"
```

- [ ] **Step 6: Update project state without claiming hardware success**

Update `/home/crocodile/PX4-Autopilot/state/README.md`, `TODO.md`, and `LOG.md` with commit IDs, test counts, build sizes, remaining bench work, and any proxy/submodule lessons. Mark real-vehicle validation pending until physical evidence exists.

---

## Execution Checkpoints

- Checkpoint A after Task 3: pure logic reviewed and all host tests green.
- Checkpoint B after Task 5: M2006 driver builds and sends zero by default.
- Checkpoint C after Task 7: transformation and arming safety reviewed together.
- Checkpoint D after Task 9: full software verification and size report.
- Final checkpoint after Task 10: human reviews bench procedure before firmware is exercised on hardware.

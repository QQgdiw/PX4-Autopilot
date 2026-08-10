# HX8 UART Servo and Transformation Stall Protection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add continuous-position stall protection to the M8 PWM transformation servo and a selectable HX8-U45H-M UART backend with equivalent latched-fault, release, arming, and LED behavior.

**Architecture:** Keep normalization, progress detection, HX8 protocol, and request scheduling in host-tested libraries. Keep `hybrid_vehicle_control` as state/backend arbiter, `hx8_uart_servo` as UART/uORB adapter, and Commander as the sole board-LED and arming-check owner.

**Tech Stack:** PX4 v1.16.1, C++17, uORB, PX4 parameters, NuttX termios, `px4::ScheduledWorkItem`, GoogleTest, `zeroone_x6_hybrid`.

## Global Constraints

- Work on `change1_v1.16.1`; target only `zeroone_x6_hybrid`.
- The only required firmware build is `make zeroone_x6_hybrid`; do not adapt or build `zeroone_x6_default`.
- `HYB_ACT_TYPE=0` is reboot-required PWM compatibility mode; `1` is reboot-required HX8 mode. No runtime fallback.
- PWM mode does not open EXT2. HX8 mode keeps M8 disabled.
- HX8 uses EXT2 `/dev/ttyS3`, 115200 8N1 through an automatic-direction half-duplex TTL adapter. No `TIOCSSINGLEWIRE` and no DIR/OE GPIO.
- AS5600 and paired TMAG5273 are auto-selected in PWM sensor mode. Runtime loss faults; it never degrades to timing.
- `HYB_SENS_EN=0` preserves PWM maximum-time completion. HX8 always requires fresh internal-angle feedback.
- Healthy endpoints keep holding position. A PWM fault publishes NaN; M8 disarmed/failsafe raw values stay zero.
- An HX8 fault makes bounded stop-and-release attempts, then rejects position commands. Total UART loss still relies on verified onboard protection because no power switch exists.
- Use HRT for freshness, fault, and uORB timestamps. Preserve the first fault as primary; secondary HX8 failures remain in HX8 status/events.
- Fault clear is explicit and fully disarmed; it never resumes the target. Intermediate position becomes unarmable `UNKNOWN`.
- Commander remains the sole board LED owner.
- Never commit `hardware_reference/` or `.superpowers/`. Redirect test/build logs and show at most 40 failure lines.

## File Map

- Pure transformation: create `TransformationPosition.{hpp,cpp,Test.cpp}`; modify `TransformationStateMachine.{hpp,cpp,Test.cpp}` and `src/lib/hybrid_control/CMakeLists.txt`.
- Contracts/module: modify `msg/HybridVehicleStatus.msg`, `msg/CMakeLists.txt`, and `src/modules/hybrid_vehicle_control/*`; create `msg/Hx8ServoCommand.msg` and `msg/Hx8ServoStatus.msg`.
- Commander: modify `HybridStatusGuard.hpp`, `CommanderHybridStatusTest.cpp`, `Commander.{hpp,cpp}`, and `HealthAndArmingChecks/checks/hybridCheck{.cpp,Test.cpp}`.
- HX8 pure library: create `src/lib/hx8_servo/{CMakeLists.txt,Hx8Protocol.*,Hx8ProtocolTest.cpp,Hx8Controller.*,Hx8ControllerTest.cpp}`.
- HX8 driver: create `src/drivers/actuators/hx8_uart_servo/{Kconfig,CMakeLists.txt,module.yaml,Hx8UartServo.*,hx8_uart_servo_main.cpp}`.
- Hybrid target/startup: modify `boards/zeroone/x6/hybrid.px4board`, `ROMFS/px4fmu_common/init.d/rc.hybrid_apps`, and airframe `22001_quad_rover`.
- Commissioning: create `docs/hybrid/hx8-u45h-m-commissioning.md`.

---

### Task 1: Normalized Position and Directed Progress

**Files:**
- Create: `src/lib/hybrid_control/TransformationPosition.hpp`
- Create: `src/lib/hybrid_control/TransformationPosition.cpp`
- Create: `src/lib/hybrid_control/TransformationPositionTest.cpp`
- Modify: `src/lib/hybrid_control/CMakeLists.txt`

**Interfaces:**

```cpp
namespace hybrid_control {
enum class SensorSource : uint8_t { None, As5600, Tmag5273, Hx8 };
struct PositionSample {
	float normalized{NAN};
	bool valid{false};
	bool endpoint_confirmed{false};
	SensorSource source{SensorSource::None};
	uint64_t timestamp_us{0};
};
struct TmagVector { float x; float y; float z; };
float normalizeAs5600(float angle, float quad_angle, float rover_angle);
float tmagMagnitude(const TmagVector &sample);
class TmagRatioFilter {
public:
	PositionSample update(const TmagVector &quad, const TmagVector &rover,
		bool quad_valid, bool rover_valid, bool endpoint_confirmed, uint64_t timestamp_us);
	void reset();
private:
	float _filtered{NAN};
};
enum class ProgressResult : uint8_t { Idle, Progress, NoProgress, Reached, Invalid };
class DirectedProgressMonitor {
public:
	void start(float position, float target, uint64_t now_us);
	ProgressResult update(const PositionSample &, float target, float minimum_progress, uint64_t timeout_us);
	void reset();
	uint64_t noProgressElapsed(uint64_t now_us) const;
private:
	float _anchor{NAN};
	float _direction{0.f};
	uint64_t _anchor_time_us{0};
	bool _active{false};
};
}
```

`TmagRatioFilter` computes `B_rover/(B_quad+B_rover)`, rejects non-finite axes and denominator below `1e-6f`, clamps `[0,1]`, and uses fixed `alpha=0.25f`. AS5600 unwraps directed endpoint travel and returns NaN for identical endpoints. Progress resets its anchor only after directed net displacement reaches the configured delta.

- [ ] **Step 1: Write failing tests**

Cover AS5600 wrap (`quad=6.1`, `rover=0.2`), TMAG equal magnitudes (`0.5`), endpoint dominance, invalid denominator, filter first sample, forward progress, reverse, oscillation, reached, and invalid samples. Include:

```cpp
monitor.start(0.20f, 1.0f, 1_s);
EXPECT_EQ(monitor.update({0.21f, true, false, SensorSource::As5600, 1500_ms}, 1.f, .02f, 800_ms),
	ProgressResult::Progress);
EXPECT_EQ(monitor.update({0.19f, true, false, SensorSource::As5600, 1801_ms}, 1.f, .02f, 800_ms),
	ProgressResult::NoProgress);
```

- [ ] **Step 2: Verify red phase**

Run `make tests TESTFILTER=TransformationPosition > /tmp/hx8-t1-red.log 2>&1`; expect nonzero because the files are absent.

- [ ] **Step 3: Implement the interfaces without uORB/parameters**

Use finite wrap math; fixed-size state only. The caller resets filter and monitor on source change.

- [ ] **Step 4: Register and verify green phase**

Add `TransformationPosition.cpp` to `hybrid_control` and `px4_add_unit_gtest(SRC TransformationPositionTest.cpp LINKLIBS hybrid_control)`. Run the same test; expect exit 0.

- [ ] **Step 5: Commit**

```bash
git add src/lib/hybrid_control/TransformationPosition.* src/lib/hybrid_control/CMakeLists.txt
git commit -m "feat[hybrid]: add transformation position tracking"
```

### Task 2: State-Machine Stall and Fault Semantics

**Files:**
- Modify: `src/lib/hybrid_control/TransformationStateMachine.hpp`
- Modify: `src/lib/hybrid_control/TransformationStateMachine.cpp`
- Modify: `src/lib/hybrid_control/TransformationStateMachineTest.cpp`

**Interfaces:**

```cpp
enum class ActuatorBackend : uint8_t { Pwm=0, Hx8=1 };
enum class TransformFault : uint8_t {
	None=0, NoSensor=1, SensorConflict=2, SensorTimeout=3, TransitionTimeout=4,
	InvalidServoConfig=5, InvalidConfiguration=6, Stall=7,
	ActuatorCommunication=8, ActuatorProtection=9, ActuatorConfigMismatch=10,
	ActuatorCommandRejected=11
};
struct ActuatorHealth {
	bool online{true};
	bool healthy{true};
	bool config_verified{true};
	bool command_accepted{true};
	uint8_t protection_flags{0};
};
```

Append backend, `stall_timeout_s`, and `stall_distance` to `TransformationConfig`; append normalized `PositionSample`, `ActuatorHealth`, and `actuator_command_effective` to input; append `release_requested` and `no_progress_elapsed_us` to output. Preserve PWM `servo_enabled/servo_value` during migration.

- [ ] **Step 1: Write failing tests**

Test 0.8 s immobility, net 0.02 progress, reverse/oscillation, no stable-hold stall, endpoint-before-stall, independent absolute timeout, source reset, faults 8--11, first-cause latch, release output, armed clear rejection, and disarmed intermediate clear to `Unknown`/`None`.

- [ ] **Step 2: Run red phase**

Run `make tests TESTFILTER=TransformationStateMachine > /tmp/hx8-t2-red.log 2>&1`; expect compile/test failure on new semantics.

- [ ] **Step 3: Implement ordered fault evaluation**

```cpp
if (!input.actuator.online) enterFault(TransformFault::ActuatorCommunication);
else if (!input.actuator.config_verified) enterFault(TransformFault::ActuatorConfigMismatch);
else if (!input.actuator.healthy || input.actuator.protection_flags) enterFault(TransformFault::ActuatorProtection);
else if (!input.actuator.command_accepted) enterFault(TransformFault::ActuatorCommandRejected);
else if (input.position.endpoint_confirmed) setStable(targetState(), input.position.source);
else if (progress == ProgressResult::NoProgress) enterFault(TransformFault::Stall);
else if (absolute_deadline_expired) enterFault(TransformFault::TransitionTimeout);
```

`enterFault()` never overwrites a nonzero fault. Fault output is `servo_enabled=false`, `servo_value=NAN`, `release_requested=true`, target `None`.

- [ ] **Step 4: Run green phase**

Run `make tests TESTFILTER='(TransformationStateMachine|TransformationPosition)' > /tmp/hx8-t2-green.log 2>&1`; expect exit 0.

- [ ] **Step 5: Commit**

```bash
git add src/lib/hybrid_control/TransformationStateMachine.*
git commit -m "feat[hybrid]: latch transformation stall faults"
```

### Task 3: uORB Contracts and Parameters

**Files:**
- Modify: `msg/HybridVehicleStatus.msg`
- Create: `msg/Hx8ServoCommand.msg`
- Create: `msg/Hx8ServoStatus.msg`
- Modify: `msg/CMakeLists.txt`
- Modify: `src/modules/hybrid_vehicle_control/hybrid_vehicle_control_params.c`

**Interfaces:** append without renumbering old constants:

```text
uint8 SENSOR_HX8 = 3
uint8 ACTUATOR_PWM = 0
uint8 ACTUATOR_HX8 = 1
uint8 TRANSFORM_FAULT_STALL = 7
uint8 TRANSFORM_FAULT_ACTUATOR_COMMUNICATION = 8
uint8 TRANSFORM_FAULT_ACTUATOR_PROTECTION = 9
uint8 TRANSFORM_FAULT_ACTUATOR_CONFIG_MISMATCH = 10
uint8 TRANSFORM_FAULT_ACTUATOR_COMMAND_REJECTED = 11
uint8 actuator_backend
float32 position_normalized
bool position_valid
bool actuator_online
bool actuator_healthy
bool actuator_config_verified
uint8 actuator_protection_flags
uint64 no_progress_elapsed
```

Create `Hx8ServoCommand.msg`:

```text
uint64 timestamp
uint32 sequence
uint8 COMMAND_NONE = 0
uint8 COMMAND_MOVE = 1
uint8 COMMAND_RELEASE = 2
uint8 COMMAND_HOLD = 3
uint8 type
uint8 servo_id
float32 target_angle_deg
uint16 move_time_ms
uint16 acceleration_time_ms
uint16 deceleration_time_ms
uint16 power_mw
```

Create `Hx8ServoStatus.msg`:

```text
uint64 timestamp
uint64 timestamp_sample
uint64 last_valid_response
uint32 command_sequence
uint32 rx_valid_count
uint32 rx_error_count
uint32 timeout_count
uint32 retry_count
uint8 servo_id
uint8 RESULT_NONE = 0
uint8 RESULT_ACCEPTED = 1
uint8 RESULT_REJECTED = 2
uint8 RESULT_TIMEOUT = 3
uint8 RESULT_PROTOCOL_ERROR = 4
uint8 STATUS_COMMAND_EXECUTING = 1
uint8 STATUS_COMMAND_ERROR = 2
uint8 PROTECTION_STALL = 4
uint8 PROTECTION_HIGH_VOLTAGE = 8
uint8 PROTECTION_LOW_VOLTAGE = 16
uint8 PROTECTION_OVERCURRENT = 32
uint8 PROTECTION_POWER = 64
uint8 PROTECTION_TEMPERATURE = 128
bool online
bool healthy
bool config_verified
bool command_accepted
bool persistent_write_active
float32 angle_deg
float32 voltage_v
float32 current_a
float32 power_w
float32 temperature_c
uint8 status_flags
uint8 protection_flags
uint8 command_result
```

Convert the protocol temperature ADC to Celsius with the vendor appendix table; do not treat raw ADC as degrees. Preserve the complete raw byte in `status_flags`; set `protection_flags = status_flags & 0xfc` so command-executing/error bits are not mislabeled as protection.

- [ ] **Step 1: Add exact messages and parameters**

Add `HYB_ACT_TYPE` enum default 0/reboot-required, `HYB_STALL_T` default 0.8 s range 0.1--5.0, and `HYB_STALL_D` default 0.02 range 0.001--0.25. Document `HYB_SENS_EN` as PWM-only external feedback behavior.

- [ ] **Step 2: Generate metadata**

Run `make tests TESTFILTER=TransformationStateMachine > /tmp/hx8-t3-gen.log 2>&1`; expect exit 0 and generated headers for both HX8 topics.

- [ ] **Step 3: Add constant consistency checks**

Add `static_assert` checks in `TransformationStateMachineTest.cpp` matching faults 7--11 to uORB. Run its test; expect exit 0.

- [ ] **Step 4: Commit**

```bash
git add msg/HybridVehicleStatus.msg msg/Hx8ServoCommand.msg msg/Hx8ServoStatus.msg msg/CMakeLists.txt src/modules/hybrid_vehicle_control/hybrid_vehicle_control_params.c src/lib/hybrid_control/TransformationStateMachineTest.cpp
git commit -m "feat[hybrid]: define servo backend status contracts"
```

### Task 4: PWM Sensor and Fault Integration

**Files:**
- Modify: `src/modules/hybrid_vehicle_control/hybrid_vehicle_control.hpp`
- Modify: `src/modules/hybrid_vehicle_control/hybrid_vehicle_control.cpp`
- Modify: `ROMFS/px4fmu_common/init.d/airframes/22001_quad_rover`
- Test: `src/lib/hybrid_control/TransformationPositionTest.cpp`

**Interfaces:** Consume full `magnetic_sensor.mag_{x,y,z}` and AS5600 angle. Emit one normalized `PositionSample`; prefer fresh AS5600, otherwise require both configured fresh TMAG device IDs. Reset progress on source change. Keep endpoint debounce/thresholds independent from the TMAG continuous ratio.

Replace the cache API exactly with:

```cpp
class TmagSampleCache {
public:
	void update(uint32_t device_id, const TmagVector &vector, uint64_t timestamp_us);
	bool validFor(int32_t device_id, uint64_t now_us, uint64_t timeout_us) const;
	const TmagVector &vector() const;
	uint64_t timestamp() const;
};
```

- [ ] **Step 1: Extend failing cache/source tests**

Replace scalar TMAG cache with `TmagVector`; test ID matching, non-finite axes, stale data, and AS5600-to-TMAG switch resetting progress.

- [ ] **Step 2: Verify red phase**

Run `make tests TESTFILTER='(TransformationPosition|TransformationStateMachine)' > /tmp/hx8-t4-red.log 2>&1`; expect failure until XYZ/source integration exists.

- [ ] **Step 3: Build the common position sample**

```cpp
if (encoder_valid) {
	position = {normalizeAs5600(_current_mechanism_angle, config.quad_angle, config.rover_angle),
		true, encoder_endpoint, SensorSource::As5600, _last_encoder_timestamp};
} else if (tmag_quad_valid && tmag_rover_valid) {
	position = _tmag_ratio_filter.update(_tmag_quad_cache.vector(), _tmag_rover_cache.vector(),
		true, true, tmag_endpoint, MIN(_tmag_quad_cache.timestamp(), _tmag_rover_cache.timestamp()));
} else if (config.sensors_enabled) {
	position = {NAN, false, false, SensorSource::None, 0};
}
```

Sensor-disabled PWM mode keeps existing time completion. Sensor-enabled invalid feedback enters sensor fault after `HYB_SENS_TO`.

- [ ] **Step 4: Enforce PWM fault output**

Initialize every `actuator_servos.control[]` to NaN and write the configured value only for PWM backend with no fault and state-machine enable. Fault, lockdown, manual lockdown, or force failsafe overrides RC commissioning. Preserve:

```sh
param set-default PWM_MAIN_FUNC8 201
param set-default PWM_MAIN_DIS8 0
param set-default PWM_MAIN_FAIL8 0
```

- [ ] **Step 5: Add explicit clear command**

Implement only `hybrid_vehicle_control clear_fault`; require `!armed && !prearmed`, valid configuration, and fresh selected feedback. Call `clearFault(true)`; never resume the target.

- [ ] **Step 6: Verify tests and hybrid compile**

Run focused tests, then `make zeroone_x6_hybrid > /tmp/hx8-t4-build.log 2>&1`; both must exit 0.

- [ ] **Step 7: Commit**

```bash
git add src/modules/hybrid_vehicle_control src/lib/hybrid_control/TransformationPositionTest.cpp ROMFS/px4fmu_common/init.d/airframes/22001_quad_rover
git commit -m "feat[hybrid]: protect PWM servo from transformation stalls"
```

### Task 5: Commander Red LED and Arming Rules

**Files:**
- Modify: `src/modules/commander/HybridStatusGuard.hpp`
- Modify: `src/modules/commander/CommanderHybridStatusTest.cpp`
- Modify: `src/modules/commander/Commander.hpp`
- Modify: `src/modules/commander/Commander.cpp`
- Modify: `src/modules/commander/HealthAndArmingChecks/checks/hybridCheck.cpp`
- Modify: `src/modules/commander/HealthAndArmingChecks/checks/hybridCheckTest.cpp`

**Interfaces:**

```cpp
enum class HybridRedPattern : uint8_t { Off, OverloadFast, FaultSlow, StallDouble, CombinedTriple };
HybridRedPattern hybridRedPattern(const hybrid_vehicle_status_s &, bool status_fresh, bool overload);
bool hybridRedLedOn(HybridRedPattern, hrt_abstime phase_us);
```

Waveforms: overload toggles each 50 ms; generic fault is 500 ms on/off; stall is 150 on, 150 off, 150 on, 1000 off; combined is three 150 ms on pulses separated by 150 ms off, then 1000 ms off. Stale hybrid status maps to generic fault and denies arming.

- [ ] **Step 1: Write failing truth-table/waveform tests**

Cover healthy, overload, generic, stall, overload+fault, and stale. Assert each 150/500 ms boundary.

- [ ] **Step 2: Verify red phase**

Run `make tests TESTFILTER=CommanderHybridStatus > /tmp/hx8-t5-red.log 2>&1`; expect missing helper failures.

- [ ] **Step 3: Implement Commander-only LED ownership**

Cache full hybrid status. Replace final overload-only red block with `hybridRedLedOn()`. No hybrid or HX8 code may call `BOARD_OVERLOAD_LED_*`.

- [ ] **Step 4: Extend arming checks**

PWM sensor mode requires confirmed position/no fault; PWM open-loop retains legacy stable-state rule. HX8 requires fresh normalized position, online, healthy, config verified, zero protection, known stable endpoint, and no fault. `UNKNOWN`, Transitioning, Fault, or stale denies arming.

- [ ] **Step 5: Verify green phase**

Run `make tests TESTFILTER='(CommanderHybridStatus|HybridCheck)' > /tmp/hx8-t5-green.log 2>&1`; expect exit 0.

- [ ] **Step 6: Commit**

```bash
git add src/modules/commander
git commit -m "feat[commander]: indicate transformation stall faults"
```

### Task 6: HX8 Frame Codec and Streaming Parser

**Files:**
- Create: `src/lib/hx8_servo/CMakeLists.txt`
- Create: `src/lib/hx8_servo/Hx8Protocol.hpp`
- Create: `src/lib/hx8_servo/Hx8Protocol.cpp`
- Create: `src/lib/hx8_servo/Hx8ProtocolTest.cpp`

**Interfaces:** little-endian; request header `0x12 0x4c`; response header `0x05 0x1c`; checksum is modulo-256 sum of preceding bytes. Support only `0x01`, `0x03`, `0x04`, `0x0A`, `0x0B`, `0x16`, `0x18`.

```cpp
namespace hx8 {
constexpr size_t MaxFrameSize = 64;
enum class CommandId : uint8_t { Ping=0x01, ParamRead=0x03, ParamWrite=0x04,
	AngleRead=0x0A, TimedMove=0x0B, Status=0x16, Stop=0x18 };
enum class ParseResult : uint8_t { NeedMore, FrameReady, BadLength, BadChecksum, WrongId, UnknownCommand };
struct Frame { CommandId command; uint8_t servo_id; uint8_t payload[56]; uint8_t payload_length; };
size_t encodeRequest(CommandId, uint8_t servo_id, const uint8_t *payload, size_t payload_length,
	uint8_t *out, size_t capacity);
class StreamParser {
public:
	ParseResult push(uint8_t byte, uint8_t expected_servo_id, Frame &frame);
	void reset();
};
}
```

- [ ] **Step 1: Write official golden-frame tests**

Include official ping:

```cpp
const uint8_t expected[] {0x12, 0x4c, 0x01, 0x01, 0x00, 0x60};
EXPECT_EQ(encodeRequest(CommandId::Ping, 0, nullptr, 0, encoded, sizeof(encoded)), sizeof(expected));
EXPECT_EQ(memcmp(encoded, expected, sizeof(expected)), 0);
```

Use these additional vendor golden frames:

```cpp
const uint8_t timed_move[] {0x12,0x4c,0x0b,0x0b,0x00,0x84,0x03,0x58,0x02,0x64,0x00,0xc8,0x00,0x00,0x00,0x81};
const uint8_t angle_request[] {0x12,0x4c,0x0a,0x01,0x00,0x69};
const uint8_t angle_response[] {0x05,0x1c,0x0a,0x03,0x00,0x86,0x03,0xb7};
const uint8_t status_request[] {0x12,0x4c,0x16,0x01,0x00,0x75};
const uint8_t status_response[] {0x05,0x1c,0x16,0x10,0x00,0x83,0x1e,0x1e,0x00,0xea,0x00,0x2c,0x07,0x01,0xaf,0x0b,0x00,0x00,0x00,0x00,0xde};
const uint8_t hold_example[] {0x12,0x4c,0x18,0x04,0x00,0x11,0x70,0x17,0x12};
const uint8_t release_zero_power[] {0x12,0x4c,0x18,0x04,0x00,0x10,0x00,0x00,0x8a};
```

Test garbage prefix, partial, concatenated, invalid length/checksum, wrong ID, unknown command, oversized payload, and TX echo followed by response.

- [ ] **Step 2: Verify red phase**

Run `make tests TESTFILTER=Hx8Protocol > /tmp/hx8-t6-red.log 2>&1`; expect failure because the library is absent.

- [ ] **Step 3: Implement bounded parser**

No dynamic allocation. Resynchronize without discarding a possible `0x05` header prefix. Treat request-header frames as optional echo, not responses. Reject oversize before payload copy.

- [ ] **Step 4: Register and verify green phase**

Create the library/test CMake entries; rerun the test and expect exit 0.

- [ ] **Step 5: Commit**

```bash
git add src/lib/hx8_servo
git commit -m "feat[hx8]: add UART protocol codec"
```

### Task 7: HX8 Scheduler, Protection Verification, and Gates

**Files:**
- Create: `src/lib/hx8_servo/Hx8Controller.hpp`
- Create: `src/lib/hx8_servo/Hx8Controller.cpp`
- Create: `src/lib/hx8_servo/Hx8ControllerTest.cpp`
- Modify: `src/lib/hx8_servo/CMakeLists.txt`

**Interfaces:** vendor parameter IDs are response 33, ID 34, baud 36, stall release 37, stall-power 38 mW, voltage low/high 39/40 mV, temperature limit 41 thermistor ADC (converted from the PX4 Celsius parameter), power 42 mW, current 43 mA, power-on lock 46.

```cpp
struct ProtectionConfig {
	uint8_t response_enabled{1};
	uint8_t stall_release_enabled{1};
	uint16_t stall_power_mw{0};
	uint16_t voltage_min_mv{9000};
	uint16_t voltage_max_mv{12600};
	uint16_t temperature_limit_c{0};
	uint16_t power_limit_mw{0};
	uint16_t current_limit_ma{0};
	uint8_t power_on_lock{0};
};
enum class RequestPriority : uint8_t { EmergencyRelease, Target, Status, Angle, Config, Diagnostic };
struct ControllerInput { uint64_t now_us; bool armed; bool prearmed; bool lockdown; bool failsafe; bool explicit_commissioning; };
struct MotionCommand {
	uint64_t timestamp_us; uint32_t sequence; uint8_t type; uint8_t servo_id;
	float target_angle_deg; uint16_t move_time_ms; uint16_t acceleration_time_ms;
	uint16_t deceleration_time_ms; uint16_t power_mw;
};
struct ControllerStatus {
	uint64_t sample_time_us; uint64_t last_valid_response_us; uint32_t command_sequence;
	uint32_t rx_valid_count; uint32_t rx_error_count; uint32_t timeout_count; uint32_t retry_count;
	uint8_t servo_id; bool online; bool healthy; bool config_verified; bool command_accepted;
	bool persistent_write_active; float angle_deg; float voltage_v; float current_a;
	float power_w; float temperature_c; uint8_t status_flags; uint8_t protection_flags; uint8_t command_result;
};
struct PendingRequest { bool valid; RequestPriority priority; CommandId command; uint8_t payload[32]; uint8_t payload_length; };
class Controller {
public:
	void setExpectedConfig(const ProtectionConfig &);
	void setTarget(const MotionCommand &);
	void requestRelease(uint32_t sequence);
	void requestPersistentWrite();
	PendingRequest update(const ControllerInput &);
	void acceptResponse(const Frame &, uint64_t now_us);
	void notifyTimeout(uint64_t now_us);
	const ControllerStatus &status() const;
};
```

Zero stall power, temperature limit, power, or current is an uncalibrated sentinel: `config_verified=false`, motion prohibited. Boot only reads. Persistent writes require fully disarmed, not prearmed, explicit commissioning, and per-item readback.

The documented protocol exposes no model or firmware-version field. The boot sequence therefore pings the configured ID and reads every required protection parameter; software verification is limited to ID, protocol behavior, and complete configuration match. The physical HX8-U45H-M model is a commissioning-record requirement.

- [ ] **Step 1: Write failing scheduler/gate tests**

Test one outstanding request, 20 ms spacing, release priority, target-on-change, bounded retry, 20 Hz moving/5 Hz stable monitoring, sequence/expiry rejection, lockdown/failsafe rejection, uncalibrated rejection, read-only boot, disarmed write, readback, mismatch.

- [ ] **Step 2: Verify red phase**

Run `make tests TESTFILTER=Hx8Controller > /tmp/hx8-t7-red.log 2>&1`; expect missing controller failure.

- [ ] **Step 3: Implement deterministic scheduler**

Use fixed arrays and HRT microseconds. Define `ResponseTimeoutUs=30000`, `MaxRetries=2`, `MinimumCommandSpacingUs=20000`. Exhausted retry sets offline/increments timeout but emergency release remains requestable and motion remains prohibited.

- [ ] **Step 4: Verify green phase**

Run `make tests TESTFILTER='Hx8(Protocol|Controller)' > /tmp/hx8-t7-green.log 2>&1`; expect exit 0.

- [ ] **Step 5: Commit**

```bash
git add src/lib/hx8_servo
git commit -m "feat[hx8]: add protected request scheduler"
```

### Task 8: PX4 HX8 UART Driver

**Files:**
- Create: `src/drivers/actuators/hx8_uart_servo/Kconfig`
- Create: `src/drivers/actuators/hx8_uart_servo/CMakeLists.txt`
- Create: `src/drivers/actuators/hx8_uart_servo/module.yaml`
- Create: `src/drivers/actuators/hx8_uart_servo/Hx8UartServo.hpp`
- Create: `src/drivers/actuators/hx8_uart_servo/Hx8UartServo.cpp`
- Create: `src/drivers/actuators/hx8_uart_servo/hx8_uart_servo_main.cpp`
- Modify: `boards/zeroone/x6/hybrid.px4board`

**Interfaces:** consume `hx8_servo_command`, `actuator_armed`, `vehicle_control_mode`; publish `hx8_servo_status` and critical events. Parameters: `HX8_SER_CFG` EXT2; `HX8_BAUD=115200`; `HX8_ID=0`; `HX8_ANG_QUD=0` and `HX8_ANG_ROV=0` (equal angles are invalid until calibrated); `HX8_MOVE_T=1000`, `HX8_ACC_T=100`, `HX8_DEC_T=100` ms; `HX8_PWR_LIM=0` (uncalibrated/invalid); expected config `HX8_CFG_SPWR/TEMP/PWR/CUR` default 0, `HX8_CFG_VMIN=9000`, `HX8_CFG_VMAX=12600`, `HX8_CFG_STL=1`, `HX8_CFG_RSP=1`, `HX8_CFG_BOOT=0`.

- [ ] **Step 1: Add build metadata and schema**

```yaml
module_name: HX8 UART Servo
serial_config:
  - command: hx8_uart_servo start -d ${SERIAL_DEV}
    port_config_param:
      name: HX8_SER_CFG
      group: HX8 UART Servo
      default: EXT2
```

All serial/ID/angle/timing/expected-config parameters are reboot-required. Validate supported baud, ID 0--254, single-turn angles, `move > accel + decel`, `move < HYBRID_TRANS_T`, and nonzero calibrated protection values.

- [ ] **Step 2: Implement UART safely**

Before `open()`, require `HYB_ACT_TYPE==1`; otherwise exit without touching the device. Open `O_RDWR|O_NOCTTY|O_NONBLOCK`, configure raw 8N1/no flow control, flush RX, use `write()` then `tcdrain()`, and parse bounded RX. Never call `TIOCSSINGLEWIRE` or GPIO APIs.

- [ ] **Step 3: Connect controller/uORB**

Schedule at 5 ms. Convert uORB commands to pure `MotionCommand`, feed arming/failsafe state to `Controller`, send one pending request, feed valid frames back with `hrt_absolute_time()`, and convert `ControllerStatus` to uORB with separate sample/publication timestamps.

- [ ] **Step 4: Add explicit commissioning CLI**

Commands: `start -d`, `status`, `config check`, `config write`, `stop`. `config write` requires fully disarmed explicit commissioning, writes each item once, reads it back, and returns nonzero on mismatch. Startup never writes NVM.

- [ ] **Step 5: Compile hybrid target**

Add `CONFIG_DRIVERS_ACTUATORS_HX8_UART_SERVO=y` only to `hybrid.px4board`. Run `make zeroone_x6_hybrid > /tmp/hx8-t8-build.log 2>&1`; expect exit 0 and generated `HX8_SER_CFG`.

- [ ] **Step 6: Commit**

```bash
git add src/drivers/actuators/hx8_uart_servo boards/zeroone/x6/hybrid.px4board
git commit -m "feat[hx8]: add half-duplex UART servo driver"
```

### Task 9: HX8 Backend Integration and Mutual Exclusion

**Files:**
- Modify: `src/modules/hybrid_vehicle_control/hybrid_vehicle_control.hpp`
- Modify: `src/modules/hybrid_vehicle_control/hybrid_vehicle_control.cpp`
- Modify: `ROMFS/px4fmu_common/init.d/rc.hybrid_apps`
- Modify: `src/modules/commander/HealthAndArmingChecks/checks/hybridCheck.cpp`
- Modify: `src/modules/commander/HealthAndArmingChecks/checks/hybridCheckTest.cpp`

**Interfaces:** Map fresh HX8 angle from configured Quad/Rover degrees to `PositionSample` with source HX8. New state target emits one sequenced move; fault emits bounded release; stable state monitors at 5 Hz. Hybrid status summarizes backend/position/health/config/protection; electrical details remain in HX8 status.

- [ ] **Step 1: Write failing backend tests**

Cover default PWM, HX8 normalization/freshness, one sequence per new target, fault release/block, HX8 all-NaN PWM, PWM no HX8 command, stable communication fault, and secondary release timeout preserving stall primary.

- [ ] **Step 2: Verify red phase**

Run `make tests TESTFILTER='(TransformationStateMachine|HybridCheck)' > /tmp/hx8-t9-red.log 2>&1`; expect failure until integrated.

- [ ] **Step 3: Integrate HX8 uORB adapter**

Require matching ID, fresh sample, finite angle, online/healthy/config verified, zero protection. HX8 internal feedback is mandatory regardless of `HYB_SENS_EN`; external sensors are neither fused nor required.

- [ ] **Step 4: Enforce M8/EXT2 exclusion**

HX8 mode publishes all-NaN servos; PWM mode never publishes HX8 commands. In `rc.hybrid_apps`, only query HX8 status when `param compare HYB_ACT_TYPE 1`; retain the driver's pre-open gate as authoritative.

- [ ] **Step 5: Complete arming/status**

Publish fresh hybrid status every cycle. Any fault forces `TRANSITION_FAULT` even during RC commissioning. HX8 clear additionally requires communication restored and recoverable protection cleared.

- [ ] **Step 6: Verify focused suite and build**

```bash
make tests TESTFILTER='(TransformationPosition|TransformationStateMachine|Hx8|CommanderHybridStatus|HybridCheck)' > /tmp/hx8-t9-tests.log 2>&1
make zeroone_x6_hybrid > /tmp/hx8-t9-build.log 2>&1
```

Expect both exit 0.

- [ ] **Step 7: Commit**

```bash
git add src/modules/hybrid_vehicle_control src/modules/commander/HealthAndArmingChecks ROMFS/px4fmu_common/init.d/rc.hybrid_apps
git commit -m "feat[hybrid]: integrate selectable HX8 servo backend"
```

### Task 10: Verification and Bench Acceptance

**Files:**
- Create: `docs/hybrid/hx8-u45h-m-commissioning.md`
- Modify only if evidence requires correction: files touched by Tasks 1--9

**Interfaces:** Produce separate host-test, firmware-build, PWM physical, and HX8 physical evidence. Never claim torque release from pulse/UART loss without current measurements.

- [ ] **Step 1: Write commissioning guide**

Document 9.0--12.6 V/5.5 A supply, common ground, FC TX/RX to automatic adapter and one bus wire to servo, EXT2, no DIR, endpoint calibration, mandatory nonzero protection calibration, `config check/write`, readback, fault clear, and backend rollback only by parameter plus reboot.

- [ ] **Step 2: Run fresh focused tests**

```bash
make tests TESTFILTER='(TransformationPosition|TransformationStateMachine|Hx8|CommanderHybridStatus|HybridCheck)' > /tmp/hx8-final-tests.log 2>&1
```

Expect exit 0 and zero failed tests.

- [ ] **Step 3: Build only hybrid firmware**

Run `make zeroone_x6_hybrid > /tmp/hx8-final-build.log 2>&1`; expect exit 0 and record artifact size. Do not build another target.

- [ ] **Step 4: PWM bench matrix**

For AS5600 and TMAG separately: record full travel/monotonicity; run 20 transitions each direction; induce stalls at several positions; prove detection by `HYB_STALL_T+100 ms`; scope M8 pulse loss within one control cycle; measure current before/after NaN; verify slow/double/combined LED. If current stays unsafe, report PWM thermal protection unproven.

- [ ] **Step 5: HX8 bench matrix**

Capture logic-analyzer turnaround, protection readback, telemetry, move acceptance, induced stall release, communication fault, UART removal during stall, voltage protection/power-cycle, and disarmed clear. Confirm onboard release before accepting the no-power-switch residual risk.

- [ ] **Step 6: Review complete diff**

```bash
git diff 1d334659711465485d747bc1d34fd485ff52b690...HEAD --check
git status --short
```

Expect no whitespace errors, only intentional files plus known untracked `.superpowers/`, and no staged `hardware_reference/`.

- [ ] **Step 7: Commit guide**

```bash
git add docs/hybrid/hx8-u45h-m-commissioning.md
git commit -m "docs[hybrid]: document HX8 servo commissioning"
```

## Execution Checkpoints

- A after Task 2: review pure position/stall semantics.
- B after Task 5: PWM and Commander path builds/tests independently of HX8.
- C after Task 7: HX8 protocol/scheduler host-tested before UART access.
- D after Task 9: both backends integrate and hybrid-only build succeeds.
- E after Task 10: software and physical evidence reported separately.

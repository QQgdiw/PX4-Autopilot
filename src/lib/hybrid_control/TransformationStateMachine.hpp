#pragma once

#include <cstdint>

#include "TransformationPosition.hpp"

namespace hybrid_control
{

enum class HybridState : uint8_t { Flying, TransitionToRover, Driving, TransitionToQuad, Unknown, Fault };
enum class HybridTarget : uint8_t { None, Flying, Driving };
enum class ActuatorBackend : uint8_t { Pwm = 0, Hx8 = 1 };
enum class TransformFault : uint8_t {
	None = 0,
	NoSensor = 1,
	SensorConflict = 2,
	SensorTimeout = 3,
	TransitionTimeout = 4,
	InvalidServoConfig = 5,
	InvalidConfiguration = 6,
	Stall = 7,
	ActuatorCommunication = 8,
	ActuatorProtection = 9,
	ActuatorConfigMismatch = 10,
	ActuatorCommandRejected = 11
};

struct ActuatorHealth {
	bool online{true};
	bool healthy{true};
	bool config_verified{true};
	bool command_accepted{true};
	uint8_t protection_flags{0};
};

struct TransformationConfig {
	bool sensors_enabled;
	int32_t configured_boot_state;
	float quad_servo;
	float rover_servo;
	float quad_angle;
	float rover_angle;
	float angle_tolerance;
	float sensor_timeout_s;
	float debounce_s;
	float max_transition_s;
	int32_t tmag_quad_device_id;
	int32_t tmag_rover_device_id;
	float tmag_quad_threshold;
	float tmag_rover_threshold;
	ActuatorBackend backend{ActuatorBackend::Pwm};
	float stall_timeout_s{0.8f};
	float stall_distance{0.05f};
	int32_t hx8_id{0}; float hx8_quad_angle{0.f}; float hx8_rover_angle{90.f};
	int32_t hx8_move_ms{1000}; int32_t hx8_acc_ms{100}; int32_t hx8_dec_ms{100}; int32_t hx8_power{1};
};

TransformFault validateTransformationConfig(const TransformationConfig &config);

class TransformationConfigTracker
{
public:
	bool update(const TransformationConfig &requested, bool safe_to_apply);
	const TransformationConfig &active() const { return _active; }
	bool hasActive() const { return _has_active; }
	bool hasPending() const { return _has_pending; }

private:
	TransformationConfig _active{};
	TransformationConfig _pending{};
	bool _has_active{false};
	bool _has_pending{false};
};

class ManualControlCache
{
public:
	void update(uint64_t timestamp, float value, uint64_t now_us, uint64_t timeout_us);
	bool fresh(uint64_t now_us, uint64_t timeout_us) const;
	float value() const { return _value; }

private:
	uint64_t _timestamp{0};
	float _value{0.f};
};

class TmagSampleCache
{
public:
	void update(uint32_t device_id, const TmagVector &vector, uint64_t timestamp_us);
	bool validFor(int32_t device_id, uint64_t now_us, uint64_t timeout_us) const;
	const TmagVector &vector() const { return _vector; }
	uint64_t timestamp() const { return _timestamp; }

private:
	uint32_t _device_id{0};
	TmagVector _vector{};
	uint64_t _timestamp{0};
	bool _initialized{false};
};

struct TransformationInput {
	uint64_t now_us;
	bool as5600_valid;
	float as5600_angle;
	bool tmag_quad_valid;
	bool tmag_quad_active;
	bool tmag_rover_valid;
	bool tmag_rover_active;
	PositionSample position{};
	ActuatorHealth actuator{};
	bool actuator_command_effective{false};
};

struct TransformationOutput {
	HybridState state;
	HybridTarget target;
	SensorSource source;
	TransformFault fault;
	bool position_confirmed;
	bool servo_enabled;
	float servo_value;
	bool release_requested{false};
	uint64_t no_progress_elapsed_us{0};
};

bool isTransformationFaulted(const TransformationOutput &output);
bool transformationPwmCommandEffective(ActuatorBackend backend, const TransformationOutput &output,
		bool manual_override, bool armed, bool prearmed, bool lockdown, bool manual_lockdown, bool force_failsafe);
bool stablePositionSafe(const TransformationOutput &output, bool sensors_enabled);
bool manualCommissioningPermitted(const TransformationOutput &output, bool armed, bool prearmed, bool manual_fresh);

class TransformationStateMachine
{
public:
	TransformationOutput initialize(const TransformationConfig &config, const TransformationInput &input);
	TransformationOutput request(HybridTarget target, uint64_t now_us);
	TransformationOutput update(const TransformationInput &input);
	TransformationOutput clearFault(bool disarmed);

private:
	enum class Endpoint : uint8_t { None, Quad, Rover };

	Endpoint as5600Endpoint(const TransformationInput &input) const;
	SensorSource stablePositionSource(const TransformationInput &input) const;
	bool sensorConflict(const TransformationInput &input) const;
	Endpoint hx8Endpoint(const TransformationInput &input) const;
	bool targetDetected(const TransformationInput &input);
	void enterFault(TransformFault fault);
	void setStable(HybridState state, SensorSource source);
	void refreshServoOutput();

	TransformationConfig _config{};
	TransformationOutput _output{HybridState::Unknown, HybridTarget::None, SensorSource::None,
				     TransformFault::None, false, false, 0.f};
	uint64_t _transition_started_us{0};
	uint64_t _target_detected_us{0};
	bool _target_detection_active{false};
	uint64_t _stable_mismatch_started_us{0};
	bool _stable_mismatch_active{false};
	DirectedProgressMonitor _progress_monitor{};
	SensorSource _progress_source{SensorSource::None};
	bool _initialized{false};
};

} // namespace hybrid_control

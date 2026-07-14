#pragma once

#include <cstdint>

namespace hybrid_control
{

enum class HybridState : uint8_t { Flying, TransitionToRover, Driving, TransitionToQuad, Unknown, Fault };
enum class HybridTarget : uint8_t { None, Flying, Driving };
enum class SensorSource : uint8_t { None, As5600, Tmag5273 };
enum class TransformFault : uint8_t {
	None, NoSensor, SensorConflict, SensorTimeout, TransitionTimeout, InvalidServoConfig
};

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
	bool sensorConflict(const TransformationInput &input) const;
	bool targetDetected(const TransformationInput &input);
	void enterFault(TransformFault fault);
	void setStable(HybridState state, SensorSource source);
	void refreshServoOutput();

	TransformationConfig _config{};
	TransformationOutput _output{HybridState::Unknown, HybridTarget::None, SensorSource::None,
				     TransformFault::None, false, 0.f};
	uint64_t _transition_started_us{0};
	uint64_t _target_detected_us{0};
	bool _target_detection_active{false};
	bool _initialized{false};
};

} // namespace hybrid_control

#include "TransformationStateMachine.hpp"

#include <cmath>
#include <cstring>

namespace hybrid_control
{

namespace
{
bool sameFloat(float lhs, float rhs)
{
	uint32_t lhs_bits{};
	uint32_t rhs_bits{};
	static_assert(sizeof(lhs_bits) == sizeof(lhs), "unexpected float size");
	std::memcpy(&lhs_bits, &lhs, sizeof(lhs));
	std::memcpy(&rhs_bits, &rhs, sizeof(rhs));
	return lhs_bits == rhs_bits;
}

bool sameConfig(const TransformationConfig &lhs, const TransformationConfig &rhs)
{
	return lhs.sensors_enabled == rhs.sensors_enabled
	       && lhs.configured_boot_state == rhs.configured_boot_state
	       && sameFloat(lhs.quad_servo, rhs.quad_servo)
	       && sameFloat(lhs.rover_servo, rhs.rover_servo)
	       && sameFloat(lhs.quad_angle, rhs.quad_angle)
	       && sameFloat(lhs.rover_angle, rhs.rover_angle)
	       && sameFloat(lhs.angle_tolerance, rhs.angle_tolerance)
	       && sameFloat(lhs.sensor_timeout_s, rhs.sensor_timeout_s)
	       && sameFloat(lhs.debounce_s, rhs.debounce_s)
	       && sameFloat(lhs.max_transition_s, rhs.max_transition_s)
	       && lhs.tmag_quad_device_id == rhs.tmag_quad_device_id
	       && lhs.tmag_rover_device_id == rhs.tmag_rover_device_id
	       && sameFloat(lhs.tmag_quad_threshold, rhs.tmag_quad_threshold)
	       && sameFloat(lhs.tmag_rover_threshold, rhs.tmag_rover_threshold);
}

bool inRange(float value, float minimum, float maximum)
{
	return std::isfinite(value) && value >= minimum && value <= maximum;
}

uint64_t secondsToMicroseconds(float seconds)
{
	return static_cast<uint64_t>(seconds * 1000000.f);
}
}

TransformFault validateTransformationConfig(const TransformationConfig &config)
{
	if (!std::isfinite(config.quad_servo) || !std::isfinite(config.rover_servo)
	    || config.quad_servo < -1.f || config.quad_servo > 1.f
	    || config.rover_servo < -1.f || config.rover_servo > 1.f
	    || std::fabs(config.quad_servo - config.rover_servo) < 0.1f) {
		return TransformFault::InvalidServoConfig;
	}

	if ((config.configured_boot_state != 0 && config.configured_boot_state != 1)
	    || !inRange(config.quad_angle, 0.f, 6.28f)
	    || !inRange(config.rover_angle, 0.f, 6.28f)
	    || !inRange(config.angle_tolerance, 0.f, 3.14f)
	    || !inRange(config.sensor_timeout_s, 0.01f, 5.f)
	    || !inRange(config.debounce_s, 0.f, 2.f)
	    || !inRange(config.max_transition_s, 0.1f, 10.f)
	    || config.tmag_quad_device_id < 0
	    || config.tmag_rover_device_id < 0
	    || config.tmag_quad_device_id == config.tmag_rover_device_id
	    || !inRange(config.tmag_quad_threshold, 0.f, 100.f)
	    || !inRange(config.tmag_rover_threshold, 0.f, 100.f)) {
		return TransformFault::InvalidConfiguration;
	}

	return TransformFault::None;
}

bool TransformationConfigTracker::update(const TransformationConfig &requested, bool safe_to_apply)
{
	if (!_has_active) {
		_pending = requested;
		_has_pending = true;

		if (!safe_to_apply) {
			return false;
		}

		_active = _pending;
		_has_active = true;
		_has_pending = false;
		return true;
	}

	if (sameConfig(requested, _active)) {
		_has_pending = false;
		return false;
	}

	_pending = requested;
	_has_pending = true;

	if (!safe_to_apply) {
		return false;
	}

	_active = _pending;
	_has_pending = false;
	return true;
}

void ManualControlCache::update(uint64_t timestamp, float value, uint64_t now_us, uint64_t timeout_us)
{
	if (timestamp != 0 && now_us >= timestamp && now_us - timestamp <= timeout_us && std::isfinite(value)) {
		_timestamp = timestamp;
		_value = value;

	} else {
		_timestamp = 0;
		_value = 0.f;
	}
}

bool ManualControlCache::fresh(uint64_t now_us, uint64_t timeout_us) const
{
	return _timestamp != 0 && now_us >= _timestamp && now_us - _timestamp <= timeout_us;
}

void TmagSampleCache::update(uint32_t device_id, float value, uint64_t timestamp)
{
	_device_id = device_id;
	_value = value;
	_timestamp = timestamp;
	_initialized = true;
}

bool TmagSampleCache::validFor(int32_t device_id, uint64_t now_us, uint64_t timeout_us) const
{
	return _initialized && device_id >= 0 && _device_id == static_cast<uint32_t>(device_id)
	       && _timestamp != 0 && now_us >= _timestamp && now_us - _timestamp <= timeout_us
	       && std::isfinite(_value);
}

TransformationStateMachine::Endpoint TransformationStateMachine::as5600Endpoint(const TransformationInput &input) const
{
	if (!input.as5600_valid) {
		return Endpoint::None;
	}

	if (std::fabs(input.as5600_angle - _config.quad_angle) <= _config.angle_tolerance) {
		return Endpoint::Quad;
	}

	if (std::fabs(input.as5600_angle - _config.rover_angle) <= _config.angle_tolerance) {
		return Endpoint::Rover;
	}

	return Endpoint::None;
}

bool TransformationStateMachine::sensorConflict(const TransformationInput &input) const
{
	const bool quad_active = input.tmag_quad_valid && input.tmag_quad_active;
	const bool rover_active = input.tmag_rover_valid && input.tmag_rover_active;

	if (quad_active && rover_active) {
		return true;
	}

	const Endpoint endpoint = as5600Endpoint(input);
	return (endpoint == Endpoint::Quad && rover_active) || (endpoint == Endpoint::Rover && quad_active);
}

void TransformationStateMachine::refreshServoOutput()
{
	switch (_output.state) {
	case HybridState::Flying:
	case HybridState::TransitionToQuad:
		_output.servo_enabled = true;
		_output.servo_value = _config.quad_servo;
		break;

	case HybridState::Driving:
	case HybridState::TransitionToRover:
		_output.servo_enabled = true;
		_output.servo_value = _config.rover_servo;
		break;

	case HybridState::Unknown:
	case HybridState::Fault:
		_output.servo_enabled = false;
		_output.servo_value = 0.f;
		break;
	}
}

void TransformationStateMachine::enterFault(TransformFault fault)
{
	_output.state = HybridState::Fault;
	_output.fault = fault;
	_target_detection_active = false;
	refreshServoOutput();
}

void TransformationStateMachine::setStable(HybridState state, SensorSource source)
{
	_output.state = state;
	_output.target = state == HybridState::Flying ? HybridTarget::Flying : HybridTarget::Driving;
	_output.source = source;
	_output.fault = TransformFault::None;
	_target_detection_active = false;
	refreshServoOutput();
}

TransformationOutput TransformationStateMachine::initialize(const TransformationConfig &config,
		const TransformationInput &input)
{
	_config = config;
	_initialized = true;
	_output = {HybridState::Unknown, HybridTarget::None, SensorSource::None, TransformFault::None, false, 0.f};
	_target_detection_active = false;

	const TransformFault configuration_fault = validateTransformationConfig(config);

	if (configuration_fault != TransformFault::None) {
		enterFault(configuration_fault);
		return _output;
	}

	if (!config.sensors_enabled) {
		setStable(config.configured_boot_state == 0 ? HybridState::Flying : HybridState::Driving, SensorSource::None);

		return _output;
	}

	if (sensorConflict(input)) {
		enterFault(TransformFault::SensorConflict);
		return _output;
	}

	const Endpoint endpoint = as5600Endpoint(input);

	if (endpoint == Endpoint::Quad) {
		setStable(HybridState::Flying, SensorSource::As5600);

	} else if (endpoint == Endpoint::Rover) {
		setStable(HybridState::Driving, SensorSource::As5600);

	} else if (!input.as5600_valid && input.tmag_quad_valid && input.tmag_quad_active) {
		setStable(HybridState::Flying, SensorSource::Tmag5273);

	} else if (!input.as5600_valid && input.tmag_rover_valid && input.tmag_rover_active) {
		setStable(HybridState::Driving, SensorSource::Tmag5273);
	}

	return _output;
}

TransformationOutput TransformationStateMachine::request(HybridTarget target, uint64_t now_us)
{
	if (!_initialized || _output.state == HybridState::Fault || target == HybridTarget::None) {
		return _output;
	}

	if ((target == HybridTarget::Flying && _output.state == HybridState::Flying)
	    || (target == HybridTarget::Driving && _output.state == HybridState::Driving)
	    || (target == HybridTarget::Flying && _output.state == HybridState::TransitionToQuad)
	    || (target == HybridTarget::Driving && _output.state == HybridState::TransitionToRover)) {
		return _output;
	}

	_output.target = target;
	_output.fault = TransformFault::None;
	_output.source = SensorSource::None;
	_target_detection_active = false;
	_transition_started_us = now_us;

	_output.state = target == HybridTarget::Flying ? HybridState::TransitionToQuad : HybridState::TransitionToRover;
	refreshServoOutput();
	return _output;
}

bool TransformationStateMachine::targetDetected(const TransformationInput &input)
{
	const Endpoint wanted = _output.target == HybridTarget::Flying ? Endpoint::Quad : Endpoint::Rover;

	if (input.as5600_valid) {
		_output.source = SensorSource::As5600;
		return as5600Endpoint(input) == wanted;
	}

	const bool target_tmag = wanted == Endpoint::Quad
				 ? input.tmag_quad_valid && input.tmag_quad_active
				 : input.tmag_rover_valid && input.tmag_rover_active;
	_output.source = target_tmag ? SensorSource::Tmag5273 : SensorSource::None;
	return target_tmag;
}

TransformationOutput TransformationStateMachine::update(const TransformationInput &input)
{
	if (!_initialized || _output.state == HybridState::Fault) {
		return _output;
	}

	if (_config.sensors_enabled && sensorConflict(input)) {
		enterFault(TransformFault::SensorConflict);
		return _output;
	}

	const bool transitioning = _output.state == HybridState::TransitionToQuad
				   || _output.state == HybridState::TransitionToRover;

	if (!transitioning) {
		if (_config.sensors_enabled && _output.state == HybridState::Unknown) {
			const Endpoint endpoint = as5600Endpoint(input);

			if (endpoint == Endpoint::Quad) {
				setStable(HybridState::Flying, SensorSource::As5600);

			} else if (endpoint == Endpoint::Rover) {
				setStable(HybridState::Driving, SensorSource::As5600);

			} else if (!input.as5600_valid && input.tmag_quad_valid && input.tmag_quad_active) {
				setStable(HybridState::Flying, SensorSource::Tmag5273);

			} else if (!input.as5600_valid && input.tmag_rover_valid && input.tmag_rover_active) {
				setStable(HybridState::Driving, SensorSource::Tmag5273);
			}
		}

		return _output;
	}

	const uint64_t elapsed = input.now_us - _transition_started_us;
	const uint64_t max_transition_us = secondsToMicroseconds(_config.max_transition_s);

	if (!_config.sensors_enabled) {
		if (elapsed >= max_transition_us) {
			setStable(_output.target == HybridTarget::Flying ? HybridState::Flying : HybridState::Driving,
				  SensorSource::None);
		}

		return _output;
	}

	const bool detected = targetDetected(input);

	if (detected) {
		if (!_target_detection_active) {
			_target_detection_active = true;
			_target_detected_us = input.now_us;
		}

		if (input.now_us - _target_detected_us >= secondsToMicroseconds(_config.debounce_s)) {
			setStable(_output.target == HybridTarget::Flying ? HybridState::Flying : HybridState::Driving,
				  _output.source);
			return _output;
		}

	} else {
		_target_detection_active = false;
	}

	if (!input.as5600_valid && _output.source == SensorSource::None
	    && elapsed >= secondsToMicroseconds(_config.sensor_timeout_s)) {
		enterFault(TransformFault::SensorTimeout);

	} else if (elapsed >= max_transition_us) {
		enterFault(TransformFault::TransitionTimeout);
	}

	return _output;
}

TransformationOutput TransformationStateMachine::clearFault(bool disarmed)
{
	if (_output.state != HybridState::Fault || !disarmed
	    || _output.fault == TransformFault::InvalidServoConfig
	    || _output.fault == TransformFault::InvalidConfiguration) {
		return _output;
	}

	_output.fault = TransformFault::None;
	_output.source = SensorSource::None;
	_output.target = HybridTarget::None;
	_output.state = _config.sensors_enabled ? HybridState::Unknown
			: (_config.configured_boot_state == 0 ? HybridState::Flying : HybridState::Driving);

	if (_output.state != HybridState::Flying && _output.state != HybridState::Driving) {
		_output.state = HybridState::Unknown;
	}

	refreshServoOutput();
	return _output;
}

} // namespace hybrid_control

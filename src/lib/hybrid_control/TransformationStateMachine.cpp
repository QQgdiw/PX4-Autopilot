#include "TransformationStateMachine.hpp"

#include <cmath>

namespace hybrid_control
{

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

	if (!std::isfinite(config.quad_servo) || !std::isfinite(config.rover_servo)
	    || config.quad_servo < -1.f || config.quad_servo > 1.f
	    || config.rover_servo < -1.f || config.rover_servo > 1.f
	    || std::fabs(config.quad_servo - config.rover_servo) < 0.1f) {
		enterFault(TransformFault::InvalidServoConfig);
		return _output;
	}

	if (!config.sensors_enabled) {
		if (config.configured_boot_state == HybridState::Flying || config.configured_boot_state == HybridState::Driving) {
			setStable(config.configured_boot_state, SensorSource::None);
		}

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

	_output.target = target;
	_output.fault = TransformFault::None;
	_output.source = SensorSource::None;
	_target_detection_active = false;
	_transition_started_us = now_us;

	if (target == HybridTarget::Flying && _output.state == HybridState::Flying) {
		return _output;
	}

	if (target == HybridTarget::Driving && _output.state == HybridState::Driving) {
		return _output;
	}

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

	if (!_config.sensors_enabled) {
		if (elapsed >= _config.max_transition_us) {
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

		if (input.now_us - _target_detected_us >= _config.debounce_us) {
			setStable(_output.target == HybridTarget::Flying ? HybridState::Flying : HybridState::Driving,
				_output.source);
			return _output;
		}

	} else {
		_target_detection_active = false;
	}

	if (!input.as5600_valid && _output.source == SensorSource::None && elapsed >= _config.sensor_timeout_us) {
		enterFault(TransformFault::SensorTimeout);

	} else if (elapsed >= _config.max_transition_us) {
		enterFault(TransformFault::TransitionTimeout);
	}

	return _output;
}

TransformationOutput TransformationStateMachine::clearFault(bool disarmed)
{
	if (_output.state != HybridState::Fault || !disarmed) {
		return _output;
	}

	_output.fault = TransformFault::None;
	_output.source = SensorSource::None;
	_output.target = HybridTarget::None;
	_output.state = _config.sensors_enabled ? HybridState::Unknown : _config.configured_boot_state;

	if (_output.state != HybridState::Flying && _output.state != HybridState::Driving) {
		_output.state = HybridState::Unknown;
	}

	refreshServoOutput();
	return _output;
}

} // namespace hybrid_control

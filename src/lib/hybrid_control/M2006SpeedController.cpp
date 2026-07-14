#include "M2006SpeedController.hpp"

#include <algorithm>

namespace hybrid_control
{

void M2006SpeedController::configure(const SpeedControllerConfig &config)
{
	_config = config;
	reset();
}

void M2006SpeedController::reset()
{
	_target_rpm = 0.f;
	_integral = 0.f;
	_last_measurement = 0.f;
	_measurement_valid = false;
}

int16_t M2006SpeedController::update(float normalized, float measured_rpm, float dt, bool enabled)
{
	if (!enabled) {
		reset();
		return 0;
	}

	const float limited_normalized = std::max(-1.f, std::min(normalized, 1.f));
	const float desired_rpm = limited_normalized * _config.max_rpm;
	const float max_target_change = std::max(_config.rpm_slew * dt, 0.f);
	const float target_change = std::max(-max_target_change,
					     std::min(desired_rpm - _target_rpm, max_target_change));
	_target_rpm += target_change;

	const float error = _target_rpm - measured_rpm;
	const float derivative = dt > 0.f && _measurement_valid ? -(measured_rpm - _last_measurement) / dt : 0.f;
	_last_measurement = measured_rpm;
	_measurement_valid = true;

	const float output_without_integral = _config.kp * error + _config.kd * derivative
					      + _config.kff * _target_rpm;
	const float integral_candidate = dt > 0.f ? _integral + _config.ki * error * dt : _integral;
	const float current_limit = std::max(static_cast<float>(_config.current_limit), 0.f);
	const float output_candidate = output_without_integral + integral_candidate;

	if ((output_candidate <= current_limit && output_candidate >= -current_limit)
	    || (output_candidate > current_limit && error < 0.f)
	    || (output_candidate < -current_limit && error > 0.f)) {
		_integral = integral_candidate;
	}

	const float output = std::max(-current_limit,
				      std::min(output_without_integral + _integral, current_limit));
	return static_cast<int16_t>(output);
}

} // namespace hybrid_control

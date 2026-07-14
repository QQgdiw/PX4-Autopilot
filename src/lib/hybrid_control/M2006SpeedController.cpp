#include "M2006SpeedController.hpp"

#include <cmath>

namespace hybrid_control
{

namespace
{

float clampFloat(const float value, const float lower, const float upper)
{
	return value < lower ? lower : (value > upper ? upper : value);
}

} // namespace

void M2006SpeedController::configure(const SpeedControllerConfig &config)
{
	if (std::isfinite(config.max_rpm) && std::isfinite(config.kp) && std::isfinite(config.ki)
	    && std::isfinite(config.kd) && std::isfinite(config.kff) && std::isfinite(config.rpm_slew)) {
		_config = config;

	} else {
		_config = {};
	}

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
	if (!enabled || !std::isfinite(normalized) || !std::isfinite(measured_rpm) || !std::isfinite(dt)) {
		reset();
		return 0;
	}

	const float limited_normalized = clampFloat(normalized, -1.f, 1.f);
	const float desired_rpm = limited_normalized * _config.max_rpm;
	const float max_target_change = _config.rpm_slew * dt > 0.f ? _config.rpm_slew * dt : 0.f;

	if (!std::isfinite(desired_rpm) || !std::isfinite(max_target_change)) {
		reset();
		return 0;
	}

	const float target_change = clampFloat(desired_rpm - _target_rpm, -max_target_change, max_target_change);
	const float target_rpm = _target_rpm + target_change;
	const float error = target_rpm - measured_rpm;
	const float derivative = dt > 0.f && _measurement_valid ? -(measured_rpm - _last_measurement) / dt : 0.f;

	if (!std::isfinite(target_rpm) || !std::isfinite(error) || !std::isfinite(derivative)) {
		reset();
		return 0;
	}

	const float output_without_integral = _config.kp * error + _config.kd * derivative
					      + _config.kff * target_rpm;
	const float integral_candidate = dt > 0.f ? _integral + _config.ki * error * dt : _integral;
	const float current_limit = _config.current_limit > 0 ? static_cast<float>(_config.current_limit) : 0.f;
	const float output_candidate = output_without_integral + integral_candidate;

	if (!std::isfinite(output_without_integral) || !std::isfinite(integral_candidate)
	    || !std::isfinite(output_candidate)) {
		reset();
		return 0;
	}

	float integral = _integral;

	if ((output_candidate <= current_limit && output_candidate >= -current_limit)
	    || (output_candidate > current_limit && error < 0.f)
	    || (output_candidate < -current_limit && error > 0.f)) {
		integral = integral_candidate;
	}

	const float unclamped_output = output_without_integral + integral;

	if (!std::isfinite(unclamped_output)) {
		reset();
		return 0;
	}

	const float output = clampFloat(unclamped_output, -current_limit, current_limit);

	if (!std::isfinite(output)) {
		reset();
		return 0;
	}

	_target_rpm = target_rpm;
	_integral = integral;
	_last_measurement = measured_rpm;
	_measurement_valid = true;
	return static_cast<int16_t>(output);
}

} // namespace hybrid_control

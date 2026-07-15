#pragma once

#include <cstdint>

namespace hybrid_control
{

struct SpeedControllerConfig {
	float max_rpm;
	float kp;
	float ki;
	float kd;
	float kff;
	int16_t current_limit;
	float rpm_slew;
};

class M2006SpeedController
{
public:
	bool configure(const SpeedControllerConfig &config);
	void reset();
	int16_t update(float normalized, float measured_rpm, float dt, bool enabled);
	float targetRpm() const { return _target_rpm; }
	float integral() const { return _integral; }
	bool valid() const { return _valid; }

private:
	SpeedControllerConfig _config{};
	float _target_rpm{0.f};
	float _integral{0.f};
	float _last_measurement{0.f};
	bool _measurement_valid{false};
	bool _valid{false};
};

} // namespace hybrid_control

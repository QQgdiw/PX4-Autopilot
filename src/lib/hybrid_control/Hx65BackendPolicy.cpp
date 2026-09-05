#include "Hx65BackendPolicy.hpp"

#include <cmath>

namespace hybrid_control
{

bool Hx65BackendPolicy::parametersValid(int32_t left_id, int32_t right_id, int32_t hx8_id,
					int32_t left_quad, int32_t left_rover, int32_t right_quad, int32_t right_rover,
					int32_t speed, int32_t acceleration, int32_t tolerance)
{
	const auto endpoint = [](int32_t value) { return value >= -30719 && value <= 30719; };
	return left_id >= 0 && left_id <= 253 && right_id >= 0 && right_id <= 253 && left_id != right_id
	       && left_id != hx8_id && right_id != hx8_id && endpoint(left_quad) && endpoint(left_rover)
	       && endpoint(right_quad) && endpoint(right_rover) && left_quad != left_rover
	       && right_quad != right_rover && speed > 0 && speed <= 3400 && acceleration >= 0
	       && acceleration <= 254 && tolerance > 0 && tolerance < std::abs(left_rover - left_quad) / 2
	       && tolerance < std::abs(right_rover - right_quad) / 2;
}

bool Hx65BackendPolicy::statusUsable(bool left_online, bool right_online, bool left_healthy, bool right_healthy,
				     bool left_verified, bool right_verified, bool motion_config_valid, bool fresh)
{
	return left_online && right_online && left_healthy && right_healthy && left_verified && right_verified
	       && motion_config_valid && fresh;
}

float Hx65BackendPolicy::normalizePair(int16_t left, int16_t right, int16_t left_quad, int16_t left_rover,
				       int16_t right_quad, int16_t right_rover)
{
	const float left_span = static_cast<float>(left_rover - left_quad);
	const float right_span = static_cast<float>(right_rover - right_quad);

	if (fabsf(left_span) < 1.f || fabsf(right_span) < 1.f) {
		return NAN;
	}

	const float left_normalized = static_cast<float>(left - left_quad) / left_span;
	const float right_normalized = static_cast<float>(right - right_quad) / right_span;
	return 0.5f * (left_normalized + right_normalized);
}

float Hx65BackendPolicy::normalizedSkew(int16_t left, int16_t right, int16_t left_quad, int16_t left_rover,
					int16_t right_quad, int16_t right_rover)
{
	const float left_span = static_cast<float>(left_rover - left_quad);
	const float right_span = static_cast<float>(right_rover - right_quad);

	if (fabsf(left_span) < 1.f || fabsf(right_span) < 1.f) {
		return NAN;
	}

	return fabsf(static_cast<float>(left - left_quad) / left_span
		     - static_cast<float>(right - right_quad) / right_span);
}

bool Hx65BackendPolicy::endpointMatches(int16_t left, int16_t right, bool rover, int16_t left_quad,
					int16_t left_rover, int16_t right_quad, int16_t right_rover, int32_t tolerance)
{
	const int32_t left_target = rover ? left_rover : left_quad;
	const int32_t right_target = rover ? right_rover : right_quad;
	return tolerance >= 0 && std::abs(static_cast<int32_t>(left) - left_target) <= tolerance
	       && std::abs(static_cast<int32_t>(right) - right_target) <= tolerance;
}

} // namespace hybrid_control

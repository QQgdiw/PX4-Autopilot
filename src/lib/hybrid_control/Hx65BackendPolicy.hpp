#pragma once

#include <cstdint>

namespace hybrid_control
{

struct Hx65BackendPolicy {
	static bool parametersValid(int32_t left_id, int32_t right_id, int32_t hx8_id,
				    int32_t left_quad, int32_t left_rover, int32_t right_quad, int32_t right_rover,
				    int32_t speed, int32_t acceleration, int32_t tolerance);
	static bool statusUsable(bool left_online, bool right_online, bool left_healthy, bool right_healthy,
				 bool left_verified, bool right_verified, bool motion_config_valid, bool fresh);
	static float normalizePair(int16_t left, int16_t right, int16_t left_quad, int16_t left_rover,
				   int16_t right_quad, int16_t right_rover);
	static float normalizedSkew(int16_t left, int16_t right, int16_t left_quad, int16_t left_rover,
				    int16_t right_quad, int16_t right_rover);
	static bool endpointMatches(int16_t left, int16_t right, bool rover, int16_t left_quad,
				    int16_t left_rover, int16_t right_quad, int16_t right_rover, int32_t tolerance);
};

} // namespace hybrid_control

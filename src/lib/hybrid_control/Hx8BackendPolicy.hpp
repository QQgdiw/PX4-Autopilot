#pragma once

#include <cstdint>

#include "TransformationStateMachine.hpp"

namespace hybrid_control
{

struct Hx8BackendPolicy
{
	static bool statusUsable(uint8_t status_id, uint8_t configured_id, bool online, bool healthy,
				bool config_verified, uint8_t protection_flags, bool fresh, float angle_deg);
	static float normalizeAngle(float angle_deg, float quad_deg, float rover_deg);
	static bool endpointMatches(float normalized, bool driving_target, float tolerance = 0.02f);
	static bool endpointMatchesAngleTolerance(float normalized, bool driving_target, float tolerance_rad,
				float quad_deg, float rover_deg);
	static bool parametersValid(int32_t id, float quad_deg, float rover_deg, int32_t move, int32_t acc,
				int32_t dec, int32_t power, float transition_s);
	static bool commandAccepted(bool accepted, uint8_t result, uint8_t pending_result,
				uint8_t accepted_result);
};

enum class Hx8CommandAction : uint8_t { None, Move, Hold, Release };

struct Hx8CommandDecision {
	Hx8CommandAction action{Hx8CommandAction::None};
	HybridTarget target{HybridTarget::None};
	uint32_t sequence{0};
};

class Hx8CommandPolicy
{
public:
	Hx8CommandDecision update(ActuatorBackend backend, const TransformationOutput &output, uint64_t now_us);
	void resetAfterFaultClear();
	uint32_t lastMotionSequence() const { return _last_motion_sequence; }
	bool motionAcknowledged(uint32_t status_sequence, bool accepted, uint8_t result, uint8_t accepted_result) const;

private:
	uint32_t nextSequence();
	uint32_t _sequence{0};
	uint32_t _last_motion_sequence{0};
	HybridTarget _last_target{HybridTarget::None};
	uint64_t _last_hold_us{0};
	uint8_t _release_attempts{0};
	uint64_t _last_release_us{0};
	bool _release_sent{false};
};

} // namespace hybrid_control

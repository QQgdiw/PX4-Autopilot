#include "Hx8BackendPolicy.hpp"

#include <cmath>

#include "TransformationPosition.hpp"

namespace hybrid_control
{

bool Hx8BackendPolicy::statusUsable(uint8_t status_id, uint8_t configured_id, bool online, bool healthy,
		bool config_verified, uint8_t protection_flags, bool fresh, float angle_deg)
{
	return status_id == configured_id && online && healthy && config_verified && protection_flags == 0 && fresh
	       && std::isfinite(angle_deg);
}

float Hx8BackendPolicy::normalizeAngle(float angle_deg, float quad_deg, float rover_deg)
{
	return normalizeAs5600(angle_deg * static_cast<float>(M_PI / 180.0),
				quad_deg * static_cast<float>(M_PI / 180.0), rover_deg * static_cast<float>(M_PI / 180.0));
}

bool Hx8BackendPolicy::endpointMatches(float normalized, bool driving_target, float tolerance)
{
	const float target = driving_target ? 1.f : 0.f;
	return std::isfinite(normalized) && std::isfinite(tolerance) && tolerance >= 0.f
	       && fabsf(normalized - target) <= tolerance;
}

bool Hx8BackendPolicy::commandAccepted(bool accepted, uint8_t result, uint8_t rejected_result,
		uint8_t timeout_result, uint8_t protocol_error_result)
{
	return accepted || (result != rejected_result && result != timeout_result && result != protocol_error_result);
}

uint32_t Hx8CommandPolicy::nextSequence()
{
	if (++_sequence == 0) {
		++_sequence;
	}

	return _sequence;
}

Hx8CommandDecision Hx8CommandPolicy::update(ActuatorBackend backend, const TransformationOutput &output, uint64_t now_us)
{
	if (backend != ActuatorBackend::Hx8) {
		return {};
	}

	if (isTransformationFaulted(output)) {
		if (_release_attempts >= 3) {
			return {};
		}

		++_release_attempts;
		return {Hx8CommandAction::Release, HybridTarget::None, nextSequence()};
	}

	_release_attempts = 0;

	if ((output.target == HybridTarget::Flying || output.target == HybridTarget::Driving)
	    && (output.state == HybridState::TransitionToQuad || output.state == HybridState::TransitionToRover)
	    && output.target != _last_target) {
		_last_target = output.target;
		_last_motion_sequence = nextSequence();
		return {Hx8CommandAction::Move, output.target, _last_motion_sequence};
	}

	const bool stable = output.state == HybridState::Flying || output.state == HybridState::Driving;

	if (stable && now_us >= _last_hold_us && now_us - _last_hold_us >= 200'000) {
		_last_hold_us = now_us;
		const HybridTarget target = output.state == HybridState::Flying ? HybridTarget::Flying : HybridTarget::Driving;
		return {Hx8CommandAction::Hold, target, nextSequence()};
	}

	return {};
}

void Hx8CommandPolicy::resetAfterFaultClear()
{
	_last_target = HybridTarget::None;
	_last_motion_sequence = 0;
	_release_attempts = 0;
	_last_hold_us = 0;
}

bool Hx8CommandPolicy::motionAcknowledged(uint32_t status_sequence, bool accepted, uint8_t result,
		uint8_t accepted_result) const
{
	return _last_motion_sequence != 0 && status_sequence == _last_motion_sequence && accepted
	       && result == accepted_result;
}

} // namespace hybrid_control

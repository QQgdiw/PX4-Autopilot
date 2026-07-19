#include "Hx8BackendPolicy.hpp"

#include <cmath>

#include "TransformationPosition.hpp"

namespace hybrid_control
{

float Hx8BackendPolicy::wrappedSpanDegrees(float quad_deg, float rover_deg)
{
	const float delta = (rover_deg - quad_deg) * static_cast<float>(M_PI / 180.0);
	return fabsf(std::atan2(std::sin(delta), std::cos(delta))) * static_cast<float>(180.0 / M_PI);
}

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

bool Hx8BackendPolicy::endpointMatchesAngleTolerance(float normalized, bool driving_target, float tolerance_rad,
		float quad_deg, float rover_deg)
{
	const float span = wrappedSpanDegrees(quad_deg, rover_deg) * static_cast<float>(M_PI / 180.0);
	return span > 1e-6f && endpointMatches(normalized, driving_target, tolerance_rad / span);
}

bool Hx8BackendPolicy::endpointAnyMatchesAngleTolerance(float normalized, float tolerance_rad,
		float quad_deg, float rover_deg)
{
	const float span = wrappedSpanDegrees(quad_deg, rover_deg) * static_cast<float>(M_PI / 180.0);
	if (!(span > 1e-6f) || !std::isfinite(tolerance_rad) || tolerance_rad < 0.f || tolerance_rad / span >= 0.5f) {
		return false;
	}
	const float tol = tolerance_rad / span;
	return std::isfinite(normalized) && (fabsf(normalized) <= tol || fabsf(normalized - 1.f) <= tol);
}

bool Hx8BackendPolicy::parametersValid(int32_t id, float quad_deg, float rover_deg, int32_t move, int32_t acc,
		int32_t dec, int32_t power, float transition_s)
{
	if (!(id >= 0 && id <= 254 && std::isfinite(quad_deg) && std::isfinite(rover_deg)
	       && quad_deg >= -180.f && quad_deg <= 180.f && rover_deg >= -180.f && rover_deg <= 180.f
	       && fabsf(quad_deg - rover_deg) > 1e-4f && move > 0 && acc >= 0 && dec >= 0
	       && move <= UINT16_MAX && acc <= UINT16_MAX && dec <= UINT16_MAX && power > 0 && power <= UINT16_MAX
	       && std::isfinite(transition_s) && transition_s > 0.f && static_cast<float>(move) < transition_s * 1000.f)) {
		return false;
	}
	return static_cast<uint32_t>(move) > static_cast<uint32_t>(acc) + static_cast<uint32_t>(dec);
}

bool Hx8BackendPolicy::commandAccepted(bool accepted, uint8_t result, uint8_t pending_result,
		uint8_t accepted_result)
{
	return (!accepted && result == pending_result) || (accepted && result == accepted_result);
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
		if (_release_attempts >= 3 || (_release_sent && now_us - _last_release_us < 20'000)) {
			return {};
		}

		++_release_attempts;
		_last_release_us = now_us;
		_release_sent = true;
		return {Hx8CommandAction::Release, HybridTarget::None, nextSequence()};
	}

	_release_attempts = 0;
	_release_sent = false;

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
	_release_sent = false;
	_last_hold_us = 0;
}

bool Hx8CommandPolicy::motionAcknowledged(uint32_t status_sequence, bool accepted, uint8_t result,
		uint8_t accepted_result) const
{
	return _last_motion_sequence != 0 && status_sequence == _last_motion_sequence && accepted
	       && result == accepted_result;
}

} // namespace hybrid_control

#include "HybridTransitionPolicy.hpp"

namespace hybrid_control
{

namespace
{
constexpr uint8_t NavigationStateManual = 0;
constexpr uint8_t NavigationStatePosctl = 2;
constexpr uint8_t NavigationStateAutoMission = 3;
constexpr uint8_t NavigationStateAutoLoiter = 4;
constexpr uint8_t NavigationStateAutoRtl = 5;
constexpr uint8_t NavigationStateAcro = 10;
constexpr uint8_t NavigationStateOffboard = 14;
constexpr uint8_t NavigationStateStab = 15;

TransitionDecision reject(CommandResult result, RejectReason reason)
{
	return {false, HybridTarget::None, result, reason};
}

HybridTarget transitionTarget(HybridState state)
{
	switch (state) {
	case HybridState::TransitionToRover:
		return HybridTarget::Driving;

	case HybridState::TransitionToQuad:
		return HybridTarget::Flying;

	default:
		return HybridTarget::None;
	}
}

} // namespace

TransitionDecision decideTransition(const TransitionRequest &request)
{
	if (request.requested != HybridTarget::Flying && request.requested != HybridTarget::Driving) {
		return reject(CommandResult::Denied, RejectReason::InvalidTarget);
	}

	if (request.faulted || request.state == HybridState::Fault) {
		return reject(CommandResult::Denied, RejectReason::TransformationFault);
	}

	if (!request.land_sample_fresh) {
		return reject(CommandResult::TemporarilyRejected, RejectReason::LandDetectorStale);
	}

	if (!request.landed) {
		return reject(CommandResult::TemporarilyRejected, RejectReason::NotLanded);
	}

	const HybridTarget stable_target = request.state == HybridState::Flying ? HybridTarget::Flying
				       : request.state == HybridState::Driving ? HybridTarget::Driving : HybridTarget::None;

	if (stable_target != HybridTarget::None) {
		if (request.requested == stable_target) {
			return {false, stable_target, CommandResult::Accepted, RejectReason::None};
		}

		return {true, request.requested, CommandResult::InProgress, RejectReason::None};
	}

	const HybridTarget current_target = request.active_target != HybridTarget::None ? request.active_target :
					     transitionTarget(request.state);

	if (current_target != HybridTarget::None) {
		if (request.requested == current_target) {
			return {false, current_target, CommandResult::InProgress, RejectReason::None};
		}

		return reject(CommandResult::TemporarilyRejected, RejectReason::OppositeTransition);
	}

	return reject(CommandResult::Denied, RejectReason::UnknownState);
}

bool modeAllowedForShape(HybridState state, uint8_t nav_state)
{
	if (state == HybridState::Flying) {
		return true;
	}

	if (state != HybridState::Driving) {
		return false;
	}

	switch (nav_state) {
	case NavigationStateManual:
	case NavigationStatePosctl:
	case NavigationStateAutoMission:
	case NavigationStateAutoLoiter:
	case NavigationStateAutoRtl:
	case NavigationStateAcro:
	case NavigationStateOffboard:
	case NavigationStateStab:
		return true;

	default:
		return false;
	}
}

bool offboardInputFreshAfter(uint64_t input_timestamp, uint64_t completion_timestamp, uint64_t now)
{
	return input_timestamp > completion_timestamp && input_timestamp <= now;
}

} // namespace hybrid_control

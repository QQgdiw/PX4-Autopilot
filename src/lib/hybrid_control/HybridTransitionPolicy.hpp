#pragma once

#include <cstdint>

#include "TransformationStateMachine.hpp"

namespace hybrid_control
{

enum class RejectReason : uint8_t {
	None,
	InvalidTarget,
	ReservedParameter,
	NotLanded,
	LandDetectorStale,
	OppositeTransition,
	TransformationFault,
	UnknownState
};

enum class CommandResult : uint8_t {
	Accepted = 0,
	TemporarilyRejected = 1,
	Denied = 2,
	Failed = 4,
	InProgress = 5
};

struct TransitionRequest {
	bool land_sample_fresh;
	bool landed;
	bool faulted;
	HybridState state;
	HybridTarget requested;
	HybridTarget active_target{HybridTarget::None};
};

struct TransitionDecision {
	bool start;
	HybridTarget target;
	CommandResult ack_result;
	RejectReason reject_reason;
};

TransitionDecision decideTransition(const TransitionRequest &request);
bool configurationUpdatePermitted(bool armed, bool prearmed, HybridState state);
bool modeAllowedForShape(HybridState state, uint8_t nav_state);
bool offboardInputFreshAfter(uint64_t input_timestamp, uint64_t completion_timestamp, uint64_t now);

} // namespace hybrid_control

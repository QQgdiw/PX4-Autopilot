/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <cstdint>

namespace mini_vehicle_control
{

enum class OutputIssueReason : uint8_t {
	None = 0,
	NoSource,
	BeforeModeEpoch,
	FutureTimestamp,
	StaleSource,
	NonFiniteControl,
};

constexpr OutputIssueReason classifySourceFreshness(uint64_t now, uint64_t source_timestamp,
		uint64_t mode_changed_at, uint64_t timeout)
{
	if (source_timestamp == 0) {
		return OutputIssueReason::NoSource;
	}

	if (source_timestamp <= mode_changed_at) {
		return OutputIssueReason::BeforeModeEpoch;
	}

	if (now < source_timestamp) {
		return OutputIssueReason::FutureTimestamp;
	}

	if (now - source_timestamp > timeout) {
		return OutputIssueReason::StaleSource;
	}

	return OutputIssueReason::None;
}

constexpr const char *outputIssueReasonName(OutputIssueReason reason)
{
	switch (reason) {
	case OutputIssueReason::None:
		return "none";

	case OutputIssueReason::NoSource:
		return "no-source";

	case OutputIssueReason::BeforeModeEpoch:
		return "pre-epoch";

	case OutputIssueReason::FutureTimestamp:
		return "future";

	case OutputIssueReason::StaleSource:
		return "stale";

	case OutputIssueReason::NonFiniteControl:
		return "non-finite";
	}

	return "unknown";
}

} // namespace mini_vehicle_control

/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <uORB/topics/hybrid_vehicle_status.h>
#include <uORB/topics/vehicle_command_ack.h>

#include <cmath>

static constexpr hrt_abstime kHybridTransitionMissionStatusTimeoutUs{1000000};

struct HybridTransitionMissionItemKey {
	uint32_t mission_id{0};
	int32_t sequence{-1};
};

class HybridTransitionMissionActivation
{
public:
	void activate(const HybridTransitionMissionItemKey &key)
	{
		if (!_key_valid || key.mission_id != _key.mission_id || key.sequence != _key.sequence) {
			_key = key;
			_key_valid = true;
			_issued = false;
			_sequence_snapshot = 0;
			_already_stable = false;
			_command_timestamp = 0;
		}
	}

	bool shouldIssue(const HybridTransitionMissionItemKey &key)
	{
		activate(key);
		return !_issued;
	}

	void recordIssued(uint32_t sequence_snapshot, bool already_stable, uint64_t command_timestamp)
	{
		_sequence_snapshot = sequence_snapshot;
		_already_stable = already_stable;
		_command_timestamp = command_timestamp;
		_issued = true;
	}

	void reset() { *this = {}; }
	uint32_t sequenceSnapshot() const { return _sequence_snapshot; }
	bool alreadyStable() const { return _already_stable; }
	uint64_t commandTimestamp() const { return _command_timestamp; }

private:
	HybridTransitionMissionItemKey _key{};
	bool _key_valid{false};
	bool _issued{false};
	uint32_t _sequence_snapshot{0};
	bool _already_stable{false};
	uint64_t _command_timestamp{0};
};

enum class HybridTransitionMissionOutcome {
	Unrelated,
	Waiting,
	AwaitStableState,
	Failed
};

inline uint8_t hybridTransitionMissionTarget(float value)
{
	if (fabsf(value - 1.f) <= 0.f) {
		return hybrid_vehicle_status_s::TARGET_FLYING;
	}

	if (fabsf(value - 2.f) <= 0.f) {
		return hybrid_vehicle_status_s::TARGET_DRIVING;
	}

	return hybrid_vehicle_status_s::TARGET_NONE;
}

inline HybridTransitionMissionOutcome hybridTransitionMissionOutcome(uint64_t issued_command_timestamp,
		const hybrid_vehicle_status_s &status)
{
	if (issued_command_timestamp == 0 || status.command_timestamp != issued_command_timestamp) {
		return HybridTransitionMissionOutcome::Unrelated;
	}

	switch (status.command_result) {
	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED:
		return HybridTransitionMissionOutcome::AwaitStableState;

	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED:
	case vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED:
		return HybridTransitionMissionOutcome::Failed;

	default:
		return HybridTransitionMissionOutcome::Waiting;
	}
}

inline bool hybridTransitionMissionStatusFresh(const hybrid_vehicle_status_s &status, hrt_abstime now)
{
	return status.timestamp != 0 && now >= status.timestamp
	       && now - status.timestamp <= kHybridTransitionMissionStatusTimeoutUs;
}

inline bool hybridTransitionMissionReached(uint32_t minimum_sequence, uint8_t target,
		const hybrid_vehicle_status_s &status)
{
	const uint8_t stable_state = target == hybrid_vehicle_status_s::TARGET_FLYING
				     ? hybrid_vehicle_status_s::HYBRID_STATE_FLYING
				     : hybrid_vehicle_status_s::HYBRID_STATE_DRIVING;

	return (target == hybrid_vehicle_status_s::TARGET_FLYING
		|| target == hybrid_vehicle_status_s::TARGET_DRIVING)
	       && status.transition_sequence >= minimum_sequence
	       && status.target_state == target
	       && status.current_state == stable_state;
}

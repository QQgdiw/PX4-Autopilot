/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
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

#include "HybridTransitionMission.hpp"

#include <drivers/drv_hrt.h>
#include <gtest/gtest.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <cfloat>

static hybrid_vehicle_status_s makeHybridStatus(uint32_t sequence, uint8_t state, uint8_t target)
{
	hybrid_vehicle_status_s status{};
	status.transition_sequence = sequence;
	status.current_state = state;
	status.target_state = target;
	return status;
}

TEST(HybridTransitionMission, CompletesOnlyMatchingStableSequence)
{
	EXPECT_FALSE(hybridTransitionMissionReached(7, hybrid_vehicle_status_s::TARGET_DRIVING,
			makeHybridStatus(6, hybrid_vehicle_status_s::HYBRID_STATE_DRIVING,
					 hybrid_vehicle_status_s::TARGET_DRIVING)));
	EXPECT_FALSE(hybridTransitionMissionReached(7, hybrid_vehicle_status_s::TARGET_DRIVING,
			makeHybridStatus(7, hybrid_vehicle_status_s::HYBRID_STATE_TRANSITIONING,
					 hybrid_vehicle_status_s::TARGET_DRIVING)));
	EXPECT_TRUE(hybridTransitionMissionReached(7, hybrid_vehicle_status_s::TARGET_DRIVING,
			makeHybridStatus(7, hybrid_vehicle_status_s::HYBRID_STATE_DRIVING,
					 hybrid_vehicle_status_s::TARGET_DRIVING)));
}

TEST(HybridTransitionMission, RejectsOppositeStableTarget)
{
	EXPECT_FALSE(hybridTransitionMissionReached(7, hybrid_vehicle_status_s::TARGET_DRIVING,
			makeHybridStatus(8, hybrid_vehicle_status_s::HYBRID_STATE_FLYING,
					 hybrid_vehicle_status_s::TARGET_FLYING)));
}

TEST(HybridTransitionMission, FaultNeverCompletesItem)
{
	EXPECT_FALSE(hybridTransitionMissionReached(7, hybrid_vehicle_status_s::TARGET_FLYING,
			makeHybridStatus(7, hybrid_vehicle_status_s::HYBRID_STATE_TRANSITION_FAULT,
					 hybrid_vehicle_status_s::TARGET_FLYING)));
}

TEST(HybridTransitionMission, RequiresFreshStatusTimestamp)
{
	hybrid_vehicle_status_s status = makeHybridStatus(7, hybrid_vehicle_status_s::HYBRID_STATE_DRIVING,
				 hybrid_vehicle_status_s::TARGET_DRIVING);
	constexpr hrt_abstime now = 2000000;

	status.timestamp = 0;
	EXPECT_FALSE(hybridTransitionMissionStatusFresh(status, now));

	status.timestamp = now + 1;
	EXPECT_FALSE(hybridTransitionMissionStatusFresh(status, now));

	status.timestamp = now - 1000000 - 1;
	EXPECT_FALSE(hybridTransitionMissionStatusFresh(status, now));

	status.timestamp = now - 1000000;
	EXPECT_TRUE(hybridTransitionMissionStatusFresh(status, now));
}

TEST(HybridTransitionMission, SameItemReentryPreservesActivation)
{
	HybridTransitionMissionActivation activation;
	const HybridTransitionMissionItemKey key{42, 7};

	EXPECT_TRUE(activation.shouldIssue(key));
	activation.recordIssued(9, true, 123456);
	EXPECT_FALSE(activation.shouldIssue(key));
	EXPECT_EQ(activation.sequenceSnapshot(), 9u);
	EXPECT_TRUE(activation.alreadyStable());
	EXPECT_EQ(activation.commandTimestamp(), 123456u);
}

TEST(HybridTransitionMission, NewItemResetsActivation)
{
	HybridTransitionMissionActivation activation;
	activation.shouldIssue({42, 7});
	activation.recordIssued(9, true, 123456);

	EXPECT_TRUE(activation.shouldIssue({42, 8}));
	EXPECT_EQ(activation.sequenceSnapshot(), 0u);
	EXPECT_FALSE(activation.alreadyStable());
	EXPECT_EQ(activation.commandTimestamp(), 0u);

	activation.recordIssued(10, false, 234567);
	EXPECT_TRUE(activation.shouldIssue({42, 7}));
}

TEST(HybridTransitionMission, ValidatesExactTargetWithoutUnsafeCast)
{
	EXPECT_EQ(int(hybridTransitionMissionTarget(1.f)), int(hybrid_vehicle_status_s::TARGET_FLYING));
	EXPECT_EQ(int(hybridTransitionMissionTarget(2.f)), int(hybrid_vehicle_status_s::TARGET_DRIVING));
	EXPECT_EQ(int(hybridTransitionMissionTarget(-1.f)), int(hybrid_vehicle_status_s::TARGET_NONE));
	EXPECT_EQ(int(hybridTransitionMissionTarget(1.5f)), int(hybrid_vehicle_status_s::TARGET_NONE));
	EXPECT_EQ(int(hybridTransitionMissionTarget(FLT_MAX)), int(hybrid_vehicle_status_s::TARGET_NONE));
}

TEST(HybridTransitionMission, CorrelatesOnlyIssuedCommandOutcome)
{
	hybrid_vehicle_status_s status{};
	status.command_timestamp = 999;
	status.command_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
	EXPECT_EQ(hybridTransitionMissionOutcome(123, status), HybridTransitionMissionOutcome::Unrelated);

	status.command_timestamp = 123;
	status.command_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS;
	EXPECT_EQ(hybridTransitionMissionOutcome(123, status), HybridTransitionMissionOutcome::Waiting);

	status.command_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	EXPECT_EQ(hybridTransitionMissionOutcome(123, status), HybridTransitionMissionOutcome::AwaitStableState);

	for (const uint8_t result : {vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED,
		     vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED,
		     vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED}) {
		status.command_result = result;
		EXPECT_EQ(hybridTransitionMissionOutcome(123, status), HybridTransitionMissionOutcome::Failed);
	}
}

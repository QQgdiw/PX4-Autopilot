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

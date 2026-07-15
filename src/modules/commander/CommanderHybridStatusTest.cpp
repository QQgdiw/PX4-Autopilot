/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "HybridStatusGuard.hpp"

using namespace time_literals;

TEST(CommanderHybridStatus, FaultOverridesStableState)
{
	hybrid_vehicle_status_s status{};
	status.timestamp = 1_s;
	status.current_state = hybrid_vehicle_status_s::HYBRID_STATE_FLYING;
	status.fault_reason = hybrid_vehicle_status_s::TRANSFORM_FAULT_SENSOR_TIMEOUT;

	EXPECT_EQ(commander::hybridStateForCommander(status, 1_s), hybrid_vehicle_status_s::HYBRID_STATE_TRANSITION_FAULT);
	EXPECT_FALSE(commander::hybridStateEnablesControl(hybrid_vehicle_status_s::HYBRID_STATE_TRANSITION_FAULT));
}

TEST(CommanderHybridStatus, TransitionAndStaleStatusDisableControl)
{
	hybrid_vehicle_status_s transition{};
	transition.timestamp = 1_s;
	transition.current_state = hybrid_vehicle_status_s::HYBRID_STATE_TRANSITIONING;
	EXPECT_FALSE(commander::hybridStateEnablesControl(commander::hybridStateForCommander(transition, 1_s)));

	hybrid_vehicle_status_s stale{};
	stale.current_state = hybrid_vehicle_status_s::HYBRID_STATE_DRIVING;
	EXPECT_EQ(commander::hybridStateForCommander(stale, 1_s), hybrid_vehicle_status_s::HYBRID_STATE_UNKNOWN);
	EXPECT_FALSE(commander::hybridStateEnablesControl(hybrid_vehicle_status_s::HYBRID_STATE_UNKNOWN));
}

TEST(CommanderHybridStatus, FreshUpdateTriggersImmediateProcessing)
{
	EXPECT_TRUE(commander::shouldProcessHybridStatus(true, true));
	EXPECT_FALSE(commander::shouldProcessHybridStatus(true, false));
	EXPECT_FALSE(commander::shouldProcessHybridStatus(false, true));
}

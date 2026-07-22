/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "commander_helper.h"
#include "HybridStatusGuard.hpp"

using namespace time_literals;

TEST(CommanderHybridStatus, IndependentIdentityIsNotVtol)
{
	vehicle_status_s status{};
	status.system_type = commander::VehicleTypeQuadRover;
	EXPECT_TRUE(commander::is_quad_rover(status));
	EXPECT_FALSE(commander::is_vtol(status));
}

TEST(CommanderHybridStatus, RoverRejectsAltitudeAndTransitionRejectsAllModes)
{
	EXPECT_FALSE(commander::hybridModeAllowed(hybrid_vehicle_status_s::HYBRID_STATE_DRIVING,
			vehicle_status_s::NAVIGATION_STATE_ALTCTL));
	EXPECT_TRUE(commander::hybridModeAllowed(hybrid_vehicle_status_s::HYBRID_STATE_DRIVING,
			vehicle_status_s::NAVIGATION_STATE_AUTO_RTL));
	EXPECT_FALSE(commander::hybridModeAllowed(hybrid_vehicle_status_s::HYBRID_STATE_TRANSITIONING,
			vehicle_status_s::NAVIGATION_STATE_MANUAL));
}

TEST(CommanderHybridStatus, UnstableShapeHasNoPhysicalVehicleType)
{
	EXPECT_EQ(int(commander::hybridVehicleType(hybrid_vehicle_status_s::HYBRID_STATE_FLYING)),
		  int(vehicle_status_s::VEHICLE_TYPE_ROTARY_WING));
	EXPECT_EQ(int(commander::hybridVehicleType(hybrid_vehicle_status_s::HYBRID_STATE_DRIVING)),
		  int(vehicle_status_s::VEHICLE_TYPE_ROVER));
	EXPECT_EQ(int(commander::hybridVehicleType(hybrid_vehicle_status_s::HYBRID_STATE_TRANSITIONING)), 0);
	EXPECT_EQ(int(commander::hybridVehicleType(hybrid_vehicle_status_s::HYBRID_STATE_TRANSITION_FAULT)), 0);
	EXPECT_EQ(int(commander::hybridVehicleType(hybrid_vehicle_status_s::HYBRID_STATE_UNKNOWN)), 0);
}

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

TEST(CommanderHybridStatus, SelectsRedPatternFromFaultAndOverload)
{
	hybrid_vehicle_status_s status{};
	status.current_state = hybrid_vehicle_status_s::HYBRID_STATE_FLYING;

	EXPECT_EQ(commander::hybridRedPattern(status, true, false), commander::HybridRedPattern::Off);
	EXPECT_EQ(commander::hybridRedPattern(status, true, true), commander::HybridRedPattern::OverloadFast);

	status.fault_reason = hybrid_vehicle_status_s::TRANSFORM_FAULT_SENSOR_TIMEOUT;
	EXPECT_EQ(commander::hybridRedPattern(status, true, false), commander::HybridRedPattern::FaultSlow);
	EXPECT_EQ(commander::hybridRedPattern(status, true, true), commander::HybridRedPattern::CombinedTriple);

	status.fault_reason = hybrid_vehicle_status_s::TRANSFORM_FAULT_STALL;
	EXPECT_EQ(commander::hybridRedPattern(status, true, false), commander::HybridRedPattern::StallDouble);
	EXPECT_EQ(commander::hybridRedPattern(status, true, true), commander::HybridRedPattern::CombinedTriple);

	status.fault_reason = hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE;
	EXPECT_EQ(commander::hybridRedPattern(status, false, false), commander::HybridRedPattern::FaultSlow);
	EXPECT_EQ(commander::hybridRedPattern(status, false, true), commander::HybridRedPattern::CombinedTriple);

	status.current_state = hybrid_vehicle_status_s::HYBRID_STATE_TRANSITION_FAULT;
	EXPECT_EQ(commander::hybridRedPattern(status, true, false), commander::HybridRedPattern::FaultSlow);
}

TEST(CommanderHybridStatus, RedWaveformBoundaries)
{
	using commander::HybridRedPattern;
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::Off, 0));

	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::OverloadFast, 0));
	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::OverloadFast, 50_ms - 1));
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::OverloadFast, 50_ms));
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::OverloadFast, 100_ms - 1));
	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::OverloadFast, 100_ms));

	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::FaultSlow, 500_ms - 1));
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::FaultSlow, 500_ms));
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::FaultSlow, 1_s - 1));
	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::FaultSlow, 1_s));

	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::StallDouble, 150_ms - 1));
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::StallDouble, 150_ms));
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::StallDouble, 300_ms - 1));
	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::StallDouble, 300_ms));
	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::StallDouble, 450_ms - 1));
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::StallDouble, 450_ms));
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::StallDouble, 1450_ms - 1));
	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::StallDouble, 1450_ms));

	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::CombinedTriple, 150_ms - 1));
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::CombinedTriple, 150_ms));
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::CombinedTriple, 300_ms - 1));
	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::CombinedTriple, 300_ms));
	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::CombinedTriple, 450_ms - 1));
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::CombinedTriple, 450_ms));
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::CombinedTriple, 600_ms - 1));
	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::CombinedTriple, 600_ms));
	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::CombinedTriple, 750_ms - 1));
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::CombinedTriple, 750_ms));
	EXPECT_FALSE(commander::hybridRedLedOn(HybridRedPattern::CombinedTriple, 1750_ms - 1));
	EXPECT_TRUE(commander::hybridRedLedOn(HybridRedPattern::CombinedTriple, 1750_ms));
}

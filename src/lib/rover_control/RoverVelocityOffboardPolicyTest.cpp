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
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "RoverVelocityOffboardPolicy.hpp"

#include <cmath>

TEST(RoverVelocityOffboardPolicy, RequiresFreshPostTransitionInput)
{
	EXPECT_FALSE(roverVelocityInputUsable({99, 0.3f, 0.2f}, 100, 150, 100, true));
	EXPECT_FALSE(roverVelocityInputUsable({100, 0.3f, 0.2f}, 100, 150, 100, true));
	EXPECT_FALSE(roverVelocityInputUsable({120, NAN, 0.2f}, 100, 150, 100, true));
	EXPECT_FALSE(roverVelocityInputUsable({120, -0.3f, 0.2f}, 100, 150, 100, false));
	EXPECT_TRUE(roverVelocityInputUsable({120, -0.3f, 0.2f}, 100, 150, 100, true));
}

TEST(RoverVelocityOffboardPolicy, AcceptsExactTimeoutAndSignedCommands)
{
	EXPECT_TRUE(roverVelocityInputUsable({50, -0.3f, -0.2f}, 49, 150, 100, true));
	EXPECT_TRUE(roverVelocityInputUsable({50, 0.3f, 0.2f}, 49, 150, 100, true));
	EXPECT_FALSE(roverVelocityInputUsable({50, 0.3f, 0.2f}, 49, 151, 100, true));
}

TEST(RoverVelocityOffboardPolicy, RejectsInvalidTimestampsAndFields)
{
	EXPECT_FALSE(roverVelocityInputUsable({0, 0.3f, 0.2f}, 0, 100, 100, true));
	EXPECT_FALSE(roverVelocityInputUsable({101, 0.3f, 0.2f}, 0, 100, 100, true));
	EXPECT_FALSE(roverVelocityInputUsable({100, INFINITY, 0.2f}, 0, 100, 100, true));
	EXPECT_FALSE(roverVelocityInputUsable({100, 0.3f, -INFINITY}, 0, 100, 100, true));
	EXPECT_FALSE(roverVelocityInputUsable({100, 0.3f, NAN}, 0, 100, 100, true));
}

TEST(RoverVelocityOffboardPolicy, RequiresExactDedicatedSelection)
{
	RoverVelocityOffboardMode mode{100, false, false, false, false, false, false, false, true};
	EXPECT_TRUE(roverVelocityModeUsable(mode, 150, 50));

	mode.velocity = true;
	EXPECT_FALSE(roverVelocityModeUsable(mode, 150, 50));
	mode.velocity = false;
	mode.position = true;
	EXPECT_FALSE(roverVelocityModeUsable(mode, 150, 50));
	mode.position = false;
	mode.acceleration = true;
	EXPECT_FALSE(roverVelocityModeUsable(mode, 150, 50));
	mode.acceleration = false;
	mode.attitude = true;
	EXPECT_FALSE(roverVelocityModeUsable(mode, 150, 50));
	mode.attitude = false;
	mode.body_rate = true;
	EXPECT_FALSE(roverVelocityModeUsable(mode, 150, 50));
	mode.body_rate = false;
	mode.thrust_and_torque = true;
	EXPECT_FALSE(roverVelocityModeUsable(mode, 150, 50));
	mode.thrust_and_torque = false;
	mode.direct_actuator = true;
	EXPECT_FALSE(roverVelocityModeUsable(mode, 150, 50));
	mode.direct_actuator = false;
	mode.rover_velocity = false;
	EXPECT_FALSE(roverVelocityModeUsable(mode, 150, 50));

	mode = {100, false, false, false, false, false, false, false, true};
	EXPECT_TRUE(roverVelocityModeUsable(mode, 150, 50));
	EXPECT_FALSE(roverVelocityModeUsable(mode, 151, 50));
	mode.timestamp = 152;
	EXPECT_FALSE(roverVelocityModeUsable(mode, 151, 50));
}

TEST(RoverVelocityOffboardPolicy, RequiresFreshHealthyDrivingStatus)
{
	EXPECT_TRUE(roverDrivingStatusUsable({100, 100, true, true}, 1100, 1000));
	EXPECT_FALSE(roverDrivingStatusUsable({100, 100, true, true}, 1101, 1000));
	EXPECT_FALSE(roverDrivingStatusUsable({1102, 100, true, true}, 1101, 1000));
	EXPECT_FALSE(roverDrivingStatusUsable({100, 100, false, true}, 1100, 1000));
	EXPECT_FALSE(roverDrivingStatusUsable({100, 100, true, false}, 1100, 1000));
	EXPECT_FALSE(roverDrivingStatusUsable({0, 100, true, true}, 1100, 1000));
}

TEST(RoverVelocityOffboardPolicy, QuadRoverRejectsLegacyOffboardSelections)
{
	RoverVelocityOffboardMode mode{100, false, true, false, false, false, false, false, false};
	const RoverVelocityDrivingStatus healthy{100, 50, true, true};
	const RoverVelocityDrivingStatus stale{1, 50, true, true};
	const RoverVelocityDrivingStatus fault{100, 50, true, false};

	EXPECT_FALSE(roverOffboardModeAvailable(true, mode, healthy, 150, 100, 100));
	mode.velocity = false;
	mode.body_rate = true;
	EXPECT_FALSE(roverOffboardModeAvailable(true, mode, healthy, 150, 100, 100));
	mode.body_rate = false;
	mode.rover_velocity = true;
	EXPECT_TRUE(roverOffboardModeAvailable(true, mode, healthy, 150, 100, 100));
	EXPECT_FALSE(roverOffboardModeAvailable(true, mode, stale, 150, 100, 100));
	EXPECT_FALSE(roverOffboardModeAvailable(true, mode, fault, 150, 100, 100));
}

TEST(RoverVelocityOffboardPolicy, LegacyRoverKeepsGenericOffboardSelections)
{
	const RoverVelocityDrivingStatus no_hybrid_status{};
	RoverVelocityOffboardMode mode{100, false, true, false, false, false, false, false, false};
	EXPECT_TRUE(roverOffboardModeAvailable(false, mode, no_hybrid_status, 150, 100, 100));

	mode.velocity = false;
	mode.body_rate = true;
	EXPECT_TRUE(roverOffboardModeAvailable(false, mode, no_hybrid_status, 150, 100, 100));
}

TEST(RoverVelocityOffboardPolicy, QuadRoverControllersOwnEveryOffboardSelection)
{
	EXPECT_TRUE(roverVelocityDedicatedControlRequired(true, true, false, false));
	EXPECT_TRUE(roverVelocityDedicatedControlRequired(true, true, true, true));
	EXPECT_FALSE(roverVelocityDedicatedControlRequired(true, false, true, true));
	EXPECT_FALSE(roverVelocityDedicatedControlRequired(false, true, false, true));
	EXPECT_TRUE(roverVelocityDedicatedControlRequired(false, true, true, true));
	EXPECT_FALSE(roverVelocityDedicatedControlRequired(false, true, true, false));
}

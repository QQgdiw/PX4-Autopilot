/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "commander_helper.h"
#include "HybridStatusGuard.hpp"
#include "HybridOffboardGuard.hpp"
#include "ModeUtil/control_mode.hpp"
#include <lib/rover_control/RoverVelocityOffboardPolicy.hpp>

using namespace time_literals;

TEST(CommanderHybridStatus, QuadOffboardPositionReachesMulticopterControl)
{
	hybrid_vehicle_status_s status{};
	status.timestamp = 100;
	status.current_state = hybrid_vehicle_status_s::HYBRID_STATE_FLYING;
	status.propulsion_ready = true;
	offboard_control_mode_s mode{};
	mode.timestamp = 100;
	mode.position = true;
	EXPECT_TRUE(commander::hybridOffboardModeAvailable(mode, status,
			vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, 150, 100));
	vehicle_control_mode_s control{};
	mode_util::getVehicleControlMode(vehicle_status_s::NAVIGATION_STATE_OFFBOARD,
					 vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, mode, control);
	EXPECT_TRUE(control.flag_control_position_enabled);
	EXPECT_TRUE(control.flag_control_velocity_enabled);
	EXPECT_TRUE(control.flag_control_attitude_enabled);
	EXPECT_TRUE(control.flag_control_rates_enabled);
	mode.rover_velocity = true;
	EXPECT_FALSE(commander::hybridOffboardModeAvailable(mode, status,
			vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, 150, 100));
}

TEST(CommanderHybridStatus, RoverOffboardRequiresDedicatedPostTransitionMode)
{
	hybrid_vehicle_status_s status{};
	status.timestamp = 100;
	status.current_state = hybrid_vehicle_status_s::HYBRID_STATE_DRIVING;
	status.propulsion_ready = true;
	status.transition_completed_timestamp = 90;
	offboard_control_mode_s mode{};
	mode.timestamp = 100;
	mode.rover_velocity = true;
	EXPECT_TRUE(commander::hybridOffboardModeAvailable(mode, status,
			vehicle_status_s::VEHICLE_TYPE_ROVER, 150, 100));
	vehicle_control_mode_s control{};
	mode_util::getVehicleControlMode(vehicle_status_s::NAVIGATION_STATE_OFFBOARD,
					 vehicle_status_s::VEHICLE_TYPE_ROVER, mode, control);
	EXPECT_TRUE(control.flag_control_velocity_enabled);
	EXPECT_TRUE(control.flag_control_rates_enabled);
	EXPECT_FALSE(control.flag_control_position_enabled);
	EXPECT_FALSE(control.flag_control_attitude_enabled);
	mode.position = true;
	EXPECT_FALSE(commander::hybridOffboardModeAvailable(mode, status,
			vehicle_status_s::VEHICLE_TYPE_ROVER, 150, 100));
	mode.rover_velocity = false;
	EXPECT_FALSE(commander::hybridOffboardModeAvailable(mode, status,
			vehicle_status_s::VEHICLE_TYPE_ROVER, 150, 100));
	mode.position = false;
	mode.rover_velocity = true;
	mode.timestamp = 90;
	EXPECT_FALSE(commander::hybridOffboardModeAvailable(mode, status,
			vehicle_status_s::VEHICLE_TYPE_ROVER, 150, 100));
}

TEST(CommanderHybridStatus, HybridOffboardRejectsUnsafeStateAndOwnership)
{
	hybrid_vehicle_status_s status{};
	status.timestamp = 100;
	status.current_state = hybrid_vehicle_status_s::HYBRID_STATE_FLYING;
	status.propulsion_ready = true;
	status.actuator_backend = hybrid_vehicle_status_s::ACTUATOR_HX65;
	status.propulsion_owner = hybrid_vehicle_status_s::PROPULSION_QUAD;
	offboard_control_mode_s mode{};
	mode.timestamp = 100;
	mode.position = true;
	const auto available = [&]() {
		return commander::hybridOffboardModeAvailable(mode, status,
				vehicle_status_s::VEHICLE_TYPE_ROTARY_WING, 150, 100);
	};
	EXPECT_TRUE(available());
	status.propulsion_owner = hybrid_vehicle_status_s::PROPULSION_ROVER;
	EXPECT_FALSE(available());
	status.propulsion_owner = hybrid_vehicle_status_s::PROPULSION_QUAD;
	status.propulsion_ready = false;
	EXPECT_FALSE(available());
	status.propulsion_ready = true;
	status.fault_reason = hybrid_vehicle_status_s::TRANSFORM_FAULT_ACTUATOR_COMMUNICATION;
	EXPECT_FALSE(available());
	status.fault_reason = hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE;
	status.timestamp = 0;
	EXPECT_FALSE(available());
	status.timestamp = 151;
	EXPECT_FALSE(available());
	status.timestamp = 100;
	mode.timestamp = 49;
	EXPECT_FALSE(available());
	mode.timestamp = 100;

	for (const auto state : {
		     hybrid_vehicle_status_s::HYBRID_STATE_TRANSITIONING,
		     hybrid_vehicle_status_s::HYBRID_STATE_TRANSITION_FAULT, hybrid_vehicle_status_s::HYBRID_STATE_UNKNOWN
	     }) {
		status.current_state = state;
		EXPECT_FALSE(available());
	}
}

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

	EXPECT_EQ(commander::hybridStateForCommander(status, 1_s),
		  static_cast<uint8_t>(hybrid_vehicle_status_s::HYBRID_STATE_TRANSITION_FAULT));
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
	EXPECT_EQ(commander::hybridStateForCommander(stale, 1_s),
		  static_cast<uint8_t>(hybrid_vehicle_status_s::HYBRID_STATE_UNKNOWN));
	EXPECT_FALSE(commander::hybridStateEnablesControl(hybrid_vehicle_status_s::HYBRID_STATE_UNKNOWN));
}

TEST(CommanderHybridStatus, SequenceFaultPreservesAnExistingSafePropulsionOwner)
{
	hybrid_vehicle_status_s airborne{};
	airborne.timestamp = 1_s;
	airborne.current_state = hybrid_vehicle_status_s::HYBRID_STATE_FLYING;
	airborne.sequence_fault = hybrid_vehicle_status_s::SEQUENCE_FAULT_GEAR_COMMUNICATION;
	airborne.propulsion_owner = hybrid_vehicle_status_s::PROPULSION_QUAD;
	EXPECT_EQ(commander::hybridStateForCommander(airborne, 1_s),
		  static_cast<uint8_t>(hybrid_vehicle_status_s::HYBRID_STATE_FLYING));

	airborne.propulsion_owner = hybrid_vehicle_status_s::PROPULSION_NONE;
	EXPECT_EQ(commander::hybridStateForCommander(airborne, 1_s),
		  static_cast<uint8_t>(hybrid_vehicle_status_s::HYBRID_STATE_TRANSITION_FAULT));
}

TEST(CommanderHybridStatus, RoverOffboardAvailabilityRequiresDedicatedModeAndHealthyShape)
{
	RoverVelocityOffboardMode mode{1_s, false, true, false, false, false, false, false, false};
	RoverVelocityDrivingStatus status{1_s, 500_ms, true, true};
	EXPECT_FALSE(roverOffboardModeAvailable(true, mode, status, 1_s, 1_s, 1_s));

	mode.velocity = false;
	mode.body_rate = true;
	EXPECT_FALSE(roverOffboardModeAvailable(true, mode, status, 1_s, 1_s, 1_s));

	mode.body_rate = false;
	mode.rover_velocity = true;
	EXPECT_TRUE(roverOffboardModeAvailable(true, mode, status, 1_s, 1_s, 1_s));

	status.fault_free = false;
	EXPECT_FALSE(roverOffboardModeAvailable(true, mode, status, 1_s, 1_s, 1_s));
	status = {1, 500_ms, true, true};
	EXPECT_FALSE(roverOffboardModeAvailable(true, mode, status, 2_s, 1_s, 1_s));
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

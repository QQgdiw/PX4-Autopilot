/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 ****************************************************************************/

#include <gtest/gtest.h>
#include "offboardCheck.hpp"
#include <uORB/Publication.hpp>

TEST(OffboardCheck, HybridQuadUsesPositionEstimatorAndRoverUsesDedicatedMode)
{
	uORB::Publication<hybrid_vehicle_status_s> hybrid_pub{ORB_ID(hybrid_vehicle_status)};
	uORB::Publication<offboard_control_mode_s> mode_pub{ORB_ID(offboard_control_mode)};
	hybrid_vehicle_status_s hybrid{};
	hybrid.current_state = hybrid_vehicle_status_s::HYBRID_STATE_FLYING;
	hybrid.propulsion_ready = true;
	offboard_control_mode_s mode{};
	mode.position = true;
	vehicle_status_s vehicle{};
	vehicle.is_quad_rover = true;
	vehicle.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	failsafe_flags_s flags{};
	Report reporter{flags};
	Context context{vehicle};
	OffboardChecks checks;
	const auto available = [&]() {
		hybrid.timestamp = hrt_absolute_time();
		mode.timestamp = hybrid.timestamp;
		hybrid_pub.publish(hybrid);
		mode_pub.publish(mode);
		checks.checkAndReport(context, reporter);
		return !flags.offboard_control_signal_lost;
	};

	EXPECT_TRUE(available());
	flags.local_position_invalid = true;
	EXPECT_FALSE(available());
	flags.local_position_invalid = false;
	mode.position = false;
	mode.velocity = true;
	EXPECT_TRUE(available());
	flags.local_velocity_invalid = true;
	EXPECT_FALSE(available());
	flags.local_velocity_invalid = false;
	mode.rover_velocity = true;
	EXPECT_FALSE(available());
	mode.velocity = false;
	EXPECT_FALSE(available());
	hybrid.current_state = hybrid_vehicle_status_s::HYBRID_STATE_DRIVING;
	vehicle.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROVER;
	EXPECT_TRUE(available());
	flags.angular_velocity_invalid = true;
	EXPECT_FALSE(available());
	flags.angular_velocity_invalid = false;
	flags.local_velocity_invalid = true;
	EXPECT_FALSE(available());
	flags.local_velocity_invalid = false;
	mode.rover_velocity = false;
	mode.position = true;
	EXPECT_FALSE(available());

	// Ordinary aircraft and Rover retain the standard position-mode check.
	vehicle.is_quad_rover = false;

	for (const auto type : {
		     vehicle_status_s::VEHICLE_TYPE_ROTARY_WING,
		     vehicle_status_s::VEHICLE_TYPE_ROVER, vehicle_status_s::VEHICLE_TYPE_FIXED_WING
	     }) {
		vehicle.vehicle_type = type;
		EXPECT_TRUE(available());
		flags.local_position_invalid = true;
		EXPECT_FALSE(available());
		flags.local_position_invalid = false;
	}
}

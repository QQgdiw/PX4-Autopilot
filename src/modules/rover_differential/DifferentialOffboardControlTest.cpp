/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "DifferentialRateControl/DifferentialRateControl.hpp"
#include "DifferentialVelControl/DifferentialVelControl.hpp"

#include <px4_platform_common/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/hybrid_vehicle_status.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/rover_steering_setpoint.h>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/topics/rover_velocity_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class DifferentialOffboardControlTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
		setFloat("COM_OF_LOSS_T", 1.f);
		setFloat("RO_SPEED_LIM", 1.f);
		setFloat("RO_MAX_THR_SPEED", 1.f);
		setFloat("RO_ACCEL_LIM", 1.f);
		setFloat("RO_DECEL_LIM", 1.f);
		setFloat("RO_SPEED_P", 1.f);
		setFloat("RO_YAW_RATE_LIM", 90.f);
		setFloat("RO_YAW_ACCEL_LIM", 90.f);
		setFloat("RO_YAW_DECEL_LIM", 90.f);
		setFloat("RD_WHEEL_TRACK", 0.5f);
		setFloat("RD_MAX_THR_YAW_R", 0.5f);
	}

	void setFloat(const char *name, float value)
	{
		const param_t parameter = param_find(name);
		ASSERT_NE(parameter, PARAM_INVALID);
		ASSERT_EQ(param_set(parameter, &value), PX4_OK);
	}

	void publishVehicleInputs(bool is_quad_rover, const offboard_control_mode_s &offboard,
				  const vehicle_control_mode_s &control, const hybrid_vehicle_status_s &hybrid,
				  const rover_velocity_setpoint_s &rover_setpoint, const trajectory_setpoint_s &trajectory = {})
	{
		const hrt_abstime sensor_timestamp = hrt_absolute_time();
		vehicle_status_s vehicle_status{};
		vehicle_status.timestamp = sensor_timestamp;
		vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROVER;
		vehicle_status.is_quad_rover = is_quad_rover;
		_vehicle_status_pub.publish(vehicle_status);
		_offboard_pub.publish(offboard);
		_control_mode_pub.publish(control);
		_hybrid_status_pub.publish(hybrid);
		_rover_setpoint_pub.publish(rover_setpoint);
		_trajectory_pub.publish(trajectory);

		vehicle_attitude_s attitude{};
		attitude.timestamp = sensor_timestamp;
		attitude.timestamp_sample = sensor_timestamp;
		attitude.q[0] = 1.f;
		_attitude_pub.publish(attitude);
		vehicle_local_position_s local_position{};
		local_position.timestamp = sensor_timestamp;
		local_position.timestamp_sample = sensor_timestamp;
		local_position.v_xy_valid = true;
		_local_position_pub.publish(local_position);
		vehicle_angular_velocity_s angular_velocity{};
		angular_velocity.timestamp = sensor_timestamp;
		angular_velocity.timestamp_sample = sensor_timestamp;
		_angular_velocity_pub.publish(angular_velocity);
	}

	uORB::Publication<vehicle_status_s> _vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<offboard_control_mode_s> _offboard_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_control_mode_s> _control_mode_pub{ORB_ID(vehicle_control_mode)};
	uORB::Publication<hybrid_vehicle_status_s> _hybrid_status_pub{ORB_ID(hybrid_vehicle_status)};
	uORB::Publication<rover_velocity_setpoint_s> _rover_setpoint_pub{ORB_ID(rover_velocity_setpoint)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_local_position_s> _local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_angular_velocity_s> _angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};
};

TEST_F(DifferentialOffboardControlTest, QuadRoverLegacyBodyRatePublishesZeroThrottleAndSteering)
{
	const hrt_abstime now = hrt_absolute_time();
	offboard_control_mode_s offboard{};
	offboard.timestamp = now;
	offboard.body_rate = true;
	vehicle_control_mode_s control{};
	control.flag_armed = true;
	control.flag_control_offboard_enabled = true;
	control.flag_control_rates_enabled = true;
	control.flag_control_allocation_enabled = true;
	hybrid_vehicle_status_s hybrid{};
	hybrid.timestamp = now;
	hybrid.current_state = hybrid_vehicle_status_s::HYBRID_STATE_DRIVING;
	rover_velocity_setpoint_s rover_setpoint{};
	rover_setpoint.timestamp = now;

	publishVehicleInputs(true, offboard, control, hybrid, rover_setpoint);
	DifferentialVelControl velocity_control{nullptr};
	DifferentialRateControl rate_control{nullptr};
	uORB::SubscriptionData<rover_throttle_setpoint_s> throttle{ORB_ID(rover_throttle_setpoint)};
	uORB::SubscriptionData<rover_steering_setpoint_s> steering{ORB_ID(rover_steering_setpoint)};

	velocity_control.updateVelControl();
	rate_control.updateRateControl();
	ASSERT_TRUE(throttle.update());
	ASSERT_TRUE(steering.update());
	EXPECT_FLOAT_EQ(throttle.get().throttle_body_x, 0.f);
	EXPECT_FLOAT_EQ(steering.get().normalized_speed_diff, 0.f);

	offboard.body_rate = false;
	offboard.velocity = true;
	control.flag_control_velocity_enabled = true;
	publishVehicleInputs(true, offboard, control, hybrid, rover_setpoint);
	velocity_control.updateVelControl();
	rate_control.updateRateControl();
	ASSERT_TRUE(throttle.update());
	ASSERT_TRUE(steering.update());
	EXPECT_FLOAT_EQ(throttle.get().throttle_body_x, 0.f);
	EXPECT_FLOAT_EQ(steering.get().normalized_speed_diff, 0.f);
}

TEST_F(DifferentialOffboardControlTest, DedicatedInputMustBePostTransitionAndStatusHealthy)
{
	const hrt_abstime now = hrt_absolute_time();
	offboard_control_mode_s offboard{};
	offboard.timestamp = now;
	offboard.rover_velocity = true;
	vehicle_control_mode_s control{};
	control.flag_armed = true;
	control.flag_control_offboard_enabled = true;
	control.flag_control_velocity_enabled = true;
	control.flag_control_rates_enabled = true;
	control.flag_control_allocation_enabled = true;
	hybrid_vehicle_status_s hybrid{};
	hybrid.timestamp = now;
	hybrid.transition_completed_timestamp = now;
	hybrid.current_state = hybrid_vehicle_status_s::HYBRID_STATE_DRIVING;
	rover_velocity_setpoint_s rover_setpoint{};
	rover_setpoint.timestamp = now;
	rover_setpoint.speed_body_x = 0.5f;
	rover_setpoint.yaw_rate = 0.2f;

	publishVehicleInputs(true, offboard, control, hybrid, rover_setpoint);
	DifferentialVelControl velocity_control{nullptr};
	DifferentialRateControl rate_control{nullptr};
	uORB::SubscriptionData<rover_throttle_setpoint_s> throttle{ORB_ID(rover_throttle_setpoint)};
	uORB::SubscriptionData<rover_steering_setpoint_s> steering{ORB_ID(rover_steering_setpoint)};
	velocity_control.updateVelControl();
	rate_control.updateRateControl();
	ASSERT_TRUE(throttle.update());
	ASSERT_TRUE(steering.update());
	EXPECT_FLOAT_EQ(throttle.get().throttle_body_x, 0.f);
	EXPECT_FLOAT_EQ(steering.get().normalized_speed_diff, 0.f);

	hybrid.timestamp = hrt_absolute_time();
	hybrid.transition_completed_timestamp = now - 1;
	hybrid.fault_reason = hybrid_vehicle_status_s::TRANSFORM_FAULT_SENSOR_TIMEOUT;
	rover_setpoint.timestamp = now;
	publishVehicleInputs(true, offboard, control, hybrid, rover_setpoint);
	velocity_control.updateVelControl();
	rate_control.updateRateControl();
	ASSERT_TRUE(throttle.update());
	ASSERT_TRUE(steering.update());
	EXPECT_FLOAT_EQ(throttle.get().throttle_body_x, 0.f);
	EXPECT_FLOAT_EQ(steering.get().normalized_speed_diff, 0.f);

	hybrid.fault_reason = hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE;
	hybrid.timestamp = 1;
	publishVehicleInputs(true, offboard, control, hybrid, rover_setpoint);
	velocity_control.updateVelControl();
	rate_control.updateRateControl();
	ASSERT_TRUE(throttle.update());
	ASSERT_TRUE(steering.update());
	EXPECT_FLOAT_EQ(throttle.get().throttle_body_x, 0.f);
	EXPECT_FLOAT_EQ(steering.get().normalized_speed_diff, 0.f);

	hybrid.timestamp = hrt_absolute_time();
	hybrid.transition_completed_timestamp = now - 1;
	rover_setpoint.timestamp = now;
	publishVehicleInputs(true, offboard, control, hybrid, rover_setpoint);
	velocity_control.updateVelControl();
	rate_control.updateRateControl();
	ASSERT_TRUE(throttle.update());
	ASSERT_TRUE(steering.update());
	EXPECT_GT(throttle.get().throttle_body_x, 0.f);
	EXPECT_GT(steering.get().normalized_speed_diff, 0.f);
}

TEST_F(DifferentialOffboardControlTest, NonHybridLegacyVelocityAndBodyRateRemainActive)
{
	const hrt_abstime now = hrt_absolute_time();
	vehicle_control_mode_s control{};
	control.flag_armed = true;
	control.flag_control_offboard_enabled = true;
	control.flag_control_velocity_enabled = true;
	control.flag_control_rates_enabled = true;
	control.flag_control_allocation_enabled = true;
	offboard_control_mode_s offboard{};
	offboard.timestamp = now;
	offboard.velocity = true;
	trajectory_setpoint_s trajectory{};
	trajectory.velocity[0] = 0.5f;
	trajectory.velocity[1] = 0.f;
	trajectory.yawspeed = 0.2f;
	publishVehicleInputs(false, offboard, control, {}, {}, trajectory);

	DifferentialVelControl velocity_control{nullptr};
	uORB::SubscriptionData<rover_throttle_setpoint_s> throttle{ORB_ID(rover_throttle_setpoint)};
	velocity_control.updateVelControl();
	ASSERT_TRUE(throttle.update());
	EXPECT_GT(throttle.get().throttle_body_x, 0.f);

	offboard.velocity = false;
	offboard.body_rate = true;
	control.flag_control_velocity_enabled = false;
	publishVehicleInputs(false, offboard, control, {}, {}, trajectory);
	DifferentialRateControl rate_control{nullptr};
	uORB::SubscriptionData<rover_steering_setpoint_s> steering{ORB_ID(rover_steering_setpoint)};
	rate_control.updateRateControl();
	ASSERT_TRUE(steering.update());
	EXPECT_GT(steering.get().normalized_speed_diff, 0.f);
}

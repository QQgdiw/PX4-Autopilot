// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>
#include <unistd.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include "FlightTaskAuto.hpp"

class AutoHarness : public FlightTaskAuto
{
public:
	float landHeading() const { return _land_heading; }
	bool localHeadingValid() const { return _is_local_yaw_good_for_control; }
	bool update() override
	{
		_deltatime = 0.02f;
		return FlightTaskAuto::update();
	}
};

class FlightTaskAutoTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		param_control_autosave(false);
		int32_t rc_help = 0;
		param_set(param_find("MPC_LAND_RC_HELP"), &rc_help);
		int32_t wv_enable = 0;
		param_set(param_find("WV_EN"), &wv_enable);
		_position.timestamp = hrt_absolute_time();
		_position.timestamp_sample = _position.timestamp;
		_position.xy_valid = _position.z_valid = true;
		_position.v_xy_valid = _position.v_z_valid = true;
		_position.ref_timestamp = 1;
		_position.ref_lat = 47.;
		_position.ref_lon = 8.;
		_position.ref_alt = 500.f;
		_position.z = -10.f;
		_position.heading = 0.7f;
		_position.heading_good_for_local_control = true;
		_triplet.current.valid = true;
		_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;
		_triplet.current.lat = NAN;
		_triplet.current.lon = NAN;
		_triplet.current.alt = 500.f;
		_triplet.current.yaw = NAN;
		_triplet.current.cruising_speed = NAN;
		_triplet.current.acceptance_radius = 1.f;
	}

	void publish()
	{
		_position.timestamp = hrt_absolute_time();
		_position.timestamp_sample = _position.timestamp;
		_position_pub.publish(_position);
		_triplet_pub.publish(_triplet);
	}

	void start(AutoHarness &task)
	{
		publish();
		ASSERT_TRUE(task.updateInitialize());
		ASSERT_TRUE(task.activate(FlightTask::empty_trajectory_setpoint));
		step(task);
	}

	void step(AutoHarness &task)
	{
		publish();
		ASSERT_TRUE(task.updateInitialize());
		ASSERT_TRUE(task.update());
	}

	vehicle_local_position_s _position{};
	position_setpoint_triplet_s _triplet{};
	uORB::Publication<vehicle_local_position_s> _position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<position_setpoint_triplet_s> _triplet_pub{ORB_ID(position_setpoint_triplet)};
};

TEST_F(FlightTaskAutoTest, FrdLandingHoldsFixedLocalHeading)
{
	// FRD vision has a valid local yaw, but no absolute (north) alignment.
	_position.heading_good_for_control = false;
	FlightTaskAuto task;
	publish();
	ASSERT_TRUE(task.updateInitialize());
	ASSERT_TRUE(task.activate(FlightTask::empty_trajectory_setpoint));
	ASSERT_TRUE(task.updateInitialize());
	ASSERT_TRUE(task.update());
	EXPECT_NEAR(task.getTrajectorySetpoint().yaw, 0.7f, 1e-5f);
	_position.heading = 1.f; // disturbance must not move the captured target
	publish();
	ASSERT_TRUE(task.updateInitialize());
	ASSERT_TRUE(task.update());
	EXPECT_NEAR(task.getTrajectorySetpoint().yaw, 0.7f, 1e-5f);
}

TEST_F(FlightTaskAutoTest, HeadingLossAndRecoveryRecapturesOnce)
{
	AutoHarness task;
	start(task);
	EXPECT_NEAR(task.landHeading(), 0.7f, 1e-5f);
	_position.heading_good_for_local_control = false;
	_position.heading = 1.2f;
	step(task);
	EXPECT_FALSE(PX4_ISFINITE(task.getTrajectorySetpoint().yaw));
	EXPECT_FLOAT_EQ(task.getTrajectorySetpoint().yawspeed, 0.f);
	EXPECT_FALSE(PX4_ISFINITE(task.landHeading()));
	_position.heading_good_for_local_control = true;
	step(task);
	EXPECT_NEAR(task.landHeading(), 1.2f, 1e-5f);
	_position.heading = 1.5f;
	step(task);
	EXPECT_NEAR(task.landHeading(), 1.2f, 1e-5f);
}

TEST_F(FlightTaskAutoTest, HeadingResetAndEstimatorSwitchPreserveRelativeTarget)
{
	_position.heading = 3.f;
	AutoHarness task;
	start(task);
	_position.heading = matrix::wrap_pi(3.f + 0.6f);
	_position.delta_heading = 0.6f;
	_position.heading_reset_counter++;
	step(task);
	EXPECT_NEAR(task.landHeading(), matrix::wrap_pi(3.6f), 1e-5f);
	EXPECT_NEAR(task.getTrajectorySetpoint().yaw, task.landHeading(), 1e-5f);
	_position.heading += 0.1f;
	step(task);
	EXPECT_NEAR(task.landHeading(), matrix::wrap_pi(3.6f), 1e-5f);
	// Selector also synthesizes a heading reset on an instance switch.
	_position.heading_good_for_local_control = false;
	_position.heading_reset_counter++;
	_position.delta_heading = -1.f;
	step(task);
	EXPECT_FALSE(PX4_ISFINITE(task.landHeading()));
}

TEST_F(FlightTaskAutoTest, NonFiniteOrStalePositionDoesNotQualifyHeading)
{
	AutoHarness task;
	start(task);
	_position.heading = NAN;
	step(task);
	EXPECT_FALSE(PX4_ISFINITE(task.getTrajectorySetpoint().yaw));
	_position.timestamp = 0;
	_position_pub.publish(_position);
	EXPECT_FALSE(task.updateInitialize());
	EXPECT_FALSE(task.localHeadingValid());
	// Stale rejection is tested with a future timestamp (unsigned age fails closed).
	_position.timestamp = hrt_absolute_time() + 1000000;
	_position_pub.publish(_position);
	EXPECT_FALSE(task.updateInitialize());
	EXPECT_FALSE(task.localHeadingValid());
	_position.heading = 0.7f;
	publish();
	ASSERT_TRUE(task.updateInitialize());
	usleep(510000);
	EXPECT_FALSE(task.updateInitialize());
	EXPECT_FALSE(task.localHeadingValid());
}

TEST_F(FlightTaskAutoTest, ReentryDoesNotReusePreviousLandingHeading)
{
	AutoHarness task;
	start(task);
	_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;
	step(task);
	EXPECT_FALSE(PX4_ISFINITE(task.getTrajectorySetpoint().yaw)); // other Auto still requires north
	_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;
	_position.heading = -0.8f;
	step(task);
	EXPECT_NEAR(task.landHeading(), -0.8f, 1e-5f);
	_position.heading = 1.2f;
	start(task); // task activation after Offboard/manual takeover
	EXPECT_NEAR(task.landHeading(), 1.2f, 1e-5f);
}

TEST_F(FlightTaskAutoTest, ExplicitAbsoluteYawRequiresNorthAlignment)
{
	_triplet.current.yaw = -1.f;
	AutoHarness task;
	start(task);
	EXPECT_NEAR(task.landHeading(), 0.7f, 1e-5f); // not an FRD command
	_position.heading_good_for_control = true;
	start(task);
	EXPECT_NEAR(task.landHeading(), -1.f, 1e-5f);
	_position.heading = 0.2f;
	step(task);
	EXPECT_NEAR(task.landHeading(), -1.f, 1e-5f);
}

TEST_F(FlightTaskAutoTest, OffboardHandoffCapturesLocalYawRatherThanOldCommand)
{
	AutoHarness task;
	publish();
	ASSERT_TRUE(task.updateInitialize());
	auto last = FlightTask::empty_trajectory_setpoint;
	last.yaw = -2.f;
	ASSERT_TRUE(task.activate(last));
	step(task);
	EXPECT_NEAR(task.landHeading(), 0.7f, 1e-5f);
	_position.heading = 1.f;
	step(task);
	EXPECT_NEAR(task.landHeading(), 0.7f, 1e-5f);
	EXPECT_TRUE(PX4_ISFINITE(task.getTrajectorySetpoint().yaw));
}

TEST_F(FlightTaskAutoTest, AlignedNedAndOtherAutoRemainUnchanged)
{
	_position.heading_good_for_control = true;
	AutoHarness task;
	start(task);
	EXPECT_TRUE(PX4_ISFINITE(task.getTrajectorySetpoint().yaw));
	_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	_triplet.current.yaw = 0.7f;
	step(task);
	EXPECT_NEAR(task.getTrajectorySetpoint().yaw, 0.7f, 1e-5f);
	_position.heading_good_for_control = false;
	step(task);
	EXPECT_FALSE(PX4_ISFINITE(task.getTrajectorySetpoint().yaw));
	EXPECT_FLOAT_EQ(task.getTrajectorySetpoint().yawspeed, 0.f);
}

TEST_F(FlightTaskAutoTest, RcAssistanceRetainsYawRateButCannotRestoreInvalidHeading)
{
	int32_t rc_help = 1;
	param_set(param_find("MPC_LAND_RC_HELP"), &rc_help);
	AutoHarness task;
	start(task);
	uORB::Publication<manual_control_setpoint_s> manual_pub{ORB_ID(manual_control_setpoint)};
	manual_control_setpoint_s manual{};
	manual.timestamp = hrt_absolute_time();
	manual.valid = true;
	manual.yaw = 0.6f;
	manual_pub.publish(manual);
	step(task);
	EXPECT_GT(task.getTrajectorySetpoint().yawspeed, 0.f);
	_position.heading_good_for_local_control = false;
	manual.pitch = 0.4f;
	manual.timestamp = hrt_absolute_time();
	manual_pub.publish(manual);
	step(task);
	EXPECT_FALSE(PX4_ISFINITE(task.landHeading()));
	EXPECT_FALSE(PX4_ISFINITE(task.getTrajectorySetpoint().yaw));
	EXPECT_GT(task.getTrajectorySetpoint().yawspeed, 0.f);
}

TEST_F(FlightTaskAutoTest, LocalLandingIsIndependentOfGlobalYawMode)
{
	for (int32_t mode = 0; mode <= 5; ++mode) {
		param_set(param_find("MPC_YAW_MODE"), &mode);
		AutoHarness task;
		_position.heading = 0.7f;
		start(task);
		EXPECT_NEAR(task.landHeading(), 0.7f, 1e-5f) << mode;
		_position.heading = 1.f;
		step(task);
		EXPECT_NEAR(task.landHeading(), 0.7f, 1e-5f) << mode;
	}
}

TEST_F(FlightTaskAutoTest, WeatherVaneKeepsItsRateControl)
{
	int32_t enabled = 1;
	param_set(param_find("WV_EN"), &enabled);
	uORB::Publication<vehicle_attitude_setpoint_s> attitude_pub{ORB_ID(vehicle_attitude_setpoint)};
	vehicle_attitude_setpoint_s attitude{};
	attitude.q_d[0] = 1.f;
	attitude_pub.publish(attitude);
	AutoHarness task;
	start(task);
	EXPECT_FALSE(PX4_ISFINITE(task.getTrajectorySetpoint().yaw));
	EXPECT_FLOAT_EQ(task.getTrajectorySetpoint().yawspeed, 0.f);
}

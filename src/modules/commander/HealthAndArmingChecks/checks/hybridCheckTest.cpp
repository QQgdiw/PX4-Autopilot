/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "hybridCheck.hpp"

#include <uORB/uORB.h>
#include <uORB/topics/hybrid_vehicle_status.h>
#include <uORB/topics/m2006_motor_status.h>

namespace
{

void publishHybridStatus(uint8_t state, uint8_t fault_reason = hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE,
			 hrt_abstime timestamp = hrt_absolute_time(), bool sensors_enabled = false,
			 bool position_confirmed = true)
{
	hybrid_vehicle_status_s status{};
	status.timestamp = timestamp;
	status.current_state = state;
	status.fault_reason = fault_reason;
	status.sensors_enabled = sensors_enabled;
	status.position_confirmed = position_confirmed;
	orb_advertise(ORB_ID(hybrid_vehicle_status), &status);
}

void publishM2006Status(bool left_online, bool right_online, uint32_t faults = m2006_motor_status_s::DRIVE_FAULT_NONE)
{
	m2006_motor_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.online[0] = left_online;
	status.online[1] = right_online;
	status.fault_flags = faults;
	orb_advertise(ORB_ID(m2006_motor_status), &status);
}

} // namespace

class HybridCheckReporterHarness
{
public:
	static bool canArm(HybridChecks &checks, vehicle_status_s &vehicle_status)
	{
		failsafe_flags_s failsafe_flags{};
		Report reporter{failsafe_flags};
		Context context{vehicle_status};
		reporter.reset();
		reporter.prepare(vehicle_status.vehicle_type);
		checks.checkAndReport(context, reporter);
		reporter.finalize();
		reporter.report(false);
		return reporter.canArm(vehicle_status_s::NAVIGATION_STATE_POSCTL);
	}
};

bool canArm(HybridChecks &checks, vehicle_status_s &vehicle_status)
{
	return HybridCheckReporterHarness::canArm(checks, vehicle_status);
}

class HybridCheckTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		_vehicle_status.is_quad_rover = true;
		_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
		_configuration.m2006_enabled = 1;
		_configuration.servo_function = 201;
		_configuration.m7_disarmed = 1000;
		_configuration.m7_failsafe = -1;
		_configuration.m8_disarmed = 0;
		_configuration.m8_failsafe = 0;
		_configuration.quad_servo_target = -0.5f;
		_configuration.rover_servo_target = 0.5f;
		_configuration.speed_p = 1.f;
	}

	vehicle_status_s _vehicle_status{};
	HybridCheckConfiguration _configuration{};
};

TEST_F(HybridCheckTest, FlyingDoesNotRequireM2006)
{
	publishHybridStatus(hybrid_vehicle_status_s::HYBRID_STATE_FLYING);
	publishM2006Status(false, false, m2006_motor_status_s::DRIVE_FAULT_CAN);
	HybridChecks checks;
	checks.setConfigurationForTesting(_configuration);
	EXPECT_TRUE(canArm(checks, _vehicle_status));
}

TEST_F(HybridCheckTest, DrivingRequiresBothM2006Online)
{
	publishHybridStatus(hybrid_vehicle_status_s::HYBRID_STATE_DRIVING);
	publishM2006Status(true, false);
	HybridChecks checks;
	checks.setConfigurationForTesting(_configuration);
	EXPECT_FALSE(canArm(checks, _vehicle_status));
}

TEST_F(HybridCheckTest, UnknownTransitionAndFaultRejectArming)
{
	publishM2006Status(true, true);
	HybridChecks checks;
	checks.setConfigurationForTesting(_configuration);
	publishHybridStatus(hybrid_vehicle_status_s::HYBRID_STATE_UNKNOWN);
	EXPECT_FALSE(canArm(checks, _vehicle_status));
	publishHybridStatus(hybrid_vehicle_status_s::HYBRID_STATE_TRANSITION_FAULT);
	EXPECT_FALSE(canArm(checks, _vehicle_status));
	publishHybridStatus(hybrid_vehicle_status_s::HYBRID_STATE_TRANSITIONING);
	EXPECT_FALSE(canArm(checks, _vehicle_status));
	publishHybridStatus(hybrid_vehicle_status_s::HYBRID_STATE_FLYING,
			    hybrid_vehicle_status_s::TRANSFORM_FAULT_SENSOR_TIMEOUT);
	EXPECT_FALSE(canArm(checks, _vehicle_status));
	publishHybridStatus(hybrid_vehicle_status_s::HYBRID_STATE_FLYING,
			    hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE, 0);
	EXPECT_FALSE(canArm(checks, _vehicle_status));
}

TEST_F(HybridCheckTest, UnsafeM8MappingRejectsArming)
{
	publishHybridStatus(hybrid_vehicle_status_s::HYBRID_STATE_FLYING);
	_configuration.m8_failsafe = 1000;
	HybridChecks checks;
	checks.setConfigurationForTesting(_configuration);
	EXPECT_FALSE(canArm(checks, _vehicle_status));
	_configuration.m8_failsafe = 0;
	_configuration.quad_servo_target = -2.f;
	checks.setConfigurationForTesting(_configuration);
	EXPECT_FALSE(canArm(checks, _vehicle_status));
}

TEST_F(HybridCheckTest, AllZeroControllerRejectsDriving)
{
	publishHybridStatus(hybrid_vehicle_status_s::HYBRID_STATE_DRIVING);
	publishM2006Status(true, true);
	_configuration.speed_p = 0.f;
	HybridChecks checks;
	checks.setConfigurationForTesting(_configuration);
	EXPECT_FALSE(canArm(checks, _vehicle_status));
}

TEST_F(HybridCheckTest, EnabledSensorsRequireCurrentPositionConfirmation)
{
	publishHybridStatus(hybrid_vehicle_status_s::HYBRID_STATE_FLYING,
			    hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE, hrt_absolute_time(), true, false);
	HybridChecks checks;
	checks.setConfigurationForTesting(_configuration);
	EXPECT_FALSE(canArm(checks, _vehicle_status));
}

TEST_F(HybridCheckTest, NegativeControllerGainRejectsDriving)
{
	publishHybridStatus(hybrid_vehicle_status_s::HYBRID_STATE_DRIVING);
	publishM2006Status(true, true);
	HybridChecks checks;
	_configuration.speed_p = -1.f;
	checks.setConfigurationForTesting(_configuration);
	EXPECT_FALSE(canArm(checks, _vehicle_status));

	_configuration.speed_p = 1.f;
	_configuration.speed_d = -1.f;
	checks.setConfigurationForTesting(_configuration);
	EXPECT_FALSE(canArm(checks, _vehicle_status));
}

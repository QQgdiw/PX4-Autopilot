/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "../Common.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/topics/hybrid_vehicle_status.h>
#include <uORB/topics/m2006_motor_status.h>

struct HybridCheckConfiguration {
	int32_t m2006_enabled{0};
	int32_t uavcan_enabled{0};
	int32_t cyphal_enabled{0};
	int32_t motor5_function{0};
	int32_t motor6_function{0};
	int32_t servo_function{0};
	int32_t servo_disarmed{0};
	int32_t servo_failsafe{0};
	float quad_servo_target{NAN};
	float rover_servo_target{NAN};
	float speed_p{0.f};
	float speed_i{0.f};
	float speed_ff{0.f};
};

class HybridChecks : public HealthAndArmingCheckBase
{
public:
	HybridChecks() = default;
	~HybridChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	void setConfigurationForTesting(const HybridCheckConfiguration &configuration) { _test_configuration = configuration; }
	FRIEND_TEST(HybridCheckTest, FlyingDoesNotRequireM2006);
	FRIEND_TEST(HybridCheckTest, DrivingRequiresBothM2006Online);
	FRIEND_TEST(HybridCheckTest, UnknownTransitionAndFaultRejectArming);
	FRIEND_TEST(HybridCheckTest, UnsafeM8MappingRejectsArming);
	FRIEND_TEST(HybridCheckTest, AllZeroControllerRejectsDriving);
	bool getConfiguration(HybridCheckConfiguration &configuration) const;
	bool hasSafeServoMapping(const HybridCheckConfiguration &configuration) const;
	bool hasConfiguredSpeedController(const HybridCheckConfiguration &configuration) const;
	bool hasM2006Conflict(const HybridCheckConfiguration &configuration) const;
	bool isM2006Healthy(const m2006_motor_status_s &status) const;

	uORB::Subscription _hybrid_status_sub{ORB_ID(hybrid_vehicle_status)};
	uORB::Subscription _m2006_status_sub{ORB_ID(m2006_motor_status)};
	HybridCheckConfiguration _test_configuration{};
};

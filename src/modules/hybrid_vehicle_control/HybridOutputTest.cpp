/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 ****************************************************************************/

#include <gtest/gtest.h>
#include "hybrid_vehicle_control.hpp"

using namespace hybrid_control;

class HybridOutputTest : public ::testing::Test
{
protected:
	HybridVehicleControl module;
	uORB::Subscription output_sub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_motors_s> mc_pub{ORB_ID(actuator_motors_mc)};
	uORB::Publication<actuator_motors_s> rover_pub{ORB_ID(actuator_motors_rover)};

	void SetUp() override
	{
		TransformationConfig config{};
		config.sensors_enabled = true;
		config.backend = ActuatorBackend::Hx65;
		module._transformation_config_tracker.update(config, true);
		shape(HybridState::Flying, PropulsionOwner::Quad);
	}

	void shape(HybridState state, PropulsionOwner owner)
	{
		module._transformation_output.state = state;
		module._transformation_output.fault = TransformFault::None;
		module._transformation_output.position_confirmed = true;
		module._sequence_initialized = true;
		module._sequence_output.propulsion_owner = owner;
		module._sequence_output.propulsion_ready = true;
	}

	void send(uint64_t timestamp)
	{
		actuator_motors_s motors{};
		motors.timestamp = timestamp;
		motors.timestamp_sample = timestamp - 10;
		motors.control[0] = 0.3f;
		motors.control[1] = 0.7f;
		motors.reversible_flags = 3;
		mc_pub.publish(motors);
		rover_pub.publish(motors);
	}

	actuator_motors_s output(uint64_t now)
	{
		module.publish_motor_outputs(now);
		actuator_motors_s result{};
		EXPECT_TRUE(output_sub.copy(&result));
		return result;
	}

	void stopped(uint64_t now)
	{
		const auto result = output(now);

		for (float value : result.control) {
			EXPECT_TRUE(std::isnan(value));
		}
	}

	void gate(int gate)
	{
		if (gate == 0) { module._actuator_armed.manual_lockdown = true; }

		if (gate == 1) { module._actuator_armed.lockdown = true; }

		if (gate == 2) { module._actuator_armed.force_failsafe = true; }

		if (gate == 3) { module._manual_commissioning_active = true; }

		if (gate == 4) { module._sequence_output.propulsion_ready = false; }

		if (gate == 5) { module._transformation_output.position_confirmed = false; }
	}
	void reset_gates()
	{
		module._actuator_armed = {};
		module._manual_commissioning_active = false;
		shape(HybridState::Flying, PropulsionOwner::Quad);
	}
	void epoch(uint64_t timestamp) { module._transition_complete_time = timestamp; }
	bool management(uint64_t now) { return module.management_due(now); }
};

TEST_F(HybridOutputTest, NewControlsBypassManagementAndRetainSourceTime)
{
	unsigned management_count = 0;

	for (uint64_t now = 100000; now < 200000; now += 1000) {
		management_count += management(now);
		send(now);
		const auto result = output(now);
		EXPECT_EQ(result.timestamp, now);
		EXPECT_EQ(result.timestamp_sample, now - 10);
		EXPECT_FLOAT_EQ(result.control[0], 0.3f);
		EXPECT_TRUE(std::isnan(result.control[4]));
	}

	EXPECT_EQ(management_count, 5u);
	const auto repeated = output(200000);
	EXPECT_EQ(repeated.timestamp, 199000u);
	EXPECT_EQ(repeated.timestamp_sample, 198990u);
	stopped(299001);
}

TEST_F(HybridOutputTest, ColdStartTransitionEpochAndRoverMapping)
{
	stopped(1000000); // no source is valid at cold start
	send(1000000);
	shape(HybridState::TransitionToRover, PropulsionOwner::None);
	stopped(1000001);
	shape(HybridState::Driving, PropulsionOwner::Rover);
	epoch(1000010);
	stopped(1000020);
	send(1000030);
	const auto result = output(1000040);
	EXPECT_FLOAT_EQ(result.control[4], 0.7f);
	EXPECT_FLOAT_EQ(result.control[5], 0.3f);
	EXPECT_EQ(result.reversible_flags, 48);
	EXPECT_TRUE(std::isnan(result.control[0]));
	shape(HybridState::Fault, PropulsionOwner::None);
	stopped(1000041);
	shape(HybridState::Flying, PropulsionOwner::Rover);
	stopped(1000042);
}

TEST_F(HybridOutputTest, KillFaultCommissioningAndReadinessOverrideFreshInput)
{
	send(2000000);

	for (int index = 0; index < 6; ++index) {
		reset_gates();
		gate(index);
		stopped(2000001);
	}
}

#include <gtest/gtest.h>

#include <limits>

#include "TransformationStateMachine.hpp"

using namespace hybrid_control;

namespace
{
TransformationConfig config(bool sensors_enabled = true)
{
	return {sensors_enabled, HybridState::Flying, -0.7f, 0.8f, 10.f, 170.f, 2.f,
		1000000, 100000, 3000000};
}

TransformationInput input(uint64_t now_us = 0)
{
	return {now_us, false, 0.f, false, false, false, false};
}
}

TEST(TransformationStateMachine, As5600CompletesAfterDebounce)
{
	TransformationStateMachine machine;
	auto initial = input();
	initial.as5600_valid = true;
	initial.as5600_angle = 10.f;
	EXPECT_EQ(machine.initialize(config(), initial).state, HybridState::Flying);
	EXPECT_EQ(machine.request(HybridTarget::Driving, 0).state, HybridState::TransitionToRover);

	auto sensed = input();
	sensed.as5600_valid = true;
	sensed.as5600_angle = 170.f;
	EXPECT_EQ(machine.update(sensed).state, HybridState::TransitionToRover);
	sensed.now_us = 99999;
	EXPECT_EQ(machine.update(sensed).state, HybridState::TransitionToRover);
	sensed.now_us = 100000;
	auto output = machine.update(sensed);
	EXPECT_EQ(output.state, HybridState::Driving);
	EXPECT_EQ(output.source, SensorSource::As5600);
	EXPECT_TRUE(output.servo_enabled);
	EXPECT_FLOAT_EQ(output.servo_value, 0.8f);
}

TEST(TransformationStateMachine, FallsBackToTargetTmag)
{
	TransformationStateMachine machine;
	EXPECT_EQ(machine.initialize(config(), input()).state, HybridState::Unknown);
	machine.request(HybridTarget::Driving, 0);
	auto sensed = input();
	sensed.tmag_rover_valid = true;
	sensed.tmag_rover_active = true;
	EXPECT_EQ(machine.update(sensed).source, SensorSource::Tmag5273);
	sensed.now_us = 100000;
	EXPECT_EQ(machine.update(sensed).state, HybridState::Driving);
}

TEST(TransformationStateMachine, SensorConflictFaultsAndReleasesServo)
{
	TransformationStateMachine machine;
	machine.initialize(config(), input());
	machine.request(HybridTarget::Driving, 0);
	auto sensed = input();
	sensed.as5600_valid = true;
	sensed.as5600_angle = 170.f;
	sensed.tmag_quad_valid = true;
	sensed.tmag_quad_active = true;
	auto output = machine.update(sensed);
	EXPECT_EQ(output.state, HybridState::Fault);
	EXPECT_EQ(output.fault, TransformFault::SensorConflict);
	EXPECT_FALSE(output.servo_enabled);

	TransformationStateMachine both_tmag;
	both_tmag.initialize(config(), input());
	both_tmag.request(HybridTarget::Driving, 0);
	sensed = input();
	sensed.tmag_quad_valid = sensed.tmag_rover_valid = true;
	sensed.tmag_quad_active = sensed.tmag_rover_active = true;
	EXPECT_EQ(both_tmag.update(sensed).fault, TransformFault::SensorConflict);
}

TEST(TransformationStateMachine, EnabledSensorTimeoutFaults)
{
	TransformationStateMachine machine;
	machine.initialize(config(), input());
	machine.request(HybridTarget::Driving, 0);
	auto no_sensor = input(999999);
	EXPECT_EQ(machine.update(no_sensor).state, HybridState::TransitionToRover);
	no_sensor.now_us = 1000000;
	auto output = machine.update(no_sensor);
	EXPECT_EQ(output.state, HybridState::Fault);
	EXPECT_EQ(output.fault, TransformFault::SensorTimeout);
	EXPECT_FALSE(output.servo_enabled);
}

TEST(TransformationStateMachine, DisabledSensorsCompleteByTime)
{
	TransformationStateMachine machine;
	machine.initialize(config(false), input());
	machine.request(HybridTarget::Driving, 10);
	auto elapsed = input(3000009);
	EXPECT_EQ(machine.update(elapsed).state, HybridState::TransitionToRover);
	elapsed.now_us = 3000010;
	auto output = machine.update(elapsed);
	EXPECT_EQ(output.state, HybridState::Driving);
	EXPECT_EQ(output.source, SensorSource::None);
	EXPECT_TRUE(output.servo_enabled);
}

TEST(TransformationStateMachine, StartupUsesConfiguredStateOnlyWhenSensorsDisabled)
{
	auto boot_driving = config(false);
	boot_driving.configured_boot_state = HybridState::Driving;
	TransformationStateMachine without_sensors;
	EXPECT_EQ(without_sensors.initialize(boot_driving, input()).state, HybridState::Driving);

	boot_driving.sensors_enabled = true;
	TransformationStateMachine with_sensors;
	EXPECT_EQ(with_sensors.initialize(boot_driving, input()).state, HybridState::Unknown);
	auto actual = input();
	actual.as5600_valid = true;
	actual.as5600_angle = 10.f;
	EXPECT_EQ(with_sensors.update(actual).state, HybridState::Flying);
}

TEST(TransformationStateMachine, FaultClearsOnlyDisarmedWithExplicitRequest)
{
	TransformationStateMachine machine;
	machine.initialize(config(), input());
	machine.request(HybridTarget::Driving, 0);
	auto conflict = input();
	conflict.tmag_quad_valid = conflict.tmag_rover_valid = true;
	conflict.tmag_quad_active = conflict.tmag_rover_active = true;
	EXPECT_EQ(machine.update(conflict).state, HybridState::Fault);

	auto healthy = input(1);
	healthy.as5600_valid = true;
	healthy.as5600_angle = 10.f;
	EXPECT_EQ(machine.update(healthy).state, HybridState::Fault);
	EXPECT_EQ(machine.clearFault(false).state, HybridState::Fault);
	EXPECT_EQ(machine.clearFault(true).state, HybridState::Unknown);
}

TEST(TransformationStateMachine, InvalidServoConfigFaults)
{
	TransformationStateMachine machine;
	auto invalid = config(false);
	invalid.rover_servo = 1.1f;
	auto output = machine.initialize(invalid, input());
	EXPECT_EQ(output.state, HybridState::Fault);
	EXPECT_EQ(output.fault, TransformFault::InvalidServoConfig);
	EXPECT_FALSE(output.servo_enabled);

	invalid = config(false);
	invalid.rover_servo = invalid.quad_servo + 0.09f;
	EXPECT_EQ(machine.initialize(invalid, input()).fault, TransformFault::InvalidServoConfig);
}

TEST(TransformationStateMachine, OutOfRangeServoConfigCannotClearFault)
{
	TransformationStateMachine machine;
	auto invalid = config(false);
	invalid.rover_servo = 1.1f;
	ASSERT_EQ(machine.initialize(invalid, input()).fault, TransformFault::InvalidServoConfig);

	auto output = machine.clearFault(true);
	EXPECT_EQ(output.state, HybridState::Fault);
	EXPECT_EQ(output.fault, TransformFault::InvalidServoConfig);
	EXPECT_FALSE(output.servo_enabled);
}

TEST(TransformationStateMachine, NonFiniteServoConfigCannotClearFault)
{
	TransformationStateMachine machine;
	auto invalid = config(false);
	invalid.quad_servo = std::numeric_limits<float>::quiet_NaN();
	ASSERT_EQ(machine.initialize(invalid, input()).fault, TransformFault::InvalidServoConfig);

	auto output = machine.clearFault(true);
	EXPECT_EQ(output.state, HybridState::Fault);
	EXPECT_EQ(output.fault, TransformFault::InvalidServoConfig);
	EXPECT_FALSE(output.servo_enabled);
}

TEST(TransformationStateMachine, StableSameTargetRequestPreservesOutput)
{
	TransformationStateMachine machine;
	auto sensed = input();
	sensed.as5600_valid = true;
	sensed.as5600_angle = 10.f;
	const auto before = machine.initialize(config(), sensed);
	ASSERT_EQ(before.state, HybridState::Flying);
	ASSERT_EQ(before.source, SensorSource::As5600);

	const auto after = machine.request(HybridTarget::Flying, 123456);
	EXPECT_EQ(after.state, before.state);
	EXPECT_EQ(after.source, before.source);
	EXPECT_EQ(after.servo_enabled, before.servo_enabled);
	EXPECT_FLOAT_EQ(after.servo_value, before.servo_value);
}

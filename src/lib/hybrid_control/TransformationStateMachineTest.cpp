#include <gtest/gtest.h>

#include <limits>

#include "TransformationStateMachine.hpp"

using namespace hybrid_control;

namespace
{
TransformationConfig config(bool sensors_enabled = true)
{
	return {sensors_enabled, 0, -0.7f, 0.8f, 0.5f, 3.f, 0.05f,
		1.f, 0.1f, 3.f, 53, 34, 5.f, 5.f};
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
	initial.as5600_angle = 0.5f;
	EXPECT_EQ(machine.initialize(config(), initial).state, HybridState::Flying);
	EXPECT_EQ(machine.request(HybridTarget::Driving, 0).state, HybridState::TransitionToRover);

	auto sensed = input();
	sensed.as5600_valid = true;
	sensed.as5600_angle = 3.f;
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
	sensed.as5600_angle = 3.f;
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
	boot_driving.configured_boot_state = 1;
	TransformationStateMachine without_sensors;
	EXPECT_EQ(without_sensors.initialize(boot_driving, input()).state, HybridState::Driving);

	boot_driving.sensors_enabled = true;
	TransformationStateMachine with_sensors;
	EXPECT_EQ(with_sensors.initialize(boot_driving, input()).state, HybridState::Unknown);
	auto actual = input();
	actual.as5600_valid = true;
	actual.as5600_angle = 0.5f;
	EXPECT_EQ(with_sensors.update(actual).state, HybridState::Flying);
}

TEST(TransformationStateMachine, StableStateRequiresContinuousEndpointConfirmation)
{
	TransformationStateMachine machine;
	auto sensed = input();
	sensed.as5600_valid = true;
	sensed.as5600_angle = 0.5f;
	ASSERT_EQ(machine.initialize(config(), sensed).state, HybridState::Flying);

	auto missing = input(50000);
	auto output = machine.update(missing);
	EXPECT_EQ(output.state, HybridState::Flying);
	EXPECT_FALSE(output.position_confirmed);
	EXPECT_FALSE(stablePositionSafe(output, true));

	missing.now_us = 150000;
	output = machine.update(missing);
	EXPECT_EQ(output.state, HybridState::Fault);
	EXPECT_EQ(output.fault, TransformFault::NoSensor);
}

TEST(TransformationStateMachine, StableStateFallsBackToMatchingTmag)
{
	TransformationStateMachine machine;
	auto sensed = input();
	sensed.as5600_valid = true;
	sensed.as5600_angle = 3.f;
	ASSERT_EQ(machine.initialize(config(), sensed).state, HybridState::Driving);

	auto fallback = input(100000);
	fallback.tmag_rover_valid = true;
	fallback.tmag_rover_active = true;
	const auto output = machine.update(fallback);
	EXPECT_EQ(output.state, HybridState::Driving);
	EXPECT_TRUE(output.position_confirmed);
	EXPECT_EQ(output.source, SensorSource::Tmag5273);
}

TEST(TransformationStateMachine, As5600EndpointComparisonWrapsAtTwoPi)
{
	auto cfg = config();
	cfg.quad_angle = 0.01f;
	TransformationStateMachine machine;
	auto sensed = input();
	sensed.as5600_valid = true;
	sensed.as5600_angle = 6.28f;
	const auto output = machine.initialize(cfg, sensed);
	EXPECT_EQ(output.state, HybridState::Flying);
	EXPECT_TRUE(output.position_confirmed);
}

TEST(TransformationStateMachine, RepeatedTransitionRequestDoesNotRestartTimeout)
{
	TransformationStateMachine machine;
	auto cfg = config();
	cfg.sensor_timeout_s = 5.f;
	ASSERT_EQ(machine.initialize(cfg, input()).state, HybridState::Unknown);
	ASSERT_EQ(machine.request(HybridTarget::Driving, 100).state, HybridState::TransitionToRover);
	EXPECT_EQ(machine.request(HybridTarget::Driving, 2900000).state, HybridState::TransitionToRover);

	const auto output = machine.update(input(3000100));
	EXPECT_EQ(output.state, HybridState::Fault);
	EXPECT_EQ(output.fault, TransformFault::TransitionTimeout);
}

TEST(TransformationStateMachine, RepeatedOpenLoopRequestCompletesAtOriginalDeadline)
{
	TransformationStateMachine machine;
	ASSERT_EQ(machine.initialize(config(false), input()).state, HybridState::Flying);
	ASSERT_EQ(machine.request(HybridTarget::Driving, 100).state, HybridState::TransitionToRover);
	EXPECT_EQ(machine.request(HybridTarget::Driving, 2900000).state, HybridState::TransitionToRover);
	EXPECT_EQ(machine.update(input(3000099)).state, HybridState::TransitionToRover);
	EXPECT_EQ(machine.update(input(3000100)).state, HybridState::Driving);
}

TEST(TransformationStateMachine, ConfigurationChangesApplyOnlyWhenSafe)
{
	TransformationConfigTracker tracker;
	auto active = config(false);
	ASSERT_TRUE(tracker.update(active, true));

	auto changed = active;
	changed.rover_servo = 0.9f;
	changed.tmag_rover_threshold = 8.f;
	EXPECT_FALSE(tracker.update(changed, false));
	EXPECT_TRUE(tracker.hasPending());
	EXPECT_FLOAT_EQ(tracker.active().rover_servo, 0.8f);
	EXPECT_FLOAT_EQ(tracker.active().tmag_rover_threshold, 5.f);

	EXPECT_TRUE(tracker.update(changed, true));
	EXPECT_FALSE(tracker.hasPending());
	EXPECT_FLOAT_EQ(tracker.active().rover_servo, 0.9f);
	EXPECT_FLOAT_EQ(tracker.active().tmag_rover_threshold, 8.f);
}

TEST(TransformationStateMachine, InitialConfigurationWaitsForSafeState)
{
	TransformationConfigTracker tracker;
	const auto requested = config(false);

	EXPECT_FALSE(tracker.update(requested, false));
	EXPECT_FALSE(tracker.hasActive());
	EXPECT_TRUE(tracker.hasPending());
	EXPECT_TRUE(tracker.update(requested, true));
	EXPECT_TRUE(tracker.hasActive());
	EXPECT_FALSE(tracker.hasPending());
}

TEST(TransformationStateMachine, RevertedPendingConfigurationIsNotApplied)
{
	TransformationConfigTracker tracker;
	const auto active = config(false);
	ASSERT_TRUE(tracker.update(active, true));
	auto changed = active;
	changed.quad_angle = 2.f;

	EXPECT_FALSE(tracker.update(changed, false));
	EXPECT_TRUE(tracker.hasPending());
	EXPECT_FALSE(tracker.update(active, false));
	EXPECT_FALSE(tracker.hasPending());
	EXPECT_FALSE(tracker.update(active, true));
	EXPECT_FLOAT_EQ(tracker.active().quad_angle, active.quad_angle);
}

TEST(TransformationStateMachine, InvalidGeneralConfigurationFaultsAndReleasesServo)
{
	TransformationStateMachine machine;
	auto invalid = config(false);
	invalid.configured_boot_state = 2;
	auto output = machine.initialize(invalid, input());
	EXPECT_EQ(output.fault, TransformFault::InvalidConfiguration);
	EXPECT_FALSE(output.servo_enabled);

	invalid = config(false);
	invalid.max_transition_s = std::numeric_limits<float>::quiet_NaN();
	EXPECT_EQ(machine.initialize(invalid, input()).fault, TransformFault::InvalidConfiguration);

	invalid = config(false);
	invalid.sensor_timeout_s = -1.f;
	EXPECT_EQ(machine.initialize(invalid, input()).fault, TransformFault::InvalidConfiguration);

	invalid = config(false);
	invalid.debounce_s = std::numeric_limits<float>::infinity();
	EXPECT_EQ(machine.initialize(invalid, input()).fault, TransformFault::InvalidConfiguration);

	invalid = config(false);
	invalid.max_transition_s = 10.1f;
	EXPECT_EQ(machine.initialize(invalid, input()).fault, TransformFault::InvalidConfiguration);

	invalid = config(false);
	invalid.sensor_timeout_s = std::numeric_limits<float>::max();
	EXPECT_EQ(validateTransformationConfig(invalid), TransformFault::InvalidConfiguration);

	invalid = config(false);
	invalid.quad_angle = std::numeric_limits<float>::infinity();
	EXPECT_EQ(machine.initialize(invalid, input()).fault, TransformFault::InvalidConfiguration);

	invalid = config(false);
	invalid.rover_angle = -0.1f;
	EXPECT_EQ(machine.initialize(invalid, input()).fault, TransformFault::InvalidConfiguration);

	invalid = config(false);
	invalid.angle_tolerance = -0.1f;
	EXPECT_EQ(machine.initialize(invalid, input()).fault, TransformFault::InvalidConfiguration);

	invalid = config(false);
	invalid.tmag_quad_threshold = -0.1f;
	output = machine.initialize(invalid, input());
	EXPECT_EQ(output.state, HybridState::Fault);
	EXPECT_EQ(output.fault, TransformFault::InvalidConfiguration);
	EXPECT_FALSE(output.servo_enabled);

	invalid = config(false);
	invalid.tmag_rover_threshold = std::numeric_limits<float>::quiet_NaN();
	output = machine.initialize(invalid, input());
	EXPECT_EQ(output.fault, TransformFault::InvalidConfiguration);
	EXPECT_EQ(machine.clearFault(true).fault, TransformFault::InvalidConfiguration);
}

TEST(TransformationStateMachine, InvalidTmagDeviceIdsAreConfigurationFaults)
{
	auto invalid = config(false);
	invalid.tmag_rover_device_id = invalid.tmag_quad_device_id;
	EXPECT_EQ(validateTransformationConfig(invalid), TransformFault::InvalidConfiguration);

	invalid = config(false);
	invalid.tmag_quad_device_id = -1;
	EXPECT_EQ(validateTransformationConfig(invalid), TransformFault::InvalidConfiguration);
}

TEST(TransformationStateMachine, FreshNonFiniteManualValueClearsPreviousValidSample)
{
	ManualControlCache cache;
	cache.update(100, 0.75f, 100, 1000000);
	ASSERT_TRUE(cache.fresh(500000, 1000000));
	ASSERT_FLOAT_EQ(cache.value(), 0.75f);

	cache.update(500000, std::numeric_limits<float>::quiet_NaN(), 500000, 1000000);
	EXPECT_FALSE(cache.fresh(500000, 1000000));
	EXPECT_FLOAT_EQ(cache.value(), 0.f);
}

TEST(TransformationStateMachine, TmagCacheRejectsOldDeviceAfterIdChange)
{
	TmagSampleCache cache;
	cache.update(53, 8.f, 100);
	ASSERT_TRUE(cache.validFor(53, 500000, 1000000));
	EXPECT_FALSE(cache.validFor(54, 500000, 1000000));
}

TEST(TransformationStateMachine, FaultAlwaysBlocksManualCommissioning)
{
	TransformationStateMachine timeout_machine;
	auto cfg = config();
	ASSERT_EQ(timeout_machine.initialize(cfg, input()).state, HybridState::Unknown);
	ASSERT_EQ(timeout_machine.request(HybridTarget::Driving, 0).state, HybridState::TransitionToRover);
	const auto timeout_fault = timeout_machine.update(input(1000000));
	ASSERT_EQ(timeout_fault.fault, TransformFault::SensorTimeout);
	EXPECT_TRUE(isTransformationFaulted(timeout_fault));
	EXPECT_FALSE(manualCommissioningPermitted(timeout_fault, false, true, true));

	TransformationStateMachine invalid_machine;
	auto invalid = config(false);
	invalid.tmag_rover_device_id = invalid.tmag_quad_device_id;
	const auto config_fault = invalid_machine.initialize(invalid, input());
	ASSERT_EQ(config_fault.fault, TransformFault::InvalidConfiguration);
	EXPECT_TRUE(isTransformationFaulted(config_fault));
	EXPECT_FALSE(manualCommissioningPermitted(config_fault, false, true, true));

	TransformationStateMachine healthy_machine;
	const auto healthy = healthy_machine.initialize(config(false), input());
	ASSERT_FALSE(isTransformationFaulted(healthy));
	EXPECT_TRUE(manualCommissioningPermitted(healthy, false, true, true));
}

TEST(TransformationStateMachine, ManualCommissioningRequiresFreshPrearmedFaultFreeState)
{
	const TransformationOutput healthy{HybridState::Unknown, HybridTarget::None, SensorSource::None,
					   TransformFault::None, false, false, 0.f};
	const TransformationOutput state_fault{HybridState::Fault, HybridTarget::None, SensorSource::None,
					       TransformFault::None, false, false, 0.f};
	const TransformationOutput reported_fault{HybridState::Flying, HybridTarget::Flying, SensorSource::None,
			TransformFault::SensorTimeout, false, true, -0.7f};

	EXPECT_TRUE(manualCommissioningPermitted(healthy, false, true, true));
	EXPECT_FALSE(manualCommissioningPermitted(healthy, true, true, true));
	EXPECT_FALSE(manualCommissioningPermitted(healthy, false, false, true));
	EXPECT_FALSE(manualCommissioningPermitted(healthy, false, true, false));
	EXPECT_FALSE(manualCommissioningPermitted(state_fault, false, true, true));
	EXPECT_FALSE(manualCommissioningPermitted(reported_fault, false, true, true));
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
	healthy.as5600_angle = 0.5f;
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
	sensed.as5600_angle = 0.5f;
	const auto before = machine.initialize(config(), sensed);
	ASSERT_EQ(before.state, HybridState::Flying);
	ASSERT_EQ(before.source, SensorSource::As5600);

	const auto after = machine.request(HybridTarget::Flying, 123456);
	EXPECT_EQ(after.state, before.state);
	EXPECT_EQ(after.source, before.source);
	EXPECT_EQ(after.servo_enabled, before.servo_enabled);
	EXPECT_FLOAT_EQ(after.servo_value, before.servo_value);
}

TEST(TransformationStateMachine, StallsAfterExactNoProgressTimeout)
{
	TransformationStateMachine machine;
	auto cfg = config();
	cfg.sensor_timeout_s = 5.f;
	cfg.stall_timeout_s = 0.8f;
	cfg.stall_distance = 0.05f;
	ASSERT_EQ(machine.initialize(cfg, input()).state, HybridState::Unknown);
	ASSERT_EQ(machine.request(HybridTarget::Driving, 0).state, HybridState::TransitionToRover);

	auto stationary = input();
	stationary.actuator_command_effective = true;
	stationary.position = {0.2f, true, false, SensorSource::Hx8, 0};
	EXPECT_EQ(machine.update(stationary).state, HybridState::TransitionToRover);
	stationary.now_us = stationary.position.timestamp_us = 799999;
	EXPECT_EQ(machine.update(stationary).state, HybridState::TransitionToRover);
	stationary.now_us = stationary.position.timestamp_us = 800000;
	const auto output = machine.update(stationary);
	EXPECT_EQ(output.fault, TransformFault::Stall);
	EXPECT_EQ(output.no_progress_elapsed_us, 800000u);
}

TEST(TransformationStateMachine, SmallNetProgressDoesNotPostponeStall)
{
	TransformationStateMachine machine;
	auto cfg = config();
	cfg.sensor_timeout_s = 5.f;
	cfg.stall_timeout_s = 0.8f;
	cfg.stall_distance = 0.05f;
	machine.initialize(cfg, input());
	machine.request(HybridTarget::Driving, 0);

	auto sample = input();
	sample.actuator_command_effective = true;
	sample.position = {0.2f, true, false, SensorSource::Hx8, 0};
	machine.update(sample);
	sample.now_us = sample.position.timestamp_us = 400000;
	sample.position.normalized = 0.22f;
	EXPECT_EQ(machine.update(sample).state, HybridState::TransitionToRover);
	sample.now_us = sample.position.timestamp_us = 800000;
	EXPECT_EQ(machine.update(sample).fault, TransformFault::Stall);
}

TEST(TransformationStateMachine, ReverseAndOscillatingMotionDoesNotCountAsProgress)
{
	TransformationStateMachine machine;
	auto cfg = config();
	cfg.sensor_timeout_s = 5.f;
	cfg.stall_timeout_s = 0.8f;
	cfg.stall_distance = 0.05f;
	machine.initialize(cfg, input());
	machine.request(HybridTarget::Driving, 0);

	auto sample = input();
	sample.actuator_command_effective = true;
	sample.position = {0.2f, true, false, SensorSource::As5600, 0};
	machine.update(sample);
	sample.now_us = sample.position.timestamp_us = 300000;
	sample.position.normalized = 0.18f;
	machine.update(sample);
	sample.now_us = sample.position.timestamp_us = 600000;
	sample.position.normalized = 0.22f;
	machine.update(sample);
	sample.now_us = sample.position.timestamp_us = 800000;
	EXPECT_EQ(machine.update(sample).fault, TransformFault::Stall);
}

TEST(TransformationStateMachine, StableHoldNeverPositionStalls)
{
	TransformationStateMachine machine;
	auto sensed = input();
	sensed.as5600_valid = true;
	sensed.as5600_angle = 0.5f;
	sensed.actuator_command_effective = true;
	sensed.position = {0.f, true, true, SensorSource::As5600, 0};
	ASSERT_EQ(machine.initialize(config(), sensed).state, HybridState::Flying);

	sensed.now_us = sensed.position.timestamp_us = 800000;
	const auto output = machine.update(sensed);
	EXPECT_EQ(output.state, HybridState::Flying);
	EXPECT_EQ(output.fault, TransformFault::None);
}

TEST(TransformationStateMachine, EndpointConfirmationWinsAtStallBoundary)
{
	TransformationStateMachine machine;
	auto cfg = config();
	cfg.sensor_timeout_s = 5.f;
	cfg.stall_timeout_s = 0.8f;
	machine.initialize(cfg, input());
	machine.request(HybridTarget::Driving, 0);

	auto sample = input();
	sample.actuator_command_effective = true;
	sample.position = {0.2f, true, false, SensorSource::Hx8, 0};
	machine.update(sample);
	sample.now_us = sample.position.timestamp_us = 800000;
	sample.position.endpoint_confirmed = true;
	const auto output = machine.update(sample);
	EXPECT_EQ(output.state, HybridState::Driving);
	EXPECT_EQ(output.fault, TransformFault::None);
	EXPECT_EQ(output.source, SensorSource::Hx8);
}

TEST(TransformationStateMachine, AbsoluteTransitionTimeoutRemainsIndependent)
{
	TransformationStateMachine machine;
	auto cfg = config();
	cfg.sensor_timeout_s = 5.f;
	cfg.max_transition_s = 0.5f;
	cfg.stall_timeout_s = 0.8f;
	machine.initialize(cfg, input());
	machine.request(HybridTarget::Driving, 0);

	auto sample = input();
	sample.actuator_command_effective = true;
	sample.position = {0.2f, true, false, SensorSource::Hx8, 0};
	machine.update(sample);
	sample.now_us = sample.position.timestamp_us = 500000;
	EXPECT_EQ(machine.update(sample).fault, TransformFault::TransitionTimeout);
}

TEST(TransformationStateMachine, StallWinsWhenBothDeadlinesExpireTogether)
{
	TransformationStateMachine machine;
	auto cfg = config();
	cfg.sensor_timeout_s = 5.f;
	cfg.max_transition_s = 0.8f;
	cfg.stall_timeout_s = 0.8f;
	cfg.stall_distance = 0.05f;
	machine.initialize(cfg, input());
	machine.request(HybridTarget::Driving, 0);

	auto sample = input();
	sample.actuator_command_effective = true;
	sample.position = {0.2f, true, false, SensorSource::Hx8, 0};
	machine.update(sample);
	sample.now_us = sample.position.timestamp_us = 800000;
	EXPECT_EQ(machine.update(sample).fault, TransformFault::Stall);
}

TEST(TransformationStateMachine, PositionSourceChangeRestartsProgressWindow)
{
	TransformationStateMachine machine;
	auto cfg = config();
	cfg.sensor_timeout_s = 5.f;
	cfg.max_transition_s = 3.f;
	cfg.stall_timeout_s = 0.8f;
	machine.initialize(cfg, input());
	machine.request(HybridTarget::Driving, 0);

	auto sample = input();
	sample.actuator_command_effective = true;
	sample.position = {0.2f, true, false, SensorSource::As5600, 0};
	machine.update(sample);
	sample.now_us = sample.position.timestamp_us = 700000;
	sample.position.source = SensorSource::Hx8;
	EXPECT_EQ(machine.update(sample).state, HybridState::TransitionToRover);
	sample.now_us = sample.position.timestamp_us = 1499999;
	EXPECT_EQ(machine.update(sample).state, HybridState::TransitionToRover);
	sample.now_us = sample.position.timestamp_us = 1500000;
	EXPECT_EQ(machine.update(sample).fault, TransformFault::Stall);
}

TEST(TransformationStateMachine, ProgressingHx8PositionPreventsLegacySensorTimeout)
{
	TransformationStateMachine machine;
	auto cfg = config();
	cfg.sensor_timeout_s = 0.5f;
	cfg.max_transition_s = 2.f;
	cfg.stall_timeout_s = 0.8f;
	cfg.stall_distance = 0.05f;
	machine.initialize(cfg, input());
	machine.request(HybridTarget::Driving, 0);

	auto sample = input();
	sample.actuator_command_effective = true;
	sample.position = {0.1f, true, false, SensorSource::Hx8, 0};
	machine.update(sample);
	sample.now_us = sample.position.timestamp_us = 400000;
	sample.position.normalized = 0.3f;
	ASSERT_EQ(machine.update(sample).state, HybridState::TransitionToRover);
	sample.now_us = sample.position.timestamp_us = 500000;
	sample.position.normalized = 0.4f;
	const auto output = machine.update(sample);
	EXPECT_EQ(output.state, HybridState::TransitionToRover);
	EXPECT_EQ(output.fault, TransformFault::None);
}

TEST(TransformationStateMachine, ActuatorHealthFaultsUseDefinedPriority)
{
	auto actuatorFault = [](const ActuatorHealth &health) {
		TransformationStateMachine machine;
		auto cfg = config();
		cfg.sensor_timeout_s = 5.f;
		machine.initialize(cfg, input());
		machine.request(HybridTarget::Driving, 0);
		auto unhealthy = input();
		unhealthy.actuator = health;
		return machine.update(unhealthy);
	};

	EXPECT_EQ(actuatorFault({false, false, false, false, 1}).fault,
		  TransformFault::ActuatorCommunication);
	EXPECT_EQ(actuatorFault({true, true, false, true, 0}).fault,
		  TransformFault::ActuatorConfigMismatch);
	EXPECT_EQ(actuatorFault({true, false, true, true, 0}).fault,
		  TransformFault::ActuatorProtection);
	EXPECT_EQ(actuatorFault({true, true, true, true, 1}).fault,
		  TransformFault::ActuatorProtection);
	EXPECT_EQ(actuatorFault({true, true, true, false, 0}).fault,
		  TransformFault::ActuatorCommandRejected);
}

TEST(TransformationStateMachine, FirstFaultCauseIsLatchedAndRequestsRelease)
{
	TransformationStateMachine machine;
	auto cfg = config();
	cfg.sensor_timeout_s = 5.f;
	machine.initialize(cfg, input());
	machine.request(HybridTarget::Driving, 0);
	auto failed = input();
	failed.actuator.online = false;
	auto output = machine.update(failed);
	ASSERT_EQ(output.fault, TransformFault::ActuatorCommunication);
	EXPECT_FALSE(output.servo_enabled);
	EXPECT_TRUE(std::isnan(output.servo_value));
	EXPECT_TRUE(output.release_requested);
	EXPECT_EQ(output.target, HybridTarget::None);

	failed.actuator.online = true;
	failed.actuator.healthy = false;
	output = machine.update(failed);
	EXPECT_EQ(output.fault, TransformFault::ActuatorCommunication);
}

TEST(TransformationStateMachine, FaultClearRequiresDisarmedAndReturnsUnknown)
{
	TransformationStateMachine machine;
	auto cfg = config();
	cfg.sensor_timeout_s = 5.f;
	machine.initialize(cfg, input());
	machine.request(HybridTarget::Driving, 0);
	auto failed = input();
	failed.actuator.command_accepted = false;
	ASSERT_EQ(machine.update(failed).fault, TransformFault::ActuatorCommandRejected);

	EXPECT_EQ(machine.clearFault(false).fault, TransformFault::ActuatorCommandRejected);
	const auto cleared = machine.clearFault(true);
	EXPECT_EQ(cleared.state, HybridState::Unknown);
	EXPECT_EQ(cleared.target, HybridTarget::None);
	EXPECT_EQ(cleared.fault, TransformFault::None);
	EXPECT_FALSE(cleared.release_requested);
}

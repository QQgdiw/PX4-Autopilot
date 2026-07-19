#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "Hx8Controller.hpp"

using namespace hx8;

namespace
{

constexpr uint8_t ServoId = 7;

ProtectionConfig validConfig()
{
	ProtectionConfig config {};
	config.stall_power_mw = 3000;
	config.temperature_adc = 741;
	config.power_limit_mw = 4500;
	config.current_limit_ma = 2200;
	return config;
}

MotionCommand motion(uint32_t sequence, uint64_t timestamp_us = 0, float angle_deg = 42.f)
{
	MotionCommand command {};
	command.timestamp_us = timestamp_us;
	command.sequence = sequence;
	command.type = 0;
	command.servo_id = ServoId;
	command.target_angle_deg = angle_deg;
	command.move_time_ms = 600;
	command.acceleration_time_ms = 100;
	command.deceleration_time_ms = 120;
	command.power_mw = 2500;
	return command;
}

ControllerInput input(uint64_t now_us, bool armed = false, bool prearmed = false)
{
	ControllerInput value {};
	value.now_us = now_us;
	value.armed = armed;
	value.prearmed = prearmed;
	return value;
}

uint16_t read16(const uint8_t *bytes)
{
	return static_cast<uint16_t>(bytes[0]) | (static_cast<uint16_t>(bytes[1]) << 8);
}

uint16_t expectedValue(uint8_t parameter, const ProtectionConfig &config)
{
	switch (parameter) {
	case 33: return config.response_enabled;

	case 34: return ServoId;

	case 36: return 5;

	case 37: return config.stall_release_enabled;

	case 38: return config.stall_power_mw;

	case 39: return config.voltage_min_mv;

	case 40: return config.voltage_max_mv;

	case 41: return config.temperature_adc;

	case 42: return config.power_limit_mw;

	case 43: return config.current_limit_ma;

	case 46: return config.power_on_lock;

	default: return 0;
	}
}

Frame response(CommandId command, uint8_t servo_id, uint16_t value = 0, uint8_t length = 0)
{
	Frame frame {};
	frame.command = command;
	frame.servo_id = servo_id;
	frame.payload_length = length;
	frame.payload[0] = static_cast<uint8_t>(value);
	frame.payload[1] = static_cast<uint8_t>(value >> 8);
	return frame;
}

uint64_t finishBoot(Controller &controller, const ProtectionConfig &config = validConfig())
{
	controller.setExpectedConfig(config);
	// Select the configured servo ID without leaving a runnable target after boot.
	controller.setTarget(motion(1, UINT64_MAX));
	uint64_t now = 0;
	PendingRequest request = controller.update(input(now));
	EXPECT_TRUE(request.valid);
	EXPECT_EQ(request.command, CommandId::Ping);
	EXPECT_EQ(request.payload_length, 0);
	controller.acceptResponse(response(CommandId::Ping, ServoId), now);

	for (unsigned i = 0; i < 11; ++i) {
		now += Controller::MinimumCommandSpacingUs;
		request = controller.update(input(now));
		EXPECT_TRUE(request.valid);
		EXPECT_EQ(request.command, CommandId::ParamRead);
		EXPECT_EQ(request.payload_length, 1);

		if (!request.valid || request.payload_length != 1) {
			return now;
		}

		const uint8_t parameter = request.payload[0];
		const uint16_t value = expectedValue(parameter, config);
		controller.acceptResponse(response(CommandId::ParamRead, ServoId, value, value > UINT8_MAX ? 2 : 1), now);
	}

	EXPECT_TRUE(controller.status().online);
	EXPECT_TRUE(controller.status().config_verified);
	return now;
}

} // namespace

TEST(Hx8Controller, AllowsOnlyOneOutstandingRequestAndEnforcesSpacing)
{
	Controller controller;
	controller.setExpectedConfig(validConfig());
	controller.setTarget(motion(1));
	PendingRequest request = controller.update(input(0));
	ASSERT_TRUE(request.valid);
	EXPECT_EQ(request.command, CommandId::Ping);
	EXPECT_FALSE(controller.update(input(100000)).valid);
	controller.acceptResponse(response(CommandId::Ping, ServoId), 1000);
	EXPECT_FALSE(controller.update(input(Controller::MinimumCommandSpacingUs - 1)).valid);
	EXPECT_TRUE(controller.update(input(Controller::MinimumCommandSpacingUs)).valid);
}

TEST(Hx8Controller, ConfiguredServoIdIsUsedForBootBeforeAnyTarget)
{
	Controller controller;
	controller.setExpectedConfig(validConfig());
	controller.setServoId(ServoId);
	const PendingRequest request = controller.update(input(0));
	ASSERT_TRUE(request.valid);
	EXPECT_EQ(request.command, CommandId::Ping);
	EXPECT_EQ(controller.status().servo_id, ServoId);
}

TEST(Hx8Controller, RetriesTwiceThenMarksOfflineAndStillReleases)
{
	Controller controller;
	controller.setExpectedConfig(validConfig());
	controller.setTarget(motion(1));
	ASSERT_TRUE(controller.update(input(0)).valid);

	for (unsigned retry = 0; retry < Controller::MaxRetries; ++retry) {
		const uint64_t now = (retry + 1) * Controller::ResponseTimeoutUs;
		controller.notifyTimeout(now);
		PendingRequest request = controller.update(input(now));
		ASSERT_TRUE(request.valid);
		EXPECT_EQ(request.command, CommandId::Ping);
	}

	controller.notifyTimeout((Controller::MaxRetries + 1) * Controller::ResponseTimeoutUs);
	EXPECT_FALSE(controller.status().online);
	EXPECT_EQ(controller.status().timeout_count, 1u);
	EXPECT_EQ(controller.status().retry_count, Controller::MaxRetries);
	controller.requestRelease(50);
	PendingRequest release = controller.update(input(100000));
	ASSERT_TRUE(release.valid);
	EXPECT_EQ(release.priority, RequestPriority::EmergencyRelease);
	EXPECT_EQ(release.command, CommandId::Stop);
	ASSERT_EQ(release.payload_length, 3);
	EXPECT_EQ(release.payload[0], 0x10);
	EXPECT_EQ(read16(&release.payload[1]), 0);
}

TEST(Hx8Controller, ReleasePreemptsTargetAndTargetUsesTimedMovePayload)
{
	Controller controller;
	uint64_t now = finishBoot(controller);
	controller.setTarget(motion(2, now, -12.3f));
	controller.requestRelease(3);
	now += Controller::MinimumCommandSpacingUs;
	PendingRequest request = controller.update(input(now, true));
	ASSERT_TRUE(request.valid);
	EXPECT_EQ(request.priority, RequestPriority::EmergencyRelease);
	controller.acceptResponse(response(CommandId::Stop, ServoId, 0, 1), now);

	now += Controller::MinimumCommandSpacingUs;
	request = controller.update(input(now, true));
	ASSERT_TRUE(request.valid);
	EXPECT_EQ(request.priority, RequestPriority::Target);
	EXPECT_EQ(request.command, CommandId::TimedMove);
	ASSERT_EQ(request.payload_length, 10);
	EXPECT_EQ(static_cast<int16_t>(read16(request.payload)), -123);
	EXPECT_EQ(read16(&request.payload[2]), 600);
	EXPECT_EQ(read16(&request.payload[4]), 100);
	EXPECT_EQ(read16(&request.payload[6]), 120);
	EXPECT_EQ(read16(&request.payload[8]), 2500);
}

TEST(Hx8Controller, RejectsRepeatedOutOfOrderAndExpiredCommands)
{
	Controller controller;
	uint64_t now = finishBoot(controller);
	controller.setTarget(motion(10, now));
	now += Controller::MinimumCommandSpacingUs;
	PendingRequest request = controller.update(input(now, true));
	ASSERT_TRUE(request.valid);
	controller.acceptResponse(response(CommandId::TimedMove, ServoId, 0, 1), now);
	EXPECT_TRUE(controller.status().command_accepted);

	controller.setTarget(motion(10, now, 30.f));
	now += Controller::MinimumCommandSpacingUs;
	EXPECT_NE(controller.update(input(now, true)).priority, RequestPriority::Target);
	controller.setTarget(motion(9, now, 30.f));
	now += Controller::MinimumCommandSpacingUs;
	EXPECT_NE(controller.update(input(now, true)).priority, RequestPriority::Target);
	controller.setTarget(motion(11, now - Controller::CommandExpiryUs - 1, 30.f));
	now += Controller::MinimumCommandSpacingUs;
	EXPECT_NE(controller.update(input(now, true)).priority, RequestPriority::Target);
	EXPECT_FALSE(controller.status().command_accepted);
}

TEST(Hx8Controller, GatesMotionOnArmingSafetyAndVerifiedCalibration)
{
	Controller controller;
	uint64_t now = finishBoot(controller);
	controller.setTarget(motion(2, now));
	now += Controller::MinimumCommandSpacingUs;
	EXPECT_NE(controller.update(input(now)).priority, RequestPriority::Target);

	ControllerInput unsafe = input(now += Controller::MinimumCommandSpacingUs, true);
	unsafe.lockdown = true;
	EXPECT_NE(controller.update(unsafe).priority, RequestPriority::Target);
	unsafe = input(now += Controller::MinimumCommandSpacingUs, true);
	unsafe.failsafe = true;
	EXPECT_NE(controller.update(unsafe).priority, RequestPriority::Target);

	PendingRequest target = controller.update(input(now += Controller::MinimumCommandSpacingUs, false, true));
	ASSERT_TRUE(target.valid);
	EXPECT_EQ(target.priority, RequestPriority::Target);

	ProtectionConfig invalid = validConfig();
	invalid.current_limit_ma = 0;
	controller.setExpectedConfig(invalid);
	controller.setTarget(motion(3, now));
	controller.acceptResponse(response(CommandId::TimedMove, ServoId, 0, 1), now);
	EXPECT_FALSE(controller.status().config_verified);
	EXPECT_NE(controller.update(input(now += Controller::MinimumCommandSpacingUs, true)).priority, RequestPriority::Target);
}

TEST(Hx8Controller, EveryZeroCalibrationSentinelProhibitsMotion)
{
	for (unsigned field = 0; field < 4; ++field) {
		Controller controller;
		ProtectionConfig config = validConfig();

		if (field == 0) { config.stall_power_mw = 0; }

		if (field == 1) { config.temperature_adc = 0; }

		if (field == 2) { config.power_limit_mw = 0; }

		if (field == 3) { config.current_limit_ma = 0; }

		controller.setExpectedConfig(config);
		controller.setTarget(motion(1));
		EXPECT_FALSE(controller.status().config_verified);
		PendingRequest request = controller.update(input(0, true));
		EXPECT_NE(request.priority, RequestPriority::Target);
	}
}

TEST(Hx8Controller, BootPingsAndReadsCompleteConfigurationWithoutWriting)
{
	Controller controller;
	const ProtectionConfig config = validConfig();
	controller.setExpectedConfig(config);
	controller.setTarget(motion(1));
	uint64_t now = 0;
	bool seen[47] {};
	PendingRequest request = controller.update(input(now));
	ASSERT_EQ(request.command, CommandId::Ping);
	controller.acceptResponse(response(CommandId::Ping, ServoId), now);

	for (unsigned i = 0; i < 11; ++i) {
		now += Controller::MinimumCommandSpacingUs;
		request = controller.update(input(now));
		ASSERT_TRUE(request.valid);
		EXPECT_NE(request.command, CommandId::ParamWrite);
		ASSERT_EQ(request.command, CommandId::ParamRead);
		seen[request.payload[0]] = true;
		const uint16_t value = expectedValue(request.payload[0], config);
		controller.acceptResponse(response(CommandId::ParamRead, ServoId, value, value > UINT8_MAX ? 2 : 1), now);
	}

	for (uint8_t parameter : {33, 34, 36, 37, 38, 39, 40, 41, 42, 43, 46}) {
		EXPECT_TRUE(seen[parameter]);
	}
	EXPECT_TRUE(controller.status().config_verified);
}

TEST(Hx8Controller, ConfigurationMismatchPreventsVerification)
{
	Controller controller;
	const ProtectionConfig config = validConfig();
	controller.setExpectedConfig(config);
	controller.setTarget(motion(1));
	uint64_t now = 0;
	PendingRequest request = controller.update(input(now));
	controller.acceptResponse(response(CommandId::Ping, ServoId), now);

	for (unsigned i = 0; i < 11; ++i) {
		now += Controller::MinimumCommandSpacingUs;
		request = controller.update(input(now));
		const uint8_t parameter = request.payload[0];
		uint16_t value = expectedValue(parameter, config);

		if (parameter == 42) { ++value; }

		controller.acceptResponse(response(CommandId::ParamRead, ServoId, value, value > UINT8_MAX ? 2 : 1), now);
	}

	EXPECT_FALSE(controller.status().config_verified);
	controller.setTarget(motion(2, now));
	EXPECT_NE(controller.update(input(now += Controller::MinimumCommandSpacingUs, true)).priority,
		  RequestPriority::Target);
}

TEST(Hx8Controller, PersistentWritesRequireExplicitFullyDisarmedCommissioning)
{
	Controller controller;
	uint64_t now = finishBoot(controller);
	controller.requestPersistentWrite();
	ControllerInput gate = input(now += Controller::MinimumCommandSpacingUs, true);
	gate.explicit_commissioning = true;
	EXPECT_NE(controller.update(gate).command, CommandId::ParamWrite);
	gate = input(now += Controller::MinimumCommandSpacingUs, false, true);
	gate.explicit_commissioning = true;
	EXPECT_NE(controller.update(gate).command, CommandId::ParamWrite);
	gate = input(now += Controller::MinimumCommandSpacingUs);
	EXPECT_NE(controller.update(gate).command, CommandId::ParamWrite);
	gate.now_us = now += Controller::MinimumCommandSpacingUs;
	gate.explicit_commissioning = true;
	PendingRequest write = controller.update(gate);
	ASSERT_TRUE(write.valid);
	EXPECT_EQ(write.command, CommandId::ParamWrite);
	EXPECT_TRUE(controller.status().persistent_write_active);
}

TEST(Hx8Controller, PersistentWriteUsesPerItemReadbackAndCompletes)
{
	Controller controller;
	const ProtectionConfig config = validConfig();
	uint64_t now = finishBoot(controller, config);
	controller.requestPersistentWrite();
	ControllerInput commissioning = input(now);
	commissioning.explicit_commissioning = true;
	unsigned writes = 0;
	unsigned reads = 0;

	while (writes < 9) {
		commissioning.now_us = now += Controller::MinimumCommandSpacingUs;
		PendingRequest request = controller.update(commissioning);
		ASSERT_TRUE(request.valid);
		ASSERT_EQ(request.command, CommandId::ParamWrite);
		const uint8_t parameter = request.payload[0];
		EXPECT_EQ(read16(&request.payload[1]), expectedValue(parameter, config));
		controller.acceptResponse(response(CommandId::ParamWrite, ServoId, 0, 1), now);
		++writes;

		commissioning.now_us = now += Controller::MinimumCommandSpacingUs;
		request = controller.update(commissioning);
		ASSERT_TRUE(request.valid);
		ASSERT_EQ(request.command, CommandId::ParamRead);
		EXPECT_EQ(request.payload[0], parameter);
		const uint16_t value = expectedValue(parameter, config);
		controller.acceptResponse(response(CommandId::ParamRead, ServoId, value, value > UINT8_MAX ? 2 : 1), now);
		++reads;
	}

	EXPECT_EQ(reads, 9u);
	EXPECT_FALSE(controller.status().persistent_write_active);
	EXPECT_TRUE(controller.status().config_verified);
	EXPECT_EQ(controller.status().persistent_write_result, OperationResult::Accepted);
}

TEST(Hx8Controller, PersistentReadbackMismatchAbortsAndMarksUnverified)
{
	Controller controller;
	uint64_t now = finishBoot(controller);
	controller.requestPersistentWrite();
	ControllerInput commissioning = input(now);
	commissioning.explicit_commissioning = true;
	commissioning.now_us = now += Controller::MinimumCommandSpacingUs;
	PendingRequest request = controller.update(commissioning);
	ASSERT_EQ(request.command, CommandId::ParamWrite);
	controller.acceptResponse(response(CommandId::ParamWrite, ServoId, 0, 1), now);
	commissioning.now_us = now += Controller::MinimumCommandSpacingUs;
	request = controller.update(commissioning);
	ASSERT_EQ(request.command, CommandId::ParamRead);
	controller.acceptResponse(response(CommandId::ParamRead, ServoId, 99, 1), now);
	EXPECT_FALSE(controller.status().persistent_write_active);
	EXPECT_FALSE(controller.status().config_verified);
}

TEST(Hx8Controller, MonitorsAtMovingAndStableCadencesAndDecodesStatus)
{
	Controller controller;
	uint64_t now = finishBoot(controller);
	PendingRequest request = controller.update(input(now += Controller::StableMonitorIntervalUs - 1));
	EXPECT_FALSE(request.valid);
	request = controller.update(input(++now));
	ASSERT_TRUE(request.valid);
	EXPECT_EQ(request.priority, RequestPriority::Status);
	Frame status = response(CommandId::Status, ServoId);
	status.payload_length = 15;
	const uint8_t payload[] {0x28, 0x23, 0xd0, 0x07, 0x94, 0x11, 0xe5, 0x02, 0x01,
				 0x7b, 0x00, 0x00, 0x00, 0x00, 0x00
				};

	for (unsigned i = 0; i < sizeof(payload); ++i) { status.payload[i] = payload[i]; }

	controller.acceptResponse(status, now);
	EXPECT_FLOAT_EQ(controller.status().voltage_v, 9.f);
	EXPECT_FLOAT_EQ(controller.status().current_a, 2.f);
	EXPECT_FLOAT_EQ(controller.status().power_w, 4.5f);
	EXPECT_FLOAT_EQ(controller.status().angle_deg, 12.3f);
	EXPECT_EQ(controller.status().status_flags, 1);
	EXPECT_TRUE(controller.status().healthy);

	EXPECT_FALSE(controller.update(input(now + Controller::MovingMonitorIntervalUs - 1)).valid);
	request = controller.update(input(now += Controller::MovingMonitorIntervalUs));
	EXPECT_EQ(request.priority, RequestPriority::Status);
	status.payload[8] = 0x44;
	controller.acceptResponse(status, now);
	EXPECT_FALSE(controller.status().healthy);
	EXPECT_EQ(controller.status().protection_flags, 0x44);
	EXPECT_FALSE(controller.update(input(now + Controller::StableMonitorIntervalUs - 1)).valid);
	EXPECT_EQ(controller.update(input(now + Controller::StableMonitorIntervalUs)).priority, RequestPriority::Status);
}

TEST(Hx8Controller, RejectsUnexpectedAndMalformedResponses)
{
	Controller controller;
	controller.setExpectedConfig(validConfig());
	controller.setTarget(motion(1));
	ASSERT_TRUE(controller.update(input(0)).valid);
	controller.acceptResponse(response(CommandId::Status, ServoId), 1000);
	EXPECT_EQ(controller.status().rx_error_count, 1u);
	EXPECT_EQ(controller.status().rx_valid_count, 0u);
	EXPECT_FALSE(controller.status().online);
}

TEST(Hx8Controller, AbortsOutstandingWriteWhenCommissioningGateIsLost)
{
	Controller controller;
	uint64_t now = finishBoot(controller);
	controller.requestPersistentWrite();
	ControllerInput commissioning = input(now += Controller::MinimumCommandSpacingUs);
	commissioning.explicit_commissioning = true;
	PendingRequest write = controller.update(commissioning);
	ASSERT_TRUE(write.valid);
	ASSERT_EQ(write.command, CommandId::ParamWrite);
	EXPECT_TRUE(controller.status().persistent_write_active);

	ControllerInput unsafe = input(now += Controller::MinimumCommandSpacingUs, true);
	unsafe.explicit_commissioning = true;
	EXPECT_FALSE(controller.update(unsafe).valid);
	EXPECT_FALSE(controller.status().persistent_write_active);
	EXPECT_FALSE(controller.status().config_verified);

	controller.acceptResponse(response(CommandId::ParamWrite, ServoId, 0, 1), now);
	EXPECT_FALSE(controller.status().persistent_write_active);
}

TEST(Hx8Controller, AbortsWriteOnUnexpectedResponseAndIgnoresLaterValidResponse)
{
	auto verifyMismatchAbortsWrite = [](const Frame & mismatch) {
		Controller controller;
		uint64_t now = finishBoot(controller);
		controller.requestPersistentWrite();
		ControllerInput commissioning = input(now += Controller::MinimumCommandSpacingUs);
		commissioning.explicit_commissioning = true;
		PendingRequest write = controller.update(commissioning);
		ASSERT_TRUE(write.valid);
		ASSERT_EQ(write.command, CommandId::ParamWrite);
		ASSERT_TRUE(controller.status().persistent_write_active);

		controller.acceptResponse(mismatch, now);
		EXPECT_FALSE(controller.status().persistent_write_active);
		EXPECT_FALSE(controller.status().config_verified);
		EXPECT_EQ(controller.status().rx_error_count, 1u);

		// Once aborted, a later response must not resurrect the transaction.
		controller.acceptResponse(response(CommandId::ParamWrite, ServoId, 0, 1), now);
		EXPECT_FALSE(controller.status().persistent_write_active);
		EXPECT_FALSE(controller.status().config_verified);
		EXPECT_EQ(controller.status().rx_error_count, 2u);
	};

	{
		SCOPED_TRACE("wrong command");
		verifyMismatchAbortsWrite(response(CommandId::Status, ServoId, 0, 15));
	}
	{
		SCOPED_TRACE("wrong servo ID");
		verifyMismatchAbortsWrite(response(CommandId::ParamWrite, ServoId + 1, 0, 1));
	}
	{
		SCOPED_TRACE("malformed payload length");
		verifyMismatchAbortsWrite(response(CommandId::ParamWrite, ServoId, 0, 2));
	}
}

TEST(Hx8Controller, ReportsConfigurationCheckOnlyAfterBootReadCompletes)
{
	Controller controller;
	controller.setExpectedConfig(validConfig());
	controller.setServoId(ServoId);
	EXPECT_FALSE(controller.status().config_check_complete);

	finishBoot(controller);
	EXPECT_TRUE(controller.status().config_check_complete);
	EXPECT_TRUE(controller.status().config_verified);
}

TEST(Hx8Controller, ProtocolErrorAbortsPersistentWriteFailClosed)
{
	Controller controller;
	uint64_t now = finishBoot(controller);
	controller.requestPersistentWrite();
	ControllerInput commissioning = input(now += Controller::MinimumCommandSpacingUs);
	commissioning.explicit_commissioning = true;
	ASSERT_EQ(controller.update(commissioning).command, CommandId::ParamWrite);

	controller.notifyProtocolError();

	EXPECT_EQ(controller.status().protocol_error_count, 1u);
	EXPECT_FALSE(controller.status().persistent_write_active);
	EXPECT_FALSE(controller.status().config_verified);
	EXPECT_EQ(controller.status().persistent_write_result, OperationResult::ProtocolError);
}

TEST(Hx8Controller, TransportErrorIsExplicitAndAbortsPersistentWrite)
{
	Controller controller;
	uint64_t now = finishBoot(controller);
	controller.requestPersistentWrite();
	ControllerInput commissioning = input(now += Controller::MinimumCommandSpacingUs);
	commissioning.explicit_commissioning = true;
	ASSERT_EQ(controller.update(commissioning).command, CommandId::ParamWrite);

	controller.notifyTransportError();

	EXPECT_EQ(controller.status().transport_error_count, 1u);
	EXPECT_FALSE(controller.status().online);
	EXPECT_FALSE(controller.status().healthy);
	EXPECT_FALSE(controller.status().persistent_write_active);
	EXPECT_EQ(controller.status().persistent_write_result, OperationResult::ProtocolError);
}

TEST(Hx8Controller, LocalCommandRejectionHasExplicitTerminalResult)
{
	Controller controller;
	controller.rejectCommand(42);

	EXPECT_EQ(controller.status().command_sequence, 42u);
	EXPECT_FALSE(controller.status().command_accepted);
	EXPECT_EQ(controller.status().command_result, static_cast<uint8_t>(OperationResult::Rejected));
}

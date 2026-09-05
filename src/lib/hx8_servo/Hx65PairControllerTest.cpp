#include <gtest/gtest.h>

#include "Hx65PairController.hpp"

using namespace hx65;

namespace
{

StatusFrame response(uint8_t id, const uint8_t *parameters = nullptr, uint8_t length = 0, uint8_t error = 0)
{
	StatusFrame frame {};
	frame.servo_id = id;
	frame.error = error;
	frame.parameter_length = length;

	for (uint8_t i = 0; i < length; ++i) {
		frame.parameters[i] = parameters[i];
	}

	return frame;
}

uint64_t finishBoot(PairController &controller)
{
	uint64_t now = 0;
	const uint8_t identity_left[] {1, 4, 0, 1};
	const uint8_t identity_right[] {2, 4, 0, 1};
	const uint8_t protection[] {44};
	const uint8_t mode[] {0};
	const StatusFrame replies[] {
		response(1, identity_left, 4), response(1, protection, 1), response(1, mode, 1),
		response(2, identity_right, 4), response(2, protection, 1), response(2, mode, 1)
	};

	for (const StatusFrame &reply : replies) {
		const PendingRequest request = controller.update(now);
		EXPECT_TRUE(request.valid);
		EXPECT_NE(request.instruction, Instruction::Ping);
		controller.acceptResponse(reply, now);
		now += PairController::MinimumCommandSpacingUs;
	}

	return now;
}

} // namespace

TEST(Hx65PairController, VerifiesBothServosBeforeHealthy)
{
	PairController controller;
	controller.setConfig({});
	EXPECT_TRUE(controller.bootPending());
	finishBoot(controller);
	EXPECT_FALSE(controller.bootPending());
	EXPECT_TRUE(controller.status().servo[0].online);
	EXPECT_TRUE(controller.status().servo[1].online);
	EXPECT_TRUE(controller.status().servo[0].config_verified);
	EXPECT_TRUE(controller.status().servo[1].config_verified);
	EXPECT_TRUE(controller.status().servo[0].healthy);
	EXPECT_TRUE(controller.status().servo[1].healthy);
}

TEST(Hx65PairController, RejectsDuplicateIdsAndIgnoresReadableBaudCode)
{
	PairController duplicate;
	PairConfig invalid {};
	invalid.right_id = invalid.left_id;
	duplicate.setConfig(invalid);
	EXPECT_FALSE(duplicate.update(0).valid);

	PairController mismatch;
	mismatch.setConfig({});
	PendingRequest request = mismatch.update(0);
	ASSERT_TRUE(request.valid);
	ASSERT_EQ(request.kind, RequestKind::Identity);
	const uint8_t identity_with_other_rate[] {1, 0, 0, 1};
	mismatch.acceptResponse(response(1, identity_with_other_rate, 4), 0);
	request = mismatch.update(PairController::MinimumCommandSpacingUs);
	EXPECT_TRUE(request.valid);
	EXPECT_EQ(request.kind, RequestKind::Protection);
}

TEST(Hx65PairController, BootStartsWithTargetedIdentityReadAndNeverPings)
{
	PairController controller;
	controller.setConfig({});
	const PendingRequest first = controller.update(0);
	ASSERT_TRUE(first.valid);
	EXPECT_EQ(first.instruction, Instruction::Read);
	EXPECT_EQ(first.kind, RequestKind::Identity);
	EXPECT_EQ(first.servo_id, 1);
	ASSERT_EQ(first.parameter_length, 2);
	EXPECT_EQ(first.parameters[0], 0x05);
	EXPECT_EQ(first.parameters[1], 4);

	PairController complete_boot;
	complete_boot.setConfig({});
	finishBoot(complete_boot);
}

TEST(Hx65PairController, StagesBothTargetsBeforeBroadcastAction)
{
	PairController controller;
	controller.setConfig({});
	uint64_t now = finishBoot(controller);
	PairCommand command {};
	command.timestamp_us = now;
	command.sequence = 9;
	command.left_target_steps = -4096;
	command.right_target_steps = 4096;
	command.speed_steps_s = 1000;
	command.acceleration = 10;
	controller.setTarget(command);

	PendingRequest request = controller.update(now);
	ASSERT_TRUE(request.valid);
	EXPECT_EQ(request.instruction, Instruction::RegWrite);
	EXPECT_EQ(request.servo_id, 1);
	ASSERT_EQ(request.parameter_length, 9);
	EXPECT_EQ(request.parameters[0], 0x28);
	EXPECT_EQ(request.parameters[1], 1);
	EXPECT_EQ(request.parameters[2], 10);
	EXPECT_EQ(request.parameters[3], 0x00);
	EXPECT_EQ(request.parameters[4], 0x90);
	controller.acceptResponse(response(1), now);

	now += PairController::MinimumCommandSpacingUs;
	request = controller.update(now);
	ASSERT_TRUE(request.valid);
	EXPECT_EQ(request.instruction, Instruction::RegWrite);
	EXPECT_EQ(request.servo_id, 2);
	EXPECT_EQ(request.parameters[3], 0x00);
	EXPECT_EQ(request.parameters[4], 0x10);
	controller.acceptResponse(response(2), now);

	now += PairController::MinimumCommandSpacingUs;
	request = controller.update(now);
	ASSERT_TRUE(request.valid);
	EXPECT_EQ(request.instruction, Instruction::Action);
	EXPECT_EQ(request.servo_id, BroadcastId);
	EXPECT_FALSE(request.expects_response);
	controller.completeNoResponse(true, now);
	EXPECT_EQ(controller.status().command_sequence, 9u);
	EXPECT_EQ(controller.status().command_result, CommandResult::Accepted);
}

TEST(Hx65PairController, DecodesContiguousRuntimeMonitor)
{
	PairController controller;
	controller.setConfig({});
	uint64_t now = finishBoot(controller);
	PendingRequest request = controller.update(now);
	ASSERT_EQ(request.kind, RequestKind::Monitor);
	ASSERT_EQ(request.servo_id, 1);
	uint8_t monitor[15] {};
	monitor[0] = 0xd0;
	monitor[1] = 0x87;
	monitor[2] = 0x64;
	monitor[4] = 0x32;
	monitor[6] = 111;
	monitor[7] = 42;
	monitor[10] = 1;
	monitor[13] = 0x34;
	monitor[14] = 0x12;
	controller.acceptResponse(response(1, monitor, sizeof(monitor)), now);
	const ServoStatus &status = controller.status().servo[0];
	EXPECT_EQ(status.position_steps, -2000);
	EXPECT_EQ(status.speed_steps_s, 100);
	EXPECT_EQ(status.load, 50);
	EXPECT_FLOAT_EQ(status.voltage_v, 11.1f);
	EXPECT_FLOAT_EQ(status.temperature_c, 42.f);
	EXPECT_EQ(status.current_ma, 0x1234);
	EXPECT_TRUE(status.moving);
	EXPECT_TRUE(status.position_valid);
	EXPECT_TRUE(status.healthy);
}

TEST(Hx65PairController, BootRetryIsScheduledAfterTimeout)
{
	PairController controller;
	controller.setConfig({});
	PendingRequest request = controller.update(0);
	ASSERT_TRUE(request.valid);
	controller.notifyTimeout(PairController::ResponseTimeoutUs);
	request = controller.update(PairController::ResponseTimeoutUs + PairController::MinimumCommandSpacingUs);
	ASSERT_TRUE(request.valid);
	EXPECT_EQ(request.kind, RequestKind::Identity);
	EXPECT_EQ(controller.status().retry_count, 1u);
	EXPECT_EQ(controller.status().servo[0].timeout_count, 1u);
	EXPECT_EQ(controller.status().servo[0].retry_count, 1u);
	EXPECT_EQ(controller.status().servo[1].timeout_count, 0u);
}

TEST(Hx65PairController, RxProtocolErrorDoesNotDiscardOutstandingResponse)
{
	PairController controller;
	controller.setConfig({});
	PendingRequest request = controller.update(0);
	ASSERT_TRUE(request.valid);
	ASSERT_EQ(request.kind, RequestKind::Identity);

	controller.notifyRxProtocolError();
	EXPECT_TRUE(controller.hasOutstandingRequest());
	EXPECT_FALSE(controller.update(PairController::MinimumCommandSpacingUs).valid);
	EXPECT_EQ(controller.status().protocol_error_count, 1u);

	const uint8_t identity_left[] {1, 4, 0, 1};
	controller.acceptResponse(response(1, identity_left, 4), PairController::MinimumCommandSpacingUs);
	request = controller.update(2 * PairController::MinimumCommandSpacingUs);
	EXPECT_TRUE(request.valid);
	EXPECT_EQ(request.kind, RequestKind::Protection);
}

TEST(Hx65PairController, MonitorRetryKeepsLastVerifiedOnlineUntilExhausted)
{
	PairController controller;
	controller.setConfig({});
	uint64_t now = finishBoot(controller);
	PendingRequest request = controller.update(now);
	ASSERT_TRUE(request.valid);
	ASSERT_EQ(request.kind, RequestKind::Monitor);
	const unsigned side = request.side == Side::Left ? 0u : 1u;

	for (uint8_t retry = 0; retry < PairController::MaxRetries; ++retry) {
		now += PairController::ResponseTimeoutUs;
		controller.notifyTimeout(now);
		EXPECT_TRUE(controller.status().servo[side].online);
		EXPECT_TRUE(controller.status().servo[side].healthy);
		now += PairController::MinimumCommandSpacingUs;
		request = controller.update(now);
		ASSERT_TRUE(request.valid);
		EXPECT_EQ(request.kind, RequestKind::Monitor);
	}

	now += PairController::ResponseTimeoutUs;
	controller.notifyTimeout(now);
	EXPECT_FALSE(controller.status().servo[side].online);
	EXPECT_FALSE(controller.status().servo[side].healthy);
	EXPECT_EQ(controller.status().timeout_count, 3u);
	EXPECT_EQ(controller.status().retry_count, 2u);
	EXPECT_EQ(controller.status().servo[side].timeout_count, 3u);
	EXPECT_EQ(controller.status().servo[side].retry_count, 2u);
	EXPECT_EQ(controller.status().servo[side == 0u ? 1u : 0u].timeout_count, 0u);
}

TEST(Hx65PairController, EmergencyReleasePreemptsBootVerification)
{
	PairController controller;
	controller.setConfig({});
	controller.requestRelease(7);
	PendingRequest request = controller.update(0);
	ASSERT_TRUE(request.valid);
	EXPECT_EQ(request.kind, RequestKind::Release);
	EXPECT_EQ(request.servo_id, BroadcastId);
	EXPECT_FALSE(request.expects_response);
	controller.completeNoResponse(true, 0);
	request = controller.update(PairController::MinimumCommandSpacingUs);
	EXPECT_EQ(request.kind, RequestKind::Identity);
}

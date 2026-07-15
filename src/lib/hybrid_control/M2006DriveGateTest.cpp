#include <gtest/gtest.h>

#include "M2006DriveGate.hpp"

using namespace hybrid_control;

TEST(M2006DriveGate, RequiresBothFeedbackStreams)
{
	M2006DriveGate gate;
	DriveGateInput input{false, true, false, true, true, {true, true}, false, 0};
	EXPECT_FALSE(gate.update(input));
	input.now_us = 100000;
	EXPECT_FALSE(gate.update(input));
	input.armed = true;
	EXPECT_TRUE(gate.update(input));
}

TEST(M2006DriveGate, FeedbackQualificationIsIndependentOfCommandHealth)
{
	M2006DriveGate gate;
	DriveGateInput input{false, true, false, false, true, {true, true}, false, 0};
	EXPECT_FALSE(gate.update(input));
	input.now_us = 100000;
	EXPECT_FALSE(gate.update(input));
	input.command_fresh = true;
	input.armed = true;
	EXPECT_TRUE(gate.update(input));
}

TEST(M2006DriveGate, LatchesFeedbackLossUntilDisarmedHealthy)
{
	M2006DriveGate gate;
	DriveGateInput input{false, true, false, true, true, {true, true}, false, 0};
	gate.update(input);
	input.now_us = 100000;
	gate.update(input);
	input.armed = true;
	ASSERT_TRUE(gate.update(input));
	input.feedback_healthy[1] = false;
	EXPECT_FALSE(gate.update(input));
	input.feedback_healthy[1] = true;
	EXPECT_FALSE(gate.update(input));
	input.armed = false;
	input.now_us = 200000;
	EXPECT_FALSE(gate.update(input));
	input.now_us = 300000;
	EXPECT_FALSE(gate.update(input));
	input.armed = true;
	EXPECT_TRUE(gate.update(input));
}

TEST(M2006DriveGate, ReportsAndLatchesCanAndCommandFaults)
{
	M2006DriveGate gate;
	DriveGateInput input{false, true, false, true, true, {true, true}, false, 0};
	gate.update(input);
	input.now_us = 100000;
	gate.update(input);
	input.armed = true;
	ASSERT_TRUE(gate.update(input));

	input.can_error = true;
	input.command_finite = false;
	EXPECT_FALSE(gate.update(input));
	EXPECT_EQ(gate.faultBits(), DriveFaultCan | DriveFaultCommand);

	input.can_error = false;
	input.command_finite = true;
	EXPECT_FALSE(gate.update(input));
}

TEST(M2006DriveGate, LeavingDrivingOnlyDisablesOutput)
{
	M2006DriveGate gate;
	DriveGateInput input{false, true, false, true, true, {true, true}, false, 0};
	gate.update(input);
	input.now_us = 100000;
	gate.update(input);
	input.armed = true;
	ASSERT_TRUE(gate.update(input));

	input.driving = false;
	EXPECT_FALSE(gate.update(input));
	EXPECT_EQ(gate.faultBits(), DriveFaultNone);
	input.driving = true;
	EXPECT_TRUE(gate.update(input));
}

TEST(M2006DriveGate, ArmedNonDrivingBadInputsDoNotLatchFaults)
{
	M2006DriveGate gate;
	DriveGateInput input{true, false, false, false, false, {false, false}, true, 0};
	EXPECT_FALSE(gate.update(input));
	EXPECT_EQ(gate.faultBits(), DriveFaultNone);

	input.driving = true;
	input.command_fresh = true;
	input.command_finite = true;
	input.feedback_healthy[0] = true;
	input.feedback_healthy[1] = true;
	input.can_error = false;
	EXPECT_FALSE(gate.update(input));
	input.now_us = 100000;
	EXPECT_TRUE(gate.update(input));
	EXPECT_EQ(gate.faultBits(), DriveFaultNone);
}

TEST(M2006DriveGate, OutputInhibitionDoesNotDisableFaultMonitoring)
{
	M2006DriveGate gate;
	DriveGateInput input{false, true, true, true, true, {true, true}, false, 0};
	gate.update(input);
	input.now_us = 100000;
	gate.update(input);
	input.armed = true;
	ASSERT_FALSE(gate.update(input));

	input.can_error = true;
	EXPECT_FALSE(gate.update(input));
	EXPECT_EQ(gate.faultBits(), DriveFaultCan);
}

TEST(M2006DriveGate, CommandTimestampFreshnessRejectsStaleAndFutureSamples)
{
	EXPECT_TRUE(commandTimestampFresh(900000, 1000000, 100000));
	EXPECT_FALSE(commandTimestampFresh(899999, 1000000, 100000));
	EXPECT_FALSE(commandTimestampFresh(1000001, 1000000, 100000));
	EXPECT_FALSE(commandTimestampFresh(0, 1000000, 100000));
}

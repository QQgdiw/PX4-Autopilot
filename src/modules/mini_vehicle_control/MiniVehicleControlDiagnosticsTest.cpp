/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "mini_vehicle_control_diagnostics.hpp"

#include <gtest/gtest.h>

namespace
{
constexpr uint64_t Timeout = 200000;

bool originalFreshnessCheck(uint64_t now, uint64_t source_timestamp, uint64_t mode_changed_at)
{
	return source_timestamp > mode_changed_at
	       && now >= source_timestamp
	       && now - source_timestamp <= Timeout;
}
}

TEST(MiniVehicleControlDiagnostics, ClassifiesFreshnessFailures)
{
	using mini_vehicle_control::OutputIssueReason;
	using mini_vehicle_control::classifySourceFreshness;

	EXPECT_EQ(classifySourceFreshness(1000000, 0, 0, Timeout), OutputIssueReason::NoSource);
	EXPECT_EQ(classifySourceFreshness(1000000, 900000, 900000, Timeout), OutputIssueReason::BeforeModeEpoch);
	EXPECT_EQ(classifySourceFreshness(1000000, 1100000, 0, Timeout), OutputIssueReason::FutureTimestamp);
	EXPECT_EQ(classifySourceFreshness(1000000, 799999, 0, Timeout), OutputIssueReason::StaleSource);
	EXPECT_EQ(classifySourceFreshness(1000000, 800000, 0, Timeout), OutputIssueReason::None);
}

TEST(MiniVehicleControlDiagnostics, PreservesOriginalFreshnessDecision)
{
	using mini_vehicle_control::OutputIssueReason;
	using mini_vehicle_control::classifySourceFreshness;

	struct TestCase {
		uint64_t now;
		uint64_t source_timestamp;
		uint64_t mode_changed_at;
	};

	constexpr TestCase cases[] {
		{1000000, 0, 0},
		{1000000, 900000, 900000},
		{1000000, 1100000, 0},
		{1000000, 799999, 0},
		{1000000, 800000, 0},
		{1000000, 999999, 900000},
	};

	for (const TestCase &test_case : cases) {
		const bool classified_fresh = classifySourceFreshness(test_case.now, test_case.source_timestamp,
					      test_case.mode_changed_at, Timeout) == OutputIssueReason::None;
		EXPECT_EQ(classified_fresh,
			  originalFreshnessCheck(test_case.now, test_case.source_timestamp, test_case.mode_changed_at));
	}
}

TEST(MiniVehicleControlDiagnostics, PostSubscriptionTimestampAvoidsSchedulingRace)
{
	using mini_vehicle_control::OutputIssueReason;
	using mini_vehicle_control::classifySourceFreshness;

	constexpr uint64_t run_timestamp = 1000000;
	constexpr uint64_t source_timestamp = 1000100;
	constexpr uint64_t post_subscription_timestamp = 1000101;

	EXPECT_EQ(classifySourceFreshness(run_timestamp, source_timestamp, 0, Timeout),
		  OutputIssueReason::FutureTimestamp);
	EXPECT_EQ(classifySourceFreshness(post_subscription_timestamp, source_timestamp, 0, Timeout),
		  OutputIssueReason::None);
}

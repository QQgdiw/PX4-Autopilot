#include <gtest/gtest.h>
#include <uORB/topics/vehicle_status.h>

#include "HybridTransitionPolicy.hpp"

using namespace hybrid_control;

constexpr uint8_t NAVIGATION_STATE_MANUAL = vehicle_status_s::NAVIGATION_STATE_MANUAL;
constexpr uint8_t NAVIGATION_STATE_ALTCTL = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
constexpr uint8_t NAVIGATION_STATE_OFFBOARD = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;

TEST(HybridTransitionPolicy, DeniesStaleOrUnlandedRequests)
{
	EXPECT_EQ(decideTransition({false, true, false, HybridState::Flying, HybridTarget::Driving, HybridTarget::None}).reject_reason,
		  RejectReason::LandDetectorStale);
	EXPECT_EQ(decideTransition({true, false, false, HybridState::Flying, HybridTarget::Driving, HybridTarget::None}).reject_reason,
		  RejectReason::NotLanded);
}

TEST(HybridTransitionPolicy, AcceptsOnlyValidStableTarget)
{
	EXPECT_TRUE(decideTransition({true, true, false, HybridState::Flying, HybridTarget::Driving}).start);
	EXPECT_EQ(decideTransition({true, true, false, HybridState::Flying, HybridTarget::Flying}).ack_result,
		  CommandResult::Accepted);
}

TEST(HybridTransitionPolicy, OppositeTransitionIsTemporarilyRejected)
{
	const auto result = decideTransition({true, true, false, HybridState::TransitionToRover,
					      HybridTarget::Flying, HybridTarget::Driving});
	EXPECT_FALSE(result.start);
	EXPECT_EQ(result.ack_result, CommandResult::TemporarilyRejected);
	EXPECT_EQ(result.reject_reason, RejectReason::OppositeTransition);
}

TEST(HybridTransitionPolicy, RejectsModesOutsideStableShape)
{
	EXPECT_FALSE(modeAllowedForShape(HybridState::Driving, NAVIGATION_STATE_ALTCTL));
	EXPECT_TRUE(modeAllowedForShape(HybridState::Driving, NAVIGATION_STATE_OFFBOARD));
	EXPECT_FALSE(modeAllowedForShape(HybridState::TransitionToRover, NAVIGATION_STATE_MANUAL));
}

TEST(HybridTransitionPolicy, RequiresInputAfterCompletionEpoch)
{
	EXPECT_FALSE(offboardInputFreshAfter(99, 100, 150));
	EXPECT_TRUE(offboardInputFreshAfter(101, 100, 150));
}

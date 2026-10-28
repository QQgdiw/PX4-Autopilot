#include "HybridIdleBeacon.hpp"
#include <gtest/gtest.h>

using namespace hybrid_control;

TEST(HybridIdleBeacon, DelayAndRepeat)
{
	HybridIdleBeacon beacon;
	EXPECT_FALSE(beacon.update(1, true, 100, 30));
	EXPECT_FALSE(beacon.update(100, true, 100, 30));
	EXPECT_TRUE(beacon.update(101, true, 100, 30));
	EXPECT_FALSE(beacon.update(130, true, 100, 30));
	EXPECT_TRUE(beacon.update(131, true, 100, 30));
}

TEST(HybridIdleBeacon, IneligibleResetsRatherThanPauses)
{
	HybridIdleBeacon beacon;
	beacon.update(1, true, 100, 30);
	EXPECT_FALSE(beacon.update(100, false, 100, 30));
	EXPECT_FALSE(beacon.update(200, true, 100, 30));
	EXPECT_FALSE(beacon.update(299, true, 100, 30));
	EXPECT_TRUE(beacon.update(300, true, 100, 30));
}

TEST(HybridIdleBeacon, DisabledInvalidTimeAndParametersReset)
{
	HybridIdleBeacon beacon;
	EXPECT_FALSE(beacon.update(1, true, 0, 30));
	EXPECT_FALSE(beacon.update(2, true, 100, 0));
	EXPECT_FALSE(beacon.update(0, true, 100, 30));
	EXPECT_FALSE(beacon.update(100, true, 100, 30));
	EXPECT_FALSE(beacon.update(150, true, 200, 30));
	EXPECT_FALSE(beacon.update(300, true, 200, 30));
	EXPECT_TRUE(beacon.update(350, true, 200, 30));
	EXPECT_FALSE(beacon.update(10, true, 200, 30));
	EXPECT_FALSE(beacon.update(500011, true, 200, 30));
	EXPECT_FALSE(beacon.update(500210, true, 200, 30));
	EXPECT_TRUE(beacon.update(500211, true, 200, 30));
}

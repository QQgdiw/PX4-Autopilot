#include "Hx65BackendPolicy.hpp"

#include <gtest/gtest.h>

using namespace hybrid_control;

TEST(Hx65BackendPolicy, NormalizesMirroredPair)
{
	EXPECT_FLOAT_EQ(Hx65BackendPolicy::normalizePair(0, 0, -1000, 1000, 1000, -1000), 0.5f);
	EXPECT_FLOAT_EQ(Hx65BackendPolicy::normalizedSkew(0, 0, -1000, 1000, 1000, -1000), 0.f);
	EXPECT_FLOAT_EQ(Hx65BackendPolicy::normalizedSkew(0, 500, -1000, 1000, 1000, -1000), 0.25f);
	EXPECT_TRUE(Hx65BackendPolicy::endpointMatches(990, -990, true, -1000, 1000, 1000, -1000, 20));
	EXPECT_FALSE(Hx65BackendPolicy::endpointMatches(990, -900, true, -1000, 1000, 1000, -1000, 20));
}

TEST(Hx65BackendPolicy, RejectsDuplicateBusIdsAndOverlappingTolerance)
{
	EXPECT_FALSE(Hx65BackendPolicy::parametersValid(1, 1, 0, -1000, 1000, 1000, -1000, 1000, 10, 100));
	EXPECT_FALSE(Hx65BackendPolicy::parametersValid(1, 2, 1, -1000, 1000, 1000, -1000, 1000, 10, 100));
	EXPECT_FALSE(Hx65BackendPolicy::parametersValid(1, 2, 0, -100, 100, 100, -100, 1000, 10, 100));
}

#include <gtest/gtest.h>

#include "CanOwnership.hpp"

using namespace uavcan_can;

TEST(CanOwnership, ClaimIsExclusiveUntilOwnerReleasesFailedInitialization)
{
	EXPECT_EQ(currentOwner(), Owner::None);
	EXPECT_TRUE(claim(Owner::M2006));
	EXPECT_EQ(currentOwner(), Owner::M2006);
	EXPECT_FALSE(claim(Owner::DroneCan));
	EXPECT_FALSE(claim(Owner::M2006));
	EXPECT_EQ(currentOwner(), Owner::M2006);
	EXPECT_FALSE(release(Owner::DroneCan));
	EXPECT_TRUE(release(Owner::M2006));
	EXPECT_EQ(currentOwner(), Owner::None);
}

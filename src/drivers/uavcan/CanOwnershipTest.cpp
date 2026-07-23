#include <gtest/gtest.h>

#include "CanOwnership.hpp"

using namespace uavcan_can;

class CanOwnershipTest : public ::testing::Test
{
protected:
	void TearDown() override
	{
		for (const Owner owner : {Owner::DroneCan, Owner::M2006}) {
			release(owner, Can1Mask);
			release(owner, Can2Mask);
		}
	}
};

TEST_F(CanOwnershipTest, AllowsDifferentOwnersOnDifferentPhysicalBuses)
{
	EXPECT_TRUE(claim(Owner::DroneCan, Can1Mask));
	EXPECT_TRUE(claim(Owner::M2006, Can2Mask));
	EXPECT_EQ(currentOwner(0), Owner::DroneCan);
	EXPECT_EQ(currentOwner(1), Owner::M2006);
}

TEST_F(CanOwnershipTest, RejectsOverlappingClaimAndWrongRelease)
{
	ASSERT_TRUE(claim(Owner::DroneCan, Can1Mask));
	EXPECT_FALSE(claim(Owner::M2006, Can1Mask));
	EXPECT_FALSE(release(Owner::M2006, Can1Mask));
	EXPECT_TRUE(release(Owner::DroneCan, Can1Mask));
}

TEST_F(CanOwnershipTest, ClaimsMultipleBusesAllOrNothing)
{
	ASSERT_TRUE(claim(Owner::DroneCan, Can2Mask));
	EXPECT_FALSE(claim(Owner::M2006, Can1Mask | Can2Mask));
	EXPECT_EQ(currentOwner(0), Owner::None);
	EXPECT_EQ(currentOwner(1), Owner::DroneCan);
}

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

TEST_F(CanOwnershipTest, RejectsInvalidTwoBitOwner)
{
	const Owner invalid_owner = static_cast<Owner>(3);

	EXPECT_FALSE(claim(invalid_owner, Can1Mask));
	EXPECT_EQ(currentOwner(0), Owner::None);

	// Clean up the invalid state created by the implementation under test during RED.
	release(invalid_owner, Can1Mask);
}

TEST_F(CanOwnershipTest, RejectsOwnerThatWouldCorruptNeighboringInterface)
{
	const Owner invalid_owner = static_cast<Owner>(4);

	EXPECT_FALSE(claim(invalid_owner, Can1Mask));
	EXPECT_EQ(currentOwner(0), Owner::None);
	EXPECT_EQ(currentOwner(1), Owner::None);
}

TEST_F(CanOwnershipTest, RejectsInvalidOwnerReleaseWithoutModifyingBuses)
{
	ASSERT_TRUE(claim(Owner::DroneCan, Can1Mask));
	ASSERT_TRUE(claim(Owner::M2006, Can2Mask));

	EXPECT_FALSE(release(static_cast<Owner>(3), Can1Mask));
	EXPECT_FALSE(release(static_cast<Owner>(4), Can2Mask));
	EXPECT_EQ(currentOwner(0), Owner::DroneCan);
	EXPECT_EQ(currentOwner(1), Owner::M2006);
}

TEST_F(CanOwnershipTest, RejectsEmptyMaskWithoutModifyingBuses)
{
	ASSERT_TRUE(claim(Owner::DroneCan, Can1Mask));

	EXPECT_FALSE(claim(Owner::M2006, 0));
	EXPECT_FALSE(release(Owner::DroneCan, 0));
	EXPECT_EQ(currentOwner(0), Owner::DroneCan);
	EXPECT_EQ(currentOwner(1), Owner::None);
}

TEST_F(CanOwnershipTest, ReturnsNoneForOutOfRangePhysicalInterface)
{
	ASSERT_TRUE(claim(Owner::DroneCan, Can1Mask));
	ASSERT_TRUE(claim(Owner::M2006, Can2Mask));

	EXPECT_EQ(currentOwner(16), Owner::None);
	EXPECT_EQ(currentOwner(UINT8_MAX), Owner::None);
	EXPECT_EQ(currentOwner(0), Owner::DroneCan);
	EXPECT_EQ(currentOwner(1), Owner::M2006);
}

TEST_F(CanOwnershipTest, ReleasesMultipleBusesAllOrNothing)
{
	ASSERT_TRUE(claim(Owner::DroneCan, Can1Mask));
	ASSERT_TRUE(claim(Owner::M2006, Can2Mask));

	EXPECT_FALSE(release(Owner::DroneCan, Can1Mask | Can2Mask));
	EXPECT_EQ(currentOwner(0), Owner::DroneCan);
	EXPECT_EQ(currentOwner(1), Owner::M2006);
}

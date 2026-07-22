#include <gtest/gtest.h>

#include <uavcan_stm32h7/bus_off.hpp>

TEST(BusOffCleanup, CancelsOnlyMatchingPendingMailboxes)
{
	const auto result = uavcan_stm32h7::makeBusOffCleanup(0b1011U, 0b0110U);
	EXPECT_EQ(result.cancel_mask, 0b0010U);
	EXPECT_EQ(result.remaining_software_pending, 0b1001U);
}

TEST(BusOffCleanup, HandlesNoHardwarePendingMailboxes)
{
	const auto result = uavcan_stm32h7::makeBusOffCleanup(0b0101U, 0U);
	EXPECT_EQ(result.cancel_mask, 0U);
	EXPECT_EQ(result.remaining_software_pending, 0b0101U);
}

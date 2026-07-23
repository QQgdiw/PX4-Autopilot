#include <gtest/gtest.h>

#include <uavcan_stm32h7/can_iface_binding.hpp>
#include <uavcan_stm32h7/can_interface_map.hpp>

using uavcan_stm32h7::PhysicalCan1;
using uavcan_stm32h7::PhysicalCan2;
using uavcan_stm32h7::makeCanInterfaceMap;

TEST(CanInterfaceMap, MapsEachSinglePhysicalBusToLogicalZero)
{
	const auto can1 = makeCanInterfaceMap(PhysicalCan1, 2);
	ASSERT_TRUE(can1.valid);
	ASSERT_EQ(can1.count, 1);
	EXPECT_EQ(can1.physical_index[0], 0);

	const auto can2 = makeCanInterfaceMap(PhysicalCan2, 2);
	ASSERT_TRUE(can2.valid);
	ASSERT_EQ(can2.count, 1);
	EXPECT_EQ(can2.physical_index[0], 1);
}

TEST(CanInterfaceMap, PreservesPhysicalOrderForTwoBuses)
{
	const auto map = makeCanInterfaceMap(PhysicalCan1 | PhysicalCan2, 2);
	ASSERT_TRUE(map.valid);
	ASSERT_EQ(map.count, 2);
	EXPECT_EQ(map.physical_index[0], 0);
	EXPECT_EQ(map.physical_index[1], 1);
}

TEST(CanInterfaceMap, RejectsEmptyOrUnsupportedMasks)
{
	EXPECT_FALSE(makeCanInterfaceMap(0, 2).valid);
	EXPECT_FALSE(makeCanInterfaceMap(PhysicalCan2, 1).valid);
	EXPECT_FALSE(makeCanInterfaceMap(1U << 2, 2).valid);
}

TEST(CanIfaceBinding, KeepsTouchedBusReservedAfterOwnerDetaches)
{
	uavcan_stm32h7::CanIfaceBinding binding;
	int first_owner;
	int second_owner;

	EXPECT_TRUE(binding.bind(&first_owner));
	EXPECT_TRUE(binding.bind(&first_owner));
	EXPECT_FALSE(binding.bind(&second_owner));
	EXPECT_FALSE(binding.detach(&second_owner));
	EXPECT_EQ(binding.iface(), &first_owner);
	EXPECT_TRUE(binding.detach(&first_owner));
	EXPECT_EQ(binding.iface(), nullptr);
	EXPECT_TRUE(binding.reserved());
	EXPECT_FALSE(binding.bind(&first_owner));
	EXPECT_FALSE(binding.bind(&second_owner));
}

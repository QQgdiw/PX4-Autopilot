#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include <uavcan_stm32h7/can_iface_binding.hpp>
#include <uavcan_stm32h7/can_init_once.hpp>
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

namespace
{
std::atomic<bool> init_callback_entered{false};
std::atomic<bool> release_init_callback{false};
std::atomic<unsigned> init_callback_count{0};

void blockingInitializer()
{
	++init_callback_count;
	init_callback_entered = true;

	while (!release_init_callback.load()) {
		std::this_thread::yield();
	}
}

template<typename Predicate>
bool waitUntil(Predicate predicate)
{
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);

	while (!predicate() && std::chrono::steady_clock::now() < deadline) {
		std::this_thread::yield();
	}

	return predicate();
}
} // namespace

TEST(CanInitOnce, WaitsForInitializerCompletion)
{
	uavcan_stm32h7::CanInitOnce state = UAVCAN_STM32H7_CAN_INIT_ONCE_INITIALIZER;
	std::atomic<bool> first_result{false};
	std::atomic<bool> second_result{false};
	std::atomic<bool> second_returned{false};
	init_callback_entered = false;
	release_init_callback = false;
	init_callback_count = 0;

	std::thread first([&state, &first_result]() {
		first_result = uavcan_stm32h7::runCanInitOnce(state, blockingInitializer);
	});
	const bool first_entered = waitUntil([]() { return init_callback_entered.load(); });

	std::thread second([&state, &second_result, &second_returned]() {
		second_result = uavcan_stm32h7::runCanInitOnce(state, blockingInitializer);
		second_returned = true;
	});
	const bool second_waited = waitUntil([&state]() { return uavcan_stm32h7::canInitOnceWaiterCount(state) == 1; });
	const bool returned_before_release = second_returned.load();

	release_init_callback = true;
	first.join();
	second.join();
	EXPECT_TRUE(first_entered);
	EXPECT_TRUE(second_waited);
	EXPECT_FALSE(returned_before_release);
	EXPECT_TRUE(first_result.load());
	EXPECT_TRUE(second_result.load());
	EXPECT_TRUE(second_returned.load());
	EXPECT_EQ(init_callback_count.load(), 1U);
	EXPECT_EQ(pthread_cond_destroy(&state.condition), 0);
	EXPECT_EQ(pthread_mutex_destroy(&state.mutex), 0);
}

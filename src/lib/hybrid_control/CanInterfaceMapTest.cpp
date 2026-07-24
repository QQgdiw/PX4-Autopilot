#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include <uavcan_stm32h7/can_iface_binding.hpp>
#include <uavcan_stm32h7/can_driver_topology.hpp>
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

TEST(CanDriverView, ExposesConfiguredInterfacesBeforeHardwareInit)
{
	int physical_can1;
	int physical_can2;

	const uavcan_stm32h7::CanDriverView<int> can1(PhysicalCan1, physical_can1, physical_can2);
	ASSERT_TRUE(can1.valid());
	ASSERT_EQ(can1.count(), 1);
	EXPECT_EQ(can1.physicalIndex(0), 0);
	EXPECT_EQ(can1.getIface(0), &physical_can1);

	const uavcan_stm32h7::CanDriverView<int> can2(PhysicalCan2, physical_can1, physical_can2);
	ASSERT_TRUE(can2.valid());
	ASSERT_EQ(can2.count(), 1);
	EXPECT_EQ(can2.physicalIndex(0), 1);
	EXPECT_EQ(can2.getIface(0), &physical_can2);

	const uavcan_stm32h7::CanDriverView<int> both(PhysicalCan1 | PhysicalCan2, physical_can1, physical_can2);
	ASSERT_TRUE(both.valid());
	ASSERT_EQ(both.count(), 2);
	EXPECT_EQ(both.physicalIndex(0), 0);
	EXPECT_EQ(both.physicalIndex(1), 1);
	EXPECT_EQ(both.getIface(0), &physical_can1);
	EXPECT_EQ(both.getIface(1), &physical_can2);
}

TEST(CanDriverView, RejectsInvalidConfiguredMasks)
{
	int physical_can1;
	int physical_can2;

	const uavcan_stm32h7::CanDriverView<int> empty(0, physical_can1, physical_can2);
	EXPECT_FALSE(empty.valid());
	EXPECT_EQ(empty.count(), 0);
	EXPECT_EQ(empty.getIface(0), nullptr);

	const uavcan_stm32h7::CanDriverView<int> unavailable_can2(PhysicalCan2, physical_can1);
	EXPECT_FALSE(unavailable_can2.valid());
	EXPECT_EQ(unavailable_can2.count(), 0);
	EXPECT_EQ(unavailable_can2.getIface(0), nullptr);

	const uavcan_stm32h7::CanDriverView<int> unsupported(1U << 2, physical_can1, physical_can2);
	EXPECT_FALSE(unsupported.valid());
	EXPECT_EQ(unsupported.count(), 0);
	EXPECT_EQ(unsupported.getIface(0), nullptr);

	for (uint32_t mask = 0; mask <= UINT16_MAX; ++mask) {
		if (mask != PhysicalCan1 && mask != PhysicalCan2 && mask != (PhysicalCan1 | PhysicalCan2)) {
			const uavcan_stm32h7::CanDriverView<int> invalid(mask, physical_can1, physical_can2);
			EXPECT_FALSE(invalid.valid()) << "mask " << mask;
			EXPECT_EQ(invalid.count(), 0) << "mask " << mask;
			EXPECT_EQ(invalid.getIface(0), nullptr) << "mask " << mask;
		}
	}
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

#pragma once

#include <cstdint>

namespace uavcan_stm32h7
{

struct BusOffCleanup {
	uint32_t cancel_mask;
	uint32_t remaining_software_pending;
};

constexpr BusOffCleanup makeBusOffCleanup(const uint32_t software_pending,
		const uint32_t hardware_pending)
{
	const uint32_t cancel_mask = software_pending & hardware_pending;
	return {cancel_mask, software_pending & ~cancel_mask};
}

} // namespace uavcan_stm32h7

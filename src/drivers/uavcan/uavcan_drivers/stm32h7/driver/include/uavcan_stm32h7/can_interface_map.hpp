#pragma once

#include <cstdint>

namespace uavcan_stm32h7
{

constexpr uint8_t MaxPhysicalCanInterfaces{2};
constexpr uint16_t PhysicalCan1{1U << 0};
constexpr uint16_t PhysicalCan2{1U << 1};

struct CanInterfaceMap {
	uint8_t physical_index[MaxPhysicalCanInterfaces]{};
	uint8_t count{0};
	bool valid{false};
};

constexpr CanInterfaceMap makeCanInterfaceMap(const uint16_t enabled_interfaces,
		const uint8_t physical_interface_count)
{
	CanInterfaceMap result{};

	if (physical_interface_count == 0 || physical_interface_count > MaxPhysicalCanInterfaces) {
		return result;
	}

	const uint16_t supported_mask = static_cast<uint16_t>((1U << physical_interface_count) - 1U);

	if (enabled_interfaces == 0 || (enabled_interfaces & ~supported_mask) != 0) {
		return result;
	}

	for (uint8_t physical = 0; physical < physical_interface_count; ++physical) {
		if ((enabled_interfaces & static_cast<uint16_t>(1U << physical)) != 0) {
			result.physical_index[result.count++] = physical;
		}
	}

	result.valid = true;
	return result;
}

} // namespace uavcan_stm32h7

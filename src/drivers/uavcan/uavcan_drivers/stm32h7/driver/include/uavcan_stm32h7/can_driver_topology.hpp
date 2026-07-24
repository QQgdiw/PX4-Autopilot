#pragma once

#include <cstdint>
#include <uavcan_stm32h7/can_interface_map.hpp>

namespace uavcan_stm32h7
{
class CanDriverTopology
{
public:
	constexpr CanDriverTopology(uint16_t enabled_interfaces, uint8_t physical_interface_count) :
		_map(makeCanInterfaceMap(enabled_interfaces, physical_interface_count)) {}

	constexpr bool valid() const { return _map.valid; }
	constexpr uint8_t count() const { return _map.count; }
	constexpr uint8_t physicalIndex(uint8_t logical_index) const
	{
		return logical_index < _map.count ? _map.physical_index[logical_index] : UINT8_MAX;
	}

private:
	CanInterfaceMap _map;
};
} // namespace uavcan_stm32h7

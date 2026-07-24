#pragma once

#include <cstdint>
#include <uavcan_stm32h7/can_interface_map.hpp>

namespace uavcan_stm32h7
{
template <typename Iface>
class CanDriverView
{
public:
	CanDriverView(uint16_t enabled_interfaces, Iface &iface0) :
		_map(makeCanInterfaceMap(enabled_interfaces, 1)),
		_logical_ifaces{}
	{
		initialize(&iface0, nullptr);
	}

	CanDriverView(uint16_t enabled_interfaces, Iface &iface0, Iface &iface1) :
		_map(makeCanInterfaceMap(enabled_interfaces, 2)),
		_logical_ifaces{}
	{
		initialize(&iface0, &iface1);
	}

	constexpr bool valid() const { return _map.valid; }
	constexpr uint8_t count() const { return _map.count; }
	constexpr uint8_t physicalIndex(uint8_t logical_index) const
	{
		return logical_index < _map.count ? _map.physical_index[logical_index] : UINT8_MAX;
	}
	Iface *getIface(uint8_t logical_index) const
	{
		return logical_index < _map.count ? _logical_ifaces[logical_index] : nullptr;
	}

private:
	void initialize(Iface *iface0, Iface *iface1)
	{
		Iface *const physical_ifaces[MaxPhysicalCanInterfaces]{iface0, iface1};

		for (uint8_t logical = 0; logical < _map.count; ++logical) {
			_logical_ifaces[logical] = physical_ifaces[_map.physical_index[logical]];
		}
	}

	CanInterfaceMap _map;
	Iface *_logical_ifaces[MaxPhysicalCanInterfaces];
};
} // namespace uavcan_stm32h7

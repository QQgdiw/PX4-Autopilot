#include "CanOwnership.hpp"

#include <px4_platform_common/atomic.h>

namespace uavcan_can
{

namespace
{
constexpr uint8_t OwnerFieldWidth{2};
constexpr uint8_t PhysicalInterfaceCount{16};

px4::atomic<uint32_t> g_owners{0};

uint32_t ownerFieldMask(uint16_t physical_mask)
{
	uint32_t field_mask = 0;

	for (uint8_t index = 0; index < PhysicalInterfaceCount; ++index) {
		if (physical_mask & (1U << index)) {
			field_mask |= 0x3U << (index * OwnerFieldWidth);
		}
	}

	return field_mask;
}

uint32_t encodedOwner(Owner owner, uint16_t physical_mask)
{
	uint32_t encoded_owner = 0;

	for (uint8_t index = 0; index < PhysicalInterfaceCount; ++index) {
		if (physical_mask & (1U << index)) {
			encoded_owner |= static_cast<uint32_t>(owner) << (index * OwnerFieldWidth);
		}
	}

	return encoded_owner;
}
}

bool claim(Owner owner, uint16_t physical_mask)
{
	if (owner == Owner::None || physical_mask == 0) {
		return false;
	}

	const uint32_t field_mask = ownerFieldMask(physical_mask);
	const uint32_t encoded_owner = encodedOwner(owner, physical_mask);
	uint32_t observed = g_owners.load();

	while ((observed & field_mask) == 0) {
		if (g_owners.compare_exchange(&observed, observed | encoded_owner)) {
			return true;
		}
	}

	return false;
}

bool release(Owner owner, uint16_t physical_mask)
{
	if (owner == Owner::None || physical_mask == 0) {
		return false;
	}

	const uint32_t field_mask = ownerFieldMask(physical_mask);
	const uint32_t encoded_owner = encodedOwner(owner, physical_mask);
	uint32_t observed = g_owners.load();

	while ((observed & field_mask) == encoded_owner) {
		if (g_owners.compare_exchange(&observed, observed & ~field_mask)) {
			return true;
		}
	}

	return false;
}

Owner currentOwner(uint8_t physical_index)
{
	if (physical_index >= PhysicalInterfaceCount) {
		return Owner::None;
	}

	const uint32_t owner = (g_owners.load() >> (physical_index * OwnerFieldWidth)) & 0x3U;
	return static_cast<Owner>(owner);
}

} // namespace uavcan_can

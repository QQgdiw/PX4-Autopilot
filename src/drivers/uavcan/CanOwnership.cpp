#include "CanOwnership.hpp"

#include <px4_platform_common/atomic.h>

namespace uavcan_can
{

namespace
{
px4::atomic<uint8_t> g_owner{static_cast<uint8_t>(Owner::None)};
}

bool claim(Owner owner)
{
	uint8_t expected = static_cast<uint8_t>(Owner::None);
	return owner != Owner::None && g_owner.compare_exchange(&expected, static_cast<uint8_t>(owner));
}

Owner currentOwner()
{
	return static_cast<Owner>(g_owner.load());
}

} // namespace uavcan_can

#pragma once

#include <cstdint>

namespace uavcan_can
{

enum class Owner : uint8_t { None, DroneCan, M2006 };

bool claim(Owner owner);
bool release(Owner owner);
Owner currentOwner();

} // namespace uavcan_can

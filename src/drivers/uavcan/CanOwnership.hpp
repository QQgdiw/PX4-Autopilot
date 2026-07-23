#pragma once

#include <cstdint>

namespace uavcan_can
{

enum class Owner : uint8_t { None, DroneCan, M2006 };

constexpr uint16_t Can1Mask{1U << 0};
constexpr uint16_t Can2Mask{1U << 1};

bool claim(Owner owner, uint16_t physical_mask);
bool release(Owner owner, uint16_t physical_mask);
Owner currentOwner(uint8_t physical_index);

} // namespace uavcan_can

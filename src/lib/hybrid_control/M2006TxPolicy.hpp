#pragma once

namespace hybrid_control
{

constexpr bool shouldTransmitM2006Command(const bool left_online, const bool right_online)
{
	return left_online || right_online;
}

} // namespace hybrid_control

#pragma once

#include <cerrno>

namespace hx8_cli
{

constexpr int driverInstanceStatus(bool available)
{
	return available ? 0 : -ENODEV;
}

} // namespace hx8_cli

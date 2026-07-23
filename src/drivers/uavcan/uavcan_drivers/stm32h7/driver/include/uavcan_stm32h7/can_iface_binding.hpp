#pragma once

namespace uavcan_stm32h7
{

class CanIfaceBinding
{
	void *iface_{nullptr};
	// FDCAN has no safe deinit path; a touched bus stays reserved until reboot.
	bool reserved_{false};

public:
	bool canBind(void *const iface) const
	{
		return iface != nullptr && (!reserved_ || iface_ == iface);
	}

	bool bind(void *const iface)
	{
		if (!canBind(iface)) {
			return false;
		}

		iface_ = iface;
		reserved_ = true;
		return true;
	}

	bool detach(void *const iface)
	{
		if (iface_ != iface || iface == nullptr) {
			return false;
		}

		iface_ = nullptr;
		return true;
	}

	void *iface() const { return iface_; }
	bool reserved() const { return reserved_; }
};

} // namespace uavcan_stm32h7

/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <uORB/topics/hybrid_vehicle_status.h>

namespace commander
{

constexpr hrt_abstime HybridStatusTimeoutUs{1000000};

inline bool hybridStatusIsFresh(const hybrid_vehicle_status_s &status, hrt_abstime now)
{
	return status.timestamp != 0 && now >= status.timestamp && now - status.timestamp <= HybridStatusTimeoutUs;
}

inline bool hybridStateEnablesControl(uint8_t state)
{
	return state == hybrid_vehicle_status_s::HYBRID_STATE_FLYING
	       || state == hybrid_vehicle_status_s::HYBRID_STATE_DRIVING;
}

inline uint8_t hybridStateForCommander(const hybrid_vehicle_status_s &status, hrt_abstime now)
{
	if (!hybridStatusIsFresh(status, now)) {
		return hybrid_vehicle_status_s::HYBRID_STATE_UNKNOWN;
	}

	if (status.fault_reason != hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE) {
		return hybrid_vehicle_status_s::HYBRID_STATE_TRANSITION_FAULT;
	}

	return status.current_state;
}

inline bool shouldProcessHybridStatus(bool is_quad_rover, bool status_updated)
{
	return is_quad_rover && status_updated;
}

} // namespace commander

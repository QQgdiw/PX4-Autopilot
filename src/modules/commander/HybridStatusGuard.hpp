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

enum class HybridRedPattern : uint8_t {
	Off,
	OverloadFast,
	FaultSlow,
	StallDouble,
	CombinedTriple
};

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

inline HybridRedPattern hybridRedPattern(const hybrid_vehicle_status_s &status, bool status_fresh, bool overload)
{
	if (!status_fresh) {
		return overload ? HybridRedPattern::CombinedTriple : HybridRedPattern::FaultSlow;
	}

	const bool fault = status.current_state == hybrid_vehicle_status_s::HYBRID_STATE_TRANSITION_FAULT
			   || status.fault_reason != hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE;

	if (overload && fault) {
		return HybridRedPattern::CombinedTriple;
	}

	if (overload) {
		return HybridRedPattern::OverloadFast;
	}

	if (status.fault_reason == hybrid_vehicle_status_s::TRANSFORM_FAULT_STALL) {
		return HybridRedPattern::StallDouble;
	}

	return fault ? HybridRedPattern::FaultSlow : HybridRedPattern::Off;
}

inline bool hybridRedLedOn(HybridRedPattern pattern, hrt_abstime phase_us)
{
	switch (pattern) {
	case HybridRedPattern::OverloadFast:
		return phase_us % 100000 < 50000;

	case HybridRedPattern::FaultSlow:
		return phase_us % 1000000 < 500000;

	case HybridRedPattern::StallDouble: {
			const hrt_abstime phase = phase_us % 1450000;
			return phase < 150000 || (phase >= 300000 && phase < 450000);
		}

	case HybridRedPattern::CombinedTriple: {
			const hrt_abstime phase = phase_us % 1750000;
			return phase < 150000 || (phase >= 300000 && phase < 450000)
			       || (phase >= 600000 && phase < 750000);
		}

	case HybridRedPattern::Off:
	default:
		return false;
	}
}

} // namespace commander

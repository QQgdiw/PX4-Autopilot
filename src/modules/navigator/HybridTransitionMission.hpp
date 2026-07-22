/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <uORB/topics/hybrid_vehicle_status.h>

static constexpr hrt_abstime kHybridTransitionMissionStatusTimeoutUs{1000000};

inline bool hybridTransitionMissionStatusFresh(const hybrid_vehicle_status_s &status, hrt_abstime now)
{
	return status.timestamp != 0 && now >= status.timestamp
	       && now - status.timestamp <= kHybridTransitionMissionStatusTimeoutUs;
}

inline bool hybridTransitionMissionReached(uint32_t minimum_sequence, uint8_t target,
		const hybrid_vehicle_status_s &status)
{
	const uint8_t stable_state = target == hybrid_vehicle_status_s::TARGET_FLYING
				     ? hybrid_vehicle_status_s::HYBRID_STATE_FLYING
				     : hybrid_vehicle_status_s::HYBRID_STATE_DRIVING;

	return (target == hybrid_vehicle_status_s::TARGET_FLYING
		|| target == hybrid_vehicle_status_s::TARGET_DRIVING)
	       && status.transition_sequence >= minimum_sequence
	       && status.target_state == target
	       && status.current_state == stable_state;
}

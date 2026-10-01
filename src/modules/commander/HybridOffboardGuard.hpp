/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 ****************************************************************************/

#pragma once

#include "HybridStatusGuard.hpp"
#include <lib/rover_control/RoverVelocityOffboardPolicy.hpp>
#include <uORB/topics/offboard_control_mode.h>

namespace commander
{

inline bool hybridOffboardModeAvailable(const offboard_control_mode_s &mode,
					const hybrid_vehicle_status_s &status, uint8_t vehicle_type, hrt_abstime now, hrt_abstime maximum_age)
{
	if (!roverVelocityTimestampUsable(mode.timestamp, now, maximum_age)
	    || !roverVelocityTimestampUsable(status.timestamp, now, HybridStatusTimeoutUs)
	    || status.fault_reason != hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE
	    || status.sequence_fault != hybrid_vehicle_status_s::SEQUENCE_FAULT_NONE
	    || !status.propulsion_ready) {
		return false;
	}

	const bool flying = status.current_state == hybrid_vehicle_status_s::HYBRID_STATE_FLYING
			    && vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;
	const bool driving = status.current_state == hybrid_vehicle_status_s::HYBRID_STATE_DRIVING
			     && vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER;

	// Only the coordinated HX65 backend publishes propulsion ownership.
	if (status.actuator_backend == hybrid_vehicle_status_s::ACTUATOR_HX65
	    && status.propulsion_owner != (flying ? hybrid_vehicle_status_s::PROPULSION_QUAD
					   : hybrid_vehicle_status_s::PROPULSION_ROVER)) {
		return false;
	}

	if (flying) {
		return !mode.rover_velocity && (mode.position || mode.velocity || mode.acceleration
						|| mode.attitude || mode.body_rate || mode.thrust_and_torque || mode.direct_actuator);
	}

	const RoverVelocityOffboardMode rover_mode{mode.timestamp, mode.position, mode.velocity, mode.acceleration,
			mode.attitude, mode.body_rate, mode.thrust_and_torque, mode.direct_actuator, mode.rover_velocity};
	return driving && roverVelocityModeUsable(rover_mode, now, maximum_age)
	       && mode.timestamp > status.transition_completed_timestamp;
}

} // namespace commander

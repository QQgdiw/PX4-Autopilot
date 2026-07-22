/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 ****************************************************************************/

#pragma once

#include <cmath>
#include <cstdint>

struct RoverVelocityOffboardInput {
	uint64_t timestamp;
	float speed_body_x;
	float yaw_rate;
};

struct RoverVelocityOffboardMode {
	uint64_t timestamp;
	bool position;
	bool velocity;
	bool acceleration;
	bool attitude;
	bool body_rate;
	bool thrust_and_torque;
	bool direct_actuator;
	bool rover_velocity;
};

struct RoverVelocityDrivingStatus {
	uint64_t timestamp;
	uint64_t transition_completed_timestamp;
	bool driving;
	bool fault_free;
};

inline bool roverVelocityTimestampUsable(uint64_t timestamp, uint64_t now, uint64_t maximum_age)
{
	return timestamp != 0 && timestamp <= now && now - timestamp <= maximum_age;
}

inline bool roverVelocityModeUsable(const RoverVelocityOffboardMode &mode, uint64_t now, uint64_t maximum_age)
{
	const bool other_control_bit = mode.position || mode.velocity || mode.acceleration || mode.attitude || mode.body_rate
				       || mode.thrust_and_torque || mode.direct_actuator;

	return mode.rover_velocity && !other_control_bit
	       && roverVelocityTimestampUsable(mode.timestamp, now, maximum_age);
}

inline bool roverDrivingStatusUsable(const RoverVelocityDrivingStatus &status, uint64_t now, uint64_t maximum_age)
{
	return status.driving && status.fault_free
	       && roverVelocityTimestampUsable(status.timestamp, now, maximum_age);
}

inline bool roverOffboardModeAvailable(bool is_quad_rover, const RoverVelocityOffboardMode &mode,
		const RoverVelocityDrivingStatus &status, uint64_t now, uint64_t mode_maximum_age,
		uint64_t status_maximum_age)
{
	const bool any_control_bit = mode.position || mode.velocity || mode.acceleration || mode.attitude || mode.body_rate
				     || mode.thrust_and_torque || mode.direct_actuator || mode.rover_velocity;

	if (!is_quad_rover && !mode.rover_velocity) {
		return any_control_bit && roverVelocityTimestampUsable(mode.timestamp, now, mode_maximum_age);
	}

	return roverVelocityModeUsable(mode, now, mode_maximum_age)
	       && roverDrivingStatusUsable(status, now, status_maximum_age);
}

inline bool roverVelocityDedicatedControlRequired(bool is_quad_rover, bool offboard_enabled,
		bool rover_velocity_selected, bool legacy_velocity_control_active)
{
	return offboard_enabled
	       && (is_quad_rover || (rover_velocity_selected && legacy_velocity_control_active));
}

inline bool roverVelocityInputUsable(const RoverVelocityOffboardInput &input, uint64_t transition_completed_timestamp,
		uint64_t now, uint64_t maximum_age, bool driving_healthy)
{
	return driving_healthy && input.timestamp > transition_completed_timestamp
	       && roverVelocityTimestampUsable(input.timestamp, now, maximum_age)
	       && std::isfinite(input.speed_body_x) && std::isfinite(input.yaw_rate);
}

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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef ROVER_POSITION_TUNING_STATUS_HPP
#define ROVER_POSITION_TUNING_STATUS_HPP

#include <mathlib/mathlib.h>
#include <mavlink.h>
#include <mavlink/mavlink_stream.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/hybrid_vehicle_status.h>
#include <uORB/topics/rover_position_status.h>
#include <uORB/topics/vehicle_status.h>

class MavlinkStreamRoverPositionTuningStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamRoverPositionTuningStatus(mavlink); }
	static constexpr const char *get_name_static() { return "ROVER_POSITION_TUNING_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ROVER_POSITION_TUNING_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _status_sub.advertised() ? MAVLINK_MSG_ID_ROVER_POSITION_TUNING_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamRoverPositionTuningStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _status_sub{ORB_ID(rover_position_status)};
	uORB::Subscription _hybrid_status_sub{ORB_ID(hybrid_vehicle_status)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	rover_position_status_s _status{};
	hybrid_vehicle_status_s _hybrid_status{};
	vehicle_status_s _vehicle_status{};

	bool send() override
	{
		if (_mavlink->get_free_tx_buf() < get_size()) {
			return false;
		}

		_hybrid_status_sub.update(&_hybrid_status);
		_vehicle_status_sub.update(&_vehicle_status);
		const bool rover_active = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER
					  && _vehicle_status.timestamp > 0 && hrt_elapsed_time(&_vehicle_status.timestamp) < 1_s
					  && _hybrid_status.timestamp > 0 && hrt_elapsed_time(&_hybrid_status.timestamp) < 200_ms
					  && _hybrid_status.current_state == hybrid_vehicle_status_s::HYBRID_STATE_DRIVING
					  && _hybrid_status.fault_reason == hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE;
		const bool status_updated = _status_sub.update(&_status);
		const bool status_fresh = _status.timestamp > 0 && hrt_elapsed_time(&_status.timestamp) < 200_ms;

		if (rover_active && !status_updated && status_fresh) {
			return false;
		}

		mavlink_rover_position_tuning_status_t msg{};
		msg.time_usec = rover_active && status_fresh && _status.active ? _status.timestamp :
				math::max(_hybrid_status.timestamp, _vehicle_status.timestamp);
		msg.drive_type = rover_active ? ROVER_DRIVE_TYPE_DIFFERENTIAL : ROVER_DRIVE_TYPE_UNKNOWN;
		msg.xy_reset_counter = rover_active && status_fresh && _status.active ? _status.xy_reset_counter : 0;
		msg.position_north = NAN;
		msg.position_east = NAN;
		msg.target_north = NAN;
		msg.target_east = NAN;
		msg.crosstrack_error = NAN;
		msg.lookahead_distance = NAN;
		msg.target_bearing = NAN;
		msg.distance_to_waypoint = NAN;

		if (rover_active && status_fresh && _status.active) {
			msg.valid_flags |= ROVER_TUNING_STATUS_FLAGS_CONTROLLER_ACTIVE;

			if (_status.position_valid && PX4_ISFINITE(_status.position_ned[0]) && PX4_ISFINITE(_status.position_ned[1])) {
				msg.position_north = _status.position_ned[0];
				msg.position_east = _status.position_ned[1];
				msg.valid_flags |= ROVER_TUNING_STATUS_FLAGS_POSITION_VALID;
			}

			if (_status.target_valid && PX4_ISFINITE(_status.target_ned[0]) && PX4_ISFINITE(_status.target_ned[1])) {
				msg.target_north = _status.target_ned[0];
				msg.target_east = _status.target_ned[1];
				msg.valid_flags |= ROVER_TUNING_STATUS_FLAGS_TARGET_VALID;
			}

			if (_status.path_valid && PX4_ISFINITE(_status.crosstrack_error)
			    && PX4_ISFINITE(_status.lookahead_distance) && PX4_ISFINITE(_status.target_bearing)
			    && PX4_ISFINITE(_status.distance_to_waypoint)) {
				msg.crosstrack_error = _status.crosstrack_error;
				msg.lookahead_distance = _status.lookahead_distance;
				msg.target_bearing = _status.target_bearing;
				msg.distance_to_waypoint = _status.distance_to_waypoint;
				msg.valid_flags |= ROVER_TUNING_STATUS_FLAGS_PATH_VALID;
			}
		}

		mavlink_msg_rover_position_tuning_status_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // ROVER_POSITION_TUNING_STATUS_HPP

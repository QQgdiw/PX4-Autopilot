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

#ifndef ROVER_ATTITUDE_TUNING_STATUS_HPP
#define ROVER_ATTITUDE_TUNING_STATUS_HPP

#include <mathlib/mathlib.h>
#include <mavlink.h>
#include <mavlink/mavlink_stream.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/hybrid_vehicle_status.h>
#include <uORB/topics/rover_attitude_status.h>
#include <uORB/topics/rover_rate_setpoint.h>
#include <uORB/topics/vehicle_status.h>

class MavlinkStreamRoverAttitudeTuningStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamRoverAttitudeTuningStatus(mavlink); }
	static constexpr const char *get_name_static() { return "ROVER_ATTITUDE_TUNING_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ROVER_ATTITUDE_TUNING_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _status_sub.advertised() ? MAVLINK_MSG_ID_ROVER_ATTITUDE_TUNING_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamRoverAttitudeTuningStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _status_sub{ORB_ID(rover_attitude_status)};
	uORB::Subscription _rate_setpoint_sub{ORB_ID(rover_rate_setpoint)};
	uORB::Subscription _hybrid_status_sub{ORB_ID(hybrid_vehicle_status)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	rover_rate_setpoint_s _rate_setpoint{};
	hybrid_vehicle_status_s _hybrid_status{};
	vehicle_status_s _vehicle_status{};

	bool send() override
	{
		rover_attitude_status_s status{};

		if (_mavlink->get_free_tx_buf() < get_size()) {
			return false;
		}

		_hybrid_status_sub.update(&_hybrid_status);
		_vehicle_status_sub.update(&_vehicle_status);
		const bool status_updated = _status_sub.update(&status);
		const bool rover_active = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER
					  && _vehicle_status.timestamp > 0 && hrt_elapsed_time(&_vehicle_status.timestamp) < 1_s
					  && _hybrid_status.timestamp > 0 && hrt_elapsed_time(&_hybrid_status.timestamp) < 200_ms
					  && _hybrid_status.current_state == hybrid_vehicle_status_s::HYBRID_STATE_DRIVING;
		const bool controller_active = rover_active && status_updated && status.active;

		if (rover_active && !status_updated) {
			return false;
		}

		_rate_setpoint_sub.update(&_rate_setpoint);

		mavlink_rover_attitude_tuning_status_t msg{};
		msg.time_usec = rover_active && status_updated ? status.timestamp :
				math::max(_hybrid_status.timestamp, _vehicle_status.timestamp);
		msg.drive_type = rover_active ? ROVER_DRIVE_TYPE_DIFFERENTIAL : ROVER_DRIVE_TYPE_UNKNOWN;
		msg.yaw_response = NAN;
		msg.yaw_setpoint = NAN;
		msg.yaw_rate_setpoint = NAN;

		if (controller_active) {
			msg.valid_flags |= ROVER_TUNING_STATUS_FLAGS_CONTROLLER_ACTIVE;

			if (PX4_ISFINITE(status.measured_yaw)) {
				msg.yaw_response = status.measured_yaw;
				msg.valid_flags |= ROVER_TUNING_STATUS_FLAGS_RESPONSE_PRIMARY_VALID;
			}

			if (PX4_ISFINITE(status.adjusted_yaw_setpoint)) {
				msg.yaw_setpoint = status.adjusted_yaw_setpoint;
				msg.valid_flags |= ROVER_TUNING_STATUS_FLAGS_SETPOINT_PRIMARY_VALID;
			}

			if (_rate_setpoint.timestamp == status.timestamp && PX4_ISFINITE(_rate_setpoint.yaw_rate_setpoint)) {
				msg.yaw_rate_setpoint = _rate_setpoint.yaw_rate_setpoint;
				msg.valid_flags |= ROVER_TUNING_STATUS_FLAGS_OUTPUT_PRIMARY_VALID;
			}
		}

		mavlink_msg_rover_attitude_tuning_status_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // ROVER_ATTITUDE_TUNING_STATUS_HPP

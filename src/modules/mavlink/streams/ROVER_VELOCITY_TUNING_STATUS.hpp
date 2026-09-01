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

#ifndef ROVER_VELOCITY_TUNING_STATUS_HPP
#define ROVER_VELOCITY_TUNING_STATUS_HPP

#include <mathlib/mathlib.h>
#include <mavlink.h>
#include <mavlink/mavlink_stream.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/hybrid_vehicle_status.h>
#include <uORB/topics/rover_throttle_setpoint.h>
#include <uORB/topics/rover_velocity_status.h>
#include <uORB/topics/vehicle_status.h>

class MavlinkStreamRoverVelocityTuningStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamRoverVelocityTuningStatus(mavlink); }
	static constexpr const char *get_name_static() { return "ROVER_VELOCITY_TUNING_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ROVER_VELOCITY_TUNING_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _status_sub.advertised() ? MAVLINK_MSG_ID_ROVER_VELOCITY_TUNING_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamRoverVelocityTuningStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _status_sub{ORB_ID(rover_velocity_status)};
	uORB::Subscription _throttle_sub{ORB_ID(rover_throttle_setpoint)};
	uORB::Subscription _hybrid_status_sub{ORB_ID(hybrid_vehicle_status)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	rover_velocity_status_s _status{};
	rover_throttle_setpoint_s _throttle{};
	hybrid_vehicle_status_s _hybrid_status{};
	vehicle_status_s _vehicle_status{};

	bool send() override
	{
		if (_mavlink->get_free_tx_buf() < get_size()) {
			return false;
		}

		_hybrid_status_sub.update(&_hybrid_status);
		_vehicle_status_sub.update(&_vehicle_status);
		const bool status_updated = _status_sub.update(&_status);
		const bool rover_active = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROVER
					  && _vehicle_status.timestamp > 0 && hrt_elapsed_time(&_vehicle_status.timestamp) < 1_s
					  && _hybrid_status.timestamp > 0 && hrt_elapsed_time(&_hybrid_status.timestamp) < 200_ms
					  && _hybrid_status.current_state == hybrid_vehicle_status_s::HYBRID_STATE_DRIVING
					  && _hybrid_status.fault_reason == hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE;
		const bool status_fresh = _status.timestamp > 0 && hrt_elapsed_time(&_status.timestamp) < 200_ms;
		const bool controller_active = rover_active && status_fresh && _status.active;

		if (rover_active && !status_updated && status_fresh) {
			return false;
		}

		_throttle_sub.update(&_throttle);

		mavlink_rover_velocity_tuning_status_t msg{};
		msg.time_usec = controller_active ? _status.timestamp :
				math::max(_hybrid_status.timestamp, _vehicle_status.timestamp);
		msg.drive_type = rover_active ? ROVER_DRIVE_TYPE_DIFFERENTIAL : ROVER_DRIVE_TYPE_UNKNOWN;
		msg.speed_body_x_response = NAN;
		msg.speed_body_x_setpoint = NAN;
		msg.integral_body_x = NAN;
		msg.throttle_body_x = NAN;
		msg.speed_body_y_response = NAN;
		msg.speed_body_y_setpoint = NAN;
		msg.integral_body_y = NAN;
		msg.throttle_body_y = NAN;

		if (controller_active) {
			msg.valid_flags |= ROVER_TUNING_STATUS_FLAGS_CONTROLLER_ACTIVE;

			if (PX4_ISFINITE(_status.measured_speed_body_x)) {
				msg.speed_body_x_response = _status.measured_speed_body_x;
				msg.valid_flags |= ROVER_TUNING_STATUS_FLAGS_RESPONSE_PRIMARY_VALID;
			}

			if (PX4_ISFINITE(_status.adjusted_speed_body_x_setpoint)) {
				msg.speed_body_x_setpoint = _status.adjusted_speed_body_x_setpoint;
				msg.valid_flags |= ROVER_TUNING_STATUS_FLAGS_SETPOINT_PRIMARY_VALID;
			}

			if (PX4_ISFINITE(_status.pid_throttle_body_x_integral)) {
				msg.integral_body_x = _status.pid_throttle_body_x_integral;
				msg.valid_flags |= ROVER_TUNING_STATUS_FLAGS_INTEGRAL_PRIMARY_VALID;
			}

			if (_throttle.timestamp == _status.timestamp && PX4_ISFINITE(_throttle.throttle_body_x)) {
				msg.throttle_body_x = _throttle.throttle_body_x;
				msg.valid_flags |= ROVER_TUNING_STATUS_FLAGS_OUTPUT_PRIMARY_VALID;
			}
		}

		mavlink_msg_rover_velocity_tuning_status_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // ROVER_VELOCITY_TUNING_STATUS_HPP

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

#ifndef HYBRID_VEHICLE_STATUS_HPP
#define HYBRID_VEHICLE_STATUS_HPP

#include <uORB/topics/hybrid_vehicle_status.h>

class MavlinkStreamHybridVehicleStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamHybridVehicleStatus(mavlink); }

	static constexpr const char *get_name_static() { return "HYBRID_VEHICLE_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_HYBRID_VEHICLE_STATUS; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _hybrid_status_sub.advertised() ? MAVLINK_MSG_ID_HYBRID_VEHICLE_STATUS_LEN
		       + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamHybridVehicleStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _hybrid_status_sub{ORB_ID(hybrid_vehicle_status)};

	bool send() override
	{
		if (_mavlink->get_status()->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
			return false;
		}

		hybrid_vehicle_status_s hybrid_status;

		if (_hybrid_status_sub.update(&hybrid_status)) {
			mavlink_hybrid_vehicle_status_t msg{};
			msg.timestamp = hybrid_status.timestamp;
			msg.transition_sequence = hybrid_status.transition_sequence;
			msg.transition_elapsed_ms = static_cast<uint32_t>(hybrid_status.transition_elapsed);
			msg.position_normalized = hybrid_status.position_normalized;
			msg.current_state = hybrid_status.current_state;
			msg.target_state = hybrid_status.target_state;
			msg.fault_reason = hybrid_status.fault_reason;
			msg.command_result = hybrid_status.command_result;
			msg.sensor_source = hybrid_status.sensor_source;
			msg.actuator_backend = hybrid_status.actuator_backend;
			msg.actuator_protection_flags = hybrid_status.actuator_protection_flags;
			msg.command_timestamp = hybrid_status.command_timestamp;

			msg.flags = (hybrid_status.sensors_enabled ? HYBRID_VEHICLE_STATUS_FLAGS_SENSORS_ENABLED : 0)
				    | (hybrid_status.position_confirmed ? HYBRID_VEHICLE_STATUS_FLAGS_POSITION_CONFIRMED : 0)
				    | (hybrid_status.position_valid ? HYBRID_VEHICLE_STATUS_FLAGS_POSITION_VALID : 0)
				    | (hybrid_status.actuator_online ? HYBRID_VEHICLE_STATUS_FLAGS_ACTUATOR_ONLINE : 0)
				    | (hybrid_status.actuator_healthy ? HYBRID_VEHICLE_STATUS_FLAGS_ACTUATOR_HEALTHY : 0)
				    | (hybrid_status.actuator_config_verified ? HYBRID_VEHICLE_STATUS_FLAGS_ACTUATOR_CONFIG_VERIFIED : 0)
				    | (hybrid_status.landed ? HYBRID_VEHICLE_STATUS_FLAGS_LANDED : 0)
				    | (hybrid_status.land_detection_fresh ? HYBRID_VEHICLE_STATUS_FLAGS_LAND_DETECTION_FRESH : 0);

			mavlink_msg_hybrid_vehicle_status_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // HYBRID_VEHICLE_STATUS_HPP

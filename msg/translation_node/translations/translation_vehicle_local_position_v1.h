// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <px4_msgs_old/msg/vehicle_local_position_v0.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

// Optional companion-side conversion only; the firmware publishes v1, not v0.
// v0 cannot prove local heading validity. Do not infer it from FRD configuration.
class VehicleLocalPositionV1Translation
{
public:
	using MessageOlder = px4_msgs_old::msg::VehicleLocalPositionV0;
	using MessageNewer = px4_msgs::msg::VehicleLocalPosition;
	static_assert(MessageOlder::MESSAGE_VERSION == 0);
	static_assert(MessageNewer::MESSAGE_VERSION == 1);
	static constexpr const char *kTopic = "fmu/out/vehicle_local_position";

	static void fromOlder(const MessageOlder &msg_older, MessageNewer &msg_newer)
	{
		msg_newer.timestamp = msg_older.timestamp;
		msg_newer.timestamp_sample = msg_older.timestamp_sample;
		msg_newer.xy_valid = msg_older.xy_valid;
		msg_newer.z_valid = msg_older.z_valid;
		msg_newer.v_xy_valid = msg_older.v_xy_valid;
		msg_newer.v_z_valid = msg_older.v_z_valid;
		msg_newer.x = msg_older.x;
		msg_newer.y = msg_older.y;
		msg_newer.z = msg_older.z;
		msg_newer.delta_xy = msg_older.delta_xy;
		msg_newer.xy_reset_counter = msg_older.xy_reset_counter;
		msg_newer.delta_z = msg_older.delta_z;
		msg_newer.z_reset_counter = msg_older.z_reset_counter;
		msg_newer.vx = msg_older.vx;
		msg_newer.vy = msg_older.vy;
		msg_newer.vz = msg_older.vz;
		msg_newer.z_deriv = msg_older.z_deriv;
		msg_newer.delta_vxy = msg_older.delta_vxy;
		msg_newer.vxy_reset_counter = msg_older.vxy_reset_counter;
		msg_newer.delta_vz = msg_older.delta_vz;
		msg_newer.vz_reset_counter = msg_older.vz_reset_counter;
		msg_newer.ax = msg_older.ax;
		msg_newer.ay = msg_older.ay;
		msg_newer.az = msg_older.az;
		msg_newer.heading = msg_older.heading;
		msg_newer.heading_var = msg_older.heading_var;
		msg_newer.unaided_heading = msg_older.unaided_heading;
		msg_newer.delta_heading = msg_older.delta_heading;
		msg_newer.heading_reset_counter = msg_older.heading_reset_counter;
		msg_newer.heading_good_for_control = msg_older.heading_good_for_control;
		msg_newer.tilt_var = msg_older.tilt_var;
		msg_newer.xy_global = msg_older.xy_global;
		msg_newer.z_global = msg_older.z_global;
		msg_newer.ref_timestamp = msg_older.ref_timestamp;
		msg_newer.ref_lat = msg_older.ref_lat;
		msg_newer.ref_lon = msg_older.ref_lon;
		msg_newer.ref_alt = msg_older.ref_alt;
		msg_newer.dist_bottom_valid = msg_older.dist_bottom_valid;
		msg_newer.dist_bottom = msg_older.dist_bottom;
		msg_newer.dist_bottom_var = msg_older.dist_bottom_var;
		msg_newer.delta_dist_bottom = msg_older.delta_dist_bottom;
		msg_newer.dist_bottom_reset_counter = msg_older.dist_bottom_reset_counter;
		msg_newer.dist_bottom_sensor_bitfield = msg_older.dist_bottom_sensor_bitfield;
		msg_newer.eph = msg_older.eph;
		msg_newer.epv = msg_older.epv;
		msg_newer.evh = msg_older.evh;
		msg_newer.evv = msg_older.evv;
		msg_newer.dead_reckoning = msg_older.dead_reckoning;
		msg_newer.vxy_max = msg_older.vxy_max;
		msg_newer.vz_max = msg_older.vz_max;
		msg_newer.hagl_min = msg_older.hagl_min;
		msg_newer.hagl_max_z = msg_older.hagl_max_z;
		msg_newer.hagl_max_xy = msg_older.hagl_max_xy;
		msg_newer.heading_good_for_local_control = false;
	}

	static void toOlder(const MessageNewer &msg_newer, MessageOlder &msg_older)
	{
		msg_older.timestamp = msg_newer.timestamp;
		msg_older.timestamp_sample = msg_newer.timestamp_sample;
		msg_older.xy_valid = msg_newer.xy_valid;
		msg_older.z_valid = msg_newer.z_valid;
		msg_older.v_xy_valid = msg_newer.v_xy_valid;
		msg_older.v_z_valid = msg_newer.v_z_valid;
		msg_older.x = msg_newer.x;
		msg_older.y = msg_newer.y;
		msg_older.z = msg_newer.z;
		msg_older.delta_xy = msg_newer.delta_xy;
		msg_older.xy_reset_counter = msg_newer.xy_reset_counter;
		msg_older.delta_z = msg_newer.delta_z;
		msg_older.z_reset_counter = msg_newer.z_reset_counter;
		msg_older.vx = msg_newer.vx;
		msg_older.vy = msg_newer.vy;
		msg_older.vz = msg_newer.vz;
		msg_older.z_deriv = msg_newer.z_deriv;
		msg_older.delta_vxy = msg_newer.delta_vxy;
		msg_older.vxy_reset_counter = msg_newer.vxy_reset_counter;
		msg_older.delta_vz = msg_newer.delta_vz;
		msg_older.vz_reset_counter = msg_newer.vz_reset_counter;
		msg_older.ax = msg_newer.ax;
		msg_older.ay = msg_newer.ay;
		msg_older.az = msg_newer.az;
		msg_older.heading = msg_newer.heading;
		msg_older.heading_var = msg_newer.heading_var;
		msg_older.unaided_heading = msg_newer.unaided_heading;
		msg_older.delta_heading = msg_newer.delta_heading;
		msg_older.heading_reset_counter = msg_newer.heading_reset_counter;
		msg_older.heading_good_for_control = msg_newer.heading_good_for_control;
		msg_older.tilt_var = msg_newer.tilt_var;
		msg_older.xy_global = msg_newer.xy_global;
		msg_older.z_global = msg_newer.z_global;
		msg_older.ref_timestamp = msg_newer.ref_timestamp;
		msg_older.ref_lat = msg_newer.ref_lat;
		msg_older.ref_lon = msg_newer.ref_lon;
		msg_older.ref_alt = msg_newer.ref_alt;
		msg_older.dist_bottom_valid = msg_newer.dist_bottom_valid;
		msg_older.dist_bottom = msg_newer.dist_bottom;
		msg_older.dist_bottom_var = msg_newer.dist_bottom_var;
		msg_older.delta_dist_bottom = msg_newer.delta_dist_bottom;
		msg_older.dist_bottom_reset_counter = msg_newer.dist_bottom_reset_counter;
		msg_older.dist_bottom_sensor_bitfield = msg_newer.dist_bottom_sensor_bitfield;
		msg_older.eph = msg_newer.eph;
		msg_older.epv = msg_newer.epv;
		msg_older.evh = msg_newer.evh;
		msg_older.evv = msg_newer.evv;
		msg_older.dead_reckoning = msg_newer.dead_reckoning;
		msg_older.vxy_max = msg_newer.vxy_max;
		msg_older.vz_max = msg_newer.vz_max;
		msg_older.hagl_min = msg_newer.hagl_min;
		msg_older.hagl_max_z = msg_newer.hagl_max_z;
		msg_older.hagl_max_xy = msg_newer.hagl_max_xy;
	}
};

REGISTER_TOPIC_TRANSLATION_DIRECT(VehicleLocalPositionV1Translation);

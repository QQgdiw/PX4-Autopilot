/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "DifferentialPosControl.hpp"

using namespace time_literals;

DifferentialPosControl::DifferentialPosControl(ModuleParams *parent) : ModuleParams(parent)
{
	_differential_velocity_setpoint_pub.advertise();
	_rover_position_setpoint_pub.advertise();
	_pure_pursuit_status_pub.advertise();
	_rover_position_status_pub.advertise();

	// Initially set to NaN to indicate that the rover has no position setpoint
	_rover_position_setpoint.position_ned[0] = NAN;
	_rover_position_setpoint.position_ned[1] = NAN;

	updateParams();
}

void DifferentialPosControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;
}

void DifferentialPosControl::updatePosControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	updateSubscriptions();

	bool controller_active = _vehicle_control_mode.flag_control_position_enabled && _vehicle_control_mode.flag_armed;

	if (controller_active) {
		controller_active = runSanityChecks();
	}

	if (!controller_active) {
		resetInactiveState();

	} else {
		updatePositionControlSource(selectPositionControlSource());
	}

	resetPositionStatus(controller_active);

	if (controller_active) {
		if (_position_control_source == PositionControlSource::Offboard) {
			generatePositionSetpoint();
		}

		generateVelocitySetpoint();

	}

	_rover_position_status_pub.publish(_rover_position_status);

}

void DifferentialPosControl::resetInactiveState()
{
	const hrt_abstime now = hrt_absolute_time();

	if (now > _source_epoch) {
		_source_epoch = now;
	}

	resetSourceState();
	_position_control_source = PositionControlSource::Inactive;
	_position_control_source_id = 0xff;
	discardPendingSourceInputs(PositionControlSource::Inactive);
}

DifferentialPosControl::PositionControlSource DifferentialPosControl::selectPositionControlSource() const
{
	if (_vehicle_control_mode.flag_control_manual_enabled && _vehicle_control_mode.flag_control_position_enabled) {
		return PositionControlSource::Manual;
	}

	if (_vehicle_control_mode.flag_control_auto_enabled) {
		return PositionControlSource::Auto;
	}

	if (_vehicle_control_mode.flag_control_offboard_enabled) {
		return PositionControlSource::Offboard;
	}

	return PositionControlSource::GoTo;
}

void DifferentialPosControl::updatePositionControlSource(PositionControlSource source)
{
	if (source == _position_control_source && _vehicle_status.nav_state == _position_control_source_id) {
		return;
	}

	hrt_abstime mode_epoch = _vehicle_control_mode.timestamp;

	if (_vehicle_status.timestamp > mode_epoch) {
		mode_epoch = _vehicle_status.timestamp;
	}

	if (mode_epoch == 0) {
		mode_epoch = _timestamp;
	}

	if (mode_epoch > _source_epoch) {
		_source_epoch = mode_epoch;
	}

	// A source switch invalidates all latched targets. The selected topic is consumed below,
	// but only a sample newer than this mode epoch can repopulate the source cache.
	resetSourceState();
	_position_control_source = source;
	_position_control_source_id = _vehicle_status.nav_state;
	discardPendingSourceInputs(source);
}

void DifferentialPosControl::resetSourceState()
{
	_course_control = false;
	_auto_target_valid = false;
	_manual_control_setpoint_valid = false;
	_manual_control_setpoint = {};
	_curr_wp_type = position_setpoint_s::SETPOINT_TYPE_IDLE;
	_curr_wp_ned = Vector2f(NAN, NAN);
	_prev_wp_ned = Vector2f(NAN, NAN);
	_next_wp_ned = Vector2f(NAN, NAN);
	_waypoint_transition_angle = NAN;
	_cruising_speed = 0.f;
	_pos_ctl_course_direction = Vector2f(NAN, NAN);
	_pos_ctl_start_position_ned = Vector2f(NAN, NAN);
	_source_input_timestamp = 0;
	_offboard_mode_timestamp = 0;
	_rover_position_setpoint = {};
	_rover_position_setpoint.position_ned[0] = NAN;
	_rover_position_setpoint.position_ned[1] = NAN;
	_offboard_control_mode = {};
}

void DifferentialPosControl::discardPendingSourceInputs(PositionControlSource active_source)
{
	// Keep the selected source pending for its handler; consume other topics so they cannot
	// be replayed when a later mode re-enters that source.
	if (active_source != PositionControlSource::Manual) {
		manual_control_setpoint_s discarded_manual{};
		_manual_control_setpoint_sub.update(&discarded_manual);
	}

	if (active_source != PositionControlSource::Auto) {
		position_setpoint_triplet_s discarded_triplet{};
		_position_setpoint_triplet_sub.update(&discarded_triplet);
	}

	if (active_source != PositionControlSource::Offboard) {
		trajectory_setpoint_s discarded_trajectory{};
		offboard_control_mode_s discarded_offboard_mode{};
		_trajectory_setpoint_sub.update(&discarded_trajectory);
		_offboard_control_mode_sub.update(&discarded_offboard_mode);
	}

	if (active_source != PositionControlSource::GoTo) {
		rover_position_setpoint_s discarded_position_setpoint{};
		_rover_position_setpoint_sub.update(&discarded_position_setpoint);
	}
}

void DifferentialPosControl::publishStopSetpoint()
{
	differential_velocity_setpoint_s differential_velocity_setpoint{};
	differential_velocity_setpoint.timestamp = _timestamp;
	differential_velocity_setpoint.speed = 0.f;
	differential_velocity_setpoint.bearing = _vehicle_yaw;
	_differential_velocity_setpoint_pub.publish(differential_velocity_setpoint);
}

bool DifferentialPosControl::isSourceInputNew(uint64_t timestamp) const
{
	return timestamp > _source_epoch && timestamp > _source_input_timestamp;
}

void DifferentialPosControl::updateSubscriptions()
{
	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&_vehicle_status);
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		const bool xy_reset = _local_position_timestamp != 0
				      && vehicle_local_position.xy_reset_counter != _xy_reset_counter;

		if (xy_reset) {
			_course_control = false;
			_auto_target_valid = false;
			const Vector2f delta_xy(vehicle_local_position.delta_xy[0], vehicle_local_position.delta_xy[1]);

			if (_rover_position_setpoint.timestamp > 0
			    && _rover_position_setpoint.timestamp < vehicle_local_position.timestamp) {
				Vector2f target_ned(_rover_position_setpoint.position_ned[0], _rover_position_setpoint.position_ned[1]);

				if (target_ned.isAllFinite() && delta_xy.isAllFinite()) {
					target_ned += delta_xy;
					_rover_position_setpoint.position_ned[0] = target_ned(0);
					_rover_position_setpoint.position_ned[1] = target_ned(1);

				} else {
					_rover_position_setpoint.position_ned[0] = NAN;
					_rover_position_setpoint.position_ned[1] = NAN;
				}
			}
		}

		const bool projection_ref_changed = _global_ned_proj_ref.isInitialized()
						    && _global_ned_proj_ref.getProjectionReferenceTimestamp() != vehicle_local_position.ref_timestamp;

		if (!_global_ned_proj_ref.isInitialized() || projection_ref_changed) {
			_global_ned_proj_ref.initReference(vehicle_local_position.ref_lat, vehicle_local_position.ref_lon,
							   vehicle_local_position.ref_timestamp);

			if (projection_ref_changed) {
				_auto_target_valid = false;
			}
		}

		_curr_pos_ned = Vector2f(vehicle_local_position.x, vehicle_local_position.y);
		_local_position_valid = vehicle_local_position.xy_valid && _curr_pos_ned.isAllFinite();
		_local_position_timestamp = vehicle_local_position.timestamp;
		_xy_reset_counter = vehicle_local_position.xy_reset_counter;
		const Vector3f velocity_in_local_frame(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		const Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
		_vehicle_speed_body_x = fabsf(velocity_in_body_frame(0)) > _param_ro_speed_th.get() ? velocity_in_body_frame(0) : 0.f;
	}

}

void DifferentialPosControl::resetPositionStatus(bool active)
{
	_rover_position_status = {};
	_rover_position_status.timestamp = _timestamp;
	_rover_position_status.active = active;
	_rover_position_status.xy_reset_counter = _xy_reset_counter;
	_rover_position_status.position_ned[0] = NAN;
	_rover_position_status.position_ned[1] = NAN;
	_rover_position_status.target_ned[0] = NAN;
	_rover_position_status.target_ned[1] = NAN;
	_rover_position_status.crosstrack_error = NAN;
	_rover_position_status.lookahead_distance = NAN;
	_rover_position_status.target_bearing = NAN;
	_rover_position_status.distance_to_waypoint = NAN;

	if (active && _local_position_valid && hrt_elapsed_time(&_local_position_timestamp) < 500_ms) {
		_rover_position_status.position_ned[0] = _curr_pos_ned(0);
		_rover_position_status.position_ned[1] = _curr_pos_ned(1);
		_rover_position_status.position_valid = true;
	}
}

void DifferentialPosControl::setPositionTarget(const Vector2f &target_ned)
{
	if (target_ned.isAllFinite()) {
		_rover_position_status.target_ned[0] = target_ned(0);
		_rover_position_status.target_ned[1] = target_ned(1);
		_rover_position_status.target_valid = true;
	}
}

void DifferentialPosControl::setPathStatus(const Vector2f &target_ned,
		const pure_pursuit_status_s &pure_pursuit_status)
{
	setPositionTarget(target_ned);

	if (_rover_position_status.position_valid && _rover_position_status.target_valid
	    && PX4_ISFINITE(pure_pursuit_status.crosstrack_error)
	    && PX4_ISFINITE(pure_pursuit_status.lookahead_distance)
	    && PX4_ISFINITE(pure_pursuit_status.target_bearing)
	    && PX4_ISFINITE(pure_pursuit_status.distance_to_waypoint)) {
		_rover_position_status.crosstrack_error = pure_pursuit_status.crosstrack_error;
		_rover_position_status.lookahead_distance = pure_pursuit_status.lookahead_distance;
		_rover_position_status.target_bearing = pure_pursuit_status.target_bearing;
		_rover_position_status.distance_to_waypoint = pure_pursuit_status.distance_to_waypoint;
		_rover_position_status.path_valid = true;
	}
}

void DifferentialPosControl::generatePositionSetpoint()
{
	offboard_control_mode_s offboard_control_mode{};

	if (_offboard_control_mode_sub.update(&offboard_control_mode)
	    && isSourceInputNew(offboard_control_mode.timestamp)
	    && offboard_control_mode.timestamp > _offboard_mode_timestamp) {
		_offboard_control_mode = offboard_control_mode;
		_offboard_mode_timestamp = offboard_control_mode.timestamp;

		if (!_offboard_control_mode.position) {
			_rover_position_setpoint = {};
			_rover_position_setpoint.position_ned[0] = NAN;
			_rover_position_setpoint.position_ned[1] = NAN;
		}
	}

	if (!_offboard_control_mode.position) {
		return;
	}

	trajectory_setpoint_s trajectory_setpoint{};

	if (!_trajectory_setpoint_sub.update(&trajectory_setpoint)
	    || !isSourceInputNew(trajectory_setpoint.timestamp)) {
		return;
	}

	_source_input_timestamp = trajectory_setpoint.timestamp;

	// Translate trajectory setpoint to rover position setpoint
	rover_position_setpoint_s rover_position_setpoint{};
	rover_position_setpoint.timestamp = _timestamp;
	rover_position_setpoint.position_ned[0] = trajectory_setpoint.position[0];
	rover_position_setpoint.position_ned[1] = trajectory_setpoint.position[1];
	rover_position_setpoint.cruising_speed = _param_ro_speed_limit.get();
	rover_position_setpoint.yaw = NAN;
	_rover_position_setpoint = rover_position_setpoint;
	_rover_position_setpoint_pub.publish(rover_position_setpoint);

}

void DifferentialPosControl::generateVelocitySetpoint()
{
	switch (_position_control_source) {
	case PositionControlSource::Manual:
		manualPositionMode();
		break;

	case PositionControlSource::Auto:
		autoPositionMode();
		break;

	case PositionControlSource::GoTo: {
			rover_position_setpoint_s rover_position_setpoint{};

			if (_rover_position_setpoint_sub.update(&rover_position_setpoint)
			    && isSourceInputNew(rover_position_setpoint.timestamp)) {
				_rover_position_setpoint = rover_position_setpoint;
				_source_input_timestamp = rover_position_setpoint.timestamp;
			}

			if (PX4_ISFINITE(_rover_position_setpoint.position_ned[0])
			    && PX4_ISFINITE(_rover_position_setpoint.position_ned[1])) {
				goToPositionMode();

			} else {
				publishStopSetpoint();
			}

			break;
		}

	case PositionControlSource::Offboard:
		if (PX4_ISFINITE(_rover_position_setpoint.position_ned[0])
		    && PX4_ISFINITE(_rover_position_setpoint.position_ned[1])) {
			goToPositionMode();

		} else {
			publishStopSetpoint();
		}

		break;

	case PositionControlSource::Inactive:
	default:
		publishStopSetpoint();
		break;
	}

}

void DifferentialPosControl::manualPositionMode()
{
	manual_control_setpoint_s new_manual_control_setpoint{};

	if (_manual_control_setpoint_sub.update(&new_manual_control_setpoint)
	    && isSourceInputNew(new_manual_control_setpoint.timestamp)) {
		_manual_control_setpoint = new_manual_control_setpoint;
		_manual_control_setpoint_valid = new_manual_control_setpoint.valid
						 && PX4_ISFINITE(new_manual_control_setpoint.pitch)
						 && PX4_ISFINITE(new_manual_control_setpoint.roll);
		_source_input_timestamp = new_manual_control_setpoint.timestamp;
	}

	if (!_manual_control_setpoint_valid) {
		_course_control = false;
		publishStopSetpoint();
		return;
	}

	const manual_control_setpoint_s &manual_control_setpoint = _manual_control_setpoint;

	const float speed_body_x_setpoint = math::interpolate<float>(manual_control_setpoint.pitch,
					    -1.f, 1.f, -_param_ro_speed_limit.get(), _param_ro_speed_limit.get());
	const float bearing_scaling = math::min(_max_yaw_rate / _param_ro_yaw_p.get(),
						_param_rd_trans_drv_trn.get() - FLT_EPSILON);
	const float bearing_delta = math::interpolate<float>(math::deadzone(manual_control_setpoint.roll,
				    _param_ro_yaw_stick_dz.get()), -1.f, 1.f, -bearing_scaling, bearing_scaling);

	if (fabsf(speed_body_x_setpoint) < FLT_EPSILON) { // Turn on spot
		_course_control = false;
		const float bearing_setpoint = matrix::wrap_pi(_vehicle_yaw + bearing_delta);
		differential_velocity_setpoint_s differential_velocity_setpoint{};
		differential_velocity_setpoint.timestamp = _timestamp;
		differential_velocity_setpoint.speed = 0.f;
		differential_velocity_setpoint.bearing = bearing_setpoint;
		_differential_velocity_setpoint_pub.publish(differential_velocity_setpoint);

	} else if (fabsf(bearing_delta) > FLT_EPSILON) { // Closed loop yaw rate control
		_course_control = false;
		const float bearing_setpoint = matrix::wrap_pi(_vehicle_yaw + bearing_delta);
		differential_velocity_setpoint_s differential_velocity_setpoint{};
		differential_velocity_setpoint.timestamp = _timestamp;
		differential_velocity_setpoint.speed = speed_body_x_setpoint;
		differential_velocity_setpoint.bearing = bearing_setpoint;
		_differential_velocity_setpoint_pub.publish(differential_velocity_setpoint);

	} else { // Course control if the steering input is zero (keep driving on a straight line)
		if (!_course_control) {
			_pos_ctl_course_direction = Vector2f(cos(_vehicle_yaw), sin(_vehicle_yaw));
			_pos_ctl_start_position_ned = _curr_pos_ned;
			_course_control = true;
		}

		// Construct a 'target waypoint' for course control s.t. it is never within the maximum lookahead of the rover
		const Vector2f start_to_curr_pos = _curr_pos_ned - _pos_ctl_start_position_ned;
		const float vector_scaling = fabsf(start_to_curr_pos * _pos_ctl_course_direction) + _param_pp_lookahd_max.get();
		const Vector2f target_waypoint_ned = _pos_ctl_start_position_ned + sign(speed_body_x_setpoint) *
						     vector_scaling * _pos_ctl_course_direction;
		pure_pursuit_status_s pure_pursuit_status{};
		pure_pursuit_status.timestamp = _timestamp;
		const float bearing_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					       _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), target_waypoint_ned, _pos_ctl_start_position_ned,
					       _curr_pos_ned, fabsf(speed_body_x_setpoint));
		_pure_pursuit_status_pub.publish(pure_pursuit_status);
		setPathStatus(target_waypoint_ned, pure_pursuit_status);
		differential_velocity_setpoint_s differential_velocity_setpoint{};
		differential_velocity_setpoint.timestamp = _timestamp;
		differential_velocity_setpoint.speed = speed_body_x_setpoint;
		differential_velocity_setpoint.bearing = speed_body_x_setpoint > -FLT_EPSILON ? bearing_setpoint : matrix::wrap_pi(
					bearing_setpoint + M_PI_F);
		_differential_velocity_setpoint_pub.publish(differential_velocity_setpoint);
	}
}

void DifferentialPosControl::autoPositionMode()
{
	position_setpoint_triplet_s position_setpoint_triplet{};

	if (_position_setpoint_triplet_sub.update(&position_setpoint_triplet)
	    && isSourceInputNew(position_setpoint_triplet.timestamp)) {
		_source_input_timestamp = position_setpoint_triplet.timestamp;
		_curr_wp_type = position_setpoint_triplet.current.type;

		RoverControl::globalToLocalSetpointTriplet(_curr_wp_ned, _prev_wp_ned, _next_wp_ned, position_setpoint_triplet,
				_curr_pos_ned, _global_ned_proj_ref);
		_auto_target_valid = position_setpoint_triplet.current.valid
				     && position_setpoint_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_IDLE
				     && _curr_wp_ned.isAllFinite();

		if (_auto_target_valid) {
			_waypoint_transition_angle = RoverControl::calcWaypointTransitionAngle(_prev_wp_ned, _curr_wp_ned, _next_wp_ned);

			// Waypoint cruising speed
			_cruising_speed = position_setpoint_triplet.current.cruising_speed > 0.f ? math::constrain(
						  position_setpoint_triplet.current.cruising_speed, 0.f, _param_ro_speed_limit.get()) : _param_ro_speed_limit.get();

		} else {
			_curr_wp_ned = Vector2f(NAN, NAN);
			_prev_wp_ned = Vector2f(NAN, NAN);
			_next_wp_ned = Vector2f(NAN, NAN);
			_waypoint_transition_angle = NAN;
			_cruising_speed = 0.f;
		}
	}

	if (!_auto_target_valid) {
		differential_velocity_setpoint_s differential_velocity_setpoint{};
		differential_velocity_setpoint.timestamp = _timestamp;
		differential_velocity_setpoint.speed = 0.f;
		differential_velocity_setpoint.bearing = _vehicle_yaw;
		_differential_velocity_setpoint_pub.publish(differential_velocity_setpoint);
		return;
	}

	// Distances to waypoints
	const float distance_to_curr_wp = sqrt(powf(_curr_pos_ned(0) - _curr_wp_ned(0),
					       2) + powf(_curr_pos_ned(1) - _curr_wp_ned(1), 2));

	// Check stopping conditions
	bool auto_stop{false};

	if (_curr_wp_type == position_setpoint_s::SETPOINT_TYPE_LAND
	    || !_next_wp_ned.isAllFinite()) { // Check stopping conditions
		auto_stop = distance_to_curr_wp < _param_nav_acc_rad.get();
	}

	if (auto_stop) {
		differential_velocity_setpoint_s differential_velocity_setpoint{};
		differential_velocity_setpoint.timestamp = _timestamp;
		differential_velocity_setpoint.speed = 0.f;
		differential_velocity_setpoint.bearing = _vehicle_yaw;
		_differential_velocity_setpoint_pub.publish(differential_velocity_setpoint);

	} else {
		setPositionTarget(_curr_wp_ned);
		const float speed_body_x_setpoint = calcSpeedSetpoint(_cruising_speed, distance_to_curr_wp, _param_ro_decel_limit.get(),
						    _param_ro_jerk_limit.get(), _waypoint_transition_angle, _param_ro_speed_limit.get(), _param_rd_trans_drv_trn.get(),
						    _param_rd_miss_spd_gain.get(), _curr_wp_type);
		pure_pursuit_status_s pure_pursuit_status{};
		pure_pursuit_status.timestamp = _timestamp;
		const float bearing_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					       _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), _curr_wp_ned, _prev_wp_ned, _curr_pos_ned,
					       fabsf(speed_body_x_setpoint));
		_pure_pursuit_status_pub.publish(pure_pursuit_status);

		if (_auto_target_valid) {
			setPathStatus(_curr_wp_ned, pure_pursuit_status);
		}

		differential_velocity_setpoint_s differential_velocity_setpoint{};
		differential_velocity_setpoint.timestamp = _timestamp;
		differential_velocity_setpoint.speed = speed_body_x_setpoint;
		differential_velocity_setpoint.bearing = bearing_setpoint;
		_differential_velocity_setpoint_pub.publish(differential_velocity_setpoint);
	}

}

float DifferentialPosControl::calcSpeedSetpoint(const float cruising_speed, const float distance_to_curr_wp,
		const float max_decel, const float max_jerk, const float waypoint_transition_angle, const float max_speed,
		const float trans_drv_trn, const float miss_spd_gain, int curr_wp_type)
{
	// Upcoming stop
	if (max_decel > FLT_EPSILON && max_jerk > FLT_EPSILON && (!PX4_ISFINITE(waypoint_transition_angle)
			|| _waypoint_transition_angle < M_PI_F - trans_drv_trn || curr_wp_type == position_setpoint_s::SETPOINT_TYPE_LAND
			|| curr_wp_type == position_setpoint_s::SETPOINT_TYPE_IDLE)) {
		const float straight_line_speed = math::trajectory::computeMaxSpeedFromDistance(max_jerk,
						  max_decel, distance_to_curr_wp, 0.f);
		return math::min(straight_line_speed, cruising_speed);
	}

	// Straight line speed
	if (max_jerk > FLT_EPSILON && max_decel > FLT_EPSILON && miss_spd_gain > FLT_EPSILON) {
		const float speed_reduction = math::constrain(miss_spd_gain * math::interpolate(M_PI_F - _waypoint_transition_angle,
					      0.f, M_PI_F, 0.f, 1.f), 0.f, 1.f);
		const float straight_line_speed = math::trajectory::computeMaxSpeedFromDistance(max_jerk, max_decel,
						  distance_to_curr_wp,
						  max_speed * (1.f - speed_reduction));
		return math::min(straight_line_speed, cruising_speed);
	}

	return cruising_speed; // Fallthrough

}

void DifferentialPosControl::goToPositionMode()
{
	const Vector2f target_waypoint_ned(_rover_position_setpoint.position_ned[0], _rover_position_setpoint.position_ned[1]);
	const float distance_to_target = (target_waypoint_ned - _curr_pos_ned).norm();

	if (distance_to_target > _param_nav_acc_rad.get()) {
		setPositionTarget(target_waypoint_ned);
		const float speed_setpoint = math::trajectory::computeMaxSpeedFromDistance(_param_ro_jerk_limit.get(),
					     _param_ro_decel_limit.get(), distance_to_target, 0.f);
		const float max_speed = PX4_ISFINITE(_rover_position_setpoint.cruising_speed) ?
					_rover_position_setpoint.cruising_speed :
					_param_ro_speed_limit.get();
		const float speed_body_x_setpoint = math::min(speed_setpoint, max_speed);
		pure_pursuit_status_s pure_pursuit_status{};
		pure_pursuit_status.timestamp = _timestamp;
		const float bearing_setpoint = PurePursuit::calcTargetBearing(pure_pursuit_status, _param_pp_lookahd_gain.get(),
					       _param_pp_lookahd_max.get(), _param_pp_lookahd_min.get(), target_waypoint_ned, _curr_pos_ned,
					       _curr_pos_ned, fabsf(speed_body_x_setpoint));
		_pure_pursuit_status_pub.publish(pure_pursuit_status);
		setPathStatus(target_waypoint_ned, pure_pursuit_status);
		differential_velocity_setpoint_s differential_velocity_setpoint{};
		differential_velocity_setpoint.timestamp = _timestamp;
		differential_velocity_setpoint.speed = speed_body_x_setpoint;
		differential_velocity_setpoint.bearing = bearing_setpoint;
		_differential_velocity_setpoint_pub.publish(differential_velocity_setpoint);

	} else {
		differential_velocity_setpoint_s differential_velocity_setpoint{};
		differential_velocity_setpoint.timestamp = _timestamp;
		differential_velocity_setpoint.speed = 0.f;
		differential_velocity_setpoint.bearing = _vehicle_yaw;
		_differential_velocity_setpoint_pub.publish(differential_velocity_setpoint);
	}
}

bool DifferentialPosControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_yaw_rate_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;
	}

	_prev_param_check_passed = ret;
	return ret;
}

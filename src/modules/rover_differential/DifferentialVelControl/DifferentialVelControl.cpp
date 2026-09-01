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

#include "DifferentialVelControl.hpp"

using namespace time_literals;

DifferentialVelControl::DifferentialVelControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_throttle_setpoint_pub.advertise();
	_rover_attitude_setpoint_pub.advertise();
	_differential_velocity_setpoint_pub.advertise();
	_rover_velocity_status_pub.advertise();
	updateParams();
}

void DifferentialVelControl::updateParams()
{
	ModuleParams::updateParams();
	_pid_speed.setGains(_param_ro_speed_p.get(), _param_ro_speed_i.get(), 0.f);
	_pid_speed.setIntegralLimit(1.f);
	_pid_speed.setOutputLimit(1.f);

	if (_param_ro_accel_limit.get() > FLT_EPSILON) {
		_speed_setpoint.setSlewRate(_param_ro_accel_limit.get());
	}
}

void DifferentialVelControl::resetInactiveState()
{
	_timestamp = hrt_absolute_time();
	_dt = 0.f;
	_vehicle_control_mode = {};
	_vehicle_attitude_timestamp = 0;
	_vehicle_velocity_timestamp = 0;
	_vehicle_velocity_valid = false;
	_vehicle_speed_body_x = 0.f;
	_vehicle_speed_body_y = 0.f;
	_pid_speed.resetIntegral();
	_speed_setpoint.setForcedValue(0.f);
	_speed_body_x_setpoint = 0.f;
	_current_state = DrivingState::DRIVING;
	_differential_velocity_setpoint = {};
	_rover_steering_setpoint = {};
	_rover_velocity_setpoint = {};
	_offboard_control_mode = {};
	_awaiting_trajectory_setpoint = true;
	_awaiting_velocity_setpoint = true;
	_awaiting_attitude_sample = true;
	_awaiting_velocity_sample = true;

	trajectory_setpoint_s discarded_trajectory{};
	vehicle_control_mode_s discarded_control_mode{};
	offboard_control_mode_s discarded_offboard_mode{};
	differential_velocity_setpoint_s discarded_velocity_setpoint{};
	rover_steering_setpoint_s discarded_steering_setpoint{};
	rover_velocity_setpoint_s discarded_rover_velocity_setpoint{};
	_trajectory_setpoint_sub.update(&discarded_trajectory);
	_offboard_control_mode_sub.update(&discarded_offboard_mode);
	_differential_velocity_setpoint_sub.update(&discarded_velocity_setpoint);
	_rover_steering_setpoint_sub.update(&discarded_steering_setpoint);
	_rover_velocity_setpoint_sub.update(&discarded_rover_velocity_setpoint);
	_vehicle_control_mode_sub.update(&discarded_control_mode);
	vehicle_attitude_s discarded_attitude{};
	vehicle_local_position_s discarded_local_position{};
	_vehicle_attitude_sub.update(&discarded_attitude);
	_vehicle_local_position_sub.update(&discarded_local_position);
}

void DifferentialVelControl::updateVelControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	updateSubscriptions();
	const bool rover_velocity_control_active = roverVelocityControlActive();

	bool controller_active{false};

	if (rover_velocity_control_active && _vehicle_control_mode.flag_armed) {
		const bool sanity_checks_passed = runSanityChecks();

		if (_vehicle_control_mode.flag_control_velocity_enabled && sanity_checks_passed
		    && !_awaiting_attitude_sample && !_awaiting_velocity_sample && roverVelocityInputValid()) {
			controller_active = generateThrottleSetpoint(_rover_velocity_setpoint.speed_body_x);

		} else {
			stopRoverVelocityControl();
		}

	} else if (_vehicle_control_mode.flag_control_velocity_enabled && _vehicle_control_mode.flag_armed) {
		const bool sanity_checks_passed = runSanityChecks();

		if (sanity_checks_passed && !_awaiting_attitude_sample && !_awaiting_velocity_sample) {
			if (_vehicle_control_mode.flag_control_offboard_enabled) { // Offboard Velocity Control
				generateVelocitySetpoint();
			}

			controller_active = generateAttitudeAndThrottleSetpoint();

		} else {
			_pid_speed.resetIntegral();
			_speed_setpoint.setForcedValue(0.f);
			_speed_body_x_setpoint = 0.f;
		}

	} else { // Reset controller and slew rate when velocity control is not active
		_pid_speed.resetIntegral();
		_speed_setpoint.setForcedValue(0.f);
		_speed_body_x_setpoint = 0.f;
	}

	// Publish position controller status (logging only)
	rover_velocity_status_s rover_velocity_status{};
	rover_velocity_status.timestamp = _timestamp;
	const bool velocity_valid = _vehicle_velocity_valid && _vehicle_velocity_timestamp > 0
				    && hrt_elapsed_time(&_vehicle_velocity_timestamp) < 500_ms
				    && _vehicle_attitude_timestamp > 0
				    && hrt_elapsed_time(&_vehicle_attitude_timestamp) < 500_ms;
	rover_velocity_status.measured_speed_body_x = velocity_valid ? _vehicle_speed_body_x : NAN;
	rover_velocity_status.speed_body_x_setpoint = _speed_body_x_setpoint;
	rover_velocity_status.adjusted_speed_body_x_setpoint = _speed_setpoint.getState();
	rover_velocity_status.measured_speed_body_y = velocity_valid ? _vehicle_speed_body_y : NAN;
	rover_velocity_status.speed_body_y_setpoint = NAN;
	rover_velocity_status.adjusted_speed_body_y_setpoint = NAN;
	rover_velocity_status.pid_throttle_body_x_integral = _pid_speed.getIntegral();
	rover_velocity_status.pid_throttle_body_y_integral = NAN;
	rover_velocity_status.active = controller_active;
	_rover_velocity_status_pub.publish(rover_velocity_status);
}
void DifferentialVelControl::updateSubscriptions()
{
	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);
		_vehicle_attitude_timestamp = vehicle_attitude.timestamp_sample;
		_vehicle_attitude_quaternion = matrix::Quatf(vehicle_attitude.q);
		_vehicle_yaw = matrix::Eulerf(_vehicle_attitude_quaternion).psi();
		_awaiting_attitude_sample = !PX4_ISFINITE(_vehicle_yaw);
	}

	if (_vehicle_local_position_sub.updated()) {
		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);
		_vehicle_velocity_timestamp = vehicle_local_position.timestamp_sample;
		_vehicle_velocity_valid = vehicle_local_position.v_xy_valid;
		const Vector3f velocity_in_local_frame(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
		const Vector3f velocity_in_body_frame = _vehicle_attitude_quaternion.rotateVectorInverse(velocity_in_local_frame);
		_vehicle_speed_body_x = fabsf(velocity_in_body_frame(0)) > _param_ro_speed_th.get() ? velocity_in_body_frame(0) : 0.f;
		_vehicle_speed_body_y = fabsf(velocity_in_body_frame(1)) > _param_ro_speed_th.get() ? velocity_in_body_frame(1) : 0.f;
		_awaiting_velocity_sample = !_vehicle_velocity_valid || !velocity_in_body_frame.isAllFinite();
	}

	if (_offboard_control_mode_sub.updated()) {
		_offboard_control_mode_sub.copy(&_offboard_control_mode);
	}

	if (_rover_velocity_setpoint_sub.updated()) {
		_rover_velocity_setpoint_sub.copy(&_rover_velocity_setpoint);
	}

	if (_hybrid_vehicle_status_sub.updated()) {
		_hybrid_vehicle_status_sub.copy(&_hybrid_vehicle_status);
	}

	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&_vehicle_status);
	}

}

void DifferentialVelControl::generateVelocitySetpoint()
{
	trajectory_setpoint_s trajectory_setpoint{};
	bool trajectory_available = _trajectory_setpoint_sub.update(&trajectory_setpoint);

	if (trajectory_available) {
		_awaiting_trajectory_setpoint = false;

	} else if (!_awaiting_trajectory_setpoint) {
		trajectory_available = _trajectory_setpoint_sub.copy(&trajectory_setpoint);
	}

	const bool offboard_vel_control = _offboard_control_mode.velocity && !_offboard_control_mode.position;

	const Vector2f velocity_in_local_frame(trajectory_setpoint.velocity[0], trajectory_setpoint.velocity[1]);

	if (trajectory_available && offboard_vel_control && velocity_in_local_frame.isAllFinite()) {
		differential_velocity_setpoint_s differential_velocity_setpoint{};
		differential_velocity_setpoint.timestamp = _timestamp;
		differential_velocity_setpoint.speed = velocity_in_local_frame.norm();
		differential_velocity_setpoint.bearing = atan2f(velocity_in_local_frame(1), velocity_in_local_frame(0));
		_differential_velocity_setpoint_pub.publish(differential_velocity_setpoint);
	}
}

bool DifferentialVelControl::generateAttitudeAndThrottleSetpoint()
{
	if (_differential_velocity_setpoint_sub.updated()) {
		_differential_velocity_setpoint_sub.copy(&_differential_velocity_setpoint);
		_awaiting_velocity_setpoint = false;
	}

	if (_awaiting_velocity_setpoint) {
		return false;
	}

	// Attitude Setpoint
	rover_attitude_setpoint_s rover_attitude_setpoint{};
	rover_attitude_setpoint.timestamp = _timestamp;
	rover_attitude_setpoint.yaw_setpoint = _differential_velocity_setpoint.bearing;
	_rover_attitude_setpoint_pub.publish(rover_attitude_setpoint);

	// Throttle Setpoint
	const float heading_error = matrix::wrap_pi(_differential_velocity_setpoint.bearing - _vehicle_yaw);

	if (_current_state == DrivingState::DRIVING && fabsf(heading_error) > _param_rd_trans_drv_trn.get()) {
		_current_state = DrivingState::SPOT_TURNING;

	} else if (_current_state == DrivingState::SPOT_TURNING && fabsf(heading_error) < _param_rd_trans_trn_drv.get()) {
		_current_state = DrivingState::DRIVING;
	}

	float speed_body_x_setpoint = 0.f;

	if (_current_state == DrivingState::DRIVING) {
		speed_body_x_setpoint = math::constrain(_differential_velocity_setpoint.speed, -_param_ro_speed_limit.get(),
							_param_ro_speed_limit.get());

	}

	return generateThrottleSetpoint(speed_body_x_setpoint);
}

bool DifferentialVelControl::generateThrottleSetpoint(float speed_body_x_setpoint)
{
	speed_body_x_setpoint = math::constrain(speed_body_x_setpoint, -_param_ro_speed_limit.get(),
						_param_ro_speed_limit.get());
	const float speed_body_x_setpoint_normalized = math::interpolate<float>(speed_body_x_setpoint,
			-_param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get(), -1.f, 1.f);

	if (_rover_steering_setpoint_sub.updated()) {
		_rover_steering_setpoint_sub.copy(&_rover_steering_setpoint);
	}

	if (fabsf(speed_body_x_setpoint_normalized) > 1.f - fabsf(
		    _rover_steering_setpoint.normalized_speed_diff)) { // Adjust an infeasible speed demand for the steering demand.
		speed_body_x_setpoint = math::interpolate<float>(sign(speed_body_x_setpoint_normalized) * (1.f - fabsf(
						_rover_steering_setpoint.normalized_speed_diff)), -1.f, 1.f,
					- _param_ro_max_thr_speed.get(), _param_ro_max_thr_speed.get());
	}

	_speed_body_x_setpoint = speed_body_x_setpoint;

	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = _timestamp;
	rover_throttle_setpoint.throttle_body_x = RoverControl::speedControl(_speed_setpoint, _pid_speed,
			speed_body_x_setpoint, _vehicle_speed_body_x, _param_ro_accel_limit.get(), _param_ro_decel_limit.get(),
			_param_ro_max_thr_speed.get(), _dt);
	rover_throttle_setpoint.throttle_body_y = 0.f;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
	return true;

}

void DifferentialVelControl::stopRoverVelocityControl()
{
	_pid_speed.resetIntegral();
	_speed_setpoint.setForcedValue(0.f);
	_speed_body_x_setpoint = 0.f;

	rover_throttle_setpoint_s rover_throttle_setpoint{};
	rover_throttle_setpoint.timestamp = _timestamp;
	rover_throttle_setpoint.throttle_body_x = 0.f;
	rover_throttle_setpoint.throttle_body_y = 0.f;
	_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
}

bool DifferentialVelControl::roverVelocityControlActive() const
{
	const bool legacy_velocity_control_active = _vehicle_control_mode.flag_control_velocity_enabled
			&& !_vehicle_control_mode.flag_control_position_enabled
			&& !_vehicle_control_mode.flag_control_altitude_enabled
			&& !_vehicle_control_mode.flag_control_climb_rate_enabled
			&& !_vehicle_control_mode.flag_control_acceleration_enabled
			&& !_vehicle_control_mode.flag_control_attitude_enabled
			&& _vehicle_control_mode.flag_control_rates_enabled
			&& _vehicle_control_mode.flag_control_allocation_enabled;

	return roverVelocityDedicatedControlRequired(_vehicle_status.is_quad_rover,
			_vehicle_control_mode.flag_control_offboard_enabled, _offboard_control_mode.rover_velocity,
			legacy_velocity_control_active);
}

bool DifferentialVelControl::roverVelocityInputValid() const
{
	const hrt_abstime maximum_age = static_cast<hrt_abstime>(_param_com_of_loss_t.get() * 1_s);
	const RoverVelocityOffboardMode mode{_offboard_control_mode.timestamp,
					     _offboard_control_mode.position, _offboard_control_mode.velocity, _offboard_control_mode.acceleration,
					     _offboard_control_mode.attitude, _offboard_control_mode.body_rate,
					     _offboard_control_mode.thrust_and_torque, _offboard_control_mode.direct_actuator,
					     _offboard_control_mode.rover_velocity};
	const RoverVelocityDrivingStatus status{_hybrid_vehicle_status.timestamp,
						_hybrid_vehicle_status.transition_completed_timestamp,
						_hybrid_vehicle_status.current_state == hybrid_vehicle_status_s::HYBRID_STATE_DRIVING,
						_hybrid_vehicle_status.fault_reason == hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE};
	const RoverVelocityOffboardInput input{_rover_velocity_setpoint.timestamp,
					       _rover_velocity_setpoint.speed_body_x, _rover_velocity_setpoint.yaw_rate};
	const bool driving_healthy = roverDrivingStatusUsable(status, _timestamp, 1_s);

	return roverVelocityModeUsable(mode, _timestamp, maximum_age)
	       && roverVelocityInputUsable(input, status.transition_completed_timestamp, _timestamp, maximum_age,
					   driving_healthy);

}

bool DifferentialVelControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_speed_limit.get() < FLT_EPSILON) {
		ret = false;

		if (_prev_param_check_passed) {
			events::send<float>(events::ID("differential_posVel_control_conf_invalid_speed_lim"), events::Log::Error,
					    "Invalid configuration of necessary parameter RO_SPEED_LIM", _param_ro_speed_limit.get());
		}

	}

	if (_param_ro_max_thr_speed.get() < FLT_EPSILON && _param_ro_speed_p.get() < FLT_EPSILON) {
		ret = false;

		if (_prev_param_check_passed) {
			events::send<float, float>(events::ID("differential_posVel_control_conf_invalid_speed_control"), events::Log::Error,
						   "Invalid configuration for speed control: Neither feed forward (RO_MAX_THR_SPEED) nor feedback (RO_SPEED_P) is setup",
						   _param_ro_max_thr_speed.get(),
						   _param_ro_speed_p.get());
		}
	}

	_prev_param_check_passed = ret;
	return ret;
}

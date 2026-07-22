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

#include "DifferentialRateControl.hpp"

using namespace time_literals;

DifferentialRateControl::DifferentialRateControl(ModuleParams *parent) : ModuleParams(parent)
{
	_rover_rate_setpoint_pub.advertise();
	_rover_throttle_setpoint_pub.advertise();
	_rover_steering_setpoint_pub.advertise();
	_rover_rate_status_pub.advertise();
	updateParams();
}

void DifferentialRateControl::updateParams()
{
	ModuleParams::updateParams();
	_max_yaw_rate = _param_ro_yaw_rate_limit.get() * M_DEG_TO_RAD_F;
	_max_yaw_accel = _param_ro_yaw_accel_limit.get() * M_DEG_TO_RAD_F;
	_max_yaw_decel = _param_ro_yaw_decel_limit.get() * M_DEG_TO_RAD_F;
	_pid_yaw_rate.setGains(_param_ro_yaw_rate_p.get(), _param_ro_yaw_rate_i.get(), 0.f);
	_pid_yaw_rate.setIntegralLimit(1.f);
	_pid_yaw_rate.setOutputLimit(1.f);
	_adjusted_yaw_rate_setpoint.setSlewRate(_max_yaw_accel);
}

void DifferentialRateControl::updateRateControl()
{
	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	if (_vehicle_angular_velocity_sub.updated()) {
		vehicle_angular_velocity_s vehicle_angular_velocity{};
		_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);
		_vehicle_yaw_rate = fabsf(vehicle_angular_velocity.xyz[2]) > _param_ro_yaw_rate_th.get() * M_DEG_TO_RAD_F ?
				    vehicle_angular_velocity.xyz[2] : 0.f;
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

	const bool rover_velocity_control_active = roverVelocityControlActive();

	if (rover_velocity_control_active && _vehicle_control_mode.flag_armed) {
		const bool sanity_checks_passed = runSanityChecks();

		if (_vehicle_control_mode.flag_control_rates_enabled && sanity_checks_passed && roverVelocityInputValid()) {
			rover_rate_setpoint_s rover_rate_setpoint{};
			rover_rate_setpoint.timestamp = _timestamp;
			rover_rate_setpoint.yaw_rate_setpoint = _rover_velocity_setpoint.yaw_rate;
			_rover_rate_setpoint = rover_rate_setpoint;
			_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
			generateSteeringSetpoint(false);

		} else {
			stopRoverVelocityControl();
		}

	} else if (_vehicle_control_mode.flag_control_rates_enabled && _vehicle_control_mode.flag_armed) {
		const bool sanity_checks_passed = runSanityChecks();

		if (sanity_checks_passed) {
			if (_vehicle_control_mode.flag_control_manual_enabled || _vehicle_control_mode.flag_control_offboard_enabled) {
				generateRateAndThrottleSetpoint();
			}
			generateSteeringSetpoint();

		} else {
			_pid_yaw_rate.resetIntegral();
			_adjusted_yaw_rate_setpoint.setForcedValue(0.f);
		}

	} else { // Reset controller and slew rate when rate control is not active
		_pid_yaw_rate.resetIntegral();
		_adjusted_yaw_rate_setpoint.setForcedValue(0.f);
	}

	// Publish rate controller status (logging only)
	rover_rate_status_s rover_rate_status;
	rover_rate_status.timestamp = _timestamp;
	rover_rate_status.measured_yaw_rate = _vehicle_yaw_rate;
	rover_rate_status.adjusted_yaw_rate_setpoint = _adjusted_yaw_rate_setpoint.getState();
	rover_rate_status.pid_yaw_rate_integral = _pid_yaw_rate.getIntegral();
	_rover_rate_status_pub.publish(rover_rate_status);

}

void DifferentialRateControl::generateRateAndThrottleSetpoint()
{
	const bool acro_mode_enabled = _vehicle_control_mode.flag_control_manual_enabled
				       && !_vehicle_control_mode.flag_control_position_enabled && !_vehicle_control_mode.flag_control_attitude_enabled;

	if (acro_mode_enabled && _manual_control_setpoint_sub.updated()) { // Acro Mode
		manual_control_setpoint_s manual_control_setpoint{};

		if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
			rover_throttle_setpoint_s rover_throttle_setpoint{};
			rover_throttle_setpoint.timestamp = _timestamp;
			rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.pitch;
			rover_throttle_setpoint.throttle_body_y = 0.f;
			_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
			rover_rate_setpoint_s rover_rate_setpoint{};
			rover_rate_setpoint.timestamp = _timestamp;
			rover_rate_setpoint.yaw_rate_setpoint = math::interpolate<float> (manual_control_setpoint.roll, -1.f, 1.f,
								-_max_yaw_rate, _max_yaw_rate);
			_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
		}

	} else if (_vehicle_control_mode.flag_control_offboard_enabled) { // Offboard rate control
		trajectory_setpoint_s trajectory_setpoint{};
		_trajectory_setpoint_sub.copy(&trajectory_setpoint);

		const bool offboard_rate_control = _offboard_control_mode.body_rate && !_offboard_control_mode.position
						   && !_offboard_control_mode.velocity && !_offboard_control_mode.attitude;

		if (offboard_rate_control && PX4_ISFINITE(trajectory_setpoint.yawspeed)) {
			rover_rate_setpoint_s rover_rate_setpoint{};
			rover_rate_setpoint.timestamp = _timestamp;
			rover_rate_setpoint.yaw_rate_setpoint = trajectory_setpoint.yawspeed;
			_rover_rate_setpoint_pub.publish(rover_rate_setpoint);
		}
	}
}

void DifferentialRateControl::generateSteeringSetpoint(bool apply_setpoint_threshold)
{
	if (_rover_rate_setpoint_sub.updated()) {
		_rover_rate_setpoint_sub.copy(&_rover_rate_setpoint);

	}

	float speed_diff_normalized{0.f};

	if (PX4_ISFINITE(_rover_rate_setpoint.yaw_rate_setpoint) && PX4_ISFINITE(_vehicle_yaw_rate)) {
		const float yaw_rate_setpoint = !apply_setpoint_threshold
						|| fabsf(_rover_rate_setpoint.yaw_rate_setpoint) > _param_ro_yaw_rate_th.get() * M_DEG_TO_RAD_F
						? _rover_rate_setpoint.yaw_rate_setpoint : 0.f;
		speed_diff_normalized = RoverControl::rateControl(_adjusted_yaw_rate_setpoint, _pid_yaw_rate,
					yaw_rate_setpoint, _vehicle_yaw_rate, _param_rd_max_thr_yaw_r.get(), _max_yaw_accel,
					_max_yaw_decel, _param_rd_wheel_track.get(), _dt);
	}

	if (_vehicle_control_mode.flag_control_manual_enabled
	    && _vehicle_control_mode.flag_control_rates_enabled
	    && fabsf(_rover_rate_setpoint.yaw_rate_setpoint) > 0.05f) {
		static hrt_abstime last_debug_print{0};

		if (hrt_elapsed_time(&last_debug_print) > 250_ms) {
			last_debug_print = hrt_absolute_time();
			PX4_INFO("[RD_RATE_DBG] yaw_sp=%.3f yaw=%.3f diff=%.3f int=%.3f",
				 (double)_rover_rate_setpoint.yaw_rate_setpoint,
				 (double)_vehicle_yaw_rate,
				 (double)speed_diff_normalized,
				 (double)_pid_yaw_rate.getIntegral());
		}
	}

	rover_steering_setpoint_s rover_steering_setpoint{};
	rover_steering_setpoint.timestamp = _timestamp;
	rover_steering_setpoint.normalized_speed_diff = speed_diff_normalized;
	_rover_steering_setpoint_pub.publish(rover_steering_setpoint);
}

void DifferentialRateControl::stopRoverVelocityControl()
{
	_pid_yaw_rate.resetIntegral();
	_adjusted_yaw_rate_setpoint.setForcedValue(0.f);
	_rover_rate_setpoint = {};
	_rover_rate_setpoint.timestamp = _timestamp;
	_rover_rate_setpoint.yaw_rate_setpoint = 0.f;
	_rover_rate_setpoint_pub.publish(_rover_rate_setpoint);

	rover_steering_setpoint_s rover_steering_setpoint{};
	rover_steering_setpoint.timestamp = _timestamp;
	rover_steering_setpoint.normalized_speed_diff = 0.f;
	_rover_steering_setpoint_pub.publish(rover_steering_setpoint);
}

bool DifferentialRateControl::roverVelocityControlActive() const
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

bool DifferentialRateControl::roverVelocityInputValid() const
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

bool DifferentialRateControl::runSanityChecks()
{
	bool ret = true;

	if (_param_ro_yaw_rate_limit.get() < FLT_EPSILON) {
		ret = false;

		if (_prev_param_check_passed) {
			events::send<float>(events::ID("differential_rate_control_conf_invalid_yaw_rate_lim"), events::Log::Error,
					    "Invalid configuration of necessary parameter RO_YAW_RATE_LIM", _param_ro_yaw_rate_limit.get());
		}

	}

	if ((_param_rd_wheel_track.get() < FLT_EPSILON || _param_rd_max_thr_yaw_r.get() < FLT_EPSILON)
	    && _param_ro_yaw_rate_p.get() < FLT_EPSILON) {
		ret = false;

		if (_prev_param_check_passed) {
			events::send<float, float, float>(events::ID("differential_rate_control_conf_invalid_rate_control"), events::Log::Error,
							  "Invalid configuration for rate control: Neither feed forward nor feedback is setup", _param_rd_wheel_track.get(),
							  _param_rd_max_thr_yaw_r.get(), _param_ro_yaw_rate_p.get());
		}
	}

	_prev_param_check_passed = ret;
	return ret;
}

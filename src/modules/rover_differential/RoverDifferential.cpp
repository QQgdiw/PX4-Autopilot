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

#include "RoverDifferential.hpp"

using namespace time_literals;

RoverDifferential::RoverDifferential() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_rover_throttle_setpoint_pub.advertise();
	_rover_steering_setpoint_pub.advertise();
	updateParams();
}

bool RoverDifferential::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz
	return true;
}

void RoverDifferential::updateParams()
{
	ModuleParams::updateParams();

	if (_param_ro_accel_limit.get() > FLT_EPSILON && _param_ro_max_thr_speed.get() > FLT_EPSILON) {
		_throttle_body_x_setpoint.setSlewRate(_param_ro_accel_limit.get() / _param_ro_max_thr_speed.get());
	}
}

void RoverDifferential::Run()
{
	vehicle_status_s status{};

	if (_vehicle_status_sub.copy(&status)) {
		_vehicle_status = status;

		if (status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROVER) {
			actuator_motors_s stop_motors{};
			stop_motors.timestamp = hrt_absolute_time();
			stop_motors.reversible_flags = _param_r_rev.get();
			for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
				stop_motors.control[i] = NAN; // NaN 在底层 PWM 驱动中代表断电/不驱动
			}
			_actuator_motors_pub.publish(stop_motors);
			return; // 立即退出，下方的所有解算逻辑统统跳过！
		}
	}

	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	const hrt_abstime timestamp_prev = _timestamp;
	_timestamp = hrt_absolute_time();
	_dt = math::constrain(_timestamp - timestamp_prev, 1_ms, 5000_ms) * 1e-6f;

	_differential_pos_control.updatePosControl();
	_differential_vel_control.updateVelControl();
	_differential_att_control.updateAttControl();
	_differential_rate_control.updateRateControl();

	if (_vehicle_control_mode_sub.updated()) {
		_vehicle_control_mode_sub.copy(&_vehicle_control_mode);
	}

	const bool full_manual_mode_enabled = _vehicle_control_mode.flag_control_manual_enabled
					      && !_vehicle_control_mode.flag_control_position_enabled && !_vehicle_control_mode.flag_control_attitude_enabled
					      && !_vehicle_control_mode.flag_control_rates_enabled;

	if (full_manual_mode_enabled) { // Manual mode
		generateSteeringAndThrottleSetpoint();
	}

	if (_vehicle_control_mode.flag_armed) {
		generateActuatorSetpoint();

	}

}

void RoverDifferential::generateSteeringAndThrottleSetpoint()
{
	manual_control_setpoint_s manual_control_setpoint{};

	if (_manual_control_setpoint_sub.update(&manual_control_setpoint)) {
		rover_steering_setpoint_s rover_steering_setpoint{};
		rover_steering_setpoint.timestamp = _timestamp;
		rover_steering_setpoint.normalized_speed_diff = RoverControl::manualSteeringInput(manual_control_setpoint.roll);
		_rover_steering_setpoint_pub.publish(rover_steering_setpoint);
		rover_throttle_setpoint_s rover_throttle_setpoint{};
		rover_throttle_setpoint.timestamp = _timestamp;
		rover_throttle_setpoint.throttle_body_x = manual_control_setpoint.pitch;
		rover_throttle_setpoint.throttle_body_y = 0.f;
		_rover_throttle_setpoint_pub.publish(rover_throttle_setpoint);
	}
}

void RoverDifferential::generateActuatorSetpoint()
{
	if (_rover_throttle_setpoint_sub.updated()) {
		_rover_throttle_setpoint_sub.copy(&_rover_throttle_setpoint);
	}

	if (_actuator_motors_sub.updated()) {
		actuator_motors_s actuator_motors{};
		_actuator_motors_sub.copy(&actuator_motors);
		_current_throttle_body_x = (actuator_motors.control[0] + actuator_motors.control[1]) / 2.f;
	}

	if (_rover_steering_setpoint_sub.updated()) {
		_rover_steering_setpoint_sub.copy(&_rover_steering_setpoint);
	}

	if (!PX4_ISFINITE(_current_throttle_body_x)) {
		_current_throttle_body_x = 0.0f;
	}

	const float throttle_body_x = RoverControl::throttleControl(_throttle_body_x_setpoint,
				      _rover_throttle_setpoint.throttle_body_x, _current_throttle_body_x, _param_ro_accel_limit.get(),
				      _param_ro_decel_limit.get(), _param_ro_max_thr_speed.get(), _dt);

	// // ==========================================================
	// // [NaN 猎手探针] 每 50 次循环打印一次关键变量，抓出毒药源头！
	// // ==========================================================
	// static int nan_debug_cnt = 0;
	// if (nan_debug_cnt++ % 50 == 0) {
	// 	mavlink_log_info(&_mavlink_log_pub,
	// 	"[Rover] T_SP:%.2f S_SP:%.2f cTX:%.2f fTX:%.2f",
	// 	(double)_rover_throttle_setpoint.throttle_body_x,
	// 	(double)_rover_steering_setpoint.normalized_speed_diff,
	// 	(double)_current_throttle_body_x,
	// 	(double)throttle_body_x);
	// }
	// // ==========================================================

	actuator_motors_s actuator_motors{};
	actuator_motors.reversible_flags = _param_r_rev.get();
	const Vector2f wheel_commands = computeInverseKinematics(throttle_body_x,
					_rover_steering_setpoint.normalized_speed_diff);
	wheel_commands.copyTo(actuator_motors.control);

	if (fabsf(_rover_steering_setpoint.normalized_speed_diff) > 0.05f) {
		static hrt_abstime last_debug_print{0};

		if (hrt_elapsed_time(&last_debug_print) > 250_ms) {
			last_debug_print = hrt_absolute_time();
			PX4_INFO("[RD_MIX_DBG] nav:%u type:%u quad:%d mode m:%d p:%d a:%d r:%d thr=%.3f steer=%.3f L=%.3f R=%.3f",
				 (unsigned)_vehicle_status.nav_state,
				 (unsigned)_vehicle_status.vehicle_type,
				 (int)_vehicle_status.is_quad_rover,
				 (int)_vehicle_control_mode.flag_control_manual_enabled,
				 (int)_vehicle_control_mode.flag_control_position_enabled,
				 (int)_vehicle_control_mode.flag_control_attitude_enabled,
				 (int)_vehicle_control_mode.flag_control_rates_enabled,
				 (double)throttle_body_x,
				 (double)_rover_steering_setpoint.normalized_speed_diff,
				 (double)wheel_commands(0),
				 (double)wheel_commands(1));
		}
	}

	actuator_motors.timestamp = _timestamp;
	_actuator_motors_pub.publish(actuator_motors);
}

Vector2f RoverDifferential::computeInverseKinematics(float throttle_body_x, const float speed_diff_normalized)
{
	float max_motor_command = fabsf(throttle_body_x) + fabsf(speed_diff_normalized);

	if (max_motor_command > 1.0f) { // Prioritize yaw rate if a normalized motor command exceeds limit of 1
		float excess = fabsf(max_motor_command - 1.0f);
		throttle_body_x -= sign(throttle_body_x) * excess;
	}

	// Calculate the left and right wheel speeds
	return Vector2f(throttle_body_x - speed_diff_normalized,
			throttle_body_x + speed_diff_normalized);
}

int RoverDifferential::task_spawn(int argc, char *argv[])
{
	RoverDifferential *instance = new RoverDifferential();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int RoverDifferential::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RoverDifferential::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Rover differential module.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rover_differential", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int rover_differential_main(int argc, char *argv[])
{
	return RoverDifferential::main(argc, argv);
}

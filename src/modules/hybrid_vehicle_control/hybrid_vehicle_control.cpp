/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "hybrid_vehicle_control.hpp"

#include <cmath>

#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>

using namespace time_literals;
using hybrid_control::HybridState;
using hybrid_control::HybridTarget;
using hybrid_control::SensorSource;
using hybrid_control::TransformFault;
using hybrid_control::TransformationConfig;
using hybrid_control::TransformationInput;

static_assert(static_cast<uint8_t>(TransformFault::InvalidConfiguration)
	      == hybrid_vehicle_status_s::TRANSFORM_FAULT_INVALID_CONFIGURATION,
	      "transformation fault values must match the public uORB contract");

namespace
{
static constexpr hrt_abstime MANUAL_CONTROL_TIMEOUT = 1_s;

bool timestamp_fresh(uint64_t timestamp, hrt_abstime now, uint64_t timeout_us)
{
	return timestamp != 0 && now >= timestamp && now - timestamp <= timeout_us;
}

float clamp_servo(float value)
{
	return fmaxf(-1.f, fminf(value, 1.f));
}
} // namespace

HybridVehicleControl::HybridVehicleControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();
}

HybridVehicleControl::~HybridVehicleControl() = default;

bool HybridVehicleControl::init()
{
	ScheduleOnInterval(20_ms);
	return true;
}

int HybridVehicleControl::task_spawn(int argc, char *argv[])
{
	HybridVehicleControl *instance = new HybridVehicleControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int HybridVehicleControl::custom_command(int argc, char *argv[])
{
	return print_usage("unrecognized command");
}

int HybridVehicleControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Hybrid vehicle control module (Quad-Rover).
Arbitrates native multicopter and differential-rover outputs and controls the
positional transformation servo.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hybrid_vehicle_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int hybrid_vehicle_control_main(int argc, char *argv[])
{
	return HybridVehicleControl::main(argc, argv);
}

TransformationConfig HybridVehicleControl::transformation_config() const
{
	return {
		_param_hyb_sens_en.get(),
		_param_hyb_boot_st.get(),
		_param_hyb_sv_qud.get(),
		_param_hyb_sv_rov.get(),
		_param_hybrid_ang_qud.get(),
		_param_hybrid_ang_rov.get(),
		_param_hyb_ang_tol.get(),
		_param_hyb_sens_to.get(),
		_param_hyb_dbnc_t.get(),
		_param_hybrid_trans_t.get(),
		_param_hyb_mag_id_qud.get(),
		_param_hyb_mag_id_rov.get(),
		_param_hyb_mag_thr_qud.get(),
		_param_hyb_mag_thr_rov.get()
	};
}

TransformationInput HybridVehicleControl::update_transformation_input(hrt_abstime now, const TransformationConfig &config)
{
	sensor_encoder_s encoder{};

	if (_encoder_sub.update(&encoder)) {
		_current_mechanism_angle = encoder.position_rad;
		_last_encoder_timestamp = encoder.timestamp;
		_encoder_healthy = encoder.status_flags == 0;
	}

	for (size_t i = 0; i < _magnetic_subs.size(); ++i) {
		magnetic_sensor_s magnetic{};

		if (_magnetic_subs[i].update(&magnetic)) {
			if (magnetic.device_id == static_cast<uint32_t>(config.tmag_quad_device_id)) {
				_tmag_quad_cache.update(magnetic.device_id, magnetic.mag_z, magnetic.timestamp);
			}

			if (magnetic.device_id == static_cast<uint32_t>(config.tmag_rover_device_id)) {
				_tmag_rover_cache.update(magnetic.device_id, magnetic.mag_z, magnetic.timestamp);
			}
		}
	}

	const uint64_t sensor_timeout_us = static_cast<uint64_t>(config.sensor_timeout_s * 1_s);
	const bool encoder_valid = timestamp_fresh(_last_encoder_timestamp, now, sensor_timeout_us)
				   && _encoder_healthy && std::isfinite(_current_mechanism_angle);
	const bool tmag_quad_valid = _tmag_quad_cache.validFor(config.tmag_quad_device_id, now, sensor_timeout_us);
	const bool tmag_rover_valid = _tmag_rover_cache.validFor(config.tmag_rover_device_id, now, sensor_timeout_us);

	return {
		now,
		encoder_valid,
		_current_mechanism_angle,
		tmag_quad_valid,
		tmag_quad_valid && fabsf(_tmag_quad_cache.value()) >= config.tmag_quad_threshold,
		tmag_rover_valid,
		tmag_rover_valid && fabsf(_tmag_rover_cache.value()) >= config.tmag_rover_threshold
	};
}

void HybridVehicleControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	updateParams();
	_actuator_armed_sub.update(&_actuator_armed);
	_vehicle_control_mode_sub.update(&_vcontrol_mode);

	const hrt_abstime now = hrt_absolute_time();
	const TransformationConfig requested_config = transformation_config();
	const bool safe_to_apply = !_actuator_armed.armed && !_actuator_armed.prearmed;
	const bool configuration_applied = _transformation_config_tracker.update(requested_config, safe_to_apply);
	TransformationInput input{now, false, 0.f, false, false, false, false};

	if (!_transformation_config_tracker.hasActive()) {
		publish_status(input, now);
		publish_servo(now);
		publish_motor_outputs(now);
		return;
	}

	const TransformationConfig &active_config = _transformation_config_tracker.active();
	const TransformFault configuration_fault = hybrid_control::validateTransformationConfig(active_config);

	if (configuration_fault == TransformFault::None) {
		input = update_transformation_input(now, active_config);
	}

	if (!_transformation_initialized || configuration_applied) {
		_transformation_output = _transformation.initialize(active_config, input);
		_transformation_initialized = true;
		_manual_commissioning_active = false;
		_transition_timing_active = false;
	}

	update_state_machine(input);
	publish_status(input, now);
	publish_servo(now);
	publish_motor_outputs(now);
}

void HybridVehicleControl::update_state_machine(const TransformationInput &input)
{
	bool request_rover = false;
	bool request_quad = false;

	vehicle_command_s command{};

	while (_vehicle_command_sub.update(&command)) {
		if (command.command != vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION) {
			continue;
		}

		const int target = static_cast<int>(command.param1 + 0.5f);
		request_rover = target == 4;
		request_quad = target == 3;

		vehicle_command_ack_s ack{};
		ack.timestamp = input.now_us;
		ack.command = command.command;
		ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
		ack.target_system = command.source_system;
		ack.target_component = command.source_component;
		_vehicle_command_ack_pub.publish(ack);
	}

	manual_control_setpoint_s manual{};

	if (_manual_control_setpoint_sub.update(&manual)) {
		float transfer_switch = -1.f;
		float manual_rc_value = 0.f;
		const bool manual_sample_fresh = timestamp_fresh(manual.timestamp, input.now_us, MANUAL_CONTROL_TIMEOUT);

		switch (_rc_map_trans_sw_val) {
		case 5: transfer_switch = manual.aux1; break;
		case 6: transfer_switch = manual.aux2; break;
		case 7: transfer_switch = manual.aux3; break;
		case 8: transfer_switch = manual.aux4; break;
		case 9: transfer_switch = manual.aux5; break;
		case 10: transfer_switch = manual.aux6; break;
		default: break;
		}

		switch (_param_hybrid_man_ch.get()) {
		case 1: manual_rc_value = manual.aux1; break;
		case 2: manual_rc_value = manual.aux2; break;
		case 3: manual_rc_value = manual.aux3; break;
		case 4: manual_rc_value = manual.aux4; break;
		case 5: manual_rc_value = manual.aux5; break;
		case 6: manual_rc_value = manual.aux6; break;
		default: manual_rc_value = 0.f; break;
		}

		_manual_control_cache.update(manual.timestamp, manual_rc_value, input.now_us, MANUAL_CONTROL_TIMEOUT);

		if (_manual_control_cache.fresh(input.now_us, MANUAL_CONTROL_TIMEOUT)) {
			const float current_manual_value = _manual_control_cache.value();

			if (_manual_value_initialized
			    && fabsf(current_manual_value - _last_manual_value) > 0.5f
			    && !_actuator_armed.armed && _actuator_armed.prearmed) {
				_manual_commissioning_active = true;
			}

			_manual_value_initialized = true;
			_last_manual_value = current_manual_value;
		}

		if (manual_sample_fresh && std::isfinite(transfer_switch)
		    && fabsf(transfer_switch - last_transfer_switch) > 0.5f) {
			_manual_commissioning_active = false;

			if (_vcontrol_mode.flag_control_auto_enabled) {
				vehicle_command_s mode_command{};
				mode_command.timestamp = input.now_us;
				mode_command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
				mode_command.param1 = 1.f;
				mode_command.param2 = 3.f;
				_vehicle_command_pub.publish(mode_command);
			}

			if (transfer_switch > 0.5f && last_transfer_switch <= 0.5f) {
				request_rover = true;
				request_quad = false;

			} else if (transfer_switch < -0.5f && last_transfer_switch >= -0.5f) {
				request_quad = true;
				request_rover = false;
			}
		}

		if (manual_sample_fresh && std::isfinite(transfer_switch)) {
			last_transfer_switch = transfer_switch;
		}
	}

	const bool manual_control_fresh = _manual_control_cache.fresh(input.now_us, MANUAL_CONTROL_TIMEOUT);

	if (!manual_control_fresh) {
		_manual_commissioning_active = false;
		_manual_value_initialized = false;
	}

	if (_actuator_armed.armed || !_actuator_armed.prearmed) {
		_manual_commissioning_active = false;
	}

	HybridTarget requested_target = HybridTarget::None;

	if (request_rover && check_safe_to_transform(true)) {
		requested_target = HybridTarget::Driving;

	} else if (request_quad) {
		requested_target = HybridTarget::Flying;
	}

	if (requested_target != HybridTarget::None) {
		_manual_commissioning_active = false;

		if (_transformation_output.state == HybridState::Fault && !_actuator_armed.armed) {
			_transformation_output = _transformation.clearFault(true);
		}

		const HybridState previous_state = _transformation_output.state;
		_transformation_output = _transformation.request(requested_target, input.now_us);

		if (_transformation_output.state != previous_state
		    && (_transformation_output.state == HybridState::TransitionToQuad
			|| _transformation_output.state == HybridState::TransitionToRover)) {
			_transition_start_time = input.now_us;
			_transition_timing_active = true;
		}
	}

	_transformation_output = _transformation.update(input);
}

bool HybridVehicleControl::check_safe_to_transform(bool to_rover)
{
	if (to_rover) {
		vehicle_local_position_s local_position{};

		if (_vehicle_local_position_sub.copy(&local_position)
		    && local_position.z_valid && -local_position.z > _param_hybrid_max_z.get()) {
			return false;
		}
	}

	return true;
}

void HybridVehicleControl::publish_status(const TransformationInput &input, hrt_abstime now)
{
	hybrid_vehicle_status_s status{};
	status.timestamp = now;
	status.as5600_valid = input.as5600_valid;
	status.tmag_quad_valid = input.tmag_quad_valid;
	status.tmag_rover_valid = input.tmag_rover_valid;

	if (_manual_commissioning_active) {
		status.current_state = hybrid_vehicle_status_s::HYBRID_STATE_UNKNOWN;
		status.target_state = hybrid_vehicle_status_s::TARGET_NONE;
		status.sensor_source = hybrid_vehicle_status_s::SENSOR_NONE;
		status.fault_reason = hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE;

	} else {
		switch (_transformation_output.state) {
		case HybridState::Flying:
			status.current_state = hybrid_vehicle_status_s::HYBRID_STATE_FLYING;
			break;
		case HybridState::Driving:
			status.current_state = hybrid_vehicle_status_s::HYBRID_STATE_DRIVING;
			break;
		case HybridState::TransitionToQuad:
		case HybridState::TransitionToRover:
			status.current_state = hybrid_vehicle_status_s::HYBRID_STATE_TRANSITIONING;
			break;
		case HybridState::Unknown:
			status.current_state = hybrid_vehicle_status_s::HYBRID_STATE_UNKNOWN;
			break;
		case HybridState::Fault:
			status.current_state = hybrid_vehicle_status_s::HYBRID_STATE_TRANSITION_FAULT;
			break;
		}

		switch (_transformation_output.target) {
		case HybridTarget::None: status.target_state = hybrid_vehicle_status_s::TARGET_NONE; break;
		case HybridTarget::Flying: status.target_state = hybrid_vehicle_status_s::TARGET_FLYING; break;
		case HybridTarget::Driving: status.target_state = hybrid_vehicle_status_s::TARGET_DRIVING; break;
		}

		switch (_transformation_output.source) {
		case SensorSource::None: status.sensor_source = hybrid_vehicle_status_s::SENSOR_NONE; break;
		case SensorSource::As5600: status.sensor_source = hybrid_vehicle_status_s::SENSOR_AS5600; break;
		case SensorSource::Tmag5273: status.sensor_source = hybrid_vehicle_status_s::SENSOR_TMAG5273; break;
		}

		status.fault_reason = static_cast<uint8_t>(_transformation_output.fault);
	}

	const bool transitioning = _transformation_output.state == HybridState::TransitionToQuad
				   || _transformation_output.state == HybridState::TransitionToRover;

	if (!_manual_commissioning_active && _transition_timing_active
	    && (transitioning || _transformation_output.state == HybridState::Fault)) {
		status.transition_elapsed = now - _transition_start_time;

	} else {
		status.transition_elapsed = 0;

		if (!transitioning) {
			_transition_timing_active = false;
		}
	}

	_hybrid_status_pub.publish(status);
}

void HybridVehicleControl::publish_servo(hrt_abstime now)
{
	actuator_servos_s servos{};
	servos.timestamp = now;
	servos.timestamp_sample = now;

	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; ++i) {
		servos.control[i] = NAN;
	}

	const bool outputs_enabled = (_actuator_armed.armed || _actuator_armed.prearmed)
				     && !_actuator_armed.lockdown && !_actuator_armed.manual_lockdown
				     && !_actuator_armed.force_failsafe;

	if (outputs_enabled && _manual_commissioning_active
	    && _manual_control_cache.fresh(now, MANUAL_CONTROL_TIMEOUT)) {
		servos.control[0] = clamp_servo(_manual_control_cache.value());

	} else if (outputs_enabled && _transformation_output.servo_enabled) {
		servos.control[0] = _transformation_output.servo_value;
	}

	_actuator_servos_pub.publish(servos);
}

void HybridVehicleControl::publish_motor_outputs(hrt_abstime now)
{
	_actuator_motors_mc_sub.update(&_mc_motors);
	_actuator_motors_rover_sub.update(&_rover_motors);

	actuator_motors_s motors{};
	motors.timestamp = now;
	motors.timestamp_sample = now;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		motors.control[i] = NAN;
	}

	if (!_manual_commissioning_active && _transformation_output.state == HybridState::Flying) {
		for (int i = 0; i < 4; ++i) {
			motors.control[i] = _mc_motors.control[i];
		}

		motors.reversible_flags = _mc_motors.reversible_flags & 0x0f;

	} else if (!_manual_commissioning_active && _transformation_output.state == HybridState::Driving) {
		// RoverDifferential owns mixing: right is source 1 -> final 4, left is source 0 -> final 5.
		motors.control[4] = _rover_motors.control[1];
		motors.control[5] = _rover_motors.control[0];
		motors.reversible_flags = ((_rover_motors.reversible_flags & (1u << 1)) ? (1u << 4) : 0)
					  | ((_rover_motors.reversible_flags & (1u << 0)) ? (1u << 5) : 0);
	}

	_actuator_motors_final_pub.publish(motors);
}

void HybridVehicleControl::updateParams()
{
	ModuleParams::updateParams();

	if (_param_handle_rc_map_trans_sw == PARAM_INVALID) {
		_param_handle_rc_map_trans_sw = param_find("RC_MAP_TRANS_SW");
	}

	if (_param_handle_rc_map_trans_sw != PARAM_INVALID) {
		param_get(_param_handle_rc_map_trans_sw, &_rc_map_trans_sw_val);
	}
}

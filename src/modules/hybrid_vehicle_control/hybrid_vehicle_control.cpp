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

#include <climits>
#include <cmath>
#include <cstring>

#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <parameters/param.h>

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
static constexpr hrt_abstime LAND_DETECTION_TIMEOUT = 2_s;
static constexpr hrt_abstime STARTUP_PROBE_WINDOW = 10_s;

bool timestamp_fresh(uint64_t timestamp, hrt_abstime now, uint64_t timeout_us)
{
	return timestamp != 0 && now >= timestamp && now - timestamp <= timeout_us;
}

float clamp_servo(float value)
{
	return fmaxf(-1.f, fminf(value, 1.f));
}

bool reserved_parameter_valid(double value)
{
	return fpclassify(value) == FP_ZERO || std::isnan(value);
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
	if (argc >= 1 && strcmp(argv[0], "clear_fault") == 0) {
		if (!is_running()) {
			PX4_ERR("module not running");
			return PX4_ERROR;
		}

		return get_instance()->clear_fault();
	}

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
	PRINT_MODULE_USAGE_COMMAND_DESCR("clear_fault", "Clear a latched transformation fault while fully disarmed");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int hybrid_vehicle_control_main(int argc, char *argv[])
{
	return HybridVehicleControl::main(argc, argv);
}

TransformationConfig HybridVehicleControl::transformation_config() const
{
	const auto backend = static_cast<hybrid_control::ActuatorBackend>(_param_hyb_act_type.get());
	return {
		_param_hyb_sens_en.get() || backend == hybrid_control::ActuatorBackend::Hx8,
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
		_param_hyb_mag_thr_rov.get(),
		backend,
		_param_hyb_stall_t.get(), _param_hyb_stall_d.get(), _param_hx8_id.get(), _param_hx8_ang_qud.get(),
		_param_hx8_ang_rov.get(), _param_hx8_move_t.get(), _param_hx8_acc_t.get(), _param_hx8_dec_t.get(),
		_param_hx8_pwr_lim.get()
	};
}

bool HybridVehicleControl::selected_feedback_fresh(hrt_abstime now, const TransformationConfig &config) const
{
	if (config.backend == hybrid_control::ActuatorBackend::Hx8) {
		return timestamp_fresh(_hx8_status.last_valid_response, now, 500_ms)
		       && hybrid_control::Hx8BackendPolicy::statusUsable(_hx8_status.servo_id,
				static_cast<uint8_t>(_param_hx8_id.get()), _hx8_status.online, _hx8_status.healthy,
				_hx8_status.config_verified, _hx8_status.protection_flags, true, _hx8_status.angle_deg);
	}

	if (!config.sensors_enabled) {
		return true;
	}

	const uint64_t timeout_us = static_cast<uint64_t>(config.sensor_timeout_s * 1_s);
	const bool encoder_valid = timestamp_fresh(_last_encoder_timestamp, now, timeout_us)
				   && _encoder_healthy && std::isfinite(_current_mechanism_angle);

	if (encoder_valid) {
		return true;
	}

	return hybrid_control::tmagPairValid(
		       _tmag_quad_cache.validFor(config.tmag_quad_device_id, now, timeout_us),
		       _tmag_rover_cache.validFor(config.tmag_rover_device_id, now, timeout_us));
}

bool HybridVehicleControl::transformation_pwm_command_effective() const
{
	if (!_transformation_config_tracker.hasActive()) {
		return false;
	}

	return hybrid_control::transformationPwmCommandEffective(_transformation_config_tracker.active().backend,
			_transformation_output, _manual_commissioning_active, _actuator_armed.armed, _actuator_armed.prearmed,
			_actuator_armed.lockdown, _actuator_armed.manual_lockdown, _actuator_armed.force_failsafe);
}

int HybridVehicleControl::clear_fault()
{
	if (_actuator_armed.armed || _actuator_armed.prearmed) {
		PX4_ERR("clear_fault requires fully disarmed vehicle");
		return PX4_ERROR;
	}

	if (!_transformation_initialized || !_transformation_config_tracker.hasActive()
	    || !hybrid_control::isTransformationFaulted(_transformation_output)) {
		PX4_ERR("no initialized transformation fault to clear");
		return PX4_ERROR;
	}

	const TransformationConfig &config = _transformation_config_tracker.active();

	if (hybrid_control::validateTransformationConfig(config) != TransformFault::None) {
		PX4_ERR("transformation configuration is invalid");
		return PX4_ERROR;
	}

	if (!selected_feedback_fresh(hrt_absolute_time(), config)) {
		PX4_ERR("required transformation feedback is not fresh");
		return PX4_ERROR;
	}

	_transformation_output = _transformation.clearFault(true);
	_hx8_last_target = HybridTarget::None;
	_hx8_command_policy.resetAfterFaultClear();
	_manual_commissioning_active = false;
	_transition_timing_active = false;

	if (hybrid_control::isTransformationFaulted(_transformation_output)) {
		PX4_ERR("transformation fault cannot be cleared");
		return PX4_ERROR;
	}

	return PX4_OK;
}

TransformationInput HybridVehicleControl::update_transformation_input(hrt_abstime now, const TransformationConfig &config)
{
	_hx8_status_sub.update(&_hx8_status);
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
				_tmag_quad_cache.update(magnetic.device_id, {magnetic.mag_x, magnetic.mag_y, magnetic.mag_z},
						magnetic.timestamp);
			}

			if (magnetic.device_id == static_cast<uint32_t>(config.tmag_rover_device_id)) {
				_tmag_rover_cache.update(magnetic.device_id, {magnetic.mag_x, magnetic.mag_y, magnetic.mag_z},
						magnetic.timestamp);
			}
		}
	}

	const uint64_t sensor_timeout_us = static_cast<uint64_t>(config.sensor_timeout_s * 1_s);
	const bool encoder_valid = timestamp_fresh(_last_encoder_timestamp, now, sensor_timeout_us)
				   && _encoder_healthy && std::isfinite(_current_mechanism_angle);
	const bool tmag_quad_valid = _tmag_quad_cache.validFor(config.tmag_quad_device_id, now, sensor_timeout_us);
	const bool tmag_rover_valid = _tmag_rover_cache.validFor(config.tmag_rover_device_id, now, sensor_timeout_us);
	const bool tmag_pair_valid = hybrid_control::tmagPairValid(tmag_quad_valid, tmag_rover_valid);
	const bool tmag_quad_active = tmag_pair_valid
				      && fabsf(_tmag_quad_cache.vector().z) >= config.tmag_quad_threshold;
	const bool tmag_rover_active = tmag_pair_valid
				       && fabsf(_tmag_rover_cache.vector().z) >= config.tmag_rover_threshold;
	hybrid_control::PositionSample position{};
	hybrid_control::ActuatorHealth actuator{};
	actuator.online = true;
	actuator.healthy = true;
	actuator.config_verified = true;
	actuator.command_accepted = true;
	if (config.backend == hybrid_control::ActuatorBackend::Hx8) {
		const bool fresh = timestamp_fresh(_hx8_status.last_valid_response, now, 500_ms);
		const bool valid = hybrid_control::Hx8BackendPolicy::statusUsable(_hx8_status.servo_id,
				static_cast<uint8_t>(_param_hx8_id.get()), _hx8_status.online, _hx8_status.healthy,
				_hx8_status.config_verified, _hx8_status.protection_flags, fresh, _hx8_status.angle_deg);
		const float normalized = hybrid_control::Hx8BackendPolicy::normalizeAngle(_hx8_status.angle_deg,
				_param_hx8_ang_qud.get(), _param_hx8_ang_rov.get());
		const bool endpoint = valid && hybrid_control::Hx8BackendPolicy::endpointAnyMatchesAngleTolerance(normalized,
				config.angle_tolerance, _param_hx8_ang_qud.get(), _param_hx8_ang_rov.get());
		position = {normalized, valid, endpoint, SensorSource::Hx8, _hx8_status.last_valid_response};
		actuator.online = fresh && _hx8_status.online && _hx8_status.servo_id == static_cast<uint8_t>(_param_hx8_id.get());
		actuator.healthy = _hx8_status.healthy && _hx8_status.protection_flags == 0;
		actuator.config_verified = _hx8_status.config_verified;
		actuator.command_accepted = _hx8_command_policy.motionCommandHealthy(_hx8_status.command_sequence,
				_hx8_status.command_accepted, _hx8_status.command_result,
				hx8_servo_status_s::RESULT_NONE, hx8_servo_status_s::RESULT_ACCEPTED);

		if (!actuator.command_accepted) {
			PX4_ERR("HX8 motion unhealthy expected_seq=%" PRIu32 " status_seq=%" PRIu32 " accepted=%d result=%u",
				_hx8_command_policy.lastMotionSequence(), _hx8_status.command_sequence,
				_hx8_status.command_accepted, _hx8_status.command_result);
		}

		actuator.protection_flags = _hx8_status.protection_flags;
	}

	if (config.backend == hybrid_control::ActuatorBackend::Hx8) {
		// Internal HX8 angle is authoritative.
	} else if (encoder_valid) {
		position = {hybrid_control::normalizeAs5600(_current_mechanism_angle, config.quad_angle, config.rover_angle),
			    true, false, SensorSource::As5600, _last_encoder_timestamp};

	} else if (tmag_pair_valid) {
		if (_position_source != SensorSource::Tmag5273) {
			_tmag_ratio_filter.reset();
		}

		const uint64_t timestamp = _tmag_quad_cache.timestamp() < _tmag_rover_cache.timestamp()
					   ? _tmag_quad_cache.timestamp() : _tmag_rover_cache.timestamp();
		position = _tmag_ratio_filter.update(_tmag_quad_cache.vector(), _tmag_rover_cache.vector(),
				true, true, false, timestamp);

	} else if (config.sensors_enabled) {
		position = {NAN, false, false, SensorSource::None, 0};
	}

	_position_source = position.source;

	const bool hx8_backend = config.backend == hybrid_control::ActuatorBackend::Hx8;
	return {
		now,
		hx8_backend ? false : encoder_valid,
		_current_mechanism_angle,
		hx8_backend ? false : tmag_pair_valid,
		hx8_backend ? false : tmag_quad_active,
		hx8_backend ? false : tmag_pair_valid,
		hx8_backend ? false : tmag_rover_active,
		position,
		actuator,
		(config.backend == hybrid_control::ActuatorBackend::Hx8)
			? (_transformation_output.state == HybridState::TransitionToQuad
			   || _transformation_output.state == HybridState::TransitionToRover)
			: transformation_pwm_command_effective()
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
	_vehicle_land_detected_sub.update(&_vehicle_land_detected);
	_vehicle_control_mode_sub.update(&_vcontrol_mode);

	const hrt_abstime now = hrt_absolute_time();

	if (_startup_probe_started == 0) {
		_startup_probe_started = now;
		PX4_INFO("HYBDBG startup t=0 backend=%d rc_man_ch=%d",
			 (int)_param_hyb_act_type.get(), (int)_param_hybrid_man_ch.get());
	}

	const TransformationConfig requested_config = transformation_config();
	const bool safe_to_apply = hybrid_control::configurationUpdatePermitted(_actuator_armed.armed,
				   _actuator_armed.prearmed, _transformation_output.state);
	const bool configuration_applied = _transformation_config_tracker.update(requested_config, safe_to_apply);
	TransformationInput input{now, false, 0.f, false, false, false, false};

	if (!_transformation_config_tracker.hasActive()) {
		publish_status(input, now);
		publish_servo(now);
		publish_motor_outputs(now);
		return;
	}

	const TransformationConfig &active_config = _transformation_config_tracker.active();
	const bool startup_probe_active = now - _startup_probe_started <= STARTUP_PROBE_WINDOW;

	if (startup_probe_active && active_config.backend == hybrid_control::ActuatorBackend::Hx8) {
		const bool changed = !_startup_probe_hx8_seen
				     || _startup_probe_hx8_online != _hx8_status.online
				     || _startup_probe_hx8_healthy != _hx8_status.healthy
				     || _startup_probe_hx8_verified != _hx8_status.config_verified
				     || _startup_probe_hx8_command_accepted != _hx8_status.command_accepted
				     || _startup_probe_hx8_result != _hx8_status.command_result
				     || _startup_probe_hx8_sequence != _hx8_status.command_sequence;

		if (changed) {
			const unsigned long long elapsed = static_cast<unsigned long long>(now - _startup_probe_started);
			const unsigned long long age = _hx8_status.last_valid_response != 0
						       && now >= _hx8_status.last_valid_response
						       ? static_cast<unsigned long long>(now - _hx8_status.last_valid_response) : ULLONG_MAX;
			PX4_INFO("HYBDBG hx8 t=%llu online=%d healthy=%d verified=%d accepted=%d result=%u seq=%u id=%u rsp_age=%llu angle=%.3f",
				 (unsigned long long)elapsed, (int)_hx8_status.online, (int)_hx8_status.healthy,
				 (int)_hx8_status.config_verified, (int)_hx8_status.command_accepted,
				 (unsigned)_hx8_status.command_result, (unsigned)_hx8_status.command_sequence,
				 (unsigned)_hx8_status.servo_id, age, (double)_hx8_status.angle_deg);
			_startup_probe_hx8_seen = true;
			_startup_probe_hx8_online = _hx8_status.online;
			_startup_probe_hx8_healthy = _hx8_status.healthy;
			_startup_probe_hx8_verified = _hx8_status.config_verified;
			_startup_probe_hx8_command_accepted = _hx8_status.command_accepted;
			_startup_probe_hx8_result = _hx8_status.command_result;
			_startup_probe_hx8_sequence = _hx8_status.command_sequence;
		}
	}

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
	publish_hx8_command(now);
	publish_motor_outputs(now);
}

void HybridVehicleControl::update_state_machine(const TransformationInput &input)
{
	const bool hx8_feedback_bad = _transformation_config_tracker.hasActive()
				      && _transformation_config_tracker.active().backend == hybrid_control::ActuatorBackend::Hx8
				      && (!input.position.valid || !input.position.endpoint_confirmed || !input.actuator.online
					  || !input.actuator.healthy || !input.actuator.config_verified);
	const bool airborne_quad_feedback_degraded = _transformation_output.state == hybrid_control::HybridState::Flying
				      && _actuator_armed.armed && !_vehicle_land_detected.landed
				      && !_vehicle_land_detected.maybe_landed && hx8_feedback_bad;

	// Keep the established Quad control state through a transient HX8 feedback
	// loss while airborne. New transformation requests remain blocked by the
	// unchanged state and no command is generated until feedback recovers.
	if (airborne_quad_feedback_degraded) {
		return;
	}
	vehicle_command_s command{};

	while (_vehicle_command_sub.update(&command)) {
		if (command.command != vehicle_command_s::VEHICLE_CMD_DO_HYBRID_TRANSITION) {
			continue;
		}

		HybridTarget target = HybridTarget::None;

		if (fabsf(command.param1 - 1.f) <= FLT_EPSILON) {
			target = HybridTarget::Flying;

		} else if (fabsf(command.param1 - 2.f) <= FLT_EPSILON) {
			target = HybridTarget::Driving;
		}

		const bool reserved_valid = reserved_parameter_valid(command.param2)
					    && reserved_parameter_valid(command.param3)
					    && reserved_parameter_valid(command.param4)
					    && reserved_parameter_valid(command.param5)
					    && reserved_parameter_valid(command.param6)
					    && reserved_parameter_valid(command.param7);

		if (target == HybridTarget::None || !reserved_valid) {
			_last_command_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
			_last_command_reject_reason = target == HybridTarget::None
						      ? hybrid_vehicle_status_s::REJECT_INVALID_TARGET
						      : hybrid_vehicle_status_s::REJECT_RESERVED_PARAMETER;
			_last_command_timestamp = command.timestamp;

			if (command.from_external) {
				publish_transition_ack(command.command, command.source_system, command.source_component,
						       _last_command_result, input.now_us);
			}

			continue;
		}

		request_transition(target, input.now_us, &command);
	}

	manual_control_setpoint_s manual{};

	if (_manual_control_setpoint_sub.update(&manual)) {
		float manual_rc_value = 0.f;

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

			if (!_startup_probe_first_rc_logged && input.now_us - _startup_probe_started <= STARTUP_PROBE_WINDOW) {
				PX4_INFO("HYBDBG rc-first t=%llu value=%.3f mapped_ch=%d",
					 (unsigned long long)(input.now_us - _startup_probe_started),
					 (double)current_manual_value, (int)_param_hybrid_man_ch.get());
				_startup_probe_first_rc_logged = true;
			}

			if (_manual_value_initialized
			    && fabsf(current_manual_value - _last_manual_value) > 0.5f
			    && hybrid_control::manualCommissioningPermitted(_transformation_output,
				    _actuator_armed.armed, _actuator_armed.prearmed, true)) {
				_manual_commissioning_active = true;
			}

			_manual_value_initialized = true;
			_last_manual_value = current_manual_value;
		}

	}

	manual_control_switches_s switches{};

	if (_manual_control_switches_sub.update(&switches)
	    && timestamp_fresh(switches.timestamp, input.now_us, MANUAL_CONTROL_TIMEOUT)) {
		const bool previous_switch_valid = _last_transition_switch == manual_control_switches_s::SWITCH_POS_ON
							  || _last_transition_switch == manual_control_switches_s::SWITCH_POS_OFF;
		const bool switch_changed = previous_switch_valid && switches.transition_switch != _last_transition_switch;

		if (switch_changed
		    && (switches.transition_switch == manual_control_switches_s::SWITCH_POS_ON
			|| switches.transition_switch == manual_control_switches_s::SWITCH_POS_OFF)) {
			PX4_INFO("HYBDBG transition-switch previous=%u current=%u",
				 (unsigned)_last_transition_switch, (unsigned)switches.transition_switch);
			_manual_commissioning_active = false;

			if (_vcontrol_mode.flag_control_auto_enabled) {
				vehicle_command_s mode_command{};
				mode_command.timestamp = input.now_us;
				mode_command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
				mode_command.param1 = 1.f;
				mode_command.param2 = 3.f;
				_vehicle_command_pub.publish(mode_command);
			}

			if (switches.transition_switch == manual_control_switches_s::SWITCH_POS_ON) {
				request_transition(HybridTarget::Driving, input.now_us);

			} else {
				request_transition(HybridTarget::Flying, input.now_us);
			}
		}

		if (!previous_switch_valid
		    && (switches.transition_switch == manual_control_switches_s::SWITCH_POS_ON
			|| switches.transition_switch == manual_control_switches_s::SWITCH_POS_OFF)) {
			PX4_INFO("HYBDBG transition-switch baseline=%u", (unsigned)switches.transition_switch);
		}

		_last_transition_switch = switches.transition_switch;
	}

	const bool manual_control_fresh = _manual_control_cache.fresh(input.now_us, MANUAL_CONTROL_TIMEOUT);

	if (!manual_control_fresh) {
		_manual_commissioning_active = false;
		_manual_value_initialized = false;
	}

	if (hybrid_control::isTransformationFaulted(_transformation_output)
	    || _actuator_armed.armed || !_actuator_armed.prearmed
	    || _actuator_armed.lockdown || _actuator_armed.manual_lockdown || _actuator_armed.force_failsafe) {
		_manual_commissioning_active = false;
	}

	TransformationInput state_input = input;
	state_input.actuator_command_effective = _transformation_config_tracker.active().backend == hybrid_control::ActuatorBackend::Hx8
		? _hx8_command_policy.motionAcknowledged(_hx8_status.command_sequence, _hx8_status.command_accepted,
			_hx8_status.command_result, hx8_servo_status_s::RESULT_ACCEPTED)
		: transformation_pwm_command_effective();
	const HybridState previous_state = _transformation_output.state;
	const TransformFault previous_fault = _transformation_output.fault;
	_transformation_output = _transformation.update(state_input);
	update_transition_lifecycle(previous_state, input.now_us);

	const bool first_fault = previous_fault == TransformFault::None
				 && _transformation_output.fault != TransformFault::None
				 && !_startup_probe_fault_logged;

	if ((input.now_us - _startup_probe_started <= STARTUP_PROBE_WINDOW || first_fault)
	    && (_transformation_output.state != previous_state || _transformation_output.fault != previous_fault)) {
		PX4_INFO("HYBDBG state t=%llu state=%u-%u target=%u fault=%u-%u hx8[o=%d h=%d v=%d a=%d r=%u seq=%u] pos=%.3f valid=%d",
			 (unsigned long long)(input.now_us - _startup_probe_started), (unsigned)previous_state,
			 (unsigned)_transformation_output.state, (unsigned)_transformation_output.target,
			 (unsigned)previous_fault, (unsigned)_transformation_output.fault, (int)state_input.actuator.online,
			 (int)state_input.actuator.healthy, (int)state_input.actuator.config_verified,
			 (int)state_input.actuator.command_accepted, (unsigned)_hx8_status.command_result,
			 (unsigned)_hx8_status.command_sequence, (double)state_input.position.normalized,
			 (int)state_input.position.valid);

		if (first_fault) {
			_startup_probe_fault_logged = true;
		}
	}

	if (hybrid_control::isTransformationFaulted(_transformation_output)) {
		_manual_commissioning_active = false;
	}
}

void HybridVehicleControl::request_transition(HybridTarget target, hrt_abstime now,
		const vehicle_command_s *command_context)
{
	const bool land_detection_fresh = timestamp_fresh(_vehicle_land_detected.timestamp, now,
						 LAND_DETECTION_TIMEOUT);
	PX4_INFO("HYBDBG land-gate target=%s fresh=%d landed=%d maybe=%d timestamp=%llu",
		 target == HybridTarget::Driving ? "rover" : "quad", (int)land_detection_fresh,
		 (int)_vehicle_land_detected.landed, (int)_vehicle_land_detected.maybe_landed,
		 (unsigned long long)_vehicle_land_detected.timestamp);

	const auto decision = hybrid_control::decideTransition({
		land_detection_fresh,
		_vehicle_land_detected.landed,
		hybrid_control::isTransformationFaulted(_transformation_output),
		_transformation_output.state,
		target,
		_transformation_output.target
	});
	_last_command_result = static_cast<uint8_t>(decision.ack_result);
	_last_command_reject_reason = static_cast<uint8_t>(decision.reject_reason);
	_last_command_timestamp = command_context != nullptr ? command_context->timestamp : 0;

	if (!decision.start) {
		if (command_context != nullptr && command_context->from_external) {
			publish_transition_ack(command_context->command, command_context->source_system,
					       command_context->source_component, _last_command_result, now);
		}

		return;
	}

	const HybridState previous_state = _transformation_output.state;
	_transformation_output = _transformation.request(decision.target, now);
	const bool transition_started = _transformation_output.state != previous_state
					&& (_transformation_output.state == HybridState::TransitionToQuad
					    || _transformation_output.state == HybridState::TransitionToRover);

	if (!transition_started) {
		_last_command_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
		_last_command_reject_reason = hybrid_vehicle_status_s::REJECT_UNKNOWN_STATE;

		if (command_context != nullptr && command_context->from_external) {
			publish_transition_ack(command_context->command, command_context->source_system,
					       command_context->source_component, _last_command_result, now);
		}

		return;
	}

	_manual_commissioning_active = false;
	_transition_start_time = now;
	_transition_timing_active = true;
	++_transition_sequence;
	_transition_command_timestamp = _last_command_timestamp;

	if (command_context != nullptr && command_context->from_external) {
		_pending_transition_ack = {true, _transition_sequence, command_context->command,
					   command_context->source_system, command_context->source_component};
		publish_transition_ack(_pending_transition_ack.command, _pending_transition_ack.target_system,
				       _pending_transition_ack.target_component,
				       vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS, now);
	}
}

void HybridVehicleControl::publish_transition_ack(uint32_t command, uint8_t target_system, uint16_t target_component,
		uint8_t result, hrt_abstime now)
{
	vehicle_command_ack_s ack{};
	ack.timestamp = now;
	ack.command = command;
	ack.result = result;
	ack.result_param2 = _transition_sequence;
	ack.target_system = target_system;
	ack.target_component = target_component;
	_vehicle_command_ack_pub.publish(ack);
}

void HybridVehicleControl::update_transition_lifecycle(HybridState previous_state, hrt_abstime now)
{
	const bool was_transitioning = previous_state == HybridState::TransitionToQuad
				     || previous_state == HybridState::TransitionToRover;

	if (!was_transitioning) {
		return;
	}

	const bool completed = (previous_state == HybridState::TransitionToQuad
				&& _transformation_output.state == HybridState::Flying)
			       || (previous_state == HybridState::TransitionToRover
				   && _transformation_output.state == HybridState::Driving);
	const bool failed = hybrid_control::isTransformationFaulted(_transformation_output);

	if (!completed && !failed) {
		return;
	}

	_last_command_result = completed ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED
			       : vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
	_last_command_reject_reason = completed ? hybrid_vehicle_status_s::REJECT_NONE
				      : hybrid_vehicle_status_s::REJECT_TRANSFORMATION_FAULT;
	_last_command_timestamp = _transition_command_timestamp;

	if (completed) {
		_transition_complete_time = now;
	}

	if (_pending_transition_ack.active && _pending_transition_ack.sequence == _transition_sequence) {
		publish_transition_ack(_pending_transition_ack.command, _pending_transition_ack.target_system,
				       _pending_transition_ack.target_component, _last_command_result, now);
		_pending_transition_ack.active = false;
	}
}

void HybridVehicleControl::publish_status(const TransformationInput &input, hrt_abstime now)
{
	hybrid_vehicle_status_s status{};
	status.timestamp = now;
	status.as5600_valid = input.as5600_valid;
	status.tmag_quad_valid = input.tmag_quad_valid;
	status.tmag_rover_valid = input.tmag_rover_valid;
	status.sensors_enabled = _transformation_config_tracker.active().sensors_enabled;
	status.position_confirmed = _transformation_output.position_confirmed;
	status.actuator_backend = _transformation_config_tracker.active().backend == hybrid_control::ActuatorBackend::Hx8
					 ? hybrid_vehicle_status_s::ACTUATOR_HX8 : hybrid_vehicle_status_s::ACTUATOR_PWM;
	status.position_normalized = input.position.normalized;
	status.position_valid = input.position.valid;
	status.actuator_online = input.actuator.online;
	status.actuator_healthy = input.actuator.healthy;
	status.actuator_config_verified = input.actuator.config_verified;
	status.actuator_protection_flags = input.actuator.protection_flags;
	status.no_progress_elapsed = _transformation_output.no_progress_elapsed_us;
	status.transition_sequence = _transition_sequence;
	status.transition_completed_timestamp = _transition_complete_time;
	status.command_result = _last_command_result;
	status.command_reject_reason = _last_command_reject_reason;
	status.command_timestamp = _last_command_timestamp;
	status.landed = _vehicle_land_detected.landed;
	status.land_detection_fresh = timestamp_fresh(_vehicle_land_detected.timestamp, now, LAND_DETECTION_TIMEOUT);
	const bool transformation_faulted = hybrid_control::isTransformationFaulted(_transformation_output);

	if (_manual_commissioning_active && !transformation_faulted) {
		status.current_state = hybrid_vehicle_status_s::HYBRID_STATE_UNKNOWN;
		status.target_state = hybrid_vehicle_status_s::TARGET_NONE;
		status.sensor_source = hybrid_vehicle_status_s::SENSOR_NONE;
		status.fault_reason = hybrid_vehicle_status_s::TRANSFORM_FAULT_NONE;

	} else {
		if (transformation_faulted) {
			status.current_state = hybrid_vehicle_status_s::HYBRID_STATE_TRANSITION_FAULT;

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
		case SensorSource::Hx8: status.sensor_source = hybrid_vehicle_status_s::SENSOR_HX8; break;
		}

		status.fault_reason = static_cast<uint8_t>(_transformation_output.fault);
	}

	const bool transitioning = _transformation_output.state == HybridState::TransitionToQuad
				   || _transformation_output.state == HybridState::TransitionToRover;

	if (!_manual_commissioning_active && _transition_timing_active
	    && (transitioning || transformation_faulted)) {
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
	const bool transformation_faulted = hybrid_control::isTransformationFaulted(_transformation_output);
	const bool pwm_backend = _transformation_config_tracker.hasActive()
				 && _transformation_config_tracker.active().backend == hybrid_control::ActuatorBackend::Pwm;

	if (!transformation_faulted && outputs_enabled && pwm_backend) {
		if (_manual_commissioning_active && _manual_control_cache.fresh(now, MANUAL_CONTROL_TIMEOUT)) {
			servos.control[0] = clamp_servo(_manual_control_cache.value());

		} else if (transformation_pwm_command_effective()) {
			servos.control[0] = _transformation_output.servo_value;
		}
	}

	_actuator_servos_pub.publish(servos);
}

void HybridVehicleControl::publish_hx8_command(hrt_abstime now)
{
	if (!_transformation_config_tracker.hasActive()
	    || _transformation_config_tracker.active().backend != hybrid_control::ActuatorBackend::Hx8) {
		return;
	}

	const int32_t move = _param_hx8_move_t.get();
	const int32_t acc = _param_hx8_acc_t.get();
	const int32_t dec = _param_hx8_dec_t.get();
	const int32_t power = _param_hx8_pwr_lim.get();
	if (!hybrid_control::Hx8BackendPolicy::parametersValid(_param_hx8_id.get(), _param_hx8_ang_qud.get(),
			_param_hx8_ang_rov.get(), move, acc, dec, power, _param_hybrid_trans_t.get())) {
		return;
	}
	hx8_servo_command_s command{};
	command.timestamp = now;
	command.servo_id = static_cast<uint8_t>(_param_hx8_id.get());
	command.move_time_ms = static_cast<uint16_t>(move);
	command.acceleration_time_ms = static_cast<uint16_t>(acc);
	command.deceleration_time_ms = static_cast<uint16_t>(dec);
	command.power_mw = static_cast<uint16_t>(power);
	// Transformation is permitted while normally disarmed. Explicit lockdown and
	// failsafe remain hard inhibits; the transformation state machine separately
	// gates unhealthy actuator feedback and faulted states.
	const bool motion_enabled = !_actuator_armed.lockdown && !_actuator_armed.manual_lockdown
				    && !_actuator_armed.force_failsafe;
	const auto decision = _hx8_command_policy.update(hybrid_control::ActuatorBackend::Hx8, _transformation_output, now,
			      motion_enabled);
	if (decision.action == hybrid_control::Hx8CommandAction::None) {
		return;
	}

	PX4_INFO("HYBDBG hx8-command action=%u target=%u seq=%u armed=%d prearmed=%d lockdown=%d",
		 (unsigned)decision.action, (unsigned)decision.target, (unsigned)decision.sequence,
		 (int)_actuator_armed.armed, (int)_actuator_armed.prearmed, (int)_actuator_armed.lockdown);

	command.sequence = decision.sequence;
	command.type = decision.action == hybrid_control::Hx8CommandAction::Move ? hx8_servo_command_s::COMMAND_MOVE
			: decision.action == hybrid_control::Hx8CommandAction::Hold ? hx8_servo_command_s::COMMAND_HOLD
			: hx8_servo_command_s::COMMAND_RELEASE;
	if (command.type != hx8_servo_command_s::COMMAND_RELEASE) {
		command.target_angle_deg = decision.target == HybridTarget::Flying ? _param_hx8_ang_qud.get() : _param_hx8_ang_rov.get();
	}
	_hx8_command_pub.publish(command);
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

	const bool transformation_faulted = hybrid_control::isTransformationFaulted(_transformation_output);
	const bool position_safe = hybrid_control::stablePositionSafe(_transformation_output,
				  _transformation_config_tracker.active().sensors_enabled);
	const bool mc_fresh = hybrid_control::commandTimestampFresh(_mc_motors.timestamp, now, 100_ms)
			      && hybrid_control::offboardInputFreshAfter(_mc_motors.timestamp, _transition_complete_time, now);
	const bool rover_fresh = hybrid_control::commandTimestampFresh(_rover_motors.timestamp, now, 100_ms)
				 && hybrid_control::offboardInputFreshAfter(_rover_motors.timestamp, _transition_complete_time, now);

	if (!transformation_faulted && position_safe && !_manual_commissioning_active && mc_fresh
	    && _transformation_output.state == HybridState::Flying) {
		for (int i = 0; i < 4; ++i) {
			motors.control[i] = _mc_motors.control[i];
		}

		motors.reversible_flags = _mc_motors.reversible_flags & 0x0f;

	} else if (!transformation_faulted && position_safe && !_manual_commissioning_active && rover_fresh
		   && _transformation_output.state == HybridState::Driving) {
		// RoverDifferential owns mixing: right is source 1 -> final 4, left is source 0 -> final 5.
		motors.control[4] = _rover_motors.control[1];
		motors.control[5] = _rover_motors.control[0];
		motors.reversible_flags = ((_rover_motors.reversible_flags & (1u << 1)) ? (1u << 4) : 0)
					  | ((_rover_motors.reversible_flags & (1u << 0)) ? (1u << 5) : 0);
	}

	_actuator_motors_final_pub.publish(motors);
}

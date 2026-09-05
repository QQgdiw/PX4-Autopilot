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

bool gear_configuration_valid(float down, float clear, float stowed, float tolerance)
{
	return std::isfinite(down) && std::isfinite(clear) && std::isfinite(stowed) && std::isfinite(tolerance)
	       && down >= -368640.f && down <= 368640.f && clear >= -368640.f && clear <= 368640.f
	       && stowed >= -368640.f && stowed <= 368640.f && tolerance > 0.f
	       && fabsf(stowed - down) > 2.f * tolerance && (clear - down) * (stowed - clear) > 0.f
	       && fabsf(clear - down) > tolerance && fabsf(stowed - clear) > tolerance;
}

bool gear_at(float angle, float target, float tolerance)
{
	return std::isfinite(angle) && fabsf(angle - target) <= tolerance;
}

bool gear_at_or_beyond_clear(float angle, float down, float clear, float stowed, float tolerance)
{
	if (!std::isfinite(angle)) {
		return false;
	}

	return stowed > down ? angle >= clear - tolerance && angle <= stowed + tolerance
	       : angle <= clear + tolerance && angle >= stowed - tolerance;
}

HybridTarget sequence_target(hybrid_control::SequenceState state)
{
	switch (state) {
	case hybrid_control::SequenceState::StableQuad:
	case hybrid_control::SequenceState::RoverToQuadPrepare:
	case hybrid_control::SequenceState::RoverToQuadTransform:
	case hybrid_control::SequenceState::QuadWaitAirborne:
	case hybrid_control::SequenceState::QuadRetract:
		return HybridTarget::Flying;

	case hybrid_control::SequenceState::QuadToRoverPrepare:
	case hybrid_control::SequenceState::QuadToRoverTransform:
	case hybrid_control::SequenceState::RoverRetract:
	case hybrid_control::SequenceState::StableRover:
		return HybridTarget::Driving;

	case hybrid_control::SequenceState::Fault:
	default:
		return HybridTarget::None;
	}
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
		_param_hyb_sens_en.get() || backend != hybrid_control::ActuatorBackend::Pwm,
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
		_param_hx8_pwr_lim.get(), _param_h65_l_id.get(), _param_h65_r_id.get(), _param_h65_l_qud.get(),
		_param_h65_l_rov.get(), _param_h65_r_qud.get(), _param_h65_r_rov.get(), _param_h65_speed.get(),
		_param_h65_acc.get(), _param_h65_tol.get(), _param_h65_skew.get()
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

	if (config.backend == hybrid_control::ActuatorBackend::Hx65) {
		const bool fresh = timestamp_fresh(_hx65_status.left_last_valid_response, now, 500_ms)
				   && timestamp_fresh(_hx65_status.right_last_valid_response, now, 500_ms);
		return _hx65_status.left_position_valid && _hx65_status.right_position_valid
		       && hybrid_control::Hx65BackendPolicy::statusUsable(_hx65_status.left_online,
				_hx65_status.right_online, _hx65_status.left_healthy, _hx65_status.right_healthy,
				_hx65_status.left_config_verified, _hx65_status.right_config_verified,
				_hx65_status.motion_config_valid, fresh);
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

	const bool transformation_faulted = hybrid_control::isTransformationFaulted(_transformation_output);
	const bool sequence_faulted = _sequence_initialized
				      && _sequence_output.fault != hybrid_control::SequenceFault::None;

	if (!_transformation_initialized || !_transformation_config_tracker.hasActive()
	    || (!transformation_faulted && !sequence_faulted)) {
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

	if (transformation_faulted) {
		_transformation_output = _transformation.clearFault(true);
	}

	_sequence_initialized = false;
	_sequence_requested_target = HybridTarget::None;
	_active_transition_target = HybridTarget::None;
	_pending_transition_ack.active = false;
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
	_hx65_status_sub.update(&_hx65_status);
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

	} else if (config.backend == hybrid_control::ActuatorBackend::Hx65) {
		const bool fresh = timestamp_fresh(_hx65_status.left_last_valid_response, now, 500_ms)
				   && timestamp_fresh(_hx65_status.right_last_valid_response, now, 500_ms);
		const bool status_valid = _hx65_status.left_position_valid && _hx65_status.right_position_valid
					  && hybrid_control::Hx65BackendPolicy::statusUsable(_hx65_status.left_online,
				_hx65_status.right_online, _hx65_status.left_healthy, _hx65_status.right_healthy,
				_hx65_status.left_config_verified, _hx65_status.right_config_verified,
				_hx65_status.motion_config_valid, fresh);
		const float normalized = hybrid_control::Hx65BackendPolicy::normalizePair(
					 _hx65_status.left_position_steps, _hx65_status.right_position_steps,
					 static_cast<int16_t>(_param_h65_l_qud.get()), static_cast<int16_t>(_param_h65_l_rov.get()),
					 static_cast<int16_t>(_param_h65_r_qud.get()), static_cast<int16_t>(_param_h65_r_rov.get()));
		const float skew = hybrid_control::Hx65BackendPolicy::normalizedSkew(
				 _hx65_status.left_position_steps, _hx65_status.right_position_steps,
				 static_cast<int16_t>(_param_h65_l_qud.get()), static_cast<int16_t>(_param_h65_l_rov.get()),
				 static_cast<int16_t>(_param_h65_r_qud.get()), static_cast<int16_t>(_param_h65_r_rov.get()));
		const bool valid = status_valid && std::isfinite(skew) && skew <= _param_h65_skew.get();
		const bool stopped = !_hx65_status.left_moving && !_hx65_status.right_moving;
		const bool quad_endpoint = valid && stopped && hybrid_control::Hx65BackendPolicy::endpointMatches(
						 _hx65_status.left_position_steps, _hx65_status.right_position_steps, false,
						 static_cast<int16_t>(_param_h65_l_qud.get()), static_cast<int16_t>(_param_h65_l_rov.get()),
						 static_cast<int16_t>(_param_h65_r_qud.get()), static_cast<int16_t>(_param_h65_r_rov.get()),
						 _param_h65_tol.get());
		const bool rover_endpoint = valid && stopped && hybrid_control::Hx65BackendPolicy::endpointMatches(
						  _hx65_status.left_position_steps, _hx65_status.right_position_steps, true,
						  static_cast<int16_t>(_param_h65_l_qud.get()), static_cast<int16_t>(_param_h65_l_rov.get()),
						  static_cast<int16_t>(_param_h65_r_qud.get()), static_cast<int16_t>(_param_h65_r_rov.get()),
						  _param_h65_tol.get());
		position = {normalized, valid, quad_endpoint || rover_endpoint, SensorSource::Hx65,
			    _hx65_status.left_last_valid_response < _hx65_status.right_last_valid_response
			    ? _hx65_status.left_last_valid_response : _hx65_status.right_last_valid_response};
		actuator.online = fresh && _hx65_status.left_online && _hx65_status.right_online;
		actuator.healthy = _hx65_status.left_healthy && _hx65_status.right_healthy
				   && _hx65_status.left_error_flags == 0 && _hx65_status.right_error_flags == 0
				   && std::isfinite(skew) && skew <= _param_h65_skew.get();
		actuator.config_verified = _hx65_status.left_config_verified && _hx65_status.right_config_verified
					   && _hx65_status.motion_config_valid;
		const bool accepted = _hx65_status.command_result == hx65_servo_status_s::RESULT_ACCEPTED;
		actuator.command_accepted = _hx65_command_policy.motionCommandHealthy(_hx65_status.command_sequence,
				accepted, _hx65_status.command_result, hx65_servo_status_s::RESULT_NONE,
				hx65_servo_status_s::RESULT_ACCEPTED);
		actuator.protection_flags = _hx65_status.left_error_flags | _hx65_status.right_error_flags;
	}

	if (config.backend != hybrid_control::ActuatorBackend::Pwm) {
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

	const bool uart_backend = config.backend != hybrid_control::ActuatorBackend::Pwm;
	return {
		now,
		uart_backend ? false : encoder_valid,
		_current_mechanism_angle,
		uart_backend ? false : tmag_pair_valid,
		uart_backend ? false : tmag_quad_active,
		uart_backend ? false : tmag_pair_valid,
		uart_backend ? false : tmag_rover_active,
		position,
		actuator,
		(config.backend != hybrid_control::ActuatorBackend::Pwm)
			? (config.backend == hybrid_control::ActuatorBackend::Hx8
			   ? _hx8_command_policy.motionAcknowledged(_hx8_status.command_sequence, _hx8_status.command_accepted,
				_hx8_status.command_result, hx8_servo_status_s::RESULT_ACCEPTED)
			   : _hx65_command_policy.motionAcknowledged(_hx65_status.command_sequence,
				_hx65_status.command_result == hx65_servo_status_s::RESULT_ACCEPTED,
				_hx65_status.command_result, hx65_servo_status_s::RESULT_ACCEPTED))
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
	const bool safe_to_apply = _active_transition_target == HybridTarget::None
				   && hybrid_control::configurationUpdatePermitted(_actuator_armed.armed,
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

	TransformFault configuration_fault = hybrid_control::validateTransformationConfig(active_config);

	if (configuration_fault == TransformFault::None
	    && active_config.backend == hybrid_control::ActuatorBackend::Hx65
	    && !hybrid_control::Hx65BackendPolicy::parametersValid(_param_h65_l_id.get(), _param_h65_r_id.get(),
		    _param_hx8_id.get(), _param_h65_l_qud.get(), _param_h65_l_rov.get(), _param_h65_r_qud.get(),
		    _param_h65_r_rov.get(), _param_h65_speed.get(), _param_h65_acc.get(), _param_h65_tol.get())) {
		configuration_fault = TransformFault::InvalidConfiguration;
	}

	if (configuration_fault == TransformFault::None) {
		input = update_transformation_input(now, active_config);
	}

	if (!_transformation_initialized || configuration_applied) {
		_transformation_output = _transformation.initialize(active_config, input);
		_transformation_initialized = true;
		_sequence_initialized = false;
		_sequence_requested_target = HybridTarget::None;
		_active_transition_target = HybridTarget::None;
		_pending_transition_ack.active = false;
		_manual_commissioning_active = false;
		_transition_timing_active = false;
	}

	update_state_machine(input);
	publish_status(input, now);
	publish_servo(now);
	publish_hx8_command(now);
	publish_hx65_command(now);
	publish_gear_command(now);
	publish_motor_outputs(now);
}

void HybridVehicleControl::update_state_machine(const TransformationInput &input)
{
	const bool hx8_feedback_bad = _transformation_config_tracker.hasActive()
				      && _transformation_config_tracker.active().backend != hybrid_control::ActuatorBackend::Pwm
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

	const bool mixed_sequence = _transformation_config_tracker.active().backend
				    == hybrid_control::ActuatorBackend::Hx65;
	auto make_sequence_input = [this, &input](HybridTarget requested_target) {
		const float gear_angle = _hx8_status.angle_deg;
		const float gear_down = _param_lg_ang_dn.get();
		const float gear_clear = _param_lg_ang_clr.get();
		const float gear_stowed = _param_lg_ang_stw.get();
		const float gear_tolerance = _param_lg_ang_tol.get();
		const bool gear_config_valid = gear_configuration_valid(gear_down, gear_clear, gear_stowed, gear_tolerance);
		const bool gear_fresh = timestamp_fresh(_hx8_status.last_valid_response, input.now_us, 500_ms);
		const bool gear_command_healthy = _gear_sequence == 0 || _hx8_status.command_sequence != _gear_sequence
						 || _hx8_status.command_result == hx8_servo_status_s::RESULT_NONE
						 || (_hx8_status.command_accepted
						     && _hx8_status.command_result == hx8_servo_status_s::RESULT_ACCEPTED);
		const bool land_fresh = timestamp_fresh(_vehicle_land_detected.timestamp, input.now_us, LAND_DETECTION_TIMEOUT);
		return hybrid_control::SequenceInput {input.now_us, requested_target, _transformation_output.state,
				_transformation_output.fault, _actuator_armed.armed,
				land_fresh && _vehicle_land_detected.landed,
				gear_config_valid && gear_fresh && _hx8_status.online
				&& _hx8_status.servo_id == static_cast<uint8_t>(_param_hx8_id.get()),
				gear_config_valid && gear_command_healthy && _hx8_status.healthy && _hx8_status.config_verified
				&& _hx8_status.protection_flags == 0,
				gear_at(gear_angle, gear_down, gear_tolerance),
				gear_at_or_beyond_clear(gear_angle, gear_down, gear_clear, gear_stowed, gear_tolerance),
				gear_at(gear_angle, gear_stowed, gear_tolerance)};
	};

	if (mixed_sequence && !_sequence_initialized
	    && (_transformation_output.state == HybridState::Flying
		|| _transformation_output.state == HybridState::Driving)) {
		const hybrid_control::SequenceConfig config {_param_lg_auto_en.get(),
			static_cast<uint64_t>(_param_lg_timeout.get() * 1_s),
			static_cast<uint64_t>(_param_lg_land_t.get() * 1_s),
			static_cast<uint64_t>(_param_lg_air_t.get() * 1_s)};
		_sequence_output = _sequence.initialize(config, make_sequence_input(HybridTarget::None));
		_sequence_initialized = true;
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
		float manual_gear_value = 0.f;

		switch (_param_hybrid_man_ch.get()) {
		case 1: manual_rc_value = manual.aux1; break;
		case 2: manual_rc_value = manual.aux2; break;
		case 3: manual_rc_value = manual.aux3; break;
		case 4: manual_rc_value = manual.aux4; break;
		case 5: manual_rc_value = manual.aux5; break;
		case 6: manual_rc_value = manual.aux6; break;
		default: manual_rc_value = 0.f; break;
		}

		switch (_param_lg_man_ch.get()) {
		case 1: manual_gear_value = manual.aux1; break;
		case 2: manual_gear_value = manual.aux2; break;
		case 3: manual_gear_value = manual.aux3; break;
		case 4: manual_gear_value = manual.aux4; break;
		case 5: manual_gear_value = manual.aux5; break;
		case 6: manual_gear_value = manual.aux6; break;
		default: manual_gear_value = 0.f; break;
		}

		if (manual.timestamp != 0 && input.now_us >= manual.timestamp
		    && input.now_us - manual.timestamp <= MANUAL_CONTROL_TIMEOUT) {
			_manual_gear_value = manual_gear_value;
			_manual_gear_timestamp = manual.timestamp;
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

			if (_transformation_config_tracker.active().backend != hybrid_control::ActuatorBackend::Hx65
			    && _manual_value_initialized
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

	hybrid_control::SequenceInput sequence_input{};

	if (mixed_sequence && _sequence_initialized) {
		sequence_input = make_sequence_input(_sequence_requested_target);
		_sequence_requested_target = HybridTarget::None;
		_sequence_output = _sequence.update(sequence_input);

		if (_sequence_output.request_disarm
		    && (input.now_us - _last_disarm_request >= 500_ms)) {
			vehicle_command_s disarm{};
			disarm.timestamp = input.now_us;
			disarm.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
			disarm.param1 = 0.f;
			_vehicle_command_pub.publish(disarm);
			_last_disarm_request = input.now_us;
		}

		const HybridTarget shape_request = _sequence_output.shape_request;

		if (shape_request != HybridTarget::None) {
			const HybridState previous_state = _transformation_output.state;
			_transformation_output = _transformation.request(shape_request, input.now_us);

			if (_transformation_output.state != previous_state
			    && (_transformation_output.state == HybridState::TransitionToQuad
				|| _transformation_output.state == HybridState::TransitionToRover)) {
				_transition_timing_active = true;
			}
		}
	}

	TransformationInput state_input = input;
	const auto backend = _transformation_config_tracker.active().backend;
	state_input.actuator_command_effective = backend == hybrid_control::ActuatorBackend::Hx8
		? _hx8_command_policy.motionAcknowledged(_hx8_status.command_sequence, _hx8_status.command_accepted,
			_hx8_status.command_result, hx8_servo_status_s::RESULT_ACCEPTED)
		: backend == hybrid_control::ActuatorBackend::Hx65
		? _hx65_command_policy.motionAcknowledged(_hx65_status.command_sequence,
			_hx65_status.command_result == hx65_servo_status_s::RESULT_ACCEPTED,
			_hx65_status.command_result, hx65_servo_status_s::RESULT_ACCEPTED)
		: transformation_pwm_command_effective();
	const HybridState previous_state = _transformation_output.state;
	const TransformFault previous_fault = _transformation_output.fault;
	_transformation_output = _transformation.update(state_input);

	if (mixed_sequence && _sequence_initialized) {
		_sequence_output = _sequence.update(make_sequence_input(HybridTarget::None));
	}

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
	_last_command_timestamp = command_context != nullptr ? command_context->timestamp : 0;
	const bool mixed_sequence = _transformation_config_tracker.hasActive()
				    && _transformation_config_tracker.active().backend == hybrid_control::ActuatorBackend::Hx65;

	if (mixed_sequence) {
		bool start = false;
		_last_command_reject_reason = hybrid_vehicle_status_s::REJECT_NONE;

		if (!_sequence_initialized) {
			_last_command_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
			_last_command_reject_reason = hybrid_vehicle_status_s::REJECT_UNKNOWN_STATE;

		} else if (_sequence_output.fault != hybrid_control::SequenceFault::None
			   || hybrid_control::isTransformationFaulted(_transformation_output)) {
			_last_command_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
			_last_command_reject_reason = hybrid_vehicle_status_s::REJECT_TRANSFORMATION_FAULT;

		} else {
			const auto sequence_state = _sequence_output.state;
			const HybridTarget active_target = sequence_target(sequence_state);
			const bool stable = sequence_state == hybrid_control::SequenceState::StableQuad
					    || sequence_state == hybrid_control::SequenceState::StableRover;

			if (active_target == target) {
				_last_command_result = stable ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED
						       : vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS;

			} else if (stable) {
				start = true;
				_last_command_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS;

			} else {
				_last_command_result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
				_last_command_reject_reason = hybrid_vehicle_status_s::REJECT_OPPOSITE_TRANSITION;
			}
		}

		if (!start) {
			if (command_context != nullptr && command_context->from_external) {
				publish_transition_ack(command_context->command, command_context->source_system,
						       command_context->source_component, _last_command_result, now);
			}

			return;
		}

		_manual_commissioning_active = false;
		_sequence_requested_target = target;
		_active_transition_target = target;
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

		return;
	}

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
	_active_transition_target = decision.target;

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
	const bool mixed_sequence = _transformation_config_tracker.hasActive()
				    && _transformation_config_tracker.active().backend == hybrid_control::ActuatorBackend::Hx65;

	if (mixed_sequence) {
		if (_active_transition_target == HybridTarget::None) {
			return;
		}

		const bool failed = hybrid_control::isTransformationFaulted(_transformation_output)
				    || !_sequence_initialized
				    || _sequence_output.fault != hybrid_control::SequenceFault::None;
		const bool flying_ready = _active_transition_target == HybridTarget::Flying
					  && _transformation_output.state == HybridState::Flying
					  && _sequence_output.propulsion_owner == hybrid_control::PropulsionOwner::Quad
					  && _sequence_output.propulsion_ready;
		const bool driving_ready = _active_transition_target == HybridTarget::Driving
					   && _transformation_output.state == HybridState::Driving
					   && _sequence_output.propulsion_owner == hybrid_control::PropulsionOwner::Rover
					   && _sequence_output.propulsion_ready;
		const bool completed = flying_ready || driving_ready;

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

		_sequence_requested_target = HybridTarget::None;
		_active_transition_target = HybridTarget::None;
		_transition_timing_active = false;
		return;
	}

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

	_active_transition_target = HybridTarget::None;
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
	const auto backend = _transformation_config_tracker.active().backend;
	status.actuator_backend = backend == hybrid_control::ActuatorBackend::Hx8
				 ? hybrid_vehicle_status_s::ACTUATOR_HX8
				 : backend == hybrid_control::ActuatorBackend::Hx65
				 ? hybrid_vehicle_status_s::ACTUATOR_HX65 : hybrid_vehicle_status_s::ACTUATOR_PWM;
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
	status.sequence_state = _sequence_initialized ? static_cast<uint8_t>(_sequence_output.state)
				: hybrid_vehicle_status_s::SEQUENCE_FAULT;
	status.sequence_fault = _sequence_initialized ? static_cast<uint8_t>(_sequence_output.fault)
				: hybrid_vehicle_status_s::SEQUENCE_FAULT_NONE;
	status.propulsion_owner = _sequence_initialized ? static_cast<uint8_t>(_sequence_output.propulsion_owner)
				  : hybrid_vehicle_status_s::PROPULSION_NONE;
	status.propulsion_ready = backend != hybrid_control::ActuatorBackend::Hx65
				  ? (_transformation_output.state == HybridState::Flying
				     || _transformation_output.state == HybridState::Driving)
				  : _sequence_initialized && _sequence_output.propulsion_ready;
	status.gear_angle_deg = _hx8_status.angle_deg;
	status.gear_online = timestamp_fresh(_hx8_status.last_valid_response, now, 500_ms) && _hx8_status.online;
	const bool gear_command_healthy = _gear_sequence == 0 || _hx8_status.command_sequence != _gear_sequence
					 || _hx8_status.command_result == hx8_servo_status_s::RESULT_NONE
					 || (_hx8_status.command_accepted
					     && _hx8_status.command_result == hx8_servo_status_s::RESULT_ACCEPTED);
	status.gear_healthy = status.gear_online && _hx8_status.healthy && _hx8_status.config_verified
			      && _hx8_status.protection_flags == 0 && gear_command_healthy;
	status.gear_down = gear_at(status.gear_angle_deg, _param_lg_ang_dn.get(), _param_lg_ang_tol.get());
	status.gear_clear = gear_at_or_beyond_clear(status.gear_angle_deg, _param_lg_ang_dn.get(),
			    _param_lg_ang_clr.get(), _param_lg_ang_stw.get(), _param_lg_ang_tol.get());
	status.gear_stowed = gear_at(status.gear_angle_deg, _param_lg_ang_stw.get(), _param_lg_ang_tol.get());
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

		if (backend == hybrid_control::ActuatorBackend::Hx65 && _sequence_initialized) {
			const auto sequence_state = _sequence_output.state;
			const bool logical_rover = sequence_state == hybrid_control::SequenceState::QuadToRoverTransform
						   || sequence_state == hybrid_control::SequenceState::RoverRetract
						   || sequence_state == hybrid_control::SequenceState::StableRover;
			status.current_state = logical_rover
						     || (sequence_state == hybrid_control::SequenceState::Fault
							 && _sequence_output.propulsion_owner == hybrid_control::PropulsionOwner::Rover)
						     ? hybrid_vehicle_status_s::HYBRID_STATE_DRIVING
						     : sequence_state == hybrid_control::SequenceState::Fault
						       && _sequence_output.propulsion_owner == hybrid_control::PropulsionOwner::None
						     ? hybrid_vehicle_status_s::HYBRID_STATE_TRANSITION_FAULT
						     : hybrid_vehicle_status_s::HYBRID_STATE_FLYING;
		}

		switch (_transformation_output.target) {
		case HybridTarget::None: status.target_state = hybrid_vehicle_status_s::TARGET_NONE; break;
		case HybridTarget::Flying: status.target_state = hybrid_vehicle_status_s::TARGET_FLYING; break;
		case HybridTarget::Driving: status.target_state = hybrid_vehicle_status_s::TARGET_DRIVING; break;
		}

		if (backend == hybrid_control::ActuatorBackend::Hx65 && _sequence_initialized) {
			const auto sequence_state = _sequence_output.state;
			if (sequence_state == hybrid_control::SequenceState::QuadToRoverPrepare
			    || sequence_state == hybrid_control::SequenceState::QuadToRoverTransform
			    || sequence_state == hybrid_control::SequenceState::RoverRetract) {
				status.target_state = hybrid_vehicle_status_s::TARGET_DRIVING;

			} else if (sequence_state == hybrid_control::SequenceState::RoverToQuadPrepare
				   || sequence_state == hybrid_control::SequenceState::RoverToQuadTransform
				   || sequence_state == hybrid_control::SequenceState::QuadWaitAirborne
				   || sequence_state == hybrid_control::SequenceState::QuadRetract) {
				status.target_state = hybrid_vehicle_status_s::TARGET_FLYING;
			}
		}

		switch (_transformation_output.source) {
		case SensorSource::None: status.sensor_source = hybrid_vehicle_status_s::SENSOR_NONE; break;
		case SensorSource::As5600: status.sensor_source = hybrid_vehicle_status_s::SENSOR_AS5600; break;
		case SensorSource::Tmag5273: status.sensor_source = hybrid_vehicle_status_s::SENSOR_TMAG5273; break;
		case SensorSource::Hx8: status.sensor_source = hybrid_vehicle_status_s::SENSOR_HX8; break;
		case SensorSource::Hx65: status.sensor_source = hybrid_vehicle_status_s::SENSOR_HX65; break;
		}

		status.fault_reason = static_cast<uint8_t>(_transformation_output.fault);
	}

	const bool sequence_transitioning = backend == hybrid_control::ActuatorBackend::Hx65
					    && _active_transition_target != HybridTarget::None;
	const bool transitioning = sequence_transitioning
				   || _transformation_output.state == HybridState::TransitionToQuad
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

void HybridVehicleControl::publish_hx65_command(hrt_abstime now)
{
	if (!_transformation_config_tracker.hasActive()
	    || _transformation_config_tracker.active().backend != hybrid_control::ActuatorBackend::Hx65) {
		return;
	}

	if (!hybrid_control::Hx65BackendPolicy::parametersValid(_param_h65_l_id.get(), _param_h65_r_id.get(),
		    _param_hx8_id.get(), _param_h65_l_qud.get(), _param_h65_l_rov.get(), _param_h65_r_qud.get(),
		    _param_h65_r_rov.get(), _param_h65_speed.get(), _param_h65_acc.get(), _param_h65_tol.get())) {
		return;
	}

	const bool motion_enabled = !_actuator_armed.lockdown && !_actuator_armed.manual_lockdown
				    && !_actuator_armed.force_failsafe;
	const auto decision = _hx65_command_policy.update(hybrid_control::ActuatorBackend::Hx65,
			      _transformation_output, now, motion_enabled);

	if (decision.action == hybrid_control::Hx8CommandAction::None) {
		return;
	}

	hx65_servo_command_s command{};
	command.timestamp = now;
	command.sequence = decision.sequence;
	command.type = decision.action == hybrid_control::Hx8CommandAction::Release
		       ? hx65_servo_command_s::COMMAND_RELEASE_PAIR : hx65_servo_command_s::COMMAND_MOVE_PAIR;
	command.speed_steps_s = static_cast<uint16_t>(_param_h65_speed.get());
	command.acceleration = static_cast<uint8_t>(_param_h65_acc.get());

	if (command.type == hx65_servo_command_s::COMMAND_MOVE_PAIR) {
		const bool quad = decision.target == HybridTarget::Flying;
		command.left_target_steps = static_cast<int16_t>(quad ? _param_h65_l_qud.get() : _param_h65_l_rov.get());
		command.right_target_steps = static_cast<int16_t>(quad ? _param_h65_r_qud.get() : _param_h65_r_rov.get());
	}

	_hx65_command_pub.publish(command);
}

void HybridVehicleControl::publish_gear_command(hrt_abstime now)
{
	if (!_transformation_config_tracker.hasActive()
	    || _transformation_config_tracker.active().backend != hybrid_control::ActuatorBackend::Hx65) {
		return;
	}

	hybrid_control::GearTarget target = hybrid_control::GearTarget::None;
	bool hold_current = false;

	if (_param_lg_auto_en.get()) {
		target = _sequence_initialized ? _sequence_output.gear_target : hybrid_control::GearTarget::None;
		hold_current = _sequence_initialized && _sequence_output.fault != hybrid_control::SequenceFault::None;

	} else if (timestamp_fresh(_manual_gear_timestamp, now, MANUAL_CONTROL_TIMEOUT)) {
		if (_manual_gear_value < -0.5f) {
			target = hybrid_control::GearTarget::Down;

		} else if (_manual_gear_value > 0.5f) {
			target = hybrid_control::GearTarget::Stowed;

		} else {
			hold_current = true;
		}

	} else {
		hold_current = true;
	}

	if (target == _last_gear_target && !hold_current) {
		return;
	}

	if (target == hybrid_control::GearTarget::None && !hold_current) {
		return;
	}

	if (hold_current && _last_gear_target == hybrid_control::GearTarget::None) {
		return;
	}

	const int32_t move = _param_lg_move_t.get();
	const int32_t acc = _param_lg_acc_t.get();
	const int32_t dec = _param_lg_dec_t.get();
	const int32_t power = _param_lg_pwr_lim.get();

	if (move <= acc + dec || acc < 20 || dec < 20 || power <= 0 || power > UINT16_MAX
	    || move <= 0 || move > UINT16_MAX || !std::isfinite(_hx8_status.angle_deg)) {
		return;
	}

	hx8_servo_command_s command{};
	command.timestamp = now;
	command.sequence = ++_gear_sequence;
	command.servo_id = static_cast<uint8_t>(_param_hx8_id.get());
	command.type = hold_current ? hx8_servo_command_s::COMMAND_GEAR_HOLD : hx8_servo_command_s::COMMAND_GEAR_MOVE;
	const float bounded_current = fmaxf(fminf(_param_lg_ang_dn.get(), _param_lg_ang_stw.get()),
					    fminf(fmaxf(_param_lg_ang_dn.get(), _param_lg_ang_stw.get()),
						  _hx8_status.angle_deg));
	command.target_angle_deg = hold_current ? bounded_current
				   : target == hybrid_control::GearTarget::Down ? _param_lg_ang_dn.get()
				   : _param_lg_ang_stw.get();
	command.move_time_ms = static_cast<uint16_t>(move);
	command.acceleration_time_ms = static_cast<uint16_t>(acc);
	command.deceleration_time_ms = static_cast<uint16_t>(dec);
	command.power_mw = static_cast<uint16_t>(power);
	_hx8_command_pub.publish(command);
	_last_gear_target = hold_current ? hybrid_control::GearTarget::None : target;
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
	const bool mixed_backend = _transformation_config_tracker.active().backend == hybrid_control::ActuatorBackend::Hx65;
	const bool allow_quad = !mixed_backend || (_sequence_initialized
				&& _sequence_output.propulsion_owner == hybrid_control::PropulsionOwner::Quad);
	const bool allow_rover = !mixed_backend || (_sequence_initialized
				 && _sequence_output.propulsion_owner == hybrid_control::PropulsionOwner::Rover);

	if (!transformation_faulted && position_safe && !_manual_commissioning_active && mc_fresh && allow_quad
	    && _transformation_output.state == HybridState::Flying) {
		for (int i = 0; i < 4; ++i) {
			motors.control[i] = _mc_motors.control[i];
		}

		motors.reversible_flags = _mc_motors.reversible_flags & 0x0f;

	} else if (!transformation_faulted && position_safe && !_manual_commissioning_active && rover_fresh && allow_rover
		   && _transformation_output.state == HybridState::Driving) {
		// RoverDifferential owns mixing: right is source 1 -> final 4, left is source 0 -> final 5.
		motors.control[4] = _rover_motors.control[1];
		motors.control[5] = _rover_motors.control[0];
		motors.reversible_flags = ((_rover_motors.reversible_flags & (1u << 1)) ? (1u << 4) : 0)
					  | ((_rover_motors.reversible_flags & (1u << 0)) ? (1u << 5) : 0);
	}

	_actuator_motors_final_pub.publish(motors);
}

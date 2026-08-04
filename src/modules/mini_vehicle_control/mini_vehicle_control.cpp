/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "mini_vehicle_control.hpp"

#include <climits>
#include <cmath>

#include <px4_platform_common/log.h>
#include <px4_platform_common/sem.hpp>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

namespace
{
constexpr int RoverLeftOutputIndex = 6;  // MAIN7 / Motor 7
constexpr int RoverRightOutputIndex = 7; // MAIN8 / Motor 8
constexpr hrt_abstime LandDetectorTimeout = 1500_ms;
constexpr hrt_abstime ActuatorOutputTimeout = 200_ms;
constexpr hrt_abstime InvalidSourceAge = UINT64_MAX;
constexpr uint8_t SafetyFlagLockdown = 1u << 0;
constexpr uint8_t SafetyFlagManualLockdown = 1u << 1;
constexpr uint8_t SafetyFlagForceFailsafe = 1u << 2;
}

MiniVehicleControl::MiniVehicleControl() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	px4_sem_init(&_diagnostics_lock, 0, 1);
}

MiniVehicleControl::~MiniVehicleControl()
{
	if (_mavlink_log_pub != nullptr) {
		orb_unadvertise(_mavlink_log_pub);
	}

	px4_sem_destroy(&_diagnostics_lock);
}

bool MiniVehicleControl::init()
{
	ScheduleOnInterval(20_ms);
	return true;
}

int MiniVehicleControl::task_spawn(int argc, char *argv[])
{
	MiniVehicleControl *instance = new MiniVehicleControl();

	if (instance != nullptr) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	PX4_ERR("allocation failed");
	return PX4_ERROR;
}

int MiniVehicleControl::custom_command(int argc, char *argv[])
{
	return print_usage("unrecognized command");
}

int MiniVehicleControl::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Output and mode arbiter for the non-transforming mini quad-rover.
Quad to Rover switching is accepted only while the multicopter land detector reports landed.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("mini_vehicle_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mini_vehicle_control_main(int argc, char *argv[])
{
	return MiniVehicleControl::main(argc, argv);
}

void MiniVehicleControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	_actuator_armed_sub.update(&_actuator_armed);
	_vehicle_control_mode_sub.update(&_vehicle_control_mode);
	_vehicle_land_detected_sub.update(&_land_detected);
	process_vehicle_commands();
	process_rc_switch();
	publish_status(now);
	publish_motor_outputs();
}

hrt_abstime MiniVehicleControl::source_age(hrt_abstime now, hrt_abstime source_timestamp)
{
	if (source_timestamp == 0 || now < source_timestamp) {
		return InvalidSourceAge;
	}

	return now - source_timestamp;
}

const char *MiniVehicleControl::mode_name(Mode mode)
{
	return mode == Mode::Quad ? "Quad" : "Rover";
}

void MiniVehicleControl::copy_diagnostics(Diagnostics &diagnostics) const
{
	SmartLock lock(_diagnostics_lock);
	diagnostics = _diagnostics;
}

int MiniVehicleControl::print_status()
{
	Diagnostics diagnostics{};
	copy_diagnostics(diagnostics);
	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime diagnostics_age = now >= diagnostics.updated_at ? now - diagnostics.updated_at : 0;

	PX4_INFO("mode: %s, diagnostic age: %llu us", mode_name(diagnostics.current_mode),
		 static_cast<unsigned long long>(diagnostics_age));
	PX4_INFO("armed: %s, lockdown: %s, manual: %s, failsafe: %s",
		 diagnostics.armed ? "yes" : "no",
		 diagnostics.lockdown ? "yes" : "no",
		 diagnostics.manual_lockdown ? "yes" : "no",
		 diagnostics.force_failsafe ? "yes" : "no");

	if (diagnostics.current_source_age == InvalidSourceAge) {
		PX4_INFO("source: timestamp %llu, age unavailable",
			 static_cast<unsigned long long>(diagnostics.current_source_timestamp));

	} else {
		PX4_INFO("source: timestamp %llu, age %llu us, armed max %llu us",
			 static_cast<unsigned long long>(diagnostics.current_source_timestamp),
			 static_cast<unsigned long long>(diagnostics.current_source_age),
			 static_cast<unsigned long long>(diagnostics.max_source_age_while_armed));
	}

	PX4_INFO("current issue: %s", mini_vehicle_control::outputIssueReasonName(diagnostics.current_reason));
	PX4_INFO("output issues: %lu, recoveries: %lu, safety blocks: %lu, armed drops: %lu",
		 static_cast<unsigned long>(diagnostics.output_issue_count),
		 static_cast<unsigned long>(diagnostics.output_recovery_count),
		 static_cast<unsigned long>(diagnostics.safety_block_count),
		 static_cast<unsigned long>(diagnostics.armed_drop_count));

	if (diagnostics.last_issue_timestamp != 0) {
		const hrt_abstime issue_age = now >= diagnostics.last_issue_timestamp
					  ? now - diagnostics.last_issue_timestamp : 0;

		if (diagnostics.last_issue_source_age == InvalidSourceAge) {
			PX4_INFO("last issue: %s/%s, %llu us ago, source %llu, age unavailable",
				 mode_name(diagnostics.last_issue_mode),
				 mini_vehicle_control::outputIssueReasonName(diagnostics.last_issue_reason),
				 static_cast<unsigned long long>(issue_age),
				 static_cast<unsigned long long>(diagnostics.last_issue_source_timestamp));

		} else {
			PX4_INFO("last issue: %s/%s, %llu us ago, source %llu, age %llu us",
				 mode_name(diagnostics.last_issue_mode),
				 mini_vehicle_control::outputIssueReasonName(diagnostics.last_issue_reason),
				 static_cast<unsigned long long>(issue_age),
				 static_cast<unsigned long long>(diagnostics.last_issue_source_timestamp),
				 static_cast<unsigned long long>(diagnostics.last_issue_source_age));
		}

		PX4_INFO("last controls: [%.5f, %.5f, %.5f, %.5f]",
			 static_cast<double>(diagnostics.last_issue_controls[0]),
			 static_cast<double>(diagnostics.last_issue_controls[1]),
			 static_cast<double>(diagnostics.last_issue_controls[2]),
			 static_cast<double>(diagnostics.last_issue_controls[3]));
		PX4_INFO("last issue safety: lockdown=%s, manual=%s, failsafe=%s",
			 diagnostics.last_issue_lockdown ? "yes" : "no",
			 diagnostics.last_issue_manual_lockdown ? "yes" : "no",
			 diagnostics.last_issue_force_failsafe ? "yes" : "no");
	}

	if (diagnostics.last_safety_block_timestamp != 0) {
		PX4_INFO("last safety flags: lockdown=%s, manual=%s, failsafe=%s",
			 diagnostics.last_safety_flags & SafetyFlagLockdown ? "yes" : "no",
			 diagnostics.last_safety_flags & SafetyFlagManualLockdown ? "yes" : "no",
			 diagnostics.last_safety_flags & SafetyFlagForceFailsafe ? "yes" : "no");
	}

	if (diagnostics.last_recovery_timestamp != 0) {
		const hrt_abstime recovery_age = now >= diagnostics.last_recovery_timestamp
					     ? now - diagnostics.last_recovery_timestamp : 0;
		PX4_INFO("last output recovery: %llu us ago", static_cast<unsigned long long>(recovery_age));
	}

	if (diagnostics.last_armed_drop_timestamp != 0) {
		const hrt_abstime armed_drop_age = now >= diagnostics.last_armed_drop_timestamp
					       ? now - diagnostics.last_armed_drop_timestamp : 0;
		PX4_INFO("last armed drop: %llu us ago", static_cast<unsigned long long>(armed_drop_age));
	}

	return 0;
}

uint8_t MiniVehicleControl::request_mode(Mode requested_mode)
{
	if (requested_mode == _mode) {
		return vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	}

	if (requested_mode == Mode::Rover) {
		const hrt_abstime now = hrt_absolute_time();
		const bool land_status_fresh = _land_detected.timestamp != 0
					       && now >= _land_detected.timestamp
					       && now - _land_detected.timestamp <= LandDetectorTimeout;

		if (!land_status_fresh || !_land_detected.landed) {
			PX4_WARN("Rover switch rejected: vehicle is not landed");
			return vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
		}
	}

	_mode = requested_mode;
	_mode_changed_at = hrt_absolute_time();
	PX4_INFO("switched directly to %s mode", _mode == Mode::Rover ? "Rover" : "Quad");
	return vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
}

void MiniVehicleControl::process_vehicle_commands()
{
	vehicle_command_s command{};

	while (_vehicle_command_sub.update(&command)) {
		if (command.command != vehicle_command_s::VEHICLE_CMD_DO_VTOL_TRANSITION) {
			continue;
		}

		uint8_t result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		const int requested_state = static_cast<int>(command.param1 + 0.5f);

		if (requested_state == 4) {
			result = request_mode(Mode::Rover);

		} else if (requested_state == 3) {
			result = request_mode(Mode::Quad);
		}

		vehicle_command_ack_s ack{};
		ack.timestamp = hrt_absolute_time();
		ack.command = command.command;
		ack.result = result;
		ack.target_system = command.source_system;
		ack.target_component = command.source_component;
		_vehicle_command_ack_pub.publish(ack);
	}
}

void MiniVehicleControl::process_rc_switch()
{
	manual_control_switches_s switches{};

	if (!_manual_control_switches_sub.update(&switches)
	    || switches.transition_switch == manual_control_switches_s::SWITCH_POS_NONE
	    || switches.transition_switch == _previous_transition_switch) {
		return;
	}

	_previous_transition_switch = switches.transition_switch;

	if (_vehicle_control_mode.flag_control_auto_enabled) {
		leave_auto_mode();
	}

	if (switches.transition_switch == manual_control_switches_s::SWITCH_POS_ON) {
		request_mode(Mode::Rover);

	} else if (switches.transition_switch == manual_control_switches_s::SWITCH_POS_OFF) {
		request_mode(Mode::Quad);
	}
}

void MiniVehicleControl::leave_auto_mode()
{
	vehicle_command_s command{};
	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = 1.f;
	command.param2 = 3.f; // PX4_CUSTOM_MAIN_MODE_POSCTL
	_vehicle_command_pub.publish(command);
}

void MiniVehicleControl::publish_status(hrt_abstime now)
{
	hybrid_vehicle_status_s status{};
	status.timestamp = now;
	status.current_state = _mode == Mode::Rover
			       ? hybrid_vehicle_status_s::HYBRID_STATE_DRIVING
			       : hybrid_vehicle_status_s::HYBRID_STATE_FLYING;
	_hybrid_status_pub.publish(status);
}

void MiniVehicleControl::update_diagnostics(hrt_abstime now, const actuator_motors_s &selected_motors,
		mini_vehicle_control::OutputIssueReason reason)
{
	const hrt_abstime selected_source_age = source_age(now, selected_motors.timestamp);
	const bool output_issue_now = _actuator_armed.armed
				      && reason != mini_vehicle_control::OutputIssueReason::None;
	uint8_t safety_flags = 0;

	if (_actuator_armed.lockdown) {
		safety_flags |= SafetyFlagLockdown;
	}

	if (_actuator_armed.manual_lockdown) {
		safety_flags |= SafetyFlagManualLockdown;
	}

	if (_actuator_armed.force_failsafe) {
		safety_flags |= SafetyFlagForceFailsafe;
	}

	const bool safety_block_now = _actuator_armed.armed && safety_flags != 0;
	bool issue_started = false;
	bool issue_recovered = false;
	bool safety_started = false;
	bool safety_recovered = false;

	{
		SmartLock lock(_diagnostics_lock);
		_diagnostics.updated_at = now;
		_diagnostics.current_source_timestamp = selected_motors.timestamp;
		_diagnostics.current_source_age = selected_source_age;
		_diagnostics.current_reason = reason;
		_diagnostics.current_mode = _mode;
		_diagnostics.armed = _actuator_armed.armed;
		_diagnostics.lockdown = _actuator_armed.lockdown;
		_diagnostics.manual_lockdown = _actuator_armed.manual_lockdown;
		_diagnostics.force_failsafe = _actuator_armed.force_failsafe;

		if (_actuator_armed.armed && !_was_armed) {
			_diagnostics.max_source_age_while_armed = 0;
		}

		if (!_actuator_armed.armed && _was_armed) {
			_diagnostics.armed_drop_count++;
			_diagnostics.last_armed_drop_timestamp = now;
		}

		if (_actuator_armed.armed && selected_source_age != InvalidSourceAge
		    && selected_source_age > _diagnostics.max_source_age_while_armed) {
			_diagnostics.max_source_age_while_armed = selected_source_age;
		}

		if (output_issue_now && !_output_issue_active) {
			_output_issue_active = true;
			issue_started = true;
			_diagnostics.output_issue_count++;
			_diagnostics.last_issue_timestamp = now;
			_diagnostics.last_issue_source_timestamp = selected_motors.timestamp;
			_diagnostics.last_issue_source_age = selected_source_age;
			_diagnostics.last_issue_reason = reason;
			_diagnostics.last_issue_mode = _mode;
			_diagnostics.last_issue_lockdown = _actuator_armed.lockdown;
			_diagnostics.last_issue_manual_lockdown = _actuator_armed.manual_lockdown;
			_diagnostics.last_issue_force_failsafe = _actuator_armed.force_failsafe;

			for (int i = 0; i < 4; ++i) {
				_diagnostics.last_issue_controls[i] = selected_motors.control[i];
			}

		} else if (!output_issue_now && _output_issue_active) {
			_output_issue_active = false;

			if (_actuator_armed.armed) {
				issue_recovered = true;
				_diagnostics.output_recovery_count++;
				_diagnostics.last_recovery_timestamp = now;
			}
		}

		if (safety_block_now && !_safety_block_active) {
			_safety_block_active = true;
			safety_started = true;
			_diagnostics.safety_block_count++;
			_diagnostics.last_safety_block_timestamp = now;
			_diagnostics.last_safety_flags = safety_flags;

		} else if (!safety_block_now && _safety_block_active) {
			_safety_block_active = false;
			safety_recovered = _actuator_armed.armed;
		}

		_was_armed = _actuator_armed.armed;
	}

	const uint64_t source_age_ms_64 = selected_source_age == InvalidSourceAge ? 0 : selected_source_age / 1000;
	const unsigned long source_age_ms = source_age_ms_64 > ULONG_MAX ? ULONG_MAX
					    : static_cast<unsigned long>(source_age_ms_64);

	if (issue_started) {
		mavlink_log_warning(&_mavlink_log_pub, "Mini output issue: %s (%lu ms)\t",
				    mini_vehicle_control::outputIssueReasonName(reason), source_age_ms);

	} else if (issue_recovered) {
		mavlink_log_info(&_mavlink_log_pub, "Mini output recovered\t");
	}

	if (safety_started) {
		mavlink_log_warning(&_mavlink_log_pub, "Mini safety block: L%u M%u F%u\t",
				    static_cast<unsigned>(_actuator_armed.lockdown),
				    static_cast<unsigned>(_actuator_armed.manual_lockdown),
				    static_cast<unsigned>(_actuator_armed.force_failsafe));

	} else if (safety_recovered) {
		mavlink_log_info(&_mavlink_log_pub, "Mini safety block cleared\t");
	}
}

void MiniVehicleControl::publish_motor_outputs()
{
	_actuator_motors_mc_sub.update(&_mc_motors);
	_actuator_motors_rover_sub.update(&_rover_motors);
	// The copied source can be newer than Run()'s timestamp if its publisher ran in between.
	const hrt_abstime now = hrt_absolute_time();

	actuator_motors_s motors{};
	motors.timestamp = now;
	motors.timestamp_sample = now;

	for (float &control : motors.control) {
		control = NAN;
	}

	const actuator_motors_s &selected_motors = _mode == Mode::Quad ? _mc_motors : _rover_motors;
	mini_vehicle_control::OutputIssueReason issue_reason = mini_vehicle_control::classifySourceFreshness(now,
			selected_motors.timestamp, _mode_changed_at, ActuatorOutputTimeout);
	const bool selected_output_is_fresh = issue_reason == mini_vehicle_control::OutputIssueReason::None;

	if (!selected_output_is_fresh) {
		update_diagnostics(now, selected_motors, issue_reason);
		_actuator_motors_pub.publish(motors);
		return;
	}

	if (_mode == Mode::Quad) {
		for (int i = 0; i < 4; ++i) {
			motors.control[i] = _mc_motors.control[i];

			if (!PX4_ISFINITE(_mc_motors.control[i])) {
				issue_reason = mini_vehicle_control::OutputIssueReason::NonFiniteControl;
			}
		}

		motors.reversible_flags = _mc_motors.reversible_flags;

	} else {
		// RoverDifferential publishes control[0] as the right-wheel command and
		// control[1] as the left-wheel command. Keep that semantic order while
		// routing to the mini's physical MAIN7 (left) and MAIN8 (right) outputs.
		if (PX4_ISFINITE(_rover_motors.control[0])) {
			motors.control[RoverRightOutputIndex] = fmaxf(_rover_motors.control[0], 0.f);

		} else {
			issue_reason = mini_vehicle_control::OutputIssueReason::NonFiniteControl;
		}

		if (PX4_ISFINITE(_rover_motors.control[1])) {
			motors.control[RoverLeftOutputIndex] = fmaxf(_rover_motors.control[1], 0.f);

		} else {
			issue_reason = mini_vehicle_control::OutputIssueReason::NonFiniteControl;
		}

		// MAIN7/8 are intentionally non-reversible. FunctionMotors maps [0, 1]
		// to the complete configured duty-cycle range when these flags are clear.
		motors.reversible_flags = 0;
	}

	update_diagnostics(now, selected_motors, issue_reason);
	_actuator_motors_pub.publish(motors);
}

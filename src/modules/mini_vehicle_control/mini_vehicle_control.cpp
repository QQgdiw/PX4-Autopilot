/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "mini_vehicle_control.hpp"

#include <cmath>

#include <px4_platform_common/log.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

namespace
{
constexpr int RoverLeftOutputIndex = 6;  // MAIN7 / Motor 7
constexpr int RoverRightOutputIndex = 7; // MAIN8 / Motor 8
constexpr hrt_abstime LandDetectorTimeout = 1500_ms;
constexpr hrt_abstime ActuatorOutputTimeout = 200_ms;
}

MiniVehicleControl::MiniVehicleControl() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
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
	_vehicle_control_mode_sub.update(&_vehicle_control_mode);
	_vehicle_land_detected_sub.update(&_land_detected);
	process_vehicle_commands();
	process_rc_switch();
	publish_status(now);
	publish_motor_outputs(now);
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

void MiniVehicleControl::publish_motor_outputs(hrt_abstime now)
{
	_actuator_motors_mc_sub.update(&_mc_motors);
	_actuator_motors_rover_sub.update(&_rover_motors);

	actuator_motors_s motors{};
	motors.timestamp = now;
	motors.timestamp_sample = now;

	for (float &control : motors.control) {
		control = NAN;
	}

	const actuator_motors_s &selected_motors = _mode == Mode::Quad ? _mc_motors : _rover_motors;
	const bool selected_output_is_fresh = selected_motors.timestamp > _mode_changed_at
					      && now >= selected_motors.timestamp
					      && now - selected_motors.timestamp <= ActuatorOutputTimeout;

	if (!selected_output_is_fresh) {
		_actuator_motors_pub.publish(motors);
		return;
	}

	if (_mode == Mode::Quad) {
		for (int i = 0; i < 4; ++i) {
			motors.control[i] = _mc_motors.control[i];
		}

		motors.reversible_flags = _mc_motors.reversible_flags;

	} else {
		// RoverDifferential publishes control[0] as the right-wheel command and
		// control[1] as the left-wheel command. Keep that semantic order while
		// routing to the mini's physical MAIN7 (left) and MAIN8 (right) outputs.
		if (PX4_ISFINITE(_rover_motors.control[0])) {
			motors.control[RoverRightOutputIndex] = fmaxf(_rover_motors.control[0], 0.f);
		}

		if (PX4_ISFINITE(_rover_motors.control[1])) {
			motors.control[RoverLeftOutputIndex] = fmaxf(_rover_motors.control[1], 0.f);
		}

		// MAIN7/8 are intentionally non-reversible. FunctionMotors maps [0, 1]
		// to the complete configured duty-cycle range when these flags are clear.
		motors.reversible_flags = 0;
	}

	_actuator_motors_pub.publish(motors);
}

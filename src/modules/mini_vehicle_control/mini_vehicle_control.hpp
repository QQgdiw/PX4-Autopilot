/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/hybrid_vehicle_status.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>

class MiniVehicleControl : public ModuleBase<MiniVehicleControl>, public px4::ScheduledWorkItem
{
public:
	MiniVehicleControl();
	~MiniVehicleControl() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	enum class Mode : uint8_t {
		Quad,
		Rover,
	};

	void Run() override;
	void process_vehicle_commands();
	void process_rc_switch();
	uint8_t request_mode(Mode requested_mode);
	void publish_status(hrt_abstime now);
	void publish_motor_outputs(hrt_abstime now);
	void leave_auto_mode();

	uORB::Subscription _actuator_motors_mc_sub{ORB_ID(actuator_motors_mc)};
	uORB::Subscription _actuator_motors_rover_sub{ORB_ID(actuator_motors_rover)};
	uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<hybrid_vehicle_status_s> _hybrid_status_pub{ORB_ID(hybrid_vehicle_status)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};

	actuator_motors_s _mc_motors{};
	actuator_motors_s _rover_motors{};
	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_land_detected_s _land_detected{};

	Mode _mode{Mode::Quad};
	hrt_abstime _mode_changed_at{0};
	uint8_t _previous_transition_switch{manual_control_switches_s::SWITCH_POS_NONE};
};

extern "C" __EXPORT int mini_vehicle_control_main(int argc, char *argv[]);

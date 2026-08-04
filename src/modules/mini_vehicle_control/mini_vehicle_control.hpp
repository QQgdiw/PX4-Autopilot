/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "mini_vehicle_control_diagnostics.hpp"

#include <px4_platform_common/module.h>
#include <px4_platform_common/sem.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
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
	~MiniVehicleControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	enum class Mode : uint8_t {
		Quad,
		Rover,
	};

	struct Diagnostics {
		hrt_abstime updated_at{0};
		hrt_abstime current_source_timestamp{0};
		hrt_abstime current_source_age{0};
		hrt_abstime max_source_age_while_armed{0};
		hrt_abstime last_issue_timestamp{0};
		hrt_abstime last_issue_source_timestamp{0};
		hrt_abstime last_issue_source_age{0};
		hrt_abstime last_recovery_timestamp{0};
		hrt_abstime last_safety_block_timestamp{0};
		hrt_abstime last_armed_drop_timestamp{0};
		uint32_t output_issue_count{0};
		uint32_t output_recovery_count{0};
		uint32_t safety_block_count{0};
		uint32_t armed_drop_count{0};
		mini_vehicle_control::OutputIssueReason current_reason{mini_vehicle_control::OutputIssueReason::None};
		mini_vehicle_control::OutputIssueReason last_issue_reason{mini_vehicle_control::OutputIssueReason::None};
		Mode current_mode{Mode::Quad};
		Mode last_issue_mode{Mode::Quad};
		float last_issue_controls[4] {};
		bool armed{false};
		bool lockdown{false};
		bool manual_lockdown{false};
		bool force_failsafe{false};
		bool last_issue_lockdown{false};
		bool last_issue_manual_lockdown{false};
		bool last_issue_force_failsafe{false};
		uint8_t last_safety_flags{0};
	};

	void Run() override;
	void process_vehicle_commands();
	void process_rc_switch();
	uint8_t request_mode(Mode requested_mode);
	void publish_status(hrt_abstime now);
	void publish_motor_outputs();
	void leave_auto_mode();
	void update_diagnostics(hrt_abstime now, const actuator_motors_s &selected_motors,
				mini_vehicle_control::OutputIssueReason reason);
	void copy_diagnostics(Diagnostics &diagnostics) const;
	static hrt_abstime source_age(hrt_abstime now, hrt_abstime source_timestamp);
	static const char *mode_name(Mode mode);

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
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
	actuator_armed_s _actuator_armed{};
	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_land_detected_s _land_detected{};

	Mode _mode{Mode::Quad};
	hrt_abstime _mode_changed_at{0};
	uint8_t _previous_transition_switch{manual_control_switches_s::SWITCH_POS_NONE};
	orb_advert_t _mavlink_log_pub{nullptr};

	Diagnostics _diagnostics{};
	mutable px4_sem_t _diagnostics_lock;
	bool _output_issue_active{false};
	bool _safety_block_active{false};
	bool _was_armed{false};
};

extern "C" __EXPORT int mini_vehicle_control_main(int argc, char *argv[]);

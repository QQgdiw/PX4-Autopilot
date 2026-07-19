#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/Serial.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/hx8_servo_command.h>
#include <uORB/topics/hx8_servo_status.h>

#include <lib/hx8_servo/Hx8Controller.hpp>

class Hx8UartServo final : public ModuleBase<Hx8UartServo>, public px4::ScheduledWorkItem
{
public:
	Hx8UartServo(const char *device);
	~Hx8UartServo() override;
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	void Run() override;
	int init();
	int print_status() override;
	int cli_config_check();
	int cli_config_write();

private:
	bool load_parameters();
	bool valid_parameters() const;
	void publish_status();
	void emit_events();
	void receive();
	int send(const hx8::PendingRequest &request);
	int configure_uart();

	char _device[64] {};
	int _fd{-1};
	hx8::Controller _controller;
	hx8::StreamParser _parser;
	uORB::Subscription _command_sub{ORB_ID(hx8_servo_command)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Publication<hx8_servo_status_s> _status_pub{ORB_ID(hx8_servo_status)};
	actuator_armed_s _armed {};
	vehicle_control_mode_s _mode {};
	hx8_servo_command_s _command {};
	uint32_t _last_sequence{0};
	bool _explicit_commissioning{false};
	bool _event_offline{false};
	bool _event_protection{false};
	bool _event_config{false};
	bool _event_rejected{false};
	hx8::ProtectionConfig _protection {};
};

#pragma once

#include <fcntl.h>
#include <cmath>
#include <termios.h>
#include <unistd.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/atomic.h>
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
	enum class CommissioningState : uint8_t {
		Idle,
		Requested,
		Active,
		Success,
		Denied,
		Failed,
		CancelRequested
	};

	enum StatusSnapshot : uint32_t {
		StatusOnline = 1u << 0,
		StatusHealthy = 1u << 1,
		StatusConfigVerified = 1u << 2,
		StatusConfigCheckComplete = 1u << 3
	};

	struct ProtectionSnapshot {
		uint64_t sample_time_us{0};
		uint32_t command_sequence{0};
		float angle_deg{NAN};
		float voltage_v{NAN};
		float current_a{NAN};
		float power_w{NAN};
		float temperature_c{NAN};
		uint8_t status_flags{0};
		uint8_t protection_flags{0};
		uint8_t command_result{0};
	};

	bool load_parameters();
	void publish_status();
	void publish_atomic_status();
	void emit_events();
	void receive();
	void process_commissioning_request(uint64_t now);
	void finish_commissioning_request();
	void complete_commissioning(CommissioningState state);
	int consume_commissioning_terminal();
	bool endpoint_angle_match(float angle_deg) const;
	bool valid_motion_command(const hx8_servo_command_s &command) const;
	int send(const hx8::PendingRequest &request);
	int configure_uart();
	int open_uart();

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
	bool _commissioning_started{false};
	uint64_t _commissioning_deadline{0};
	px4::atomic<uint8_t> _commissioning_state{static_cast<uint8_t>(CommissioningState::Idle)};
	px4::atomic<uint32_t> _status_snapshot{0};
	px4::atomic<uint32_t> _status_error_count{0};
	px4::atomic<bool> _first_protection_snapshot_valid{false};
	ProtectionSnapshot _first_protection_snapshot{};
	uint32_t _tx_count{0};
	uint32_t _tx_error_count{0};
	int _last_tx_error{0};
	bool _event_offline{false};
	bool _event_protection{false};
	bool _event_config{false};
	bool _event_rejected{false};
	bool _event_protocol{false};
	hx8::ProtectionConfig _protection {};
	uint8_t _configured_servo_id{0};
	float _quad_angle_deg{NAN};
	float _rover_angle_deg{NAN};
	uint16_t _move_time_ms{0};
	uint16_t _acceleration_time_ms{0};
	uint16_t _deceleration_time_ms{0};
	uint16_t _run_power_mw{0};
	float _transition_time_s{NAN};
};

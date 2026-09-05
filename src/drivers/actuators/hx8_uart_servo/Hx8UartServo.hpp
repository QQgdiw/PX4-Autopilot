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
#include <uORB/topics/hx65_servo_command.h>
#include <uORB/topics/hx65_servo_status.h>

#include <lib/hx8_servo/Hx8Controller.hpp>
#include <lib/hx8_servo/Hx65PairController.hpp>

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
	int cli_trace();

private:
	static constexpr uint64_t BusQuietIntervalUs = 5000;
	static constexpr uint64_t Hx8ToHx65RecoveryUs = 40000;
	static constexpr uint8_t Hx65RxTraceCapacity = 16;
	static constexpr uint32_t Hx65RxTraceValid = 1u << 31;
	static constexpr uint16_t Hx65BootTraceDataCapacity = 512;
	static constexpr uint8_t Hx65BootTraceEntryCapacity = 16;
	static constexpr uint8_t Hx65BootTraceBytesPerEntry = 32;
	static_assert(Hx65BootTraceEntryCapacity * Hx65BootTraceBytesPerEntry == Hx65BootTraceDataCapacity,
		      "HX trace ring must retain exactly 512 raw bytes");
	static constexpr uint32_t Hx65BootTraceValid = 1u << 31;
	static constexpr uint32_t Hx65BootTraceActive = 1u << 30;
	static constexpr uint32_t Hx65BootTraceTruncated = 1u << 29;
	static constexpr uint32_t Hx65MonitorTraceArmed = 1u << 28;

	enum class ActiveProtocol : uint8_t { None, Hx8, Hx65 };
	enum class Hx65BootTraceType : uint8_t {
		Hx8Tx = 1, Hx8Rx, Hx8Timeout, Hx65Tx, Hx65Rx, Hx65Parse, Hx65Timeout
	};
	enum class Hx65BootTraceOutcome : uint8_t { None, Success, Failed, Capacity, MonitorTimeout };

	struct Hx65BootTraceEntry {
		uint64_t timestamp_us{0};
		uint8_t data_length{0};
		uint8_t type{0};
		uint8_t servo_id{0};
		uint8_t request_kind{0};
		uint8_t result{0};
		uint8_t data[Hx65BootTraceBytesPerEntry] {};
	};

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
		StatusConfigCheckComplete = 1u << 3,
		StatusHx65Online = 1u << 4,
		StatusHx65Healthy = 1u << 5,
		StatusHx65ConfigVerified = 1u << 6
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

	bool load_parameters(bool mixed_bus);
	void publish_status();
	void publish_hx65_status();
	void publish_atomic_status();
	void emit_events();
	void receive();
	void capture_hx65_rx_trace(uint8_t result, uint8_t expected_id, hx65::RequestKind kind);
	void start_hx65_boot_trace();
	void arm_hx65_monitor_trace();
	void append_hx65_boot_trace(Hx65BootTraceType type, uint8_t servo_id, hx65::RequestKind kind,
				    uint8_t result, const uint8_t *data, uint16_t length, uint64_t now);
	void freeze_hx65_boot_trace(Hx65BootTraceOutcome outcome, bool truncated = false);
	void process_commissioning_request(uint64_t now);
	void finish_commissioning_request();
	void complete_commissioning(CommissioningState state);
	int consume_commissioning_terminal();
	bool endpoint_angle_match(float angle_deg) const;
	bool valid_motion_command(const hx8_servo_command_s &command) const;
	bool valid_hx65_motion_command(const hx65_servo_command_s &command) const;
	int send(const hx8::PendingRequest &request);
	int send(const hx65::PendingRequest &request);
	bool try_send_hx65(uint64_t now);
	int configure_uart();
	int open_uart();

	char _device[64] {};
	int _fd{-1};
	hx8::Controller _controller;
	hx8::StreamParser _parser;
	hx65::PairController _hx65_controller;
	hx65::StreamParser _hx65_parser;
	ActiveProtocol _active_protocol{ActiveProtocol::None};
	uint64_t _bus_quiet_until{0};
	uint64_t _hx65_not_before{0};
	bool _prefer_hx65_after_hx8{false};
	uint8_t _hx65_rx_trace[Hx65RxTraceCapacity] {};
	uint8_t _hx65_rx_trace_size{0};
	Hx65BootTraceEntry _hx65_boot_trace_entries[Hx65BootTraceEntryCapacity] {};
	uint16_t _hx65_boot_trace_data_count{0};
	uint8_t _hx65_boot_trace_entry_count{0};
	uint8_t _hx65_boot_trace_next_entry{0};
	bool _hx65_boot_trace_active{false};
	bool _hx65_monitor_trace_armed{false};
	bool _hx65_boot_trace_wrapped{false};
	uORB::Subscription _command_sub{ORB_ID(hx8_servo_command)};
	uORB::Subscription _hx65_command_sub{ORB_ID(hx65_servo_command)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Publication<hx8_servo_status_s> _status_pub{ORB_ID(hx8_servo_status)};
	uORB::Publication<hx65_servo_status_s> _hx65_status_pub{ORB_ID(hx65_servo_status)};
	actuator_armed_s _armed {};
	vehicle_control_mode_s _mode {};
	hx8_servo_command_s _command {};
	hx65_servo_command_s _hx65_command {};
	uint32_t _last_sequence{0};
	uint32_t _hx65_last_sequence{0};
	bool _explicit_commissioning{false};
	bool _commissioning_started{false};
	uint64_t _commissioning_deadline{0};
	px4::atomic<uint8_t> _commissioning_state{static_cast<uint8_t>(CommissioningState::Idle)};
	px4::atomic<uint32_t> _status_snapshot{0};
	px4::atomic<uint32_t> _status_error_count{0};
	px4::atomic<uint32_t> _hx65_position_snapshot{0};
	px4::atomic<uint32_t> _hx65_left_timeout_snapshot{0};
	px4::atomic<uint32_t> _hx65_left_retry_snapshot{0};
	px4::atomic<uint32_t> _hx65_right_timeout_snapshot{0};
	px4::atomic<uint32_t> _hx65_right_retry_snapshot{0};
	px4::atomic<uint32_t> _hx65_rx_error_trace_0{0};
	px4::atomic<uint32_t> _hx65_rx_error_trace_1{0};
	px4::atomic<uint32_t> _hx65_rx_error_trace_2{0};
	px4::atomic<uint32_t> _hx65_rx_error_trace_3{0};
	px4::atomic<uint32_t> _hx65_rx_error_meta{0};
	px4::atomic<uint32_t> _hx65_boot_trace_meta{0};
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
	int32_t _baudrate{1000000};
	uint8_t _configured_servo_id{0};
	float _quad_angle_deg{NAN};
	float _rover_angle_deg{NAN};
	uint16_t _move_time_ms{0};
	uint16_t _acceleration_time_ms{0};
	uint16_t _deceleration_time_ms{0};
	uint16_t _run_power_mw{0};
	float _transition_time_s{NAN};
	bool _mixed_bus{false};
	bool _mixed_motion_config_valid{false};
	float _gear_down_angle_deg{NAN};
	float _gear_clear_angle_deg{NAN};
	float _gear_stowed_angle_deg{NAN};
	uint16_t _gear_move_time_ms{0};
	uint16_t _gear_acceleration_time_ms{0};
	uint16_t _gear_deceleration_time_ms{0};
	uint16_t _gear_run_power_mw{0};
	hx65::PairConfig _hx65_config{};
	int16_t _hx65_left_quad{-32768};
	int16_t _hx65_left_rover{-32768};
	int16_t _hx65_right_quad{-32768};
	int16_t _hx65_right_rover{-32768};
	uint16_t _hx65_speed{0};
	uint8_t _hx65_acceleration{0};
};

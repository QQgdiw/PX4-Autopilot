#pragma once

#include <cstdint>

#include "Hx8Protocol.hpp"

namespace hx8
{

struct ProtectionConfig {
	uint8_t response_enabled{1};
	uint8_t stall_release_enabled{1};
	uint16_t stall_power_mw{0};
	uint16_t voltage_min_mv{9000};
	uint16_t voltage_max_mv{12600};
	uint16_t temperature_adc{0};
	uint16_t power_limit_mw{0};
	uint16_t current_limit_ma{0};
	uint8_t power_on_lock{0};
};

enum class RequestPriority : uint8_t { EmergencyRelease, Target, Status, Angle, Config, Diagnostic };

struct ControllerInput {
	uint64_t now_us;
	bool armed;
	bool prearmed;
	bool lockdown;
	bool failsafe;
	bool explicit_commissioning;
};

struct MotionCommand {
	uint64_t timestamp_us;
	uint32_t sequence;
	uint8_t type;
	uint8_t servo_id;
	float target_angle_deg;
	uint16_t move_time_ms;
	uint16_t acceleration_time_ms;
	uint16_t deceleration_time_ms;
	uint16_t power_mw;
};

struct ControllerStatus {
	uint64_t sample_time_us;
	uint64_t last_valid_response_us;
	uint32_t command_sequence;
	uint32_t rx_valid_count;
	uint32_t rx_error_count;
	uint32_t timeout_count;
	uint32_t retry_count;
	uint8_t servo_id;
	bool online;
	bool healthy;
	bool config_verified;
	bool command_accepted;
	bool persistent_write_active;
	float angle_deg;
	float voltage_v;
	float current_a;
	float power_w;
	float temperature_c;
	uint8_t status_flags;
	uint8_t protection_flags;
	uint8_t command_result;
};

struct PendingRequest {
	bool valid;
	RequestPriority priority;
	CommandId command;
	uint8_t payload[32];
	uint8_t payload_length;
};

class Controller
{
public:
	static constexpr uint64_t ResponseTimeoutUs = 30000;
	static constexpr uint8_t MaxRetries = 2;
	static constexpr uint64_t MinimumCommandSpacingUs = 20000;
	static constexpr uint64_t MovingMonitorIntervalUs = 50000;
	static constexpr uint64_t StableMonitorIntervalUs = 200000;
	static constexpr uint64_t CommandExpiryUs = 500000;

	void setExpectedConfig(const ProtectionConfig &config);
	void setTarget(const MotionCommand &command);
	void requestRelease(uint32_t sequence);
	void requestPersistentWrite();
	PendingRequest update(const ControllerInput &input);
	void acceptResponse(const Frame &frame, uint64_t now_us);
	void notifyTimeout(uint64_t now_us);
	const ControllerStatus &status() const { return _status; }

private:
	enum class BootState : uint8_t { Ping, Read, Complete, Failed };
	enum class WriteState : uint8_t { Idle, Write, Readback };

	static bool calibrated(const ProtectionConfig &config);
	static bool isByteParameter(uint8_t parameter);
	uint16_t expectedParameterValue(uint8_t parameter) const;
	PendingRequest makeRequest(RequestPriority priority, CommandId command, const uint8_t *payload,
				   uint8_t payload_length, uint64_t now_us);
	PendingRequest makeParameterRead(uint8_t parameter, uint64_t now_us);
	PendingRequest makeParameterWrite(uint8_t parameter, uint64_t now_us);
	bool responseShapeValid(const Frame &frame) const;
	void handleParameterRead(const Frame &frame, uint64_t now_us);
	void handleCommandResponse(const Frame &frame);
	void handleStatusResponse(const Frame &frame);
	void finishBootRead(bool matches, uint64_t now_us);
	void abortPersistentWrite();

	ProtectionConfig _expected {};
	MotionCommand _target {};
	ControllerStatus _status {};
	PendingRequest _outstanding {};
	PendingRequest _retry_request {};
	uint64_t _last_request_us{0};
	uint64_t _last_monitor_us{0};
	uint32_t _last_target_sequence{0};
	uint32_t _last_release_sequence{0};
	uint8_t _servo_id{0};
	uint8_t _boot_index{0};
	uint8_t _write_index{0};
	uint8_t _request_retry_count{0};
	uint8_t _outstanding_parameter{0};
	BootState _boot_state{BootState::Ping};
	WriteState _write_state{WriteState::Idle};
	bool _expected_calibrated{false};
	bool _boot_matches{true};
	bool _sent_any{false};
	bool _retry_pending{false};
	bool _target_pending{false};
	bool _release_pending{false};
	bool _persistent_write_requested{false};
};

} // namespace hx8

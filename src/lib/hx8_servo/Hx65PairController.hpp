#pragma once

#include "Hx65Protocol.hpp"

#include <cstdint>

namespace hx65
{

enum class Side : uint8_t { Left = 0, Right = 1 };
enum class RequestKind : uint8_t { None, Identity, Protection, Mode, StageMove, Action, Release, Monitor };
enum class CommandResult : uint8_t { None, Accepted, Rejected, Timeout, ProtocolError };

struct PairConfig {
	uint8_t left_id{1};
	uint8_t right_id{2};
	uint8_t response_level{1};
	uint8_t protection_mask{44};
	uint8_t operating_mode{0};
};

struct PairCommand {
	uint64_t timestamp_us{0};
	uint32_t sequence{0};
	int16_t left_target_steps{0};
	int16_t right_target_steps{0};
	uint16_t speed_steps_s{0};
	uint8_t acceleration{0};
};

struct ServoStatus {
	uint64_t last_valid_response{0};
	uint32_t timeout_count{0};
	uint32_t retry_count{0};
	int16_t position_steps{0};
	int16_t speed_steps_s{0};
	int16_t load{0};
	uint16_t current_ma{0};
	float voltage_v{0.f};
	float temperature_c{0.f};
	uint8_t error_flags{0};
	bool moving{false};
	bool online{false};
	bool healthy{false};
	bool config_verified{false};
	bool position_valid{false};
};

struct PairStatus {
	ServoStatus servo[2] {};
	uint32_t command_sequence{0};
	CommandResult command_result{CommandResult::None};
	uint32_t timeout_count{0};
	uint32_t retry_count{0};
	uint32_t protocol_error_count{0};
};

struct PendingRequest {
	bool valid{false};
	bool expects_response{true};
	Instruction instruction{Instruction::Read};
	RequestKind kind{RequestKind::None};
	Side side{Side::Left};
	uint8_t servo_id{0};
	uint8_t parameters[20] {};
	uint8_t parameter_length{0};
};

class PairController
{
public:
	static constexpr uint64_t ResponseTimeoutUs = 30000;
	static constexpr uint64_t MinimumCommandSpacingUs = 10000;
	static constexpr uint64_t MonitorIntervalUs = 50000;
	static constexpr uint64_t CommandExpiryUs = 500000;
	static constexpr uint8_t MaxRetries = 2;

	void setConfig(const PairConfig &config);
	void setTarget(const PairCommand &command);
	void requestRelease(uint32_t sequence);
	void rejectCommand(uint32_t sequence);
	PendingRequest update(uint64_t now_us);
	void acceptResponse(const StatusFrame &frame, uint64_t now_us);
	void completeNoResponse(bool transmitted, uint64_t now_us);
	void notifyTimeout(uint64_t now_us);
	void notifyRxProtocolError();
	void notifyProtocolError();
	const PairStatus &status() const { return _status; }
	bool hasOutstandingRequest() const { return _outstanding.valid; }
	bool bootPending() const { return _boot_step != BootStep::Complete && _boot_step != BootStep::Failed; }
	uint8_t outstandingServoId() const { return _outstanding.servo_id; }
	RequestKind outstandingKind() const { return _outstanding.kind; }

private:
	enum class BootStep : uint8_t {
		LeftIdentity, LeftProtection, LeftMode,
		RightIdentity, RightProtection, RightMode, Complete, Failed
	};
	enum class MotionStep : uint8_t { Idle, StageLeft, StageRight, Action };

	PendingRequest makeRequest(Instruction instruction, RequestKind kind, Side side, uint8_t servo_id,
				   const uint8_t *parameters, uint8_t parameter_length, bool expects_response,
				   uint64_t now_us);
	PendingRequest makeBootRequest(uint64_t now_us);
	PendingRequest makeMotionRequest(uint64_t now_us);
	void finishBootResponse(const StatusFrame &frame, uint64_t now_us);
	void finishMotionResponse(const StatusFrame &frame);
	void finishMonitorResponse(const StatusFrame &frame, uint64_t now_us);
	void failOutstanding(CommandResult result);
	ServoStatus &servo(Side side) { return _status.servo[static_cast<uint8_t>(side)]; }
	uint8_t id(Side side) const { return side == Side::Left ? _config.left_id : _config.right_id; }

	PairConfig _config{};
	PairStatus _status{};
	BootStep _boot_step{BootStep::LeftIdentity};
	MotionStep _motion_step{MotionStep::Idle};
	PairCommand _command{};
	PendingRequest _outstanding{};
	PendingRequest _retry_request{};
	uint64_t _last_request_us{0};
	uint64_t _last_monitor_us{0};
	uint32_t _last_command_sequence{0};
	uint32_t _last_release_sequence{0};
	uint8_t _request_retry_count{0};
	Side _monitor_side{Side::Left};
	bool _sent_any{false};
	bool _retry_pending{false};
	bool _release_pending{false};
};

} // namespace hx65

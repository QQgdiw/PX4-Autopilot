#include "Hx65PairController.hpp"

#include <cstring>

namespace hx65
{

namespace
{

constexpr uint8_t IdentityAddress = 0x05;
constexpr uint8_t IdentityLength = 4;
constexpr uint8_t ProtectionAddress = 0x13;
constexpr uint8_t ModeAddress = 0x21;
constexpr uint8_t TorqueAddress = 0x28;
constexpr uint8_t MonitorAddress = 0x38;
constexpr uint8_t MonitorLength = 15;

uint16_t read16(const uint8_t *bytes)
{
	return static_cast<uint16_t>(bytes[0]) | (static_cast<uint16_t>(bytes[1]) << 8);
}

void write16(uint8_t *bytes, uint16_t value)
{
	bytes[0] = static_cast<uint8_t>(value);
	bytes[1] = static_cast<uint8_t>(value >> 8);
}

bool targetValid(int16_t target)
{
	return target >= -30719 && target <= 30719;
}

} // namespace

void PairController::setConfig(const PairConfig &config)
{
	_config = config;
	_status = {};
	_boot_step = (_config.left_id != _config.right_id && _config.left_id <= 253 && _config.right_id <= 253
		      && _config.response_level == 1 && _config.operating_mode == 0)
		     ? BootStep::LeftIdentity : BootStep::Failed;
	_motion_step = MotionStep::Idle;
	_outstanding = {};
	_retry_request = {};
	_retry_pending = false;
	_release_pending = false;
	_sent_any = false;
	_request_retry_count = 0;
}

void PairController::setTarget(const PairCommand &command)
{
	const bool valid = command.sequence > _last_command_sequence && targetValid(command.left_target_steps)
			   && targetValid(command.right_target_steps) && command.speed_steps_s > 0
			   && command.speed_steps_s <= 3400;

	if (!valid) {
		_status.command_result = CommandResult::Rejected;
		return;
	}

	_command = command;
	_last_command_sequence = command.sequence;
	_status.command_sequence = command.sequence;
	_status.command_result = CommandResult::None;
	_motion_step = MotionStep::StageLeft;
}

void PairController::requestRelease(uint32_t sequence)
{
	if (sequence > _last_release_sequence) {
		_last_release_sequence = sequence;
		_status.command_sequence = sequence;
		_status.command_result = CommandResult::None;
		_release_pending = true;
		_motion_step = MotionStep::Idle;
	}
}

void PairController::rejectCommand(uint32_t sequence)
{
	_status.command_sequence = sequence;
	_status.command_result = CommandResult::Rejected;
}

PendingRequest PairController::makeRequest(Instruction instruction, RequestKind kind, Side side, uint8_t servo_id,
		const uint8_t *parameters, uint8_t parameter_length, bool expects_response, uint64_t now_us)
{
	PendingRequest request {};
	request.valid = true;
	request.expects_response = expects_response;
	request.instruction = instruction;
	request.kind = kind;
	request.side = side;
	request.servo_id = servo_id;
	request.parameter_length = parameter_length;

	if (parameter_length > 0) {
		memcpy(request.parameters, parameters, parameter_length);
	}

	_outstanding = request;
	_retry_request = request;
	_last_request_us = now_us;
	_sent_any = true;
	return request;
}

PendingRequest PairController::makeBootRequest(uint64_t now_us)
{
	Side side = _boot_step < BootStep::RightIdentity ? Side::Left : Side::Right;

	switch (_boot_step) {
	case BootStep::LeftIdentity:
	case BootStep::RightIdentity: {
			const uint8_t parameters[] {IdentityAddress, IdentityLength};
			return makeRequest(Instruction::Read, RequestKind::Identity, side, id(side), parameters,
					   sizeof(parameters), true, now_us);
		}

	case BootStep::LeftProtection:
	case BootStep::RightProtection: {
			const uint8_t parameters[] {ProtectionAddress, 1};
			return makeRequest(Instruction::Read, RequestKind::Protection, side, id(side), parameters,
					   sizeof(parameters), true, now_us);
		}

	case BootStep::LeftMode:
	case BootStep::RightMode: {
			const uint8_t parameters[] {ModeAddress, 1};
			return makeRequest(Instruction::Read, RequestKind::Mode, side, id(side), parameters,
					   sizeof(parameters), true, now_us);
		}

	case BootStep::Complete:
	case BootStep::Failed:
		return {};
	}

	return {};
}

PendingRequest PairController::makeMotionRequest(uint64_t now_us)
{
	if (_motion_step == MotionStep::Action) {
		return makeRequest(Instruction::Action, RequestKind::Action, Side::Left, BroadcastId, nullptr, 0, false, now_us);
	}

	const Side side = _motion_step == MotionStep::StageLeft ? Side::Left : Side::Right;
	const int16_t target = side == Side::Left ? _command.left_target_steps : _command.right_target_steps;
	// Stage torque enable and the complete position command atomically. This also
	// re-enables a servo after a previous emergency release.
	uint8_t parameters[9] {TorqueAddress, 1, _command.acceleration, 0, 0, 0, 0, 0, 0};
	write16(&parameters[3], encodeSignedMagnitude(target, 15));
	write16(&parameters[7], _command.speed_steps_s);
	return makeRequest(Instruction::RegWrite, RequestKind::StageMove, side, id(side), parameters,
			   sizeof(parameters), true, now_us);
}

PendingRequest PairController::update(uint64_t now_us)
{
	if (_outstanding.valid || (_sent_any && now_us - _last_request_us < MinimumCommandSpacingUs)) {
		return {};
	}

	if (_retry_pending) {
		_retry_pending = false;
		_outstanding = _retry_request;
		_last_request_us = now_us;
		return _outstanding;
	}

	if (_release_pending) {
		const uint8_t parameters[] {TorqueAddress, 0};
		return makeRequest(Instruction::Write, RequestKind::Release, Side::Left, BroadcastId, parameters,
				   sizeof(parameters), false, now_us);
	}

	if (_boot_step != BootStep::Complete) {
		return makeBootRequest(now_us);
	}

	if (_motion_step != MotionStep::Idle) {
		if (now_us < _command.timestamp_us || now_us - _command.timestamp_us > CommandExpiryUs
		    || !_status.servo[0].healthy || !_status.servo[1].healthy) {
			_motion_step = MotionStep::Idle;
			_status.command_result = CommandResult::Rejected;
			return {};
		}

		return makeMotionRequest(now_us);
	}

	if (now_us - _last_monitor_us >= MonitorIntervalUs) {
		const uint8_t parameters[] {MonitorAddress, MonitorLength};
		const Side side = _monitor_side;
		_monitor_side = side == Side::Left ? Side::Right : Side::Left;
		_last_monitor_us = now_us;
		return makeRequest(Instruction::Read, RequestKind::Monitor, side, id(side), parameters,
				   sizeof(parameters), true, now_us);
	}

	return {};
}

void PairController::finishBootResponse(const StatusFrame &frame, uint64_t now_us)
{
	ServoStatus &current = servo(_outstanding.side);
	bool matches = frame.error == 0;

	switch (_outstanding.kind) {
	case RequestKind::Identity:
		matches = matches && frame.parameter_length == IdentityLength && frame.parameters[0] == id(_outstanding.side)
			  && frame.parameters[3] == _config.response_level;
		break;

	case RequestKind::Protection:
		matches = matches && frame.parameter_length == 1 && frame.parameters[0] == _config.protection_mask;
		break;

	case RequestKind::Mode:
		matches = matches && frame.parameter_length == 1 && frame.parameters[0] == _config.operating_mode;
		break;

	default:
		matches = false;
		break;
	}

	current.last_valid_response = now_us;
	current.online = true;

	if (!matches) {
		current.config_verified = false;
		current.healthy = false;
		_boot_step = BootStep::Failed;
		return;
	}

	_boot_step = static_cast<BootStep>(static_cast<uint8_t>(_boot_step) + 1u);

	if (_boot_step == BootStep::RightIdentity) {
		_status.servo[0].config_verified = true;
		_status.servo[0].healthy = true;
	}

	if (_boot_step == BootStep::Complete) {
		_status.servo[1].config_verified = true;
		_status.servo[1].healthy = true;
		_last_monitor_us = now_us - MonitorIntervalUs;
	}
}

void PairController::finishMotionResponse(const StatusFrame &frame)
{
	if (frame.error != 0 || frame.parameter_length != 0) {
		failOutstanding(CommandResult::Rejected);
		return;
	}

	_motion_step = _motion_step == MotionStep::StageLeft ? MotionStep::StageRight : MotionStep::Action;
}

void PairController::finishMonitorResponse(const StatusFrame &frame, uint64_t now_us)
{
	ServoStatus &current = servo(_outstanding.side);

	if (frame.parameter_length != MonitorLength) {
		current.healthy = false;
		++_status.protocol_error_count;
		return;
	}

	current.last_valid_response = now_us;
	current.position_steps = static_cast<int16_t>(decodeSignedMagnitude(read16(&frame.parameters[0]), 15));
	current.speed_steps_s = static_cast<int16_t>(decodeSignedMagnitude(read16(&frame.parameters[2]), 15));
	current.load = static_cast<int16_t>(decodeSignedMagnitude(read16(&frame.parameters[4]), 10));
	current.voltage_v = frame.parameters[6] * 0.1f;
	current.temperature_c = frame.parameters[7];
	current.error_flags = frame.error;
	current.moving = frame.parameters[10] != 0;
	current.current_ma = read16(&frame.parameters[13]);
	current.position_valid = true;
	current.online = true;
	current.healthy = current.config_verified && current.error_flags == 0;
}

void PairController::acceptResponse(const StatusFrame &frame, uint64_t now_us)
{
	if (!_outstanding.valid || !_outstanding.expects_response || frame.servo_id != _outstanding.servo_id) {
		notifyProtocolError();
		return;
	}

	const RequestKind kind = _outstanding.kind;

	if (kind == RequestKind::Identity || kind == RequestKind::Protection
	    || kind == RequestKind::Mode) {
		finishBootResponse(frame, now_us);

	} else if (kind == RequestKind::StageMove) {
		finishMotionResponse(frame);

	} else if (kind == RequestKind::Monitor) {
		finishMonitorResponse(frame, now_us);
	}

	_outstanding = {};
	_request_retry_count = 0;
}

void PairController::completeNoResponse(bool transmitted, uint64_t now_us)
{
	if (!_outstanding.valid || _outstanding.expects_response) {
		return;
	}

	const RequestKind kind = _outstanding.kind;
	_outstanding = {};
	_last_request_us = now_us;

	if (!transmitted) {
		failOutstanding(CommandResult::ProtocolError);
		return;
	}

	if (kind == RequestKind::Action) {
		_motion_step = MotionStep::Idle;
		_status.command_result = CommandResult::Accepted;

	} else if (kind == RequestKind::Release) {
		_release_pending = false;
		_status.command_result = CommandResult::Accepted;
	}
}

void PairController::failOutstanding(CommandResult result)
{
	_motion_step = MotionStep::Idle;
	_release_pending = false;
	_status.command_result = result;
	_outstanding = {};
}

void PairController::notifyTimeout(uint64_t now_us)
{
	if (!_outstanding.valid || now_us - _last_request_us < ResponseTimeoutUs) {
		return;
	}

	++_status.timeout_count;
	const Side timed_out_side = _outstanding.side;
	ServoStatus &timed_out_servo = servo(timed_out_side);
	++timed_out_servo.timeout_count;
	_outstanding = {};
	_last_request_us = now_us;

	if (_request_retry_count < MaxRetries) {
		++_request_retry_count;
		++_status.retry_count;
		++timed_out_servo.retry_count;
		_retry_pending = true;

	} else {
		timed_out_servo.online = false;
		timed_out_servo.healthy = false;
		_request_retry_count = 0;
		_boot_step = _boot_step == BootStep::Complete ? _boot_step : BootStep::Failed;
		failOutstanding(CommandResult::Timeout);
	}
}

void PairController::notifyProtocolError()
{
	++_status.protocol_error_count;

	if (_outstanding.valid) {
		servo(_outstanding.side).healthy = false;
		failOutstanding(CommandResult::ProtocolError);
	}
}

void PairController::notifyRxProtocolError()
{
	++_status.protocol_error_count;
}

} // namespace hx65

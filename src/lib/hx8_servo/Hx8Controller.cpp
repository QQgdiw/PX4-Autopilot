#include "Hx8Controller.hpp"

#include <cmath>
#include <cstring>

namespace hx8
{

constexpr uint64_t Controller::ResponseTimeoutUs;
constexpr uint8_t Controller::MaxRetries;
constexpr uint64_t Controller::MinimumCommandSpacingUs;
constexpr uint64_t Controller::MovingMonitorIntervalUs;
constexpr uint64_t Controller::StableMonitorIntervalUs;
constexpr uint64_t Controller::CommandExpiryUs;

namespace
{

constexpr uint8_t BootParameters[] {33, 34, 36, 37, 38, 39, 40, 41, 42, 43, 46};
constexpr uint8_t WritableParameters[] {33, 37, 38, 39, 40, 41, 42, 43, 46};
constexpr uint8_t MovingFlag = 1u << 0;
constexpr uint8_t ErrorAndProtectionFlags = 0xfe;

uint16_t read16(const uint8_t *bytes)
{
	return static_cast<uint16_t>(bytes[0]) | (static_cast<uint16_t>(bytes[1]) << 8);
}

uint32_t read32(const uint8_t *bytes)
{
	return static_cast<uint32_t>(bytes[0]) | (static_cast<uint32_t>(bytes[1]) << 8)
	       | (static_cast<uint32_t>(bytes[2]) << 16) | (static_cast<uint32_t>(bytes[3]) << 24);
}

void write16(uint8_t *bytes, uint16_t value)
{
	bytes[0] = static_cast<uint8_t>(value);
	bytes[1] = static_cast<uint8_t>(value >> 8);
}

float adcToTemperature(uint16_t adc)
{
	if (adc == 0 || adc >= 4096) {
		return NAN;
	}

	const double sample = adc;
	return static_cast<float>(1.0 / (std::log(sample / (4096.0 - sample)) / 3435.0
					 + 1.0 / (273.15 + 25.0)) - 273.15);
}

} // namespace

bool Controller::calibrated(const ProtectionConfig &config)
{
	return config.stall_power_mw != 0 && config.temperature_adc != 0 && config.power_limit_mw != 0
	       && config.current_limit_ma != 0;
}

bool Controller::isByteParameter(uint8_t parameter)
{
	return parameter == 33 || parameter == 34 || parameter == 36 || parameter == 37 || parameter == 46;
}

void Controller::setExpectedConfig(const ProtectionConfig &config)
{
	_expected = config;
	_expected_calibrated = calibrated(config);
	_status.config_verified = false;
	_status.healthy = false;
	_boot_matches = true;
	_boot_index = 0;
	_boot_state = BootState::Ping;
	_write_state = WriteState::Idle;
	_status.persistent_write_active = false;
}

void Controller::setTarget(const MotionCommand &command)
{
	if (command.sequence <= _last_target_sequence || command.type != 0 || !std::isfinite(command.target_angle_deg)
	    || command.target_angle_deg < -180.f || command.target_angle_deg > 180.f) {
		_status.command_accepted = false;
		return;
	}

	if (_servo_id != command.servo_id) {
		_servo_id = command.servo_id;
		_status.servo_id = command.servo_id;

		if (_sent_any) {
			_status.config_verified = false;
			_status.healthy = false;
			_boot_matches = true;
			_boot_index = 0;
			_boot_state = BootState::Ping;
		}
	}

	_target = command;
	_last_target_sequence = command.sequence;
	_target_pending = true;
	_status.command_sequence = command.sequence;
	_status.command_accepted = false;
}

void Controller::requestRelease(uint32_t sequence)
{
	if (sequence > _last_release_sequence) {
		_last_release_sequence = sequence;
		_release_pending = true;
	}
}

void Controller::requestPersistentWrite()
{
	_persistent_write_requested = true;
}

PendingRequest Controller::makeRequest(RequestPriority priority, CommandId command, const uint8_t *payload,
				       uint8_t payload_length, uint64_t now_us)
{
	PendingRequest request {};
	request.valid = true;
	request.priority = priority;
	request.command = command;
	request.payload_length = payload_length;

	if (payload_length > 0) {
		memcpy(request.payload, payload, payload_length);
	}

	_outstanding = request;
	_retry_request = request;
	_last_request_us = now_us;
	_request_retry_count = 0;
	_sent_any = true;
	return request;
}

PendingRequest Controller::makeParameterRead(uint8_t parameter, uint64_t now_us)
{
	_outstanding_parameter = parameter;
	return makeRequest(RequestPriority::Config, CommandId::ParamRead, &parameter, 1, now_us);
}

PendingRequest Controller::makeParameterWrite(uint8_t parameter, uint64_t now_us)
{
	uint8_t payload[3] {parameter, 0, 0};
	write16(&payload[1], expectedParameterValue(parameter));
	_outstanding_parameter = parameter;
	return makeRequest(RequestPriority::Config, CommandId::ParamWrite, payload,
			   isByteParameter(parameter) ? 2 : 3, now_us);
}

PendingRequest Controller::update(const ControllerInput &input)
{
	PendingRequest none {};
	const bool fully_disarmed = !input.armed && !input.prearmed;
	const bool commissioning_allowed = fully_disarmed && input.explicit_commissioning && !input.lockdown
					   && !input.failsafe;

	if (_write_state != WriteState::Idle && !commissioning_allowed) {
		_outstanding = {};
		_retry_request = {};
		_retry_pending = false;
		_request_retry_count = 0;
		abortPersistentWrite();
	}

	if (_outstanding.valid) {
		return none;
	}

	if (_sent_any && input.now_us - _last_request_us < MinimumCommandSpacingUs) {
		return none;
	}

	if (_release_pending) {
		const uint8_t payload[] {0x10, 0, 0};
		_release_pending = false;
		_status.command_sequence = _last_release_sequence;
		return makeRequest(RequestPriority::EmergencyRelease, CommandId::Stop, payload, sizeof(payload), input.now_us);
	}

	if (_retry_pending) {
		_retry_pending = false;
		_outstanding = _retry_request;
		_last_request_us = input.now_us;
		return _outstanding;
	}

	if (_persistent_write_requested && _write_state == WriteState::Idle && commissioning_allowed
	    && _boot_state == BootState::Complete && _expected_calibrated) {
		_write_index = 0;
		_write_state = WriteState::Write;
		_status.persistent_write_active = true;
		_status.config_verified = false;
		_status.healthy = false;
	}

	if (_write_state == WriteState::Write) {
		return makeParameterWrite(WritableParameters[_write_index], input.now_us);
	}

	if (_write_state == WriteState::Readback) {
		return makeParameterRead(WritableParameters[_write_index], input.now_us);
	}

	if (_boot_state == BootState::Ping) {
		return makeRequest(RequestPriority::Diagnostic, CommandId::Ping, nullptr, 0, input.now_us);
	}

	if (_boot_state == BootState::Read) {
		return makeParameterRead(BootParameters[_boot_index], input.now_us);
	}

	const bool motion_allowed = (input.armed || input.prearmed) && !input.lockdown && !input.failsafe
				    && _status.online && _status.healthy && _status.config_verified;

	if (_target_pending) {
		if (input.now_us < _target.timestamp_us || input.now_us - _target.timestamp_us > CommandExpiryUs) {
			_target_pending = false;
			_status.command_accepted = false;

		} else if (motion_allowed) {
			uint8_t payload[10] {};
			write16(payload, static_cast<uint16_t>(static_cast<int16_t>(::lround(_target.target_angle_deg * 10.f))));
			write16(&payload[2], _target.move_time_ms);
			write16(&payload[4], _target.acceleration_time_ms);
			write16(&payload[6], _target.deceleration_time_ms);
			write16(&payload[8], _target.power_mw);
			_target_pending = false;
			return makeRequest(RequestPriority::Target, CommandId::TimedMove, payload, sizeof(payload), input.now_us);
		}
	}

	if (_boot_state == BootState::Complete) {
		const uint64_t interval = (_status.status_flags & MovingFlag) ? MovingMonitorIntervalUs : StableMonitorIntervalUs;

		if (input.now_us - _last_monitor_us >= interval) {
			_last_monitor_us = input.now_us;
			return makeRequest(RequestPriority::Status, CommandId::Status, nullptr, 0, input.now_us);
		}
	}

	return none;
}

bool Controller::responseShapeValid(const Frame &frame) const
{
	switch (frame.command) {
	case CommandId::Ping: return frame.payload_length == 0;

	case CommandId::ParamRead:
		return frame.payload_length == (isByteParameter(_outstanding_parameter) ? 1 : 2);

	case CommandId::ParamWrite:
	case CommandId::TimedMove:
	case CommandId::Stop: return frame.payload_length == 1;

	case CommandId::AngleRead: return frame.payload_length == 2;

	case CommandId::Status: return frame.payload_length == 15;
	}

	return false;
}

void Controller::acceptResponse(const Frame &frame, uint64_t now_us)
{
	if (!_outstanding.valid || frame.command != _outstanding.command || frame.servo_id != _servo_id
	    || !responseShapeValid(frame)) {
		++_status.rx_error_count;

		if (_write_state != WriteState::Idle) {
			_outstanding = {};
			_retry_request = {};
			_retry_pending = false;
			_request_retry_count = 0;
			abortPersistentWrite();
		}

		return;
	}

	_outstanding.valid = false;
	_retry_pending = false;
	_request_retry_count = 0;
	++_status.rx_valid_count;
	_status.sample_time_us = now_us;
	_status.last_valid_response_us = now_us;
	_status.online = true;

	switch (frame.command) {
	case CommandId::Ping:
		if (_boot_state == BootState::Ping) {
			_boot_state = BootState::Read;
		}

		break;

	case CommandId::ParamRead: handleParameterRead(frame, now_us); break;

	case CommandId::ParamWrite:
	case CommandId::TimedMove:
	case CommandId::Stop: handleCommandResponse(frame); break;

	case CommandId::Status: handleStatusResponse(frame); break;

	case CommandId::AngleRead:
		_status.angle_deg = static_cast<int16_t>(read16(frame.payload)) * 0.1f;
		break;
	}
}

void Controller::handleParameterRead(const Frame &frame, uint64_t now_us)
{
	const uint16_t value = frame.payload_length == 1 ? frame.payload[0] : read16(frame.payload);
	const bool matches = value == expectedParameterValue(_outstanding_parameter);

	if (_write_state == WriteState::Readback) {
		if (!matches) {
			abortPersistentWrite();
			return;
		}

		++_write_index;

		if (_write_index == sizeof(WritableParameters)) {
			_write_state = WriteState::Idle;
			_persistent_write_requested = false;
			_status.persistent_write_active = false;
			_status.config_verified = _expected_calibrated;
			_status.healthy = _status.config_verified && (_status.status_flags & ErrorAndProtectionFlags) == 0;

		} else {
			_write_state = WriteState::Write;
		}

		return;
	}

	if (_boot_state == BootState::Read) {
		finishBootRead(matches, now_us);
	}
}

void Controller::handleCommandResponse(const Frame &frame)
{
	_status.command_result = frame.payload[0];

	if (frame.command == CommandId::ParamWrite) {
		if (_write_state != WriteState::Write || frame.payload[0] != 0) {
			abortPersistentWrite();

		} else {
			_write_state = WriteState::Readback;
		}

	} else {
		_status.command_accepted = frame.payload[0] == 0;
	}
}

void Controller::handleStatusResponse(const Frame &frame)
{
	_status.voltage_v = read16(&frame.payload[0]) * 0.001f;
	_status.current_a = read16(&frame.payload[2]) * 0.001f;
	_status.power_w = read16(&frame.payload[4]) * 0.001f;
	_status.temperature_c = adcToTemperature(read16(&frame.payload[6]));
	_status.status_flags = frame.payload[8];
	_status.protection_flags = frame.payload[8] & ErrorAndProtectionFlags;
	_status.angle_deg = static_cast<int32_t>(read32(&frame.payload[9])) * 0.1f;
	_status.healthy = _status.config_verified && _status.protection_flags == 0;
}

void Controller::finishBootRead(bool matches, uint64_t now_us)
{
	_boot_matches = _boot_matches && matches;
	++_boot_index;

	if (_boot_index == sizeof(BootParameters)) {
		_boot_state = BootState::Complete;
		_status.config_verified = _expected_calibrated && _boot_matches;
		_status.healthy = _status.config_verified;
		_last_monitor_us = now_us;
	}
}

void Controller::abortPersistentWrite()
{
	_write_state = WriteState::Idle;
	_persistent_write_requested = false;
	_status.persistent_write_active = false;
	_status.config_verified = false;
	_status.healthy = false;
}

uint16_t Controller::expectedParameterValue(uint8_t parameter) const
{
	switch (parameter) {
	case 33: return _expected.response_enabled;

	case 34: return _servo_id;

	case 36: return 5;

	case 37: return _expected.stall_release_enabled;

	case 38: return _expected.stall_power_mw;

	case 39: return _expected.voltage_min_mv;

	case 40: return _expected.voltage_max_mv;

	case 41: return _expected.temperature_adc;

	case 42: return _expected.power_limit_mw;

	case 43: return _expected.current_limit_ma;

	case 46: return _expected.power_on_lock;

	default: return 0;
	}
}

void Controller::notifyTimeout(uint64_t now_us)
{
	if (!_outstanding.valid || now_us - _last_request_us < ResponseTimeoutUs) {
		return;
	}

	_outstanding.valid = false;

	if (_request_retry_count < MaxRetries) {
		++_request_retry_count;
		++_status.retry_count;
		_retry_pending = true;
		return;
	}

	++_status.timeout_count;
	_status.online = false;
	_status.healthy = false;
	_status.config_verified = false;
	_retry_pending = false;

	if (_write_state != WriteState::Idle) {
		abortPersistentWrite();
	}

	if (_boot_state != BootState::Complete) {
		_boot_state = BootState::Failed;
	}
}

} // namespace hx8

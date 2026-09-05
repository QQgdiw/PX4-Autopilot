#include "Hx8UartServo.hpp"

#include <cerrno>
#include <cmath>
#include <cinttypes>
#include <cstring>

#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>

using namespace time_literals;

namespace
{

constexpr uint64_t WorkQueueCommissioningTimeoutUs = 5_s;
constexpr uint64_t CliCommissioningTimeoutUs = 6_s;
constexpr int32_t MinimumSupplyMv = 9000;
constexpr int32_t MaximumSupplyMv = 12600;

template<typename T>
bool get_parameter(const char *name, T &value)
{
	const param_t handle = param_find(name);
	return handle != PARAM_INVALID && param_get(handle, &value) == PX4_OK;
}

bool valid_u16(int32_t value)
{
	return value > 0 && value <= UINT16_MAX;
}

bool baud_to_speed(int32_t baudrate, speed_t &speed)
{
	switch (baudrate) {
	case 50: speed = B50; break;

	case 75: speed = B75; break;

	case 110: speed = B110; break;

	case 134: speed = B134; break;

	case 150: speed = B150; break;

	case 200: speed = B200; break;

	case 300: speed = B300; break;

	case 600: speed = B600; break;

	case 1200: speed = B1200; break;

	case 1800: speed = B1800; break;

	case 2400: speed = B2400; break;

	case 4800: speed = B4800; break;

	case 9600: speed = B9600; break;

	case 19200: speed = B19200; break;

	case 38400: speed = B38400; break;

	case 57600: speed = B57600; break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 500000: speed = B500000; break;

	case 921600: speed = B921600; break;

	case 1000000: speed = B1000000; break;

	case 1500000: speed = B1500000; break;

	case 2000000: speed = B2000000; break;

	case 3000000: speed = B3000000; break;

	default: return false;
	}

	return true;
}

} // namespace

Hx8UartServo::Hx8UartServo(const char *device) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device))
{
	strncpy(_device, device, sizeof(_device) - 1);
}

Hx8UartServo::~Hx8UartServo()
{
	ScheduleClear();

	if (_fd >= 0) {
		close(_fd);
	}
}

bool Hx8UartServo::load_parameters(bool mixed_bus)
{
	int32_t baud{};
	int32_t id{};
	int32_t move{};
	int32_t acceleration{};
	int32_t deceleration{};
	int32_t run_power{};
	int32_t response{};
	int32_t stall_release{};
	int32_t stall_power{};
	int32_t temperature_limit_c{};
	int32_t power_limit{};
	int32_t current_limit{};
	int32_t voltage_min{};
	int32_t voltage_max{};
	int32_t power_on_lock{};
	float quad_angle{};
	float rover_angle{};
	float transition_time{};
	float gear_down{};
	float gear_clear{};
	float gear_stowed{};
	int32_t gear_move{};
	int32_t gear_acceleration{};
	int32_t gear_deceleration{};
	int32_t gear_power{};
	int32_t hx65_left_id{};
	int32_t hx65_right_id{};
	int32_t hx65_protection{};
	int32_t hx65_left_quad{};
	int32_t hx65_left_rover{};
	int32_t hx65_right_quad{};
	int32_t hx65_right_rover{};
	int32_t hx65_speed{};
	int32_t hx65_acceleration{};

	if (!get_parameter("HX_BAUD", baud) || !get_parameter("HX8_ID", id)
	    || !get_parameter("HX8_MOVE_T", move) || !get_parameter("HX8_ACC_T", acceleration)
	    || !get_parameter("HX8_DEC_T", deceleration) || !get_parameter("HX8_PWR_LIM", run_power)
	    || !get_parameter("HX8_CFG_RSP", response) || !get_parameter("HX8_CFG_STL", stall_release)
	    || !get_parameter("HX8_CFG_SPWR", stall_power) || !get_parameter("HX8_CFG_TEMP", temperature_limit_c)
	    || !get_parameter("HX8_CFG_PWR", power_limit) || !get_parameter("HX8_CFG_CUR", current_limit)
	    || !get_parameter("HX8_CFG_VMIN", voltage_min) || !get_parameter("HX8_CFG_VMAX", voltage_max)
	    || !get_parameter("HX8_CFG_BOOT", power_on_lock) || !get_parameter("HX8_ANG_QUD", quad_angle)
	    || !get_parameter("HX8_ANG_ROV", rover_angle) || !get_parameter("HYBRID_TRANS_T", transition_time)) {
		return false;
	}

	const bool valid_angles = std::isfinite(quad_angle) && std::isfinite(rover_angle)
				  && quad_angle >= -180.f && quad_angle <= 180.f
				  && rover_angle >= -180.f && rover_angle <= 180.f
				  && fabsf(quad_angle - rover_angle) > 1e-5f;
	const bool valid_timing = acceleration >= 0 && acceleration <= UINT16_MAX
				  && deceleration >= 0 && deceleration <= UINT16_MAX
				  && valid_u16(move) && move > acceleration + deceleration
				  && std::isfinite(transition_time) && transition_time > 0.f
				  && static_cast<float>(move) < transition_time * 1000.f;
	const bool valid_protection = response == 1
				      && stall_release == 1
				      && (power_on_lock == 0 || power_on_lock == 1)
				      && valid_u16(stall_power) && valid_u16(temperature_limit_c)
				      && valid_u16(power_limit) && valid_u16(current_limit)
				      && voltage_min >= MinimumSupplyMv && voltage_max <= MaximumSupplyMv
				      && voltage_min < voltage_max;

	if (id < 0 || id > 254 || !valid_protection) {
		return false;
	}

	if (mixed_bus) {
		if (!get_parameter("LG_ANG_DN", gear_down) || !get_parameter("LG_ANG_CLR", gear_clear)
		    || !get_parameter("LG_ANG_STW", gear_stowed) || !get_parameter("LG_MOVE_T", gear_move)
		    || !get_parameter("LG_ACC_T", gear_acceleration) || !get_parameter("LG_DEC_T", gear_deceleration)
		    || !get_parameter("LG_PWR_LIM", gear_power) || !get_parameter("H65_L_ID", hx65_left_id)
		    || !get_parameter("H65_R_ID", hx65_right_id) || !get_parameter("H65_PROT", hx65_protection)
		    || !get_parameter("H65_L_QUD", hx65_left_quad) || !get_parameter("H65_L_ROV", hx65_left_rover)
		    || !get_parameter("H65_R_QUD", hx65_right_quad) || !get_parameter("H65_R_ROV", hx65_right_rover)
		    || !get_parameter("H65_SPEED", hx65_speed) || !get_parameter("H65_ACC", hx65_acceleration)) {
			return false;
		}

		const auto valid_gear_angle = [](float angle) {
			return std::isfinite(angle) && angle >= -368640.f && angle <= 368640.f;
		};
		const bool valid_gear = valid_gear_angle(gear_down) && valid_gear_angle(gear_clear)
					&& valid_gear_angle(gear_stowed) && fabsf(gear_down - gear_clear) > 0.05f
					&& fabsf(gear_clear - gear_stowed) > 0.05f
					&& (gear_clear - gear_down) * (gear_stowed - gear_clear) > 0.f
					&& gear_acceleration >= 20
					&& gear_deceleration >= 20 && valid_u16(gear_move)
					&& gear_move > gear_acceleration + gear_deceleration && valid_u16(gear_power)
					&& gear_power <= power_limit;
		const auto valid_hx65_endpoint = [](int32_t endpoint) { return endpoint >= -30719 && endpoint <= 30719; };
		const bool valid_hx65_device = hx65_left_id >= 0 && hx65_left_id <= 253 && hx65_right_id >= 0
					       && hx65_right_id <= 253 && hx65_left_id != hx65_right_id && hx65_left_id != id
					       && hx65_right_id != id && hx65_protection >= 0 && hx65_protection <= 63
					       && hx65_speed > 0 && hx65_speed <= 3400 && hx65_acceleration >= 0
					       && hx65_acceleration <= 254;
		const bool valid_hx65_endpoints = valid_hx65_endpoint(hx65_left_quad)
						  && valid_hx65_endpoint(hx65_left_rover)
						  && valid_hx65_endpoint(hx65_right_quad) && valid_hx65_endpoint(hx65_right_rover)
						  && hx65_left_quad != hx65_left_rover && hx65_right_quad != hx65_right_rover;

		if (!valid_hx65_device) {
			return false;
		}

		_mixed_motion_config_valid = valid_gear && valid_hx65_endpoints;

		_gear_down_angle_deg = gear_down;
		_gear_clear_angle_deg = gear_clear;
		_gear_stowed_angle_deg = gear_stowed;
		_gear_move_time_ms = static_cast<uint16_t>(gear_move);
		_gear_acceleration_time_ms = static_cast<uint16_t>(gear_acceleration);
		_gear_deceleration_time_ms = static_cast<uint16_t>(gear_deceleration);
		_gear_run_power_mw = static_cast<uint16_t>(gear_power);
		_hx65_config.left_id = static_cast<uint8_t>(hx65_left_id);
		_hx65_config.right_id = static_cast<uint8_t>(hx65_right_id);
		_hx65_config.protection_mask = static_cast<uint8_t>(hx65_protection);
		_hx65_left_quad = static_cast<int16_t>(hx65_left_quad);
		_hx65_left_rover = static_cast<int16_t>(hx65_left_rover);
		_hx65_right_quad = static_cast<int16_t>(hx65_right_quad);
		_hx65_right_rover = static_cast<int16_t>(hx65_right_rover);
		_hx65_speed = static_cast<uint16_t>(hx65_speed);
		_hx65_acceleration = static_cast<uint8_t>(hx65_acceleration);
		_hx65_controller.setConfig(_hx65_config);

	} else if (!valid_u16(run_power) || run_power > power_limit || !valid_angles || !valid_timing) {
		return false;
	}

	_protection.response_enabled = static_cast<uint8_t>(response);
	_protection.stall_release_enabled = static_cast<uint8_t>(stall_release);
	_protection.stall_power_mw = static_cast<uint16_t>(stall_power);
	_protection.temperature_limit_c = static_cast<uint16_t>(temperature_limit_c);
	_protection.power_limit_mw = static_cast<uint16_t>(power_limit);
	_protection.current_limit_ma = static_cast<uint16_t>(current_limit);
	_protection.voltage_min_mv = static_cast<uint16_t>(voltage_min);
	_protection.voltage_max_mv = static_cast<uint16_t>(voltage_max);
	_protection.power_on_lock = static_cast<uint8_t>(power_on_lock);
	_baudrate = baud;
	_configured_servo_id = static_cast<uint8_t>(id);
	_quad_angle_deg = quad_angle;
	_rover_angle_deg = rover_angle;
	_move_time_ms = static_cast<uint16_t>(move);
	_acceleration_time_ms = static_cast<uint16_t>(acceleration);
	_deceleration_time_ms = static_cast<uint16_t>(deceleration);
	_run_power_mw = static_cast<uint16_t>(run_power);
	_transition_time_s = transition_time;
	_controller.setExpectedConfig(_protection);
	_controller.setServoId(_configured_servo_id);
	return true;
}

bool Hx8UartServo::endpoint_angle_match(float angle_deg) const
{
	constexpr float AngleToleranceDeg = 0.05f;
	return std::isfinite(angle_deg)
	       && (fabsf(angle_deg - _quad_angle_deg) <= AngleToleranceDeg
		   || fabsf(angle_deg - _rover_angle_deg) <= AngleToleranceDeg);
}

bool Hx8UartServo::valid_motion_command(const hx8_servo_command_s &command) const
{
	const bool gear_command = command.type == hx8_servo_command_s::COMMAND_GEAR_MOVE
				  || command.type == hx8_servo_command_s::COMMAND_GEAR_HOLD;

	if (_mixed_bus != gear_command || (_mixed_bus && !_mixed_motion_config_valid)) {
		return false;
	}

	const bool endpoint_match = gear_command
				    ? std::isfinite(command.target_angle_deg)
				    && ((command.type == hx8_servo_command_s::COMMAND_GEAR_HOLD
					 && command.target_angle_deg >= fminf(_gear_down_angle_deg, _gear_stowed_angle_deg)
					 && command.target_angle_deg <= fmaxf(_gear_down_angle_deg, _gear_stowed_angle_deg))
					|| (command.type == hx8_servo_command_s::COMMAND_GEAR_MOVE
					    && (fabsf(command.target_angle_deg - _gear_down_angle_deg) <= 0.05f
						|| fabsf(command.target_angle_deg - _gear_clear_angle_deg) <= 0.05f
						|| fabsf(command.target_angle_deg - _gear_stowed_angle_deg) <= 0.05f)))
				    : endpoint_angle_match(command.target_angle_deg);
	const float angle_limit = gear_command ? 368640.f : 180.f;
	const uint16_t move_time = gear_command ? _gear_move_time_ms : _move_time_ms;
	const uint16_t acceleration_time = gear_command ? _gear_acceleration_time_ms : _acceleration_time_ms;
	const uint16_t deceleration_time = gear_command ? _gear_deceleration_time_ms : _deceleration_time_ms;
	const uint16_t run_power = gear_command ? _gear_run_power_mw : _run_power_mw;

	if (command.servo_id != _configured_servo_id || !endpoint_match || fabsf(command.target_angle_deg) > angle_limit
	    || command.move_time_ms != move_time || command.acceleration_time_ms != acceleration_time
	    || command.deceleration_time_ms != deceleration_time || command.power_mw == 0
	    || command.power_mw > run_power || command.move_time_ms <= command.acceleration_time_ms
	    + command.deceleration_time_ms || (!gear_command
					       && static_cast<float>(command.move_time_ms) >= _transition_time_s * 1000.f)) {
		return false;
	}

	return true;
}

bool Hx8UartServo::valid_hx65_motion_command(const hx65_servo_command_s &command) const
{
	const bool quad = command.left_target_steps == _hx65_left_quad
			  && command.right_target_steps == _hx65_right_quad;
	const bool rover = command.left_target_steps == _hx65_left_rover
			   && command.right_target_steps == _hx65_right_rover;
	return _mixed_bus && _mixed_motion_config_valid && (quad || rover) && command.speed_steps_s == _hx65_speed
	       && command.acceleration == _hx65_acceleration;
}

int Hx8UartServo::configure_uart()
{
	termios uart_config{};
	speed_t speed{};

	if (!baud_to_speed(_baudrate, speed)) {
		PX4_ERR("unsupported HX bus baud rate: %" PRId32, _baudrate);
		return -EINVAL;
	}

	if (tcgetattr(_fd, &uart_config) != 0) {
		return -errno;
	}

	cfmakeraw(&uart_config);

	if (cfsetspeed(&uart_config, speed) != 0) {
		return -errno;
	}

	uart_config.c_cflag |= CLOCAL | CREAD;
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
	uart_config.c_cflag = (uart_config.c_cflag & ~CSIZE) | CS8;

	if (tcsetattr(_fd, TCSANOW, &uart_config) != 0) {
		return -errno;
	}

	if (tcflush(_fd, TCIFLUSH) != 0) {
		return -errno;
	}

	return PX4_OK;
}

int Hx8UartServo::init()
{
	int32_t backend{};
	const param_t backend_handle = param_find("HYB_ACT_TYPE");

	if (backend_handle == PARAM_INVALID || param_get(backend_handle, &backend) != PX4_OK
	    || (backend != 1 && backend != 2)) {
		return -EINVAL;
	}

	_mixed_bus = backend == 2;

	if (!load_parameters(_mixed_bus)) {
		return -EINVAL;
	}

	ScheduleOnInterval(5_ms);
	return PX4_OK;
}

int Hx8UartServo::open_uart()
{
	if (_fd >= 0) {
		return PX4_OK;
	}

	_fd = open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		return -errno;
	}

	const int uart_result = configure_uart();

	if (uart_result != PX4_OK) {
		close(_fd);
		_fd = -1;
		return uart_result;
	}

	PX4_INFO("HX servo bus %s configured for %" PRId32 " 8N1", _device, _baudrate);
	return PX4_OK;
}

void Hx8UartServo::receive()
{
	uint8_t buffer[64] {};
	const ssize_t bytes_read = read(_fd, buffer, sizeof(buffer));

	if (bytes_read < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			_controller.notifyTransportError();
		}

		return;
	}

	if (bytes_read > 0) {
		if (_active_protocol == ActiveProtocol::Hx8) {
			const uint64_t rx_time = hrt_absolute_time();
			_hx65_not_before = rx_time + Hx8ToHx65RecoveryUs;
			_prefer_hx65_after_hx8 = true;

			if (_hx65_boot_trace_active) {
				append_hx65_boot_trace(Hx65BootTraceType::Hx8Rx, _configured_servo_id,
						       hx65::RequestKind::None, 0, buffer,
						       static_cast<uint16_t>(bytes_read), rx_time);
			}

		} else if (_active_protocol == ActiveProtocol::Hx65 && _hx65_boot_trace_active) {
			append_hx65_boot_trace(Hx65BootTraceType::Hx65Rx, _hx65_controller.outstandingServoId(),
					       _hx65_controller.outstandingKind(), 0, buffer,
					       static_cast<uint16_t>(bytes_read), hrt_absolute_time());
		}
	}

	for (ssize_t i = 0; i < bytes_read; ++i) {
		if (_active_protocol == ActiveProtocol::Hx8) {
			hx8::Frame frame {};
			const hx8::ParseResult result = _parser.push(buffer[i], _controller.status().servo_id, frame);

			if (result == hx8::ParseResult::FrameReady) {
				const uint64_t response_time = hrt_absolute_time();
				_controller.acceptResponse(frame, response_time);
				_bus_quiet_until = response_time + BusQuietIntervalUs;
				_active_protocol = ActiveProtocol::None;

			} else if (result != hx8::ParseResult::NeedMore) {
				_controller.notifyProtocolError();
			}

		} else if (_active_protocol == ActiveProtocol::Hx65) {
			if (_hx65_rx_trace_size < Hx65RxTraceCapacity) {
				_hx65_rx_trace[_hx65_rx_trace_size++] = buffer[i];
			}

			hx65::StatusFrame frame {};
			const uint8_t expected_id = _hx65_controller.outstandingServoId();
			const hx65::ParseResult result = _hx65_parser.push(buffer[i], expected_id, frame);

			if (result == hx65::ParseResult::FrameReady) {
				const uint64_t response_time = hrt_absolute_time();
				const hx65::RequestKind response_kind = _hx65_controller.outstandingKind();
				append_hx65_boot_trace(Hx65BootTraceType::Hx65Parse, expected_id,
						       response_kind, static_cast<uint8_t>(result),
						       nullptr, 0, response_time);
				_hx65_controller.acceptResponse(frame, response_time);
				_bus_quiet_until = response_time + BusQuietIntervalUs;
				_active_protocol = ActiveProtocol::None;

			} else if (result != hx65::ParseResult::NeedMore) {
				append_hx65_boot_trace(Hx65BootTraceType::Hx65Parse, expected_id,
						       _hx65_controller.outstandingKind(), static_cast<uint8_t>(result),
						       nullptr, 0, hrt_absolute_time());
				capture_hx65_rx_trace(static_cast<uint8_t>(result), expected_id,
						      _hx65_controller.outstandingKind());

				// Keep waiting for the expected response after an unrelated or
				// damaged frame. The parser has already reset for resynchronization;
				// the normal timeout path owns retry and terminal failure.
				_hx65_controller.notifyRxProtocolError();
			}
		}
	}
}

void Hx8UartServo::start_hx65_boot_trace()
{
	_hx65_boot_trace_data_count = 0;
	_hx65_boot_trace_entry_count = 0;
	_hx65_boot_trace_next_entry = 0;
	_hx65_boot_trace_wrapped = false;
	_hx65_boot_trace_active = true;
	_hx65_boot_trace_meta.store(Hx65BootTraceActive
				    | (_hx65_monitor_trace_armed ? Hx65MonitorTraceArmed : 0));
}

void Hx8UartServo::arm_hx65_monitor_trace()
{
	_hx65_monitor_trace_armed = true;
	start_hx65_boot_trace();
}

void Hx8UartServo::append_hx65_boot_trace(Hx65BootTraceType type, uint8_t servo_id,
		hx65::RequestKind kind, uint8_t result, const uint8_t *data, uint16_t length, uint64_t now)
{
	if (!_hx65_boot_trace_active) {
		return;
	}

	uint16_t consumed = 0;

	do {
		uint16_t chunk = length - consumed;

		if (chunk > Hx65BootTraceBytesPerEntry) {
			chunk = Hx65BootTraceBytesPerEntry;
		}

		Hx65BootTraceEntry &entry = _hx65_boot_trace_entries[_hx65_boot_trace_next_entry];

		if (_hx65_boot_trace_entry_count == Hx65BootTraceEntryCapacity) {
			_hx65_boot_trace_data_count -= entry.data_length;
			_hx65_boot_trace_wrapped = true;

		} else {
			++_hx65_boot_trace_entry_count;
		}

		entry = {};
		entry.timestamp_us = now;
		entry.data_length = static_cast<uint8_t>(chunk);
		entry.type = static_cast<uint8_t>(type);
		entry.servo_id = servo_id;
		entry.request_kind = static_cast<uint8_t>(kind);
		entry.result = result;

		if (chunk > 0) {
			memcpy(entry.data, &data[consumed], chunk);
			_hx65_boot_trace_data_count += chunk;
			consumed += chunk;
		}

		_hx65_boot_trace_next_entry = static_cast<uint8_t>(
						      (_hx65_boot_trace_next_entry + 1u) % Hx65BootTraceEntryCapacity);
	} while (consumed < length);
}

void Hx8UartServo::freeze_hx65_boot_trace(Hx65BootTraceOutcome outcome, bool truncated)
{
	if (!_hx65_boot_trace_active) {
		return;
	}

	_hx65_boot_trace_active = false;
	_hx65_monitor_trace_armed = false;
	const uint32_t meta = Hx65BootTraceValid
			      | ((truncated || _hx65_boot_trace_wrapped) ? Hx65BootTraceTruncated : 0)
			      | (static_cast<uint32_t>(outcome) << 24)
			      | (static_cast<uint32_t>(_hx65_boot_trace_entry_count) << 16)
			      | _hx65_boot_trace_data_count;
	_hx65_boot_trace_meta.store(meta);
}

void Hx8UartServo::capture_hx65_rx_trace(uint8_t result, uint8_t expected_id, hx65::RequestKind kind)
{
	if ((_hx65_rx_error_meta.load() & Hx65RxTraceValid) != 0) {
		return;
	}

	uint32_t words[4] {};

	for (uint8_t byte = 0; byte < _hx65_rx_trace_size; ++byte) {
		words[byte / 4] |= static_cast<uint32_t>(_hx65_rx_trace[byte]) << (8 * (byte % 4));
	}

	_hx65_rx_error_trace_0.store(words[0]);
	_hx65_rx_error_trace_1.store(words[1]);
	_hx65_rx_error_trace_2.store(words[2]);
	_hx65_rx_error_trace_3.store(words[3]);
	const uint32_t meta = Hx65RxTraceValid | (static_cast<uint32_t>(result) << 24)
			      | (static_cast<uint32_t>(expected_id) << 16) | (static_cast<uint32_t>(kind) << 8)
			      | _hx65_rx_trace_size;
	_hx65_rx_error_meta.store(meta);
}

int Hx8UartServo::send(const hx8::PendingRequest &request)
{
	uint8_t frame[64] {};
	const size_t frame_size = hx8::encodeRequest(request.command, _controller.status().servo_id,
				  request.payload, request.payload_length, frame, sizeof(frame));

	if (frame_size == 0) {
		return -EINVAL;
	}

	const uint64_t tx_time = hrt_absolute_time();
	_hx65_not_before = tx_time + Hx8ToHx65RecoveryUs;
	_prefer_hx65_after_hx8 = true;
	append_hx65_boot_trace(Hx65BootTraceType::Hx8Tx, _configured_servo_id,
			       static_cast<hx65::RequestKind>(static_cast<uint8_t>(request.command)), 0, frame,
			       static_cast<uint16_t>(frame_size), tx_time);

	const ssize_t bytes_written = write(_fd, frame, frame_size);

	if (bytes_written != static_cast<ssize_t>(frame_size)) {
		return bytes_written >= 0 ? -EIO : -errno;
	}

	return tcdrain(_fd) == 0 ? PX4_OK : -errno;
}

int Hx8UartServo::send(const hx65::PendingRequest &request)
{
	uint8_t frame[hx65::MaxFrameSize] {};
	const size_t frame_size = hx65::encodeInstruction(request.instruction, request.servo_id, request.parameters,
				  request.parameter_length, frame, sizeof(frame));

	if (frame_size == 0) {
		return -EINVAL;
	}

	append_hx65_boot_trace(Hx65BootTraceType::Hx65Tx, request.servo_id, request.kind, 0, frame,
			       static_cast<uint16_t>(frame_size), hrt_absolute_time());

	const ssize_t bytes_written = write(_fd, frame, frame_size);

	if (bytes_written != static_cast<ssize_t>(frame_size)) {
		return bytes_written >= 0 ? -EIO : -errno;
	}

	return tcdrain(_fd) == 0 ? PX4_OK : -errno;
}

bool Hx8UartServo::try_send_hx65(uint64_t now)
{
	if (!_mixed_bus || _active_protocol != ActiveProtocol::None || now < _bus_quiet_until
	    || now < _hx65_not_before) {
		return false;
	}

	const auto request = _hx65_controller.update(now);

	if (!request.valid) {
		return false;
	}

	_prefer_hx65_after_hx8 = false;
	_hx65_parser.reset();
	_hx65_rx_trace_size = 0;
	_active_protocol = ActiveProtocol::Hx65;
	++_tx_count;
	const int send_result = send(request);

	if (!request.expects_response) {
		_hx65_controller.completeNoResponse(send_result == PX4_OK, now);
		_bus_quiet_until = hrt_absolute_time() + BusQuietIntervalUs;
		_active_protocol = ActiveProtocol::None;

	} else if (send_result != PX4_OK) {
		_hx65_controller.notifyProtocolError();
		_active_protocol = ActiveProtocol::None;
	}

	if (send_result != PX4_OK) {
		++_tx_error_count;
		_last_tx_error = send_result;
	}

	return true;
}

void Hx8UartServo::complete_commissioning(CommissioningState state)
{
	_explicit_commissioning = false;
	_commissioning_started = false;
	_commissioning_deadline = 0;
	_commissioning_state.store(static_cast<uint8_t>(state));
}

int Hx8UartServo::consume_commissioning_terminal()
{
	uint8_t observed = _commissioning_state.load();

	while (observed == static_cast<uint8_t>(CommissioningState::Success)
	       || observed == static_cast<uint8_t>(CommissioningState::Denied)
	       || observed == static_cast<uint8_t>(CommissioningState::Failed)) {
		const uint8_t terminal = observed;

		if (_commissioning_state.compare_exchange(&observed, static_cast<uint8_t>(CommissioningState::Idle))) {
			if (terminal == static_cast<uint8_t>(CommissioningState::Success)) {
				return PX4_OK;
			}

			return terminal == static_cast<uint8_t>(CommissioningState::Denied) ? -EPERM : -EIO;
		}
	}

	return -EAGAIN;
}

void Hx8UartServo::process_commissioning_request(uint64_t now)
{
	const auto state = static_cast<CommissioningState>(_commissioning_state.load());
	const bool unsafe = _armed.armed || _armed.prearmed || _armed.lockdown || _armed.manual_lockdown
			    || _armed.force_failsafe;

	if (state == CommissioningState::CancelRequested) {
		_controller.cancelPersistentWrite();
		complete_commissioning(CommissioningState::Idle);
		return;
	}

	if (state == CommissioningState::Requested) {
		if (unsafe) {
			complete_commissioning(CommissioningState::Denied);
			return;
		}

		_explicit_commissioning = true;
		_commissioning_started = false;
		_commissioning_deadline = now + WorkQueueCommissioningTimeoutUs;
		_controller.requestPersistentWrite();
		_commissioning_state.store(static_cast<uint8_t>(CommissioningState::Active));
		return;
	}

	if (state == CommissioningState::Active && (unsafe || now >= _commissioning_deadline)) {
		_controller.cancelPersistentWrite();
		complete_commissioning(unsafe ? CommissioningState::Denied : CommissioningState::Failed);
	}
}

void Hx8UartServo::finish_commissioning_request()
{
	if (static_cast<CommissioningState>(_commissioning_state.load()) != CommissioningState::Active) {
		return;
	}

	const auto &status = _controller.status();
	_commissioning_started = _commissioning_started || status.persistent_write_active;

	if (_commissioning_started && !status.persistent_write_active) {
		const bool success = status.persistent_write_result == hx8::OperationResult::Accepted
				     && status.config_check_complete && status.config_verified && status.healthy;
		complete_commissioning(success ? CommissioningState::Success : CommissioningState::Failed);

	} else if (status.persistent_write_result == hx8::OperationResult::ProtocolError
		   || status.persistent_write_result == hx8::OperationResult::Timeout
		   || status.persistent_write_result == hx8::OperationResult::Rejected) {
		complete_commissioning(CommissioningState::Failed);
	}
}

void Hx8UartServo::publish_atomic_status()
{
	const auto &status = _controller.status();
	uint32_t snapshot = 0;

	if (status.online) {
		snapshot |= StatusOnline;
	}

	if (status.healthy) {
		snapshot |= StatusHealthy;
	}

	if (status.config_verified) {
		snapshot |= StatusConfigVerified;
	}

	if (status.config_check_complete) {
		snapshot |= StatusConfigCheckComplete;
	}

	if (_mixed_bus) {
		const auto &pair = _hx65_controller.status();

		if (pair.servo[0].online && pair.servo[1].online) {
			snapshot |= StatusHx65Online;
		}

		if (pair.servo[0].healthy && pair.servo[1].healthy) {
			snapshot |= StatusHx65Healthy;
		}

		if (pair.servo[0].config_verified && pair.servo[1].config_verified && _mixed_motion_config_valid) {
			snapshot |= StatusHx65ConfigVerified;
		}

		const uint32_t positions = static_cast<uint16_t>(pair.servo[0].position_steps)
					   | (static_cast<uint32_t>(static_cast<uint16_t>(pair.servo[1].position_steps)) << 16);
		_hx65_position_snapshot.store(positions);
		_hx65_left_timeout_snapshot.store(pair.servo[0].timeout_count);
		_hx65_left_retry_snapshot.store(pair.servo[0].retry_count);
		_hx65_right_timeout_snapshot.store(pair.servo[1].timeout_count);
		_hx65_right_retry_snapshot.store(pair.servo[1].retry_count);
	}

	_status_snapshot.store(snapshot);
	const auto &pair = _hx65_controller.status();
	_status_error_count.store(status.timeout_count + status.protocol_error_count + status.transport_error_count
				  + (_mixed_bus ? pair.timeout_count + pair.protocol_error_count : 0));
}

void Hx8UartServo::publish_status()
{
	const auto &status = _controller.status();
	hx8_servo_status_s output {};
	output.timestamp = hrt_absolute_time();
	output.timestamp_sample = status.sample_time_us;
	output.last_valid_response = status.last_valid_response_us;
	output.command_sequence = status.command_sequence;
	output.rx_valid_count = status.rx_valid_count;
	output.rx_error_count = status.rx_error_count;
	output.timeout_count = status.timeout_count;
	output.retry_count = status.retry_count;
	output.servo_id = status.servo_id;
	output.online = status.online;
	output.healthy = status.healthy;
	output.config_verified = status.config_verified;
	output.command_accepted = status.command_accepted;
	output.persistent_write_active = status.persistent_write_active;
	output.angle_deg = status.angle_deg;
	output.voltage_v = status.voltage_v;
	output.current_a = status.current_a;
	output.power_w = status.power_w;
	output.temperature_c = status.temperature_c;
	output.status_flags = status.status_flags;
	output.protection_flags = status.protection_flags;
	output.command_result = status.command_result;
	_status_pub.publish(output);
}

void Hx8UartServo::publish_hx65_status()
{
	const hx65::PairStatus &status = _hx65_controller.status();
	const hx65::ServoStatus &left = status.servo[static_cast<uint8_t>(hx65::Side::Left)];
	const hx65::ServoStatus &right = status.servo[static_cast<uint8_t>(hx65::Side::Right)];
	hx65_servo_status_s output {};
	output.timestamp = hrt_absolute_time();
	output.left_last_valid_response = left.last_valid_response;
	output.right_last_valid_response = right.last_valid_response;
	output.command_sequence = status.command_sequence;
	output.timeout_count = status.timeout_count;
	output.retry_count = status.retry_count;
	output.protocol_error_count = status.protocol_error_count;
	output.command_result = static_cast<uint8_t>(status.command_result);
	output.left_id = _hx65_config.left_id;
	output.right_id = _hx65_config.right_id;
	output.left_online = left.online;
	output.right_online = right.online;
	output.left_healthy = left.healthy;
	output.right_healthy = right.healthy;
	output.left_config_verified = left.config_verified;
	output.right_config_verified = right.config_verified;
	output.motion_config_valid = _mixed_motion_config_valid;
	output.left_position_valid = left.position_valid;
	output.right_position_valid = right.position_valid;
	output.left_moving = left.moving;
	output.right_moving = right.moving;
	output.left_position_steps = left.position_steps;
	output.right_position_steps = right.position_steps;
	output.left_speed_steps_s = left.speed_steps_s;
	output.right_speed_steps_s = right.speed_steps_s;
	output.left_load = left.load;
	output.right_load = right.load;
	output.left_current_ma = left.current_ma;
	output.right_current_ma = right.current_ma;
	output.left_voltage_v = left.voltage_v;
	output.right_voltage_v = right.voltage_v;
	output.left_temperature_c = left.temperature_c;
	output.right_temperature_c = right.temperature_c;
	output.left_error_flags = left.error_flags;
	output.right_error_flags = right.error_flags;
	_hx65_status_pub.publish(output);
}

void Hx8UartServo::emit_events()
{
	const auto &status = _controller.status();
	const bool offline = !status.online && (status.timeout_count > 0 || status.transport_error_count > 0);
	const bool protection = status.protection_flags != 0;
	const bool mismatch = status.config_check_complete && status.online && !status.config_verified;
	const bool rejected = status.command_result == static_cast<uint8_t>(hx8::OperationResult::Rejected);
	const bool protocol = status.protocol_error_count > 0;

	if (offline && !_event_offline) {
		/* EVENT */
		events::send(events::ID("hx8_uart_communication"), events::Log::Critical,
			     "HX8 UART communication failure");
	}

	if (protection && !_event_protection) {
		if (!_first_protection_snapshot_valid.load()) {
			_first_protection_snapshot.sample_time_us = status.sample_time_us;
			_first_protection_snapshot.command_sequence = status.command_sequence;
			_first_protection_snapshot.angle_deg = status.angle_deg;
			_first_protection_snapshot.voltage_v = status.voltage_v;
			_first_protection_snapshot.current_a = status.current_a;
			_first_protection_snapshot.power_w = status.power_w;
			_first_protection_snapshot.temperature_c = status.temperature_c;
			_first_protection_snapshot.status_flags = status.status_flags;
			_first_protection_snapshot.protection_flags = status.protection_flags;
			_first_protection_snapshot.command_result = status.command_result;
			_first_protection_snapshot_valid.store(true);

			PX4_WARN("HX8 protection snapshot flags=0x%02x protection=0x%02x seq=%" PRIu32
				 " angle=%.3f V=%.3f I=%.3f P=%.3f T=%.2f result=%u",
				 _first_protection_snapshot.status_flags, _first_protection_snapshot.protection_flags,
				 _first_protection_snapshot.command_sequence, (double)_first_protection_snapshot.angle_deg,
				 (double)_first_protection_snapshot.voltage_v, (double)_first_protection_snapshot.current_a,
				 (double)_first_protection_snapshot.power_w, (double)_first_protection_snapshot.temperature_c,
				 (unsigned)_first_protection_snapshot.command_result);
		}

		/* EVENT */
		events::send(events::ID("hx8_uart_protection"), events::Log::Critical,
			     "HX8 actuator protection active");
	}

	if (mismatch && !_event_config) {
		/* EVENT */
		events::send(events::ID("hx8_uart_config"), events::Log::Critical,
			     "HX8 actuator configuration mismatch");
	}

	if (rejected && !_event_rejected) {
		/* EVENT */
		events::send(events::ID("hx8_uart_command"), events::Log::Critical,
			     "HX8 actuator command rejected");
	}

	if (protocol && !_event_protocol) {
		/* EVENT */
		events::send(events::ID("hx8_uart_protocol"), events::Log::Critical,
			     "HX8 UART protocol error");
	}

	_event_offline = offline;
	_event_protection = protection;
	_event_config = mismatch;
	_event_rejected = rejected;
	_event_protocol = protocol;
}

void Hx8UartServo::Run()
{
	const int uart_result = open_uart();

	if (uart_result != PX4_OK) {
		_last_tx_error = uart_result;
		publish_atomic_status();
		publish_status();
		return;
	}

	_armed_sub.copy(&_armed);
	_mode_sub.copy(&_mode);
	const uint64_t now = hrt_absolute_time();

	if (_mixed_bus && _hx65_controller.bootPending() && !_hx65_boot_trace_active
	    && (_hx65_boot_trace_meta.load() & (Hx65BootTraceValid | Hx65BootTraceActive)) == 0) {
		start_hx65_boot_trace();
	}

	process_commissioning_request(now);

	if (_command_sub.updated() && _command_sub.copy(&_command)) {
		if (_command.sequence <= _last_sequence) {
			PX4_ERR("local sequence reject seq=%" PRIu32 " last=%" PRIu32 " type=%u target=%.3f",
				_command.sequence, _last_sequence, _command.type, (double)_command.target_angle_deg);
			_controller.rejectCommand(_command.sequence);

		} else {
			_last_sequence = _command.sequence;

			if (_command.type == hx8_servo_command_s::COMMAND_RELEASE) {
				_controller.requestRelease(_command.sequence);

			} else if (_command.type == hx8_servo_command_s::COMMAND_MOVE
				   || _command.type == hx8_servo_command_s::COMMAND_HOLD
				   || _command.type == hx8_servo_command_s::COMMAND_GEAR_MOVE
				   || _command.type == hx8_servo_command_s::COMMAND_GEAR_HOLD) {
				if (!valid_motion_command(_command)) {
					PX4_ERR("local motion reject seq=%" PRIu32 " target=%.3f endpoints=%.3f/%.3f "
						"id=%u/%u time=%u/%u acc=%u/%u dec=%u/%u power=%u/%u trans=%.3f",
						_command.sequence, (double)_command.target_angle_deg, (double)_quad_angle_deg,
						(double)_rover_angle_deg, _command.servo_id, _configured_servo_id,
						_command.move_time_ms, _move_time_ms, _command.acceleration_time_ms, _acceleration_time_ms,
						_command.deceleration_time_ms, _deceleration_time_ms, _command.power_mw, _run_power_mw,
						(double)_transition_time_s);
					_controller.rejectCommand(_command.sequence);

				} else {
					const bool gear = _command.type == hx8_servo_command_s::COMMAND_GEAR_MOVE
							  || _command.type == hx8_servo_command_s::COMMAND_GEAR_HOLD;
					hx8::MotionCommand command {_command.timestamp, _command.sequence, _command.type, _configured_servo_id,
								    _command.target_angle_deg,
								    gear ? _gear_move_time_ms : _move_time_ms,
								    gear ? _gear_acceleration_time_ms : _acceleration_time_ms,
								    gear ? _gear_deceleration_time_ms : _deceleration_time_ms,
								    gear ? _gear_run_power_mw : _run_power_mw};
					_controller.setTarget(command);
					const auto &status = _controller.status();

					if (status.command_sequence == _command.sequence
					    && status.command_result == static_cast<uint8_t>(hx8::OperationResult::None)) {
						PX4_INFO("motion queued seq=%" PRIu32 " type=%u target=%.3f",
							 _command.sequence, _command.type, (double)_command.target_angle_deg);

					} else {
						PX4_ERR("controller motion reject seq=%" PRIu32 " status_seq=%" PRIu32 " result=%u",
							_command.sequence, status.command_sequence, status.command_result);
					}
				}

			} else {
				_controller.rejectCommand(_command.sequence);
			}
		}
	}

	if (_mixed_bus && _hx65_command_sub.updated() && _hx65_command_sub.copy(&_hx65_command)) {
		if (_hx65_command.sequence <= _hx65_last_sequence) {
			_hx65_controller.rejectCommand(_hx65_command.sequence);

		} else {
			_hx65_last_sequence = _hx65_command.sequence;

			if (_hx65_command.type == hx65_servo_command_s::COMMAND_RELEASE_PAIR) {
				_hx65_controller.requestRelease(_hx65_command.sequence);

			} else if (_hx65_command.type == hx65_servo_command_s::COMMAND_MOVE_PAIR
				   && !_armed.armed && !_armed.lockdown && !_armed.manual_lockdown && !_armed.force_failsafe
				   && valid_hx65_motion_command(_hx65_command)) {
				const hx65::PairCommand command {_hx65_command.timestamp, _hx65_command.sequence,
								 _hx65_command.left_target_steps, _hx65_command.right_target_steps,
								 _hx65_command.speed_steps_s, _hx65_command.acceleration};
				_hx65_controller.setTarget(command);

			} else {
				_hx65_controller.rejectCommand(_hx65_command.sequence);
			}
		}
	}

	receive();
	const bool hx65_waiting = _active_protocol == ActiveProtocol::Hx65
				  && _hx65_controller.hasOutstandingRequest();
	const bool hx8_waiting = _active_protocol == ActiveProtocol::Hx8 && _controller.hasOutstandingRequest();
	const hx8::CommandId hx8_command = hx8_waiting ? _controller.outstandingCommand() : hx8::CommandId::Status;
	const uint8_t hx65_expected_id = hx65_waiting ? _hx65_controller.outstandingServoId() : 0;
	const hx65::RequestKind hx65_kind = hx65_waiting ? _hx65_controller.outstandingKind() : hx65::RequestKind::None;
	const uint32_t hx8_timeouts_before = _controller.status().timeout_count;
	const uint32_t hx65_timeouts_before = _hx65_controller.status().timeout_count;
	_controller.notifyTimeout(now);
	_hx65_controller.notifyTimeout(now);

	if (hx8_waiting && _controller.status().timeout_count != hx8_timeouts_before) {
		append_hx65_boot_trace(Hx65BootTraceType::Hx8Timeout, _configured_servo_id,
				       static_cast<hx65::RequestKind>(static_cast<uint8_t>(hx8_command)), 0,
				       nullptr, 0, now);
	}

	if (hx65_waiting && _hx65_controller.status().timeout_count != hx65_timeouts_before) {
		append_hx65_boot_trace(Hx65BootTraceType::Hx65Timeout, hx65_expected_id, hx65_kind, 0,
				       nullptr, 0, now);

		if (hx65_kind == hx65::RequestKind::Monitor && _hx65_monitor_trace_armed) {
			freeze_hx65_boot_trace(Hx65BootTraceOutcome::MonitorTimeout);
		}

		capture_hx65_rx_trace(5, hx65_expected_id, hx65_kind);
	}

	if (_active_protocol == ActiveProtocol::Hx8 && !_controller.hasOutstandingRequest()) {
		_active_protocol = ActiveProtocol::None;
	}

	if (_active_protocol == ActiveProtocol::Hx65 && !_hx65_controller.hasOutstandingRequest()) {
		_active_protocol = ActiveProtocol::None;
	}

	// Complete the H65 pair's boot handshake before emitting any HX8 bytes.
	// Both vendor tools are reliable when only one protocol is active; keeping
	// this phase exclusive also prevents another device protocol from leaving
	// an H65 parser in a partial-frame state before its identity is verified.
	const bool hx65_boot_pending = _mixed_bus && _hx65_controller.bootPending();

	if (!hx65_boot_pending && !_hx65_monitor_trace_armed
	    && (_hx65_boot_trace_meta.load() & Hx65BootTraceValid) == 0) {
		const hx65::PairStatus &status = _hx65_controller.status();
		const bool verified = status.servo[0].config_verified && status.servo[1].config_verified;

		if (verified) {
			// Boot succeeded. Reuse the bounded buffer for the first failed
			// steady-state monitor transaction instead of retaining normal boot traffic.
			arm_hx65_monitor_trace();

		} else if (_hx65_boot_trace_active) {
			freeze_hx65_boot_trace(Hx65BootTraceOutcome::Failed);
		}
	}

	const hx8::ControllerInput input {now, _armed.armed, _armed.prearmed,
					  _armed.lockdown || _armed.manual_lockdown, _armed.force_failsafe,
					  _explicit_commissioning};
	const bool hx65_recovery_ready = _prefer_hx65_after_hx8 && now >= _hx65_not_before
					 && now >= _bus_quiet_until && !_controller.emergencyReleasePending();

	if (hx65_recovery_ready && !try_send_hx65(now)) {
		// No H65 work is due at the end of the recovery interval. Do not
		// reserve the UART against ordinary HX8 work until another HX8 transaction.
		_prefer_hx65_after_hx8 = false;
	}

	if (!hx65_boot_pending && _active_protocol == ActiveProtocol::None && now >= _bus_quiet_until) {
		const auto request = _controller.update(input);

		if (request.valid) {
			_parser.reset();
			_active_protocol = ActiveProtocol::Hx8;
			++_tx_count;
			const int send_result = send(request);

			if (send_result != PX4_OK) {
				++_tx_error_count;
				_last_tx_error = send_result;
				_controller.notifyTransportError();
				_active_protocol = ActiveProtocol::None;
			}
		}
	}


	try_send_hx65(now);

	finish_commissioning_request();
	emit_events();
	publish_atomic_status();
	publish_status();

	if (_mixed_bus) {
		publish_hx65_status();
	}
}

int Hx8UartServo::print_status()
{
	const uint32_t snapshot = _status_snapshot.load();
	PX4_INFO("HX8 UART device=%s online=%d healthy=%d verified=%d errors=%" PRIu32
		 " tx=%" PRIu32 " tx_errors=%" PRIu32 " last_tx_error=%d endpoints=%.3f/%.3f",
		 _device,
		 (snapshot & StatusOnline) != 0, (snapshot & StatusHealthy) != 0,
		 (snapshot & StatusConfigVerified) != 0, _status_error_count.load(),
		 _tx_count, _tx_error_count, _last_tx_error, (double)_quad_angle_deg,
		 (double)_rover_angle_deg);

	if (_first_protection_snapshot_valid.load()) {
		const ProtectionSnapshot &protection = _first_protection_snapshot;
		PX4_INFO("HX8 first protection t=%" PRIu64 " flags=0x%02x protection=0x%02x seq=%" PRIu32
			 " angle=%.3f V=%.3f I=%.3f P=%.3f T=%.2f result=%u",
			 protection.sample_time_us, protection.status_flags, protection.protection_flags,
			 protection.command_sequence, (double)protection.angle_deg, (double)protection.voltage_v,
			 (double)protection.current_a, (double)protection.power_w,
			 (double)protection.temperature_c, (unsigned)protection.command_result);

	} else {
		PX4_INFO("HX8 first protection=none");
	}

	if (_mixed_bus) {
		const uint32_t positions = _hx65_position_snapshot.load();
		PX4_INFO("HX65 pair online=%d healthy=%d verified=%d motion_config=%d left=%d right=%d",
			 (snapshot & StatusHx65Online) != 0, (snapshot & StatusHx65Healthy) != 0,
			 (snapshot & StatusHx65ConfigVerified) != 0, _mixed_motion_config_valid,
			 static_cast<int16_t>(positions & 0xffffu), static_cast<int16_t>(positions >> 16));
		PX4_INFO("HX65 comm left timeout=%" PRIu32 " retry=%" PRIu32
			 " right timeout=%" PRIu32 " retry=%" PRIu32,
			 _hx65_left_timeout_snapshot.load(), _hx65_left_retry_snapshot.load(),
			 _hx65_right_timeout_snapshot.load(), _hx65_right_retry_snapshot.load());

		const uint32_t trace_meta = _hx65_rx_error_meta.load();

		if ((trace_meta & Hx65RxTraceValid) != 0) {
			const uint32_t words[] {_hx65_rx_error_trace_0.load(), _hx65_rx_error_trace_1.load(),
						_hx65_rx_error_trace_2.load(), _hx65_rx_error_trace_3.load()
					       };
			const uint8_t trace_size = static_cast<uint8_t>(trace_meta);
			char trace_hex[2 * Hx65RxTraceCapacity + 1] {};
			constexpr char digits[] = "0123456789abcdef";

			for (uint8_t byte = 0; byte < trace_size; ++byte) {
				const uint8_t value = static_cast<uint8_t>(words[byte / 4] >>(8 * (byte % 4)));
				trace_hex[2 * byte] = digits[value >> 4];
				trace_hex[2 * byte + 1] = digits[value & 0x0f];
			}

			PX4_INFO("HX65 first rx error result=%u expected_id=%u kind=%u bytes=%s",
				 (unsigned)((trace_meta >> 24) & 0x0f), (unsigned)((trace_meta >> 16) & 0xff),
				 (unsigned)((trace_meta >> 8) & 0xff), trace_hex);
		}
	}

	return PX4_OK;
}

int Hx8UartServo::cli_config_check()
{
	const uint32_t snapshot = _status_snapshot.load();
	uint32_t required = StatusOnline | StatusHealthy | StatusConfigVerified | StatusConfigCheckComplete;

	if (_mixed_bus) {
		required |= StatusHx65Online | StatusHx65Healthy | StatusHx65ConfigVerified;
	}

	return (snapshot & required) == required ? PX4_OK : -EIO;
}

int Hx8UartServo::cli_trace()
{
	const uint32_t meta = _hx65_boot_trace_meta.load();

	if ((meta & Hx65BootTraceValid) == 0) {
		PX4_INFO("HX65 trace active=%d monitor_armed=%d; retry after first monitor timeout",
			 (meta & Hx65BootTraceActive) != 0, (meta & Hx65MonitorTraceArmed) != 0);
		return -EAGAIN;
	}

	const uint8_t entry_count = static_cast<uint8_t>(meta >> 16);
	const uint16_t data_count = static_cast<uint16_t>(meta);
	const unsigned outcome = (meta >> 24) & 0x0f;
	PX4_INFO("HX65 diagnostic trace outcome=%u wrapped=%d entries=%u bytes=%u",
		 outcome, (meta & Hx65BootTraceTruncated) != 0, (unsigned)entry_count, (unsigned)data_count);
	constexpr char digits[] = "0123456789abcdef";
	const uint8_t first_entry = entry_count == Hx65BootTraceEntryCapacity ? _hx65_boot_trace_next_entry : 0;
	const uint64_t first_timestamp = entry_count > 0 ? _hx65_boot_trace_entries[first_entry].timestamp_us : 0;

	for (uint8_t index = 0; index < entry_count; ++index) {
		const uint8_t slot = static_cast<uint8_t>((first_entry + index) % Hx65BootTraceEntryCapacity);
		const Hx65BootTraceEntry &entry = _hx65_boot_trace_entries[slot];
		const char *type = entry.type == static_cast<uint8_t>(Hx65BootTraceType::Hx8Tx) ? "HX8_TX"
				   : entry.type == static_cast<uint8_t>(Hx65BootTraceType::Hx8Rx) ? "HX8_RX"
				   : entry.type == static_cast<uint8_t>(Hx65BootTraceType::Hx8Timeout) ? "HX8_TIMEOUT"
				   : entry.type == static_cast<uint8_t>(Hx65BootTraceType::Hx65Tx) ? "H65_TX"
				   : entry.type == static_cast<uint8_t>(Hx65BootTraceType::Hx65Rx) ? "H65_RX"
				   : entry.type == static_cast<uint8_t>(Hx65BootTraceType::Hx65Parse) ? "H65_PARSE"
				   : "H65_TIMEOUT";
		PX4_INFO("HX trace %02u +%" PRIu64 "us %s id=%u code=%u result=%u len=%u",
			 (unsigned)index, entry.timestamp_us - first_timestamp, type, (unsigned)entry.servo_id,
			 (unsigned)entry.request_kind, (unsigned)entry.result,
			 (unsigned)entry.data_length);

		for (uint8_t offset = 0; offset < entry.data_length; offset += 16) {
			const uint8_t chunk = entry.data_length - offset > 16 ? 16 : entry.data_length - offset;
			char hex[2 * 16 + 1] {};

			for (uint8_t byte = 0; byte < chunk; ++byte) {
				const uint8_t value = entry.data[offset + byte];
				hex[2 * byte] = digits[value >> 4];
				hex[2 * byte + 1] = digits[value & 0x0f];
			}

			PX4_INFO("HX data %02u.%u offset=%u bytes=%s", (unsigned)index,
				 (unsigned)(offset / 16), (unsigned)offset, hex);
		}
	}

	return PX4_OK;
}

int Hx8UartServo::cli_config_write()
{
	uint8_t expected = static_cast<uint8_t>(CommissioningState::Idle);

	if (!_commissioning_state.compare_exchange(&expected, static_cast<uint8_t>(CommissioningState::Requested))) {
		return -EBUSY;
	}

	const uint64_t deadline = hrt_absolute_time() + CliCommissioningTimeoutUs;

	while (hrt_absolute_time() < deadline) {
		const int result = consume_commissioning_terminal();

		if (result != -EAGAIN) {
			return result;
		}

		px4_usleep(20_ms);
	}

	expected = static_cast<uint8_t>(CommissioningState::Requested);

	if (!_commissioning_state.compare_exchange(&expected, static_cast<uint8_t>(CommissioningState::CancelRequested))) {
		expected = static_cast<uint8_t>(CommissioningState::Active);

		if (!_commissioning_state.compare_exchange(&expected, static_cast<uint8_t>(CommissioningState::CancelRequested))) {
			// The work queue may have completed between the last poll and cancel CAS.
			// Consume that terminal state so the next config write is not stranded busy.
			const int result = consume_commissioning_terminal();

			if (result != -EAGAIN) {
				return result;
			}
		}
	}

	return -ETIMEDOUT;
}

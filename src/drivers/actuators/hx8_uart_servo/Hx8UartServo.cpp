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

bool Hx8UartServo::load_parameters()
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

	if (!get_parameter("HX8_BAUD", baud) || !get_parameter("HX8_ID", id)
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

	if (baud != 115200 || id < 0 || id > 254 || !valid_u16(run_power)
	    || run_power > power_limit || !valid_angles || !valid_timing || !valid_protection) {
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
	if (command.servo_id != _configured_servo_id || !endpoint_angle_match(command.target_angle_deg)
	    || command.target_angle_deg < -180.f || command.target_angle_deg > 180.f
	    || command.move_time_ms != _move_time_ms || command.acceleration_time_ms != _acceleration_time_ms
	    || command.deceleration_time_ms != _deceleration_time_ms || command.power_mw == 0
	    || command.power_mw > _run_power_mw || command.move_time_ms <= command.acceleration_time_ms
	    + command.deceleration_time_ms
	    || static_cast<float>(command.move_time_ms) >= _transition_time_s * 1000.f) {
		return false;
	}

	return true;
}

int Hx8UartServo::configure_uart()
{
	termios uart_config{};

	if (tcgetattr(_fd, &uart_config) != 0) {
		return -errno;
	}

	cfmakeraw(&uart_config);
	cfsetspeed(&uart_config, B115200);
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

	if (backend_handle == PARAM_INVALID || param_get(backend_handle, &backend) != PX4_OK || backend != 1) {
		return -EINVAL;
	}

	if (!load_parameters()) {
		return -EINVAL;
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

	ScheduleOnInterval(5_ms);
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

	for (ssize_t i = 0; i < bytes_read; ++i) {
		hx8::Frame frame {};
		const hx8::ParseResult result = _parser.push(buffer[i], _controller.status().servo_id, frame);

		if (result == hx8::ParseResult::FrameReady) {
			_controller.acceptResponse(frame, hrt_absolute_time());

		} else if (result != hx8::ParseResult::NeedMore) {
			_controller.notifyProtocolError();
		}
	}
}

int Hx8UartServo::send(const hx8::PendingRequest &request)
{
	uint8_t frame[64] {};
	const size_t frame_size = hx8::encodeRequest(request.command, _controller.status().servo_id,
				  request.payload, request.payload_length, frame, sizeof(frame));

	if (frame_size == 0) {
		return -EINVAL;
	}

	const ssize_t bytes_written = write(_fd, frame, frame_size);

	if (bytes_written != static_cast<ssize_t>(frame_size)) {
		return bytes_written >= 0 ? -EIO : -errno;
	}

	return tcdrain(_fd) == 0 ? PX4_OK : -errno;
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

	_status_snapshot.store(snapshot);
	_status_error_count.store(status.timeout_count + status.protocol_error_count + status.transport_error_count);
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
	_armed_sub.copy(&_armed);
	_mode_sub.copy(&_mode);
	const uint64_t now = hrt_absolute_time();
	process_commissioning_request(now);

	if (_command_sub.updated() && _command_sub.copy(&_command)) {
		if (_command.sequence <= _last_sequence) {
			_controller.rejectCommand(_command.sequence);

		} else {
			_last_sequence = _command.sequence;

			if (_command.type == hx8_servo_command_s::COMMAND_RELEASE) {
				_controller.requestRelease(_command.sequence);

			} else if (_command.type == hx8_servo_command_s::COMMAND_MOVE
				   || _command.type == hx8_servo_command_s::COMMAND_HOLD) {
				if (!valid_motion_command(_command)) {
					_controller.rejectCommand(_command.sequence);

				} else {
					hx8::MotionCommand command {_command.timestamp, _command.sequence, 0, _configured_servo_id,
								    _command.target_angle_deg, _move_time_ms,
								    _acceleration_time_ms, _deceleration_time_ms,
								    _run_power_mw};
					_controller.setTarget(command);
				}

			} else {
				_controller.rejectCommand(_command.sequence);
			}
		}
	}

	receive();
	_controller.notifyTimeout(now);
	const hx8::ControllerInput input {now, _armed.armed, _armed.prearmed,
					  _armed.lockdown || _armed.manual_lockdown, _armed.force_failsafe,
					  _explicit_commissioning};
	const auto request = _controller.update(input);

	if (request.valid && send(request) != PX4_OK) {
		_controller.notifyTransportError();
	}

	finish_commissioning_request();
	emit_events();
	publish_atomic_status();
	publish_status();
}

int Hx8UartServo::print_status()
{
	const uint32_t snapshot = _status_snapshot.load();
	PX4_INFO("HX8 UART online=%d healthy=%d verified=%d errors=%" PRIu32,
		 (snapshot & StatusOnline) != 0, (snapshot & StatusHealthy) != 0,
		 (snapshot & StatusConfigVerified) != 0, _status_error_count.load());
	return PX4_OK;
}

int Hx8UartServo::cli_config_check()
{
	const uint32_t snapshot = _status_snapshot.load();
	const uint32_t required = StatusOnline | StatusHealthy | StatusConfigVerified | StatusConfigCheckComplete;
	return (snapshot & required) == required ? PX4_OK : -EIO;
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

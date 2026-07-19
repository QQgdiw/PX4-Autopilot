#include "Hx8UartServo.hpp"

#include <cerrno>
#include <cstring>
#include <cmath>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/events.h>

using namespace time_literals;

Hx8UartServo::Hx8UartServo(const char *device) : ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device))
{
	strncpy(_device, device, sizeof(_device) - 1);
}

Hx8UartServo::~Hx8UartServo()
{
	ScheduleClear();
	if (_fd >= 0) { close(_fd); }
}

bool Hx8UartServo::load_parameters()
{
	auto get = [](const char *name, int32_t &v) {
		param_t p = param_find(name); return p != PARAM_INVALID && param_get(p, &v) == 0;
	};
	int32_t v{};
	if (!get("HX8_ID", v) || v < 0 || v > 254) return false;
	const uint8_t id = static_cast<uint8_t>(v);
	get("HX8_CFG_RSP", v); _protection.response_enabled = (uint8_t)v;
	get("HX8_CFG_STL", v); _protection.stall_release_enabled = (uint8_t)v;
	get("HX8_CFG_SPWR", v); _protection.stall_power_mw = (uint16_t)v;
	get("HX8_CFG_TADC", v); _protection.temperature_adc = (uint16_t)v;
	get("HX8_CFG_PWR", v); _protection.power_limit_mw = (uint16_t)v;
	get("HX8_CFG_CUR", v); _protection.current_limit_ma = (uint16_t)v;
	get("HX8_CFG_VMIN", v); _protection.voltage_min_mv = (uint16_t)v;
	get("HX8_CFG_VMAX", v); _protection.voltage_max_mv = (uint16_t)v;
	get("HX8_CFG_BOOT", v); _protection.power_on_lock = (uint8_t)v;
	_controller.setExpectedConfig(_protection);
	_controller.setServoId(id);
	return valid_parameters();
}

bool Hx8UartServo::valid_parameters() const
{
	int32_t baud{}, id{}, move{}, acc{}, dec{}, power{};
	float quad{}, rover{}, trans{};
	auto get = [](const char *n, auto &v) { param_t p=param_find(n); return p != PARAM_INVALID && param_get(p,&v)==0; };
	if (!get("HX8_BAUD", baud) || !get("HX8_ID", id) || !get("HX8_MOVE_T", move) ||
	    !get("HX8_ACC_T", acc) || !get("HX8_DEC_T", dec) || !get("HX8_PWR_LIM", power) ||
	    !get("HX8_ANG_QUD", quad) || !get("HX8_ANG_ROV", rover) || !get("HYBRID_TRANS_T", trans)) return false;
	return baud == 115200 && id >= 0 && id <= 254 && std::isfinite(quad) && std::isfinite(rover) &&
	       fabsf(quad - rover) > 1e-5f && move > acc + dec && move < trans * 1000.f && power > 0 && _protection.stall_power_mw &&
	       _protection.temperature_adc && _protection.power_limit_mw && _protection.current_limit_ma;
}

int Hx8UartServo::configure_uart()
{
	termios tio{};
	if (tcgetattr(_fd, &tio) != 0) return -errno;
	cfmakeraw(&tio); cfsetspeed(&tio, B115200);
	tio.c_cflag |= CLOCAL | CREAD; tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS); tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
	if (tcsetattr(_fd, TCSANOW, &tio) != 0) return -errno;
	tcflush(_fd, TCIFLUSH); return 0;
}

int Hx8UartServo::init()
{
	int32_t backend{};
	param_t p = param_find("HYB_ACT_TYPE");
	if (p == PARAM_INVALID || param_get(p, &backend) != 0 || backend != 1) return -EINVAL;
	if (!load_parameters()) return -EINVAL;
	_fd = open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (_fd < 0 || configure_uart() != 0) return -errno;
	ScheduleOnInterval(5_ms);
	return 0;
}

void Hx8UartServo::receive()
{
	uint8_t buf[64]; const ssize_t n = read(_fd, buf, sizeof(buf));
	if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
		_controller.notifyTimeout(hrt_absolute_time() + hx8::Controller::ResponseTimeoutUs);
		return;
	}
	for (ssize_t i=0; i<n; i++) { hx8::Frame frame{}; if (_parser.push(buf[i], _controller.status().servo_id, frame) == hx8::ParseResult::FrameReady) _controller.acceptResponse(frame, hrt_absolute_time()); }
}

int Hx8UartServo::send(const hx8::PendingRequest &request)
{
	uint8_t frame[64]; const size_t n = hx8::encodeRequest(request.command, _controller.status().servo_id, request.payload, request.payload_length, frame, sizeof(frame));
	if (!n) {
		return -EINVAL;
	}
	if (write(_fd, frame, n) != (ssize_t)n) {
		return -errno;
	}
	return tcdrain(_fd);
}

void Hx8UartServo::publish_status()
{
	const auto &s = _controller.status(); hx8_servo_status_s out{};
	out.timestamp = hrt_absolute_time(); out.timestamp_sample = s.sample_time_us; out.last_valid_response = s.last_valid_response_us;
	out.command_sequence = s.command_sequence; out.rx_valid_count=s.rx_valid_count; out.rx_error_count=s.rx_error_count; out.timeout_count=s.timeout_count; out.retry_count=s.retry_count;
	out.servo_id=s.servo_id; out.online=s.online; out.healthy=s.healthy; out.config_verified=s.config_verified; out.command_accepted=s.command_accepted; out.persistent_write_active=s.persistent_write_active;
	out.angle_deg=s.angle_deg; out.voltage_v=s.voltage_v; out.current_a=s.current_a; out.power_w=s.power_w; out.temperature_c=s.temperature_c; out.status_flags=s.status_flags; out.protection_flags=s.protection_flags; out.command_result=s.command_result;
	if (s.command_sequence != 0) out.command_result = s.command_accepted ? hx8_servo_status_s::RESULT_ACCEPTED : hx8_servo_status_s::RESULT_REJECTED;
	_status_pub.publish(out);
}

void Hx8UartServo::emit_events()
{
	const auto &s = _controller.status();
	const bool offline = !s.online;
	const bool protection = s.protection_flags != 0;
	const bool mismatch = s.online && !s.config_verified;
	const bool rejected = s.command_sequence != 0 && !s.command_accepted;
	if (offline && !_event_offline) {
		/* EVENT */
		events::send(events::ID("hx8_uart_communication"), events::Log::Critical,
				     "HX8 UART communication timeout");
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
	_event_offline = offline; _event_protection = protection; _event_config = mismatch; _event_rejected = rejected;
}

void Hx8UartServo::Run()
{
	_armed_sub.copy(&_armed);
	_mode_sub.copy(&_mode);
	const uint64_t now = hrt_absolute_time();
	if (_command_sub.updated() && _command_sub.copy(&_command) && _command.sequence > _last_sequence) {
		_last_sequence = _command.sequence;
		if (_command.type == hx8_servo_command_s::COMMAND_RELEASE) _controller.requestRelease(_command.sequence);
		else if (_command.type == hx8_servo_command_s::COMMAND_MOVE || _command.type == hx8_servo_command_s::COMMAND_HOLD) {
			hx8::MotionCommand cmd{_command.timestamp, _command.sequence, 0, _command.servo_id, _command.target_angle_deg, _command.move_time_ms, _command.acceleration_time_ms, _command.deceleration_time_ms, _command.power_mw};
			_controller.setTarget(cmd);
		}
	}
	receive();
	_controller.notifyTimeout(now);
	hx8::ControllerInput input{now, _armed.armed, _armed.prearmed, _armed.lockdown || _armed.manual_lockdown, _armed.force_failsafe, _explicit_commissioning};
	const auto request = _controller.update(input);
	if (request.valid && send(request) != 0) {
		_controller.notifyTimeout(now + hx8::Controller::ResponseTimeoutUs);
	}
	emit_events(); publish_status();
}

int Hx8UartServo::print_status() { PX4_INFO("HX8 UART fd=%d online=%d verified=%d", _fd, _controller.status().online, _controller.status().config_verified); return 0; }

int Hx8UartServo::cli_config_check()
{
	return (_controller.status().online && _controller.status().healthy && _controller.status().config_verified) ? 0 : -EIO;
}

int Hx8UartServo::cli_config_write()
{
	actuator_armed_s live{};
	if (!_armed_sub.copy(&live) || live.armed || live.prearmed || live.lockdown || live.manual_lockdown || live.force_failsafe) return -EPERM;
	_explicit_commissioning = true;
	_controller.requestPersistentWrite();
	const uint64_t deadline = hrt_absolute_time() + 5_s;
	while (hrt_absolute_time() < deadline) {
		if (!_armed_sub.copy(&live) || live.armed || live.prearmed || live.lockdown || live.manual_lockdown || live.force_failsafe) break;
		if (!_controller.status().persistent_write_active && _controller.status().config_verified) {
			_explicit_commissioning = false;
			return 0;
		}
		px4_usleep(20_ms);
	}
	_explicit_commissioning = false;
	return -ETIMEDOUT;
}

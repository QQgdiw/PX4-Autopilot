#include "M2006Can.hpp"
#include "CanOwnership.hpp"

#include <cstring>

#include <px4_platform_common/board_common.h>
#include <px4_platform_common/events.h>

using hybrid_control::MotorIndex;

namespace
{

int16_t boundedCurrentLimit(const int32_t value)
{
	return static_cast<int16_t>(value < 0 ? 0 : (value > 10000 ? 10000 : value));
}

} // namespace

M2006Can::M2006Can() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan)
{
}

M2006Can::~M2006Can()
{
	ScheduleClear();
	sendZeroBestEffort();
}

bool M2006Can::init()
{
	if (!_param_enable.get()) {
		PX4_ERR("M2K_EN is disabled");
		return false;
	}

	const int32_t left_id = _param_left_id.get();
	const int32_t right_id = _param_right_id.get();

	if (!hybrid_control::validM2006MotorIds(left_id, right_id)) {
		PX4_ERR("invalid C610 IDs: left=%" PRId32 " right=%" PRId32, left_id, right_id);
		return false;
	}

	_left_id = static_cast<uint8_t>(left_id);
	_right_id = static_cast<uint8_t>(right_id);

	if ((board_get_can_interfaces() & uavcan_can::Can2Mask) == 0) {
		PX4_ERR("CAN2 is unavailable");
		events::send(events::ID("m2006_can2_unavailable"), events::Log::Error,
			     "M2006 CAN2 is unavailable");
		return false;
	}

	if (!uavcan_can::claim(uavcan_can::Owner::M2006, uavcan_can::Can2Mask)) {
		PX4_ERR("CAN2 is already initialized; reboot required");
		events::send(events::ID("m2006_can_runtime_ownership_conflict"), events::Log::Error,
			     "M2006 CAN2 is already owned; reboot required");
		return false;
	}

	(void)UAVCAN_DRIVER::SystemClock::instance();
	const int can_result = _can.init(1000000);

	if (can_result < 0) {
		PX4_ERR("CAN2 initialization failed: %d", can_result);
		return false;
	}

	_iface = _can.driver.getIface(0);

	if (_iface == nullptr) {
		PX4_ERR("CAN2 interface unavailable");
		return false;
	}

	// CanIface::init() configures the FDCAN global filter to route unmatched
	// frames to FIFO0. Re-entering INIT after that initial setup is unsupported
	// on this board, so software C610-ID decoding performs the acceptance filter.

	updateControllerConfiguration();
	_last_can_error_count = _iface->getErrorCount();
	ScheduleOnInterval(RunIntervalUs, RunIntervalUs);
	return true;
}

void M2006Can::updateControllerConfiguration()
{
	const hybrid_control::SpeedControllerConfig config{
		_param_max_rpm.get(),
		_param_speed_p.get(),
		_param_speed_i.get(),
		_param_speed_d.get(),
		_param_speed_ff.get(),
		boundedCurrentLimit(_param_current_limit.get()),
		_param_rpm_slew.get()
	};

	_controller_config_valid = _speed[0].configure(config);
	_controller_config_valid = _speed[1].configure(config) && _controller_config_valid;
}

void M2006Can::receiveFeedback()
{
	_rx_error = false;

	for (;;) {
		uavcan::CanFrame frame{};
		uavcan::MonotonicTime timestamp{};
		uavcan::UtcTime utc_timestamp{};
		uavcan::CanIOFlags flags{};
		const int result = _iface->receive(frame, timestamp, utc_timestamp, flags);

		if (result == 0) {
			break;
		}

		if (result < 0) {
			_rx_error = true;
			break;
		}

		MotorIndex index{};
		hybrid_control::C610Feedback feedback{};

		if (hybrid_control::decodeC610Feedback(frame.id, frame.dlc, frame.data,
						       _left_id, _right_id,
						       index, feedback)) {
			const unsigned motor = index == MotorIndex::Left ? 0U : 1U;
			_feedback[motor] = feedback;
			_feedback_timestamp[motor] = hrt_absolute_time();
			_feedback_seen[motor] = true;
			++_rx_count[motor];
		}
	}
}

bool M2006Can::sendCommand(const int16_t left, const int16_t right)
{
	const hybrid_control::C610CommandFrame command = hybrid_control::makeC610Command(left, right);
	uavcan::CanFrame frame{};
	frame.id = command.id;
	frame.dlc = command.dlc;
	std::memcpy(frame.data, command.data, sizeof(command.data));
	const uavcan::MonotonicTime deadline = UAVCAN_DRIVER::SystemClock::instance().getMonotonic()
					       + uavcan::MonotonicDuration::fromUSec(RunIntervalUs);
	const int result = _iface->send(frame, deadline, uavcan::CanIOFlagAbortOnError);

	if (result > 0) {
		++_tx_count;
		_consecutive_tx_failures = 0;
		return true;
	}

	++_consecutive_tx_failures;

	if (result == 0) {
		++_tx_full_count;

	} else {
		++_tx_error_count;
	}

	return false;
}

void M2006Can::sendZeroBestEffort()
{
	if (_iface == nullptr) {
		return;
	}

	if (!hybrid_control::shouldTransmitM2006Command(_online_previous[0], _online_previous[1])) {
		return;
	}

	for (unsigned attempt = 0; attempt < 3; ++attempt) {
		(void)sendCommand(0, 0);
	}
}

void M2006Can::publishStatus(const hrt_abstime now, const bool online[2])
{
	m2006_motor_status_s status{};
	status.timestamp = now;

	for (unsigned motor = 0; motor < 2; ++motor) {
		status.feedback_timestamp[motor] = _feedback_timestamp[motor];
		status.target_rpm[motor] = _speed[motor].targetRpm();
		status.measured_rpm[motor] = _feedback[motor].rpm;
		status.speed_error[motor] = status.target_rpm[motor] - status.measured_rpm[motor];
		status.current_command[motor] = _current_command[motor];
		status.torque_current[motor] = _feedback[motor].torque_current;
		status.encoder[motor] = _feedback[motor].encoder;
		status.online[motor] = online[motor];
	}

	status.fault_flags = _gate.faultBits();
	status.rx_count = _rx_count[0] + _rx_count[1];
	status.tx_count = _tx_count;
	status.tx_full_count = _tx_full_count;
	status.tx_error_count = _tx_error_count;
	status.timeout_count = _timeout_count;
	status.can_error_count = _iface->getErrorCount();
	_status_pub.publish(status);
}

void M2006Can::Run()
{
	if (should_exit()) {
		sendZeroBestEffort();
		exit_and_cleanup();
		return;
	}

	uavcan::CanSelectMasks masks{};
	const uavcan::CanFrame *pending_tx[uavcan::MaxCanIfaces] {};
	const uavcan::MonotonicTime can_now = UAVCAN_DRIVER::SystemClock::instance().getMonotonic();
	(void)static_cast<uavcan::ICanDriver &>(_can.driver).select(masks, pending_tx, can_now);

	receiveFeedback();
	_motors_sub.update(&_motors);
	_armed_sub.update(&_armed);
	_hybrid_status_sub.update(&_hybrid_status);

	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		updateParams();
		updateControllerConfiguration();
	}

	const hrt_abstime now = hrt_absolute_time();

	const uint64_t feedback_timeout_us = static_cast<uint64_t>(_param_feedback_timeout.get() * 1e6f);
	bool online[2] {};

	for (unsigned motor = 0; motor < 2; ++motor) {
		online[motor] = _feedback_seen[motor] && now >= _feedback_timestamp[motor]
				&& now - _feedback_timestamp[motor] <= feedback_timeout_us;

		if (_online_previous[motor] && !online[motor]) {
			++_timeout_count;
		}

		_online_previous[motor] = online[motor];
	}

	const uint64_t current_can_error_count = _iface->getErrorCount();
	const bool can_error = _rx_error || current_can_error_count > _last_can_error_count
			       || _consecutive_tx_failures >= TxFailureLimit;
	_last_can_error_count = current_can_error_count;

	const hybrid_control::M2006NormalizedCommand command = hybrid_control::adaptM2006Command(
				_motors.control, _param_left_reverse.get(), _param_right_reverse.get());
	const uint64_t command_timeout_us = static_cast<uint64_t>(_param_command_timeout.get() * 1e6f);
	const bool command_fresh = hybrid_control::commandTimestampFresh(_motors.timestamp, now, command_timeout_us);
	const bool output_inhibited = _armed.lockdown || _armed.manual_lockdown || _armed.force_failsafe;
	const bool hybrid_driving = _hybrid_status.current_state == hybrid_vehicle_status_s::HYBRID_STATE_DRIVING;
	const hybrid_control::DriveGateInput gate_input{
		_armed.armed,
		hybrid_driving,
		output_inhibited,
		command_fresh,
		command.finite &&_controller_config_valid,
		{online[0], online[1]},
		can_error,
		now
	};
	const bool drive_enabled = _gate.update(gate_input);
	float dt = 0.002f;

	if (_last_run != 0 && now > _last_run) {
		const float elapsed = static_cast<float>(now - _last_run) * 1e-6f;
		dt = elapsed < 0.02f ? elapsed : 0.02f;
	}

	_last_run = now;

	if (drive_enabled) {
		_current_command[0] = _speed[0].update(command.left, _feedback[0].rpm, dt, true);
		_current_command[1] = _speed[1].update(command.right, _feedback[1].rpm, dt, true);

	} else {
		_speed[0].reset();
		_speed[1].reset();
		_current_command[0] = 0;
		_current_command[1] = 0;
	}

	if (hybrid_control::shouldTransmitM2006Command(online[0], online[1])) {
		(void)sendCommand(_current_command[0], _current_command[1]);
	}

	if (_last_status_publish == 0 || now - _last_status_publish >= StatusIntervalUs) {
		publishStatus(now, online);
		_last_status_publish = now;
	}
}

int M2006Can::print_status()
{
	PX4_INFO("CAN2 1Mbps: rx=%" PRIu32 "/%" PRIu32 " tx=%" PRIu32,
		 _rx_count[0], _rx_count[1], _tx_count);
	PX4_INFO("tx full=%" PRIu32 " error=%" PRIu32 " consecutive=%u hw errors=%" PRIu64,
		 _tx_full_count, _tx_error_count, _consecutive_tx_failures,
		 _iface != nullptr ? _iface->getErrorCount() : 0);
	PX4_INFO("fault=0x%08" PRIx32 " current=%d/%d", _gate.faultBits(),
		 static_cast<int>(_current_command[0]), static_cast<int>(_current_command[1]));
	return 0;
}

M2006Can *M2006Can::instantiate(int argc, char *argv[])
{
	return new M2006Can();
}

int M2006Can::task_spawn(int argc, char *argv[])
{
	M2006Can *instance = instantiate(argc, argv);

	if (instance != nullptr) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int M2006Can::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int M2006Can::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Standalone DJI M2006/C610 CAN2 wheel driver");
	PRINT_MODULE_USAGE_NAME("m2006_can", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int m2006_can_main(int argc, char *argv[])
{
	return M2006Can::main(argc, argv);
}

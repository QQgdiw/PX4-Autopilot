#include "M2006Can.hpp"
#include "CanOwnership.hpp"

#include <cstring>
#include <inttypes.h>

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
#if defined(UAVCAN_STM32H7_NUTTX)
	_h7_iface = _can.driver.getIface(0);
#endif

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
	_last_tx_id = frame.id;
	_last_tx_dlc = frame.dlc;
	std::memcpy(_last_tx_data, frame.data, sizeof(_last_tx_data));
	_last_tx_valid = true;
	const uavcan::MonotonicTime deadline = UAVCAN_DRIVER::SystemClock::instance().getMonotonic()
					       + uavcan::MonotonicDuration::fromUSec(RunIntervalUs);
	const int result = _iface->send(frame, deadline, uavcan::CanIOFlagAbortOnError);
	_last_tx_result = result;

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
	const uint64_t previous_can_error_count = _last_can_error_count;
	const bool can_error = _rx_error || current_can_error_count > _last_can_error_count
			       || _consecutive_tx_failures >= TxFailureLimit;
	const uint64_t can_error_delta = current_can_error_count > previous_can_error_count
					 ? current_can_error_count - previous_can_error_count : 0;
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
	const uint32_t fault_bits = _gate.faultBits();

	// Report each newly latched gate fault once. This preserves the existing
	// safety behavior while exposing the per-cycle inputs needed to diagnose
	// arm/drive startup races without flooding the console every 2 ms.
	if (fault_bits != 0) {
		const uint32_t new_fault_bits = fault_bits & ~_last_reported_fault_bits;

		if (new_fault_bits != 0) {
			const uint64_t command_age_us = now >= _motors.timestamp ? now - _motors.timestamp : 0;
			const bool command_future = _motors.timestamp > now;
			PX4_WARN("gate fault new=0x%08" PRIx32 " total=0x%08" PRIx32
				 " armed=%d driving=%d inhibit=%d online=%d/%d"
				 " cmd_fresh=%d cmd_finite=%d cfg=%d can=%d rxerr=%d"
				 " can_count=%" PRIu64 " delta=%" PRIu64
				 " tx_fail=%u cmd_age=%" PRIu64 " future=%d"
				 " ctrl=%.4f/%.4f",
				 new_fault_bits, fault_bits,
				 static_cast<int>(_armed.armed), static_cast<int>(hybrid_driving),
				 static_cast<int>(output_inhibited), static_cast<int>(online[0]),
				 static_cast<int>(online[1]), static_cast<int>(command_fresh),
				 static_cast<int>(command.finite), static_cast<int>(_controller_config_valid),
				 static_cast<int>(can_error), static_cast<int>(_rx_error),
				 current_can_error_count, can_error_delta, _consecutive_tx_failures,
				 command_age_us, static_cast<int>(command_future),
				 static_cast<double>(command.left), static_cast<double>(command.right));
		}

		_last_reported_fault_bits = fault_bits;

	} else {
		_last_reported_fault_bits = 0;
	}
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
	if (_last_tx_valid) {
		PX4_INFO("last TX id=0x%03" PRIx32 " dlc=%u result=%d data=%02x %02x %02x %02x %02x %02x %02x %02x",
			_last_tx_id, static_cast<unsigned>(_last_tx_dlc), _last_tx_result,
			_last_tx_data[0], _last_tx_data[1], _last_tx_data[2], _last_tx_data[3],
			_last_tx_data[4], _last_tx_data[5], _last_tx_data[6], _last_tx_data[7]);
	} else {
		PX4_INFO("last TX: none");
	}
#if defined(UAVCAN_STM32H7_NUTTX)
	uavcan_stm32h7::CanIface::ErrorSnapshot snapshot{};
	if (_h7_iface != nullptr) {
		PX4_INFO("H7 err internal=%" PRIu64 " rx_overflow=%" PRIu32
			 " cel=%" PRIu32 " busoff=%" PRIu32 " tx_timeout=%" PRIu32
			 " rx_fifo_lost=%" PRIu32 " abort=%" PRIu32 " rxq=%u",
			 _h7_iface->getInternalErrorCount(), _h7_iface->getRxQueueOverflowCount(),
			 _h7_iface->getCanErrorLogCount(), _h7_iface->getBusOffCount(),
			 _h7_iface->getTxTimeoutCount(), _h7_iface->getRxFifoLostCount(),
			 _h7_iface->getVoluntaryTxAbortCount(), _h7_iface->getRxQueueLength());
	}

	if (_h7_iface != nullptr && _h7_iface->getErrorSnapshot(snapshot)) {
		const uint32_t psr = snapshot.psr;
		const uint32_t ecr = snapshot.ecr;
		PX4_INFO("first snapshot kind=0x%02" PRIx32 " t=%" PRIu64 "us",
			 snapshot.kind, snapshot.monotonic_usec);
		PX4_INFO("PSR=0x%08" PRIx32 " LEC=%" PRIu32 " DLEC=%" PRIu32 " ACT=%" PRIu32 " EP=%" PRIu32 " EW=%" PRIu32 " BO=%" PRIu32,
			 psr,
			 static_cast<uavcan::uint32_t>((psr & FDCAN_PSR_LEC) >> FDCAN_PSR_LEC_Pos),
			 static_cast<uavcan::uint32_t>((psr & FDCAN_PSR_DLEC) >> FDCAN_PSR_DLEC_Pos),
			 static_cast<uavcan::uint32_t>((psr & FDCAN_PSR_ACT) >> FDCAN_PSR_ACT_Pos),
			 static_cast<uavcan::uint32_t>((psr & FDCAN_PSR_EP) ? 1U : 0U),
			 static_cast<uavcan::uint32_t>((psr & FDCAN_PSR_EW) ? 1U : 0U),
			 static_cast<uavcan::uint32_t>((psr & FDCAN_PSR_BO) ? 1U : 0U));
		if (snapshot.ecr_valid) {
			PX4_INFO("ECR=0x%08" PRIx32 " TEC=%" PRIu32 " REC=%" PRIu32 " CEL=%" PRIu32,
				 ecr,
				 static_cast<uavcan::uint32_t>((ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos),
				 static_cast<uavcan::uint32_t>((ecr & FDCAN_ECR_TREC) >> FDCAN_ECR_TREC_Pos),
				 static_cast<uavcan::uint32_t>((ecr & FDCAN_ECR_CEL) >> FDCAN_ECR_CEL_Pos));

		} else {
			PX4_INFO("ECR=not sampled for software-path snapshot");
		}
		PX4_INFO("IR=0x%08" PRIx32 " TXFQS=0x%08" PRIx32 " TXBRP=0x%08" PRIx32,
			 snapshot.ir, snapshot.txfqs, snapshot.txbrp);
		PX4_INFO("TXBTO=0x%08" PRIx32 " TXBCF=0x%08" PRIx32 " CCCR=0x%08" PRIx32,
			 snapshot.txbto, snapshot.txbcf, snapshot.cccr);

	} else {
		PX4_INFO("first snapshot=none");
	}
#endif
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

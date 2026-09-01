#include "M2006DriveGate.hpp"

namespace hybrid_control
{

namespace
{
constexpr uint64_t feedback_health_delay_us = 100000;
}

bool commandTimestampFresh(uint64_t timestamp, uint64_t now_us, uint64_t timeout_us)
{
	return timestamp != 0 && now_us >= timestamp && now_us - timestamp <= timeout_us;
}

bool M2006DriveGate::update(const DriveGateInput &input)
{
	const bool feedback_healthy = input.feedback_healthy[0] && input.feedback_healthy[1];
	const bool command_healthy = input.command_fresh && input.command_finite;
	const bool all_healthy = feedback_healthy && command_healthy && !input.can_error;
	// A disarmed/non-driving vehicle may legitimately publish NaN wheel outputs.
	// Recovery must therefore depend on feedback and CAN health, while command
	// freshness/finiteness remains mandatory whenever armed driving is enabled.
	const bool recovery_healthy = feedback_healthy && !input.can_error;

	if (!input.armed || !input.driving) {
		_command_qualified = false;

	} else if (command_healthy) {
		// RoverDifferential starts publishing only after arming. Treat the first
		// armed cycle without a sample as startup qualification, not a fault.
		_command_qualified = true;
	}

	if (!feedback_healthy) {
		_feedback_timer_active = false;

	} else if (!_feedback_timer_active) {
		_both_healthy_since = input.now_us;
		_feedback_timer_active = true;
	}

	const bool feedback_ready = _feedback_timer_active
				    && input.now_us >= _both_healthy_since
				    && input.now_us - _both_healthy_since >= feedback_health_delay_us;

	if (input.armed && input.driving) {
		if (!input.feedback_healthy[0]) {
			_fault_bits |= DriveFaultLeftFeedback;
		}

		if (!input.feedback_healthy[1]) {
			_fault_bits |= DriveFaultRightFeedback;
		}

		if (input.can_error) {
			_fault_bits |= DriveFaultCan;
		}

		if (_command_qualified && !command_healthy) {
			_fault_bits |= DriveFaultCommand;
		}
	}

	if (_fault_bits == DriveFaultNone) {
		_recovery_timer_active = false;

	} else if (input.armed || !recovery_healthy) {
		_recovery_timer_active = false;

	} else if (!_recovery_timer_active) {
		_recovery_healthy_since = input.now_us;
		_recovery_timer_active = true;
	}

	const bool recovery_ready = _recovery_timer_active
				    && input.now_us >= _recovery_healthy_since
				    && input.now_us - _recovery_healthy_since >= feedback_health_delay_us;

	if (!input.armed && _fault_bits != DriveFaultNone && recovery_ready) {
		_fault_bits = DriveFaultNone;
		_recovery_timer_active = false;
	}

	return input.armed && input.driving && !input.output_inhibited && all_healthy && feedback_ready
	       && _fault_bits == DriveFaultNone;
}

} // namespace hybrid_control

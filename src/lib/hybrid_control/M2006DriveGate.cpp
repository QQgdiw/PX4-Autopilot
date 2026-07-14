#include "M2006DriveGate.hpp"

namespace hybrid_control
{

namespace
{
constexpr uint64_t feedback_health_delay_us = 100000;
}

bool M2006DriveGate::update(const DriveGateInput &input)
{
	const bool feedback_healthy = input.feedback_healthy[0] && input.feedback_healthy[1];
	const bool command_healthy = input.command_fresh && input.command_finite;
	const bool all_healthy = feedback_healthy && command_healthy && !input.can_error;

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

		if (!command_healthy) {
			_fault_bits |= DriveFaultCommand;
		}
	}

	if (_fault_bits == DriveFaultNone) {
		_recovery_timer_active = false;

	} else if (input.armed || !all_healthy) {
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

	return input.armed && input.driving && all_healthy && feedback_ready
	       && _fault_bits == DriveFaultNone;
}

} // namespace hybrid_control

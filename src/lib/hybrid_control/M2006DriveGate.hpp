#pragma once

#include <cstdint>

namespace hybrid_control
{

enum DriveFault : uint32_t {
	DriveFaultNone = 0,
	DriveFaultLeftFeedback = 1u << 0,
	DriveFaultRightFeedback = 1u << 1,
	DriveFaultCan = 1u << 2,
	DriveFaultCommand = 1u << 3,
};

struct DriveGateInput {
	bool armed;
	bool driving;
	bool output_inhibited;
	bool command_fresh;
	bool command_finite;
	bool feedback_healthy[2];
	bool can_error;
	uint64_t now_us;
};

bool commandTimestampFresh(uint64_t timestamp, uint64_t now_us, uint64_t timeout_us);

class M2006DriveGate
{
public:
	bool update(const DriveGateInput &input);
	uint32_t faultBits() const { return _fault_bits; }

private:
	uint32_t _fault_bits{DriveFaultNone};
	uint64_t _both_healthy_since{0};
	uint64_t _recovery_healthy_since{0};
	bool _feedback_timer_active{false};
	bool _recovery_timer_active{false};
};

} // namespace hybrid_control

#pragma once

#include <cstdint>

namespace hybrid_control
{

enum class MotorIndex : uint8_t { Left = 0, Right = 1 };

struct C610Feedback {
	uint16_t encoder;
	int16_t rpm;
	int16_t torque_current;
};

struct C610CommandFrame {
	uint32_t id{0x200};
	uint8_t data[8] {};
	uint8_t dlc{8};
};

C610CommandFrame makeC610Command(int16_t left, int16_t right);

bool decodeC610Feedback(uint32_t id, uint8_t dlc, const uint8_t data[8],
			uint8_t left_id, uint8_t right_id,
			MotorIndex &index, C610Feedback &feedback);

} // namespace hybrid_control

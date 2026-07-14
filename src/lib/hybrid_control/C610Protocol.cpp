#include "C610Protocol.hpp"

namespace hybrid_control
{

namespace
{

constexpr uint32_t CanFrameFlagMask = 0xE0000000U;

uint16_t decodeUnsigned(const uint8_t high, const uint8_t low)
{
	return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) | low);
}

int16_t decodeSigned(const uint8_t high, const uint8_t low)
{
	const uint16_t value = decodeUnsigned(high, low);
	return value <= 0x7FFFU ? static_cast<int16_t>(value) : static_cast<int16_t>(static_cast<int32_t>(value) - 0x10000);
}

void encodeSigned(const int16_t value, uint8_t *destination)
{
	const uint16_t raw = static_cast<uint16_t>(value);
	destination[0] = static_cast<uint8_t>(raw >> 8);
	destination[1] = static_cast<uint8_t>(raw);
}

} // namespace

C610CommandFrame makeC610Command(const int16_t left, const int16_t right)
{
	C610CommandFrame frame{};
	encodeSigned(left, &frame.data[0]);
	encodeSigned(right, &frame.data[2]);
	return frame;
}

bool decodeC610Feedback(const uint32_t id, const uint8_t dlc, const uint8_t data[8],
			const uint8_t left_id, const uint8_t right_id,
			MotorIndex &index, C610Feedback &feedback)
{
	if (dlc != 8 || (id & CanFrameFlagMask) != 0 || left_id < 1 || left_id > 4
	    || right_id < 1 || right_id > 4 || left_id == right_id) {
		return false;
	}

	if (id == 0x200U + left_id) {
		index = MotorIndex::Left;

	} else if (id == 0x200U + right_id) {
		index = MotorIndex::Right;

	} else {
		return false;
	}

	feedback.encoder = decodeUnsigned(data[0], data[1]);
	feedback.rpm = decodeSigned(data[2], data[3]);
	feedback.torque_current = decodeSigned(data[4], data[5]);
	return true;
}

} // namespace hybrid_control

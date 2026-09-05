#pragma once

#include <cstddef>
#include <cstdint>

namespace hx65
{

constexpr size_t MaxFrameSize = 96;
constexpr size_t MaxParameterSize = MaxFrameSize - 6;
constexpr uint8_t BroadcastId = 0xfe;

enum class Instruction : uint8_t {
	Ping = 0x01,
	Read = 0x02,
	Write = 0x03,
	RegWrite = 0x04,
	Action = 0x05,
	Reset = 0x06,
	SyncRead = 0x82,
	SyncWrite = 0x83
};

enum class ParseResult : uint8_t {
	NeedMore,
	FrameReady,
	BadLength,
	BadChecksum,
	WrongId
};

struct StatusFrame {
	uint8_t servo_id{0};
	uint8_t error{0};
	uint8_t parameters[MaxParameterSize] {};
	uint8_t parameter_length{0};
};

size_t encodeInstruction(Instruction instruction, uint8_t servo_id, const uint8_t *parameters,
			 size_t parameter_length, uint8_t *out, size_t capacity);

uint16_t encodeSignedMagnitude(int32_t value, uint8_t sign_bit);
int32_t decodeSignedMagnitude(uint16_t value, uint8_t sign_bit);

class StreamParser
{
public:
	ParseResult push(uint8_t byte, uint8_t expected_servo_id, StatusFrame &frame);
	void reset();

private:
	uint8_t _buffer[MaxFrameSize] {};
	uint8_t _size{0};
	uint8_t _expected_size{0};
};

} // namespace hx65

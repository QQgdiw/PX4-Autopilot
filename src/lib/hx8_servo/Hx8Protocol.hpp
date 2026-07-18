#pragma once

#include <cstddef>
#include <cstdint>

namespace hx8
{

constexpr size_t MaxFrameSize = 64;

enum class CommandId : uint8_t {
	Ping = 0x01,
	ParamRead = 0x03,
	ParamWrite = 0x04,
	AngleRead = 0x0A,
	TimedMove = 0x0B,
	Status = 0x16,
	Stop = 0x18
};

enum class ParseResult : uint8_t {
	NeedMore,
	FrameReady,
	BadLength,
	BadChecksum,
	WrongId,
	UnknownCommand
};

struct Frame {
	CommandId command;
	uint8_t servo_id;
	uint8_t payload[56];
	uint8_t payload_length;
};

size_t encodeRequest(CommandId command, uint8_t servo_id, const uint8_t *payload, size_t payload_length,
		     uint8_t *out, size_t capacity);

class StreamParser
{
public:
	ParseResult push(uint8_t byte, uint8_t expected_servo_id, Frame &frame);
	void reset();

private:
	uint8_t _buffer[MaxFrameSize] {};
	uint8_t _size{0};
	uint8_t _frame_size{0};
};

} // namespace hx8

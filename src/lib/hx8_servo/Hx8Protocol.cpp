#include "Hx8Protocol.hpp"

#include <cstring>

namespace hx8
{

namespace
{

constexpr uint8_t RequestHeaderFirst = 0x12;
constexpr uint8_t RequestHeaderSecond = 0x4c;
constexpr uint8_t ResponseHeaderFirst = 0x05;
constexpr uint8_t ResponseHeaderSecond = 0x1c;
constexpr size_t FrameOverhead = 6;
constexpr size_t MaxPayloadSize = sizeof(Frame::payload);

bool isSupportedCommand(uint8_t command)
{
	switch (static_cast<CommandId>(command)) {
	case CommandId::Ping:
	case CommandId::ParamRead:
	case CommandId::ParamWrite:
	case CommandId::AngleRead:
	case CommandId::TimedMove:
	case CommandId::Status:
	case CommandId::Stop:
		return true;
	}

	return false;
}

uint8_t checksum(const uint8_t *data, size_t length)
{
	uint8_t sum = 0;

	for (size_t i = 0; i < length; ++i) {
		sum = static_cast<uint8_t>(sum + data[i]);
	}

	return sum;
}

bool isHeaderFirst(uint8_t byte)
{
	return byte == ResponseHeaderFirst || byte == RequestHeaderFirst;
}

} // namespace

size_t encodeRequest(CommandId command, uint8_t servo_id, const uint8_t *payload, size_t payload_length,
		     uint8_t *out, size_t capacity)
{
	if (!isSupportedCommand(static_cast<uint8_t>(command)) || out == nullptr || payload_length > MaxPayloadSize
	    || (payload_length > 0 && payload == nullptr)) {
		return 0;
	}

	const size_t frame_size = payload_length + FrameOverhead;

	if (frame_size > MaxFrameSize || capacity < frame_size) {
		return 0;
	}

	out[0] = RequestHeaderFirst;
	out[1] = RequestHeaderSecond;
	out[2] = static_cast<uint8_t>(command);
	out[3] = static_cast<uint8_t>(payload_length + 1);
	out[4] = servo_id;

	if (payload_length > 0) {
		memcpy(&out[5], payload, payload_length);
	}

	out[frame_size - 1] = checksum(out, frame_size - 1);
	return frame_size;
}

ParseResult StreamParser::push(uint8_t byte, uint8_t expected_servo_id, Frame &frame)
{
	if (_size == 0) {
		if (isHeaderFirst(byte)) {
			_buffer[_size++] = byte;
		}

		return ParseResult::NeedMore;
	}

	if (_size == 1) {
		const uint8_t expected_second = _buffer[0] == ResponseHeaderFirst ? ResponseHeaderSecond : RequestHeaderSecond;

		if (byte == expected_second) {
			_buffer[_size++] = byte;

		} else if (isHeaderFirst(byte)) {
			_buffer[0] = byte;

		} else {
			reset();
		}

		return ParseResult::NeedMore;
	}

	_buffer[_size++] = byte;

	if (_size == 4) {
		const uint8_t data_length = _buffer[3];

		if (data_length == 0 || data_length > MaxPayloadSize + 1) {
			reset();
			return ParseResult::BadLength;
		}

		_frame_size = static_cast<uint8_t>(data_length + 5);
	}

	if (_frame_size == 0 || _size < _frame_size) {
		return ParseResult::NeedMore;
	}

	const bool is_response = _buffer[0] == ResponseHeaderFirst;
	const uint8_t command = _buffer[2];
	const uint8_t servo_id = _buffer[4];
	const uint8_t payload_length = static_cast<uint8_t>(_buffer[3] - 1);
	ParseResult result = ParseResult::NeedMore;

	if (checksum(_buffer, _frame_size - 1) != _buffer[_frame_size - 1]) {
		result = ParseResult::BadChecksum;

	} else if (is_response && !isSupportedCommand(command)) {
		result = ParseResult::UnknownCommand;

	} else if (is_response && servo_id != expected_servo_id) {
		result = ParseResult::WrongId;

	} else if (is_response) {
		frame.command = static_cast<CommandId>(command);
		frame.servo_id = servo_id;
		frame.payload_length = payload_length;

		if (payload_length > 0) {
			memcpy(frame.payload, &_buffer[5], payload_length);
		}

		result = ParseResult::FrameReady;
	}

	reset();
	return result;
}

void StreamParser::reset()
{
	_size = 0;
	_frame_size = 0;
}

} // namespace hx8

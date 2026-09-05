#include "Hx65Protocol.hpp"

#include <cstring>

namespace hx65
{

namespace
{

constexpr uint8_t Header = 0xff;
constexpr size_t MinimumFrameSize = 6;

uint8_t checksum(const uint8_t *bytes, size_t length)
{
	uint8_t sum = 0;

	for (size_t i = 0; i < length; ++i) {
		sum = static_cast<uint8_t>(sum + bytes[i]);
	}

	return static_cast<uint8_t>(~sum);
}

} // namespace

size_t encodeInstruction(Instruction instruction, uint8_t servo_id, const uint8_t *parameters,
			 size_t parameter_length, uint8_t *out, size_t capacity)
{
	const size_t total_length = parameter_length + MinimumFrameSize;

	if (out == nullptr || capacity < total_length || total_length > MaxFrameSize
	    || (parameter_length > 0 && parameters == nullptr)) {
		return 0;
	}

	out[0] = Header;
	out[1] = Header;
	out[2] = servo_id;
	out[3] = static_cast<uint8_t>(parameter_length + 2);
	out[4] = static_cast<uint8_t>(instruction);

	if (parameter_length > 0) {
		memcpy(&out[5], parameters, parameter_length);
	}

	out[total_length - 1] = checksum(&out[2], parameter_length + 3);
	return total_length;
}

uint16_t encodeSignedMagnitude(int32_t value, uint8_t sign_bit)
{
	const uint16_t sign_mask = static_cast<uint16_t>(1u << sign_bit);
	const uint16_t magnitude_mask = static_cast<uint16_t>(sign_mask - 1u);
	const uint32_t magnitude = value < 0 ? static_cast<uint32_t>(-value) : static_cast<uint32_t>(value);
	const uint16_t limited = static_cast<uint16_t>(magnitude > magnitude_mask ? magnitude_mask : magnitude);
	return value < 0 ? static_cast<uint16_t>(limited | sign_mask) : limited;
}

int32_t decodeSignedMagnitude(uint16_t value, uint8_t sign_bit)
{
	const uint16_t sign_mask = static_cast<uint16_t>(1u << sign_bit);
	const int32_t magnitude = value & static_cast<uint16_t>(sign_mask - 1u);
	return value & sign_mask ? -magnitude : magnitude;
}

void StreamParser::reset()
{
	_size = 0;
	_expected_size = 0;
}

ParseResult StreamParser::push(uint8_t byte, uint8_t expected_servo_id, StatusFrame &frame)
{
	if (_size == 0) {
		if (byte == Header) {
			_buffer[_size++] = byte;
		}

		return ParseResult::NeedMore;
	}

	if (_size == 1) {
		if (byte != Header) {
			reset();
			return ParseResult::NeedMore;
		}

		_buffer[_size++] = byte;
		return ParseResult::NeedMore;
	}

	// Servo IDs are limited to 0..253 and 0xff can only be a header byte.
	// Keep the parser aligned to the last two bytes of an arbitrary-length
	// 0xff preamble instead of interpreting the third 0xff as a servo ID.
	if (_size == 2 && byte == Header) {
		return ParseResult::NeedMore;
	}

	if (_size >= MaxFrameSize) {
		reset();
		return ParseResult::BadLength;
	}

	_buffer[_size++] = byte;

	if (_size == 4) {
		_expected_size = static_cast<uint8_t>(_buffer[3] + 4u);

		if (_expected_size < MinimumFrameSize || _expected_size > MaxFrameSize) {
			reset();
			return ParseResult::BadLength;
		}
	}

	if (_expected_size == 0 || _size < _expected_size) {
		return ParseResult::NeedMore;
	}

	if (_buffer[2] != expected_servo_id) {
		reset();
		return ParseResult::WrongId;
	}

	if (checksum(&_buffer[2], _expected_size - 3) != _buffer[_expected_size - 1]) {
		reset();
		return ParseResult::BadChecksum;
	}

	frame.servo_id = _buffer[2];
	frame.error = _buffer[4];
	frame.parameter_length = static_cast<uint8_t>(_buffer[3] - 2u);

	if (frame.parameter_length > 0) {
		memcpy(frame.parameters, &_buffer[5], frame.parameter_length);
	}

	reset();
	return ParseResult::FrameReady;
}

} // namespace hx65

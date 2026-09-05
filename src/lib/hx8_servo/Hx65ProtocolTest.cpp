#include <gtest/gtest.h>

#include "Hx65Protocol.hpp"

#include <algorithm>
#include <array>

using namespace hx65;

TEST(Hx65Protocol, EncodesVendorPingVector)
{
	uint8_t encoded[MaxFrameSize] {};
	const std::array<uint8_t, 6> expected {0xff, 0xff, 0x01, 0x02, 0x01, 0xfb};
	ASSERT_EQ(encodeInstruction(Instruction::Ping, 1, nullptr, 0, encoded, sizeof(encoded)), expected.size());
	EXPECT_TRUE(std::equal(expected.begin(), expected.end(), encoded));
}

TEST(Hx65Protocol, EncodesVendorPositionWriteVector)
{
	const std::array<uint8_t, 7> parameters {0x2a, 0x00, 0x08, 0x00, 0x00, 0xe8, 0x03};
	const std::array<uint8_t, 13> expected {0xff, 0xff, 0x01, 0x09, 0x03, 0x2a, 0x00,
		0x08, 0x00, 0x00, 0xe8, 0x03, 0xd5};
	uint8_t encoded[MaxFrameSize] {};
	ASSERT_EQ(encodeInstruction(Instruction::Write, 1, parameters.data(), parameters.size(), encoded,
				    sizeof(encoded)), expected.size());
	EXPECT_TRUE(std::equal(expected.begin(), expected.end(), encoded));
}

TEST(Hx65Protocol, ParsesVendorReadResponse)
{
	const std::array<uint8_t, 8> response {0xff, 0xff, 0x01, 0x04, 0x00, 0x18, 0x05, 0xdd};
	StreamParser parser;
	StatusFrame frame {};
	ParseResult result = ParseResult::NeedMore;

	for (uint8_t byte : response) {
		result = parser.push(byte, 1, frame);
	}

	ASSERT_EQ(result, ParseResult::FrameReady);
	EXPECT_EQ(frame.servo_id, 1);
	EXPECT_EQ(frame.error, 0);
	ASSERT_EQ(frame.parameter_length, 2);
	EXPECT_EQ(frame.parameters[0], 0x18);
	EXPECT_EQ(frame.parameters[1], 0x05);
}

TEST(Hx65Protocol, ParsesPingResponseAfterRepeatedHeaderPreamble)
{
	const std::array<uint8_t, 8> response {0xff, 0xff, 0xff, 0xff, 0x01, 0x02, 0x00, 0xfc};
	StreamParser parser;
	StatusFrame frame {};
	ParseResult result = ParseResult::NeedMore;

	for (uint8_t byte : response) {
		result = parser.push(byte, 1, frame);
	}

	EXPECT_EQ(result, ParseResult::FrameReady);
	EXPECT_EQ(frame.servo_id, 1);
	EXPECT_EQ(frame.error, 0);
	EXPECT_EQ(frame.parameter_length, 0);
}

TEST(Hx65Protocol, RejectsWrongIdAndChecksum)
{
	std::array<uint8_t, 6> response {0xff, 0xff, 0x01, 0x02, 0x00, 0xfc};
	StreamParser parser;
	StatusFrame frame {};
	ParseResult result = ParseResult::NeedMore;

	for (uint8_t byte : response) {
		result = parser.push(byte, 2, frame);
	}

	EXPECT_EQ(result, ParseResult::WrongId);
	response.back() ^= 1u;

	for (uint8_t byte : response) {
		result = parser.push(byte, 1, frame);
	}

	EXPECT_EQ(result, ParseResult::BadChecksum);
}

TEST(Hx65Protocol, ConvertsVendorSignedMagnitude)
{
	EXPECT_EQ(encodeSignedMagnitude(2000, 15), 2000);
	EXPECT_EQ(encodeSignedMagnitude(-2000, 15), static_cast<uint16_t>(0x87d0));
	EXPECT_EQ(decodeSignedMagnitude(0x87d0, 15), -2000);
	EXPECT_EQ(decodeSignedMagnitude(0x07d0, 15), 2000);
}

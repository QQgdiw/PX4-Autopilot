#include <gtest/gtest.h>

#include <cstring>

#include "Hx8Protocol.hpp"

using namespace hx8;

namespace
{

ParseResult feed(StreamParser &parser, const uint8_t *data, size_t length, uint8_t expected_id, Frame &frame)
{
	ParseResult result = ParseResult::NeedMore;

	for (size_t i = 0; i < length; ++i) {
		result = parser.push(data[i], expected_id, frame);
	}

	return result;
}

void expectEncoded(CommandId command, uint8_t servo_id, const uint8_t *payload, size_t payload_length,
		   const uint8_t *expected, size_t expected_length)
{
	uint8_t encoded[MaxFrameSize] {};
	ASSERT_EQ(encodeRequest(command, servo_id, payload, payload_length, encoded, sizeof(encoded)), expected_length);
	EXPECT_EQ(memcmp(encoded, expected, expected_length), 0);
}

} // namespace

TEST(Hx8Protocol, EncodesOfficialAndVendorGoldenRequests)
{
	const uint8_t ping[] {0x12, 0x4c, 0x01, 0x01, 0x00, 0x60};
	expectEncoded(CommandId::Ping, 0, nullptr, 0, ping, sizeof(ping));

	const uint8_t timed_payload[] {0x84, 0x03, 0x58, 0x02, 0x64, 0x00, 0xc8, 0x00, 0x00, 0x00};
	const uint8_t timed_move[] {0x12, 0x4c, 0x0b, 0x0b, 0x00, 0x84, 0x03, 0x58, 0x02, 0x64, 0x00, 0xc8,
				    0x00, 0x00, 0x00, 0x81};
	expectEncoded(CommandId::TimedMove, 0, timed_payload, sizeof(timed_payload), timed_move, sizeof(timed_move));

	const uint8_t angle_request[] {0x12, 0x4c, 0x0a, 0x01, 0x00, 0x69};
	expectEncoded(CommandId::AngleRead, 0, nullptr, 0, angle_request, sizeof(angle_request));

	const uint8_t status_request[] {0x12, 0x4c, 0x16, 0x01, 0x00, 0x75};
	expectEncoded(CommandId::Status, 0, nullptr, 0, status_request, sizeof(status_request));

	const uint8_t hold_payload[] {0x11, 0x70, 0x17};
	const uint8_t hold_example[] {0x12, 0x4c, 0x18, 0x04, 0x00, 0x11, 0x70, 0x17, 0x12};
	expectEncoded(CommandId::Stop, 0, hold_payload, sizeof(hold_payload), hold_example, sizeof(hold_example));

	const uint8_t release_payload[] {0x10, 0x00, 0x00};
	const uint8_t release_zero_power[] {0x12, 0x4c, 0x18, 0x04, 0x00, 0x10, 0x00, 0x00, 0x8a};
	expectEncoded(CommandId::Stop, 0, release_payload, sizeof(release_payload), release_zero_power,
		      sizeof(release_zero_power));

	const uint8_t read_payload[] {0x16};
	const uint8_t read_request[] {0x12, 0x4c, 0x03, 0x02, 0x00, 0x16, 0x79};
	expectEncoded(CommandId::ParamRead, 0, read_payload, sizeof(read_payload), read_request, sizeof(read_request));

	const uint8_t write_payload[] {0x16, 0x34, 0x12};
	const uint8_t write_request[] {0x12, 0x4c, 0x04, 0x04, 0x00, 0x16, 0x34, 0x12, 0xc2};
	expectEncoded(CommandId::ParamWrite, 0, write_payload, sizeof(write_payload), write_request,
		      sizeof(write_request));
}

TEST(Hx8Protocol, RejectsInvalidEncodeArgumentsAndOversizedPayload)
{
	uint8_t encoded[MaxFrameSize] {};
	uint8_t payload[57] {};
	EXPECT_EQ(encodeRequest(CommandId::Ping, 0, nullptr, 1, encoded, sizeof(encoded)), 0u);
	EXPECT_EQ(encodeRequest(CommandId::Ping, 0, payload, sizeof(payload), encoded, sizeof(encoded)), 0u);
	EXPECT_EQ(encodeRequest(CommandId::Ping, 0, nullptr, 0, nullptr, sizeof(encoded)), 0u);
	EXPECT_EQ(encodeRequest(CommandId::Ping, 0, nullptr, 0, encoded, 5), 0u);
	EXPECT_EQ(encodeRequest(static_cast<CommandId>(0x99), 0, nullptr, 0, encoded, sizeof(encoded)), 0u);
}

TEST(Hx8Protocol, ParsesGoldenResponsesLittleEndian)
{
	StreamParser parser;
	Frame frame {};
	const uint8_t angle_response[] {0x05, 0x1c, 0x0a, 0x03, 0x00, 0x86, 0x03, 0xb7};
	EXPECT_EQ(feed(parser, angle_response, sizeof(angle_response), 0, frame), ParseResult::FrameReady);
	EXPECT_EQ(frame.command, CommandId::AngleRead);
	EXPECT_EQ(frame.servo_id, 0);
	ASSERT_EQ(frame.payload_length, 2);
	EXPECT_EQ(frame.payload[0], 0x86);
	EXPECT_EQ(frame.payload[1], 0x03);

	const uint8_t status_response[] {0x05, 0x1c, 0x16, 0x10, 0x00, 0x83, 0x1e, 0x1e, 0x00, 0xea, 0x00,
				     0x2c, 0x07, 0x01, 0xaf, 0x0b, 0x00, 0x00, 0x00, 0x00, 0xde};
	EXPECT_EQ(feed(parser, status_response, sizeof(status_response), 0, frame), ParseResult::FrameReady);
	EXPECT_EQ(frame.command, CommandId::Status);
	ASSERT_EQ(frame.payload_length, 15);
	EXPECT_EQ(frame.payload[0], 0x83);
	EXPECT_EQ(frame.payload[14], 0x00);
}

TEST(Hx8Protocol, HandlesGarbagePartialAndRepeatedHeaderPrefix)
{
	StreamParser parser;
	Frame frame {};
	const uint8_t prefix[] {0xff, 0x05, 0x05, 0x1c, 0x0a, 0x03};
	EXPECT_EQ(feed(parser, prefix, sizeof(prefix), 0, frame), ParseResult::NeedMore);
	const uint8_t suffix[] {0x00, 0x86, 0x03, 0xb7};
	EXPECT_EQ(feed(parser, suffix, sizeof(suffix), 0, frame), ParseResult::FrameReady);
	EXPECT_EQ(frame.command, CommandId::AngleRead);
}

TEST(Hx8Protocol, ParsesConcatenatedFrames)
{
	StreamParser parser;
	Frame frame {};
	const uint8_t responses[] {
		0x05, 0x1c, 0x0a, 0x03, 0x00, 0x86, 0x03, 0xb7,
		0x05, 0x1c, 0x01, 0x01, 0x00, 0x23
	};
	unsigned ready_count = 0;

	for (uint8_t byte : responses) {
		if (parser.push(byte, 0, frame) == ParseResult::FrameReady) {
			++ready_count;
		}
	}

	EXPECT_EQ(ready_count, 2u);
	EXPECT_EQ(frame.command, CommandId::Ping);
}

TEST(Hx8Protocol, ReportsMalformedFramesAndRecovers)
{
	StreamParser parser;
	Frame frame {};
	const uint8_t bad_length[] {0x05, 0x1c, 0x0a, 0x00};
	EXPECT_EQ(feed(parser, bad_length, sizeof(bad_length), 0, frame), ParseResult::BadLength);

	const uint8_t too_long[] {0x05, 0x1c, 0x0a, 0x3a};
	EXPECT_EQ(feed(parser, too_long, sizeof(too_long), 0, frame), ParseResult::BadLength);

	const uint8_t bad_checksum[] {0x05, 0x1c, 0x0a, 0x03, 0x00, 0x86, 0x03, 0x00};
	EXPECT_EQ(feed(parser, bad_checksum, sizeof(bad_checksum), 0, frame), ParseResult::BadChecksum);

	const uint8_t wrong_id[] {0x05, 0x1c, 0x01, 0x01, 0x02, 0x25};
	EXPECT_EQ(feed(parser, wrong_id, sizeof(wrong_id), 0, frame), ParseResult::WrongId);

	const uint8_t unknown[] {0x05, 0x1c, 0x99, 0x01, 0x00, 0xbb};
	EXPECT_EQ(feed(parser, unknown, sizeof(unknown), 0, frame), ParseResult::UnknownCommand);

	const uint8_t valid[] {0x05, 0x1c, 0x01, 0x01, 0x00, 0x23};
	EXPECT_EQ(feed(parser, valid, sizeof(valid), 0, frame), ParseResult::FrameReady);
}

TEST(Hx8Protocol, PreservesNextFramePrefixConsumedAsBadChecksum)
{
	StreamParser parser;
	Frame frame {};
	const uint8_t traffic[] {
		0x05, 0x1c, 0x0a, 0x01, 0x00,
		0x05, 0x1c, 0x01, 0x01, 0x00, 0x23
	};
	unsigned checksum_errors = 0;
	unsigned ready_count = 0;

	for (uint8_t byte : traffic) {
		const ParseResult result = parser.push(byte, 0, frame);
		checksum_errors += result == ParseResult::BadChecksum;
		ready_count += result == ParseResult::FrameReady;
	}

	EXPECT_EQ(checksum_errors, 1u);
	EXPECT_EQ(ready_count, 1u);
	EXPECT_EQ(frame.command, CommandId::Ping);
}

TEST(Hx8Protocol, IgnoresRequestEchoBeforeResponse)
{
	StreamParser parser;
	Frame frame {};
	const uint8_t traffic[] {
		0x12, 0x4c, 0x0a, 0x01, 0x00, 0x69,
		0x05, 0x1c, 0x0a, 0x03, 0x00, 0x86, 0x03, 0xb7
	};
	unsigned ready_count = 0;

	for (uint8_t byte : traffic) {
		if (parser.push(byte, 0, frame) == ParseResult::FrameReady) {
			++ready_count;
		}
	}

	EXPECT_EQ(ready_count, 1u);
	EXPECT_EQ(frame.command, CommandId::AngleRead);
}

TEST(Hx8Protocol, ResetDiscardsPartialFrame)
{
	StreamParser parser;
	Frame frame {};
	const uint8_t partial[] {0x05, 0x1c, 0x0a, 0x03, 0x00};
	EXPECT_EQ(feed(parser, partial, sizeof(partial), 0, frame), ParseResult::NeedMore);
	parser.reset();
	const uint8_t valid[] {0x05, 0x1c, 0x01, 0x01, 0x00, 0x23};
	EXPECT_EQ(feed(parser, valid, sizeof(valid), 0, frame), ParseResult::FrameReady);
}

#include <gtest/gtest.h>

#include <cstring>

#include "C610Protocol.hpp"

using namespace hybrid_control;

TEST(C610Protocol, EncodesGroupedCurrentBigEndian)
{
	const C610CommandFrame frame = makeC610Command(-10000, 10000, 1, 2);
	EXPECT_EQ(frame.id, 0x200u);
	EXPECT_EQ(frame.dlc, 8);
	const uint8_t expected[8] {0xD8, 0xF0, 0x27, 0x10, 0, 0, 0, 0};
	EXPECT_EQ(0, memcmp(frame.data, expected, sizeof(expected)));
}

TEST(C610Protocol, ConfiguredSlotsMatchFeedbackIdentity)
{
	const uint8_t raw[8] {};

	for (uint8_t left = 1; left <= 4; ++left) {
		for (uint8_t right = 1; right <= 4; ++right) {
			if (left == right) { continue; }

			const auto frame = makeC610Command(-10000, 10000, left, right);

			for (uint8_t id = 1; id <= 4; ++id) {
				const uint16_t value = (uint16_t(frame.data[2 * (id - 1)]) << 8) | frame.data[2 * (id - 1) + 1];
				EXPECT_EQ(value, id == left ? 0xD8F0 : (id == right ? 0x2710 : 0));
			}

			MotorIndex index{};
			C610Feedback feedback{};
			ASSERT_TRUE(decodeC610Feedback(0x200 + left, 8, raw, left, right, index, feedback));
			EXPECT_EQ(index, MotorIndex::Left);
			ASSERT_TRUE(decodeC610Feedback(0x200 + right, 8, raw, left, right, index, feedback));
			EXPECT_EQ(index, MotorIndex::Right);
			const auto stopped = makeC610Command(0, 0, left, right);
			EXPECT_EQ(0, memcmp(stopped.data, raw, sizeof(raw)));
		}
	}
}

TEST(C610Protocol, InvalidIdsCannotEncodeNonzeroCurrent)
{
	const uint8_t zero[8] {};
	const uint8_t ids[][2] {{0, 2}, {1, 0}, {5, 2}, {1, 5}, {255, 1}, {2, 2}};

	for (const auto &pair : ids) {
		const auto frame = makeC610Command(-10000, 10000, pair[0], pair[1]);
		EXPECT_EQ(0, memcmp(frame.data, zero, sizeof(zero)));
	}
}

TEST(C610Protocol, DecodesConfiguredFeedbackIds)
{
	const uint8_t raw[8] {0x12, 0x34, 0xFE, 0xD4, 0x03, 0xE8, 0, 0};
	MotorIndex index{};
	C610Feedback feedback{};
	ASSERT_TRUE(decodeC610Feedback(0x201, 8, raw, 1, 2, index, feedback));
	EXPECT_EQ(index, MotorIndex::Left);
	EXPECT_EQ(feedback.encoder, 0x1234);
	EXPECT_EQ(feedback.rpm, -300);
	EXPECT_EQ(feedback.torque_current, 1000);
	ASSERT_TRUE(decodeC610Feedback(0x202, 8, raw, 1, 2, index, feedback));
	EXPECT_EQ(index, MotorIndex::Right);
}

TEST(C610Protocol, RejectsMalformedOrUnconfiguredFrames)
{
	constexpr uint32_t can_eff_flag = 0x80000000U;
	constexpr uint32_t can_rtr_flag = 0x40000000U;
	constexpr uint32_t can_err_flag = 0x20000000U;
	const uint8_t raw[8] {};
	MotorIndex index{};
	C610Feedback feedback{};

	EXPECT_FALSE(decodeC610Feedback(0x201, 7, raw, 1, 2, index, feedback));
	EXPECT_FALSE(decodeC610Feedback(0x203, 8, raw, 1, 2, index, feedback));
	EXPECT_FALSE(decodeC610Feedback(0x201 | can_eff_flag, 8, raw, 1, 2, index, feedback));
	EXPECT_FALSE(decodeC610Feedback(0x201 | can_rtr_flag, 8, raw, 1, 2, index, feedback));
	EXPECT_FALSE(decodeC610Feedback(0x201 | can_err_flag, 8, raw, 1, 2, index, feedback));
}

TEST(C610Protocol, RejectsInvalidMotorConfiguration)
{
	const uint8_t raw[8] {};
	MotorIndex index{};
	C610Feedback feedback{};

	EXPECT_FALSE(decodeC610Feedback(0x201, 8, raw, 1, 1, index, feedback));
	EXPECT_FALSE(decodeC610Feedback(0x201, 8, raw, 0, 2, index, feedback));
	EXPECT_FALSE(decodeC610Feedback(0x201, 8, raw, 1, 5, index, feedback));
}

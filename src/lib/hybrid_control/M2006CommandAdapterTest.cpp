#include <gtest/gtest.h>

#include <limits>

#include "M2006CommandAdapter.hpp"

using namespace hybrid_control;

TEST(M2006CommandAdapter, MapsFinalActuatorMotorIndices)
{
	const float controls[6] {0.1f, 0.2f, 0.3f, 0.4f, -0.5f, 0.6f};
	const M2006NormalizedCommand command = adaptM2006Command(controls, false, false);

	EXPECT_FLOAT_EQ(command.left, 0.6f);
	EXPECT_FLOAT_EQ(command.right, -0.5f);
	EXPECT_TRUE(command.finite);
}

TEST(M2006CommandAdapter, AppliesIndependentDirectionReversal)
{
	const float controls[6] {0.f, 0.f, 0.f, 0.f, -0.5f, 0.6f};
	const M2006NormalizedCommand command = adaptM2006Command(controls, true, false);

	EXPECT_FLOAT_EQ(command.left, -0.6f);
	EXPECT_FLOAT_EQ(command.right, -0.5f);
	EXPECT_TRUE(command.finite);
}

TEST(M2006CommandAdapter, RejectsEitherNonFiniteWheelCommand)
{
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const float controls[6] {0.f, 0.f, 0.f, 0.f, -0.5f, nan};
	const M2006NormalizedCommand command = adaptM2006Command(controls, false, false);

	EXPECT_FALSE(command.finite);
	EXPECT_FLOAT_EQ(command.left, 0.f);
	EXPECT_FLOAT_EQ(command.right, 0.f);
}

TEST(M2006CommandAdapter, AcceptsOnlyConfiguredPhysicalMotorIds)
{
	EXPECT_TRUE(validM2006MotorIds(1, 2));
	EXPECT_FALSE(validM2006MotorIds(2, 1));
	EXPECT_FALSE(validM2006MotorIds(1, 3));
	EXPECT_FALSE(validM2006MotorIds(1, 1));
}

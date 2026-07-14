#include <gtest/gtest.h>

#include "M2006SpeedController.hpp"

using namespace hybrid_control;

TEST(M2006SpeedController, SaturatesAndResets)
{
	M2006SpeedController controller;
	controller.configure({500.f, 2.f, 4.f, 0.f, 0.f, 1000, 50000.f});
	EXPECT_EQ(controller.update(1.f, 0.f, 0.01f, true), 1000);
	EXPECT_FLOAT_EQ(controller.integral(), 0.f);
	EXPECT_EQ(controller.update(1.f, 0.f, 0.01f, false), 0);
	EXPECT_FLOAT_EQ(controller.targetRpm(), 0.f);
	EXPECT_FLOAT_EQ(controller.integral(), 0.f);
}

TEST(M2006SpeedController, IntegratesWhenUnsaturated)
{
	M2006SpeedController controller;
	controller.configure({100.f, 0.f, 1.f, 0.f, 0.f, 1000, 1000.f});
	EXPECT_EQ(controller.update(1.f, 0.f, 0.1f, true), 10);
	EXPECT_FLOAT_EQ(controller.integral(), 10.f);
}

TEST(M2006SpeedController, SlewsTarget)
{
	M2006SpeedController controller;
	controller.configure({500.f, 1.f, 0.f, 0.f, 0.f, 10000, 500.f});
	controller.update(2.f, 0.f, 0.1f, true);
	EXPECT_FLOAT_EQ(controller.targetRpm(), 50.f);
}

TEST(M2006SpeedController, UsesDerivativeOnMeasurement)
{
	M2006SpeedController controller;
	controller.configure({500.f, 0.f, 0.f, 1.f, 0.f, 10000, 50000.f});
	EXPECT_EQ(controller.update(0.f, 10.f, 0.1f, true), 0);
	EXPECT_EQ(controller.update(0.f, 20.f, 0.1f, true), -100);
	EXPECT_EQ(controller.update(0.f, 20.f, 0.1f, false), 0);
	EXPECT_EQ(controller.update(0.f, 20.f, 0.1f, true), 0);
}

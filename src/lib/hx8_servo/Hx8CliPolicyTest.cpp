#include <gtest/gtest.h>

#include "Hx8CliPolicy.hpp"

TEST(Hx8CliPolicy, MissingDriverReturnsFailure)
{
	EXPECT_NE(hx8_cli::driverInstanceStatus(false), 0);
	EXPECT_EQ(hx8_cli::driverInstanceStatus(true), 0);
}

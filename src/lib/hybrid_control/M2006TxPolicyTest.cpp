#include <gtest/gtest.h>

#include "M2006TxPolicy.hpp"

using hybrid_control::shouldTransmitM2006Command;

TEST(M2006TxPolicy, RequiresAtLeastOneOnlineMotor)
{
	EXPECT_FALSE(shouldTransmitM2006Command(false, false));
	EXPECT_TRUE(shouldTransmitM2006Command(true, false));
	EXPECT_TRUE(shouldTransmitM2006Command(false, true));
	EXPECT_TRUE(shouldTransmitM2006Command(true, true));
}

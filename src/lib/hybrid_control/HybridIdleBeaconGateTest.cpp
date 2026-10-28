#include <drivers/dshot/HybridIdleBeaconGate.hpp>
#include <gtest/gtest.h>

class HybridIdleBeaconGate : public ::testing::Test
{
protected:
	vehicle_status_s vehicle{};
	hybrid_vehicle_status_s hybrid{};
	actuator_armed_s armed{};
	void SetUp() override
	{
		vehicle = {};
		hybrid = {};
		armed = {};
		vehicle.timestamp = hybrid.timestamp = armed.timestamp = 1000000;
		vehicle.is_quad_rover = true;
		hybrid.current_state = hybrid_vehicle_status_s::HYBRID_STATE_FLYING;
		hybrid.actuator_backend = hybrid_vehicle_status_s::ACTUATOR_HX65;
		hybrid.propulsion_owner = hybrid_vehicle_status_s::PROPULSION_QUAD;
		hybrid.sequence_state = hybrid_vehicle_status_s::SEQUENCE_STABLE_QUAD;
		hybrid.propulsion_ready = hybrid.position_confirmed = true;
		hybrid.actuator_online = hybrid.actuator_healthy = hybrid.actuator_config_verified = true;
		hybrid.gear_online = hybrid.gear_healthy = true;
	}
	bool allowed() { return hybridIdleBeaconEligible(1000000, vehicle, hybrid, armed, true, true, 15, false); }
};

TEST_F(HybridIdleBeaconGate, HealthyQuadOnly)
{
	ASSERT_TRUE(allowed());

	for (uint8_t state = 1; state <= 4; ++state) {
		hybrid.current_state = state;
		EXPECT_FALSE(allowed());
	}

	SetUp();
	vehicle.is_quad_rover = false;
	EXPECT_FALSE(allowed());
}

TEST_F(HybridIdleBeaconGate, FreshStateAndArmingRequired)
{
	for (auto timestamp : {uint64_t(0), uint64_t(499999), uint64_t(1000001)}) {
		SetUp(); vehicle.timestamp = timestamp; EXPECT_FALSE(allowed());
		SetUp(); hybrid.timestamp = timestamp; EXPECT_FALSE(allowed());
		SetUp(); armed.timestamp = timestamp; EXPECT_FALSE(allowed());
	}
	SetUp(); armed.armed = true; EXPECT_FALSE(allowed());
	SetUp(); armed.prearmed = true; EXPECT_FALSE(allowed());
	SetUp(); armed.manual_lockdown = true; EXPECT_FALSE(allowed());
	SetUp(); armed.lockdown = true; EXPECT_FALSE(allowed());
	SetUp(); armed.force_failsafe = true; EXPECT_FALSE(allowed());
	SetUp(); armed.in_esc_calibration_mode = true; EXPECT_FALSE(allowed());
}

TEST_F(HybridIdleBeaconGate, FaultReadinessAndGearGates)
{
	SetUp(); hybrid.fault_reason = 1; EXPECT_FALSE(allowed());
	SetUp(); hybrid.sequence_fault = 1; EXPECT_FALSE(allowed());
	SetUp(); hybrid.propulsion_ready = false; EXPECT_FALSE(allowed());
	SetUp(); hybrid.position_confirmed = false; EXPECT_FALSE(allowed());
	SetUp(); hybrid.actuator_online = false; EXPECT_FALSE(allowed());
	SetUp(); hybrid.actuator_healthy = false; EXPECT_FALSE(allowed());
	SetUp(); hybrid.actuator_config_verified = false; EXPECT_FALSE(allowed());
	SetUp(); hybrid.actuator_protection_flags = 1; EXPECT_FALSE(allowed());
	SetUp(); hybrid.gear_online = false; EXPECT_FALSE(allowed());
	SetUp(); hybrid.gear_healthy = false; EXPECT_FALSE(allowed());
	SetUp(); hybrid.propulsion_owner = hybrid_vehicle_status_s::PROPULSION_ROVER; EXPECT_FALSE(allowed());

	for (uint8_t state = 1; state <= 9; ++state) {
		SetUp(); hybrid.sequence_state = state; EXPECT_FALSE(allowed());
	}
}

TEST_F(HybridIdleBeaconGate, TestActiveNonzeroOutputsMissingRotorAndOtherCommandsInhibit)
{
	EXPECT_FALSE(hybridIdleBeaconEligible(1000000, vehicle, hybrid, armed, false, true, 15, false));
	EXPECT_FALSE(hybridIdleBeaconEligible(1000000, vehicle, hybrid, armed, true, false, 15, false));
	EXPECT_FALSE(hybridIdleBeaconEligible(1000000, vehicle, hybrid, armed, true, true, 7, false));
	EXPECT_FALSE(hybridIdleBeaconEligible(1000000, vehicle, hybrid, armed, true, true, 15, true));
}

TEST_F(HybridIdleBeaconGate, AutoQuadWaitingForTakeoffMayBeepOnlyWithGearDown)
{
	hybrid.sequence_state = hybrid_vehicle_status_s::SEQUENCE_QUAD_WAIT_AIRBORNE;
	hybrid.gear_down = true;
	EXPECT_TRUE(allowed());
	hybrid.gear_down = false;
	EXPECT_FALSE(allowed());
}

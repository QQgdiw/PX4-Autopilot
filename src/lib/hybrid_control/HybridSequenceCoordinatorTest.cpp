#include "HybridSequenceCoordinator.hpp"

#include <gtest/gtest.h>

using namespace hybrid_control;

namespace
{

SequenceInput quadInput(uint64_t now = 1)
{
	SequenceInput input{};
	input.now_us = now;
	input.shape_state = HybridState::Flying;
	input.gear_online = true;
	input.gear_healthy = true;
	input.gear_stowed = true;
	return input;
}

} // namespace

TEST(HybridSequenceCoordinator, QuadToRoverPreservesQuadUntilLandedAndGearDown)
{
	HybridSequenceCoordinator coordinator;
	SequenceConfig config{};
	config.landed_debounce_us = 100;
	SequenceInput input = quadInput();
	coordinator.initialize(config, input);

	input.requested_target = HybridTarget::Driving;
	input.armed = true;
	SequenceOutput output = coordinator.update(input);
	EXPECT_EQ(output.state, SequenceState::QuadToRoverPrepare);
	EXPECT_EQ(output.propulsion_owner, PropulsionOwner::Quad);
	EXPECT_EQ(output.gear_target, GearTarget::Down);

	input.requested_target = HybridTarget::None;
	input.gear_stowed = false;
	input.gear_down = true;
	input.landed = true;
	input.now_us = 50;
	coordinator.update(input);
	input.now_us = 150;
	output = coordinator.update(input);
	EXPECT_TRUE(output.request_disarm);
	EXPECT_EQ(output.propulsion_owner, PropulsionOwner::Quad);

	input.armed = false;
	output = coordinator.update(input);
	EXPECT_EQ(output.state, SequenceState::QuadToRoverTransform);
	EXPECT_EQ(output.shape_request, HybridTarget::Driving);
	EXPECT_EQ(output.propulsion_owner, PropulsionOwner::None);

	input.shape_state = HybridState::Driving;
	output = coordinator.update(input);
	EXPECT_EQ(output.state, SequenceState::RoverRetract);
	EXPECT_FALSE(output.propulsion_ready);

	input.gear_clear = true;
	output = coordinator.update(input);
	EXPECT_TRUE(output.propulsion_ready);
	EXPECT_EQ(output.propulsion_owner, PropulsionOwner::Rover);
	EXPECT_EQ(output.gear_target, GearTarget::Stowed);
}

TEST(HybridSequenceCoordinator, RoverToQuadIsNotReadyUntilShapeCompletes)
{
	HybridSequenceCoordinator coordinator;
	SequenceInput input = quadInput();
	input.shape_state = HybridState::Driving;
	input.gear_stowed = true;
	coordinator.initialize({}, input);

	input.requested_target = HybridTarget::Flying;
	input.armed = true;
	SequenceOutput output = coordinator.update(input);
	EXPECT_EQ(output.state, SequenceState::RoverToQuadPrepare);
	EXPECT_EQ(output.propulsion_owner, PropulsionOwner::None);
	EXPECT_TRUE(output.request_disarm);

	input.requested_target = HybridTarget::None;
	input.armed = false;
	input.gear_stowed = false;
	input.gear_down = true;
	output = coordinator.update(input);
	EXPECT_EQ(output.shape_request, HybridTarget::Flying);

	input.shape_state = HybridState::Flying;
	output = coordinator.update(input);
	EXPECT_EQ(output.state, SequenceState::QuadWaitAirborne);
	EXPECT_TRUE(output.propulsion_ready);
	EXPECT_EQ(output.propulsion_owner, PropulsionOwner::Quad);

	input.landed = false;
	input.now_us += 1000001;
	coordinator.update(input);
	input.now_us += 1000001;
	output = coordinator.update(input);
	EXPECT_EQ(output.state, SequenceState::QuadRetract);
	EXPECT_EQ(output.gear_target, GearTarget::Stowed);
}

TEST(HybridSequenceCoordinator, ManualGearSkipsPositionStagesButRetainsLandingAndDisarm)
{
	HybridSequenceCoordinator coordinator;
	SequenceConfig config{};
	config.automatic_gear = false;
	config.landed_debounce_us = 1;
	config.gear_motion_timeout_us = 10;
	SequenceInput input = quadInput();
	coordinator.initialize(config, input);

	input.requested_target = HybridTarget::Driving;
	input.armed = true;
	input.gear_stowed = false;
	SequenceOutput output = coordinator.update(input);
	EXPECT_EQ(output.gear_target, GearTarget::None);
	EXPECT_EQ(output.state, SequenceState::QuadToRoverPrepare);

	input.now_us += 20;
	output = coordinator.update(input);
	EXPECT_EQ(output.state, SequenceState::QuadToRoverPrepare);
	EXPECT_EQ(output.fault, SequenceFault::None);

	input.landed = true;
	input.now_us += 2;
	coordinator.update(input);
	input.now_us += 2;
	output = coordinator.update(input);
	EXPECT_TRUE(output.request_disarm);
	EXPECT_EQ(output.state, SequenceState::QuadToRoverPrepare);

	input.armed = false;
	output = coordinator.update(input);
	EXPECT_EQ(output.state, SequenceState::QuadToRoverTransform);

	input.shape_state = HybridState::Driving;
	output = coordinator.update(input);
	EXPECT_EQ(output.state, SequenceState::StableRover);
	EXPECT_TRUE(output.propulsion_ready);
	EXPECT_FALSE(input.gear_down);
	EXPECT_FALSE(input.gear_stowed);

	input.requested_target = HybridTarget::Flying;
	input.armed = true;
	output = coordinator.update(input);
	EXPECT_EQ(output.state, SequenceState::RoverToQuadPrepare);
	EXPECT_TRUE(output.request_disarm);

	input.requested_target = HybridTarget::None;
	input.armed = false;
	output = coordinator.update(input);
	EXPECT_EQ(output.state, SequenceState::RoverToQuadTransform);

	input.shape_state = HybridState::Flying;
	output = coordinator.update(input);
	EXPECT_EQ(output.state, SequenceState::StableQuad);
	EXPECT_TRUE(output.propulsion_ready);
}

TEST(HybridSequenceCoordinator, ManualGearStillRequiresHealthyGearAtTransitionEntry)
{
	HybridSequenceCoordinator coordinator;
	SequenceConfig config{};
	config.automatic_gear = false;
	SequenceInput input = quadInput();
	coordinator.initialize(config, input);

	input.requested_target = HybridTarget::Driving;
	input.gear_healthy = false;
	const SequenceOutput output = coordinator.update(input);
	EXPECT_EQ(output.state, SequenceState::Fault);
	EXPECT_EQ(output.fault, SequenceFault::GearCommunication);
	EXPECT_EQ(output.propulsion_owner, PropulsionOwner::Quad);
	EXPECT_TRUE(output.propulsion_ready);
}

TEST(HybridSequenceCoordinator, AirborneGearFailureDoesNotRemoveQuadControl)
{
	HybridSequenceCoordinator coordinator;
	SequenceInput input = quadInput();
	coordinator.initialize({}, input);
	input.requested_target = HybridTarget::Driving;
	input.gear_online = false;
	const SequenceOutput output = coordinator.update(input);
	EXPECT_EQ(output.state, SequenceState::Fault);
	EXPECT_EQ(output.fault, SequenceFault::GearCommunication);
	EXPECT_EQ(output.propulsion_owner, PropulsionOwner::Quad);
	EXPECT_TRUE(output.propulsion_ready);
}

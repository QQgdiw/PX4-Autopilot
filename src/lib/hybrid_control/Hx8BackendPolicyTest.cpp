#include <gtest/gtest.h>

#include "Hx8BackendPolicy.hpp"

using hybrid_control::Hx8BackendPolicy;

TEST(Hx8BackendPolicy, RejectsMismatchedOrUnsafeStatus)
{
	EXPECT_FALSE(Hx8BackendPolicy::statusUsable(2, 1, true, true, true, 0, true, 20.f));
	EXPECT_FALSE(Hx8BackendPolicy::statusUsable(1, 1, true, true, true, 0, false, 20.f));
	EXPECT_FALSE(Hx8BackendPolicy::statusUsable(1, 1, false, true, true, 0, true, 20.f));
	EXPECT_FALSE(Hx8BackendPolicy::statusUsable(1, 1, true, false, true, 0, true, 20.f));
	EXPECT_FALSE(Hx8BackendPolicy::statusUsable(1, 1, true, true, false, 0, true, 20.f));
	EXPECT_FALSE(Hx8BackendPolicy::statusUsable(1, 1, true, true, true, 1, true, 20.f));
	EXPECT_FALSE(Hx8BackendPolicy::statusUsable(1, 1, true, true, true, 0, true, NAN));
	EXPECT_TRUE(Hx8BackendPolicy::statusUsable(1, 1, true, true, true, 0, true, 20.f));
}

TEST(Hx8BackendPolicy, UsesConfiguredHx8EndpointsAndTarget)
{
	const float normalized = Hx8BackendPolicy::normalizeAngle(0.f, 0.f, 90.f);
	EXPECT_NEAR(normalized, 0.f, 1e-5f);
	EXPECT_TRUE(Hx8BackendPolicy::endpointMatches(normalized, false));
	EXPECT_FALSE(Hx8BackendPolicy::endpointMatches(normalized, true));
	EXPECT_FALSE(Hx8BackendPolicy::endpointMatches(0.f, true));
	EXPECT_FALSE(Hx8BackendPolicy::endpointMatchesAngleTolerance(0.f, true, 0.05f, 0.f, 90.f));
	EXPECT_TRUE(Hx8BackendPolicy::parametersValid(1, 0.f, 90.f, 1000, 100, 100, 500, 3.f));
	EXPECT_FALSE(Hx8BackendPolicy::parametersValid(255, 0.f, 90.f, 1000, 100, 100, 500, 3.f));
	EXPECT_FALSE(Hx8BackendPolicy::parametersValid(1, 0.f, 0.f, 1000, 100, 100, 500, 3.f));
	EXPECT_FALSE(Hx8BackendPolicy::parametersValid(1, 0.f, 90.f, INT32_MAX, 100, 100, 500, 3.f));
	EXPECT_FALSE(Hx8BackendPolicy::parametersValid(1, 0.f, 90.f, 1000, UINT16_MAX + 1, 0, 500, 3.f));
}

TEST(Hx8BackendPolicy, TimeoutAndProtocolAreNotAccepted)
{
	EXPECT_TRUE(Hx8BackendPolicy::commandAccepted(false, 1, 2, 3, 4));
	EXPECT_FALSE(Hx8BackendPolicy::commandAccepted(false, 3, 2, 3, 4));
	EXPECT_FALSE(Hx8BackendPolicy::commandAccepted(false, 4, 2, 3, 4));
}

TEST(Hx8CommandPolicy, PwmNeverCommandsAndHx8MovesOncePerNewTarget)
{
	hybrid_control::Hx8CommandPolicy policy;
	hybrid_control::TransformationOutput output{hybrid_control::HybridState::TransitionToRover,
		hybrid_control::HybridTarget::Driving, hybrid_control::SensorSource::None,
		hybrid_control::TransformFault::None, false, false, 0.f};
	EXPECT_EQ(policy.update(hybrid_control::ActuatorBackend::Pwm, output, 0).action,
		hybrid_control::Hx8CommandAction::None);
	auto first = policy.update(hybrid_control::ActuatorBackend::Hx8, output, 0);
	EXPECT_EQ(first.action, hybrid_control::Hx8CommandAction::Move);
	EXPECT_NE(first.sequence, 0u);
	EXPECT_EQ(policy.update(hybrid_control::ActuatorBackend::Hx8, output, 20'000).action,
		hybrid_control::Hx8CommandAction::None);
}

TEST(Hx8CommandPolicy, FaultOnlyReleasesBoundedlyAndClearAllowsSameTargetAgain)
{
	hybrid_control::Hx8CommandPolicy policy;
	hybrid_control::TransformationOutput output{hybrid_control::HybridState::TransitionToRover,
		hybrid_control::HybridTarget::Driving, hybrid_control::SensorSource::None,
		hybrid_control::TransformFault::None, false, false, 0.f};
	EXPECT_EQ(policy.update(hybrid_control::ActuatorBackend::Hx8, output, 0).action,
		hybrid_control::Hx8CommandAction::Move);
	output.state = hybrid_control::HybridState::Fault;
	output.fault = hybrid_control::TransformFault::Stall;
	for (int i = 0; i < 3; ++i) {
		auto release = policy.update(hybrid_control::ActuatorBackend::Hx8, output, i * 20'000 + 1);
		EXPECT_EQ(release.action, hybrid_control::Hx8CommandAction::Release);
		EXPECT_NE(release.sequence, 0u);
	}
	EXPECT_EQ(policy.update(hybrid_control::ActuatorBackend::Hx8, output, 3 * 20'000 + 2).action,
		hybrid_control::Hx8CommandAction::None);
	EXPECT_EQ(policy.update(hybrid_control::ActuatorBackend::Hx8, output, 4).action,
		hybrid_control::Hx8CommandAction::None);
	policy.resetAfterFaultClear();
	output.state = hybrid_control::HybridState::TransitionToRover;
	output.fault = hybrid_control::TransformFault::None;
	EXPECT_EQ(policy.update(hybrid_control::ActuatorBackend::Hx8, output, 5).action,
		hybrid_control::Hx8CommandAction::Move);
}

TEST(Hx8CommandPolicy, StableHoldCarriesCurrentEndpointAndAckMatchesSequence)
{
	hybrid_control::Hx8CommandPolicy policy;
	hybrid_control::TransformationOutput output{hybrid_control::HybridState::TransitionToQuad,
		hybrid_control::HybridTarget::Flying, hybrid_control::SensorSource::None,
		hybrid_control::TransformFault::None, false, false, 0.f};
	auto move = policy.update(hybrid_control::ActuatorBackend::Hx8, output, 0);
	EXPECT_FALSE(policy.motionAcknowledged(move.sequence, false, 1, 1));
	EXPECT_TRUE(policy.motionAcknowledged(move.sequence, true, 1, 1));
	output.state = hybrid_control::HybridState::Flying;
	auto hold = policy.update(hybrid_control::ActuatorBackend::Hx8, output, 200'000);
	EXPECT_EQ(hold.action, hybrid_control::Hx8CommandAction::Hold);
	EXPECT_EQ(hold.target, hybrid_control::HybridTarget::Flying);
}

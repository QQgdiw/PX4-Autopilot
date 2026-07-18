#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "TransformationPosition.hpp"
#include "TransformationStateMachine.hpp"

using namespace hybrid_control;

namespace
{
constexpr uint64_t kOneSecondUs = 1'000'000;
constexpr uint64_t kElevenHundredMillisecondsUs = 1'100'000;
constexpr uint64_t kTwelveHundredMillisecondsUs = 1'200'000;
constexpr uint64_t kThirteenHundredMillisecondsUs = 1'300'000;
constexpr uint64_t kFifteenHundredMillisecondsUs = 1'500'000;
constexpr uint64_t kSixteenHundredMillisecondsUs = 1'600'000;
constexpr uint64_t kSeventeenHundredMillisecondsUs = 1'700'000;
constexpr uint64_t kEighteenHundredMillisecondsUs = 1'800'000;
constexpr uint64_t kEighteenHundredAndOneMillisecondsUs = 1'801'000;
constexpr uint64_t kTwentyFiveHundredAndOneMillisecondsUs = 2'501'000;
constexpr uint64_t kEightHundredMillisecondsUs = 800'000;

TransformationConfig transformationConfig()
{
	return {true, 0, -0.7f, 0.8f, 0.5f, 3.f, 0.05f,
		1.f, 0.1f, 3.f, 53, 34, 5.f, 5.f};
}

TransformationInput transformationInput(uint64_t now_us = 0)
{
	return {now_us, false, 0.f, false, false, false, false};
}
}

TEST(TransformationPosition, TmagCacheRequiresMatchingFreshFiniteVector)
{
	TmagSampleCache cache;
	cache.update(53, {1.f, 2.f, 3.f}, 100);

	EXPECT_TRUE(cache.validFor(53, 500'000, kOneSecondUs));
	EXPECT_FALSE(cache.validFor(54, 500'000, kOneSecondUs));
	EXPECT_FALSE(cache.validFor(53, 1'000'101, kOneSecondUs));
	EXPECT_FLOAT_EQ(cache.vector().x, 1.f);
	EXPECT_FLOAT_EQ(cache.vector().y, 2.f);
	EXPECT_FLOAT_EQ(cache.vector().z, 3.f);
	EXPECT_EQ(cache.timestamp(), 100u);
}

TEST(TransformationPosition, TmagCacheRejectsEveryNonFiniteAxis)
{
	TmagSampleCache cache;
	const float nonfinite = std::numeric_limits<float>::quiet_NaN();

	cache.update(53, {nonfinite, 2.f, 3.f}, 100);
	EXPECT_FALSE(cache.validFor(53, 100, kOneSecondUs));
	cache.update(53, {1.f, nonfinite, 3.f}, 101);
	EXPECT_FALSE(cache.validFor(53, 101, kOneSecondUs));
	cache.update(53, {1.f, 2.f, nonfinite}, 102);
	EXPECT_FALSE(cache.validFor(53, 102, kOneSecondUs));
}

TEST(TransformationPosition, As5600ToTmagSwitchRestartsProgressEpoch)
{
	TransformationStateMachine machine;
	auto config = transformationConfig();
	config.sensor_timeout_s = 5.f;
	config.max_transition_s = 3.f;
	config.stall_timeout_s = 0.8f;
	machine.initialize(config, transformationInput());
	machine.request(HybridTarget::Driving, 0);

	auto sample = transformationInput();
	sample.actuator_command_effective = true;
	sample.position = {0.2f, true, false, SensorSource::As5600, 0};
	machine.update(sample);
	sample.now_us = sample.position.timestamp_us = 700'000;
	sample.position.source = SensorSource::Tmag5273;
	EXPECT_EQ(machine.update(sample).state, HybridState::TransitionToRover);
	sample.now_us = sample.position.timestamp_us = 1'499'999;
	EXPECT_EQ(machine.update(sample).state, HybridState::TransitionToRover);
	sample.now_us = sample.position.timestamp_us = 1'500'000;
	EXPECT_EQ(machine.update(sample).fault, TransformFault::Stall);
}

TEST(TransformationPosition, As5600NormalizesWrappedTravel)
{
	EXPECT_FLOAT_EQ(normalizeAs5600(6.1f, 6.1f, 0.2f), 0.f);
	EXPECT_FLOAT_EQ(normalizeAs5600(0.2f, 6.1f, 0.2f), 1.f);
	EXPECT_NEAR(normalizeAs5600(0.f, 6.1f, 0.2f), 0.47808f, 1e-4f);
	EXPECT_TRUE(std::isnan(normalizeAs5600(1.f, 2.f, 2.f)));
}

TEST(TransformationPosition, TmagRatioUsesVectorMagnitude)
{
	TmagRatioFilter filter;
	auto sample = filter.update({3.f, 4.f, 0.f}, {0.f, 0.f, 5.f}, true, true, false, 17);
	EXPECT_TRUE(sample.valid);
	EXPECT_FLOAT_EQ(sample.normalized, 0.5f);
	EXPECT_EQ(sample.source, SensorSource::Tmag5273);
	EXPECT_EQ(sample.timestamp_us, 17u);
}

TEST(TransformationPosition, TmagRatioTracksEndpointDominanceAndFiltering)
{
	TmagRatioFilter filter;
	auto quad = filter.update({10.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, true, true, true, 1);
	EXPECT_FLOAT_EQ(quad.normalized, 0.f);
	EXPECT_TRUE(quad.endpoint_confirmed);

	auto rover = filter.update({0.f, 0.f, 0.f}, {10.f, 0.f, 0.f}, true, true, false, 2);
	EXPECT_FLOAT_EQ(rover.normalized, 0.25f);

	filter.reset();
	rover = filter.update({0.f, 0.f, 0.f}, {10.f, 0.f, 0.f}, true, true, false, 3);
	EXPECT_FLOAT_EQ(rover.normalized, 1.f);
}

TEST(TransformationPosition, TmagRatioRejectsInvalidInput)
{
	TmagRatioFilter filter;
	auto zero = filter.update({}, {}, true, true, false, 1);
	EXPECT_FALSE(zero.valid);
	EXPECT_TRUE(std::isnan(zero.normalized));

	auto nonfinite = filter.update({std::numeric_limits<float>::infinity(), 0.f, 0.f}, {1.f, 0.f, 0.f},
					true, true, false, 2);
	EXPECT_FALSE(nonfinite.valid);
	EXPECT_TRUE(std::isnan(nonfinite.normalized));
}

TEST(TransformationPosition, DirectedProgressResetsOnlyAfterNetForwardDisplacement)
{
	DirectedProgressMonitor monitor;
	monitor.start(0.20f, 1.0f, kOneSecondUs);
	EXPECT_EQ(monitor.update({0.21f, true, false, SensorSource::As5600, kFifteenHundredMillisecondsUs},
				 1.f, .02f, kEightHundredMillisecondsUs),
		  ProgressResult::Progress);
	EXPECT_EQ(monitor.update({0.19f, true, false, SensorSource::As5600, kEighteenHundredAndOneMillisecondsUs},
				 1.f, .02f, kEightHundredMillisecondsUs),
		  ProgressResult::NoProgress);

	monitor.start(0.20f, 1.f, kOneSecondUs);
	EXPECT_EQ(monitor.update({0.18f, true, false, SensorSource::As5600, kThirteenHundredMillisecondsUs},
				 1.f, .02f, kEightHundredMillisecondsUs),
		  ProgressResult::Progress);
	EXPECT_EQ(monitor.update({0.21f, true, false, SensorSource::As5600, kSixteenHundredMillisecondsUs},
				 1.f, .02f, kEightHundredMillisecondsUs),
		  ProgressResult::Progress);
	EXPECT_EQ(monitor.update({0.220001f, true, false, SensorSource::As5600, kSeventeenHundredMillisecondsUs},
				 1.f, .02f, kEightHundredMillisecondsUs),
		  ProgressResult::Progress);
	EXPECT_EQ(monitor.noProgressElapsed(kSeventeenHundredMillisecondsUs), 0u);
	EXPECT_EQ(monitor.update({0.21f, true, false, SensorSource::As5600, kTwentyFiveHundredAndOneMillisecondsUs},
				 1.f, .02f, kEightHundredMillisecondsUs),
		  ProgressResult::NoProgress);
}

TEST(TransformationPosition, DirectedProgressDoesNotResetJustBelowConfiguredDelta)
{
	DirectedProgressMonitor monitor;
	monitor.start(0.20f, 1.f, kOneSecondUs);
	EXPECT_EQ(monitor.update({0.2199995f, true, false, SensorSource::As5600, kFifteenHundredMillisecondsUs},
				 1.f, .02f, kEightHundredMillisecondsUs),
		  ProgressResult::Progress);
	EXPECT_EQ(monitor.noProgressElapsed(kFifteenHundredMillisecondsUs), 500'000u);
}

TEST(TransformationPosition, DirectedProgressTimesOutAtExactBoundary)
{
	DirectedProgressMonitor monitor;
	monitor.start(0.20f, 1.f, kOneSecondUs);
	EXPECT_EQ(monitor.update({0.21f, true, false, SensorSource::As5600, kEighteenHundredMillisecondsUs},
				 1.f, .02f, kEightHundredMillisecondsUs),
		  ProgressResult::NoProgress);
}

TEST(TransformationPosition, DirectedProgressSupportsReverseDirection)
{
	DirectedProgressMonitor monitor;
	monitor.start(0.8f, 0.f, kOneSecondUs);
	EXPECT_EQ(monitor.update({0.77f, true, false, SensorSource::Hx8, kTwelveHundredMillisecondsUs},
				 0.f, .02f, kEightHundredMillisecondsUs),
		  ProgressResult::Progress);
	EXPECT_EQ(monitor.noProgressElapsed(kTwelveHundredMillisecondsUs), 0u);
}

TEST(TransformationPosition, DirectedProgressHandlesReachedInvalidAndIdle)
{
	DirectedProgressMonitor monitor;
	PositionSample valid{0.2f, true, false, SensorSource::As5600, kOneSecondUs};
	EXPECT_EQ(monitor.update(valid, 1.f, .02f, kEightHundredMillisecondsUs), ProgressResult::Idle);

	monitor.start(0.2f, 1.f, kOneSecondUs);
	PositionSample invalid{NAN, false, false, SensorSource::As5600, kElevenHundredMillisecondsUs};
	EXPECT_EQ(monitor.update(invalid, 1.f, .02f, kEightHundredMillisecondsUs), ProgressResult::Invalid);

	PositionSample reached{1.f, true, true, SensorSource::As5600, kTwelveHundredMillisecondsUs};
	EXPECT_EQ(monitor.update(reached, 1.f, .02f, kEightHundredMillisecondsUs), ProgressResult::Reached);
	EXPECT_EQ(monitor.update(valid, 1.f, .02f, kEightHundredMillisecondsUs), ProgressResult::Idle);
}

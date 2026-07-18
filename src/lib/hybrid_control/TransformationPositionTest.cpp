#include <gtest/gtest.h>

#include <drivers/drv_hrt.h>

#include <cmath>
#include <limits>

#include "TransformationPosition.hpp"

using namespace hybrid_control;
using namespace time_literals;

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
	monitor.start(0.20f, 1.0f, 1_s);
	EXPECT_EQ(monitor.update({0.21f, true, false, SensorSource::As5600, 1500_ms}, 1.f, .02f, 800_ms),
		  ProgressResult::Progress);
	EXPECT_EQ(monitor.update({0.19f, true, false, SensorSource::As5600, 1801_ms}, 1.f, .02f, 800_ms),
		  ProgressResult::NoProgress);

	monitor.start(0.20f, 1.f, 1_s);
	EXPECT_EQ(monitor.update({0.18f, true, false, SensorSource::As5600, 1300_ms}, 1.f, .02f, 800_ms),
		  ProgressResult::Progress);
	EXPECT_EQ(monitor.update({0.21f, true, false, SensorSource::As5600, 1600_ms}, 1.f, .02f, 800_ms),
		  ProgressResult::Progress);
	EXPECT_EQ(monitor.update({0.22f, true, false, SensorSource::As5600, 1700_ms}, 1.f, .02f, 800_ms),
		  ProgressResult::Progress);
	EXPECT_EQ(monitor.noProgressElapsed(1700_ms), 0u);
	EXPECT_EQ(monitor.update({0.21f, true, false, SensorSource::As5600, 2501_ms}, 1.f, .02f, 800_ms),
		  ProgressResult::NoProgress);
}

TEST(TransformationPosition, DirectedProgressSupportsReverseDirection)
{
	DirectedProgressMonitor monitor;
	monitor.start(0.8f, 0.f, 1_s);
	EXPECT_EQ(monitor.update({0.77f, true, false, SensorSource::Hx8, 1200_ms}, 0.f, .02f, 800_ms),
		  ProgressResult::Progress);
	EXPECT_EQ(monitor.noProgressElapsed(1200_ms), 0u);
}

TEST(TransformationPosition, DirectedProgressHandlesReachedInvalidAndIdle)
{
	DirectedProgressMonitor monitor;
	PositionSample valid{0.2f, true, false, SensorSource::As5600, 1_s};
	EXPECT_EQ(monitor.update(valid, 1.f, .02f, 800_ms), ProgressResult::Idle);

	monitor.start(0.2f, 1.f, 1_s);
	PositionSample invalid{NAN, false, false, SensorSource::As5600, 1100_ms};
	EXPECT_EQ(monitor.update(invalid, 1.f, .02f, 800_ms), ProgressResult::Invalid);

	PositionSample reached{1.f, true, true, SensorSource::As5600, 1200_ms};
	EXPECT_EQ(monitor.update(reached, 1.f, .02f, 800_ms), ProgressResult::Reached);
	EXPECT_EQ(monitor.update(valid, 1.f, .02f, 800_ms), ProgressResult::Idle);
}

// Copyright (c) 2026 PX4 Development Team. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>
#include <unistd.h>
#include <parameters/param.h>
#include <uORB/PublicationMulti.hpp>
#include "EKF2Selector.hpp"

// Exercise the real selector forwarding/reset path, not a copied selection policy.
class EKF2SelectorTestHarness
{
public:
	static void forward(EKF2Selector &selector, int instance)
	{
		selector._selected_instance = instance;
		selector.PublishVehicleLocalPosition();
	}
};

TEST(EKF2Selector, LocalHeadingQualificationFollowsSelectedSample)
{
	param_control_autosave(false);
	hrt_init(); // ScheduledWorkItem destruction cancels its timer under the HRT lock.
	EKF2Selector selector;
	uORB::PublicationMulti<vehicle_local_position_s> pub0{ORB_ID(estimator_local_position)};
	uORB::PublicationMulti<vehicle_local_position_s> pub1{ORB_ID(estimator_local_position)};
	uORB::Subscription sub{ORB_ID(vehicle_local_position)};
	vehicle_local_position_s sample{};
	sample.timestamp = hrt_absolute_time();
	sample.timestamp_sample = sample.timestamp;
	sample.heading = 0.5f;
	sample.x = 1.f;
	sample.heading_good_for_local_control = true;
	pub0.publish(sample);
	EKF2SelectorTestHarness::forward(selector, pub0.get_instance());
	vehicle_local_position_s output{};
	ASSERT_TRUE(sub.update(&output));
	EXPECT_TRUE(output.heading_good_for_local_control);
	EXPECT_FALSE(output.heading_good_for_control);
	EXPECT_EQ(output.timestamp_sample, sample.timestamp_sample);
	EXPECT_FLOAT_EQ(output.heading, sample.heading);
	EXPECT_FLOAT_EQ(output.x, sample.x);
	const uint8_t previous_reset = output.heading_reset_counter;

	usleep(1000);
	sample.timestamp = hrt_absolute_time();
	sample.timestamp_sample = sample.timestamp;
	sample.heading = -0.8f;
	sample.x = 10.f;
	sample.heading_good_for_local_control = false;
	pub1.publish(sample);
	ASSERT_NE(pub0.get_instance(), pub1.get_instance());
	EKF2SelectorTestHarness::forward(selector, pub1.get_instance());
	ASSERT_TRUE(sub.update(&output));
	EXPECT_FALSE(output.heading_good_for_local_control);
	EXPECT_EQ(output.timestamp_sample, sample.timestamp_sample);
	EXPECT_FLOAT_EQ(output.heading, sample.heading);
	EXPECT_FLOAT_EQ(output.x, sample.x);
	EXPECT_EQ(output.heading_reset_counter, uint8_t(previous_reset + 1));
	EXPECT_NEAR(output.delta_heading, -1.3f, 1e-5f);

	// New status on the unselected estimator must not qualify selected position.
	sample.heading_good_for_local_control = true;
	pub0.publish(sample);
	EKF2SelectorTestHarness::forward(selector, pub1.get_instance());
	EXPECT_FALSE(sub.updated());
	usleep(1000);
	sample.timestamp = hrt_absolute_time();
	sample.timestamp_sample = sample.timestamp;
	sample.heading_good_for_local_control = true;
	pub1.publish(sample);
	EKF2SelectorTestHarness::forward(selector, pub1.get_instance());
	ASSERT_TRUE(sub.update(&output));
	EXPECT_TRUE(output.heading_good_for_local_control);
	EXPECT_EQ(output.timestamp_sample, sample.timestamp_sample);
	// Out-of-order sample must not be made fresh by selector republishing.
	sample.timestamp_sample--;
	sample.heading_good_for_local_control = false;
	pub1.publish(sample);
	EKF2SelectorTestHarness::forward(selector, pub1.get_instance());
	EXPECT_FALSE(sub.updated());

	sample.timestamp = hrt_absolute_time();
	sample.timestamp_sample = sample.timestamp;
	pub1.publish(sample);
	usleep(25000); // selector's existing publication-age limit is 20 ms
	EKF2SelectorTestHarness::forward(selector, pub1.get_instance());
	EXPECT_FALSE(sub.updated());
}

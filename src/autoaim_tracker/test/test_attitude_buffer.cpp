#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "autoaim_tracker/attitude_buffer.hpp"

namespace {

using autoaim_tracker::AttitudeBuffer;
using autoaim_tracker::AttitudeSample;

TEST(AttitudeBuffer, RejectsNonMonotonicAndNonFiniteSamples) {
    AttitudeBuffer buffer(8);
    EXPECT_TRUE(buffer.push(1.0, 0.1, 0.2));
    EXPECT_FALSE(buffer.push(1.0, 0.3, 0.4));
    EXPECT_FALSE(buffer.push(0.5, 0.3, 0.4));
    EXPECT_FALSE(buffer.push(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
    EXPECT_EQ(buffer.size(), 1U);
}

TEST(AttitudeBuffer, PicksTheClosestSampleOnEitherSide) {
    AttitudeBuffer buffer(8);
    buffer.push(0.0, 0.1, 0.0);
    buffer.push(0.01, 0.2, 0.0);
    buffer.push(0.02, 0.3, 0.0);

    AttitudeSample sample;
    ASSERT_TRUE(buffer.nearest(0.004, 0.05, &sample));
    EXPECT_DOUBLE_EQ(sample.yaw, 0.1);
    ASSERT_TRUE(buffer.nearest(0.006, 0.05, &sample));
    EXPECT_DOUBLE_EQ(sample.yaw, 0.2);
}

TEST(AttitudeBuffer, RejectsWhenTheNearestSampleIsTooFarAway) {
    AttitudeBuffer buffer(8);
    buffer.push(0.0, 0.1, 0.0);

    AttitudeSample sample;
    EXPECT_FALSE(buffer.nearest(0.06, 0.05, &sample));
    EXPECT_TRUE(buffer.nearest(0.05, 0.05, &sample));
}

TEST(AttitudeBuffer, EmptyBufferNeverMatches) {
    AttitudeBuffer buffer(8);
    AttitudeSample sample;
    EXPECT_FALSE(buffer.nearest(0.0, 1.0, &sample));
    EXPECT_TRUE(buffer.empty());
}

TEST(AttitudeBuffer, TrimsTheOldestSamplesOnceFull) {
    AttitudeBuffer buffer(3);
    for (int i = 0; i < 5; ++i) {
        buffer.push(static_cast<double>(i), static_cast<double>(i), 0.0);
    }
    EXPECT_EQ(buffer.size(), 3U);

    AttitudeSample sample;
    EXPECT_FALSE(buffer.nearest(0.0, 0.1, &sample));
    ASSERT_TRUE(buffer.nearest(4.0, 0.1, &sample));
    EXPECT_DOUBLE_EQ(sample.yaw, 4.0);
}

TEST(ComposeTarget, AddsTheAttitudeToTheOpticalAxisOffset) {
    AttitudeSample attitude;
    attitude.stamp = 12.0;
    attitude.yaw = 0.4;
    attitude.pitch = -0.1;

    double yaw_target = 0.0;
    double pitch_target = 0.0;
    autoaim_tracker::compose_target(attitude, -0.15, 0.05, &yaw_target, &pitch_target);

    EXPECT_DOUBLE_EQ(yaw_target, 0.25);
    EXPECT_DOUBLE_EQ(pitch_target, -0.05);
}

} // namespace

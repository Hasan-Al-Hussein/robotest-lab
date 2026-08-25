// Copyright 2026 Hasan Ahmed
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <limits>
#include <string>

#include "rmw/types.h"
#include "robotest_faults/pass_through.hpp"
#include "robotest_faults/qos_profiles.hpp"
#include "gtest/gtest.h"

namespace robotest_faults
{
namespace
{

sensor_msgs::msg::LaserScan valid_scan()
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp.sec = 12;
  scan.header.stamp.nanosec = 345U;
  scan.header.frame_id = "lidar_link";
  scan.angle_min = -1.0F;
  scan.angle_max = 1.0F;
  scan.angle_increment = 0.5F;
  scan.time_increment = 0.01F;
  scan.scan_time = 0.05F;
  scan.range_min = 0.1F;
  scan.range_max = 8.0F;
  scan.ranges = {1.0F, std::numeric_limits<float>::infinity(),
    std::numeric_limits<float>::quiet_NaN(), 2.0F, 3.0F};
  scan.intensities = {2.0F, 3.0F};
  return scan;
}

nav_msgs::msg::Odometry valid_odom()
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp.sec = 42;
  odom.header.stamp.nanosec = 123456789U;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = 1.25;
  odom.pose.pose.position.y = -0.75;
  odom.pose.pose.position.z = 0.02;
  odom.pose.pose.orientation.z = 0.5;
  odom.pose.pose.orientation.w = std::sqrt(0.75);
  odom.twist.twist.linear.x = 0.4;
  odom.twist.twist.angular.z = -0.1;
  odom.pose.covariance[0] = 0.01;
  odom.twist.covariance[35] = 0.02;
  return odom;
}

sensor_msgs::msg::Imu valid_imu_without_orientation()
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp.sec = 9;
  imu.header.frame_id = "imu_link";
  imu.orientation_covariance[0] = -1.0;
  imu.angular_velocity.z = 0.2;
  imu.linear_acceleration.z = 9.81;
  return imu;
}

TEST(PassThrough,
     ScanPreservesStampFrameAndPayloadIncludingNonFiniteRangeMarkers) {
  const auto input = valid_scan();
  const auto result = validate_and_copy_scan(input, "lidar_link");

  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.message.header.stamp.sec, input.header.stamp.sec);
  EXPECT_EQ(result.message.header.stamp.nanosec, input.header.stamp.nanosec);
  EXPECT_EQ(result.message.header.frame_id, input.header.frame_id);
  ASSERT_EQ(result.message.ranges.size(), input.ranges.size());
  EXPECT_FLOAT_EQ(result.message.ranges[0], input.ranges[0]);
  EXPECT_TRUE(std::isinf(result.message.ranges[1]));
  EXPECT_TRUE(std::isnan(result.message.ranges[2]));
  EXPECT_EQ(result.message.intensities, input.intensities);
}

TEST(PassThrough, ScanRejectsUnexpectedFrameWithoutProducingOutput) {
  auto input = valid_scan();
  input.header.frame_id = "base_scan";

  const auto result = validate_and_copy_scan(input, "lidar_link");
  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.reason.empty());
}

TEST(PassThrough, OdomOutputAndTransformShareInputStampFramesAndPose) {
  const auto input = valid_odom();
  const auto result = validate_and_copy_odom(input, "odom", "base_footprint");
  ASSERT_TRUE(result.accepted) << result.reason;

  const auto transform = make_odom_transform(result.message);
  EXPECT_EQ(result.message.header.stamp.sec, input.header.stamp.sec);
  EXPECT_EQ(result.message.header.stamp.nanosec, input.header.stamp.nanosec);
  EXPECT_EQ(result.message.header.frame_id, "odom");
  EXPECT_EQ(result.message.child_frame_id, "base_footprint");
  EXPECT_EQ(transform.header.stamp.sec, result.message.header.stamp.sec);
  EXPECT_EQ(transform.header.stamp.nanosec,
            result.message.header.stamp.nanosec);
  EXPECT_EQ(transform.header.frame_id, result.message.header.frame_id);
  EXPECT_EQ(transform.child_frame_id, result.message.child_frame_id);
  EXPECT_DOUBLE_EQ(transform.transform.translation.x,
                   input.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(transform.transform.translation.y,
                   input.pose.pose.position.y);
  EXPECT_DOUBLE_EQ(transform.transform.rotation.z,
                   input.pose.pose.orientation.z);
  EXPECT_DOUBLE_EQ(result.message.twist.twist.linear.x,
                   input.twist.twist.linear.x);
  EXPECT_DOUBLE_EQ(result.message.pose.covariance[0], input.pose.covariance[0]);
}

TEST(PassThrough, OdomRejectsWrongChildFrameAndNonNormalizedQuaternion) {
  auto input = valid_odom();
  input.child_frame_id = "base_link";
  EXPECT_FALSE(
      validate_and_copy_odom(input, "odom", "base_footprint").accepted);

  input.child_frame_id = "base_footprint";
  input.pose.pose.orientation.w = 0.0;
  input.pose.pose.orientation.z = 0.0;
  EXPECT_FALSE(
      validate_and_copy_odom(input, "odom", "base_footprint").accepted);
}

TEST(PassThrough, ImuAcceptsStandardUnavailableOrientationAndPreservesVectors) {
  const auto input = valid_imu_without_orientation();
  const auto result = validate_and_copy_imu(input, "imu_link");

  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.message.header.frame_id, input.header.frame_id);
  EXPECT_EQ(result.message.header.stamp.sec, input.header.stamp.sec);
  EXPECT_DOUBLE_EQ(result.message.orientation_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(result.message.angular_velocity.z, input.angular_velocity.z);
  EXPECT_DOUBLE_EQ(result.message.linear_acceleration.z,
                   input.linear_acceleration.z);
}

void expect_sensor_qos(const rclcpp::QoS & qos, const std::size_t depth)
{
  const auto profile = qos.get_rmw_qos_profile();
  EXPECT_EQ(profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(profile.depth, depth);
  EXPECT_EQ(profile.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(QosProfiles, MatchTheBoundedTopicContractExactly) {
  expect_sensor_qos(scan_sensor_qos(), 5U);
  expect_sensor_qos(odom_sensor_qos(), 10U);
  expect_sensor_qos(imu_sensor_qos(), 10U);

  const auto event_profile = fault_event_qos().get_rmw_qos_profile();
  EXPECT_EQ(event_profile.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(event_profile.depth, 100U);
  EXPECT_EQ(event_profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  EXPECT_EQ(event_profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

}  // namespace
}  // namespace robotest_faults

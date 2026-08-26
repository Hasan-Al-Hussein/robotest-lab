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

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "robotest_faults/fault_protocol.hpp"
#include "robotest_faults/pass_through.hpp"

namespace robotest_faults
{
namespace
{

using FaultEvent = robotest_interfaces::msg::FaultEvent;
using FaultSpec = robotest_interfaces::msg::FaultSpec;

builtin_interfaces::msg::Time make_time(
  const std::int32_t seconds,
  const std::uint32_t nanoseconds = 0U)
{
  builtin_interfaces::msg::Time value;
  value.sec = seconds;
  value.nanosec = nanoseconds;
  return value;
}

builtin_interfaces::msg::Duration make_duration(
  const std::int64_t nanoseconds)
{
  builtin_interfaces::msg::Duration value;
  value.sec = static_cast<std::int32_t>(
    nanoseconds / kNanosecondsPerSecond);
  value.nanosec = static_cast<std::uint32_t>(
    nanoseconds % kNanosecondsPerSecond);
  return value;
}

FaultSpec make_fault(
  std::string id,
  const std::uint8_t target,
  const std::uint8_t mode,
  const std::int64_t start_ns,
  const std::int64_t duration_ns,
  std::string parameters = "{}",
  const std::uint64_t seed = 42U)
{
  FaultSpec fault;
  fault.schema_version = FaultSpec::SCHEMA_VERSION;
  fault.fault_id = std::move(id);
  fault.target = target;
  fault.mode = mode;
  fault.start_offset = make_duration(start_ns);
  fault.duration = make_duration(duration_ns);
  fault.seed = seed;
  fault.parameters_json = std::move(parameters);
  return fault;
}

void prepare_and_arm(
  FaultProtocol & protocol,
  const std::vector<FaultSpec> & faults,
  const builtin_interfaces::msg::Time & t0 = make_time(100))
{
  const auto canonical = canonicalize_schedule(faults);
  if (!canonical.accepted) {
    throw std::logic_error(canonical.message);
  }
  robotest_interfaces::srv::PreloadFaultSchedule::Request preload_request;
  preload_request.schedule_hash = canonical.schedule.sha256;
  for (const auto & fault : faults) {
    preload_request.faults.push_back(fault);
  }
  const auto preload = protocol.preload(preload_request, make_time(90));
  if (!preload.accepted) {
    throw std::logic_error(preload.message);
  }

  robotest_interfaces::srv::ArmFaultSchedule::Request arm_request;
  arm_request.schedule_hash = preload.schedule_hash;
  arm_request.generation = preload.generation;
  arm_request.goal_uuid.uuid[0] = 1U;
  arm_request.accepted_goal_stamp = t0;
  const auto armed = protocol.arm(arm_request, t0);
  if (!armed.accepted) {
    throw std::logic_error(armed.message);
  }
}

sensor_msgs::msg::LaserScan make_scan(
  const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = "lidar_link";
  scan.angle_min = -1.0F;
  scan.angle_max = 1.0F;
  scan.angle_increment = 0.5F;
  scan.time_increment = 0.01F;
  scan.scan_time = 0.05F;
  scan.range_min = 0.1F;
  scan.range_max = 8.0F;
  scan.ranges = {1.0F, 2.0F, 3.0F};
  return scan;
}

nav_msgs::msg::Odometry make_odom(
  const builtin_interfaces::msg::Time & stamp,
  const double yaw = 0.3)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = 3.0;
  odom.pose.pose.position.y = 4.0;
  odom.pose.pose.position.z = 0.25;
  odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
  odom.pose.pose.orientation.w = std::cos(yaw / 2.0);
  odom.twist.twist.linear.x = 0.7;
  odom.twist.twist.linear.y = -0.2;
  odom.twist.twist.angular.z = 0.4;
  odom.pose.covariance[0] = 0.01;
  odom.pose.covariance[7] = 0.02;
  odom.twist.covariance[0] = 0.03;
  odom.twist.covariance[35] = 0.04;
  return odom;
}

sensor_msgs::msg::Imu make_imu(
  const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = "imu_link";
  imu.orientation_covariance[0] = -1.0;
  imu.angular_velocity.z = 0.2;
  imu.linear_acceleration.z = 9.81;
  return imu;
}

double yaw_of(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::atan2(
    2.0 * (quaternion.w * quaternion.z +
    quaternion.x * quaternion.y),
    1.0 - 2.0 * (quaternion.y * quaternion.y +
    quaternion.z * quaternion.z));
}

TEST(FaultApplication, PreparedScheduleIsInert)
{
  FaultProtocol protocol;
  const std::vector<FaultSpec> faults{
    make_fault("drop", 1U, 1U, 10000000000LL, 2000000000LL)};
  const auto canonical = canonicalize_schedule(faults);
  ASSERT_TRUE(canonical.accepted);
  robotest_interfaces::srv::PreloadFaultSchedule::Request request;
  request.schedule_hash = canonical.schedule.sha256;
  request.faults.push_back(faults.front());
  ASSERT_TRUE(protocol.preload(request, make_time(90)).accepted);

  const auto input = make_scan(make_time(110));
  const auto result = protocol.process_scan(input, "lidar_link");
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_TRUE(result.publish);
  EXPECT_EQ(result.message.ranges, input.ranges);
  EXPECT_TRUE(result.events.empty());
}

TEST(FaultApplication, LidarDropoutUsesHalfOpenIntervalAndExactCounters)
{
  FaultProtocol protocol;
  prepare_and_arm(
    protocol,
    {make_fault("drop", 1U, 1U, 10000000000LL, 2000000000LL)});

  auto before = protocol.process_scan(
    make_scan(make_time(109, 999999999U)), "lidar_link");
  ASSERT_TRUE(before.accepted);
  EXPECT_TRUE(before.publish);
  EXPECT_TRUE(before.events.empty());

  auto start = protocol.process_scan(make_scan(make_time(110)), "lidar_link");
  ASSERT_TRUE(start.accepted);
  EXPECT_FALSE(start.publish);
  ASSERT_EQ(start.events.size(), 2U);
  EXPECT_EQ(start.events[0].event_type, FaultEvent::EVENT_ACTIVATED);
  EXPECT_EQ(start.events[1].event_type, FaultEvent::EVENT_FIRST_AFFECTED);
  EXPECT_EQ(start.events[0].event_sequence + 1U,
            start.events[1].event_sequence);
  EXPECT_EQ(start.events[1].input_sequence, 2U);
  EXPECT_EQ(start.events[1].raw_input_count, 2U);
  EXPECT_EQ(start.events[1].validated_output_count, 1U);
  EXPECT_EQ(start.events[1].affected_message_count, 1U);
  EXPECT_EQ(start.events[1].schema_version, FaultEvent::SCHEMA_VERSION);
  EXPECT_EQ(start.events[1].state_before, FaultEvent::STATE_ARMED);
  EXPECT_EQ(start.events[1].state_after, FaultEvent::STATE_ARMED);
  EXPECT_TRUE(start.events[1].accepted);
  EXPECT_FALSE(start.events[1].replayed);
  EXPECT_EQ(start.events[1].seed, 42U);
  EXPECT_FALSE(start.events[1].committed_schedule_hash.empty());
  EXPECT_EQ(start.events[1].header.stamp, start.events[1].actual_time);
  EXPECT_EQ(start.events[1].bound_t0.sec, 100);
  EXPECT_EQ(start.events[1].arm_commit_time.sec, 100);
  EXPECT_EQ(start.events[1].configured_activation_time.sec, 110);
  EXPECT_EQ(start.events[1].configured_deactivation_time.sec, 112);

  auto middle = protocol.process_scan(make_scan(make_time(111)), "lidar_link");
  ASSERT_TRUE(middle.accepted);
  EXPECT_FALSE(middle.publish);
  EXPECT_TRUE(middle.events.empty());

  const auto endpoint_input = make_scan(make_time(112));
  auto endpoint = protocol.process_scan(endpoint_input, "lidar_link");
  ASSERT_TRUE(endpoint.accepted);
  EXPECT_TRUE(endpoint.publish);
  EXPECT_EQ(endpoint.message.ranges, endpoint_input.ranges);
  ASSERT_EQ(endpoint.events.size(), 2U);
  EXPECT_EQ(endpoint.events[0].event_type, FaultEvent::EVENT_DEACTIVATED);
  EXPECT_EQ(endpoint.events[0].validated_output_count, 1U);
  EXPECT_EQ(endpoint.events[1].event_type, FaultEvent::EVENT_FIRST_RESTORED);
  EXPECT_EQ(endpoint.events[1].validated_output_count, 2U);
  EXPECT_EQ(endpoint.events[1].raw_input_count, 4U);
  EXPECT_EQ(endpoint.events[1].affected_message_count, 2U);

  auto later = protocol.process_scan(make_scan(make_time(113)), "lidar_link");
  EXPECT_TRUE(later.accepted);
  EXPECT_TRUE(later.publish);
  EXPECT_TRUE(later.events.empty());

  protocol.reset(make_time(114));
  auto after_reset = protocol.process_scan(
    make_scan(make_time(110)), "lidar_link");
  EXPECT_TRUE(after_reset.accepted);
  EXPECT_TRUE(after_reset.publish);
  EXPECT_TRUE(after_reset.events.empty());

  prepare_and_arm(
    protocol,
    {make_fault("drop", 1U, 1U, 10000000000LL, 2000000000LL)});
  const auto after_rearm = protocol.process_scan(
    make_scan(make_time(110)), "lidar_link");
  ASSERT_EQ(after_rearm.events.size(), 2U);
  EXPECT_EQ(after_rearm.events[0].input_sequence, 1U);
  EXPECT_EQ(after_rearm.events[0].raw_input_count, 1U);
  EXPECT_EQ(after_rearm.events[0].validated_output_count, 0U);
}

TEST(FaultApplication, OdomDriftIsLeftComposedAndPreservesAllOtherFields)
{
  FaultProtocol protocol;
  prepare_and_arm(
    protocol,
    {make_fault(
        "drift", 2U, 4U, 10000000000LL, 20000000000LL,
        "{\"x_rate_nm_per_s\":1000000000,"
        "\"yaw_rate_nrad_per_s\":500000000}")});

  const auto input = make_odom(make_time(112));
  const auto raw_copy = input;
  const auto result = protocol.process_odom(
    input, "odom", "base_footprint");
  ASSERT_TRUE(result.accepted) << result.reason;
  ASSERT_TRUE(result.publish);
  ASSERT_EQ(result.events.size(), 2U);
  EXPECT_EQ(result.events[0].event_type, FaultEvent::EVENT_ACTIVATED);
  EXPECT_EQ(result.events[1].event_type, FaultEvent::EVENT_FIRST_AFFECTED);
  EXPECT_EQ(result.events[1].validated_output_count, 1U);
  EXPECT_EQ(result.events[1].affected_message_count, 1U);

  constexpr double delta_yaw = 1.0;
  constexpr double dx = 2.0;
  const double expected_x =
    std::cos(delta_yaw) * input.pose.pose.position.x -
    std::sin(delta_yaw) * input.pose.pose.position.y + dx;
  const double expected_y =
    std::sin(delta_yaw) * input.pose.pose.position.x +
    std::cos(delta_yaw) * input.pose.pose.position.y;
  EXPECT_NEAR(result.message.pose.pose.position.x, expected_x, 1.0e-12);
  EXPECT_NEAR(result.message.pose.pose.position.y, expected_y, 1.0e-12);
  EXPECT_NEAR(yaw_of(result.message.pose.pose.orientation), 1.3, 1.0e-12);

  const double measured_dx =
    result.message.pose.pose.position.x -
    (std::cos(delta_yaw) * input.pose.pose.position.x -
    std::sin(delta_yaw) * input.pose.pose.position.y);
  const double measured_dy =
    result.message.pose.pose.position.y -
    (std::sin(delta_yaw) * input.pose.pose.position.x +
    std::cos(delta_yaw) * input.pose.pose.position.y);
  EXPECT_NEAR(measured_dx, dx, 1.0e-12);
  EXPECT_NEAR(measured_dy, 0.0, 1.0e-12);
  EXPECT_NEAR(
    yaw_of(result.message.pose.pose.orientation) -
    yaw_of(input.pose.pose.orientation),
    delta_yaw, 1.0e-12);

  EXPECT_EQ(result.message.header.stamp.sec, input.header.stamp.sec);
  EXPECT_EQ(result.message.header.stamp.nanosec, input.header.stamp.nanosec);
  EXPECT_EQ(result.message.header.frame_id, input.header.frame_id);
  EXPECT_EQ(result.message.child_frame_id, input.child_frame_id);
  EXPECT_DOUBLE_EQ(
    result.message.pose.pose.position.z, input.pose.pose.position.z);
  EXPECT_EQ(result.message.pose.covariance, input.pose.covariance);
  EXPECT_EQ(result.message.twist.covariance, input.twist.covariance);
  EXPECT_DOUBLE_EQ(
    result.message.twist.twist.linear.x, input.twist.twist.linear.x);
  EXPECT_DOUBLE_EQ(
    result.message.twist.twist.linear.y, input.twist.twist.linear.y);
  EXPECT_DOUBLE_EQ(
    result.message.twist.twist.angular.z, input.twist.twist.angular.z);
  EXPECT_DOUBLE_EQ(input.pose.pose.position.x, raw_copy.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(input.pose.pose.position.y, raw_copy.pose.pose.position.y);

  const auto transform = make_odom_transform(result.message);
  EXPECT_EQ(transform.header, result.message.header);
  EXPECT_EQ(transform.child_frame_id, result.message.child_frame_id);
  EXPECT_DOUBLE_EQ(
    transform.transform.translation.x, result.message.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(
    transform.transform.translation.y, result.message.pose.pose.position.y);
  EXPECT_EQ(transform.transform.rotation, result.message.pose.pose.orientation);

  const auto final_active = protocol.process_odom(
    make_odom(make_time(129, 750000000U)), "odom", "base_footprint");
  ASSERT_TRUE(final_active.accepted);
  EXPECT_TRUE(final_active.publish);
  EXPECT_NEAR(
    final_active.message.pose.pose.position.x -
    (std::cos(9.875) * 3.0 - std::sin(9.875) * 4.0),
    19.75, 1.0e-10);

  const auto endpoint_input = make_odom(make_time(130));
  const auto endpoint = protocol.process_odom(
    endpoint_input, "odom", "base_footprint");
  ASSERT_TRUE(endpoint.accepted);
  EXPECT_TRUE(endpoint.publish);
  EXPECT_DOUBLE_EQ(
    endpoint.message.pose.pose.position.x,
    endpoint_input.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(
    endpoint.message.pose.pose.position.y,
    endpoint_input.pose.pose.position.y);
  ASSERT_EQ(endpoint.events.size(), 2U);
  EXPECT_EQ(endpoint.events[0].event_type, FaultEvent::EVENT_DEACTIVATED);
  EXPECT_EQ(endpoint.events[1].event_type, FaultEvent::EVENT_FIRST_RESTORED);
  EXPECT_EQ(endpoint.events[1].affected_message_count, 2U);
}

TEST(FaultApplication, DriftAtActivationHasZeroOffsetButIsAffected)
{
  FaultProtocol protocol;
  prepare_and_arm(
    protocol,
    {make_fault(
        "drift", 2U, 4U, 10000000000LL, 1000000000LL,
        "{\"x_rate_nm_per_s\":100,\"yaw_rate_nrad_per_s\":200}")});
  const auto input = make_odom(make_time(110));
  const auto result = protocol.process_odom(
    input, "odom", "base_footprint");
  ASSERT_TRUE(result.accepted);
  EXPECT_DOUBLE_EQ(
    result.message.pose.pose.position.x, input.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(
    result.message.pose.pose.position.y, input.pose.pose.position.y);
  ASSERT_EQ(result.events.size(), 2U);
  EXPECT_EQ(result.events[1].event_type, FaultEvent::EVENT_FIRST_AFFECTED);
  EXPECT_EQ(result.events[1].affected_message_count, 1U);
}

TEST(FaultApplication, SameSeedAndInputsProduceIdenticalOutputsAndEvidence)
{
  const std::vector<FaultSpec> faults{
    make_fault(
      "drift", 2U, 4U, 10000000000LL, 2000000000LL,
      "{\"x_rate_nm_per_s\":123456789,"
      "\"yaw_rate_nrad_per_s\":-987654321}", 8675309U)};
  FaultProtocol first;
  FaultProtocol second;
  prepare_and_arm(first, faults);
  prepare_and_arm(second, faults);
  const auto input = make_odom(make_time(111, 250000000U));
  const auto first_result = first.process_odom(
    input, "odom", "base_footprint");
  const auto second_result = second.process_odom(
    input, "odom", "base_footprint");

  ASSERT_TRUE(first_result.accepted);
  ASSERT_TRUE(second_result.accepted);
  EXPECT_EQ(first_result.publish, second_result.publish);
  EXPECT_EQ(first_result.message, second_result.message);
  ASSERT_EQ(first_result.events.size(), second_result.events.size());
  for (std::size_t index = 0U; index < first_result.events.size(); ++index) {
    EXPECT_EQ(first_result.events[index], second_result.events[index]);
  }
}

TEST(FaultApplication, PassThroughModeAndImuRemainUnmodified)
{
  FaultProtocol protocol;
  prepare_and_arm(
    protocol,
    {make_fault("imu_pass", 3U, 0U, 10000000000LL, 1000000000LL)});
  const auto input = make_imu(make_time(110));
  const auto active = protocol.process_imu(input, "imu_link");
  ASSERT_TRUE(active.accepted);
  EXPECT_TRUE(active.publish);
  EXPECT_EQ(active.message, input);
  ASSERT_EQ(active.events.size(), 1U);
  EXPECT_EQ(active.events[0].event_type, FaultEvent::EVENT_ACTIVATED);

  const auto endpoint_input = make_imu(make_time(111));
  const auto endpoint = protocol.process_imu(endpoint_input, "imu_link");
  ASSERT_TRUE(endpoint.accepted);
  EXPECT_TRUE(endpoint.publish);
  EXPECT_EQ(endpoint.message, endpoint_input);
  ASSERT_EQ(endpoint.events.size(), 1U);
  EXPECT_EQ(endpoint.events[0].event_type, FaultEvent::EVENT_DEACTIVATED);
}

TEST(FaultApplication, InvalidAndRegressingInputsAreRejectedWithoutOutput)
{
  FaultProtocol protocol;
  prepare_and_arm(
    protocol,
    {make_fault("drop", 1U, 1U, 10000000000LL, 2000000000LL)});
  auto invalid = make_scan(make_time(110));
  invalid.header.frame_id = "wrong";
  const auto invalid_result = protocol.process_scan(invalid, "lidar_link");
  EXPECT_FALSE(invalid_result.accepted);
  EXPECT_FALSE(invalid_result.publish);
  EXPECT_TRUE(invalid_result.events.empty());

  const auto active = protocol.process_scan(
    make_scan(make_time(110)), "lidar_link");
  ASSERT_TRUE(active.accepted);
  ASSERT_EQ(active.events.size(), 2U);
  EXPECT_EQ(active.events[0].input_sequence, 1U);
  EXPECT_EQ(active.events[0].raw_input_count, 1U);

  const auto regressing = protocol.process_scan(
    make_scan(make_time(109)), "lidar_link");
  EXPECT_FALSE(regressing.accepted);
  EXPECT_FALSE(regressing.publish);
  EXPECT_FALSE(regressing.reason.empty());
}

TEST(FaultApplication, TouchingDropoutsRestoreOnlyAtFirstUnmodifiedOutput)
{
  FaultProtocol protocol;
  prepare_and_arm(
    protocol,
      {
        make_fault("first", 1U, 1U, 10000000000LL, 1000000000LL),
        make_fault("second", 1U, 1U, 11000000000LL, 1000000000LL),
    });
  EXPECT_FALSE(protocol.process_scan(
      make_scan(make_time(110)), "lidar_link").publish);

  const auto touching = protocol.process_scan(
    make_scan(make_time(111)), "lidar_link");
  EXPECT_TRUE(touching.accepted);
  EXPECT_FALSE(touching.publish);
  ASSERT_EQ(touching.events.size(), 3U);
  EXPECT_EQ(touching.events[0].event_type, FaultEvent::EVENT_DEACTIVATED);
  EXPECT_EQ(touching.events[1].event_type, FaultEvent::EVENT_ACTIVATED);
  EXPECT_EQ(touching.events[2].event_type, FaultEvent::EVENT_FIRST_AFFECTED);

  const auto restored = protocol.process_scan(
    make_scan(make_time(112)), "lidar_link");
  EXPECT_TRUE(restored.accepted);
  EXPECT_TRUE(restored.publish);
  ASSERT_EQ(restored.events.size(), 3U);
  EXPECT_EQ(restored.events[0].event_type, FaultEvent::EVENT_DEACTIVATED);
  EXPECT_EQ(restored.events[1].event_type, FaultEvent::EVENT_FIRST_RESTORED);
  EXPECT_EQ(restored.events[1].fault_id, "first");
  EXPECT_EQ(restored.events[2].event_type, FaultEvent::EVENT_FIRST_RESTORED);
  EXPECT_EQ(restored.events[2].fault_id, "second");
}

}  // namespace
}  // namespace robotest_faults

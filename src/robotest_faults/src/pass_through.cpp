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

#include "robotest_faults/pass_through.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <string>

namespace robotest_faults
{
namespace
{

constexpr double kQuaternionNormTolerance = 1.0e-3;
constexpr uint32_t kNanosecondsPerSecond = 1000000000U;

bool valid_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec >= 0 && stamp.nanosec < kNanosecondsPerSecond;
}

bool valid_frame(const std::string & actual, const std::string & expected)
{
  return !actual.empty() && actual == expected && actual.front() != '/';
}

bool normalized_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const std::array<double, 4> values{quaternion.x, quaternion.y, quaternion.z,
    quaternion.w};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }

  const double squared_norm =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return std::abs(squared_norm - 1.0) <= kQuaternionNormTolerance;
}

template<typename ContainerT> bool all_finite(const ContainerT & values)
{
  for (const auto value : values) {
    if (!std::isfinite(static_cast<double>(value))) {
      return false;
    }
  }
  return true;
}

bool finite_vector(const geometry_msgs::msg::Vector3 & vector)
{
  return std::isfinite(vector.x) && std::isfinite(vector.y) &&
         std::isfinite(vector.z);
}

}  // namespace

ScanPassThroughResult
validate_and_copy_scan(
  const sensor_msgs::msg::LaserScan & input,
  const std::string & expected_frame)
{
  ScanPassThroughResult result;
  if (!valid_stamp(input.header.stamp)) {
    result.reason = "scan stamp is outside the ROS time domain";
    return result;
  }
  if (!valid_frame(input.header.frame_id, expected_frame)) {
    result.reason = "scan frame_id does not match the configured sensor frame";
    return result;
  }
  if (!std::isfinite(input.angle_min) || !std::isfinite(input.angle_max) ||
    !std::isfinite(input.angle_increment) || input.angle_increment <= 0.0F ||
    !std::isfinite(input.time_increment) || input.time_increment < 0.0F ||
    !std::isfinite(input.scan_time) || input.scan_time < 0.0F ||
    !std::isfinite(input.range_min) || input.range_min < 0.0F ||
    !std::isfinite(input.range_max) || input.range_max <= input.range_min ||
    input.ranges.empty())
  {
    result.reason = "scan metadata or sample count is invalid";
    return result;
  }

  // Infinite and NaN ranges have standard LaserScan meaning and are preserved.
  result.accepted = true;
  result.message = input;
  return result;
}

OdomPassThroughResult
validate_and_copy_odom(
  const nav_msgs::msg::Odometry & input,
  const std::string & expected_frame,
  const std::string & expected_child_frame)
{
  OdomPassThroughResult result;
  if (!valid_stamp(input.header.stamp)) {
    result.reason = "odometry stamp is outside the ROS time domain";
    return result;
  }
  if (!valid_frame(input.header.frame_id, expected_frame) ||
    !valid_frame(input.child_frame_id, expected_child_frame))
  {
    result.reason =
      "odometry frame_id or child_frame_id violates the TF contract";
    return result;
  }
  const auto & position = input.pose.pose.position;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(position.z) ||
    !normalized_quaternion(input.pose.pose.orientation) ||
    !finite_vector(input.twist.twist.linear) ||
    !finite_vector(input.twist.twist.angular) ||
    !all_finite(input.pose.covariance) ||
    !all_finite(input.twist.covariance))
  {
    result.reason =
      "odometry contains a non-finite value or non-normalized orientation";
    return result;
  }

  result.accepted = true;
  result.message = input;
  return result;
}

ImuPassThroughResult validate_and_copy_imu(
  const sensor_msgs::msg::Imu & input,
  const std::string & expected_frame)
{
  ImuPassThroughResult result;
  if (!valid_stamp(input.header.stamp)) {
    result.reason = "IMU stamp is outside the ROS time domain";
    return result;
  }
  if (!valid_frame(input.header.frame_id, expected_frame)) {
    result.reason = "IMU frame_id does not match the configured sensor frame";
    return result;
  }

  const bool orientation_unavailable = input.orientation_covariance[0] == -1.0;
  if ((!orientation_unavailable && !normalized_quaternion(input.orientation)) ||
    !finite_vector(input.angular_velocity) ||
    !finite_vector(input.linear_acceleration) ||
    !all_finite(input.orientation_covariance) ||
    !all_finite(input.angular_velocity_covariance) ||
    !all_finite(input.linear_acceleration_covariance))
  {
    result.reason =
      "IMU contains a non-finite value or non-normalized orientation";
    return result;
  }

  result.accepted = true;
  result.message = input;
  return result;
}

geometry_msgs::msg::TransformStamped
make_odom_transform(const nav_msgs::msg::Odometry & validated_odom)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header = validated_odom.header;
  transform.child_frame_id = validated_odom.child_frame_id;
  transform.transform.translation.x = validated_odom.pose.pose.position.x;
  transform.transform.translation.y = validated_odom.pose.pose.position.y;
  transform.transform.translation.z = validated_odom.pose.pose.position.z;
  transform.transform.rotation = validated_odom.pose.pose.orientation;
  return transform;
}

}  // namespace robotest_faults

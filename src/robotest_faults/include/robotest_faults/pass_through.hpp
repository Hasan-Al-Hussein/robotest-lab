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

#ifndef ROBOTEST_FAULTS__PASS_THROUGH_HPP_
#define ROBOTEST_FAULTS__PASS_THROUGH_HPP_

#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robotest_faults
{

template<typename MessageT>
struct PassThroughResult
{
  bool accepted{false};
  MessageT message{};
  std::string reason{};
};

using ScanPassThroughResult = PassThroughResult<sensor_msgs::msg::LaserScan>;
using OdomPassThroughResult = PassThroughResult<nav_msgs::msg::Odometry>;
using ImuPassThroughResult = PassThroughResult<sensor_msgs::msg::Imu>;

ScanPassThroughResult
validate_and_copy_scan(
  const sensor_msgs::msg::LaserScan & input,
  const std::string & expected_frame);

OdomPassThroughResult
validate_and_copy_odom(
  const nav_msgs::msg::Odometry & input,
  const std::string & expected_frame,
  const std::string & expected_child_frame);

ImuPassThroughResult validate_and_copy_imu(
  const sensor_msgs::msg::Imu & input,
  const std::string & expected_frame);

geometry_msgs::msg::TransformStamped
make_odom_transform(const nav_msgs::msg::Odometry & validated_odom);

}  // namespace robotest_faults

#endif  // ROBOTEST_FAULTS__PASS_THROUGH_HPP_

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

#ifndef ROBOTEST_FAULTS__QOS_PROFILES_HPP_
#define ROBOTEST_FAULTS__QOS_PROFILES_HPP_

#include "rclcpp/qos.hpp"

namespace robotest_faults
{

inline rclcpp::QoS scan_sensor_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
}

inline rclcpp::QoS odom_sensor_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
}

inline rclcpp::QoS imu_sensor_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
}

inline rclcpp::QoS fault_event_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile();
}

}  // namespace robotest_faults

#endif  // ROBOTEST_FAULTS__QOS_PROFILES_HPP_

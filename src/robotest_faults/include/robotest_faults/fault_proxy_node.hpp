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

#ifndef ROBOTEST_FAULTS__FAULT_PROXY_NODE_HPP_
#define ROBOTEST_FAULTS__FAULT_PROXY_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robotest_faults/phase1_schedule.hpp"
#include "robotest_interfaces/msg/fault_event.hpp"
#include "robotest_interfaces/srv/load_fault_schedule.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace robotest_faults
{

class FaultProxyNode : public rclcpp::Node {
public:
  explicit FaultProxyNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr message);
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr message);
  void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr message);

  void on_load_schedule(
    const std::shared_ptr<
      robotest_interfaces::srv::LoadFaultSchedule::Request>
    request,
    std::shared_ptr<robotest_interfaces::srv::LoadFaultSchedule::Response>
    response);
  void on_reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void publish_control_event(
    uint8_t event_type,
    const std::string & schedule_hash,
    const std::string & detail);

  const std::string expected_scan_frame_;
  const std::string expected_odom_frame_;
  const std::string expected_base_frame_;
  const std::string expected_imu_frame_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<robotest_interfaces::msg::FaultEvent>::SharedPtr
    event_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

  rclcpp::Service<robotest_interfaces::srv::LoadFaultSchedule>::SharedPtr
    load_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  std::mutex schedule_mutex_;
  Phase1ScheduleState schedule_state_;
};

}  // namespace robotest_faults

#endif  // ROBOTEST_FAULTS__FAULT_PROXY_NODE_HPP_

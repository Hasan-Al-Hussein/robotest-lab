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

#include "robotest_faults/fault_proxy_node.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "robotest_faults/pass_through.hpp"
#include "robotest_faults/qos_profiles.hpp"

namespace robotest_faults
{
namespace
{

void require_relative_frame(
  const std::string & parameter_name,
  const std::string & frame)
{
  if (frame.empty() || frame.front() == '/' ||
    std::any_of(frame.begin(), frame.end(),
    [](const unsigned char character) {
      return std::isspace(character) != 0;
                  }))
  {
    throw std::invalid_argument(parameter_name +
                                " must be a non-empty relative frame ID");
  }
}

}  // namespace

FaultProxyNode::FaultProxyNode(const rclcpp::NodeOptions & options)
: Node("fault_proxy", options),
  expected_scan_frame_(
    declare_parameter<std::string>("expected_scan_frame", "lidar_link")),
  expected_odom_frame_(
    declare_parameter<std::string>("expected_odom_frame", "odom")),
  expected_base_frame_(declare_parameter<std::string>("expected_base_frame",
                                                          "base_footprint")),
  expected_imu_frame_(
    declare_parameter<std::string>("expected_imu_frame", "imu_link"))
{
  require_relative_frame("expected_scan_frame", expected_scan_frame_);
  require_relative_frame("expected_odom_frame", expected_odom_frame_);
  require_relative_frame("expected_base_frame", expected_base_frame_);
  require_relative_frame("expected_imu_frame", expected_imu_frame_);

  scan_publisher_ =
    create_publisher<sensor_msgs::msg::LaserScan>("scan", scan_sensor_qos());
  odom_publisher_ =
    create_publisher<nav_msgs::msg::Odometry>("odom", odom_sensor_qos());
  imu_publisher_ =
    create_publisher<sensor_msgs::msg::Imu>("imu", imu_sensor_qos());
  event_publisher_ = create_publisher<robotest_interfaces::msg::FaultEvent>(
      "faults/events", fault_event_qos());

  scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "raw/scan", scan_sensor_qos(),
      std::bind(&FaultProxyNode::on_scan, this, std::placeholders::_1));
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "raw/odom", odom_sensor_qos(),
      std::bind(&FaultProxyNode::on_odom, this, std::placeholders::_1));
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "raw/imu", imu_sensor_qos(),
      std::bind(&FaultProxyNode::on_imu, this, std::placeholders::_1));

  preload_service_ =
    create_service<robotest_interfaces::srv::PreloadFaultSchedule>(
      "faults/preload_schedule",
      std::bind(
        &FaultProxyNode::on_preload_schedule, this,
        std::placeholders::_1, std::placeholders::_2));
  arm_service_ = create_service<robotest_interfaces::srv::ArmFaultSchedule>(
      "faults/arm_schedule",
      std::bind(
        &FaultProxyNode::on_arm_schedule, this,
        std::placeholders::_1, std::placeholders::_2));
  reset_service_ = create_service<std_srvs::srv::Trigger>(
      "faults/reset", std::bind(&FaultProxyNode::on_reset, this,
                                std::placeholders::_1, std::placeholders::_2));

  transform_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  RCLCPP_INFO(
      get_logger(),
      "Phase 3 fault proxy is active in RESET pass-through state");
}

void FaultProxyNode::on_scan(
  sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
  auto result = protocol_.process_scan(*message, expected_scan_frame_);
  if (!result.accepted) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Rejected raw scan: %s", result.reason.c_str());
    return;
  }
  if (result.publish) {
    scan_publisher_->publish(result.message);
  }
  publish_events(result.events);
}

void FaultProxyNode::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  auto result = protocol_.process_odom(
    *message, expected_odom_frame_, expected_base_frame_);
  if (!result.accepted) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Rejected raw odometry: %s", result.reason.c_str());
    return;
  }

  if (result.publish) {
    // Both outputs derive from this one validated pose and stamp. There is no
    // separate odometry-to-TF transformation state path.
    odom_publisher_->publish(result.message);
    transform_broadcaster_->sendTransform(make_odom_transform(result.message));
  }
  publish_events(result.events);
}

void FaultProxyNode::on_imu(sensor_msgs::msg::Imu::ConstSharedPtr message)
{
  auto result = protocol_.process_imu(*message, expected_imu_frame_);
  if (!result.accepted) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Rejected raw IMU: %s", result.reason.c_str());
    return;
  }
  if (result.publish) {
    imu_publisher_->publish(result.message);
  }
  publish_events(result.events);
}

void FaultProxyNode::on_preload_schedule(
  const std::shared_ptr<
    robotest_interfaces::srv::PreloadFaultSchedule::Request>
  request,
  std::shared_ptr<robotest_interfaces::srv::PreloadFaultSchedule::Response>
  response)
{
  std::lock_guard<std::mutex> transaction_lock(control_transaction_mutex_);
  const builtin_interfaces::msg::Time operation_stamp = now();
  const auto result = protocol_.preload(*request, operation_stamp);

  response->accepted = result.accepted;
  response->replayed = result.replayed;
  response->state = static_cast<std::uint8_t>(result.state);
  response->schedule_hash = result.schedule_hash;
  response->loaded_count = result.loaded_count;
  response->generation = result.generation;
  response->message = result.message;
  event_publisher_->publish(result.event);
}

void FaultProxyNode::on_arm_schedule(
  const std::shared_ptr<robotest_interfaces::srv::ArmFaultSchedule::Request>
  request,
  std::shared_ptr<robotest_interfaces::srv::ArmFaultSchedule::Response>
  response)
{
  std::lock_guard<std::mutex> transaction_lock(control_transaction_mutex_);
  const builtin_interfaces::msg::Time operation_stamp = now();
  const auto result = protocol_.arm(*request, operation_stamp);

  response->accepted = result.accepted;
  response->replayed = result.replayed;
  response->state = static_cast<std::uint8_t>(result.state);
  response->schedule_hash = result.schedule_hash;
  response->generation = result.generation;
  response->goal_uuid = result.goal_uuid;
  response->accepted_goal_stamp = result.accepted_goal_stamp;
  response->arm_commit_stamp = result.arm_commit_stamp;
  response->arm_margin_ns = result.arm_margin_ns;
  response->message = result.message;
  event_publisher_->publish(result.event);
}

void FaultProxyNode::on_reset(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  static_cast<void>(request);
  std::lock_guard<std::mutex> transaction_lock(control_transaction_mutex_);
  const builtin_interfaces::msg::Time operation_stamp = now();
  const auto result = protocol_.reset(operation_stamp);
  response->success = true;
  response->message = result.message;
  event_publisher_->publish(result.event);
}

void FaultProxyNode::publish_events(
  const std::vector<robotest_interfaces::msg::FaultEvent> & events)
{
  for (const auto & event : events) {
    event_publisher_->publish(event);
  }
}

}  // namespace robotest_faults

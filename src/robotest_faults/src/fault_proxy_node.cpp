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

  load_service_ = create_service<robotest_interfaces::srv::LoadFaultSchedule>(
      "faults/load_schedule",
      std::bind(&FaultProxyNode::on_load_schedule, this, std::placeholders::_1,
                std::placeholders::_2));
  reset_service_ = create_service<std_srvs::srv::Trigger>(
      "faults/reset", std::bind(&FaultProxyNode::on_reset, this,
                                std::placeholders::_1, std::placeholders::_2));

  transform_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  RCLCPP_INFO(
      get_logger(),
      "Phase 1 fault proxy is active in deterministic pass-through mode");
}

void FaultProxyNode::on_scan(
  sensor_msgs::msg::LaserScan::ConstSharedPtr message)
{
  auto result = validate_and_copy_scan(*message, expected_scan_frame_);
  if (!result.accepted) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Rejected raw scan: %s", result.reason.c_str());
    return;
  }
  scan_publisher_->publish(result.message);
}

void FaultProxyNode::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  auto result = validate_and_copy_odom(*message, expected_odom_frame_,
                                       expected_base_frame_);
  if (!result.accepted) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Rejected raw odometry: %s", result.reason.c_str());
    return;
  }

  // Both outputs derive from the same validated message and therefore retain
  // exactly the input odometry stamp, frames, pose, twist, and covariance.
  odom_publisher_->publish(result.message);
  transform_broadcaster_->sendTransform(make_odom_transform(result.message));
}

void FaultProxyNode::on_imu(sensor_msgs::msg::Imu::ConstSharedPtr message)
{
  auto result = validate_and_copy_imu(*message, expected_imu_frame_);
  if (!result.accepted) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Rejected raw IMU: %s", result.reason.c_str());
    return;
  }
  imu_publisher_->publish(result.message);
}

void FaultProxyNode::on_load_schedule(
  const std::shared_ptr<robotest_interfaces::srv::LoadFaultSchedule::Request>
  request,
  std::shared_ptr<robotest_interfaces::srv::LoadFaultSchedule::Response>
  response)
{
  ScheduleLoadResult result;
  {
    std::lock_guard<std::mutex> lock(schedule_mutex_);
    result = schedule_state_.validate_and_replace(*request);
  }

  response->accepted = result.accepted;
  response->schedule_hash = request->schedule_hash;
  response->loaded_count = static_cast<uint32_t>(result.loaded_count);
  response->message = result.message;

  const auto event_type =
    result.accepted ?
    robotest_interfaces::msg::FaultEvent::EVENT_SCHEDULE_LOADED :
    robotest_interfaces::msg::FaultEvent::EVENT_SCHEDULE_REJECTED;
  publish_control_event(event_type, request->schedule_hash, result.message);
}

void FaultProxyNode::on_reset(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  static_cast<void>(request);
  {
    std::lock_guard<std::mutex> lock(schedule_mutex_);
    schedule_state_.reset();
  }

  response->success = true;
  response->message =
    "fault state cleared; deterministic pass-through mode active";
  publish_control_event(robotest_interfaces::msg::FaultEvent::EVENT_RESET, "",
                        response->message);
}

void FaultProxyNode::publish_control_event(
  const uint8_t event_type,
  const std::string & schedule_hash,
  const std::string & detail)
{
  robotest_interfaces::msg::FaultEvent event;
  event.header.stamp = now();
  event.schema_version = robotest_interfaces::msg::FaultEvent::SCHEMA_VERSION;
  event.event_type = event_type;
  event.target = robotest_interfaces::msg::FaultSpec::TARGET_UNSPECIFIED;
  event.mode = robotest_interfaces::msg::FaultSpec::MODE_PASS_THROUGH;
  event.configured_time = event.header.stamp;
  event.actual_time = event.header.stamp;
  event.schedule_hash = schedule_hash;
  event.detail = detail;
  event_publisher_->publish(event);
}

}  // namespace robotest_faults

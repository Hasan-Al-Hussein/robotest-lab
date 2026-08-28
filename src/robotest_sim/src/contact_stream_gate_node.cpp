// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

#include <cstdlib>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "robotest_sim/contact_stream_gate.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace robotest_sim
{
namespace
{

#ifndef ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256
#error "contact stream gate source inventory digest was not supplied by CMake"
#endif

constexpr char kSourceInventoryIdentity[] =
  "ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256="
  ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256;
static_assert(sizeof(kSourceInventoryIdentity) == 111U);

}  // namespace

class ContactStreamGateNode final : public rclcpp::Node
{
public:
  ContactStreamGateNode()
  : Node("contact_stream_gate")
  {
    RCLCPP_INFO(get_logger(), "%s", kSourceInventoryIdentity);
    const auto raw_qos = rclcpp::QoS(rclcpp::KeepLast(kRawContactQosDepth))
      .reliable()
      .durability_volatile();
    const auto public_qos = rclcpp::QoS(rclcpp::KeepLast(kPublicContactQosDepth))
      .reliable()
      .durability_volatile();
    publisher_ = create_publisher<ros_gz_interfaces::msg::Contacts>(
      "validation/contacts", public_qos);
    subscription_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
      "internal/raw_contacts",
      raw_qos,
      [this](const ros_gz_interfaces::msg::Contacts::ConstSharedPtr message) {
        handle_raw_contact(*message);
      });
    clock_subscription_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock",
      rclcpp::QoS(rclcpp::KeepLast(1U)).best_effort().durability_volatile(),
      [this](const rosgraph_msgs::msg::Clock::ConstSharedPtr message) {
        if (message->clock.sec < 0 || message->clock.nanosec >= 1000000000U) {
          throw std::runtime_error("contact stream gate rejected invalid /clock stamp");
        }
        const auto stamp_ns = static_cast<std::int64_t>(message->clock.sec) * 1000000000LL +
        static_cast<std::int64_t>(message->clock.nanosec);
        const auto drained_ready_raw_count = drain_ready_raw_history();
        const auto decision = policy_.observe_clock(stamp_ns);
        if (decision.fatal) {
          throw std::runtime_error(
                  "contact stream gate watchdog failed: " + decision.detail +
                  ", drained_ready_raw_count=" + std::to_string(drained_ready_raw_count));
        }
      });
  }

private:
  void handle_raw_contact(const ros_gz_interfaces::msg::Contacts & message)
  {
    auto decision = policy_.observe(message);
    if (decision.output.has_value()) {
      publisher_->publish(std::move(*decision.output));
    }
    if (decision.fatal) {
      throw std::runtime_error("contact stream gate rejected input: " + decision.detail);
    }
  }

  std::size_t drain_ready_raw_history()
  {
    return internal::drain_ready_raw_contacts(
      [this](ros_gz_interfaces::msg::Contacts & message) {
        rclcpp::MessageInfo message_info;
        return subscription_->take(message, message_info);
      },
      [this](const ros_gz_interfaces::msg::Contacts & message) {
        handle_raw_contact(message);
      });
  }

  ContactStreamPolicy policy_;
  rclcpp::Publisher<ros_gz_interfaces::msg::Contacts>::SharedPtr publisher_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr subscription_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
};

}  // namespace robotest_sim

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotest_sim::ContactStreamGateNode>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

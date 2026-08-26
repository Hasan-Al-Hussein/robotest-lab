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

#ifndef ROBOTEST_FAULTS__FAULT_PROTOCOL_HPP_
#define ROBOTEST_FAULTS__FAULT_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robotest_interfaces/msg/fault_event.hpp"
#include "robotest_interfaces/msg/fault_spec.hpp"
#include "robotest_interfaces/srv/arm_fault_schedule.hpp"
#include "robotest_interfaces/srv/preload_fault_schedule.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"

namespace robotest_faults
{

constexpr std::size_t kMaximumFaultCount = 16U;
constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr std::int64_t kMinimumArmingMarginNanoseconds = 500000000LL;
constexpr std::int64_t kMaximumScheduleNanoseconds = 3600000000000LL;
constexpr std::int64_t kMaximumDriftRateScaled = 1000000000LL;

enum class FaultControlState : std::uint8_t
{
  kReset = robotest_interfaces::msg::FaultEvent::STATE_RESET,
  kPrepared = robotest_interfaces::msg::FaultEvent::STATE_PREPARED,
  kArmed = robotest_interfaces::msg::FaultEvent::STATE_ARMED,
};

struct CanonicalFault
{
  robotest_interfaces::msg::FaultSpec specification{};
  std::int64_t start_offset_ns{0};
  std::int64_t duration_ns{0};
  std::int64_t x_rate_nm_per_s{0};
  std::int64_t yaw_rate_nrad_per_s{0};
};

struct CanonicalSchedule
{
  std::vector<CanonicalFault> faults{};
  std::string bytes{};
  std::string sha256{};
  std::optional<std::int64_t> earliest_start_offset_ns{};
};

struct CanonicalScheduleResult
{
  bool accepted{false};
  CanonicalSchedule schedule{};
  std::string message{};
};

std::string sha256_hex(std::string_view bytes);

CanonicalScheduleResult canonicalize_schedule(
  const std::vector<robotest_interfaces::msg::FaultSpec> & faults);

struct PreloadOutcome
{
  bool accepted{false};
  bool replayed{false};
  FaultControlState state{FaultControlState::kReset};
  std::string schedule_hash{};
  std::uint32_t loaded_count{0U};
  std::uint64_t generation{0U};
  std::string message{};
  robotest_interfaces::msg::FaultEvent event{};
};

struct ArmOutcome
{
  bool accepted{false};
  bool replayed{false};
  FaultControlState state{FaultControlState::kReset};
  std::string schedule_hash{};
  std::uint64_t generation{0U};
  unique_identifier_msgs::msg::UUID goal_uuid{};
  builtin_interfaces::msg::Time accepted_goal_stamp{};
  builtin_interfaces::msg::Time arm_commit_stamp{};
  std::int64_t arm_margin_ns{0};
  std::string message{};
  robotest_interfaces::msg::FaultEvent event{};
};

struct ResetOutcome
{
  FaultControlState state_before{FaultControlState::kReset};
  std::string message{};
  robotest_interfaces::msg::FaultEvent event{};
};

template<typename MessageT>
struct FaultProcessOutcome
{
  bool accepted{false};
  bool publish{false};
  MessageT message{};
  std::string reason{};
  std::vector<robotest_interfaces::msg::FaultEvent> events{};
};

using ScanFaultOutcome = FaultProcessOutcome<sensor_msgs::msg::LaserScan>;
using OdomFaultOutcome = FaultProcessOutcome<nav_msgs::msg::Odometry>;
using ImuFaultOutcome = FaultProcessOutcome<sensor_msgs::msg::Imu>;

struct FaultProtocolSnapshot
{
  FaultControlState state{FaultControlState::kReset};
  std::string schedule_hash{};
  std::string canonical_bytes{};
  std::uint32_t fault_count{0U};
  std::uint64_t generation{0U};
  unique_identifier_msgs::msg::UUID goal_uuid{};
  builtin_interfaces::msg::Time accepted_goal_stamp{};
  builtin_interfaces::msg::Time arm_commit_stamp{};
  std::int64_t arm_margin_ns{0};
};

class FaultProtocol
{
public:
  explicit FaultProtocol(
    std::uint64_t last_allocated_generation_for_test = 0U);

  FaultProtocol(const FaultProtocol &) = delete;
  FaultProtocol & operator=(const FaultProtocol &) = delete;

  PreloadOutcome preload(
    const robotest_interfaces::srv::PreloadFaultSchedule::Request & request,
    const builtin_interfaces::msg::Time & operation_stamp);

  ArmOutcome arm(
    const robotest_interfaces::srv::ArmFaultSchedule::Request & request,
    const builtin_interfaces::msg::Time & operation_stamp);

  ResetOutcome reset(const builtin_interfaces::msg::Time & operation_stamp);

  ScanFaultOutcome process_scan(
    const sensor_msgs::msg::LaserScan & input,
    const std::string & expected_frame);

  OdomFaultOutcome process_odom(
    const nav_msgs::msg::Odometry & input,
    const std::string & expected_frame,
    const std::string & expected_child_frame);

  ImuFaultOutcome process_imu(
    const sensor_msgs::msg::Imu & input,
    const std::string & expected_frame);

  FaultProtocolSnapshot snapshot() const;

private:
  struct StreamCounters
  {
    std::uint64_t input_sequence{0U};
    std::uint64_t raw_input_count{0U};
    std::uint64_t validated_output_count{0U};
    std::optional<std::int64_t> last_input_stamp_ns{};
  };

  struct RuntimeFault
  {
    CanonicalFault canonical{};
    bool activated{false};
    bool deactivated{false};
    bool first_restored{false};
    bool restoration_pending{false};
    std::uint64_t affected_message_count{0U};
  };

  std::uint8_t state_wire_value() const noexcept;
  std::uint32_t committed_fault_count() const noexcept;

  robotest_interfaces::msg::FaultEvent make_control_event_locked(
    std::uint8_t event_type,
    const builtin_interfaces::msg::Time & actual_time,
    FaultControlState state_before,
    bool accepted,
    bool replayed,
    const std::string & requested_hash,
    std::uint64_t requested_generation,
    std::uint32_t requested_fault_count,
    const unique_identifier_msgs::msg::UUID & requested_goal_uuid,
    const builtin_interfaces::msg::Time & requested_t0,
    const std::string & detail);

  robotest_interfaces::msg::FaultEvent make_data_event_locked(
    std::uint8_t event_type,
    const builtin_interfaces::msg::Time & actual_time,
    const RuntimeFault & fault,
    const StreamCounters & counters,
    const std::string & detail);

  std::vector<robotest_interfaces::msg::FaultEvent>
  begin_target_input_locked(
    std::uint8_t target,
    const builtin_interfaces::msg::Time & stamp,
    StreamCounters & counters,
    RuntimeFault *& active_fault,
    std::string & rejection);

  void append_restoration_events_locked(
    std::uint8_t target,
    const builtin_interfaces::msg::Time & stamp,
    const StreamCounters & counters,
    std::vector<robotest_interfaces::msg::FaultEvent> & events);

  void reset_runtime_locked();

  mutable std::mutex mutex_;
  FaultControlState state_{FaultControlState::kReset};
  std::uint64_t last_allocated_generation_{0U};
  std::uint64_t generation_{0U};
  std::uint64_t event_sequence_{0U};
  CanonicalSchedule schedule_{};
  std::vector<RuntimeFault> runtime_faults_{};
  unique_identifier_msgs::msg::UUID bound_goal_uuid_{};
  builtin_interfaces::msg::Time accepted_goal_stamp_{};
  builtin_interfaces::msg::Time arm_commit_stamp_{};
  std::int64_t arm_margin_ns_{0};
  std::array<StreamCounters, 4U> stream_counters_{};
};

}  // namespace robotest_faults

#endif  // ROBOTEST_FAULTS__FAULT_PROTOCOL_HPP_

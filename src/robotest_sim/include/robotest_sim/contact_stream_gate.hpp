// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

#ifndef ROBOTEST_SIM__CONTACT_STREAM_GATE_HPP_
#define ROBOTEST_SIM__CONTACT_STREAM_GATE_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ros_gz_interfaces/msg/contact.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "std_msgs/msg/header.hpp"

namespace robotest_sim
{

inline constexpr std::int64_t kContactHeartbeatPeriodNs = 200000000;
inline constexpr std::int64_t kPrivateContactAggregatePeriodNs = 20000000;
inline constexpr std::int64_t kContactReleaseGapNs = 250000000;
inline constexpr std::int64_t kMaxPublicSnapshotGapNs = 220000000;
inline constexpr std::int64_t kMaxPendingBatchClockLagNs = 220000000;
inline constexpr std::int64_t kMaxPublicSnapshotClockLagNs = 220000000;
inline constexpr std::int64_t kMaxRawClockLagNs = 220000000;
inline constexpr std::size_t kRawContactQosDepth = 64U;
inline constexpr std::size_t kPublicContactQosDepth = 10U;
inline constexpr std::size_t kMaxActiveContactPairs = 16U;
inline constexpr std::size_t kMaxRawContactRecords = 16U;
inline constexpr std::size_t kMaxRawMessagesPerBatch = 1U;
inline constexpr std::size_t kMaxActiveContactRecords = 16U;
inline constexpr std::size_t kMaxContactRecordsPerPair = 4U;
inline constexpr std::size_t kMaxContactPointsPerRecord = 64U;
inline constexpr std::size_t kMaxCollisionNameBytes = 4096U;
inline constexpr std::size_t kMaxBodyNameBytes = 4096U;
inline constexpr std::size_t kMaxFrameIdBytes = 256U;
inline constexpr std::size_t kMaxContactStringBytes = 8192U;
inline constexpr std::size_t kMaxMessageStringBytes = 65536U;
inline constexpr std::size_t kMaxActiveStringBytes = 65536U;

enum class ContactForwardReason
{
  kSuppressed,
  kInitialSnapshot,
  kPairSetTransition,
  kSteadyStateHeartbeat,
  kFatalSemanticInput,
  kFatalStructuralInput,
};

struct ContactGateDecision
{
  std::optional<ros_gz_interfaces::msg::Contacts> output;
  bool fatal{false};
  ContactForwardReason reason{ContactForwardReason::kSuppressed};
  std::size_t active_pair_count{0U};
  std::size_t active_record_count{0U};
  std::string detail;
};

namespace internal
{

// A SingleThreadedExecutor reports readiness per subscription, not per queued
// sample. Process the bounded private history before an independently queued
// /clock callback evaluates source liveness.
template<typename TakeReady, typename HandleReady>
std::size_t drain_ready_raw_contacts(
  TakeReady && take_ready,
  HandleReady && handle_ready)
{
  std::size_t drained_count = 0U;
  while (drained_count < kRawContactQosDepth) {
    ros_gz_interfaces::msg::Contacts message;
    if (!take_ready(message)) {
      break;
    }
    handle_ready(message);
    ++drained_count;
  }
  return drained_count;
}

}  // namespace internal

class ContactStreamPolicy
{
public:
  ContactGateDecision observe(const ros_gz_interfaces::msg::Contacts & message);
  ContactGateDecision observe_clock(std::int64_t clock_stamp_ns);

private:
  using ContactPair = std::pair<std::string, std::string>;

  struct ContactRecords
  {
    std::size_t string_bytes{0U};
    std::vector<ros_gz_interfaces::msg::Contact> records;
  };

  struct ActivePair : ContactRecords
  {
    std::int64_t last_seen_stamp_ns{0};
  };

  struct PendingBatch
  {
    std_msgs::msg::Header header;
    std::map<ContactPair, ContactRecords> pairs;
    std::size_t message_count{0U};
    std::size_t record_count{0U};
    std::size_t string_bytes{0U};
    bool semantic_fatal{false};
    std::string fatal_detail;
  };

  ContactGateDecision finalize_pending();
  ros_gz_interfaces::msg::Contacts snapshot(
    const std_msgs::msg::Header & completed_header) const;

  std::map<ContactPair, ActivePair> active_pairs_;
  std::optional<PendingBatch> pending_batch_;
  std::optional<std::int64_t> last_raw_stamp_ns_;
  std::optional<std::int64_t> last_forward_stamp_ns_;
  std::optional<std::int64_t> last_clock_stamp_ns_;
  bool synchronized_{false};
};

}  // namespace robotest_sim

#endif  // ROBOTEST_SIM__CONTACT_STREAM_GATE_HPP_

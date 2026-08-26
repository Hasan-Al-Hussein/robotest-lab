// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

#include "robotest_sim/contact_stream_gate.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "ros_gz_interfaces/msg/contact.hpp"

namespace robotest_sim
{
namespace
{

using Contact = ros_gz_interfaces::msg::Contact;
using ContactPair = std::pair<std::string, std::string>;

constexpr std::array<std::string_view, 7> kRobotCollisions = {
  "robotest::base_footprint::base_footprint_fixed_joint_lump__base_link_collision_collision",
  "robotest::front_caster_link::"
  "front_caster_link_fixed_joint_lump__front_caster_collision_collision",
  "robotest::imu_link::imu_link_collision_collision",
  "robotest::left_wheel_link::"
  "left_wheel_link_fixed_joint_lump__left_wheel_collision_collision",
  "robotest::lidar_link::lidar_link_collision_collision",
  "robotest::rear_caster_link::"
  "rear_caster_link_fixed_joint_lump__rear_caster_collision_collision",
  "robotest::right_wheel_link::"
  "right_wheel_link_fixed_joint_lump__right_wheel_collision_collision",
};

constexpr std::string_view kGroundCollision =
  "ground_plane::ground_link::ground_collision";

constexpr std::array<std::string_view, 4> kSupportCollisions = {
  "robotest::front_caster_link::"
  "front_caster_link_fixed_joint_lump__front_caster_collision_collision",
  "robotest::left_wheel_link::"
  "left_wheel_link_fixed_joint_lump__left_wheel_collision_collision",
  "robotest::rear_caster_link::"
  "rear_caster_link_fixed_joint_lump__rear_caster_collision_collision",
  "robotest::right_wheel_link::"
  "right_wheel_link_fixed_joint_lump__right_wheel_collision_collision",
};

enum class PairDisposition
{
  kRoutine,
  kCountable,
  kUnexpectedNonRobot,
  kUnknownRobot,
  kMalformed,
};

struct ValidatedMessage
{
  struct Records
  {
    std::size_t string_bytes{0U};
    std::vector<Contact> records;
  };

  std::map<ContactPair, Records> pairs;
  std::size_t record_count{0U};
  std::size_t string_bytes{0U};
  bool semantic_fatal{false};
  std::string fatal_detail;
};

bool checked_add(const std::size_t value, std::size_t & total, const std::size_t maximum)
{
  if (value > maximum || total > maximum - value) {
    return false;
  }
  total += value;
  return true;
}

bool valid_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U;
}

std::optional<std::int64_t> stamp_ns(const std_msgs::msg::Header & header)
{
  if (!valid_stamp(header.stamp)) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(header.stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(header.stamp.nanosec);
}

bool is_known_robot_collision(const std::string_view name)
{
  return std::find(kRobotCollisions.begin(), kRobotCollisions.end(), name) !=
         kRobotCollisions.end();
}

bool is_robot_model_name(const std::string_view name)
{
  const auto separator = name.find("::");
  return separator != std::string_view::npos && name.substr(0U, separator) == "robotest";
}

bool is_valid_scoped_name(const std::string_view name)
{
  if (name.empty()) {
    return false;
  }
  std::size_t segment_count = 0U;
  std::size_t start = 0U;
  while (true) {
    const auto separator = name.find("::", start);
    const auto end = separator == std::string_view::npos ? name.size() : separator;
    if (end == start) {
      return false;
    }
    ++segment_count;
    if (separator == std::string_view::npos) {
      break;
    }
    start = separator + 2U;
    if (start >= name.size()) {
      return false;
    }
  }
  return segment_count >= 3U;
}

bool is_support_pair(const std::string_view first, const std::string_view second)
{
  const std::string_view robot = first == kGroundCollision ? second : first;
  const std::string_view counterpart = first == kGroundCollision ? first : second;
  return counterpart == kGroundCollision &&
         std::find(kSupportCollisions.begin(), kSupportCollisions.end(), robot) !=
         kSupportCollisions.end();
}

PairDisposition classify_pair(const std::string_view first, const std::string_view second)
{
  if (!is_valid_scoped_name(first) || !is_valid_scoped_name(second)) {
    return PairDisposition::kMalformed;
  }
  const bool first_known_robot = is_known_robot_collision(first);
  const bool second_known_robot = is_known_robot_collision(second);
  if ((is_robot_model_name(first) && !first_known_robot) ||
    (is_robot_model_name(second) && !second_known_robot))
  {
    return PairDisposition::kUnknownRobot;
  }
  if (first_known_robot && second_known_robot) {
    return PairDisposition::kRoutine;
  }
  if (is_support_pair(first, second)) {
    return PairDisposition::kRoutine;
  }
  if (!first_known_robot && !second_known_robot) {
    return PairDisposition::kUnexpectedNonRobot;
  }
  return PairDisposition::kCountable;
}

std::string semantic_detail(const PairDisposition disposition)
{
  switch (disposition) {
    case PairDisposition::kUnexpectedNonRobot:
      return "unexpected non-robot contact pair";
    case PairDisposition::kUnknownRobot:
      return "contact references an unknown robotest collision";
    case PairDisposition::kMalformed:
      return "contact contains a malformed scoped collision name";
    case PairDisposition::kRoutine:
    case PairDisposition::kCountable:
      return {};
  }
  return "unreachable contact classification";
}

ContactPair normalized_pair(const std::string & first, const std::string & second)
{
  return second < first ? ContactPair{second, first} : ContactPair{first, second};
}

std::size_t pair_key_string_bytes(const ContactPair & pair)
{
  return pair.first.size() + pair.second.size();
}

bool finite_vector(const geometry_msgs::msg::Vector3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool finite_wrench(const geometry_msgs::msg::Wrench & value)
{
  return finite_vector(value.force) && finite_vector(value.torque);
}

std::optional<std::string> validate_contact(
  const Contact & contact,
  std::size_t & message_string_bytes,
  std::size_t & contact_string_bytes)
{
  const auto & first = contact.collision1.name;
  const auto & second = contact.collision2.name;
  if (first.size() > kMaxCollisionNameBytes || second.size() > kMaxCollisionNameBytes) {
    return "collision name exceeds the 4096-byte bound";
  }
  for (const auto size : {first.size(), second.size()}) {
    if (!checked_add(size, contact_string_bytes, kMaxContactStringBytes) ||
      !checked_add(size, message_string_bytes, kMaxMessageStringBytes))
    {
      return "contact or raw-batch string budget exceeded";
    }
  }

  const auto position_count = contact.positions.size();
  const auto detail_count = contact.normals.size();
  if (position_count == 0U || position_count > kMaxContactPointsPerRecord ||
    detail_count > position_count ||
    contact.depths.size() != detail_count || contact.wrenches.size() != detail_count)
  {
    return "contact arrays violate the bounded Gazebo alignment contract";
  }
  if (!std::all_of(contact.positions.begin(), contact.positions.end(), finite_vector)) {
    return "contact position contains a non-finite value";
  }
  for (std::size_t index = 0U; index < detail_count; ++index) {
    if (!finite_vector(contact.normals[index]) ||
      !std::isfinite(contact.depths[index]) || contact.depths[index] < 0.0)
    {
      return "contact point contains a non-finite value or negative depth";
    }
    const auto & wrench = contact.wrenches[index];
    if (!valid_stamp(wrench.header.stamp) ||
      wrench.header.frame_id.size() > kMaxFrameIdBytes ||
      wrench.body_1_name.data.size() > kMaxBodyNameBytes ||
      wrench.body_2_name.data.size() > kMaxBodyNameBytes ||
      !finite_wrench(wrench.body_1_wrench) || !finite_wrench(wrench.body_2_wrench))
    {
      return "joint wrench violates its stamp, frame, body-name, or finite-value bound";
    }
    for (const auto size : {
          wrench.header.frame_id.size(),
          wrench.body_1_name.data.size(),
          wrench.body_2_name.data.size()})
    {
      if (!checked_add(size, contact_string_bytes, kMaxContactStringBytes) ||
        !checked_add(size, message_string_bytes, kMaxMessageStringBytes))
      {
        return "nested joint-wrench strings exceed the bounded byte budget";
      }
    }
  }
  return std::nullopt;
}

std::optional<std::string> validate_message(
  const ros_gz_interfaces::msg::Contacts & message,
  ValidatedMessage & result)
{
  if (message.contacts.empty()) {
    return "raw Gazebo contact message cannot be empty";
  }
  if (message.contacts.size() > kMaxRawContactRecords) {
    return "one raw contact message exceeds the 16-record bound";
  }
  result.record_count = message.contacts.size();
  result.string_bytes = message.header.frame_id.size();
  for (const auto & contact : message.contacts) {
    std::size_t contact_string_bytes = 0U;
    if (const auto error = validate_contact(
        contact, result.string_bytes, contact_string_bytes))
    {
      return error;
    }
    const auto pair = normalized_pair(contact.collision1.name, contact.collision2.name);
    const auto [group_iterator, inserted] = result.pairs.try_emplace(pair);
    auto & group = group_iterator->second;
    if (inserted && !checked_add(
        pair_key_string_bytes(pair), result.string_bytes, kMaxMessageStringBytes))
    {
      return "raw message strings plus normalized pair keys exceed the byte budget";
    }
    if (group.records.size() >= kMaxContactRecordsPerPair) {
      return "one raw contact pair exceeds the four-record duplicate bound";
    }
    if (!checked_add(
        contact_string_bytes, group.string_bytes, kMaxContactStringBytes))
    {
      return "one raw contact pair exceeds the contact string budget";
    }
    group.records.push_back(contact);
    const auto detail = semantic_detail(
      classify_pair(contact.collision1.name, contact.collision2.name));
    if (!detail.empty()) {
      result.semantic_fatal = true;
      if (result.fatal_detail.empty()) {
        result.fatal_detail = detail;
      }
    }
  }
  return std::nullopt;
}

}  // namespace

ros_gz_interfaces::msg::Contacts ContactStreamPolicy::snapshot(
  const std_msgs::msg::Header & completed_header) const
{
  ros_gz_interfaces::msg::Contacts result;
  result.header = completed_header;
  std::size_t record_count = 0U;
  for (const auto & entry : active_pairs_) {
    record_count += entry.second.records.size();
  }
  result.contacts.reserve(record_count);
  for (const auto & entry : active_pairs_) {
    result.contacts.insert(
      result.contacts.end(), entry.second.records.begin(), entry.second.records.end());
  }
  return result;
}

ContactGateDecision ContactStreamPolicy::finalize_pending()
{
  ContactGateDecision decision;
  if (!pending_batch_.has_value()) {
    return decision;
  }
  auto batch = std::move(*pending_batch_);
  pending_batch_.reset();
  const auto completed_stamp = stamp_ns(batch.header);
  if (!completed_stamp.has_value()) {
    decision.fatal = true;
    decision.reason = ContactForwardReason::kFatalStructuralInput;
    decision.detail = "pending raw contact batch lost its valid simulation stamp";
    return decision;
  }
  if (batch.semantic_fatal && !synchronized_) {
    decision.fatal = true;
    decision.reason = ContactForwardReason::kFatalSemanticInput;
    decision.detail = batch.fatal_detail;
    return decision;
  }
  if (!synchronized_) {
    synchronized_ = true;
    return decision;
  }

  const auto structural_fatal = [&decision](const std::string & detail) {
      decision.fatal = true;
      decision.reason = ContactForwardReason::kFatalStructuralInput;
      decision.detail = detail;
      return decision;
    };
  if (last_forward_stamp_ns_.has_value() &&
    *completed_stamp - *last_forward_stamp_ns_ > kMaxPublicSnapshotGapNs)
  {
    const auto gap_ns = *completed_stamp - *last_forward_stamp_ns_;
    return structural_fatal(
      "completed raw contact stream advanced more than 220 ms beyond the last "
      "public snapshot: last_public_stamp_ns=" + std::to_string(*last_forward_stamp_ns_) +
      ", completed_stamp_ns=" + std::to_string(*completed_stamp) +
      ", gap_ns=" + std::to_string(gap_ns) +
      ", completed_batch_message_count=" + std::to_string(batch.message_count));
  }

  std::set<ContactPair> expired_pairs;
  std::size_t projected_pair_count = active_pairs_.size();
  std::size_t projected_record_count = 0U;
  std::size_t projected_string_bytes = 0U;
  for (const auto & entry : active_pairs_) {
    const bool absent = batch.pairs.find(entry.first) == batch.pairs.end();
    const bool release_proven = absent &&
      *completed_stamp - entry.second.last_seen_stamp_ns > kContactReleaseGapNs;
    if (release_proven) {
      expired_pairs.insert(entry.first);
      --projected_pair_count;
      continue;
    }
    projected_record_count += entry.second.records.size();
    projected_string_bytes += entry.second.string_bytes + pair_key_string_bytes(entry.first);
  }

  bool pair_set_changed = !expired_pairs.empty();
  for (const auto & entry : batch.pairs) {
    const auto active = active_pairs_.find(entry.first);
    const bool retained = active != active_pairs_.end() &&
      expired_pairs.find(entry.first) == expired_pairs.end();
    if (retained) {
      projected_record_count -= active->second.records.size();
      projected_string_bytes -=
        active->second.string_bytes + pair_key_string_bytes(active->first);
    } else {
      ++projected_pair_count;
      pair_set_changed = true;
    }
    projected_record_count += entry.second.records.size();
    projected_string_bytes += entry.second.string_bytes + pair_key_string_bytes(entry.first);
  }

  if (projected_pair_count > kMaxActiveContactPairs) {
    return structural_fatal("active contact state exceeds the 16-pair bound");
  }
  if (projected_record_count > kMaxActiveContactRecords) {
    return structural_fatal("active contact state exceeds the 16-record bound");
  }
  if (projected_string_bytes > kMaxActiveStringBytes) {
    return structural_fatal("active contact state exceeds the 65536-byte string budget");
  }

  for (const auto & pair : expired_pairs) {
    active_pairs_.erase(pair);
  }
  for (auto & entry : batch.pairs) {
    ActivePair state;
    state.last_seen_stamp_ns = *completed_stamp;
    state.string_bytes = entry.second.string_bytes;
    state.records = std::move(entry.second.records);
    active_pairs_.insert_or_assign(entry.first, std::move(state));
  }

  decision.active_pair_count = active_pairs_.size();
  decision.active_record_count = projected_record_count;
  if (batch.semantic_fatal) {
    decision.output = snapshot(batch.header);
    decision.fatal = true;
    decision.reason = ContactForwardReason::kFatalSemanticInput;
    decision.detail = batch.fatal_detail;
  } else if (!last_forward_stamp_ns_.has_value()) {
    decision.output = snapshot(batch.header);
    decision.reason = ContactForwardReason::kInitialSnapshot;
  } else if (pair_set_changed) {
    decision.output = snapshot(batch.header);
    decision.reason = ContactForwardReason::kPairSetTransition;
  } else if (*completed_stamp - *last_forward_stamp_ns_ >= kContactHeartbeatPeriodNs) {
    decision.output = snapshot(batch.header);
    decision.reason = ContactForwardReason::kSteadyStateHeartbeat;
  }
  if (decision.output.has_value()) {
    if (decision.output->contacts.empty() ||
      decision.output->contacts.size() > kMaxActiveContactRecords)
    {
      decision.output.reset();
      decision.fatal = true;
      decision.reason = ContactForwardReason::kFatalStructuralInput;
      decision.detail = "public contact snapshot must contain between one and 16 records";
      return decision;
    }
    last_forward_stamp_ns_ = *completed_stamp;
  }
  return decision;
}

ContactGateDecision ContactStreamPolicy::observe_clock(const std::int64_t clock_stamp_ns)
{
  ContactGateDecision decision;
  const auto fatal = [&decision](const std::string & detail) {
      decision.fatal = true;
      decision.reason = ContactForwardReason::kFatalStructuralInput;
      decision.detail = detail;
      return decision;
    };
  if (clock_stamp_ns < 0) {
    return fatal("simulation clock stamp is negative");
  }
  if (last_clock_stamp_ns_.has_value() && clock_stamp_ns < *last_clock_stamp_ns_) {
    return fatal("simulation clock stamp regressed");
  }
  last_clock_stamp_ns_ = clock_stamp_ns;
  if (pending_batch_.has_value()) {
    const auto pending_stamp = stamp_ns(pending_batch_->header);
    if (!pending_stamp.has_value() ||
      clock_stamp_ns - *pending_stamp > kMaxPendingBatchClockLagNs)
    {
      if (pending_batch_->semantic_fatal) {
        return fatal("pending semantic contact failure did not reach a closing raw stamp");
      }
      return fatal("pending raw contact batch did not close within 220 ms of /clock");
    }
  }
  if (last_raw_stamp_ns_.has_value() &&
    clock_stamp_ns - *last_raw_stamp_ns_ > kMaxRawClockLagNs)
  {
    return fatal("raw contact stream is more than 220 ms behind simulation clock");
  }
  if (!synchronized_) {
    return decision;
  }
  // Do not compare /clock with the last public snapshot here. The clock and
  // private raw subscriptions have no causal callback order, so /clock can run
  // just before a queued raw callback closes a timely heartbeat. The finalized
  // raw-stamp invariant in finalize_pending() owns public source cadence, while
  // the pending/raw checks above own source silence.
  return decision;
}

ContactGateDecision ContactStreamPolicy::observe(
  const ros_gz_interfaces::msg::Contacts & message)
{
  ContactGateDecision decision;
  const auto incoming_stamp = stamp_ns(message.header);
  const auto fatal_current = [&decision](const std::string & detail) {
      decision.fatal = true;
      decision.reason = ContactForwardReason::kFatalStructuralInput;
      decision.detail = detail;
      return decision;
    };

  if (!incoming_stamp.has_value()) {
    return fatal_current("raw contact message has an invalid simulation stamp");
  }
  if (last_raw_stamp_ns_.has_value() && *incoming_stamp < *last_raw_stamp_ns_) {
    return fatal_current("raw contact simulation stamp regressed");
  }
  if (!message.header.frame_id.empty()) {
    return fatal_current("raw Gazebo contact frame_id must be empty");
  }

  ValidatedMessage validated;
  if (const auto error = validate_message(message, validated)) {
    return fatal_current(*error);
  }

  if (pending_batch_.has_value() && *incoming_stamp > *last_raw_stamp_ns_) {
    decision = finalize_pending();
    if (decision.fatal) {
      return decision;
    }
  }
  if (!pending_batch_.has_value()) {
    PendingBatch batch;
    batch.header = message.header;
    for (auto & entry : validated.pairs) {
      ContactRecords records;
      records.string_bytes = entry.second.string_bytes;
      records.records = std::move(entry.second.records);
      batch.pairs.emplace(entry.first, std::move(records));
    }
    batch.record_count = validated.record_count;
    batch.message_count = 1U;
    batch.string_bytes = validated.string_bytes;
    batch.semantic_fatal = validated.semantic_fatal;
    batch.fatal_detail = std::move(validated.fatal_detail);
    pending_batch_ = std::move(batch);
  } else {
    auto & batch = *pending_batch_;
    if (batch.message_count >= kMaxRawMessagesPerBatch) {
      decision.fatal = true;
      decision.reason = ContactForwardReason::kFatalStructuralInput;
      decision.detail = "equal-stamp raw contact batch exceeds the seven-sensor bound";
      return decision;
    }
    ++batch.message_count;
    if (batch.record_count + validated.record_count > kMaxRawContactRecords) {
      decision.fatal = true;
      decision.reason = ContactForwardReason::kFatalStructuralInput;
      decision.detail = "equal-stamp raw contact batch exceeds the 16-record bound";
      return decision;
    }
    std::size_t incremental_string_bytes = 0U;
    for (const auto & entry : validated.pairs) {
      if (!checked_add(
          entry.second.string_bytes, incremental_string_bytes, kMaxMessageStringBytes) ||
        (batch.pairs.find(entry.first) == batch.pairs.end() &&
        !checked_add(
          pair_key_string_bytes(entry.first),
          incremental_string_bytes,
          kMaxMessageStringBytes)))
      {
        decision.fatal = true;
        decision.reason = ContactForwardReason::kFatalStructuralInput;
        decision.detail =
          "equal-stamp raw contact batch strings plus normalized pair keys exceed the budget";
        return decision;
      }
    }
    if (!checked_add(
        incremental_string_bytes, batch.string_bytes, kMaxMessageStringBytes))
    {
      decision.fatal = true;
      decision.reason = ContactForwardReason::kFatalStructuralInput;
      decision.detail =
        "equal-stamp raw contact batch strings plus normalized pair keys exceed the budget";
      return decision;
    }
    batch.record_count += validated.record_count;
    for (auto & entry : validated.pairs) {
      auto & group = batch.pairs[entry.first];
      if (group.records.size() + entry.second.records.size() >
        kMaxContactRecordsPerPair)
      {
        decision.fatal = true;
        decision.reason = ContactForwardReason::kFatalStructuralInput;
        decision.detail = "equal-stamp pair exceeds the four-record duplicate bound";
        return decision;
      }
      if (!checked_add(
          entry.second.string_bytes, group.string_bytes, kMaxContactStringBytes))
      {
        decision.fatal = true;
        decision.reason = ContactForwardReason::kFatalStructuralInput;
        decision.detail = "equal-stamp pair exceeds the contact string budget";
        return decision;
      }
      group.records.insert(
        group.records.end(),
        std::make_move_iterator(entry.second.records.begin()),
        std::make_move_iterator(entry.second.records.end()));
    }
    batch.semantic_fatal = batch.semantic_fatal || validated.semantic_fatal;
    if (batch.fatal_detail.empty()) {
      batch.fatal_detail = std::move(validated.fatal_detail);
    }
  }

  last_raw_stamp_ns_ = *incoming_stamp;
  return decision;
}

}  // namespace robotest_sim

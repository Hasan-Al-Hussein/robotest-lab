// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

#include "robotest_sim/contact_aggregator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace robotest_sim
{
namespace
{

constexpr std::size_t kMaxAggregateRecords = 16U;
constexpr std::size_t kMaxRecordsPerPair = 4U;
constexpr std::size_t kMaxContactPointsPerRecord = 64U;
constexpr std::size_t kMaxCollisionNameBytes = 4096U;
constexpr std::size_t kMaxBodyNameBytes = 4096U;
constexpr std::size_t kMaxFrameIdBytes = 256U;
constexpr std::size_t kMaxPairStringBytes = 8192U;
constexpr std::size_t kMaxAggregateStringBytes = 65536U;

bool checked_add(
  const std::size_t value, std::size_t & total,
  const std::size_t maximum)
{
  if (value > maximum || total > maximum - value) {
    return false;
  }
  total += value;
  return true;
}

bool valid_stamp(const gz::msgs::Time & stamp)
{
  return stamp.sec() >= 0 && stamp.nsec() >= 0 && stamp.nsec() < 1000000000;
}

bool finite_vector(const gz::msgs::Vector3d & value)
{
  return std::isfinite(value.x()) && std::isfinite(value.y()) &&
         std::isfinite(value.z());
}

bool finite_wrench(const gz::msgs::Wrench & value)
{
  return finite_vector(value.force()) && finite_vector(value.torque());
}

struct RetainedWrenchHeaderStrings
{
  std::size_t frame_id_bytes;
  std::size_t total_bytes;
};

std::optional<RetainedWrenchHeaderStrings>
retained_wrench_header_strings(const gz::msgs::Header & header)
{
  if (header.data_size() == 0) {
    return RetainedWrenchHeaderStrings{0U, 0U};
  }
  if (header.data_size() != 1) {
    return std::nullopt;
  }
  const auto & datum = header.data(0);
  if (datum.key() != "frame_id" || datum.value_size() != 1) {
    return std::nullopt;
  }
  return RetainedWrenchHeaderStrings{
    datum.value(0).size(), datum.key().size() + datum.value(0).size()};
}

void copy_vector_projection(
  const gz::msgs::Vector3d & source, gz::msgs::Vector3d & target)
{
  target.set_x(source.x());
  target.set_y(source.y());
  target.set_z(source.z());
}

void copy_entity_projection(
  const gz::msgs::Entity & source, gz::msgs::Entity & target)
{
  target.set_id(source.id());
  target.set_name(source.name());
  target.set_type(source.type());
}

void copy_wrench_projection(
  const gz::msgs::Wrench & source, gz::msgs::Wrench & target)
{
  copy_vector_projection(source.force(), *target.mutable_force());
  copy_vector_projection(source.torque(), *target.mutable_torque());
}

void copy_joint_wrench_projection(
  const gz::msgs::JointWrench & source, gz::msgs::JointWrench & target)
{
  auto *target_stamp = target.mutable_header()->mutable_stamp();
  target_stamp->set_sec(source.header().stamp().sec());
  target_stamp->set_nsec(source.header().stamp().nsec());
  if (source.header().data_size() == 1) {
    const auto & source_datum = source.header().data(0);
    auto *target_datum = target.mutable_header()->add_data();
    target_datum->set_key("frame_id");
    target_datum->add_value(source_datum.value(0));
  }
  target.set_body_1_name(source.body_1_name());
  target.set_body_1_id(source.body_1_id());
  target.set_body_2_name(source.body_2_name());
  target.set_body_2_id(source.body_2_id());
  copy_wrench_projection(source.body_1_wrench(),
                         *target.mutable_body_1_wrench());
  copy_wrench_projection(source.body_2_wrench(),
                         *target.mutable_body_2_wrench());
}

gz::msgs::Contact ros_projected_contact(const gz::msgs::Contact & source)
{
  gz::msgs::Contact target;
  copy_entity_projection(source.collision1(), *target.mutable_collision1());
  copy_entity_projection(source.collision2(), *target.mutable_collision2());
  for (const auto & position : source.position()) {
    copy_vector_projection(position, *target.add_position());
  }
  for (const auto & normal : source.normal()) {
    copy_vector_projection(normal, *target.add_normal());
  }
  for (const auto depth : source.depth()) {
    target.add_depth(depth);
  }
  for (const auto & wrench : source.wrench()) {
    copy_joint_wrench_projection(wrench, *target.add_wrench());
  }
  return target;
}

std::optional<std::string> validate_contact(
  const gz::msgs::Contact & contact,
  std::size_t & contact_string_bytes)
{
  const auto & first = contact.collision1().name();
  const auto & second = contact.collision2().name();
  if (first.empty() || second.empty()) {
    return "contact aggregate collision names must be nonempty";
  }
  if (first.size() > kMaxCollisionNameBytes ||
    second.size() > kMaxCollisionNameBytes)
  {
    return "contact aggregate collision name exceeds the 4096-byte bound";
  }
  for (const auto size : {first.size(), second.size()}) {
    if (!checked_add(size, contact_string_bytes, kMaxPairStringBytes)) {
      return "contact aggregate pair exceeds the 8192-byte string bound";
    }
  }

  const auto position_count = contact.position_size();
  const auto detail_count = contact.normal_size();
  if (position_count == 0 ||
    position_count > static_cast<int>(kMaxContactPointsPerRecord) ||
    detail_count > position_count || contact.depth_size() != detail_count ||
    contact.wrench_size() != detail_count)
  {
    return "contact aggregate arrays violate the bounded Gazebo alignment "
           "contract";
  }
  for (const auto & position : contact.position()) {
    if (!finite_vector(position)) {
      return "contact aggregate position contains a non-finite value";
    }
  }
  for (int index = 0; index < detail_count; ++index) {
    if (!finite_vector(contact.normal(index)) ||
      !std::isfinite(contact.depth(index)) || contact.depth(index) < 0.0)
    {
      return "contact aggregate detail contains a non-finite value or negative "
             "depth";
    }
    const auto & wrench = contact.wrench(index);
    const auto header_strings = retained_wrench_header_strings(wrench.header());
    if (!header_strings.has_value()) {
      return "contact aggregate wrench header violates the frame_id-only data "
             "grammar";
    }
    if (!valid_stamp(wrench.header().stamp()) ||
      header_strings->frame_id_bytes > kMaxFrameIdBytes ||
      wrench.body_1_name().size() > kMaxBodyNameBytes ||
      wrench.body_2_name().size() > kMaxBodyNameBytes ||
      !finite_wrench(wrench.body_1_wrench()) ||
      !finite_wrench(wrench.body_2_wrench()))
    {
      return "contact aggregate wrench violates its stamp, frame, body-name, "
             "or finite-value bound";
    }
    for (const auto size : {header_strings->total_bytes,
        wrench.body_1_name().size(),
        wrench.body_2_name().size()})
    {
      if (!checked_add(size, contact_string_bytes, kMaxPairStringBytes)) {
        return "contact aggregate pair exceeds the 8192-byte string bound";
      }
    }
  }
  return std::nullopt;
}

void stamp_header(gz::msgs::Header & header, const std::int64_t stamp_ns)
{
  auto *stamp = header.mutable_stamp();
  stamp->set_sec(stamp_ns / 1000000000LL);
  stamp->set_nsec(static_cast<std::int32_t>(stamp_ns % 1000000000LL));
  header.clear_data();
}

}  // namespace

std::optional<std::string> ContactAggregatorPolicy::validated_step_groups(
  const gz::msgs::Contacts * const * contacts,
  const std::size_t source_count,
  ContactGroups & groups)
{
  if (contacts == nullptr || source_count == 0U ||
    source_count > kContactAggregateSourceCount)
  {
    return "contact aggregate source set must contain one to seven messages";
  }

  // Bound the complete physics-step record count before projecting any
  // source record. Source messages are observed synchronously and never
  // retained; only their bounded ROS Contact projections may enter groups.
  std::size_t record_count = 0U;
  for (std::size_t source_index = 0U; source_index < source_count;
    ++source_index)
  {
    if (contacts[source_index] == nullptr) {
      return "contact aggregate source message pointer must be nonnull";
    }
    const auto source_record_count =
      static_cast<std::size_t>(contacts[source_index]->contact_size());
    if (source_record_count > kMaxAggregateRecords - record_count) {
      return "one contact aggregate physics step exceeds the 16-record bound";
    }
    record_count += source_record_count;
  }

  for (std::size_t source_index = 0U; source_index < source_count;
    ++source_index)
  {
    for (const auto & contact : contacts[source_index]->contact()) {
      std::size_t contact_string_bytes = 0U;
      if (const auto error = validate_contact(contact, contact_string_bytes)) {
        return error;
      }
      auto first = contact.collision1().name();
      auto second = contact.collision2().name();
      if (second < first) {
        std::swap(first, second);
      }
      auto & group = groups[{std::move(first), std::move(second)}];
      if (group.records.size() >= kMaxRecordsPerPair) {
        return "one contact aggregate physics-step pair exceeds the four-record "
               "bound";
      }
      if (!checked_add(contact_string_bytes, group.string_bytes,
                       kMaxPairStringBytes))
      {
        return "one contact aggregate physics-step pair exceeds the string bound";
      }
      // Retain only the ROS Contact projection. This canonicalizes every
      // protobuf-only Header, Contact.world, Vector3d header, inner Wrench
      // header/force_offset, and unknown field without ever copying them into
      // interval storage. The admitted outer JointWrench header is bounded by
      // the frame_id-only grammar above.
      group.records.push_back(ros_projected_contact(contact));
    }
  }
  return validate_complete_groups(groups);
}

std::optional<std::string>
ContactAggregatorPolicy::validate_complete_groups(const ContactGroups & groups)
{
  std::size_t record_count = 0U;
  std::size_t string_bytes = 0U;
  for (const auto & entry : groups) {
    if (entry.second.records.empty() ||
      entry.second.records.size() > kMaxRecordsPerPair)
    {
      return "contact aggregate pair group violates the one-to-four-record "
             "bound";
    }
    if (!checked_add(entry.second.records.size(), record_count,
                     kMaxAggregateRecords))
    {
      return "contact aggregate interval union exceeds the 16-record bound";
    }
    if (!checked_add(entry.first.first.size() + entry.first.second.size(),
                     string_bytes, kMaxAggregateStringBytes) ||
      !checked_add(entry.second.string_bytes, string_bytes,
                     kMaxAggregateStringBytes))
    {
      return "contact aggregate interval union exceeds the 65536-byte string "
             "bound";
    }
  }
  return std::nullopt;
}

gz::msgs::Contacts
ContactAggregatorPolicy::aggregate_at(
  const std::int64_t boundary_stamp_ns,
  const ContactGroups & groups)
{
  gz::msgs::Contacts result;
  stamp_header(*result.mutable_header(), boundary_stamp_ns);
  for (const auto & entry : groups) {
    for (const auto & record : entry.second.records) {
      auto *output = result.add_contact();
      *output = record;
      // The aggregate timestamp is the interval watermark. Each retained
      // record is the bounded ROS Contact projection of the latest physics-
      // step group; only the outer and per-Contact headers are rewritten here.
      // Admitted JointWrench stamp/frame_id and all other ROS-visible fields
      // remain field-exact.
      stamp_header(*output->mutable_header(), boundary_stamp_ns);
    }
  }
  return result;
}

ContactAggregateDecision ContactAggregatorPolicy::fail(std::string detail)
{
  fatal_detail_ = std::move(detail);
  ContactAggregateDecision decision;
  decision.fatal = true;
  decision.detail = *fatal_detail_;
  return decision;
}

ContactAggregateDecision
ContactAggregatorPolicy::observe(
  const std::int64_t simulation_stamp_ns,
  const gz::msgs::Contacts & current_contacts)
{
  const gz::msgs::Contacts * const current_sources[] = {&current_contacts};
  return observe_sources(simulation_stamp_ns, current_sources, 1U);
}

ContactAggregateDecision
ContactAggregatorPolicy::observe(
  const std::int64_t simulation_stamp_ns,
  const ContactAggregateSources & current_sources)
{
  return observe_sources(
    simulation_stamp_ns, current_sources.data(), current_sources.size());
}

ContactAggregateDecision
ContactAggregatorPolicy::observe_sources(
  const std::int64_t simulation_stamp_ns,
  const gz::msgs::Contacts * const * current_sources,
  const std::size_t source_count)
{
  ContactAggregateDecision decision;
  if (fatal_detail_.has_value()) {
    decision.fatal = true;
    decision.detail = *fatal_detail_;
    return decision;
  }
  if (simulation_stamp_ns < 0) {
    return fail("contact aggregate simulation stamp must be nonnegative");
  }
  if (last_observed_stamp_ns_.has_value() &&
    simulation_stamp_ns <= *last_observed_stamp_ns_)
  {
    return fail("contact aggregate simulation stamp did not advance strictly");
  }

  ContactGroups step_groups;
  if (const auto error = validated_step_groups(
      current_sources, source_count, step_groups))
  {
    return fail(*error);
  }
  last_observed_stamp_ns_ = simulation_stamp_ns;

  if (!next_boundary_stamp_ns_.has_value()) {
    if (simulation_stamp_ns % kContactAggregatePeriodNs != 0) {
      return decision;
    }
    if (simulation_stamp_ns >
      std::numeric_limits<std::int64_t>::max() - kContactAggregatePeriodNs)
    {
      return fail("contact aggregate boundary arithmetic overflowed");
    }
    // The first observed grid boundary synchronizes the policy. Its ending
    // interval is incomplete, so it is intentionally discarded. The first
    // authoritative interval is (this boundary, next boundary].
    next_boundary_stamp_ns_ = simulation_stamp_ns + kContactAggregatePeriodNs;
    return decision;
  }

  if (simulation_stamp_ns > *next_boundary_stamp_ns_) {
    return fail(
        "contact aggregate physics observations skipped a 20 ms grid boundary");
  }

  // Interval membership is a union, but evidence for each member is the
  // complete latest physics-step group (one to four records in source order).
  // Replacement is the declared reduction; absence never removes a pair from
  // the current interval and no pair is retained into the next interval.
  auto projected = interval_groups_;
  for (auto & entry : step_groups) {
    projected.insert_or_assign(entry.first, std::move(entry.second));
  }
  if (const auto error = validate_complete_groups(projected)) {
    return fail(*error);
  }
  interval_groups_ = std::move(projected);

  if (simulation_stamp_ns < *next_boundary_stamp_ns_) {
    return decision;
  }

  if (*next_boundary_stamp_ns_ >
    std::numeric_limits<std::int64_t>::max() - kContactAggregatePeriodNs)
  {
    return fail("contact aggregate boundary arithmetic overflowed");
  }
  if (interval_groups_.empty()) {
    if (stream_started_) {
      return fail(
        "contact aggregate interval became empty after the sensed stream started");
    }
    interval_groups_.clear();
    *next_boundary_stamp_ns_ += kContactAggregatePeriodNs;
    return decision;
  }

  decision.output = aggregate_at(*next_boundary_stamp_ns_, interval_groups_);
  interval_groups_.clear();
  stream_started_ = true;
  *next_boundary_stamp_ns_ += kContactAggregatePeriodNs;
  return decision;
}

void ContactAggregatorPolicy::reset()
{
  last_observed_stamp_ns_.reset();
  next_boundary_stamp_ns_.reset();
  interval_groups_.clear();
  fatal_detail_.reset();
  stream_started_ = false;
}

}  // namespace robotest_sim

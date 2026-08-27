// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

#include "robotest_sim/contact_aggregator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <utility>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"

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
constexpr std::size_t kMaxEntityAncestryDepth = 64U;

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

namespace internal
{

gz::sim::Entity top_level_model_ancestor(
  const gz::sim::Entity entity,
  const gz::sim::EntityComponentManager & ecm)
{
  std::set<gz::sim::Entity> visited;
  auto current = entity;
  auto top_level_model = gz::sim::kNullEntity;
  for (std::size_t depth = 0U;
    current != gz::sim::kNullEntity && depth < kMaxEntityAncestryDepth;
    ++depth)
  {
    if (!visited.insert(current).second) {
      return gz::sim::kNullEntity;
    }
    if (ecm.Component<gz::sim::components::Model>(current) != nullptr) {
      top_level_model = current;
    }
    const auto parent = ecm.ParentEntity(current);
    if (parent == gz::sim::kNullEntity) {
      return top_level_model;
    }
    if (ecm.Component<gz::sim::components::World>(parent) != nullptr) {
      return top_level_model;
    }
    current = parent;
  }
  return gz::sim::kNullEntity;
}

bool nameless_contact_sensor_under_model(
  const gz::sim::Entity sensor_entity,
  const gz::sim::Entity model_entity,
  const gz::sim::EntityComponentManager & ecm)
{
  return model_entity != gz::sim::kNullEntity &&
         ecm.Component<gz::sim::components::Name>(sensor_entity) == nullptr &&
         top_level_model_ancestor(sensor_entity, ecm) == model_entity;
}

ContactProfileAccumulator::ContactProfileAccumulator(
  const bool enabled) noexcept
: enabled_(enabled)
{
}

bool ContactProfileAccumulator::enabled() const noexcept
{
  return enabled_;
}

void ContactProfileAccumulator::disable() noexcept
{
  enabled_ = false;
}

void ContactProfileAccumulator::lock(
  const std::int64_t simulation_stamp_ns) noexcept
{
  if (!enabled_ || simulation_stamp_ns < 0) {
    return;
  }
  cumulative_ns_.fill(0U);
  profile_epoch_start_sim_stamp_ns_ = simulation_stamp_ns;
  linux_tid_.reset();
  observation_count_ = 0U;
  rescan_count_ = 0U;
  publish_count_ = 0U;
  saturated_ = false;
  locked_ = true;
  restart_cadence(simulation_stamp_ns);
}

void ContactProfileAccumulator::restart_cadence(
  const std::int64_t simulation_stamp_ns) noexcept
{
  if (!enabled_ || !locked_ || simulation_stamp_ns < 0) {
    return;
  }
  constexpr auto maximum = std::numeric_limits<std::int64_t>::max();
  next_emission_stamp_ns_ =
    simulation_stamp_ns > maximum - kContactProfileEmissionPeriodNs ?
    maximum : simulation_stamp_ns + kContactProfileEmissionPeriodNs;
}

void ContactProfileAccumulator::saturating_add(
  std::uint64_t & target,
  const std::uint64_t increment) noexcept
{
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  if (increment > maximum - target) {
    target = maximum;
    saturated_ = true;
  } else {
    target += increment;
  }
}

void ContactProfileAccumulator::add_timing(
  const ContactProfileCategory category,
  const std::uint64_t elapsed_ns,
  const std::int64_t linux_tid) noexcept
{
  if (!enabled_ || !locked_) {
    return;
  }
  if (linux_tid <= 0) {
    disable();
    return;
  }
  if (!linux_tid_.has_value()) {
    linux_tid_ = linux_tid;
  } else if (*linux_tid_ != linux_tid) {
    // A single record must never combine CPU clocks from different threads.
    disable();
    return;
  }
  const auto index = static_cast<std::size_t>(category);
  if (index >= cumulative_ns_.size()) {
    disable();
    return;
  }
  saturating_add(cumulative_ns_[index], elapsed_ns);
}

void ContactProfileAccumulator::count(std::uint64_t & target) noexcept
{
  if (enabled_ && locked_) {
    saturating_add(target, 1U);
  }
}

void ContactProfileAccumulator::count_observation() noexcept
{
  count(observation_count_);
}

void ContactProfileAccumulator::count_rescan() noexcept
{
  count(rescan_count_);
}

void ContactProfileAccumulator::count_publish() noexcept
{
  count(publish_count_);
}

std::optional<std::string> ContactProfileAccumulator::emit_if_due(
  const std::int64_t simulation_stamp_ns)
{
  if (!enabled_ || !locked_ || !linux_tid_.has_value() ||
    !profile_epoch_start_sim_stamp_ns_.has_value() ||
    simulation_stamp_ns < 0 || !next_emission_stamp_ns_.has_value() ||
    simulation_stamp_ns < *next_emission_stamp_ns_)
  {
    return std::nullopt;
  }

  const auto scheduled_stamp_ns = *next_emission_stamp_ns_;
  constexpr auto maximum = std::numeric_limits<std::int64_t>::max();
  const auto intervals =
    (simulation_stamp_ns - scheduled_stamp_ns) /
    kContactProfileEmissionPeriodNs + 1;
  if (intervals >
    (maximum - scheduled_stamp_ns) / kContactProfileEmissionPeriodNs)
  {
    next_emission_stamp_ns_.reset();
  } else {
    next_emission_stamp_ns_ =
      scheduled_stamp_ns + intervals * kContactProfileEmissionPeriodNs;
  }

  std::uint64_t measured_total_ns = 0U;
  for (const auto elapsed_ns : cumulative_ns_) {
    saturating_add(measured_total_ns, elapsed_ns);
  }
  const auto category = [this](const ContactProfileCategory value) {
      return cumulative_ns_[static_cast<std::size_t>(value)];
    };
  std::string record;
  record.reserve(640U);
  record += "{\"cached_event_state_check_ns\":" +
    std::to_string(category(ContactProfileCategory::CachedEventStateCheck));
  record += ",\"clock_id\":\"CLOCK_THREAD_CPUTIME_ID\"";
  record += ",\"contact_policy_protobuf_ns\":" +
    std::to_string(category(ContactProfileCategory::ContactPolicyProtobuf));
  record += ",\"exhaustive_event_rescan_ns\":" +
    std::to_string(category(ContactProfileCategory::ExhaustiveEventRescan));
  record += ",\"linux_tid\":" + std::to_string(*linux_tid_);
  record += ",\"locked_binding_validation_ns\":" +
    std::to_string(category(ContactProfileCategory::LockedBindingValidation));
  record += ",\"measured_total_ns\":" + std::to_string(measured_total_ns);
  record += ",\"observation_count\":" + std::to_string(observation_count_);
  record += ",\"profile_epoch_start_sim_stamp_ns\":" +
    std::to_string(*profile_epoch_start_sim_stamp_ns_);
  record += ",\"publish_count\":" + std::to_string(publish_count_);
  record += ",\"publish_ns\":" +
    std::to_string(category(ContactProfileCategory::Publish));
  record += ",\"rescan_count\":" + std::to_string(rescan_count_);
  record += saturated_ ? ",\"saturated\":true" : ",\"saturated\":false";
  record += ",\"schema_version\":1";
  record += ",\"sim_stamp_ns\":" + std::to_string(simulation_stamp_ns) + "}";
  return record;
}

std::array<bool, 2U> cached_inventory_component_changes(
  const gz::sim::EntityComponentManager & ecm,
  const std::vector<gz::sim::Entity> & model_entities,
  const std::vector<gz::sim::Entity> & sensor_entities,
  const std::vector<gz::sim::Entity> & link_entities,
  const std::vector<gz::sim::Entity> & collision_entities)
{
  std::array<bool, 2U> changes{};
  const auto inspect_entity = [&ecm, &changes](
    const gz::sim::Entity entity,
    const gz::sim::ComponentTypeId primary_type) {
      const std::array<gz::sim::ComponentTypeId, 3U> types = {{
        primary_type,
        gz::sim::components::Name::typeId,
        gz::sim::components::ParentEntity::typeId,
      }};
      for (const auto type : types) {
        const auto state = ecm.ComponentState(entity, type);
        changes[0] = changes[0] ||
          state == gz::sim::ComponentState::OneTimeChange;
        changes[1] = changes[1] ||
          state == gz::sim::ComponentState::PeriodicChange;
      }
      return changes[0] || changes[1];
    };
  const auto inspect_cache = [&inspect_entity](const auto & entities,
    const gz::sim::ComponentTypeId primary_type) {
      return std::any_of(
        entities.begin(), entities.end(),
        [&inspect_entity, primary_type](const auto entity) {
          return inspect_entity(entity, primary_type);
        });
    };

  if (inspect_cache(model_entities, gz::sim::components::Model::typeId) ||
    inspect_cache(sensor_entities,
      gz::sim::components::ContactSensor::typeId) ||
    inspect_cache(link_entities, gz::sim::components::Link::typeId) ||
    inspect_cache(collision_entities,
      gz::sim::components::Collision::typeId))
  {
    return changes;
  }
  return changes;
}

template<typename InventoryComponent>
bool inventory_type_has_one_time_structural_change(
  const gz::sim::EntityComponentManager & ecm)
{
  bool changed = false;
  ecm.Each<InventoryComponent>(
    [&ecm, &changed](const gz::sim::Entity & entity,
    const InventoryComponent *) {
      const std::array<gz::sim::ComponentTypeId, 3U> types = {{
        InventoryComponent::typeId,
        gz::sim::components::Name::typeId,
        gz::sim::components::ParentEntity::typeId,
      }};
      changed = std::any_of(
        types.begin(), types.end(),
        [&ecm, entity](const auto type) {
          return ecm.ComponentState(entity, type) ==
                 gz::sim::ComponentState::OneTimeChange;
        });
      return !changed;
    });
  return changed;
}

bool relevant_one_time_inventory_component_changed(
  const gz::sim::EntityComponentManager & ecm)
{
  // Gazebo Sim 8 exposes a periodic changed-type set, but its one-time set is
  // private. The caller guards these typed views with
  // HasOneTimeComponentChanges(), so existing-entity component promotion is
  // visible without traversing any global view during steady-state steps.
  return
    inventory_type_has_one_time_structural_change<
    gz::sim::components::Model>(ecm) ||
    inventory_type_has_one_time_structural_change<
    gz::sim::components::ContactSensor>(ecm) ||
    inventory_type_has_one_time_structural_change<
    gz::sim::components::Link>(ecm) ||
    inventory_type_has_one_time_structural_change<
    gz::sim::components::Collision>(ecm);
}

bool relevant_periodic_inventory_type_changed(
  const gz::sim::EntityComponentManager & ecm)
{
  if (!ecm.HasPeriodicComponentChanges()) {
    return false;
  }
  const auto & changed_types = ecm.ComponentTypesWithPeriodicChanges();
  const std::array<gz::sim::ComponentTypeId, 6U> inventory_types = {{
    gz::sim::components::Model::typeId,
    gz::sim::components::Name::typeId,
    gz::sim::components::ParentEntity::typeId,
    gz::sim::components::ContactSensor::typeId,
    gz::sim::components::Link::typeId,
    gz::sim::components::Collision::typeId,
  }};
  return std::any_of(
    inventory_types.begin(), inventory_types.end(),
    [&changed_types](const auto type) {
      return changed_types.find(type) != changed_types.end();
    });
}

bool locked_inventory_scan_required(
  const bool new_entities,
  const bool entities_marked_for_removal,
  const bool removed_components,
  const bool relevant_one_time_change,
  const bool relevant_periodic_change,
  const bool,
  const bool) noexcept
{
  // The final two arguments witness unrelated one-time / periodic activity.
  // They are intentionally excluded so physics-rate payload and Pose changes
  // cannot make the structural inventory scan hot.
  return new_entities || entities_marked_for_removal || removed_components ||
         relevant_one_time_change || relevant_periodic_change;
}

}  // namespace internal

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
  for (auto & entry : step_groups) {
    interval_groups_.insert_or_assign(entry.first, std::move(entry.second));
  }
  if (const auto error = validate_complete_groups(interval_groups_)) {
    return fail(*error);
  }

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

// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"
#include "google/protobuf/unknown_field_set.h"
#include "robotest_sim/contact_aggregator.hpp"
#include "gtest/gtest.h"

namespace robotest_sim
{
namespace internal
{

bool nameless_contact_sensor_under_model(
  gz::sim::Entity sensor_entity,
  gz::sim::Entity model_entity,
  const gz::sim::EntityComponentManager & ecm);

std::array<bool, 2U> cached_inventory_component_changes(
  const gz::sim::EntityComponentManager & ecm,
  const std::vector<gz::sim::Entity> & model_entities,
  const std::vector<gz::sim::Entity> & sensor_entities,
  const std::vector<gz::sim::Entity> & link_entities,
  const std::vector<gz::sim::Entity> & collision_entities);

bool relevant_one_time_inventory_component_changed(
  const gz::sim::EntityComponentManager & ecm);

bool relevant_periodic_inventory_type_changed(
  const gz::sim::EntityComponentManager & ecm);

bool locked_inventory_scan_required(
  bool new_entities,
  bool entities_marked_for_removal,
  bool removed_components,
  bool relevant_one_time_change,
  bool relevant_periodic_change,
  bool unrelated_one_time_change,
  bool unrelated_periodic_change) noexcept;

}  // namespace internal

namespace
{

using Pair = std::pair<std::string, std::string>;
using SourceMessages =
  std::array<gz::msgs::Contacts, kContactAggregateSourceCount>;

ContactAggregateSources source_views(const SourceMessages & messages)
{
  ContactAggregateSources views{};
  for (std::size_t index = 0U; index < messages.size(); ++index) {
    views[index] = &messages[index];
  }
  return views;
}

gz::msgs::Contact contact_for(
  const std::string & first,
  const std::string & second,
  const double marker = 1.0)
{
  gz::msgs::Contact contact;
  contact.mutable_collision1()->set_name(first);
  contact.mutable_collision2()->set_name(second);
  contact.add_position()->set_x(marker);
  return contact;
}

gz::msgs::Contacts
contacts_for(const std::vector<gz::msgs::Contact> & contacts)
{
  gz::msgs::Contacts message;
  for (const auto & contact : contacts) {
    *message.add_contact() = contact;
  }
  return message;
}

gz::msgs::Contacts one_contact(
  const std::string & first,
  const std::string & second,
  const double marker = 1.0)
{
  return contacts_for({contact_for(first, second, marker)});
}

std::int64_t stamp_ns(const gz::msgs::Header & header)
{
  return header.stamp().sec() * 1000000000LL + header.stamp().nsec();
}

void set_stamp(gz::msgs::Header & header, const std::int64_t value_ns)
{
  header.mutable_stamp()->set_sec(value_ns / 1000000000LL);
  header.mutable_stamp()->set_nsec(
    static_cast<std::int32_t>(value_ns % 1000000000LL));
}

Pair normalized_pair(const gz::msgs::Contact & contact)
{
  auto first = contact.collision1().name();
  auto second = contact.collision2().name();
  if (second < first) {
    std::swap(first, second);
  }
  return {std::move(first), std::move(second)};
}

void synchronize(
  ContactAggregatorPolicy & policy,
  const std::int64_t boundary_ns = 1000000000LL)
{
  const auto decision = policy.observe(boundary_ns, gz::msgs::Contacts());
  ASSERT_FALSE(decision.fatal);
  ASSERT_FALSE(decision.output.has_value());
}

void set_inventory_component_states(
  gz::sim::EntityComponentManager & ecm,
  const gz::sim::Entity entity,
  const gz::sim::ComponentTypeId primary_type,
  const gz::sim::ComponentState state)
{
  ecm.SetChanged(entity, primary_type, state);
  ecm.SetChanged(entity, gz::sim::components::Name::typeId, state);
  ecm.SetChanged(entity, gz::sim::components::ParentEntity::typeId, state);
}

bool real_ecm_inventory_gate_requires_scan(
  const gz::sim::EntityComponentManager & ecm)
{
  const bool has_one_time_component_changes =
    ecm.HasOneTimeComponentChanges();
  const bool relevant_one_time_change =
    has_one_time_component_changes &&
    internal::relevant_one_time_inventory_component_changed(ecm);
  return internal::locked_inventory_scan_required(
    ecm.HasNewEntities(), ecm.HasEntitiesMarkedForRemoval(),
    ecm.HasRemovedComponents(), relevant_one_time_change,
    internal::relevant_periodic_inventory_type_changed(ecm),
    has_one_time_component_changes, ecm.HasPeriodicComponentChanges());
}

template<typename Component>
void expect_existing_entity_promotion_requires_scan(const Component & component)
{
  gz::sim::EntityComponentManager ecm;
  const auto entity = ecm.CreateEntity();
  ecm.ClearNewlyCreatedEntities();
  ecm.SetAllComponentsUnchanged();
  ASSERT_FALSE(ecm.HasNewEntities());
  ASSERT_FALSE(ecm.HasOneTimeComponentChanges());

  ASSERT_NE(ecm.CreateComponent(entity, component), nullptr);
  ASSERT_FALSE(ecm.HasNewEntities());
  ASSERT_TRUE(ecm.HasOneTimeComponentChanges());
  EXPECT_TRUE(
    internal::relevant_one_time_inventory_component_changed(ecm));
  EXPECT_TRUE(real_ecm_inventory_gate_requires_scan(ecm));
}

template<typename InventoryComponent, typename AddedComponentFactory>
void expect_existing_structural_entity_addition_requires_scan(
  const InventoryComponent & inventory_component,
  AddedComponentFactory make_added_component)
{
  gz::sim::EntityComponentManager ecm;
  const auto parent = ecm.CreateEntity();
  const auto entity = ecm.CreateEntity();
  ASSERT_NE(ecm.CreateComponent(entity, inventory_component), nullptr);
  ecm.ClearNewlyCreatedEntities();
  ecm.SetAllComponentsUnchanged();
  ASSERT_FALSE(ecm.HasNewEntities());
  ASSERT_FALSE(ecm.HasOneTimeComponentChanges());

  ASSERT_NE(
    ecm.CreateComponent(entity, make_added_component(parent)), nullptr);
  ASSERT_FALSE(ecm.HasNewEntities());
  ASSERT_TRUE(ecm.HasOneTimeComponentChanges());
  EXPECT_TRUE(
    internal::relevant_one_time_inventory_component_changed(ecm));
  EXPECT_TRUE(real_ecm_inventory_gate_requires_scan(ecm));
}

}  // namespace

TEST(ContactAggregatorInventoryGate,
     RelevantEventsRetriggerButHighRatePayloadChangesDoNot) {
  EXPECT_FALSE(internal::locked_inventory_scan_required(
      false, false, false, false, false, false, false));

  struct GateCase
  {
    const char *name;
    std::array<bool, 5U> events;
  };
  const std::array<GateCase, 5U> cases = {{
    {"new entity", {true, false, false, false, false}},
    {"marked removal", {false, true, false, false, false}},
    {"removed relevant entity or component",
      {false, false, true, false, false}},
    {"relevant one-time component state",
      {false, false, false, true, false}},
    {"relevant periodic component state",
      {false, false, false, false, true}},
  }};
  for (const auto & test_case : cases) {
    SCOPED_TRACE(test_case.name);
    EXPECT_TRUE(internal::locked_inventory_scan_required(
        test_case.events[0], test_case.events[1], test_case.events[2],
        test_case.events[3], test_case.events[4], false, false));
  }

  // Global one-time controller changes and high-rate periodic Pose /
  // ContactSensorData changes are intentionally irrelevant to source identity.
  EXPECT_FALSE(internal::locked_inventory_scan_required(
      false, false, false, false, false, true, false));
  EXPECT_FALSE(internal::locked_inventory_scan_required(
      false, false, false, false, false, false, true));
  EXPECT_FALSE(internal::locked_inventory_scan_required(
      false, false, false, false, false, true, true));
}

TEST(ContactAggregatorInventoryGate,
     CachedRealEcmStatesSelectStructuralChangesOnly) {
  gz::sim::EntityComponentManager ecm;
  const auto parent = ecm.CreateEntity();
  const auto model = ecm.CreateEntity();
  const auto sensor = ecm.CreateEntity();
  const auto link = ecm.CreateEntity();
  const auto collision = ecm.CreateEntity();
  const auto unbound_link = ecm.CreateEntity();
  const auto unbound_collision = ecm.CreateEntity();

  ASSERT_NE(ecm.CreateComponent(model, gz::sim::components::Model()), nullptr);
  ASSERT_NE(
    ecm.CreateComponent(model, gz::sim::components::Name("model")), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      model, gz::sim::components::ParentEntity(parent)), nullptr);
  ASSERT_NE(
    ecm.CreateComponent(sensor, gz::sim::components::ContactSensor()), nullptr);
  ASSERT_NE(
    ecm.CreateComponent(sensor, gz::sim::components::Name("sensor")), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      sensor, gz::sim::components::ParentEntity(link)), nullptr);
  ASSERT_NE(ecm.CreateComponent(link, gz::sim::components::Link()), nullptr);
  ASSERT_NE(
    ecm.CreateComponent(link, gz::sim::components::Name("link")), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      link, gz::sim::components::ParentEntity(model)), nullptr);
  ASSERT_NE(
    ecm.CreateComponent(collision, gz::sim::components::Collision()), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      collision, gz::sim::components::Name("collision")), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      collision, gz::sim::components::ParentEntity(link)), nullptr);
  ASSERT_NE(
    ecm.CreateComponent(unbound_link, gz::sim::components::Link()), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      unbound_link, gz::sim::components::Name("spare_link")), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      unbound_link, gz::sim::components::ParentEntity(model)), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      unbound_collision, gz::sim::components::Collision()), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      unbound_collision, gz::sim::components::Name("spare_collision")),
    nullptr);
  ASSERT_NE(ecm.CreateComponent(
      unbound_collision, gz::sim::components::ParentEntity(link)), nullptr);

  const std::vector<gz::sim::Entity> models{model};
  const std::vector<gz::sim::Entity> sensors{sensor};
  const std::vector<gz::sim::Entity> links{link, unbound_link};
  const std::vector<gz::sim::Entity> collisions{
    collision, unbound_collision};
  const auto changes = [&]() {
      return internal::cached_inventory_component_changes(
        ecm, models, sensors, links, collisions);
    };
  const auto clear_relevant_states = [&]() {
      set_inventory_component_states(
        ecm, model, gz::sim::components::Model::typeId,
        gz::sim::ComponentState::NoChange);
      set_inventory_component_states(
        ecm, sensor, gz::sim::components::ContactSensor::typeId,
        gz::sim::ComponentState::NoChange);
      set_inventory_component_states(
        ecm, link, gz::sim::components::Link::typeId,
        gz::sim::ComponentState::NoChange);
      set_inventory_component_states(
        ecm, collision, gz::sim::components::Collision::typeId,
        gz::sim::ComponentState::NoChange);
      set_inventory_component_states(
        ecm, unbound_link, gz::sim::components::Link::typeId,
        gz::sim::ComponentState::NoChange);
      set_inventory_component_states(
        ecm, unbound_collision, gz::sim::components::Collision::typeId,
        gz::sim::ComponentState::NoChange);
    };

  clear_relevant_states();
  EXPECT_EQ(changes(), (std::array<bool, 2U>{false, false}));

  ecm.SetChanged(
    sensor, gz::sim::components::ContactSensor::typeId,
    gz::sim::ComponentState::OneTimeChange);
  EXPECT_EQ(changes(), (std::array<bool, 2U>{true, false}));
  clear_relevant_states();

  ecm.SetChanged(
    sensor, gz::sim::components::Name::typeId,
    gz::sim::ComponentState::PeriodicChange);
  EXPECT_EQ(changes(), (std::array<bool, 2U>{false, true}));
  clear_relevant_states();

  ecm.SetChanged(
    sensor, gz::sim::components::ParentEntity::typeId,
    gz::sim::ComponentState::OneTimeChange);
  EXPECT_EQ(changes(), (std::array<bool, 2U>{true, false}));
  clear_relevant_states();

  ecm.SetChanged(
    link, gz::sim::components::Link::typeId,
    gz::sim::ComponentState::PeriodicChange);
  EXPECT_EQ(changes(), (std::array<bool, 2U>{false, true}));
  clear_relevant_states();

  ecm.SetChanged(
    collision, gz::sim::components::Collision::typeId,
    gz::sim::ComponentState::OneTimeChange);
  EXPECT_EQ(changes(), (std::array<bool, 2U>{true, false}));
  clear_relevant_states();

  struct UnboundOneTimeCase
  {
    const char *name;
    gz::sim::Entity entity;
    gz::sim::ComponentTypeId type;
  };
  const std::array<UnboundOneTimeCase, 6U> unbound_cases = {{
    {"unbound link primary", unbound_link,
      gz::sim::components::Link::typeId},
    {"unbound link name", unbound_link,
      gz::sim::components::Name::typeId},
    {"unbound link parent", unbound_link,
      gz::sim::components::ParentEntity::typeId},
    {"unbound collision primary", unbound_collision,
      gz::sim::components::Collision::typeId},
    {"unbound collision name", unbound_collision,
      gz::sim::components::Name::typeId},
    {"unbound collision parent", unbound_collision,
      gz::sim::components::ParentEntity::typeId},
  }};
  for (const auto & test_case : unbound_cases) {
    SCOPED_TRACE(test_case.name);
    ecm.SetChanged(
      test_case.entity, test_case.type,
      gz::sim::ComponentState::OneTimeChange);
    EXPECT_EQ(changes(), (std::array<bool, 2U>{true, false}));
    clear_relevant_states();
  }

  ASSERT_NE(
    ecm.CreateComponent(model, gz::sim::components::Pose()), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      collision, gz::sim::components::ContactSensorData()), nullptr);
  ecm.SetChanged(
    model, gz::sim::components::Pose::typeId,
    gz::sim::ComponentState::PeriodicChange);
  ecm.SetChanged(
    collision, gz::sim::components::ContactSensorData::typeId,
    gz::sim::ComponentState::PeriodicChange);
  EXPECT_EQ(changes(), (std::array<bool, 2U>{false, false}));

  // ComponentTypesWithPeriodicChanges() retains type-level update state for
  // the current ECM cycle, so isolate the type-filter assertions from the
  // relevant periodic changes exercised above.
  gz::sim::EntityComponentManager periodic_ecm;
  const auto periodic_model = periodic_ecm.CreateEntity();
  const auto periodic_collision = periodic_ecm.CreateEntity();
  const auto periodic_sensor = periodic_ecm.CreateEntity();
  ASSERT_NE(periodic_ecm.CreateComponent(
      periodic_model, gz::sim::components::Pose()), nullptr);
  ASSERT_NE(periodic_ecm.CreateComponent(
      periodic_collision, gz::sim::components::ContactSensorData()), nullptr);
  ASSERT_NE(periodic_ecm.CreateComponent(
      periodic_sensor, gz::sim::components::Name("sensor")), nullptr);
  periodic_ecm.SetChanged(
    periodic_model, gz::sim::components::Pose::typeId,
    gz::sim::ComponentState::PeriodicChange);
  periodic_ecm.SetChanged(
    periodic_collision, gz::sim::components::ContactSensorData::typeId,
    gz::sim::ComponentState::PeriodicChange);
  EXPECT_FALSE(
    internal::relevant_periodic_inventory_type_changed(periodic_ecm));

  periodic_ecm.SetChanged(
    periodic_sensor, gz::sim::components::Name::typeId,
    gz::sim::ComponentState::PeriodicChange);
  EXPECT_TRUE(
    internal::relevant_periodic_inventory_type_changed(periodic_ecm));
}

TEST(ContactAggregatorInventoryGate,
     ExistingEntityStructuralPromotionRetriggersRealEcmInventoryScan) {
  expect_existing_entity_promotion_requires_scan(
    gz::sim::components::Model());
  expect_existing_entity_promotion_requires_scan(
    gz::sim::components::ContactSensor());
  expect_existing_entity_promotion_requires_scan(
    gz::sim::components::Link());
  expect_existing_entity_promotion_requires_scan(
    gz::sim::components::Collision());

  const auto expect_name_and_parent_additions = [](const auto & component) {
      expect_existing_structural_entity_addition_requires_scan(
        component, [](const gz::sim::Entity) {
          return gz::sim::components::Name("promoted");
        });
      expect_existing_structural_entity_addition_requires_scan(
        component, [](const gz::sim::Entity parent) {
          return gz::sim::components::ParentEntity(parent);
        });
    };
  expect_name_and_parent_additions(gz::sim::components::Model());
  expect_name_and_parent_additions(gz::sim::components::ContactSensor());
  expect_name_and_parent_additions(gz::sim::components::Link());
  expect_name_and_parent_additions(gz::sim::components::Collision());
}

TEST(ContactAggregatorInventoryGate,
     UnrelatedExistingEntityOneTimeChurnDoesNotRetriggerInventoryScan) {
  {
    gz::sim::EntityComponentManager ecm;
    const auto entity = ecm.CreateEntity();
    ecm.ClearNewlyCreatedEntities();
    ecm.SetAllComponentsUnchanged();
    ASSERT_NE(
      ecm.CreateComponent(entity, gz::sim::components::Name("generic")),
      nullptr);
    ASSERT_FALSE(ecm.HasNewEntities());
    ASSERT_TRUE(ecm.HasOneTimeComponentChanges());
    EXPECT_FALSE(
      internal::relevant_one_time_inventory_component_changed(ecm));
    EXPECT_FALSE(real_ecm_inventory_gate_requires_scan(ecm));
  }

  {
    gz::sim::EntityComponentManager ecm;
    const auto entity = ecm.CreateEntity();
    ASSERT_NE(
      ecm.CreateComponent(entity, gz::sim::components::Model()), nullptr);
    ecm.ClearNewlyCreatedEntities();
    ecm.SetAllComponentsUnchanged();
    ASSERT_NE(
      ecm.CreateComponent(entity, gz::sim::components::Pose()), nullptr);
    ASSERT_FALSE(ecm.HasNewEntities());
    ASSERT_TRUE(ecm.HasOneTimeComponentChanges());
    EXPECT_FALSE(
      internal::relevant_one_time_inventory_component_changed(ecm));
    EXPECT_FALSE(real_ecm_inventory_gate_requires_scan(ecm));
  }
}

TEST(ContactAggregatorInventoryGate,
     NamelessContactSensorUnderRobotModelIsNotSilentlyIgnored) {
  gz::sim::EntityComponentManager ecm;
  const auto world = ecm.CreateEntity();
  const auto robot_model = ecm.CreateEntity();
  const auto robot_link = ecm.CreateEntity();
  const auto robot_sensor = ecm.CreateEntity();
  const auto other_model = ecm.CreateEntity();
  const auto other_link = ecm.CreateEntity();
  const auto other_sensor = ecm.CreateEntity();

  ASSERT_NE(
    ecm.CreateComponent(world, gz::sim::components::World()), nullptr);
  ASSERT_NE(
    ecm.CreateComponent(robot_model, gz::sim::components::Model()), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      robot_model, gz::sim::components::ParentEntity(world)), nullptr);
  ASSERT_NE(
    ecm.CreateComponent(robot_link, gz::sim::components::Link()), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      robot_link, gz::sim::components::ParentEntity(robot_model)), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      robot_sensor, gz::sim::components::ContactSensor()), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      robot_sensor, gz::sim::components::ParentEntity(robot_link)), nullptr);

  ASSERT_NE(
    ecm.CreateComponent(other_model, gz::sim::components::Model()), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      other_model, gz::sim::components::ParentEntity(world)), nullptr);
  ASSERT_NE(
    ecm.CreateComponent(other_link, gz::sim::components::Link()), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      other_link, gz::sim::components::ParentEntity(other_model)), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      other_sensor, gz::sim::components::ContactSensor()), nullptr);
  ASSERT_NE(ecm.CreateComponent(
      other_sensor, gz::sim::components::ParentEntity(other_link)), nullptr);

  EXPECT_TRUE(internal::nameless_contact_sensor_under_model(
      robot_sensor, robot_model, ecm));
  EXPECT_FALSE(internal::nameless_contact_sensor_under_model(
      other_sensor, robot_model, ecm));

  ASSERT_NE(ecm.CreateComponent(
      robot_sensor, gz::sim::components::Name("named_sensor")), nullptr);
  EXPECT_FALSE(internal::nameless_contact_sensor_under_model(
      robot_sensor, robot_model, ecm));
}

TEST(ContactAggregatorProfile,
     CanonicalRecordIsExactAndCumulativeValuesAreMonotonic) {
  internal::ContactProfileAccumulator profile(true);
  constexpr std::int64_t lock_stamp_ns = 1000;
  constexpr std::int64_t linux_tid = 4321;
  profile.lock(lock_stamp_ns);
  profile.add_timing(
    internal::ContactProfileCategory::LockedBindingValidation, 10U, linux_tid);
  profile.add_timing(
    internal::ContactProfileCategory::CachedEventStateCheck, 20U, linux_tid);
  profile.add_timing(
    internal::ContactProfileCategory::ExhaustiveEventRescan, 30U, linux_tid);
  profile.add_timing(
    internal::ContactProfileCategory::ContactPolicyProtobuf, 40U, linux_tid);
  profile.add_timing(
    internal::ContactProfileCategory::Publish, 50U, linux_tid);
  profile.count_observation();
  profile.count_observation();
  profile.count_rescan();
  profile.count_publish();

  EXPECT_FALSE(profile.emit_if_due(
      lock_stamp_ns + internal::kContactProfileEmissionPeriodNs - 1)
    .has_value());
  const auto first = profile.emit_if_due(
    lock_stamp_ns + internal::kContactProfileEmissionPeriodNs);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(
    *first,
    "{\"cached_event_state_check_ns\":20,"
    "\"clock_id\":\"CLOCK_THREAD_CPUTIME_ID\","
    "\"contact_policy_protobuf_ns\":40,"
    "\"exhaustive_event_rescan_ns\":30,"
    "\"linux_tid\":4321,"
    "\"locked_binding_validation_ns\":10,"
    "\"measured_total_ns\":150,"
    "\"observation_count\":2,"
    "\"profile_epoch_start_sim_stamp_ns\":1000,"
    "\"publish_count\":1,"
    "\"publish_ns\":50,"
    "\"rescan_count\":1,"
    "\"saturated\":false,"
    "\"schema_version\":1,"
    "\"sim_stamp_ns\":5000001000}");
  EXPECT_FALSE(profile.emit_if_due(
      lock_stamp_ns + internal::kContactProfileEmissionPeriodNs)
    .has_value());

  profile.add_timing(
    internal::ContactProfileCategory::LockedBindingValidation, 1U, linux_tid);
  profile.add_timing(
    internal::ContactProfileCategory::CachedEventStateCheck, 2U, linux_tid);
  profile.add_timing(
    internal::ContactProfileCategory::ExhaustiveEventRescan, 3U, linux_tid);
  profile.add_timing(
    internal::ContactProfileCategory::ContactPolicyProtobuf, 4U, linux_tid);
  profile.add_timing(
    internal::ContactProfileCategory::Publish, 5U, linux_tid);
  profile.count_observation();
  profile.count_rescan();
  profile.count_publish();
  const auto second = profile.emit_if_due(
    lock_stamp_ns + 2 * internal::kContactProfileEmissionPeriodNs);
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(
    *second,
    "{\"cached_event_state_check_ns\":22,"
    "\"clock_id\":\"CLOCK_THREAD_CPUTIME_ID\","
    "\"contact_policy_protobuf_ns\":44,"
    "\"exhaustive_event_rescan_ns\":33,"
    "\"linux_tid\":4321,"
    "\"locked_binding_validation_ns\":11,"
    "\"measured_total_ns\":165,"
    "\"observation_count\":3,"
    "\"profile_epoch_start_sim_stamp_ns\":1000,"
    "\"publish_count\":2,"
    "\"publish_ns\":55,"
    "\"rescan_count\":2,"
    "\"saturated\":false,"
    "\"schema_version\":1,"
    "\"sim_stamp_ns\":10000001000}");
}

TEST(ContactAggregatorProfile, SimulationTimeJumpNeverProducesABurst) {
  internal::ContactProfileAccumulator profile(true);
  constexpr std::int64_t linux_tid = 17;
  profile.lock(0);
  profile.add_timing(
    internal::ContactProfileCategory::LockedBindingValidation, 1U, linux_tid);
  EXPECT_TRUE(profile.emit_if_due(
      internal::kContactProfileEmissionPeriodNs)
    .has_value());
  EXPECT_FALSE(profile.emit_if_due(
      internal::kContactProfileEmissionPeriodNs)
    .has_value());
  EXPECT_TRUE(profile.emit_if_due(
      6 * internal::kContactProfileEmissionPeriodNs)
    .has_value());
  EXPECT_FALSE(profile.emit_if_due(
      6 * internal::kContactProfileEmissionPeriodNs)
    .has_value());
  EXPECT_FALSE(profile.emit_if_due(
      7 * internal::kContactProfileEmissionPeriodNs - 1)
    .has_value());
  EXPECT_TRUE(profile.emit_if_due(
      7 * internal::kContactProfileEmissionPeriodNs)
    .has_value());
}

TEST(ContactAggregatorProfile, DisabledOrMixedThreadStateCannotEmit) {
  internal::ContactProfileAccumulator disabled(false);
  disabled.lock(0);
  disabled.add_timing(
    internal::ContactProfileCategory::Publish, 1U, 7);
  disabled.count_observation();
  EXPECT_FALSE(disabled.emit_if_due(
      internal::kContactProfileEmissionPeriodNs)
    .has_value());

  internal::ContactProfileAccumulator mixed_thread(true);
  mixed_thread.lock(0);
  mixed_thread.add_timing(
    internal::ContactProfileCategory::LockedBindingValidation, 1U, 7);
  mixed_thread.add_timing(
    internal::ContactProfileCategory::CachedEventStateCheck, 1U, 8);
  EXPECT_FALSE(mixed_thread.enabled());
  EXPECT_FALSE(mixed_thread.emit_if_due(
      internal::kContactProfileEmissionPeriodNs)
    .has_value());
}

TEST(ContactAggregatorProfile, NewEpochClearsCountersTimingsAndThreadIdentity) {
  internal::ContactProfileAccumulator profile(true);
  profile.lock(100);
  profile.add_timing(
    internal::ContactProfileCategory::LockedBindingValidation, 99U, 7);
  profile.count_observation();
  profile.count_rescan();

  profile.lock(200);
  profile.add_timing(internal::ContactProfileCategory::Publish, 3U, 8);
  profile.count_publish();
  const auto record = profile.emit_if_due(
    200 + internal::kContactProfileEmissionPeriodNs);
  ASSERT_TRUE(record.has_value());
  EXPECT_NE(record->find("\"linux_tid\":8"), std::string::npos);
  EXPECT_NE(record->find("\"measured_total_ns\":3"), std::string::npos);
  EXPECT_NE(
    record->find("\"profile_epoch_start_sim_stamp_ns\":200"),
    std::string::npos);
  EXPECT_NE(record->find("\"observation_count\":0"), std::string::npos);
  EXPECT_NE(record->find("\"rescan_count\":0"), std::string::npos);
  EXPECT_NE(record->find("\"publish_count\":1"), std::string::npos);
  EXPECT_EQ(record->find("\"linux_tid\":7"), std::string::npos);
}

TEST(ContactAggregatorProfile, SaturationIsExplicitAndNeverWraps) {
  internal::ContactProfileAccumulator profile(true);
  profile.lock(0);
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  profile.add_timing(
    internal::ContactProfileCategory::LockedBindingValidation, maximum, 9);
  profile.add_timing(
    internal::ContactProfileCategory::CachedEventStateCheck, 1U, 9);
  const auto record = profile.emit_if_due(
    internal::kContactProfileEmissionPeriodNs);
  ASSERT_TRUE(record.has_value());
  EXPECT_NE(
    record->find("\"measured_total_ns\":" + std::to_string(maximum)),
    std::string::npos);
  EXPECT_NE(record->find("\"saturated\":true"), std::string::npos);
}

TEST(ContactAggregatorPolicy,
     SuppressesEmptyBoundariesUntilFirstNonemptyInterval) {
  ContactAggregatorPolicy policy;
  synchronize(policy);

  const auto first = policy.observe(1020000000LL, gz::msgs::Contacts());
  EXPECT_FALSE(first.fatal);
  EXPECT_FALSE(first.output.has_value());

  const auto second = policy.observe(1040000000LL, gz::msgs::Contacts());
  EXPECT_FALSE(second.fatal);
  EXPECT_FALSE(second.output.has_value());

  const auto support =
    one_contact("ground::link::collision", "robot::wheel::collision");
  EXPECT_FALSE(policy.observe(1042000000LL, support).output.has_value());
  const auto started = policy.observe(1060000000LL, gz::msgs::Contacts());
  ASSERT_TRUE(started.output.has_value());
  EXPECT_EQ(stamp_ns(started.output->header()), 1060000000LL);
  EXPECT_EQ(started.output->contact_size(), 1);
}

TEST(ContactAggregatorPolicy,
     IntervalUnionRetainsOneStepTransientOnlyUntilBoundary) {
  ContactAggregatorPolicy policy;
  synchronize(policy);

  const auto wall =
    one_contact("robot::base::collision", "wall::link::collision", 2.0);
  const auto support =
    one_contact("ground::link::collision", "robot::wheel::collision", 3.0);
  EXPECT_FALSE(policy.observe(1002000000LL, wall).output.has_value());
  EXPECT_FALSE(
      policy.observe(1004000000LL, gz::msgs::Contacts()).output.has_value());
  EXPECT_FALSE(policy.observe(1018000000LL, support).output.has_value());

  const auto completed = policy.observe(1020000000LL, gz::msgs::Contacts());
  ASSERT_TRUE(completed.output.has_value());
  ASSERT_EQ(completed.output->contact_size(), 2);
  EXPECT_EQ(normalized_pair(completed.output->contact(0)),
            Pair("ground::link::collision", "robot::wheel::collision"));
  EXPECT_EQ(normalized_pair(completed.output->contact(1)),
            Pair("robot::base::collision", "wall::link::collision"));
}

TEST(ContactAggregatorPolicy, EmptyIntervalAfterStreamStartFailsClosed) {
  ContactAggregatorPolicy policy;
  synchronize(policy);
  const auto support =
    one_contact("ground::link::collision", "robot::wheel::collision");
  EXPECT_FALSE(policy.observe(1002000000LL, support).fatal);
  ASSERT_TRUE(
    policy.observe(1020000000LL, gz::msgs::Contacts()).output.has_value());

  const auto empty = policy.observe(1040000000LL, gz::msgs::Contacts());
  EXPECT_TRUE(empty.fatal);
  EXPECT_FALSE(empty.output.has_value());
  EXPECT_NE(empty.detail.find("became empty"), std::string::npos);
}

TEST(ContactAggregatorPolicy,
     LatestCompleteStepGroupReplacesEarlierGroupDeterministically) {
  ContactAggregatorPolicy policy;
  synchronize(policy);
  const std::string first = "robot::base::collision";
  const std::string second = "wall::link::collision";

  EXPECT_FALSE(policy
    .observe(1002000000LL,
                            contacts_for({contact_for(first, second, 1.0)}))
    .output.has_value());
  EXPECT_FALSE(policy
    .observe(1010000000LL,
                            contacts_for({contact_for(first, second, 7.0),
      contact_for(second, first, 8.0)}))
    .output.has_value());

  const auto completed = policy.observe(1020000000LL, gz::msgs::Contacts());
  ASSERT_TRUE(completed.output.has_value());
  ASSERT_EQ(completed.output->contact_size(), 2);
  EXPECT_DOUBLE_EQ(completed.output->contact(0).position(0).x(), 7.0);
  EXPECT_DOUBLE_EQ(completed.output->contact(1).position(0).x(), 8.0);
  EXPECT_EQ(completed.output->contact(0).collision1().name(), first);
  EXPECT_EQ(completed.output->contact(1).collision1().name(), second);
}

TEST(ContactAggregatorPolicy,
     MultiStepSerializedOutputAndFatalDetailMatchReferenceContract) {
  ContactAggregatorPolicy policy;
  synchronize(policy);

  EXPECT_FALSE(policy.observe(
      1002000000LL,
      contacts_for({
      contact_for("z::collision", "y::collision", 1.0),
      contact_for("b::collision", "a::collision", 2.0),
      })).fatal);
  EXPECT_FALSE(policy.observe(
      1004000000LL,
      contacts_for({
      contact_for("z::collision", "y::collision", 7.0),
      contact_for("y::collision", "z::collision", 8.0),
      contact_for("c::collision", "d::collision", 3.0),
      })).fatal);
  EXPECT_FALSE(policy.observe(1018000000LL, gz::msgs::Contacts()).fatal);
  const auto completed = policy.observe(
    1020000000LL, one_contact("m::collision", "n::collision", 4.0));
  ASSERT_TRUE(completed.output.has_value());

  auto expected = contacts_for({
      contact_for("b::collision", "a::collision", 2.0),
      contact_for("c::collision", "d::collision", 3.0),
      contact_for("m::collision", "n::collision", 4.0),
      contact_for("z::collision", "y::collision", 7.0),
      contact_for("y::collision", "z::collision", 8.0),
    });
  set_stamp(*expected.mutable_header(), 1020000000LL);
  for (auto & contact : *expected.mutable_contact()) {
    set_stamp(*contact.mutable_header(), 1020000000LL);
  }
  EXPECT_EQ(completed.output->SerializeAsString(), expected.SerializeAsString());

  std::vector<gz::msgs::Contact> sixteen_pairs;
  for (int index = 0; index < 16; ++index) {
    sixteen_pairs.push_back(contact_for(
        "robot::collision_" + std::to_string(index),
        "wall::collision_" + std::to_string(index)));
  }
  EXPECT_FALSE(policy.observe(1022000000LL, contacts_for(sixteen_pairs)).fatal);
  const auto overflow = policy.observe(
    1024000000LL,
    one_contact("robot::collision_16", "wall::collision_16"));
  EXPECT_TRUE(overflow.fatal);
  EXPECT_FALSE(overflow.output.has_value());
  EXPECT_EQ(
    overflow.detail,
    "contact aggregate interval union exceeds the 16-record bound");
  EXPECT_EQ(policy.observe(1026000000LL, gz::msgs::Contacts()).detail,
            overflow.detail);
}

TEST(ContactAggregatorPolicy,
     CanonicalPairBlocksPreserveWithinStepDuplicateOrder) {
  ContactAggregatorPolicy policy;
  synchronize(policy);
  const auto unordered = contacts_for({
      contact_for("z::link::collision", "y::link::collision", 1.0),
      contact_for("b::link::collision", "a::link::collision", 2.0),
      contact_for("y::link::collision", "z::link::collision", 3.0),
  });

  EXPECT_FALSE(policy.observe(1002000000LL, unordered).fatal);
  const auto completed = policy.observe(1020000000LL, gz::msgs::Contacts());
  ASSERT_TRUE(completed.output.has_value());
  ASSERT_EQ(completed.output->contact_size(), 3);
  EXPECT_EQ(normalized_pair(completed.output->contact(0)),
            Pair("a::link::collision", "b::link::collision"));
  EXPECT_EQ(normalized_pair(completed.output->contact(1)),
            Pair("y::link::collision", "z::link::collision"));
  EXPECT_EQ(normalized_pair(completed.output->contact(2)),
            Pair("y::link::collision", "z::link::collision"));
  EXPECT_DOUBLE_EQ(completed.output->contact(1).position(0).x(), 1.0);
  EXPECT_DOUBLE_EQ(completed.output->contact(2).position(0).x(), 3.0);
}

TEST(ContactAggregatorPolicy,
     BoundaryAndProtobufOnlyFieldsCanonicalizeToTheRosProjection) {
  ContactAggregatorPolicy policy;
  synchronize(policy);
  auto contact = contact_for("robot::base::collision", "wall::link::collision");
  contact.mutable_header()->mutable_stamp()->set_sec(41);
  auto *contact_metadata = contact.mutable_header()->add_data();
  contact_metadata->set_key("source");
  contact_metadata->add_value(std::string(9000, 'c'));
  contact.mutable_collision1()->set_id(11U);
  contact.mutable_collision1()->set_type(gz::msgs::Entity::COLLISION);
  auto *first_entity_metadata =
    contact.mutable_collision1()->mutable_header()->add_data();
  first_entity_metadata->set_key("source");
  first_entity_metadata->add_value(std::string(9000, 'e'));
  contact.mutable_collision2()->set_id(12U);
  contact.mutable_collision2()->set_type(gz::msgs::Entity::COLLISION);
  contact.mutable_world()->set_name(std::string(9000, 'w'));
  auto *position_metadata = contact.mutable_position(0)->mutable_header()->add_data();
  position_metadata->set_key("source");
  position_metadata->add_value(std::string(9000, 'p'));
  auto *normal = contact.add_normal();
  normal->set_z(1.0);
  auto *normal_metadata = normal->mutable_header()->add_data();
  normal_metadata->set_key("source");
  normal_metadata->add_value(std::string(9000, 'n'));
  contact.add_depth(0.25);
  auto *wrench = contact.add_wrench();
  auto *source_stamp = wrench->mutable_header()->mutable_stamp();
  source_stamp->set_sec(5);
  source_stamp->set_nsec(17);
  source_stamp->GetReflection()->MutableUnknownFields(source_stamp)
  ->AddLengthDelimited(19000, std::string(9000, 't'));
  ASSERT_EQ(
    source_stamp->GetReflection()->GetUnknownFields(*source_stamp).field_count(),
    1);
  auto *frame = wrench->mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value("joint_frame");
  wrench->set_body_1_name("robot");
  wrench->set_body_1_id(21U);
  wrench->set_body_2_name("wall");
  wrench->set_body_2_id(22U);
  wrench->mutable_body_1_wrench()->mutable_force()->set_x(3.5);
  wrench->mutable_body_1_wrench()->mutable_force_offset()->set_x(
    std::numeric_limits<double>::quiet_NaN());
  auto *inner_wrench_metadata =
    wrench->mutable_body_1_wrench()->mutable_header()->add_data();
  inner_wrench_metadata->set_key("source");
  inner_wrench_metadata->add_value(std::string(9000, 'i'));
  wrench->mutable_body_2_wrench()->mutable_torque()->set_z(-2.5);

  SourceMessages source_messages;
  *source_messages[2].add_contact() = contact;
  const auto sources = source_views(source_messages);
  EXPECT_FALSE(policy.observe(1002000000LL, sources).fatal);
  const auto completed = policy.observe(1020000000LL, gz::msgs::Contacts());
  ASSERT_TRUE(completed.output.has_value());
  ASSERT_EQ(completed.output->contact_size(), 1);
  const auto & output = completed.output->contact(0);
  EXPECT_EQ(stamp_ns(completed.output->header()), 1020000000LL);
  EXPECT_EQ(completed.output->header().data_size(), 0);
  EXPECT_EQ(stamp_ns(output.header()), 1020000000LL);
  EXPECT_EQ(output.header().data_size(), 0);
  EXPECT_EQ(output.collision1().id(), 11U);
  EXPECT_EQ(output.collision1().type(), gz::msgs::Entity::COLLISION);
  EXPECT_EQ(output.collision1().header().data_size(), 0);
  EXPECT_EQ(output.collision2().id(), 12U);
  EXPECT_EQ(output.collision2().type(), gz::msgs::Entity::COLLISION);
  EXPECT_EQ(output.collision2().header().data_size(), 0);
  EXPECT_FALSE(output.has_world());
  EXPECT_EQ(output.position(0).header().data_size(), 0);
  EXPECT_EQ(output.normal(0).header().data_size(), 0);
  ASSERT_EQ(output.wrench_size(), 1);
  EXPECT_EQ(stamp_ns(output.wrench(0).header()), 5000000017LL);
  EXPECT_EQ(
    output.wrench(0).header().stamp().GetReflection()->GetUnknownFields(
      output.wrench(0).header().stamp()).field_count(),
    0);
  ASSERT_EQ(output.wrench(0).header().data_size(), 1);
  EXPECT_EQ(output.wrench(0).header().data(0).key(), "frame_id");
  ASSERT_EQ(output.wrench(0).header().data(0).value_size(), 1);
  EXPECT_EQ(output.wrench(0).header().data(0).value(0), "joint_frame");
  EXPECT_EQ(output.wrench(0).body_1_name(), "robot");
  EXPECT_EQ(output.wrench(0).body_1_id(), 21U);
  EXPECT_EQ(output.wrench(0).body_2_id(), 22U);
  EXPECT_DOUBLE_EQ(output.wrench(0).body_1_wrench().force().x(), 3.5);
  EXPECT_EQ(output.wrench(0).body_1_wrench().header().data_size(), 0);
  EXPECT_DOUBLE_EQ(
    output.wrench(0).body_1_wrench().force_offset().x(), 0.0);
  EXPECT_DOUBLE_EQ(output.wrench(0).body_2_wrench().torque().z(), -2.5);
}

TEST(ContactAggregatorPolicy,
     MultiSourceGroupsPreserveBindingThenRepeatedFieldOrder) {
  ContactAggregatorPolicy policy;
  synchronize(policy);
  SourceMessages source_messages;
  const std::string first = "robot::base::collision";
  const std::string second = "wall::link::collision";
  *source_messages[0].add_contact() = contact_for(first, second, 1.0);
  *source_messages[0].add_contact() = contact_for(second, first, 2.0);
  *source_messages[1].add_contact() = contact_for(first, second, 3.0);
  *source_messages[1].add_contact() = contact_for(second, first, 4.0);

  EXPECT_FALSE(policy.observe(1002000000LL, source_views(source_messages)).fatal);
  SourceMessages empty_sources;
  const auto completed =
    policy.observe(1020000000LL, source_views(empty_sources));
  ASSERT_TRUE(completed.output.has_value());
  ASSERT_EQ(completed.output->contact_size(), 4);
  for (int index = 0; index < 4; ++index) {
    EXPECT_DOUBLE_EQ(
      completed.output->contact(index).position(0).x(), index + 1.0);
  }
}

TEST(ContactAggregatorPolicy,
     MultiSourceGlobalRecordBoundAcceptsSixteenAndRejectsSeventeen) {
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    SourceMessages source_messages;
    for (int index = 0; index < 9; ++index) {
      *source_messages[0].add_contact() = contact_for(
        "robot::link::collision_" + std::to_string(index),
        "wall::link::collision_" + std::to_string(index));
    }
    for (int index = 9; index < 16; ++index) {
      *source_messages[1].add_contact() = contact_for(
        "robot::link::collision_" + std::to_string(index),
        "wall::link::collision_" + std::to_string(index));
    }
    EXPECT_FALSE(
      policy.observe(1002000000LL, source_views(source_messages)).fatal);
    SourceMessages empty_sources;
    const auto completed =
      policy.observe(1020000000LL, source_views(empty_sources));
    ASSERT_TRUE(completed.output.has_value());
    EXPECT_EQ(completed.output->contact_size(), 16);
  }
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    SourceMessages source_messages;
    for (int index = 0; index < 9; ++index) {
      *source_messages[0].add_contact() = contact_for(
        "robot::link::collision_" + std::to_string(index),
        "wall::link::collision_" + std::to_string(index));
    }
    for (int index = 9; index < 17; ++index) {
      *source_messages[1].add_contact() = contact_for(
        "robot::link::collision_" + std::to_string(index),
        "wall::link::collision_" + std::to_string(index));
    }
    const auto overflow =
      policy.observe(1002000000LL, source_views(source_messages));
    EXPECT_TRUE(overflow.fatal);
    EXPECT_FALSE(overflow.output.has_value());
    EXPECT_NE(overflow.detail.find("16-record"), std::string::npos);
  }
}

TEST(ContactAggregatorPolicy, MultiSourceNullPointerFailsClosed) {
  ContactAggregatorPolicy policy;
  synchronize(policy);
  SourceMessages source_messages;
  auto sources = source_views(source_messages);
  sources[6] = nullptr;

  const auto decision = policy.observe(1002000000LL, sources);
  EXPECT_TRUE(decision.fatal);
  EXPECT_FALSE(decision.output.has_value());
  EXPECT_NE(decision.detail.find("nonnull"), std::string::npos);
}

TEST(ContactAggregatorPolicy, BoundaryStepBelongsToTheRightClosedInterval) {
  ContactAggregatorPolicy policy;
  synchronize(policy);
  const auto contact =
    one_contact("robot::base::collision", "wall::link::collision");

  const auto completed = policy.observe(1020000000LL, contact);
  ASSERT_TRUE(completed.output.has_value());
  ASSERT_EQ(completed.output->contact_size(), 1);
  EXPECT_EQ(stamp_ns(completed.output->contact(0).header()), 1020000000LL);
}

TEST(ContactAggregatorPolicy,
     RegressionAndSkippedGridBoundaryLatchFatalUntilReset) {
  ContactAggregatorPolicy policy;
  EXPECT_FALSE(policy.observe(1001000000LL, gz::msgs::Contacts()).fatal);
  EXPECT_FALSE(policy.observe(1020000000LL, gz::msgs::Contacts()).fatal);
  EXPECT_TRUE(policy.observe(1040000001LL, gz::msgs::Contacts()).fatal);
  EXPECT_TRUE(policy.observe(1060000000LL, gz::msgs::Contacts()).fatal);

  policy.reset();
  EXPECT_FALSE(policy.observe(0, gz::msgs::Contacts()).fatal);
  EXPECT_TRUE(policy.observe(0, gz::msgs::Contacts()).fatal);
  policy.reset();
  EXPECT_FALSE(policy.observe(20000000LL, gz::msgs::Contacts()).fatal);
  EXPECT_FALSE(policy.observe(40000000LL, gz::msgs::Contacts()).fatal);
}

TEST(ContactAggregatorPolicy, CurrentStepAndPerPairRecordBoundsFailClosed) {
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    std::vector<gz::msgs::Contact> records;
    for (int index = 0; index < 17; ++index) {
      records.push_back(
          contact_for("robot::link::collision_" + std::to_string(index),
                      "wall::link::collision_" + std::to_string(index)));
    }
    EXPECT_TRUE(policy.observe(1002000000LL, contacts_for(records)).fatal);
  }
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    std::vector<gz::msgs::Contact> records;
    for (int index = 0; index < 5; ++index) {
      records.push_back(contact_for("robot::link::collision",
                                    "wall::link::collision", index));
    }
    EXPECT_TRUE(policy.observe(1002000000LL, contacts_for(records)).fatal);
  }
}

TEST(ContactAggregatorPolicy,
     IntervalUnionOverflowFailsInsteadOfDroppingTransientPairs) {
  ContactAggregatorPolicy policy;
  synchronize(policy);
  std::vector<gz::msgs::Contact> first_step;
  for (int index = 0; index < 16; ++index) {
    first_step.push_back(
        contact_for("robot::link::collision_" + std::to_string(index),
                    "wall::link::collision_" + std::to_string(index)));
  }
  EXPECT_FALSE(policy.observe(1002000000LL, contacts_for(first_step)).fatal);
  const auto seventeenth =
    one_contact("robot::link::collision_16", "wall::link::collision_16");
  const auto overflow = policy.observe(1004000000LL, seventeenth);
  EXPECT_TRUE(overflow.fatal);
  EXPECT_FALSE(overflow.output.has_value());
  EXPECT_NE(overflow.detail.find("interval union"), std::string::npos);

  const auto latched = policy.observe(1006000000LL, gz::msgs::Contacts());
  EXPECT_TRUE(latched.fatal);
  EXPECT_EQ(latched.detail, overflow.detail);

  policy.reset();
  synchronize(policy, 2000000000LL);
  const auto fresh =
    one_contact("ground::link::collision", "robot::wheel::collision", 19.0);
  EXPECT_FALSE(policy.observe(2002000000LL, fresh).fatal);
  const auto completed = policy.observe(2020000000LL, gz::msgs::Contacts());
  ASSERT_TRUE(completed.output.has_value());
  ASSERT_EQ(completed.output->contact_size(), 1);
  EXPECT_DOUBLE_EQ(completed.output->contact(0).position(0).x(), 19.0);
}

TEST(ContactAggregatorPolicy,
     StructuralPointAlignmentAndFiniteBoundsFailClosed) {
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    auto contact =
      contact_for("robot::link::collision", "wall::link::collision");
    contact.clear_position();
    EXPECT_TRUE(policy.observe(1002000000LL, contacts_for({contact})).fatal);
  }
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    auto contact =
      contact_for("robot::link::collision", "wall::link::collision");
    for (int index = 1; index < 65; ++index) {
      contact.add_position()->set_x(index);
    }
    EXPECT_TRUE(policy.observe(1002000000LL, contacts_for({contact})).fatal);
  }
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    auto contact =
      contact_for("robot::link::collision", "wall::link::collision");
    contact.add_normal()->set_z(1.0);
    EXPECT_TRUE(policy.observe(1002000000LL, contacts_for({contact})).fatal);
  }
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    auto contact =
      contact_for("robot::link::collision", "wall::link::collision");
    contact.mutable_position(0)->set_x(std::numeric_limits<double>::infinity());
    EXPECT_TRUE(policy.observe(1002000000LL, contacts_for({contact})).fatal);
  }
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    auto contact =
      contact_for("robot::link::collision", "wall::link::collision");
    contact.add_normal()->set_z(1.0);
    contact.add_depth(-0.01);
    contact.add_wrench();
    EXPECT_TRUE(policy.observe(1002000000LL, contacts_for({contact})).fatal);
  }
}

TEST(ContactAggregatorPolicy, CollisionWrenchAndStringBoundsFailClosed) {
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    EXPECT_TRUE(
        policy.observe(1002000000LL, one_contact("", "wall::link::collision"))
      .fatal);
  }
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    EXPECT_TRUE(policy
      .observe(1002000000LL, one_contact(std::string(4097, 'r'),
                                                       "wall::link::collision"))
      .fatal);
  }
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    auto contact =
      contact_for("robot::link::collision", "wall::link::collision");
    contact.add_normal()->set_z(1.0);
    contact.add_depth(0.01);
    auto *wrench = contact.add_wrench();
    wrench->mutable_header()->mutable_stamp()->set_nsec(1000000000);
    EXPECT_TRUE(policy.observe(1002000000LL, contacts_for({contact})).fatal);
  }
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    auto contact =
      contact_for("robot::link::collision", "wall::link::collision");
    contact.add_normal()->set_z(1.0);
    contact.add_depth(0.01);
    auto *wrench = contact.add_wrench();
    auto *frame = wrench->mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(std::string(257, 'f'));
    EXPECT_TRUE(policy.observe(1002000000LL, contacts_for({contact})).fatal);
  }
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    auto contact =
      contact_for("robot::link::collision", "wall::link::collision");
    contact.add_normal()->set_z(1.0);
    contact.add_depth(0.01);
    auto *wrench = contact.add_wrench();
    wrench->set_body_1_name(std::string(4097, 'b'));
    EXPECT_TRUE(policy.observe(1002000000LL, contacts_for({contact})).fatal);
  }
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    auto contact =
      contact_for("robot::link::collision", "wall::link::collision");
    contact.add_normal()->set_z(1.0);
    contact.add_depth(0.01);
    auto *wrench = contact.add_wrench();
    wrench->mutable_body_1_wrench()->mutable_force()->set_x(
        std::numeric_limits<double>::quiet_NaN());
    EXPECT_TRUE(policy.observe(1002000000LL, contacts_for({contact})).fatal);
  }
}

TEST(ContactAggregatorPolicy,
     WrenchHeaderDataGrammarAndFixedKeyBytesFailClosed) {
  const auto contact_with_wrench = []() {
      auto contact = contact_for("a", "b");
      contact.add_normal()->set_z(1.0);
      contact.add_depth(0.01);
      contact.add_wrench();
      return contact;
    };
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    auto contact = contact_with_wrench();
    auto *datum = contact.mutable_wrench(0)->mutable_header()->add_data();
    datum->set_key("source");
    datum->add_value("sensor");
    const auto decision =
      policy.observe(1002000000LL, contacts_for({contact}));
    EXPECT_TRUE(decision.fatal);
    EXPECT_NE(decision.detail.find("frame_id-only"), std::string::npos);
  }
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    auto contact = contact_with_wrench();
    for (int index = 0; index < 2; ++index) {
      auto *datum = contact.mutable_wrench(0)->mutable_header()->add_data();
      datum->set_key("frame_id");
      datum->add_value("frame");
    }
    EXPECT_TRUE(
      policy.observe(1002000000LL, contacts_for({contact})).fatal);
  }
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    auto contact = contact_with_wrench();
    auto *datum = contact.mutable_wrench(0)->mutable_header()->add_data();
    datum->set_key("frame_id");
    datum->add_value("frame_a");
    datum->add_value("frame_b");
    EXPECT_TRUE(
      policy.observe(1002000000LL, contacts_for({contact})).fatal);
  }

  const auto contact_at_pair_string_bound = [](const std::size_t body_name_bytes) {
      auto contact = contact_for("a", "b");
      for (int index = 0; index < 31; ++index) {
        if (index > 0) {
          contact.add_position();
        }
        contact.add_normal()->set_z(1.0);
        contact.add_depth(0.01);
        auto *wrench = contact.add_wrench();
        auto *frame = wrench->mutable_header()->add_data();
        frame->set_key("frame_id");
        frame->add_value(std::string(256, 'f'));
        if (index == 0) {
          wrench->set_body_1_name(std::string(body_name_bytes, 'b'));
        }
      }
      return contact;
    };
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    const auto exact = policy.observe(
      1002000000LL, contacts_for({contact_at_pair_string_bound(6U)}));
    EXPECT_FALSE(exact.fatal);
  }
  {
    ContactAggregatorPolicy policy;
    synchronize(policy);
    const auto overflow = policy.observe(
      1002000000LL, contacts_for({contact_at_pair_string_bound(7U)}));
    EXPECT_TRUE(overflow.fatal);
    EXPECT_NE(overflow.detail.find("8192-byte"), std::string::npos);
  }
}

TEST(ContactAggregatorPolicy, AggregateStringBudgetFailsWithoutTruncation) {
  ContactAggregatorPolicy policy;
  synchronize(policy);
  std::vector<gz::msgs::Contact> records;
  for (int index = 0; index < 5; ++index) {
    auto first = std::string(3998, static_cast<char>('a' + index));
    auto second = std::string(3998, static_cast<char>('k' + index));
    first += std::to_string(index);
    second += std::to_string(index);
    records.push_back(contact_for(first, second));
  }
  const auto overflow = policy.observe(1002000000LL, contacts_for(records));
  EXPECT_TRUE(overflow.fatal);
  EXPECT_NE(overflow.detail.find("65536-byte"), std::string::npos);
}

}  // namespace robotest_sim

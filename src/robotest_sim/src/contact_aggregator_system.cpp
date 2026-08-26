// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iterator>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <sdf/Element.hh>

#include "gz/common/Console.hh"
#include "gz/plugin/Register.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Events.hh"
#include "gz/sim/System.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/ContactSensorData.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/World.hh"
#include "gz/transport/Node.hh"
#include "robotest_sim/contact_aggregator.hpp"

namespace robotest_sim
{
namespace
{

#ifndef ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256
#error "contact pipeline source inventory digest was not supplied by CMake"
#endif

constexpr char kSourceInventoryIdentity[] =
  "ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_"
  "SHA256=" ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256;
static_assert(sizeof(kSourceInventoryIdentity) == 111U);

constexpr char kRobotModelName[] = "robotest";
constexpr char kOutputTopic[] = "/robotest/internal/contact_aggregate";
constexpr std::size_t kMaxEntityAncestryDepth = 64U;

struct ExpectedSensorBinding
{
  const char *sensor_name;
  const char *link_name;
  const char *collision_name;
};

constexpr std::array<ExpectedSensorBinding, kContactAggregateSourceCount>
kExpectedBindings = {{
  {
    "chassis_contact_sensor",
    "base_footprint",
    "base_footprint_fixed_joint_lump__base_link_collision_collision",
  },
  {
    "front_caster_contact_sensor",
    "front_caster_link",
    "front_caster_link_fixed_joint_lump__front_caster_collision_collision",
  },
  {
    "imu_body_contact_sensor",
    "imu_link",
    "imu_link_collision_collision",
  },
  {
    "left_wheel_contact_sensor",
    "left_wheel_link",
    "left_wheel_link_fixed_joint_lump__left_wheel_collision_collision",
  },
  {
    "lidar_body_contact_sensor",
    "lidar_link",
    "lidar_link_collision_collision",
  },
  {
    "rear_caster_contact_sensor",
    "rear_caster_link",
    "rear_caster_link_fixed_joint_lump__rear_caster_collision_collision",
  },
  {
    "right_wheel_contact_sensor",
    "right_wheel_link",
    "right_wheel_link_fixed_joint_lump__right_wheel_collision_collision",
  },
}};

std::optional<std::size_t>
expected_binding_index(const std::string & sensor_name)
{
  const auto iterator =
    std::find_if(kExpectedBindings.begin(), kExpectedBindings.end(),
      [&sensor_name](const auto & expected) {
        return sensor_name == expected.sensor_name;
                   });
  if (iterator == kExpectedBindings.end()) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(
    std::distance(kExpectedBindings.begin(), iterator));
}

gz::sim::Entity
top_level_model_ancestor(
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

}  // namespace

class ContactAggregatorSystem final : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate,
  public gz::sim::ISystemReset {
public:
  void Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager & event_manager) override
  {
    if (!sdf || !sdf->HasElement("output_topic") ||
      !sdf->HasElement("publish_period_ns") ||
      !sdf->HasElement("robot_model_name"))
    {
      throw std::runtime_error(
              "contact aggregator requires output_topic, publish_period_ns, "
              "and robot_model_name");
    }
    const auto output_topic = sdf->Get<std::string>("output_topic");
    const auto publish_period_ns = sdf->Get<std::uint64_t>("publish_period_ns");
    const auto robot_model_name = sdf->Get<std::string>("robot_model_name");
    if (output_topic != kOutputTopic ||
      publish_period_ns !=
      static_cast<std::uint64_t>(kContactAggregatePeriodNs) ||
      robot_model_name != kRobotModelName)
    {
      throw std::runtime_error(
          "contact aggregator configuration differs from the frozen contract");
    }

    event_manager_ = &event_manager;
    publisher_ = transport_node_.Advertise<gz::msgs::Contacts>(kOutputTopic);
    if (!publisher_) {
      throw std::runtime_error(
          "contact aggregator could not advertise its output topic");
    }
    gzmsg << kSourceInventoryIdentity << std::endl;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (fatal_latched_) {
      emit_stop_once();
      return;
    }

    try {
      discover_and_validate_bindings(ecm);
    } catch (const std::exception & error) {
      latch_fatal(std::string("contact aggregator discovery failed: ") +
                  error.what());
    } catch (...) {
      latch_fatal(
          "contact aggregator discovery failed with an unknown exception");
    }
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || fatal_latched_ || !bindings_locked_) {
      return;
    }

    try {
      if (!ecm.HasEntity(robot_model_entity_)) {
        latch_fatal(
            "locked robotest model disappeared before contact aggregation");
        return;
      }

      ContactAggregateSources current_sources{};
      for (std::size_t index = 0U; index < bindings_.size(); ++index) {
        const auto & binding = bindings_[index];
        if (!ecm.HasEntity(binding.sensor_entity) ||
          !ecm.HasEntity(binding.collision_entity))
        {
          latch_fatal(
              std::string("locked contact binding disappeared for sensor ") +
              kExpectedBindings[index].sensor_name);
          return;
        }
        const auto *sensor = ecm.Component<gz::sim::components::ContactSensor>(
            binding.sensor_entity);
        const auto *contacts =
          ecm.Component<gz::sim::components::ContactSensorData>(
                binding.collision_entity);
        if (sensor == nullptr || contacts == nullptr) {
          latch_fatal(
              std::string("locked contact component disappeared for sensor ") +
              kExpectedBindings[index].sensor_name);
          return;
        }
        current_sources[index] = &contacts->Data();
      }

      const auto stamp_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime)
        .count();
      auto decision = policy_.observe(stamp_ns, current_sources);
      if (decision.fatal) {
        latch_fatal(std::move(decision.detail));
        return;
      }
      if (decision.output.has_value() &&
        !publisher_.Publish(*decision.output))
      {
        latch_fatal("contact aggregator failed to publish an aggregate sample");
      }
    } catch (const std::exception & error) {
      latch_fatal(std::string("contact aggregator post-update failed: ") +
                  error.what());
    } catch (...) {
      latch_fatal(
          "contact aggregator post-update failed with an unknown exception");
    }
  }

  void Reset(
    const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &) override
  {
    policy_.reset();
  }

private:
  struct ContactBinding
  {
    gz::sim::Entity sensor_entity{gz::sim::kNullEntity};
    gz::sim::Entity collision_entity{gz::sim::kNullEntity};
  };

  using DiscoveredBindings =
    std::array<std::optional<ContactBinding>, kExpectedBindings.size()>;

  void latch_fatal(std::string detail)
  {
    if (fatal_latched_) {
      return;
    }
    fatal_latched_ = true;
    fatal_detail_ = std::move(detail);
    gzerr << "Contact aggregator fatal: " << fatal_detail_ << std::endl;
  }

  void emit_stop_once()
  {
    if (stop_emitted_) {
      return;
    }
    stop_emitted_ = true;
    if (event_manager_ == nullptr) {
      gzerr << "Contact aggregator cannot emit Stop without an EventManager"
            << std::endl;
      return;
    }
    event_manager_->Emit<gz::sim::events::Stop>();
  }

  void discover_and_validate_bindings(gz::sim::EntityComponentManager & ecm)
  {
    const auto & readonly_ecm =
      static_cast<const gz::sim::EntityComponentManager &>(ecm);

    std::vector<gz::sim::Entity> robot_models;
    readonly_ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
      [&readonly_ecm, &robot_models](const gz::sim::Entity & entity,
      const gz::sim::components::Model *,
      const gz::sim::components::Name *name) {
        const auto parent = readonly_ecm.ParentEntity(entity);
        if (name->Data() == kRobotModelName &&
        parent != gz::sim::kNullEntity &&
        readonly_ecm.Component<gz::sim::components::World>(parent) !=
        nullptr)
        {
          robot_models.push_back(entity);
        }
        return true;
        });

    if (robot_models.size() > 1U) {
      latch_fatal("multiple top-level robotest models expose ambiguous contact "
                  "sources");
      return;
    }
    const auto discovered_model =
      robot_models.empty() ? gz::sim::kNullEntity : robot_models.front();
    if (bindings_locked_ && discovered_model != robot_model_entity_) {
      latch_fatal("locked top-level robotest model disappeared or rebound");
      return;
    }

    DiscoveredBindings discovered;
    std::set<gz::sim::Entity> collision_entities;
    std::optional<std::string> discovery_error;
    readonly_ecm.Each<
      gz::sim::components::ContactSensor,
      gz::sim::components::Name>([&readonly_ecm, discovered_model,
      &discovered, &collision_entities,
      &discovery_error](
        const gz::sim::Entity & sensor_entity,
        const gz::sim::components::ContactSensor
        *sensor,
        const gz::sim::components::Name
        *sensor_name) {
        if (discovery_error.has_value()) {
          return false;
        }

        const auto expected_index = expected_binding_index(sensor_name->Data());
        const auto top_model =
        top_level_model_ancestor(sensor_entity, readonly_ecm);
        const bool under_robotest = discovered_model != gz::sim::kNullEntity &&
        top_model == discovered_model;
        if (!expected_index.has_value()) {
          if (under_robotest) {
            discovery_error =
            "unexpected contact sensor under top-level robotest model: " +
            sensor_name->Data();
          }
          return !discovery_error.has_value();
        }
        if (!under_robotest) {
          discovery_error = "expected-name contact sensor exists outside the "
          "top-level robotest model: " +
          sensor_name->Data();
          return false;
        }
        if (discovered[*expected_index].has_value()) {
          discovery_error =
          "duplicate robotest contact sensor: " + sensor_name->Data();
          return false;
        }

        const auto & expected = kExpectedBindings[*expected_index];
        const auto link_entity = readonly_ecm.ParentEntity(sensor_entity);
        const auto *link =
        readonly_ecm.Component<gz::sim::components::Link>(link_entity);
        const auto *link_name =
        readonly_ecm.Component<gz::sim::components::Name>(link_entity);
        if (link == nullptr || link_name == nullptr ||
        link_name->Data() != expected.link_name ||
        readonly_ecm.ParentEntity(link_entity) != discovered_model ||
        top_level_model_ancestor(link_entity, readonly_ecm) !=
        discovered_model)
        {
          discovery_error =
          std::string("contact sensor has the wrong link ancestry: ") +
          expected.sensor_name;
          return false;
        }

        const auto sensor_sdf = sensor->Data();
        if (!sensor_sdf || !sensor_sdf->HasElement("contact")) {
          discovery_error =
          std::string("contact sensor lacks contact configuration: ") +
          expected.sensor_name;
          return false;
        }
        const auto contact_element = sensor_sdf->GetElement("contact");
        if (!contact_element || !contact_element->HasElement("collision")) {
          discovery_error =
          std::string("contact sensor lacks a collision mapping: ") +
          expected.sensor_name;
          return false;
        }
        const auto collision_element = contact_element->GetElement("collision");
        if (!collision_element ||
        collision_element->GetNextElement("collision") != nullptr ||
        collision_element->Get<std::string>() != expected.collision_name)
        {
          discovery_error =
          std::string(
                "contact sensor collision mapping differs from contract: ") +
          expected.sensor_name;
          return false;
        }

        const auto matches = readonly_ecm.ChildrenByComponents(
          link_entity, gz::sim::components::Collision(),
          gz::sim::components::Name(expected.collision_name));
        if (matches.size() != 1U ||
        top_level_model_ancestor(matches.front(), readonly_ecm) !=
        discovered_model)
        {
          discovery_error =
          std::string(
                "contact sensor collision entity is missing or ambiguous: ") +
          expected.sensor_name;
          return false;
        }
        if (!collision_entities.insert(matches.front()).second) {
          discovery_error =
          "multiple robotest contact sensors bind one collision entity";
          return false;
        }

        discovered[*expected_index] =
        ContactBinding{sensor_entity, matches.front()};
        return true;
    });

    if (discovery_error.has_value()) {
      latch_fatal(std::move(*discovery_error));
      return;
    }

    const auto discovered_count = static_cast<std::size_t>(
      std::count_if(discovered.begin(), discovered.end(),
      [](const auto & binding) {return binding.has_value();}));
    if (bindings_locked_ && discovered_count != kExpectedBindings.size()) {
      latch_fatal("one or more locked robotest contact sensors disappeared");
      return;
    }
    if (discovered_count != kExpectedBindings.size()) {
      return;
    }

    for (std::size_t index = 0U; index < discovered.size(); ++index) {
      const auto binding = *discovered[index];
      if (bindings_locked_ &&
        (bindings_[index].sensor_entity != binding.sensor_entity ||
        bindings_[index].collision_entity != binding.collision_entity))
      {
        latch_fatal(std::string("locked contact source rebound for sensor ") +
                    kExpectedBindings[index].sensor_name);
        return;
      }
      auto *contact_data =
        ecm.Component<gz::sim::components::ContactSensorData>(
              binding.collision_entity);
      if (contact_data == nullptr) {
        if (bindings_locked_) {
          latch_fatal(
            std::string("locked contact data disappeared for sensor ") +
            kExpectedBindings[index].sensor_name);
          return;
        }
        contact_data = ecm.CreateComponent(
            binding.collision_entity, gz::sim::components::ContactSensorData());
      }
      if (contact_data == nullptr) {
        latch_fatal(std::string("cannot create contact data for sensor ") +
                    kExpectedBindings[index].sensor_name);
        return;
      }
    }

    if (!bindings_locked_) {
      for (std::size_t index = 0U; index < discovered.size(); ++index) {
        bindings_[index] = *discovered[index];
      }
      robot_model_entity_ = discovered_model;
      bindings_locked_ = true;
      gzmsg << "Contact aggregator locked all seven robotest sensor bindings"
            << std::endl;
    }
  }

  ContactAggregatorPolicy policy_;
  gz::transport::Node transport_node_;
  gz::transport::Node::Publisher publisher_;
  gz::sim::EventManager *event_manager_{nullptr};
  std::array<ContactBinding, kExpectedBindings.size()> bindings_{};
  gz::sim::Entity robot_model_entity_{gz::sim::kNullEntity};
  bool bindings_locked_{false};
  bool fatal_latched_{false};
  bool stop_emitted_{false};
  std::string fatal_detail_;
};

}  // namespace robotest_sim

GZ_ADD_PLUGIN(robotest_sim::ContactAggregatorSystem, gz::sim::System,
              robotest_sim::ContactAggregatorSystem::ISystemConfigure,
              robotest_sim::ContactAggregatorSystem::ISystemPreUpdate,
              robotest_sim::ContactAggregatorSystem::ISystemPostUpdate,
              robotest_sim::ContactAggregatorSystem::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(robotest_sim::ContactAggregatorSystem,
                    "robotest_sim::ContactAggregatorSystem")

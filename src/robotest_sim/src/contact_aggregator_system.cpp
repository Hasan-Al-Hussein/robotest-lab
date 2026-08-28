// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
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
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"
#include "gz/transport/Node.hh"
#include "robotest_sim/contact_aggregator.hpp"

namespace robotest_sim
{
namespace internal
{

gz::sim::Entity top_level_model_ancestor(
  gz::sim::Entity entity,
  const gz::sim::EntityComponentManager & ecm);

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

#ifndef ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256
#error "contact pipeline source inventory digest was not supplied by CMake"
#endif

constexpr char kSourceInventoryIdentity[] =
  "ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_"
  "SHA256=" ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256;
static_assert(sizeof(kSourceInventoryIdentity) == 111U);

constexpr char kRobotModelName[] = "robotest";
constexpr char kOutputTopic[] = "/robotest/internal/contact_aggregate";
constexpr char kProfileEnvironment[] = "ROBOTEST_CONTACT_PROFILE";
constexpr char kProfilePrefix[] = "ROBOTEST_CONTACT_PROFILE ";

struct ThreadCpuTimingStart
{
  std::uint64_t cpu_time_ns{0U};
  std::int64_t linux_tid{0};
};

bool contact_profile_requested() noexcept
{
  const auto *value = std::getenv(kProfileEnvironment);
  return value != nullptr && std::string_view(value) == "1";
}

std::int64_t simulation_stamp_ns(const gz::sim::UpdateInfo & info)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime)
         .count();
}

std::optional<std::uint64_t> thread_cpu_time_ns() noexcept
{
  timespec value{};
  if (::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0 ||
    value.tv_sec < 0 || value.tv_nsec < 0 || value.tv_nsec >= 1000000000L)
  {
    return std::nullopt;
  }
  constexpr std::uint64_t nanoseconds_per_second = 1000000000ULL;
  const auto seconds = static_cast<std::uint64_t>(value.tv_sec);
  if (seconds >
    (std::numeric_limits<std::uint64_t>::max() -
    static_cast<std::uint64_t>(value.tv_nsec)) / nanoseconds_per_second)
  {
    return std::nullopt;
  }
  return seconds * nanoseconds_per_second +
         static_cast<std::uint64_t>(value.tv_nsec);
}

std::int64_t linux_thread_id() noexcept
{
  static thread_local const auto linux_tid =
    static_cast<std::int64_t>(::syscall(SYS_gettid));
  return linux_tid;
}

std::optional<ThreadCpuTimingStart> thread_cpu_timing_start() noexcept
{
  const auto cpu_time_ns = thread_cpu_time_ns();
  const auto linux_tid = linux_thread_id();
  if (!cpu_time_ns.has_value() || linux_tid <= 0) {
    return std::nullopt;
  }
  return ThreadCpuTimingStart{*cpu_time_ns, linux_tid};
}

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
    profile_ =
      internal::ContactProfileAccumulator(contact_profile_requested());
    publisher_ = transport_node_.Advertise<gz::msgs::Contacts>(kOutputTopic);
    if (!publisher_) {
      throw std::runtime_error(
          "contact aggregator could not advertise its output topic");
    }
    gzmsg << kSourceInventoryIdentity << std::endl;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (fatal_latched_) {
      emit_stop_once();
      return;
    }

    if (bindings_locked_) {
      return;
    }

    try {
      const bool was_locked = bindings_locked_;
      discover_and_lock_bindings(ecm);
      if (!was_locked && bindings_locked_) {
        profile_.lock(simulation_stamp_ns(info));
      }
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
    if (profile_.failure_pending()) {
      emit_contact_profile_if_due(simulation_stamp_ns(info));
    }
    if (fatal_latched_ || !bindings_locked_) {
      return;
    }

    try {
      if (profile_.enabled()) {
        post_update_profiled(info, ecm);
        if (profile_.failure_pending()) {
          emit_contact_profile_if_due(simulation_stamp_ns(info));
        }
        return;
      }

      if (!validate_locked_binding_state(ecm)) {
        return;
      }
      const bool locked_entity_removed =
        locked_entity_marked_for_removal(ecm);
      if (locked_entity_removed) {
        latch_fatal("a locked robotest contact entity was marked for removal");
        return;
      }
      if (inventory_event_requires_full_validation(ecm) &&
        !validate_locked_inventory(ecm))
      {
        return;
      }
      if (info.paused) {
        return;
      }

      const auto stamp_ns = simulation_stamp_ns(info);
      ContactAggregateSources current_sources{};
      for (std::size_t index = 0U; index < bindings_.size(); ++index) {
        const auto & binding = bindings_[index];
        const auto *contacts =
          ecm.Component<gz::sim::components::ContactSensorData>(
          binding.collision_entity);
        current_sources[index] = &contacts->Data();
      }
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
    if (profile_.failure_pending()) {
      emit_contact_profile_if_due(simulation_stamp_ns(info));
    }
  }

  void Reset(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager &) override
  {
    policy_.reset();
    // Simulation time can rewind on reset. Start a new cumulative profile
    // epoch so one record never mixes CPU work from separate simulator epochs.
    profile_.lock(simulation_stamp_ns(info));
  }

private:
  struct ContactBinding
  {
    gz::sim::Entity sensor_entity{gz::sim::kNullEntity};
    gz::sim::Entity link_entity{gz::sim::kNullEntity};
    gz::sim::Entity collision_entity{gz::sim::kNullEntity};
  };

  using DiscoveredBindings =
    std::array<std::optional<ContactBinding>, kExpectedBindings.size()>;

  struct InventorySnapshot
  {
    gz::sim::Entity model_entity{gz::sim::kNullEntity};
    DiscoveredBindings bindings{};
    std::vector<gz::sim::Entity> model_entities;
    std::vector<gz::sim::Entity> sensor_entities;
    std::vector<gz::sim::Entity> link_entities;
    std::vector<gz::sim::Entity> collision_entities;
    std::optional<std::string> error;
  };

  void finish_profile_timing(
    const internal::ContactProfileCategory category,
    const std::optional<ThreadCpuTimingStart> & start) noexcept
  {
    if (!start.has_value()) {
      profile_.fail(internal::ContactProfileFailure::ClockUnavailable);
      return;
    }
    const auto finish = thread_cpu_timing_start();
    if (!finish.has_value()) {
      profile_.fail(internal::ContactProfileFailure::ClockUnavailable);
      return;
    }
    if (finish->linux_tid != start->linux_tid) {
      profile_.fail(internal::ContactProfileFailure::InvalidThreadIdentity);
      return;
    }
    if (finish->cpu_time_ns < start->cpu_time_ns) {
      profile_.fail(internal::ContactProfileFailure::ClockRegressed);
      return;
    }
    profile_.add_timing(
      category, finish->cpu_time_ns - start->cpu_time_ns, start->linux_tid);
  }

  template<typename Operation>
  auto measure_profile_category(
    const internal::ContactProfileCategory category,
    Operation && operation)
  {
    if (!profile_.enabled()) {
      return operation();
    }
    const auto start = thread_cpu_timing_start();
    if (!start.has_value()) {
      profile_.fail(internal::ContactProfileFailure::ClockUnavailable);
      return operation();
    }
    try {
      auto result = operation();
      finish_profile_timing(category, start);
      return result;
    } catch (...) {
      finish_profile_timing(category, start);
      throw;
    }
  }

  void write_profile_failure_record(const std::string & record) noexcept
  {
    try {
      // A prior failed write leaves the global stream failed. Clear only the
      // stream state so a later callback can make the promised retry.
      std::cerr.clear();
      std::cerr << kProfilePrefix << record << std::endl;
      if (std::cerr.good()) {
        profile_.acknowledge_failure_emitted();
      }
    } catch (...) {
      // Leave the failure pending so the next callback can retry stderr.
    }
  }

  void emit_contact_profile_if_due(const std::int64_t stamp_ns) noexcept
  {
    if (profile_.failure_pending()) {
      emit_pending_profile_failure(stamp_ns);
      return;
    }
    if (!profile_.enabled()) {
      return;
    }
    try {
      const auto record = profile_.emit_if_due(stamp_ns);
      if (record.has_value()) {
        if (profile_.failure_pending()) {
          write_profile_failure_record(*record);
          return;
        }
        std::cout << kProfilePrefix << *record << std::endl;
        if (!std::cout) {
          profile_.fail(internal::ContactProfileFailure::OutputUnavailable);
          emit_pending_profile_failure(stamp_ns);
        }
      }
    } catch (...) {
      // Diagnostics must never change contact or simulator behavior.
      profile_.fail(internal::ContactProfileFailure::OutputUnavailable);
      emit_pending_profile_failure(stamp_ns);
    }
  }

  void emit_pending_profile_failure(const std::int64_t stamp_ns) noexcept
  {
    try {
      const auto record = profile_.emit_if_due(stamp_ns);
      if (record.has_value()) {
        write_profile_failure_record(*record);
      }
    } catch (...) {
      // Leave the failure pending so the next callback can retry stderr.
    }
  }

  void post_update_profiled(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm)
  {
    const auto callback_linux_tid = linux_thread_id();
    if (callback_linux_tid <= 0) {
      profile_.fail(internal::ContactProfileFailure::InvalidThreadIdentity);
    }
    const bool locked_binding_valid = measure_profile_category(
      internal::ContactProfileCategory::LockedBindingValidation,
      [this, &ecm]() {return validate_locked_binding_state(ecm);});
    if (!locked_binding_valid) {
      return;
    }

    const auto inventory_event = measure_profile_category(
        internal::ContactProfileCategory::CachedEventStateCheck,
      [this, &ecm]() {
        const bool locked_entity_removed =
        locked_entity_marked_for_removal(ecm);
        const bool rescan_required = !locked_entity_removed &&
        inventory_event_requires_full_validation(ecm);
        return std::make_pair(locked_entity_removed, rescan_required);
        });
    if (inventory_event.first) {
      latch_fatal("a locked robotest contact entity was marked for removal");
      return;
    }
    if (inventory_event.second) {
      profile_.count_rescan(callback_linux_tid);
      const bool inventory_valid = measure_profile_category(
        internal::ContactProfileCategory::ExhaustiveEventRescan,
        [this, &ecm]() {return validate_locked_inventory(ecm);});
      if (!inventory_valid) {
        return;
      }
    }
    if (info.paused) {
      return;
    }

    const auto stamp_ns = simulation_stamp_ns(info);
    profile_.count_observation(callback_linux_tid);
    auto decision = measure_profile_category(
      internal::ContactProfileCategory::ContactPolicyProtobuf,
      [this, &ecm, stamp_ns]() {
        ContactAggregateSources current_sources{};
        for (std::size_t index = 0U; index < bindings_.size(); ++index) {
          const auto & binding = bindings_[index];
          const auto *contacts =
          ecm.Component<gz::sim::components::ContactSensorData>(
            binding.collision_entity);
          current_sources[index] = &contacts->Data();
        }
        return policy_.observe(stamp_ns, current_sources);
      });
    if (decision.fatal) {
      latch_fatal(std::move(decision.detail));
      return;
    }
    if (decision.output.has_value()) {
      const bool published = measure_profile_category(
        internal::ContactProfileCategory::Publish,
        [this, &decision]() {
          return publisher_.Publish(*decision.output);
        });
      if (!published) {
        latch_fatal("contact aggregator failed to publish an aggregate sample");
        return;
      }
      profile_.count_publish(callback_linux_tid);
    }
    emit_contact_profile_if_due(stamp_ns);
  }

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

  InventorySnapshot scan_inventory(
    const gz::sim::EntityComponentManager & ecm) const
  {
    InventorySnapshot snapshot;
    std::size_t robot_model_count = 0U;
    ecm.Each<gz::sim::components::Model>(
      [&ecm, &snapshot, &robot_model_count](const gz::sim::Entity & entity,
      const gz::sim::components::Model *) {
        snapshot.model_entities.push_back(entity);
        const auto *name = ecm.Component<gz::sim::components::Name>(entity);
        if (name == nullptr) {
          return true;
        }
        const auto parent = ecm.ParentEntity(entity);
        if (name->Data() == kRobotModelName &&
        parent != gz::sim::kNullEntity &&
        ecm.Component<gz::sim::components::World>(parent) != nullptr)
        {
          ++robot_model_count;
          if (snapshot.model_entity == gz::sim::kNullEntity) {
            snapshot.model_entity = entity;
          }
        }
        return true;
      });

    if (robot_model_count > 1U) {
      snapshot.error =
        "multiple top-level robotest models expose ambiguous contact sources";
      return snapshot;
    }

    ecm.Each<gz::sim::components::Link>(
      [&snapshot](const gz::sim::Entity & entity,
      const gz::sim::components::Link *) {
        snapshot.link_entities.push_back(entity);
        return true;
      });
    ecm.Each<gz::sim::components::Collision>(
      [&snapshot](const gz::sim::Entity & entity,
      const gz::sim::components::Collision *) {
        snapshot.collision_entities.push_back(entity);
        return true;
      });

    std::set<gz::sim::Entity> collision_entities;
    ecm.Each<gz::sim::components::ContactSensor>(
      [&ecm, &snapshot, &collision_entities](
        const gz::sim::Entity & sensor_entity,
        const gz::sim::components::ContactSensor *sensor) {
        snapshot.sensor_entities.push_back(sensor_entity);
        if (snapshot.error.has_value()) {
          return false;
        }
        const auto *sensor_name =
        ecm.Component<gz::sim::components::Name>(sensor_entity);
        if (sensor_name == nullptr) {
          if (internal::nameless_contact_sensor_under_model(
              sensor_entity, snapshot.model_entity, ecm))
          {
            snapshot.error =
            "contact sensor under top-level robotest model lacks a Name "
            "component";
          }
          return !snapshot.error.has_value();
        }

        const auto expected_index = expected_binding_index(sensor_name->Data());
        const auto top_model =
        internal::top_level_model_ancestor(sensor_entity, ecm);
        const bool under_robotest =
        snapshot.model_entity != gz::sim::kNullEntity &&
        top_model == snapshot.model_entity;
        if (!expected_index.has_value()) {
          if (under_robotest) {
            snapshot.error =
            "unexpected contact sensor under top-level robotest model: " +
            sensor_name->Data();
          }
          return !snapshot.error.has_value();
        }
        if (!under_robotest) {
          snapshot.error = "expected-name contact sensor exists outside the "
          "top-level robotest model: " + sensor_name->Data();
          return false;
        }
        if (snapshot.bindings[*expected_index].has_value()) {
          snapshot.error =
          "duplicate robotest contact sensor: " + sensor_name->Data();
          return false;
        }

        const auto & expected = kExpectedBindings[*expected_index];
        const auto link_entity = ecm.ParentEntity(sensor_entity);
        const auto *link = ecm.Component<gz::sim::components::Link>(link_entity);
        const auto *link_name =
        ecm.Component<gz::sim::components::Name>(link_entity);
        if (link == nullptr || link_name == nullptr ||
        link_name->Data() != expected.link_name ||
        ecm.ParentEntity(link_entity) != snapshot.model_entity ||
        internal::top_level_model_ancestor(link_entity, ecm) !=
        snapshot.model_entity)
        {
          snapshot.error =
          std::string("contact sensor has the wrong link ancestry: ") +
          expected.sensor_name;
          return false;
        }

        const auto sensor_sdf = sensor->Data();
        if (!sensor_sdf || !sensor_sdf->HasElement("contact")) {
          snapshot.error =
          std::string("contact sensor lacks contact configuration: ") +
          expected.sensor_name;
          return false;
        }
        const auto contact_element = sensor_sdf->GetElement("contact");
        if (!contact_element || !contact_element->HasElement("collision")) {
          snapshot.error =
          std::string("contact sensor lacks a collision mapping: ") +
          expected.sensor_name;
          return false;
        }
        const auto collision_element = contact_element->GetElement("collision");
        if (!collision_element ||
        collision_element->GetNextElement("collision") != nullptr ||
        collision_element->Get<std::string>() != expected.collision_name)
        {
          snapshot.error =
          std::string(
              "contact sensor collision mapping differs from contract: ") +
          expected.sensor_name;
          return false;
        }

        const auto matches = ecm.ChildrenByComponents(
          link_entity, gz::sim::components::Collision(),
          gz::sim::components::Name(expected.collision_name));
        if (matches.size() != 1U ||
        internal::top_level_model_ancestor(matches.front(), ecm) !=
        snapshot.model_entity)
        {
          snapshot.error = std::string(
            "contact sensor collision entity is missing or ambiguous: ") +
          expected.sensor_name;
          return false;
        }
        if (!collision_entities.insert(matches.front()).second) {
          snapshot.error =
          "multiple robotest contact sensors bind one collision entity";
          return false;
        }

        snapshot.bindings[*expected_index] =
        ContactBinding{sensor_entity, link_entity, matches.front()};
        return true;
      });
    return snapshot;
  }

  static std::size_t discovered_binding_count(
    const DiscoveredBindings & bindings)
  {
    return static_cast<std::size_t>(
      std::count_if(bindings.begin(), bindings.end(),
      [](const auto & binding) {return binding.has_value();}));
  }

  void refresh_locked_inventory_cache(const InventorySnapshot & snapshot)
  {
    cached_model_entities_ = snapshot.model_entities;
    cached_sensor_entities_ = snapshot.sensor_entities;
    cached_link_entities_ = snapshot.link_entities;
    cached_collision_entities_ = snapshot.collision_entities;
  }

  void discover_and_lock_bindings(gz::sim::EntityComponentManager & ecm)
  {
    const auto snapshot = scan_inventory(ecm);
    if (snapshot.error.has_value()) {
      latch_fatal(*snapshot.error);
      return;
    }
    if (discovered_binding_count(snapshot.bindings) !=
      kExpectedBindings.size())
    {
      return;
    }

    for (std::size_t index = 0U; index < snapshot.bindings.size(); ++index) {
      const auto binding = *snapshot.bindings[index];
      auto *contact_data =
        ecm.Component<gz::sim::components::ContactSensorData>(
          binding.collision_entity);
      if (contact_data == nullptr) {
        contact_data = ecm.CreateComponent(
          binding.collision_entity, gz::sim::components::ContactSensorData());
      }
      if (contact_data == nullptr) {
        latch_fatal(std::string("cannot create contact data for sensor ") +
                    kExpectedBindings[index].sensor_name);
        return;
      }
    }

    for (std::size_t index = 0U; index < snapshot.bindings.size(); ++index) {
      bindings_[index] = *snapshot.bindings[index];
    }
    robot_model_entity_ = snapshot.model_entity;
    bindings_locked_ = true;
    refresh_locked_inventory_cache(snapshot);
    gzmsg << "Contact aggregator locked all seven robotest sensor bindings"
          << std::endl;
  }

  bool validate_locked_binding_state(
    const gz::sim::EntityComponentManager & ecm)
  {
    const auto *model =
      ecm.Component<gz::sim::components::Model>(robot_model_entity_);
    const auto *model_name =
      ecm.Component<gz::sim::components::Name>(robot_model_entity_);
    const auto model_parent = ecm.ParentEntity(robot_model_entity_);
    if (!ecm.HasEntity(robot_model_entity_) || model == nullptr ||
      model_name == nullptr || model_name->Data() != kRobotModelName ||
      model_parent == gz::sim::kNullEntity ||
      ecm.Component<gz::sim::components::World>(model_parent) == nullptr)
    {
      latch_fatal(
        "locked robotest model disappeared or changed before aggregation");
      return false;
    }

    for (std::size_t index = 0U; index < bindings_.size(); ++index) {
      const auto & binding = bindings_[index];
      const auto & expected = kExpectedBindings[index];
      const auto *sensor =
        ecm.Component<gz::sim::components::ContactSensor>(
          binding.sensor_entity);
      const auto *sensor_name =
        ecm.Component<gz::sim::components::Name>(binding.sensor_entity);
      const auto *link =
        ecm.Component<gz::sim::components::Link>(binding.link_entity);
      const auto *link_name =
        ecm.Component<gz::sim::components::Name>(binding.link_entity);
      const auto *collision =
        ecm.Component<gz::sim::components::Collision>(
          binding.collision_entity);
      const auto *collision_name =
        ecm.Component<gz::sim::components::Name>(binding.collision_entity);
      const auto *contacts =
        ecm.Component<gz::sim::components::ContactSensorData>(
          binding.collision_entity);
      if (!ecm.HasEntity(binding.sensor_entity) ||
        !ecm.HasEntity(binding.link_entity) ||
        !ecm.HasEntity(binding.collision_entity) || sensor == nullptr ||
        sensor_name == nullptr || sensor_name->Data() != expected.sensor_name ||
        link == nullptr || link_name == nullptr ||
        link_name->Data() != expected.link_name || collision == nullptr ||
        collision_name == nullptr ||
        collision_name->Data() != expected.collision_name ||
        contacts == nullptr ||
        ecm.ParentEntity(binding.sensor_entity) != binding.link_entity ||
        ecm.ParentEntity(binding.collision_entity) != binding.link_entity ||
        ecm.ParentEntity(binding.link_entity) != robot_model_entity_)
      {
        latch_fatal(
          std::string("locked contact binding changed for sensor ") +
          expected.sensor_name);
        return false;
      }
    }
    return true;
  }

  bool locked_entity_marked_for_removal(
    const gz::sim::EntityComponentManager & ecm) const
  {
    if (!ecm.HasEntitiesMarkedForRemoval()) {
      return false;
    }
    bool locked_entity_removed = false;
    ecm.EachRemoved<gz::sim::components::Name>(
      [this, &locked_entity_removed](const gz::sim::Entity & entity,
      const gz::sim::components::Name *) {
        locked_entity_removed = entity == robot_model_entity_;
        for (const auto & binding : bindings_) {
          locked_entity_removed = locked_entity_removed ||
          entity == binding.sensor_entity || entity == binding.link_entity ||
          entity == binding.collision_entity;
        }
        return !locked_entity_removed;
      });
    return locked_entity_removed;
  }

  bool inventory_event_requires_full_validation(
    const gz::sim::EntityComponentManager & ecm)
  {
    const auto cached_changes = internal::cached_inventory_component_changes(
      ecm, cached_model_entities_, cached_sensor_entities_,
      cached_link_entities_, cached_collision_entities_);
    const bool has_one_time_component_changes =
      ecm.HasOneTimeComponentChanges();
    const bool relevant_one_time_change =
      cached_changes[0] ||
      (has_one_time_component_changes &&
      internal::relevant_one_time_inventory_component_changed(ecm));
    const bool relevant_periodic_change =
      cached_changes[1] ||
      internal::relevant_periodic_inventory_type_changed(ecm);

    // A ContactSensor SDF mutation must be followed by ECM::SetChanged on the
    // ContactSensor component. Unsignalled in-place SDF mutation violates the
    // ECM change-state contract and is intentionally unsupported.
    return internal::locked_inventory_scan_required(
      ecm.HasNewEntities(), ecm.HasEntitiesMarkedForRemoval(),
      ecm.HasRemovedComponents(), relevant_one_time_change,
      relevant_periodic_change,
      has_one_time_component_changes, ecm.HasPeriodicComponentChanges());
  }

  bool validate_locked_inventory(
    const gz::sim::EntityComponentManager & ecm)
  {
    const auto snapshot = scan_inventory(ecm);
    if (snapshot.error.has_value()) {
      latch_fatal(*snapshot.error);
      return false;
    }
    if (snapshot.model_entity != robot_model_entity_) {
      latch_fatal("locked top-level robotest model disappeared or rebound");
      return false;
    }
    if (discovered_binding_count(snapshot.bindings) !=
      kExpectedBindings.size())
    {
      latch_fatal("one or more locked robotest contact sensors disappeared");
      return false;
    }
    for (std::size_t index = 0U; index < snapshot.bindings.size(); ++index) {
      const auto & discovered = *snapshot.bindings[index];
      const auto & locked = bindings_[index];
      if (discovered.sensor_entity != locked.sensor_entity ||
        discovered.link_entity != locked.link_entity ||
        discovered.collision_entity != locked.collision_entity)
      {
        latch_fatal(std::string("locked contact source rebound for sensor ") +
                    kExpectedBindings[index].sensor_name);
        return false;
      }
    }
    refresh_locked_inventory_cache(snapshot);
    return true;
  }

  ContactAggregatorPolicy policy_;
  gz::transport::Node transport_node_;
  gz::transport::Node::Publisher publisher_;
  gz::sim::EventManager *event_manager_{nullptr};
  std::array<ContactBinding, kExpectedBindings.size()> bindings_{};
  std::vector<gz::sim::Entity> cached_model_entities_;
  std::vector<gz::sim::Entity> cached_sensor_entities_;
  std::vector<gz::sim::Entity> cached_link_entities_;
  std::vector<gz::sim::Entity> cached_collision_entities_;
  internal::ContactProfileAccumulator profile_;
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

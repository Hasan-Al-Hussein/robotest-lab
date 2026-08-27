// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

#ifndef ROBOTEST_SIM__CONTACT_AGGREGATOR_HPP_
#define ROBOTEST_SIM__CONTACT_AGGREGATOR_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gz/msgs/contacts.pb.h"

namespace robotest_sim
{

inline constexpr std::int64_t kContactAggregatePeriodNs = 20000000;
inline constexpr std::size_t kContactAggregateSourceCount = 7U;

namespace internal
{

inline constexpr std::int64_t kContactProfileEmissionPeriodNs = 5000000000LL;

enum class ContactProfileCategory : std::size_t
{
  LockedBindingValidation = 0U,
  CachedEventStateCheck = 1U,
  ExhaustiveEventRescan = 2U,
  ContactPolicyProtobuf = 3U,
  Publish = 4U,
  Count = 5U,
};

/// Fixed-size cumulative state behind the optional live attribution log.
class ContactProfileAccumulator {
public:
  explicit ContactProfileAccumulator(bool enabled = false) noexcept;

  bool enabled() const noexcept;
  void disable() noexcept;
  void lock(std::int64_t simulation_stamp_ns) noexcept;
  void restart_cadence(std::int64_t simulation_stamp_ns) noexcept;
  void add_timing(
    ContactProfileCategory category,
    std::uint64_t elapsed_ns,
    std::int64_t linux_tid) noexcept;
  void count_observation() noexcept;
  void count_rescan() noexcept;
  void count_publish() noexcept;
  std::optional<std::string> emit_if_due(std::int64_t simulation_stamp_ns);

private:
  void saturating_add(
    std::uint64_t & target,
    std::uint64_t increment) noexcept;
  void count(std::uint64_t & target) noexcept;

  std::array<std::uint64_t,
    static_cast<std::size_t>(ContactProfileCategory::Count)> cumulative_ns_{};
  std::optional<std::int64_t> next_emission_stamp_ns_;
  std::optional<std::int64_t> profile_epoch_start_sim_stamp_ns_;
  std::optional<std::int64_t> linux_tid_;
  std::uint64_t observation_count_{0U};
  std::uint64_t rescan_count_{0U};
  std::uint64_t publish_count_{0U};
  bool enabled_{false};
  bool locked_{false};
  bool saturated_{false};
};

}  // namespace internal

using ContactAggregateSources =
  std::array<const gz::msgs::Contacts *, kContactAggregateSourceCount>;

struct ContactAggregateDecision
{
  std::optional<gz::msgs::Contacts> output;
  bool fatal{false};
  std::string detail;
};

/// Observe every physics step while publishing a bounded transport stream.
class ContactAggregatorPolicy {
public:
  ContactAggregateDecision observe(
    std::int64_t simulation_stamp_ns,
    const gz::msgs::Contacts & current_contacts);
  ContactAggregateDecision observe(
    std::int64_t simulation_stamp_ns,
    const ContactAggregateSources & current_sources);
  void reset();

private:
  using ContactPair = std::pair<std::string, std::string>;

  struct ContactGroup
  {
    std::size_t string_bytes{0U};
    std::vector<gz::msgs::Contact> records;
  };

  using ContactGroups = std::map<ContactPair, ContactGroup>;

  ContactAggregateDecision observe_sources(
    std::int64_t simulation_stamp_ns,
    const gz::msgs::Contacts * const * current_sources,
    std::size_t source_count);
  static std::optional<std::string>
  validated_step_groups(
    const gz::msgs::Contacts * const * contacts,
    std::size_t source_count,
    ContactGroups & groups);
  static std::optional<std::string>
  validate_complete_groups(const ContactGroups & groups);
  static gz::msgs::Contacts aggregate_at(
    std::int64_t boundary_stamp_ns,
    const ContactGroups & groups);
  ContactAggregateDecision fail(std::string detail);

  std::optional<std::int64_t> last_observed_stamp_ns_;
  std::optional<std::int64_t> next_boundary_stamp_ns_;
  ContactGroups interval_groups_;
  std::optional<std::string> fatal_detail_;
  bool stream_started_{false};
};

}  // namespace robotest_sim

#endif  // ROBOTEST_SIM__CONTACT_AGGREGATOR_HPP_

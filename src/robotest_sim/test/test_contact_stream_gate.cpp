// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "robotest_sim/contact_stream_gate.hpp"
#include "ros_gz_interfaces/msg/contact.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"

namespace robotest_sim
{
namespace
{

constexpr const char * kGround = "ground_plane::ground_link::ground_collision";
constexpr const char * kChassis =
  "robotest::base_footprint::base_footprint_fixed_joint_lump__base_link_collision_collision";
constexpr const char * kLidar = "robotest::lidar_link::lidar_link_collision_collision";
constexpr const char * kWall = "phase3_contact_control_wall::link::collision";
constexpr const char * kBox = "box::link::collision";
constexpr const char * kLeftWheel =
  "robotest::left_wheel_link::"
  "left_wheel_link_fixed_joint_lump__left_wheel_collision_collision";
constexpr const char * kRightWheel =
  "robotest::right_wheel_link::"
  "right_wheel_link_fixed_joint_lump__right_wheel_collision_collision";
constexpr const char * kFrontCaster =
  "robotest::front_caster_link::"
  "front_caster_link_fixed_joint_lump__front_caster_collision_collision";
constexpr const char * kRearCaster =
  "robotest::rear_caster_link::"
  "rear_caster_link_fixed_joint_lump__rear_caster_collision_collision";

using Pair = std::pair<std::string, std::string>;

ros_gz_interfaces::msg::Contact contact_for(
  const Pair & pair,
  const double force = 3.5,
  const double depth = 0.0125)
{
  ros_gz_interfaces::msg::Contact contact;
  contact.collision1.name = pair.first;
  contact.collision2.name = pair.second;
  contact.positions.resize(1U);
  contact.positions[0].x = 1.25;
  contact.normals.resize(1U);
  contact.normals[0].x = 1.0;
  contact.depths = {depth};
  contact.wrenches.resize(1U);
  contact.wrenches[0].body_1_name.data = pair.first;
  contact.wrenches[0].body_2_name.data = pair.second;
  contact.wrenches[0].body_1_wrench.force.x = force;
  return contact;
}

ros_gz_interfaces::msg::Contacts message_at(
  const std::int64_t stamp_ns,
  const std::vector<Pair> & pairs,
  const std::string & frame = "")
{
  ros_gz_interfaces::msg::Contacts message;
  message.header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
  message.header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
  message.header.frame_id = frame;
  for (const auto & pair : pairs) {
    message.contacts.push_back(contact_for(pair));
  }
  return message;
}

std::string scoped_name_with_size(const std::string & model, const std::size_t size)
{
  const std::string prefix = model + "::";
  constexpr const char * suffix = "::collision";
  const auto suffix_size = std::char_traits<char>::length(suffix);
  EXPECT_GE(size, prefix.size() + suffix_size + 1U);
  return prefix + std::string(size - prefix.size() - suffix_size, 'x') + suffix;
}

ros_gz_interfaces::msg::Contacts minimal_message_at(
  const std::int64_t stamp_ns,
  const std::vector<Pair> & pairs)
{
  auto message = message_at(stamp_ns, pairs);
  for (auto & contact : message.contacts) {
    contact.normals.clear();
    contact.depths.clear();
    contact.wrenches.clear();
  }
  return message;
}

ContactGateDecision observe(
  ContactStreamPolicy & policy,
  const std::int64_t stamp_ns,
  const std::vector<Pair> & pairs)
{
  return policy.observe(message_at(stamp_ns, pairs));
}

std::vector<Pair> normalized_output_pairs(const ContactGateDecision & decision)
{
  std::vector<Pair> result;
  if (!decision.output.has_value()) {
    return result;
  }
  for (const auto & contact : decision.output->contacts) {
    auto pair = Pair{contact.collision1.name, contact.collision2.name};
    if (pair.second < pair.first) {
      std::swap(pair.first, pair.second);
    }
    result.push_back(std::move(pair));
  }
  return result;
}

TEST(ContactStreamPolicy, EqualStampSupportBatchWallAndReleaseAreCompleteAndBounded)
{
  ContactStreamPolicy policy;
  const std::vector<Pair> supports = {
    {kLeftWheel, kGround},
    {kRightWheel, kGround},
    {kFrontCaster, kGround},
    {kRearCaster, kGround},
  };

  EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).output.has_value());

  for (const auto & support : supports) {
    EXPECT_FALSE(policy.observe(message_at(1000000000LL, {support})).output.has_value());
  }
  auto decision = observe(policy, 1002000000LL, {supports[0]});
  ASSERT_TRUE(decision.output.has_value());
  EXPECT_EQ(decision.output->header.stamp.sec, 1);
  EXPECT_EQ(decision.output->header.stamp.nanosec, 0U);
  EXPECT_EQ(decision.output->contacts.size(), 4U);
  EXPECT_EQ(decision.reason, ContactForwardReason::kInitialSnapshot);

  std::int64_t stamp = 1004000000LL;
  for (; stamp <= 1202000000LL; stamp += 2000000LL) {
    const auto index = static_cast<std::size_t>(stamp / 2000000LL) % supports.size();
    decision = observe(policy, stamp, {supports[index]});
  }
  ASSERT_TRUE(decision.output.has_value());
  EXPECT_EQ(decision.reason, ContactForwardReason::kSteadyStateHeartbeat);
  EXPECT_EQ(decision.output->contacts.size(), 4U);

  EXPECT_FALSE(observe(policy, stamp, {{kChassis, kWall}}).output.has_value());
  stamp += 2000000LL;
  decision = observe(policy, stamp, {supports[0]});
  ASSERT_TRUE(decision.output.has_value());
  EXPECT_EQ(decision.reason, ContactForwardReason::kPairSetTransition);
  EXPECT_EQ(decision.output->contacts.size(), 5U);

  const auto wall_stamp = stamp - 2000000LL;
  for (stamp += 2000000LL; stamp <= wall_stamp + kContactReleaseGapNs; stamp += 2000000LL) {
    const auto index = static_cast<std::size_t>(stamp / 2000000LL) % supports.size();
    decision = observe(policy, stamp, {supports[index]});
    EXPECT_FALSE(
      decision.output.has_value() && decision.output->contacts.size() == supports.size());
  }
  // Equality is still active. The first completed, strictly later absent stamp
  // proves release; the following raw stamp finalizes that snapshot.
  const auto first_strictly_absent_stamp = wall_stamp + kContactReleaseGapNs + 2000000LL;
  EXPECT_FALSE(observe(
    policy, first_strictly_absent_stamp, {supports[0]}).output.has_value());
  decision = observe(policy, first_strictly_absent_stamp + 2000000LL, {supports[1]});
  ASSERT_TRUE(decision.output.has_value());
  EXPECT_EQ(decision.reason, ContactForwardReason::kPairSetTransition);
  EXPECT_EQ(decision.output->contacts.size(), 4U);
  EXPECT_TRUE(std::none_of(
    decision.output->contacts.begin(), decision.output->contacts.end(),
      [](const auto & contact) {
        return contact.collision1.name == kWall || contact.collision2.name == kWall;
    }));

  const auto malformed_stamp = first_strictly_absent_stamp + 4000000LL;
  EXPECT_FALSE(observe(
    policy, malformed_stamp, {{kBox, kWall}}).fatal);
  const auto malformed = observe(
    policy, malformed_stamp + 2000000LL, {supports[2]});
  EXPECT_TRUE(malformed.fatal);
  ASSERT_TRUE(malformed.output.has_value());
  EXPECT_NE(std::find_if(
      malformed.output->contacts.begin(), malformed.output->contacts.end(),
      [](const auto & contact) {
        return contact.collision1.name == kBox && contact.collision2.name == kWall;
      }), malformed.output->contacts.end());
}

TEST(ContactStreamPolicy, PersistentStateIsGloballyBoundedToFiveHertz)
{
  ContactStreamPolicy policy;
  std::size_t published = 0U;
  for (std::int64_t stamp = 1000000000LL; stamp <= 3004000000LL; stamp += 2000000LL) {
    const auto decision = observe(policy, stamp, {{kChassis, kWall}});
    if (decision.output.has_value()) {
      ++published;
      EXPECT_EQ(decision.output->contacts.size(), 1U);
    }
  }
  // Completed public stamps span exactly 1.0 through 3.0 seconds.
  EXPECT_EQ(published, 11U);
}

TEST(ContactStreamPolicy, EqualityRemainsOneEpisodeThenSettledAbsenceAllowsRecontact)
{
  ContactStreamPolicy policy;
  EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).output.has_value());
  ASSERT_FALSE(observe(policy, 1000000000LL, {{kChassis, kWall}}).output.has_value());
  ASSERT_TRUE(observe(policy, 1002000000LL, {{kLeftWheel, kGround}}).output.has_value());

  for (std::int64_t stamp = 1004000000LL; stamp <= 1250000000LL; stamp += 2000000LL) {
    (void)observe(policy, stamp, {{kLeftWheel, kGround}});
  }
  // The wall is absent in the completed 1.250 s batch, exactly 250 ms
  // after its last sighting. It must remain active; recontact at the next
  // stamp therefore continues the same episode.
  EXPECT_FALSE(observe(policy, 1252000000LL, {{kChassis, kWall}}).output.has_value());
  EXPECT_FALSE(observe(policy, 1254000000LL, {{kLeftWheel, kGround}}).output.has_value());

  ContactGateDecision heartbeat;
  for (std::int64_t stamp = 1256000000LL; stamp <= 1404000000LL; stamp += 2000000LL) {
    auto decision = observe(policy, stamp, {{kLeftWheel, kGround}});
    if (decision.output.has_value()) {
      heartbeat = std::move(decision);
    }
  }
  ASSERT_TRUE(heartbeat.output.has_value());
  EXPECT_EQ(heartbeat.output->contacts.size(), 2U);

  // The recontact was completed at 1.252 s, so equality is now 1.502 s.
  for (std::int64_t stamp = 1406000000LL; stamp <= 1504000000LL; stamp += 2000000LL) {
    const auto decision = observe(policy, stamp, {{kLeftWheel, kGround}});
    EXPECT_FALSE(
      decision.output.has_value() && decision.output->contacts.size() == 1U);
  }
  auto release = observe(policy, 1506000000LL, {{kLeftWheel, kGround}});
  ASSERT_TRUE(release.output.has_value());
  EXPECT_EQ(release.output->contacts.size(), 1U);
  EXPECT_TRUE(std::none_of(
    release.output->contacts.begin(), release.output->contacts.end(),
      [](const auto & contact) {
        return contact.collision1.name == kWall || contact.collision2.name == kWall;
    }));
  EXPECT_EQ(release.reason, ContactForwardReason::kPairSetTransition);

  EXPECT_FALSE(observe(policy, 1508000000LL, {{kChassis, kWall}}).output.has_value());
  const auto recontact = observe(policy, 1510000000LL, {{kLeftWheel, kGround}});
  ASSERT_TRUE(recontact.output.has_value());
  EXPECT_EQ(recontact.reason, ContactForwardReason::kPairSetTransition);
  EXPECT_EQ(recontact.output->contacts.size(), 2U);
}

TEST(ContactStreamPolicy, PairMigrationAndTwoCounterpartsRemainInCompleteSnapshots)
{
  ContactStreamPolicy policy;
  constexpr const char * kSecondWall = "second_wall::link::collision";
  EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).output.has_value());
  EXPECT_FALSE(observe(policy, 1000000000LL, {{kChassis, kWall}}).output.has_value());
  ASSERT_TRUE(observe(policy, 1002000000LL, {{kLidar, kWall}}).output.has_value());

  auto migration = observe(policy, 1004000000LL, {{kChassis, kSecondWall}});
  ASSERT_TRUE(migration.output.has_value());
  EXPECT_EQ(migration.active_pair_count, 2U);
  EXPECT_EQ(migration.output->contacts.size(), 2U);

  auto second_counterpart = observe(policy, 1006000000LL, {{kLidar, kWall}});
  ASSERT_TRUE(second_counterpart.output.has_value());
  EXPECT_EQ(second_counterpart.active_pair_count, 3U);
  EXPECT_EQ(second_counterpart.output->contacts.size(), 3U);

  for (std::int64_t stamp = 1008000000LL; stamp <= 1206000000LL; stamp += 2000000LL) {
    const auto pair = stamp % 4000000LL == 0 ? Pair{kLidar, kWall} :
    Pair{kChassis, kSecondWall};
    second_counterpart = observe(policy, stamp, {pair});
  }
  ASSERT_TRUE(second_counterpart.output.has_value());
  EXPECT_EQ(second_counterpart.output->contacts.size(), 3U);
}

TEST(ContactStreamPolicy, DuplicateRecordsAndNestedPayloadsRemainExact)
{
  ContactStreamPolicy policy;
  EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).output.has_value());
  auto message = message_at(1000000000LL, {});
  message.contacts.push_back(contact_for({kChassis, kWall}, 2.0, 0.01));
  message.contacts.push_back(contact_for({kChassis, kWall}, 17.0, 0.02));
  EXPECT_FALSE(policy.observe(message).output.has_value());

  // This later callback contributes a lexicographically earlier normalized
  // pair. Pair blocks must be canonical even though duplicate records within
  // the wall pair retain their delivered callback order.
  auto earlier_pair = message_at(1000000000LL, {{kLidar, kBox}});
  const std::vector<ros_gz_interfaces::msg::Contact> expected = {
    earlier_pair.contacts[0], message.contacts[0], message.contacts[1]};
  EXPECT_FALSE(policy.observe(earlier_pair).output.has_value());

  const auto decision = observe(policy, 1002000000LL, {{kLeftWheel, kGround}});
  ASSERT_TRUE(decision.output.has_value());
  ASSERT_EQ(decision.output->contacts.size(), 3U);
  EXPECT_EQ(decision.output->contacts, expected);
  const auto maximum_force = std::max_element(
    decision.output->contacts.begin(), decision.output->contacts.end(),
    [](const auto & left, const auto & right) {
      return left.wrenches[0].body_1_wrench.force.x <
             right.wrenches[0].body_1_wrench.force.x;
    });
  ASSERT_NE(maximum_force, decision.output->contacts.end());
  EXPECT_DOUBLE_EQ(maximum_force->wrenches[0].body_1_wrench.force.x, 17.0);
}

TEST(ContactStreamPolicy, SequentialSeventeenthPairFailsClosed)
{
  ContactStreamPolicy policy;
  EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).output.has_value());
  std::int64_t stamp = 1000000000LL;
  for (std::size_t index = 0U; index < kMaxActiveContactPairs; ++index) {
    const auto wall = "wall_" + std::to_string(index) + "::link::collision";
    EXPECT_FALSE(observe(policy, stamp, {{kChassis, wall}}).fatal);
    stamp += 2000000LL;
  }
  EXPECT_FALSE(observe(
    policy, stamp, {{kChassis, "wall_16::link::collision"}}).fatal);
  const auto overflow = observe(policy, stamp + 2000000LL, {{kLeftWheel, kGround}});
  EXPECT_TRUE(overflow.fatal);
  EXPECT_FALSE(overflow.output.has_value());
  EXPECT_EQ(overflow.reason, ContactForwardReason::kFatalStructuralInput);
}

TEST(ContactStreamPolicy, RawBatchDuplicateAndNestedBoundsFailClosed)
{
  {
    ContactStreamPolicy policy;
    auto message = message_at(1000000000LL, {});
    for (std::size_t index = 0U; index <= kMaxContactRecordsPerPair; ++index) {
      message.contacts.push_back(contact_for({kChassis, kWall}));
    }
    const auto overflow = policy.observe(message);
    EXPECT_TRUE(overflow.fatal);
    EXPECT_FALSE(overflow.output.has_value());
  }
  {
    ContactStreamPolicy policy;
    auto message = message_at(1000000000LL, {});
    for (std::size_t index = 0U; index <= kMaxRawContactRecords; ++index) {
      message.contacts.push_back(contact_for(
          {kChassis, "wall_" + std::to_string(index) + "::link::collision"}));
    }
    const auto overflow = policy.observe(message);
    EXPECT_TRUE(overflow.fatal);
    EXPECT_FALSE(overflow.output.has_value());
  }
  {
    ContactStreamPolicy policy;
    const std::vector<Pair> pairs = {
      {kLeftWheel, kGround},
      {kRightWheel, kGround},
      {kFrontCaster, kGround},
      {kRearCaster, kGround},
      {kChassis, kWall},
      {kLidar, kWall},
      {kChassis, kBox},
      {kLidar, kBox},
    };
    for (std::size_t index = 0U; index < kMaxRawMessagesPerBatch; ++index) {
      EXPECT_FALSE(policy.observe(message_at(1000000000LL, {pairs[index]})).fatal);
    }
    const auto overflow = policy.observe(
      message_at(1000000000LL, {pairs[kMaxRawMessagesPerBatch]}));
    EXPECT_TRUE(overflow.fatal);
    EXPECT_FALSE(overflow.output.has_value());
  }
  {
    ContactStreamPolicy policy;
    auto message = message_at(1000000000LL, {{kChassis, kWall}});
    message.contacts[0].positions.resize(kMaxContactPointsPerRecord + 1U);
    const auto decision = policy.observe(message);
    EXPECT_TRUE(decision.fatal);
    EXPECT_FALSE(decision.output.has_value());
  }
  {
    ContactStreamPolicy policy;
    EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).output.has_value());
    auto message = message_at(1000000000LL, {{kChassis, kWall}});
    message.contacts[0].normals.clear();
    message.contacts[0].depths.resize(2U);
    const auto decision = policy.observe(message);
    EXPECT_TRUE(decision.fatal);
    EXPECT_FALSE(decision.output.has_value());
  }
  {
    ContactStreamPolicy policy;
    EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).output.has_value());
    auto message = message_at(1000000000LL, {{kChassis, kWall}});
    message.contacts[0].normals.clear();
    message.contacts[0].depths.clear();
    message.contacts[0].wrenches.clear();
    EXPECT_FALSE(policy.observe(message).fatal);
    const auto decision = observe(policy, 1002000000LL, {{kLeftWheel, kGround}});
    ASSERT_TRUE(decision.output.has_value());
    ASSERT_EQ(decision.output->contacts.size(), 1U);
    EXPECT_EQ(decision.output->contacts[0], message.contacts[0]);
  }
  {
    ContactStreamPolicy policy;
    auto message = message_at(1000000000LL, {{kChassis, kWall}});
    message.contacts[0].wrenches[0].body_1_name.data =
      std::string(kMaxBodyNameBytes + 1U, 'x');
    const auto decision = policy.observe(message);
    EXPECT_TRUE(decision.fatal);
    EXPECT_FALSE(decision.output.has_value());
  }
}

TEST(ContactStreamPolicy, StringBudgetsIncludeOneNormalizedKeyCopyPerStoredPair)
{
  {
    std::vector<Pair> four_pairs;
    std::vector<Pair> five_pairs;
    for (std::size_t index = 0U; index < 5U; ++index) {
      Pair pair{
        scoped_name_with_size("outside_a_" + std::to_string(index), 3400U),
        scoped_name_with_size("outside_b_" + std::to_string(index), 3400U)};
      five_pairs.push_back(pair);
      if (index < 4U) {
        four_pairs.push_back(std::move(pair));
      }
    }
    ContactStreamPolicy accepted;
    EXPECT_FALSE(accepted.observe(minimal_message_at(1000000000LL, four_pairs)).fatal);
    ContactStreamPolicy rejected;
    const auto overflow = rejected.observe(minimal_message_at(1000000000LL, five_pairs));
    EXPECT_TRUE(overflow.fatal);
    EXPECT_FALSE(overflow.output.has_value());
  }

  {
    ContactStreamPolicy policy;
    EXPECT_FALSE(policy.observe(
      minimal_message_at(998000000LL, {{kLeftWheel, kGround}})).fatal);
    for (std::size_t index = 0U; index < 8U; ++index) {
      const Pair pair{
        kChassis,
        scoped_name_with_size("active_" + std::to_string(index), 3900U)};
      EXPECT_FALSE(policy.observe(
        minimal_message_at(
          1000000000LL + static_cast<std::int64_t>(index) * 2000000LL,
            {pair})).fatal);
    }
    const Pair ninth{kChassis, scoped_name_with_size("active_8", 3900U)};
    EXPECT_FALSE(policy.observe(minimal_message_at(1016000000LL, {ninth})).fatal);
    const auto overflow = policy.observe(
      minimal_message_at(1018000000LL, {{kLeftWheel, kGround}}));
    EXPECT_TRUE(overflow.fatal);
    EXPECT_FALSE(overflow.output.has_value());
  }
}

TEST(ContactStreamPolicy, FrameRegressionAndLargeAdvanceFailClosed)
{
  {
    ContactStreamPolicy policy;
    const auto decision = policy.observe(
      message_at(1000000000LL, {{kChassis, kWall}}, "world"));
    EXPECT_TRUE(decision.fatal);
    EXPECT_FALSE(decision.output.has_value());
  }
  {
    ContactStreamPolicy policy;
    ASSERT_FALSE(observe(policy, 1000000000LL, {{kChassis, kWall}}).fatal);
    ASSERT_FALSE(observe(policy, 1002000000LL, {{kChassis, kWall}}).fatal);
    const auto decision = observe(policy, 1000000000LL, {{kChassis, kWall}});
    EXPECT_TRUE(decision.fatal);
    EXPECT_FALSE(decision.output.has_value());
  }
  {
    ContactStreamPolicy policy;
    ASSERT_FALSE(observe(policy, 1000000000LL, {{kChassis, kWall}}).fatal);
    EXPECT_FALSE(observe(
      policy, 1000000000LL + kMaxRawStampAdvanceNs, {{kChassis, kWall}}).fatal);
  }
  {
    ContactStreamPolicy policy;
    ASSERT_FALSE(observe(policy, 1000000000LL, {{kChassis, kWall}}).fatal);
    const auto decision = observe(
      policy, 1000000000LL + kMaxRawStampAdvanceNs + 1LL, {{kChassis, kWall}});
    EXPECT_TRUE(decision.fatal);
    EXPECT_FALSE(decision.output.has_value());
  }
}

TEST(ContactStreamPolicy, SlowJoinWarmupIsSuppressedAndOneMessageBatchIsAccepted)
{
  ContactStreamPolicy policy;
  EXPECT_FALSE(policy.observe(message_at(1000000000LL, {{kLeftWheel, kGround}})).fatal);
  EXPECT_FALSE(policy.observe(message_at(1000000000LL, {{kRightWheel, kGround}})).fatal);
  EXPECT_FALSE(observe(policy, 1002000000LL, {{kChassis, kWall}}).output.has_value());
  ASSERT_TRUE(observe(policy, 1004000000LL, {{kChassis, kWall}}).output.has_value());

  EXPECT_FALSE(policy.observe(message_at(1006000000LL, {{kChassis, kWall}})).fatal);
  const auto completed = policy.observe(message_at(1008000000LL, {{kChassis, kWall}}));
  EXPECT_FALSE(completed.fatal);
}

TEST(ContactStreamPolicy, ClockCanDispatchAheadOfTimelyCausalHeartbeat)
{
  ContactStreamPolicy policy;
  EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).fatal);
  EXPECT_FALSE(observe(policy, 1000000000LL, {{kChassis, kWall}}).fatal);
  ASSERT_TRUE(observe(policy, 1002000000LL, {{kChassis, kWall}}).output.has_value());

  // /clock is independently scheduled and may overtake raw callbacks that are
  // already queued. Pending/raw source lag is only 218000001 ns here, so the
  // public callback comparison must not preempt the causal heartbeat.
  EXPECT_FALSE(policy.observe_clock(1220000001LL).fatal);

  ContactGateDecision heartbeat;
  for (std::int64_t stamp = 1004000000LL; stamp <= 1202000000LL; stamp += 2000000LL) {
    auto decision = observe(policy, stamp, {{kChassis, kWall}});
    if (decision.output.has_value()) {
      heartbeat = std::move(decision);
    }
  }
  ASSERT_TRUE(heartbeat.output.has_value());
  EXPECT_EQ(heartbeat.reason, ContactForwardReason::kSteadyStateHeartbeat);
  EXPECT_EQ(heartbeat.output->header.stamp.sec, 1);
  EXPECT_EQ(heartbeat.output->header.stamp.nanosec, 200000000U);
}

TEST(ContactStreamPolicy, IrregularRawProgressKeepsCausalHeartbeatWithinGap)
{
  ContactStreamPolicy policy;
  EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).fatal);
  EXPECT_FALSE(observe(policy, 1000000000LL, {{kChassis, kWall}}).fatal);
  ASSERT_TRUE(observe(policy, 1002000000LL, {{kChassis, kWall}}).output.has_value());

  for (std::int64_t stamp = 1020000000LL; stamp <= 1180000000LL; stamp += 20000000LL) {
    const auto decision = observe(policy, stamp, {{kChassis, kWall}});
    EXPECT_FALSE(decision.fatal);
    EXPECT_FALSE(decision.output.has_value());
  }
  EXPECT_FALSE(observe(policy, 1199999999LL, {{kChassis, kWall}}).output.has_value());
  EXPECT_FALSE(observe(policy, 1219999999LL, {{kChassis, kWall}}).output.has_value());
  const auto heartbeat = observe(policy, 1239999999LL, {{kChassis, kWall}});
  ASSERT_TRUE(heartbeat.output.has_value());
  EXPECT_EQ(heartbeat.reason, ContactForwardReason::kSteadyStateHeartbeat);
  EXPECT_EQ(heartbeat.output->header.stamp.sec, 1);
  EXPECT_EQ(heartbeat.output->header.stamp.nanosec, 219999999U);
}

TEST(ContactStreamPolicy, RawClockWatchdogAllowsPauseAndFailsOnSilence)
{
  ContactStreamPolicy policy;
  EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).fatal);
  EXPECT_FALSE(observe(policy, 1000000000LL, {{kChassis, kWall}}).fatal);
  ASSERT_TRUE(observe(policy, 1002000000LL, {{kChassis, kWall}}).output.has_value());
  EXPECT_FALSE(policy.observe_clock(1005000000LL).fatal);
  EXPECT_FALSE(policy.observe_clock(1005000000LL).fatal);
  EXPECT_FALSE(policy.observe_clock(1222000000LL).fatal);
  const auto silence = policy.observe_clock(1222000001LL);
  EXPECT_TRUE(silence.fatal);
  EXPECT_EQ(silence.reason, ContactForwardReason::kFatalStructuralInput);
  EXPECT_NE(silence.detail.find("pending"), std::string::npos);
}

TEST(ContactStreamPolicy, PendingSemanticFailureCannotHideBehindRawSilence)
{
  ContactStreamPolicy policy;
  EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).fatal);
  EXPECT_FALSE(observe(policy, 1000000000LL, {{kChassis, kWall}}).fatal);
  ASSERT_TRUE(observe(policy, 1002000000LL, {{kChassis, kWall}}).output.has_value());
  EXPECT_FALSE(policy.observe(
    message_at(1004000000LL, {{"box::link::collision", "wall::link::collision"}})).fatal);
  EXPECT_FALSE(policy.observe_clock(1220000000LL).fatal);
  const auto failure = policy.observe_clock(1224000001LL);
  EXPECT_TRUE(failure.fatal);
  EXPECT_NE(failure.detail.find("semantic"), std::string::npos);
}

TEST(ContactStreamPolicy, SevenSensorEmptyFrameBatchIsAccepted)
{
  ContactStreamPolicy policy;
  EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).output.has_value());
  const std::vector<Pair> pairs = {
    {kLeftWheel, kGround},
    {kRightWheel, kGround},
    {kFrontCaster, kGround},
    {kRearCaster, kGround},
    {kChassis, kWall},
    {kLidar, kWall},
    {kChassis, kBox},
  };
  for (const auto & pair : pairs) {
    EXPECT_FALSE(policy.observe(message_at(1000000000LL, {pair})).fatal);
  }
  const auto decision = observe(policy, 1002000000LL, {{kLeftWheel, kGround}});
  ASSERT_TRUE(decision.output.has_value());
  EXPECT_TRUE(decision.output->header.frame_id.empty());
  EXPECT_EQ(decision.output->contacts.size(), pairs.size());
}

TEST(ContactStreamPolicy, EmptyRawMessageAndZeroPositionContactFailClosed)
{
  {
    ContactStreamPolicy policy;
    const auto decision = policy.observe(message_at(1000000000LL, {}));
    EXPECT_TRUE(decision.fatal);
    EXPECT_FALSE(decision.output.has_value());
  }
  {
    ContactStreamPolicy policy;
    auto message = message_at(1000000000LL, {{kChassis, kWall}});
    message.contacts[0].positions.clear();
    message.contacts[0].normals.clear();
    message.contacts[0].depths.clear();
    message.contacts[0].wrenches.clear();
    const auto decision = policy.observe(message);
    EXPECT_TRUE(decision.fatal);
    EXPECT_FALSE(decision.output.has_value());
  }
}

TEST(ContactStreamPolicy, FirstBatchSilenceFailsClosedBeforeSynchronization)
{
  {
    ContactStreamPolicy policy;
    EXPECT_FALSE(policy.observe(
      message_at(1000000000LL, {{kLeftWheel, kGround}})).fatal);
    EXPECT_FALSE(policy.observe_clock(1220000000LL).fatal);
    const auto silence = policy.observe_clock(1220000001LL);
    EXPECT_TRUE(silence.fatal);
    EXPECT_NE(silence.detail.find("pending"), std::string::npos);
  }
  {
    ContactStreamPolicy policy;
    EXPECT_FALSE(policy.observe(message_at(
      1000000000LL, {{"box::link::collision", "wall::link::collision"}})).fatal);
    const auto silence = policy.observe_clock(1220000001LL);
    EXPECT_TRUE(silence.fatal);
    EXPECT_NE(silence.detail.find("semantic"), std::string::npos);
  }
}

TEST(ContactStreamPolicy, FirstFinalizedSemanticBatchFailsBeforeWarmupOutput)
{
  ContactStreamPolicy policy;
  EXPECT_FALSE(policy.observe(message_at(
    1000000000LL, {{"box::link::collision", "wall::link::collision"}})).fatal);
  const auto decision = observe(policy, 1002000000LL, {{kLeftWheel, kGround}});
  EXPECT_TRUE(decision.fatal);
  EXPECT_FALSE(decision.output.has_value());
  EXPECT_EQ(decision.reason, ContactForwardReason::kFatalSemanticInput);
}

TEST(ContactStreamPolicy, ClockRegressionFailsClosed)
{
  ContactStreamPolicy policy;
  EXPECT_FALSE(policy.observe_clock(1000000000LL).fatal);
  const auto regression = policy.observe_clock(999999999LL);
  EXPECT_TRUE(regression.fatal);
  EXPECT_FALSE(regression.output.has_value());
}

TEST(ContactStreamPolicy, SemanticFailuresEmitOneCompleteSnapshotThenFail)
{
  const std::vector<Pair> invalid_pairs = {
    {"box::link::collision", "wall::link::collision"},
    {"robotest::unknown_link::collision", kGround},
    {"", kGround},
    {"model::collision", kGround},
    {"model::::collision", kGround},
    {"::model::link::collision", kGround},
    {"model::link::collision::", kGround},
  };
  for (const auto & pair : invalid_pairs) {
    ContactStreamPolicy policy;
    EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).output.has_value());
    EXPECT_FALSE(observe(policy, 1000000000LL, {pair}).fatal);
    const auto decision = observe(policy, 1002000000LL, {{kLeftWheel, kGround}});
    EXPECT_TRUE(decision.fatal);
    ASSERT_TRUE(decision.output.has_value());
    ASSERT_EQ(decision.output->contacts.size(), 1U);
    EXPECT_EQ(decision.output->contacts[0].collision1.name, pair.first);
    EXPECT_EQ(decision.output->contacts[0].collision2.name, pair.second);
  }
}

TEST(ContactStreamPolicy, ProducerShapeAndFiniteValueViolationsFailClosed)
{
  const auto fatal_for = [](ros_gz_interfaces::msg::Contacts message) {
      ContactStreamPolicy policy;
      const auto decision = policy.observe(message);
      EXPECT_TRUE(decision.fatal);
      EXPECT_FALSE(decision.output.has_value());
    };
  {
    auto message = message_at(1000000000LL, {{kChassis, kWall}});
    message.contacts[0].positions[0].x = std::numeric_limits<double>::quiet_NaN();
    fatal_for(std::move(message));
  }
  {
    auto message = message_at(1000000000LL, {{kChassis, kWall}});
    message.contacts[0].normals[0].z = std::numeric_limits<double>::infinity();
    fatal_for(std::move(message));
  }
  {
    auto message = message_at(1000000000LL, {{kChassis, kWall}});
    message.contacts[0].depths[0] = -0.001;
    fatal_for(std::move(message));
  }
  {
    auto message = message_at(1000000000LL, {{kChassis, kWall}});
    message.contacts[0].depths[0] = std::numeric_limits<double>::quiet_NaN();
    fatal_for(std::move(message));
  }
  {
    auto message = message_at(1000000000LL, {{kChassis, kWall}});
    message.contacts[0].wrenches[0].body_1_wrench.force.x =
      std::numeric_limits<double>::infinity();
    fatal_for(std::move(message));
  }
  {
    auto message = message_at(1000000000LL, {{kChassis, kWall}});
    message.contacts[0].wrenches[0].header.stamp.nanosec = 1000000000U;
    fatal_for(std::move(message));
  }
  {
    auto message = message_at(1000000000LL, {{kChassis, kWall}});
    message.contacts[0].wrenches[0].header.frame_id =
      std::string(kMaxFrameIdBytes + 1U, 'f');
    fatal_for(std::move(message));
  }
}

TEST(ContactStreamPolicy, OneSameCounterpartPairCanExpireWhileOtherStaysPresent)
{
  ContactStreamPolicy policy;
  const Pair first{kChassis, kWall};
  const Pair second{kLidar, kWall};
  EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).fatal);
  EXPECT_FALSE(observe(policy, 1000000000LL, {first, second}).fatal);
  auto seeded = observe(policy, 1002000000LL, {second});
  ASSERT_TRUE(seeded.output.has_value());
  ASSERT_EQ(seeded.output->contacts.size(), 2U);

  ContactGateDecision transition;
  for (std::int64_t stamp = 1004000000LL; stamp <= 1254000000LL; stamp += 2000000LL) {
    transition = observe(policy, stamp, {second});
  }
  ASSERT_TRUE(transition.output.has_value());
  EXPECT_EQ(transition.reason, ContactForwardReason::kPairSetTransition);
  EXPECT_EQ(transition.active_pair_count, 1U);
  ASSERT_EQ(transition.output->contacts.size(), 1U);
  auto normalized_second = second;
  if (normalized_second.second < normalized_second.first) {
    std::swap(normalized_second.first, normalized_second.second);
  }
  EXPECT_EQ(normalized_output_pairs(transition), std::vector<Pair>({normalized_second}));
}

TEST(ContactStreamPolicy, InternalPairsAreRoutineAndSnapshotPairsAreSorted)
{
  ContactStreamPolicy policy;
  EXPECT_FALSE(observe(policy, 998000000LL, {{kLeftWheel, kGround}}).output.has_value());
  auto message = message_at(1000000000LL, {{kChassis, kBox}, {kChassis, kLidar}});
  const auto before = message.contacts;
  EXPECT_FALSE(policy.observe(message).output.has_value());
  const auto decision = observe(policy, 1002000000LL, {{kLeftWheel, kGround}});
  ASSERT_TRUE(decision.output.has_value());
  EXPECT_FALSE(decision.fatal);
  ASSERT_EQ(decision.output->contacts.size(), 2U);
  auto pairs = normalized_output_pairs(decision);
  EXPECT_TRUE(std::is_sorted(pairs.begin(), pairs.end()));
  for (const auto & contact : before) {
    EXPECT_NE(std::find(decision.output->contacts.begin(), decision.output->contacts.end(),
          contact), decision.output->contacts.end());
  }
}

TEST(ContactStreamPolicy, ThreeHundredTenSecondSteadyCapacityMathRemainsSafe)
{
  constexpr std::size_t duration_seconds = 310U;
  constexpr std::size_t heartbeat_hz = 5U;
  constexpr std::size_t message_capacity = 8192U;
  constexpr std::size_t record_capacity = 32768U;
  constexpr std::size_t steady_messages = duration_seconds * heartbeat_hz + 1U;
  constexpr std::size_t steady_records = steady_messages * kMaxActiveContactRecords;
  static_assert(steady_messages < message_capacity);
  static_assert(steady_records < record_capacity);
  EXPECT_EQ(steady_messages, 1551U);
  EXPECT_EQ(steady_records, 24816U);
}

}  // namespace
}  // namespace robotest_sim

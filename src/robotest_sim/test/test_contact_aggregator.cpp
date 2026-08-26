// Copyright 2026 Hasan Ahmed
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "google/protobuf/unknown_field_set.h"
#include "robotest_sim/contact_aggregator.hpp"
#include "gtest/gtest.h"

namespace robotest_sim
{
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

}  // namespace

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
  EXPECT_NE(overflow.detail.find("interval union"), std::string::npos);
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

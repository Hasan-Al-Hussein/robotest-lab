// Copyright 2026 Hasan Ahmed
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "robotest_faults/fault_protocol.hpp"

namespace robotest_faults
{
namespace
{

using FaultEvent = robotest_interfaces::msg::FaultEvent;
using FaultSpec = robotest_interfaces::msg::FaultSpec;
using ArmRequest = robotest_interfaces::srv::ArmFaultSchedule::Request;
using PreloadRequest =
  robotest_interfaces::srv::PreloadFaultSchedule::Request;

builtin_interfaces::msg::Time make_time(
  const std::int32_t seconds,
  const std::uint32_t nanoseconds = 0U)
{
  builtin_interfaces::msg::Time value;
  value.sec = seconds;
  value.nanosec = nanoseconds;
  return value;
}

builtin_interfaces::msg::Duration make_duration(
  const std::int64_t nanoseconds)
{
  builtin_interfaces::msg::Duration value;
  value.sec = static_cast<std::int32_t>(
    nanoseconds / kNanosecondsPerSecond);
  value.nanosec = static_cast<std::uint32_t>(
    nanoseconds % kNanosecondsPerSecond);
  return value;
}

unique_identifier_msgs::msg::UUID make_uuid(const std::uint8_t marker = 1U)
{
  unique_identifier_msgs::msg::UUID value;
  value.uuid[0] = marker;
  value.uuid[15] = static_cast<std::uint8_t>(marker + 1U);
  return value;
}

FaultSpec make_fault(
  std::string id,
  const std::uint8_t target,
  const std::uint8_t mode,
  const std::int64_t start_ns = 10000000000LL,
  const std::int64_t duration_ns = 2000000000LL,
  std::string parameters = "{}",
  const std::uint64_t seed = 42U)
{
  FaultSpec fault;
  fault.schema_version = FaultSpec::SCHEMA_VERSION;
  fault.fault_id = std::move(id);
  fault.target = target;
  fault.mode = mode;
  fault.start_offset = make_duration(start_ns);
  fault.duration = make_duration(duration_ns);
  fault.seed = seed;
  fault.parameters_json = std::move(parameters);
  return fault;
}

PreloadRequest make_preload(const std::vector<FaultSpec> & faults)
{
  const auto canonical = canonicalize_schedule(faults);
  if (!canonical.accepted) {
    throw std::logic_error("test attempted to preload an invalid schedule");
  }
  PreloadRequest request;
  request.schedule_hash = canonical.schedule.sha256;
  for (const auto & fault : faults) {
    request.faults.push_back(fault);
  }
  return request;
}

ArmRequest make_arm(
  const PreloadOutcome & preload,
  const builtin_interfaces::msg::Time & t0 = make_time(100))
{
  ArmRequest request;
  request.schedule_hash = preload.schedule_hash;
  request.generation = preload.generation;
  request.goal_uuid = make_uuid();
  request.accepted_goal_stamp = t0;
  return request;
}

TEST(CanonicalSchedule, MatchesAllThreeKnownAnswers)
{
  const auto empty = canonicalize_schedule({});
  ASSERT_TRUE(empty.accepted) << empty.message;
  EXPECT_EQ(empty.schedule.bytes, "{\"schema_version\":1,\"faults\":[]}");
  EXPECT_EQ(
    empty.schedule.sha256,
    "26080d7dc8f4108a369962ecad1d2e29941a991af68beb415067eae1dc1de6f8");

  const auto scan = canonicalize_schedule(
    {make_fault(
        "scan_drop", FaultSpec::TARGET_SCAN,
        FaultSpec::MODE_LIDAR_DROPOUT)});
  ASSERT_TRUE(scan.accepted) << scan.message;
  EXPECT_EQ(
    scan.schedule.bytes,
    "{\"schema_version\":1,\"faults\":[{\"schema_version\":1,"
    "\"fault_id\":\"scan_drop\",\"target\":1,\"mode\":1,"
    "\"start_offset_ns\":10000000000,\"duration_ns\":2000000000,"
    "\"seed\":42,\"parameters\":{}}]}");
  EXPECT_EQ(
    scan.schedule.sha256,
    "5f93838ca7c0be214858ffe8f62fa351b82d260f6223f139dfaf6bd3dcd224c8");

  const auto drift = canonicalize_schedule(
    {make_fault(
        "odom_drift", FaultSpec::TARGET_ODOM, FaultSpec::MODE_ODOM_DRIFT,
        10000000000LL, 20000000000LL,
        "{\"x_rate_nm_per_s\":10000000,"
        "\"yaw_rate_nrad_per_s\":5000000}")});
  ASSERT_TRUE(drift.accepted) << drift.message;
  EXPECT_EQ(
    drift.schedule.bytes,
    "{\"schema_version\":1,\"faults\":[{\"schema_version\":1,"
    "\"fault_id\":\"odom_drift\",\"target\":2,\"mode\":4,"
    "\"start_offset_ns\":10000000000,\"duration_ns\":20000000000,"
    "\"seed\":42,\"parameters\":{\"x_rate_nm_per_s\":10000000,"
    "\"yaw_rate_nrad_per_s\":5000000}}]}");
  EXPECT_EQ(
    drift.schedule.sha256,
    "3c72bc48e05221a52058112e58c94b578550deb2ee1ff4ad0d719675917335eb");
}

TEST(CanonicalSchedule, NormalizesWireOrderWhitespaceAndParameterKeyOrder)
{
  const auto first = make_fault(
    "odom", FaultSpec::TARGET_ODOM, FaultSpec::MODE_ODOM_DRIFT,
    10000000000LL, 2000000000LL,
    "{ \"yaw_rate_nrad_per_s\" : 5, \"x_rate_nm_per_s\" : 10 }");
  const auto second = make_fault(
    "scan", FaultSpec::TARGET_SCAN, FaultSpec::MODE_LIDAR_DROPOUT,
    20000000000LL);
  const auto one = canonicalize_schedule({second, first});

  auto normalized_first = first;
  normalized_first.parameters_json =
    "{\"x_rate_nm_per_s\":10,\"yaw_rate_nrad_per_s\":5}";
  const auto two = canonicalize_schedule({normalized_first, second});

  ASSERT_TRUE(one.accepted) << one.message;
  ASSERT_TRUE(two.accepted) << two.message;
  EXPECT_EQ(one.schedule.bytes, two.schedule.bytes);
  EXPECT_EQ(one.schedule.sha256, two.schedule.sha256);
  EXPECT_LT(
    one.schedule.bytes.find("\"fault_id\":\"odom\""),
    one.schedule.bytes.find("\"fault_id\":\"scan\""));
}

TEST(CanonicalSchedule, RejectsDuplicateJsonKeysAndUnknownParameters)
{
  auto duplicate = make_fault(
    "drift", FaultSpec::TARGET_ODOM, FaultSpec::MODE_ODOM_DRIFT,
    10000000000LL, 2000000000LL,
    "{\"x_rate_nm_per_s\":1,\"x_rate_nm_per_s\":2,"
    "\"yaw_rate_nrad_per_s\":3}");
  EXPECT_FALSE(canonicalize_schedule({duplicate}).accepted);

  duplicate.parameters_json =
    "{\"x_rate_nm_per_s\":1,\"yaw_rate_nrad_per_s\":3,\"extra\":0}";
  EXPECT_FALSE(canonicalize_schedule({duplicate}).accepted);
  duplicate.parameters_json =
    "{\"x_rate_nm_per_s\":1.0,\"yaw_rate_nrad_per_s\":3}";
  EXPECT_FALSE(canonicalize_schedule({duplicate}).accepted);
  duplicate.parameters_json = "{not-json}";
  EXPECT_FALSE(canonicalize_schedule({duplicate}).accepted);
  duplicate.parameters_json =
    "{/* comment */\"x_rate_nm_per_s\":1,"
    "\"yaw_rate_nrad_per_s\":3}";
  EXPECT_FALSE(canonicalize_schedule({duplicate}).accepted);
}

TEST(CanonicalSchedule, AcceptsOnlyFrozenTargetModePairs)
{
  EXPECT_TRUE(canonicalize_schedule(
      {make_fault("scan_pass", 1U, 0U)}).accepted);
  EXPECT_TRUE(canonicalize_schedule(
      {make_fault("odom_pass", 2U, 0U)}).accepted);
  EXPECT_TRUE(canonicalize_schedule(
      {make_fault("imu_pass", 3U, 0U)}).accepted);
  EXPECT_TRUE(canonicalize_schedule(
      {make_fault("drop", 1U, 1U)}).accepted);
  EXPECT_TRUE(canonicalize_schedule(
      {make_fault(
          "drift", 2U, 4U, 10000000000LL, 2000000000LL,
          "{\"x_rate_nm_per_s\":1,\"yaw_rate_nrad_per_s\":0}")}).accepted);

  EXPECT_FALSE(canonicalize_schedule(
      {make_fault("bad", 2U, 1U)}).accepted);
  EXPECT_FALSE(canonicalize_schedule(
      {make_fault("bad", 3U, 6U)}).accepted);
  EXPECT_FALSE(canonicalize_schedule(
      {make_fault("bad", 0U, 0U)}).accepted);
}

TEST(CanonicalSchedule, EnforcesCountIdentifiersTimingAndRateBounds)
{
  std::vector<FaultSpec> sixteen;
  for (std::size_t index = 0U; index < kMaximumFaultCount; ++index) {
    sixteen.push_back(make_fault(
        "f" + std::to_string(index), FaultSpec::TARGET_SCAN,
        FaultSpec::MODE_PASS_THROUGH,
        500000000LL + static_cast<std::int64_t>(index) * 2000000000LL,
        1000000000LL));
  }
  ASSERT_TRUE(canonicalize_schedule(sixteen).accepted);
  EXPECT_EQ(PreloadRequest().faults.max_size(), kMaximumFaultCount);
  sixteen.push_back(make_fault(
      "overflow", FaultSpec::TARGET_SCAN, FaultSpec::MODE_PASS_THROUGH,
      40000000000LL, 1000000000LL));
  EXPECT_FALSE(canonicalize_schedule(sixteen).accepted);

  auto fault = make_fault("bad id", 1U, 0U);
  EXPECT_FALSE(canonicalize_schedule({fault}).accepted);
  fault = make_fault("\xC3\xA9", 1U, 0U);
  EXPECT_FALSE(canonicalize_schedule({fault}).accepted);
  fault = make_fault("schema", 1U, 0U);
  fault.schema_version = 2U;
  EXPECT_FALSE(canonicalize_schedule({fault}).accepted);
  fault = make_fault(std::string(65U, 'a'), 1U, 0U);
  EXPECT_FALSE(canonicalize_schedule({fault}).accepted);
  EXPECT_FALSE(canonicalize_schedule(
      {make_fault("duplicate", 1U, 0U),
        make_fault("duplicate", 2U, 0U)}).accepted);

  fault = make_fault("early", 1U, 0U, 499999999LL);
  EXPECT_FALSE(canonicalize_schedule({fault}).accepted);
  fault = make_fault("zero_duration", 1U, 0U, 500000000LL, 0LL);
  EXPECT_FALSE(canonicalize_schedule({fault}).accepted);
  fault = make_fault(
    "late_end", 1U, 0U, kMaximumScheduleNanoseconds, 1LL);
  EXPECT_FALSE(canonicalize_schedule({fault}).accepted);
  fault = make_fault("not_normalized", 1U, 0U);
  fault.start_offset.nanosec = 1000000000U;
  EXPECT_FALSE(canonicalize_schedule({fault}).accepted);

  fault = make_fault(
    "zero_drift", 2U, 4U, 10000000000LL, 2000000000LL,
    "{\"x_rate_nm_per_s\":0,\"yaw_rate_nrad_per_s\":0}");
  EXPECT_FALSE(canonicalize_schedule({fault}).accepted);
  fault.parameters_json =
    "{\"x_rate_nm_per_s\":1000000001,\"yaw_rate_nrad_per_s\":0}";
  EXPECT_FALSE(canonicalize_schedule({fault}).accepted);
  fault = make_fault("pass_parameters", 1U, 0U);
  fault.parameters_json = "{\"unexpected\":1}";
  EXPECT_FALSE(canonicalize_schedule({fault}).accepted);

  fault = make_fault(
    "maximum_end", 1U, 0U, kMaximumScheduleNanoseconds - 1LL, 1LL);
  EXPECT_TRUE(canonicalize_schedule({fault}).accepted);
}

TEST(CanonicalSchedule, RejectsSameTargetOverlapButAllowsTouchAndCrossTarget)
{
  const auto scan_one = make_fault(
    "one", 1U, 0U, 1000000000LL, 2000000000LL);
  const auto scan_overlap = make_fault(
    "overlap", 1U, 1U, 2000000000LL, 2000000000LL);
  EXPECT_FALSE(canonicalize_schedule({scan_one, scan_overlap}).accepted);

  const auto scan_touch = make_fault(
    "touch", 1U, 1U, 3000000000LL, 2000000000LL);
  EXPECT_TRUE(canonicalize_schedule({scan_touch, scan_one}).accepted);

  const auto odom_overlap = make_fault(
    "cross", 2U, 0U, 2000000000LL, 2000000000LL);
  EXPECT_TRUE(canonicalize_schedule({scan_one, odom_overlap}).accepted);
}

TEST(FaultProtocolState, PreloadReplayReplaceArmReplayAndResetAreExact)
{
  FaultProtocol protocol;
  const auto first_request = make_preload(
    {make_fault("scan", 1U, 1U)});
  auto first = protocol.preload(first_request, make_time(90));
  ASSERT_TRUE(first.accepted) << first.message;
  EXPECT_FALSE(first.replayed);
  EXPECT_EQ(first.state, FaultControlState::kPrepared);
  EXPECT_EQ(first.generation, 1U);
  EXPECT_EQ(first.event.event_sequence, 1U);
  EXPECT_EQ(first.event.state_before, FaultEvent::STATE_RESET);
  EXPECT_EQ(first.event.state_after, FaultEvent::STATE_PREPARED);
  EXPECT_EQ(first.event.requested_fault_count, 1U);
  EXPECT_EQ(first.event.committed_fault_count, 1U);

  auto replay = protocol.preload(first_request, make_time(91));
  ASSERT_TRUE(replay.accepted);
  EXPECT_TRUE(replay.replayed);
  EXPECT_EQ(replay.generation, 1U);
  EXPECT_EQ(replay.event.event_sequence, 2U);

  auto equivalent = first_request;
  equivalent.faults.front().parameters_json = "{   }";
  replay = protocol.preload(equivalent, make_time(92));
  EXPECT_TRUE(replay.accepted);
  EXPECT_TRUE(replay.replayed);
  EXPECT_EQ(replay.generation, 1U);

  const auto second_request = make_preload(
    {make_fault("odom", 2U, 0U)});
  const auto second = protocol.preload(second_request, make_time(93));
  ASSERT_TRUE(second.accepted);
  EXPECT_FALSE(second.replayed);
  EXPECT_EQ(second.generation, 2U);

  const auto arm_request = make_arm(second);
  const auto armed = protocol.arm(arm_request, make_time(100, 100U));
  ASSERT_TRUE(armed.accepted) << armed.message;
  EXPECT_FALSE(armed.replayed);
  EXPECT_EQ(armed.state, FaultControlState::kArmed);
  EXPECT_EQ(armed.arm_margin_ns, 9999999900LL);
  EXPECT_EQ(armed.event.event_type, FaultEvent::EVENT_ARMED);
  EXPECT_EQ(armed.event.requested_generation, 2U);
  EXPECT_EQ(armed.event.committed_generation, 2U);
  EXPECT_EQ(armed.event.bound_goal_uuid.uuid, arm_request.goal_uuid.uuid);

  const auto arm_replay = protocol.arm(arm_request, make_time(101));
  ASSERT_TRUE(arm_replay.accepted);
  EXPECT_TRUE(arm_replay.replayed);
  EXPECT_EQ(arm_replay.arm_commit_stamp.sec, 100);
  EXPECT_EQ(arm_replay.arm_commit_stamp.nanosec, 100U);

  const auto armed_preload_replay =
    protocol.preload(second_request, make_time(102));
  EXPECT_TRUE(armed_preload_replay.accepted);
  EXPECT_TRUE(armed_preload_replay.replayed);
  const auto rejected = protocol.preload(first_request, make_time(103));
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.generation, 2U);

  const auto reset = protocol.reset(make_time(104));
  EXPECT_EQ(reset.state_before, FaultControlState::kArmed);
  EXPECT_FALSE(reset.event.replayed);
  auto snapshot = protocol.snapshot();
  EXPECT_EQ(snapshot.state, FaultControlState::kReset);
  EXPECT_EQ(snapshot.generation, 0U);
  EXPECT_TRUE(snapshot.schedule_hash.empty());

  const auto reset_replay = protocol.reset(make_time(105));
  EXPECT_TRUE(reset_replay.event.replayed);
  const auto third = protocol.preload(first_request, make_time(106));
  EXPECT_TRUE(third.accepted);
  EXPECT_EQ(third.generation, 3U);
}

TEST(FaultProtocolState, RejectionsAreAtomicAndNeverAllocateGeneration)
{
  FaultProtocol protocol;
  auto request = make_preload({make_fault("scan", 1U, 1U)});
  request.schedule_hash.assign(64U, '0');
  const auto bad_hash = protocol.preload(request, make_time(1));
  EXPECT_FALSE(bad_hash.accepted);
  EXPECT_EQ(bad_hash.event.event_type, FaultEvent::EVENT_SCHEDULE_REJECTED);
  EXPECT_EQ(protocol.snapshot().state, FaultControlState::kReset);
  EXPECT_EQ(protocol.snapshot().generation, 0U);

  auto mixed = make_preload({make_fault("valid", 1U, 0U)});
  mixed.faults.push_back(make_fault("invalid", 2U, 1U));
  const auto rejected = protocol.preload(mixed, make_time(2));
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(protocol.snapshot().state, FaultControlState::kReset);

  const auto valid = protocol.preload(
    make_preload({make_fault("valid", 1U, 0U)}), make_time(3));
  EXPECT_TRUE(valid.accepted);
  EXPECT_EQ(valid.generation, 1U);
}

TEST(FaultProtocolState, ArmValidatesEveryBindingAndMarginPrecondition)
{
  FaultProtocol protocol;
  ArmRequest arm_without_preload;
  arm_without_preload.schedule_hash.assign(64U, '0');
  arm_without_preload.generation = 1U;
  arm_without_preload.goal_uuid = make_uuid();
  arm_without_preload.accepted_goal_stamp = make_time(100);
  EXPECT_FALSE(protocol.arm(arm_without_preload, make_time(100)).accepted);

  const auto preload = protocol.preload(
    make_preload(
      {make_fault("early", 1U, 1U, 500000000LL, 1000000000LL)}),
    make_time(90));
  ASSERT_TRUE(preload.accepted);
  auto request = make_arm(preload);

  auto invalid = request;
  invalid.schedule_hash.assign(64U, '0');
  EXPECT_FALSE(protocol.arm(invalid, make_time(100)).accepted);
  invalid = request;
  ++invalid.generation;
  EXPECT_FALSE(protocol.arm(invalid, make_time(100)).accepted);
  invalid = request;
  invalid.goal_uuid = unique_identifier_msgs::msg::UUID();
  EXPECT_FALSE(protocol.arm(invalid, make_time(100)).accepted);
  invalid = request;
  invalid.accepted_goal_stamp = make_time(0);
  EXPECT_FALSE(protocol.arm(invalid, make_time(100)).accepted);
  invalid = request;
  invalid.accepted_goal_stamp.nanosec = 1000000000U;
  EXPECT_FALSE(protocol.arm(invalid, make_time(100)).accepted);
  EXPECT_FALSE(protocol.arm(request, make_time(99, 999999999U)).accepted);
  EXPECT_FALSE(protocol.arm(request, make_time(100, 1U)).accepted);
  EXPECT_EQ(protocol.snapshot().state, FaultControlState::kPrepared);

  const auto armed = protocol.arm(request, make_time(100));
  ASSERT_TRUE(armed.accepted) << armed.message;
  EXPECT_EQ(armed.arm_margin_ns, 500000000LL);

  auto different = request;
  different.goal_uuid = make_uuid(7U);
  EXPECT_FALSE(protocol.arm(different, make_time(100)).accepted);
  different = request;
  different.schedule_hash.assign(64U, '0');
  EXPECT_FALSE(protocol.arm(different, make_time(100)).accepted);
  different = request;
  ++different.generation;
  EXPECT_FALSE(protocol.arm(different, make_time(100)).accepted);
  different = request;
  different.accepted_goal_stamp = make_time(100, 1U);
  EXPECT_FALSE(protocol.arm(different, make_time(100)).accepted);
  EXPECT_EQ(protocol.snapshot().goal_uuid.uuid, request.goal_uuid.uuid);
}

TEST(FaultProtocolState, EmptyScheduleArmsWithZeroMargin)
{
  FaultProtocol protocol;
  const auto preload = protocol.preload(make_preload({}), make_time(90));
  ASSERT_TRUE(preload.accepted);
  const auto armed = protocol.arm(make_arm(preload), make_time(100));
  EXPECT_TRUE(armed.accepted) << armed.message;
  EXPECT_EQ(armed.arm_margin_ns, 0);
}

TEST(FaultProtocolState, RejectsGenerationAndRosTimeOverflow)
{
  FaultProtocol exhausted(std::numeric_limits<std::uint64_t>::max());
  EXPECT_FALSE(exhausted.preload(make_preload({}), make_time(1)).accepted);
  EXPECT_EQ(exhausted.snapshot().state, FaultControlState::kReset);

  FaultProtocol protocol;
  const auto preload = protocol.preload(
    make_preload(
      {make_fault("late", 1U, 0U, 500000000LL, 1000000000LL)}),
    make_time(1));
  ASSERT_TRUE(preload.accepted);
  auto request = make_arm(
    preload, make_time(std::numeric_limits<std::int32_t>::max()));
  EXPECT_FALSE(protocol.arm(
      request, make_time(std::numeric_limits<std::int32_t>::max())).accepted);
}

TEST(FaultProtocolConcurrency, CompetingPreloadsCommitOnlyWholeGenerations)
{
  FaultProtocol protocol;
  const auto request_a = make_preload({make_fault("a", 1U, 0U)});
  const auto request_b = make_preload({make_fault("b", 2U, 0U)});
  std::atomic<bool> start{false};
  PreloadOutcome outcome_a;
  PreloadOutcome outcome_b;

  std::thread thread_a([&]() {
      while (!start.load()) {}
      outcome_a = protocol.preload(request_a, make_time(1));
    });
  std::thread thread_b([&]() {
      while (!start.load()) {}
      outcome_b = protocol.preload(request_b, make_time(1));
    });
  start.store(true);
  thread_a.join();
  thread_b.join();

  ASSERT_TRUE(outcome_a.accepted);
  ASSERT_TRUE(outcome_b.accepted);
  EXPECT_EQ(
    std::set<std::uint64_t>({outcome_a.generation, outcome_b.generation}),
    std::set<std::uint64_t>({1U, 2U}));
  const auto snapshot = protocol.snapshot();
  EXPECT_EQ(snapshot.state, FaultControlState::kPrepared);
  EXPECT_EQ(snapshot.generation, 2U);
  EXPECT_TRUE(
    snapshot.schedule_hash == request_a.schedule_hash ||
    snapshot.schedule_hash == request_b.schedule_hash);
  EXPECT_TRUE(
    snapshot.canonical_bytes.find("\"fault_id\":\"a\"") !=
    std::string::npos ||
    snapshot.canonical_bytes.find("\"fault_id\":\"b\"") !=
    std::string::npos);
  EXPECT_EQ(
    snapshot.canonical_bytes.find("\"fault_id\":\"a\"") !=
    std::string::npos,
    snapshot.schedule_hash == request_a.schedule_hash);
}

TEST(FaultProtocolConcurrency, ArmAndReplacementNeverProducePartialBinding)
{
  FaultProtocol protocol;
  const auto request_a = make_preload({make_fault("a", 1U, 0U)});
  const auto prepared = protocol.preload(request_a, make_time(1));
  ASSERT_TRUE(prepared.accepted);
  const auto arm_request = make_arm(prepared);
  const auto request_b = make_preload({make_fault("b", 2U, 0U)});
  std::atomic<bool> start{false};
  ArmOutcome arm_outcome;
  PreloadOutcome preload_outcome;

  std::thread arm_thread([&]() {
      while (!start.load()) {}
      arm_outcome = protocol.arm(arm_request, make_time(100));
    });
  std::thread preload_thread([&]() {
      while (!start.load()) {}
      preload_outcome = protocol.preload(request_b, make_time(100));
    });
  start.store(true);
  arm_thread.join();
  preload_thread.join();

  const auto snapshot = protocol.snapshot();
  if (arm_outcome.accepted) {
    EXPECT_FALSE(preload_outcome.accepted);
    EXPECT_EQ(snapshot.state, FaultControlState::kArmed);
    EXPECT_EQ(snapshot.generation, prepared.generation);
    EXPECT_EQ(snapshot.goal_uuid.uuid, arm_request.goal_uuid.uuid);
  } else {
    EXPECT_TRUE(preload_outcome.accepted);
    EXPECT_EQ(snapshot.state, FaultControlState::kPrepared);
    EXPECT_EQ(snapshot.generation, prepared.generation + 1U);
    EXPECT_EQ(snapshot.schedule_hash, request_b.schedule_hash);
  }
}

}  // namespace
}  // namespace robotest_faults

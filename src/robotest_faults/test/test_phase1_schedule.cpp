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

#include <string>

#include "robotest_faults/phase1_schedule.hpp"
#include "gtest/gtest.h"

namespace robotest_faults
{
namespace
{

using LoadFaultSchedule = robotest_interfaces::srv::LoadFaultSchedule;
using FaultSpec = robotest_interfaces::msg::FaultSpec;

LoadFaultSchedule::Request
valid_empty_request(const char hash_character = 'a')
{
  LoadFaultSchedule::Request request;
  request.schedule_hash = std::string(64U, hash_character);
  request.mission_start.sec = 10;
  request.mission_start.nanosec = 42U;
  return request;
}

FaultSpec pass_through_fault(const std::string & fault_id)
{
  FaultSpec fault;
  fault.schema_version = FaultSpec::SCHEMA_VERSION;
  fault.fault_id = fault_id;
  fault.target = FaultSpec::TARGET_SCAN;
  fault.mode = FaultSpec::MODE_PASS_THROUGH;
  fault.start_offset.sec = 1;
  fault.duration.sec = 2;
  fault.seed = 12345U;
  fault.parameters_json = "{}";
  return fault;
}

TEST(Phase1Schedule, AcceptsEmptyAndExplicitPassThroughSchedules) {
  Phase1ScheduleState state;
  auto request = valid_empty_request();
  auto result = state.validate_and_replace(request);
  ASSERT_TRUE(result.accepted) << result.message;
  EXPECT_EQ(result.loaded_count, 0U);

  request.faults.push_back(pass_through_fault("baseline.scan"));
  result = state.validate_and_replace(request);
  ASSERT_TRUE(result.accepted) << result.message;
  EXPECT_EQ(result.loaded_count, 1U);
  ASSERT_EQ(state.faults().size(), 1U);
  EXPECT_EQ(state.faults().front().fault_id, "baseline.scan");
}

TEST(Phase1Schedule,
     UnsupportedFaultIsRejectedAtomicallyAndPreservesPriorState) {
  Phase1ScheduleState state;
  auto initial = valid_empty_request('a');
  initial.faults.push_back(pass_through_fault("accepted.baseline"));
  ASSERT_TRUE(state.validate_and_replace(initial).accepted);

  auto rejected = valid_empty_request('b');
  auto unsupported = pass_through_fault("unsupported.dropout");
  unsupported.mode = FaultSpec::MODE_LIDAR_DROPOUT;
  rejected.faults.push_back(unsupported);

  const auto result = state.validate_and_replace(rejected);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.loaded_count, 0U);
  EXPECT_EQ(state.schedule_hash(), std::string(64U, 'a'));
  ASSERT_EQ(state.faults().size(), 1U);
  EXPECT_EQ(state.faults().front().fault_id, "accepted.baseline");
}

TEST(Phase1Schedule, RejectsDuplicateIdsBadHashAndNegativeDuration) {
  Phase1ScheduleState state;

  auto duplicate = valid_empty_request();
  duplicate.faults.push_back(pass_through_fault("same-id"));
  duplicate.faults.push_back(pass_through_fault("same-id"));
  EXPECT_FALSE(state.validate_and_replace(duplicate).accepted);

  auto bad_hash = valid_empty_request('A');
  EXPECT_FALSE(state.validate_and_replace(bad_hash).accepted);

  auto negative_duration = valid_empty_request();
  auto fault = pass_through_fault("negative-duration");
  fault.duration.sec = -1;
  negative_duration.faults.push_back(fault);
  EXPECT_FALSE(state.validate_and_replace(negative_duration).accepted);
}

TEST(Phase1Schedule, ResetClearsScheduleAndDeterministicRunState) {
  Phase1ScheduleState state;
  auto request = valid_empty_request();
  request.faults.push_back(pass_through_fault("baseline"));
  ASSERT_TRUE(state.validate_and_replace(request).accepted);

  state.reset();
  EXPECT_TRUE(state.schedule_hash().empty());
  EXPECT_TRUE(state.faults().empty());
  EXPECT_EQ(state.mission_start().sec, 0);
  EXPECT_EQ(state.mission_start().nanosec, 0U);
}

}  // namespace
}  // namespace robotest_faults

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

#include "robotest_faults/phase1_schedule.hpp"

#include <cctype>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace robotest_faults
{
namespace
{

constexpr uint32_t kNanosecondsPerSecond = 1000000000U;

bool valid_time(const builtin_interfaces::msg::Time & time)
{
  return time.sec >= 0 && time.nanosec < kNanosecondsPerSecond;
}

bool valid_duration(const builtin_interfaces::msg::Duration & duration)
{
  return duration.sec >= 0 && duration.nanosec < kNanosecondsPerSecond;
}

bool valid_hash(const std::string & hash)
{
  if (hash.size() != 64U) {
    return false;
  }
  for (const unsigned char character : hash) {
    if (!std::isdigit(character) && !(character >= 'a' && character <= 'f')) {
      return false;
    }
  }
  return true;
}

bool valid_fault_id(const std::string & fault_id)
{
  if (fault_id.empty() || fault_id.size() > 64U) {
    return false;
  }
  for (const unsigned char character : fault_id) {
    if (!std::isalnum(character) && character != '.' && character != '_' &&
      character != '-')
    {
      return false;
    }
  }
  return std::isalnum(static_cast<unsigned char>(fault_id.front())) != 0;
}

bool valid_target(const uint8_t target)
{
  using FaultSpec = robotest_interfaces::msg::FaultSpec;
  return target == FaultSpec::TARGET_SCAN || target == FaultSpec::TARGET_ODOM ||
         target == FaultSpec::TARGET_IMU;
}

ScheduleLoadResult reject(const std::string & message)
{
  return ScheduleLoadResult{false, 0U, message};
}

}  // namespace

ScheduleLoadResult Phase1ScheduleState::validate_and_replace(
  const robotest_interfaces::srv::LoadFaultSchedule::Request & request)
{
  using FaultSpec = robotest_interfaces::msg::FaultSpec;

  if (!valid_hash(request.schedule_hash)) {
    return reject(
        "schedule_hash must be a 64-character lowercase SHA-256 digest");
  }
  if (!valid_time(request.mission_start)) {
    return reject("mission_start is outside the ROS time domain");
  }

  std::unordered_set<std::string> fault_ids;
  std::vector<FaultSpec> candidate;
  candidate.reserve(request.faults.size());
  for (const auto & fault : request.faults) {
    if (fault.schema_version != FaultSpec::SCHEMA_VERSION) {
      return reject("fault specification has an unsupported schema_version");
    }
    if (!valid_fault_id(fault.fault_id)) {
      return reject("fault_id must use 1-64 alphanumeric, dot, underscore, or "
                    "hyphen characters");
    }
    if (!fault_ids.insert(fault.fault_id).second) {
      return reject("fault_id values must be unique within a schedule");
    }
    if (!valid_target(fault.target)) {
      return reject("fault specification has an unsupported target stream");
    }
    if (!valid_duration(fault.start_offset) ||
      !valid_duration(fault.duration))
    {
      return reject("fault offsets and durations must be non-negative "
                    "normalized durations");
    }
    if (fault.mode != FaultSpec::MODE_PASS_THROUGH) {
      return reject("Phase 1 accepts only MODE_PASS_THROUGH specifications");
    }
    if (!fault.parameters_json.empty() && fault.parameters_json != "{}") {
      return reject("MODE_PASS_THROUGH parameters_json must be empty or {}");
    }
    candidate.push_back(fault);
  }

  // Commit only after every element validates; failures leave prior state
  // intact.
  schedule_hash_ = request.schedule_hash;
  mission_start_ = request.mission_start;
  faults_ = std::move(candidate);
  return ScheduleLoadResult{true, faults_.size(),
    "pass-through schedule loaded atomically"};
}

void Phase1ScheduleState::reset()
{
  schedule_hash_.clear();
  mission_start_ = builtin_interfaces::msg::Time{};
  faults_.clear();
}

const std::string & Phase1ScheduleState::schedule_hash() const noexcept
{
  return schedule_hash_;
}

const builtin_interfaces::msg::Time &
Phase1ScheduleState::mission_start() const noexcept
{
  return mission_start_;
}

const std::vector<robotest_interfaces::msg::FaultSpec> &
Phase1ScheduleState::faults() const noexcept
{
  return faults_;
}

}  // namespace robotest_faults

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

#ifndef ROBOTEST_FAULTS__PHASE1_SCHEDULE_HPP_
#define ROBOTEST_FAULTS__PHASE1_SCHEDULE_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "robotest_interfaces/msg/fault_spec.hpp"
#include "robotest_interfaces/srv/load_fault_schedule.hpp"

namespace robotest_faults
{

struct ScheduleLoadResult
{
  bool accepted{false};
  std::size_t loaded_count{0U};
  std::string message{};
};

class Phase1ScheduleState {
public:
  ScheduleLoadResult validate_and_replace(
    const robotest_interfaces::srv::LoadFaultSchedule::Request & request);

  void reset();

  const std::string & schedule_hash() const noexcept;
  const builtin_interfaces::msg::Time & mission_start() const noexcept;
  const std::vector<robotest_interfaces::msg::FaultSpec> &
  faults() const noexcept;

private:
  std::string schedule_hash_{};
  builtin_interfaces::msg::Time mission_start_{};
  std::vector<robotest_interfaces::msg::FaultSpec> faults_{};
};

}  // namespace robotest_faults

#endif  // ROBOTEST_FAULTS__PHASE1_SCHEDULE_HPP_

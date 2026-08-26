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

#include "robotest_faults/fault_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_set>
#include <utility>

#include "nlohmann/json.hpp"
#include "openssl/evp.h"
#include "robotest_faults/pass_through.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robotest_faults
{
namespace
{

using FaultEvent = robotest_interfaces::msg::FaultEvent;
using FaultSpec = robotest_interfaces::msg::FaultSpec;
using Json = nlohmann::json;

constexpr std::size_t kSha256ByteCount = 32U;

struct ParsedParameters
{
  bool accepted{false};
  std::int64_t x_rate_nm_per_s{0};
  std::int64_t yaw_rate_nrad_per_s{0};
  std::string canonical_json{};
  std::string message{};
};

bool normalized_time(const builtin_interfaces::msg::Time & value)
{
  return value.sec >= 0 &&
         value.nanosec < static_cast<std::uint32_t>(kNanosecondsPerSecond);
}

bool positive_time(const builtin_interfaces::msg::Time & value)
{
  return normalized_time(value) && (value.sec > 0 || value.nanosec > 0U);
}

bool normalized_duration(const builtin_interfaces::msg::Duration & value)
{
  return value.sec >= 0 &&
         value.nanosec < static_cast<std::uint32_t>(kNanosecondsPerSecond);
}

std::int64_t time_to_nanoseconds(const builtin_interfaces::msg::Time & value)
{
  return static_cast<std::int64_t>(value.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(value.nanosec);
}

std::int64_t duration_to_nanoseconds(
  const builtin_interfaces::msg::Duration & value)
{
  return static_cast<std::int64_t>(value.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(value.nanosec);
}

builtin_interfaces::msg::Time nanoseconds_to_time(const std::int64_t value)
{
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(value / kNanosecondsPerSecond);
  result.nanosec = static_cast<std::uint32_t>(value % kNanosecondsPerSecond);
  return result;
}

bool same_time(
  const builtin_interfaces::msg::Time & left,
  const builtin_interfaces::msg::Time & right)
{
  return left.sec == right.sec && left.nanosec == right.nanosec;
}

bool same_uuid(
  const unique_identifier_msgs::msg::UUID & left,
  const unique_identifier_msgs::msg::UUID & right)
{
  return left.uuid == right.uuid;
}

bool zero_uuid(const unique_identifier_msgs::msg::UUID & value)
{
  return std::all_of(
    value.uuid.begin(), value.uuid.end(),
    [](const std::uint8_t byte) {return byte == 0U;});
}

bool lowercase_sha256(const std::string & value)
{
  if (value.size() != 64U) {
    return false;
  }
  return std::all_of(
    value.begin(), value.end(),
    [](const unsigned char character) {
      return (character >= static_cast<unsigned char>('0') &&
             character <= static_cast<unsigned char>('9')) ||
             (character >= static_cast<unsigned char>('a') &&
             character <= static_cast<unsigned char>('f'));
    });
}

bool valid_fault_id(const std::string & value)
{
  const auto ascii_alphanumeric = [](const unsigned char character) {
      return (character >= static_cast<unsigned char>('0') &&
             character <= static_cast<unsigned char>('9')) ||
             (character >= static_cast<unsigned char>('A') &&
             character <= static_cast<unsigned char>('Z')) ||
             (character >= static_cast<unsigned char>('a') &&
             character <= static_cast<unsigned char>('z'));
    };
  if (value.empty() || value.size() > 64U ||
    !ascii_alphanumeric(static_cast<unsigned char>(value.front())))
  {
    return false;
  }
  return std::all_of(
    value.begin(), value.end(),
    [&ascii_alphanumeric](const unsigned char character) {
      return ascii_alphanumeric(character) || character == '.' ||
             character == '_' || character == '-';
    });
}

std::optional<std::int64_t> bounded_json_integer(
  const Json & value)
{
  if (value.is_number_unsigned()) {
    const auto unsigned_value = value.get<std::uint64_t>();
    if (unsigned_value >
      static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
    {
      return std::nullopt;
    }
    return static_cast<std::int64_t>(unsigned_value);
  }
  if (value.is_number_integer()) {
    return value.get<std::int64_t>();
  }
  return std::nullopt;
}

ParsedParameters parse_parameters(
  const FaultSpec & fault)
{
  ParsedParameters result;
  if (fault.parameters_json.empty()) {
    result.message = "parameters_json must contain a JSON object";
    return result;
  }

  bool duplicate_key = false;
  std::vector<std::unordered_set<std::string>> object_keys;
  const Json::parser_callback_t callback =
    [&duplicate_key, &object_keys](
    const int,
    const Json::parse_event_t event,
    Json & parsed) {
      if (event == Json::parse_event_t::object_start) {
        object_keys.emplace_back();
      } else if (event == Json::parse_event_t::key) {
        if (object_keys.empty()) {
          duplicate_key = true;
        } else {
          const auto & key = parsed.get_ref<const std::string &>();
          if (!object_keys.back().insert(key).second) {
            duplicate_key = true;
          }
        }
      } else if (event == Json::parse_event_t::object_end) {
        if (!object_keys.empty()) {
          object_keys.pop_back();
        }
      }
      return true;
    };

  Json parameters;
  try {
    parameters = Json::parse(
      fault.parameters_json, callback, true, false);
  } catch (const Json::exception &) {
    result.message = "parameters_json is not strict UTF-8 JSON";
    return result;
  }
  if (duplicate_key) {
    result.message = "parameters_json contains a duplicate object key";
    return result;
  }
  if (!parameters.is_object()) {
    result.message = "parameters_json must decode to an object";
    return result;
  }

  if (fault.mode == FaultSpec::MODE_PASS_THROUGH ||
    fault.mode == FaultSpec::MODE_LIDAR_DROPOUT)
  {
    if (!parameters.empty()) {
      result.message = "pass-through and LiDAR dropout parameters must be {}";
      return result;
    }
    result.accepted = true;
    result.canonical_json = "{}";
    return result;
  }

  if (fault.mode != FaultSpec::MODE_ODOM_DRIFT) {
    result.message = "fault mode is not implemented in Phase 3";
    return result;
  }
  if (parameters.size() != 2U ||
    !parameters.contains("x_rate_nm_per_s") ||
    !parameters.contains("yaw_rate_nrad_per_s"))
  {
    result.message =
      "odometry drift requires exactly x_rate_nm_per_s and "
      "yaw_rate_nrad_per_s";
    return result;
  }

  const auto x_rate =
    bounded_json_integer(parameters.at("x_rate_nm_per_s"));
  const auto yaw_rate =
    bounded_json_integer(parameters.at("yaw_rate_nrad_per_s"));
  if (!x_rate.has_value() || !yaw_rate.has_value()) {
    result.message = "odometry drift rates must be signed 64-bit integers";
    return result;
  }
  if (*x_rate < -kMaximumDriftRateScaled ||
    *x_rate > kMaximumDriftRateScaled ||
    *yaw_rate < -kMaximumDriftRateScaled ||
    *yaw_rate > kMaximumDriftRateScaled)
  {
    result.message = "odometry drift rate exceeds the Phase 3 bound";
    return result;
  }
  if (*x_rate == 0 && *yaw_rate == 0) {
    result.message = "odometry drift must configure at least one nonzero rate";
    return result;
  }

  result.accepted = true;
  result.x_rate_nm_per_s = *x_rate;
  result.yaw_rate_nrad_per_s = *yaw_rate;
  result.canonical_json =
    "{\"x_rate_nm_per_s\":" + std::to_string(*x_rate) +
    ",\"yaw_rate_nrad_per_s\":" + std::to_string(*yaw_rate) + "}";
  return result;
}

bool valid_target_mode(const std::uint8_t target, const std::uint8_t mode)
{
  if (mode == FaultSpec::MODE_PASS_THROUGH) {
    return target == FaultSpec::TARGET_SCAN ||
           target == FaultSpec::TARGET_ODOM ||
           target == FaultSpec::TARGET_IMU;
  }
  if (mode == FaultSpec::MODE_LIDAR_DROPOUT) {
    return target == FaultSpec::TARGET_SCAN;
  }
  if (mode == FaultSpec::MODE_ODOM_DRIFT) {
    return target == FaultSpec::TARGET_ODOM;
  }
  return false;
}

std::string canonical_fault_json(const CanonicalFault & fault)
{
  const auto & specification = fault.specification;
  return
    "{\"schema_version\":1,\"fault_id\":\"" +
    specification.fault_id + "\",\"target\":" +
    std::to_string(specification.target) + ",\"mode\":" +
    std::to_string(specification.mode) + ",\"start_offset_ns\":" +
    std::to_string(fault.start_offset_ns) + ",\"duration_ns\":" +
    std::to_string(fault.duration_ns) + ",\"seed\":" +
    std::to_string(specification.seed) + ",\"parameters\":" +
    specification.parameters_json + '}';
}

std::size_t stream_index(const std::uint8_t target)
{
  if (target > FaultSpec::TARGET_IMU) {
    throw std::logic_error("fault target has no stream counter");
  }
  return static_cast<std::size_t>(target);
}

bool increment_counter(std::uint64_t & value)
{
  if (value == std::numeric_limits<std::uint64_t>::max()) {
    return false;
  }
  ++value;
  return true;
}

nav_msgs::msg::Odometry apply_odom_drift(
  const nav_msgs::msg::Odometry & input,
  const CanonicalFault & fault,
  const std::int64_t elapsed_ns)
{
  nav_msgs::msg::Odometry output = input;
  const double elapsed_s =
    static_cast<double>(elapsed_ns) /
    static_cast<double>(kNanosecondsPerSecond);
  const double x_rate =
    static_cast<double>(fault.x_rate_nm_per_s) /
    static_cast<double>(kNanosecondsPerSecond);
  const double yaw_rate =
    static_cast<double>(fault.yaw_rate_nrad_per_s) /
    static_cast<double>(kNanosecondsPerSecond);
  const double dx = x_rate * elapsed_s;
  const double delta_yaw = yaw_rate * elapsed_s;
  const double cosine = std::cos(delta_yaw);
  const double sine = std::sin(delta_yaw);

  const double raw_x = input.pose.pose.position.x;
  const double raw_y = input.pose.pose.position.y;
  output.pose.pose.position.x = cosine * raw_x - sine * raw_y + dx;
  output.pose.pose.position.y = sine * raw_x + cosine * raw_y;

  tf2::Quaternion raw_orientation;
  tf2::fromMsg(input.pose.pose.orientation, raw_orientation);
  tf2::Quaternion drift_orientation;
  drift_orientation.setRPY(0.0, 0.0, delta_yaw);
  tf2::Quaternion validated_orientation =
    drift_orientation * raw_orientation;
  validated_orientation.normalize();
  output.pose.pose.orientation = tf2::toMsg(validated_orientation);
  return output;
}

}  // namespace

std::string sha256_hex(const std::string_view bytes)
{
  using ContextPointer =
    std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
  ContextPointer context(EVP_MD_CTX_new(), &EVP_MD_CTX_free);
  if (!context ||
    EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1 ||
    EVP_DigestUpdate(context.get(), bytes.data(), bytes.size()) != 1)
  {
    throw std::runtime_error("OpenSSL SHA-256 initialization failed");
  }

  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0U;
  if (EVP_DigestFinal_ex(
      context.get(), digest.data(), &digest_size) != 1 ||
    digest_size != kSha256ByteCount)
  {
    throw std::runtime_error("OpenSSL SHA-256 finalization failed");
  }

  constexpr char hexadecimal[] = "0123456789abcdef";
  std::string output;
  output.reserve(kSha256ByteCount * 2U);
  for (std::size_t index = 0U; index < kSha256ByteCount; ++index) {
    output.push_back(hexadecimal[(digest[index] >> 4U) & 0x0fU]);
    output.push_back(hexadecimal[digest[index] & 0x0fU]);
  }
  return output;
}

CanonicalScheduleResult canonicalize_schedule(
  const std::vector<FaultSpec> & faults)
{
  CanonicalScheduleResult result;
  if (faults.size() > kMaximumFaultCount) {
    result.message = "a Phase 3 schedule contains at most 16 faults";
    return result;
  }

  std::unordered_set<std::string> fault_ids;
  result.schedule.faults.reserve(faults.size());
  for (const auto & specification : faults) {
    if (specification.schema_version != FaultSpec::SCHEMA_VERSION) {
      result.message = "fault specification schema_version is unsupported";
      return result;
    }
    if (!valid_fault_id(specification.fault_id)) {
      result.message =
        "fault_id must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}";
      return result;
    }
    if (!fault_ids.insert(specification.fault_id).second) {
      result.message = "fault_id values must be unique";
      return result;
    }
    if (!valid_target_mode(specification.target, specification.mode)) {
      result.message = "fault target and mode are not a supported Phase 3 pair";
      return result;
    }
    if (!normalized_duration(specification.start_offset) ||
      !normalized_duration(specification.duration))
    {
      result.message = "fault offsets and durations must be normalized";
      return result;
    }

    const std::int64_t start_ns =
      duration_to_nanoseconds(specification.start_offset);
    const std::int64_t duration_ns =
      duration_to_nanoseconds(specification.duration);
    if (start_ns < kMinimumArmingMarginNanoseconds) {
      result.message = "fault start_offset must be at least 0.5 seconds";
      return result;
    }
    if (duration_ns <= 0) {
      result.message = "fault duration must be positive";
      return result;
    }
    if (start_ns > kMaximumScheduleNanoseconds ||
      duration_ns > kMaximumScheduleNanoseconds ||
      start_ns > kMaximumScheduleNanoseconds - duration_ns)
    {
      result.message = "fault start, duration, and end must fit 3600 seconds";
      return result;
    }

    const ParsedParameters parameters = parse_parameters(specification);
    if (!parameters.accepted) {
      result.message = parameters.message;
      return result;
    }

    CanonicalFault canonical;
    canonical.specification = specification;
    canonical.specification.parameters_json = parameters.canonical_json;
    canonical.start_offset_ns = start_ns;
    canonical.duration_ns = duration_ns;
    canonical.x_rate_nm_per_s = parameters.x_rate_nm_per_s;
    canonical.yaw_rate_nrad_per_s = parameters.yaw_rate_nrad_per_s;
    result.schedule.faults.push_back(std::move(canonical));
  }

  std::sort(
    result.schedule.faults.begin(), result.schedule.faults.end(),
    [](const CanonicalFault & left, const CanonicalFault & right) {
      return std::tie(
        left.start_offset_ns, left.specification.target,
        left.specification.fault_id) <
             std::tie(
        right.start_offset_ns, right.specification.target,
        right.specification.fault_id);
    });

  std::array<std::optional<std::int64_t>, 4U> prior_end_by_target{};
  for (const auto & fault : result.schedule.faults) {
    const std::size_t index = stream_index(fault.specification.target);
    if (prior_end_by_target[index].has_value() &&
      fault.start_offset_ns < *prior_end_by_target[index])
    {
      result.message = "same-target fault intervals must not overlap";
      return result;
    }
    prior_end_by_target[index] =
      fault.start_offset_ns + fault.duration_ns;
  }

  std::string output = "{\"schema_version\":1,\"faults\":[";
  for (std::size_t index = 0U;
    index < result.schedule.faults.size(); ++index)
  {
    if (index != 0U) {
      output.push_back(',');
    }
    output += canonical_fault_json(result.schedule.faults[index]);
  }
  output += "]}";
  result.schedule.bytes = std::move(output);

  try {
    result.schedule.sha256 = sha256_hex(result.schedule.bytes);
  } catch (const std::runtime_error & error) {
    result.message = error.what();
    return result;
  }
  if (!result.schedule.faults.empty()) {
    result.schedule.earliest_start_offset_ns =
      result.schedule.faults.front().start_offset_ns;
  }
  result.accepted = true;
  result.message = "canonical schedule validated";
  return result;
}

FaultProtocol::FaultProtocol(
  const std::uint64_t last_allocated_generation_for_test)
: last_allocated_generation_(last_allocated_generation_for_test)
{
}

std::uint8_t FaultProtocol::state_wire_value() const noexcept
{
  return static_cast<std::uint8_t>(state_);
}

std::uint32_t FaultProtocol::committed_fault_count() const noexcept
{
  return static_cast<std::uint32_t>(schedule_.faults.size());
}

FaultEvent FaultProtocol::make_control_event_locked(
  const std::uint8_t event_type,
  const builtin_interfaces::msg::Time & actual_time,
  const FaultControlState state_before,
  const bool accepted,
  const bool replayed,
  const std::string & requested_hash,
  const std::uint64_t requested_generation,
  const std::uint32_t requested_fault_count,
  const unique_identifier_msgs::msg::UUID & requested_goal_uuid,
  const builtin_interfaces::msg::Time & requested_t0,
  const std::string & detail)
{
  if (!increment_counter(event_sequence_)) {
    throw std::overflow_error("fault event sequence exhausted");
  }
  FaultEvent event;
  event.header.stamp = actual_time;
  event.schema_version = FaultEvent::SCHEMA_VERSION;
  event.event_sequence = event_sequence_;
  event.event_type = event_type;
  event.state_before = static_cast<std::uint8_t>(state_before);
  event.state_after = state_wire_value();
  event.accepted = accepted;
  event.replayed = replayed;
  event.requested_schedule_hash = requested_hash;
  event.committed_schedule_hash = schedule_.sha256;
  event.requested_generation = requested_generation;
  event.committed_generation = generation_;
  event.requested_fault_count = requested_fault_count;
  event.committed_fault_count = committed_fault_count();
  event.requested_goal_uuid = requested_goal_uuid;
  event.bound_goal_uuid = bound_goal_uuid_;
  event.requested_t0 = requested_t0;
  event.bound_t0 = accepted_goal_stamp_;
  event.arm_commit_time = arm_commit_stamp_;
  event.arm_margin_ns = arm_margin_ns_;
  event.actual_time = actual_time;
  event.target = FaultSpec::TARGET_UNSPECIFIED;
  event.mode = FaultSpec::MODE_PASS_THROUGH;
  event.detail = detail;
  return event;
}

FaultEvent FaultProtocol::make_data_event_locked(
  const std::uint8_t event_type,
  const builtin_interfaces::msg::Time & actual_time,
  const RuntimeFault & fault,
  const StreamCounters & counters,
  const std::string & detail)
{
  if (!increment_counter(event_sequence_)) {
    throw std::overflow_error("fault event sequence exhausted");
  }
  const std::int64_t t0_ns = time_to_nanoseconds(accepted_goal_stamp_);
  FaultEvent event;
  event.header.stamp = actual_time;
  event.schema_version = FaultEvent::SCHEMA_VERSION;
  event.event_sequence = event_sequence_;
  event.event_type = event_type;
  event.state_before = state_wire_value();
  event.state_after = state_wire_value();
  event.accepted = true;
  event.replayed = false;
  event.committed_schedule_hash = schedule_.sha256;
  event.committed_generation = generation_;
  event.committed_fault_count = committed_fault_count();
  event.bound_goal_uuid = bound_goal_uuid_;
  event.bound_t0 = accepted_goal_stamp_;
  event.arm_commit_time = arm_commit_stamp_;
  event.arm_margin_ns = arm_margin_ns_;
  event.fault_id = fault.canonical.specification.fault_id;
  event.target = fault.canonical.specification.target;
  event.mode = fault.canonical.specification.mode;
  event.configured_activation_time = nanoseconds_to_time(
    t0_ns + fault.canonical.start_offset_ns);
  event.configured_deactivation_time = nanoseconds_to_time(
    t0_ns + fault.canonical.start_offset_ns +
    fault.canonical.duration_ns);
  event.actual_time = actual_time;
  event.seed = fault.canonical.specification.seed;
  event.input_sequence = counters.input_sequence;
  event.raw_input_count = counters.raw_input_count;
  event.validated_output_count = counters.validated_output_count;
  event.affected_message_count = fault.affected_message_count;
  event.detail = detail;
  return event;
}

void FaultProtocol::reset_runtime_locked()
{
  runtime_faults_.clear();
  runtime_faults_.reserve(schedule_.faults.size());
  for (const auto & canonical : schedule_.faults) {
    RuntimeFault runtime;
    runtime.canonical = canonical;
    runtime_faults_.push_back(std::move(runtime));
  }
  stream_counters_ = {};
}

PreloadOutcome FaultProtocol::preload(
  const robotest_interfaces::srv::PreloadFaultSchedule::Request & request,
  const builtin_interfaces::msg::Time & operation_stamp)
{
  const std::vector<FaultSpec> requested_faults(
    request.faults.begin(), request.faults.end());
  const CanonicalScheduleResult canonical =
    canonicalize_schedule(requested_faults);
  std::lock_guard<std::mutex> lock(mutex_);
  const FaultControlState state_before = state_;

  auto reject = [&](const std::string & message) {
      PreloadOutcome outcome;
      outcome.state = state_;
      outcome.schedule_hash = schedule_.sha256;
      outcome.loaded_count = committed_fault_count();
      outcome.generation = generation_;
      outcome.message = message;
      outcome.event = make_control_event_locked(
        FaultEvent::EVENT_SCHEDULE_REJECTED, operation_stamp, state_before,
        false, false, request.schedule_hash, 0U,
        static_cast<std::uint32_t>(request.faults.size()),
        unique_identifier_msgs::msg::UUID(),
        builtin_interfaces::msg::Time(), message);
      return outcome;
    };

  if (!normalized_time(operation_stamp)) {
    return reject("preload operation stamp is not normalized");
  }
  if (!canonical.accepted) {
    return reject(canonical.message);
  }
  if (!lowercase_sha256(request.schedule_hash) ||
    request.schedule_hash != canonical.schedule.sha256)
  {
    return reject("claimed schedule hash does not match canonical bytes");
  }

  const bool same_schedule =
    state_ != FaultControlState::kReset &&
    schedule_.bytes == canonical.schedule.bytes &&
    schedule_.sha256 == canonical.schedule.sha256;
  if (state_ == FaultControlState::kArmed && !same_schedule) {
    return reject("an armed generation rejects a different preload");
  }

  bool replayed = false;
  if (same_schedule) {
    replayed = true;
  } else {
    if (last_allocated_generation_ ==
      std::numeric_limits<std::uint64_t>::max())
    {
      return reject("schedule generation allocator is exhausted");
    }
    ++last_allocated_generation_;
    generation_ = last_allocated_generation_;
    schedule_ = canonical.schedule;
    state_ = FaultControlState::kPrepared;
    bound_goal_uuid_ = unique_identifier_msgs::msg::UUID();
    accepted_goal_stamp_ = builtin_interfaces::msg::Time();
    arm_commit_stamp_ = builtin_interfaces::msg::Time();
    arm_margin_ns_ = 0;
    reset_runtime_locked();
  }

  PreloadOutcome outcome;
  outcome.accepted = true;
  outcome.replayed = replayed;
  outcome.state = state_;
  outcome.schedule_hash = schedule_.sha256;
  outcome.loaded_count = committed_fault_count();
  outcome.generation = generation_;
  outcome.message =
    replayed ? "exact canonical preload replay accepted" :
    "canonical schedule prepared atomically";
  outcome.event = make_control_event_locked(
    FaultEvent::EVENT_SCHEDULE_LOADED, operation_stamp, state_before, true,
    replayed, request.schedule_hash, 0U,
    static_cast<std::uint32_t>(request.faults.size()),
    unique_identifier_msgs::msg::UUID(), builtin_interfaces::msg::Time(),
    outcome.message);
  return outcome;
}

ArmOutcome FaultProtocol::arm(
  const robotest_interfaces::srv::ArmFaultSchedule::Request & request,
  const builtin_interfaces::msg::Time & operation_stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const FaultControlState state_before = state_;

  auto populate_committed = [&](ArmOutcome & outcome) {
      outcome.state = state_;
      outcome.schedule_hash = schedule_.sha256;
      outcome.generation = generation_;
      outcome.goal_uuid = bound_goal_uuid_;
      outcome.accepted_goal_stamp = accepted_goal_stamp_;
      outcome.arm_commit_stamp = arm_commit_stamp_;
      outcome.arm_margin_ns = arm_margin_ns_;
    };
  auto reject = [&](const std::string & message) {
      ArmOutcome outcome;
      populate_committed(outcome);
      outcome.message = message;
      outcome.event = make_control_event_locked(
        FaultEvent::EVENT_ARM_REJECTED, operation_stamp, state_before, false,
        false, request.schedule_hash, request.generation,
        committed_fault_count(),
        request.goal_uuid, request.accepted_goal_stamp, message);
      return outcome;
    };

  if (!normalized_time(operation_stamp)) {
    return reject("arm operation stamp is not normalized");
  }

  const bool exact_replay =
    state_ == FaultControlState::kArmed &&
    request.schedule_hash == schedule_.sha256 &&
    request.generation == generation_ &&
    same_uuid(request.goal_uuid, bound_goal_uuid_) &&
    same_time(request.accepted_goal_stamp, accepted_goal_stamp_);
  if (exact_replay) {
    ArmOutcome outcome;
    outcome.accepted = true;
    outcome.replayed = true;
    populate_committed(outcome);
    outcome.message = "exact arm replay accepted";
    outcome.event = make_control_event_locked(
      FaultEvent::EVENT_ARMED, operation_stamp, state_before, true, true,
      request.schedule_hash, request.generation, committed_fault_count(),
      request.goal_uuid, request.accepted_goal_stamp, outcome.message);
    return outcome;
  }

  if (state_ != FaultControlState::kPrepared) {
    return reject("arm requires one matching PREPARED generation");
  }
  if (!lowercase_sha256(request.schedule_hash) ||
    request.schedule_hash != schedule_.sha256 ||
    request.generation != generation_)
  {
    return reject("arm hash or generation does not match PREPARED state");
  }
  if (zero_uuid(request.goal_uuid)) {
    return reject("arm goal UUID must not be all zero");
  }
  if (!positive_time(request.accepted_goal_stamp)) {
    return reject("authoritative accepted-goal T0 must be positive");
  }

  const std::int64_t t0_ns =
    time_to_nanoseconds(request.accepted_goal_stamp);
  const std::int64_t commit_ns = time_to_nanoseconds(operation_stamp);
  if (commit_ns < t0_ns) {
    return reject("arm commit stamp precedes authoritative T0");
  }

  constexpr std::int64_t maximum_ros_time_ns =
    static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) *
    kNanosecondsPerSecond + (kNanosecondsPerSecond - 1);
  for (const auto & fault : schedule_.faults) {
    const std::int64_t end_offset_ns =
      fault.start_offset_ns + fault.duration_ns;
    if (t0_ns > maximum_ros_time_ns - end_offset_ns) {
      return reject("T0 plus configured interval exceeds ROS Time range");
    }
  }

  std::int64_t margin_ns = 0;
  if (schedule_.earliest_start_offset_ns.has_value()) {
    if (t0_ns >
      std::numeric_limits<std::int64_t>::max() -
      *schedule_.earliest_start_offset_ns)
    {
      return reject("T0 plus earliest activation overflows ROS time");
    }
    margin_ns =
      t0_ns + *schedule_.earliest_start_offset_ns - commit_ns;
    if (margin_ns < kMinimumArmingMarginNanoseconds) {
      return reject("arm commit has less than 0.5 seconds of margin");
    }
  }

  state_ = FaultControlState::kArmed;
  bound_goal_uuid_ = request.goal_uuid;
  accepted_goal_stamp_ = request.accepted_goal_stamp;
  arm_commit_stamp_ = operation_stamp;
  arm_margin_ns_ = margin_ns;
  reset_runtime_locked();

  ArmOutcome outcome;
  outcome.accepted = true;
  outcome.state = state_;
  outcome.schedule_hash = schedule_.sha256;
  outcome.generation = generation_;
  outcome.goal_uuid = bound_goal_uuid_;
  outcome.accepted_goal_stamp = accepted_goal_stamp_;
  outcome.arm_commit_stamp = arm_commit_stamp_;
  outcome.arm_margin_ns = arm_margin_ns_;
  outcome.message = "prepared generation bound atomically to goal UUID and T0";
  outcome.event = make_control_event_locked(
    FaultEvent::EVENT_ARMED, operation_stamp, state_before, true, false,
    request.schedule_hash, request.generation, committed_fault_count(),
    request.goal_uuid, request.accepted_goal_stamp, outcome.message);
  return outcome;
}

ResetOutcome FaultProtocol::reset(
  const builtin_interfaces::msg::Time & operation_stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const FaultControlState state_before = state_;
  const std::string previous_hash = schedule_.sha256;
  const std::uint64_t previous_generation = generation_;
  const std::uint32_t previous_count = committed_fault_count();
  const auto previous_goal_uuid = bound_goal_uuid_;
  const auto previous_t0 = accepted_goal_stamp_;

  state_ = FaultControlState::kReset;
  generation_ = 0U;
  schedule_ = {};
  runtime_faults_.clear();
  bound_goal_uuid_ = unique_identifier_msgs::msg::UUID();
  accepted_goal_stamp_ = builtin_interfaces::msg::Time();
  arm_commit_stamp_ = builtin_interfaces::msg::Time();
  arm_margin_ns_ = 0;
  stream_counters_ = {};

  ResetOutcome outcome;
  outcome.state_before = state_before;
  outcome.message =
    state_before == FaultControlState::kReset ?
    "fault state already RESET; idempotent reset accepted" :
    "fault state cleared; deterministic pass-through active";
  outcome.event = make_control_event_locked(
    FaultEvent::EVENT_RESET, operation_stamp, state_before, true,
    state_before == FaultControlState::kReset, previous_hash,
    previous_generation, previous_count, previous_goal_uuid, previous_t0,
    outcome.message);
  return outcome;
}

std::vector<FaultEvent>
FaultProtocol::begin_target_input_locked(
  const std::uint8_t target,
  const builtin_interfaces::msg::Time & stamp,
  StreamCounters & counters,
  RuntimeFault *& active_fault,
  std::string & rejection)
{
  std::vector<FaultEvent> events;
  active_fault = nullptr;
  const std::int64_t stamp_ns = time_to_nanoseconds(stamp);
  if (counters.last_input_stamp_ns.has_value() &&
    stamp_ns < *counters.last_input_stamp_ns)
  {
    rejection = "raw input stamp regressed within the armed generation";
    return events;
  }
  if (!increment_counter(counters.input_sequence) ||
    !increment_counter(counters.raw_input_count))
  {
    rejection = "target stream counter exhausted";
    return events;
  }
  counters.last_input_stamp_ns = stamp_ns;

  const std::int64_t t0_ns = time_to_nanoseconds(accepted_goal_stamp_);
  for (auto & fault : runtime_faults_) {
    if (fault.canonical.specification.target != target) {
      continue;
    }
    const std::int64_t activation_ns =
      t0_ns + fault.canonical.start_offset_ns;
    const std::int64_t deactivation_ns =
      activation_ns + fault.canonical.duration_ns;

    if (!fault.deactivated && stamp_ns >= deactivation_ns) {
      fault.deactivated = true;
      fault.restoration_pending =
        fault.affected_message_count > 0U &&
        fault.canonical.specification.mode != FaultSpec::MODE_PASS_THROUGH;
      events.push_back(make_data_event_locked(
          FaultEvent::EVENT_DEACTIVATED, stamp, fault, counters,
          "first target input at or after the half-open interval end"));
    }

    if (!fault.deactivated && stamp_ns >= activation_ns &&
      stamp_ns < deactivation_ns)
    {
      if (active_fault != nullptr) {
        rejection = "multiple same-target faults are active";
        return events;
      }
      active_fault = &fault;
      if (!fault.activated) {
        fault.activated = true;
        events.push_back(make_data_event_locked(
            FaultEvent::EVENT_ACTIVATED, stamp, fault, counters,
            "first target input inside the configured interval"));
      }
    }
  }
  return events;
}

void FaultProtocol::append_restoration_events_locked(
  const std::uint8_t target,
  const builtin_interfaces::msg::Time & stamp,
  const StreamCounters & counters,
  std::vector<FaultEvent> & events)
{
  for (auto & fault : runtime_faults_) {
    if (fault.canonical.specification.target == target &&
      fault.restoration_pending && !fault.first_restored)
    {
      fault.first_restored = true;
      fault.restoration_pending = false;
      events.push_back(make_data_event_locked(
          FaultEvent::EVENT_FIRST_RESTORED, stamp, fault, counters,
          "first corresponding unmodified validated output"));
    }
  }
}

ScanFaultOutcome FaultProtocol::process_scan(
  const sensor_msgs::msg::LaserScan & input,
  const std::string & expected_frame)
{
  const auto validated = validate_and_copy_scan(input, expected_frame);
  ScanFaultOutcome outcome;
  if (!validated.accepted) {
    outcome.reason = validated.reason;
    return outcome;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  outcome.accepted = true;
  if (state_ != FaultControlState::kArmed) {
    outcome.publish = true;
    outcome.message = validated.message;
    return outcome;
  }

  auto & counters = stream_counters_[stream_index(FaultSpec::TARGET_SCAN)];
  RuntimeFault * active_fault = nullptr;
  outcome.events = begin_target_input_locked(
    FaultSpec::TARGET_SCAN, input.header.stamp, counters, active_fault,
    outcome.reason);
  if (!outcome.reason.empty()) {
    outcome.accepted = false;
    return outcome;
  }

  bool unmodified_output = true;
  if (active_fault != nullptr &&
    active_fault->canonical.specification.mode ==
    FaultSpec::MODE_LIDAR_DROPOUT)
  {
    unmodified_output = false;
    if (!increment_counter(active_fault->affected_message_count)) {
      outcome.accepted = false;
      outcome.reason = "fault affected-message counter exhausted";
      return outcome;
    }
    if (active_fault->affected_message_count == 1U) {
      outcome.events.push_back(make_data_event_locked(
          FaultEvent::EVENT_FIRST_AFFECTED, input.header.stamp, *active_fault,
          counters, "first validated scan suppressed by LiDAR dropout"));
    }
    outcome.publish = false;
    return outcome;
  }

  if (!increment_counter(counters.validated_output_count)) {
    outcome.accepted = false;
    outcome.reason = "validated scan counter exhausted";
    return outcome;
  }
  outcome.publish = true;
  outcome.message = validated.message;
  if (unmodified_output) {
    append_restoration_events_locked(
      FaultSpec::TARGET_SCAN, input.header.stamp, counters, outcome.events);
  }
  return outcome;
}

OdomFaultOutcome FaultProtocol::process_odom(
  const nav_msgs::msg::Odometry & input,
  const std::string & expected_frame,
  const std::string & expected_child_frame)
{
  const auto validated =
    validate_and_copy_odom(input, expected_frame, expected_child_frame);
  OdomFaultOutcome outcome;
  if (!validated.accepted) {
    outcome.reason = validated.reason;
    return outcome;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  outcome.accepted = true;
  if (state_ != FaultControlState::kArmed) {
    outcome.publish = true;
    outcome.message = validated.message;
    return outcome;
  }

  auto & counters = stream_counters_[stream_index(FaultSpec::TARGET_ODOM)];
  RuntimeFault * active_fault = nullptr;
  outcome.events = begin_target_input_locked(
    FaultSpec::TARGET_ODOM, input.header.stamp, counters, active_fault,
    outcome.reason);
  if (!outcome.reason.empty()) {
    outcome.accepted = false;
    return outcome;
  }

  bool unmodified_output = true;
  outcome.message = validated.message;
  if (active_fault != nullptr &&
    active_fault->canonical.specification.mode == FaultSpec::MODE_ODOM_DRIFT)
  {
    unmodified_output = false;
    if (active_fault->affected_message_count ==
      std::numeric_limits<std::uint64_t>::max())
    {
      outcome.accepted = false;
      outcome.reason = "fault affected-message counter exhausted";
      return outcome;
    }
    if (counters.validated_output_count ==
      std::numeric_limits<std::uint64_t>::max())
    {
      outcome.accepted = false;
      outcome.reason = "validated odometry counter exhausted";
      return outcome;
    }
    ++active_fault->affected_message_count;
    ++counters.validated_output_count;
    const std::int64_t activation_ns =
      time_to_nanoseconds(accepted_goal_stamp_) +
      active_fault->canonical.start_offset_ns;
    const std::int64_t elapsed_ns =
      time_to_nanoseconds(input.header.stamp) - activation_ns;
    outcome.message =
      apply_odom_drift(validated.message, active_fault->canonical, elapsed_ns);
    if (active_fault->affected_message_count == 1U) {
      outcome.events.push_back(make_data_event_locked(
          FaultEvent::EVENT_FIRST_AFFECTED, input.header.stamp, *active_fault,
          counters, "first validated odometry transformed by SE(2) drift"));
    }
  }

  if (unmodified_output &&
    !increment_counter(counters.validated_output_count))
  {
    outcome.accepted = false;
    outcome.reason = "validated odometry counter exhausted";
    return outcome;
  }
  outcome.publish = true;
  if (unmodified_output) {
    append_restoration_events_locked(
      FaultSpec::TARGET_ODOM, input.header.stamp, counters, outcome.events);
  }
  return outcome;
}

ImuFaultOutcome FaultProtocol::process_imu(
  const sensor_msgs::msg::Imu & input,
  const std::string & expected_frame)
{
  const auto validated = validate_and_copy_imu(input, expected_frame);
  ImuFaultOutcome outcome;
  if (!validated.accepted) {
    outcome.reason = validated.reason;
    return outcome;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  outcome.accepted = true;
  if (state_ != FaultControlState::kArmed) {
    outcome.publish = true;
    outcome.message = validated.message;
    return outcome;
  }

  auto & counters = stream_counters_[stream_index(FaultSpec::TARGET_IMU)];
  RuntimeFault * active_fault = nullptr;
  outcome.events = begin_target_input_locked(
    FaultSpec::TARGET_IMU, input.header.stamp, counters, active_fault,
    outcome.reason);
  if (!outcome.reason.empty()) {
    outcome.accepted = false;
    return outcome;
  }
  static_cast<void>(active_fault);

  if (!increment_counter(counters.validated_output_count)) {
    outcome.accepted = false;
    outcome.reason = "validated IMU counter exhausted";
    return outcome;
  }
  outcome.publish = true;
  outcome.message = validated.message;
  append_restoration_events_locked(
    FaultSpec::TARGET_IMU, input.header.stamp, counters, outcome.events);
  return outcome;
}

FaultProtocolSnapshot FaultProtocol::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  FaultProtocolSnapshot result;
  result.state = state_;
  result.schedule_hash = schedule_.sha256;
  result.canonical_bytes = schedule_.bytes;
  result.fault_count = committed_fault_count();
  result.generation = generation_;
  result.goal_uuid = bound_goal_uuid_;
  result.accepted_goal_stamp = accepted_goal_stamp_;
  result.arm_commit_stamp = arm_commit_stamp_;
  result.arm_margin_ns = arm_margin_ns_;
  return result;
}

}  // namespace robotest_faults

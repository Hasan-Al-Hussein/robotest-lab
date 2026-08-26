# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

# Version 2 values are an append-only wire contract.
uint32 SCHEMA_VERSION=2

uint8 STATE_RESET=0
uint8 STATE_PREPARED=1
uint8 STATE_ARMED=2

# Values 1-6 preserve the Phase 1 numeric assignments.
uint8 EVENT_SCHEDULE_LOADED=1
uint8 EVENT_SCHEDULE_REJECTED=2
uint8 EVENT_RESET=3
uint8 EVENT_ACTIVATED=4
uint8 EVENT_FIRST_AFFECTED=5
uint8 EVENT_DEACTIVATED=6
uint8 EVENT_ARMED=7
uint8 EVENT_ARM_REJECTED=8
uint8 EVENT_FIRST_RESTORED=9

# header.stamp is the simulation stamp at which the event was emitted.
std_msgs/Header header
uint32 schema_version
uint64 event_sequence
uint8 event_type
uint8 state_before
uint8 state_after
bool accepted
bool replayed

# Request and committed values make rejection and exact replay auditable.
string<=64 requested_schedule_hash
string<=64 committed_schedule_hash
uint64 requested_generation
uint64 committed_generation
uint32 requested_fault_count
uint32 committed_fault_count
unique_identifier_msgs/UUID requested_goal_uuid
unique_identifier_msgs/UUID bound_goal_uuid
builtin_interfaces/Time requested_t0
builtin_interfaces/Time bound_t0
builtin_interfaces/Time arm_commit_time
int64 arm_margin_ns

# Data-path identity and configured/actual timing.
string<=64 fault_id
uint8 target
uint8 mode
builtin_interfaces/Time configured_activation_time
builtin_interfaces/Time configured_deactivation_time
builtin_interfaces/Time actual_time
uint64 seed

# input_sequence/raw/validated are cumulative for the target stream.
# affected_message_count is cumulative for this fault.
uint64 input_sequence
uint64 raw_input_count
uint64 validated_output_count
uint64 affected_message_count

# Operator context only; acceptance never parses this field.
string<=256 detail

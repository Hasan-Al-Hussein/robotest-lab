# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

uint32 SCHEMA_VERSION=1

uint8 EVENT_SCHEDULE_LOADED=1
uint8 EVENT_SCHEDULE_REJECTED=2
uint8 EVENT_RESET=3
uint8 EVENT_ACTIVATED=4
uint8 EVENT_FIRST_AFFECTED=5
uint8 EVENT_DEACTIVATED=6

# header.stamp is the simulation time at which the event was emitted.
std_msgs/Header header
uint32 schema_version
uint8 event_type

string<=64 fault_id
uint8 target
uint8 mode

# configured_time is the intended transition time. actual_time records when the
# input-driven state machine observed that transition.
builtin_interfaces/Time configured_time
builtin_interfaces/Time actual_time
uint64 seed
string<=64 schedule_hash
uint64 input_sequence
uint64 affected_message_count
string<=256 detail

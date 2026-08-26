# Copyright 2026 Hasan Ahmed
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Normative limits copied from the Phase 3 metrics contract."""

from __future__ import annotations

CONTRACT_REVISION = 2
NANOSECONDS_PER_SECOND = 1_000_000_000
MICROMETRES_PER_METRE = 1_000_000

ALIGNMENT_GAP_NS = 250_000_000
CONTACT_RELEASE_GAP_NS = 250_000_000
RECOVERY_WINDOW_NS = 1_000_000_000
RECOVERY_SCAN_GAP_NS = 400_000_000
RECOVERY_LIFECYCLE_MARGIN_NS = 200_000_000
RECOVERY_PROGRESS_METRES = 0.05
RECOVERY_PROGRESS_YAW_RAD = 0.10
LOCALIZATION_MINIMUM_COVERAGE = 0.95
PLAN_GOAL_TOLERANCE_M = 0.05
PATH_LENGTH_EPSILON_M = 0.000001

STRING_FIELD_MAX_BYTES = 4_096
GROUND_TRUTH_CAPACITY = 8_192
TF_EDGE_CAPACITY = 8_192
ODOMETRY_CAPACITY = 8_192
SCAN_CAPACITY = 2_048
COMMAND_CAPACITY = 4_096
PLAN_MESSAGE_CAPACITY = 1_024
PLAN_POSE_CAPACITY = 65_536
CONTACT_MESSAGE_CAPACITY = 8_192
CONTACT_RECORD_CAPACITY = 32_768
WORLD_STATS_CAPACITY = 4_096
STATE_EVENT_CAPACITY = 1_024
FAULT_EVENT_CAPACITY = 512

PER_RUN_JSON_MAX_BYTES = 33_554_432
PER_RUN_CSV_MAX_BYTES = 1_048_576
LOG_MAX_BYTES = 8_388_608
PNG_MAX_BYTES = 4_194_304
PNG_MAX_COUNT = 8
PER_RUN_DIRECTORY_MAX_BYTES = 268_435_456
AGGREGATE_DIRECTORY_MAX_BYTES = 67_108_864

PLAN_HASH_VERSION = b'robotest-plan-geometry-v1\0'

NAV2_LIFECYCLE_NODES = (
    'map_server',
    'amcl',
    'planner_server',
    'controller_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
    'collision_monitor',
)

BUFFER_CAPACITIES = {
    'ground_truth': GROUND_TRUTH_CAPACITY,
    'tf_map_odom': TF_EDGE_CAPACITY,
    'tf_odom_base_footprint': TF_EDGE_CAPACITY,
    'raw_odom': ODOMETRY_CAPACITY,
    'odom': ODOMETRY_CAPACITY,
    'raw_scan': SCAN_CAPACITY,
    'scan': SCAN_CAPACITY,
    'cmd_vel': COMMAND_CAPACITY,
    'contacts': CONTACT_MESSAGE_CAPACITY,
    'world_stats': WORLD_STATS_CAPACITY,
    'state_events': STATE_EVENT_CAPACITY,
    'fault_events': FAULT_EVENT_CAPACITY,
}

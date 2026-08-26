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

"""Frozen Phase 3 scenario-controller constants."""

from __future__ import annotations

from enum import IntEnum


class ExitCode(IntEnum):
    """Stable process exit codes shared by both installed executables."""

    COMPLETE = 0
    VALIDATION_ERROR = 2
    WALL_TIMEOUT = 20
    PROTOCOL_ERROR = 21
    INFRASTRUCTURE_ERROR = 22
    ARTIFACT_ERROR = 23
    SCENARIO_FAILED = 24


SCHEMA_VERSION = 1
ACTION_NAME = 'follow_waypoints'
ACTION_STATUS_TOPIC = f'{ACTION_NAME}/_action/status'
ACTION_FEEDBACK_TOPIC = f'{ACTION_NAME}/_action/feedback'
ACTION_CANCEL_SERVICE = f'{ACTION_NAME}/_action/cancel_goal'
PLAN_TOPIC = 'navigation/plan'
GROUND_TRUTH_TOPIC = 'validation/ground_truth'
ENTITY_POSE_TOPIC = 'validation/scenario_entity_poses'
ENTITY_POSE_HEARTBEAT_MODEL = 'ground_plane'
CONTACT_TOPIC = 'validation/contacts'
FINAL_COMMAND_TOPIC = 'cmd_vel'
SPAWN_SERVICE = 'scenario/spawn_entity'
SET_POSE_SERVICE = 'scenario/set_entity_pose'
DELETE_SERVICE = 'scenario/delete_entity'
CLOCK_TOPIC = '/clock'

STATUS_CAPACITY = 1_024
FEEDBACK_CAPACITY = 4_096
PLAN_CAPACITY = 1_024
PLAN_POSE_CAPACITY = 65_536
GROUND_TRUTH_CAPACITY = 8_192
GROUND_TRUTH_ALIGNMENT_NS = 250_000_000
ACTOR_STATE_CAPACITY = 1_024
CONTACT_SUMMARY_CAPACITY = 8_192
CONTACT_RECORD_CAPACITY = 32_768
COMMAND_CAPACITY = 4_096

PATH_LENGTH_EPSILON_M = 0.000001
PLAN_ENDPOINT_TOLERANCE_M = 0.05
S2_TRIGGER_OFFSET_NS = 2_000_000_000
S2_OBSERVATION_DEADLINE_OFFSET_NS = 2_250_000_000
S2_CLEARANCE_M = 0.70
S2_TARGET = (-1.0, -3.5, 0.4, 0.0)
S2_EXCLUSION_RECT = (-1.55, -0.45, -4.05, -2.95)
S3_TARGET_COUNT = 121
S3_PERIOD_NS = 100_000_000
S3_PARKED = (-0.8, 1.5, 0.4, 0.0)
S3_FINAL = (0.8, 1.5, 0.4, 0.0)
ACTOR_INITIAL_POSITION_TOLERANCE_M = 0.01
ACTOR_POSITION_TOLERANCE_M = 0.02
ACTOR_YAW_TOLERANCE_RAD = 0.01
ACTOR_OBSERVATION_LATENCY_NS = 200_000_000
ACTOR_CLEANUP_QUIET_NS = 250_000_000
DDS_DRAIN_GRACE_S = 0.05

CONTROL_FORWARD_MPS = 0.05
CONTROL_REVERSE_MPS = -0.05
CONTROL_COMMAND_PERIOD_NS = 50_000_000
CONTROL_CONTACT_DEADLINE_NS = 12_000_000_000
CONTROL_STOP_DEADLINE_NS = 100_000_000
CONTROL_HOLD_NS = 250_000_000
CONTROL_REVERSE_NS = 1_000_000_000
CONTACT_RELEASE_GAP_NS = 250_000_000
CONTROL_WALL_POSE = (0.70, -3.50, 0.40, 0.0)
CONTROL_ROBOT_START = (0.0, -3.5, 0.0)
CONTROL_WALL_COLLISION = 'phase3_contact_control_wall::link::collision'
CONTROL_SOURCE_GRAPH_MISSING_CONFIRM_NS = 100_000_000

DEFAULT_SERVICE_TIMEOUT_S = 2.0
DEFAULT_CONTROL_WALL_TIMEOUT_S = 30.0
MAX_STRING_BYTES = 4_096
MAX_JSON_BYTES = 33_554_432

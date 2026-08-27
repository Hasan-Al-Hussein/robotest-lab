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

"""Frozen controller-provenance projections."""

from robotest_scenarios.provenance import (
    contact_control_arm_protocol,
    contact_control_arm_protocol_sha256,
    contact_control_configuration,
)


def test_contact_control_arm_protocol_value_and_hash_are_frozen() -> None:
    protocol = {
        'ack_max_bytes': 4096,
        'ack_producer': 'robotest_scenarios/contact_control_driver',
        'action': 'start_positive_control_motion',
        'command_delivery_probe': {
            'max_sim_lag_ns': 100_000_000,
            'policy': (
                'exact_two_matches_then_one_untracked_zero_with_partial_ordered_'
                'collector_progress_before_arm'
            ),
            'progress_max_bytes': 4096,
            'progress_producer': 'robotest_metrics/metrics_collector',
            'progress_schema_version': 1,
            'public_topic': '/robotest/cmd_vel',
            'required_subscription_count': 2,
            'safe_zero': {
                'angular_z_rad_s': 0.0,
                'linear_x_m_s': 0.0,
                'linear_y_m_s': 0.0,
            },
        },
        'fresh_clock_policy': 'strictly_newer_positive_stamp_after_valid_arm',
        'request_max_bytes': 4096,
        'request_producer': 'robotest_phase3/benchmark_runner',
        'schema_version': 2,
        'wait_deadline_policy': 'complete_fixture_steady_wall_deadline_without_reset',
    }
    assert contact_control_arm_protocol() == protocol
    assert (
        contact_control_arm_protocol_sha256()
        == 'e2237df774efa356b5a7c0d72dca7f9a13450311d37b26d35af1ff74b1ba8ac3'
    )
    assert contact_control_configuration()['arm_protocol'] == protocol

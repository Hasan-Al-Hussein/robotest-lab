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
        'fresh_clock_policy': 'strictly_newer_positive_stamp_after_valid_arm',
        'request_max_bytes': 4096,
        'request_producer': 'robotest_phase3/benchmark_runner',
        'schema_version': 1,
        'wait_deadline_policy': 'complete_fixture_steady_wall_deadline_without_reset',
    }
    assert contact_control_arm_protocol() == protocol
    assert (
        contact_control_arm_protocol_sha256()
        == '4a457026aa68f696f6926030a4721c705edfae5b74158d8494e07cf58922315f'
    )
    assert contact_control_configuration()['arm_protocol'] == protocol

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

"""Deterministic plan and actor geometry tests."""

import pytest
from robotest_scenarios.constants import S2_EXCLUSION_RECT
from robotest_scenarios.errors import ProtocolError
from robotest_scenarios.geometry import (
    plan_avoids_rectangle,
    plan_geometry_sha256,
    scenario3_target,
    segment_intersects_closed_rectangle,
    validate_plan,
)


def test_plan_hash_deduplicates_and_rounds_half_away_from_zero() -> None:
    original = plan_geometry_sha256('map', ((0.0, 0.0), (0.0000005, -0.0000005)))
    equivalent = plan_geometry_sha256('map', ((0.0, 0.0), (0.000001, -0.000001)))
    duplicate = plan_geometry_sha256('map', ((0.0, 0.0), (0.0, 0.0), (0.000001, -0.000001)))
    assert original == equivalent == duplicate


def test_plan_validation_is_geometry_only() -> None:
    evidence = validate_plan('map', ((0.0, -3.5), (-2.0, -3.5)), (-2.0, -3.5))
    assert evidence.length_m == pytest.approx(2.0)
    assert evidence.endpoint_error_m == 0.0
    with pytest.raises(ProtocolError, match='frame'):
        validate_plan('world', ((0.0, -3.5), (-2.0, -3.5)), (-2.0, -3.5))


def test_closed_obstacle_boundary_is_not_safe() -> None:
    assert segment_intersects_closed_rectangle((-2.0, -3.5), (-1.55, -3.5), S2_EXCLUSION_RECT)
    assert not plan_avoids_rectangle(((-2.0, -3.5), (-1.55, -3.5)), S2_EXCLUSION_RECT)
    assert plan_avoids_rectangle(((-2.0, -4.1), (0.0, -4.1)), S2_EXCLUSION_RECT)


@pytest.mark.parametrize(
    ('index', 'x_value', 'offset_ns'),
    [(0, -0.8, 0), (40, 0.0, 4_000_000_000), (80, 0.0, 8_000_000_000), (120, 0.8, 12_000_000_000)],
)
def test_scenario3_frozen_targets(index: int, x_value: float, offset_ns: int) -> None:
    target = scenario3_target(index, 10_000_000_000)
    assert target['x'] == x_value
    assert target['stamp_ns'] == 10_000_000_000 + offset_ns
    assert (target['y'], target['z'], target['yaw']) == (1.5, 0.4, 0.0)

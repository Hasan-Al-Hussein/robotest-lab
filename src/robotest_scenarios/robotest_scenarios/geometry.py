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

"""Pure deterministic plan and actor geometry operations."""

from __future__ import annotations

import hashlib
import math
import struct
from collections.abc import Iterable
from dataclasses import dataclass
from itertools import pairwise

from robotest_scenarios.artifacts import canonical_json_bytes
from robotest_scenarios.constants import PATH_LENGTH_EPSILON_M, PLAN_ENDPOINT_TOLERANCE_M
from robotest_scenarios.errors import ProtocolError

_PLAN_TAG = b'robotest-plan-geometry-v1\0'
_TRAJECTORY_TAG = b'robotest-scenario-trajectory-v1\0'


@dataclass(frozen=True, slots=True)
class PlanEvidence:
    """Validated path geometry and its contract hash."""

    frame_id: str
    points: tuple[tuple[float, float], ...]
    length_m: float
    endpoint_error_m: float
    geometry_sha256: str

    def as_dict(self) -> dict[str, object]:
        """Return bounded evidence fields without duplicating pose messages."""
        return {
            'endpoint_error_m': self.endpoint_error_m,
            'frame_id': self.frame_id,
            'geometry_sha256': self.geometry_sha256,
            'length_m': self.length_m,
            'point_count': len(self.points),
            'points': [{'x': point[0], 'y': point[1]} for point in self.points],
        }


def stamp_to_ns(stamp: object, *, positive: bool = False) -> int:
    """Validate and convert a builtin_interfaces/Time-like object."""
    try:
        seconds = stamp.sec
        nanoseconds = stamp.nanosec
    except AttributeError as exc:
        raise ProtocolError('ROS timestamp fields are unavailable') from exc
    if (
        isinstance(seconds, bool)
        or not isinstance(seconds, int)
        or isinstance(nanoseconds, bool)
        or not isinstance(nanoseconds, int)
        or seconds < 0
        or not 0 <= nanoseconds < 1_000_000_000
    ):
        raise ProtocolError('ROS timestamp fields are invalid')
    value = seconds * 1_000_000_000 + nanoseconds
    if positive and value <= 0:
        raise ProtocolError('ROS timestamp must be positive')
    return value


def normalize_yaw(value: float) -> float:
    """Normalize an angle into [-pi, pi)."""
    if not math.isfinite(value):
        raise ProtocolError('yaw must be finite')
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def shortest_yaw_error(actual: float, target: float) -> float:
    """Return absolute shortest-arc yaw error."""
    return abs(normalize_yaw(actual - target))


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert one finite quaternion to yaw after normalization."""
    values = (x, y, z, w)
    if not all(math.isfinite(item) for item in values):
        raise ProtocolError('quaternion contains a non-finite value')
    norm = math.sqrt(sum(item * item for item in values))
    if norm <= 1e-12:
        raise ProtocolError('quaternion norm is zero')
    x_n, y_n, z_n, w_n = (item / norm for item in values)
    return math.atan2(
        2.0 * (w_n * z_n + x_n * y_n),
        1.0 - 2.0 * (y_n * y_n + z_n * z_n),
    )


def plan_length(points: Iterable[tuple[float, float]]) -> float:
    """Calculate raw planar polyline length."""
    point_list = tuple(points)
    return sum(
        math.hypot(current[0] - previous[0], current[1] - previous[1])
        for previous, current in pairwise(point_list)
    )


def _quantize_micrometre(value: float) -> int:
    if not math.isfinite(value):
        raise ProtocolError('plan coordinate is not finite')
    magnitude = math.floor(abs(value) * 1_000_000 + 0.5)
    result = -magnitude if value < 0.0 else magnitude
    if not -(2**63) <= result < 2**63:
        raise ProtocolError('quantized plan coordinate exceeds signed 64-bit range')
    return result


def plan_geometry_sha256(frame_id: str, points: Iterable[tuple[float, float]]) -> str:
    """Hash plan geometry exactly as frozen by metrics-contract revision 2."""
    frame = frame_id.encode('utf-8', errors='strict')
    if len(frame) > 2**32 - 1:
        raise ProtocolError('plan frame ID is too long')
    quantized: list[tuple[int, int]] = []
    for x_value, y_value in points:
        pair = (_quantize_micrometre(x_value), _quantize_micrometre(y_value))
        if not quantized or pair != quantized[-1]:
            quantized.append(pair)
    digest = hashlib.sha256()
    digest.update(_PLAN_TAG)
    digest.update(struct.pack('!I', len(frame)))
    digest.update(frame)
    digest.update(struct.pack('!Q', len(quantized)))
    for x_value, y_value in quantized:
        digest.update(struct.pack('!qq', x_value, y_value))
    return digest.hexdigest()


def validate_plan(
    frame_id: str,
    points: Iterable[tuple[float, float]],
    waypoint: tuple[float, float],
) -> PlanEvidence:
    """Validate one leg plan and produce its stable evidence."""
    point_list = tuple(points)
    if frame_id != 'map':
        raise ProtocolError(f'plan frame must be map, found {frame_id!r}')
    if len(point_list) < 2:
        raise ProtocolError('plan must contain at least two poses')
    if not all(math.isfinite(value) for point in point_list for value in point):
        raise ProtocolError('plan contains a non-finite planar coordinate')
    length = plan_length(point_list)
    if length <= PATH_LENGTH_EPSILON_M:
        raise ProtocolError('plan length does not exceed the frozen epsilon')
    endpoint_error = math.hypot(
        point_list[-1][0] - waypoint[0],
        point_list[-1][1] - waypoint[1],
    )
    if endpoint_error > PLAN_ENDPOINT_TOLERANCE_M:
        raise ProtocolError('plan endpoint does not match the required waypoint')
    return PlanEvidence(
        frame_id=frame_id,
        points=point_list,
        length_m=length,
        endpoint_error_m=endpoint_error,
        geometry_sha256=plan_geometry_sha256(frame_id, point_list),
    )


def segment_intersects_closed_rectangle(
    start: tuple[float, float],
    end: tuple[float, float],
    rectangle: tuple[float, float, float, float],
) -> bool:
    """Return true when a segment touches or crosses a closed axis-aligned rectangle."""
    x_min, x_max, y_min, y_max = rectangle
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    lower = 0.0
    upper = 1.0
    for p_value, q_value in (
        (-delta_x, start[0] - x_min),
        (delta_x, x_max - start[0]),
        (-delta_y, start[1] - y_min),
        (delta_y, y_max - start[1]),
    ):
        if p_value == 0.0:
            if q_value < 0.0:
                return False
            continue
        ratio = q_value / p_value
        if p_value < 0.0:
            lower = max(lower, ratio)
        else:
            upper = min(upper, ratio)
        if lower > upper:
            return False
    return True


def plan_avoids_rectangle(
    points: Iterable[tuple[float, float]],
    rectangle: tuple[float, float, float, float],
) -> bool:
    """Require every segment to remain outside the closed exclusion rectangle."""
    point_list = tuple(points)
    return all(
        not segment_intersects_closed_rectangle(previous, current, rectangle)
        for previous, current in pairwise(point_list)
    )


def scenario3_target(index: int, anchor_stamp_ns: int) -> dict[str, int | float]:
    """Return one exact ADR 0006 trajectory target."""
    if not 0 <= index <= 120:
        raise ValueError('scenario 3 target index must be in 0..120')
    if index <= 40:
        x_value = (-80 + 2 * index) / 100.0
    elif index <= 80:
        x_value = 0.0
    else:
        x_value = (2 * (index - 80)) / 100.0
    return {
        'index': index,
        'stamp_ns': anchor_stamp_ns + index * 100_000_000,
        'x': x_value,
        'y': 1.5,
        'yaw': 0.0,
        'z': 0.4,
    }


def trajectory_sha256(records: object) -> str:
    """Hash bounded trajectory records with an explicit version tag."""
    return hashlib.sha256(_TRAJECTORY_TAG + canonical_json_bytes(records)).hexdigest()

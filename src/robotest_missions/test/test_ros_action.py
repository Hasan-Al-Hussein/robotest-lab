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

"""Focused tests for the direct rclpy action response projection."""

from types import SimpleNamespace

import pytest
from robotest_missions.execution import ActionProtocolError
from robotest_missions.ros_action import (
    ACTION_STATUS_TOPIC,
    RclpyFollowWaypointsDriver,
)

from robotest_missions import ros_action


class CompletedFuture:
    """Small completed-future fixture matching the rclpy methods in use."""

    def __init__(self, value) -> None:
        self._value = value

    def done(self) -> bool:
        return True

    def result(self):
        return self._value


class AcceptedHandle:
    """Action goal-handle fixture with a server response timestamp."""

    accepted = True
    goal_id = SimpleNamespace(uuid=bytes(range(16)))

    def __init__(self, stamp) -> None:
        self.stamp = stamp
        self.result_future = object()

    def get_result_async(self):
        return self.result_future


def driver_for(handle) -> RclpyFollowWaypointsDriver:
    driver = object.__new__(RclpyFollowWaypointsDriver)
    driver._send_future = CompletedFuture(handle)
    driver._goal_handle = None
    driver._result_future = None
    driver._cancel_future = None
    driver._goal_uuid_bytes = None
    driver._status_stamps = {}
    driver._status_protocol_errors = {}
    return driver


def status_entry(
    uuid_bytes: bytes,
    *,
    sec: int,
    nanosec: int,
    status_code: int = 1,
):
    return SimpleNamespace(
        goal_info=SimpleNamespace(
            goal_id=SimpleNamespace(uuid=uuid_bytes),
            stamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        ),
        status=status_code,
    )


def test_goal_response_records_zero_raw_stamp_without_treating_it_as_t0() -> None:
    driver = driver_for(AcceptedHandle(SimpleNamespace(sec=0, nanosec=0)))
    response = driver.poll_goal_response()
    assert response is not None
    assert response.accepted is True
    assert response.goal_uuid == '00010203-0405-0607-0809-0a0b0c0d0e0f'
    assert response.response_stamp_ns == 0
    assert driver._result_future is driver._goal_handle.result_future
    assert driver.accepted_goal_is_active() is True


@pytest.mark.parametrize(
    'stamp',
    [
        SimpleNamespace(sec=-1, nanosec=999_999_999),
        SimpleNamespace(sec=1, nanosec=-1),
        SimpleNamespace(sec=1, nanosec=1_000_000_000),
        SimpleNamespace(sec=True, nanosec=0),
        object(),
    ],
)
def test_goal_response_rejects_invalid_server_stamp(stamp) -> None:
    driver = driver_for(AcceptedHandle(stamp))
    with pytest.raises(ActionProtocolError, match='timestamp'):
        driver.poll_goal_response()
    assert driver.accepted_goal_is_active() is True
    assert driver._result_future is driver._goal_handle.result_future


def test_status_matches_exact_uuid_and_projects_authoritative_t0() -> None:
    exact_uuid = bytes(range(16))
    other_uuid = bytes(reversed(range(16)))
    driver = driver_for(AcceptedHandle(SimpleNamespace(sec=0, nanosec=0)))
    driver.poll_goal_response()
    driver._on_status(
        SimpleNamespace(
            status_list=[
                status_entry(other_uuid, sec=99, nanosec=0),
                status_entry(exact_uuid, sec=12, nanosec=345),
            ]
        )
    )
    assert driver.poll_accepted_status_stamp_ns() == 12_000_000_345


def test_status_rejects_conflicting_t0_for_exact_uuid() -> None:
    exact_uuid = bytes(range(16))
    driver = driver_for(AcceptedHandle(SimpleNamespace(sec=0, nanosec=0)))
    driver.poll_goal_response()
    driver._on_status(SimpleNamespace(status_list=[status_entry(exact_uuid, sec=12, nanosec=345)]))
    driver._on_status(SimpleNamespace(status_list=[status_entry(exact_uuid, sec=13, nanosec=345)]))
    with pytest.raises(ActionProtocolError, match='conflicting action-status'):
        driver.poll_accepted_status_stamp_ns()


def test_status_rejects_zero_to_positive_t0_change_for_exact_uuid() -> None:
    exact_uuid = bytes(range(16))
    driver = driver_for(AcceptedHandle(SimpleNamespace(sec=0, nanosec=0)))
    driver.poll_goal_response()
    driver._on_status(SimpleNamespace(status_list=[status_entry(exact_uuid, sec=0, nanosec=0)]))
    assert driver.poll_accepted_status_stamp_ns() is None
    driver._on_status(SimpleNamespace(status_list=[status_entry(exact_uuid, sec=1, nanosec=0)]))
    with pytest.raises(ActionProtocolError, match='conflicting action-status'):
        driver.poll_accepted_status_stamp_ns()


def test_status_rejects_structurally_invalid_t0_for_exact_uuid() -> None:
    exact_uuid = bytes(range(16))
    driver = driver_for(AcceptedHandle(SimpleNamespace(sec=0, nanosec=0)))
    driver.poll_goal_response()
    driver._on_status(
        SimpleNamespace(
            status_list=[
                status_entry(exact_uuid, sec=12, nanosec=1_000_000_000),
            ]
        )
    )
    with pytest.raises(ActionProtocolError, match='invalid action status'):
        driver.poll_accepted_status_stamp_ns()


@pytest.mark.parametrize('status_code', [0, 7, -1, True, '1'])
def test_status_rejects_invalid_status_code_for_exact_uuid(status_code) -> None:
    exact_uuid = bytes(range(16))
    driver = driver_for(AcceptedHandle(SimpleNamespace(sec=0, nanosec=0)))
    driver.poll_goal_response()
    driver._on_status(
        SimpleNamespace(
            status_list=[
                status_entry(
                    exact_uuid,
                    sec=12,
                    nanosec=345,
                    status_code=status_code,
                )
            ]
        )
    )
    with pytest.raises(ActionProtocolError, match='status code'):
        driver.poll_accepted_status_stamp_ns()


def test_status_subscription_uses_relative_name_and_action_status_qos(monkeypatch) -> None:
    captured = {}

    class FakeNode:
        def create_subscription(self, message_type, topic, callback, qos):
            captured.update(
                message_type=message_type,
                topic=topic,
                callback=callback,
                qos=qos,
            )
            return object()

    monkeypatch.setattr(ros_action, 'ActionClient', lambda *args: object())
    driver = RclpyFollowWaypointsDriver(FakeNode())
    assert ACTION_STATUS_TOPIC == 'follow_waypoints/_action/status'
    assert captured['message_type'] is ros_action.GoalStatusArray
    assert captured['topic'] == ACTION_STATUS_TOPIC
    assert captured['callback'] == driver._on_status
    assert captured['qos'] is ros_action.qos_profile_action_status_default

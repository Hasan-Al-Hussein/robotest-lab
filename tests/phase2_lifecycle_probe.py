#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Boundedly prove that an exact set of ROS 2 lifecycle nodes is active."""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path
from typing import Any

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.node import Node


class LifecycleProbe(Node):
    """Query several lifecycle services from one DDS participant."""

    def __init__(self, namespace: str, names: list[str]) -> None:
        super().__init__('phase2_lifecycle_probe', namespace='/robotest/evidence')
        base = '/' + namespace.strip('/')
        self._state_clients = {
            name: self.create_client(GetState, f'{base}/{name}/get_state') for name in names
        }


def probe_states(
    namespace: str,
    names: list[str],
    wall_timeout: float,
    watch_pid: int | None,
) -> dict[str, Any]:
    """Poll all requested nodes concurrently until each reports ACTIVE."""
    probe = LifecycleProbe(namespace, names)
    statuses = {
        name: {
            'attempts': 0,
            'label': None,
            'service': probe._state_clients[name].srv_name,
            'service_seen': False,
            'state_id': None,
        }
        for name in names
    }
    pending: dict[str, Any] = {}
    next_request = {name: 0.0 for name in names}
    deadline = time.monotonic() + wall_timeout
    failure = None
    try:
        while time.monotonic() < deadline:
            if watch_pid is not None:
                try:
                    os.kill(watch_pid, 0)
                except ProcessLookupError:
                    failure = f'watched launch process {watch_pid} exited'
                    break
            rclpy.spin_once(probe, timeout_sec=0.05)
            now = time.monotonic()
            for name, client in probe._state_clients.items():
                future = pending.get(name)
                if future is not None and future.done():
                    try:
                        response = future.result()
                    except Exception as error:  # pragma: no cover - middleware-specific
                        statuses[name]['error'] = f'{type(error).__name__}: {error}'
                    else:
                        statuses[name]['label'] = response.current_state.label
                        statuses[name]['state_id'] = int(response.current_state.id)
                    pending.pop(name, None)
                    next_request[name] = now + 0.25
                if statuses[name]['state_id'] == State.PRIMARY_STATE_ACTIVE:
                    continue
                if name not in pending and now >= next_request[name]:
                    if client.service_is_ready():
                        statuses[name]['service_seen'] = True
                        statuses[name]['attempts'] += 1
                        pending[name] = client.call_async(GetState.Request())
                    else:
                        next_request[name] = now + 0.25
            if all(item['state_id'] == State.PRIMARY_STATE_ACTIVE for item in statuses.values()):
                break
        active = all(item['state_id'] == State.PRIMARY_STATE_ACTIVE for item in statuses.values())
        if not active and failure is None:
            failure = f'lifecycle probe exceeded its {wall_timeout:.3f}s wall deadline'
        return {
            'verdict': 'PASS' if active and failure is None else 'FAIL',
            'failure': failure,
            'namespace': '/' + namespace.strip('/'),
            'required_state': {'id': State.PRIMARY_STATE_ACTIVE, 'label': 'active'},
            'states': statuses,
            'wall_timeout_s': wall_timeout,
            'watch_pid': watch_pid,
        }
    finally:
        probe.destroy_node()


def main() -> int:
    """Run the bounded lifecycle proof and persist canonical JSON/text evidence."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='/robotest')
    parser.add_argument('--node', action='append', required=True, dest='nodes')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--text-dir', type=Path)
    parser.add_argument('--text-prefix', default='lifecycle-')
    parser.add_argument('--wall-timeout', type=float, default=90.0)
    parser.add_argument('--watch-pid', type=int)
    args = parser.parse_args()
    if not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0.0:
        parser.error('--wall-timeout must be finite and positive')
    if len(args.nodes) != len(set(args.nodes)):
        parser.error('--node values must be unique')
    if any(not name or '/' in name for name in args.nodes):
        parser.error('--node values must be non-empty relative node names')

    rclpy.init()
    try:
        result = probe_states(
            args.namespace,
            args.nodes,
            args.wall_timeout,
            args.watch_pid,
        )
    finally:
        rclpy.shutdown()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    output_pending = args.output.with_name(f'.{args.output.name}.pending')
    output_pending.write_text(
        json.dumps(result, allow_nan=False, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    output_pending.replace(args.output)
    if args.text_dir is not None:
        args.text_dir.mkdir(parents=True, exist_ok=True)
        for name, status in result['states'].items():
            label = status['label'] or 'unavailable'
            state_id = status['state_id']
            (args.text_dir / f'{args.text_prefix}{name}.txt').write_text(
                f'{label} [{state_id}]\n', encoding='utf-8'
            )
    print(json.dumps(result, allow_nan=False, indent=2, sort_keys=True))
    return 0 if result['verdict'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())

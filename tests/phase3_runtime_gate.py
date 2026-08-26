#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: I001

"""Boundedly prove Phase 3 namespace, QoS, ownership, and isolation gates."""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time
from typing import Any

from phase3_orchestration import atomic_write_json, EvidenceError

MAX_ENDPOINTS = 4096
CONTACT_MESSAGE_TYPE = 'ros_gz_interfaces/msg/Contacts'
MAX_ATTEMPTS = 4096
MAX_PROC_ENTRIES = 65_536
MAX_PROC_MAPS_BYTES = 8 * 1024 * 1024
MAX_PROC_MAPS_LINES = 65_536
MAX_PROC_MAP_LINE_BYTES = 16 * 1024
MAX_PROC_METADATA_BYTES = 1024 * 1024
MAX_DSO_MAPPINGS = 64
MAX_EVIDENCE_STRING_BYTES = 4096

CONTACT_PIPELINE_SOURCE_PATHS = (
    'src/robotest_description/urdf/robotest_gazebo.xacro',
    'src/robotest_sim/CMakeLists.txt',
    'src/robotest_sim/config/bridge.yaml',
    'src/robotest_sim/include/robotest_sim/contact_aggregator.hpp',
    'src/robotest_sim/include/robotest_sim/contact_stream_gate.hpp',
    'src/robotest_sim/launch/sim.launch.py',
    'src/robotest_sim/src/contact_aggregator.cpp',
    'src/robotest_sim/src/contact_aggregator_system.cpp',
    'src/robotest_sim/src/contact_stream_gate.cpp',
    'src/robotest_sim/src/contact_stream_gate_node.cpp',
    'src/robotest_sim/worlds/robotest_lab.sdf',
)
CONTACT_AGGREGATOR_BUILD_PATH = Path('build/robotest_sim/librobotest_contact_aggregator_system.so')
CONTACT_AGGREGATOR_INSTALLED_PATH = Path(
    'install/robotest_sim/lib/robotest_sim/librobotest_contact_aggregator_system.so'
)


@dataclass(frozen=True)
class ProcMapEntry:
    """One bounded, validated Linux ``/proc/<pid>/maps`` record."""

    address_start: int
    address_end: int
    permissions: str
    offset: int
    device: int
    inode: int
    path: str | None


AUTONOMY_NODE_NAMES = {
    'amcl',
    'behavior_server',
    'bt_navigator',
    'collision_monitor',
    'controller_server',
    'fault_proxy',
    'lifecycle_manager_navigation',
    'map_server',
    'mission_runner',
    'planner_server',
    'velocity_smoother',
    'waypoint_follower',
}

VALIDATION_TOPICS = (
    '/robotest/validation/contacts',
    '/robotest/validation/ground_truth',
    '/robotest/validation/scenario_entity_poses',
    '/robotest/validation/world_stats',
)

QOS_CONTRACTS: dict[str, tuple[str, str, int]] = {
    '/clock': ('BEST_EFFORT', 'VOLATILE', 1),
    '/robotest/map': ('RELIABLE', 'TRANSIENT_LOCAL', 1),
    '/robotest/raw/scan': ('BEST_EFFORT', 'VOLATILE', 5),
    '/robotest/scan': ('BEST_EFFORT', 'VOLATILE', 5),
    '/robotest/raw/odom': ('BEST_EFFORT', 'VOLATILE', 10),
    '/robotest/odom': ('BEST_EFFORT', 'VOLATILE', 10),
    '/robotest/raw/imu': ('BEST_EFFORT', 'VOLATILE', 10),
    '/robotest/imu': ('BEST_EFFORT', 'VOLATILE', 10),
    '/robotest/navigation/plan': ('RELIABLE', 'VOLATILE', 5),
    '/robotest/cmd_vel_nav': ('RELIABLE', 'VOLATILE', 1),
    '/robotest/cmd_vel_smoothed': ('RELIABLE', 'VOLATILE', 1),
    '/robotest/cmd_vel': ('RELIABLE', 'VOLATILE', 1),
    '/robotest/cmd_vel_behavior_unused': ('RELIABLE', 'VOLATILE', 1),
    '/robotest/collision_monitor_state': ('RELIABLE', 'VOLATILE', 10),
    '/robotest/validation/ground_truth': ('RELIABLE', 'VOLATILE', 10),
    '/robotest/validation/contacts': ('RELIABLE', 'VOLATILE', 10),
    '/robotest/internal/raw_contacts': ('RELIABLE', 'VOLATILE', 64),
    '/robotest/validation/world_stats': ('RELIABLE', 'VOLATILE', 10),
    '/robotest/validation/scenario_entity_poses': ('RELIABLE', 'VOLATILE', 10),
    '/robotest/faults/events': ('RELIABLE', 'VOLATILE', 100),
    '/tf': ('RELIABLE', 'VOLATILE', 100),
    '/tf_static': ('RELIABLE', 'TRANSIENT_LOCAL', 1),
}

CANDIDATE_REQUIRED_NODES = AUTONOMY_NODE_NAMES - {'mission_runner'} | {
    'metrics_collector',
    'contact_stream_gate',
    'parameter_bridge',
    'phase3_goal_observer',
    'robot_state_publisher',
    'scenario_bridge',
    'scenario_controller',
}

POSITIVE_FORBIDDEN_NODES = AUTONOMY_NODE_NAMES - {'fault_proxy'}

CANDIDATE_EXPECTED_PUBLISHERS: dict[str, set[str]] = {
    '/clock': {'/robotest/parameter_bridge'},
    '/robotest/map': {'/robotest/map_server'},
    '/robotest/raw/scan': {'/robotest/parameter_bridge'},
    '/robotest/scan': {'/robotest/fault_proxy'},
    '/robotest/raw/odom': {'/robotest/parameter_bridge'},
    '/robotest/odom': {'/robotest/fault_proxy'},
    '/robotest/raw/imu': {'/robotest/parameter_bridge'},
    '/robotest/imu': {'/robotest/fault_proxy'},
    '/robotest/navigation/plan': {'/robotest/planner_server'},
    '/robotest/cmd_vel_nav': {'/robotest/controller_server'},
    '/robotest/cmd_vel_smoothed': {'/robotest/velocity_smoother'},
    '/robotest/cmd_vel': {'/robotest/collision_monitor'},
    '/robotest/cmd_vel_behavior_unused': {'/robotest/behavior_server'},
    '/robotest/collision_monitor_state': {'/robotest/collision_monitor'},
    '/robotest/validation/ground_truth': {'/robotest/parameter_bridge'},
    '/robotest/validation/contacts': {'/robotest/contact_stream_gate'},
    '/robotest/internal/raw_contacts': {'/robotest/parameter_bridge'},
    '/robotest/validation/world_stats': {'/robotest/parameter_bridge'},
    '/robotest/validation/scenario_entity_poses': {'/robotest/parameter_bridge'},
    '/robotest/faults/events': {'/robotest/fault_proxy'},
    '/tf': {
        '/robotest/amcl',
        '/robotest/fault_proxy',
        '/robotest/robot_state_publisher',
    },
    '/tf_static': {'/robotest/robot_state_publisher'},
}

CANDIDATE_EXPECTED_COMMAND_SUBSCRIBERS: dict[str, set[str]] = {
    '/robotest/cmd_vel_nav': {'/robotest/velocity_smoother'},
    '/robotest/cmd_vel_smoothed': {'/robotest/collision_monitor'},
    '/robotest/cmd_vel': {'/robotest/parameter_bridge'},
    '/robotest/cmd_vel_behavior_unused': set(),
}

CANDIDATE_EXPECTED_CONTACT_SUBSCRIBERS: dict[str, set[str]] = {
    '/robotest/internal/raw_contacts': {'/robotest/contact_stream_gate'},
}

SCENARIO_SERVICES = (
    '/robotest/scenario/delete_entity',
    '/robotest/scenario/set_entity_pose',
    '/robotest/scenario/spawn_entity',
)


def _pid_alive(pid: int | None) -> bool:
    if pid is None:
        return True
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _contact_gate_source_inventory_sha256(workspace: Path) -> str:
    inventory = {
        'schema_version': 1,
        'sources': [
            {'path': path, 'sha256': _sha256(workspace / path)}
            for path in CONTACT_PIPELINE_SOURCE_PATHS
        ],
    }
    payload = (
        json.dumps(
            inventory,
            allow_nan=False,
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True,
        ).encode('utf-8')
        + b'\n'
    )
    return hashlib.sha256(payload).hexdigest()


def _elf_build_id(path: Path) -> str | None:
    result = subprocess.run(
        ['readelf', '-n', str(path)],
        capture_output=True,
        check=False,
        text=True,
        timeout=10.0,
    )
    for line in result.stdout.splitlines():
        if 'Build ID:' in line:
            return line.split('Build ID:', 1)[1].strip()
    return None


CONTACT_GATE_SOURCE_TAG = b'ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256='


def _elf_embedded_source_inventory_sha256(path: Path) -> str:
    pattern = re.compile(re.escape(CONTACT_GATE_SOURCE_TAG) + rb'([0-9a-f]{64})')
    overlap = b''
    matches: set[str] = set()
    retained = len(CONTACT_GATE_SOURCE_TAG) + 63
    with path.open('rb') as stream:
        while chunk := stream.read(1024 * 1024):
            combined = overlap + chunk
            for match in pattern.finditer(combined):
                matches.add(match.group(1).decode('ascii'))
                if len(matches) > 1:
                    raise EvidenceError(
                        f'contact gate ELF has conflicting tagged source inventory values: {path}'
                    )
            overlap = combined[-retained:]
    if len(matches) != 1:
        raise EvidenceError(f'contact gate ELF lacks tagged source inventory: {path}')
    return next(iter(matches))


def _canonical_sha256(value: Any) -> str:
    payload = (
        json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True,
        ).encode('utf-8')
        + b'\n'
    )
    return hashlib.sha256(payload).hexdigest()


def _contact_aggregator_build_install_record(workspace: Path) -> dict[str, Any]:
    """Bind the built and installed contact-aggregator DSO to one source digest."""
    workspace = workspace.resolve(strict=True)
    build_declared_path = workspace / CONTACT_AGGREGATOR_BUILD_PATH
    installed_declared_path = workspace / CONTACT_AGGREGATOR_INSTALLED_PATH
    build_path = build_declared_path.resolve(strict=True)
    installed_path = installed_declared_path.resolve(strict=True)
    if not build_path.is_file() or not installed_path.is_file():
        raise EvidenceError('contact aggregator build/install artifact is not a regular file')
    try:
        build_relative_path = build_path.relative_to(workspace).as_posix()
        installed_relative_path = installed_path.relative_to(workspace).as_posix()
    except ValueError as exc:
        raise EvidenceError(
            'contact aggregator build/install artifact escapes the workspace'
        ) from exc

    build_sha256 = _sha256(build_path)
    installed_sha256 = _sha256(installed_path)
    build_elf_build_id = _elf_build_id(build_path)
    installed_elf_build_id = _elf_build_id(installed_path)
    if build_elf_build_id is None or installed_elf_build_id is None:
        raise EvidenceError('contact aggregator build/install ELF build ID is unavailable')
    source_inventory_sha256 = _contact_gate_source_inventory_sha256(workspace)
    build_embedded_source_inventory_sha256 = _elf_embedded_source_inventory_sha256(build_path)
    installed_embedded_source_inventory_sha256 = _elf_embedded_source_inventory_sha256(
        installed_path
    )
    build_embedded_source_inventory_match = (
        build_embedded_source_inventory_sha256 == source_inventory_sha256
    )
    installed_embedded_source_inventory_match = (
        installed_embedded_source_inventory_sha256 == source_inventory_sha256
    )
    installed_declared_is_symlink = installed_declared_path.is_symlink()
    build_install_samefile = build_path.samefile(installed_path)
    build_install_sha256_match = build_sha256 == installed_sha256
    build_install_build_id_match = build_elf_build_id == installed_elf_build_id
    build_install_embedded_source_inventory_match = (
        build_embedded_source_inventory_sha256 == installed_embedded_source_inventory_sha256
    )
    if not all(
        (
            build_embedded_source_inventory_match,
            installed_embedded_source_inventory_match,
            build_install_sha256_match,
            build_install_build_id_match,
            build_install_embedded_source_inventory_match,
        )
    ):
        raise EvidenceError('contact aggregator build/install source or ELF identity differs')
    if installed_declared_is_symlink and not build_install_samefile:
        raise EvidenceError(
            'contact aggregator symlink install does not resolve to the build artifact'
        )
    return {
        'build_elf_build_id': build_elf_build_id,
        'build_embedded_source_inventory_match': build_embedded_source_inventory_match,
        'build_embedded_source_inventory_sha256': build_embedded_source_inventory_sha256,
        'build_install_build_id_match': build_install_build_id_match,
        'build_install_embedded_source_inventory_match': (
            build_install_embedded_source_inventory_match
        ),
        'build_install_samefile': build_install_samefile,
        'build_install_sha256_match': build_install_sha256_match,
        'build_path': build_relative_path,
        'build_regular_file': True,
        'build_sha256': build_sha256,
        'installed_declared_is_symlink': installed_declared_is_symlink,
        'installed_declared_path': CONTACT_AGGREGATOR_INSTALLED_PATH.as_posix(),
        'installed_elf_build_id': installed_elf_build_id,
        'installed_embedded_source_inventory_match': installed_embedded_source_inventory_match,
        'installed_embedded_source_inventory_sha256': (installed_embedded_source_inventory_sha256),
        'installed_path': installed_relative_path,
        'installed_regular_file': True,
        'installed_sha256': installed_sha256,
        'package': 'robotest_sim',
        'schema_version': 1,
        'source_inventory_sha256': source_inventory_sha256,
    }


def _validated_contact_aggregator_build_install_record(
    workspace: Path,
    expected: Mapping[str, Any] | None,
) -> dict[str, Any]:
    actual = _contact_aggregator_build_install_record(workspace)
    if expected is not None and dict(expected) != actual:
        raise EvidenceError('contact aggregator runtime build/install record differs')
    return actual


def _read_bounded_bytes(path: Path, maximum_bytes: int, label: str) -> bytes:
    with path.open('rb') as stream:
        payload = stream.read(maximum_bytes + 1)
    if len(payload) > maximum_bytes:
        raise EvidenceError(f'{label} exceeds the {maximum_bytes}-byte bound')
    return payload


def _parse_proc_maps(payload: bytes) -> tuple[ProcMapEntry, ...]:
    """Parse a bounded Linux maps payload without trusting path text as identity."""
    if len(payload) > MAX_PROC_MAPS_BYTES:
        raise EvidenceError(f'/proc maps exceeds the {MAX_PROC_MAPS_BYTES}-byte bound')
    lines = payload.splitlines()
    if len(lines) > MAX_PROC_MAPS_LINES:
        raise EvidenceError(f'/proc maps exceeds the {MAX_PROC_MAPS_LINES}-line bound')
    entries: list[ProcMapEntry] = []
    previous_end = 0
    for line_number, line in enumerate(lines, start=1):
        if not line or len(line) > MAX_PROC_MAP_LINE_BYTES:
            raise EvidenceError(f'/proc maps line {line_number} is empty or oversized')
        fields = line.split(None, 5)
        if len(fields) < 5:
            raise EvidenceError(f'/proc maps line {line_number} has fewer than five fields')
        try:
            address_parts = fields[0].split(b'-', 1)
            if len(address_parts) != 2:
                raise ValueError('address range is missing its separator')
            address_start = int(address_parts[0], 16)
            address_end = int(address_parts[1], 16)
            permissions = fields[1].decode('ascii', errors='strict')
            offset = int(fields[2], 16)
            device_parts = fields[3].split(b':', 1)
            if len(device_parts) != 2:
                raise ValueError('device is missing its separator')
            device = os.makedev(int(device_parts[0], 16), int(device_parts[1], 16))
            inode_text = fields[4].decode('ascii', errors='strict')
            if not inode_text.isdecimal():
                raise ValueError('inode is not decimal')
            inode = int(inode_text, 10)
            raw_path = fields[5] if len(fields) == 6 else None
            path = None
            if raw_path is not None:
                path = raw_path.decode('utf-8', errors='surrogateescape')
        except (OverflowError, UnicodeError, ValueError) as exc:
            raise EvidenceError(f'/proc maps line {line_number} is malformed: {exc}') from exc
        if (
            address_start < previous_end
            or address_end <= address_start
            or re.fullmatch(r'[r-][w-][x-][ps]', permissions) is None
        ):
            raise EvidenceError(f'/proc maps line {line_number} has invalid range or permissions')
        entries.append(
            ProcMapEntry(
                address_start=address_start,
                address_end=address_end,
                permissions=permissions,
                offset=offset,
                device=device,
                inode=inode,
                path=path,
            )
        )
        previous_end = address_end
    return tuple(entries)


def _read_proc_maps(proc_root: Path, pid: int) -> tuple[ProcMapEntry, ...]:
    payload = _read_bounded_bytes(
        proc_root / str(pid) / 'maps',
        MAX_PROC_MAPS_BYTES,
        f'/proc/{pid}/maps',
    )
    return _parse_proc_maps(payload)


def _map_entry_record(entry: ProcMapEntry) -> dict[str, Any]:
    return {
        'address_end': entry.address_end,
        'address_start': entry.address_start,
        'device': entry.device,
        'inode': entry.inode,
        'offset': entry.offset,
        'path': entry.path,
        'permissions': entry.permissions,
    }


def _contact_aggregator_map_owners(
    proc_root: Path,
    descendants: set[int],
    *,
    installed_device: int,
    installed_inode: int,
) -> list[tuple[int, tuple[ProcMapEntry, ...]]]:
    """Return descendant owners of the exact installed DSO device/inode."""
    if len(descendants) > MAX_PROC_ENTRIES:
        raise EvidenceError('launch descendant set exceeds the 65,536-process bound')
    owners: list[tuple[int, tuple[ProcMapEntry, ...]]] = []
    for pid in sorted(descendants):
        try:
            mappings = _read_proc_maps(proc_root, pid)
        except (FileNotFoundError, PermissionError):
            continue
        matching = tuple(
            entry
            for entry in mappings
            if entry.device == installed_device and entry.inode == installed_inode
        )
        if not matching:
            continue
        if any(entry.path is None or entry.path.endswith(' (deleted)') for entry in matching):
            raise EvidenceError('contact aggregator has an anonymous or deleted live mapping')
        if len(matching) > MAX_DSO_MAPPINGS:
            raise EvidenceError('contact aggregator live mapping count exceeds 64')
        try:
            mapping_paths = [entry.path.encode('utf-8') for entry in matching if entry.path]
        except UnicodeEncodeError as exc:
            raise EvidenceError('contact aggregator live mapping path is not valid UTF-8') from exc
        if any(len(path) > MAX_EVIDENCE_STRING_BYTES for path in mapping_paths):
            raise EvidenceError('contact aggregator live mapping path exceeds 4096 UTF-8 bytes')
        if not any(entry.offset == 0 for entry in matching):
            raise EvidenceError('contact aggregator live mappings lack an offset-zero segment')
        if not any('x' in entry.permissions for entry in matching):
            raise EvidenceError('contact aggregator live mappings lack an executable segment')
        owners.append((pid, matching))
    return owners


def _one_contact_aggregator_map_owner(
    proc_root: Path,
    descendants: set[int],
    *,
    installed_device: int,
    installed_inode: int,
) -> tuple[int, tuple[ProcMapEntry, ...]]:
    owners = _contact_aggregator_map_owners(
        proc_root,
        descendants,
        installed_device=installed_device,
        installed_inode=installed_inode,
    )
    if len(owners) != 1:
        raise EvidenceError(
            f'expected exactly one live contact aggregator DSO owner, found {len(owners)}'
        )
    return owners[0]


def _proc_stat_fields(stat_text: str) -> list[str]:
    closing = stat_text.rfind(')')
    if closing < 0:
        return []
    fields_start = closing + 2
    return stat_text[fields_start:].split()


def _proc_parent_pid(stat_text: str) -> int:
    fields = _proc_stat_fields(stat_text)
    if len(fields) < 2:
        raise ValueError('malformed /proc stat')
    return int(fields[1])


def _descendant_pids(root_pid: int, proc_root: Path = Path('/proc')) -> set[int]:
    parents: dict[int, int] = {}
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        if len(parents) >= MAX_PROC_ENTRIES:
            raise EvidenceError('/proc process set exceeds the 65,536-process bound')
        try:
            parents[int(entry.name)] = _proc_parent_pid(
                (entry / 'stat').read_text(encoding='ascii')
            )
        except (FileNotFoundError, PermissionError, UnicodeError, ValueError):
            continue
    descendants = {root_pid}
    changed = True
    while changed:
        changed = False
        for pid, parent in parents.items():
            if parent in descendants and pid not in descendants:
                descendants.add(pid)
                changed = True
    return descendants


def _proc_process_fields(proc_root: Path, pid: int) -> tuple[int, int, int, int]:
    payload = _read_bounded_bytes(
        proc_root / str(pid) / 'stat',
        MAX_PROC_METADATA_BYTES,
        f'/proc/{pid}/stat',
    )
    fields = _proc_stat_fields(payload.decode('ascii', errors='strict'))
    if len(fields) < 20:
        raise EvidenceError(f'/proc/{pid}/stat is malformed')
    try:
        return int(fields[1]), int(fields[2]), int(fields[3]), int(fields[19])
    except ValueError as exc:
        raise EvidenceError(f'/proc/{pid}/stat process identity is malformed') from exc


def _proc_environment(proc_root: Path, pid: int) -> dict[str, str]:
    payload = _read_bounded_bytes(
        proc_root / str(pid) / 'environ',
        MAX_PROC_METADATA_BYTES,
        f'/proc/{pid}/environ',
    )
    result: dict[str, str] = {}
    for entry in payload.split(b'\0'):
        if not entry:
            continue
        key, separator, value = entry.partition(b'=')
        if not separator:
            raise EvidenceError(f'/proc/{pid}/environ contains a malformed entry')
        decoded_key = key.decode('utf-8', errors='strict')
        decoded_value = value.decode('utf-8', errors='strict')
        if not decoded_key or decoded_key in result:
            raise EvidenceError(f'/proc/{pid}/environ contains an empty or duplicate key')
        result[decoded_key] = decoded_value
    return result


def _contact_gate_binary_attestation(
    workspace: Path,
    launch_pid: int,
    expected_domain_id: int,
    expected_gz_partition: str,
) -> dict[str, Any]:
    """Bind the one live gate process to the exact installed and built ELF."""
    workspace = workspace.resolve(strict=True)
    build_relative_path = Path('build/robotest_sim/contact_stream_gate')
    installed_relative_path = Path('install/robotest_sim/lib/robotest_sim/contact_stream_gate')
    build_declared_path = workspace / build_relative_path
    installed_declared_path = workspace / installed_relative_path
    try:
        build_path = build_declared_path.resolve(strict=True)
        installed_path = installed_declared_path.resolve(strict=True)
        descendants = _descendant_pids(launch_pid)
        candidates: list[tuple[int, Path]] = []
        for pid in sorted(descendants):
            try:
                executable = Path(f'/proc/{pid}/exe').resolve(strict=True)
            except (FileNotFoundError, PermissionError, OSError):
                continue
            if executable.name == 'contact_stream_gate':
                candidates.append((pid, executable))
        if len(candidates) != 1:
            raise EvidenceError(
                f'expected exactly one live contact_stream_gate process, found {len(candidates)}'
            )
        live_pid, live_path = candidates[0]
        stat_text = Path(f'/proc/{live_pid}/stat').read_text(encoding='ascii')
        stat_fields = _proc_stat_fields(stat_text)
        if len(stat_fields) < 20:
            raise EvidenceError('live contact gate /proc stat is malformed')
        live_ppid = int(stat_fields[1])
        live_pgid = int(stat_fields[2])
        live_sid = int(stat_fields[3])
        live_start_ticks = int(stat_fields[19])
        live_stat_before = Path(f'/proc/{live_pid}/exe').stat()
        cmdline = Path(f'/proc/{live_pid}/cmdline').read_bytes()
        environment_entries = Path(f'/proc/{live_pid}/environ').read_bytes().split(b'\0')
        environment = {
            key.decode('utf-8', errors='strict'): value.decode('utf-8', errors='strict')
            for entry in environment_entries
            if entry
            for key, separator, value in [entry.partition(b'=')]
            if separator
        }
        observed_domain_id = environment.get('ROS_DOMAIN_ID')
        observed_gz_partition = environment.get('GZ_PARTITION')
        source_inventory_sha256 = _contact_gate_source_inventory_sha256(workspace)
        build_hash = _sha256(build_path)
        installed_hash = _sha256(installed_path)
        live_hash = _sha256(Path(f'/proc/{live_pid}/exe'))
        build_id = _elf_build_id(build_path)
        installed_build_id = _elf_build_id(installed_path)
        live_build_id = _elf_build_id(Path(f'/proc/{live_pid}/exe'))
        installed_stat = installed_path.stat()
        live_stat = Path(f'/proc/{live_pid}/exe').stat()
        final_stat_text = Path(f'/proc/{live_pid}/stat').read_text(encoding='ascii')
        final_fields = _proc_stat_fields(final_stat_text)
        identity_revalidated_after_hashing = (
            len(final_fields) >= 20
            and int(final_fields[19]) == live_start_ticks
            and live_stat_before.st_dev == live_stat.st_dev
            and live_stat_before.st_ino == live_stat.st_ino
        )
        installed_regular_executable = installed_path.is_file() and os.access(
            installed_path, os.X_OK
        )
        build_regular_executable = build_path.is_file() and os.access(build_path, os.X_OK)
        installed_declared_is_symlink = installed_declared_path.is_symlink()
        installed_declared_samefile = installed_declared_path.samefile(installed_path)
        build_install_samefile = build_path.samefile(installed_path)
        install_path_binding_valid = (
            installed_declared_is_symlink
            and build_install_samefile
            and installed_path == build_path
        ) or (not installed_declared_is_symlink and installed_path == installed_declared_path)
        build_install_match = (
            build_id is not None
            and installed_build_id is not None
            and build_id == installed_build_id
        )
        build_install_sha256_match = build_hash == installed_hash
        live_installed_inode_match = (
            live_stat.st_dev == installed_stat.st_dev and live_stat.st_ino == installed_stat.st_ino
        )
        live_installed_sha256_match = live_hash == installed_hash
        live_installed_build_id_match = (
            live_build_id is not None and live_build_id == installed_build_id
        )
        build_embedded_source_inventory_sha256 = _elf_embedded_source_inventory_sha256(build_path)
        installed_embedded_source_inventory_sha256 = _elf_embedded_source_inventory_sha256(
            installed_path
        )
        live_embedded_source_inventory_sha256 = _elf_embedded_source_inventory_sha256(
            Path(f'/proc/{live_pid}/exe')
        )
        build_embedded_source_inventory_match = (
            build_embedded_source_inventory_sha256 == source_inventory_sha256
        )
        installed_embedded_source_inventory_match = (
            installed_embedded_source_inventory_sha256 == source_inventory_sha256
        )
        live_embedded_source_inventory_match = (
            live_embedded_source_inventory_sha256 == source_inventory_sha256
        )
        process_identity_match = (
            live_pgid == launch_pid
            and live_sid == launch_pid
            and observed_domain_id == str(expected_domain_id)
            and observed_gz_partition == expected_gz_partition
        )
        passed = (
            build_install_match
            and build_install_sha256_match
            and build_regular_executable
            and installed_regular_executable
            and installed_declared_samefile
            and install_path_binding_valid
            and live_installed_inode_match
            and live_installed_sha256_match
            and live_installed_build_id_match
            and build_embedded_source_inventory_match
            and installed_embedded_source_inventory_match
            and live_embedded_source_inventory_match
            and process_identity_match
            and identity_revalidated_after_hashing
        )
        return {
            'build_embedded_source_inventory_match': (build_embedded_source_inventory_match),
            'build_embedded_source_inventory_sha256': (build_embedded_source_inventory_sha256),
            'build_elf_build_id': build_id,
            'build_install_build_id_match': build_install_match,
            'build_install_samefile': build_install_samefile,
            'build_install_sha256_match': build_install_sha256_match,
            'build_path': build_relative_path.as_posix(),
            'build_regular_executable': build_regular_executable,
            'build_sha256': build_hash,
            'exact_live_process_count': len(candidates),
            'installed_device': installed_stat.st_dev,
            'installed_declared_is_symlink': installed_declared_is_symlink,
            'installed_declared_path': installed_relative_path.as_posix(),
            'installed_declared_samefile': installed_declared_samefile,
            'installed_embedded_source_inventory_match': (
                installed_embedded_source_inventory_match
            ),
            'installed_embedded_source_inventory_sha256': (
                installed_embedded_source_inventory_sha256
            ),
            'installed_elf_build_id': installed_build_id,
            'installed_inode': installed_stat.st_ino,
            'installed_path': installed_path.relative_to(workspace).as_posix(),
            'installed_regular_executable': installed_regular_executable,
            'installed_sha256': installed_hash,
            'identity_revalidated_after_hashing': identity_revalidated_after_hashing,
            'launch_root_pid': launch_pid,
            'live_cmdline_sha256': hashlib.sha256(cmdline).hexdigest(),
            'live_device': live_stat.st_dev,
            'live_elf_build_id': live_build_id,
            'live_embedded_source_inventory_match': live_embedded_source_inventory_match,
            'live_embedded_source_inventory_sha256': live_embedded_source_inventory_sha256,
            'live_executable_link': os.readlink(f'/proc/{live_pid}/exe'),
            'live_executable_path': str(live_path),
            'live_executable_sha256': live_hash,
            'live_inode': live_stat.st_ino,
            'live_installed_build_id_match': live_installed_build_id_match,
            'live_installed_inode_match': live_installed_inode_match,
            'live_installed_sha256_match': live_installed_sha256_match,
            'live_pgid': live_pgid,
            'live_pid': live_pid,
            'live_ppid': live_ppid,
            'live_sid': live_sid,
            'live_size_bytes': live_stat.st_size,
            'live_start_ticks': live_start_ticks,
            'observed_gz_partition': observed_gz_partition,
            'observed_ros_domain_id': observed_domain_id,
            'package': 'robotest_sim',
            'process_identity_match': process_identity_match,
            'schema_version': 1,
            'source_inventory_sha256': source_inventory_sha256,
            'verdict': 'PASS' if passed else 'FAIL',
        }
    except (EvidenceError, OSError, subprocess.SubprocessError, ValueError) as exc:
        return {
            'error': str(exc)[:4096],
            'launch_root_pid': launch_pid,
            'package': 'robotest_sim',
            'schema_version': 1,
            'verdict': 'FAIL',
        }


def _contact_aggregator_binary_attestation(
    workspace: Path,
    launch_pid: int,
    expected_domain_id: int,
    expected_gz_partition: str,
    *,
    build_install_record: Mapping[str, Any] | None = None,
    proc_root: Path = Path('/proc'),
) -> dict[str, Any]:
    """Bind one live Gazebo-loaded DSO mapping to its installed source-tagged ELF."""
    try:
        workspace = workspace.resolve(strict=True)
        proc_root = proc_root.resolve(strict=True)
        frozen = _validated_contact_aggregator_build_install_record(
            workspace,
            build_install_record,
        )
        installed_path = (workspace / str(frozen['installed_path'])).resolve(strict=True)
        installed_stat_before = installed_path.stat()
        if not installed_path.is_file():
            raise EvidenceError('installed contact aggregator is not a regular file')

        descendants_before = _descendant_pids(launch_pid, proc_root)
        live_pid, initial_mappings = _one_contact_aggregator_map_owner(
            proc_root,
            descendants_before,
            installed_device=installed_stat_before.st_dev,
            installed_inode=installed_stat_before.st_ino,
        )
        proc_dir = proc_root / str(live_pid)
        live_ppid, live_pgid, live_sid, live_start_ticks = _proc_process_fields(
            proc_root,
            live_pid,
        )
        live_executable = proc_dir / 'exe'
        live_executable_link_before = os.readlink(live_executable)
        live_executable_path = live_executable.resolve(strict=True)
        live_executable_stat_before = live_executable.stat()
        live_cmdline = _read_bounded_bytes(
            proc_dir / 'cmdline',
            MAX_PROC_METADATA_BYTES,
            f'/proc/{live_pid}/cmdline',
        )
        environment = _proc_environment(proc_root, live_pid)
        observed_domain_id = environment.get('ROS_DOMAIN_ID')
        observed_gz_partition = environment.get('GZ_PARTITION')
        process_identity_match = (
            live_pgid == launch_pid
            and live_sid == launch_pid
            and observed_domain_id == str(expected_domain_id)
            and observed_gz_partition == expected_gz_partition
        )

        initial_mapping_records = [_map_entry_record(entry) for entry in initial_mappings]
        live_mapping_fingerprint_sha256 = _canonical_sha256(initial_mapping_records)
        live_mapping_paths = sorted({entry.path for entry in initial_mappings if entry.path})
        installed_sha256_after_mapping = _sha256(installed_path)
        installed_embedded_source_after_mapping = _elf_embedded_source_inventory_sha256(
            installed_path
        )

        installed_stat_after = installed_path.stat()
        final_ppid, final_pgid, final_sid, final_start_ticks = _proc_process_fields(
            proc_root,
            live_pid,
        )
        live_executable_link_after = os.readlink(live_executable)
        live_executable_stat_after = live_executable.stat()
        descendants_after = _descendant_pids(launch_pid, proc_root)
        final_pid, final_mappings = _one_contact_aggregator_map_owner(
            proc_root,
            descendants_after,
            installed_device=installed_stat_after.st_dev,
            installed_inode=installed_stat_after.st_ino,
        )
        final_mapping_records = [_map_entry_record(entry) for entry in final_mappings]

        installed_identity_revalidated_after_hashing = (
            installed_stat_before.st_dev == installed_stat_after.st_dev
            and installed_stat_before.st_ino == installed_stat_after.st_ino
            and installed_stat_before.st_size == installed_stat_after.st_size
            and installed_stat_before.st_mtime_ns == installed_stat_after.st_mtime_ns
            and installed_stat_before.st_ctime_ns == installed_stat_after.st_ctime_ns
            and installed_sha256_after_mapping == frozen['installed_sha256']
            and installed_embedded_source_after_mapping
            == frozen['installed_embedded_source_inventory_sha256']
        )
        identity_revalidated_after_hashing = (
            final_pid == live_pid
            and final_ppid == live_ppid
            and final_pgid == live_pgid
            and final_sid == live_sid
            and final_start_ticks == live_start_ticks
            and live_executable_stat_before.st_dev == live_executable_stat_after.st_dev
            and live_executable_stat_before.st_ino == live_executable_stat_after.st_ino
            and live_executable_link_before == live_executable_link_after
        )
        maps_revalidated_after_hashing = final_mapping_records == initial_mapping_records
        live_installed_inode_match = all(
            entry.device == installed_stat_after.st_dev
            and entry.inode == installed_stat_after.st_ino
            for entry in final_mappings
        )
        live_installed_sha256_match = (
            live_installed_inode_match
            and installed_sha256_after_mapping == frozen['installed_sha256']
        )
        live_installed_build_id_match = (
            live_installed_inode_match
            and frozen['installed_elf_build_id'] == frozen['build_elf_build_id']
        )
        installed_inventory_sha256 = frozen['installed_embedded_source_inventory_sha256']
        live_embedded_source_inventory_sha256 = installed_inventory_sha256
        live_embedded_source_inventory_match = (
            live_installed_inode_match
            and live_embedded_source_inventory_sha256 == frozen['source_inventory_sha256']
        )
        live_mapping_has_offset_zero = any(entry.offset == 0 for entry in final_mappings)
        live_mapping_has_executable = any('x' in entry.permissions for entry in final_mappings)

        stable_identity = {
            'build_elf_build_id': frozen['build_elf_build_id'],
            'build_path': frozen['build_path'],
            'build_sha256': frozen['build_sha256'],
            'installed_declared_path': frozen['installed_declared_path'],
            'installed_device': installed_stat_after.st_dev,
            'installed_elf_build_id': frozen['installed_elf_build_id'],
            'installed_embedded_source_inventory_sha256': frozen[
                'installed_embedded_source_inventory_sha256'
            ],
            'installed_inode': installed_stat_after.st_ino,
            'installed_path': frozen['installed_path'],
            'installed_sha256': frozen['installed_sha256'],
            'launch_root_pid': launch_pid,
            'live_cmdline_sha256': hashlib.sha256(live_cmdline).hexdigest(),
            'live_executable_link': live_executable_link_after,
            'live_executable_path': str(live_executable_path),
            'live_mapping_device': installed_stat_after.st_dev,
            'live_mapping_fingerprint_sha256': live_mapping_fingerprint_sha256,
            'live_mapping_inode': installed_stat_after.st_ino,
            'live_mapping_paths': live_mapping_paths,
            'live_pgid': live_pgid,
            'live_pid': live_pid,
            'live_ppid': live_ppid,
            'live_sid': live_sid,
            'live_start_ticks': live_start_ticks,
            'observed_gz_partition': observed_gz_partition,
            'observed_ros_domain_id': observed_domain_id,
            'source_inventory_sha256': frozen['source_inventory_sha256'],
        }
        stable_identity_sha256 = _canonical_sha256(stable_identity)
        passed = all(
            (
                frozen['build_embedded_source_inventory_match'],
                frozen['installed_embedded_source_inventory_match'],
                frozen['build_install_sha256_match'],
                frozen['build_install_build_id_match'],
                frozen['build_install_embedded_source_inventory_match'],
                process_identity_match,
                installed_identity_revalidated_after_hashing,
                identity_revalidated_after_hashing,
                maps_revalidated_after_hashing,
                live_installed_inode_match,
                live_installed_sha256_match,
                live_installed_build_id_match,
                live_embedded_source_inventory_match,
                live_mapping_has_offset_zero,
                live_mapping_has_executable,
            )
        )
        return {
            **frozen,
            'attestation_method': 'proc_maps_exact_device_inode',
            'exact_live_process_count': 1,
            'identity_revalidated_after_hashing': identity_revalidated_after_hashing,
            'installed_device': installed_stat_after.st_dev,
            'installed_identity_revalidated_after_hashing': (
                installed_identity_revalidated_after_hashing
            ),
            'installed_inode': installed_stat_after.st_ino,
            'installed_size_bytes': installed_stat_after.st_size,
            'launch_root_pid': launch_pid,
            'live_cmdline_sha256': stable_identity['live_cmdline_sha256'],
            'live_elf_build_id': frozen['installed_elf_build_id'],
            'live_embedded_source_inventory_match': live_embedded_source_inventory_match,
            'live_embedded_source_inventory_sha256': (live_embedded_source_inventory_sha256),
            'live_executable_link': live_executable_link_after,
            'live_executable_path': str(live_executable_path),
            'live_installed_build_id_match': live_installed_build_id_match,
            'live_installed_inode_match': live_installed_inode_match,
            'live_installed_sha256_match': live_installed_sha256_match,
            'live_mapping_count': len(final_mappings),
            'live_mapping_device': installed_stat_after.st_dev,
            'live_mapping_fingerprint_sha256': live_mapping_fingerprint_sha256,
            'live_mapping_has_executable': live_mapping_has_executable,
            'live_mapping_has_offset_zero': live_mapping_has_offset_zero,
            'live_mapping_inode': installed_stat_after.st_ino,
            'live_mapping_paths': live_mapping_paths,
            'live_pgid': live_pgid,
            'live_pid': live_pid,
            'live_ppid': live_ppid,
            'live_sid': live_sid,
            'live_start_ticks': live_start_ticks,
            'maps_revalidated_after_hashing': maps_revalidated_after_hashing,
            'observed_gz_partition': observed_gz_partition,
            'observed_ros_domain_id': observed_domain_id,
            'process_identity_match': process_identity_match,
            'stable_identity': stable_identity,
            'stable_identity_sha256': stable_identity_sha256,
            'verdict': 'PASS' if passed else 'FAIL',
        }
    except (
        EvidenceError,
        OSError,
        subprocess.SubprocessError,
        UnicodeError,
        ValueError,
    ) as exc:
        return {
            'error': str(exc)[:4096],
            'launch_root_pid': launch_pid,
            'package': 'robotest_sim',
            'schema_version': 1,
            'verdict': 'FAIL',
        }


def _fq_node_name(info: Any) -> str:
    namespace = str(info.node_namespace).rstrip('/')
    return f'{namespace}/{info.node_name}' if namespace else f'/{info.node_name}'


def _policy_name(value: Any) -> str:
    name = getattr(value, 'name', None)
    return str(name) if name is not None else str(value).rsplit('.', 1)[-1]


def _endpoint_record(info: Any) -> dict[str, Any]:
    qos = info.qos_profile
    return {
        'depth': int(qos.depth),
        'durability': _policy_name(qos.durability),
        'history': _policy_name(qos.history),
        'gid': bytes(info.endpoint_gid).hex(),
        'node': _fq_node_name(info),
        'reliability': _policy_name(qos.reliability),
        'topic_type': str(info.topic_type),
    }


def _exact_endpoint_owners(
    endpoints: list[dict[str, Any]],
    expected_nodes: set[str],
    *,
    expected_type: str | None = None,
) -> bool:
    """Require exact endpoint cardinality, identity, type, and distinct GIDs."""
    nodes = [item['node'] for item in endpoints]
    gids = [item['gid'] for item in endpoints]
    return (
        len(endpoints) == len(expected_nodes)
        and set(nodes) == expected_nodes
        and len(set(gids)) == len(gids)
        and all(gids)
        and (
            expected_type is None or all(item['topic_type'] == expected_type for item in endpoints)
        )
    )


def _qos_status(record: dict[str, Any], expected: tuple[str, str, int]) -> dict[str, Any]:
    """Classify live QoS without turning Fast DDS UNKNOWN/0 into a false mismatch."""
    reliability, durability, depth = expected
    history = record['history']
    observed_depth = record['depth']
    introspection_complete = history not in {'UNKNOWN', 'SYSTEM_DEFAULT'} and observed_depth > 0
    explicit_keep_all = history == 'KEEP_ALL'
    positive_keep_last = history == 'KEEP_LAST' and observed_depth > 0
    exact_depth_mismatch = positive_keep_last and observed_depth != depth
    policy_contract_pass = (
        record['reliability'] == reliability
        and record['durability'] == durability
        and not explicit_keep_all
        and not exact_depth_mismatch
    )
    return {
        'bounded_depth_live_proven': positive_keep_last,
        'exact_depth_live_proven': positive_keep_last and observed_depth == depth,
        'explicit_keep_all': explicit_keep_all,
        'expected_depth': depth,
        'introspection_complete': introspection_complete,
        'policy_contract_pass': policy_contract_pass,
    }


def _qos_matches(record: dict[str, Any], expected: tuple[str, str, int]) -> bool:
    return _qos_status(record, expected)['policy_contract_pass']


def _node_snapshot(node: Any) -> list[str]:
    names = []
    for name, namespace in node.get_node_names_and_namespaces():
        prefix = namespace.rstrip('/')
        names.append(f'{prefix}/{name}' if prefix else f'/{name}')
    if len(names) > MAX_ENDPOINTS:
        raise EvidenceError('node graph exceeds the 4,096-name bound')
    return sorted(set(names))


def _service_snapshot(node: Any) -> set[str]:
    services = {name for name, _types in node.get_service_names_and_types()}
    if len(services) > MAX_ENDPOINTS:
        raise EvidenceError('service graph exceeds the 4,096-name bound')
    return services


def _topic_evidence(node: Any, topic: str) -> dict[str, Any]:
    publishers = [_endpoint_record(item) for item in node.get_publishers_info_by_topic(topic)]
    subscribers = [_endpoint_record(item) for item in node.get_subscriptions_info_by_topic(topic)]
    if len(publishers) + len(subscribers) > MAX_ENDPOINTS:
        raise EvidenceError(f'{topic} endpoint graph exceeds the bound')
    expected = QOS_CONTRACTS[topic]
    checks = [
        {
            **_qos_status(endpoint, expected),
            'node': endpoint['node'],
            'side': side,
        }
        for side, endpoints in (('publisher', publishers), ('subscriber', subscribers))
        for endpoint in endpoints
    ]
    return {
        'bounded_depth_live_proven': bool(checks)
        and all(item['bounded_depth_live_proven'] for item in checks),
        'exact_depth_live_proven': bool(checks)
        and all(item['exact_depth_live_proven'] for item in checks),
        'expected': {
            'depth': expected[2],
            'durability': expected[1],
            'history': 'KEEP_LAST',
            'reliability': expected[0],
        },
        'publishers': sorted(publishers, key=lambda item: (item['node'], item['topic_type'])),
        'publisher_qos_pass': bool(publishers)
        and all(_qos_matches(item, expected) for item in publishers),
        'qos_checks': sorted(checks, key=lambda item: (item['side'], item['node'])),
        'qos_introspection_complete': bool(checks)
        and all(item['introspection_complete'] for item in checks),
        'subscribers': sorted(subscribers, key=lambda item: (item['node'], item['topic_type'])),
        'subscriber_qos_pass': all(_qos_matches(item, expected) for item in subscribers),
    }


def _candidate_evaluation(node: Any) -> tuple[bool, dict[str, Any]]:
    nodes = _node_snapshot(node)
    short_names = {value.rsplit('/', 1)[-1] for value in nodes}
    services = _service_snapshot(node)
    topics = {name: _topic_evidence(node, name) for name in QOS_CONTRACTS}
    publisher_ownership = {
        topic: _exact_endpoint_owners(
            topics[topic]['publishers'],
            expected,
            expected_type=(
                CONTACT_MESSAGE_TYPE
                if topic in {'/robotest/internal/raw_contacts', '/robotest/validation/contacts'}
                else None
            ),
        )
        for topic, expected in CANDIDATE_EXPECTED_PUBLISHERS.items()
    }
    command_subscriber_ownership = {
        topic: _exact_endpoint_owners(topics[topic]['subscribers'], expected)
        for topic, expected in CANDIDATE_EXPECTED_COMMAND_SUBSCRIBERS.items()
    }
    contact_subscriber_ownership = {
        topic: _exact_endpoint_owners(
            topics[topic]['subscribers'], expected, expected_type=CONTACT_MESSAGE_TYPE
        )
        for topic, expected in CANDIDATE_EXPECTED_CONTACT_SUBSCRIBERS.items()
    }
    cmd_owner_pass = publisher_ownership['/robotest/cmd_vel']
    autonomy_leaks: list[dict[str, str]] = []
    for topic in VALIDATION_TOPICS:
        for subscriber in topics[topic]['subscribers']:
            short_name = subscriber['node'].rsplit('/', 1)[-1]
            if short_name in AUTONOMY_NODE_NAMES:
                autonomy_leaks.append({'node': subscriber['node'], 'topic': topic})
    project_endpoints = [
        endpoint
        for evidence in topics.values()
        for role in ('publishers', 'subscribers')
        for endpoint in evidence[role]
    ]
    endpoint_namespace_pass = all(
        endpoint['node'].startswith('/robotest/') for endpoint in project_endpoints
    )
    required_nodes_missing = sorted(CANDIDATE_REQUIRED_NODES - short_names)
    required_nodes_outside_namespace = sorted(
        value
        for value in nodes
        if value.rsplit('/', 1)[-1] in CANDIDATE_REQUIRED_NODES
        and not value.startswith('/robotest/')
    )
    namespace_pass = endpoint_namespace_pass and not required_nodes_outside_namespace
    services_missing = sorted(set(SCENARIO_SERVICES) - services)
    legacy_service_absent = '/robotest/faults/load_schedule' not in services
    qos_pass = all(
        evidence['publisher_qos_pass'] and evidence['subscriber_qos_pass']
        for evidence in topics.values()
    )
    qos_introspection_complete = all(
        evidence['qos_introspection_complete'] for evidence in topics.values()
    )
    bounded_depth_live_proven = all(
        evidence['bounded_depth_live_proven'] for evidence in topics.values()
    )
    passed = (
        not required_nodes_missing
        and not required_nodes_outside_namespace
        and not services_missing
        and legacy_service_absent
        and cmd_owner_pass
        and all(publisher_ownership.values())
        and all(command_subscriber_ownership.values())
        and all(contact_subscriber_ownership.values())
        and not autonomy_leaks
        and namespace_pass
        and qos_pass
    )
    return passed, {
        'autonomy_validation_leaks': autonomy_leaks,
        'bounded_depth_live_proven_for_all_endpoints': bounded_depth_live_proven,
        'cmd_vel_owner_pass': cmd_owner_pass,
        'command_subscriber_ownership': command_subscriber_ownership,
        'contact_subscriber_ownership': contact_subscriber_ownership,
        'exact_static_qos_depth_contract': {
            topic: evidence['expected'] for topic, evidence in topics.items()
        },
        'legacy_fault_service_absent': legacy_service_absent,
        'mode': 'candidate',
        'namespace_isolation_pass': namespace_pass,
        'nodes': nodes,
        'publisher_ownership': publisher_ownership,
        'qos_contract_pass': qos_pass,
        'qos_introspection_complete': qos_introspection_complete,
        'required_nodes_missing': required_nodes_missing,
        'required_nodes_outside_namespace': required_nodes_outside_namespace,
        'scenario_services_missing': services_missing,
        'topics': topics,
        'validation_autonomy_isolation_pass': not autonomy_leaks,
    }


def _positive_evaluation(node: Any) -> tuple[bool, dict[str, Any]]:
    nodes = _node_snapshot(node)
    short_names = {value.rsplit('/', 1)[-1] for value in nodes}
    services = _service_snapshot(node)
    relevant = {
        name: _topic_evidence(node, name)
        for name in (
            '/clock',
            '/robotest/cmd_vel',
            '/robotest/internal/raw_contacts',
            '/robotest/validation/contacts',
            '/robotest/validation/ground_truth',
            '/robotest/validation/scenario_entity_poses',
            '/robotest/validation/world_stats',
        )
    }
    cmd_publishers = relevant['/robotest/cmd_vel']['publishers']
    cmd_owner_pass = len(cmd_publishers) == 1 and cmd_publishers[0]['node'].endswith(
        '/contact_control_driver'
    )
    contact_publisher_ownership = {
        topic: _exact_endpoint_owners(
            relevant[topic]['publishers'],
            CANDIDATE_EXPECTED_PUBLISHERS[topic],
            expected_type=CONTACT_MESSAGE_TYPE,
        )
        for topic in ('/robotest/internal/raw_contacts', '/robotest/validation/contacts')
    }
    contact_subscriber_ownership = {
        '/robotest/internal/raw_contacts': _exact_endpoint_owners(
            relevant['/robotest/internal/raw_contacts']['subscribers'],
            {'/robotest/contact_stream_gate'},
            expected_type=CONTACT_MESSAGE_TYPE,
        )
    }
    required_nodes = {
        'contact_control_driver',
        'contact_stream_gate',
        'fault_proxy',
        'metrics_collector',
        'parameter_bridge',
        'robot_state_publisher',
        'scenario_bridge',
    }
    required_nodes_missing = sorted(required_nodes - short_names)
    forbidden_present = sorted(POSITIVE_FORBIDDEN_NODES & short_names)
    services_missing = sorted(
        {
            '/robotest/scenario/delete_entity',
            '/robotest/scenario/spawn_entity',
        }
        - services
    )
    namespace_pass = all(
        endpoint['node'].startswith('/robotest/')
        for evidence in relevant.values()
        for role in ('publishers', 'subscribers')
        for endpoint in evidence[role]
    )
    qos_pass = all(
        evidence['publisher_qos_pass'] and evidence['subscriber_qos_pass']
        for evidence in relevant.values()
    )
    qos_introspection_complete = all(
        evidence['qos_introspection_complete'] for evidence in relevant.values()
    )
    bounded_depth_live_proven = all(
        evidence['bounded_depth_live_proven'] for evidence in relevant.values()
    )
    passed = (
        cmd_owner_pass
        and all(contact_publisher_ownership.values())
        and all(contact_subscriber_ownership.values())
        and not forbidden_present
        and not required_nodes_missing
        and not services_missing
        and namespace_pass
        and qos_pass
    )
    return passed, {
        'bounded_depth_live_proven_for_all_endpoints': bounded_depth_live_proven,
        'cmd_vel_owner_pass': cmd_owner_pass,
        'contact_publisher_ownership': contact_publisher_ownership,
        'contact_subscriber_ownership': contact_subscriber_ownership,
        'exact_static_qos_depth_contract': {
            topic: evidence['expected'] for topic, evidence in relevant.items()
        },
        'forbidden_nodes_present': forbidden_present,
        'mode': 'positive_control',
        'namespace_isolation_pass': namespace_pass,
        'nodes': nodes,
        'qos_contract_pass': qos_pass,
        'qos_introspection_complete': qos_introspection_complete,
        'required_nodes_missing': required_nodes_missing,
        'scenario_services_missing': services_missing,
        'topics': relevant,
        'validation_autonomy_isolation_pass': not forbidden_present,
    }


def _contact_stream_evaluation(node: Any) -> tuple[bool, dict[str, Any]]:
    nodes = _node_snapshot(node)
    topics = {
        name: _topic_evidence(node, name)
        for name in ('/robotest/internal/raw_contacts', '/robotest/validation/contacts')
    }
    publisher_ownership = {
        topic: _exact_endpoint_owners(
            topics[topic]['publishers'],
            CANDIDATE_EXPECTED_PUBLISHERS[topic],
            expected_type=CONTACT_MESSAGE_TYPE,
        )
        for topic in topics
    }
    raw_subscriber_ownership = _exact_endpoint_owners(
        topics['/robotest/internal/raw_contacts']['subscribers'],
        {'/robotest/contact_stream_gate'},
        expected_type=CONTACT_MESSAGE_TYPE,
    )
    qos_pass = all(
        evidence['publisher_qos_pass'] and evidence['subscriber_qos_pass']
        for evidence in topics.values()
    )
    gate_present = '/robotest/contact_stream_gate' in nodes
    checks = (
        gate_present,
        all(publisher_ownership.values()),
        raw_subscriber_ownership,
        qos_pass,
    )
    passed = all(checks)
    return passed, {
        'contact_stream_gate_present': gate_present,
        'mode': 'contact_stream',
        'nodes': nodes,
        'publisher_ownership': publisher_ownership,
        'qos_contract_pass': qos_pass,
        'raw_subscriber_ownership': raw_subscriber_ownership,
        'topics': topics,
    }


def _empty_evaluation(node: Any) -> tuple[bool, dict[str, Any]]:
    nodes = _node_snapshot(node)
    self_name = node.get_fully_qualified_name()
    remaining = [value for value in nodes if value != self_name]
    return not remaining, {
        'mode': 'empty',
        'nodes': nodes,
        'remaining_nodes': remaining,
    }


def _run(arguments: argparse.Namespace) -> int:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    if arguments.output.exists() or arguments.output.is_symlink():
        raise EvidenceError(f'runtime gate output already exists: {arguments.output}')
    if not 0.0 < arguments.wall_timeout_s <= 120.0:
        raise EvidenceError('wall timeout must be in (0, 120] seconds')
    if arguments.mode != 'empty' and (
        arguments.workspace is None
        or not _pid_alive(arguments.launch_pid)
        or arguments.expected_domain_id is None
        or arguments.expected_gz_partition is None
    ):
        raise EvidenceError(
            'live runtime gates require workspace, launch PID, domain, and partition'
        )
    rclpy.init(args=list(arguments.ros_args))
    node = Node('phase3_runtime_gate', namespace='/robotest/evidence')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    started_ns = time.monotonic_ns()
    deadline_ns = started_ns + int(arguments.wall_timeout_s * 1_000_000_000)
    attempt = 0
    last: dict[str, Any] = {}
    try:
        while rclpy.ok() and time.monotonic_ns() < deadline_ns:
            if not _pid_alive(arguments.watch_pid):
                raise EvidenceError('watched process exited before runtime gate converged')
            attempt += 1
            if attempt > MAX_ATTEMPTS:
                raise EvidenceError('runtime gate exceeded 4,096 graph attempts')
            executor.spin_once(timeout_sec=0.05)
            if arguments.mode == 'candidate':
                passed, last = _candidate_evaluation(node)
            elif arguments.mode == 'positive-control':
                passed, last = _positive_evaluation(node)
            elif arguments.mode == 'contact-stream':
                passed, last = _contact_stream_evaluation(node)
            else:
                passed, last = _empty_evaluation(node)
            if passed and arguments.mode != 'empty':
                gate_attestation = _contact_gate_binary_attestation(
                    arguments.workspace.resolve(),
                    arguments.launch_pid,
                    arguments.expected_domain_id,
                    arguments.expected_gz_partition,
                )
                aggregator_attestation = _contact_aggregator_binary_attestation(
                    arguments.workspace.resolve(),
                    arguments.launch_pid,
                    arguments.expected_domain_id,
                    arguments.expected_gz_partition,
                )
                last['contact_gate_binary_attestation'] = gate_attestation
                last['contact_aggregator_binary_attestation'] = aggregator_attestation
                passed = (
                    gate_attestation.get('verdict') == 'PASS'
                    and aggregator_attestation.get('verdict') == 'PASS'
                )
            if passed:
                result = {
                    **last,
                    'attempt_count': attempt,
                    'elapsed_wall_s': (time.monotonic_ns() - started_ns) / 1_000_000_000,
                    'producer': 'robotest_phase3/runtime_gate',
                    'schema_version': 1,
                    'verdict': 'PASS',
                }
                atomic_write_json(arguments.output, result, sidecar=True)
                return 0
        result = {
            **last,
            'attempt_count': attempt,
            'elapsed_wall_s': (time.monotonic_ns() - started_ns) / 1_000_000_000,
            'producer': 'robotest_phase3/runtime_gate',
            'schema_version': 1,
            'verdict': 'FAIL',
        }
        atomic_write_json(arguments.output, result, sidecar=True)
        return 1
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _self_test() -> int:
    class Policy:
        def __init__(self, name: str) -> None:
            self.name = name

    class Qos:
        history = Policy('KEEP_LAST')
        reliability = Policy('RELIABLE')
        durability = Policy('VOLATILE')
        depth = 10

    class Endpoint:
        endpoint_gid = bytes.fromhex('01' * 16)
        node_namespace = '/robotest'
        node_name = 'metrics_collector'
        topic_type = 'std_msgs/msg/String'
        qos_profile = Qos()

    record = _endpoint_record(Endpoint())
    assert record['node'] == '/robotest/metrics_collector'
    assert _qos_matches(record, ('RELIABLE', 'VOLATILE', 10))
    assert not _qos_matches(record, ('BEST_EFFORT', 'VOLATILE', 10))
    unknown = {**record, 'depth': 0, 'history': 'UNKNOWN'}
    unknown_status = _qos_status(unknown, ('RELIABLE', 'VOLATILE', 10))
    assert unknown_status['policy_contract_pass']
    assert not unknown_status['introspection_complete']
    assert not unknown_status['bounded_depth_live_proven']
    keep_all = {**record, 'depth': 0, 'history': 'KEEP_ALL'}
    assert not _qos_status(keep_all, ('RELIABLE', 'VOLATILE', 10))['policy_contract_pass']
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--mode', choices=('candidate', 'positive-control', 'contact-stream', 'empty')
    )
    parser.add_argument('--output', type=Path)
    parser.add_argument('--launch-pid', type=int)
    parser.add_argument('--workspace', type=Path)
    parser.add_argument('--expected-domain-id', type=int)
    parser.add_argument('--expected-gz-partition')
    parser.add_argument('--watch-pid', type=int)
    parser.add_argument('--wall-timeout-s', type=float, default=90.0)
    parser.add_argument('--self-test', action='store_true')
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the runtime gate or its pure self-test."""
    arguments, ros_args = _parser().parse_known_args(argv)
    if arguments.self_test:
        return _self_test()
    if arguments.mode is None or arguments.output is None:
        print('phase3 runtime gate requires --mode and --output', file=os.sys.stderr)
        return 2
    arguments.ros_args = ros_args
    try:
        return _run(arguments)
    except (EvidenceError, RuntimeError) as exc:
        print(f'phase3 runtime gate error: {exc}', file=os.sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())

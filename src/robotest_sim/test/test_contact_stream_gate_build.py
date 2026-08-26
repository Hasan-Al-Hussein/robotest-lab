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

"""Build-time provenance regressions for the compiled contact pipeline."""

from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import subprocess
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
TAG = b'ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256='
# This regression performs two clean ROS C++ executable builds. Keep the bounds
# generous enough for cold CI hosts while retaining deterministic termination.
CONFIGURE_TIMEOUT_SECONDS = 180
BUILD_TIMEOUT_SECONDS = 360
WINDOWS_DRIVE_MOUNT = re.compile(r'^/mnt/[A-Za-z](?:/|$)')
SOURCE_PATHS = (
    (
        'src/robotest_description/urdf/robotest_gazebo.xacro',
        '../robotest_description/urdf/robotest_gazebo.xacro',
    ),
    ('src/robotest_sim/CMakeLists.txt', 'CMakeLists.txt'),
    ('src/robotest_sim/config/bridge.yaml', 'config/bridge.yaml'),
    (
        'src/robotest_sim/include/robotest_sim/contact_aggregator.hpp',
        'include/robotest_sim/contact_aggregator.hpp',
    ),
    (
        'src/robotest_sim/include/robotest_sim/contact_stream_gate.hpp',
        'include/robotest_sim/contact_stream_gate.hpp',
    ),
    ('src/robotest_sim/launch/sim.launch.py', 'launch/sim.launch.py'),
    ('src/robotest_sim/src/contact_aggregator.cpp', 'src/contact_aggregator.cpp'),
    (
        'src/robotest_sim/src/contact_aggregator_system.cpp',
        'src/contact_aggregator_system.cpp',
    ),
    ('src/robotest_sim/src/contact_stream_gate.cpp', 'src/contact_stream_gate.cpp'),
    ('src/robotest_sim/src/contact_stream_gate_node.cpp', 'src/contact_stream_gate_node.cpp'),
    ('src/robotest_sim/worlds/robotest_lab.sdf', 'worlds/robotest_lab.sdf'),
)


def _linux_tool_environment() -> dict[str, str]:
    """Exclude inherited Windows tools from the isolated WSL build proof."""

    environment = os.environ.copy()
    path_entries = [
        entry
        for entry in environment.get('PATH', '').split(os.pathsep)
        if entry and WINDOWS_DRIVE_MOUNT.match(entry) is None
    ]
    if not path_entries:
        raise RuntimeError('isolated build proof has no Linux tool path')
    environment['PATH'] = os.pathsep.join(path_entries)
    return environment


def _inventory_sha256(source_root: Path) -> str:
    inventory = {
        'schema_version': 1,
        'sources': [
            {
                'path': repository_path,
                'sha256': hashlib.sha256((source_root / package_path).read_bytes()).hexdigest(),
            }
            for repository_path, package_path in SOURCE_PATHS
        ],
    }
    canonical = (
        json.dumps(
            inventory,
            allow_nan=False,
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True,
        )
        + '\n'
    ).encode('utf-8')
    return hashlib.sha256(canonical).hexdigest()


def _embedded_sha256(executable: Path) -> str:
    matches = set(re.findall(re.escape(TAG) + rb'([0-9a-f]{64})', executable.read_bytes()))
    assert len(matches) == 1
    return matches.pop().decode('ascii')


def _build(source_root: Path, build_root: Path, environment: dict[str, str]) -> None:
    subprocess.run(
        [
            'cmake',
            '--build',
            str(build_root),
            '--target',
            'contact_stream_gate',
            'robotest_contact_aggregator_system',
            '-j2',
        ],
        check=True,
        cwd=source_root,
        env=environment,
        timeout=BUILD_TIMEOUT_SECONDS,
    )


def test_linux_tool_environment_excludes_wsl_windows_drive_mounts(monkeypatch) -> None:
    monkeypatch.setenv(
        'PATH',
        '/usr/local/bin:/mnt/c/WINDOWS/system32:/usr/bin:/mnt/d/custom/bin',
    )
    assert _linux_tool_environment()['PATH'] == '/usr/local/bin:/usr/bin'


def test_incremental_build_reconfigures_and_rebinds_pipeline_inventory(tmp_path: Path) -> None:
    source_root = tmp_path / 'robotest_sim'
    build_root = tmp_path / 'build'
    environment = _linux_tool_environment()
    shutil.copytree(PACKAGE_ROOT, source_root)
    shutil.copytree(PACKAGE_ROOT.parent / 'robotest_description', tmp_path / 'robotest_description')
    subprocess.run(
        [
            'cmake',
            '-S',
            str(source_root),
            '-B',
            str(build_root),
            '-DBUILD_TESTING=OFF',
        ],
        check=True,
        cwd=source_root,
        env=environment,
        timeout=CONFIGURE_TIMEOUT_SECONDS,
    )
    _build(source_root, build_root, environment)
    executable = build_root / 'contact_stream_gate'
    plugin = build_root / 'librobotest_contact_aggregator_system.so'
    initial = _embedded_sha256(executable)
    initial_plugin = _embedded_sha256(plugin)
    assert initial == _inventory_sha256(source_root)
    assert initial_plugin == initial

    policy_source = source_root / 'src' / 'contact_stream_gate.cpp'
    policy_source.write_bytes(
        policy_source.read_bytes() + b'\n// Configure-dependency regression probe.\n'
    )
    _build(source_root, build_root, environment)
    updated = _embedded_sha256(executable)
    updated_plugin = _embedded_sha256(plugin)
    assert updated == _inventory_sha256(source_root)
    assert updated_plugin == updated
    assert updated != initial

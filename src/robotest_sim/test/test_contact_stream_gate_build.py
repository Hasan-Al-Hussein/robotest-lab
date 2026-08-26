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

"""Build-time provenance regressions for the compiled contact-stream gate."""

from __future__ import annotations

import hashlib
import json
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
SOURCE_PATHS = (
    ('src/robotest_sim/CMakeLists.txt', 'CMakeLists.txt'),
    (
        'src/robotest_sim/include/robotest_sim/contact_stream_gate.hpp',
        'include/robotest_sim/contact_stream_gate.hpp',
    ),
    ('src/robotest_sim/src/contact_stream_gate.cpp', 'src/contact_stream_gate.cpp'),
    ('src/robotest_sim/src/contact_stream_gate_node.cpp', 'src/contact_stream_gate_node.cpp'),
)


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


def _build(source_root: Path, build_root: Path) -> None:
    subprocess.run(
        ['cmake', '--build', str(build_root), '--target', 'contact_stream_gate', '-j2'],
        check=True,
        cwd=source_root,
        timeout=BUILD_TIMEOUT_SECONDS,
    )


def test_incremental_build_reconfigures_and_rebinds_source_inventory(tmp_path: Path) -> None:
    source_root = tmp_path / 'robotest_sim'
    build_root = tmp_path / 'build'
    shutil.copytree(PACKAGE_ROOT, source_root)
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
        timeout=CONFIGURE_TIMEOUT_SECONDS,
    )
    _build(source_root, build_root)
    executable = build_root / 'contact_stream_gate'
    initial = _embedded_sha256(executable)
    assert initial == _inventory_sha256(source_root)

    policy_source = source_root / 'src' / 'contact_stream_gate.cpp'
    policy_source.write_bytes(
        policy_source.read_bytes() + b'\n// Configure-dependency regression probe.\n'
    )
    _build(source_root, build_root)
    updated = _embedded_sha256(executable)
    assert updated == _inventory_sha256(source_root)
    assert updated != initial

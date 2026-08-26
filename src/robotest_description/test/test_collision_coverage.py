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

"""Fail-closed tests for generated collision coverage and provenance."""

# ruff: noqa: I001

from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
import re
import sys
from types import ModuleType
from typing import Any
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
GENERATOR_PATH = PACKAGE_ROOT / 'tools' / 'generate_collision_coverage.py'
MANIFEST_PATH = REPOSITORY_ROOT / 'config' / 'collision-coverage.yaml'
SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')
HASH_FIELDS = {
    'bridge_sha256',
    'contact_configuration_sha256',
    'manifest_sha256',
    'rendered_sdf_sha256',
    'robot_description_sha256',
    'world_source_sha256',
}
ROOT_FIELDS = {
    'schema_version',
    'robot_model',
    'contact_topic',
    'robot_collisions',
    'covered_collisions',
    'rendered_robot_collisions',
    'support_pairs',
    *HASH_FIELDS,
}


def _load_generator() -> ModuleType:
    spec = importlib.util.spec_from_file_location('generate_collision_coverage', GENERATOR_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def generator() -> ModuleType:
    return _load_generator()


@pytest.fixture(scope='module')
def rendered_sdf(generator: ModuleType) -> bytes:
    return generator.render_robot_sdf(REPOSITORY_ROOT)


@pytest.fixture(scope='module')
def generated_manifest(generator: ModuleType) -> dict[str, Any]:
    return generator.build_manifest(REPOSITORY_ROOT)


def test_committed_manifest_exactly_matches_current_sources(
    generator: ModuleType,
    generated_manifest: dict[str, Any],
) -> None:
    payload = MANIFEST_PATH.read_bytes()
    committed = yaml.safe_load(payload)

    assert committed == generated_manifest
    assert payload == generator.manifest_yaml_bytes(generated_manifest)
    assert set(committed) == ROOT_FIELDS
    assert committed['schema_version'] == 2
    assert all(SHA256_PATTERN.fullmatch(committed[field]) for field in HASH_FIELDS)
    assert b'&id' not in payload and b'*id' not in payload

    body = dict(committed)
    declared_hash = body.pop('manifest_sha256')
    assert declared_hash == generator.canonical_sha256(body)


def test_manifest_hashes_real_rendered_and_source_evidence(
    generator: ModuleType,
    generated_manifest: dict[str, Any],
    rendered_sdf: bytes,
) -> None:
    geometries, contact_hash = generator.extract_rendered_coverage(rendered_sdf)
    names = [geometry['name'] for geometry in geometries]

    assert generated_manifest['robot_collisions'] == geometries
    assert generated_manifest['covered_collisions'] == names
    assert generated_manifest['rendered_robot_collisions'] == names
    assert generated_manifest['contact_configuration_sha256'] == contact_hash
    assert generated_manifest['rendered_sdf_sha256'] == hashlib.sha256(rendered_sdf).hexdigest()
    assert generated_manifest['bridge_sha256'] == generator.bridge_sha256(REPOSITORY_ROOT)
    assert generated_manifest['robot_description_sha256'] == (
        generator.robot_description_sha256(REPOSITORY_ROOT)
    )
    assert generated_manifest['world_source_sha256'] == (
        generator.world_source_sha256(REPOSITORY_ROOT)
    )


def _mutate_rendered_sdf(rendered_sdf: bytes, mutation: str) -> bytes:
    root = ET.fromstring(rendered_sdf)
    chassis = root.find("./model/link[@name='base_footprint']")
    assert chassis is not None
    sensor = chassis.find("./sensor[@name='chassis_contact_sensor']")
    collision = chassis.find('collision')
    assert sensor is not None and collision is not None
    if mutation == 'topic':
        topic = sensor.find('contact/topic')
        assert topic is not None
        topic.text = '/wrong/contacts'
    elif mutation == 'sensor':
        sensor.set('name', 'wrong_contact_sensor')
    elif mutation == 'always_on':
        always_on = sensor.find('always_on')
        assert always_on is not None
        always_on.text = 'false'
    elif mutation == 'missing_collision':
        chassis.remove(collision)
    else:
        raise AssertionError(f'unknown mutation {mutation}')
    return ET.tostring(root, encoding='utf-8')


@pytest.mark.parametrize('mutation', ['topic', 'sensor', 'always_on', 'missing_collision'])
def test_rendered_coverage_rejects_missing_or_misbound_evidence(
    generator: ModuleType,
    rendered_sdf: bytes,
    mutation: str,
) -> None:
    with pytest.raises(generator.CoverageGenerationError):
        generator.extract_rendered_coverage(_mutate_rendered_sdf(rendered_sdf, mutation))


def test_bridge_validation_rejects_contact_mapping_drift(
    generator: ModuleType,
    tmp_path: Path,
) -> None:
    source = REPOSITORY_ROOT / 'src' / 'robotest_sim' / 'config' / 'bridge.yaml'
    document = yaml.safe_load(source.read_text(encoding='utf-8'))
    contact = next(
        entry for entry in document if entry.get('ros_topic_name') == 'validation/contacts'
    )
    contact['publisher_queue'] = 9
    destination = tmp_path / 'src' / 'robotest_sim' / 'config' / 'bridge.yaml'
    destination.parent.mkdir(parents=True)
    destination.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(generator.CoverageGenerationError, match='contact bridge'):
        generator.bridge_sha256(tmp_path)


def test_world_validation_rejects_missing_contact_system(
    generator: ModuleType,
    tmp_path: Path,
) -> None:
    source = REPOSITORY_ROOT / 'src' / 'robotest_sim' / 'worlds' / 'robotest_lab.sdf'
    text = source.read_text(encoding='utf-8').replace(
        'gz-sim-contact-system',
        'gz-sim-wrong-contact-system',
        1,
    )
    destination = tmp_path / 'src' / 'robotest_sim' / 'worlds' / 'robotest_lab.sdf'
    destination.parent.mkdir(parents=True)
    destination.write_text(text, encoding='utf-8')

    with pytest.raises(generator.CoverageGenerationError, match='Contact system'):
        generator.world_source_sha256(tmp_path)


def test_check_mode_fails_closed_for_stale_manifest(
    generator: ModuleType,
    tmp_path: Path,
) -> None:
    assert generator.main(['--repository-root', str(REPOSITORY_ROOT), '--mode', 'check']) == 0
    stale = tmp_path / 'collision-coverage.yaml'
    stale.write_text('schema_version: 2\n', encoding='utf-8')
    assert (
        generator.main(
            [
                '--repository-root',
                str(REPOSITORY_ROOT),
                '--output',
                str(stale),
                '--mode',
                'check',
            ]
        )
        == 2
    )

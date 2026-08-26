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

"""Non-circular manifest creation and verification for one result bundle."""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from robotest_metrics.artifacts import (
    canonical_json_bytes,
    file_sha256,
    require_directory_within_cap,
    validate_csv_projection,
    write_bytes_atomic,
    write_json_atomic,
)
from robotest_metrics.constants import (
    LOG_MAX_BYTES,
    PER_RUN_CSV_MAX_BYTES,
    PER_RUN_DIRECTORY_MAX_BYTES,
    PER_RUN_JSON_MAX_BYTES,
    PNG_MAX_BYTES,
    PNG_MAX_COUNT,
)
from robotest_metrics.errors import ArtifactError
from robotest_metrics.schema_validation import validate_document

MANIFEST_NAME = 'run-artifacts.manifest.json'
MANIFEST_SIDECAR_NAME = f'{MANIFEST_NAME}.sha256'
_BASE_ARTIFACTS = {'report.html', 'report.md', 'run-result.csv', 'run-result.json'}
_CHART_ARTIFACTS = {
    'localization-error.png',
    'measurement-summary.png',
    'real-time-factor.png',
    'trajectory.png',
}
_OUTPUT_ARTIFACTS = _BASE_ARTIFACTS | _CHART_ARTIFACTS


def _regular_files(output: Path) -> dict[str, Path]:
    files: dict[str, Path] = {}
    try:
        entries = list(output.iterdir())
    except OSError as exc:
        raise ArtifactError(f'cannot inspect result bundle: {exc}') from exc
    for entry in entries:
        if entry.is_symlink() or not entry.is_file():
            raise ArtifactError(f'result bundle entry is not a regular file: {entry.name}')
        files[entry.name] = entry
    return files


def _artifact_cap(name: str) -> int:
    if name == 'run-result.json':
        return PER_RUN_JSON_MAX_BYTES
    if name == 'run-result.csv':
        return PER_RUN_CSV_MAX_BYTES
    if name in {'report.md', 'report.html'}:
        return LOG_MAX_BYTES
    if name in _CHART_ARTIFACTS:
        return PNG_MAX_BYTES
    raise ArtifactError(f'output manifest contains an unsupported artifact: {name}')


def _artifact_records(files: Mapping[str, Path]) -> list[dict[str, Any]]:
    names = set(files)
    if not names >= _BASE_ARTIFACTS or not names <= _OUTPUT_ARTIFACTS:
        missing = sorted(_BASE_ARTIFACTS - names)
        unexpected = sorted(names - _OUTPUT_ARTIFACTS)
        raise ArtifactError(
            f'result artifact set is incomplete or unexpected; missing={missing}, '
            f'unexpected={unexpected}'
        )
    chart_count = len(names & _CHART_ARTIFACTS)
    if chart_count > PNG_MAX_COUNT:
        raise ArtifactError(f'chart count {chart_count} exceeds {PNG_MAX_COUNT}')
    records: list[dict[str, Any]] = []
    for name in sorted(names):
        path = files[name]
        try:
            byte_count = path.stat().st_size
            digest = file_sha256(path)
        except OSError as exc:
            raise ArtifactError(f'cannot inspect output artifact {name}: {exc}') from exc
        if byte_count <= 0 or byte_count > _artifact_cap(name):
            raise ArtifactError(f'output artifact {name} violates its byte cap')
        records.append({'bytes': byte_count, 'path': name, 'sha256': digest})
    return records


def write_result_manifest(
    output_directory: str | Path,
    result: Mapping[str, Any],
) -> dict[str, Any]:
    """Write and then independently verify the manifest and detached hash."""
    output = Path(output_directory)
    files = _regular_files(output)
    forbidden = {MANIFEST_NAME, MANIFEST_SIDECAR_NAME} & set(files)
    if forbidden:
        raise ArtifactError(f'result bundle already contains manifest files: {sorted(forbidden)}')
    records = _artifact_records(files)
    run_record = next(record for record in records if record['path'] == 'run-result.json')
    identity = result.get('identity')
    if not isinstance(identity, Mapping):
        raise ArtifactError('run result identity is unavailable for output manifest')
    run_id = identity.get('run_id')
    if not isinstance(run_id, str) or not run_id:
        raise ArtifactError('run result has no bounded run ID for output manifest')
    manifest = {
        'artifacts': records,
        'identity': {
            'run_id': run_id,
            'run_result_sha256': run_record['sha256'],
        },
        'producer': 'robotest_metrics/metrics_analyze',
        'quality': {
            'artifact_bytes_excluding_manifest': sum(record['bytes'] for record in records),
            'artifact_count': len(records),
            'caps_within_limits': True,
            'hashes_verified': True,
            'path_set_complete': True,
        },
        'schema_version': 1,
    }
    validate_document(manifest, 'run-artifacts-manifest.schema.json')
    record = write_json_atomic(
        manifest,
        output / MANIFEST_NAME,
        maximum_bytes=PER_RUN_JSON_MAX_BYTES,
    )
    sidecar = f'{record["sha256"]}  {MANIFEST_NAME}\n'.encode('ascii')
    write_bytes_atomic(
        sidecar,
        output / MANIFEST_SIDECAR_NAME,
        maximum_bytes=256,
    )
    verified = verify_result_bundle(output)
    if verified != manifest:
        raise ArtifactError('verified output manifest differs from the authored manifest')
    return manifest


def verify_result_bundle(output_directory: str | Path) -> dict[str, Any]:
    """Verify canonical manifest bytes, detached hash, artifact hashes, and caps."""
    output = Path(output_directory)
    files = _regular_files(output)
    if MANIFEST_NAME not in files or MANIFEST_SIDECAR_NAME not in files:
        raise ArtifactError('result bundle manifest or detached hash is missing')
    manifest_path = files.pop(MANIFEST_NAME)
    sidecar_path = files.pop(MANIFEST_SIDECAR_NAME)
    try:
        payload = manifest_path.read_bytes()
        manifest = json.loads(payload)
        sidecar = sidecar_path.read_bytes()
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ArtifactError(f'cannot read result bundle manifest: {exc}') from exc
    if not isinstance(manifest, dict) or payload != canonical_json_bytes(manifest):
        raise ArtifactError('result bundle manifest is not canonical JSON')
    validate_document(manifest, 'run-artifacts-manifest.schema.json')
    manifest_hash = hashlib.sha256(payload).hexdigest()
    expected_sidecar = f'{manifest_hash}  {MANIFEST_NAME}\n'.encode('ascii')
    if sidecar != expected_sidecar:
        raise ArtifactError('result bundle manifest detached hash does not match')
    records = manifest['artifacts']
    if not isinstance(records, list):
        raise ArtifactError('result bundle manifest artifacts are invalid')
    declared_names = [record.get('path') for record in records]
    if len(declared_names) != len(set(declared_names)) or set(declared_names) != set(files):
        raise ArtifactError('result bundle manifest path set does not match disk')
    actual_records = _artifact_records(files)
    if actual_records != records:
        raise ArtifactError('result bundle artifact bytes or hashes do not match manifest')
    run_record = next(record for record in records if record['path'] == 'run-result.json')
    if manifest['identity']['run_result_sha256'] != run_record['sha256']:
        raise ArtifactError('manifest run-result identity hash does not reconcile')
    try:
        result_payload = files['run-result.json'].read_bytes()
        result = json.loads(result_payload)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ArtifactError(f'cannot read canonical run result from bundle: {exc}') from exc
    if not isinstance(result, dict) or result_payload != canonical_json_bytes(result):
        raise ArtifactError('bundled run result is not canonical JSON')
    validate_document(result, 'run-result.schema.json')
    identity = result.get('identity')
    if not isinstance(identity, Mapping) or (
        identity.get('run_id') != manifest['identity']['run_id']
    ):
        raise ArtifactError('manifest and run-result run IDs differ')
    validate_csv_projection(result, files['run-result.csv'])
    quality = manifest['quality']
    if quality['artifact_count'] != len(records) or quality[
        'artifact_bytes_excluding_manifest'
    ] != sum(record['bytes'] for record in records):
        raise ArtifactError('result bundle manifest counters do not reconcile')
    require_directory_within_cap(output, maximum_bytes=PER_RUN_DIRECTORY_MAX_BYTES)
    return manifest

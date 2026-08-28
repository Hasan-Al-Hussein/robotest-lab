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

"""Canonical, bounded and atomic Phase 3 artifact writers."""

from __future__ import annotations

import csv
import hashlib
import io
import json
import os
import tempfile
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from robotest_metrics.constants import (
    PER_RUN_CSV_MAX_BYTES,
    PER_RUN_DIRECTORY_MAX_BYTES,
    PER_RUN_JSON_MAX_BYTES,
)
from robotest_metrics.errors import ArtifactError

CSV_PROJECTION_CONTRACT = 'bounded_scalar_summary_v1'
CSV_PROJECTION_CONTRACT_COLUMN = '__robotest_projection_contract'
CSV_PROJECTION_JSON_SHA256_COLUMN = '__robotest_canonical_json_sha256'


def canonical_json_bytes(document: Any, *, trailing_newline: bool = True) -> bytes:
    """Serialize strict JSON with stable ordering and no non-finite extension values."""
    try:
        serialized = json.dumps(
            document,
            allow_nan=False,
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True,
        )
    except (TypeError, ValueError) as exc:
        raise ArtifactError(f'document is not canonical-JSON serializable: {exc}') from exc
    if trailing_newline:
        serialized += '\n'
    return serialized.encode('utf-8')


def canonical_sha256(document: Any) -> str:
    """Hash the exact canonical bytes used by the JSON artifact."""
    return hashlib.sha256(canonical_json_bytes(document)).hexdigest()


def file_sha256(path: str | Path) -> str:
    """Hash a file without reading it into an unbounded byte string."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        while block := source.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _atomic_write_bytes(path: Path, payload: bytes, maximum_bytes: int) -> int:
    if maximum_bytes <= 0:
        raise ValueError('maximum_bytes must be positive')
    if len(payload) > maximum_bytes:
        raise ArtifactError(
            f'{path.name} is {len(payload)} bytes and exceeds its {maximum_bytes}-byte cap'
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, 'wb') as target:
            target.write(payload)
            target.flush()
            os.fsync(target.fileno())
        os.replace(temporary_path, path)
        try:
            parent_descriptor = os.open(path.parent, os.O_RDONLY)
        except OSError:
            parent_descriptor = None
        if parent_descriptor is not None:
            try:
                os.fsync(parent_descriptor)
            finally:
                os.close(parent_descriptor)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise
    return len(payload)


def write_bytes_atomic(
    payload: bytes,
    path: str | Path,
    *,
    maximum_bytes: int,
) -> dict[str, Any]:
    """Atomically write already-bounded bytes and return their provenance."""
    destination = Path(path)
    byte_count = _atomic_write_bytes(destination, payload, maximum_bytes)
    return {
        'bytes': byte_count,
        'path': str(destination),
        'sha256': hashlib.sha256(payload).hexdigest(),
    }


def write_json_atomic(
    document: Any,
    path: str | Path,
    *,
    maximum_bytes: int = PER_RUN_JSON_MAX_BYTES,
) -> dict[str, Any]:
    """Write one canonical JSON artifact atomically and return its provenance."""
    payload = canonical_json_bytes(document)
    destination = Path(path)
    byte_count = _atomic_write_bytes(destination, payload, maximum_bytes)
    return {
        'bytes': byte_count,
        'path': str(destination),
        'sha256': hashlib.sha256(payload).hexdigest(),
    }


def _csv_scalar(value: Any) -> str:
    if value is None:
        return ''
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (int, float)):
        return canonical_json_bytes(value, trailing_newline=False).decode('utf-8')
    if isinstance(value, str):
        return value
    return canonical_json_bytes(value, trailing_newline=False).decode('utf-8')


def _csv_sequence_descriptor(value: list[Any] | tuple[Any, ...]) -> str:
    return _csv_scalar(
        {
            'element_count': len(value),
            'kind': 'sequence',
            'sha256': canonical_sha256(value),
        }
    )


def flatten_document(document: Mapping[str, Any]) -> dict[str, str]:
    """Create a deterministic dotted-key, scalar-string projection."""
    flattened: dict[str, str] = {}

    def visit(prefix: str, value: Any) -> None:
        if isinstance(value, Mapping) and value:
            for key in sorted(value):
                if not isinstance(key, str) or not key:
                    raise ArtifactError('CSV projection keys must be non-empty strings')
                visit(f'{prefix}.{key}' if prefix else key, value[key])
            return
        if not prefix:
            raise ArtifactError('CSV projection root must be a non-empty object')
        flattened[prefix] = _csv_scalar(value)

    visit('', document)
    return flattened


def scalar_summary_projection(document: Mapping[str, Any]) -> dict[str, str]:
    """Create the bounded scalar-summary CSV projection of a canonical document."""
    if not document:
        raise ArtifactError('CSV projection root must be a non-empty object')
    reserved = {
        CSV_PROJECTION_CONTRACT_COLUMN,
        CSV_PROJECTION_JSON_SHA256_COLUMN,
    }
    if any(key in document for key in reserved):
        raise ArtifactError('CSV projection root uses a reserved column name')
    flattened: dict[str, str] = {
        CSV_PROJECTION_CONTRACT_COLUMN: CSV_PROJECTION_CONTRACT,
        CSV_PROJECTION_JSON_SHA256_COLUMN: canonical_sha256(document),
    }

    def visit(prefix: str, value: Any) -> None:
        if isinstance(value, Mapping) and value:
            for key in sorted(value):
                if not isinstance(key, str) or not key:
                    raise ArtifactError('CSV projection keys must be non-empty strings')
                visit(f'{prefix}.{key}' if prefix else key, value[key])
            return
        if not prefix:
            raise ArtifactError('CSV projection root must be a non-empty object')
        if prefix in flattened:
            raise ArtifactError(f'CSV projection column collision at {prefix!r}')
        flattened[prefix] = (
            _csv_sequence_descriptor(value)
            if isinstance(value, (list, tuple))
            else _csv_scalar(value)
        )

    visit('', document)
    return flattened


def _one_row_projection_bytes(projection: Mapping[str, str]) -> bytes:
    output = io.StringIO(newline='')
    writer = csv.DictWriter(output, fieldnames=sorted(projection), lineterminator='\n')
    writer.writeheader()
    writer.writerow(projection)
    return output.getvalue().encode('utf-8')


def one_row_csv_bytes(document: Mapping[str, Any]) -> bytes:
    """Encode the bounded per-run scalar-summary projection as one CSV row."""
    return _one_row_projection_bytes(scalar_summary_projection(document))


def legacy_flattened_one_row_csv_bytes(document: Mapping[str, Any]) -> bytes:
    """Encode the complete legacy flattened projection used by aggregate CSV."""
    return _one_row_projection_bytes(flatten_document(document))


def _write_csv_payload_atomic(
    payload: bytes,
    path: str | Path,
    maximum_bytes: int,
) -> dict[str, Any]:
    destination = Path(path)
    byte_count = _atomic_write_bytes(destination, payload, maximum_bytes)
    return {
        'bytes': byte_count,
        'path': str(destination),
        'sha256': hashlib.sha256(payload).hexdigest(),
    }


def write_csv_atomic(
    document: Mapping[str, Any],
    path: str | Path,
    *,
    maximum_bytes: int = PER_RUN_CSV_MAX_BYTES,
) -> dict[str, Any]:
    """Write the bounded per-run scalar-summary CSV projection atomically."""
    return _write_csv_payload_atomic(one_row_csv_bytes(document), path, maximum_bytes)


def write_legacy_flattened_csv_atomic(
    document: Mapping[str, Any],
    path: str | Path,
    *,
    maximum_bytes: int,
) -> dict[str, Any]:
    """Write the complete legacy flattened aggregate CSV atomically."""
    return _write_csv_payload_atomic(
        legacy_flattened_one_row_csv_bytes(document),
        path,
        maximum_bytes,
    )


def validate_csv_projection(document: Mapping[str, Any], path: str | Path) -> None:
    """Fail unless the stored CSV contains exactly the canonical header and row."""
    expected = one_row_csv_bytes(document)
    try:
        actual = Path(path).read_bytes()
    except OSError as exc:
        raise ArtifactError(f'cannot read CSV projection: {exc}') from exc
    if actual != expected:
        raise ArtifactError('CSV projection does not exactly match canonical JSON')


def directory_size_bytes(path: str | Path) -> int:
    """Return the bounded artifact directory's regular-file byte count."""
    root = Path(path)
    return sum(item.stat().st_size for item in root.rglob('*') if item.is_file())


def require_directory_within_cap(
    path: str | Path,
    *,
    maximum_bytes: int = PER_RUN_DIRECTORY_MAX_BYTES,
) -> int:
    """Fail closed when a completed per-run artifact directory exceeds its cap."""
    byte_count = directory_size_bytes(path)
    if byte_count > maximum_bytes:
        raise ArtifactError(
            f'artifact directory is {byte_count} bytes and exceeds {maximum_bytes} bytes'
        )
    return byte_count


def write_result_pair(
    result: Mapping[str, Any],
    output_directory: str | Path,
) -> dict[str, Any]:
    """Preflight, atomically write, and reconcile canonical JSON plus CSV."""
    output = Path(output_directory)
    json_payload = canonical_json_bytes(result)
    csv_payload = one_row_csv_bytes(result)
    if len(json_payload) > PER_RUN_JSON_MAX_BYTES:
        raise ArtifactError('canonical run JSON exceeds its byte cap')
    if len(csv_payload) > PER_RUN_CSV_MAX_BYTES:
        raise ArtifactError('canonical run CSV exceeds its byte cap')
    json_path = output / 'run-result.json'
    csv_path = output / 'run-result.csv'
    json_record = write_json_atomic(result, json_path)
    csv_record = write_csv_atomic(result, csv_path)
    validate_csv_projection(result, csv_path)
    return {
        'csv': csv_record,
        'directory_bytes': require_directory_within_cap(output),
        'json': json_record,
    }

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

"""Canonical, bounded, atomic JSON artifact helpers."""

from __future__ import annotations

import hashlib
import json
import os
import re
from contextlib import suppress
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

from robotest_scenarios.constants import MAX_JSON_BYTES
from robotest_scenarios.errors import ArtifactError, ValidationError

_RUN_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$')
_CANDIDATE_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$')


def bounded_diagnostic(value: object, *, max_bytes: int = 4096) -> str:
    """Return a deterministic UTF-8-safe diagnostic bounded by encoded bytes."""
    if max_bytes <= 0:
        raise ValueError('diagnostic byte bound must be positive')
    encoded = str(value).encode('utf-8', errors='replace')
    if len(encoded) <= max_bytes:
        return encoded.decode('utf-8')
    return encoded[:max_bytes].decode('utf-8', errors='ignore')


def validate_run_id(value: str) -> str:
    """Validate one bounded run identity."""
    if _RUN_ID_PATTERN.fullmatch(value) is None:
        raise ValidationError('run ID must match [A-Za-z0-9][A-Za-z0-9._-]{0,127}')
    return value


def validate_candidate_id(value: str) -> str:
    """Validate one bounded candidate identity."""
    if _CANDIDATE_ID_PATTERN.fullmatch(value) is None:
        raise ValidationError('candidate ID must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}')
    return value


def validate_new_output(path_value: str, *, label: str) -> Path:
    """Require a non-existing regular-file target below an existing directory."""
    path = Path(path_value).expanduser().resolve(strict=False)
    if path.exists():
        raise ValidationError(f'{label} already exists: {path}')
    if not path.parent.is_dir():
        raise ValidationError(f'{label} parent directory does not exist: {path.parent}')
    if path.name in {'', '.', '..'}:
        raise ValidationError(f'{label} must name a file')
    sidecar = sha256_sidecar(path)
    if sidecar.exists():
        raise ValidationError(f'{label} checksum sidecar already exists: {sidecar}')
    return path


def canonical_json_bytes(value: Any) -> bytes:
    """Encode canonical project JSON with finite-number enforcement."""
    try:
        text = json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True,
        )
    except (TypeError, ValueError) as exc:
        raise ArtifactError(f'canonical JSON encoding failed: {exc}') from exc
    encoded = (text + '\n').encode('utf-8')
    if len(encoded) > MAX_JSON_BYTES:
        raise ArtifactError(f'canonical JSON is {len(encoded)} bytes; cap is {MAX_JSON_BYTES}')
    return encoded


def sha256_sidecar(path: Path) -> Path:
    """Return the sibling checksum sidecar path."""
    return path.with_name(path.name + '.sha256')


def load_schema(schema_path: Path) -> dict[str, Any]:
    """Load and validate one local Draft 2020-12 JSON schema."""
    try:
        value = json.loads(schema_path.read_text(encoding='utf-8'))
        Draft202012Validator.check_schema(value)
    except (OSError, json.JSONDecodeError, TypeError, ValueError) as exc:
        raise ArtifactError(f'invalid result schema {schema_path}: {exc}') from exc
    return value


def validate_against_schema(value: Any, schema: dict[str, Any]) -> None:
    """Raise one deterministic error for the first schema violation."""
    errors = sorted(Draft202012Validator(schema).iter_errors(value), key=lambda item: item.path)
    if errors:
        error = errors[0]
        location = '.'.join(str(part) for part in error.absolute_path) or '<root>'
        raise ArtifactError(f'result schema violation at {location}: {error.message}')


def _atomic_write(path: Path, data: bytes) -> None:
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    try:
        with temporary.open('xb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists():
            raise ArtifactError(f'refusing to replace existing artifact: {path}')
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except (OSError, ArtifactError) as exc:
        with suppress(OSError):
            temporary.unlink(missing_ok=True)
        if isinstance(exc, ArtifactError):
            raise
        raise ArtifactError(f'atomic write failed for {path}: {exc}') from exc


def write_canonical_json(
    path: Path,
    value: Any,
    *,
    schema: dict[str, Any] | None = None,
    write_sidecar: bool = True,
) -> str:
    """Validate, atomically write, and hash one canonical JSON object."""
    if schema is not None:
        validate_against_schema(value, schema)
    encoded = canonical_json_bytes(value)
    digest = hashlib.sha256(encoded).hexdigest()
    _atomic_write(path, encoded)
    if write_sidecar:
        sidecar = sha256_sidecar(path)
        line = f'{digest}  {path.name}\n'.encode('ascii')
        _atomic_write(sidecar, line)
    return digest

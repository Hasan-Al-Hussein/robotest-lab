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

"""JSON Schema discovery and deterministic validation errors."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from jsonschema import Draft202012Validator

from robotest_metrics.errors import ArtifactError


def schema_directory() -> Path:
    """Resolve source-checkout schemas first, then the installed package."""
    source = Path(__file__).resolve().parents[1] / 'schema'
    if source.is_dir():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory('robotest_metrics')) / 'schema'
        if installed.is_dir():
            return installed
    except (ImportError, LookupError):
        pass
    raise ArtifactError('robotest_metrics schema directory is unavailable')


def load_schema(name: str) -> dict[str, Any]:
    """Load one package-owned schema by basename."""
    if Path(name).name != name or not name.endswith('.json'):
        raise ArtifactError('schema name must be a JSON basename')
    path = schema_directory() / name
    try:
        schema = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ArtifactError(f'cannot load schema {name}: {exc}') from exc
    if not isinstance(schema, dict):
        raise ArtifactError(f'schema {name} is not an object')
    Draft202012Validator.check_schema(schema)
    return schema


def validate_document(document: Any, schema_name: str) -> None:
    """Validate and report the first error using stable path ordering."""
    validator = Draft202012Validator(load_schema(schema_name))
    errors = sorted(
        validator.iter_errors(document),
        key=lambda error: tuple(str(component) for component in error.absolute_path),
    )
    if not errors:
        return
    error = errors[0]
    path = '.'.join(str(component) for component in error.absolute_path) or '<root>'
    raise ArtifactError(f'{schema_name} validation failed at {path}: {error.message}')

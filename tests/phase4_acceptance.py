#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Fail-closed Phase 4 process, package, and Scenario 6 evidence utilities.

The module intentionally has no ROS imports.  Runtime ROS observation lives in
``phase4_active_goal_probe.py``; everything here is deterministic and unit
testable without starting Gazebo, systemd, or a supervisor process.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import grp
import hashlib
import json
import math
import os
import pwd
import re
import signal
import stat
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from collections.abc import Mapping, Sequence
from datetime import UTC, datetime
from itertools import pairwise
from pathlib import Path
from typing import Any

SCHEMA_VERSION = 1
PRODUCER = 'robotest_phase4/acceptance_verifier'
MAX_JSON_BYTES = 32 * 1024 * 1024
MAX_TEXT_BYTES = 8 * 1024 * 1024
MAX_TIMELINE_ENTRIES = 4096
MAX_PROCESS_COUNT = 65536
MAX_PROC_FILE_BYTES = 1024 * 1024
MAX_EVIDENCE_FILES = 512
MAX_SOURCE_FILES = 20000
MAX_PACKAGE_BYTES = 512 * 1024 * 1024
READY_FAILURE_TARGET_S = 3.0
PROCESS_GROUP_EXIT_TARGET_S = 5.0
RECOVERY_TARGET_S = 30.0
EXPECTED_BACKOFF_MS = 1000
EXPECTED_CHILD = 'robotest-stack'
EXPECTED_UNIT = 'robotest-supervisor.service'
SHA256_RE = re.compile(r'^[0-9a-f]{64}$')
RUN_ID_RE = re.compile(r'^phase4-[0-9]{8}T[0-9]{6}Z-[0-9]+$')
PARTITION_RE = re.compile(r'^[A-Za-z][A-Za-z0-9_]{0,127}$')
_DEFAULT_API = object()

LIFECYCLE_KEYS = frozenset(
    {
        'baseline_package',
        'completed_utc',
        'conffile',
        'final_state',
        'quality',
        'schema_version',
        'state',
        'upgrade_package',
        'verdict',
    }
)
LIFECYCLE_PACKAGE_KEYS = frozenset(
    {
        'architecture',
        'name',
        'sha256',
        'vendor_config_sha256',
        'version',
    }
)
LIFECYCLE_CONFFILE_KEYS = frozenset(
    {
        'modified_sha256',
        'modified_value',
        'preserved_during_remove',
        'preserved_during_upgrade',
        'removed_during_purge',
        'upgrade_vendor_default_restored_on_final_install',
    }
)
LIFECYCLE_STATE_KEYS = frozenset(
    {
        'log_directory_preserved',
        'sentinel_preserved_during_purge',
        'sentinel_preserved_during_remove',
        'state_directory_preserved',
    }
)
LIFECYCLE_FINAL_KEYS = frozenset(
    {
        'configuration_mode_owner',
        'dpkg_verify_clean',
        'installed_binary_sha256',
        'installed_helper_sha256',
        'installed_smoke_contract_passed',
        'installed_unit_sha256',
        'log_directory_mode_owner',
        'package_installed',
        'service_account_render_group',
        'service_account_video_group',
        'service_active',
        'service_enabled',
        'stale_dpkg_conffile_artifacts',
        'state_directory_mode_owner',
        'systemd_unit_verified',
    }
)
LIFECYCLE_QUALITY_KEYS = frozenset(
    {
        'baseline_provenance_verified',
        'both_package_manifests_exact',
        'genuine_versioned_upgrade',
        'legacy_noble_lintian_static_built_using_warning',
        'lintian_unexpected_diagnostics',
    }
)
INSTALLED_STATE_KEYS = frozenset(
    {
        'architecture',
        'captured_utc',
        'configuration_mode_owner',
        'configuration_sha256',
        'dpkg_verify_clean',
        'installed_binary_sha256',
        'installed_helper_sha256',
        'installed_smoke_contract_passed',
        'installed_unit_sha256',
        'log_directory_mode_owner',
        'package_installed',
        'package_name',
        'schema_version',
        'service_account_render_group',
        'service_account_video_group',
        'service_active',
        'service_enabled',
        'stale_dpkg_conffile_artifacts',
        'state_directory_mode_owner',
        'systemd_unit_verified',
        'verdict',
        'version',
    }
)
REPRODUCIBILITY_KEYS = frozenset(
    {
        'binary_packages_byte_identical',
        'builds',
        'completed_utc',
        'files',
        'generated_metadata_policy',
        'schema_version',
        'source_manifest_byte_identical',
        'verdict',
    }
)
REPRODUCIBILITY_FILE_KEYS = frozenset(
    {
        'build_a_sha256',
        'build_b_sha256',
        'comparison',
        'name',
        'normalized_sha256',
        'size_bytes',
    }
)
BASELINE_SIDECAR_KEYS = frozenset(
    {'created_utc', 'final_package', 'fixture_kind', 'fixture_package', 'schema_version'}
)
BASELINE_FINAL_PACKAGE_KEYS = frozenset({'name', 'sha256', 'version'})
BASELINE_FIXTURE_PACKAGE_KEYS = frozenset({'listen_address', 'name', 'sha256', 'version'})
PACKAGE_INTEGRITY_KEYS = frozenset(
    {
        'baseline_package_sha256',
        'baseline_sidecar_sha256',
        'build_a_binary_sha256',
        'build_a_debug_sha256',
        'build_b_binary_sha256',
        'build_b_debug_sha256',
        'package_directory',
        'reproducibility_evidence_sha256',
        'schema_version',
        'source_manifest_sha256',
        'upgrade_package_sha256',
        'verdict',
    }
)
CLEANUP_KEYS = frozenset(
    {
        'captured_utc',
        'daemon_reloaded',
        'dropin_absent',
        'followup_last_pgid',
        'followup_process_group_empty',
        'heartbeat_absent',
        'interrupted_last_pgid',
        'interrupted_process_group_empty',
        'mission_process_group_empty',
        'original_child_pgid',
        'original_process_group_empty',
        'owned_paths_only',
        'probe_last_pgid',
        'probe_process_group_empty',
        'replacement_child_pgid',
        'replacement_process_group_empty',
        'run_state_directory_absent',
        'schema_version',
        'service_disabled',
        'service_inactive',
        'source_unchanged',
        'startup_result_absent',
    }
)
INTERRUPTED_OUTCOME_KEYS = frozenset(
    {
        'bounded_wait',
        'captured_utc',
        'csv_present',
        'json_present',
        'process_exit_code',
        'process_group_empty',
        'schema_version',
        'terminated_by_harness',
    }
)


class EvidenceError(RuntimeError):
    """Raised for missing, ambiguous, malformed, or unbounded evidence."""


def utc_now() -> str:
    """Return an RFC 3339 UTC timestamp."""
    return datetime.now(UTC).isoformat(timespec='microseconds').replace('+00:00', 'Z')


def canonical_json_bytes(value: Any) -> bytes:
    """Serialize strict canonical JSON with one trailing newline."""
    try:
        payload = json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True,
        )
    except (TypeError, ValueError) as exc:
        raise EvidenceError(f'value is not strict JSON: {exc}') from exc
    return (payload + '\n').encode('utf-8')


def file_sha256(path: Path) -> str:
    """Hash one regular non-symlink file without an unbounded read."""
    if path.is_symlink() or not path.is_file():
        raise EvidenceError(f'not a regular non-symlink file: {path}')
    digest = hashlib.sha256()
    with path.open('rb') as source:
        while block := source.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _read_bounded_bytes(path: Path, maximum_bytes: int) -> bytes:
    if path.is_symlink() or not path.is_file():
        raise EvidenceError(f'not a regular non-symlink file: {path}')
    if path.stat().st_size > maximum_bytes:
        raise EvidenceError(f'{path} exceeds its {maximum_bytes}-byte cap')
    payload = path.read_bytes()
    if len(payload) > maximum_bytes:
        raise EvidenceError(f'{path} exceeds its {maximum_bytes}-byte cap')
    return payload


def _atomic_write(path: Path, payload: bytes, *, mode: int = 0o644) -> None:
    if len(payload) > MAX_JSON_BYTES:
        raise EvidenceError(f'{path} exceeds the {MAX_JSON_BYTES}-byte artifact cap')
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, mode)
        with os.fdopen(descriptor, 'wb') as target:
            target.write(payload)
            target.flush()
            os.fsync(target.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        with contextlib.suppress(OSError):
            os.close(descriptor)
        temporary.unlink(missing_ok=True)
        raise


def atomic_write_json(path: Path, value: Any, *, mode: int = 0o644) -> None:
    """Atomically write strict canonical JSON."""
    _atomic_write(path, canonical_json_bytes(value), mode=mode)


def load_json(path: Path, *, maximum_bytes: int = MAX_JSON_BYTES) -> Any:
    """Load one bounded strict JSON value and reject trailing data."""
    if path.is_symlink() or not path.is_file():
        raise EvidenceError(f'missing regular JSON file: {path}')
    size = path.stat().st_size
    if size > maximum_bytes:
        raise EvidenceError(f'{path} exceeds its {maximum_bytes}-byte cap')
    try:
        text = path.read_text(encoding='utf-8')
        decoder = json.JSONDecoder()
        value, end = decoder.raw_decode(text)
        if text[end:].strip():
            raise EvidenceError(f'{path} contains trailing JSON data')
        return value
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise EvidenceError(f'cannot decode {path}: {exc}') from exc


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    """Load a bounded JSON-lines file with object-only records."""
    if path.is_symlink() or not path.is_file():
        raise EvidenceError(f'missing regular JSONL file: {path}')
    if path.stat().st_size > MAX_JSON_BYTES:
        raise EvidenceError(f'{path} exceeds the JSONL byte cap')
    records: list[dict[str, Any]] = []
    try:
        with path.open(encoding='utf-8') as source:
            for line_number, line in enumerate(source, 1):
                if line_number > MAX_TIMELINE_ENTRIES:
                    raise EvidenceError(f'{path} exceeds the record cap')
                if not line.endswith('\n'):
                    raise EvidenceError(f'{path} has an incomplete final record')
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise EvidenceError(f'{path}:{line_number} is not an object')
                records.append(value)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise EvidenceError(f'cannot decode {path}: {exc}') from exc
    return records


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise EvidenceError(f'{name} must be an object')
    return value


def _integer(value: Any, name: str, *, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise EvidenceError(f'{name} must be an integer')
    if minimum is not None and value < minimum:
        raise EvidenceError(f'{name} must be >= {minimum}')
    return value


def _number(value: Any, name: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EvidenceError(f'{name} must be numeric')
    result = float(value)
    if not math.isfinite(result):
        raise EvidenceError(f'{name} must be finite')
    if minimum is not None and result < minimum:
        raise EvidenceError(f'{name} must be >= {minimum}')
    return result


def _string(value: Any, name: str, *, maximum: int = 4096) -> str:
    if not isinstance(value, str) or not value:
        raise EvidenceError(f'{name} must be a non-empty string')
    if len(value.encode('utf-8')) > maximum:
        raise EvidenceError(f'{name} exceeds the UTF-8 byte cap')
    return value


def _sha256(value: Any, name: str) -> str:
    result = _string(value, name, maximum=64)
    if SHA256_RE.fullmatch(result) is None:
        raise EvidenceError(f'{name} must be a lowercase SHA-256')
    return result


def _boolean(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise EvidenceError(f'{name} must be a boolean')
    return value


def _exact_keys(value: Mapping[str, Any], expected: frozenset[str], name: str) -> None:
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise EvidenceError(f'{name} schema mismatch; missing={missing}, extra={extra}')


def _utc_timestamp(value: Any, name: str) -> str:
    result = _string(value, name, maximum=64)
    try:
        parsed = datetime.fromisoformat(result.replace('Z', '+00:00'))
    except ValueError as exc:
        raise EvidenceError(f'{name} must be an ISO-8601 timestamp') from exc
    if parsed.tzinfo is None or parsed.utcoffset() != UTC.utcoffset(parsed):
        raise EvidenceError(f'{name} must have an explicit UTC offset')
    return result


def _run_command(
    command: Sequence[str],
    *,
    accepted_returncodes: frozenset[int] = frozenset({0}),
    timeout_s: float = 60.0,
) -> subprocess.CompletedProcess[str]:
    try:
        result = subprocess.run(
            list(command),
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise EvidenceError(f'command failed to execute: {command[0]}: {exc}') from exc
    if result.returncode not in accepted_returncodes:
        stderr = result.stderr.strip()[:4096]
        raise EvidenceError(f'command returned {result.returncode}: {command!r}: {stderr}')
    if len(result.stdout.encode('utf-8')) > MAX_TEXT_BYTES:
        raise EvidenceError(f'command output exceeded byte cap: {command!r}')
    return result


def _package_field(package: Path, field: str) -> str:
    output = _run_command(['dpkg-deb', '--field', str(package), field]).stdout.strip()
    return _string(output, f'{package.name} {field}', maximum=256)


def package_artifact_descriptor(package: Path) -> dict[str, str]:
    """Bind package metadata and required installed payload bytes to one .deb."""
    if package.is_symlink():
        raise EvidenceError(f'package must not be a symbolic link: {package}')
    package = package.resolve(strict=True)
    if not package.is_file():
        raise EvidenceError(f'package is not a regular non-symlink file: {package}')
    if package.stat().st_size > MAX_PACKAGE_BYTES:
        raise EvidenceError(f'package exceeds the {MAX_PACKAGE_BYTES}-byte cap: {package}')
    with tempfile.TemporaryDirectory(prefix='robotest-phase4-package.') as temporary:
        extraction = Path(temporary) / 'root'
        extraction.mkdir()
        _run_command(['dpkg-deb', '--extract', str(package), str(extraction)])
        config = extraction / 'etc/robotest-supervisor/config.json'
        binary = extraction / 'usr/bin/robotest-supervisor'
        helper = extraction / 'usr/libexec/robotest-supervisor/start-robotest-stack'
        unit = extraction / 'usr/lib/systemd/system/robotest-supervisor.service'
        config_value = _mapping(
            load_json(config, maximum_bytes=MAX_TEXT_BYTES),
            f'{package.name} vendor config',
        )
        listen_address = _string(
            config_value.get('listen_address'),
            f'{package.name} vendor listen_address',
            maximum=256,
        )
        modified_value = dict(config_value)
        modified_value['listen_address'] = '127.0.0.1:9081'
        modified_payload = (
            json.dumps(modified_value, indent=2, sort_keys=True).encode('utf-8') + b'\n'
        )
        return {
            'name': _package_field(package, 'Package'),
            'version': _package_field(package, 'Version'),
            'architecture': _package_field(package, 'Architecture'),
            'sha256': file_sha256(package),
            'vendor_config_sha256': file_sha256(config),
            'vendor_listen_address': listen_address,
            'modified_config_sha256': hashlib.sha256(modified_payload).hexdigest(),
            'installed_binary_sha256': file_sha256(binary),
            'installed_helper_sha256': file_sha256(helper),
            'installed_unit_sha256': file_sha256(unit),
        }


def _normalize_generated_metadata(name: str, payload: bytes) -> bytes:
    try:
        text = payload.decode('utf-8')
    except UnicodeDecodeError as exc:
        raise EvidenceError(f'generated package metadata is not UTF-8: {name}') from exc
    if name.endswith('.buildinfo'):
        text, count = re.subn(
            r'^Build-Date: .+$',
            'Build-Date: <dpkg-wall-clock-build-date>',
            text,
            count=1,
            flags=re.MULTILINE,
        )
        if count != 1:
            raise EvidenceError(f'{name} has no unique Build-Date field')
        return text.encode()
    if name.endswith('.changes'):
        text, count = re.subn(
            r'^(\s+)\S+(\s+.*\.buildinfo)$',
            r'\1<buildinfo-checksum>\2',
            text,
            flags=re.MULTILINE,
        )
        if count != 3:
            raise EvidenceError(f'{name} has {count} buildinfo checksums instead of 3')
        return text.encode()
    if name == 'SHA256SUMS':
        text, buildinfo_count = re.subn(
            r'^[0-9a-f]{64}(  \S+\.buildinfo)$',
            r'<buildinfo-sha256>\1',
            text,
            flags=re.MULTILINE,
        )
        text, changes_count = re.subn(
            r'^[0-9a-f]{64}(  \S+\.changes)$',
            r'<changes-sha256>\1',
            text,
            flags=re.MULTILINE,
        )
        if buildinfo_count != 1 or changes_count != 1:
            raise EvidenceError('SHA256SUMS has an unexpected generated-metadata shape')
        return text.encode()
    return payload


def _package_build_files(directory: Path) -> dict[str, Path]:
    if directory.is_symlink() or not directory.is_dir():
        raise EvidenceError(f'package build directory is invalid: {directory}')
    files: dict[str, Path] = {}
    for path in directory.iterdir():
        if path.is_symlink() or not path.is_file():
            raise EvidenceError(f'package build contains a non-regular entry: {path}')
        if path.name in files:
            raise EvidenceError(f'duplicate package build filename: {path.name}')
        files[path.name] = path
    return files


def _validate_local_sha256sums(directory: Path, files: Mapping[str, Path]) -> None:
    manifest = files.get('SHA256SUMS')
    if manifest is None:
        raise EvidenceError(f'{directory} is missing SHA256SUMS')
    payload = _read_bounded_bytes(manifest, MAX_TEXT_BYTES)
    if payload and not payload.endswith(b'\n'):
        raise EvidenceError(f'{manifest} has an incomplete final line')
    expected_names = set(files) - {'SHA256SUMS'}
    observed: dict[str, str] = {}
    for line_number, raw_line in enumerate(payload.decode('utf-8').splitlines(), 1):
        match = re.fullmatch(r'([0-9a-f]{64})  ([^/\s]+)', raw_line)
        if match is None:
            raise EvidenceError(f'{manifest}:{line_number} is not canonical')
        digest, name = match.groups()
        if name in observed:
            raise EvidenceError(f'{manifest} repeats {name}')
        observed[name] = digest
    if set(observed) != expected_names:
        raise EvidenceError(f'{manifest} does not cover the exact build file set')
    for name, digest in observed.items():
        if file_sha256(files[name]) != digest:
            raise EvidenceError(f'{manifest} does not match {name}')


def verify_package_candidate(
    package_directory: Path,
    upgrade_package: Path,
    baseline_package: Path,
) -> dict[str, Any]:
    """Independently reconcile both builds, repro evidence, and baseline provenance."""
    if package_directory.is_symlink():
        raise EvidenceError(f'package directory must not be a symlink: {package_directory}')
    package_directory = package_directory.resolve(strict=True)
    if not package_directory.is_dir():
        raise EvidenceError(f'package directory is invalid: {package_directory}')
    build_a = _package_build_files(package_directory / 'build-a')
    build_b = _package_build_files(package_directory / 'build-b')
    if set(build_a) != set(build_b):
        raise EvidenceError('build-a and build-b file sets differ')
    _validate_local_sha256sums(package_directory / 'build-a', build_a)
    _validate_local_sha256sums(package_directory / 'build-b', build_b)

    reproducibility_path = package_directory / 'reproducibility.json'
    reproducibility = _mapping(load_json(reproducibility_path), 'package reproducibility evidence')
    _exact_keys(
        reproducibility,
        REPRODUCIBILITY_KEYS,
        'package reproducibility evidence',
    )
    if _integer(reproducibility.get('schema_version'), 'repro schema_version') != 1:
        raise EvidenceError('package reproducibility schema_version must be 1')
    if reproducibility.get('verdict') != 'PASS':
        raise EvidenceError('package reproducibility verdict must be PASS')
    _utc_timestamp(reproducibility.get('completed_utc'), 'repro completed_utc')
    if reproducibility.get('builds') != ['build-a', 'build-b']:
        raise EvidenceError('package reproducibility builds are not exact')
    if (
        _boolean(
            reproducibility.get('binary_packages_byte_identical'),
            'binary_packages_byte_identical',
        )
        is not True
    ):
        raise EvidenceError('binary package reproducibility is not proven')
    if (
        _boolean(
            reproducibility.get('source_manifest_byte_identical'),
            'source_manifest_byte_identical',
        )
        is not True
    ):
        raise EvidenceError('source manifest reproducibility is not proven')
    policy = (
        'dpkg .buildinfo Build-Date is wall-clock metadata; .changes and SHA256SUMS '
        'may differ only through the cascading .buildinfo/.changes checksums'
    )
    if reproducibility.get('generated_metadata_policy') != policy:
        raise EvidenceError('package reproducibility metadata policy is not exact')

    records = _mapping_list(reproducibility.get('files'), 'reproducibility files')
    names = [_string(record.get('name'), 'repro file name', maximum=255) for record in records]
    if names != sorted(names) or len(names) != len(set(names)) or set(names) != set(build_a):
        raise EvidenceError('reproducibility file list is not the exact sorted build set')
    binary_names = [name for name in names if name.endswith('.deb') and not name.endswith('.ddeb')]
    debug_names = [name for name in names if name.endswith('.ddeb')]
    buildinfo_names = [name for name in names if name.endswith('.buildinfo')]
    changes_names = [name for name in names if name.endswith('.changes')]
    if not (
        len(names) == 6
        and len(binary_names) == 1
        and len(debug_names) == 1
        and len(buildinfo_names) == 1
        and len(changes_names) == 1
        and {'SHA256SUMS', 'SOURCE-MANIFEST.json'} <= set(names)
    ):
        raise EvidenceError('package build artifact set is not exact')

    for record, name in zip(records, names, strict=True):
        _exact_keys(record, REPRODUCIBILITY_FILE_KEYS, f'reproducibility file {name}')
        first_payload = _read_bounded_bytes(build_a[name], MAX_PACKAGE_BYTES)
        second_payload = _read_bounded_bytes(build_b[name], MAX_PACKAGE_BYTES)
        comparison = 'byte_identical'
        normalized = first_payload
        if first_payload != second_payload:
            if not (
                name == 'SHA256SUMS' or name.endswith('.buildinfo') or name.endswith('.changes')
            ):
                raise EvidenceError(f'cross-build byte mismatch: {name}')
            normalized = _normalize_generated_metadata(name, first_payload)
            if normalized != _normalize_generated_metadata(name, second_payload):
                raise EvidenceError(f'cross-build normalized mismatch: {name}')
            comparison = 'normalized_dpkg_build_date_only'
        expected_record = {
            'build_a_sha256': hashlib.sha256(first_payload).hexdigest(),
            'build_b_sha256': hashlib.sha256(second_payload).hexdigest(),
            'comparison': comparison,
            'name': name,
            'normalized_sha256': hashlib.sha256(normalized).hexdigest(),
            'size_bytes': len(first_payload),
        }
        if len(first_payload) != len(second_payload):
            raise EvidenceError(f'cross-build size mismatch: {name}')
        for field in ('build_a_sha256', 'build_b_sha256', 'normalized_sha256'):
            _sha256(record.get(field), f'{name}.{field}')
        _integer(record.get('size_bytes'), f'{name}.size_bytes', minimum=0)
        if dict(record) != expected_record:
            raise EvidenceError(f'reproducibility evidence does not reconcile {name}')

    binary_name = binary_names[0]
    debug_name = debug_names[0]
    source_name = 'SOURCE-MANIFEST.json'
    for name in (binary_name, debug_name, source_name):
        if file_sha256(build_a[name]) != file_sha256(build_b[name]):
            raise EvidenceError(f'required byte-identical artifact differs: {name}')

    upgrade_package = upgrade_package.resolve(strict=True)
    baseline_package = baseline_package.resolve(strict=True)
    if upgrade_package != build_a[binary_name].resolve(strict=True):
        raise EvidenceError('selected upgrade package is not the build-a binary')
    upgrade = package_artifact_descriptor(upgrade_package)
    baseline = package_artifact_descriptor(baseline_package)
    sidecar_path = Path(f'{baseline_package}.fixture.json')
    if sidecar_path.is_symlink():
        raise EvidenceError('baseline provenance sidecar must not be a symlink')
    sidecar = _mapping(load_json(sidecar_path), 'baseline provenance sidecar')
    _exact_keys(sidecar, BASELINE_SIDECAR_KEYS, 'baseline provenance sidecar')
    if _integer(sidecar.get('schema_version'), 'baseline sidecar schema_version') != 1:
        raise EvidenceError('baseline provenance schema_version must be 1')
    _utc_timestamp(sidecar.get('created_utc'), 'baseline sidecar created_utc')
    if sidecar.get('fixture_kind') != 'genuine_lower_version_different_default_conffile':
        raise EvidenceError('baseline provenance fixture_kind is not exact')
    final_package = _mapping(sidecar.get('final_package'), 'sidecar final_package')
    fixture_package = _mapping(sidecar.get('fixture_package'), 'sidecar fixture_package')
    _exact_keys(final_package, BASELINE_FINAL_PACKAGE_KEYS, 'sidecar final_package')
    _exact_keys(fixture_package, BASELINE_FIXTURE_PACKAGE_KEYS, 'sidecar fixture_package')
    expected_final = {
        'name': upgrade_package.name,
        'sha256': upgrade['sha256'],
        'version': upgrade['version'],
    }
    expected_fixture = {
        'listen_address': baseline['vendor_listen_address'],
        'name': baseline_package.name,
        'sha256': baseline['sha256'],
        'version': baseline['version'],
    }
    if dict(final_package) != expected_final or dict(fixture_package) != expected_fixture:
        raise EvidenceError('baseline provenance does not bind both selected packages')
    if baseline['vendor_config_sha256'] == upgrade['vendor_config_sha256']:
        raise EvidenceError('baseline and upgrade vendor configurations are identical')

    report = {
        'baseline_package_sha256': baseline['sha256'],
        'baseline_sidecar_sha256': file_sha256(sidecar_path),
        'build_a_binary_sha256': file_sha256(build_a[binary_name]),
        'build_a_debug_sha256': file_sha256(build_a[debug_name]),
        'build_b_binary_sha256': file_sha256(build_b[binary_name]),
        'build_b_debug_sha256': file_sha256(build_b[debug_name]),
        'package_directory': str(package_directory),
        'reproducibility_evidence_sha256': file_sha256(reproducibility_path),
        'schema_version': SCHEMA_VERSION,
        'source_manifest_sha256': file_sha256(build_a[source_name]),
        'upgrade_package_sha256': upgrade['sha256'],
        'verdict': 'PASS',
    }
    _exact_keys(report, PACKAGE_INTEGRITY_KEYS, 'package integrity report')
    return report


def _validate_lifecycle_package(
    value: Any,
    expected: Mapping[str, Any],
    name: str,
) -> Mapping[str, Any]:
    package = _mapping(value, name)
    _exact_keys(package, LIFECYCLE_PACKAGE_KEYS, name)
    expected_fields = {key: expected.get(key) for key in LIFECYCLE_PACKAGE_KEYS}
    for field in ('name', 'version', 'architecture'):
        _string(package.get(field), f'{name}.{field}', maximum=256)
        _string(expected_fields.get(field), f'expected {name}.{field}', maximum=256)
    for field in ('sha256', 'vendor_config_sha256'):
        _sha256(package.get(field), f'{name}.{field}')
        _sha256(expected_fields.get(field), f'expected {name}.{field}')
    if dict(package) != expected_fields:
        raise EvidenceError(f'{name} does not match its selected package artifact')
    return package


def validate_lifecycle_evidence(
    value: Any,
    upgrade: Mapping[str, Any],
    baseline: Mapping[str, Any],
) -> None:
    """Validate the exact lifecycle schema and every frozen proof fact."""
    lifecycle = _mapping(value, 'package lifecycle')
    _exact_keys(lifecycle, LIFECYCLE_KEYS, 'package lifecycle')
    if _integer(lifecycle.get('schema_version'), 'package lifecycle schema_version') != (
        SCHEMA_VERSION
    ):
        raise EvidenceError('package lifecycle schema_version must be 1')
    if lifecycle.get('verdict') != 'PASS':
        raise EvidenceError('package lifecycle verdict must be PASS')
    _utc_timestamp(lifecycle.get('completed_utc'), 'package lifecycle completed_utc')

    baseline_value = _validate_lifecycle_package(
        lifecycle.get('baseline_package'), baseline, 'baseline_package'
    )
    upgrade_value = _validate_lifecycle_package(
        lifecycle.get('upgrade_package'), upgrade, 'upgrade_package'
    )
    if baseline_value['name'] != 'robotest-supervisor' or upgrade_value['name'] != (
        'robotest-supervisor'
    ):
        raise EvidenceError('lifecycle packages must both be robotest-supervisor')
    if baseline_value['architecture'] != upgrade_value['architecture']:
        raise EvidenceError('lifecycle package architectures differ')
    version_check = _run_command(
        [
            'dpkg',
            '--compare-versions',
            str(baseline_value['version']),
            'lt',
            str(upgrade_value['version']),
        ],
        accepted_returncodes=frozenset({0, 1}),
    )
    if version_check.returncode != 0:
        raise EvidenceError('lifecycle baseline version is not lower than the upgrade')
    if baseline_value['vendor_config_sha256'] == upgrade_value['vendor_config_sha256']:
        raise EvidenceError('lifecycle packages do not have distinct vendor configurations')

    conffile = _mapping(lifecycle.get('conffile'), 'package lifecycle conffile')
    _exact_keys(conffile, LIFECYCLE_CONFFILE_KEYS, 'package lifecycle conffile')
    modified_sha = _sha256(conffile.get('modified_sha256'), 'conffile.modified_sha256')
    expected_modified_sha = _sha256(
        baseline.get('modified_config_sha256'), 'expected modified config SHA'
    )
    if modified_sha != expected_modified_sha or modified_sha in {
        baseline_value['vendor_config_sha256'],
        upgrade_value['vendor_config_sha256'],
    }:
        raise EvidenceError('modified conffile SHA is not the exact lifecycle mutation')
    modified_value = _mapping(conffile.get('modified_value'), 'conffile.modified_value')
    _exact_keys(modified_value, frozenset({'listen_address'}), 'conffile.modified_value')
    if dict(modified_value) != {'listen_address': '127.0.0.1:9081'}:
        raise EvidenceError('conffile.modified_value is not the frozen lifecycle mutation')
    for field in (
        'preserved_during_remove',
        'preserved_during_upgrade',
        'removed_during_purge',
        'upgrade_vendor_default_restored_on_final_install',
    ):
        if _boolean(conffile.get(field), f'conffile.{field}') is not True:
            raise EvidenceError(f'conffile.{field} must be true')

    state_value = _mapping(lifecycle.get('state'), 'package lifecycle state')
    _exact_keys(state_value, LIFECYCLE_STATE_KEYS, 'package lifecycle state')
    for field in sorted(LIFECYCLE_STATE_KEYS):
        if _boolean(state_value.get(field), f'state.{field}') is not True:
            raise EvidenceError(f'state.{field} must be true')

    final = _mapping(lifecycle.get('final_state'), 'package lifecycle final_state')
    _exact_keys(final, LIFECYCLE_FINAL_KEYS, 'package lifecycle final_state')
    expected_final: dict[str, Any] = {
        'configuration_mode_owner': '644:root:root',
        'dpkg_verify_clean': True,
        'installed_binary_sha256': upgrade.get('installed_binary_sha256'),
        'installed_helper_sha256': upgrade.get('installed_helper_sha256'),
        'installed_smoke_contract_passed': True,
        'installed_unit_sha256': upgrade.get('installed_unit_sha256'),
        'log_directory_mode_owner': ('750:robotest-supervisor:robotest-supervisor'),
        'package_installed': True,
        'service_account_render_group': True,
        'service_account_video_group': True,
        'service_active': False,
        'service_enabled': False,
        'stale_dpkg_conffile_artifacts': 0,
        'state_directory_mode_owner': ('750:robotest-supervisor:robotest-supervisor'),
        'systemd_unit_verified': True,
    }
    for field in (
        'installed_binary_sha256',
        'installed_helper_sha256',
        'installed_unit_sha256',
    ):
        _sha256(final.get(field), f'final_state.{field}')
        _sha256(expected_final.get(field), f'expected final_state.{field}')
    for field in (
        'dpkg_verify_clean',
        'installed_smoke_contract_passed',
        'package_installed',
        'service_account_render_group',
        'service_account_video_group',
        'service_active',
        'service_enabled',
        'systemd_unit_verified',
    ):
        _boolean(final.get(field), f'final_state.{field}')
    _integer(
        final.get('stale_dpkg_conffile_artifacts'),
        'final_state.stale_dpkg_conffile_artifacts',
        minimum=0,
    )
    for field in (
        'configuration_mode_owner',
        'log_directory_mode_owner',
        'state_directory_mode_owner',
    ):
        _string(final.get(field), f'final_state.{field}', maximum=128)
    if dict(final) != expected_final:
        raise EvidenceError('package lifecycle final_state is incomplete or tampered')

    quality = _mapping(lifecycle.get('quality'), 'package lifecycle quality')
    _exact_keys(quality, LIFECYCLE_QUALITY_KEYS, 'package lifecycle quality')
    for field in (
        'baseline_provenance_verified',
        'both_package_manifests_exact',
        'genuine_versioned_upgrade',
    ):
        if _boolean(quality.get(field), f'quality.{field}') is not True:
            raise EvidenceError(f'quality.{field} must be true')
    _boolean(
        quality.get('legacy_noble_lintian_static_built_using_warning'),
        'quality.legacy_noble_lintian_static_built_using_warning',
    )
    if (
        _integer(
            quality.get('lintian_unexpected_diagnostics'),
            'quality.lintian_unexpected_diagnostics',
            minimum=0,
        )
        != 0
    ):
        raise EvidenceError('quality.lintian_unexpected_diagnostics must be zero')


def _mode_owner(path: Path) -> str:
    if path.is_symlink() or not path.exists():
        raise EvidenceError(f'installed path is missing or a symlink: {path}')
    metadata = path.stat()
    return (
        f'{stat.S_IMODE(metadata.st_mode):o}:'
        f'{pwd.getpwuid(metadata.st_uid).pw_name}:'
        f'{grp.getgrgid(metadata.st_gid).gr_name}'
    )


def validate_installed_package_state(
    value: Any,
    lifecycle: Mapping[str, Any],
    upgrade: Mapping[str, Any],
) -> None:
    """Validate the exact direct re-observation of the installed candidate."""
    report = _mapping(value, 'installed package state')
    _exact_keys(report, INSTALLED_STATE_KEYS, 'installed package state')
    final = _mapping(lifecycle.get('final_state'), 'package lifecycle final_state')
    expected: dict[str, Any] = {
        'architecture': upgrade.get('architecture'),
        'captured_utc': report.get('captured_utc'),
        'configuration_mode_owner': final.get('configuration_mode_owner'),
        'configuration_sha256': upgrade.get('vendor_config_sha256'),
        'dpkg_verify_clean': final.get('dpkg_verify_clean'),
        'installed_binary_sha256': final.get('installed_binary_sha256'),
        'installed_helper_sha256': final.get('installed_helper_sha256'),
        'installed_smoke_contract_passed': final.get('installed_smoke_contract_passed'),
        'installed_unit_sha256': final.get('installed_unit_sha256'),
        'log_directory_mode_owner': final.get('log_directory_mode_owner'),
        'package_installed': final.get('package_installed'),
        'package_name': upgrade.get('name'),
        'schema_version': SCHEMA_VERSION,
        'service_account_render_group': final.get('service_account_render_group'),
        'service_account_video_group': final.get('service_account_video_group'),
        'service_active': final.get('service_active'),
        'service_enabled': final.get('service_enabled'),
        'stale_dpkg_conffile_artifacts': final.get('stale_dpkg_conffile_artifacts'),
        'state_directory_mode_owner': final.get('state_directory_mode_owner'),
        'systemd_unit_verified': final.get('systemd_unit_verified'),
        'verdict': 'PASS',
        'version': upgrade.get('version'),
    }
    if _integer(report.get('schema_version'), 'installed package schema_version') != (
        SCHEMA_VERSION
    ):
        raise EvidenceError('installed package schema_version must be 1')
    _utc_timestamp(report.get('captured_utc'), 'installed package captured_utc')
    for field in (
        'architecture',
        'configuration_mode_owner',
        'log_directory_mode_owner',
        'package_name',
        'state_directory_mode_owner',
        'verdict',
        'version',
    ):
        _string(report.get(field), f'installed package {field}', maximum=256)
    for field in (
        'configuration_sha256',
        'installed_binary_sha256',
        'installed_helper_sha256',
        'installed_unit_sha256',
    ):
        _sha256(report.get(field), f'installed package {field}')
    for field in (
        'dpkg_verify_clean',
        'installed_smoke_contract_passed',
        'package_installed',
        'service_account_render_group',
        'service_account_video_group',
        'service_active',
        'service_enabled',
        'systemd_unit_verified',
    ):
        _boolean(report.get(field), f'installed package {field}')
    _integer(
        report.get('stale_dpkg_conffile_artifacts'),
        'installed package stale_dpkg_conffile_artifacts',
        minimum=0,
    )
    if dict(report) != expected:
        raise EvidenceError('installed package state does not match lifecycle evidence')


def validate_cleanup_evidence(
    value: Any,
    *,
    interrupted_pgid: int,
    followup_pgid: int,
    probe_pgid: int,
    original_pgid: int,
    replacement_pgid: int,
) -> None:
    """Require exact, write-once cleanup evidence bound to every published PGID."""
    cleanup = _mapping(value, 'cleanup evidence')
    _exact_keys(cleanup, CLEANUP_KEYS, 'cleanup evidence')
    if _integer(cleanup.get('schema_version'), 'cleanup schema_version') != 1:
        raise EvidenceError('cleanup schema_version must be 1')
    _utc_timestamp(cleanup.get('captured_utc'), 'cleanup captured_utc')
    boolean_fields = CLEANUP_KEYS - {
        'captured_utc',
        'followup_last_pgid',
        'interrupted_last_pgid',
        'original_child_pgid',
        'probe_last_pgid',
        'replacement_child_pgid',
        'schema_version',
    }
    for field in sorted(boolean_fields):
        if _boolean(cleanup.get(field), f'cleanup.{field}') is not True:
            raise EvidenceError(f'cleanup.{field} must be true')
    expected_pgids = {
        'interrupted_last_pgid': interrupted_pgid,
        'followup_last_pgid': followup_pgid,
        'probe_last_pgid': probe_pgid,
        'original_child_pgid': original_pgid,
        'replacement_child_pgid': replacement_pgid,
    }
    for field, expected in expected_pgids.items():
        actual = _integer(cleanup.get(field), f'cleanup.{field}', minimum=2)
        if actual != expected:
            raise EvidenceError(f'cleanup.{field} does not match its published PGID')


def capture_installed_package_state(
    lifecycle: Mapping[str, Any],
    upgrade: Mapping[str, Any],
    smoke_script: Path,
) -> dict[str, Any]:
    """Directly re-observe the installed candidate without changing package state."""
    if smoke_script.is_symlink():
        raise EvidenceError(f'installed smoke script must not be a symlink: {smoke_script}')
    smoke_script = smoke_script.resolve(strict=True)
    if not smoke_script.is_file():
        raise EvidenceError(f'invalid installed smoke script: {smoke_script}')
    query = (
        _run_command(
            [
                'dpkg-query',
                '--show',
                '--showformat=${Package}\t${Version}\t${Architecture}',
                'robotest-supervisor',
            ]
        )
        .stdout.strip()
        .split('\t')
    )
    if len(query) != 3:
        raise EvidenceError('installed package metadata is malformed')
    verify = _run_command(['dpkg', '--verify', 'robotest-supervisor'])
    enabled = _run_command(
        ['systemctl', 'is-enabled', EXPECTED_UNIT],
        accepted_returncodes=frozenset({0, 1, 3, 4}),
    ).stdout.strip()
    active = _run_command(
        ['systemctl', 'is-active', EXPECTED_UNIT],
        accepted_returncodes=frozenset({0, 1, 2, 3, 4}),
    ).stdout.strip()
    if enabled != 'disabled':
        raise EvidenceError(f'{EXPECTED_UNIT} is not exactly disabled: {enabled!r}')
    if active != 'inactive':
        raise EvidenceError(f'{EXPECTED_UNIT} is not exactly inactive: {active!r}')
    groups = set(_run_command(['id', '-nG', 'robotest-supervisor']).stdout.split())
    _run_command([str(smoke_script)])
    config = Path('/etc/robotest-supervisor/config.json')
    state_directory = Path('/var/lib/robotest-supervisor')
    log_directory = Path('/var/log/robotest-supervisor')
    binary = Path('/usr/bin/robotest-supervisor')
    helper = Path('/usr/libexec/robotest-supervisor/start-robotest-stack')
    unit = Path('/usr/lib/systemd/system/robotest-supervisor.service')
    report = {
        'architecture': query[2],
        'captured_utc': utc_now(),
        'configuration_mode_owner': _mode_owner(config),
        'configuration_sha256': file_sha256(config),
        'dpkg_verify_clean': not verify.stdout.strip() and not verify.stderr.strip(),
        'installed_binary_sha256': file_sha256(binary),
        'installed_helper_sha256': file_sha256(helper),
        'installed_smoke_contract_passed': True,
        'installed_unit_sha256': file_sha256(unit),
        'log_directory_mode_owner': _mode_owner(log_directory),
        'package_installed': True,
        'package_name': query[0],
        'schema_version': SCHEMA_VERSION,
        'service_account_render_group': 'render' in groups,
        'service_account_video_group': 'video' in groups,
        'service_active': active == 'active',
        'service_enabled': enabled == 'enabled',
        'stale_dpkg_conffile_artifacts': len(list(config.parent.glob('config.json.dpkg-*'))),
        'state_directory_mode_owner': _mode_owner(state_directory),
        'systemd_unit_verified': True,
        'verdict': 'PASS',
        'version': query[1],
    }
    validate_installed_package_state(report, lifecycle, upgrade)
    return report


def append_timeline(path: Path, kind: str, details: Mapping[str, Any]) -> dict[str, Any]:
    """Append one fsynced, sequence-checked timeline observation."""
    if not re.fullmatch(r'[a-z][a-z0-9_]{1,63}', kind):
        raise EvidenceError(f'invalid timeline kind: {kind!r}')
    previous = load_jsonl(path) if path.exists() else []
    sequence = len(previous) + 1
    if sequence > MAX_TIMELINE_ENTRIES:
        raise EvidenceError('timeline is saturated')
    for index, record in enumerate(previous, 1):
        if record.get('sequence') != index:
            raise EvidenceError('existing timeline sequence is not contiguous')
    record = {
        'schema_version': SCHEMA_VERSION,
        'sequence': sequence,
        'timestamp_utc': utc_now(),
        'monotonic_ns': time.monotonic_ns(),
        'kind': kind,
        'details': dict(details),
    }
    payload = canonical_json_bytes(record)
    if path.exists() and path.stat().st_size + len(payload) > MAX_JSON_BYTES:
        raise EvidenceError('timeline byte cap would be exceeded')
    path.parent.mkdir(parents=True, exist_ok=True)
    flags = os.O_WRONLY | os.O_CREAT | os.O_APPEND
    descriptor = os.open(path, flags, 0o640)
    try:
        written = os.write(descriptor, payload)
        if written != len(payload):
            raise EvidenceError('short timeline write')
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return record


def probe_http(url: str, timeout_s: float = 1.0) -> tuple[int, str]:
    """Return a bounded HTTP status/body, including HTTP error responses."""
    request = urllib.request.Request(url, method='GET')
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            body = response.read(65537)
            if len(body) > 65536:
                raise EvidenceError(f'HTTP body exceeded 65536 bytes: {url}')
            return int(response.status), body.decode('utf-8', errors='replace')
    except urllib.error.HTTPError as exc:
        body = exc.read(65537)
        if len(body) > 65536:
            raise EvidenceError(f'HTTP error body exceeded 65536 bytes: {url}') from exc
        return int(exc.code), body.decode('utf-8', errors='replace')
    except (OSError, urllib.error.URLError):
        return 0, ''


def fetch_http_json(url: str, output: Path, expected_status: int = 200) -> None:
    """Fetch one bounded HTTP JSON document and atomically retain it."""
    status, body = probe_http(url)
    if status != expected_status:
        raise EvidenceError(f'{url} returned HTTP {status}, expected {expected_status}')
    try:
        value = json.loads(body)
    except json.JSONDecodeError as exc:
        raise EvidenceError(f'{url} returned invalid JSON: {exc}') from exc
    if not isinstance(value, dict):
        raise EvidenceError(f'{url} did not return a JSON object')
    atomic_write_json(output, value)


def _read_proc_file(path: Path, *, binary: bool = False) -> bytes | str:
    if not path.is_file() or path.is_symlink():
        raise EvidenceError(f'process file is unavailable: {path}')
    if path.stat().st_size > MAX_PROC_FILE_BYTES:
        raise EvidenceError(f'process file exceeds cap: {path}')
    payload = path.read_bytes()
    if len(payload) > MAX_PROC_FILE_BYTES:
        raise EvidenceError(f'process file exceeds cap: {path}')
    if binary:
        return payload
    return payload.decode('utf-8', errors='strict')


def _parse_proc_stat(payload: str) -> dict[str, Any]:
    right = payload.rfind(')')
    left = payload.find('(')
    if left <= 0 or right <= left or not payload.endswith('\n'):
        raise EvidenceError('malformed /proc stat record')
    pid = int(payload[:left].strip())
    rest = payload[right + 2 :].split()
    if len(rest) < 20:
        raise EvidenceError('truncated /proc stat record')
    return {
        'pid': pid,
        'comm': payload[left + 1 : right],
        'state': rest[0],
        'ppid': int(rest[1]),
        'pgid': int(rest[2]),
        'start_time_ticks': int(rest[19]),
    }


def process_table(proc_root: Path = Path('/proc')) -> dict[int, dict[str, Any]]:
    """Read a bounded Linux process table, tolerating concurrent exits."""
    table: dict[int, dict[str, Any]] = {}
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        if len(table) >= MAX_PROCESS_COUNT:
            raise EvidenceError('process table exceeds the process-count cap')
        try:
            stat_value = _parse_proc_stat(_read_proc_file(entry / 'stat'))
        except (EvidenceError, FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        table[stat_value['pid']] = stat_value
    return table


def descendant_pids(table: Mapping[int, Mapping[str, Any]], root_pid: int) -> set[int]:
    """Return the exact transitive descendant PID set including ``root_pid``."""
    if root_pid not in table:
        raise EvidenceError(f'root PID {root_pid} is absent')
    result = {root_pid}
    changed = True
    while changed:
        changed = False
        for pid, record in table.items():
            if pid not in result and record.get('ppid') in result:
                result.add(pid)
                changed = True
    return result


def process_lineage(
    table: Mapping[int, Mapping[str, Any]], target_pid: int, root_pid: int
) -> list[dict[str, int]]:
    """Return the exact root-to-target PID/start-time lineage."""
    if root_pid not in table or target_pid not in table:
        raise EvidenceError('lineage endpoint is absent from the process table')
    reverse: list[dict[str, int]] = []
    current = target_pid
    visited: set[int] = set()
    while True:
        if current in visited:
            raise EvidenceError('process lineage contains a cycle')
        visited.add(current)
        record = table.get(current)
        if record is None:
            raise EvidenceError('process lineage is incomplete')
        reverse.append(
            {
                'pid': _integer(record.get('pid'), 'lineage pid', minimum=1),
                'ppid': _integer(record.get('ppid'), 'lineage ppid', minimum=0),
                'start_time_ticks': _integer(
                    record.get('start_time_ticks'), 'lineage start time', minimum=0
                ),
            }
        )
        if current == root_pid:
            break
        current = int(record.get('ppid', -1))
    reverse.reverse()
    return reverse


def _process_environment(path: Path) -> dict[str, str]:
    payload = _read_proc_file(path / 'environ', binary=True)
    assert isinstance(payload, bytes)
    environment: dict[str, str] = {}
    for item in payload.split(b'\0'):
        if not item or b'=' not in item:
            continue
        raw_key, raw_value = item.split(b'=', 1)
        try:
            key = raw_key.decode('utf-8')
            value = raw_value.decode('utf-8')
        except UnicodeDecodeError:
            continue
        environment[key] = value
    return environment


def _process_details(pid: int, proc_root: Path = Path('/proc')) -> dict[str, Any]:
    root = proc_root / str(pid)
    stat_value = _parse_proc_stat(_read_proc_file(root / 'stat'))
    raw_cmdline = _read_proc_file(root / 'cmdline', binary=True)
    assert isinstance(raw_cmdline, bytes)
    cmdline = [
        item.decode('utf-8', errors='replace')
        for item in raw_cmdline.rstrip(b'\0').split(b'\0')
        if item
    ]
    cgroup = str(_read_proc_file(root / 'cgroup')).splitlines()
    try:
        executable = os.readlink(root / 'exe')
    except OSError:
        executable = ''
    environment = _process_environment(root)
    return {
        **stat_value,
        'cmdline': cmdline,
        'cmdline_sha256': hashlib.sha256(raw_cmdline).hexdigest(),
        'cgroup': cgroup,
        'executable': executable,
        'ros_domain_id': environment.get('ROS_DOMAIN_ID'),
        'gz_partition': environment.get('GZ_PARTITION'),
    }


def _in_unit_cgroup(lines: Sequence[str], unit: str) -> bool:
    suffix = f'/system.slice/{unit}'
    return any(line.endswith(suffix) for line in lines)


def find_exact_controller(
    root_pid: int,
    pgid: int,
    ros_domain_id: int,
    gz_partition: str,
    *,
    unit: str = EXPECTED_UNIT,
    proc_root: Path = Path('/proc'),
) -> dict[str, Any]:
    """Resolve one nested Nav2 controller using lineage, PGID, cgroup, and env."""
    table = process_table(proc_root)
    descendants = descendant_pids(table, root_pid)
    candidates: list[dict[str, Any]] = []
    for pid in sorted(descendants - {root_pid}):
        if table[pid].get('pgid') != pgid or table[pid].get('state') == 'Z':
            continue
        try:
            details = _process_details(pid, proc_root)
        except (FileNotFoundError, ProcessLookupError):
            continue
        except (EvidenceError, PermissionError) as exc:
            if not (proc_root / str(pid)).exists():
                continue
            raise EvidenceError(f'cannot inspect descendant PID {pid}: {exc}') from exc
        if details.get('state') == 'Z' or details.get('pgid') != pgid:
            continue
        executable_name = Path(details['executable']).name
        if executable_name != 'controller_server':
            continue
        if details['ros_domain_id'] != str(ros_domain_id):
            continue
        if details['gz_partition'] != gz_partition:
            continue
        if not _in_unit_cgroup(details['cgroup'], unit):
            continue
        candidates.append(details)
    if len(candidates) != 1:
        raise EvidenceError(f'expected one exact nested controller_server, found {len(candidates)}')
    candidate = candidates[0]
    candidate['root_pid'] = root_pid
    candidate['unit'] = unit
    candidate['lineage'] = process_lineage(table, int(candidate['pid']), root_pid)
    if table[root_pid].get('state') == 'Z' or table[root_pid].get('pgid') != pgid:
        raise EvidenceError('managed root is zombie or left its captured process group')
    return candidate


def assert_process_identity(
    identity: Mapping[str, Any], *, proc_root: Path = Path('/proc')
) -> dict[str, Any]:
    """Re-resolve a captured PID and reject PID reuse or identity drift."""
    pid = _integer(identity.get('pid'), 'identity.pid', minimum=1)
    root_pid = _integer(identity.get('root_pid'), 'identity.root_pid', minimum=1)
    expected_pgid = _integer(identity.get('pgid'), 'identity.pgid', minimum=1)
    if Path(_string(identity.get('executable'), 'identity.executable')).name != (
        'controller_server'
    ):
        raise EvidenceError('captured executable is not exactly controller_server')
    table = process_table(proc_root)
    current_lineage = process_lineage(table, pid, root_pid)
    expected_lineage = [
        dict(item) for item in _mapping_list(identity.get('lineage'), 'identity.lineage')
    ]
    current = _process_details(pid, proc_root)
    fields = (
        'start_time_ticks',
        'pgid',
        'executable',
        'cmdline_sha256',
        'ros_domain_id',
        'gz_partition',
    )
    mismatches = [name for name in fields if current.get(name) != identity.get(name)]
    if current.get('cgroup') != identity.get('cgroup'):
        mismatches.append('cgroup')
    if current_lineage != expected_lineage:
        mismatches.append('lineage')
    if current.get('state') == 'Z':
        mismatches.append('zombie')
    if Path(str(current.get('executable', ''))).name != 'controller_server':
        mismatches.append('executable_basename')
    root = table.get(root_pid)
    if root is None or root.get('state') == 'Z':
        mismatches.append('root_state')
    elif root.get('pgid') != expected_pgid:
        mismatches.append('root_pgid')
    unit = _string(identity.get('unit'), 'identity.unit')
    if not _in_unit_cgroup(current['cgroup'], unit):
        mismatches.append('unit_cgroup')
    if mismatches:
        raise EvidenceError(f'captured process identity changed: {sorted(mismatches)}')
    return current


def signal_process_identity(
    identity: Mapping[str, Any],
    signum: int,
    *,
    proc_root: Path = Path('/proc'),
    pidfd_opener: Any = _DEFAULT_API,
    pidfd_signaler: Any = _DEFAULT_API,
    fd_closer: Any = _DEFAULT_API,
) -> int:
    """Pin, revalidate, and signal the exact task through one pidfd."""
    if signum not in {signal.SIGTERM, signal.SIGKILL}:
        raise EvidenceError(f'unsupported identity signal: {signum}')
    if pidfd_opener is _DEFAULT_API:
        pidfd_opener = getattr(os, 'pidfd_open', None)
    if pidfd_signaler is _DEFAULT_API:
        pidfd_signaler = getattr(signal, 'pidfd_send_signal', None)
    if fd_closer is _DEFAULT_API:
        fd_closer = os.close
    if not callable(pidfd_opener) or not callable(pidfd_signaler) or not callable(fd_closer):
        raise EvidenceError('pidfd signaling APIs are unavailable; numeric signaling is forbidden')
    pid = _integer(identity.get('pid'), 'identity.pid', minimum=1)
    try:
        descriptor = pidfd_opener(pid, 0)
    except (OSError, TypeError, ValueError) as exc:
        raise EvidenceError(f'could not open pidfd for exact PID {pid}: {exc}') from exc
    descriptor = _integer(descriptor, 'pidfd descriptor', minimum=0)
    try:
        current = assert_process_identity(identity, proc_root=proc_root)
        if _integer(current.get('pid'), 'current identity PID', minimum=1) != pid:
            raise EvidenceError('pidfd-pinned task does not match the captured PID')
        try:
            pidfd_signaler(descriptor, signum, None, 0)
        except (OSError, TypeError, ValueError) as exc:
            raise EvidenceError(f'could not signal pidfd for exact PID {pid}: {exc}') from exc
    finally:
        try:
            fd_closer(descriptor)
        except (OSError, TypeError, ValueError) as exc:
            raise EvidenceError(f'could not close pidfd for exact PID {pid}: {exc}') from exc
    return pid


def process_group_members(pgid: int, proc_root: Path = Path('/proc')) -> list[dict[str, Any]]:
    """Return all members of an exact Linux process group, including zombies."""
    if pgid <= 0:
        raise EvidenceError('PGID must be positive')
    return [
        dict(record)
        for _pid, record in sorted(process_table(proc_root).items())
        if record.get('pgid') == pgid
    ]


def systemd_owned_units(
    ros_domain_id: int,
    gz_partition: str,
    proc_root: Path = Path('/proc'),
) -> dict[str, list[int]]:
    """Map matching isolated processes to exact system-slice service units."""
    units: dict[str, list[int]] = {}
    unit_pattern = re.compile(r'/system\.slice/([^/]+\.service)$')
    inspected = 0
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        inspected += 1
        if inspected > MAX_PROCESS_COUNT:
            raise EvidenceError('process table exceeds the systemd-owner scan cap')
        try:
            environment = _process_environment(entry)
            if (
                environment.get('ROS_DOMAIN_ID') != str(ros_domain_id)
                or environment.get('GZ_PARTITION') != gz_partition
            ):
                continue
            cgroup = str(_read_proc_file(entry / 'cgroup')).splitlines()
        except (FileNotFoundError, ProcessLookupError):
            continue
        except (EvidenceError, PermissionError) as exc:
            if not entry.exists():
                continue
            raise EvidenceError(
                f'cannot inspect PID {entry.name} for systemd ownership: {exc}'
            ) from exc
        for line in cgroup:
            match = unit_pattern.search(line)
            if match is not None:
                units.setdefault(match.group(1), []).append(int(entry.name))
    return {unit: sorted(set(pids)) for unit, pids in sorted(units.items())}


def allocated_isolation(run_id: str, proc_root: Path = Path('/proc')) -> dict[str, Any]:
    """Select an unused bounded ROS domain and unique Gazebo partition."""
    if RUN_ID_RE.fullmatch(run_id) is None:
        raise EvidenceError('run ID does not match the Phase 4 format')
    used_domains: set[int] = set()
    used_partitions: set[str] = set()
    inspected = 0
    unreadable = 0
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        inspected += 1
        if inspected > MAX_PROCESS_COUNT:
            raise EvidenceError('process table exceeds the isolation-scan cap')
        try:
            environment = _process_environment(entry)
        except (FileNotFoundError, ProcessLookupError):
            continue
        except (EvidenceError, PermissionError) as exc:
            if not entry.exists():
                continue
            unreadable += 1
            raise EvidenceError(
                f'cannot prove isolation because PID {entry.name} is unreadable: {exc}'
            ) from exc
        raw_domain = environment.get('ROS_DOMAIN_ID')
        if raw_domain is not None and raw_domain.isdigit():
            used_domains.add(int(raw_domain))
        partition = environment.get('GZ_PARTITION')
        if partition:
            used_partitions.add(partition)
    seed = int(hashlib.sha256(run_id.encode()).hexdigest()[:8], 16)
    candidates = list(range(100, 230))
    candidates = candidates[seed % len(candidates) :] + candidates[: seed % len(candidates)]
    try:
        domain = next(value for value in candidates if value not in used_domains)
    except StopIteration as exc:
        raise EvidenceError('no unused ROS domain is available in 100..229') from exc
    partition = f'robotest_phase4_{run_id.replace("-", "_")}'
    if PARTITION_RE.fullmatch(partition) is None or partition in used_partitions:
        raise EvidenceError('generated Gazebo partition is invalid or already active')
    return {
        'schema_version': SCHEMA_VERSION,
        'run_id': run_id,
        'ros_domain_id': domain,
        'gz_partition': partition,
        'inspected_processes': inspected,
        'unreadable_process_environments': unreadable,
        'domain_was_unused': True,
        'partition_was_unused': True,
    }


def render_supervisor_config(
    template_path: Path,
    output_path: Path,
    state_directory: str,
    ros_domain_id: int,
    gz_partition: str,
) -> dict[str, Any]:
    """Render only run-scoped isolation/state values into the frozen config."""
    value = _mapping(load_json(template_path), 'supervisor template')
    frozen = {
        'listen_address': '127.0.0.1:9080',
        'heartbeat_poll_ms': 500,
        'heartbeat_stale_ms': 2000,
        'heartbeat_startup_timeout_ms': 110000,
        'termination_grace_ms': 5000,
        'shutdown_timeout_ms': 15000,
    }
    for key, expected in frozen.items():
        if value.get(key) != expected:
            raise EvidenceError(f'package config changed frozen {key}: {value.get(key)!r}')
    restart = _mapping(value.get('restart'), 'supervisor template.restart')
    expected_restart = {
        'initial_backoff_ms': 1000,
        'maximum_backoff_ms': 8000,
        'maximum_attempts': 4,
        'window_ms': 60000,
        'stable_reset_ms': 60000,
    }
    if dict(restart) != expected_restart:
        raise EvidenceError('package config changed the frozen restart policy')
    children = value.get('children')
    if not isinstance(children, list) or len(children) != 1:
        raise EvidenceError('package config must contain exactly one managed child')
    child = _mapping(children[0], 'supervisor template child')
    if (
        child.get('name') != EXPECTED_CHILD
        or child.get('argv') != ['/usr/libexec/robotest-supervisor/start-robotest-stack']
        or child.get('working_directory') != '/opt/robotest-lab'
        or child.get('required') is not True
        or child.get('heartbeat_file') != '/var/lib/robotest-supervisor/robotest-stack.heartbeat'
    ):
        raise EvidenceError('package config changed the managed-child contract')
    if not state_directory.startswith('/var/lib/robotest-supervisor/phase4-'):
        raise EvidenceError('run state directory is outside the Phase 4-owned prefix')
    if not 1 <= ros_domain_id <= 232:
        raise EvidenceError('ROS domain ID is outside the DDS range')
    if PARTITION_RE.fullmatch(gz_partition) is None:
        raise EvidenceError('Gazebo partition is invalid')
    rendered = json.loads(json.dumps(value))
    rendered['state_directory'] = state_directory
    environment = rendered['children'][0]['environment']
    environment['ROS_DOMAIN_ID'] = str(ros_domain_id)
    environment['GZ_PARTITION'] = gz_partition
    atomic_write_json(output_path, rendered, mode=0o640)
    return rendered


def render_followup_mission(output_path: Path) -> dict[str, Any]:
    """Write the one-goal fresh recovery mission as immutable canonical JSON."""
    mission = {
        'schema_version': 1,
        'mission_name': 'phase4_recovery_followup',
        'mission_seed': 42,
        'simulator_seed': 42,
        'frame_id': 'map',
        'start_pose': {'x': 0.0, 'y': -3.5, 'yaw': 0.0},
        'waypoints': [{'x': -0.5, 'y': -3.5, 'yaw': 0.0}],
        'mission_timeout_sim_s': 60.0,
        'wall_escape_timeout_s': 120.0,
        'allowed_collision_count': 0,
        'fault_schedule': None,
        'fault_seed': None,
        'expected_outcome': 'succeeded',
        'retries': 0,
    }
    atomic_write_json(output_path, mission)
    return mission


def _manifest_source_mappings(repository: Path) -> tuple[tuple[Path, Path], ...]:
    return (
        (repository / 'supervisor/cmd', Path('cmd')),
        (repository / 'supervisor/internal', Path('internal')),
        (repository / 'supervisor/go.mod', Path('go.mod')),
        (repository / 'supervisor/README.md', Path('README.md')),
        (repository / 'supervisor/config.example.json', Path('config.example.json')),
        (repository / 'LICENSE', Path('LICENSE')),
        (repository / 'packaging/debian', Path('debian')),
    )


def repository_package_manifest(repository: Path) -> dict[str, Any]:
    """Rebuild the package builder's exact source manifest."""
    entries: list[dict[str, Any]] = []
    for source, destination in _manifest_source_mappings(repository):
        if source.is_dir() and not source.is_symlink():
            paths = sorted(path for path in source.rglob('*') if path.is_file())
            if any(path.is_symlink() for path in source.rglob('*')):
                raise EvidenceError(f'package source contains a symbolic link: {source}')
            for path in paths:
                metadata = path.stat()
                entries.append(
                    {
                        'mode': format(stat.S_IMODE(metadata.st_mode), '04o'),
                        'path': (destination / path.relative_to(source)).as_posix(),
                        'sha256': file_sha256(path),
                        'size_bytes': metadata.st_size,
                    }
                )
        else:
            if source.is_symlink() or not source.is_file():
                raise EvidenceError(f'missing package source: {source}')
            metadata = source.stat()
            entries.append(
                {
                    'mode': format(stat.S_IMODE(metadata.st_mode), '04o'),
                    'path': destination.as_posix(),
                    'sha256': file_sha256(source),
                    'size_bytes': metadata.st_size,
                }
            )
    return {'schema_version': 1, 'files': sorted(entries, key=lambda item: item['path'])}


def source_snapshot(repository: Path) -> dict[str, Any]:
    """Hash the bounded implementation/contract source set used by Phase 4."""
    roots = ('config', 'docs', 'packaging', 'scenarios', 'scripts', 'src', 'supervisor', 'tests')
    ignored_parts = {
        '.git',
        '.mypy_cache',
        '.pytest_cache',
        '__pycache__',
        'artifacts',
        'build',
        'install',
        'log',
    }
    entries: list[dict[str, Any]] = []
    for name in roots:
        root = repository / name
        if not root.exists():
            raise EvidenceError(f'source snapshot root is missing: {root}')
        candidates = [root] if root.is_file() else sorted(root.rglob('*'))
        for path in candidates:
            relative = path.relative_to(repository)
            if any(part in ignored_parts for part in relative.parts):
                continue
            if path.is_symlink():
                raise EvidenceError(f'source snapshot contains a symlink: {relative}')
            if not path.is_file():
                continue
            entries.append(
                {
                    'path': relative.as_posix(),
                    'sha256': file_sha256(path),
                    'size_bytes': path.stat().st_size,
                }
            )
            if len(entries) > MAX_SOURCE_FILES:
                raise EvidenceError('source snapshot exceeds its file-count cap')
    snapshot_sha = hashlib.sha256(canonical_json_bytes(entries)).hexdigest()
    return {
        'schema_version': 1,
        'repository': str(repository.resolve()),
        'file_count': len(entries),
        'snapshot_sha256': snapshot_sha,
        'files': entries,
    }


def package_source_binding(repository: Path, manifest_path: Path) -> dict[str, Any]:
    """Compare a package candidate's source manifest to the live source bytes."""
    expected = load_json(manifest_path)
    actual = repository_package_manifest(repository)
    expected_document = _mapping(expected, 'candidate source manifest')
    expected_files = {
        _string(item.get('path'), 'manifest path'): item
        for item in _mapping_list(expected_document.get('files'), 'candidate source manifest files')
    }
    actual_files = {item['path']: item for item in actual['files']}
    missing = sorted(set(expected_files) - set(actual_files))
    extra = sorted(set(actual_files) - set(expected_files))
    mismatched = sorted(
        path
        for path in set(expected_files) & set(actual_files)
        if expected_files[path] != actual_files[path]
    )
    passed = (
        not missing
        and not extra
        and not mismatched
        and expected_document.get('schema_version') == 1
    )
    return {
        'schema_version': SCHEMA_VERSION,
        'verdict': 'PASS' if passed else 'FAIL',
        'candidate_manifest': str(manifest_path.resolve()),
        'candidate_manifest_sha256': file_sha256(manifest_path),
        'repository_manifest_sha256': hashlib.sha256(canonical_json_bytes(actual)).hexdigest(),
        'missing_paths': missing,
        'extra_paths': extra,
        'mismatched_paths': mismatched,
        'file_count': len(actual_files),
    }


def _mapping_list(value: Any, name: str) -> list[Mapping[str, Any]]:
    if not isinstance(value, list) or not all(isinstance(item, Mapping) for item in value):
        raise EvidenceError(f'{name} must be a list of objects')
    return value


def validate_mission_terminal_verdict(result: Mapping[str, Any], name: str) -> int:
    """Validate the canonical terminal/status/verdict relationship."""
    measurements = _mapping(result.get('measurements'), f'{name}.measurements')
    quality = _mapping(result.get('quality'), f'{name}.quality')
    verdict = _mapping(result.get('verdict'), f'{name}.verdict')
    exit_code = _integer(verdict.get('exit_code'), f'{name}.verdict.exit_code', minimum=0)
    expected_outcome_met = _boolean(
        verdict.get('expected_outcome_met'), f'{name}.verdict.expected_outcome_met'
    )
    phase_status = _string(
        verdict.get('phase2_action_integration_status'),
        f'{name}.verdict.phase2_action_integration_status',
        maximum=16,
    )
    if phase_status not in {'PASS', 'FAIL'}:
        raise EvidenceError(f'{name} has an invalid Phase 2 integration status')
    reason = _string(verdict.get('reason'), f'{name}.verdict.reason')
    mission_status = _string(verdict.get('mission_status'), f'{name}.verdict.mission_status')
    terminal_observed = _boolean(
        quality.get('terminal_result_observed'), f'{name}.quality.terminal_result_observed'
    )
    goal_accepted = _boolean(quality.get('goal_accepted'), f'{name}.quality.goal_accepted')
    missed_count = _integer(
        measurements.get('missed_waypoint_count'),
        f'{name}.measurements.missed_waypoint_count',
        minimum=0,
    )

    goal_status_code = measurements.get('goal_status_code')
    goal_status = measurements.get('goal_status')
    terminal_stamp = measurements.get('terminal_action_stamp_ns')
    nav2_error_code = measurements.get('nav2_error_code')
    nav2_error_message = measurements.get('nav2_error_message')
    status_names = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
    if terminal_observed:
        status_code = _integer(
            goal_status_code,
            f'{name}.measurements.goal_status_code',
            minimum=0,
        )
        if status_code not in status_names or goal_status != status_names[status_code]:
            raise EvidenceError(f'{name} terminal goal status is inconsistent')
        _integer(
            terminal_stamp,
            f'{name}.measurements.terminal_action_stamp_ns',
            minimum=0,
        )
        _integer(nav2_error_code, f'{name}.measurements.nav2_error_code')
        if not isinstance(nav2_error_message, str):
            raise EvidenceError(f'{name}.measurements.nav2_error_message must be a string')
        if not goal_accepted:
            raise EvidenceError(f'{name} observed a terminal result without an accepted goal')
        expected_mission_status = status_names[status_code]
    else:
        if any(
            value is not None
            for value in (
                goal_status_code,
                goal_status,
                terminal_stamp,
                measurements.get('terminal_action_stamp'),
                nav2_error_code,
                nav2_error_message,
            )
        ):
            raise EvidenceError(f'{name} has terminal fields without a terminal result')
        expected_mission_status = reason.upper()

    succeeded = exit_code == 0
    if expected_outcome_met is not succeeded:
        raise EvidenceError(f'{name} expected-outcome verdict disagrees with exit code')
    if phase_status != ('PASS' if succeeded else 'FAIL'):
        raise EvidenceError(f'{name} Phase 2 verdict disagrees with exit code')
    if mission_status != expected_mission_status:
        raise EvidenceError(f'{name} mission status disagrees with terminal status')
    if succeeded and not (
        terminal_observed and goal_status_code == 4 and nav2_error_code == 0 and missed_count == 0
    ):
        raise EvidenceError(f'{name} successful verdict lacks a successful terminal result')
    if terminal_observed and goal_status_code == 4 and not succeeded:
        raise EvidenceError(f'{name} reports SUCCEEDED with a nonzero exit code')
    if exit_code == 22 and goal_status_code != 6:
        raise EvidenceError(f'{name} GOAL_ABORTED exit lacks ABORTED terminal status')
    if exit_code == 23 and goal_status_code != 5:
        raise EvidenceError(f'{name} GOAL_CANCELED exit lacks CANCELED terminal status')
    return exit_code


def mission_pair(path_json: Path, path_csv: Path) -> tuple[Mapping[str, Any], bool]:
    """Validate canonical mission JSON against its normative one-row CSV."""
    result = _mapping(load_json(path_json), 'mission result')
    validate_mission_terminal_verdict(result, 'mission result')
    repository = Path(__file__).resolve().parents[1]
    mission_source = repository / 'src/robotest_missions'
    sys.path.insert(0, str(mission_source))
    try:
        from robotest_missions.artifacts import flatten_result
    except ImportError as exc:
        raise EvidenceError(f'cannot import canonical mission projection: {exc}') from exc
    expected = flatten_result(result)
    if path_csv.is_symlink() or not path_csv.is_file():
        raise EvidenceError(f'missing regular mission CSV: {path_csv}')
    if path_csv.stat().st_size > 1024 * 1024:
        raise EvidenceError('mission CSV exceeds the byte cap')
    try:
        with path_csv.open(newline='', encoding='utf-8') as source:
            reader = csv.DictReader(source)
            rows = list(reader)
    except (OSError, UnicodeError, csv.Error) as exc:
        raise EvidenceError(f'cannot parse mission CSV: {exc}') from exc
    if len(rows) != 1 or tuple(reader.fieldnames or ()) != tuple(expected):
        raise EvidenceError('mission CSV shape does not match the canonical projection')
    return result, rows[0] == expected


def _timeline_index(records: Sequence[Mapping[str, Any]]) -> dict[str, list[Mapping[str, Any]]]:
    index: dict[str, list[Mapping[str, Any]]] = {}
    previous_monotonic = -1
    for sequence, record in enumerate(records, 1):
        if record.get('schema_version') != 1 or record.get('sequence') != sequence:
            raise EvidenceError('timeline schema or sequence is invalid')
        monotonic = _integer(record.get('monotonic_ns'), 'timeline.monotonic_ns', minimum=0)
        if monotonic < previous_monotonic:
            raise EvidenceError('timeline monotonic time regressed')
        previous_monotonic = monotonic
        kind = _string(record.get('kind'), 'timeline.kind')
        _mapping(record.get('details'), 'timeline.details')
        index.setdefault(kind, []).append(record)
    return index


def _one(index: Mapping[str, list[Mapping[str, Any]]], kind: str) -> Mapping[str, Any]:
    values = index.get(kind, [])
    if len(values) != 1:
        raise EvidenceError(f'timeline requires exactly one {kind}, found {len(values)}')
    return values[0]


def _duration_seconds(start: Mapping[str, Any], end: Mapping[str, Any]) -> float:
    start_ns = _integer(start.get('monotonic_ns'), 'start monotonic', minimum=0)
    end_ns = _integer(end.get('monotonic_ns'), 'end monotonic', minimum=0)
    if end_ns < start_ns:
        raise EvidenceError('duration endpoint precedes its start')
    return (end_ns - start_ns) / 1_000_000_000


def _event_sequence(events: Sequence[Mapping[str, Any]]) -> None:
    previous_steady = -1
    for index, event in enumerate(events, 1):
        if event.get('schema_version') != 1 or event.get('sequence') != index:
            raise EvidenceError('supervisor events are not contiguous from sequence one')
        steady = _integer(event.get('steady_wall_ns'), 'event steady_wall_ns', minimum=0)
        if steady < previous_steady:
            raise EvidenceError('supervisor event steady time regressed')
        previous_steady = steady
        _string(event.get('kind'), 'event kind')


def _add_check(checks: dict[str, bool], failures: list[str], name: str, condition: bool) -> None:
    checks[name] = bool(condition)
    if not condition:
        failures.append(name)


def _relative_evidence_hashes(run_directory: Path, excluded: set[str]) -> dict[str, str]:
    files = [path for path in run_directory.rglob('*') if path.is_file()]
    if len(files) > MAX_EVIDENCE_FILES:
        raise EvidenceError('run evidence exceeds the file-count cap')
    if any(path.is_symlink() for path in run_directory.rglob('*')):
        raise EvidenceError('run evidence must not contain symbolic links')
    return {
        path.relative_to(run_directory).as_posix(): file_sha256(path)
        for path in sorted(files)
        if path.relative_to(run_directory).as_posix() not in excluded
    }


def evaluate_run(run_directory: Path) -> dict[str, Any]:
    """Evaluate all frozen Scenario 6 gates from retained raw evidence."""
    context = _mapping(load_json(run_directory / 'context.json'), 'context')
    if context.get('schema_version') != 1:
        raise EvidenceError('context schema is unsupported')
    run_id = _string(context.get('run_id'), 'context.run_id')
    timeline = load_jsonl(run_directory / 'timeline.jsonl')
    index = _timeline_index(timeline)
    initial = _one(index, 'initial_ready')
    mission_started = _one(index, 'mission_started')
    probe_started = _one(index, 'active_goal_probe_started')
    injection = _one(index, 'failure_injected')
    unavailable = _one(index, 'ready_unavailable')
    group_empty = _one(index, 'original_group_empty')
    restored = _one(index, 'ready_restored')
    interrupted_finished = _one(index, 'interrupted_mission_finished')
    followup_started = _one(index, 'followup_started')
    followup_finished = _one(index, 'followup_finished')
    service_stopped = _one(index, 'service_stopped')
    cleanup_event = _one(index, 'cleanup_complete')
    if not (
        int(initial['monotonic_ns']) < int(mission_started['monotonic_ns'])
        and int(mission_started['monotonic_ns']) < int(probe_started['monotonic_ns'])
        and int(probe_started['monotonic_ns']) < int(injection['monotonic_ns'])
        and int(injection['monotonic_ns']) < int(unavailable['monotonic_ns'])
        and int(injection['monotonic_ns']) < int(group_empty['monotonic_ns'])
        and int(unavailable['monotonic_ns']) < int(restored['monotonic_ns'])
        and int(group_empty['monotonic_ns']) < int(restored['monotonic_ns'])
        and int(restored['monotonic_ns']) < int(interrupted_finished['monotonic_ns'])
        and int(interrupted_finished['monotonic_ns']) < int(followup_started['monotonic_ns'])
        and int(followup_started['monotonic_ns']) < int(followup_finished['monotonic_ns'])
        and int(followup_finished['monotonic_ns']) < int(service_stopped['monotonic_ns'])
        and int(service_stopped['monotonic_ns']) < int(cleanup_event['monotonic_ns'])
    ):
        raise EvidenceError('Scenario 6 timeline causal order is invalid')

    initial_details = _mapping(initial['details'], 'initial_ready.details')
    injection_details = _mapping(injection['details'], 'failure_injected.details')
    restored_details = _mapping(restored['details'], 'ready_restored.details')
    mission_started_details = _mapping(mission_started['details'], 'mission_started.details')
    probe_started_details = _mapping(probe_started['details'], 'active_goal_probe_started.details')
    followup_started_details = _mapping(followup_started['details'], 'followup_started.details')
    service_stopped_details = _mapping(service_stopped['details'], 'service_stopped.details')
    original_pgid = _integer(initial_details.get('child_pgid'), 'initial child PGID', minimum=1)
    original_child_pid = _integer(initial_details.get('child_pid'), 'initial child PID', minimum=1)
    main_pid = _integer(initial_details.get('main_pid'), 'initial supervisor PID', minimum=1)
    baseline_sequence = _integer(
        injection_details.get('event_sequence_before'),
        'event sequence before injection',
        minimum=1,
    )
    recovery_end_sequence = _integer(
        restored_details.get('event_sequence_after_recovery'),
        'event sequence after recovery',
        minimum=baseline_sequence + 1,
    )
    observation_end_sequence = _integer(
        service_stopped_details.get('event_sequence_before_stop'),
        'event sequence before service stop',
        minimum=recovery_end_sequence,
    )
    if injection_details.get('original_pgid') != original_pgid:
        raise EvidenceError('injection PGID does not match initial status')

    target = _mapping(load_json(run_directory / 'controller-target.json'), 'controller target')
    target_lineage = _mapping_list(target.get('lineage'), 'controller target lineage')
    active_goal = _mapping(load_json(run_directory / 'active-goal.json'), 'active goal')
    events = load_jsonl(run_directory / 'supervisor-events.jsonl')
    _event_sequence(events)
    event_meta = _mapping(
        load_json(run_directory / 'supervisor-events.meta.json'), 'event metadata'
    )
    if observation_end_sequence > len(events):
        raise EvidenceError('pre-stop event sequence exceeds retained supervisor events')
    after = [
        event
        for event in events
        if baseline_sequence < int(event['sequence']) <= observation_end_sequence
    ]
    failures_detected = [
        event
        for event in after
        if event.get('kind') == 'failure_detected' and event.get('child') == EXPECTED_CHILD
    ]
    restart_events = [
        event
        for event in after
        if event.get('kind') == 'restart_scheduled' and event.get('child') == EXPECTED_CHILD
    ]
    child_starts = [
        event
        for event in after
        if event.get('kind') == 'child_started' and event.get('child') == EXPECTED_CHILD
    ]
    ready_false = [
        event
        for event in after
        if event.get('kind') == 'readiness_changed' and event.get('ready') is False
    ]
    ready_true = [
        event
        for event in after
        if event.get('kind') == 'readiness_changed' and event.get('ready') is True
    ]
    if len(ready_true) == 1 and int(ready_true[0]['sequence']) > recovery_end_sequence:
        raise EvidenceError('ready transition occurs after the recorded recovery bound')
    transition_kinds = {
        'failure_detected',
        'restart_scheduled',
        'child_started',
        'readiness_changed',
    }
    transition_chain = [event for event in after if event.get('kind') in transition_kinds]
    causal_chain_valid = False
    if [event.get('kind') for event in transition_chain] == [
        'failure_detected',
        'readiness_changed',
        'restart_scheduled',
        'child_started',
        'readiness_changed',
    ]:
        failure_event, false_event, restart_event, start_event, true_event = transition_chain
        chain_times = [int(event['steady_wall_ns']) for event in transition_chain]
        failure_exit = failure_event.get('exit_code')
        replacement_pid = restored_details.get('child_pid')
        replacement_pgid = restored_details.get('child_pgid')
        causal_chain_valid = (
            failure_event.get('child') == EXPECTED_CHILD
            and failure_event.get('pid') == original_child_pid
            and failure_event.get('pgid') == original_pgid
            and failure_event.get('failure_kind') == 'unexpected_exit'
            and isinstance(failure_exit, int)
            and not isinstance(failure_exit, bool)
            and failure_exit != 0
            and false_event.get('ready') is False
            and false_event.get('child') is None
            and restart_event.get('child') == EXPECTED_CHILD
            and restart_event.get('restart_attempt') == 1
            and restart_event.get('backoff_ms') == EXPECTED_BACKOFF_MS
            and restart_event.get('pid') is None
            and restart_event.get('pgid') is None
            and start_event.get('child') == EXPECTED_CHILD
            and start_event.get('pid') == replacement_pid
            and start_event.get('pgid') == replacement_pgid
            and isinstance(replacement_pid, int)
            and not isinstance(replacement_pid, bool)
            and replacement_pid > 1
            and replacement_pid == replacement_pgid
            and replacement_pgid != original_pgid
            and true_event.get('ready') is True
            and true_event.get('child') is None
            and all(left < right for left, right in pairwise(chain_times))
        )

    detection_s = _duration_seconds(injection, unavailable)
    group_cleanup_s = _duration_seconds(injection, group_empty)
    observed_recovery_s = _duration_seconds(unavailable, restored)
    event_recovery_s: float | None = None
    actual_backoff_s: float | None = None
    if len(failures_detected) == len(restart_events) == len(child_starts) == 1 and ready_true:
        event_recovery_s = (
            int(ready_true[0]['steady_wall_ns']) - int(failures_detected[0]['steady_wall_ns'])
        ) / 1_000_000_000
        actual_backoff_s = (
            int(child_starts[0]['steady_wall_ns']) - int(restart_events[0]['steady_wall_ns'])
        ) / 1_000_000_000

    health_samples = index.get('health_probe', [])
    ready_samples = index.get('ready_probe', [])
    unavailable_ns = int(unavailable['monotonic_ns'])
    restored_ns = int(restored['monotonic_ns'])
    unavailable_samples = [
        sample
        for sample in ready_samples
        if unavailable_ns <= int(sample['monotonic_ns']) < restored_ns
    ]
    unavailable_statuses = [
        _mapping(sample['details'], 'ready sample').get('http_status')
        for sample in unavailable_samples
    ]
    readiness_interval_valid = bool(unavailable_statuses) and (
        all(status == 503 for status in unavailable_statuses)
        or (
            unavailable_statuses[-1] == 200
            and all(status == 503 for status in unavailable_statuses[:-1])
        )
    )

    interrupted = _mapping(
        load_json(run_directory / 'interrupted-outcome.json'), 'interrupted outcome'
    )
    _exact_keys(interrupted, INTERRUPTED_OUTCOME_KEYS, 'interrupted outcome')
    if _integer(interrupted.get('schema_version'), 'interrupted schema_version') != 1:
        raise EvidenceError('interrupted outcome schema_version must be 1')
    _utc_timestamp(interrupted.get('captured_utc'), 'interrupted captured_utc')
    interrupted_exit = _integer(interrupted.get('process_exit_code'), 'interrupted exit')
    interrupted_json_present = _boolean(interrupted.get('json_present'), 'interrupted json_present')
    interrupted_csv_present = _boolean(interrupted.get('csv_present'), 'interrupted csv_present')
    interrupted_terminated = _boolean(
        interrupted.get('terminated_by_harness'), 'interrupted terminated_by_harness'
    )
    interrupted_group_empty = _boolean(
        interrupted.get('process_group_empty'), 'interrupted process_group_empty'
    )
    interrupted_bounded = _boolean(interrupted.get('bounded_wait'), 'interrupted bounded_wait')
    interrupted_event_details = _mapping(
        interrupted_finished.get('details'), 'interrupted_mission_finished.details'
    )
    _exact_keys(
        interrupted_event_details,
        frozenset({'exit_code', 'process_group_empty', 'terminated_by_harness'}),
        'interrupted_mission_finished.details',
    )
    interrupted_event_exit = _integer(
        interrupted_event_details.get('exit_code'), 'interrupted completion exit_code'
    )
    interrupted_event_terminated = _boolean(
        interrupted_event_details.get('terminated_by_harness'),
        'interrupted completion terminated_by_harness',
    )
    interrupted_event_group_empty = _boolean(
        interrupted_event_details.get('process_group_empty'),
        'interrupted completion process_group_empty',
    )
    interrupted_json_path = run_directory / 'interrupted-result.json'
    interrupted_csv_path = run_directory / 'interrupted-result.csv'
    actual_interrupted_json = (
        interrupted_json_path.is_file() and not interrupted_json_path.is_symlink()
    )
    actual_interrupted_csv = (
        interrupted_csv_path.is_file() and not interrupted_csv_path.is_symlink()
    )
    interrupted_success = False
    interrupted_pair_valid = (
        interrupted_json_present is actual_interrupted_json
        and interrupted_csv_present is actual_interrupted_csv
        and not interrupted_json_present
        and not interrupted_csv_present
    )
    if interrupted_json_present or interrupted_csv_present:
        if interrupted_json_present and interrupted_csv_present:
            interrupted_result, interrupted_pair_valid = mission_pair(
                interrupted_json_path,
                interrupted_csv_path,
            )
            interrupted_verdict = _mapping(
                interrupted_result.get('verdict'), 'interrupted mission verdict'
            )
            interrupted_artifact_exit = _integer(
                interrupted_verdict.get('exit_code'),
                'interrupted artifact verdict.exit_code',
                minimum=0,
            )
            interrupted_pair_valid = interrupted_pair_valid and (
                interrupted_artifact_exit == interrupted_exit == interrupted_event_exit
            )
            interrupted_success = (
                interrupted_artifact_exit == 0
                or interrupted_verdict.get('phase2_action_integration_status') == 'PASS'
                or interrupted_verdict.get('expected_outcome_met') is True
            )
        else:
            interrupted_pair_valid = False

    followup, followup_csv_valid = mission_pair(
        run_directory / 'followup-result.json', run_directory / 'followup-result.csv'
    )
    followup_identity = _mapping(followup.get('identity'), 'followup identity')
    followup_verdict = _mapping(followup.get('verdict'), 'followup verdict')
    followup_measurements = _mapping(followup.get('measurements'), 'followup measurements')
    followup_mission_sha = file_sha256(run_directory / 'followup-mission.json')
    followup_event_details = _mapping(followup_finished.get('details'), 'followup_finished.details')
    _exact_keys(
        followup_event_details,
        frozenset({'exit_code'}),
        'followup_finished.details',
    )
    followup_event_exit = _integer(
        followup_event_details.get('exit_code'), 'followup completion exit_code'
    )

    restored_status = _mapping(
        load_json(run_directory / 'ready-restored-status.json'), 'restored status'
    )
    restored_children = _mapping_list(restored_status.get('children'), 'restored children')
    service_final = _mapping(load_json(run_directory / 'service-final.json'), 'service final')
    cleanup = _mapping(load_json(run_directory / 'cleanup.json'), 'cleanup')
    interrupted_pgid = _integer(
        mission_started_details.get('pgid'), 'interrupted mission PGID', minimum=2
    )
    probe_pgid = _integer(probe_started_details.get('pgid'), 'active goal probe PGID', minimum=2)
    followup_pgid = _integer(
        followup_started_details.get('pgid'), 'followup mission PGID', minimum=2
    )
    validate_cleanup_evidence(
        cleanup,
        interrupted_pgid=interrupted_pgid,
        followup_pgid=followup_pgid,
        probe_pgid=probe_pgid,
        original_pgid=original_pgid,
        replacement_pgid=_integer(
            restored_details.get('child_pgid'), 'replacement child PGID', minimum=2
        ),
    )
    package_binding = _mapping(load_json(run_directory / 'package-binding.json'), 'package binding')
    staging = _mapping(load_json(run_directory / 'runtime-staging.json'), 'runtime staging')
    overlay_provenance = _mapping(
        load_json(run_directory / 'overlay-provenance.json'), 'overlay provenance'
    )
    overlay_source_manifest_path = run_directory / 'overlay-source-manifest.json'
    overlay_install_manifest_path = run_directory / 'overlay-install-manifest.json'
    load_json(overlay_source_manifest_path)
    load_json(overlay_install_manifest_path)
    lifecycle = _mapping(load_json(run_directory / 'package-lifecycle.json'), 'package lifecycle')
    isolation = _mapping(context.get('isolation'), 'context.isolation')
    supervisor_config = _mapping(
        load_json(run_directory / 'supervisor-config.json'), 'supervisor config used'
    )
    config_children = _mapping_list(supervisor_config.get('children'), 'supervisor config children')
    source_before = _mapping(
        load_json(run_directory / 'source-snapshot-before.json'), 'source snapshot before'
    )
    source_after = _mapping(
        load_json(run_directory / 'source-snapshot-after.json'), 'source snapshot after'
    )
    before_files = source_before.get('files')
    after_files = source_after.get('files')
    before_snapshot_hash = hashlib.sha256(canonical_json_bytes(before_files)).hexdigest()
    after_snapshot_hash = hashlib.sha256(canonical_json_bytes(after_files)).hexdigest()
    repository = Path(__file__).resolve().parents[1]
    rebound_package = package_source_binding(
        repository,
        Path(_string(package_binding.get('candidate_manifest'), 'candidate manifest')),
    )
    upgrade_context = _mapping(context.get('upgrade_package'), 'context upgrade package')
    baseline_context = _mapping(context.get('baseline_package'), 'context baseline package')
    lifecycle_context = _mapping(context.get('lifecycle_evidence'), 'context lifecycle evidence')
    upgrade_path = Path(_string(upgrade_context.get('path'), 'upgrade path'))
    baseline_path = Path(_string(baseline_context.get('path'), 'baseline path'))
    package_directory = Path(_string(context.get('package_directory'), 'context package_directory'))
    package_integrity = _mapping(
        load_json(run_directory / 'package-integrity.json'), 'package integrity'
    )
    _exact_keys(package_integrity, PACKAGE_INTEGRITY_KEYS, 'package integrity')
    rebound_integrity = verify_package_candidate(
        package_directory,
        upgrade_path,
        baseline_path,
    )
    upgrade_descriptor = package_artifact_descriptor(upgrade_path)
    baseline_descriptor = package_artifact_descriptor(baseline_path)
    validate_lifecycle_evidence(lifecycle, upgrade_descriptor, baseline_descriptor)
    lifecycle_upgrade = _mapping(lifecycle.get('upgrade_package'), 'lifecycle upgrade package')
    lifecycle_baseline = _mapping(lifecycle.get('baseline_package'), 'lifecycle baseline package')
    installed_package_state = _mapping(
        load_json(run_directory / 'installed-package-state.json'),
        'installed package state',
    )
    validate_installed_package_state(
        installed_package_state,
        lifecycle,
        upgrade_descriptor,
    )

    checks: dict[str, bool] = {}
    failures: list[str] = []
    _add_check(
        checks,
        failures,
        'healthz_remained_200',
        bool(health_samples)
        and all(
            _mapping(sample['details'], 'health sample').get('http_status') == 200
            for sample in health_samples
        ),
    )
    _add_check(
        checks,
        failures,
        'ready_503_within_3s',
        unavailable['details'].get('http_status') == 503 and detection_s <= READY_FAILURE_TARGET_S,
    )
    _add_check(
        checks,
        failures,
        'readiness_false_throughout_unavailable_interval',
        readiness_interval_valid,
    )
    _add_check(
        checks,
        failures,
        'original_process_group_empty_within_5s',
        group_cleanup_s <= PROCESS_GROUP_EXIT_TARGET_S
        and _mapping(group_empty['details'], 'group empty').get('member_count') == 0,
    )
    _add_check(checks, failures, 'exactly_one_failure_event', len(failures_detected) == 1)
    _add_check(checks, failures, 'exactly_one_restart_scheduled', len(restart_events) == 1)
    _add_check(checks, failures, 'exactly_one_replacement_child_start', len(child_starts) == 1)
    _add_check(checks, failures, 'readiness_false_event_present', len(ready_false) == 1)
    _add_check(checks, failures, 'readiness_true_event_present', len(ready_true) == 1)
    _add_check(checks, failures, 'exact_causal_supervisor_event_chain', causal_chain_valid)
    _add_check(
        checks,
        failures,
        'frozen_first_backoff_scheduled',
        len(restart_events) == 1
        and restart_events[0].get('backoff_ms') == EXPECTED_BACKOFF_MS
        and restart_events[0].get('restart_attempt') == 1,
    )
    _add_check(
        checks,
        failures,
        'actual_backoff_is_bounded',
        actual_backoff_s is not None and 0.9 <= actual_backoff_s <= 3.0,
    )
    _add_check(
        checks,
        failures,
        'ready_restored_within_30s_of_detection',
        event_recovery_s is not None and 0.0 <= event_recovery_s <= RECOVERY_TARGET_S,
    )
    _add_check(
        checks,
        failures,
        'observed_recovery_is_bounded',
        0.0 <= observed_recovery_s <= RECOVERY_TARGET_S,
    )
    _add_check(
        checks,
        failures,
        'supervisor_main_pid_stable',
        restored_details.get('main_pid') == main_pid
        and service_final.get('main_pid_before_stop') == main_pid,
    )
    _add_check(
        checks,
        failures,
        'systemd_did_not_restart_supervisor',
        initial_details.get('systemd_nrestarts') == 0
        and restored_details.get('systemd_nrestarts') == 0
        and service_final.get('nrestarts_before_stop') == 0
        and service_final.get('active_before_stop') is True
        and service_final.get('enabled_before_stop') is False,
    )
    _add_check(
        checks,
        failures,
        'only_one_systemd_owned_ros_service',
        initial_details.get('systemd_owned_ros_service_count') == 1
        and restored_details.get('systemd_owned_ros_service_count') == 1,
    )
    _add_check(
        checks,
        failures,
        'replacement_process_group_is_new',
        _integer(restored_details.get('child_pgid'), 'restored PGID', minimum=1) != original_pgid,
    )
    _add_check(
        checks,
        failures,
        'restored_status_is_exactly_ready',
        restored_status.get('schema_version') == 1
        and restored_status.get('ready') is True
        and restored_status.get('healthy') is True
        and len(restored_children) == 1
        and restored_children[0].get('name') == EXPECTED_CHILD
        and restored_children[0].get('running') is True
        and restored_children[0].get('heartbeat_fresh') is True
        and restored_children[0].get('restart_count') == 1,
    )
    _add_check(
        checks,
        failures,
        'controller_identity_is_exact',
        target.get('pid') == injection_details.get('target_pid')
        and target.get('pgid') == original_pgid
        and target.get('root_pid') == original_child_pid
        and target.get('unit') == EXPECTED_UNIT
        and target.get('ros_domain_id') == str(isolation.get('ros_domain_id'))
        and target.get('gz_partition') == isolation.get('gz_partition')
        and Path(str(target.get('executable', ''))).name == 'controller_server'
        and len(target_lineage) >= 2
        and target_lineage[0].get('pid') == original_child_pid
        and target_lineage[-1].get('pid') == target.get('pid'),
    )
    _add_check(
        checks,
        failures,
        'mission_was_active_before_injection',
        active_goal.get('schema_version') == 1
        and active_goal.get('goal_status_code') == 2
        and active_goal.get('goal_status') == 'EXECUTING'
        and active_goal.get('mission_pid') == injection_details.get('mission_pid')
        and active_goal.get('mission_process_alive') is True
        and active_goal.get('action_name') == '/robotest/follow_waypoints'
        and active_goal.get('mission_runner_is_action_client') is True,
    )
    _add_check(
        checks,
        failures,
        'published_process_groups_are_bound',
        mission_started_details.get('pid') == interrupted_pgid
        and injection_details.get('mission_pid') == interrupted_pgid
        and active_goal.get('mission_pid') == interrupted_pgid
        and probe_started_details.get('pid') == probe_pgid
        and followup_started_details.get('pid') == followup_pgid,
    )
    _add_check(checks, failures, 'interrupted_mission_pair_is_consistent', interrupted_pair_valid)
    _add_check(
        checks,
        failures,
        'interrupted_completion_matches_outcome',
        interrupted_event_exit == interrupted_exit
        and interrupted_event_terminated is interrupted_terminated
        and interrupted_event_group_empty is interrupted_group_empty,
    )
    _add_check(
        checks,
        failures,
        'interrupted_mission_never_success',
        interrupted_exit != 0 and not interrupted_success,
    )
    _add_check(
        checks,
        failures,
        'interrupted_mission_was_bounded',
        interrupted_bounded and interrupted_group_empty,
    )
    _add_check(checks, failures, 'followup_json_csv_reconcile', followup_csv_valid)
    _add_check(
        checks,
        failures,
        'exactly_one_successful_followup_completion',
        followup_event_exit == 0 and followup_event_exit == followup_verdict.get('exit_code'),
    )
    _add_check(
        checks,
        failures,
        'fresh_followup_mission_succeeded',
        followup_verdict.get('exit_code') == 0
        and followup_verdict.get('phase2_action_integration_status') == 'PASS'
        and followup_verdict.get('expected_outcome_met') is True
        and followup_measurements.get('goal_status') == 'SUCCEEDED'
        and followup_identity.get('mission_sha256') == followup_mission_sha,
    )
    _add_check(
        checks,
        failures,
        'event_store_has_no_loss',
        event_meta.get('schema_version') == 1
        and event_meta.get('saturated') is False
        and event_meta.get('dropped_events') == 0
        and event_meta.get('last_attempted_sequence') == len(events),
    )
    _add_check(
        checks,
        failures,
        'package_source_binding_passed',
        package_binding.get('verdict') == 'PASS'
        and rebound_package.get('verdict') == 'PASS'
        and rebound_package.get('candidate_manifest_sha256')
        == package_binding.get('candidate_manifest_sha256'),
    )
    _add_check(
        checks,
        failures,
        'independent_package_reproducibility_passed',
        package_integrity.get('verdict') == 'PASS' and dict(package_integrity) == rebound_integrity,
    )
    _add_check(
        checks,
        failures,
        'package_artifacts_match_lifecycle',
        file_sha256(upgrade_path)
        == _sha256(upgrade_context.get('sha256'), 'upgrade context SHA')
        == lifecycle_upgrade.get('sha256')
        and file_sha256(baseline_path)
        == _sha256(baseline_context.get('sha256'), 'baseline context SHA')
        == lifecycle_baseline.get('sha256'),
    )
    _add_check(
        checks,
        failures,
        'package_lifecycle_passed',
        file_sha256(Path(_string(lifecycle_context.get('path'), 'lifecycle evidence path')))
        == _sha256(lifecycle_context.get('sha256'), 'lifecycle evidence context SHA')
        == file_sha256(run_directory / 'package-lifecycle.json'),
    )
    _add_check(
        checks,
        failures,
        'installed_package_state_matches_lifecycle',
        installed_package_state.get('verdict') == 'PASS',
    )
    _add_check(
        checks,
        failures,
        'runtime_staging_is_bound',
        staging.get('schema_version') == 1
        and staging.get('active_path') == '/opt/robotest-lab'
        and SHA256_RE.fullmatch(str(staging.get('source_manifest_sha256', ''))) is not None
        and SHA256_RE.fullmatch(str(staging.get('install_manifest_sha256', ''))) is not None
        and dict(staging) == dict(overlay_provenance)
        and staging.get('source_manifest_sha256') == file_sha256(overlay_source_manifest_path)
        and staging.get('install_manifest_sha256') == file_sha256(overlay_install_manifest_path)
        and staging.get('git_commit') == context.get('source_git_commit')
        and staging.get('git_dirty') == context.get('source_git_dirty')
        and Path(str(context.get('active_overlay_target', ''))).name == staging.get('release_id'),
    )
    child_environment = (
        _mapping(config_children[0].get('environment'), 'supervisor child environment')
        if len(config_children) == 1
        else {}
    )
    _add_check(
        checks,
        failures,
        'run_scoped_supervisor_config_is_exact',
        supervisor_config.get('state_directory') == f'/var/lib/robotest-supervisor/{run_id}'
        and len(config_children) == 1
        and child_environment.get('ROS_DOMAIN_ID') == str(isolation.get('ros_domain_id'))
        and child_environment.get('GZ_PARTITION') == isolation.get('gz_partition'),
    )
    _add_check(
        checks,
        failures,
        'source_snapshot_unchanged_and_self_consistent',
        source_before.get('snapshot_sha256') == before_snapshot_hash
        and source_after.get('snapshot_sha256') == after_snapshot_hash
        and before_snapshot_hash == after_snapshot_hash,
    )
    _add_check(
        checks,
        failures,
        'unique_runtime_isolation',
        isinstance(isolation.get('ros_domain_id'), int)
        and 100 <= isolation['ros_domain_id'] <= 229
        and isolation.get('domain_was_unused') is True
        and isolation.get('partition_was_unused') is True
        and PARTITION_RE.fullmatch(str(isolation.get('gz_partition', ''))) is not None,
    )
    _add_check(
        checks,
        failures,
        'cleanup_complete_and_owned',
        cleanup.get('owned_paths_only') is True,
    )

    evidence_hashes = _relative_evidence_hashes(
        run_directory,
        {'scenario6-result.json', 'scenario6-result.csv', 'SHA256SUMS'},
    )
    return {
        'schema_version': SCHEMA_VERSION,
        'producer': PRODUCER,
        'identity': {
            'run_id': run_id,
            'started_utc': context.get('started_utc'),
            'completed_utc': utc_now(),
            'source_git_commit': context.get('source_git_commit'),
            'source_git_dirty': context.get('source_git_dirty'),
            'ros_domain_id': isolation.get('ros_domain_id'),
            'gz_partition': isolation.get('gz_partition'),
            'supervisor_unit': EXPECTED_UNIT,
            'managed_child': EXPECTED_CHILD,
        },
        'targets': {
            'heartbeat_period_wall_s': 0.5,
            'heartbeat_stale_wall_s': 2.0,
            'ready_failure_wall_s_max': READY_FAILURE_TARGET_S,
            'termination_allowance_wall_s': PROCESS_GROUP_EXIT_TARGET_S,
            'ready_restore_after_detection_wall_s_max': RECOVERY_TARGET_S,
            'restart_backoff_wall_s': [1.0, 2.0, 4.0, 8.0],
            'restart_attempt_limit': 4,
            'restart_window_wall_s': 60.0,
            'stable_reset_wall_s': 60.0,
        },
        'measurements': {
            'ready_503_after_injection_wall_s': detection_s,
            'original_group_empty_after_injection_wall_s': group_cleanup_s,
            'observed_ready_restore_after_503_wall_s': observed_recovery_s,
            'supervisor_recovery_time_wall_s': event_recovery_s,
            'actual_restart_backoff_wall_s': actual_backoff_s,
            'restart_scheduled_count': len(restart_events),
            'replacement_child_start_count': len(child_starts),
            'original_child_pid': original_child_pid,
            'original_child_pgid': original_pgid,
            'replacement_child_pid': restored_details.get('child_pid'),
            'replacement_child_pgid': restored_details.get('child_pgid'),
            'supervisor_main_pid': main_pid,
            'interrupted_mission_exit_code': interrupted_exit,
            'followup_mission_exit_code': followup_verdict.get('exit_code'),
        },
        'quality': {
            'checks': checks,
            'event_count': len(events),
            'event_trace_dropped': event_meta.get('dropped_events'),
            'timeline_count': len(timeline),
            'raw_evidence_sha256': evidence_hashes,
        },
        'verdict': {
            'status': 'PASS' if not failures else 'FAIL',
            'accepted': not failures,
            'failure_count': len(failures),
            'failures': failures,
        },
    }


def write_result(run_directory: Path) -> dict[str, Any]:
    """Write canonical Scenario 6 JSON and one-row CSV, including FAIL evidence."""
    try:
        result = evaluate_run(run_directory)
    except Exception as exc:
        context_path = run_directory / 'context.json'
        run_id = 'unknown'
        if context_path.is_file():
            with contextlib.suppress(EvidenceError):
                run_id = str(_mapping(load_json(context_path), 'context').get('run_id', 'unknown'))
        result = {
            'schema_version': SCHEMA_VERSION,
            'producer': PRODUCER,
            'identity': {'run_id': run_id, 'completed_utc': utc_now()},
            'targets': {},
            'measurements': {},
            'quality': {'checks': {}, 'raw_evidence_sha256': {}},
            'verdict': {
                'status': 'FAIL',
                'accepted': False,
                'failure_count': 1,
                'failures': [f'evidence_error: {type(exc).__name__}: {exc}'],
            },
        }
    json_path = run_directory / 'scenario6-result.json'
    csv_path = run_directory / 'scenario6-result.csv'
    atomic_write_json(json_path, result)
    identity = _mapping(result.get('identity'), 'result identity')
    measurements = _mapping(result.get('measurements'), 'result measurements')
    verdict = _mapping(result.get('verdict'), 'result verdict')
    row = {
        'run_id': identity.get('run_id'),
        'verdict': verdict.get('status'),
        'failure_count': verdict.get('failure_count'),
        'ready_503_after_injection_wall_s': measurements.get('ready_503_after_injection_wall_s'),
        'original_group_empty_after_injection_wall_s': measurements.get(
            'original_group_empty_after_injection_wall_s'
        ),
        'supervisor_recovery_time_wall_s': measurements.get('supervisor_recovery_time_wall_s'),
        'actual_restart_backoff_wall_s': measurements.get('actual_restart_backoff_wall_s'),
        'restart_scheduled_count': measurements.get('restart_scheduled_count'),
        'interrupted_mission_exit_code': measurements.get('interrupted_mission_exit_code'),
        'followup_mission_exit_code': measurements.get('followup_mission_exit_code'),
        'ros_domain_id': identity.get('ros_domain_id'),
        'gz_partition': identity.get('gz_partition'),
        'source_git_commit': identity.get('source_git_commit'),
    }
    descriptor, temporary_name = tempfile.mkstemp(prefix='.scenario6-result.', dir=run_directory)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8', newline='') as target:
            writer = csv.DictWriter(target, fieldnames=list(row), lineterminator='\n')
            writer.writeheader()
            writer.writerow({key: '' if value is None else value for key, value in row.items()})
            target.flush()
            os.fsync(target.fileno())
        os.chmod(temporary, 0o644)
        os.replace(temporary, csv_path)
    except BaseException:
        with contextlib.suppress(OSError):
            os.close(descriptor)
        temporary.unlink(missing_ok=True)
        raise
    return result


def write_checksums(run_directory: Path) -> None:
    """Write and immediately verify a sorted checksum manifest."""
    hashes = _relative_evidence_hashes(run_directory, {'SHA256SUMS'})
    payload = ''.join(f'{digest}  {name}\n' for name, digest in sorted(hashes.items())).encode()
    _atomic_write(run_directory / 'SHA256SUMS', payload)
    for name, expected in hashes.items():
        if file_sha256(run_directory / name) != expected:
            raise EvidenceError(f'checksum changed during manifest creation: {name}')


def _parse_details(values: Sequence[str]) -> dict[str, Any]:
    details: dict[str, Any] = {}
    for value in values:
        if '=' not in value:
            raise EvidenceError(f'--detail must be KEY=VALUE: {value!r}')
        key, raw = value.split('=', 1)
        if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', key) or key in details:
            raise EvidenceError(f'invalid or duplicate detail key: {key!r}')
        try:
            details[key] = json.loads(raw)
        except json.JSONDecodeError:
            details[key] = raw
    return details


def build_parser() -> argparse.ArgumentParser:
    """Build the bounded helper CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)

    allocate = subparsers.add_parser('allocate-isolation')
    allocate.add_argument('--run-id', required=True)
    allocate.add_argument('--output', type=Path, required=True)

    render = subparsers.add_parser('render-config')
    render.add_argument('--template', type=Path, required=True)
    render.add_argument('--output', type=Path, required=True)
    render.add_argument('--state-directory', required=True)
    render.add_argument('--ros-domain-id', type=int, required=True)
    render.add_argument('--gz-partition', required=True)

    followup = subparsers.add_parser('render-followup')
    followup.add_argument('--output', type=Path, required=True)

    binding = subparsers.add_parser('package-binding')
    binding.add_argument('--repository', type=Path, required=True)
    binding.add_argument('--manifest', type=Path, required=True)
    binding.add_argument('--output', type=Path, required=True)

    integrity = subparsers.add_parser('verify-package-candidate')
    integrity.add_argument('--package-directory', type=Path, required=True)
    integrity.add_argument('--upgrade', type=Path, required=True)
    integrity.add_argument('--baseline', type=Path, required=True)
    integrity.add_argument('--output', type=Path, required=True)

    lifecycle = subparsers.add_parser('validate-lifecycle')
    lifecycle.add_argument('--evidence', type=Path, required=True)
    lifecycle.add_argument('--upgrade', type=Path, required=True)
    lifecycle.add_argument('--baseline', type=Path, required=True)

    installed = subparsers.add_parser('capture-installed-state')
    installed.add_argument('--evidence', type=Path, required=True)
    installed.add_argument('--upgrade', type=Path, required=True)
    installed.add_argument('--baseline', type=Path, required=True)
    installed.add_argument('--smoke-script', type=Path, required=True)
    installed.add_argument('--output', type=Path, required=True)

    snapshot = subparsers.add_parser('source-snapshot')
    snapshot.add_argument('--repository', type=Path, required=True)
    snapshot.add_argument('--output', type=Path, required=True)

    record = subparsers.add_parser('record')
    record.add_argument('--timeline', type=Path, required=True)
    record.add_argument('--kind', required=True)
    record.add_argument('--detail', action='append', default=[])

    http = subparsers.add_parser('probe-http')
    http.add_argument('--timeline', type=Path, required=True)
    http.add_argument('--kind', required=True)
    http.add_argument('--url', required=True)
    http.add_argument('--timeout', type=float, default=1.0)

    fetch = subparsers.add_parser('fetch-json')
    fetch.add_argument('--url', required=True)
    fetch.add_argument('--output', type=Path, required=True)
    fetch.add_argument('--expected-status', type=int, default=200)

    controller = subparsers.add_parser('find-controller')
    controller.add_argument('--root-pid', type=int, required=True)
    controller.add_argument('--pgid', type=int, required=True)
    controller.add_argument('--ros-domain-id', type=int, required=True)
    controller.add_argument('--gz-partition', required=True)
    controller.add_argument('--unit', default=EXPECTED_UNIT)
    controller.add_argument('--output', type=Path, required=True)

    identity = subparsers.add_parser('assert-identity')
    identity.add_argument('--identity', type=Path, required=True)

    signal_identity = subparsers.add_parser('signal-identity')
    signal_identity.add_argument('--identity', type=Path, required=True)
    signal_identity.add_argument('--signal', choices=('TERM', 'KILL'), required=True)

    group = subparsers.add_parser('process-group')
    group.add_argument('--pgid', type=int, required=True)
    group.add_argument('--output', type=Path)
    group.add_argument('--require-empty', action='store_true')

    owners = subparsers.add_parser('systemd-owners')
    owners.add_argument('--ros-domain-id', type=int, required=True)
    owners.add_argument('--gz-partition', required=True)
    owners.add_argument('--expected-unit', default=EXPECTED_UNIT)
    owners.add_argument('--output', type=Path, required=True)

    compose = subparsers.add_parser('compose')
    compose.add_argument('--run-directory', type=Path, required=True)

    checksums = subparsers.add_parser('checksums')
    checksums.add_argument('--run-directory', type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run one helper operation with stable nonzero failure behavior."""
    args = build_parser().parse_args(argv)
    try:
        if args.command == 'allocate-isolation':
            value = allocated_isolation(args.run_id)
            atomic_write_json(args.output, value)
        elif args.command == 'render-config':
            render_supervisor_config(
                args.template,
                args.output,
                args.state_directory,
                args.ros_domain_id,
                args.gz_partition,
            )
        elif args.command == 'render-followup':
            render_followup_mission(args.output)
        elif args.command == 'package-binding':
            value = package_source_binding(args.repository.resolve(), args.manifest.resolve())
            atomic_write_json(args.output, value)
            if value['verdict'] != 'PASS':
                return 1
        elif args.command == 'verify-package-candidate':
            atomic_write_json(
                args.output,
                verify_package_candidate(
                    args.package_directory,
                    args.upgrade,
                    args.baseline,
                ),
            )
        elif args.command == 'validate-lifecycle':
            lifecycle = _mapping(load_json(args.evidence), 'package lifecycle')
            validate_lifecycle_evidence(
                lifecycle,
                package_artifact_descriptor(args.upgrade),
                package_artifact_descriptor(args.baseline),
            )
        elif args.command == 'capture-installed-state':
            lifecycle = _mapping(load_json(args.evidence), 'package lifecycle')
            upgrade = package_artifact_descriptor(args.upgrade)
            validate_lifecycle_evidence(
                lifecycle,
                upgrade,
                package_artifact_descriptor(args.baseline),
            )
            atomic_write_json(
                args.output,
                capture_installed_package_state(
                    lifecycle,
                    upgrade,
                    args.smoke_script,
                ),
            )
        elif args.command == 'source-snapshot':
            atomic_write_json(args.output, source_snapshot(args.repository.resolve()))
        elif args.command == 'record':
            value = append_timeline(args.timeline, args.kind, _parse_details(args.detail))
            print(value['monotonic_ns'])
        elif args.command == 'probe-http':
            status, body = probe_http(args.url, args.timeout)
            append_timeline(
                args.timeline,
                args.kind,
                {
                    'url': args.url,
                    'http_status': status,
                    'body': body,
                    'body_sha256': hashlib.sha256(body.encode()).hexdigest(),
                },
            )
            print(status)
        elif args.command == 'fetch-json':
            fetch_http_json(args.url, args.output, args.expected_status)
        elif args.command == 'find-controller':
            value = find_exact_controller(
                args.root_pid,
                args.pgid,
                args.ros_domain_id,
                args.gz_partition,
                unit=args.unit,
            )
            atomic_write_json(args.output, value)
            print(value['pid'])
        elif args.command == 'assert-identity':
            assert_process_identity(_mapping(load_json(args.identity), 'identity'))
        elif args.command == 'signal-identity':
            signum = signal.SIGTERM if args.signal == 'TERM' else signal.SIGKILL
            print(signal_process_identity(_mapping(load_json(args.identity), 'identity'), signum))
        elif args.command == 'process-group':
            members = process_group_members(args.pgid)
            value = {
                'schema_version': 1,
                'captured_utc': utc_now(),
                'pgid': args.pgid,
                'member_count': len(members),
                'members': members,
            }
            if args.output is not None:
                atomic_write_json(args.output, value)
            else:
                print(json.dumps(value, sort_keys=True))
            if args.require_empty and members:
                return 1
        elif args.command == 'systemd-owners':
            units = systemd_owned_units(args.ros_domain_id, args.gz_partition)
            value = {
                'schema_version': 1,
                'captured_utc': utc_now(),
                'ros_domain_id': args.ros_domain_id,
                'gz_partition': args.gz_partition,
                'units': units,
                'unit_count': len(units),
                'verdict': (
                    'PASS'
                    if set(units) == {args.expected_unit} and bool(units[args.expected_unit])
                    else 'FAIL'
                ),
            }
            atomic_write_json(args.output, value)
            print(len(units))
            if value['verdict'] != 'PASS':
                return 1
        elif args.command == 'compose':
            result = write_result(args.run_directory.resolve())
            if result['verdict']['status'] != 'PASS':
                return 1
        elif args.command == 'checksums':
            write_checksums(args.run_directory.resolve())
        else:
            raise EvidenceError(f'unsupported command: {args.command}')
    except (EvidenceError, OSError, ValueError) as exc:
        print(f'phase4_acceptance: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

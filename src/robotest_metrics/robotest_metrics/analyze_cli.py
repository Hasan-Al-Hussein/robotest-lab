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

"""Deterministic offline analysis CLI for one captured Phase 3 run."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from robotest_metrics.analysis import analyze_run
from robotest_metrics.artifacts import (
    canonical_sha256,
    file_sha256,
    validate_csv_projection,
    write_result_pair,
)
from robotest_metrics.bundle import (
    MANIFEST_NAME,
    MANIFEST_SIDECAR_NAME,
    write_result_manifest,
)
from robotest_metrics.constants import PER_RUN_JSON_MAX_BYTES
from robotest_metrics.errors import ArtifactError, MetricUnavailable
from robotest_metrics.failure_results import (
    automatic_failure,
    compose_artifact_finalization_failure,
    compose_infrastructure_failure,
    require_request_context_binding,
    validate_trial_context,
)
from robotest_metrics.reporting import generate_reports, load_canonical_result
from robotest_metrics.schema_validation import validate_document


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Analyze a bounded RoboTest capture into the sole Phase 3 verdict.',
    )
    parser.add_argument(
        '--input',
        type=Path,
        help='analysis-request JSON; omit only for a context-declared infrastructure failure',
    )
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument(
        '--trial-context',
        required=True,
        type=Path,
        help='immutable trial-context JSON created before trial mutation',
    )
    return parser


def _load_document(path: Path, name: str) -> Mapping[str, Any]:
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise ArtifactError(f'cannot read {name}: {exc}') from exc
    if len(payload) > PER_RUN_JSON_MAX_BYTES:
        raise ArtifactError(f'{name} exceeds the 32 MiB input bound')
    try:
        document = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ArtifactError(f'{name} is not valid UTF-8 JSON: {exc}') from exc
    if not isinstance(document, Mapping):
        raise ArtifactError(f'{name} root must be an object')
    return document


def _stale_outputs(output: Path) -> list[Path]:
    try:
        if not output.exists():
            return []
        if not output.is_dir():
            return [output]
        return list(output.iterdir())
    except OSError as exc:
        raise ArtifactError(f'cannot inspect output directory: {exc}') from exc


def _failure_evidence_sha256(path: Path, stage: str, reason: str) -> str:
    try:
        return file_sha256(path)
    except OSError:
        return canonical_sha256(
            {
                'observed_by': 'robotest_metrics/metrics_analyze',
                'path': str(path),
                'reason': reason,
                'stage': stage,
            }
        )


def _load_failure_mapping(value: Any) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise MetricUnavailable('trial context failure must be an object')
    return value


def _remove_retry_artifacts(output: Path) -> None:
    for name in (
        MANIFEST_NAME,
        MANIFEST_SIDECAR_NAME,
        'localization-error.png',
        'measurement-summary.png',
        'real-time-factor.png',
        'trajectory.png',
    ):
        try:
            (output / name).unlink(missing_ok=True)
        except OSError as exc:
            raise ArtifactError(f'cannot clear failed derived artifact {name}: {exc}') from exc


def _finalize_result(
    result: Mapping[str, Any],
    output: Path,
    *,
    include_charts: bool,
) -> None:
    write_result_pair(result, output)
    reports = generate_reports(
        output / 'run-result.json',
        include_charts=include_charts,
    )
    loaded, source_hash = load_canonical_result(output / 'run-result.json')
    if loaded != result or reports['canonical_json_sha256'] != source_hash:
        raise ArtifactError('derived report source does not match the canonical result')
    validate_csv_projection(result, output / 'run-result.csv')
    write_result_manifest(output, result)


def main(argv: Sequence[str] | None = None) -> int:
    """Analyze, atomically write, reconcile, and derive all human artifacts."""
    arguments = _parser().parse_args(argv)
    try:
        stale = _stale_outputs(arguments.output_dir)
    except ArtifactError as exc:
        print(f'metrics analysis cannot inspect outputs: {exc}', file=sys.stderr)
        return 33
    if stale:
        print(f'metrics analysis refuses stale outputs: {stale}', file=sys.stderr)
        return 33
    try:
        context = _load_document(arguments.trial_context, 'trial context')
        validate_document(context, 'trial-context.schema.json')
        validate_trial_context(context)
    except (ArtifactError, MetricUnavailable) as exc:
        print(f'metrics analysis cannot bind trial context: {exc}', file=sys.stderr)
        return 34

    declared_failure = context.get('failure')
    infrastructure_failure = False
    if declared_failure is not None:
        if arguments.input is not None:
            print(
                'metrics analysis rejects --input when trial context declares a failure',
                file=sys.stderr,
            )
            return 34
        result = compose_infrastructure_failure(
            context,
            _load_failure_mapping(declared_failure),
        )
        validate_document(result, 'run-result.schema.json')
        infrastructure_failure = True
    elif arguments.input is None:
        print(
            'metrics analysis requires --input when trial context failure is null',
            file=sys.stderr,
        )
        return 34
    else:
        stage = 'analysis_request_read'
        try:
            request = _load_document(arguments.input, 'analysis request')
            stage = 'analysis_request_schema'
            validate_document(request, 'analysis-request.schema.json')
            stage = 'trial_context_binding'
            require_request_context_binding(request, context)
            capture = request.get('capture')
            stage = 'capture_schema'
            validate_document(capture, 'capture.schema.json')
            fault = request.get('fault')
            if isinstance(fault, Mapping) and fault.get('kind') == 'lidar_dropout':
                stage = 'lifecycle_schema'
                validate_document(
                    fault.get('lifecycle_snapshot'),
                    'lifecycle-snapshot.schema.json',
                )
            stage = 'analysis'
            result = analyze_run(request)
            stage = 'run_result_schema'
            validate_document(result, 'run-result.schema.json')
        except (ArtifactError, MetricUnavailable) as exc:
            reason = str(exc)
            failure = automatic_failure(
                evidence_sha256=_failure_evidence_sha256(arguments.input, stage, reason),
                reason=reason,
                stage=stage,
            )
            result = compose_infrastructure_failure(context, failure)
            validate_document(result, 'run-result.schema.json')
            infrastructure_failure = True
    try:
        _finalize_result(
            result,
            arguments.output_dir,
            include_charts=not infrastructure_failure,
        )
    except (ArtifactError, OSError) as exc:
        reason = str(exc)
        evidence_path = arguments.input or arguments.trial_context
        failed = compose_artifact_finalization_failure(
            context,
            evidence_sha256=_failure_evidence_sha256(
                evidence_path,
                'artifact_finalization',
                reason,
            ),
            reason=reason,
        )
        try:
            _remove_retry_artifacts(arguments.output_dir)
            validate_document(failed, 'run-result.schema.json')
            _finalize_result(
                failed,
                arguments.output_dir,
                include_charts=False,
            )
        except (ArtifactError, MetricUnavailable, OSError) as nested:
            print(f'cannot finalize failed canonical verdict: {nested}', file=sys.stderr)
        print(f'metrics analysis artifact failure: {exc}', file=sys.stderr)
        return 31
    status = result['verdict']['automated_status']
    print(
        json.dumps(
            {
                'run_id': result['identity'].get('run_id'),
                'run_result': str(arguments.output_dir / 'run-result.json'),
                'status': status,
            },
            sort_keys=True,
        )
    )
    return int(result['verdict']['exit_code'])


if __name__ == '__main__':
    raise SystemExit(main())

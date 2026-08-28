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

from __future__ import annotations

import csv
import io
import json
from pathlib import Path

import pytest
import robotest_metrics.aggregate_cli as aggregate_cli_module
from robotest_metrics.artifacts import (
    CSV_PROJECTION_CONTRACT,
    CSV_PROJECTION_CONTRACT_COLUMN,
    CSV_PROJECTION_JSON_SHA256_COLUMN,
    canonical_json_bytes,
    canonical_sha256,
    legacy_flattened_one_row_csv_bytes,
    one_row_csv_bytes,
    scalar_summary_projection,
)
from robotest_metrics.constants import PER_RUN_CSV_MAX_BYTES
from robotest_metrics.errors import ArtifactError


def test_scalar_summary_projection_hash_binds_sequences_and_full_json() -> None:
    document = {
        'events': [{'kind': 'first'}, {'kind': 'second'}],
        'measurements': {'collision_count': 0, 'optional': None},
    }

    projection = scalar_summary_projection(document)
    descriptor = json.loads(projection['events'])

    assert projection[CSV_PROJECTION_CONTRACT_COLUMN] == CSV_PROJECTION_CONTRACT
    assert projection[CSV_PROJECTION_JSON_SHA256_COLUMN] == canonical_sha256(document)
    assert descriptor == {
        'element_count': 2,
        'kind': 'sequence',
        'sha256': canonical_sha256(document['events']),
    }
    assert projection['measurements.collision_count'] == '0'
    assert projection['measurements.optional'] == ''

    changed = {
        **document,
        'events': [{'kind': 'first'}, {'kind': 'changed'}],
    }
    changed_projection = scalar_summary_projection(changed)
    assert changed_projection['events'] != projection['events']
    assert (
        changed_projection[CSV_PROJECTION_JSON_SHA256_COLUMN]
        != projection[CSV_PROJECTION_JSON_SHA256_COLUMN]
    )


def test_current_candidate_cardinalities_fit_bounded_csv_without_raw_sequences() -> None:
    document = {
        'events': [],
        'measurements': {
            'actual_path': {
                'samples': [{'diagnostic': 'a' * 400, 'stamp_ns': index} for index in range(1_499)]
            },
            'localization': {
                'aligned_samples': [
                    {'diagnostic': 'l' * 400, 'stamp_ns': index} for index in range(1_497)
                ]
            },
            'real_time_factor': {'calculated_series': [0.75] * 1_481},
        },
        'quality': {
            'components': {
                'scenario': {
                    'binding': {
                        'feedback_trace': [
                            {'diagnostic': 'f' * 400, 'stamp_ns': index} for index in range(1_149)
                        ]
                    }
                }
            }
        },
    }

    assert len(canonical_json_bytes(document)) > PER_RUN_CSV_MAX_BYTES
    payload = one_row_csv_bytes(document)
    projection = scalar_summary_projection(document)

    assert len(payload) < PER_RUN_CSV_MAX_BYTES
    assert json.loads(projection['measurements.actual_path.samples'])['element_count'] == 1_499
    assert (
        json.loads(projection['measurements.localization.aligned_samples'])['element_count']
        == 1_497
    )
    assert (
        json.loads(projection['quality.components.scenario.binding.feedback_trace'])[
            'element_count'
        ]
        == 1_149
    )
    assert 'a' * 400 not in payload.decode('utf-8')


@pytest.mark.parametrize(
    'reserved_name',
    [CSV_PROJECTION_CONTRACT_COLUMN, CSV_PROJECTION_JSON_SHA256_COLUMN],
)
def test_projection_rejects_reserved_root_column_names(reserved_name: str) -> None:
    with pytest.raises(ArtifactError, match='reserved column'):
        scalar_summary_projection({reserved_name: 'forged'})


def test_projection_rejects_dotted_key_column_collision() -> None:
    document = {'a': {'b': [1, 2]}, 'a.b': [3, 4]}

    with pytest.raises(ArtifactError, match=r"column collision at 'a\.b'"):
        scalar_summary_projection(document)


def test_legacy_flattened_csv_keeps_complete_canonical_sequence_value() -> None:
    assert (
        legacy_flattened_one_row_csv_bytes({'events': [{'kind': 'first'}], 'value': 1})
        == b'events,value\n"[{""kind"":""first""}]",1\n'
    )


def test_aggregate_cli_retains_exact_legacy_complete_flattening(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source_digest = 'a' * 64
    aggregate = {
        'identity': {'candidate_id': 'candidate'},
        'series': [{'run_id': 'run-0', 'value': 1.0}, {'run_id': 'run-1', 'value': 2.0}],
        'verdict': {'automated_status': 'PASS'},
    }
    monkeypatch.setattr(
        aggregate_cli_module,
        'load_canonical_result',
        lambda _path: ({'identity': {'run_id': 'source'}}, source_digest),
    )
    monkeypatch.setattr(
        aggregate_cli_module,
        'verify_result_bundle',
        lambda _path: {'identity': {'run_result_sha256': source_digest}},
    )
    monkeypatch.setattr(aggregate_cli_module, 'validate_document', lambda *_args: None)
    monkeypatch.setattr(
        aggregate_cli_module,
        'aggregate_phase3_suite',
        lambda _results, *, metric_paths: aggregate,
    )
    output = tmp_path / 'aggregate'
    arguments: list[str] = []
    for index in range(15):
        arguments.extend(['--input', str(tmp_path / f'run-{index}' / 'run-result.json')])
    arguments.extend(
        [
            '--metric',
            'measurements.value',
            '--output-dir',
            str(output),
        ]
    )

    assert aggregate_cli_module.main(arguments) == 0

    expected = legacy_flattened_one_row_csv_bytes(aggregate)
    actual = (output / 'aggregate-result.csv').read_bytes()
    row = next(csv.DictReader(io.StringIO(actual.decode('utf-8'))))
    assert actual == expected
    assert row['series'] == canonical_json_bytes(
        aggregate['series'], trailing_newline=False
    ).decode('utf-8')
    assert CSV_PROJECTION_CONTRACT.encode() not in actual

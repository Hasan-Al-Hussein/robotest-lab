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
from robotest_metrics.analysis import analyze_run
from robotest_metrics.artifacts import (
    canonical_json_bytes,
    flatten_document,
    validate_csv_projection,
    write_bytes_atomic,
    write_result_pair,
)
from robotest_metrics.bundle import verify_result_bundle, write_result_manifest
from robotest_metrics.constants import PNG_MAX_BYTES
from robotest_metrics.errors import ArtifactError
from robotest_metrics.report_cli import main as report_main
from robotest_metrics.reporting import generate_reports, load_canonical_result


def test_canonical_json_is_stable_utf8_and_rejects_nonfinite() -> None:
    assert canonical_json_bytes({'z': 'مرحبا', 'a': 1}) == '{"a":1,"z":"مرحبا"}\n'.encode()
    with pytest.raises(ArtifactError, match='canonical-JSON'):
        canonical_json_bytes({'bad': float('nan')})


def test_flattened_csv_has_one_row_and_null_is_blank(tmp_path: Path) -> None:
    document = {'events': [], 'measurements': {'count': 0, 'missing': None}, 'pass': True}
    projection = flatten_document(document)
    assert projection == {
        'events': '[]',
        'measurements.count': '0',
        'measurements.missing': '',
        'pass': 'true',
    }
    records = write_result_pair(document, tmp_path)
    rows = list(csv.DictReader(io.StringIO((tmp_path / 'run-result.csv').read_text())))
    assert len(rows) == 1
    assert rows[0]['measurements.missing'] == ''
    validate_csv_projection(document, tmp_path / 'run-result.csv')
    assert records['json']['bytes'] == (tmp_path / 'run-result.json').stat().st_size


def test_atomic_writer_checks_cap_before_replacing_existing_file(tmp_path: Path) -> None:
    target = tmp_path / 'bounded.bin'
    target.write_bytes(b'original')
    with pytest.raises(ArtifactError, match='cap'):
        write_bytes_atomic(b'too large', target, maximum_bytes=2)
    assert target.read_bytes() == b'original'


def test_csv_reconciliation_detects_any_mutation(tmp_path: Path) -> None:
    document = {'a': 1}
    write_result_pair(document, tmp_path)
    (tmp_path / 'run-result.csv').write_text('a\n2\n', encoding='utf-8')
    with pytest.raises(ArtifactError, match='does not exactly match'):
        validate_csv_projection(document, tmp_path / 'run-result.csv')


def test_reports_and_pngs_are_derived_from_canonical_json_only(
    tmp_path: Path,
    analysis_request: dict[str, object],
) -> None:
    result = analyze_run(analysis_request)
    write_result_pair(result, tmp_path)
    artifacts = generate_reports(tmp_path / 'run-result.json')
    loaded, source_hash = load_canonical_result(tmp_path / 'run-result.json')
    assert loaded == result
    assert artifacts['canonical_json_sha256'] == source_hash
    assert source_hash in (tmp_path / 'report.md').read_text(encoding='utf-8')
    assert source_hash in (tmp_path / 'report.html').read_text(encoding='utf-8')
    chart_names = {Path(item['path']).name for item in artifacts['charts']}
    assert {'measurement-summary.png', 'trajectory.png', 'localization-error.png'} <= chart_names
    assert all(item['bytes'] <= PNG_MAX_BYTES for item in artifacts['charts'])


def test_report_loader_rejects_pretty_or_hand_edited_json(tmp_path: Path) -> None:
    path = tmp_path / 'run-result.json'
    path.write_text(json.dumps({'identity': {}}, indent=2), encoding='utf-8')
    with pytest.raises(ArtifactError, match='not canonical'):
        load_canonical_result(path)


def test_non_circular_manifest_covers_every_output_and_detects_mutation(
    tmp_path: Path,
    analysis_request: dict[str, object],
) -> None:
    result = analyze_run(analysis_request)
    write_result_pair(result, tmp_path)
    generate_reports(tmp_path / 'run-result.json', include_charts=False)
    manifest = write_result_manifest(tmp_path, result)
    paths = {record['path'] for record in manifest['artifacts']}
    assert paths == {'report.html', 'report.md', 'run-result.csv', 'run-result.json'}
    assert 'run-artifacts.manifest.json' not in paths
    assert verify_result_bundle(tmp_path) == manifest
    with (tmp_path / 'report.md').open('a', encoding='utf-8') as stream:
        stream.write('tamper\n')
    with pytest.raises(ArtifactError, match='hashes do not match'):
        verify_result_bundle(tmp_path)


def test_report_cli_refreshes_same_directory_manifest(
    tmp_path: Path,
    analysis_request: dict[str, object],
) -> None:
    result = analyze_run(analysis_request)
    write_result_pair(result, tmp_path)
    generate_reports(tmp_path / 'run-result.json', include_charts=False)
    write_result_manifest(tmp_path, result)
    assert (
        report_main(
            [
                '--input',
                str(tmp_path / 'run-result.json'),
                '--output-dir',
                str(tmp_path),
            ]
        )
        == 0
    )
    refreshed = verify_result_bundle(tmp_path)
    assert refreshed['quality']['artifact_count'] >= 4

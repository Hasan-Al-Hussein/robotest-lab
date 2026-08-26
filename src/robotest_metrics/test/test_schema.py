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

import copy

import pytest
from robotest_metrics.analysis import analyze_run
from robotest_metrics.errors import ArtifactError
from robotest_metrics.schema_validation import load_schema, validate_document


@pytest.mark.parametrize(
    'schema_name',
    [
        'analysis-request.schema.json',
        'capture.schema.json',
        'run-artifacts-manifest.schema.json',
        'run-result.schema.json',
        'trial-context.schema.json',
    ],
)
def test_packaged_schemas_are_draft_2020_12(schema_name: str) -> None:
    schema = load_schema(schema_name)
    assert schema['$schema'] == 'https://json-schema.org/draft/2020-12/schema'


def test_complete_fixture_validates_all_three_contracts(
    analysis_request: dict[str, object],
) -> None:
    validate_document(analysis_request, 'analysis-request.schema.json')
    validate_document(analysis_request['capture'], 'capture.schema.json')
    validate_document(analyze_run(analysis_request), 'run-result.schema.json')


def test_schema_rejects_nonidentity_world_alignment(
    analysis_request: dict[str, object],
) -> None:
    analysis_request['targets']['world_to_map']['x_m'] = 0.1
    with pytest.raises(ArtifactError, match='world_to_map'):
        validate_document(analysis_request, 'analysis-request.schema.json')


def test_capture_schema_requires_every_configured_stream(
    analysis_request: dict[str, object],
) -> None:
    capture = copy.deepcopy(analysis_request['capture'])
    del capture['streams']['contacts']
    with pytest.raises(ArtifactError, match='contacts'):
        validate_document(capture, 'capture.schema.json')


def test_analysis_schema_rejects_predicted_metrics_output_metadata(
    analysis_request: dict[str, object],
) -> None:
    artifacts = analysis_request['orchestrator']['artifacts']
    artifacts['canonical_json_bytes'] = 123
    with pytest.raises(ArtifactError, match='Additional properties'):
        validate_document(analysis_request, 'analysis-request.schema.json')


def test_run_result_has_exactly_six_top_level_objects(
    analysis_request: dict[str, object],
) -> None:
    result = analyze_run(analysis_request)
    assert set(result) == {'events', 'identity', 'measurements', 'quality', 'targets', 'verdict'}
    result['hand_entered_override'] = True
    with pytest.raises(ArtifactError, match='Additional properties'):
        validate_document(result, 'run-result.schema.json')

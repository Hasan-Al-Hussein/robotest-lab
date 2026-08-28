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

"""Aggregate exactly 15 ordered canonical Phase 3 run verdicts."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path

from robotest_metrics.aggregation import aggregate_phase3_suite
from robotest_metrics.artifacts import (
    require_directory_within_cap,
    write_json_atomic,
    write_legacy_flattened_csv_atomic,
)
from robotest_metrics.bundle import verify_result_bundle
from robotest_metrics.constants import AGGREGATE_DIRECTORY_MAX_BYTES
from robotest_metrics.errors import ArtifactError, MetricUnavailable
from robotest_metrics.reporting import load_canonical_result
from robotest_metrics.schema_validation import validate_document


def main(argv: Sequence[str] | None = None) -> int:
    """Validate ordered sources and atomically emit JSON/CSV aggregation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', action='append', required=True, type=Path)
    parser.add_argument('--metric', action='append', required=True)
    parser.add_argument('--output-dir', required=True, type=Path)
    arguments = parser.parse_args(argv)
    if len(arguments.input) != 15:
        print('metrics aggregate requires exactly 15 --input paths', file=sys.stderr)
        return 32
    if arguments.output_dir.exists() and any(arguments.output_dir.iterdir()):
        print('metrics aggregate refuses a non-empty output directory', file=sys.stderr)
        return 33
    try:
        loaded = [load_canonical_result(path) for path in arguments.input]
        for path, (result, digest) in zip(arguments.input, loaded, strict=True):
            manifest = verify_result_bundle(path.parent)
            if manifest['identity']['run_result_sha256'] != digest:
                raise ArtifactError('aggregate source does not match its output manifest')
            validate_document(result, 'run-result.schema.json')
        aggregate = aggregate_phase3_suite(
            [result for result, _ in loaded],
            metric_paths=arguments.metric,
        )
        aggregate['identity']['ordered_source_json_sha256'] = [digest for _, digest in loaded]
        arguments.output_dir.mkdir(parents=True, exist_ok=True)
        write_json_atomic(
            aggregate,
            arguments.output_dir / 'aggregate-result.json',
            maximum_bytes=AGGREGATE_DIRECTORY_MAX_BYTES,
        )
        write_legacy_flattened_csv_atomic(
            aggregate,
            arguments.output_dir / 'aggregate-result.csv',
            maximum_bytes=AGGREGATE_DIRECTORY_MAX_BYTES,
        )
        require_directory_within_cap(
            arguments.output_dir,
            maximum_bytes=AGGREGATE_DIRECTORY_MAX_BYTES,
        )
    except (ArtifactError, MetricUnavailable) as exc:
        print(f'metrics aggregate error: {exc}', file=sys.stderr)
        return 32
    status = aggregate['verdict']['automated_status']
    print(
        json.dumps(
            {
                'aggregate_result': str(arguments.output_dir / 'aggregate-result.json'),
                'status': status,
            },
            sort_keys=True,
        )
    )
    return 0 if status == 'PASS' else 30


if __name__ == '__main__':
    raise SystemExit(main())

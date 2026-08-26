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

"""Regenerate bounded human reports from canonical JSON only."""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path

from robotest_metrics.bundle import (
    MANIFEST_NAME,
    MANIFEST_SIDECAR_NAME,
    verify_result_bundle,
    write_result_manifest,
)
from robotest_metrics.errors import ArtifactError
from robotest_metrics.reporting import generate_reports, load_canonical_result


def main(argv: Sequence[str] | None = None) -> int:
    """Generate Markdown, HTML, and charts without accepting other inputs."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output-dir', type=Path)
    arguments = parser.parse_args(argv)
    try:
        output = arguments.output_dir or arguments.input.parent
        refresh_bundle = output.resolve() == arguments.input.parent.resolve()
        if refresh_bundle and (
            (output / MANIFEST_NAME).exists() or (output / MANIFEST_SIDECAR_NAME).exists()
        ):
            verify_result_bundle(output)
        if refresh_bundle:
            (output / MANIFEST_NAME).unlink(missing_ok=True)
            (output / MANIFEST_SIDECAR_NAME).unlink(missing_ok=True)
        result = generate_reports(arguments.input, arguments.output_dir)
        if refresh_bundle:
            document, _ = load_canonical_result(arguments.input)
            result['output_manifest'] = write_result_manifest(output, document)
    except (ArtifactError, OSError) as exc:
        print(f'metrics report error: {exc}', file=sys.stderr)
        return 31
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

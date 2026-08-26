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

"""Reports and charts whose sole input is canonical ``run-result.json``."""

from __future__ import annotations

import hashlib
import html
import io
import json
import math
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from robotest_metrics.artifacts import (
    canonical_json_bytes,
    require_directory_within_cap,
    write_bytes_atomic,
)
from robotest_metrics.constants import LOG_MAX_BYTES, PNG_MAX_BYTES, PNG_MAX_COUNT
from robotest_metrics.errors import ArtifactError


def load_canonical_result(path: str | Path) -> tuple[dict[str, Any], str]:
    """Load strict JSON and reject any bytes not in canonical representation."""
    source = Path(path)
    try:
        payload = source.read_bytes()
        document = json.loads(payload)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ArtifactError(f'cannot load canonical run result: {exc}') from exc
    if not isinstance(document, dict):
        raise ArtifactError('canonical run result must be an object')
    if payload != canonical_json_bytes(document):
        raise ArtifactError('run result is valid JSON but not canonical JSON bytes')
    return document, hashlib.sha256(payload).hexdigest()


def _scalar_rows(value: Any, prefix: str = '') -> list[tuple[str, str]]:
    rows: list[tuple[str, str]] = []
    if isinstance(value, Mapping):
        for key in sorted(value):
            child = f'{prefix}.{key}' if prefix else str(key)
            rows.extend(_scalar_rows(value[key], child))
    elif isinstance(value, list):
        rows.append((prefix, f'[{len(value)} items]'))
    elif value is None:
        rows.append((prefix, 'null'))
    elif isinstance(value, bool):
        rows.append((prefix, 'true' if value else 'false'))
    else:
        rows.append((prefix, str(value)))
    return rows


def _markdown_table(rows: list[tuple[str, str]]) -> str:
    body = ['| Field | Value |', '| --- | --- |']
    body.extend(f'| `{key}` | {value.replace("|", "\\|")} |' for key, value in rows)
    return '\n'.join(body)


def _html_table(rows: list[tuple[str, str]]) -> str:
    cells = ''.join(
        '<tr><td><code>' + html.escape(key) + '</code></td><td>' + html.escape(value) + '</td></tr>'
        for key, value in rows
    )
    return (
        '<table><thead><tr><th>Field</th><th>Value</th></tr></thead><tbody>'
        + cells
        + '</tbody></table>'
    )


def _report_text(document: Mapping[str, Any], source_hash: str) -> tuple[str, str]:
    identity = document.get('identity', {})
    verdict = document.get('verdict', {})
    run_id = identity.get('run_id', 'unknown') if isinstance(identity, Mapping) else 'unknown'
    status = (
        verdict.get('automated_status', 'UNKNOWN') if isinstance(verdict, Mapping) else 'UNKNOWN'
    )
    sections = [
        ('Measurements', _scalar_rows(document.get('measurements', {}))),
        ('Quality', _scalar_rows(document.get('quality', {}))),
        ('Verdict', _scalar_rows(verdict)),
    ]
    markdown = [
        f'# RoboTest Phase 3 Run `{run_id}`',
        '',
        f'- Automated status: **{status}**',
        f'- Canonical JSON SHA-256: `{source_hash}`',
        '- Source: `run-result.json` only',
    ]
    for title, rows in sections:
        markdown.extend(['', f'## {title}', '', _markdown_table(rows)])
    css = (
        'body{font-family:system-ui,sans-serif;max-width:1100px;margin:2rem auto;padding:0 1rem}'
        'table{border-collapse:collapse;width:100%}'
        'th,td{border:1px solid #bbb;padding:.4rem;text-align:left}'
        'code{overflow-wrap:anywhere}.pass{color:#176b2c}.fail{color:#9a1d1d}'
    )
    html_sections = ''.join(
        f'<h2>{html.escape(title)}</h2>{_html_table(rows)}' for title, rows in sections
    )
    status_class = 'pass' if status == 'PASS' else 'fail'
    html_document = (
        '<!doctype html><html lang="en"><head><meta charset="utf-8">'
        '<meta name="viewport" content="width=device-width,initial-scale=1">'
        f'<title>RoboTest Phase 3 {html.escape(str(run_id))}</title>'
        f'<style>{css}</style></head><body>'
        f'<h1>RoboTest Phase 3 Run <code>{html.escape(str(run_id))}</code></h1>'
        f'<p>Automated status: <strong class="{status_class}">'
        f'{html.escape(str(status))}</strong></p>'
        f'<p>Canonical JSON SHA-256: <code>{source_hash}</code><br>Source: '
        '<code>run-result.json</code> only.</p>'
        f'{html_sections}</body></html>\n'
    )
    return '\n'.join(markdown) + '\n', html_document


def _png_bytes(draw: Any) -> bytes:
    try:
        import matplotlib

        matplotlib.use('Agg')
        from matplotlib import pyplot as plt
    except ImportError as exc:
        raise ArtifactError('matplotlib is required to generate Phase 3 charts') from exc
    figure = plt.figure(figsize=(8, 4.5), constrained_layout=True)
    try:
        draw(figure)
        buffer = io.BytesIO()
        figure.savefig(buffer, format='png', dpi=120, metadata={'Software': 'robotest_metrics'})
        return buffer.getvalue()
    finally:
        plt.close(figure)


def _numeric_measurements(document: Mapping[str, Any]) -> list[tuple[str, float]]:
    values: list[tuple[str, float]] = []

    def visit(prefix: str, value: Any) -> None:
        if isinstance(value, Mapping):
            for key in sorted(value):
                visit(f'{prefix}.{key}' if prefix else str(key), value[key])
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            converted = float(value)
            if math.isfinite(converted):
                values.append((prefix, converted))

    visit('', document.get('measurements', {}))
    return values[:12]


def _chart_payloads(document: Mapping[str, Any]) -> list[tuple[str, bytes]]:
    payloads: list[tuple[str, bytes]] = []
    scalars = _numeric_measurements(document)
    if scalars:

        def summary_draw(figure: Any) -> None:
            axis = figure.subplots()
            labels = [item[0] for item in scalars]
            axis.barh(range(len(scalars)), [item[1] for item in scalars], color='#2474b5')
            axis.set_yticks(range(len(labels)), labels=labels)
            axis.invert_yaxis()
            axis.set_title('Canonical measurement summary')
            axis.grid(axis='x', alpha=0.25)

        payloads.append(('measurement-summary.png', _png_bytes(summary_draw)))
    measurements = document.get('measurements', {})
    if not isinstance(measurements, Mapping):
        return payloads
    actual = measurements.get('actual_path')
    samples = actual.get('samples') if isinstance(actual, Mapping) else None
    if isinstance(samples, list) and samples:
        coordinates = [
            (sample.get('x_m'), sample.get('y_m'))
            for sample in samples
            if isinstance(sample, Mapping)
            and isinstance(sample.get('x_m'), (int, float))
            and isinstance(sample.get('y_m'), (int, float))
        ]
        if coordinates:

            def trajectory_draw(figure: Any) -> None:
                axis = figure.subplots()
                axis.plot(
                    [point[0] for point in coordinates],
                    [point[1] for point in coordinates],
                    color='#d1495b',
                    linewidth=2,
                )
                axis.set_aspect('equal', adjustable='datalim')
                axis.set_xlabel('x (m)')
                axis.set_ylabel('y (m)')
                axis.set_title('Ground-truth mission path')
                axis.grid(alpha=0.25)

            payloads.append(('trajectory.png', _png_bytes(trajectory_draw)))
    localization = measurements.get('localization')
    aligned = localization.get('aligned_samples') if isinstance(localization, Mapping) else None
    if isinstance(aligned, list) and aligned:
        positions = [sample.get('position_error_m') for sample in aligned]
        yaws = [sample.get('yaw_error_rad') for sample in aligned]
        stamps = [sample.get('stamp_ns') for sample in aligned]
        if all(isinstance(value, (int, float)) for value in positions + yaws + stamps):
            origin = stamps[0]

            def localization_draw(figure: Any) -> None:
                axis = figure.subplots()
                seconds = [(stamp - origin) / 1_000_000_000 for stamp in stamps]
                axis.plot(seconds, positions, label='position error (m)')
                axis.plot(seconds, yaws, label='yaw error (rad)')
                axis.set_xlabel('elapsed simulation time (s)')
                axis.set_title('Localization error at ground-truth stamps')
                axis.legend()
                axis.grid(alpha=0.25)

            payloads.append(('localization-error.png', _png_bytes(localization_draw)))
    rtf = measurements.get('real_time_factor')
    series = rtf.get('calculated_series') if isinstance(rtf, Mapping) else None
    if (
        isinstance(series, list)
        and series
        and all(isinstance(value, (int, float)) and not isinstance(value, bool) for value in series)
    ):

        def rtf_draw(figure: Any) -> None:
            axis = figure.subplots()
            axis.plot(range(len(series)), series, color='#2a9d8f')
            axis.axhline(1.0, color='#333', linestyle='--', linewidth=1)
            axis.set_xlabel('unpaused interval index')
            axis.set_ylabel('real-time factor')
            axis.set_title('Calculated real-time factor')
            axis.grid(alpha=0.25)

        payloads.append(('real-time-factor.png', _png_bytes(rtf_draw)))
    return payloads


def generate_reports(
    canonical_json_path: str | Path,
    output_directory: str | Path | None = None,
    *,
    include_charts: bool = True,
) -> dict[str, Any]:
    """Generate bounded reports and charts from canonical JSON only."""
    source_path = Path(canonical_json_path)
    output = Path(output_directory) if output_directory is not None else source_path.parent
    document, source_hash = load_canonical_result(source_path)
    markdown, html_document = _report_text(document, source_hash)
    artifacts: dict[str, Any] = {
        'report_markdown': write_bytes_atomic(
            markdown.encode('utf-8'), output / 'report.md', maximum_bytes=LOG_MAX_BYTES
        ),
        'report_html': write_bytes_atomic(
            html_document.encode('utf-8'), output / 'report.html', maximum_bytes=LOG_MAX_BYTES
        ),
    }
    charts = _chart_payloads(document) if include_charts else []
    if len(charts) > PNG_MAX_COUNT:
        raise ArtifactError(f'chart count {len(charts)} exceeds {PNG_MAX_COUNT}')
    artifacts['charts'] = [
        write_bytes_atomic(payload, output / name, maximum_bytes=PNG_MAX_BYTES)
        for name, payload in charts
    ]
    artifacts['canonical_json_sha256'] = source_hash
    artifacts['directory_bytes'] = require_directory_within_cap(output)
    return artifacts

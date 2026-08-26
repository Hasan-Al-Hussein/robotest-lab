#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Produce and validate deterministic post-campaign release summaries and claims."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from phase5_ci import EvidenceError, atomic_write_bytes, canonical_json_bytes

RELEASE_STATUS_START = '<!-- ROBOTEST_RELEASE_STATUS_START -->'
RELEASE_STATUS_END = '<!-- ROBOTEST_RELEASE_STATUS_END -->'
RELEASE_ROADMAP_START = '<!-- ROBOTEST_RELEASE_ROADMAP_START -->'
RELEASE_ROADMAP_END = '<!-- ROBOTEST_RELEASE_ROADMAP_END -->'
GIT_SHA = re.compile(r'^[0-9a-f]{40}$')
SAFE_IDENTIFIER = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$')
PHASE4_RUN_ID = re.compile(r'^phase4-[0-9]{8}T[0-9]{6}Z-[0-9]+$')
MAX_JSON_BYTES = 64 * 1024 * 1024


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise EvidenceError(message)


def _mapping(value: object, label: str) -> dict[str, Any]:
    _require(isinstance(value, dict), f'{label} must be a mapping')
    return value


def _canonical_document(path: Path, label: str) -> tuple[dict[str, Any], bytes]:
    _require(path.is_file() and not path.is_symlink(), f'missing regular {label}: {path}')
    payload = path.read_bytes()
    _require(0 < len(payload) <= MAX_JSON_BYTES, f'{label} exceeds its size bound')
    try:
        document = _mapping(json.loads(payload), label)
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise EvidenceError(f'cannot read {label}: {exc}') from exc
    _require(payload == canonical_json_bytes(document), f'{label} is not canonical JSON')
    return document, payload


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _markdown_scalar(value: object) -> str:
    rendered = json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=False,
        separators=(',', ':'),
        sort_keys=True,
    )
    return rendered.replace('|', '\\|').replace('\n', ' ')


def _phase3_markdown(
    aggregate: Mapping[str, Any],
    *,
    candidate_id: str,
    git_sha: str,
    json_bytes: bytes,
    csv_bytes: bytes,
) -> bytes:
    scenarios = _mapping(aggregate.get('scenarios'), 'Phase 3 aggregate scenarios')
    lines = [
        '# Phase 3 acceptance evidence',
        '',
        f'- Candidate ID: `{candidate_id}`',
        f'- Candidate commit: `{git_sha}`',
        '- Automated verdict: **PASS**',
        f'- Canonical aggregate: [{candidate_id}.json]({candidate_id}.json)',
        f'- Aggregate JSON SHA-256: `{_sha256(json_bytes)}`',
        f'- Matching CSV projection: [{candidate_id}.csv]({candidate_id}.csv)',
        f'- Aggregate CSV SHA-256: `{_sha256(csv_bytes)}`',
        '',
        '## Scenario acceptance',
        '',
        '| Scenario | Passed runs | Total runs | Success rate | Verdict |',
        '| --- | ---: | ---: | ---: | --- |',
    ]
    for scenario_id in sorted(scenarios, key=int):
        scenario = _mapping(scenarios[scenario_id], f'Phase 3 scenario {scenario_id}')
        lines.append(
            f'| {scenario_id} | {scenario.get("numerator")} | {scenario.get("denominator")} | '
            f'{scenario.get("success_rate")} | {scenario.get("verdict")} |'
        )
    lines.extend(
        [
            '',
            '## Aggregated metrics',
            '',
            '| Scenario | Metric | Median | p5 | p95 |',
            '| --- | --- | ---: | ---: | ---: |',
        ]
    )
    for scenario_id in sorted(scenarios, key=int):
        metrics = _mapping(
            scenarios[scenario_id].get('metrics'), f'Phase 3 scenario {scenario_id} metrics'
        )
        for metric in sorted(metrics):
            summary = _mapping(metrics[metric], f'Phase 3 metric {metric}')
            lines.append(
                f'| {scenario_id} | `{metric}` | {_markdown_scalar(summary.get("median"))} | '
                f'{_markdown_scalar(summary.get("p5"))} | '
                f'{_markdown_scalar(summary.get("p95"))} |'
            )
    lines.extend(
        [
            '',
            'This tracked summary is a deterministic projection of the caller-selected canonical '
            'campaign aggregate. Raw run bundles remain in the checksummed local evidence tree.',
            '',
        ]
    )
    return '\n'.join(lines).encode()


def _phase4_markdown(
    result: Mapping[str, Any],
    *,
    run_id: str,
    git_sha: str,
    json_bytes: bytes,
    csv_bytes: bytes,
) -> bytes:
    targets = _mapping(result.get('targets'), 'Phase 4 targets')
    measurements = _mapping(result.get('measurements'), 'Phase 4 measurements')
    quality = _mapping(result.get('quality'), 'Phase 4 quality')
    checks = _mapping(quality.get('checks'), 'Phase 4 checks')
    lines = [
        '# Phase 4 acceptance evidence',
        '',
        f'- Run ID: `{run_id}`',
        f'- Candidate commit: `{git_sha}`',
        '- Scenario 6 verdict: **PASS**',
        f'- Canonical result: [{run_id}.json]({run_id}.json)',
        f'- Result JSON SHA-256: `{_sha256(json_bytes)}`',
        f'- Matching CSV projection: [{run_id}.csv]({run_id}.csv)',
        f'- Result CSV SHA-256: `{_sha256(csv_bytes)}`',
        '',
        '## Frozen targets',
        '',
        '| Target | Value |',
        '| --- | --- |',
        *(f'| `{key}` | {_markdown_scalar(targets[key])} |' for key in sorted(targets)),
        '',
        '## Measurements',
        '',
        '| Measurement | Value |',
        '| --- | --- |',
        *(f'| `{key}` | {_markdown_scalar(measurements[key])} |' for key in sorted(measurements)),
        '',
        '## Acceptance checks',
        '',
        '| Check | Passed |',
        '| --- | --- |',
        *(f'| `{key}` | {_markdown_scalar(checks[key])} |' for key in sorted(checks)),
        '',
        'This tracked summary is a deterministic projection of the caller-selected canonical '
        'Scenario 6 result. Raw acceptance artifacts remain in the checksummed local '
        'evidence tree.',
        '',
    ]
    return '\n'.join(lines).encode()


def final_status_block(candidate_id: str, phase4_run_id: str, git_sha: str) -> str:
    """Render the bounded qualitative README block written only in evidence commit E."""
    return (
        f'{RELEASE_STATUS_START}\n'
        '## Final acceptance status\n\n'
        'Implementation is complete. Canonical Phase 3 campaign evidence, canonical Phase 4 '
        'Scenario 6 evidence, and the exact candidate public-CI proof are recorded:\n\n'
        f'- [Phase 3 candidate evidence](docs/results/phase-3/{candidate_id}.md)\n'
        f'- [Phase 4 Scenario 6 evidence](docs/results/phase-4/{phase4_run_id}.md)\n'
        f'- [Phase 5 candidate CI proof](docs/results/phase-5/remote-{git_sha}.json)\n\n'
        'Release eligibility is determined only after the evidence commit itself passes public '
        'CI and `scripts/verify_all.sh --release-evidence` validates the exact selected '
        'artifacts.\n'
        f'{RELEASE_STATUS_END}\n'
    )


def final_roadmap_block() -> str:
    return (
        f'{RELEASE_ROADMAP_START}\n'
        '| Phase | Status | Deliverable and gate |\n'
        '| --- | --- | --- |\n'
        '| 0 | Verified locally | Environment foundation, exact install manifest, versions, '
        'and resource gates |\n'
        '| 1 | Verified development | Original robot/world, pass-through proxy, headless '
        'gates, and separate RViz evidence |\n'
        '| 2 | Verified development | Bounded seeded Nav2 mission/action result, command '
        'chain, matching JSON/CSV, and global resource/isolation gates |\n'
        '| 3 | Acceptance evidence recorded | Repeatable LiDAR-dropout and odometry-drift '
        'campaigns with metrics |\n'
        '| 4 | Acceptance evidence recorded | Go supervisor, systemd and Debian package '
        'with lifecycle proof |\n'
        '| 5 | Candidate CI recorded; final eligibility uses the evidence-only gate | '
        'Public CI and portfolio evidence on a documented clean commit |\n'
        f'{RELEASE_ROADMAP_END}\n'
    )


def _replace_region(text: str, start_marker: str, end_marker: str, replacement: str) -> str:
    _require(
        text.count(start_marker) == 1 and text.count(end_marker) == 1,
        f'candidate README marker is missing or duplicated: {start_marker}',
    )
    start = text.index(start_marker)
    end = text.index(end_marker, start) + len(end_marker)
    if end < len(text) and text[end] == '\n':
        end += 1
    return text[:start] + replacement + text[end:]


def render_final_readme(
    candidate_readme: bytes,
    *,
    candidate_id: str,
    phase4_run_id: str,
    git_sha: str,
) -> bytes:
    try:
        text = candidate_readme.decode('utf-8')
    except UnicodeError as exc:
        raise EvidenceError('candidate README is not UTF-8') from exc
    rendered = _replace_region(
        text,
        RELEASE_STATUS_START,
        RELEASE_STATUS_END,
        final_status_block(candidate_id, phase4_run_id, git_sha),
    )
    rendered = _replace_region(
        rendered,
        RELEASE_ROADMAP_START,
        RELEASE_ROADMAP_END,
        final_roadmap_block(),
    )
    return rendered.encode()


def render_final_claims(
    candidate_claims: bytes,
    *,
    candidate_id: str,
    phase4_run_id: str,
    git_sha: str,
) -> bytes:
    """Append only the three claims introduced by the bounded final-status block."""
    try:
        candidate = _mapping(json.loads(candidate_claims), 'candidate release claims')
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise EvidenceError(f'cannot read candidate release claims: {exc}') from exc
    _require(
        set(candidate) == {'claims', 'schema_version', 'scope'}
        and isinstance(candidate.get('schema_version'), int)
        and not isinstance(candidate.get('schema_version'), bool)
        and candidate.get('schema_version') == 1
        and isinstance(candidate.get('claims'), list),
        'candidate release claims schema changed',
    )
    claims = [dict(_mapping(item, 'candidate release claim')) for item in candidate['claims']]
    existing_ids = {item.get('id') for item in claims}
    appended = [
        {
            'claim_document': 'README.md',
            'claim_text': 'Canonical Phase 3 campaign evidence',
            'evidence_document': f'docs/results/phase-3/{candidate_id}.md',
            'evidence_text': 'Automated verdict: **PASS**',
            'id': 'phase3-final-acceptance',
        },
        {
            'claim_document': 'README.md',
            'claim_text': 'canonical Phase 4 Scenario 6 evidence',
            'evidence_document': f'docs/results/phase-4/{phase4_run_id}.md',
            'evidence_text': 'Scenario 6 verdict: **PASS**',
            'id': 'phase4-final-acceptance',
        },
        {
            'claim_document': 'README.md',
            'claim_text': 'the exact candidate public-CI proof',
            'evidence_document': f'docs/results/phase-5/remote-{git_sha}.json',
            'evidence_text': '"status":"PASS"',
            'id': 'phase5-candidate-public-ci',
        },
    ]
    _require(
        existing_ids.isdisjoint(item['id'] for item in appended),
        'candidate release claims already contain final claim IDs',
    )
    candidate['claims'] = claims + appended
    return canonical_json_bytes(candidate)


def _selected_evidence(
    phase3_aggregate: Path,
    phase4_scenario6: Path,
) -> dict[str, Any]:
    phase3, phase3_bytes = _canonical_document(phase3_aggregate, 'Phase 3 aggregate')
    phase4, phase4_bytes = _canonical_document(phase4_scenario6, 'Phase 4 Scenario 6 result')
    phase3_identity = _mapping(phase3.get('identity'), 'Phase 3 aggregate identity')
    phase4_identity = _mapping(phase4.get('identity'), 'Phase 4 Scenario 6 identity')
    phase3_verdict = _mapping(phase3.get('verdict'), 'Phase 3 aggregate verdict')
    phase4_verdict = _mapping(phase4.get('verdict'), 'Phase 4 Scenario 6 verdict')
    candidate_id = phase3_identity.get('candidate_id')
    run_id = phase4_identity.get('run_id')
    phase3_sha = phase3_identity.get('git_sha')
    phase4_sha = phase4_identity.get('source_git_commit')
    _require(
        isinstance(candidate_id, str) and SAFE_IDENTIFIER.fullmatch(candidate_id) is not None,
        'Phase 3 candidate ID is invalid',
    )
    _require(
        isinstance(run_id, str) and PHASE4_RUN_ID.fullmatch(run_id) is not None,
        'Phase 4 run ID is invalid',
    )
    _require(
        isinstance(phase3_sha, str)
        and GIT_SHA.fullmatch(phase3_sha) is not None
        and phase4_sha == phase3_sha,
        'release summaries do not bind one candidate commit',
    )
    _require(
        phase3_verdict.get('automated_status') == 'PASS'
        and phase4_verdict
        == {'accepted': True, 'failure_count': 0, 'failures': [], 'status': 'PASS'},
        'release summaries require canonical PASS inputs',
    )
    phase3_csv = phase3_aggregate.with_name('aggregate-result.csv')
    phase4_csv = phase4_scenario6.with_name('scenario6-result.csv')
    for path, label in (
        (phase3_csv, 'Phase 3 aggregate CSV'),
        (phase4_csv, 'Phase 4 Scenario 6 CSV'),
    ):
        _require(path.is_file() and not path.is_symlink(), f'missing regular {label}: {path}')
    return {
        'phase3': phase3,
        'candidate_id': candidate_id,
        'git_sha': phase3_sha,
        'phase3_bytes': phase3_bytes,
        'phase3_csv_bytes': phase3_csv.read_bytes(),
        'phase4_bytes': phase4_bytes,
        'phase4': phase4,
        'phase4_csv_bytes': phase4_csv.read_bytes(),
        'phase4_run_id': run_id,
    }


def _release_root(repository: Path, relative: str, *, create: bool) -> Path:
    current = repository
    for component in Path(relative).parts:
        current = current / component
        _require(not current.is_symlink(), f'release summary root contains a symlink: {current}')
        if not current.exists() and create:
            current.mkdir()
        _require(current.is_dir(), f'missing release summary directory: {current}')
    resolved = current.resolve(strict=True)
    try:
        resolved.relative_to(repository)
    except ValueError as exc:
        raise EvidenceError('release summary root escapes the repository') from exc
    return resolved


def _release_paths(
    repository: Path, selected: dict[str, Any], *, create_roots: bool = False
) -> dict[str, Path]:
    phase3_root = _release_root(repository, 'docs/results/phase-3', create=create_roots)
    phase4_root = _release_root(repository, 'docs/results/phase-4', create=create_roots)
    candidate_id = selected['candidate_id']
    run_id = selected['phase4_run_id']
    return {
        'phase3_json': phase3_root / f'{candidate_id}.json',
        'phase3_csv': phase3_root / f'{candidate_id}.csv',
        'phase3_markdown': phase3_root / f'{candidate_id}.md',
        'phase4_json': phase4_root / f'{run_id}.json',
        'phase4_csv': phase4_root / f'{run_id}.csv',
        'phase4_markdown': phase4_root / f'{run_id}.md',
    }


def _validate_exact_direct_files(paths: dict[str, Path]) -> None:
    for root in {path.parent for path in paths.values()}:
        _require(
            root.is_dir() and not root.is_symlink(), f'missing release summary directory: {root}'
        )
        expected = {path.name for path in paths.values() if path.parent == root}
        observed = {path.name for path in root.iterdir()}
        _require(observed == expected, f'release summary path set is not exact: {root}')
    for path in paths.values():
        _require(
            path.is_file() and not path.is_symlink(), f'missing regular release summary: {path}'
        )


def validate_release_documents(
    repository: Path,
    phase3_aggregate: Path,
    phase4_scenario6: Path,
    candidate_readme: bytes,
    candidate_claims: bytes,
) -> dict[str, Any]:
    """Validate six exact summaries and the README block derived from candidate C."""
    repository = repository.resolve(strict=True)
    selected = _selected_evidence(phase3_aggregate, phase4_scenario6)
    paths = _release_paths(repository, selected)
    _validate_exact_direct_files(paths)
    expected = {
        'phase3_json': selected['phase3_bytes'],
        'phase3_csv': selected['phase3_csv_bytes'],
        'phase3_markdown': _phase3_markdown(
            selected['phase3'],
            candidate_id=selected['candidate_id'],
            git_sha=selected['git_sha'],
            json_bytes=selected['phase3_bytes'],
            csv_bytes=selected['phase3_csv_bytes'],
        ),
        'phase4_json': selected['phase4_bytes'],
        'phase4_csv': selected['phase4_csv_bytes'],
        'phase4_markdown': _phase4_markdown(
            selected['phase4'],
            run_id=selected['phase4_run_id'],
            git_sha=selected['git_sha'],
            json_bytes=selected['phase4_bytes'],
            csv_bytes=selected['phase4_csv_bytes'],
        ),
    }
    for name, payload in expected.items():
        _require(
            b'\r' not in payload and payload.endswith(b'\n'),
            f'release summary text is invalid: {name}',
        )
        if name.endswith('markdown'):
            _require(len(payload) <= 256 * 1024, f'release Markdown exceeds its size bound: {name}')
            try:
                payload.decode('utf-8')
            except UnicodeError as exc:
                raise EvidenceError(f'release Markdown is not UTF-8: {name}') from exc
        _require(paths[name].read_bytes() == payload, f'release summary differs: {paths[name]}')
    expected_readme = render_final_readme(
        candidate_readme,
        candidate_id=selected['candidate_id'],
        phase4_run_id=selected['phase4_run_id'],
        git_sha=selected['git_sha'],
    )
    readme = repository / 'README.md'
    _require(readme.is_file() and not readme.is_symlink(), 'missing regular README.md')
    _require(
        readme.read_bytes() == expected_readme, 'README changed outside the final-status block'
    )
    expected_claims = render_final_claims(
        candidate_claims,
        candidate_id=selected['candidate_id'],
        phase4_run_id=selected['phase4_run_id'],
        git_sha=selected['git_sha'],
    )
    claims_path = _release_root(repository, 'config', create=False) / 'release-claims.json'
    _require(
        claims_path.is_file() and not claims_path.is_symlink(),
        'missing regular release claims',
    )
    _require(
        claims_path.read_bytes() == expected_claims,
        'release claims differ from the exact candidate extension',
    )
    return {
        'candidate_id': selected['candidate_id'],
        'claims_path': 'config/release-claims.json',
        'git_sha': selected['git_sha'],
        'paths': [paths[name].relative_to(repository).as_posix() for name in sorted(paths)],
        'phase4_run_id': selected['phase4_run_id'],
        'readme_path': 'README.md',
    }


def write_release_documents(
    repository: Path,
    phase3_aggregate: Path,
    phase4_scenario6: Path,
) -> dict[str, Any]:
    """Write the deterministic six-file projection and bounded README block."""
    repository = repository.resolve(strict=True)
    selected = _selected_evidence(phase3_aggregate, phase4_scenario6)
    paths = _release_paths(repository, selected, create_roots=True)
    for root in {path.parent for path in paths.values()}:
        expected = {path.name for path in paths.values() if path.parent == root}
        observed = {path.name for path in root.iterdir()}
        _require(observed <= expected, f'release summary directory contains stale files: {root}')
        for path in paths.values():
            if path.parent == root:
                _require(not path.is_symlink(), f'release summary is a symlink: {path}')
    payloads = {
        'phase3_json': selected['phase3_bytes'],
        'phase3_csv': selected['phase3_csv_bytes'],
        'phase3_markdown': _phase3_markdown(
            selected['phase3'],
            candidate_id=selected['candidate_id'],
            git_sha=selected['git_sha'],
            json_bytes=selected['phase3_bytes'],
            csv_bytes=selected['phase3_csv_bytes'],
        ),
        'phase4_json': selected['phase4_bytes'],
        'phase4_csv': selected['phase4_csv_bytes'],
        'phase4_markdown': _phase4_markdown(
            selected['phase4'],
            run_id=selected['phase4_run_id'],
            git_sha=selected['git_sha'],
            json_bytes=selected['phase4_bytes'],
            csv_bytes=selected['phase4_csv_bytes'],
        ),
    }
    for name, payload in payloads.items():
        atomic_write_bytes(paths[name], payload)
    readme = repository / 'README.md'
    _require(readme.is_file() and not readme.is_symlink(), 'missing regular README.md')
    candidate_readme = readme.read_bytes()
    claims_path = _release_root(repository, 'config', create=False) / 'release-claims.json'
    _require(
        claims_path.is_file() and not claims_path.is_symlink(),
        'missing regular release claims',
    )
    candidate_claims = claims_path.read_bytes()
    atomic_write_bytes(
        readme,
        render_final_readme(
            candidate_readme,
            candidate_id=selected['candidate_id'],
            phase4_run_id=selected['phase4_run_id'],
            git_sha=selected['git_sha'],
        ),
    )
    atomic_write_bytes(
        claims_path,
        render_final_claims(
            candidate_claims,
            candidate_id=selected['candidate_id'],
            phase4_run_id=selected['phase4_run_id'],
            git_sha=selected['git_sha'],
        ),
    )
    return validate_release_documents(
        repository,
        phase3_aggregate,
        phase4_scenario6,
        candidate_readme,
        candidate_claims,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository', type=Path, required=True)
    parser.add_argument('--phase3-aggregate', type=Path, required=True)
    parser.add_argument('--phase4-scenario6', type=Path, required=True)
    return parser


def main() -> int:
    arguments = _parser().parse_args()
    try:
        report = write_release_documents(
            arguments.repository,
            arguments.phase3_aggregate,
            arguments.phase4_scenario6,
        )
    except (EvidenceError, OSError, UnicodeError, json.JSONDecodeError) as error:
        raise SystemExit(str(error)) from error
    print(canonical_json_bytes(report).decode(), end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

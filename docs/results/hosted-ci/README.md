# Hosted CI receipt

This directory preserves the downloaded evidence from GitHub Actions run
[`33274853279`](https://github.com/Hasan-Al-Hussein/robotest-lab/actions/runs/33274853279).
The files in [`20260829T210238Z-19597/`](20260829T210238Z-19597/) are an
unchanged copy of that run's uploaded artifact.

## Recorded result

| Field | Recorded value |
| --- | --- |
| Source commit | Clean `039effa496abbf69d006ccca729c6ed26116c2eb` |
| Runner | GitHub-hosted Linux, Ubuntu 24.04, x86-64 |
| Verification mode | `ci`, level `L2` |
| Aggregate result | `PASS`, 27 of 27 checks passed |
| Pure-Python suite | 1,266 passed |
| Colcon package suite | 1,045 tests, 0 errors, 0 failures, 11 skipped |
| Go race suite | 4 packages passed |
| Python lint/format | Ruff check passed; 138 files already formatted |

The authoritative machine-readable summary is
[`verification-summary.json`](20260829T210238Z-19597/verification-summary.json).
The corresponding logs are
[`pure-python-tests.log`](20260829T210238Z-19597/pure-python-tests.log),
[`colcon-test-result.log`](20260829T210238Z-19597/colcon-test-result.log), and
[`go-race-tests.log`](20260829T210238Z-19597/go-race-tests.log). Source and
runner provenance are retained in
[`provenance.json`](20260829T210238Z-19597/provenance.json).

## Scope boundary

This receipt proves a clean-source hosted build, static checks, unit/package
tests, and CLI help smoke for the exact commit above. The verifier labels the
result **L2/non-live**. Its runtime-scope fields record that Gazebo, the Phase 1
and Phase 2 live verifiers, the Phase 3 campaign, Phase 4 `--apply`, hardware,
and systemd mutation were not started.

The preserved CSV also records `remote_ci_verified=false`. That field is part
of the project's later release-evidence gate; it is retained unchanged and
means this L2 artifact must not be presented as final remote/release
qualification. The GitHub runner context and run ID identify where this
particular artifact was produced, but do not raise its verification level.

## Integrity

[`SHA256SUMS`](20260829T210238Z-19597/SHA256SUMS) covers the run files, and
[`checksum-validation.txt`](20260829T210238Z-19597/checksum-validation.txt)
contains the run-time validation receipt. Recompute the manifest from inside
the run directory before relying on a redistributed copy.

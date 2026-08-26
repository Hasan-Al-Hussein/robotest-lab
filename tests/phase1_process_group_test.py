#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Pure and bounded regressions for stable Phase 1 group cleanup."""

from __future__ import annotations

import errno
import importlib.util
import signal
import subprocess
import sys
from pathlib import Path
from types import ModuleType

import pytest

REPOSITORY = Path(__file__).resolve().parents[1]
VERIFY_SCRIPT = REPOSITORY / 'scripts/verify_phase1.sh'
SHELL_HELPER = REPOSITORY / 'scripts/phase1_process_group.sh'
PIDFD_HELPER = REPOSITORY / 'scripts/phase1_pidfd_group.py'


def _load_pidfd_module() -> ModuleType:
    specification = importlib.util.spec_from_file_location('phase1_pidfd_group', PIDFD_HELPER)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


PIDFD_MODULE = _load_pidfd_module()


def _run_harness(
    tmp_path: Path,
    body: str,
    timeout: int = 15,
    *,
    start_new_session: bool = False,
) -> subprocess.CompletedProcess[str]:
    harness = tmp_path / 'harness.sh'
    evidence = tmp_path / 'cleanup.tsv'
    harness.write_text(
        '#!/usr/bin/env bash\nset -Eeuo pipefail\nIFS=$\'\\n\\t\'\nsource "$1"\n' + body,
        encoding='utf-8',
    )
    return subprocess.run(
        ['bash', str(harness), str(SHELL_HELPER), str(PIDFD_HELPER), str(evidence)],
        cwd=REPOSITORY,
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
        start_new_session=start_new_session,
    )


REAL_GROUP_PREAMBLE = r"""
pidfd_helper="$2"
evidence_path="$3"
run_directory="$(dirname "$evidence_path")"
leader_pid=""
retained_fd=""
token="fixture-$(date +%s%N)"
parent_pid="$BASHPID"

cleanup_fixture() {
  if [[ "$retained_fd" =~ ^[0-9]+$ ]]; then
    python3 "$pidfd_helper" send --fd 9 --signal KILL \
      >/dev/null 2>&1 9<&"$retained_fd" || true
    exec {retained_fd}<&-
    retained_fd=""
  fi
  if [[ -n "$leader_pid" ]]; then
    wait "$leader_pid" 2>/dev/null || true
  fi
}
trap cleanup_fixture EXIT

validate_fixture() {
  python3 "$pidfd_helper" validate \
    --fd 9 \
    --parent-pid "$parent_pid" \
    --pid "$leader_pid" \
    --token "$token" \
    > "$run_directory/validation.json" 9<&"$retained_fd"
  leader_start_ticks="$(python3 -c \
    'import json,sys; print(json.load(open(sys.argv[1]))["identity"]["start_ticks"])' \
    "$run_directory/validation.json")"
}
"""


def test_pidfd_cleanup_drains_survivor_after_leader_is_reaped(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        REAL_GROUP_PREAMBLE
        + r"""
ROBOTEST_PHASE1_GROUP_TOKEN="$token" setsid bash -c 'sleep 0.3; sleep 30 &' &
leader_pid=$!
exec {retained_fd}<"/proc/$leader_pid"
validate_fixture
wait "$leader_pid"
survivor_pid="$(phase1_process_group_snapshot "$leader_pid" \
  | awk 'substr($4, 1, 1) != "Z" {print $1; exit}')"
[[ "$survivor_pid" =~ ^[1-9][0-9]*$ ]]

phase1_stop_owned_process_group \
  launch "$leader_pid" "$leader_pid" "$leader_pid" "$leader_start_ticks" \
  "$evidence_path" "$pidfd_helper" "$retained_fd" "$run_directory" 0

[[ "$PHASE1_GROUP_TERM_SENT" == true ]]
[[ "$PHASE1_GROUP_KILL_SENT" == false ]]
[[ "$PHASE1_GROUP_DRAINED" == true ]]
[[ ! -e "$run_directory/pidfd-kill.json" ]]
! kill -0 "$survivor_pid" 2>/dev/null
grep -Fq $'event\tterm-sent\t' "$evidence_path"
grep -Fq $'event\tcleanup-pass\t' "$evidence_path"

cleanup_fixture
trap - EXIT
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize(('helper_status', 'expected_status'), [(1, 0), (2, 2)])
def test_numeric_live_drain_preserves_helper_status(
    tmp_path: Path, helper_status: int, expected_status: int
) -> None:
    result = _run_harness(
        tmp_path,
        f"""
phase1_process_group_has_live_members() {{ return {helper_status}; }}
set +e
phase1_wait_for_numeric_live_drain 4242
drain_status=$?
set -e
[[ "$drain_status" -eq {expected_status} ]]
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize(('helper_status', 'expected_status'), [(1, 0), (2, 2)])
def test_numeric_sampler_preserves_helper_status(
    tmp_path: Path, helper_status: int, expected_status: int
) -> None:
    result = _run_harness(
        tmp_path,
        f"""
sample_callback() {{ return 0; }}
phase1_process_group_has_live_members() {{ return {helper_status}; }}
set +e
phase1_sample_numeric_group_until_drain 4242 sample_callback 0
sampler_status=$?
set -e
[[ "$sampler_status" -eq {expected_status} ]]
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_evidence_append_failure_does_not_skip_physical_term_drain(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        REAL_GROUP_PREAMBLE
        + r"""
ROBOTEST_PHASE1_GROUP_TOKEN="$token" setsid sleep 30 &
leader_pid=$!
exec {retained_fd}<"/proc/$leader_pid"
validate_fixture

phase1_cleanup_record_event() { return 1; }
set +e
phase1_stop_owned_process_group \
  launch "$leader_pid" "$leader_pid" "$leader_pid" "$leader_start_ticks" \
  "$evidence_path" "$pidfd_helper" "$retained_fd" "$run_directory"
cleanup_status=$?
set -e

[[ "$cleanup_status" -eq 3 ]]
[[ "$PHASE1_GROUP_TERM_SENT" == true ]]
[[ "$PHASE1_GROUP_KILL_SENT" == false ]]
[[ "$PHASE1_GROUP_DRAINED" == true ]]
[[ ! -e "$run_directory/pidfd-kill.json" ]]
! kill -0 "$leader_pid" 2>/dev/null

cleanup_fixture
trap - EXIT
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_evidence_prepare_failure_does_not_skip_physical_term_drain(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        REAL_GROUP_PREAMBLE
        + r"""
ROBOTEST_PHASE1_GROUP_TOKEN="$token" setsid sleep 30 &
leader_pid=$!
exec {retained_fd}<"/proc/$leader_pid"
validate_fixture

set +e
phase1_stop_owned_process_group \
  launch "$leader_pid" "$leader_pid" "$leader_pid" "$leader_start_ticks" \
  "$run_directory" "$pidfd_helper" "$retained_fd" "$run_directory"
cleanup_status=$?
set -e

[[ "$cleanup_status" -eq 3 ]]
[[ "$PHASE1_GROUP_TERM_SENT" == true ]]
[[ "$PHASE1_GROUP_KILL_SENT" == false ]]
[[ "$PHASE1_GROUP_DRAINED" == true ]]
[[ ! -e "$run_directory/pidfd-kill.json" ]]
! kill -0 "$leader_pid" 2>/dev/null

cleanup_fixture
trap - EXIT
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_pidfd_json_persistence_failure_still_physically_drains(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        REAL_GROUP_PREAMBLE
        + r"""
ROBOTEST_PHASE1_GROUP_TOKEN="$token" setsid sleep 30 &
leader_pid=$!
exec {retained_fd}<"/proc/$leader_pid"
validate_fixture
mkdir "$run_directory/pidfd-term.json"

set +e
phase1_stop_owned_process_group \
  launch "$leader_pid" "$leader_pid" "$leader_pid" "$leader_start_ticks" \
  "$evidence_path" "$pidfd_helper" "$retained_fd" "$run_directory"
cleanup_status=$?
set -e

[[ "$cleanup_status" -eq 3 ]]
[[ "$PHASE1_GROUP_TERM_SENT" == true ]]
[[ "$PHASE1_GROUP_KILL_SENT" == false ]]
[[ "$PHASE1_GROUP_DRAINED" == true ]]
[[ -d "$run_directory/pidfd-term.json" ]]
[[ ! -e "$run_directory/pidfd-kill.json" ]]
! kill -0 "$leader_pid" 2>/dev/null
grep -Fq $'event\tcleanup-pass\t' "$evidence_path"
grep -Fq $'\tterm-drained' "$evidence_path"

cleanup_fixture
trap - EXIT
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_snapshot_and_fallback_evidence_failure_is_nonzero(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        r"""
pidfd_helper="$2"
evidence_path="$3"
run_directory="$(dirname "$evidence_path")"

phase1_cleanup_record_snapshot() { return 1; }
phase1_cleanup_record_event() {
  [[ "$2" != snapshot-unavailable ]]
}
phase1_pidfd_action() {
  PHASE1_PIDFD_ACTION_STATUS=empty
  return 0
}
phase1_wait_for_leader() {
  PHASE1_GROUP_WAIT_STATUS=0
  return 0
}

set +e
phase1_stop_owned_process_group \
  launch 4242 4242 4242 1 "$evidence_path" "$pidfd_helper" 9 "$run_directory"
cleanup_status=$?
set -e
[[ "$cleanup_status" -eq 3 ]]
[[ "$PHASE1_GROUP_DRAINED" == true ]]
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize(
    ('first_wait_status', 'expected_event', 'forbidden_event'),
    [
        (1, 'stable-group-present', 'pidfd-inspection-unavailable'),
        (2, 'pidfd-inspection-unavailable', 'stable-group-present'),
    ],
)
def test_term_probe_rc_is_preserved_before_safe_escalation(
    tmp_path: Path, first_wait_status: int, expected_event: str, forbidden_event: str
) -> None:
    result = _run_harness(
        tmp_path,
        f"""
pidfd_helper="$2"
evidence_path="$3"
run_directory="$(dirname "$evidence_path")"
pidfd_wait_calls=0

phase1_process_group_snapshot() {{ return 0; }}
phase1_wait_for_numeric_live_drain() {{ return 0; }}
phase1_wait_for_leader() {{ PHASE1_GROUP_WAIT_STATUS=0; }}
phase1_pidfd_action() {{
  PHASE1_PIDFD_EVIDENCE_ERROR=false
  if [[ "$4" == probe ]]; then
    PHASE1_PIDFD_ACTION_STATUS=present
  else
    PHASE1_PIDFD_ACTION_STATUS=sent
  fi
}}
phase1_wait_for_pidfd_empty() {{
  pidfd_wait_calls=$((pidfd_wait_calls + 1))
  (( pidfd_wait_calls == 1 )) && return {first_wait_status}
  return 0
}}

phase1_stop_owned_process_group \
  launch 4242 4242 4242 1 "$evidence_path" "$pidfd_helper" 9 "$run_directory"

[[ "$PHASE1_GROUP_TERM_SENT" == true ]]
[[ "$PHASE1_GROUP_KILL_SENT" == true ]]
[[ "$PHASE1_GROUP_DRAINED" == true ]]
grep -Fq $'event\t{expected_event}\t' "$evidence_path"
! grep -Fq $'event\t{forbidden_event}\t' "$evidence_path"
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize(
    ('probe_status', 'cleanup_status', 'expected_event', 'forbidden_event'),
    [
        (1, 4, 'residual-stable-group', 'pidfd-inspection-unavailable-after-kill'),
        (2, 3, 'pidfd-inspection-unavailable-after-kill', 'residual-stable-group'),
    ],
)
def test_post_kill_probe_rc_is_preserved(
    tmp_path: Path,
    probe_status: int,
    cleanup_status: int,
    expected_event: str,
    forbidden_event: str,
) -> None:
    result = _run_harness(
        tmp_path,
        f"""
pidfd_helper="$2"
evidence_path="$3"
run_directory="$(dirname "$evidence_path")"

phase1_process_group_snapshot() {{ return 0; }}
phase1_wait_for_numeric_live_drain() {{ return 1; }}
phase1_wait_for_leader() {{ PHASE1_GROUP_WAIT_STATUS=0; }}
phase1_wait_for_pidfd_empty() {{ return {probe_status}; }}
phase1_pidfd_action() {{
  PHASE1_PIDFD_EVIDENCE_ERROR=false
  if [[ "$4" == probe ]]; then
    PHASE1_PIDFD_ACTION_STATUS=present
  else
    PHASE1_PIDFD_ACTION_STATUS=sent
  fi
}}

set +e
phase1_stop_owned_process_group \
  launch 4242 4242 4242 1 "$evidence_path" "$pidfd_helper" 9 "$run_directory"
actual_cleanup_status=$?
set -e

[[ "$actual_cleanup_status" -eq {cleanup_status} ]]
[[ "$PHASE1_GROUP_KILL_SENT" == true ]]
[[ "$PHASE1_GROUP_DRAINED" == false ]]
grep -Fq '{expected_event}' "$evidence_path"
! grep -Fq '{forbidden_event}' "$evidence_path"
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_pidfd_cleanup_escalates_for_term_resistant_group(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        REAL_GROUP_PREAMBLE
        + r"""
ROBOTEST_PHASE1_GROUP_TOKEN="$token" \
  setsid bash -c 'trap "" TERM; while true; do sleep 1; done' &
leader_pid=$!
exec {retained_fd}<"/proc/$leader_pid"
validate_fixture

phase1_stop_owned_process_group \
  launch "$leader_pid" "$leader_pid" "$leader_pid" "$leader_start_ticks" \
  "$evidence_path" "$pidfd_helper" "$retained_fd" "$run_directory"

[[ "$PHASE1_GROUP_TERM_SENT" == true ]]
[[ "$PHASE1_GROUP_KILL_SENT" == true ]]
[[ "$PHASE1_GROUP_DRAINED" == true ]]
grep -Fq $'event\tkill-sent\t' "$evidence_path"
grep -Fq $'event\tcleanup-pass\t' "$evidence_path"

cleanup_fixture
trap - EXIT
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_already_empty_stable_group_is_never_signalled(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        REAL_GROUP_PREAMBLE
        + r"""
ROBOTEST_PHASE1_GROUP_TOKEN="$token" setsid sleep 0.3 &
leader_pid=$!
exec {retained_fd}<"/proc/$leader_pid"
validate_fixture
wait "$leader_pid"

phase1_stop_owned_process_group \
  launch "$leader_pid" "$leader_pid" "$leader_pid" "$leader_start_ticks" \
  "$evidence_path" "$pidfd_helper" "$retained_fd" "$run_directory" 0

[[ "$PHASE1_GROUP_TERM_SENT" == false ]]
[[ "$PHASE1_GROUP_KILL_SENT" == false ]]
[[ "$PHASE1_GROUP_DRAINED" == true ]]
[[ ! -e "$run_directory/pidfd-term.json" ]]
[[ ! -e "$run_directory/pidfd-kill.json" ]]
grep -Fq $'event\talready-empty\t' "$evidence_path"

cleanup_fixture
trap - EXIT
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_procfd_acquisition_rejects_wrong_nonce_without_signalling(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        REAL_GROUP_PREAMBLE
        + r"""
ROBOTEST_PHASE1_GROUP_TOKEN="$token" setsid sleep 30 &
leader_pid=$!
exec {retained_fd}<"/proc/$leader_pid"
set +e
python3 "$pidfd_helper" validate \
  --fd 9 \
  --parent-pid "$parent_pid" \
  --pid "$leader_pid" \
  --token wrong-token \
  > "$run_directory/wrong-validation.json" 9<&"$retained_fd"
validation_status=$?
set -e

[[ "$validation_status" -eq 3 ]]
kill -0 "$leader_pid"
python3 -c \
  'import json,sys; assert json.load(open(sys.argv[1]))["status"] == "error"' \
  "$run_directory/wrong-validation.json"

cleanup_fixture
trap - EXIT
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_pre_spawn_gate_validates_before_releasing_workload(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        r"""
pidfd_helper="$2"
evidence_path="$3"
run_directory="$(dirname "$evidence_path")"
go_fifo="$run_directory/go.pipe"
ready_fifo="$run_directory/ready.pipe"
marker="$run_directory/workload-started"
token="gate-$(date +%s%N)"
parent_pid="$BASHPID"
leader_pid=""
retained_fd=""
go_fd=""
ready_fd=""

cleanup_fixture() {
  if [[ "$retained_fd" =~ ^[0-9]+$ ]]; then
    python3 "$pidfd_helper" send --fd 9 --signal KILL \
      >/dev/null 2>&1 9<&"$retained_fd" || true
    exec {retained_fd}<&-
  fi
  [[ "$go_fd" =~ ^[0-9]+$ ]] && exec {go_fd}>&- || true
  [[ "$ready_fd" =~ ^[0-9]+$ ]] && exec {ready_fd}>&- || true
  [[ -n "$leader_pid" ]] && wait "$leader_pid" 2>/dev/null || true
}
trap cleanup_fixture EXIT

mkfifo "$go_fifo" "$ready_fifo"
exec {go_fd}<>"$go_fifo"
exec {ready_fd}<>"$ready_fifo"
ROBOTEST_PHASE1_GROUP_TOKEN="$token" setsid bash -c '
  set -Eeuo pipefail
  go_fifo="$1"
  ready_fifo="$2"
  shift 2
  exec 8<"$go_fifo"
  printf "READY\n" > "$ready_fifo"
  release_token=""
  if ! IFS= read -r -t 10 release_token <&8; then
    exec 8<&-
    exit 125
  fi
  exec 8<&-
  [[ "$release_token" == "$ROBOTEST_PHASE1_GROUP_TOKEN" ]] || exit 125
  exec "$@"
' _ "$go_fifo" "$ready_fifo" \
  bash -c 'printf "started\n" > "$1"; sleep 30' _ "$marker" \
  {go_fd}>&- {ready_fd}>&- &
leader_pid=$!
exec {retained_fd}<"/proc/$leader_pid"
python3 "$pidfd_helper" validate \
  --fd 9 --parent-pid "$parent_pid" --pid "$leader_pid" --token "$token" \
  > "$run_directory/validation.json" 9<&"$retained_fd"
leader_start_ticks="$(python3 -c \
  'import json,sys; print(json.load(open(sys.argv[1]))["identity"]["start_ticks"])' \
  "$run_directory/validation.json")"
ready=""
IFS= read -r -t 2 -u "$ready_fd" ready
[[ "$ready" == READY ]]
[[ ! -e "$marker" ]]
printf '%s\n' "$token" >&"$go_fd"
exec {go_fd}>&-
exec {ready_fd}>&-
for _ in {1..20}; do
  [[ -e "$marker" ]] && break
  sleep 0.05
done
grep -Fxq started "$marker"

phase1_stop_owned_process_group \
  launch "$leader_pid" "$leader_pid" "$leader_pid" "$leader_start_ticks" \
  "$evidence_path" "$pidfd_helper" "$retained_fd" "$run_directory"
[[ "$PHASE1_GROUP_DRAINED" == true ]]

cleanup_fixture
trap - EXIT
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_unreleased_gate_abort_reaps_wrapper_without_starting_workload(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        r"""
evidence_path="$3"
run_directory="$(dirname "$evidence_path")"
go_fifo="$run_directory/go.pipe"
ready_fifo="$run_directory/ready.pipe"
marker="$run_directory/workload-started"
go_fd=""
ready_fd=""
leader_pid=""

cleanup_fixture() {
  [[ "$go_fd" =~ ^[0-9]+$ ]] && exec {go_fd}>&- || true
  [[ "$ready_fd" =~ ^[0-9]+$ ]] && exec {ready_fd}>&- || true
  [[ -n "$leader_pid" ]] && wait "$leader_pid" 2>/dev/null || true
  rm -f -- "$go_fifo" "$ready_fifo"
}
trap cleanup_fixture EXIT

mkfifo "$go_fifo" "$ready_fifo"
exec {go_fd}<>"$go_fifo"
exec {ready_fd}<>"$ready_fifo"
ROBOTEST_PHASE1_GROUP_TOKEN=expected-release-token \
  setsid bash -c '
    set -Eeuo pipefail
    go_fifo="$1"
    ready_fifo="$2"
    marker="$3"
    exec 8<"$go_fifo"
    printf "READY\n" > "$ready_fifo"
    release_token=""
    if ! IFS= read -r -t 10 release_token <&8; then
      exec 8<&-
      exit 125
    fi
    exec 8<&-
    [[ "$release_token" == "$ROBOTEST_PHASE1_GROUP_TOKEN" ]] || exit 125
    printf "started\n" > "$marker"
    sleep 30
  ' _ "$go_fifo" "$ready_fifo" "$marker" \
  {go_fd}>&- {ready_fd}>&- &
leader_pid=$!
ready=""
IFS= read -r -t 2 -u "$ready_fd" ready
[[ "$ready" == READY ]]

phase1_abort_unreleased_launch_gate \
  launch "$leader_pid" "$go_fd" "$evidence_path" 3

[[ "$PHASE1_GROUP_DRAINED" == true ]]
[[ "$PHASE1_GROUP_WAIT_STATUS" -eq 125 ]]
[[ "$PHASE1_GROUP_TERM_SENT" == false ]]
[[ "$PHASE1_GROUP_KILL_SENT" == false ]]
[[ ! -e "$marker" ]]
grep -Fq $'event\tgate-abort-sent\t' "$evidence_path"
grep -Fq $'event\tcleanup-pass\t' "$evidence_path"

cleanup_fixture
trap - EXIT
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_validated_empty_mock_never_rechecks_or_signals_numeric_group(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        r"""
pidfd_helper="$2"
evidence_path="$3"
run_directory="$(dirname "$evidence_path")"
actions="$run_directory/actions.txt"
numeric_checks="$run_directory/numeric-checks.txt"

phase1_pidfd_action() {
  printf '%s\n' "$4" >> "$actions"
  PHASE1_PIDFD_ACTION_STATUS=empty
  return 0
}
phase1_process_group_has_live_members() {
  printf 'unexpected\n' >> "$numeric_checks"
  return 0
}
phase1_wait_for_leader() {
  PHASE1_GROUP_WAIT_STATUS=0
  return 0
}

phase1_stop_owned_process_group \
  launch 4242 4242 4242 1 "$evidence_path" "$pidfd_helper" 9 "$run_directory"

[[ "$PHASE1_GROUP_DRAINED" == true ]]
[[ "$PHASE1_GROUP_TERM_SENT" == false ]]
[[ "$PHASE1_GROUP_KILL_SENT" == false ]]
[[ "$(<"$actions")" == probe ]]
[[ ! -e "$numeric_checks" ]]
grep -Fq $'event\talready-empty\t' "$evidence_path"
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_pidfd_hard_error_has_no_numeric_signal_fallback(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        r"""
pidfd_helper="$2"
evidence_path="$3"
run_directory="$(dirname "$evidence_path")"
numeric_signals="$run_directory/numeric-signals.txt"

phase1_pidfd_action() { return 3; }
kill() {
  printf 'unexpected\n' >> "$numeric_signals"
  return 99
}
set +e
phase1_stop_owned_process_group \
  launch 4242 4242 4242 1 "$evidence_path" "$pidfd_helper" 9 "$run_directory"
cleanup_status=$?
set -e

[[ "$cleanup_status" -eq 3 ]]
[[ ! -e "$numeric_signals" ]]
grep -Fq $'event\tcleanup-failed\t' "$evidence_path"
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize(
    ('command', 'reported_status', 'protocol_status'),
    [('probe', 'sent', 'present'), ('send', 'present', 'sent')],
)
def test_command_status_mismatch_is_rejected_as_evidence(
    tmp_path: Path, command: str, reported_status: str, protocol_status: str
) -> None:
    result = _run_harness(
        tmp_path,
        f"""
pidfd_helper="$2"
evidence_path="$3"
run_directory="$(dirname "$evidence_path")"
PHASE1_PIDFD_EVIDENCE_ERROR=false
phase1_run_pidfd_command() {{
  printf '%s\\n' '{{"status":"{reported_status}"}}'
  return 0
}}

phase1_pidfd_action \
  "$pidfd_helper" 9 "$run_directory/action.json" {command}
[[ "$PHASE1_PIDFD_ACTION_STATUS" == {protocol_status} ]]
[[ "$PHASE1_PIDFD_EVIDENCE_ERROR" == true ]]
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_pending_term_is_deferred_until_critical_section_ends(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        r"""
run_directory="$(dirname "$3")"
PENDING_SIGNAL_STATUS=""
SIGNAL_DEFER_DEPTH=0
FINALIZED=0
trap 'phase1_record_signal 143' TERM

phase1_begin_signal_deferral
kill -TERM "$BASHPID"
[[ "$PENDING_SIGNAL_STATUS" -eq 143 ]]
[[ "$SIGNAL_DEFER_DEPTH" -eq 1 ]]
printf 'before-exit\n' > "$run_directory/before-exit"
phase1_end_signal_deferral
printf 'unexpected\n' > "$run_directory/after-exit"
""",
    )
    assert result.returncode == 143, result.stdout + result.stderr
    assert (tmp_path / 'before-exit').read_text(encoding='utf-8') == 'before-exit\n'
    assert not (tmp_path / 'after-exit').exists()


def test_release_gate_is_atomic_when_term_arrives_after_timestamp(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        r"""
run_directory="$(dirname "$3")"
release_capture="$run_directory/release-token.txt"
release_state="$run_directory/release-state.txt"
release_token=expected-release-token
release_timestamp=""
harness_pid="$BASHPID"
PENDING_SIGNAL_STATUS=""
SIGNAL_DEFER_DEPTH=0
FINALIZED=0

exec {go_fd}>"$release_capture"
date() {
  printf '2026-08-26T12:00:00Z\n'
  kill -TERM "$harness_pid"
}
trap 'printf "%s|%s\n" "$release_token" "$release_timestamp" > "$release_state"' EXIT
trap 'phase1_record_signal 143' TERM

phase1_release_launch_gate "$go_fd" release_token release_timestamp
printf 'unexpected\n' > "$run_directory/after-release"
""",
    )
    assert result.returncode == 143, result.stdout + result.stderr
    assert (tmp_path / 'release-token.txt').read_text(encoding='utf-8') == (
        'expected-release-token\n'
    )
    assert (tmp_path / 'release-state.txt').read_text(encoding='utf-8') == (
        '|2026-08-26T12:00:00Z\n'
    )
    assert not (tmp_path / 'after-release').exists()


def test_sampler_child_identity_is_retained_when_final_reap_is_unconfirmed(
    tmp_path: Path,
) -> None:
    result = _run_harness(
        tmp_path,
        r"""
run_directory="$(dirname "$3")"
wait_calls="$run_directory/waits.txt"
signal_calls="$run_directory/signals.txt"

phase1_wait_for_owned_child_bounded() {
  printf 'wait\n' >> "$wait_calls"
  return 1
}
kill() {
  printf '%s:%s\n' "$1" "${2:-}" >> "$signal_calls"
  return 0
}

set +e
phase1_stop_exact_child_bounded 4242
cleanup_status=$?
set -e

[[ "$cleanup_status" -eq 1 ]]
[[ "$PHASE1_CHILD_DRAINED" == false ]]
[[ "$PHASE1_CHILD_OUTCOME_STATUS" -eq 125 ]]
[[ "$(wc -l < "$wait_calls")" -eq 3 ]]
grep -Fxq -- '-TERM:4242' "$signal_calls"
grep -Fxq -- '-KILL:4242' "$signal_calls"
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_undrained_launch_cleanup_retries_and_preserves_first_failure(
    tmp_path: Path,
) -> None:
    result = _run_harness(
        tmp_path,
        r"""
run_directory="$(dirname "$3")"
verify_script="$PWD/scripts/verify_phase1.sh"
eval "$(sed -n '/^stop_owned_launch() {$/,/^}$/p' "$verify_script")"

RUN_DIR="$run_directory"
PIDFD_GROUP_HELPER="$2"
SAMPLER_PID=""
LAUNCH_PID=4242
LAUNCH_PGID=4242
LAUNCH_SID=4242
LAUNCH_START_TICKS=1
LAUNCH_PIDFD_VALIDATED=true
LAUNCH_STARTED_UTC=""
LAUNCH_STOPPED_UTC=""
LAUNCH_WAIT_STATUS=""
LAUNCH_TERM_SENT=false
LAUNCH_KILL_SENT=false
LAUNCH_GROUP_DRAINED=""
LAUNCH_CLEANUP_STATUS=""
LAUNCH_CLEANUP_FIRST_FAILURE=""
LAUNCH_CLEANUP_ATTEMPTED=0
PENDING_SIGNAL_STATUS=""
SIGNAL_DEFER_DEPTH=0
FINALIZED=0
attempts=0
exec {LAUNCH_PIDFD}</dev/null
retained_fd_number="$LAUNCH_PIDFD"

sample_launch_group() { return 0; }
phase1_stop_owned_process_group() {
  attempts=$((attempts + 1))
  PHASE1_GROUP_TERM_SENT=false
  PHASE1_GROUP_KILL_SENT=false
  if (( attempts == 1 )); then
    PHASE1_GROUP_DRAINED=false
    PHASE1_GROUP_WAIT_STATUS=""
    return 3
  fi
  PHASE1_GROUP_DRAINED=true
  PHASE1_GROUP_WAIT_STATUS=0
  return 0
}

set +e
stop_owned_launch
first_status=$?
set -e
[[ "$first_status" -eq 3 ]]
[[ "$LAUNCH_PID" -eq 4242 ]]
[[ "$LAUNCH_PGID" -eq 4242 ]]
[[ "$LAUNCH_GROUP_DRAINED" == false ]]
[[ -e "/proc/$$/fd/$retained_fd_number" ]]

set +e
stop_owned_launch
second_status=$?
set -e
[[ "$second_status" -eq 3 ]]
[[ "$attempts" -eq 2 ]]
[[ "$LAUNCH_GROUP_DRAINED" == true ]]
[[ "$LAUNCH_CLEANUP_FIRST_FAILURE" -eq 3 ]]
[[ "$LAUNCH_CLEANUP_STATUS" -eq 3 ]]
[[ -z "$LAUNCH_PID" ]]
[[ -z "$LAUNCH_PGID" ]]
[[ -z "$LAUNCH_PIDFD" ]]
[[ ! -e "/proc/$$/fd/$retained_fd_number" ]]
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_stable_empty_group_requires_bounded_leader_reap_confirmation(
    tmp_path: Path,
) -> None:
    result = _run_harness(
        tmp_path,
        r"""
pidfd_helper="$2"
evidence_path="$3"
run_directory="$(dirname "$evidence_path")"

phase1_process_group_snapshot() { return 0; }
phase1_pidfd_action() {
  PHASE1_PIDFD_ACTION_STATUS=empty
  PHASE1_PIDFD_EVIDENCE_ERROR=false
  return 0
}
phase1_wait_for_owned_child_bounded() { return 1; }

set +e
phase1_stop_owned_process_group \
  launch 4242 4242 4242 1 "$evidence_path" "$pidfd_helper" 9 "$run_directory"
cleanup_status=$?
set -e

[[ "$cleanup_status" -eq 4 ]]
[[ "$PHASE1_GROUP_DRAINED" == false ]]
[[ -z "$PHASE1_GROUP_WAIT_STATUS" ]]
grep -Fq $'event\tcleanup-failed\t' "$evidence_path"
grep -Fq $'\tleader-reap-timeout' "$evidence_path"
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_interrupted_tee_wait_is_physically_drained_before_signal_exit(
    tmp_path: Path,
) -> None:
    result = _run_harness(
        tmp_path,
        r"""
run_directory="$(dirname "$3")"
verify_script="$PWD/scripts/verify_phase1.sh"
eval "$(sed -n '/^close_verify_log_control_fds() {$/,/^}$/p' "$verify_script")"
eval "$(sed -n '/^close_verify_log() {$/,/^}$/p' "$verify_script")"

LOG_FIFO="$run_directory/not-created.pipe"
LOG_READY_FIFO="$run_directory/not-created-ready.pipe"
VERIFY_LOG_WRITER_ACTIVE=false
VERIFY_LOG_BACKUPS_OPEN=false
VERIFY_LOG_ANCHOR_FD=""
VERIFY_LOG_READY_FD=""
TEE_STARTED=true
TEE_STATUS=""
TEE_DRAINED=""
PENDING_SIGNAL_STATUS=""
SIGNAL_DEFER_DEPTH=0
FINALIZED=0
harness_pid="$BASHPID"
trap 'phase1_record_signal 143' TERM

sleep 2 &
TEE_PID=$!
(
  sleep 0.1
  kill -TERM "$harness_pid"
) &
signaler_pid=$!

phase1_begin_signal_deferral
close_verify_log
[[ "$TEE_STATUS" -eq 125 ]]
[[ "$TEE_DRAINED" == true ]]
[[ -z "$TEE_PID" ]]
[[ "$PENDING_SIGNAL_STATUS" -eq 143 ]]
printf 'drained\n' > "$run_directory/tee-drained"
wait "$signaler_pid" 2>/dev/null || true
phase1_end_signal_deferral
printf 'unexpected\n' > "$run_directory/after-tee-drain"
""",
    )
    assert result.returncode == 143, result.stdout + result.stderr
    assert (tmp_path / 'tee-drained').read_text(encoding='utf-8') == 'drained\n'
    assert not (tmp_path / 'after-tee-drain').exists()


def test_verify_log_writer_starts_and_reaps_with_complete_output(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        r"""
run_directory="$(dirname "$3")"
verify_script="$PWD/scripts/verify_phase1.sh"
eval "$(sed -n '/^close_verify_log_control_fds() {$/,/^}$/p' "$verify_script")"
eval "$(sed -n '/^verify_log_writer_child() {$/,/^}$/p' "$verify_script")"
eval "$(sed -n '/^start_verify_log() {$/,/^}$/p' "$verify_script")"
eval "$(sed -n '/^close_verify_log() {$/,/^}$/p' "$verify_script")"

RUN_DIR="$run_directory"
LOG_FIFO="$run_directory/verify.pipe"
LOG_READY_FIFO="$run_directory/verify-ready.pipe"
TEE_PID=""
TEE_STATUS=""
TEE_STARTED=false
TEE_DRAINED=""
VERIFY_LOG_BACKUPS_OPEN=false
VERIFY_LOG_WRITER_ACTIVE=false
VERIFY_LOG_ANCHOR_FD=""
VERIFY_LOG_READY_FD=""
PENDING_SIGNAL_STATUS=""
SIGNAL_DEFER_DEPTH=0
FINALIZED=0

start_verify_log
printf 'captured-line\n'
close_verify_log

[[ "$TEE_STATUS" -eq 0 ]]
[[ "$TEE_DRAINED" == true ]]
[[ -z "$TEE_PID" ]]
[[ "$VERIFY_LOG_BACKUPS_OPEN" == false ]]
[[ "$VERIFY_LOG_WRITER_ACTIVE" == false ]]
grep -Fxq captured-line "$run_directory/verify.log"
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'captured-line\n' in result.stdout


def test_verify_log_startup_group_term_cannot_deadlock_fifo_open(tmp_path: Path) -> None:
    result = _run_harness(
        tmp_path,
        r"""
run_directory="$(dirname "$3")"
verify_script="$PWD/scripts/verify_phase1.sh"
eval "$(sed -n '/^close_verify_log_control_fds() {$/,/^}$/p' "$verify_script")"
eval "$(sed -n '/^start_verify_log() {$/,/^}$/p' "$verify_script")"

RUN_DIR="$run_directory"
LOG_FIFO="$run_directory/verify.pipe"
LOG_READY_FIFO="$run_directory/verify-ready.pipe"
TEE_PID=""
TEE_STATUS=""
TEE_STARTED=false
TEE_DRAINED=""
VERIFY_LOG_BACKUPS_OPEN=false
VERIFY_LOG_WRITER_ACTIVE=false
VERIFY_LOG_ANCHOR_FD=""
VERIFY_LOG_READY_FD=""
PENDING_SIGNAL_STATUS=""
SIGNAL_DEFER_DEPTH=0
FINALIZED=0
harness_pid="$BASHPID"
trap 'phase1_record_signal 143' TERM
trap 'printf "%s\n" \
  "tee_drained=$TEE_DRAINED" \
  "tee_pid=$TEE_PID" \
  "anchor_fd=$VERIFY_LOG_ANCHOR_FD" \
  "ready_fd=$VERIFY_LOG_READY_FD" \
  "writer_active=$VERIFY_LOG_WRITER_ACTIVE" \
  > "$run_directory/startup-state"' EXIT

verify_log_writer_child() {
  kill -TERM -- "-$harness_pid"
  sleep 5
}

start_verify_log
printf 'unexpected\n' > "$run_directory/after-startup"
""",
        timeout=10,
        start_new_session=True,
    )
    assert result.returncode == 143, result.stdout + result.stderr
    assert not (tmp_path / 'after-startup').exists()
    assert (tmp_path / 'startup-state').read_text(encoding='utf-8').splitlines() == [
        'tee_drained=true',
        'tee_pid=',
        'anchor_fd=',
        'ready_fd=',
        'writer_active=false',
    ]


@pytest.mark.parametrize(('signal_name', 'expected_status'), [('INT', 130), ('TERM', 143)])
def test_group_signal_during_pidfd_action_is_deferred_until_stable_drain(
    tmp_path: Path, signal_name: str, expected_status: int
) -> None:
    body = (
        REAL_GROUP_PREAMBLE
        + r"""
ROBOTEST_PHASE1_GROUP_TOKEN="$token" setsid sleep 30 &
leader_pid=$!
exec {retained_fd}<"/proc/$leader_pid"
validate_fixture

action_marker="$run_directory/pidfd-action-started"
drain_marker="$run_directory/stable-drain-complete"
harness_pid="$BASHPID"
PENDING_SIGNAL_STATUS=""
SIGNAL_DEFER_DEPTH=0
FINALIZED=0
trap 'phase1_record_signal __STATUS__' __SIGNAL__

python3() {
  if [[ "${1:-}" == "$pidfd_helper" && "${2:-}" == send ]]; then
    : > "$action_marker"
    sleep 0.3
  fi
  command python3 "$@"
}
(
  while [[ ! -e "$action_marker" ]]; do
    sleep 0.01
  done
  kill -__SIGNAL__ -- "-$harness_pid"
) &
signaler_pid=$!

phase1_begin_signal_deferral
phase1_stop_owned_process_group \
  launch "$leader_pid" "$leader_pid" "$leader_pid" "$leader_start_ticks" \
  "$evidence_path" "$pidfd_helper" "$retained_fd" "$run_directory"
[[ "$PHASE1_GROUP_TERM_SENT" == true ]]
[[ "$PHASE1_GROUP_KILL_SENT" == false ]]
[[ "$PHASE1_GROUP_DRAINED" == true ]]
[[ "$PENDING_SIGNAL_STATUS" -eq __STATUS__ ]]
grep -Fq $'event\tcleanup-pass\t' "$evidence_path"
[[ ! -e "$run_directory/pidfd-kill.json" ]]
printf 'drained\n' > "$drain_marker"
wait "$signaler_pid" 2>/dev/null || true
cleanup_fixture
trap - EXIT
phase1_end_signal_deferral
printf 'unexpected\n' > "$run_directory/after-deferred-signal"
"""
    )
    body = body.replace('__SIGNAL__', signal_name).replace('__STATUS__', str(expected_status))
    result = _run_harness(
        tmp_path,
        body,
        timeout=20,
        start_new_session=True,
    )
    assert result.returncode == expected_status, result.stdout + result.stderr
    assert (tmp_path / 'stable-drain-complete').read_text(encoding='utf-8') == 'drained\n'
    assert not (tmp_path / 'after-deferred-signal').exists()


def test_verify_uses_validated_handle_and_resumable_signal_state_machine() -> None:
    source = VERIFY_SCRIPT.read_text(encoding='utf-8')
    helper_source = SHELL_HELPER.read_text(encoding='utf-8')

    assert 'if [[ "$LAUNCH_PIDFD_VALIDATED" == true ]]; then' in source
    assert 'if [[ "$LAUNCH_GATE_RELEASED" == true ]]; then' not in source
    assert '&& [[ "$LAUNCH_GROUP_DRAINED" == true' in source
    assert 'LAUNCH_CLEANUP_FIRST_FAILURE="$cleanup_status"' in source
    assert '"$RUN_DIR" \\\n        "$LAUNCH_WAIT_STATUS"' in source
    assert "trap 'phase1_record_signal 130' INT" in source
    assert 'phase1_begin_signal_deferral\n  LAUNCH_CLEANUP_ATTEMPTED=1' in source
    assert 'LAUNCH_PID=$!\nphase1_end_signal_deferral' in source
    assert 'SAMPLER_PID=$!\nphase1_end_signal_deferral' in source
    timer_spawn = 'exec sleep "$timeout_seconds"\n  ) &\n  timer_pid=$!'
    assert timer_spawn in helper_source
    timer_index = helper_source.index(timer_spawn)
    assert helper_source.index('phase1_begin_signal_deferral', timer_index - 300) < timer_index
    assert helper_source.index('phase1_end_signal_deferral', helper_source.index(timer_spawn)) > (
        helper_source.index(timer_spawn)
    )
    release = (
        'phase1_release_launch_gate \\\n    "$LAUNCH_GO_FD" LAUNCH_GROUP_TOKEN LAUNCH_STARTED_UTC'
    )
    close_gate = 'close_launch_gate'
    assert source.index(release) < source.rindex(close_gate)
    assert "trap '' INT TERM\n      phase1_run_pidfd_command" in helper_source
    assert 'if [[ "$SAMPLER_DRAINED" == true ]]; then\n      SAMPLER_PID=""' in source
    trap_index = source.index('trap cleanup EXIT')
    log_start_call = source.index('\nstart_verify_log\n', trap_index)
    assert trap_index < log_start_call
    log_child_definition = source.index('verify_log_writer_child() {')
    log_child_end = source.index('\n}', log_child_definition)
    log_child = source[log_child_definition:log_child_end]
    child_trap = log_child.index("trap '' INT TERM")
    child_read = log_child.index('exec {tee_read_fd}<"$LOG_FIFO"')
    child_ready = log_child.index("printf 'READY\\n'")
    assert child_trap < child_read < child_ready
    log_start_definition = source.index('start_verify_log() {')
    log_start_end = source.index('\n}', log_start_definition)
    log_start = source[log_start_definition:log_start_end]
    anchor_open = log_start.index('exec {VERIFY_LOG_ANCHOR_FD}<>"$LOG_FIFO"')
    ready_open = log_start.index('exec {VERIFY_LOG_READY_FD}<>"$LOG_READY_FIFO"')
    tee_spawn = log_start.index('verify_log_writer_child &')
    tee_capture = log_start.index('TEE_PID=$!', tee_spawn)
    ready_wait = log_start.index('IFS= read -r -t 2', tee_capture)
    writer_open = log_start.index('exec > "$LOG_FIFO" 2>&1', ready_wait)
    assert log_start.index('phase1_begin_signal_deferral') < anchor_open
    assert anchor_open < ready_open < tee_spawn < tee_capture < ready_wait < writer_open
    assert writer_open < log_start.index('phase1_end_signal_deferral', writer_open)
    assert 'sample_time="$(date +%s.%N)" || return 1' in source
    assert 'sampling_complete_through_group_drain=$sampling_complete' in source
    assert '&& -n "$SAMPLING_STARTED_UTC"' in source
    assert '&& -n "$SAMPLING_STOPPED_UTC"' in source


def _identity(*, start_ticks: int) -> object:
    return PIDFD_MODULE.ProcessIdentity(
        pid=123,
        command='fixture',
        state='S',
        parent_pid=10,
        process_group_id=123,
        session_id=123,
        start_ticks=start_ticks,
    )


def test_closing_identity_change_is_rejected_before_capability_probe() -> None:
    with pytest.raises(PIDFD_MODULE.PidfdGroupError, match='changed'):
        PIDFD_MODULE.validate_identity_pair(
            _identity(start_ticks=100),
            _identity(start_ticks=101),
            expected_pid=123,
            expected_parent_pid=10,
        )


def test_pidfd_sender_uses_group_flag_and_only_esrch_means_empty() -> None:
    calls: list[tuple[int, int, object, int]] = []

    def sender(fd: int, signal_number: int, siginfo: object, flags: int) -> None:
        calls.append((fd, signal_number, siginfo, flags))

    assert PIDFD_MODULE.group_signal_status(9, signal.SIGTERM, sender=sender) == 'sent'
    assert calls == [(9, signal.SIGTERM, None, 4)]

    def missing_sender(fd: int, signal_number: int, siginfo: object, flags: int) -> None:
        raise ProcessLookupError(errno.ESRCH, 'gone')

    assert PIDFD_MODULE.group_signal_status(9, 0, sender=missing_sender) == 'empty'

    def unsupported_sender(fd: int, signal_number: int, siginfo: object, flags: int) -> None:
        raise OSError(errno.EINVAL, 'unsupported')

    with pytest.raises(PIDFD_MODULE.PidfdGroupError, match='errno=22'):
        PIDFD_MODULE.group_signal_status(9, signal.SIGKILL, sender=unsupported_sender)


def test_pidfd_cli_exit_code_carries_empty_action_outcome(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(PIDFD_MODULE, 'group_signal_status', lambda fd, signal_number: 'empty')

    assert PIDFD_MODULE.main(['probe', '--fd', '9']) == 20
    payload = capsys.readouterr().out
    assert '"status":"empty"' in payload

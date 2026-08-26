#!/usr/bin/env bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

set -Eeuo pipefail
IFS=$'\n\t'
umask 022

SCRIPT_PATH="$(readlink -f -- "$0")"
WORKSPACE="$(cd -- "$(dirname -- "$SCRIPT_PATH")/.." && pwd -P)"
PROCESS_GROUP_HELPER="$WORKSPACE/scripts/phase1_process_group.sh"
PIDFD_GROUP_HELPER="$WORKSPACE/scripts/phase1_pidfd_group.py"
if [[ ! -f "$PROCESS_GROUP_HELPER" \
  || -L "$PROCESS_GROUP_HELPER" \
  || ! -f "$PIDFD_GROUP_HELPER" \
  || -L "$PIDFD_GROUP_HELPER" ]]; then
  echo "missing regular Phase 1 process-group helper" >&2
  exit 2
fi
# shellcheck source=phase1_process_group.sh
source "$PROCESS_GROUP_HELPER"
ROBOTEST_CPUSET="${ROBOTEST_CPUSET:-0-5}"
if [[ ! "$ROBOTEST_CPUSET" =~ ^[0-9]+([,-][0-9]+)*$ ]]; then
  echo "invalid ROBOTEST_CPUSET: $ROBOTEST_CPUSET" >&2
  exit 2
fi

# The simulator seed is a canonical base-10 uint32: 0 through 4294967295,
# inclusive. Keeping the textual form canonical also keeps evidence stable.
ROBOTEST_SIM_SEED="${ROBOTEST_SIM_SEED:-42}"
if [[ ! "$ROBOTEST_SIM_SEED" =~ ^(0|[1-9][0-9]{0,9})$ ]] \
  || (( 10#$ROBOTEST_SIM_SEED > 4294967295 )); then
  echo "invalid ROBOTEST_SIM_SEED: expected a base-10 uint32 in [0, 4294967295]" >&2
  exit 2
fi

# Apply the local six-CPU budget even when the caller omitted taskset. The
# marker prevents recursion, and every later descendant inherits this mask.
if [[ "${ROBOTEST_PHASE1_AFFINITY_APPLIED:-0}" != "1" ]]; then
  ORIGINAL_CALLER_CWD="$(pwd -P)"
  ORIGINAL_ARGV_BASE64="$({ python3 - "$0" "$@" <<'PY'
import base64
import json
import sys

payload = json.dumps(sys.argv[1:], ensure_ascii=True, separators=(',', ':')).encode()
print(base64.b64encode(payload).decode('ascii'))
PY
  } 2>/dev/null)"
  exec taskset -c "$ROBOTEST_CPUSET" env \
    ROBOTEST_PHASE1_AFFINITY_APPLIED=1 \
    ROBOTEST_CPUSET="$ROBOTEST_CPUSET" \
    ROBOTEST_SIM_SEED="$ROBOTEST_SIM_SEED" \
    ROBOTEST_PHASE1_CALLER_CWD="$ORIGINAL_CALLER_CWD" \
    ROBOTEST_PHASE1_ORIGINAL_ARGV_BASE64="$ORIGINAL_ARGV_BASE64" \
    "$SCRIPT_PATH" "$@"
fi

CALLER_CWD="${ROBOTEST_PHASE1_CALLER_CWD:-$(pwd -P)}"
ORIGINAL_ARGV_BASE64="${ROBOTEST_PHASE1_ORIGINAL_ARGV_BASE64:-}"
RUN_CREATED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
GIT_SHA="$(git -C "$WORKSPACE" rev-parse HEAD 2>/dev/null || printf unavailable)"
if GIT_STATUS_START="$(git -C "$WORKSPACE" status --porcelain=v1 --untracked-files=all 2>&1)"; then
  if [[ -n "$GIT_STATUS_START" ]]; then
    GIT_DIRTY=true
  else
    GIT_DIRTY=false
  fi
else
  GIT_DIRTY=true
fi

EVIDENCE_ROOT="$WORKSPACE/artifacts/evidence/phase1"
mkdir -p -- "$EVIDENCE_ROOT"
EVIDENCE_ROOT="$(realpath -- "$EVIDENCE_ROOT")"
case "$EVIDENCE_ROOT" in
  "$WORKSPACE"/artifacts/evidence/phase1) ;;
  *) echo "unsafe evidence root: $EVIDENCE_ROOT" >&2; exit 2 ;;
esac

RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)-$$"
RUN_DIR="$EVIDENCE_ROOT/$RUN_ID"
mkdir -p -- "$RUN_DIR"

USER_HOME_DIR="$(getent passwd "$(id -u)" | cut -d: -f6)"
OGRE2_LOG_PATH="$USER_HOME_DIR/.gz/rendering/ogre2.log"
if [[ -f "$OGRE2_LOG_PATH" && ! -L "$OGRE2_LOG_PATH" ]]; then
  cp -- "$OGRE2_LOG_PATH" "$RUN_DIR/ogre2-before-phase1-launch.log"
else
  printf 'No regular, non-symlink pre-existing OGRE2 log at %s\n' "$OGRE2_LOG_PATH" \
    > "$RUN_DIR/ogre2-before-phase1-launch.absent.txt"
fi

# Retain this run plus the four newest prior runs. Deletion is strictly scoped
# to validated direct children of the Phase 1 evidence directory.
mapfile -t RUN_DIRS < <(find "$EVIDENCE_ROOT" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' | sort -nr | awk '{print $2}')
if (( ${#RUN_DIRS[@]} > 5 )); then
  for old_dir in "${RUN_DIRS[@]:5}"; do
    old_dir="$(realpath -- "$old_dir")"
    case "$old_dir" in
      "$EVIDENCE_ROOT"/*) rm -rf -- "$old_dir" ;;
      *) echo "refusing to prune unsafe path: $old_dir" >&2; exit 2 ;;
    esac
  done
fi

LOG_FIFO="$RUN_DIR/.verify-log.pipe"
LOG_READY_FIFO="$RUN_DIR/.verify-log-ready.pipe"

LAUNCH_PID=""
LAUNCH_PGID=""
LAUNCH_SID=""
LAUNCH_START_TICKS=""
LAUNCH_PIDFD=""
LAUNCH_PIDFD_VALIDATED=false
LAUNCH_GROUP_TOKEN=""
LAUNCH_GROUP_TOKEN_SHA256=""
LAUNCH_GO_FIFO=""
LAUNCH_GO_FD=""
LAUNCH_READY_FIFO=""
LAUNCH_READY_FD=""
SAMPLER_PID=""
SAMPLER_STATUS=""
SAMPLER_DRAINED=""
LAUNCH_STARTED_UTC=""
LAUNCH_STOPPED_UTC=""
LAUNCH_WAIT_STATUS=""
LAUNCH_TERM_SENT=false
LAUNCH_KILL_SENT=false
LAUNCH_GROUP_DRAINED=""
LAUNCH_CLEANUP_STATUS=""
LAUNCH_CLEANUP_FIRST_FAILURE=""
LAUNCH_CLEANUP_ATTEMPTED=0
SAMPLING_STARTED_UTC=""
SAMPLING_STOPPED_UTC=""
TEE_STATUS=""
TEE_PID=""
TEE_STARTED=false
TEE_DRAINED=""
VERIFY_LOG_BACKUPS_OPEN=false
VERIFY_LOG_WRITER_ACTIVE=false
VERIFY_LOG_ANCHOR_FD=""
VERIFY_LOG_READY_FD=""
FINALIZED=0
PENDING_SIGNAL_STATUS=""
SIGNAL_DEFER_DEPTH=0

sample_launch_group() {
  local sample_time=""
  sample_time="$(date +%s.%N)" || return 1
  ps -eo pid=,pgid=,pcpu=,rss=,psr=,comm= \
    | awk -v now="$sample_time" -v group="$LAUNCH_PGID" \
      '$2 == group {print now "," $1 "," $2 "," $3 "," $4 "," $5 "," $6}' \
    >> "$RUN_DIR/resources.csv"
}

stop_sampler() {
  local sampler_cleanup_status=0

  phase1_begin_signal_deferral
  if [[ -n "$SAMPLER_PID" ]]; then
    if phase1_stop_exact_child_bounded "$SAMPLER_PID"; then
      sampler_cleanup_status=0
    else
      sampler_cleanup_status=$?
    fi
    SAMPLER_STATUS="$PHASE1_CHILD_OUTCOME_STATUS"
    SAMPLER_DRAINED="$PHASE1_CHILD_DRAINED"
    if [[ "$SAMPLER_DRAINED" == true ]]; then
      SAMPLER_PID=""
    fi
  fi
  if [[ "$SAMPLER_DRAINED" == true \
    && -n "$SAMPLING_STARTED_UTC" \
    && -z "$SAMPLING_STOPPED_UTC" ]]; then
    if ! SAMPLING_STOPPED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"; then
      SAMPLER_STATUS=125
    fi
  fi
  phase1_end_signal_deferral
  return "$sampler_cleanup_status"
}

stop_owned_launch() {
  local cleanup_status=0
  local launch_pid="$LAUNCH_PID"
  local launch_pgid="$LAUNCH_PGID"
  local launch_sid="$LAUNCH_SID"
  local launch_start_ticks="$LAUNCH_START_TICKS"
  local launch_pidfd="$LAUNCH_PIDFD"

  if (( LAUNCH_CLEANUP_ATTEMPTED == 1 )) \
    && [[ "$LAUNCH_GROUP_DRAINED" == true \
      && -n "$LAUNCH_CLEANUP_STATUS" ]]; then
    return "${LAUNCH_CLEANUP_STATUS:-1}"
  fi
  [[ -n "$launch_pid" ]] || return 0
  phase1_begin_signal_deferral
  LAUNCH_CLEANUP_ATTEMPTED=1

  if [[ -n "$SAMPLER_PID" ]]; then
    sample_launch_group || true
  fi
  if [[ "$LAUNCH_PIDFD_VALIDATED" == true ]]; then
    if phase1_stop_owned_process_group \
        launch \
        "$launch_pid" \
        "$launch_pgid" \
        "$launch_sid" \
        "$launch_start_ticks" \
        "$RUN_DIR/process-group-cleanup.tsv" \
        "$PIDFD_GROUP_HELPER" \
        "$launch_pidfd" \
        "$RUN_DIR" \
        "$LAUNCH_WAIT_STATUS"; then
      cleanup_status=0
    else
      cleanup_status=$?
    fi
  elif phase1_abort_unreleased_launch_gate \
      launch "$launch_pid" "$LAUNCH_GO_FD" \
      "$RUN_DIR/process-group-cleanup.tsv" 12; then
    cleanup_status=0
  else
    cleanup_status=$?
  fi
  if [[ "$PHASE1_GROUP_TERM_SENT" == true ]]; then
    LAUNCH_TERM_SENT=true
  fi
  if [[ "$PHASE1_GROUP_KILL_SENT" == true ]]; then
    LAUNCH_KILL_SENT=true
  fi
  LAUNCH_GROUP_DRAINED="$PHASE1_GROUP_DRAINED"
  LAUNCH_WAIT_STATUS="$PHASE1_GROUP_WAIT_STATUS"
  if [[ "$LAUNCH_GROUP_DRAINED" != true && "$cleanup_status" -eq 0 ]]; then
    cleanup_status=4
  fi
  if (( cleanup_status != 0 )) && [[ -z "$LAUNCH_CLEANUP_FIRST_FAILURE" ]]; then
    LAUNCH_CLEANUP_FIRST_FAILURE="$cleanup_status"
  fi
  if [[ -n "$LAUNCH_CLEANUP_FIRST_FAILURE" ]]; then
    LAUNCH_CLEANUP_STATUS="$LAUNCH_CLEANUP_FIRST_FAILURE"
  else
    LAUNCH_CLEANUP_STATUS="$cleanup_status"
  fi
  if [[ "$LAUNCH_GROUP_DRAINED" == true \
    && -n "$LAUNCH_STARTED_UTC" \
    && -z "$LAUNCH_STOPPED_UTC" ]]; then
    LAUNCH_STOPPED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  fi
  if [[ "$LAUNCH_GROUP_DRAINED" == true ]]; then
    LAUNCH_PID=""
    LAUNCH_PGID=""
    if [[ "$LAUNCH_PIDFD" =~ ^[0-9]+$ ]]; then
      exec {LAUNCH_PIDFD}<&-
      LAUNCH_PIDFD=""
    fi
  fi
  phase1_end_signal_deferral
  return "$LAUNCH_CLEANUP_STATUS"
}

close_launch_gate() {
  if [[ "$LAUNCH_GO_FD" =~ ^[0-9]+$ ]]; then
    exec {LAUNCH_GO_FD}>&-
    LAUNCH_GO_FD=""
  fi
  if [[ -n "$LAUNCH_GO_FIFO" && -p "$LAUNCH_GO_FIFO" ]]; then
    rm -f -- "$LAUNCH_GO_FIFO"
  fi
  if [[ "$LAUNCH_READY_FD" =~ ^[0-9]+$ ]]; then
    exec {LAUNCH_READY_FD}>&-
    LAUNCH_READY_FD=""
  fi
  if [[ -n "$LAUNCH_READY_FIFO" && -p "$LAUNCH_READY_FIFO" ]]; then
    rm -f -- "$LAUNCH_READY_FIFO"
  fi
  LAUNCH_GO_FIFO=""
  LAUNCH_READY_FIFO=""
  LAUNCH_GROUP_TOKEN=""
}

capture_phase1_ogre2_log() {
  local capture_path="$RUN_DIR/ogre2-phase1-launch.log"
  local absent_path="$RUN_DIR/ogre2-phase1-launch.absent.txt"
  if [[ -e "$capture_path" || -e "$absent_path" ]]; then
    return
  fi
  if [[ -z "$LAUNCH_STARTED_UTC" ]]; then
    printf 'Phase 1 launch did not start; no post-launch OGRE2 log was captured.\n' \
      > "$absent_path"
    return
  fi
  if [[ -f "$OGRE2_LOG_PATH" && ! -L "$OGRE2_LOG_PATH" ]]; then
    cp -- "$OGRE2_LOG_PATH" "$capture_path"
  else
    printf 'No regular, non-symlink Phase 1 OGRE2 log at %s\n' "$OGRE2_LOG_PATH" \
      > "$absent_path"
  fi
}

close_verify_log_control_fds() {
  local close_status=0

  if [[ "$VERIFY_LOG_ANCHOR_FD" =~ ^[0-9]+$ ]]; then
    exec {VERIFY_LOG_ANCHOR_FD}>&- || close_status=1
    VERIFY_LOG_ANCHOR_FD=""
  fi
  if [[ "$VERIFY_LOG_READY_FD" =~ ^[0-9]+$ ]]; then
    exec {VERIFY_LOG_READY_FD}>&- || close_status=1
    VERIFY_LOG_READY_FD=""
  fi
  if [[ -p "$LOG_FIFO" ]]; then
    rm -f -- "$LOG_FIFO" || close_status=1
  elif [[ -e "$LOG_FIFO" || -L "$LOG_FIFO" ]]; then
    close_status=1
  fi
  if [[ -p "$LOG_READY_FIFO" ]]; then
    rm -f -- "$LOG_READY_FIFO" || close_status=1
  elif [[ -e "$LOG_READY_FIFO" || -L "$LOG_READY_FIFO" ]]; then
    close_status=1
  fi
  return "$close_status"
}

verify_log_writer_child() {
  local tee_read_fd=""

  # The parent holds both FIFOs open read/write before this child is forked.
  # Thus a signal before this trap cannot strand the parent in a FIFO open;
  # readiness is reported only after the trap and dedicated read end exist.
  trap '' INT TERM
  if [[ "$VERIFY_LOG_ANCHOR_FD" =~ ^[0-9]+$ ]]; then
    exec {VERIFY_LOG_ANCHOR_FD}>&-
  fi
  exec {tee_read_fd}<"$LOG_FIFO"
  printf 'READY\n' >&"$VERIFY_LOG_READY_FD"
  exec {VERIFY_LOG_READY_FD}>&-
  exec tee -a "$RUN_DIR/verify.log" <&"$tee_read_fd" >&3
}

start_verify_log() {
  local ready=""
  local start_status=0

  [[ "$TEE_STARTED" == false \
    && -z "$TEE_PID" \
    && "$VERIFY_LOG_BACKUPS_OPEN" == false \
    && "$VERIFY_LOG_WRITER_ACTIVE" == false \
    && -z "$VERIFY_LOG_ANCHOR_FD" \
    && -z "$VERIFY_LOG_READY_FD" ]] || return 2

  phase1_begin_signal_deferral
  mkfifo -- "$LOG_FIFO" "$LOG_READY_FIFO"
  exec {VERIFY_LOG_ANCHOR_FD}<>"$LOG_FIFO"
  exec {VERIFY_LOG_READY_FD}<>"$LOG_READY_FIFO"
  exec 3>&1 4>&2
  VERIFY_LOG_BACKUPS_OPEN=true
  verify_log_writer_child &
  TEE_PID=$!
  TEE_STARTED=true

  if ! IFS= read -r -t 2 -u "$VERIFY_LOG_READY_FD" ready \
    || [[ "$ready" != READY ]]; then
    start_status=1
  elif exec > "$LOG_FIFO" 2>&1; then
    VERIFY_LOG_WRITER_ACTIVE=true
  else
    start_status=1
  fi
  close_verify_log_control_fds || start_status=1

  if (( start_status != 0 )); then
    if [[ "$VERIFY_LOG_WRITER_ACTIVE" == true ]]; then
      exec 1>&3 2>&4
      VERIFY_LOG_WRITER_ACTIVE=false
    fi
    if [[ "$TEE_PID" =~ ^[1-9][0-9]*$ ]]; then
      phase1_stop_exact_child_bounded "$TEE_PID" || start_status=1
      TEE_STATUS="$PHASE1_CHILD_OUTCOME_STATUS"
      TEE_DRAINED="$PHASE1_CHILD_DRAINED"
      if [[ "$TEE_DRAINED" == true ]]; then
        TEE_PID=""
      fi
    else
      TEE_STATUS=125
      TEE_DRAINED=false
    fi
  fi
  phase1_end_signal_deferral
  return "$start_status"
}

close_verify_log() {
  local tee_cleanup_status=0

  phase1_begin_signal_deferral
  if [[ "$VERIFY_LOG_WRITER_ACTIVE" == true ]]; then
    exec 1>&3 2>&4
    VERIFY_LOG_WRITER_ACTIVE=false
  fi
  close_verify_log_control_fds || tee_cleanup_status=1
  if [[ "$TEE_STARTED" == true && "$TEE_PID" =~ ^[1-9][0-9]*$ ]]; then
    if phase1_stop_exact_child_bounded "$TEE_PID"; then
      :
    else
      tee_cleanup_status=1
    fi
    TEE_STATUS="$PHASE1_CHILD_OUTCOME_STATUS"
    TEE_DRAINED="$PHASE1_CHILD_DRAINED"
    if [[ "$TEE_DRAINED" == true ]]; then
      TEE_PID=""
    fi
  elif [[ "$TEE_STARTED" == true && "$TEE_DRAINED" != true ]]; then
    TEE_STATUS=125
    TEE_DRAINED=false
    tee_cleanup_status=1
  fi
  if [[ "$VERIFY_LOG_BACKUPS_OPEN" == true ]]; then
    exec 3>&- 4>&-
    VERIFY_LOG_BACKUPS_OPEN=false
  fi
  phase1_end_signal_deferral
  return "$tee_cleanup_status"
}

write_source_config_hashes() {
  python3 - "$WORKSPACE" "$RUN_DIR/source-config-hashes.json" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

workspace = Path(sys.argv[1]).resolve()
output = Path(sys.argv[2])
explicit_files = [
    workspace / 'scripts/verify_phase1.sh',
    workspace / 'scripts/phase1_process_group.sh',
    workspace / 'scripts/phase1_pidfd_group.py',
    workspace / 'docs/testing/acceptance-criteria.md',
    workspace / 'docs/testing/verification-matrix.md',
    workspace / 'docs/architecture/metrics-contract.md',
    workspace / 'docs/architecture/topic-and-tf-contract.md',
]
source_roots = [
    workspace / 'src/robotest_interfaces',
    workspace / 'src/robotest_description',
    workspace / 'src/robotest_faults',
    workspace / 'src/robotest_sim',
]
paths = set(explicit_files)
for root in source_roots:
    paths.update(
        path
        for path in root.rglob('*')
        if path.is_file()
        and not path.is_symlink()
        and '__pycache__' not in path.parts
        and path.suffix != '.pyc'
    )

entries = []
aggregate = hashlib.sha256()
for path in sorted(paths):
    relative = path.relative_to(workspace).as_posix()
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    entries.append({'path': relative, 'sha256': digest})
    aggregate.update(relative.encode('utf-8') + b'\0' + digest.encode('ascii') + b'\n')

output.write_text(
    json.dumps(
        {
            'algorithm': 'sha256',
            'aggregate_sha256': aggregate.hexdigest(),
            'files': entries,
        },
        indent=2,
        sort_keys=True,
    ) + '\n',
    encoding='utf-8',
)
PY
}

write_command_evidence() {
  python3 - \
    "$RUN_DIR/command.json" \
    "$CALLER_CWD" \
    "$WORKSPACE" \
    "$ORIGINAL_ARGV_BASE64" \
    "$$" <<'PY'
import base64
import json
import sys
from pathlib import Path

output = Path(sys.argv[1])
caller_cwd = sys.argv[2]
workspace = sys.argv[3]
encoded_original = sys.argv[4]
effective_argv = [
    item.decode('utf-8', errors='surrogateescape')
    for item in Path(f'/proc/{sys.argv[5]}/cmdline').read_bytes().split(b'\0')
    if item
]
if encoded_original:
    original_argv = json.loads(base64.b64decode(encoded_original).decode('utf-8'))
else:
    original_argv = effective_argv

output.write_text(
    json.dumps(
        {
            'argv_scope': (
                'script argv captured before the internal affinity re-exec; '
                'parent-shell source text is not observable'
            ),
            'caller_cwd': caller_cwd,
            'effective_process_argv': effective_argv,
            'original_script_argv': original_argv,
            'workspace_cwd': workspace,
        },
        indent=2,
        sort_keys=True,
    ) + '\n',
    encoding='utf-8',
)
PY
}

write_version_evidence() {
  python3 - "$RUN_DIR/versions.json" <<'PY'
import importlib.metadata
import json
import os
import platform
import subprocess
import sys
from pathlib import Path


def command_output(argv):
    try:
        completed = subprocess.run(
            argv,
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'argv': argv, 'exit_code': None, 'output': str(error)}
    output = (completed.stdout + completed.stderr).strip()
    return {'argv': argv, 'exit_code': completed.returncode, 'output': output[:8192]}


def package_version(name):
    result = command_output(['dpkg-query', '-W', '-f=${Version}', name])
    return result['output'] if result['exit_code'] == 0 else None


try:
    colcon_version = importlib.metadata.version('colcon-core')
except importlib.metadata.PackageNotFoundError:
    colcon_version = None

packages = [
    'ros-jazzy-ros-base',
    'ros-jazzy-ros-gz',
    'ros-jazzy-navigation2',
    'ros-jazzy-nav2-bringup',
    'ros-jazzy-rmw-fastrtps-cpp',
]
rmw = command_output(
    [
        'python3',
        '-c',
        'from rclpy.utilities import get_rmw_implementation_identifier; '
        'print(get_rmw_implementation_identifier())',
    ]
)
evidence = {
    'build_type': 'CMake default (CMAKE_BUILD_TYPE not forced by this verifier)',
    'colcon_core': colcon_version,
    'commands': {
        'cmake': command_output(['cmake', '--version']),
        'gazebo': command_output(['gz', 'sim', '--versions']),
        'git': command_output(['git', '--version']),
        'go': command_output(['go', 'version']),
        'gxx': command_output(['g++', '--version']),
        'python': command_output(['python3', '--version']),
        'shellcheck': command_output(['shellcheck', '--version']),
    },
    'dpkg_packages': {name: package_version(name) for name in packages},
    'platform': {
        'kernel': platform.release(),
        'machine': platform.machine(),
        'os_release': platform.freedesktop_os_release(),
        'python_runtime': platform.python_version(),
        'wsl_distro_name': os.environ.get('WSL_DISTRO_NAME'),
    },
    'rmw_implementation': rmw['output'] if rmw['exit_code'] == 0 else rmw,
    'ros_distro': os.environ.get('ROS_DISTRO'),
}
Path(sys.argv[1]).write_text(
    json.dumps(evidence, indent=2, sort_keys=True) + '\n', encoding='utf-8'
)
PY
}

write_lifecycle_summary() {
  local exit_code="$1"
  local checksum_status="$2"
  python3 - \
    "$RUN_DIR/summary.json" \
    "$RUN_ID" \
    "$RUN_CREATED_UTC" \
    "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
    "$exit_code" \
    "$checksum_status" \
    "${ROS_DOMAIN_ID:-}" \
    "${GZ_PARTITION:-}" \
    "$LAUNCH_STARTED_UTC" \
    "$LAUNCH_STOPPED_UTC" \
    "$LAUNCH_WAIT_STATUS" \
    "$LAUNCH_TERM_SENT" \
    "$LAUNCH_KILL_SENT" \
    "$SAMPLING_STARTED_UTC" \
    "$SAMPLING_STOPPED_UTC" \
    "$TEE_STATUS" \
    "$LAUNCH_CLEANUP_STATUS" \
    "$LAUNCH_GROUP_DRAINED" \
    "$LAUNCH_SID" \
    "$LAUNCH_START_TICKS" \
    "$LAUNCH_PIDFD_VALIDATED" \
    "$LAUNCH_GROUP_TOKEN_SHA256" \
    "$SAMPLER_STATUS" \
    "$SAMPLER_DRAINED" \
    "$TEE_DRAINED" <<'PY'
import json
import sys
from pathlib import Path


def optional_int(value):
    return int(value) if value else None


summary_path = Path(sys.argv[1])
cleanup_evidence_path = summary_path.parent / 'process-group-cleanup.tsv'
cleanup_evidence = (
    cleanup_evidence_path.name
    if cleanup_evidence_path.is_file() and not cleanup_evidence_path.is_symlink()
    else None
)
summary = {
    'checksum_validation_exit_code': int(sys.argv[6]),
    'created_utc': sys.argv[3],
    'evidence_finalized_utc': sys.argv[4],
    'exit_code': int(sys.argv[5]),
    'gz_partition': sys.argv[8] or None,
    'launch': {
        'cleanup_evidence': cleanup_evidence,
        'cleanup_exit_code': optional_int(sys.argv[17]),
        'group_drained': (
            sys.argv[18] == 'true' if sys.argv[18] else None
        ),
        'kill_sent': sys.argv[13] == 'true',
        'leader_start_ticks': optional_int(sys.argv[20]),
        'pidfd_group_signal': {
            'flag': 4,
            'token_sha256': sys.argv[22] or None,
            'validated': sys.argv[21] == 'true',
            'validation_evidence': (
                'pidfd-validation.json' if sys.argv[21] == 'true' else None
            ),
        },
        'session_id': optional_int(sys.argv[19]),
        'started_utc': sys.argv[9] or None,
        'stopped_utc': sys.argv[10] or None,
        'term_sent': sys.argv[12] == 'true',
        'wait_status': optional_int(sys.argv[11]),
    },
    'resource_sampling': {
        'complete_through_group_drain': (
            sys.argv[23] == '0'
            and sys.argv[24] == 'true'
            and sys.argv[18] == 'true'
            and bool(sys.argv[14])
            and bool(sys.argv[15])
        ),
        'drained': (
            sys.argv[24] == 'true' if sys.argv[24] else None
        ),
        'exit_code': optional_int(sys.argv[23]),
        'scope': (
            'isolated launch PGID; exit_code=0 and complete_through_group_drain=true '
            'prove sampling continued from the first post-validation sample through '
            'owned process-group drain; the small process-creation to first-sample '
            'interval is not observed'
        ),
        'started_utc': sys.argv[14] or None,
        'stopped_utc': sys.argv[15] or None,
    },
    'ros_domain_id': optional_int(sys.argv[7]),
    'run_id': sys.argv[2],
    'tee_drained': (
        sys.argv[25] == 'true' if sys.argv[25] else None
    ),
    'tee_exit_code': optional_int(sys.argv[16]),
}
summary_path.write_text(
    json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8'
)
PY
}

write_provenance() {
  local exit_code="$1"
  local checksum_status="$2"
  python3 - \
    "$RUN_DIR/provenance.json" \
    "$RUN_DIR/command.json" \
    "$RUN_DIR/source-config-hashes.json" \
    "$RUN_DIR/versions.json" \
    "$RUN_DIR/git-status-start.txt" \
    "$RUN_ID" \
    "$RUN_CREATED_UTC" \
    "${GIT_SHA:-unavailable}" \
    "${GIT_DIRTY:-true}" \
    "${TARGET_SET_SHA256:-unavailable}" \
    "${SCENARIO_SHA256:-unavailable}" \
    "$ROBOTEST_CPUSET" \
    "$ROBOTEST_SIM_SEED" \
    "${ROS_DOMAIN_ID:-}" \
    "${GZ_PARTITION:-}" \
    "$exit_code" \
    "$checksum_status" <<'PY'
import json
import sys
from pathlib import Path


def read_json(path):
    candidate = Path(path)
    return json.loads(candidate.read_text(encoding='utf-8')) if candidate.is_file() else None


provenance = {
    'artifacts': {
        'checksum_manifest': 'SHA256SUMS',
        'checksum_validation': 'checksum-validation.txt',
        'canonical_json': 'run-result.json',
        'matching_csv': 'run-result.csv',
    },
    'command': read_json(sys.argv[2]),
    'effective_parameters': {
        'build_parallel_workers': 4,
        'headless': True,
        'render_sensors': True,
        'robot_namespace': '/robotest',
        'runtime_cpuset': sys.argv[12],
        'simulator_seed': int(sys.argv[13]),
        'rviz': False,
    },
    'git': {
        'dirty_at_start': sys.argv[9] == 'true',
        'sha': sys.argv[8],
        'status_porcelain_path': 'git-status-start.txt',
    },
    'identity': {
        'created_utc': sys.argv[7],
        'gz_partition': sys.argv[15] or None,
        'ros_domain_id': int(sys.argv[14]) if sys.argv[14] else None,
        'run_id': sys.argv[6],
        'verification_level': 'L3 bounded local headless simulation',
    },
    'outcome': {
        'checksum_validation_exit_code': int(sys.argv[17]),
        'exit_code': int(sys.argv[16]),
        'manual_rviz_status': 'PENDING',
    },
    'scenario': {
        'expected_outcome': 'no-fault pass-through with bounded forward motion',
        'fault_schedule_hash': None,
        'fault_seed': None,
        'mission_seed': None,
        'name': 'phase1_bounded_pass_through',
        'scenario_file': 'src/robotest_sim/worlds/robotest_lab.sdf',
        'scenario_sha256': sys.argv[11],
        'seed_status': 'fixed_simulator_seed_recorded',
        'simulator_seed': int(sys.argv[13]),
    },
    'source_config': read_json(sys.argv[3]),
    'target_set': {
        'path': 'docs/testing/acceptance-criteria.md',
        'revision': 'Frozen Phase 0 target set, revision 1',
        'sha256': sys.argv[10],
    },
    'versions': read_json(sys.argv[4]),
}
Path(sys.argv[1]).write_text(
    json.dumps(provenance, indent=2, sort_keys=True) + '\n', encoding='utf-8'
)
PY
}

write_canonical_result() {
  local exit_code="$1"
  local checksum_status="$2"
  python3 - \
    "$RUN_DIR/run-result.json" \
    "$RUN_DIR/run-result.csv" \
    "$RUN_DIR/runtime-probe.json" \
    "$RUN_ID" \
    "$RUN_CREATED_UTC" \
    "${GIT_SHA:-unavailable}" \
    "${GIT_DIRTY:-true}" \
    "${TARGET_SET_SHA256:-unavailable}" \
    "${SCENARIO_SHA256:-unavailable}" \
    "$ROBOTEST_SIM_SEED" \
    "${max_rss_kib:-}" \
    "$exit_code" \
    "$checksum_status" <<'PY'
import csv
import json
import sys
from pathlib import Path

json_path = Path(sys.argv[1])
csv_path = Path(sys.argv[2])
probe_path = Path(sys.argv[3])
probe = json.loads(probe_path.read_text(encoding='utf-8')) if probe_path.is_file() else {}
rtf = probe.get('rtf', {})
exit_code = int(sys.argv[12])
checksum_status = int(sys.argv[13])
measurements = {
    'displacement_m': probe.get('displacement_m'),
    'peak_process_group_rss_kib': int(sys.argv[11]) if sys.argv[11] else None,
    'rtf_median': rtf.get('median'),
    'rtf_p5': rtf.get('p5'),
}
result = {
    'events': [],
    'identity': {
        'created_utc': sys.argv[5],
        'git_dirty': sys.argv[7] == 'true',
        'git_sha': sys.argv[6],
        'run_id': sys.argv[4],
        'verification_level': 'L3 bounded local headless simulation',
    },
    'measurements': measurements,
    'quality': {
        'checksum_validation_exit_code': checksum_status,
        'manual_rviz_status': 'PENDING',
        'runtime_probe_verdict': probe.get('verdict', 'NOT_RUN'),
        'seed_status': 'fixed_simulator_seed_recorded',
        'simulator_seed': int(sys.argv[10]),
    },
    'targets': {
        'max_process_group_rss_kib': 6 * 1024 * 1024,
        'minimum_rtf_median': 0.80,
        'minimum_rtf_p5': 0.50,
        'scenario_sha256': sys.argv[9],
        'target_set_sha256': sys.argv[8],
    },
    'verdict': {
        'automated_status': 'PASS' if exit_code == 0 and checksum_status == 0 else 'FAIL',
        'exit_code': exit_code,
    },
}
json_path.write_text(
    json.dumps(result, separators=(',', ':'), sort_keys=True) + '\n', encoding='utf-8'
)

row = {
    'run_id': result['identity']['run_id'],
    'created_utc': result['identity']['created_utc'],
    'git_sha': result['identity']['git_sha'],
    'git_dirty': str(result['identity']['git_dirty']).lower(),
    'verification_level': result['identity']['verification_level'],
    'target_set_sha256': result['targets']['target_set_sha256'],
    'scenario_sha256': result['targets']['scenario_sha256'],
    'simulator_seed': result['quality']['simulator_seed'],
    'seed_status': result['quality']['seed_status'],
    'runtime_probe_verdict': result['quality']['runtime_probe_verdict'],
    'peak_process_group_rss_kib': measurements['peak_process_group_rss_kib'],
    'target_max_process_group_rss_kib': result['targets']['max_process_group_rss_kib'],
    'rtf_median': measurements['rtf_median'],
    'target_minimum_rtf_median': result['targets']['minimum_rtf_median'],
    'rtf_p5': measurements['rtf_p5'],
    'target_minimum_rtf_p5': result['targets']['minimum_rtf_p5'],
    'displacement_m': measurements['displacement_m'],
    'manual_rviz_status': result['quality']['manual_rviz_status'],
    'checksum_validation_exit_code': checksum_status,
    'automated_status': result['verdict']['automated_status'],
    'exit_code': exit_code,
}
with csv_path.open('w', encoding='utf-8', newline='') as stream:
    writer = csv.DictWriter(stream, fieldnames=list(row))
    writer.writeheader()
    writer.writerow(row)
PY
}

write_final_metadata() {
  local exit_code="$1"
  local checksum_status="$2"
  write_lifecycle_summary "$exit_code" "$checksum_status" &&
    write_provenance "$exit_code" "$checksum_status" &&
    write_canonical_result "$exit_code" "$checksum_status"
}

generate_checksum_manifest() {
  find "$RUN_DIR" -maxdepth 1 -type f \
    ! -name SHA256SUMS \
    ! -name checksum-validation.txt \
    -print0 \
    | sort -z \
    | xargs -0 -r sha256sum > "$RUN_DIR/SHA256SUMS"
}

validate_checksum_manifest() {
  local validation_tmp=""
  local validation_status=0
  validation_tmp="$(mktemp)" || return 1
  if sha256sum -c "$RUN_DIR/SHA256SUMS" > "$validation_tmp" 2>&1; then
    validation_status=0
  else
    validation_status=$?
  fi
  if ! mv -- "$validation_tmp" "$RUN_DIR/checksum-validation.txt"; then
    rm -f -- "$validation_tmp"
    return 1
  fi
  return "$validation_status"
}

cleanup() {
  local execution_status=$?
  local launch_cleanup_status=0
  local sampler_cleanup_status=0
  local tee_cleanup_status=0
  local checksum_status=0
  local final_status=0
  local metadata_status=0

  if (( FINALIZED == 1 )); then
    return
  fi
  FINALIZED=1
  trap - EXIT
  # The sourced process-group helper reads this shared deferral depth.
  # shellcheck disable=SC2034
  SIGNAL_DEFER_DEPTH=0
  phase1_begin_signal_deferral
  set +e

  # The sampler stays alive while the owned launch group is terminated so the
  # recorded interval covers controlled shutdown through exact group drain.
  stop_owned_launch
  launch_cleanup_status=$?
  if [[ -n "$LAUNCH_PID" && "$LAUNCH_GROUP_DRAINED" != true ]]; then
    stop_owned_launch
    launch_cleanup_status=$?
  fi
  close_launch_gate
  stop_sampler
  sampler_cleanup_status=$?
  capture_phase1_ogre2_log
  echo "Phase 1 automated execution ended with status $execution_status; finalizing evidence: $RUN_DIR"
  close_verify_log
  tee_cleanup_status=$?
  phase1_end_signal_deferral
  trap 'exit 130' INT
  trap 'exit 143' TERM
  if (( execution_status == 0 )) && [[ -n "$PENDING_SIGNAL_STATUS" ]]; then
    execution_status="$PENDING_SIGNAL_STATUS"
  fi

  final_status=$execution_status
  if (( launch_cleanup_status != 0 )); then
    final_status=1
  fi
  if [[ -n "$SAMPLER_STATUS" && "$SAMPLER_STATUS" != "0" ]]; then
    final_status=1
  fi
  if (( sampler_cleanup_status != 0 )) \
    || [[ -n "$SAMPLING_STARTED_UTC" && "$SAMPLER_DRAINED" != true ]]; then
    final_status=1
  fi
  if [[ "$TEE_STATUS" != "0" ]]; then
    final_status=1
  fi
  if (( tee_cleanup_status != 0 )) \
    || [[ "$TEE_STARTED" == true && "$TEE_DRAINED" != true ]]; then
    final_status=1
  fi
  if [[ -n "$LAUNCH_PID" \
    || -n "$SAMPLER_PID" \
    || ( "$TEE_STARTED" == true && "$TEE_DRAINED" != true ) ]]; then
    printf 'Phase 1 evidence finalization aborted: owned process cleanup was not confirmed.\n' \
      >&2
    exit 1
  fi
  write_final_metadata "$final_status" 0
  metadata_status=$?
  if (( metadata_status != 0 )); then
    final_status=1
  fi
  if ! generate_checksum_manifest; then
    final_status=1
  fi
  if validate_checksum_manifest; then
    checksum_status=0
  else
    checksum_status=$?
    final_status=1
  fi

  # If integrity or metadata finalization failed, bind that failure into the
  # lifecycle and canonical result, regenerate, and validate the final bytes.
  if (( final_status != execution_status || checksum_status != 0 || metadata_status != 0 )); then
    write_final_metadata "$final_status" "$checksum_status" || final_status=1
    generate_checksum_manifest || final_status=1
    validate_checksum_manifest || final_status=1
  fi

  if (( final_status == 0 )); then
    printf 'Phase 1 verification PASS: %s\n' "$RUN_DIR"
  else
    printf 'Phase 1 verification FAIL (status %d): %s\n' "$final_status" "$RUN_DIR" >&2
  fi
  exit "$final_status"
}

trap 'phase1_record_signal 130' INT
trap 'phase1_record_signal 143' TERM
trap cleanup EXIT

start_verify_log
write_command_evidence
write_source_config_hashes
if [[ -n "$GIT_STATUS_START" ]]; then
  printf '%s\n' "$GIT_STATUS_START" > "$RUN_DIR/git-status-start.txt"
else
  : > "$RUN_DIR/git-status-start.txt"
fi
TARGET_SET_SHA256="$(sha256sum "$WORKSPACE/docs/testing/acceptance-criteria.md" | awk '{print $1}')"
SCENARIO_SHA256="$(sha256sum "$WORKSPACE/src/robotest_sim/worlds/robotest_lab.sdf" | awk '{print $1}')"

echo "Phase 1 run: $RUN_ID"
echo "Workspace: $WORKSPACE"
printf 'requested_cpuset=%s\n' "$ROBOTEST_CPUSET" | tee "$RUN_DIR/affinity-verifier.txt"
taskset -pc $$ | tee -a "$RUN_DIR/affinity-verifier.txt"
affinity_cpu_count="$(nproc)"
printf 'inherited_cpu_count=%s\n' "$affinity_cpu_count" \
  | tee -a "$RUN_DIR/affinity-verifier.txt"
if (( affinity_cpu_count < 1 || affinity_cpu_count > 6 )); then
  echo "runtime affinity must contain between one and six CPUs" >&2
  exit 2
fi

set +u
# The Jazzy underlay is a fixed Phase 0 dependency.
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
write_version_evidence

export CMAKE_BUILD_PARALLEL_LEVEL=4
export MAKEFLAGS=-j4
export RCUTILS_COLORIZED_OUTPUT=0

cd "$WORKSPACE"
timeout --signal=TERM --kill-after=20s 900s \
  colcon build --symlink-install --parallel-workers 4 --packages-up-to robotest_sim \
  --event-handlers console_direct+ 2>&1 | tee "$RUN_DIR/colcon-build.txt"

set +u
# The successful build immediately above generates this overlay.
# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"
set -u

timeout --signal=TERM --kill-after=10s 420s \
  colcon test --parallel-workers 4 \
  --packages-select robotest_interfaces robotest_description robotest_faults robotest_sim \
  --event-handlers console_direct+ 2>&1 | tee "$RUN_DIR/colcon-test.txt"
colcon test-result --verbose | tee "$RUN_DIR/colcon-test-result.txt"

XACRO="$WORKSPACE/src/robotest_description/urdf/robotest.urdf.xacro"
WORLD="$WORKSPACE/src/robotest_sim/worlds/robotest_lab.sdf"
timeout 30s xacro "$XACRO" namespace:=/robotest use_gazebo:=true enable_ground_truth:=true \
  > "$RUN_DIR/robotest.generated.urdf"
timeout 30s check_urdf "$RUN_DIR/robotest.generated.urdf" | tee "$RUN_DIR/check-urdf.txt"
timeout 30s gz sdf -k "$WORLD" | tee "$RUN_DIR/check-sdf.txt"
timeout 30s ros2 launch robotest_sim sim.launch.py seed:="$ROBOTEST_SIM_SEED" --show-args \
  | tee "$RUN_DIR/launch-arguments.txt"

export ROS_DOMAIN_ID="$((100 + ($$ % 100)))"
export GZ_PARTITION="robotest_phase1_${RUN_ID//[^A-Za-z0-9_]/_}"
printf 'ROS_DOMAIN_ID=%s\nGZ_PARTITION=%s\nROBOTEST_SIM_SEED=%s\n' \
  "$ROS_DOMAIN_ID" "$GZ_PARTITION" "$ROBOTEST_SIM_SEED" \
  | tee "$RUN_DIR/isolation.txt"

LAUNCH_GROUP_TOKEN="$(python3 -c 'import secrets; print(secrets.token_hex(32))')"
LAUNCH_GO_FIFO="$RUN_DIR/.launch-go.pipe"
LAUNCH_READY_FIFO="$RUN_DIR/.launch-ready.pipe"
mkfifo -- "$LAUNCH_GO_FIFO" "$LAUNCH_READY_FIFO"
exec {LAUNCH_GO_FD}<>"$LAUNCH_GO_FIFO"
exec {LAUNCH_READY_FD}<>"$LAUNCH_READY_FIFO"
launch_parent_pid="$BASHPID"
phase1_begin_signal_deferral
ROBOTEST_PHASE1_GROUP_TOKEN="$LAUNCH_GROUP_TOKEN" \
  setsid bash -c '
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
  ' _ "$LAUNCH_GO_FIFO" "$LAUNCH_READY_FIFO" \
  timeout --signal=TERM --kill-after=10s 100s \
  ros2 launch robotest_sim sim.launch.py headless:=true render_sensors:=true rviz:=false \
  seed:="$ROBOTEST_SIM_SEED" \
  {LAUNCH_GO_FD}>&- {LAUNCH_READY_FD}>&- > "$RUN_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
phase1_end_signal_deferral
LAUNCH_PGID="$LAUNCH_PID"
LAUNCH_SID="$LAUNCH_PID"
if ! exec {LAUNCH_PIDFD}<"/proc/$LAUNCH_PID"; then
  echo "failed to retain the gated launch procfd" >&2
  exit 1
fi
if ! python3 "$PIDFD_GROUP_HELPER" validate \
    --fd 9 \
    --parent-pid "$launch_parent_pid" \
    --pid "$LAUNCH_PID" \
    --token "$LAUNCH_GROUP_TOKEN" \
    > "$RUN_DIR/pidfd-validation.json" 9<&"$LAUNCH_PIDFD"; then
  echo "gated launch procfd validation failed" >&2
  exit 1
fi
mapfile -t pidfd_validation_fields < <(
  python3 - "$RUN_DIR/pidfd-validation.json" <<'PY'
import json
import sys
from pathlib import Path

payload = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
if payload.get('status') != 'validated' or payload.get('group_signal_flag') != 4:
    raise SystemExit(1)
print(payload['identity']['start_ticks'])
print(payload['token_sha256'])
PY
)
if (( ${#pidfd_validation_fields[@]} != 2 )); then
  echo "gated launch procfd evidence is incomplete" >&2
  exit 1
fi
LAUNCH_START_TICKS="${pidfd_validation_fields[0]}"
LAUNCH_GROUP_TOKEN_SHA256="${pidfd_validation_fields[1]}"
LAUNCH_PIDFD_VALIDATED=true
launch_ready=""
if ! IFS= read -r -t 2 -u "$LAUNCH_READY_FD" launch_ready \
  || [[ "$launch_ready" != READY ]]; then
  echo "gated launch wrapper did not become ready" >&2
  exit 1
fi
if ! phase1_release_launch_gate \
    "$LAUNCH_GO_FD" LAUNCH_GROUP_TOKEN LAUNCH_STARTED_UTC; then
  echo "failed to release gated launch" >&2
  exit 1
fi
close_launch_gate
printf 'launch_pid=%s\nlaunch_pgid=%s\nlaunch_sid=%s\nlaunch_start_ticks=%s\npidfd_group_flag=4\npidfd_validated=true\nlaunch_started_utc=%s\n' \
  "$LAUNCH_PID" "$LAUNCH_PGID" "$LAUNCH_SID" "$LAUNCH_START_TICKS" \
  "$LAUNCH_STARTED_UTC" > "$RUN_DIR/process-group.txt"

printf 'wall_epoch_s,pid,pgid,cpu_percent,rss_kib,processor,command\n' > "$RUN_DIR/resources.csv"
SAMPLING_STARTED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
sample_launch_group
phase1_begin_signal_deferral
phase1_sample_numeric_group_until_drain \
  "$LAUNCH_PGID" sample_launch_group 1 &
SAMPLER_PID=$!
phase1_end_signal_deferral

deadline=$((SECONDS + 50))
required_topics=(
  /clock
  /robotest/raw/scan /robotest/scan
  /robotest/raw/odom /robotest/odom
  /robotest/raw/imu /robotest/imu
  /robotest/validation/ground_truth
  /robotest/validation/contacts
  /robotest/validation/world_stats
)
while (( SECONDS < deadline )); do
  kill -0 "$LAUNCH_PID" 2>/dev/null || { echo "simulation launch exited early" >&2; exit 1; }
  topic_list="$(ros2 topic list 2>/dev/null || true)"
  all_present=1
  for topic in "${required_topics[@]}"; do
    grep -Fxq "$topic" <<< "$topic_list" || all_present=0
  done
  (( all_present == 1 )) && break
  sleep 1
done
(( all_present == 1 )) || { echo "timed out waiting for Phase 1 topics" >&2; exit 1; }

ros2 topic list -t | tee "$RUN_DIR/topics.txt"
ros2 node list | tee "$RUN_DIR/nodes.txt"
for node in \
  /robotest/robot_state_publisher \
  /robotest/parameter_bridge \
  /robotest/fault_proxy; do
  safe_node="${node#/}"
  safe_node="${safe_node//\//_}"
  timeout 10s ros2 param get "$node" use_sim_time \
    | tee "$RUN_DIR/sim-time-${safe_node}.txt" \
    | grep -Fq 'Boolean value is: True'
done
for topic in "${required_topics[@]}" /robotest/cmd_vel /tf /tf_static; do
  safe_name="${topic#/}"
  safe_name="${safe_name//\//_}"
  timeout 10s ros2 topic info "$topic" -v > "$RUN_DIR/qos-${safe_name}.txt"
done

timeout --signal=TERM --kill-after=5s 55s \
  python3 "$WORKSPACE/src/robotest_sim/tools/phase1_runtime_probe.py" \
  --output "$RUN_DIR/runtime-probe.json" | tee "$RUN_DIR/runtime-probe.txt"

python3 - "$RUN_DIR/runtime-probe.json" <<'PY'
import json
import sys
from pathlib import Path

result = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
if 'tf_edges_by_publisher_gid' in result:
    raise SystemExit('runtime probe contains prohibited callback-GID TF attribution')
expected_tf = {'/robotest/fault_proxy', '/robotest/robot_state_publisher'}
expected_tf_static = {'/robotest/robot_state_publisher'}
for topic, expected in (('/tf', expected_tf), ('/tf_static', expected_tf_static)):
    actual = [item['node'] for item in result['endpoints'][topic]['publishers']]
    if len(actual) != len(expected) or set(actual) != expected:
        raise SystemExit(f'{topic} endpoint owners {sorted(actual)} != {sorted(expected)}')
qos = result['qos_introspection']
if result['bounded_depth_live_proven_for_all_endpoints'] and any(
    not status['complete'] for status in qos.values()
):
    raise SystemExit('incomplete QoS introspection was mislabeled as live bounded-depth proof')
PY

set +e
timeout 8s ros2 run tf2_ros tf2_echo odom base_footprint \
  > "$RUN_DIR/tf-odom-base_footprint.txt" 2>&1
tf_status=$?
set -e
if [[ "$tf_status" -ne 0 && "$tf_status" -ne 124 ]]; then
  echo "tf2_echo failed with status $tf_status" >&2
  exit 1
fi
grep -q 'Translation:' "$RUN_DIR/tf-odom-base_footprint.txt"

if grep -Eiq '(^|[^a-z])(fatal|segmentation fault|out of memory|oom-kill)([^a-z]|$)' "$RUN_DIR/launch.log"; then
  echo "fatal launch signature detected" >&2
  exit 1
fi

stop_owned_launch
stop_sampler
capture_phase1_ogre2_log

max_rss_kib="$(awk -F, 'NR>1 {sum[$1]+=$5} END {max=0; for (t in sum) if (sum[t]>max) max=sum[t]; print max+0}' "$RUN_DIR/resources.csv")"
sampling_complete=false
sampling_scope='isolated launch PGID; sampling did not complete through verified process-group drain'
if [[ "$SAMPLER_STATUS" == "0" \
  && "$SAMPLER_DRAINED" == true \
  && "$LAUNCH_GROUP_DRAINED" == true \
  && -n "$SAMPLING_STARTED_UTC" \
  && -n "$SAMPLING_STOPPED_UTC" ]]; then
  sampling_complete=true
  sampling_scope='isolated launch PGID from first post-validation sample through verified process-group drain'
fi
printf '%s\n' \
  "peak_process_group_rss_kib=$max_rss_kib" \
  "target_max_kib=$((6 * 1024 * 1024))" \
  "sampler_exit_code=${SAMPLER_STATUS:-unavailable}" \
  "sampler_drained=${SAMPLER_DRAINED:-unknown}" \
  "sampling_complete_through_group_drain=$sampling_complete" \
  "sampling_scope=$sampling_scope" \
  'startup_capture_gap=process creation through first sample is not observed' \
  | tee "$RUN_DIR/resource-summary.txt"
(( max_rss_kib <= 6 * 1024 * 1024 )) || { echo "Phase 1 RSS target exceeded" >&2; exit 1; }

python3 - "$RUN_DIR/resources.csv" <<'PY'
import csv
import os
import sys

allowed = os.sched_getaffinity(0)
with open(sys.argv[1], encoding='utf-8', newline='') as stream:
    escaped = sorted(
        {
            int(row['processor'])
            for row in csv.DictReader(stream)
            if int(row['processor']) not in allowed
        }
    )
if escaped:
    raise SystemExit(f'runtime processors escaped inherited affinity: {escaped}')
PY

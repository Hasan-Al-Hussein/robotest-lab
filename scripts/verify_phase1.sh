#!/usr/bin/env bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

set -Eeuo pipefail
IFS=$'\n\t'
umask 022

SCRIPT_PATH="$(readlink -f -- "$0")"
WORKSPACE="$(cd -- "$(dirname -- "$SCRIPT_PATH")/.." && pwd -P)"
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
mkfifo -- "$LOG_FIFO"
exec 3>&1 4>&2
tee -a "$RUN_DIR/verify.log" < "$LOG_FIFO" >&3 &
TEE_PID=$!
exec > "$LOG_FIFO" 2>&1
rm -f -- "$LOG_FIFO"

LAUNCH_PID=""
LAUNCH_PGID=""
SAMPLER_PID=""
LAUNCH_STARTED_UTC=""
LAUNCH_STOPPED_UTC=""
LAUNCH_WAIT_STATUS=""
LAUNCH_TERM_SENT=false
LAUNCH_KILL_SENT=false
SAMPLING_STARTED_UTC=""
SAMPLING_STOPPED_UTC=""
TEE_STATUS=""
FINALIZED=0

sample_launch_group() {
  local sample_time=""
  sample_time="$(date +%s.%N)"
  ps -eo pid=,pgid=,pcpu=,rss=,psr=,comm= \
    | awk -v now="$sample_time" -v group="$LAUNCH_PGID" \
      '$2 == group {print now "," $1 "," $2 "," $3 "," $4 "," $5 "," $6}' \
    >> "$RUN_DIR/resources.csv"
}

stop_sampler() {
  if [[ -n "$SAMPLER_PID" ]]; then
    if kill -0 "$SAMPLER_PID" 2>/dev/null; then
      kill -TERM "$SAMPLER_PID" 2>/dev/null || true
    fi
    wait "$SAMPLER_PID" 2>/dev/null || true
  fi
  SAMPLER_PID=""
  if [[ -n "$SAMPLING_STARTED_UTC" && -z "$SAMPLING_STOPPED_UTC" ]]; then
    SAMPLING_STOPPED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  fi
}

stop_owned_launch() {
  if [[ -n "$LAUNCH_PID" && -n "$LAUNCH_PGID" && "$LAUNCH_PGID" =~ ^[0-9]+$ ]]; then
    local actual_pgid=""
    local wait_status=0
    actual_pgid="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ' || true)"
    if [[ -n "$actual_pgid" && "$actual_pgid" == "$LAUNCH_PGID" ]]; then
      if kill -TERM -- "-$LAUNCH_PGID" 2>/dev/null; then
        LAUNCH_TERM_SENT=true
      fi
      if [[ -n "$SAMPLER_PID" ]]; then
        sample_launch_group || true
      fi
      for _ in {1..50}; do
        kill -0 "$LAUNCH_PID" 2>/dev/null || break
        sleep 0.1
      done
      if kill -0 "$LAUNCH_PID" 2>/dev/null &&
        kill -KILL -- "-$LAUNCH_PGID" 2>/dev/null; then
        LAUNCH_KILL_SENT=true
      fi
    fi
    if wait "$LAUNCH_PID" 2>/dev/null; then
      wait_status=0
    else
      wait_status=$?
    fi
    LAUNCH_WAIT_STATUS="$wait_status"
  fi
  if [[ -n "$LAUNCH_STARTED_UTC" && -z "$LAUNCH_STOPPED_UTC" ]]; then
    LAUNCH_STOPPED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  fi
  LAUNCH_PID=""
  LAUNCH_PGID=""
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

close_verify_log() {
  exec 1>&3 2>&4
  if wait "$TEE_PID"; then
    TEE_STATUS=0
  else
    TEE_STATUS=$?
  fi
  exec 3>&- 4>&-
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
    "$TEE_STATUS" <<'PY'
import json
import sys
from pathlib import Path


def optional_int(value):
    return int(value) if value else None


summary = {
    'checksum_validation_exit_code': int(sys.argv[6]),
    'created_utc': sys.argv[3],
    'evidence_finalized_utc': sys.argv[4],
    'exit_code': int(sys.argv[5]),
    'gz_partition': sys.argv[8] or None,
    'launch': {
        'kill_sent': sys.argv[13] == 'true',
        'started_utc': sys.argv[9] or None,
        'stopped_utc': sys.argv[10] or None,
        'term_sent': sys.argv[12] == 'true',
        'wait_status': optional_int(sys.argv[11]),
    },
    'resource_sampling': {
        'scope': (
            'isolated launch PGID, from the first sample immediately after PGID '
            'validation through owned launch-leader shutdown; the small process-creation '
            'to first-sample interval is not observed'
        ),
        'started_utc': sys.argv[14] or None,
        'stopped_utc': sys.argv[15] or None,
    },
    'ros_domain_id': optional_int(sys.argv[7]),
    'run_id': sys.argv[2],
    'tee_exit_code': optional_int(sys.argv[16]),
}
Path(sys.argv[1]).write_text(
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
  local checksum_status=0
  local final_status=0
  local metadata_status=0

  if (( FINALIZED == 1 )); then
    return
  fi
  FINALIZED=1
  trap - EXIT INT TERM
  set +e

  # The sampler stays alive while the owned launch group is terminated so the
  # recorded interval covers controlled shutdown through launch-leader exit.
  stop_owned_launch
  stop_sampler
  capture_phase1_ogre2_log
  if (( execution_status == 0 )); then
    echo "Phase 1 automated checks PASS; finalizing evidence: $RUN_DIR"
  else
    echo "Phase 1 automated checks FAIL with status $execution_status; finalizing evidence: $RUN_DIR"
  fi
  close_verify_log

  final_status=$execution_status
  if [[ "$TEE_STATUS" != "0" ]]; then
    final_status=1
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

trap 'exit 130' INT
trap 'exit 143' TERM
trap cleanup EXIT

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

LAUNCH_STARTED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
setsid timeout --signal=TERM --kill-after=10s 100s \
  ros2 launch robotest_sim sim.launch.py headless:=true render_sensors:=true rviz:=false \
  seed:="$ROBOTEST_SIM_SEED" \
  > "$RUN_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID="$(ps -o pgid= -p "$LAUNCH_PID" | tr -d ' ')"
if [[ -z "$LAUNCH_PGID" || "$LAUNCH_PGID" != "$LAUNCH_PID" ]]; then
  echo "launch did not enter its own process group" >&2
  exit 1
fi
printf 'launch_pid=%s\nlaunch_pgid=%s\nlaunch_started_utc=%s\n' \
  "$LAUNCH_PID" "$LAUNCH_PGID" "$LAUNCH_STARTED_UTC" > "$RUN_DIR/process-group.txt"

printf 'wall_epoch_s,pid,pgid,cpu_percent,rss_kib,processor,command\n' > "$RUN_DIR/resources.csv"
SAMPLING_STARTED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
sample_launch_group
(
  while kill -0 "$LAUNCH_PID" 2>/dev/null; do
    sleep 1
    kill -0 "$LAUNCH_PID" 2>/dev/null || break
    sample_launch_group
  done
) &
SAMPLER_PID=$!

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
printf '%s\n' \
  "peak_process_group_rss_kib=$max_rss_kib" \
  "target_max_kib=$((6 * 1024 * 1024))" \
  'sampling_scope=isolated launch PGID from first post-PGID-validation sample through launch-leader shutdown' \
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

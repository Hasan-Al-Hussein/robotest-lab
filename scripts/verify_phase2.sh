#!/usr/bin/env bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

set -Eeuo pipefail
IFS=$'\n\t'
umask 022

SCRIPT_PATH="$(readlink -f -- "$0")"
WORKSPACE="$(cd -- "$(dirname -- "$SCRIPT_PATH")/.." && pwd -P)"
ROBOTEST_CPUSET="${ROBOTEST_CPUSET:-0-5}"
ROBOTEST_SIM_SEED="${ROBOTEST_SIM_SEED:-42}"

if [[ ! "$ROBOTEST_CPUSET" =~ ^[0-9]+([,-][0-9]+)*$ ]]; then
  echo "invalid ROBOTEST_CPUSET: $ROBOTEST_CPUSET" >&2
  exit 2
fi
if [[ ! "$ROBOTEST_SIM_SEED" =~ ^(0|[1-9][0-9]{0,9})$ ]] \
  || (( 10#$ROBOTEST_SIM_SEED > 4294967295 )); then
  echo "invalid ROBOTEST_SIM_SEED: expected a base-10 uint32" >&2
  exit 2
fi
if [[ "$ROBOTEST_SIM_SEED" != "42" ]]; then
  echo "Phase 2 acceptance is frozen to simulator seed 42" >&2
  exit 2
fi

# Re-exec once so builds, tests, launch, mission, and probes inherit at most six
# logical CPUs even when the caller omitted taskset.
if [[ "${ROBOTEST_PHASE2_AFFINITY_APPLIED:-0}" != "1" ]]; then
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
    ROBOTEST_PHASE2_AFFINITY_APPLIED=1 \
    ROBOTEST_CPUSET="$ROBOTEST_CPUSET" \
    ROBOTEST_SIM_SEED="$ROBOTEST_SIM_SEED" \
    ROBOTEST_PHASE2_CALLER_CWD="$ORIGINAL_CALLER_CWD" \
    ROBOTEST_PHASE2_ORIGINAL_ARGV_BASE64="$ORIGINAL_ARGV_BASE64" \
    "$SCRIPT_PATH" "$@"
fi

CALLER_CWD="${ROBOTEST_PHASE2_CALLER_CWD:-$(pwd -P)}"
ORIGINAL_ARGV_BASE64="${ROBOTEST_PHASE2_ORIGINAL_ARGV_BASE64:-}"
RUN_CREATED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
GIT_SHA="$(git -C "$WORKSPACE" rev-parse HEAD 2>/dev/null || printf unavailable)"
if GIT_STATUS_START="$(git -C "$WORKSPACE" status --porcelain=v1 --untracked-files=all 2>&1)"; then
  [[ -n "$GIT_STATUS_START" ]] && GIT_DIRTY=true || GIT_DIRTY=false
else
  GIT_DIRTY=true
fi

EVIDENCE_ROOT="$WORKSPACE/artifacts/evidence/phase2"
mkdir -p -- "$EVIDENCE_ROOT"
EVIDENCE_ROOT="$(realpath -- "$EVIDENCE_ROOT")"
case "$EVIDENCE_ROOT" in
  "$WORKSPACE"/artifacts/evidence/phase2) ;;
  *) echo "unsafe evidence root: $EVIDENCE_ROOT" >&2; exit 2 ;;
esac

RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)-$$"
RUN_DIR="$EVIDENCE_ROOT/$RUN_ID"
mkdir -p -- "$RUN_DIR"

# Retain this run plus the four newest prior runs. Only validated direct
# children of the Phase 2 evidence root can be pruned.
mapfile -t RUN_DIRS < <(
  find "$EVIDENCE_ROOT" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' \
    | sort -nr | awk '{print $2}'
)
if (( ${#RUN_DIRS[@]} > 5 )); then
  for old_dir in "${RUN_DIRS[@]:5}"; do
    old_dir="$(realpath -- "$old_dir")"
    case "$old_dir" in
      "$EVIDENCE_ROOT"/*) rm -rf -- "$old_dir" ;;
      *) echo "refusing to prune unsafe path: $old_dir" >&2; exit 2 ;;
    esac
  done
fi

USER_HOME_DIR="$(getent passwd "$(id -u)" | cut -d: -f6)"
OGRE2_LOG_PATH="$USER_HOME_DIR/.gz/rendering/ogre2.log"
if [[ -f "$OGRE2_LOG_PATH" && ! -L "$OGRE2_LOG_PATH" ]]; then
  cp -- "$OGRE2_LOG_PATH" "$RUN_DIR/ogre2-before-phase2-launch.log"
else
  printf 'No regular, non-symlink pre-existing OGRE2 log at %s\n' "$OGRE2_LOG_PATH" \
    > "$RUN_DIR/ogre2-before-phase2-launch.absent.txt"
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
PROBE_PID=""
PROBE_PGID=""
MISSION_PID=""
MISSION_PGID=""
MISSION_STATUS=""
PROBE_STATUS=""
SAMPLER_PID=""
LAUNCH_STARTED_UTC=""
LAUNCH_STOPPED_UTC=""
LAUNCH_STARTED_EPOCH_NS=""
SAMPLING_STARTED_UTC=""
SAMPLING_STOPPED_UTC=""
TEST_STARTED_EPOCH_NS=""
TEST_FINISHED_EPOCH_NS=""
TEE_STATUS=""
FINALIZED=0
MAX_RSS_KIB=""
OWNED_GROUPS_FILE="$RUN_DIR/owned-process-groups.txt"
: > "$OWNED_GROUPS_FILE"

record_owned_group() {
  local role="$1"
  local pid="$2"
  local pgid="$3"
  if [[ ! "$pid" =~ ^[0-9]+$ || ! "$pgid" =~ ^[0-9]+$ || "$pid" != "$pgid" ]]; then
    echo "invalid owned process group for $role: pid=$pid pgid=$pgid" >&2
    return 1
  fi
  printf '%s %s %s\n' "$role" "$pid" "$pgid" >> "$OWNED_GROUPS_FILE"
}

sample_owned_groups() {
  local sample_time=""
  sample_time="$(date +%s.%N)"
  ps -eo pid=,pgid=,pcpu=,rss=,psr=,comm= \
    | awk -v now="$sample_time" \
      'NR==FNR {role[$3]=$1; next} ($2 in role) {
        print now "," role[$2] "," $1 "," $2 "," $3 "," $4 "," $5 "," $6
      }' "$OWNED_GROUPS_FILE" - \
    >> "$RUN_DIR/resources.csv"
}

start_sampler() {
  printf 'wall_epoch_s,role,pid,pgid,cpu_percent,rss_kib,processor,command\n' \
    > "$RUN_DIR/resources.csv"
  SAMPLING_STARTED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  sample_owned_groups
  (
    while true; do
      sleep 1
      sample_owned_groups || true
    done
  ) &
  SAMPLER_PID=$!
}

stop_sampler() {
  if [[ -n "$SAMPLER_PID" ]]; then
    kill -TERM "$SAMPLER_PID" 2>/dev/null || true
    wait "$SAMPLER_PID" 2>/dev/null || true
    SAMPLER_PID=""
  fi
  if [[ -n "$SAMPLING_STARTED_UTC" && -z "$SAMPLING_STOPPED_UTC" ]]; then
    SAMPLING_STOPPED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  fi
}

stop_owned_group() {
  local role="$1"
  local pid="$2"
  local pgid="$3"
  local actual_pgid=""
  [[ -n "$pid" && -n "$pgid" && "$pid" =~ ^[0-9]+$ && "$pgid" =~ ^[0-9]+$ ]] \
    || return 0
  actual_pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  if [[ -n "$actual_pgid" && "$actual_pgid" != "$pgid" ]]; then
    echo "refusing to signal changed $role PGID: expected $pgid, found $actual_pgid" >&2
    return 1
  fi
  if ps -eo pgid= | awk -v group="$pgid" '$1 == group {found=1} END {exit !found}'; then
    kill -TERM -- "-$pgid" 2>/dev/null || true
    for _ in {1..50}; do
      ps -eo pgid= | awk -v group="$pgid" '$1 == group {found=1} END {exit !found}' \
        || break
      sleep 0.1
    done
    if ps -eo pgid= | awk -v group="$pgid" '$1 == group {found=1} END {exit !found}'; then
      kill -KILL -- "-$pgid" 2>/dev/null || true
    fi
  fi
  wait "$pid" 2>/dev/null || true
}

stop_probe() {
  [[ -n "$PROBE_PID" ]] || return 0
  stop_owned_group probe "$PROBE_PID" "$PROBE_PGID"
  PROBE_PID=""
  PROBE_PGID=""
}

stop_mission() {
  [[ -n "$MISSION_PID" ]] || return 0
  stop_owned_group mission "$MISSION_PID" "$MISSION_PGID"
  MISSION_PID=""
  MISSION_PGID=""
}

stop_launch() {
  [[ -n "$LAUNCH_PID" ]] || return 0
  stop_owned_group launch "$LAUNCH_PID" "$LAUNCH_PGID"
  if [[ -n "$LAUNCH_STARTED_UTC" && -z "$LAUNCH_STOPPED_UTC" ]]; then
    LAUNCH_STOPPED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  fi
  LAUNCH_PID=""
  LAUNCH_PGID=""
}

capture_phase2_ogre2_log() {
  local capture="$RUN_DIR/ogre2-phase2-launch.log"
  local absent="$RUN_DIR/ogre2-phase2-launch.absent.txt"
  local evidence="$RUN_DIR/ogre2-phase2-launch-evidence.json"
  [[ -e "$capture" || -e "$absent" ]] && return 0
  if [[ -z "$LAUNCH_STARTED_EPOCH_NS" ]]; then
    printf 'Phase 2 launch did not start; no renderer log can be attributed to it.\n' \
      > "$absent"
    return 0
  fi
  if [[ ! -f "$OGRE2_LOG_PATH" || -L "$OGRE2_LOG_PATH" ]]; then
    printf 'No regular, non-symlink Phase 2 OGRE2 log was available at %s\n' \
      "$OGRE2_LOG_PATH" > "$absent"
    return 0
  fi
  if ! python3 - "$OGRE2_LOG_PATH" "$RUN_DIR/ogre2-before-phase2-launch.log" \
      "$capture" "$evidence" "$LAUNCH_STARTED_EPOCH_NS" <<'PY'
import hashlib
import json
import shutil
import sys
from pathlib import Path

live_path = Path(sys.argv[1])
before_path = Path(sys.argv[2])
capture_path = Path(sys.argv[3])
evidence_path = Path(sys.argv[4])
launch_started_ns = int(sys.argv[5])
live_stat = live_path.stat()
live_hash = hashlib.sha256(live_path.read_bytes()).hexdigest()
before_hash = (
    hashlib.sha256(before_path.read_bytes()).hexdigest() if before_path.is_file() else None
)
fresh_after_launch = live_stat.st_mtime_ns >= launch_started_ns
content_delta = before_hash is None or live_hash != before_hash
evidence = {
    'before_sha256': before_hash,
    'captured_sha256': live_hash,
    'content_delta_from_prelaunch': content_delta,
    'launch_started_epoch_ns': launch_started_ns,
    'live_log_mtime_ns': live_stat.st_mtime_ns,
    'fresh_after_launch': fresh_after_launch,
    'source_path': str(live_path),
    'verdict': 'PASS' if fresh_after_launch and content_delta else 'FAIL',
}
evidence_path.write_text(json.dumps(evidence, indent=2, sort_keys=True) + '\n', encoding='utf-8')
if not fresh_after_launch:
    raise SystemExit('OGRE2 log predates this Phase 2 launch')
if not content_delta:
    raise SystemExit('OGRE2 log did not change during this Phase 2 launch')
shutil.copyfile(live_path, capture_path)
PY
  then
    printf 'OGRE2 log was present but was not fresh, launch-attributable evidence.\n' \
      > "$absent"
    return 0
  fi
}

close_verify_log() {
  exec 1>&3 2>&4
  if wait "$TEE_PID"; then TEE_STATUS=0; else TEE_STATUS=$?; fi
  exec 3>&- 4>&-
}

write_command_evidence() {
  python3 - "$RUN_DIR/command.json" "$CALLER_CWD" "$WORKSPACE" \
    "$ORIGINAL_ARGV_BASE64" "$$" <<'PY'
import base64
import json
import sys
from pathlib import Path

effective = [
    item.decode('utf-8', errors='surrogateescape')
    for item in Path(f'/proc/{sys.argv[5]}/cmdline').read_bytes().split(b'\0')
    if item
]
original = (
    json.loads(base64.b64decode(sys.argv[4]).decode('utf-8'))
    if sys.argv[4]
    else effective
)
evidence = {
    'argv_scope': 'script argv captured before internal taskset re-exec',
    'caller_cwd': sys.argv[2],
    'effective_process_argv': effective,
    'original_script_argv': original,
    'workspace_cwd': sys.argv[3],
}
Path(sys.argv[1]).write_text(
    json.dumps(evidence, indent=2, sort_keys=True) + '\n', encoding='utf-8'
)
PY
}

record_child_command() {
  local role="$1"
  local pid="$2"
  local stdout_path="$3"
  local stderr_path="$4"
  shift 4
  python3 - "$RUN_DIR/child-commands.json" "$role" "$pid" "$(pwd -P)" \
    "$stdout_path" "$stderr_path" "$@" <<'PY'
import json
import os
import sys
from pathlib import Path

output = Path(sys.argv[1])
role = sys.argv[2]
pid = int(sys.argv[3])
cwd = sys.argv[4]
stdout_path = sys.argv[5]
stderr_path = sys.argv[6]
invoked_argv = sys.argv[7:]
observed_argv = None
observed_cwd = None
try:
    observed_argv = [
        item.decode('utf-8', errors='surrogateescape')
        for item in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        if item
    ]
    observed_cwd = os.readlink(f'/proc/{pid}/cwd')
except OSError:
    pass
document = json.loads(output.read_text(encoding='utf-8')) if output.is_file() else {
    'scope': 'exact verifier-spawned process-group leaders and invocation argv',
    'commands': {},
}
document['commands'][role] = {
    'cwd': cwd,
    'invoked_argv': invoked_argv,
    'leader_pid': pid,
    'observed_leader_argv': observed_argv,
    'observed_leader_cwd': observed_cwd,
    'stderr_path': stderr_path,
    'stdout_path': stdout_path,
}
temporary = output.with_suffix('.tmp')
temporary.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n', encoding='utf-8')
temporary.replace(output)
PY
}

write_costmap_service_evidence() {
  timeout --signal=TERM --kill-after=3s 50s \
    python3 - "$RUN_DIR/costmap-service-summary.json" <<'PY'
import hashlib
import json
import math
import time
from contextlib import suppress
from pathlib import Path

import rclpy
from nav2_msgs.srv import GetCostmap

OUTPUT = Path(__import__('sys').argv[1])
SERVICE_SPECS = (
    ('global_costmap', '/robotest/global_costmap/get_costmap', 'map'),
    ('local_costmap', '/robotest/local_costmap/get_costmap', 'odom'),
)
WAIT_FOR_SERVICE_WALL_S = 10.0
WAIT_FOR_RESPONSE_WALL_S = 15.0
summaries = []
failures = []
node = None
rclpy.init()
try:
    node = rclpy.create_node('phase2_costmap_service_evidence', namespace='/robotest/evidence')
    for label, service_name, expected_frame in SERVICE_SPECS:
        client = node.create_client(GetCostmap, service_name)
        if not client.wait_for_service(timeout_sec=WAIT_FOR_SERVICE_WALL_S):
            failures.append(f'{service_name} was unavailable within the steady-wall deadline')
            continue
        request = GetCostmap.Request()
        future = client.call_async(request)
        deadline = time.monotonic() + WAIT_FOR_RESPONSE_WALL_S
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        if not future.done():
            failures.append(f'{service_name} did not respond within the steady-wall deadline')
            continue
        try:
            response = future.result()
        except Exception as error:
            failures.append(f'{service_name} failed: {type(error).__name__}: {error}')
            continue
        costmap = response.map
        width = int(costmap.metadata.size_x)
        height = int(costmap.metadata.size_y)
        resolution = float(costmap.metadata.resolution)
        data = bytes(costmap.data)
        stamp_ns = int(costmap.header.stamp.sec) * 1_000_000_000 + int(
            costmap.header.stamp.nanosec
        )
        checks = {
            'data_size_matches_geometry': len(data) == width * height,
            'frame_matches_contract': costmap.header.frame_id == expected_frame,
            'geometry_is_nonempty': width > 0 and height > 0 and len(data) > 0,
            'resolution_is_finite_positive': math.isfinite(resolution) and resolution > 0.0,
            'stamp_is_positive': stamp_ns > 0,
        }
        summaries.append(
            {
                'checks': checks,
                'cli_equivalent_argv': [
                    'ros2',
                    'service',
                    'call',
                    service_name,
                    'nav2_msgs/srv/GetCostmap',
                    '{}',
                ],
                'cost_range': {
                    'maximum': max(data) if data else None,
                    'minimum': min(data) if data else None,
                },
                'data_sha256': hashlib.sha256(data).hexdigest(),
                'data_size': len(data),
                'expected_frame_id': expected_frame,
                'frame_id': costmap.header.frame_id,
                'height_cells': height,
                'label': label,
                'request_yaml': '{}',
                'resolution_m_per_cell': resolution,
                'service_name': service_name,
                'service_type': 'nav2_msgs/srv/GetCostmap',
                'stamp_ns': stamp_ns,
                'width_cells': width,
            }
        )
        failures.extend(
            f'{service_name}: {name}' for name, passed in checks.items() if not passed
        )
finally:
    if node is not None:
        with suppress(Exception):
            node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

evidence = {
    'failures': failures,
    'request_contract': 'default GetCostmap request ({}) for each active Nav2 costmap',
    'services': summaries,
    'verdict': 'PASS' if not failures and len(summaries) == len(SERVICE_SPECS) else 'FAIL',
    'wait_for_response_wall_s': WAIT_FOR_RESPONSE_WALL_S,
    'wait_for_service_wall_s': WAIT_FOR_SERVICE_WALL_S,
}
if len(summaries) != len(SERVICE_SPECS):
    evidence['failures'].append(
        f'observed {len(summaries)} costmap responses, expected {len(SERVICE_SPECS)}'
    )
temporary = OUTPUT.with_suffix('.tmp')
temporary.write_text(
    json.dumps(evidence, indent=2, sort_keys=True, allow_nan=False) + '\n',
    encoding='utf-8',
)
temporary.replace(OUTPUT)
print(json.dumps(evidence, indent=2, sort_keys=True, allow_nan=False))
if evidence['verdict'] != 'PASS':
    raise SystemExit('; '.join(evidence['failures']))
PY
}

write_source_config_hashes() {
  local output="${1:-$RUN_DIR/source-config-hashes.json}"
  python3 - "$WORKSPACE" "$output" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

workspace = Path(sys.argv[1]).resolve()
output = Path(sys.argv[2])
explicit = [
    workspace / '.gitignore',
    workspace / 'README.md',
    workspace / 'pyproject.toml',
    workspace / 'scripts/verify_phase2.sh',
    workspace / 'config/collision-coverage.yaml',
    workspace / 'tests/phase2_startup_gate.py',
    workspace / 'tests/phase2_runtime_probe.py',
    workspace / 'tests/phase2_lifecycle_probe.py',
    workspace / 'tests/phase2_parameter_probe.py',
    workspace / 'tests/phase2_graph_probe.py',
    workspace / 'tests/phase3_runtime_gate.py',
    workspace / 'tests/phase3_orchestration.py',
    workspace / 'docs/testing/acceptance-criteria.md',
    workspace / 'docs/testing/verification-matrix.md',
    workspace / 'docs/architecture/metrics-contract.md',
    workspace / 'docs/architecture/topic-and-tf-contract.md',
    workspace / 'docs/decisions/0003-two-phase-fault-schedule-arming.md',
    workspace / 'docs/decisions/0004-phase2-command-ownership.md',
]
roots = [
    workspace / 'scenarios',
    workspace / 'src/robotest_interfaces',
    workspace / 'src/robotest_description',
    workspace / 'src/robotest_faults',
    workspace / 'src/robotest_sim',
    workspace / 'src/robotest_navigation',
    workspace / 'src/robotest_missions',
]
paths = {path for path in explicit if path.is_file() and not path.is_symlink()}
for root in roots:
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
    aggregate.update(relative.encode() + b'\0' + digest.encode() + b'\n')
output.write_text(
    json.dumps(
        {'algorithm': 'sha256', 'aggregate_sha256': aggregate.hexdigest(), 'files': entries},
        indent=2,
        sort_keys=True,
    ) + '\n',
    encoding='utf-8',
)
PY
}

write_source_config_mutation_evidence() {
  write_source_config_hashes "$RUN_DIR/source-config-hashes-end.json" || return 1
  python3 - "$RUN_DIR/source-config-hashes.json" \
    "$RUN_DIR/source-config-hashes-end.json" \
    "$RUN_DIR/source-config-mutation.json" <<'PY'
import json
import sys
from pathlib import Path

start_path = Path(sys.argv[1])
end_path = Path(sys.argv[2])
output = Path(sys.argv[3])
start = json.loads(start_path.read_text(encoding='utf-8'))
end = json.loads(end_path.read_text(encoding='utf-8'))
start_files = {item['path']: item['sha256'] for item in start['files']}
end_files = {item['path']: item['sha256'] for item in end['files']}
added = sorted(set(end_files) - set(start_files))
removed = sorted(set(start_files) - set(end_files))
changed = sorted(
    path for path in set(start_files) & set(end_files) if start_files[path] != end_files[path]
)
failures = []
if added:
    failures.append(f'source/config files added during run: {added}')
if removed:
    failures.append(f'source/config files removed during run: {removed}')
if changed:
    failures.append(f'source/config files changed during run: {changed}')
evidence = {
    'added_paths': added,
    'changed_paths': changed,
    'end_aggregate_sha256': end['aggregate_sha256'],
    'failures': failures,
    'removed_paths': removed,
    'start_aggregate_sha256': start['aggregate_sha256'],
    'verdict': 'PASS' if not failures else 'FAIL',
}
output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + '\n', encoding='utf-8')
if failures:
    raise SystemExit('; '.join(failures))
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


def command(argv):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=10, check=False)
        return {
            'argv': argv,
            'exit_code': result.returncode,
            'output': (result.stdout + result.stderr).strip()[:8192],
        }
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'argv': argv, 'exit_code': None, 'output': str(error)}


def package(name):
    result = command(['dpkg-query', '-W', '-f=${Version}', name])
    return result['output'] if result['exit_code'] == 0 else None


try:
    colcon = importlib.metadata.version('colcon-core')
except importlib.metadata.PackageNotFoundError:
    colcon = None
evidence = {
    'build_type': 'CMake default (CMAKE_BUILD_TYPE not forced by this verifier)',
    'colcon_core': colcon,
    'commands': {
        name: command(argv)
        for name, argv in {
            'gazebo': ['gz', 'sim', '--versions'],
            'git': ['git', '--version'],
            'python': ['python3', '--version'],
            'shellcheck': ['shellcheck', '--version'],
        }.items()
    },
    'dpkg_packages': {
        name: package(name)
        for name in (
            'ros-jazzy-ros-base',
            'ros-jazzy-ros-gz',
            'ros-jazzy-navigation2',
            'ros-jazzy-nav2-bringup',
            'ros-jazzy-rmw-fastrtps-cpp',
        )
    },
    'platform': {
        'kernel': platform.release(),
        'machine': platform.machine(),
        'os_release': platform.freedesktop_os_release(),
        'python_runtime': platform.python_version(),
        'wsl_distro_name': os.environ.get('WSL_DISTRO_NAME'),
    },
    'rmw_implementation': command(
        [
            'python3',
            '-c',
            'from rclpy.utilities import get_rmw_implementation_identifier; '
            'print(get_rmw_implementation_identifier())',
        ]
    ),
    'ros_distro': os.environ.get('ROS_DISTRO'),
}
Path(sys.argv[1]).write_text(
    json.dumps(evidence, indent=2, sort_keys=True) + '\n', encoding='utf-8'
)
PY
}

write_installed_source_binding() {
  python3 - "$WORKSPACE" "$RUN_DIR/installed-source-binding.json" <<'PY'
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

workspace = Path(sys.argv[1]).resolve()
output = Path(sys.argv[2])


def share(package: str) -> Path:
    result = subprocess.run(
        ['ros2', 'pkg', 'prefix', '--share', package],
        capture_output=True,
        check=True,
        text=True,
        timeout=10,
    )
    return Path(result.stdout.strip()).resolve()


def prefix(package: str) -> Path:
    result = subprocess.run(
        ['ros2', 'pkg', 'prefix', package],
        capture_output=True,
        check=True,
        text=True,
        timeout=10,
    )
    return Path(result.stdout.strip()).resolve()


def digest(path: Path) -> str | None:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def elf_build_id(path: Path) -> str | None:
    if not path.is_file():
        return None
    result = subprocess.run(
        ['readelf', '-n', str(path)],
        capture_output=True,
        check=False,
        text=True,
        timeout=10,
    )
    for line in result.stdout.splitlines():
        if 'Build ID:' in line:
            return line.split('Build ID:', 1)[1].strip()
    return None


share_pairs = [
    ('robotest_navigation', 'launch/phase2.launch.py'),
    ('robotest_navigation', 'config/nav2_params.yaml'),
    ('robotest_navigation', 'maps/robotest_lab.yaml'),
    ('robotest_navigation', 'maps/robotest_lab.pgm'),
    ('robotest_navigation', 'behavior_trees/navigate_to_pose_actuation_free.xml'),
    ('robotest_sim', 'launch/sim.launch.py'),
    ('robotest_sim', 'config/bridge.yaml'),
    ('robotest_sim', 'worlds/robotest_lab.sdf'),
    ('robotest_missions', 'schema/mission.schema.json'),
]
entries = []
failures = []
for package, relative in share_pairs:
    source = workspace / 'src' / package / relative
    installed = share(package) / relative
    source_hash = digest(source)
    installed_hash = digest(installed)
    matches = source_hash is not None and source_hash == installed_hash
    entries.append(
        {
            'binding_type': 'package_share_file',
            'package': package,
            'relative_path': relative,
            'source_path': str(source),
            'installed_path': str(installed),
            'source_sha256': source_hash,
            'installed_sha256': installed_hash,
            'matches': matches,
        }
    )
    if not matches:
        failures.append(f'{package}/{relative}')

for module_name in (
    'robotest_missions.artifacts',
    'robotest_missions.execution',
    'robotest_missions.mission_runner',
    'robotest_missions.models',
    'robotest_missions.ros_action',
    'robotest_missions.schema',
):
    leaf = module_name.rsplit('.', 1)[-1]
    source = workspace / 'src' / 'robotest_missions' / 'robotest_missions' / f'{leaf}.py'
    result = subprocess.run(
        [
            'python3',
            '-c',
            'import importlib, pathlib, sys; '
            'print(pathlib.Path(importlib.import_module(sys.argv[1]).__file__).resolve())',
            module_name,
        ],
        capture_output=True,
        check=True,
        text=True,
        timeout=10,
    )
    installed = Path(result.stdout.strip()).resolve()
    source_hash = digest(source)
    installed_hash = digest(installed)
    matches = source_hash is not None and source_hash == installed_hash
    entries.append(
        {
            'binding_type': 'python_runtime_module',
            'module': module_name,
            'source_path': str(source),
            'installed_path': str(installed),
            'source_sha256': source_hash,
            'installed_sha256': installed_hash,
            'matches': matches,
        }
    )
    if not matches:
        failures.append(module_name)

mission_executable = prefix('robotest_missions') / 'lib/robotest_missions/mission_runner'
mission_executable_ok = mission_executable.is_file() and os.access(mission_executable, os.X_OK)
entries.append(
    {
        'binding_type': 'ros_console_executable',
        'package': 'robotest_missions',
        'executable': 'mission_runner',
        'installed_path': str(mission_executable),
        'installed_sha256': digest(mission_executable),
        'executable_bit_set': mission_executable_ok,
        'matches': mission_executable_ok,
    }
)
if not mission_executable_ok:
    failures.append('robotest_missions/mission_runner executable')

for executable, source_relative, installed_name, require_source_executable in (
    (
        'lifecycle_startup_trigger',
        'scripts/lifecycle_startup_trigger.py',
        'lifecycle_startup_trigger',
        False,
    ),
    ('generate_map.py', 'tools/generate_map.py', 'generate_map.py', True),
):
    source = workspace / 'src/robotest_navigation' / source_relative
    installed = prefix('robotest_navigation') / 'lib/robotest_navigation' / installed_name
    try:
        installed_resolved = installed.resolve(strict=True)
    except OSError:
        installed_resolved = None
    source_hash = digest(source)
    installed_hash = digest(installed_resolved) if installed_resolved is not None else None
    executable_bit_set = (
        installed_resolved is not None
        and installed_resolved.is_file()
        and os.access(installed_resolved, os.X_OK)
    )
    resolved_target_is_source_file = False
    if installed_resolved is not None and source.is_file():
        try:
            resolved_target_is_source_file = installed_resolved.samefile(source)
        except OSError:
            resolved_target_is_source_file = False
    source_mode = oct(source.stat().st_mode & 0o7777) if source.is_file() else None
    source_executable_bit_set = source.is_file() and os.access(source, os.X_OK)
    installed_mode = (
        oct(installed_resolved.stat().st_mode & 0o7777)
        if installed_resolved is not None and installed_resolved.is_file()
        else None
    )
    matches = (
        source_hash is not None
        and source_hash == installed_hash
        and executable_bit_set
        and (source_executable_bit_set or not require_source_executable)
    )
    entries.append(
        {
            'binding_type': 'installed_script_executable',
            'package': 'robotest_navigation',
            'executable': executable,
            'source_path': str(source),
            'source_sha256': source_hash,
            'source_mode_octal': source_mode,
            'source_executable_required': require_source_executable,
            'source_executable_bit_set': source_executable_bit_set,
            'installed_path': str(installed),
            'installed_is_symlink': installed.is_symlink(),
            'installed_symlink_target': (
                os.readlink(installed) if installed.is_symlink() else None
            ),
            'installed_resolved_path': (
                str(installed_resolved) if installed_resolved is not None else None
            ),
            'installed_resolved_sha256': installed_hash,
            'installed_resolved_mode_octal': installed_mode,
            'resolved_target_is_source_file': resolved_target_is_source_file,
            'executable_bit_set': executable_bit_set,
            'comparison_method': (
                'source bytes equal resolved installed executable bytes; resolved target '
                'must be a regular executable; mode and same-file identity record '
                'symlink-install truth'
            ),
            'matches': matches,
        }
    )
    if not matches:
        failures.append(f'robotest_navigation/{executable} executable')

fault_prefix = prefix('robotest_faults')
for build_relative, install_relative in (
    ('build/robotest_faults/fault_proxy_node', 'lib/robotest_faults/fault_proxy_node'),
    ('build/robotest_faults/librobotest_faults_core.so', 'lib/librobotest_faults_core.so'),
):
    build_artifact = workspace / build_relative
    installed = fault_prefix / install_relative
    build_hash = digest(build_artifact)
    installed_hash = digest(installed)
    build_id = elf_build_id(build_artifact)
    installed_build_id = elf_build_id(installed)
    matches = build_id is not None and build_id == installed_build_id
    entries.append(
        {
            'binding_type': 'built_runtime_artifact',
            'package': 'robotest_faults',
            'build_path': str(build_artifact),
            'installed_path': str(installed),
            'build_sha256': build_hash,
            'installed_sha256': installed_hash,
            'build_elf_build_id': build_id,
            'installed_elf_build_id': installed_build_id,
            'comparison_method': (
                'matching GNU ELF build-id; whole-file hashes recorded but may differ '
                'because CMake rewrites install RPATH'
            ),
            'matches': matches,
        }
    )
    if not matches:
        failures.append(f'robotest_faults/{install_relative}')

sim_prefix = prefix('robotest_sim')
gate_build = (workspace / 'build/robotest_sim/contact_stream_gate').resolve()
gate_installed_declared = sim_prefix / 'lib/robotest_sim/contact_stream_gate'
try:
    gate_installed = gate_installed_declared.resolve(strict=True)
except OSError:
    gate_installed = gate_installed_declared
gate_build_hash = digest(gate_build)
gate_installed_hash = digest(gate_installed)
gate_build_id = elf_build_id(gate_build)
gate_installed_build_id = elf_build_id(gate_installed)
gate_matches = (
    gate_build.is_file()
    and os.access(gate_build, os.X_OK)
    and gate_installed.is_file()
    and os.access(gate_installed, os.X_OK)
    and gate_installed_declared.samefile(gate_installed)
    and gate_build_hash == gate_installed_hash
    and gate_build_id is not None
    and gate_build_id == gate_installed_build_id
)
entries.append(
    {
        'binding_type': 'built_runtime_artifact',
        'package': 'robotest_sim',
        'executable': 'contact_stream_gate',
        'build_path': str(gate_build),
        'installed_declared_path': str(gate_installed_declared),
        'installed_path': str(gate_installed),
        'build_sha256': gate_build_hash,
        'installed_sha256': gate_installed_hash,
        'build_elf_build_id': gate_build_id,
        'build_install_sha256_match': gate_build_hash == gate_installed_hash,
        'installed_elf_build_id': gate_installed_build_id,
        'installed_declared_samefile': (
            gate_installed_declared.exists()
            and gate_installed_declared.samefile(gate_installed)
        ),
        'regular_executable': (
            gate_installed.is_file() and os.access(gate_installed, os.X_OK)
        ),
        'comparison_method': (
            'matching whole-file SHA-256 and GNU ELF build-id plus exact resolved '
            'installed executable identity'
        ),
        'matches': gate_matches,
    }
)
if not gate_matches:
    failures.append('robotest_sim/contact_stream_gate executable')

aggregator_build = (
    workspace / 'build/robotest_sim/librobotest_contact_aggregator_system.so'
).resolve()
aggregator_installed_declared = (
    sim_prefix / 'lib/robotest_sim/librobotest_contact_aggregator_system.so'
)
try:
    aggregator_installed = aggregator_installed_declared.resolve(strict=True)
except OSError:
    aggregator_installed = aggregator_installed_declared
aggregator_build_hash = digest(aggregator_build)
aggregator_installed_hash = digest(aggregator_installed)
aggregator_build_id = elf_build_id(aggregator_build)
aggregator_installed_build_id = elf_build_id(aggregator_installed)
aggregator_declared_samefile = (
    aggregator_installed_declared.exists()
    and aggregator_installed_declared.samefile(aggregator_installed)
)
aggregator_build_install_samefile = (
    aggregator_build.is_file()
    and aggregator_installed.is_file()
    and aggregator_build.samefile(aggregator_installed)
)
aggregator_matches = (
    aggregator_build.is_file()
    and aggregator_installed.is_file()
    and aggregator_installed_declared.is_symlink()
    and aggregator_declared_samefile
    and aggregator_build_install_samefile
    and aggregator_build_hash == aggregator_installed_hash
    and aggregator_build_id is not None
    and aggregator_build_id == aggregator_installed_build_id
)
entries.append(
    {
        'binding_type': 'built_runtime_artifact',
        'package': 'robotest_sim',
        'library': 'librobotest_contact_aggregator_system.so',
        'build_path': str(aggregator_build),
        'installed_declared_path': str(aggregator_installed_declared),
        'installed_path': str(aggregator_installed),
        'build_sha256': aggregator_build_hash,
        'installed_sha256': aggregator_installed_hash,
        'build_elf_build_id': aggregator_build_id,
        'build_install_sha256_match': (
            aggregator_build_hash == aggregator_installed_hash
        ),
        'build_install_samefile': aggregator_build_install_samefile,
        'installed_elf_build_id': aggregator_installed_build_id,
        'installed_declared_is_symlink': aggregator_installed_declared.is_symlink(),
        'installed_declared_samefile': aggregator_declared_samefile,
        'regular_file': aggregator_installed.is_file(),
        'comparison_method': (
            'matching whole-file SHA-256 and GNU ELF build-id plus exact symlink-install '
            'build/installed file identity'
        ),
        'matches': aggregator_matches,
    }
)
if not aggregator_matches:
    failures.append('robotest_sim/librobotest_contact_aggregator_system.so')
evidence = {'verdict': 'PASS' if not failures else 'FAIL', 'failures': failures, 'files': entries}
output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + '\n', encoding='utf-8')
if failures:
    raise SystemExit(f'source/install mismatch: {failures}')
PY
}

write_startup_gate_ordering_evidence() {
  python3 - "$SCRIPT_PATH" "$RUN_DIR/startup-gate-ordering.json" <<'PY'
import json
import os
import sys
import tempfile
from pathlib import Path

script = Path(sys.argv[1]).resolve()
output = Path(sys.argv[2])
lines = script.read_text(encoding='utf-8').splitlines()
markers = [
    f'# PHASE2_{name}_ORDER'
    for name in (
        'START_SAMPLER',
        'STARTUP_GATE',
        'LIFECYCLE_PROBE',
        'FINAL_LAUNCH_LOG_GATE',
        'ACCEPTANCE',
    )
]
positions = {}
failures = []
for marker in markers:
    matches = [index for index, line in enumerate(lines) if line.strip() == marker]
    if len(matches) != 1:
        failures.append(f'{marker!r} must occur exactly once; observed {len(matches)}')
    elif matches:
        positions[marker] = matches[0]

if len(positions) == len(markers):
    indices = [positions[marker] for marker in markers]
    if indices != sorted(indices) or len(set(indices)) != len(indices):
        failures.append('sampler, startup gate, and lifecycle-probe markers are not ordered')
    else:
        (
            sampler_index,
            gate_index,
            lifecycle_index,
            final_log_gate_index,
            acceptance_index,
        ) = indices
        launch_start = max(
            (index for index, line in enumerate(lines[:sampler_index]) if line == 'LAUNCH_COMMAND=('),
            default=-1,
        )
        launch_segment = '\n'.join(lines[launch_start:sampler_index])
        sampler_segment = '\n'.join(lines[sampler_index + 1 : gate_index])
        gate_segment = '\n'.join(lines[gate_index + 1 : lifecycle_index])
        lifecycle_segment = '\n'.join(lines[lifecycle_index + 1 :])
        before_final_log_gate = '\n'.join(
            lines[lifecycle_index + 1 : final_log_gate_index]
        )
        final_log_gate_segment = '\n'.join(
            lines[final_log_gate_index + 1 : acceptance_index]
        )
        requirements = {
            'launch_result_path_bound': (
                launch_start >= 0
                and 'lifecycle_startup_result_path:="$STARTUP_RESULT"' in launch_segment
            ),
            'sampler_started_before_gate': 'start_sampler' in sampler_segment,
            'startup_gate_invoked': 'python3 "$STARTUP_GATE"' in gate_segment,
            'startup_result_supplied_to_gate': '--result "$STARTUP_RESULT"' in gate_segment,
            'startup_gate_output_supplied': '--output "$STARTUP_GATE_EVIDENCE"' in gate_segment,
            'launch_pid_watched': '--watch-pid "$LAUNCH_PID"' in gate_segment,
            'lifecycle_probe_after_gate': 'python3 "$LIFECYCLE_PROBE"' in lifecycle_segment,
            'launch_stopped_before_final_log_gate': 'stop_launch' in before_final_log_gate,
            'final_launch_log_gate_invoked': (
                '--scan-launch-log "$RUN_DIR/launch.log"' in final_log_gate_segment
            ),
            'final_launch_log_gate_persisted': (
                '--output "$RUN_DIR/launch-log-signature-gate.json"'
                in final_log_gate_segment
            ),
        }
        failures.extend(name for name, passed in requirements.items() if not passed)

evidence = {
    'schema_version': 1,
    'verdict': 'PASS' if not failures else 'FAIL',
    'script_path': str(script),
    'marker_line_numbers': {
        marker: positions[marker] + 1 for marker in markers if marker in positions
    },
    'failures': failures,
}

descriptor, pending_name = tempfile.mkstemp(
    dir=output.parent, prefix=f'.{output.name}.', suffix='.pending', text=True
)
pending = Path(pending_name)
try:
    with os.fdopen(descriptor, 'w', encoding='utf-8', newline='\n') as stream:
        descriptor = -1
        stream.write(json.dumps(evidence, indent=2, sort_keys=True) + '\n')
        stream.flush()
        os.fsync(stream.fileno())
    pending.replace(output)
finally:
    if descriptor >= 0:
        os.close(descriptor)
    pending.unlink(missing_ok=True)
print(json.dumps(evidence, indent=2, sort_keys=True))
if failures:
    raise SystemExit('; '.join(failures))
PY
}

write_contact_gate_runtime_attestation() {
  local output_path="$1"
  local initial_path="${2:-}"
  python3 - \
    "$WORKSPACE" "$LAUNCH_PID" "$ROS_DOMAIN_ID" "$GZ_PARTITION" \
    "$output_path" "$initial_path" <<'PY'
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import yaml

workspace = Path(sys.argv[1]).resolve()
launch_pid = int(sys.argv[2])
domain_id = int(sys.argv[3])
partition = sys.argv[4]
output = Path(sys.argv[5])
initial_path = Path(sys.argv[6]) if sys.argv[6] else None
module_path = workspace / 'tests/phase3_runtime_gate.py'
sys.path.insert(0, str(workspace / 'tests'))
import phase3_orchestration as orchestration
spec = importlib.util.spec_from_file_location('robotest_contact_gate_attestor', module_path)
if spec is None or spec.loader is None:
    raise SystemExit('cannot load contact gate attestor')
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
attestation = module._contact_gate_binary_attestation(
    workspace, launch_pid, domain_id, partition
)
if attestation.get('verdict') != 'PASS':
    raise SystemExit(f'contact gate runtime attestation failed: {attestation}')
aggregator_attestation = module._contact_aggregator_binary_attestation(
    workspace, launch_pid, domain_id, partition
)
if aggregator_attestation.get('verdict') != 'PASS':
    raise SystemExit(
        f'contact aggregator runtime attestation failed: {aggregator_attestation}'
    )
manifest_path = workspace / 'config/collision-coverage.yaml'
manifest_bytes = manifest_path.read_bytes()
manifest = yaml.safe_load(manifest_bytes)
contact_stream = orchestration._contact_stream_manifest_v3(manifest, workspace)
declared_source_inventory_sha256 = contact_stream['gate'][
    'source_inventory_sha256'
]
if attestation.get('source_inventory_sha256') != declared_source_inventory_sha256:
    raise SystemExit('contact gate runtime source inventory differs from manifest v3')
if (
    aggregator_attestation.get('source_inventory_sha256')
    != declared_source_inventory_sha256
):
    raise SystemExit('contact aggregator runtime source inventory differs from manifest v3')
if not (
    aggregator_attestation.get('installed_declared_is_symlink') is True
    and aggregator_attestation.get('build_install_samefile') is True
):
    raise SystemExit('contact aggregator runtime lacks exact symlink-install identity')
aggregator_build_install_fields = (
    'build_elf_build_id',
    'build_embedded_source_inventory_match',
    'build_embedded_source_inventory_sha256',
    'build_install_build_id_match',
    'build_install_embedded_source_inventory_match',
    'build_install_samefile',
    'build_install_sha256_match',
    'build_path',
    'build_regular_file',
    'build_sha256',
    'installed_declared_is_symlink',
    'installed_declared_path',
    'installed_elf_build_id',
    'installed_embedded_source_inventory_match',
    'installed_embedded_source_inventory_sha256',
    'installed_path',
    'installed_regular_file',
    'installed_sha256',
    'package',
    'schema_version',
    'source_inventory_sha256',
)


def contact_aggregator_build_install_binding(attestation):
    return {
        field: attestation[field]
        for field in aggregator_build_install_fields
    }


aggregator_build_install = contact_aggregator_build_install_binding(
    aggregator_attestation
)
aggregator_build_install_sha256 = orchestration.canonical_sha256(
    aggregator_build_install
)
binding_path = output.parent / 'installed-source-binding.json'
binding_bytes = binding_path.read_bytes()
binding = json.loads(binding_bytes)
gate_records = [
    record
    for record in binding.get('files', [])
    if record.get('package') == 'robotest_sim'
    and record.get('executable') == 'contact_stream_gate'
]
if len(gate_records) != 1:
    raise SystemExit('installed source binding lacks exactly one contact gate record')
gate_record = gate_records[0]
if not (
    gate_record.get('matches') is True
    and gate_record.get('build_install_sha256_match') is True
    and gate_record.get('regular_executable') is True
    and gate_record.get('installed_declared_samefile') is True
    and Path(gate_record['build_path']).resolve()
    == (workspace / attestation['build_path']).resolve()
    and Path(gate_record['installed_declared_path']).resolve()
    == (workspace / attestation['installed_declared_path']).resolve()
    and Path(gate_record['installed_path']).resolve()
    == (workspace / attestation['installed_path']).resolve()
    and gate_record.get('build_sha256') == attestation.get('build_sha256')
    and gate_record.get('installed_sha256') == attestation.get('installed_sha256')
    and gate_record.get('build_sha256') == gate_record.get('installed_sha256')
    and gate_record.get('build_elf_build_id') == attestation.get('build_elf_build_id')
    and gate_record.get('installed_elf_build_id')
    == attestation.get('installed_elf_build_id')
):
    raise SystemExit('contact gate runtime differs from installed source binding')
aggregator_records = [
    record
    for record in binding.get('files', [])
    if record.get('package') == 'robotest_sim'
    and record.get('library') == 'librobotest_contact_aggregator_system.so'
]
if len(aggregator_records) != 1:
    raise SystemExit('installed source binding lacks exactly one contact aggregator record')
aggregator_record = aggregator_records[0]
if not (
    aggregator_record.get('matches') is True
    and aggregator_record.get('build_install_sha256_match') is True
    and aggregator_record.get('build_install_samefile') is True
    and aggregator_record.get('regular_file') is True
    and aggregator_record.get('installed_declared_samefile') is True
    and aggregator_record.get('installed_declared_is_symlink') is True
    and aggregator_record.get('installed_declared_is_symlink')
    == aggregator_attestation.get('installed_declared_is_symlink')
    and Path(aggregator_record['build_path']).resolve()
    == (workspace / aggregator_attestation['build_path']).resolve()
    and Path(aggregator_record['installed_declared_path']).resolve()
    == (workspace / aggregator_attestation['installed_declared_path']).resolve()
    and Path(aggregator_record['installed_path']).resolve()
    == (workspace / aggregator_attestation['installed_path']).resolve()
    and aggregator_record.get('build_sha256')
    == aggregator_attestation.get('build_sha256')
    and aggregator_record.get('installed_sha256')
    == aggregator_attestation.get('installed_sha256')
    and aggregator_record.get('build_elf_build_id')
    == aggregator_attestation.get('build_elf_build_id')
    and aggregator_record.get('installed_elf_build_id')
    == aggregator_attestation.get('installed_elf_build_id')
):
    raise SystemExit('contact aggregator runtime differs from installed source binding')
result = {
    'contact_aggregator_build_install_sha256': aggregator_build_install_sha256,
    'contact_aggregator_binary_attestation': aggregator_attestation,
    'contact_gate_binary_attestation': attestation,
    'manifest_declared_sha256': manifest['manifest_sha256'],
    'manifest_file_sha256': hashlib.sha256(manifest_bytes).hexdigest(),
    'manifest_path': manifest_path.relative_to(workspace).as_posix(),
    'manifest_source_inventory_sha256': declared_source_inventory_sha256,
    'installed_source_binding_sha256': hashlib.sha256(binding_bytes).hexdigest(),
    'phase': 'final' if initial_path is not None else 'ready',
    'producer': 'robotest_phase2/contact_gate_runtime_attestor',
    'schema_version': 1,
    'stable_identity': None,
}
if initial_path is not None:
    initial_bytes = initial_path.read_bytes()
    initial = json.loads(initial_bytes)
    initial_attestation = initial['contact_gate_binary_attestation']
    initial_aggregator_attestation = initial['contact_aggregator_binary_attestation']
    initial_aggregator_build_install = contact_aggregator_build_install_binding(
        initial_aggregator_attestation
    )
    if (
        initial.get('manifest_path') != result['manifest_path']
        or initial.get('manifest_file_sha256') != result['manifest_file_sha256']
        or initial.get('manifest_declared_sha256') != result['manifest_declared_sha256']
    ):
        raise SystemExit('contact stream manifest changed before final evaluation')
    if (
        initial.get('installed_source_binding_sha256')
        != result['installed_source_binding_sha256']
    ):
        raise SystemExit('installed source binding changed before final evaluation')
    stable_fields = (
        'build_elf_build_id', 'build_embedded_source_inventory_sha256',
        'build_install_sha256_match',
        'build_path', 'build_sha256', 'installed_declared_path',
        'installed_device', 'installed_elf_build_id', 'installed_inode',
        'installed_embedded_source_inventory_sha256', 'installed_path',
        'installed_sha256',
        'live_cmdline_sha256', 'live_device', 'live_elf_build_id',
        'live_embedded_source_inventory_sha256',
        'live_executable_link', 'live_executable_path', 'live_executable_sha256', 'live_inode',
        'live_pgid', 'live_pid', 'live_ppid', 'live_sid', 'live_size_bytes',
        'live_start_ticks', 'observed_gz_partition', 'observed_ros_domain_id',
        'source_inventory_sha256',
    )
    if any(initial_attestation.get(key) != attestation.get(key) for key in stable_fields):
        raise SystemExit('contact gate runtime identity changed before final evaluation')
    if (
        initial_aggregator_attestation.get('stable_identity_sha256')
        != aggregator_attestation.get('stable_identity_sha256')
        or initial_aggregator_attestation.get('stable_identity')
        != aggregator_attestation.get('stable_identity')
    ):
        raise SystemExit('contact aggregator runtime identity changed before final evaluation')
    if (
        initial_aggregator_build_install != aggregator_build_install
        or initial.get('contact_aggregator_build_install_sha256')
        != aggregator_build_install_sha256
    ):
        raise SystemExit(
            'contact aggregator build/install identity changed before final evaluation'
        )
    result['initial_artifact_sha256'] = hashlib.sha256(initial_bytes).hexdigest()
    result['stable_identity'] = True
output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
PY
}

write_fixture_proof() {
  python3 - "$WORKSPACE" "$WORKSPACE/build/robotest_missions" \
    "$RUN_DIR/fixture-proof.json" "$TEST_STARTED_EPOCH_NS" \
    "$TEST_FINISHED_EPOCH_NS" <<'PY'
import ast
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

workspace = Path(sys.argv[1])
root = Path(sys.argv[2])
output = Path(sys.argv[3])
test_started_ns = int(sys.argv[4])
test_finished_ns = int(sys.argv[5])
sources = [
    workspace / 'src/robotest_missions/test/test_artifacts.py',
    workspace / 'src/robotest_missions/test/test_execution.py',
    workspace / 'src/robotest_missions/test/test_ros_action.py',
]
expected = {
    'test_delayed_simulation_clock_blocks_dispatch_until_positive',
    'test_feedback_trace_at_fixed_capacity_can_complete',
    'test_feedback_trace_overflow_fails_closed_with_bounded_cancel',
    'test_frozen_simulation_clock_uses_wall_escape_and_bounded_cancel',
    'test_goal_response_records_zero_raw_stamp_without_treating_it_as_t0',
    'test_late_status_stamp_conflict_is_polled_before_terminal_success',
    'test_missing_status_proof_cancels_goal_before_bounded_failure',
    'test_positive_response_stamp_mismatch_fails_closed',
    'test_simulation_deadline_uses_bounded_cancel',
    'test_status_matches_exact_uuid_and_projects_authoritative_t0',
    'test_status_rejects_conflicting_t0_for_exact_uuid',
    'test_status_rejects_zero_to_positive_t0_change_for_exact_uuid',
    'test_status_subscription_uses_relative_name_and_action_status_qos',
    'test_success_artifact_rejects_feedback_trace_overflow',
    'test_timeout_cancel_not_acknowledged_is_infrastructure_error',
    'test_timeout_cancel_result_missing_is_infrastructure_error',
}
source_names = set()
for source in sources:
    source_names.update(
        node.name
        for node in ast.walk(ast.parse(source.read_text(encoding='utf-8'), filename=str(source)))
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
    )
cases = []
fresh_xml_paths = []
source_xml = root / 'pytest.xml'
retained_xml = output.parent / 'robotest-missions-pytest.xml'
source_xml_sha256 = None
retained_xml_sha256 = None
source_xml_mtime_ns = None
retained_xml_matches_source = False
xml_retention_failure = None
if not source_xml.is_file() or source_xml.is_symlink():
    xml_retention_failure = f'exact mission test-result XML is not a regular file: {source_xml}'
else:
    source_xml_mtime_ns = source_xml.stat().st_mtime_ns
    if test_started_ns <= source_xml_mtime_ns <= test_finished_ns:
        fresh_xml_paths.append(str(source_xml))
    try:
        source_xml_bytes = source_xml.read_bytes()
        document = ET.fromstring(source_xml_bytes)
    except (OSError, ET.ParseError) as error:
        xml_retention_failure = f'cannot read exact mission test-result XML: {error}'
    else:
        source_xml_sha256 = hashlib.sha256(source_xml_bytes).hexdigest()
        if retained_xml.exists() or retained_xml.is_symlink():
            xml_retention_failure = f'run-local mission test-result path already exists: {retained_xml}'
        else:
            temporary = retained_xml.with_suffix(retained_xml.suffix + '.tmp')
            try:
                with temporary.open('xb') as stream:
                    stream.write(source_xml_bytes)
                temporary.replace(retained_xml)
                retained_xml_sha256 = hashlib.sha256(retained_xml.read_bytes()).hexdigest()
                retained_xml_matches_source = retained_xml_sha256 == source_xml_sha256
            except OSError as error:
                temporary.unlink(missing_ok=True)
                xml_retention_failure = f'cannot retain mission test-result XML: {error}'
        for case in document.findall('.//testcase'):
            classname = case.get('classname', '')
            name = case.get('name', '')
            passed = not any(
                case.find(tag) is not None for tag in ('failure', 'error', 'skipped')
            )
            cases.append(
                {
                    'classname': classname,
                    'name': name,
                    'passed': passed,
                    'xml': str(source_xml),
                }
            )

passing_required_names = {
    item['name']
    for item in cases
    if item['passed'] and item['name'] in expected
}
failures = []
missing_source = sorted(expected - source_names)
missing_passes = sorted(expected - passing_required_names)
if not fresh_xml_paths:
    failures.append('exact mission pytest.xml was not freshly written by this colcon test invocation')
if xml_retention_failure is not None:
    failures.append(xml_retention_failure)
if source_xml_sha256 is not None and not retained_xml_matches_source:
    failures.append('retained mission test-result XML does not match the fresh source XML')
if missing_source:
    failures.append(f'exact fixture definitions absent from source: {missing_source}')
if missing_passes:
    failures.append(f'exact fixtures did not pass in fresh current-run XML: {missing_passes}')
evidence = {
    'verdict': 'PASS' if not failures else 'FAIL',
    'failures': failures,
    'required_exact_cases': sorted(expected),
    'source_exact_cases_found': sorted(expected & source_names),
    'fresh_passing_exact_cases': sorted(expected & passing_required_names),
    'fresh_xml_paths': fresh_xml_paths,
    'retained_test_result_xml': {
        'source_path': str(source_xml),
        'source_mtime_epoch_ns': source_xml_mtime_ns,
        'source_sha256': source_xml_sha256,
        'source_is_fresh_current_run': bool(fresh_xml_paths),
        'retained_path': retained_xml.name,
        'retained_sha256': retained_xml_sha256,
        'retained_matches_source': retained_xml_matches_source,
    },
    'test_started_epoch_ns': test_started_ns,
    'test_finished_epoch_ns': test_finished_ns,
    'all_discovered_cases': cases,
    'proof_level': 'fresh current-run unit fixtures; no unsafe live clock freeze',
}
output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + '\n', encoding='utf-8')
if failures:
    raise SystemExit('; '.join(failures))
PY
}

write_action_trace_and_validate_mission() {
  python3 - "$RUN_DIR/mission-result.json" "$RUN_DIR/mission-result.csv" \
    "$RUN_DIR/action-trace.json" "$ROBOTEST_SIM_SEED" "$SCENARIO_SHA256" <<'PY'
import csv
import json
import math
import sys
from pathlib import Path

json_path = Path(sys.argv[1])
csv_path = Path(sys.argv[2])
trace_path = Path(sys.argv[3])
seed = int(sys.argv[4])
scenario_sha256 = sys.argv[5]
expected_feedback_trace_capacity = 4096
result = json.loads(json_path.read_text(encoding='utf-8'))
with csv_path.open(encoding='utf-8', newline='') as stream:
    rows = list(csv.DictReader(stream))
if len(rows) != 1:
    raise SystemExit(f'mission CSV has {len(rows)} rows, expected one')

identity = result['identity']
targets = result['targets']
measurements = result['measurements']
quality = result['quality']
verdict = result['verdict']
required = {
    'simulator_seed': (identity.get('simulator_seed'), seed),
    'mission_seed': (identity.get('mission_seed'), 42),
    'mission_sha256': (identity.get('mission_sha256'), scenario_sha256),
    'fault_seed': (identity.get('fault_seed'), None),
    'action_type': (identity.get('action_type'), 'nav2_msgs/action/FollowWaypoints'),
    'resolved_action_name': (
        identity.get('resolved_action_name'),
        '/robotest/follow_waypoints',
    ),
    'waypoint_count': (targets.get('waypoint_count'), 3),
    'number_of_loops': (targets.get('number_of_loops'), 0),
    'goal_index': (targets.get('goal_index'), 0),
    'expected_outcome': (targets.get('expected_outcome'), 'succeeded'),
    'completed_waypoint_count': (measurements.get('completed_waypoint_count'), 3),
    'missed_waypoint_count': (measurements.get('missed_waypoint_count'), 0),
    'goal_status': (measurements.get('goal_status'), 'SUCCEEDED'),
    'nav2_error_code': (measurements.get('nav2_error_code'), 0),
    'nav2_error_message': (measurements.get('nav2_error_message'), ''),
    'fault_schedule_active': (quality.get('fault_schedule_active'), False),
    'goal_accepted': (quality.get('goal_accepted'), True),
    'invalid_feedback_count': (quality.get('invalid_feedback_count'), 0),
    'cancellation_requested': (quality.get('cancellation_requested'), False),
    'cancel_acknowledged': (quality.get('cancel_acknowledged'), None),
    'terminal_result_observed': (quality.get('terminal_result_observed'), True),
    'metrics_scope': (quality.get('metrics_scope'), 'mission_action_only'),
    'scenario1_metrics_status': (
        quality.get('scenario1_metrics_status'),
        'NOT_EVALUATED_PHASE2_MISSION_ACTION_ONLY',
    ),
    'exit_code': (verdict.get('exit_code'), 0),
    'expected_outcome_met': (verdict.get('expected_outcome_met'), True),
    'phase2_action_integration_status': (
        verdict.get('phase2_action_integration_status'),
        'PASS',
    ),
    'accepted_goal_stamp_source': (
        measurements.get('accepted_goal_stamp_source'),
        'uuid_matched_action_status_goal_info',
    ),
    'scenario1_acceptance_status': (
        verdict.get('scenario1_acceptance_status'),
        'NOT_EVALUATED_PHASE2_MISSION_ACTION_ONLY',
    ),
}
failures = [f'{name}: {actual!r} != {expected!r}' for name, (actual, expected) in required.items() if actual != expected]
feedback_trace_capacity = targets.get('feedback_trace_capacity')
feedback_trace_overflow = quality.get('feedback_trace_overflow')
feedback_trace_overflow_count = quality.get('feedback_trace_overflow_count')
feedback_count = measurements.get('feedback_count')
if (
    not isinstance(feedback_trace_capacity, int)
    or isinstance(feedback_trace_capacity, bool)
    or feedback_trace_capacity != expected_feedback_trace_capacity
):
    failures.append(
        'feedback_trace_capacity is not the exact positive integer contract value 4096'
    )
if feedback_trace_overflow is not False:
    failures.append('successful baseline requires feedback_trace_overflow=false')
if (
    not isinstance(feedback_trace_overflow_count, int)
    or isinstance(feedback_trace_overflow_count, bool)
    or feedback_trace_overflow_count != 0
):
    failures.append('successful baseline requires integer feedback_trace_overflow_count=0')
if identity.get('fault_schedule_hash') is not None:
    failures.append('fault_schedule_hash is not null in no-fault baseline')
for deferred_measurement in ('actual_path_length_m', 'collision_count', 'path_efficiency'):
    if measurements.get(deferred_measurement) is not None:
        failures.append(f'{deferred_measurement} was populated before Phase 3')
if not measurements.get('accepted_goal_uuid'):
    failures.append('accepted goal UUID is absent')
accepted_uuid = measurements.get('accepted_goal_uuid')
submission_stamp = measurements.get('goal_submission_stamp_ns')
response_stamp = measurements.get('goal_response_stamp_ns')
accepted_stamp = measurements.get('accepted_goal_stamp_ns')
terminal_stamp = measurements.get('terminal_action_stamp_ns')
if (
    not isinstance(submission_stamp, int)
    or isinstance(submission_stamp, bool)
    or submission_stamp <= 0
):
    failures.append('goal_submission_stamp_ns is not a positive integer')
if (
    not isinstance(response_stamp, int)
    or isinstance(response_stamp, bool)
    or response_stamp < 0
):
    failures.append('goal_response_stamp_ns is not a non-negative integer')
if (
    not isinstance(accepted_stamp, int)
    or isinstance(accepted_stamp, bool)
    or accepted_stamp <= 0
):
    failures.append('accepted-goal simulation stamp is absent or non-positive')
if (
    isinstance(response_stamp, int)
    and not isinstance(response_stamp, bool)
    and response_stamp > 0
    and isinstance(accepted_stamp, int)
    and response_stamp != accepted_stamp
):
    failures.append('positive raw goal-response stamp does not equal authoritative status T0')


def expected_stamp_projection(stamp_ns):
    if not isinstance(stamp_ns, int) or isinstance(stamp_ns, bool) or stamp_ns < 0:
        return None
    seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
    return {'sec': seconds, 'nanosec': nanoseconds, 'nanoseconds': stamp_ns}


for field, stamp_ns in (
    ('goal_submission_stamp', submission_stamp),
    ('goal_response_stamp', response_stamp),
    ('accepted_goal_stamp', accepted_stamp),
    ('terminal_action_stamp', terminal_stamp),
):
    expected_projection = expected_stamp_projection(stamp_ns)
    if expected_projection is not None and measurements.get(field) != expected_projection:
        failures.append(f'{field} does not reconcile with its canonical nanosecond scalar')
if (
    not isinstance(terminal_stamp, int)
    or isinstance(terminal_stamp, bool)
    or not isinstance(accepted_stamp, int)
    or isinstance(accepted_stamp, bool)
):
    failures.append('terminal/accepted simulation stamps are unavailable')
elif terminal_stamp < accepted_stamp:
    failures.append('terminal action stamp precedes accepted-goal stamp')

events = result.get('events')
if not isinstance(events, list):
    failures.append('mission events are absent or not a list')
    events = []


def events_of_kind(kind):
    return [event for event in events if isinstance(event, dict) and event.get('kind') == kind]


feedback_overflow_events = events_of_kind('feedback_trace_overflow')
if feedback_overflow_events:
    failures.append(
        f'successful baseline contains {len(feedback_overflow_events)} feedback overflow event(s)'
    )


clock_ready_events = events_of_kind('simulation_clock_ready')
if len(clock_ready_events) != 1:
    failures.append(f'simulation_clock_ready event count is {len(clock_ready_events)}, expected one')
else:
    details = clock_ready_events[0].get('details', {})
    first = details.get('first_positive_stamp_ns') if isinstance(details, dict) else None
    second = details.get('second_positive_stamp_ns') if isinstance(details, dict) else None
    if not (
        isinstance(first, int)
        and not isinstance(first, bool)
        and isinstance(second, int)
        and not isinstance(second, bool)
        and 0 < first < second
    ):
        failures.append('simulation_clock_ready does not prove two advancing positive samples')

response_events = events_of_kind('goal_response_accepted')
local_response_stamp = None
if len(response_events) != 1:
    failures.append(f'goal_response_accepted event count is {len(response_events)}, expected one')
else:
    response_event = response_events[0]
    local_response_stamp = response_event.get('sim_stamp_ns')
    details = response_event.get('details', {})
    if not isinstance(details, dict):
        failures.append('goal_response_accepted event details are not an object')
    else:
        if details.get('goal_uuid') != accepted_uuid:
            failures.append('goal_response_accepted UUID does not match accepted goal UUID')
        if details.get('response_stamp_ns') != response_stamp:
            failures.append('goal_response_accepted raw stamp does not match measurement')
    if not (
        isinstance(local_response_stamp, int)
        and not isinstance(local_response_stamp, bool)
        and isinstance(submission_stamp, int)
        and isinstance(terminal_stamp, int)
        and submission_stamp <= local_response_stamp <= terminal_stamp
    ):
        failures.append(
            'client-local goal-response observation is outside submission..terminal bounds'
        )

accepted_events = events_of_kind('goal_accepted')
if len(accepted_events) != 1:
    failures.append(f'goal_accepted event count is {len(accepted_events)}, expected one')
else:
    accepted_event = accepted_events[0]
    details = accepted_event.get('details', {})
    if accepted_event.get('sim_stamp_ns') != accepted_stamp:
        failures.append('goal_accepted event does not carry authoritative status T0')
    if not isinstance(details, dict):
        failures.append('goal_accepted event details are not an object')
    else:
        if details.get('goal_uuid') != accepted_uuid:
            failures.append('goal_accepted UUID does not match accepted goal UUID')
        if details.get('evidence_source') != 'uuid_matched_action_status_goal_info':
            failures.append('goal_accepted event has the wrong T0 evidence source')
feedback_trace = measurements.get('feedback_trace')
if not isinstance(feedback_trace, list) or not feedback_trace:
    failures.append('feedback trace is empty or not a list')
else:
    if (
        not isinstance(feedback_count, int)
        or isinstance(feedback_count, bool)
        or feedback_count != len(feedback_trace)
    ):
        failures.append('feedback_count does not equal the retained feedback-trace length')
    if len(feedback_trace) > expected_feedback_trace_capacity:
        failures.append('retained feedback trace exceeds its fixed 4096-observation capacity')
    feedback_indices = []
    feedback_stamps = []
    for index, observation in enumerate(feedback_trace):
        if not isinstance(observation, dict):
            failures.append(f'feedback observation {index} is not an object')
            continue
        current_waypoint = observation.get('current_waypoint')
        simulation_stamp = observation.get('sim_stamp_ns')
        if not isinstance(current_waypoint, int) or isinstance(current_waypoint, bool):
            failures.append(f'feedback observation {index} has a non-integer waypoint')
        else:
            feedback_indices.append(current_waypoint)
        if not isinstance(simulation_stamp, int) or isinstance(simulation_stamp, bool):
            failures.append(f'feedback observation {index} has a non-integer simulation stamp')
        else:
            feedback_stamps.append(simulation_stamp)
            # Feedback timestamps are client-local latest-/clock observations. They
            # share a clock-observation domain with submission and terminal, not
            # with the action server's authoritative status-derived T0.
            if isinstance(submission_stamp, int) and isinstance(terminal_stamp, int):
                if not submission_stamp <= simulation_stamp <= terminal_stamp:
                    failures.append(
                        f'feedback observation {index} falls outside the client-local '
                        'submission..terminal interval'
                    )
    if feedback_indices != sorted(feedback_indices):
        failures.append(f'feedback waypoint indices regress: {feedback_indices}')
    ordered_unique = list(dict.fromkeys(feedback_indices))
    if ordered_unique != [0, 1, 2]:
        failures.append(
            f'feedback waypoint coverage/order {ordered_unique} does not equal [0, 1, 2]'
        )
    if feedback_stamps != sorted(feedback_stamps):
        failures.append('feedback simulation stamps regress')
if not quality.get('terminal_result_observed'):
    failures.append('terminal action result was not observed')
completion_time = measurements.get('completion_time_sim_s')
if completion_time is None:
    failures.append('simulation completion duration is absent')
elif not isinstance(completion_time, (int, float)) or isinstance(completion_time, bool):
    failures.append('simulation completion duration is not numeric')
elif not math.isfinite(completion_time) or completion_time < 0.0:
    failures.append('simulation completion duration is non-finite or negative')
else:
    if completion_time > 180.0:
        failures.append('simulation completion duration exceeds 180 s')
    if isinstance(accepted_stamp, int) and isinstance(terminal_stamp, int):
        expected_completion = (terminal_stamp - accepted_stamp) / 1_000_000_000
        if not math.isclose(completion_time, expected_completion, rel_tol=0.0, abs_tol=1e-9):
            failures.append('simulation completion duration does not reconcile with T0/terminal')
if measurements.get('mission_wall_duration_s') is None:
    failures.append('steady-wall mission duration is absent')
elif measurements['mission_wall_duration_s'] > 300.0:
    failures.append('steady-wall mission duration exceeds 300 s')

# Reuse the package's normative projection to prove exact JSON/CSV equivalence.
from robotest_missions.artifacts import flatten_result  # noqa: E402

expected_row = flatten_result(result)
if rows[0] != expected_row:
    failures.append('mission CSV does not equal the canonical JSON projection')

trace = {
    'run_id': identity.get('run_id'),
    'mission_sha256': identity.get('mission_sha256'),
    'action_name': identity.get('resolved_action_name'),
    'accepted_goal_uuid': measurements.get('accepted_goal_uuid'),
    'goal_submission_stamp': measurements.get('goal_submission_stamp'),
    'goal_submission_stamp_ns': submission_stamp,
    'goal_response_stamp': measurements.get('goal_response_stamp'),
    'goal_response_stamp_ns': response_stamp,
    'goal_response_stamp_semantics': (
        'UNAVAILABLE_ZERO_JAZZY_RCLCPP'
        if response_stamp == 0
        else 'MATCHED_AUTHORITATIVE_STATUS_T0'
    ),
    'accepted_goal_stamp': measurements.get('accepted_goal_stamp'),
    'accepted_goal_stamp_ns': accepted_stamp,
    'accepted_goal_stamp_source': measurements.get('accepted_goal_stamp_source'),
    'terminal_action_stamp': measurements.get('terminal_action_stamp'),
    'terminal_action_stamp_ns': terminal_stamp,
    'feedback_count': feedback_count,
    'feedback_trace_capacity': feedback_trace_capacity,
    'feedback_trace_overflow': feedback_trace_overflow,
    'feedback_trace_overflow_count': feedback_trace_overflow_count,
    'feedback_trace': measurements.get('feedback_trace', []),
    'feedback_timestamp_semantics': 'client_latest_clock_at_callback',
    'events': events,
    'terminal_status': measurements.get('goal_status'),
    'missed_waypoints': measurements.get('missed_waypoints', []),
    'failures': failures,
    'verdict': 'PASS' if not failures else 'FAIL',
}
trace_path.write_text(json.dumps(trace, indent=2, sort_keys=True) + '\n', encoding='utf-8')
if failures:
    raise SystemExit('; '.join(failures))
PY
}

write_mission_runtime_correlation() {
  python3 - "$RUN_DIR/mission-result.json" "$RUN_DIR/runtime-probe.json" \
    "$RUN_DIR/mission-runtime-correlation.json" <<'PY'
import json
import math
import sys
from pathlib import Path

mission = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
probe = json.loads(Path(sys.argv[2]).read_text(encoding='utf-8'))
output = Path(sys.argv[3])
measurements = mission.get('measurements', {})
accepted_uuid = measurements.get('accepted_goal_uuid')
submission = measurements.get('goal_submission_stamp_ns')
response = measurements.get('goal_response_stamp_ns')
accepted = measurements.get('accepted_goal_stamp_ns')
accepted_source = measurements.get('accepted_goal_stamp_source')
terminal = measurements.get('terminal_action_stamp_ns')
failures = []
filter_contract = probe.get('mission_interval_filtering_contract')
expected_filter_contract = {
    'lower_bound_field': 'accepted_goal_stamp_ns',
    'upper_bound_field': 'terminal_action_stamp_ns',
    'bounds': 'inclusive',
    'command_sample_stamp_field': 'simulation_stamp_ns',
    'ground_truth_sample_stamp_field': 'stamp_ns',
}
if not isinstance(filter_contract, dict) or any(
    filter_contract.get(key) != value for key, value in expected_filter_contract.items()
):
    failures.append('runtime probe mission-interval filtering contract is absent or incompatible')
if not isinstance(accepted, int) or isinstance(accepted, bool) or accepted <= 0:
    failures.append('accepted_goal_stamp_ns is not a positive integer')
if accepted_source != 'uuid_matched_action_status_goal_info':
    failures.append('accepted_goal_stamp_source is not UUID-matched action status GoalInfo')
if not isinstance(accepted_uuid, str) or not accepted_uuid:
    failures.append('accepted_goal_uuid is absent')
if not isinstance(submission, int) or isinstance(submission, bool) or submission <= 0:
    failures.append('goal_submission_stamp_ns is not a positive integer')
if not isinstance(response, int) or isinstance(response, bool) or response < 0:
    failures.append('goal_response_stamp_ns is not a non-negative integer')
elif response > 0 and isinstance(accepted, int) and response != accepted:
    failures.append('positive raw goal-response stamp does not equal authoritative status T0')
if not isinstance(terminal, int) or isinstance(terminal, bool):
    failures.append('terminal_action_stamp_ns is not an integer')
if isinstance(accepted, int) and isinstance(terminal, int) and terminal < accepted:
    failures.append('terminal_action_stamp_ns precedes accepted_goal_stamp_ns')


def in_interval(value):
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and isinstance(accepted, int)
        and isinstance(terminal, int)
        and accepted <= value <= terminal
    )


command_evidence = {}
for topic in (
    '/robotest/cmd_vel_nav',
    '/robotest/cmd_vel_smoothed',
    '/robotest/cmd_vel',
):
    summary = probe.get('command_trace', {}).get(topic)
    if not isinstance(summary, dict):
        failures.append(f'{topic} command trace summary is absent')
        continue
    samples = summary.get('samples')
    if not isinstance(samples, list):
        failures.append(f'{topic} command samples are absent')
        continue
    if summary.get('dropped') != 0:
        failures.append(f'{topic} command trace is incomplete because samples were dropped')
    interval_samples = [
        sample
        for sample in samples
        if isinstance(sample, dict) and in_interval(sample.get('simulation_stamp_ns'))
    ]
    nonzero = []
    for sample in interval_samples:
        values = [
            sample.get('linear_x_mps'),
            sample.get('linear_y_mps'),
            sample.get('angular_z_radps'),
        ]
        if not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
            for value in values
        ):
            failures.append(f'{topic} has a non-finite or non-numeric interval sample')
            continue
        if abs(values[0]) > 1.0e-4 or abs(values[1]) > 1.0e-4 or abs(values[2]) > 1.0e-4:
            nonzero.append(sample)
    command_evidence[topic] = {
        'interval_nonzero_sample_count': len(nonzero),
        'interval_sample_count': len(interval_samples),
        'total_sample_count': len(samples),
    }
    if not interval_samples:
        failures.append(f'{topic} has no samples in the accepted-goal interval')
    elif not nonzero:
        failures.append(f'{topic} has no nonzero motion command in the accepted-goal interval')

truth_summary = probe.get('ground_truth_motion_liveness')
truth_samples = truth_summary.get('samples') if isinstance(truth_summary, dict) else None
interval_truth = []
if isinstance(truth_summary, dict) and truth_summary.get('dropped') != 0:
    failures.append('ground-truth trace is incomplete because samples were dropped')
if not isinstance(truth_samples, list):
    failures.append('ground-truth samples are absent')
else:
    interval_truth = [
        sample
        for sample in truth_samples
        if isinstance(sample, dict) and in_interval(sample.get('stamp_ns'))
    ]
truth_span = None
if len(interval_truth) < 2:
    failures.append('fewer than two ground-truth samples fall in the accepted-goal interval')
else:
    coordinates = [(sample.get('x_m'), sample.get('y_m')) for sample in interval_truth]
    if not all(
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
        for pair in coordinates
        for value in pair
    ):
        failures.append('ground-truth interval contains non-finite or non-numeric positions')
    else:
        xs = [pair[0] for pair in coordinates]
        ys = [pair[1] for pair in coordinates]
        truth_span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if truth_span <= 0.10:
            failures.append(
                f'ground-truth interval motion span {truth_span} is not greater than 0.10 m'
            )

evidence = {
    'accepted_goal_uuid': accepted_uuid,
    'accepted_goal_stamp_ns': accepted,
    'accepted_goal_stamp_source': accepted_source,
    'command_trace': command_evidence,
    'failures': failures,
    'goal_response_stamp_ns': response,
    'goal_response_stamp_semantics': (
        'UNAVAILABLE_ZERO_JAZZY_RCLCPP'
        if response == 0
        else 'MATCHED_AUTHORITATIVE_STATUS_T0'
    ),
    'goal_submission_stamp_ns': submission,
    'ground_truth_interval_sample_count': len(interval_truth),
    'ground_truth_interval_span_m': truth_span,
    'interval_semantics': (
        'inclusive authoritative UUID-matched action-status T0'
        '..client terminal-result observation'
    ),
    'terminal_action_stamp_ns': terminal,
    'verdict': 'PASS' if not failures else 'FAIL',
}
output.write_text(
    json.dumps(evidence, indent=2, sort_keys=True, allow_nan=False) + '\n',
    encoding='utf-8',
)
if failures:
    raise SystemExit('; '.join(failures))
PY
}

write_process_cleanup_evidence() {
  python3 - "$OWNED_GROUPS_FILE" "$RUN_DIR/process-cleanup.json" <<'PY'
import json
import subprocess
import sys
from pathlib import Path

groups = []
for line in Path(sys.argv[1]).read_text(encoding='utf-8').splitlines():
    role, pid, pgid = line.split()
    groups.append({'role': role, 'leader_pid': int(pid), 'pgid': int(pgid)})
result = subprocess.run(
    ['ps', '-eo', 'pid=,pgid=,ppid=,stat=,comm='],
    capture_output=True,
    check=True,
    text=True,
)
pgids = {item['pgid'] for item in groups}
remaining = []
for line in result.stdout.splitlines():
    fields = line.split(None, 4)
    if len(fields) == 5 and int(fields[1]) in pgids:
        remaining.append(
            {
                'pid': int(fields[0]),
                'pgid': int(fields[1]),
                'ppid': int(fields[2]),
                'stat': fields[3],
                'command': fields[4],
            }
        )
evidence = {
    'owned_groups': groups,
    'remaining_processes': remaining,
    'verdict': 'PASS' if not remaining else 'FAIL',
    'scope': 'exact recorded process groups only; no pkill/killall/global cleanup',
}
Path(sys.argv[2]).write_text(
    json.dumps(evidence, indent=2, sort_keys=True) + '\n', encoding='utf-8'
)
if remaining:
    raise SystemExit(f'owned processes remain after cleanup: {remaining}')
PY
}

write_final_metadata() {
  local exit_code="$1"
  local checksum_status="$2"
  python3 - "$RUN_DIR" "$RUN_ID" "$RUN_CREATED_UTC" "$GIT_SHA" "$GIT_DIRTY" \
    "${TARGET_SET_SHA256:-unavailable}" "${SCENARIO_SHA256:-unavailable}" \
    "$ROBOTEST_SIM_SEED" "$ROBOTEST_CPUSET" "${ROS_DOMAIN_ID:-}" \
    "${GZ_PARTITION:-}" "${MAX_RSS_KIB:-}" "$exit_code" "$checksum_status" \
    "$MISSION_STATUS" "$PROBE_STATUS" "$LAUNCH_STARTED_UTC" "$LAUNCH_STOPPED_UTC" \
    "$SAMPLING_STARTED_UTC" "$SAMPLING_STOPPED_UTC" "$TEE_STATUS" <<'PY'
import csv
import json
import sys
from pathlib import Path

run_dir = Path(sys.argv[1])


def read_json(name):
    path = run_dir / name
    return json.loads(path.read_text(encoding='utf-8')) if path.is_file() else None


def optional_int(value):
    return int(value) if value else None


exit_code = int(sys.argv[13])
checksum_status = int(sys.argv[14])
probe = read_json('runtime-probe.json') or {}
mission = read_json('mission-result.json') or {}
mission_measurements = mission.get('measurements', {})
startup_gate = read_json('lifecycle-startup-gate.json') or {}
startup_result = startup_gate.get('startup_result')
launch_log_gate = read_json('launch-log-signature-gate.json') or {}
rtf = probe.get('rtf', {})
fixture_verdict = (read_json('fixture-proof.json') or {}).get('verdict', 'NOT_RUN')
lifecycle_verdict = (read_json('lifecycle-states.json') or {}).get('verdict', 'NOT_RUN')
mission_verdict = mission.get('verdict', {}).get('phase2_action_integration_status', 'NOT_RUN')
probe_verdict = probe.get('verdict', 'NOT_RUN')
correlation_verdict = (read_json('mission-runtime-correlation.json') or {}).get(
    'verdict', 'NOT_RUN'
)
attempted_level = 'L3 bounded local Phase 2 headless action integration'
if (
    mission_verdict == 'PASS'
    and probe_verdict == 'PASS'
    and correlation_verdict == 'PASS'
    and launch_log_gate.get('verdict') == 'PASS'
):
    achieved_level = 'L3 bounded local Phase 2 headless action integration'
elif lifecycle_verdict == 'PASS':
    achieved_level = (
        'L2 build and launch smoke; mission action passed but runtime evidence gate failed'
        if mission_verdict == 'PASS'
        else 'L2 build and launch smoke; Phase 2 action integration not achieved'
    )
elif fixture_verdict == 'PASS':
    achieved_level = 'L1 Phase 2 unit fixtures; runtime integration not achieved'
else:
    achieved_level = 'L0 static/evidence capture; higher verification not achieved'
result = {
    'identity': {
        'run_id': sys.argv[2],
        'created_utc': sys.argv[3],
        'git_sha': sys.argv[4],
        'git_dirty': sys.argv[5] == 'true',
        'verification_level': achieved_level,
        'verification_level_achieved': achieved_level,
        'verification_level_attempted': attempted_level,
        'ros_domain_id': optional_int(sys.argv[10]),
        'gz_partition': sys.argv[11] or None,
    },
    'targets': {
        'target_set_sha256': sys.argv[6],
        'scenario_sha256': sys.argv[7],
        'simulator_seed': int(sys.argv[8]),
        'runtime_cpuset': sys.argv[9],
        'feedback_trace_capacity': mission.get('targets', {}).get(
            'feedback_trace_capacity'
        ),
        'max_process_group_rss_kib': 6 * 1024 * 1024,
        'minimum_rtf_median': 0.80,
        'minimum_rtf_p5': 0.50,
    },
    'measurements': {
        'peak_owned_process_groups_rss_kib': optional_int(sys.argv[12]),
        'rtf_median': rtf.get('median'),
        'rtf_p5': rtf.get('p5'),
        'mission_action_status': mission_measurements.get('goal_status'),
        'completed_waypoint_count': mission_measurements.get('completed_waypoint_count'),
        'feedback_count': mission_measurements.get('feedback_count'),
        'accepted_goal_uuid': mission_measurements.get('accepted_goal_uuid'),
        'goal_submission_stamp_ns': mission_measurements.get('goal_submission_stamp_ns'),
        'goal_response_stamp_ns': mission_measurements.get('goal_response_stamp_ns'),
        'accepted_goal_stamp_ns': mission_measurements.get('accepted_goal_stamp_ns'),
        'accepted_goal_stamp_source': mission_measurements.get('accepted_goal_stamp_source'),
        'terminal_action_stamp_ns': mission_measurements.get('terminal_action_stamp_ns'),
        'collision_count': None,
        'actual_path_length_m': None,
        'path_efficiency': None,
    },
    'events': [],
    'quality': {
        'mission_process_exit_code': optional_int(sys.argv[15]),
        'runtime_probe_process_exit_code': optional_int(sys.argv[16]),
        'runtime_probe_verdict': probe_verdict,
        'startup_gate_verdict': startup_gate.get('verdict', 'NOT_RUN'),
        'launch_log_signature_gate': launch_log_gate.get('verdict', 'NOT_RUN'),
        'fixture_proof': fixture_verdict,
        'feedback_trace_overflow': mission.get('quality', {}).get(
            'feedback_trace_overflow'
        ),
        'feedback_trace_overflow_count': mission.get('quality', {}).get(
            'feedback_trace_overflow_count'
        ),
        'source_install_binding': (
            read_json('installed-source-binding.json') or {}
        ).get('verdict', 'NOT_RUN'),
        'source_config_mutation': (
            read_json('source-config-mutation.json') or {}
        ).get('verdict', 'NOT_RUN'),
        'mission_runtime_correlation': correlation_verdict,
        'process_cleanup': (read_json('process-cleanup.json') or {}).get(
            'verdict', 'NOT_RUN'
        ),
        'checksum_validation_exit_code': checksum_status,
        'fault_schedule': 'EMPTY_NO_FAULT_BASELINE',
        'fault_arming': 'NOT_IMPLEMENTED_NOT_EXERCISED',
        'scenario1_metrics_status': 'DEFERRED_TO_PHASE3',
        'launch_started_utc': sys.argv[17] or None,
        'launch_stopped_utc': sys.argv[18] or None,
        'resource_sampling_started_utc': sys.argv[19] or None,
        'resource_sampling_stopped_utc': sys.argv[20] or None,
        'tee_exit_code': optional_int(sys.argv[21]),
    },
    'verdict': {
        'automated_status': 'PASS' if exit_code == 0 and checksum_status == 0 else 'FAIL',
        'exit_code': exit_code,
        'phase2_action_integration_status': (
            mission_verdict
        ),
        'scenario1_acceptance_status': 'NOT_EVALUATED_PHASE3',
    },
}
(run_dir / 'run-result.json').write_text(
    json.dumps(result, separators=(',', ':'), sort_keys=True) + '\n', encoding='utf-8'
)
row = {
    'run_id': result['identity']['run_id'],
    'created_utc': result['identity']['created_utc'],
    'git_sha': result['identity']['git_sha'],
    'git_dirty': str(result['identity']['git_dirty']).lower(),
    'verification_level_attempted': result['identity']['verification_level_attempted'],
    'verification_level_achieved': result['identity']['verification_level_achieved'],
    'target_set_sha256': result['targets']['target_set_sha256'],
    'scenario_sha256': result['targets']['scenario_sha256'],
    'simulator_seed': result['targets']['simulator_seed'],
    'runtime_cpuset': result['targets']['runtime_cpuset'],
    'feedback_trace_capacity': result['targets']['feedback_trace_capacity'],
    'peak_owned_process_groups_rss_kib': result['measurements'][
        'peak_owned_process_groups_rss_kib'
    ],
    'rtf_median': result['measurements']['rtf_median'],
    'rtf_p5': result['measurements']['rtf_p5'],
    'mission_action_status': result['measurements']['mission_action_status'],
    'completed_waypoint_count': result['measurements']['completed_waypoint_count'],
    'feedback_count': result['measurements']['feedback_count'],
    'accepted_goal_uuid': result['measurements']['accepted_goal_uuid'],
    'goal_submission_stamp_ns': result['measurements']['goal_submission_stamp_ns'],
    'goal_response_stamp_ns': result['measurements']['goal_response_stamp_ns'],
    'accepted_goal_stamp_ns': result['measurements']['accepted_goal_stamp_ns'],
    'accepted_goal_stamp_source': result['measurements']['accepted_goal_stamp_source'],
    'terminal_action_stamp_ns': result['measurements']['terminal_action_stamp_ns'],
    'runtime_probe_verdict': result['quality']['runtime_probe_verdict'],
    'startup_gate_verdict': result['quality']['startup_gate_verdict'],
    'launch_log_signature_gate': result['quality']['launch_log_signature_gate'],
    'fixture_proof': result['quality']['fixture_proof'],
    'feedback_trace_overflow': str(result['quality']['feedback_trace_overflow']).lower(),
    'feedback_trace_overflow_count': result['quality']['feedback_trace_overflow_count'],
    'source_install_binding': result['quality']['source_install_binding'],
    'source_config_mutation': result['quality']['source_config_mutation'],
    'mission_runtime_correlation': result['quality']['mission_runtime_correlation'],
    'process_cleanup': result['quality']['process_cleanup'],
    'fault_arming': result['quality']['fault_arming'],
    'scenario1_metrics_status': result['quality']['scenario1_metrics_status'],
    'automated_status': result['verdict']['automated_status'],
    'exit_code': result['verdict']['exit_code'],
}
with (run_dir / 'run-result.csv').open('w', encoding='utf-8', newline='') as stream:
    writer = csv.DictWriter(stream, fieldnames=list(row))
    writer.writeheader()
    writer.writerow(row)
with (run_dir / 'run-result.csv').open(encoding='utf-8', newline='') as stream:
    rows = list(csv.DictReader(stream))
expected_row = {key: '' if value is None else str(value) for key, value in row.items()}
if rows != [expected_row]:
    raise SystemExit('Phase 2 run-result CSV does not match canonical JSON projection')

provenance = {
    'identity': result['identity'],
    'effective_parameters': {
        'build_parallel_workers': 4,
        'headless': True,
        'lifecycle_discovery_grace_sec': 4.0,
        'lifecycle_response_timeout_sec': 60.0,
        'lifecycle_service_timeout_sec': 20.0,
        'startup_gate_wall_timeout_sec': 110.0,
        'render_sensors': True,
        'robot_namespace': '/robotest',
        'runtime_cpuset': sys.argv[9],
        'rviz': False,
        'simulator_seed': int(sys.argv[8]),
    },
    'git': {
        'sha': sys.argv[4],
        'dirty_at_start': sys.argv[5] == 'true',
        'status_porcelain_path': 'git-status-start.txt',
    },
    'scenario': {
        'name': 'phase2_baseline',
        'scenario_file': 'scenarios/phase2_baseline.yaml',
        'scenario_sha256': sys.argv[7],
        'mission_seed': mission.get('identity', {}).get('mission_seed'),
        'simulator_seed': int(sys.argv[8]),
        'fault_schedule_hash': None,
        'fault_seed': None,
        'expected_outcome': 'succeeded',
        'accepted_goal_uuid': mission_measurements.get('accepted_goal_uuid'),
        'goal_submission_stamp_ns': mission_measurements.get('goal_submission_stamp_ns'),
        'goal_response_stamp_ns': mission_measurements.get('goal_response_stamp_ns'),
        'accepted_goal_stamp_ns': mission_measurements.get('accepted_goal_stamp_ns'),
        'accepted_goal_stamp_source': mission_measurements.get(
            'accepted_goal_stamp_source'
        ),
        'terminal_action_stamp_ns': mission_measurements.get('terminal_action_stamp_ns'),
        'feedback_trace_capacity': mission.get('targets', {}).get(
            'feedback_trace_capacity'
        ),
        'feedback_count': mission_measurements.get('feedback_count'),
        'feedback_trace_overflow': mission.get('quality', {}).get(
            'feedback_trace_overflow'
        ),
        'feedback_trace_overflow_count': mission.get('quality', {}).get(
            'feedback_trace_overflow_count'
        ),
    },
    'target_set': {
        'path': 'docs/testing/acceptance-criteria.md',
        'revision': 'Frozen Phase 0 target set, revision 1',
        'sha256': sys.argv[6],
    },
    'command': read_json('command.json'),
    'child_commands': read_json('child-commands.json'),
    'source_config': read_json('source-config-hashes.json'),
    'source_config_end': read_json('source-config-hashes-end.json'),
    'source_config_mutation': read_json('source-config-mutation.json'),
    'installed_source_binding': read_json('installed-source-binding.json'),
    'startup_acceptance': {
        'result': startup_result,
        'gate': startup_gate,
    },
    'launch_log_signature_gate': launch_log_gate,
    'versions': read_json('versions.json'),
    'artifacts': {
        'canonical_json': 'run-result.json',
        'matching_csv': 'run-result.csv',
        'mission_json': 'mission-result.json',
        'mission_csv': 'mission-result.csv',
        'action_trace': 'action-trace.json',
        'mission_runtime_correlation': 'mission-runtime-correlation.json',
        'mission_fixture_proof': 'fixture-proof.json',
        'mission_test_result_xml': 'robotest-missions-pytest.xml',
        'graph_readiness': 'graph-readiness.json',
        'graph_node_identities': 'nodes-ready.txt',
        'action_ownership_during_mission': 'action-ownership-during-mission.json',
        'runtime_qos_ownership_isolation': 'runtime-probe.json',
        'contact_gate_runtime_ready': 'contact-gate-runtime-ready.json',
        'contact_gate_runtime_final': 'contact-gate-runtime-final.json',
        'lifecycle_startup_result': (
            'lifecycle-startup-result.json'
            if (run_dir / 'lifecycle-startup-result.json').is_file()
            else None
        ),
        'lifecycle_startup_gate': (
            'lifecycle-startup-gate.json'
            if (run_dir / 'lifecycle-startup-gate.json').is_file()
            else None
        ),
        'lifecycle_startup_gate_ordering': (
            'startup-gate-ordering.json'
            if (run_dir / 'startup-gate-ordering.json').is_file()
            else None
        ),
        'launch_log_signature_gate': (
            'launch-log-signature-gate.json'
            if (run_dir / 'launch-log-signature-gate.json').is_file()
            else None
        ),
        'renderer_log': 'ogre2-phase2-launch.log',
        'renderer_freshness': 'ogre2-phase2-launch-evidence.json',
        'checksum_manifest': 'SHA256SUMS',
        'checksum_validation': 'checksum-validation.txt',
    },
}
(run_dir / 'provenance.json').write_text(
    json.dumps(provenance, indent=2, sort_keys=True) + '\n', encoding='utf-8'
)
summary = {
    'run_id': sys.argv[2],
    'created_utc': sys.argv[3],
    'evidence_finalized_utc': __import__('datetime').datetime.now(
        __import__('datetime').timezone.utc
    ).strftime('%Y-%m-%dT%H:%M:%SZ'),
    'exit_code': exit_code,
    'checksum_validation_exit_code': checksum_status,
    'owned_process_groups': read_json('process-cleanup.json'),
}
(run_dir / 'summary.json').write_text(
    json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8'
)
PY
}

generate_checksum_manifest() {
  find "$RUN_DIR" -maxdepth 1 -type f \
    ! -name SHA256SUMS \
    ! -name checksum-validation.txt \
    -print0 | sort -z | xargs -0 -r sha256sum > "$RUN_DIR/SHA256SUMS"
}

validate_checksum_manifest() {
  local temporary=""
  local status=0
  temporary="$(mktemp)" || return 1
  if sha256sum -c "$RUN_DIR/SHA256SUMS" > "$temporary" 2>&1; then
    status=0
  else
    status=$?
  fi
  mv -- "$temporary" "$RUN_DIR/checksum-validation.txt" || return 1
  return "$status"
}

cleanup() {
  local execution_status=$?
  local final_status=0
  local checksum_status=0
  local metadata_status=0
  local cleanup_status=0
  local provenance_status=0
  (( FINALIZED == 0 )) || return
  FINALIZED=1
  trap - EXIT INT TERM
  set +e

  stop_mission || cleanup_status=1
  stop_probe || cleanup_status=1
  stop_launch || cleanup_status=1
  sample_owned_groups || true
  stop_sampler
  capture_phase2_ogre2_log
  write_process_cleanup_evidence || cleanup_status=1
  write_source_config_mutation_evidence || provenance_status=1
  if [[ -f "$RUN_DIR/resources.csv" ]]; then
    MAX_RSS_KIB="$(awk -F, 'NR>1 {sum[$1]+=$6} END {max=0; for (t in sum) if (sum[t]>max) max=sum[t]; print max+0}' "$RUN_DIR/resources.csv")"
  fi
  close_verify_log

  final_status=$execution_status
  [[ "$TEE_STATUS" == "0" ]] || final_status=1
  (( cleanup_status == 0 )) || final_status=1
  (( provenance_status == 0 )) || final_status=1
  if [[ -n "$MAX_RSS_KIB" ]] && (( MAX_RSS_KIB > 6 * 1024 * 1024 )); then
    final_status=1
  fi
  write_final_metadata "$final_status" 0
  metadata_status=$?
  (( metadata_status == 0 )) || final_status=1
  generate_checksum_manifest || final_status=1
  if validate_checksum_manifest; then checksum_status=0; else checksum_status=$?; final_status=1; fi

  if (( final_status != execution_status || checksum_status != 0 || metadata_status != 0 )); then
    write_final_metadata "$final_status" "$checksum_status" || final_status=1
    generate_checksum_manifest || final_status=1
    validate_checksum_manifest || final_status=1
  fi

  if (( final_status == 0 )); then
    printf 'Phase 2 verification PASS: %s\n' "$RUN_DIR"
  else
    printf 'Phase 2 verification FAIL (status %d): %s\n' "$final_status" "$RUN_DIR" >&2
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

TARGET_SET="$WORKSPACE/docs/testing/acceptance-criteria.md"
SCENARIO="$WORKSPACE/scenarios/phase2_baseline.yaml"
NAV_LAUNCH="$WORKSPACE/src/robotest_navigation/launch/phase2.launch.py"
STARTUP_GATE="$WORKSPACE/tests/phase2_startup_gate.py"
PROBE="$WORKSPACE/tests/phase2_runtime_probe.py"
LIFECYCLE_PROBE="$WORKSPACE/tests/phase2_lifecycle_probe.py"
PARAMETER_PROBE="$WORKSPACE/tests/phase2_parameter_probe.py"
GRAPH_PROBE="$WORKSPACE/tests/phase2_graph_probe.py"
for required in \
  "$TARGET_SET" "$SCENARIO" "$NAV_LAUNCH" "$STARTUP_GATE" "$PROBE" "$LIFECYCLE_PROBE" \
  "$PARAMETER_PROBE" "$GRAPH_PROBE"; do
  [[ -f "$required" && ! -L "$required" ]] || {
    echo "missing regular Phase 2 input: $required" >&2
    exit 2
  }
done
TARGET_SET_SHA256="$(sha256sum "$TARGET_SET" | awk '{print $1}')"
SCENARIO_SHA256="$(sha256sum "$SCENARIO" | awk '{print $1}')"

echo "Phase 2 run: $RUN_ID"
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
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
write_version_evidence

export CMAKE_BUILD_PARALLEL_LEVEL=4
export MAKEFLAGS=-j4
export RCUTILS_COLORIZED_OUTPUT=0
cd "$WORKSPACE"

timeout --signal=TERM --kill-after=20s 1200s \
  colcon build --symlink-install --parallel-workers 4 \
  --packages-up-to robotest_navigation robotest_missions \
  --event-handlers console_direct+ 2>&1 | tee "$RUN_DIR/colcon-build.txt"

set +u
# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"
set -u

TEST_STARTED_EPOCH_NS="$(date +%s%N)"
timeout --signal=TERM --kill-after=10s 1500s \
  colcon test --parallel-workers 4 \
  --packages-select \
    robotest_interfaces robotest_description robotest_faults robotest_sim \
    robotest_navigation robotest_missions \
  --event-handlers console_direct+ 2>&1 | tee "$RUN_DIR/colcon-test.txt"
colcon test-result --verbose | tee "$RUN_DIR/colcon-test-result.txt"
TEST_FINISHED_EPOCH_NS="$(date +%s%N)"

timeout 20s shellcheck "$SCRIPT_PATH" | tee "$RUN_DIR/shellcheck-phase2.txt"
timeout 20s python3 -m py_compile "$STARTUP_GATE"
timeout 20s python3 -m py_compile "$PROBE"
timeout 20s python3 -m py_compile "$LIFECYCLE_PROBE"
timeout 20s python3 -m py_compile "$PARAMETER_PROBE"
timeout 20s python3 -m py_compile "$GRAPH_PROBE"
timeout 20s python3 "$PROBE" --self-test | tee "$RUN_DIR/probe-self-test.txt"
timeout 20s python3 "$STARTUP_GATE" --self-test \
  | tee "$RUN_DIR/startup-gate-self-test.txt"
timeout 20s python3 "$PARAMETER_PROBE" --self-test \
  | tee "$RUN_DIR/parameter-probe-self-test.txt"
timeout --signal=TERM --kill-after=3s 15s \
  python3 "$PARAMETER_PROBE" --live-smoke-test \
  | tee "$RUN_DIR/parameter-probe-live-smoke.txt"
timeout 20s python3 "$GRAPH_PROBE" --self-test \
  | tee "$RUN_DIR/graph-probe-self-test.txt"
timeout --signal=TERM --kill-after=3s 15s \
  python3 "$GRAPH_PROBE" --live-smoke-test \
  | tee "$RUN_DIR/graph-probe-live-smoke.txt"
timeout --signal=TERM --kill-after=3s 15s \
  python3 "$GRAPH_PROBE" --live-duplicate-node-test \
  | tee "$RUN_DIR/graph-probe-duplicate-node-test.txt"
write_fixture_proof
write_installed_source_binding
write_startup_gate_ordering_evidence

timeout 30s ros2 launch robotest_navigation phase2.launch.py \
  seed:="$ROBOTEST_SIM_SEED" --show-args | tee "$RUN_DIR/launch-arguments.txt"
grep -Fq "seed" "$RUN_DIR/launch-arguments.txt"
grep -Fq "navigation_start_delay_sec" "$RUN_DIR/launch-arguments.txt"
grep -Fq "lifecycle_discovery_grace_sec" "$RUN_DIR/launch-arguments.txt"
grep -Fq "lifecycle_service_timeout_sec" "$RUN_DIR/launch-arguments.txt"
grep -Fq "lifecycle_response_timeout_sec" "$RUN_DIR/launch-arguments.txt"
grep -Fq "lifecycle_startup_result_path" "$RUN_DIR/launch-arguments.txt"

export ROS_DOMAIN_ID="$((150 + ($$ % 70)))"
export GZ_PARTITION="robotest_phase2_${RUN_ID//[^A-Za-z0-9_]/_}"
printf 'ROS_DOMAIN_ID=%s\nGZ_PARTITION=%s\nROBOTEST_SIM_SEED=%s\n' \
  "$ROS_DOMAIN_ID" "$GZ_PARTITION" "$ROBOTEST_SIM_SEED" \
  | tee "$RUN_DIR/isolation.txt"

LAUNCH_STARTED_UTC="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
LAUNCH_STARTED_EPOCH_NS="$(date +%s%N)"
STARTUP_RESULT="$RUN_DIR/lifecycle-startup-result.json"
STARTUP_GATE_EVIDENCE="$RUN_DIR/lifecycle-startup-gate.json"
if [[ -e "$STARTUP_RESULT" || -L "$STARTUP_RESULT" \
  || -e "$STARTUP_GATE_EVIDENCE" || -L "$STARTUP_GATE_EVIDENCE" ]]; then
  echo "startup acceptance evidence path unexpectedly exists" >&2
  exit 2
fi
LAUNCH_COMMAND=(
  timeout --signal=TERM --kill-after=10s 430s
  ros2 launch robotest_navigation phase2.launch.py
  namespace:=robotest seed:="$ROBOTEST_SIM_SEED"
  headless:=true render_sensors:=true rviz:=false autostart:=true
  navigation_start_delay_sec:=7.0
  lifecycle_discovery_grace_sec:=4.0 lifecycle_service_timeout_sec:=20.0
  lifecycle_response_timeout_sec:=60.0
  lifecycle_startup_result_path:="$STARTUP_RESULT"
)
setsid "${LAUNCH_COMMAND[@]}" \
  > "$RUN_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID="$(ps -o pgid= -p "$LAUNCH_PID" | tr -d ' ')"
record_owned_group launch "$LAUNCH_PID" "$LAUNCH_PGID"
record_child_command launch "$LAUNCH_PID" "$RUN_DIR/launch.log" \
  "$RUN_DIR/launch.log" "${LAUNCH_COMMAND[@]}"
printf 'launch_pid=%s\nlaunch_pgid=%s\nlaunch_started_utc=%s\n' \
  "$LAUNCH_PID" "$LAUNCH_PGID" "$LAUNCH_STARTED_UTC" \
  > "$RUN_DIR/launch-process-group.txt"
# PHASE2_START_SAMPLER_ORDER
start_sampler

# PHASE2_STARTUP_GATE_ORDER
timeout --signal=TERM --kill-after=3s 120s \
  python3 "$STARTUP_GATE" \
    --result "$STARTUP_RESULT" \
    --output "$STARTUP_GATE_EVIDENCE" \
    --watch-pid "$LAUNCH_PID" \
    --wall-timeout 110 \
    --expected-service-name /robotest/lifecycle_manager_navigation/manage_nodes \
    --expected-discovery-grace-sec 4.0 \
    --expected-service-timeout-sec 20.0 \
    --expected-response-timeout-sec 60.0 \
  | tee "$RUN_DIR/lifecycle-startup-gate.log"

# PHASE2_LIFECYCLE_PROBE_ORDER
lifecycle_nodes=(
  map_server amcl planner_server controller_server behavior_server bt_navigator
  waypoint_follower velocity_smoother collision_monitor
)
lifecycle_arguments=()
for short_name in "${lifecycle_nodes[@]}"; do
  lifecycle_arguments+=(--node "$short_name")
done
timeout --signal=TERM --kill-after=5s 120s \
  python3 "$LIFECYCLE_PROBE" \
    --namespace /robotest \
    --output "$RUN_DIR/lifecycle-states.json" \
    --text-dir "$RUN_DIR" \
    --wall-timeout 110 \
    --watch-pid "$LAUNCH_PID" \
    "${lifecycle_arguments[@]}" \
  | tee "$RUN_DIR/lifecycle-probe.log"

graph_topic_contracts=(
  /clock=rosgraph_msgs/msg/Clock
  /robotest/map=nav_msgs/msg/OccupancyGrid
  /robotest/raw/scan=sensor_msgs/msg/LaserScan
  /robotest/scan=sensor_msgs/msg/LaserScan
  /robotest/raw/odom=nav_msgs/msg/Odometry
  /robotest/odom=nav_msgs/msg/Odometry
  /robotest/raw/imu=sensor_msgs/msg/Imu
  /robotest/imu=sensor_msgs/msg/Imu
  /robotest/internal/raw_contacts=ros_gz_interfaces/msg/Contacts
  /robotest/navigation/plan=nav_msgs/msg/Path
  /robotest/cmd_vel_nav=geometry_msgs/msg/Twist
  /robotest/cmd_vel_smoothed=geometry_msgs/msg/Twist
  /robotest/cmd_vel=geometry_msgs/msg/Twist
  /robotest/cmd_vel_behavior_unused=geometry_msgs/msg/Twist
  /robotest/global_costmap/costmap=nav_msgs/msg/OccupancyGrid
  /robotest/local_costmap/costmap=nav_msgs/msg/OccupancyGrid
  /robotest/validation/ground_truth=nav_msgs/msg/Odometry
  /robotest/validation/contacts=ros_gz_interfaces/msg/Contacts
  /robotest/validation/world_stats=ros_gz_interfaces/msg/WorldStatistics
  /robotest/faults/events=robotest_interfaces/msg/FaultEvent
  /tf=tf2_msgs/msg/TFMessage
  /tf_static=tf2_msgs/msg/TFMessage
)
graph_service_contracts=(
  /robotest/global_costmap/get_costmap=nav2_msgs/srv/GetCostmap
  /robotest/local_costmap/get_costmap=nav2_msgs/srv/GetCostmap
)
graph_action_contracts=(
  /robotest/follow_waypoints=nav2_msgs/action/FollowWaypoints
)
graph_arguments=()
for contract in "${graph_topic_contracts[@]}"; do
  graph_arguments+=(--topic "$contract")
done
for contract in "${graph_service_contracts[@]}"; do
  graph_arguments+=(--service "$contract")
done
for contract in "${graph_action_contracts[@]}"; do
  graph_arguments+=(--action "$contract")
done
graph_arguments+=(
  --action-server /robotest/follow_waypoints=/robotest/waypoint_follower
)
timeout --signal=TERM --kill-after=5s 100s \
  python3 "$GRAPH_PROBE" \
    "${graph_arguments[@]}" \
    --output "$RUN_DIR/graph-readiness.json" \
    --nodes-output "$RUN_DIR/nodes-ready.txt" \
    --topics-output "$RUN_DIR/topics-ready.txt" \
    --services-output "$RUN_DIR/services-ready.txt" \
    --actions-output "$RUN_DIR/actions-ready.txt" \
    --wall-timeout 90 \
    --watch-pid "$LAUNCH_PID" \
  | tee "$RUN_DIR/graph-readiness.log"
write_contact_gate_runtime_attestation \
  "$RUN_DIR/contact-gate-runtime-ready.json"

# Collision-monitor activation is rechecked after graph convergence and before mission start.
timeout --signal=TERM --kill-after=3s 15s \
  python3 "$LIFECYCLE_PROBE" \
    --namespace /robotest \
    --node collision_monitor \
    --output "$RUN_DIR/lifecycle-before-mission.json" \
    --text-dir "$RUN_DIR" \
    --text-prefix lifecycle-before-mission- \
    --wall-timeout 10 \
    --watch-pid "$LAUNCH_PID" \
  | tee "$RUN_DIR/lifecycle-before-mission.log"

sim_time_nodes=(
  robot_state_publisher parameter_bridge fault_proxy map_server amcl
  planner_server controller_server behavior_server bt_navigator waypoint_follower
  velocity_smoother collision_monitor lifecycle_manager_navigation
)
sim_time_node_arguments=()
for short_name in "${sim_time_nodes[@]}"; do
  sim_time_node_arguments+=(--node "$short_name")
done
timeout --signal=TERM --kill-after=5s 40s \
  python3 "$PARAMETER_PROBE" \
    --namespace /robotest \
    "${sim_time_node_arguments[@]}" \
    --output "$RUN_DIR/sim-time-parameters.json" \
    --text-dir "$RUN_DIR" \
    --text-prefix sim-time- \
    --wall-timeout 30 \
    --watch-pid "$LAUNCH_PID" \
  | tee "$RUN_DIR/sim-time-parameters-probe.log"

timeout 15s ros2 topic echo --no-daemon --once \
  /robotest/map nav_msgs/msg/OccupancyGrid \
  --qos-reliability reliable --qos-durability transient_local \
  > "$RUN_DIR/map-sample.txt"
grep -Fq 'frame_id: map' "$RUN_DIR/map-sample.txt"
write_costmap_service_evidence | tee "$RUN_DIR/costmap-service-probe.log"

set +e
timeout 10s ros2 run tf2_ros tf2_echo map odom \
  > "$RUN_DIR/tf-map-odom.txt" 2>&1
tf_map_status=$?
timeout 10s ros2 run tf2_ros tf2_echo odom base_footprint \
  > "$RUN_DIR/tf-odom-base_footprint.txt" 2>&1
tf_odom_status=$?
set -e
for status in "$tf_map_status" "$tf_odom_status"; do
  [[ "$status" -eq 0 || "$status" -eq 124 ]] || {
    echo "tf2_echo failed with status $status" >&2
    exit 1
  }
done
grep -Fq 'Translation:' "$RUN_DIR/tf-map-odom.txt"
grep -Fq 'Translation:' "$RUN_DIR/tf-odom-base_footprint.txt"

ps -eo pid=,ppid=,pgid=,psr=,stat=,comm=,args= > "$RUN_DIR/processes-ready.txt"

PROBE_READY="$RUN_DIR/probe.ready"
PROBE_STOP="$RUN_DIR/probe.stop"
PROBE_COMMAND=(
  timeout --signal=TERM --kill-after=5s 345s
  python3 "$PROBE"
  --output "$RUN_DIR/runtime-probe.json"
  --ready-file "$PROBE_READY"
  --stop-file "$PROBE_STOP"
  --warmup-wall-seconds 10
  --maximum-wall-seconds 330
)
setsid "${PROBE_COMMAND[@]}" \
  > "$RUN_DIR/runtime-probe.txt" 2>&1 &
PROBE_PID=$!
PROBE_PGID="$(ps -o pgid= -p "$PROBE_PID" | tr -d ' ')"
record_owned_group probe "$PROBE_PID" "$PROBE_PGID"
record_child_command probe "$PROBE_PID" "$RUN_DIR/runtime-probe.txt" \
  "$RUN_DIR/runtime-probe.txt" "${PROBE_COMMAND[@]}"

deadline=$((SECONDS + 25))
while (( SECONDS < deadline )) && [[ ! -f "$PROBE_READY" ]]; do
  kill -0 "$PROBE_PID" 2>/dev/null || {
    echo "runtime probe exited before readiness" >&2
    exit 1
  }
  sleep 0.25
done
[[ -f "$PROBE_READY" ]] || {
  echo "runtime probe readiness timed out" >&2
  exit 1
}
timeout --signal=TERM --kill-after=3s 20s \
  python3 "$PARAMETER_PROBE" \
    --namespace /robotest/evidence \
    --node phase2_runtime_probe \
    --output "$RUN_DIR/sim-time-runtime-probe.json" \
    --text-dir "$RUN_DIR" \
    --text-prefix sim-time- \
    --wall-timeout 15 \
    --watch-pid "$PROBE_PID" \
  | tee "$RUN_DIR/sim-time-runtime-probe.log"

timeout --signal=TERM --kill-after=3s 15s \
  python3 "$LIFECYCLE_PROBE" \
    --namespace /robotest \
    --node collision_monitor \
    --output "$RUN_DIR/lifecycle-at-mission-start.json" \
    --text-dir "$RUN_DIR" \
    --text-prefix lifecycle-at-mission-start- \
    --wall-timeout 10 \
    --watch-pid "$LAUNCH_PID" \
  | tee "$RUN_DIR/lifecycle-at-mission-start.log"

MISSION_COMMAND=(
  timeout --signal=TERM --kill-after=10s 310s
  ros2 run robotest_missions mission_runner
  --mission "$SCENARIO"
  --json "$RUN_DIR/mission-result.json"
  --csv "$RUN_DIR/mission-result.csv"
  --ros-args -r __ns:=/robotest
)
setsid "${MISSION_COMMAND[@]}" \
  > "$RUN_DIR/mission-runner.log" 2>&1 &
MISSION_PID=$!
MISSION_PGID="$(ps -o pgid= -p "$MISSION_PID" | tr -d ' ')"
record_owned_group mission "$MISSION_PID" "$MISSION_PGID"
record_child_command mission "$MISSION_PID" "$RUN_DIR/mission-runner.log" \
  "$RUN_DIR/mission-runner.log" "${MISSION_COMMAND[@]}"

timeout --signal=TERM --kill-after=3s 20s \
  python3 "$GRAPH_PROBE" \
    --action /robotest/follow_waypoints=nav2_msgs/action/FollowWaypoints \
    --action-client /robotest/follow_waypoints=/robotest/mission_runner \
    --action-server /robotest/follow_waypoints=/robotest/waypoint_follower \
    --output "$RUN_DIR/action-ownership-during-mission.json" \
    --nodes-output "$RUN_DIR/nodes-during-mission.txt" \
    --topics-output "$RUN_DIR/topics-during-mission.txt" \
    --services-output "$RUN_DIR/services-during-mission.txt" \
    --actions-output "$RUN_DIR/actions-during-mission.txt" \
    --wall-timeout 15 \
    --watch-pid "$MISSION_PID" \
  | tee "$RUN_DIR/action-ownership-during-mission.log"
timeout --signal=TERM --kill-after=3s 20s \
  python3 "$PARAMETER_PROBE" \
    --namespace /robotest \
    --node mission_runner \
    --output "$RUN_DIR/sim-time-mission.json" \
    --text-dir "$RUN_DIR" \
    --text-prefix sim-time- \
    --wall-timeout 15 \
    --watch-pid "$MISSION_PID" \
  | tee "$RUN_DIR/sim-time-mission.log"
set +e
wait "$MISSION_PID"
MISSION_STATUS=$?
set -e
MISSION_PID=""
MISSION_PGID=""
printf 'mission_process_exit_code=%s\n' "$MISSION_STATUS" \
  > "$RUN_DIR/mission-command-status.txt"

# Give the probe a steady-wall stop marker so a frozen /clock cannot hang it.
printf 'mission_exit_code=%s\n' "$MISSION_STATUS" > "$PROBE_STOP"
set +e
wait "$PROBE_PID"
PROBE_STATUS=$?
set -e
PROBE_PID=""
PROBE_PGID=""
printf 'runtime_probe_process_exit_code=%s\n' "$PROBE_STATUS" \
  > "$RUN_DIR/runtime-probe-status.txt"

(( MISSION_STATUS == 0 )) || {
  echo "mission runner failed with stable exit code $MISSION_STATUS" >&2
  exit 1
}
write_action_trace_and_validate_mission
write_mission_runtime_correlation
(( PROBE_STATUS == 0 )) || {
  echo "runtime probe failed with status $PROBE_STATUS" >&2
  exit 1
}

python3 - "$RUN_DIR/runtime-probe.json" <<'PY'
import json
import sys
from pathlib import Path

result = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
if result.get('verdict') != 'PASS':
    raise SystemExit(f'runtime probe verdict: {result.get("verdict")} {result.get("failures")}')
if 'tf_edges_by_publisher_gid' in result:
    raise SystemExit('runtime probe contains prohibited callback-GID TF attribution')
if result.get('scope', {}).get('fault_arming') != 'NOT_IMPLEMENTED_NOT_EXERCISED':
    raise SystemExit('runtime probe overclaims Phase 2 fault arming')
if result.get('scope', {}).get('scenario1_metrics') != 'DEFERRED_TO_PHASE3':
    raise SystemExit('runtime probe overclaims Scenario 1 metric acceptance')
qos = result['qos_introspection']
if result['bounded_depth_live_proven_for_all_endpoints'] and any(
    not status['complete'] for status in qos.values()
):
    raise SystemExit('incomplete QoS introspection was mislabeled as live bounded-depth proof')
PY

write_contact_gate_runtime_attestation \
  "$RUN_DIR/contact-gate-runtime-final.json" \
  "$RUN_DIR/contact-gate-runtime-ready.json"

ps -eo pid=,ppid=,pgid=,psr=,stat=,comm=,args= > "$RUN_DIR/processes-post-mission.txt"

sample_owned_groups || true
stop_launch
# PHASE2_FINAL_LAUNCH_LOG_GATE_ORDER
timeout --signal=TERM --kill-after=3s 20s \
  python3 "$STARTUP_GATE" \
    --scan-launch-log "$RUN_DIR/launch.log" \
    --output "$RUN_DIR/launch-log-signature-gate.json" \
    --launch-stopped-utc "$LAUNCH_STOPPED_UTC" \
  | tee "$RUN_DIR/launch-log-signature-gate.log"
stop_sampler
capture_phase2_ogre2_log
[[ -f "$RUN_DIR/ogre2-phase2-launch.log" ]] || {
  echo "Phase 2 renderer log evidence is absent" >&2
  exit 1
}

MAX_RSS_KIB="$(awk -F, 'NR>1 {sum[$1]+=$6} END {max=0; for (t in sum) if (sum[t]>max) max=sum[t]; print max+0}' "$RUN_DIR/resources.csv")"
printf '%s\n' \
  "peak_owned_process_groups_rss_kib=$MAX_RSS_KIB" \
  "target_max_kib=$((6 * 1024 * 1024))" \
  'scope=launch, runtime probe, and mission runner recorded process groups' \
  | tee "$RUN_DIR/resource-summary.txt"
(( MAX_RSS_KIB <= 6 * 1024 * 1024 )) || {
  echo "Phase 2 RSS target exceeded" >&2
  exit 1
}

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

# PHASE2_ACCEPTANCE_ORDER
echo "Phase 2 automated checks reached evidence finalization"

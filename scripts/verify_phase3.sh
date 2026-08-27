#!/usr/bin/env bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

set -Eeuo pipefail
IFS=$'\n\t'
umask 022

SCRIPT_PATH="$(readlink -f -- "$0")"
WORKSPACE="$(cd -- "$(dirname -- "$SCRIPT_PATH")/.." && pwd -P)"
HELPER="$WORKSPACE/tests/phase3_orchestration.py"
ROBOTEST_CPUSET="${ROBOTEST_CPUSET:-0-5}"

usage() {
  cat <<'EOF'
Usage: scripts/verify_phase3.sh

Builds and tests the Phase 3 overlay with at most four workers, runs static and
pure orchestration gates, validates installed CLI entry points, and writes a
source/install build-binding artifact. It never starts Gazebo and never starts
the positive-control, smoke, or 15-run candidate campaign.
EOF
}

if (( $# > 0 )); then
  if [[ $# == 1 && "$1" == "--help" ]]; then
    usage
    exit 0
  fi
  usage >&2
  exit 2
fi

if [[ "$ROBOTEST_CPUSET" != "0-5" ]]; then
  echo 'Phase 3 verification is frozen to ROBOTEST_CPUSET=0-5' >&2
  exit 2
fi

if [[ "${ROBOTEST_PHASE3_VERIFY_AFFINITY_APPLIED:-0}" != "1" ]]; then
  exec taskset -c 0-5 env \
    ROBOTEST_PHASE3_VERIFY_AFFINITY_APPLIED=1 \
    ROBOTEST_CPUSET=0-5 \
    "$SCRIPT_PATH"
fi

if [[ ! -r /etc/os-release ]]; then
  echo 'cannot identify the WSL distribution' >&2
  exit 2
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "24.04" ]]; then
  echo "Phase 3 requires WSL Ubuntu 24.04, found ${PRETTY_NAME:-unknown}" >&2
  exit 2
fi

for required in \
  "$HELPER" \
  "$WORKSPACE/tests/phase3_benchmark_runner.py" \
  "$WORKSPACE/tests/phase3_runtime_gate.py" \
  "$WORKSPACE/tests/phase3_runtime_observer.py" \
  "$WORKSPACE/src/robotest_metrics/schema/trial-context.schema.json"; do
  [[ -f "$required" && ! -L "$required" ]] || {
    echo "missing regular Phase 3 verifier input: $required" >&2
    exit 2
  }
done

EVIDENCE_ROOT="$WORKSPACE/artifacts/evidence/phase3"
mkdir -p -- "$EVIDENCE_ROOT"
EVIDENCE_ROOT="$(realpath -- "$EVIDENCE_ROOT")"
case "$EVIDENCE_ROOT" in
  "$WORKSPACE"/artifacts/evidence/phase3) ;;
  *) echo "unsafe Phase 3 evidence root: $EVIDENCE_ROOT" >&2; exit 2 ;;
esac
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)-$$"
RUN_DIR="$EVIDENCE_ROOT/$RUN_ID"
mkdir -- "$RUN_DIR"

# Retain this run and the four newest prior verifier runs. Only resolved direct
# children of the exact Phase 3 evidence root are eligible for pruning.
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

run_logged() {
  local role="$1"
  local timeout_s="$2"
  shift 2
  local log_path="$RUN_DIR/$role.log"
  local metadata_path="$RUN_DIR/$role.log.json"
  local command_status=0
  local drain_status=0
  set +e
  timeout --signal=TERM --kill-after=20s "$timeout_s" "$@" 2>&1 \
    | python3 "$HELPER" bounded-log \
        --output "$log_path" \
        --metadata "$metadata_path" \
        --maximum-bytes 8388608
  local pipeline_status=("${PIPESTATUS[@]}")
  command_status="${pipeline_status[0]}"
  drain_status="${pipeline_status[1]}"
  set -e
  if (( command_status != 0 || drain_status != 0 )); then
    echo "$role failed: command=$command_status bounded_log=$drain_status" >&2
    tail -n 80 "$log_path" >&2 || true
    return 1
  fi
}

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

export CMAKE_BUILD_PARALLEL_LEVEL=4
export MAKEFLAGS=-j4
export RCUTILS_COLORIZED_OUTPUT=0
export ROS2CLI_NO_DAEMON=1

cd "$WORKSPACE"
printf 'requested_cpu_affinity=0-5\n' > "$RUN_DIR/affinity.txt"
taskset -pc $$ >> "$RUN_DIR/affinity.txt"
if [[ "$(nproc)" != "6" ]]; then
  echo 'taskset did not expose exactly six logical CPUs' >&2
  exit 2
fi

run_logged colcon-build 1200s \
  colcon build --symlink-install --parallel-workers 4 \
    --packages-up-to \
      robotest_navigation robotest_missions robotest_metrics robotest_scenarios \
    --event-handlers console_direct+

set +u
# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"
set -u

run_logged rosdep-check 180s \
  rosdep check --from-paths src --ignore-src

run_logged colcon-test 1500s \
  colcon test --parallel-workers 4 \
    --packages-select \
      robotest_interfaces robotest_description robotest_faults robotest_sim \
      robotest_navigation robotest_missions robotest_metrics robotest_scenarios \
    --event-handlers console_direct+
run_logged colcon-test-result 120s colcon test-result --verbose

run_logged phase3-pytest 120s \
  pytest -q \
    tests/phase3_orchestration_test.py \
    tests/phase3_benchmark_runner_test.py
run_logged phase3-graph-probe-self-test 30s \
  python3 tests/phase2_graph_probe.py --self-test
run_logged phase3-pycompile 30s \
  python3 -m py_compile \
    tests/phase2_graph_probe.py \
    tests/phase3_orchestration.py \
    tests/phase3_benchmark_runner.py \
    tests/phase3_runtime_gate.py \
    tests/phase3_runtime_observer.py
run_logged phase3-ament-flake8 60s \
  ament_flake8 \
    tests/phase2_graph_probe.py \
    tests/phase3_orchestration.py \
    tests/phase3_benchmark_runner.py \
    tests/phase3_runtime_gate.py \
    tests/phase3_runtime_observer.py \
    tests/phase3_orchestration_test.py \
    tests/phase3_benchmark_runner_test.py
run_logged phase3-observer-self-test 30s \
  python3 tests/phase3_runtime_observer.py self-test
run_logged phase3-gate-self-test 30s \
  python3 tests/phase3_runtime_gate.py --self-test
run_logged shellcheck 60s \
  shellcheck scripts/verify_phase3.sh scripts/run_benchmarks.sh

if command -v ruff >/dev/null 2>&1; then
  run_logged ruff-check 60s \
    ruff check \
      tests/phase3_orchestration.py \
      tests/phase3_benchmark_runner.py \
      tests/phase3_runtime_gate.py \
      tests/phase3_runtime_observer.py \
      tests/phase3_orchestration_test.py \
      tests/phase3_benchmark_runner_test.py
  run_logged ruff-format 60s \
    ruff format --check \
      tests/phase3_orchestration.py \
      tests/phase3_benchmark_runner.py \
      tests/phase3_runtime_gate.py \
      tests/phase3_runtime_observer.py \
      tests/phase3_orchestration_test.py \
      tests/phase3_benchmark_runner_test.py
fi

run_logged mission-help 30s ros2 run robotest_missions mission_runner --help
run_logged scenario-help 30s ros2 run robotest_scenarios scenario_controller --help
run_logged contact-help 30s ros2 run robotest_scenarios contact_control_driver --help
run_logged collector-help 30s ros2 run robotest_metrics metrics_collector --help
run_logged lifecycle-help 30s ros2 run robotest_metrics metrics_lifecycle_sampler --help
run_logged analyze-help 30s ros2 run robotest_metrics metrics_analyze --help
run_logged aggregate-help 30s ros2 run robotest_metrics metrics_aggregate --help
run_logged benchmark-runner-help 30s "$WORKSPACE/scripts/run_benchmarks.sh" --help

GIT_SHA="$(git -C "$WORKSPACE" rev-parse HEAD)"
GIT_STATUS="$(git -C "$WORKSPACE" status --porcelain=v1 --untracked-files=all)"
python3 "$HELPER" build-binding \
  --workspace "$WORKSPACE" \
  --git-sha "$GIT_SHA" \
  --git-status-porcelain "$GIT_STATUS" \
  --output "$RUN_DIR/build-binding.json"

python3 - "$RUN_DIR" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

run = Path(sys.argv[1])
summary = {
    'build_binding': str(run / 'build-binding.json'),
    'producer': 'robotest_phase3/benchmark_orchestrator',
    'runtime_campaign_started': False,
    'schema_version': 1,
    'status': 'PASS',
}
(run / 'verification-summary.json').write_text(
    json.dumps(summary, allow_nan=False, ensure_ascii=False, separators=(',', ':'), sort_keys=True)
    + '\n',
    encoding='utf-8',
)
records = []
for path in sorted(item for item in run.rglob('*') if item.is_file() and item.name != 'SHA256SUMS'):
    records.append(f'{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.relative_to(run).as_posix()}')
(run / 'SHA256SUMS').write_text('\n'.join(records) + '\n', encoding='ascii')
PY

printf 'Phase 3 static/build verification PASS: %s\n' "$RUN_DIR"
printf 'Build binding for scripts/run_benchmarks.sh: %s\n' "$RUN_DIR/build-binding.json"

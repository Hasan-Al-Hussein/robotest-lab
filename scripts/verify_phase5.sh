#!/usr/bin/env bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

set -Eeuo pipefail
IFS=$'\n\t'
umask 022
export PYTHONDONTWRITEBYTECODE=1

ORIGINAL_CWD="$(pwd -P)"
readonly ORIGINAL_CWD
readonly -a INVOCATION_ARGUMENTS=("$0" "$@")
SCRIPT_PATH="$(readlink -f -- "$0")"
SCRIPT_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd -P)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly SCRIPT_PATH SCRIPT_DIR PROJECT_ROOT
readonly HELPER="${PROJECT_ROOT}/tests/phase5_ci.py"
readonly WORKFLOW="${PROJECT_ROOT}/.github/workflows/robotest-ci.yml"
readonly EVIDENCE_ROOT="${PROJECT_ROOT}/artifacts/evidence/phase5"
readonly REMOTE_EVIDENCE_ROOT="${PROJECT_ROOT}/docs/results/phase-5"
readonly EVIDENCE_COMMIT_REMOTE_ROOT="${EVIDENCE_ROOT}/remote-evidence-commit"
readonly MAXIMUM_LOG_BYTES=8388608

MODE=""
REMOTE_SHA=""
RUN_DIRECTORY=""
WORK_ROOT=""
CHECKS_FILE=""
WORKFLOW_REPORT=""
LICENSE_REPORT=""
CLAIMS_REPORT=""
TEST_SURFACE_REPORT=""
SUMMARY_PATH=""
SUMMARY_CSV=""
PROVENANCE_REPORT=""
RETENTION_REPORT=""
SOURCE_SNAPSHOT=""
GIT_STATUS_PATH=""
FINAL_STATUS="FAIL"

log() {
  printf '[verify_phase5] %s\n' "$*"
}

die() {
  printf '[verify_phase5] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage:
  scripts/verify_phase5.sh --local
  scripts/verify_phase5.sh --ci
  scripts/verify_phase5.sh --remote <40-character-lowercase-commit-sha>
  scripts/verify_phase5.sh --remote-evidence-commit <40-character-lowercase-commit-sha>

--local and --ci run only L0-L2 checks: workflow policy, formatting, lint,
pure unit tests, a fresh bounded (at most four-worker) ROS build/package-test pass, and CLI help
smoke checks. They never start Gazebo, hardware, systemd, the Phase 3 campaign,
or the privileged Phase 4 acceptance path.

--remote is read-only with respect to GitHub. It requires gh authentication, a
public GitHub repository, and a completed successful RoboTest CI run for the
exact requested pushed SHA. It does not create a repository, push, rerun, or
dispatch a workflow. Its compact JSON, checksum manifest, and validation
sidecar are written under docs/results/phase-5/ for an explicit evidence-only
commit; the command never creates that commit itself.

--remote-evidence-commit applies the same read-only checks to the evidence-only
commit and writes its proof below ignored artifacts/evidence/phase5/. This
second proof is deliberately not committed, so it cannot demand a third CI run.

There is deliberately no implicit mode.
EOF
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

require_noble() {
  [[ -r /etc/os-release ]] || die 'Cannot identify the operating system.'
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" ]] ||
    die "Phase 5 local CI requires Ubuntu 24.04; found ${PRETTY_NAME:-unknown}."
  [[ "${WSL_DISTRO_NAME:-}" != "Ubuntu-20.04" ]] ||
    die 'Refusing to operate in the protected Ubuntu-20.04 distribution.'
}

cleanup_work_root() {
  [[ -n "${WORK_ROOT}" && -d "${WORK_ROOT}" ]] || return 0
  case "${WORK_ROOT}" in
    /tmp/robotest-phase5.*) rm -rf -- "${WORK_ROOT}" ;;
    *) printf '[verify_phase5] Refusing unsafe temporary cleanup: %s\n' "${WORK_ROOT}" >&2 ;;
  esac
  WORK_ROOT=""
}

finalize_local() {
  local execution_status=$?
  local status="FAIL"
  local provenance_status=0
  local summary_status=0
  local manifest_status=0
  local checksum_status=0
  local final_exit=0
  local argument=""
  local -a provenance_arguments=()

  trap - EXIT INT TERM
  set +e
  if ((execution_status == 0)) && [[ "${FINAL_STATUS}" == "PASS" ]]; then
    status="PASS"
  fi

  if [[ -n "${RUN_DIRECTORY}" && -d "${RUN_DIRECTORY}" ]]; then
    for argument in "${INVOCATION_ARGUMENTS[@]}"; do
      provenance_arguments+=("--command-argument=${argument}")
    done
    python3 "${HELPER}" provenance \
      "${provenance_arguments[@]}" \
      --cwd "${ORIGINAL_CWD}" \
      --git-dirty "${GIT_DIRTY}" \
      --git-sha "${GIT_SHA}" \
      --git-status "${GIT_STATUS_PATH}" \
      --maximum-workers "${MAXIMUM_WORKERS}" \
      --mode "${MODE}" \
      --output "${PROVENANCE_REPORT}" \
      --repository "${PROJECT_ROOT}" \
      --ruff "${RUFF}" \
      --source-snapshot "${SOURCE_SNAPSHOT}"
    provenance_status=$?
    if ((provenance_status != 0)); then
      status="FAIL"
      printf '[verify_phase5] Failed to finalize provenance.\n' >&2
    fi

    python3 "${HELPER}" summary \
      --checks "${CHECKS_FILE}" \
      --claims-report "${CLAIMS_REPORT}" \
      --csv-output "${SUMMARY_CSV}" \
      --git-dirty "${GIT_DIRTY}" \
      --git-sha "${GIT_SHA}" \
      --github-run-id "${GITHUB_RUN_ID:-}" \
      --github-sha "${GITHUB_SHA:-}" \
      --license-report "${LICENSE_REPORT}" \
      --maximum-workers "${MAXIMUM_WORKERS}" \
      --mode "${MODE}" \
      --output "${SUMMARY_PATH}" \
      --provenance-report "${PROVENANCE_REPORT}" \
      --retention-report "${RETENTION_REPORT}" \
      --status "${status}" \
      --source-snapshot "${SOURCE_SNAPSHOT}" \
      --test-surface-report "${TEST_SURFACE_REPORT}" \
      --workflow-report "${WORKFLOW_REPORT}"
    summary_status=$?
    if ((summary_status != 0)); then
      printf '[verify_phase5] Failed to finalize local evidence.\n' >&2
    fi

    python3 "${HELPER}" checksums --run-directory "${RUN_DIRECTORY}"
    manifest_status=$?
    if ((manifest_status == 0)); then
      validate_checksum_manifest
      checksum_status=$?
    else
      checksum_status=1
      printf '[verify_phase5] Failed to create the checksum manifest.\n' >&2
    fi
  fi
  cleanup_work_root

  if ((
    execution_status == 0 &&
      provenance_status == 0 &&
      summary_status == 0 &&
      manifest_status == 0 &&
      checksum_status == 0
  )) && [[ "${FINAL_STATUS}" == "PASS" ]]; then
    log "LOCAL CI PASS (${MODE}, L0-L2 only). Evidence: ${SUMMARY_PATH}"
    log 'Remote CI, live simulation, privileged acceptance, and hardware remain separate gates.'
  else
    final_exit="${execution_status}"
    ((final_exit != 0)) || final_exit="${provenance_status}"
    ((final_exit != 0)) || final_exit="${summary_status}"
    ((final_exit != 0)) || final_exit="${manifest_status}"
    ((final_exit != 0)) || final_exit="${checksum_status}"
    ((final_exit != 0)) || final_exit=1
    printf '[verify_phase5] FAILED (execution=%s provenance=%s summary=%s manifest=%s checksum=%s). Evidence: %s\n' \
      "${execution_status}" \
      "${provenance_status}" \
      "${summary_status}" \
      "${manifest_status}" \
      "${checksum_status}" \
      "${SUMMARY_PATH:-unavailable}" >&2
    exit "${final_exit}"
  fi
  exit 0
}

validate_checksum_manifest() {
  local temporary=""
  local validation_status=0
  temporary="$(mktemp "${RUN_DIRECTORY}/.checksum-validation.XXXXXX")" || return 1
  if (
    cd -- "${RUN_DIRECTORY}" &&
      sha256sum -c --strict SHA256SUMS
  ) >"${temporary}" 2>&1; then
    validation_status=0
  else
    validation_status=$?
  fi
  if ! mv -- "${temporary}" "${RUN_DIRECTORY}/checksum-validation.txt"; then
    rm -f -- "${temporary}"
    return 1
  fi
  return "${validation_status}"
}

run_check() {
  local name="$1"
  local timeout_seconds="$2"
  shift 2
  local log_path="${RUN_DIRECTORY}/${name}.log"
  local metadata_path="${RUN_DIRECTORY}/${name}.log.json"
  local command_status=0
  local drain_status=0
  local -a pipeline_status=()

  log "Running ${name}."
  set +e
  timeout --signal=TERM --kill-after=20s "${timeout_seconds}" "$@" 2>&1 |
    python3 "${HELPER}" bounded-log \
      --output "${log_path}" \
      --metadata "${metadata_path}" \
      --maximum-bytes "${MAXIMUM_LOG_BYTES}"
  pipeline_status=("${PIPESTATUS[@]}")
  set -e
  command_status="${pipeline_status[0]}"
  drain_status="${pipeline_status[1]}"

  if ((command_status != 0 || drain_status != 0)); then
    printf '%s\tfailed\t%s\t%s\n' \
      "${name}" "${command_status}" "${log_path#"${PROJECT_ROOT}/"}" >>"${CHECKS_FILE}"
    tail -n 100 "${log_path}" >&2 || true
    printf '[verify_phase5] %s failed: command=%s log_capture=%s\n' \
      "${name}" "${command_status}" "${drain_status}" >&2
    return 1
  fi
  printf '%s\tpassed\t0\t%s\n' \
    "${name}" "${log_path#"${PROJECT_ROOT}/"}" >>"${CHECKS_FILE}"
}

verify_remote() {
  local repository_json=""
  local repository=""
  local repository_url=""
  local visibility=""
  local remote_sha=""
  local resolved_local_sha=""
  local runs_json=""
  local output=""
  local remote_manifest=""
  local remote_validation=""
  local validation_temporary=""
  local validation_status=0
  local argument=""
  local -a remote_provenance_arguments=()
  local output_root="${REMOTE_EVIDENCE_ROOT}"

  if [[ "${MODE}" == 'remote-evidence-commit' ]]; then
    output_root="${EVIDENCE_COMMIT_REMOTE_ROOT}"
  fi

  [[ "${REMOTE_SHA}" =~ ^[0-9a-f]{40}$ ]] ||
    die 'Remote SHA must contain exactly 40 lowercase hexadecimal characters.'
  for command_name in gh git python3; do
    require_command "${command_name}"
  done
  gh auth status >/dev/null 2>&1 || die 'gh is not authenticated; remote verification is incomplete.'
  git -C "${PROJECT_ROOT}" remote get-url origin >/dev/null 2>&1 ||
    die 'No origin remote is configured; remote verification is incomplete.'
  resolved_local_sha="$(git -C "${PROJECT_ROOT}" rev-parse --verify "${REMOTE_SHA}^{commit}" 2>/dev/null)" ||
    die "Requested SHA is not a local commit: ${REMOTE_SHA}"
  [[ "${resolved_local_sha}" == "${REMOTE_SHA}" ]] || die 'Local commit resolution changed the SHA.'

  WORK_ROOT="$(mktemp -d /tmp/robotest-phase5.remote.XXXXXX)"
  trap cleanup_work_root EXIT
  repository_json="${WORK_ROOT}/repository.json"
  runs_json="${WORK_ROOT}/runs.json"
  gh repo view --json nameWithOwner,url,visibility >"${repository_json}" ||
    die 'Unable to read the configured GitHub repository.'
  mapfile -t repository_fields < <(
    python3 - "${repository_json}" <<'PY'
import json
import pathlib
import sys

value = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding='utf-8'))
for key in ('nameWithOwner', 'url', 'visibility'):
    field = value.get(key)
    if not isinstance(field, str) or not field or '\n' in field:
        raise SystemExit(f'invalid GitHub repository field: {key}')
    print(field)
PY
  )
  ((${#repository_fields[@]} == 3)) || die 'GitHub repository metadata is incomplete.'
  repository="${repository_fields[0]}"
  repository_url="${repository_fields[1]}"
  visibility="${repository_fields[2]}"
  [[ "${visibility}" == "PUBLIC" ]] || die 'The configured GitHub repository is not public.'

  remote_sha="$(gh api "repos/${repository}/commits/${REMOTE_SHA}" --jq .sha)" ||
    die 'The exact commit is not available from the configured public repository.'
  [[ "${remote_sha}" == "${REMOTE_SHA}" ]] || die 'GitHub resolved a different commit SHA.'
  gh run list \
    --repo "${repository}" \
    --workflow robotest-ci.yml \
    --commit "${REMOTE_SHA}" \
    --limit 50 \
    --json databaseId,headSha,status,conclusion,url,workflowName,createdAt >"${runs_json}" ||
    die 'Unable to list RoboTest CI workflow runs.'

  mkdir -p -- "${output_root}"
  output="${output_root}/remote-${REMOTE_SHA}.json"
  remote_manifest="${output_root}/remote-${REMOTE_SHA}.SHA256SUMS"
  remote_validation="${output_root}/remote-${REMOTE_SHA}.checksum-validation.txt"
  for argument in "${INVOCATION_ARGUMENTS[@]}"; do
    remote_provenance_arguments+=("--command-argument=${argument}")
  done
  python3 "${HELPER}" remote \
    "${remote_provenance_arguments[@]}" \
    --cwd "${ORIGINAL_CWD}" \
    --local-resolved-sha "${resolved_local_sha}" \
    --output "${output}" \
    --remote-sha "${remote_sha}" \
    --repository "${repository}" \
    --repository-path "${PROJECT_ROOT}" \
    --repository-url "${repository_url}" \
    --runs-json "${runs_json}" \
    --sha "${REMOTE_SHA}" \
    --visibility "${visibility}"
  python3 "${HELPER}" file-checksum --file "${output}" --manifest "${remote_manifest}"
  validation_temporary="$(mktemp "${output_root}/.remote-validation.XXXXXX")" ||
    die 'Unable to allocate remote checksum validation output.'
  if (
    cd -- "${output_root}" &&
      sha256sum -c --strict "remote-${REMOTE_SHA}.SHA256SUMS"
  ) >"${validation_temporary}" 2>&1; then
    validation_status=0
  else
    validation_status=$?
  fi
  if ! mv -- "${validation_temporary}" "${remote_validation}"; then
    rm -f -- "${validation_temporary}"
    die 'Unable to persist remote checksum validation output.'
  fi
  ((validation_status == 0)) || die 'Remote evidence checksum validation failed.'
  python3 "${HELPER}" file-checksum-validate \
    --file "${output}" \
    --manifest "${remote_manifest}"
  cleanup_work_root
  trap - EXIT
  log "REMOTE CI PASS for exact SHA ${REMOTE_SHA}. Evidence: ${output}"
}

if (($# == 1)) && [[ "$1" == "--help" || "$1" == "-h" ]]; then
  usage
  exit 0
elif (($# == 1)) && [[ "$1" == "--local" ]]; then
  MODE="local"
elif (($# == 1)) && [[ "$1" == "--ci" ]]; then
  MODE="ci"
elif (($# == 2)) && [[ "$1" == "--remote" ]]; then
  MODE="remote"
  REMOTE_SHA="$2"
elif (($# == 2)) && [[ "$1" == "--remote-evidence-commit" ]]; then
  MODE="remote-evidence-commit"
  REMOTE_SHA="$2"
else
  usage >&2
  exit 2
fi

[[ -f "${HELPER}" && ! -L "${HELPER}" ]] || die "Missing regular helper: ${HELPER}"
[[ -f "${WORKFLOW}" && ! -L "${WORKFLOW}" ]] || die "Missing regular workflow: ${WORKFLOW}"

if [[ "${MODE}" == "remote" || "${MODE}" == "remote-evidence-commit" ]]; then
  verify_remote
  exit 0
fi

require_noble
for command_name in bash colcon git go python3 realpath rosdep shellcheck timeout; do
  require_command "${command_name}"
done

MAXIMUM_WORKERS="${ROBOTEST_CI_MAX_WORKERS:-4}"
[[ "${MAXIMUM_WORKERS}" =~ ^[1-4]$ ]] ||
  die 'ROBOTEST_CI_MAX_WORKERS must be an integer from 1 through 4.'
export CMAKE_BUILD_PARALLEL_LEVEL="${MAXIMUM_WORKERS}"
export MAKEFLAGS="-j${MAXIMUM_WORKERS}"
export RCUTILS_COLORIZED_OUTPUT=0
export ROS2CLI_NO_DAEMON=1

[[ -r /opt/ros/jazzy/setup.bash ]] || die 'Missing ROS 2 Jazzy underlay.'
case "${AMENT_PREFIX_PATH:-}:${COLCON_PREFIX_PATH:-}" in
  *robotest-lab/install* | */opt/robotest-lab*)
    die 'Refusing an inherited RoboTest overlay; start from the Jazzy underlay only.'
    ;;
esac
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
[[ "${ROS_DISTRO:-}" == "jazzy" ]] || die "Expected ROS_DISTRO=jazzy; found ${ROS_DISTRO:-unset}."

RUFF="${PROJECT_ROOT}/.venv/bin/ruff"
[[ -x "${RUFF}" ]] || die 'Missing repository-local Ruff; run the documented tooling setup.'
[[ "$("${RUFF}" --version)" == 'ruff 0.16.4' ]] || die 'Repository Ruff version is not 0.16.4.'

GIT_SHA="$(git -C "${PROJECT_ROOT}" rev-parse HEAD)"
[[ "${GIT_SHA}" =~ ^[0-9a-f]{40}$ ]] || die 'Unable to resolve the current Git SHA.'
GIT_STATUS="$(git -C "${PROJECT_ROOT}" status --porcelain=v1 --untracked-files=all)"
if [[ -n "${GIT_STATUS}" ]]; then
  GIT_DIRTY=true
else
  GIT_DIRTY=false
fi
if [[ "${MODE}" == "ci" ]]; then
  [[ "${GITHUB_ACTIONS:-}" == "true" ]] || die '--ci is restricted to GitHub Actions.'
  [[ "${GITHUB_SHA:-}" =~ ^[0-9a-f]{40}$ ]] || die 'GitHub Actions did not provide an exact SHA.'
  [[ "${GITHUB_RUN_ID:-}" =~ ^[1-9][0-9]*$ ]] || die 'GitHub Actions run ID is invalid.'
  [[ "${GITHUB_SHA}" == "${GIT_SHA}" ]] || die 'Checked-out HEAD does not match GITHUB_SHA.'
  [[ "${GIT_DIRTY}" == "false" ]] || die 'The GitHub Actions checkout is unexpectedly dirty.'
elif [[ "${GITHUB_ACTIONS:-}" == "true" ]]; then
  die 'GitHub Actions must invoke the explicit --ci mode.'
fi
readonly GIT_SHA GIT_DIRTY GIT_STATUS

mkdir -p -- "${EVIDENCE_ROOT}"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)-$$"
RUN_DIRECTORY="${EVIDENCE_ROOT}/${RUN_ID}"
mkdir -- "${RUN_DIRECTORY}"
RUN_DIRECTORY="$(realpath -e -- "${RUN_DIRECTORY}")"
case "${RUN_DIRECTORY}" in
  "${EVIDENCE_ROOT}"/*) ;;
  *) die "Unsafe Phase 5 evidence directory: ${RUN_DIRECTORY}" ;;
esac
WORK_ROOT="$(mktemp -d /tmp/robotest-phase5.XXXXXX)"
CHECKS_FILE="${RUN_DIRECTORY}/checks.tsv"
WORKFLOW_REPORT="${RUN_DIRECTORY}/workflow-contract.json"
LICENSE_REPORT="${RUN_DIRECTORY}/license-inventory.json"
CLAIMS_REPORT="${RUN_DIRECTORY}/release-claims.json"
TEST_SURFACE_REPORT="${RUN_DIRECTORY}/test-surface.json"
SUMMARY_PATH="${RUN_DIRECTORY}/verification-summary.json"
SUMMARY_CSV="${RUN_DIRECTORY}/verification-summary.csv"
PROVENANCE_REPORT="${RUN_DIRECTORY}/provenance.json"
RETENTION_REPORT="${RUN_DIRECTORY}/retention.json"
SOURCE_SNAPSHOT="${RUN_DIRECTORY}/source-snapshot-start.json"
GIT_STATUS_PATH="${RUN_DIRECTORY}/git-status-start.txt"
printf 'name\tstatus\texit_code\tlog\n' >"${CHECKS_FILE}"
trap finalize_local EXIT
printf '%s' "${GIT_STATUS}" >"${GIT_STATUS_PATH}"

mapfile -d '' -t SHELL_FILES < <(
  find "${SCRIPT_DIR}" -maxdepth 1 -type f -name '*.sh' -print0 | LC_ALL=C sort -z
)
((${#SHELL_FILES[@]} > 0)) || die 'No repository shell scripts were found.'
mapfile -t PYTEST_FILES < <(
  find "${PROJECT_ROOT}/tests" -maxdepth 1 -type f -name '*_test.py' -printf '%p\n' |
    LC_ALL=C sort
)
((${#PYTEST_FILES[@]} > 0)) || die 'No repository pure-unit test files were found.'

run_check evidence-retention 30s \
  python3 "${HELPER}" retention \
    --evidence-root "${EVIDENCE_ROOT}" \
    --current-run "${RUN_DIRECTORY}" \
    --maximum-prior 4 \
    --output "${RETENTION_REPORT}"
run_check source-provenance-start 120s \
  python3 "${HELPER}" source-snapshot \
    --repository "${PROJECT_ROOT}" \
    --output "${SOURCE_SNAPSHOT}"
run_check workflow-contract 30s \
  python3 "${HELPER}" workflow --workflow "${WORKFLOW}" --output "${WORKFLOW_REPORT}"
run_check license-inventory 60s \
  python3 "${HELPER}" licenses --repository "${PROJECT_ROOT}" --output "${LICENSE_REPORT}"
run_check release-claims 30s \
  python3 "${HELPER}" claims --repository "${PROJECT_ROOT}" --output "${CLAIMS_REPORT}"
run_check non-live-test-surface 30s \
  python3 "${HELPER}" test-surface \
    --repository "${PROJECT_ROOT}" \
    --output "${TEST_SURFACE_REPORT}"
run_check git-diff-check 30s git -C "${PROJECT_ROOT}" diff --check
run_check bash-syntax 60s bash -n "${SHELL_FILES[@]}"
run_check shellcheck 180s shellcheck --severity=warning "${SHELL_FILES[@]}"
run_check ruff-check 180s "${RUFF}" check "${PROJECT_ROOT}/tests" "${PROJECT_ROOT}/src"
run_check ruff-format 180s "${RUFF}" format --check "${PROJECT_ROOT}/tests" "${PROJECT_ROOT}/src"
run_check pure-python-tests 600s python3 -m pytest -q "${PYTEST_FILES[@]}"

run_check phase3-static-interface 30s bash "${SCRIPT_DIR}/verify_phase3.sh" --help
run_check phase3-campaign-interface 30s bash "${SCRIPT_DIR}/run_benchmarks.sh" --help
run_check phase4-static-interface 30s bash "${SCRIPT_DIR}/verify_phase4.sh" --help

run_check go-format 60s bash -c '
  cd -- "$1"
  files="$(gofmt -l ./cmd ./internal)"
  [[ -z "${files}" ]] || { printf "Unformatted Go files:\n%s\n" "${files}" >&2; exit 1; }
' _ "${PROJECT_ROOT}/supervisor"
run_check go-race-tests 600s bash -c 'cd -- "$1" && go test -race ./...' _ \
  "${PROJECT_ROOT}/supervisor"
run_check go-vet 180s bash -c 'cd -- "$1" && go vet ./...' _ \
  "${PROJECT_ROOT}/supervisor"

run_check rosdep-check 180s rosdep check --from-paths "${PROJECT_ROOT}/src" --ignore-src \
  --rosdistro jazzy
run_check colcon-build 1200s \
  colcon --log-base "${WORK_ROOT}/log" build \
    --base-paths "${PROJECT_ROOT}/src" \
    --build-base "${WORK_ROOT}/build" \
    --install-base "${WORK_ROOT}/install" \
    --parallel-workers "${MAXIMUM_WORKERS}" \
    --symlink-install \
    --event-handlers console_cohesion+ \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

set +u
# shellcheck disable=SC1090
source "${WORK_ROOT}/install/setup.bash"
set -u
run_check colcon-test 900s \
  colcon --log-base "${WORK_ROOT}/test-log" test \
    --build-base "${WORK_ROOT}/build" \
    --install-base "${WORK_ROOT}/install" \
    --parallel-workers "${MAXIMUM_WORKERS}" \
    --event-handlers console_cohesion+
run_check colcon-test-result 120s \
  colcon test-result --test-result-base "${WORK_ROOT}/build" --verbose

run_check mission-help 30s ros2 run robotest_missions mission_runner --help
run_check scenario-help 30s ros2 run robotest_scenarios scenario_controller --help
run_check metrics-help 30s ros2 run robotest_metrics metrics_analyze --help
run_check aggregate-help 30s ros2 run robotest_metrics metrics_aggregate --help

FINAL_STATUS="PASS"

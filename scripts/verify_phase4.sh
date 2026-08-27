#!/bin/bash -p
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

validate_root_entry_environment() {
  [[ "${ROBOTEST_PHASE4_VERIFY_ROOT_ENV:-}" == 1 &&
    "${HOME:-}" == /root &&
    "${LANG:-}" == C.UTF-8 &&
    "${LC_ALL:-}" == C.UTF-8 &&
    "${PATH:-}" == /usr/sbin:/usr/bin:/sbin:/bin &&
    "${PYTHONDONTWRITEBYTECODE:-}" == 1 &&
    "${PYTHONNOUSERSITE:-}" == 1 ]] || return 1

  local variable_name
  while IFS= read -r variable_name; do
    case "${variable_name}" in
      HOME | LANG | LC_ALL | PATH | PWD | PYTHONDONTWRITEBYTECODE | \
        PYTHONNOUSERSITE | ROBOTEST_PHASE4_VERIFY_ROOT_ENV | SHLVL | _) ;;
      *) return 1 ;;
    esac
  done < <(compgen -e)
}

if ((EUID == 0)); then
  if [[ "${ROBOTEST_PHASE4_VERIFY_ROOT_ENV:-}" != 1 ]]; then
    exec /usr/bin/env -i \
      HOME=/root \
      LANG=C.UTF-8 \
      LC_ALL=C.UTF-8 \
      PATH=/usr/sbin:/usr/bin:/sbin:/bin \
      PYTHONDONTWRITEBYTECODE=1 \
      PYTHONNOUSERSITE=1 \
      ROBOTEST_PHASE4_VERIFY_ROOT_ENV=1 \
      /bin/bash -p -- "$0" "$@"
  fi
  validate_root_entry_environment || {
    /usr/bin/printf '%s\n' \
      '[verify_phase4] ERROR: privileged entry environment is not canonical.' >&2
    exit 1
  }
fi

set -Eeuo pipefail
export PYTHONDONTWRITEBYTECODE=1

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly PROJECT_ROOT
readonly HELPER="${PROJECT_ROOT}/tests/phase4_acceptance.py"
readonly ACTIVE_GOAL_PROBE="${PROJECT_ROOT}/tests/phase4_active_goal_probe.py"
readonly SERVICE="robotest-supervisor.service"
readonly BASE_URL="http://127.0.0.1:9080"
readonly DROPIN_DIRECTORY="/run/systemd/system/${SERVICE}.d"
readonly DROPIN_FILE="${DROPIN_DIRECTORY}/90-robotest-phase4-verifier.conf"
readonly LOCK_FILE="/run/lock/robotest-phase4-verifier.lock"
readonly LIVE_RUN_ROOT="/run/robotest-phase4-verifier"

APPLY=0
PACKAGE_DIRECTORY=""
STATIC_TEMP="${ROBOTEST_PHASE4_STATIC_TEMP:-}"
STATIC_WORKER_MODE="${ROBOTEST_PHASE4_STATIC_WORKER:-0}"
STATIC_WORKER_SCRATCH=""
APPLY_SOURCE_HEAD=""
RUN_ID=""
RUN_DIRECTORY=""
PUBLIC_RUN_DIRECTORY=""
OVERLAY_STAGE_SCRATCH=""
OVERLAY_STAGE_SCRIPT=""
OVERLAY_STAGE_EVIDENCE=""
STATE_RUN_DIRECTORY=""
STATE_STAGING_DIRECTORY=""
HEARTBEAT=""
STARTUP_RESULT=""
TIMELINE=""
ROS_DOMAIN_ID=""
GZ_PARTITION=""
UPGRADE_DEB=""
BASELINE_DEB=""
PACKAGE_MANIFEST=""
PACKAGE_SOURCE_REBUILD=""
PACKAGE_REBUILD_SCRATCH=""
LIFECYCLE_EVIDENCE=""
PROJECT_USER=""
PROJECT_GROUP=""
DROPIN_SHA256=""
DROPIN_STAGING_FILE=""
DROPIN_DIRECTORY_CREATED=0
SERVICE_STARTED=0
MISSION_PID=""
MISSION_PGID=""
MISSION_LAST_PGID=""
PROBE_PID=""
PROBE_PGID=""
PROBE_LAST_PGID=""
FOLLOWUP_PID=""
FOLLOWUP_PGID=""
FOLLOWUP_LAST_PGID=""
ORIGINAL_PGID=""
RESTORED_PGID=""
EVENT_SEQUENCE_BEFORE_STOP=0
PENDING_PUBLICATION_SIGNAL=0
PUBLICATION_SIGNAL_TRAPS_CLEARED=0
FINALIZED=0
FINALIZATION_STATE=0
FINALIZATION_STATUS=0
CLEANUP_STATUS=0
COMPOSE_STATUS=0
CHECKSUM_STATUS=0
FREEZE_STATUS=0
PUBLICATION_STATUS=0
LIVE_REMOVAL_STATUS=0
LIVE_RUN_OWNED=0
SERVICE_INACTIVITY_PROVEN=0
EXIT_HANDLER_ACTIVE=0
DAEMON_RELOADED=0

log() {
  printf '[verify_phase4] %s\n' "$*"
}

die() {
  printf '[verify_phase4] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: scripts/verify_phase4.sh [--package-dir DIRECTORY] \
  [--lifecycle-evidence FILE] [--apply]

Without --apply, runs only static/unit/package-source checks and makes no
installed-package, overlay, service, or process changes.

--apply is the explicit privileged gate for immutable overlay staging,
controlled systemd start/stop, and the live Scenario 6 nested-controller crash.
The separately executed package lifecycle proof must be supplied or discoverable
and must bind exactly to the selected installed candidate. Run --apply as root
only after all shared Phase 3 source is stable.
EOF
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

validated_remove_static_temp() {
  [[ -n "${STATIC_TEMP}" && -d "${STATIC_TEMP}" ]] || return 0
  case "${STATIC_TEMP}" in
    /tmp/robotest-phase4-static.*) rm -rf -- "${STATIC_TEMP}" ;;
    *) die "Refusing unsafe static temporary cleanup: ${STATIC_TEMP}" ;;
  esac
  STATIC_TEMP=""
}

validated_remove_static_worker_scratch() {
  [[ -n "${STATIC_WORKER_SCRATCH}" ]] || return 0
  case "${STATIC_WORKER_SCRATCH}" in
    /tmp/robotest-phase4-worker.*) ;;
    *) die "Refusing unsafe static worker cleanup: ${STATIC_WORKER_SCRATCH}" ;;
  esac
  [[ -d "${STATIC_WORKER_SCRATCH}" && ! -L "${STATIC_WORKER_SCRATCH}" ]] ||
    die "Static worker scratch is missing or linked: ${STATIC_WORKER_SCRATCH}"
  rm -rf -- "${STATIC_WORKER_SCRATCH}"
  STATIC_WORKER_SCRATCH=""
}

sanitized_git() {
  [[ ! -e "${PROJECT_ROOT}/.git/info/attributes" &&
    ! -L "${PROJECT_ROOT}/.git/info/attributes" ]] ||
    die 'Candidate Git info attributes are forbidden.'
  env -i \
    PATH=/usr/bin:/bin \
    LANG=C \
    HOME=/nonexistent \
    GIT_ATTR_NOSYSTEM=1 \
    GIT_OPTIONAL_LOCKS=0 \
    GIT_CONFIG_NOSYSTEM=1 \
    GIT_CONFIG_GLOBAL=/dev/null \
    GIT_NO_REPLACE_OBJECTS=1 \
    git \
      -C "${PROJECT_ROOT}" \
      --git-dir="${PROJECT_ROOT}/.git" \
      --work-tree="${PROJECT_ROOT}" \
      -c safe.directory="${PROJECT_ROOT}" \
      -c core.attributesFile=/dev/null \
      -c core.fileMode=true \
      -c core.fsmonitor=false \
      -c core.hooksPath=/dev/null \
      -c core.bare=false \
      "$@"
}

ignored_source_path_is_allowed() {
  case "$1" in
    config/release-claims.json) return 0 ;;
    config/* | scenarios/* | src/*)
      case "$1" in
        */.pytest_cache | */.pytest_cache/* | */__pycache__ | */__pycache__/*)
          return 0
          ;;
        *) return 1 ;;
      esac
      ;;
    packaging/debian | packaging/debian/* | supervisor/cmd | supervisor/cmd/* | \
      supervisor/internal | supervisor/internal/*)
      return 1
      ;;
    docs/results | docs/results/* | */.mypy_cache | */.mypy_cache/* | \
      */.pytest_cache | */.pytest_cache/* | */__pycache__ | */__pycache__/* | \
      */artifacts | */artifacts/* | */build | */build/* | \
      */install | */install/* | */log | */log/*)
      return 0
      ;;
    *) return 1 ;;
  esac
}

verify_ignored_source_clean() {
  local ignored_path
  local -a ignored_paths=()
  local -a roots=(config docs packaging scenarios scripts src supervisor tests)
  sanitized_git ls-files --others --ignored --exclude-standard -- "${roots[@]}" \
    >/dev/null || die 'Cannot inspect ignored candidate source paths.'
  mapfile -d '' -t ignored_paths < <(
    sanitized_git ls-files -z --others --ignored --exclude-standard -- "${roots[@]}"
  )
  for ignored_path in "${ignored_paths[@]}"; do
    ignored_source_path_is_allowed "${ignored_path}" ||
      die "Ignored untracked source input is forbidden: ${ignored_path}"
  done
}

verify_apply_source_clean() {
  local index_flags
  local observed_head
  local status_output
  local submodule_output
  [[ -d "${PROJECT_ROOT}/.git" && ! -L "${PROJECT_ROOT}/.git" ]] ||
    die 'Candidate Git directory is missing or linked.'
  observed_head="$(sanitized_git rev-parse --verify HEAD)" ||
    die 'Cannot resolve the candidate Git HEAD.'
  [[ "${observed_head}" =~ ^[0-9a-f]{40}$ ]] ||
    die 'The candidate Git HEAD is not a canonical object ID.'
  status_output="$(sanitized_git \
    status --porcelain=v1 --untracked-files=all --ignore-submodules=none)" ||
    die 'Cannot inspect candidate Git status.'
  [[ -z "${status_output}" ]] || die 'Authoritative --apply requires an exactly clean source tree.'
  verify_ignored_source_clean
  index_flags="$(sanitized_git ls-files -v)" ||
    die 'Cannot inspect candidate Git index flags.'
  if grep -Eq '^[a-zS] ' <<<"${index_flags}"; then
    die 'Authoritative --apply forbids assume-unchanged and skip-worktree index flags.'
  fi
  submodule_output="$(sanitized_git submodule status --recursive)" ||
    die 'Cannot inspect candidate Git submodules.'
  if grep -Eq '^[+-U]' <<<"${submodule_output}"; then
    die 'Authoritative --apply requires initialized, unmodified submodules.'
  fi
  if [[ -z "${APPLY_SOURCE_HEAD}" ]]; then
    APPLY_SOURCE_HEAD="${observed_head}"
  else
    [[ "${observed_head}" == "${APPLY_SOURCE_HEAD}" ]] ||
      die 'Candidate Git HEAD changed during static verification.'
  fi
}

validated_remove_package_rebuild_scratch() {
  [[ -n "${PACKAGE_REBUILD_SCRATCH}" ]] || return 0
  case "${PACKAGE_REBUILD_SCRATCH}" in
    /tmp/robotest-phase4-rebuild.*) ;;
    *) die "Refusing unsafe package rebuild cleanup: ${PACKAGE_REBUILD_SCRATCH}" ;;
  esac
  [[ -d "${PACKAGE_REBUILD_SCRATCH}" && ! -L "${PACKAGE_REBUILD_SCRATCH}" ]] ||
    die "Package rebuild scratch is missing or linked: ${PACKAGE_REBUILD_SCRATCH}"
  rm -rf -- "${PACKAGE_REBUILD_SCRATCH}"
  PACKAGE_REBUILD_SCRATCH=""
}

resolve_package_directory() {
  local candidate
  if [[ -n "${PACKAGE_DIRECTORY}" ]]; then
    PACKAGE_DIRECTORY="$(realpath -e -- "${PACKAGE_DIRECTORY}")"
  else
    mapfile -t candidates < <(
      find "${PROJECT_ROOT}/artifacts/packages" -mindepth 1 -maxdepth 1 -type d \
        -name 'repro-*' -print | LC_ALL=C sort -r
    )
    for candidate in "${candidates[@]}"; do
      if [[ -f "${candidate}/reproducibility.json" ]] &&
        find "${candidate}" -maxdepth 1 -type f \
          -name 'robotest-supervisor_*_baseline_*.deb' -print -quit | grep -q .; then
        PACKAGE_DIRECTORY="$(realpath -e -- "${candidate}")"
        break
      fi
    done
  fi
  [[ -n "${PACKAGE_DIRECTORY}" ]] ||
    die 'No package candidate with reproducibility and baseline evidence was found.'
  case "${PACKAGE_DIRECTORY}" in
    "${PROJECT_ROOT}/artifacts/packages/"*) ;;
    *) die "Package directory is outside artifacts/packages: ${PACKAGE_DIRECTORY}" ;;
  esac

  mapfile -t upgrade_packages < <(
    find "${PACKAGE_DIRECTORY}/build-a" -mindepth 1 -maxdepth 1 -type f \
      -name 'robotest-supervisor_*.deb' -printf '%p\n' | LC_ALL=C sort
  )
  mapfile -t baseline_packages < <(
    find "${PACKAGE_DIRECTORY}" -mindepth 1 -maxdepth 1 -type f \
      -name 'robotest-supervisor_*_baseline_*.deb' -printf '%p\n' | LC_ALL=C sort
  )
  ((${#upgrade_packages[@]} == 1)) ||
    die "Expected one upgrade .deb, found ${#upgrade_packages[@]}."
  ((${#baseline_packages[@]} == 1)) ||
    die "Expected one baseline .deb, found ${#baseline_packages[@]}."
  UPGRADE_DEB="$(realpath -e -- "${upgrade_packages[0]}")"
  BASELINE_DEB="$(realpath -e -- "${baseline_packages[0]}")"
  PACKAGE_MANIFEST="$(realpath -e -- "${PACKAGE_DIRECTORY}/build-a/SOURCE-MANIFEST.json")"
  [[ -f "${BASELINE_DEB}.fixture.json" ]] ||
    die "Baseline provenance sidecar is missing: ${BASELINE_DEB}.fixture.json"
}

resolve_lifecycle_evidence() {
  if [[ -n "${LIFECYCLE_EVIDENCE}" ]]; then
    LIFECYCLE_EVIDENCE="$(realpath -e -- "${LIFECYCLE_EVIDENCE}")"
  else
    LIFECYCLE_EVIDENCE="$(python3 - \
      "${PROJECT_ROOT}/artifacts/evidence/phase4" "${UPGRADE_DEB}" <<'PY'
import hashlib
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
package = pathlib.Path(sys.argv[2])
digest = hashlib.sha256(package.read_bytes()).hexdigest()
candidates = sorted(
    root.glob('package-lifecycle*.json'),
    key=lambda path: (path.stat().st_mtime_ns, path.name),
    reverse=True,
)
for path in candidates:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError):
        continue
    if (
        value.get('schema_version') == 1
        and value.get('verdict') == 'PASS'
        and value.get('upgrade_package', {}).get('sha256') == digest
    ):
        print(path.resolve())
        raise SystemExit(0)
raise SystemExit('no PASS lifecycle evidence matches the selected package')
PY
)" || die 'No lifecycle evidence matches the selected package candidate.'
  fi
  case "${LIFECYCLE_EVIDENCE}" in
    "${PROJECT_ROOT}/artifacts/evidence/phase4/"*.json) ;;
    *) die "Lifecycle evidence is outside artifacts/evidence/phase4: ${LIFECYCLE_EVIDENCE}" ;;
  esac
}

check_lifecycle_evidence() {
  python3 "${HELPER}" validate-lifecycle \
    --evidence "${LIFECYCLE_EVIDENCE}" \
    --upgrade "${UPGRADE_DEB}" \
    --baseline "${BASELINE_DEB}"
}

check_package_candidate() {
  local binding_output="$1"
  local integrity_output="$2"
  [[ "$(dpkg-deb -f "${UPGRADE_DEB}" Package)" == "robotest-supervisor" ]] ||
    die 'Upgrade package has the wrong Package field.'
  [[ "$(dpkg-deb -f "${BASELINE_DEB}" Package)" == "robotest-supervisor" ]] ||
    die 'Baseline package has the wrong Package field.'
  dpkg --compare-versions \
    "$(dpkg-deb -f "${BASELINE_DEB}" Version)" lt \
    "$(dpkg-deb -f "${UPGRADE_DEB}" Version)" ||
    die 'Baseline package version is not lower than the candidate.'
  python3 "${HELPER}" verify-package-candidate \
    --package-directory "${PACKAGE_DIRECTORY}" \
    --upgrade "${UPGRADE_DEB}" \
    --baseline "${BASELINE_DEB}" \
    --repository "${PROJECT_ROOT}" \
    --output "${integrity_output}"
  if ! python3 "${HELPER}" package-binding \
    --repository "${PROJECT_ROOT}" \
    --manifest "${PACKAGE_MANIFEST}" \
    --output "${binding_output}"; then
    sed -n '1,160p' "${binding_output}" >&2 || true
    die 'Package candidate source manifest does not match current package source.'
  fi
}

check_extracted_candidate_config() {
  local extracted_root="${STATIC_TEMP}/candidate-root"
  local rendered_config="${STATIC_TEMP}/candidate-config.json"
  local actual_output="${STATIC_TEMP}/candidate-config-check.txt"
  local binary="${extracted_root}/usr/bin/robotest-supervisor"
  local template="${extracted_root}/etc/robotest-supervisor/config.json"
  install -d -m 0755 -- "${extracted_root}"
  dpkg-deb -x "${UPGRADE_DEB}" "${extracted_root}"
  [[ -x "${binary}" && -f "${binary}" && ! -L "${binary}" ]] ||
    die 'Extracted candidate supervisor binary is missing, linked, or not executable.'
  [[ -f "${template}" && ! -L "${template}" ]] ||
    die 'Extracted candidate supervisor config is missing or linked.'
  python3 "${HELPER}" render-config \
    --template "${template}" \
    --output "${rendered_config}" \
    --state-directory '/var/lib/robotest-supervisor/phase4-20000101T000000Z-1' \
    --ros-domain-id 177 \
    --gz-partition robotest_phase4_static_config_check
  "${binary}" --config "${rendered_config}" --check-config >"${actual_output}"
  printf 'configuration valid\n' | cmp -s - "${actual_output}" ||
    die 'Extracted candidate did not produce the canonical config-check result.'
}

run_static_checks_worker() {
  local rebuild_directory
  log 'Running bounded static, unit, concurrency, and package-source gates.'
  if [[ -z "${STATIC_TEMP}" ]]; then
    STATIC_TEMP="$(mktemp -d /tmp/robotest-phase4-static.XXXXXX)"
  fi
  [[ -d "${STATIC_TEMP}" && ! -L "${STATIC_TEMP}" &&
    "$(stat -c '%U:%a' -- "${STATIC_TEMP}")" == "$(id -un):700" ]] ||
    die "Static worker scratch ownership or mode is unsafe: ${STATIC_TEMP}"

  for command_name in bash cmp dpkg dpkg-deb env find flock git go gofmt python3 \
    realpath sha256sum shellcheck systemd-analyze; do
    require_command "${command_name}"
  done

  bash -n \
    "${SCRIPT_DIR}/build_debian_package.sh" \
    "${SCRIPT_DIR}/create_debian_upgrade_fixture.sh" \
    "${SCRIPT_DIR}/stage_runtime_overlay.sh" \
    "${SCRIPT_DIR}/test_debian_package_lifecycle.sh" \
    "${SCRIPT_DIR}/verify_debian_reproducibility.sh" \
    "${SCRIPT_DIR}/verify_phase4.sh" \
    "${PROJECT_ROOT}/packaging/debian/start-robotest-stack" \
    "${PROJECT_ROOT}/packaging/debian/tests/installed"
  shellcheck \
    "${SCRIPT_DIR}/build_debian_package.sh" \
    "${SCRIPT_DIR}/create_debian_upgrade_fixture.sh" \
    "${SCRIPT_DIR}/stage_runtime_overlay.sh" \
    "${SCRIPT_DIR}/test_debian_package_lifecycle.sh" \
    "${SCRIPT_DIR}/verify_debian_reproducibility.sh" \
    "${SCRIPT_DIR}/verify_phase4.sh" \
    "${PROJECT_ROOT}/packaging/debian/start-robotest-stack" \
    "${PROJECT_ROOT}/packaging/debian/tests/installed"
  PYTHONPYCACHEPREFIX="${STATIC_TEMP}/pycache" \
    python3 -m py_compile "${HELPER}" "${ACTIVE_GOAL_PROBE}"
  PYTHONPATH="${PROJECT_ROOT}/src/robotest_missions:${PROJECT_ROOT}/tests" \
    python3 -m pytest -q -p no:cacheprovider \
      "${PROJECT_ROOT}/tests/phase4_acceptance_test.py"

  (
    cd -- "${PROJECT_ROOT}/supervisor"
    export GOFLAGS='-p=4'
    [[ -z "$(gofmt -l .)" ]] || {
      gofmt -l . >&2
      exit 1
    }
    go test -race ./...
    go vet ./...
    go test -cover ./...
  )

  systemd-analyze verify \
    "${PROJECT_ROOT}/packaging/debian/robotest-supervisor.service" \
    >"${STATIC_TEMP}/systemd-analyze.log" 2>&1
  check_package_candidate \
    "${STATIC_TEMP}/package-binding.json" \
    "${STATIC_TEMP}/package-integrity.json"
  rebuild_directory="${STATIC_TEMP}/source-rebuild"
  ((EUID != 0)) || die 'Static worker must not execute as root.'
  "${SCRIPT_DIR}/build_debian_package.sh" "${rebuild_directory}"
  PACKAGE_SOURCE_REBUILD="${STATIC_TEMP}/package-source-rebuild.json"
  python3 "${HELPER}" package-source-rebuild \
    --repository "${PROJECT_ROOT}" \
    --package-directory "${PACKAGE_DIRECTORY}" \
    --upgrade "${UPGRADE_DEB}" \
    --rebuilt-directory "${rebuild_directory}" \
    --output "${PACKAGE_SOURCE_REBUILD}"
  check_extracted_candidate_config
  check_lifecycle_evidence
  python3 "${HELPER}" source-snapshot \
    --repository "${PROJECT_ROOT}" \
    --output "${STATIC_TEMP}/source-snapshot.json"
  log "Static gates PASS; package candidate: ${PACKAGE_DIRECTORY}"
}

run_static_checks_as_project_user() {
  local worker_output
  local worker_attestation
  local worker_rebuild
  local worker_file_count
  local worker_total_bytes
  STATIC_TEMP="$(mktemp -d /tmp/robotest-phase4-static.XXXXXX)"
  chmod 0700 -- "${STATIC_TEMP}"
  STATIC_WORKER_SCRATCH="$(mktemp -d /tmp/robotest-phase4-worker.XXXXXX)"
  worker_output="${STATIC_WORKER_SCRATCH}/output"
  for directory in home tmp cache go-cache go-tmp pycache output; do
    mkdir -m 0700 -- "${STATIC_WORKER_SCRATCH}/${directory}"
  done
  chown "${PROJECT_USER}:${PROJECT_GROUP}" -- \
    "${STATIC_WORKER_SCRATCH}" \
    "${STATIC_WORKER_SCRATCH}/home" \
    "${STATIC_WORKER_SCRATCH}/tmp" \
    "${STATIC_WORKER_SCRATCH}/cache" \
    "${STATIC_WORKER_SCRATCH}/go-cache" \
    "${STATIC_WORKER_SCRATCH}/go-tmp" \
    "${STATIC_WORKER_SCRATCH}/pycache" \
    "${worker_output}"
  runuser -u "${PROJECT_USER}" -- env -i \
    HOME="${STATIC_WORKER_SCRATCH}/home" \
    TMPDIR="${STATIC_WORKER_SCRATCH}/tmp" \
    XDG_CACHE_HOME="${STATIC_WORKER_SCRATCH}/cache" \
    GOCACHE="${STATIC_WORKER_SCRATCH}/go-cache" \
    GOTMPDIR="${STATIC_WORKER_SCRATCH}/go-tmp" \
    PYTHONPYCACHEPREFIX="${STATIC_WORKER_SCRATCH}/pycache" \
    LANG=C.UTF-8 \
    PATH=/usr/local/go/bin:/usr/bin:/bin \
    USER="${PROJECT_USER}" \
    LOGNAME="${PROJECT_USER}" \
    ROBOTEST_PHASE4_STATIC_WORKER=1 \
    ROBOTEST_PHASE4_STATIC_TEMP="${worker_output}" \
    bash "${SCRIPT_DIR}/verify_phase4.sh" \
      --package-dir "${PACKAGE_DIRECTORY}" \
      --lifecycle-evidence "${LIFECYCLE_EVIDENCE}"
  [[ -d "${STATIC_WORKER_SCRATCH}" && ! -L "${STATIC_WORKER_SCRATCH}" &&
    "$(stat -c '%U:%G:%a' -- "${STATIC_WORKER_SCRATCH}")" == "${PROJECT_USER}:${PROJECT_GROUP}:700" ]] ||
    die 'Static worker scratch ownership or mode changed.'
  if find "${STATIC_WORKER_SCRATCH}" -xdev -type l -print -quit | grep -q .; then
    die 'Static worker output contains a symbolic link.'
  fi
  if find "${STATIC_WORKER_SCRATCH}" -xdev \! -type d \! -type f -print -quit |
    grep -q .; then
    die 'Static worker output contains a special file.'
  fi
  if find "${STATIC_WORKER_SCRATCH}" -xdev -type f -links +1 -print -quit |
    grep -q .; then
    die 'Static worker output contains a multiply linked file.'
  fi
  if find "${STATIC_WORKER_SCRATCH}" -xdev \! -user "${PROJECT_USER}" -print -quit |
    grep -q .; then
    die 'Static worker output ownership changed.'
  fi
  if find "${STATIC_WORKER_SCRATCH}" -xdev -perm /022 -print -quit | grep -q .; then
    die 'Static worker output is group- or world-writable.'
  fi
  read -r worker_file_count worker_total_bytes < <(
    find "${STATIC_WORKER_SCRATCH}" -xdev -type f -printf '%s\n' |
      awk '{count += 1; bytes += $1} END {print count + 0, bytes + 0}'
  )
  ((worker_file_count <= 20000 && worker_total_bytes <= 1073741824)) ||
    die 'Static worker output exceeds its file-count or byte cap.'
  worker_attestation="${worker_output}/package-source-rebuild.json"
  worker_rebuild="${worker_output}/source-rebuild"
  [[ -f "${worker_attestation}" && ! -L "${worker_attestation}" &&
    "$(stat -c '%h:%s' -- "${worker_attestation}")" =~ ^1:[1-9][0-9]{0,6}$ ]] ||
    die 'Static worker rebuild attestation is unsafe or oversized.'
  [[ -d "${worker_rebuild}" && ! -L "${worker_rebuild}" ]] ||
    die 'Static worker rebuild output is missing or linked.'
  PACKAGE_SOURCE_REBUILD="${STATIC_TEMP}/package-source-rebuild.json"
  python3 "${HELPER}" import-package-source-rebuild \
    --repository "${PROJECT_ROOT}" \
    --package-directory "${PACKAGE_DIRECTORY}" \
    --upgrade "${UPGRADE_DEB}" \
    --rebuilt-directory "${worker_rebuild}" \
    --attestation "${worker_attestation}" \
    --output "${PACKAGE_SOURCE_REBUILD}"
  [[ -f "${PACKAGE_SOURCE_REBUILD}" && ! -L "${PACKAGE_SOURCE_REBUILD}" &&
    "$(stat -c '%U:%G:%a:%h' -- "${PACKAGE_SOURCE_REBUILD}")" == 'root:root:644:1' ]] ||
    die 'Imported rebuild attestation ownership or mode is unsafe.'
  validated_remove_static_worker_scratch
}

remove_overlay_stage_scratch() {
  [[ -n "${OVERLAY_STAGE_SCRATCH}" ]] || return 0
  [[ "${OVERLAY_STAGE_SCRATCH}" == "${RUN_DIRECTORY}/overlay-stage-scratch" ]] || return 1
  [[ ! -e "${OVERLAY_STAGE_SCRATCH}" ]] && return 0
  [[ -d "${OVERLAY_STAGE_SCRATCH}" && ! -L "${OVERLAY_STAGE_SCRATCH}" &&
    "$(stat -c '%U:%G:%a' -- "${OVERLAY_STAGE_SCRATCH}")" == 'root:root:700' ]] ||
    return 1
  if find "${OVERLAY_STAGE_SCRATCH}" -xdev -type l -print -quit | grep -q .; then
    return 1
  fi
  if find "${OVERLAY_STAGE_SCRATCH}" -xdev \! -type d \! -type f -print -quit |
    grep -q .; then
    return 1
  fi
  if find "${OVERLAY_STAGE_SCRATCH}" -xdev -type f -links +1 -print -quit |
    grep -q .; then
    return 1
  fi
  rm -rf -- "${OVERLAY_STAGE_SCRATCH}"
}

stage_runtime_overlay_safely() {
  local result
  OVERLAY_STAGE_SCRATCH="${RUN_DIRECTORY}/overlay-stage-scratch"
  OVERLAY_STAGE_SCRIPT="${OVERLAY_STAGE_SCRATCH}/stage_runtime_overlay.sh"
  OVERLAY_STAGE_EVIDENCE="${OVERLAY_STAGE_SCRATCH}/evidence"
  [[ ! -e "${OVERLAY_STAGE_SCRATCH}" && ! -L "${OVERLAY_STAGE_SCRATCH}" ]] || return 1
  mkdir -m 0700 -- "${OVERLAY_STAGE_SCRATCH}"
  [[ "$(stat -c '%U:%G:%a' -- "${OVERLAY_STAGE_SCRATCH}")" == 'root:root:700' ]] ||
    return 1
  python3 "${HELPER}" render-overlay-stage-script \
    --template "${SCRIPT_DIR}/stage_runtime_overlay.sh" \
    --output "${OVERLAY_STAGE_SCRIPT}" \
    --project-root "${PROJECT_ROOT}" \
    --evidence-directory "${OVERLAY_STAGE_EVIDENCE}"
  [[ -f "${OVERLAY_STAGE_SCRIPT}" && ! -L "${OVERLAY_STAGE_SCRIPT}" &&
    "$(stat -c '%U:%G:%a:%h' -- "${OVERLAY_STAGE_SCRIPT}")" == 'root:root:700:1' ]] ||
    return 1
  "${OVERLAY_STAGE_SCRIPT}" --apply
  "${OVERLAY_STAGE_SCRIPT}" --check
  result="${OVERLAY_STAGE_EVIDENCE}/runtime-staging.json"
  [[ "$(realpath -e -- "${result}")" == "${result}" && -f "${result}" &&
    ! -L "${result}" && "$(stat -c '%h' -- "${result}")" == 1 ]] || return 1
  python3 - "${result}" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
payload = path.read_bytes()
if not payload or len(payload) > 8 * 1024 * 1024 or not payload.endswith(b'\n'):
    raise SystemExit('runtime staging evidence is empty, oversized, or incomplete')
value = json.loads(payload)
if not isinstance(value, dict) or value.get('schema_version') != 1:
    raise SystemExit('runtime staging evidence is not the expected JSON object')
PY
  install -m 0644 -o root -g root -- "${result}" "${RUN_DIRECTORY}/runtime-staging.json"
  remove_overlay_stage_scratch
}

capture_stable_runtime_ownership() {
  local phase="$1"
  local main_pid="$2"
  local child_pid="$3"
  local child_pgid="$4"
  local owners_output="$5"
  local affinity_output="$6"
  local owners_before="${RUN_DIRECTORY}/.${phase}-owners-before.json"
  local owners_after="${RUN_DIRECTORY}/.${phase}-owners-after.json"
  local affinity_candidate="${RUN_DIRECTORY}/.${phase}-affinity.json"
  local _attempt
  for _attempt in {1..20}; do
    rm -f -- "${owners_before}" "${owners_after}" "${affinity_candidate}"
    if python3 "${HELPER}" systemd-owners \
      --ros-domain-id "${ROS_DOMAIN_ID}" --gz-partition "${GZ_PARTITION}" \
      --output "${owners_before}" >/dev/null &&
      python3 "${HELPER}" capture-runtime-affinity \
        --main-pid "${main_pid}" \
        --managed-child-pid "${child_pid}" \
        --managed-child-pgid "${child_pgid}" \
        --ros-domain-id "${ROS_DOMAIN_ID}" \
        --gz-partition "${GZ_PARTITION}" \
        --phase "${phase}" \
        --expected-cpuset 0-5 \
        --output "${affinity_candidate}" >/dev/null &&
      python3 "${HELPER}" systemd-owners \
        --ros-domain-id "${ROS_DOMAIN_ID}" --gz-partition "${GZ_PARTITION}" \
        --output "${owners_after}" >/dev/null &&
      python3 "${HELPER}" validate-runtime-ownership \
        --before "${owners_before}" \
        --after "${owners_after}" \
        --affinity "${affinity_candidate}"; then
      mv --no-target-directory -- "${owners_after}" "${owners_output}"
      mv --no-target-directory -- "${affinity_candidate}" "${affinity_output}"
      rm -f -- "${owners_before}"
      python3 - "${owners_output}" "${affinity_output}" <<'PY'
import json
import pathlib
import sys

owners = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding='utf-8'))
affinity = json.loads(pathlib.Path(sys.argv[2]).read_text(encoding='utf-8'))
print(owners['unit_count'], affinity['process_count'])
PY
      return 0
    fi
    rm -f -- "${owners_before}" "${owners_after}" "${affinity_candidate}"
    sleep 0.05
  done
  log "Runtime ownership did not stabilize for ${phase}."
  return 1
}

record_timeline() {
  local kind="$1"
  shift
  local -a command=(python3 "${HELPER}" record --timeline "${TIMELINE}" --kind "${kind}")
  local detail
  for detail in "$@"; do
    command+=(--detail "${detail}")
  done
  "${command[@]}" >/dev/null
}

last_supervisor_event_sequence() {
  python3 - "${STATE_RUN_DIRECTORY}/events.jsonl" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
if path.is_symlink() or not path.is_file() or path.stat().st_size > 32 * 1024 * 1024:
    raise SystemExit('supervisor event log is missing, linked, or oversized')
lines = path.read_text(encoding='utf-8').splitlines()
if not lines:
    raise SystemExit('supervisor event log is empty')
value = json.loads(lines[-1])
sequence = value.get('sequence')
if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 1:
    raise SystemExit('supervisor event sequence is invalid')
print(sequence)
PY
}

group_exists() {
  local pgid="$1"
  [[ "${pgid}" =~ ^[0-9]+$ && "${pgid}" -gt 1 ]] || return 1
  kill -0 "-${pgid}" 2>/dev/null
}

captured_pgid() {
  local pid="$1"
  local pgid=""
  for _attempt in {1..50}; do
    pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ' || true)"
    if [[ "${pgid}" =~ ^[0-9]+$ ]]; then
      printf '%s\n' "${pgid}"
      return 0
    fi
    sleep 0.02
  done
  return 1
}

latch_process_publication_signal() {
  local status="$1"
  if ((PENDING_PUBLICATION_SIGNAL == 0)); then
    PENDING_PUBLICATION_SIGNAL="${status}"
  fi
}

begin_process_publication() {
  local hup_trap int_trap term_trap
  hup_trap="$(trap -p HUP)"
  int_trap="$(trap -p INT)"
  term_trap="$(trap -p TERM)"
  if [[ -z "${hup_trap}" && -z "${int_trap}" && -z "${term_trap}" ]]; then
    PUBLICATION_SIGNAL_TRAPS_CLEARED=1
  else
    PUBLICATION_SIGNAL_TRAPS_CLEARED=0
  fi
  PENDING_PUBLICATION_SIGNAL=0
  trap 'latch_process_publication_signal 129' HUP
  trap 'latch_process_publication_signal 130' INT
  trap 'latch_process_publication_signal 143' TERM
}

end_process_publication() {
  local pending_signal
  if ((PUBLICATION_SIGNAL_TRAPS_CLEARED)); then
    trap - HUP INT TERM
  else
    trap 'exit 129' HUP
    trap 'exit 130' INT
    trap 'exit 143' TERM
  fi
  PUBLICATION_SIGNAL_TRAPS_CLEARED=0
  pending_signal="${PENDING_PUBLICATION_SIGNAL}"
  PENDING_PUBLICATION_SIGNAL=0
  ((pending_signal == 0)) || exit "${pending_signal}"
}

stop_owned_group() {
  local label="$1"
  local pid="$2"
  local pgid="$3"
  local shell_pgid deadline
  [[ -n "${pgid}" ]] || return 0
  [[ "${pgid}" =~ ^[0-9]+$ && "${pgid}" -gt 1 ]] || {
    log "Refusing invalid ${label} PGID: ${pgid}"
    return 1
  }
  shell_pgid="$(ps -o pgid= -p $$ | tr -d ' ')"
  [[ "${pgid}" != "${shell_pgid}" ]] || {
    log "Refusing to signal the verifier's own process group for ${label}."
    return 1
  }
  if group_exists "${pgid}"; then
    kill -TERM "-${pgid}" 2>/dev/null || true
    deadline=$((SECONDS + 5))
    while group_exists "${pgid}" && ((SECONDS < deadline)); do
      sleep 0.05
    done
  fi
  if group_exists "${pgid}"; then
    kill -KILL "-${pgid}" 2>/dev/null || true
    deadline=$((SECONDS + 2))
    while group_exists "${pgid}" && ((SECONDS < deadline)); do
      sleep 0.05
    done
  fi
  if [[ -n "${pid}" ]]; then
    wait "${pid}" 2>/dev/null || true
  fi
  ! group_exists "${pgid}"
}

copy_supervisor_state() {
  [[ -n "${RUN_DIRECTORY}" && -d "${RUN_DIRECTORY}" ]] || return 0
  [[ -n "${STATE_RUN_DIRECTORY}" && -d "${STATE_RUN_DIRECTORY}" ]] || return 0
  local source destination
  for source in events.jsonl events.meta.json status.json lifecycle-startup-result.json; do
    [[ -f "${STATE_RUN_DIRECTORY}/${source}" &&
      ! -L "${STATE_RUN_DIRECTORY}/${source}" ]] || continue
    case "${source}" in
      events.jsonl) destination=supervisor-events.jsonl ;;
      events.meta.json) destination=supervisor-events.meta.json ;;
      status.json) destination=supervisor-status-final.json ;;
      lifecycle-startup-result.json) destination=lifecycle-startup-result.json ;;
    esac
    install -m 0644 -- "${STATE_RUN_DIRECTORY}/${source}" \
      "${RUN_DIRECTORY}/${destination}"
  done
}

write_service_final() {
  [[ -n "${RUN_DIRECTORY}" && -d "${RUN_DIRECTORY}" ]] || return 0
  [[ ! -e "${RUN_DIRECTORY}/service-final.json" ]] || return 0
  local main_pid=0 nrestarts=0 active=false enabled=false
  if systemctl show "${SERVICE}" >/dev/null 2>&1; then
    main_pid="$(systemctl show --property=MainPID --value "${SERVICE}" 2>/dev/null || printf 0)"
    nrestarts="$(systemctl show --property=NRestarts --value "${SERVICE}" 2>/dev/null || printf 0)"
    if systemctl is-active --quiet "${SERVICE}"; then
      active=true
    fi
    if [[ "$(systemctl is-enabled "${SERVICE}" 2>/dev/null || true)" == "enabled" ]]; then
      enabled=true
    fi
  fi
  python3 - "${RUN_DIRECTORY}/service-final.json" \
    "${main_pid}" "${nrestarts}" "${active}" "${enabled}" <<'PY'
import json
import os
import pathlib
import sys
import tempfile
from datetime import datetime, timezone

path = pathlib.Path(sys.argv[1])
value = {
    'schema_version': 1,
    'captured_utc': datetime.now(timezone.utc).isoformat(),
    'main_pid_before_stop': int(sys.argv[2] or 0),
    'nrestarts_before_stop': int(sys.argv[3] or 0),
    'active_before_stop': sys.argv[4] == 'true',
    'enabled_before_stop': sys.argv[5] == 'true',
}
descriptor, name = tempfile.mkstemp(prefix='.service-final.', dir=path.parent)
with os.fdopen(descriptor, 'w', encoding='utf-8') as target:
    json.dump(value, target, separators=(',', ':'), sort_keys=True)
    target.write('\n')
    target.flush()
    os.fsync(target.fileno())
os.replace(name, path)
PY
}

remove_owned_state_path() {
  local path="$1"
  local marker_required="$2"
  [[ -n "${path}" && -e "${path}" ]] || return 0
  case "${path}" in
    /var/lib/robotest-supervisor/phase4-*) ;;
    *) log "Refusing unsafe state cleanup: ${path}"; return 1 ;;
  esac
  [[ ! -L "${path}" && -d "${path}" ]] || return 1
  if [[ "${marker_required}" == true ]]; then
    [[ -f "${path}/.phase4-owned" && ! -L "${path}/.phase4-owned" &&
      "$(<"${path}/.phase4-owned")" == "${RUN_ID}" ]] || {
      log 'Refusing state cleanup without the exact run ownership marker.'
      return 1
    }
  else
    [[ "${path}" == "${STATE_STAGING_DIRECTORY}" ]] || return 1
  fi
  if find "${path}" -type l -print -quit | grep -q .; then
    log 'Refusing state cleanup because the owned directory contains a symlink.'
    return 1
  fi
  rm -rf -- "${path}"
}

remove_owned_state_directories() {
  remove_owned_state_path "${STATE_STAGING_DIRECTORY}" false || return 1
  remove_owned_state_path "${STATE_RUN_DIRECTORY}" true
}

remove_owned_dropin() {
  if [[ -n "${DROPIN_STAGING_FILE}" && -e "${DROPIN_STAGING_FILE}" ]]; then
    [[ "${DROPIN_STAGING_FILE}" == "${DROPIN_DIRECTORY}/.phase4-${RUN_ID}.tmp" &&
      -f "${DROPIN_STAGING_FILE}" && ! -L "${DROPIN_STAGING_FILE}" ]] || return 1
    rm -f -- "${DROPIN_STAGING_FILE}"
  fi
  if [[ ! -e "${DROPIN_FILE}" ]]; then
    if ((DROPIN_DIRECTORY_CREATED)); then
      rmdir -- "${DROPIN_DIRECTORY}" 2>/dev/null || true
    fi
    return 0
  fi
  [[ -f "${DROPIN_FILE}" && ! -L "${DROPIN_FILE}" ]] || return 1
  [[ -n "${DROPIN_SHA256}" &&
    "$(sha256sum "${DROPIN_FILE}" | awk '{print $1}')" == "${DROPIN_SHA256}" ]] || {
    log 'Refusing to remove a changed or unowned systemd drop-in.'
    return 1
  }
  rm -f -- "${DROPIN_FILE}"
  if ((DROPIN_DIRECTORY_CREATED)); then
    rmdir -- "${DROPIN_DIRECTORY}" 2>/dev/null || true
  fi
}

remove_owned_runtime_file() {
  local path="$1"
  [[ ! -e "${path}" ]] && return 0
  [[ -f "${path}" && ! -L "${path}" ]] || return 1
  rm -f -- "${path}"
}

freeze_live_evidence() {
  [[ "${RUN_DIRECTORY}" == "${LIVE_RUN_ROOT}/${RUN_ID}" ]] || {
    log "Refusing to freeze an unexpected live evidence path: ${RUN_DIRECTORY}"
    return 1
  }
  [[ -d "${LIVE_RUN_ROOT}" && ! -L "${LIVE_RUN_ROOT}" &&
    "$(stat -c '%U:%G:%a' -- "${LIVE_RUN_ROOT}")" == 'root:root:700' ]] || {
    log 'Live evidence root is not the exact root:root 0700 directory.'
    return 1
  }
  [[ -d "${RUN_DIRECTORY}" && ! -L "${RUN_DIRECTORY}" &&
    "$(stat -c '%U:%G:%a' -- "${RUN_DIRECTORY}")" == 'root:root:700' ]] || {
    log 'Live evidence leaf is not the exact root:root 0700 directory.'
    return 1
  }
  [[ -f "${RUN_DIRECTORY}/.phase4-live-owned" &&
    ! -L "${RUN_DIRECTORY}/.phase4-live-owned" &&
    "$(<"${RUN_DIRECTORY}/.phase4-live-owned")" == "${RUN_ID}" ]] || {
    log 'Live evidence ownership marker is absent or invalid.'
    return 1
  }
  if find "${RUN_DIRECTORY}" -xdev \( -type l -o \! -user root \) -print -quit |
    grep -q .; then
    log 'Live evidence contains a symlink or non-root-owned entry.'
    return 1
  fi
  if find "${RUN_DIRECTORY}" -xdev \! -type d \! -type f -print -quit | grep -q .; then
    log 'Live evidence contains a special filesystem entry.'
    return 1
  fi
  if find "${RUN_DIRECTORY}" -xdev -type f -links +1 -print -quit | grep -q .; then
    log 'Live evidence contains a hard-linked file.'
    return 1
  fi
  chown -R "root:${PROJECT_GROUP}" -- "${RUN_DIRECTORY}"
  find "${RUN_DIRECTORY}" -xdev -type d -exec chmod 0550 -- {} +
  find "${RUN_DIRECTORY}" -xdev -type f -exec chmod 0440 -- {} +
  chown "root:${PROJECT_GROUP}" -- "${LIVE_RUN_ROOT}"
  chmod 0710 -- "${LIVE_RUN_ROOT}"
  [[ "$(stat -c '%U:%G:%a' -- "${LIVE_RUN_ROOT}")" == "root:${PROJECT_GROUP}:710" ]] ||
    return 1
  [[ "$(stat -c '%U:%G:%a' -- "${RUN_DIRECTORY}")" == "root:${PROJECT_GROUP}:550" ]] ||
    return 1
}

publish_live_evidence() {
  [[ "${RUN_DIRECTORY}" == "${LIVE_RUN_ROOT}/${RUN_ID}" ]] || return 1
  [[ "${PUBLIC_RUN_DIRECTORY}" == "${PROJECT_ROOT}/artifacts/evidence/phase4/runs/${RUN_ID}" ]] ||
    return 1
  runuser -u "${PROJECT_USER}" -- env -i \
    HOME="${PROJECT_ROOT}" \
    LANG=C.UTF-8 \
    PATH=/usr/bin:/bin \
    PYTHONDONTWRITEBYTECODE=1 \
    /usr/bin/python3 "${HELPER}" publish-evidence \
    --source "${RUN_DIRECTORY}" \
    --destination "${PUBLIC_RUN_DIRECTORY}" \
    --repository "${PROJECT_ROOT}"
}

remove_published_live_evidence() {
  ((SERVICE_INACTIVITY_PROVEN)) || return 1
  ((PUBLICATION_STATUS == 0)) || return 1
  [[ "${RUN_DIRECTORY}" == "${LIVE_RUN_ROOT}/${RUN_ID}" ]] || return 1
  [[ -d "${RUN_DIRECTORY}" && ! -L "${RUN_DIRECTORY}" ]] || return 1
  [[ -f "${RUN_DIRECTORY}/.phase4-live-owned" &&
    ! -L "${RUN_DIRECTORY}/.phase4-live-owned" &&
    "$(<"${RUN_DIRECTORY}/.phase4-live-owned")" == "${RUN_ID}" ]] || return 1
  if find "${RUN_DIRECTORY}" -xdev -type l -print -quit | grep -q .; then
    return 1
  fi
  rm -rf -- "${RUN_DIRECTORY}"
  chown root:root -- "${LIVE_RUN_ROOT}"
  chmod 0700 -- "${LIVE_RUN_ROOT}"
  [[ ! -e "${RUN_DIRECTORY}" && ! -L "${RUN_DIRECTORY}" ]] || return 1
  LIVE_RUN_OWNED=0
}

write_cleanup_evidence() {
  local source_unchanged=false service_inactive=false service_disabled=false
  local dropin_absent=false state_absent=false heartbeat_absent=false
  local startup_absent=false mission_empty=false original_empty=false daemon_reloaded=false
  local interrupted_empty=false followup_empty=false probe_empty=false
  local replacement_empty=false

  [[ ! -e "${RUN_DIRECTORY}/cleanup.json" ]] || return 0

  python3 "${HELPER}" source-snapshot \
    --repository "${PROJECT_ROOT}" \
    --output "${RUN_DIRECTORY}/source-snapshot-after.json" || true
  if [[ -f "${RUN_DIRECTORY}/source-snapshot-before.json" &&
    -f "${RUN_DIRECTORY}/source-snapshot-after.json" ]]; then
    if python3 - "${RUN_DIRECTORY}/source-snapshot-before.json" \
      "${RUN_DIRECTORY}/source-snapshot-after.json" <<'PY'
import json
import pathlib
import sys

left = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding='utf-8'))
right = json.loads(pathlib.Path(sys.argv[2]).read_text(encoding='utf-8'))
raise SystemExit(0 if left.get('snapshot_sha256') == right.get('snapshot_sha256') else 1)
PY
    then
      source_unchanged=true
    fi
  fi
  ((SERVICE_INACTIVITY_PROVEN)) && service_inactive=true
  [[ "$(systemctl is-enabled "${SERVICE}" 2>/dev/null || true)" != "enabled" ]] &&
    service_disabled=true
  [[ ! -e "${DROPIN_FILE}" &&
    (-z "${DROPIN_STAGING_FILE}" || ! -e "${DROPIN_STAGING_FILE}") ]] &&
    dropin_absent=true
  [[ (-z "${STATE_RUN_DIRECTORY}" || ! -e "${STATE_RUN_DIRECTORY}") &&
    (-z "${STATE_STAGING_DIRECTORY}" || ! -e "${STATE_STAGING_DIRECTORY}") ]] &&
    state_absent=true
  [[ ! -e "${HEARTBEAT}" ]] && heartbeat_absent=true
  [[ ! -e "${STARTUP_RESULT}" ]] && startup_absent=true
  if [[ -n "${MISSION_LAST_PGID}" ]] &&
    python3 "${HELPER}" process-group --pgid "${MISSION_LAST_PGID}" \
      --require-empty >/dev/null; then
    interrupted_empty=true
  fi
  if [[ -n "${FOLLOWUP_LAST_PGID}" ]] &&
    python3 "${HELPER}" process-group --pgid "${FOLLOWUP_LAST_PGID}" \
      --require-empty >/dev/null; then
    followup_empty=true
  fi
  if [[ -n "${PROBE_LAST_PGID}" ]] &&
    python3 "${HELPER}" process-group --pgid "${PROBE_LAST_PGID}" \
      --require-empty >/dev/null; then
    probe_empty=true
  fi
  if [[ "${interrupted_empty}" == true && "${followup_empty}" == true &&
    "${probe_empty}" == true ]]; then
    mission_empty=true
  fi
  if [[ -n "${ORIGINAL_PGID}" ]] &&
    python3 "${HELPER}" process-group --pgid "${ORIGINAL_PGID}" \
      --require-empty >/dev/null; then
    original_empty=true
  fi
  if [[ -n "${RESTORED_PGID}" ]] &&
    python3 "${HELPER}" process-group --pgid "${RESTORED_PGID}" \
      --require-empty >/dev/null; then
    replacement_empty=true
  fi
  ((DAEMON_RELOADED)) && daemon_reloaded=true
  python3 - "${RUN_DIRECTORY}/cleanup.json" \
    "${service_inactive}" "${service_disabled}" "${dropin_absent}" \
    "${state_absent}" "${heartbeat_absent}" "${startup_absent}" \
    "${mission_empty}" "${original_empty}" "${daemon_reloaded}" \
    "${source_unchanged}" "${interrupted_empty}" "${followup_empty}" \
    "${probe_empty}" "${replacement_empty}" \
    "${MISSION_LAST_PGID:-0}" "${FOLLOWUP_LAST_PGID:-0}" \
    "${PROBE_LAST_PGID:-0}" "${ORIGINAL_PGID:-0}" "${RESTORED_PGID:-0}" <<'PY'
import json
import os
import pathlib
import sys
import tempfile
from datetime import datetime, timezone

path = pathlib.Path(sys.argv[1])
names = (
    'service_inactive', 'service_disabled', 'dropin_absent',
    'run_state_directory_absent', 'heartbeat_absent', 'startup_result_absent',
    'mission_process_group_empty', 'original_process_group_empty',
    'daemon_reloaded', 'source_unchanged',
    'interrupted_process_group_empty', 'followup_process_group_empty',
    'probe_process_group_empty', 'replacement_process_group_empty',
)
value = {
    'schema_version': 1,
    'captured_utc': datetime.now(timezone.utc).isoformat(),
    'owned_paths_only': True,
    **{name: raw == 'true' for name, raw in zip(names, sys.argv[2:16])},
    'interrupted_last_pgid': int(sys.argv[16]),
    'followup_last_pgid': int(sys.argv[17]),
    'probe_last_pgid': int(sys.argv[18]),
    'original_child_pgid': int(sys.argv[19]),
    'replacement_child_pgid': int(sys.argv[20]),
}
descriptor, temporary = tempfile.mkstemp(prefix='.cleanup.', dir=path.parent)
with os.fdopen(descriptor, 'w', encoding='utf-8') as target:
    json.dump(value, target, separators=(',', ':'), sort_keys=True)
    target.write('\n')
    target.flush()
    os.fsync(target.fileno())
os.replace(temporary, path)
PY
}

prove_service_inactive() {
  local active_state main_pid
  active_state="$(systemctl show --property=ActiveState --value "${SERVICE}" 2>/dev/null)" ||
    return 1
  main_pid="$(systemctl show --property=MainPID --value "${SERVICE}" 2>/dev/null)" ||
    return 1
  [[ "${active_state}" == "inactive" && "${main_pid}" == 0 ]]
}

finalize_live_cleanup() {
  local cleanup_status=0
  ((LIVE_RUN_OWNED)) || return 1
  [[ -n "${RUN_DIRECTORY}" && -d "${RUN_DIRECTORY}" ]] || return 1

  remove_overlay_stage_scratch || cleanup_status=1

  if [[ -n "${PROBE_PGID}" ]]; then
    stop_owned_group active-goal-probe "${PROBE_PID}" "${PROBE_PGID}" || cleanup_status=1
    PROBE_LAST_PGID="${PROBE_PGID}"
    PROBE_PID=""
    PROBE_PGID=""
  fi
  if [[ -n "${MISSION_PGID}" ]]; then
    stop_owned_group interrupted-mission "${MISSION_PID}" "${MISSION_PGID}" || cleanup_status=1
    MISSION_LAST_PGID="${MISSION_PGID}"
    MISSION_PID=""
    MISSION_PGID=""
  fi
  if [[ -n "${FOLLOWUP_PGID}" ]]; then
    stop_owned_group followup-mission "${FOLLOWUP_PID}" "${FOLLOWUP_PGID}" || cleanup_status=1
    FOLLOWUP_LAST_PGID="${FOLLOWUP_PGID}"
    FOLLOWUP_PID=""
    FOLLOWUP_PGID=""
  fi

  write_service_final || cleanup_status=1
  EVENT_SEQUENCE_BEFORE_STOP="$(last_supervisor_event_sequence)" || {
    EVENT_SEQUENCE_BEFORE_STOP=0
    cleanup_status=1
  }
  if ((SERVICE_STARTED)) || systemctl is-active --quiet "${SERVICE}"; then
    timeout --signal=TERM --kill-after=5s 40s systemctl stop "${SERVICE}" || cleanup_status=1
  fi
  if prove_service_inactive; then
    SERVICE_INACTIVITY_PROVEN=1
    SERVICE_STARTED=0
    copy_supervisor_state || cleanup_status=1
    journalctl --unit "${SERVICE}" --no-pager --output=short-iso-precise \
      --since '-30 minutes' >"${RUN_DIRECTORY}/journal.log" 2>&1 || true
    systemctl disable "${SERVICE}" >/dev/null 2>&1 || cleanup_status=1
    remove_owned_dropin || cleanup_status=1
    if systemctl daemon-reload; then
      DAEMON_RELOADED=1
    else
      cleanup_status=1
    fi
    remove_owned_state_directories || cleanup_status=1
    remove_owned_runtime_file "${HEARTBEAT}" || cleanup_status=1
    remove_owned_runtime_file "${STARTUP_RESULT}" || cleanup_status=1
  else
    log 'Service inactivity was not proven; retaining all runtime resources.'
    cleanup_status=1
  fi

  if ((SERVICE_INACTIVITY_PROVEN)) && [[ -n "${TIMELINE}" && -f "${TIMELINE}" ]]; then
    if ! python3 - "${TIMELINE}" <<'PY'
import json
import pathlib
import sys

records = [json.loads(line) for line in pathlib.Path(sys.argv[1]).read_text().splitlines()]
raise SystemExit(0 if any(item.get('kind') == 'service_stopped' for item in records) else 1)
PY
    then
      record_timeline service_stopped \
        "event_sequence_before_stop=${EVENT_SEQUENCE_BEFORE_STOP}"
    fi
  fi
  write_cleanup_evidence || cleanup_status=1
  if [[ -n "${TIMELINE}" && -f "${TIMELINE}" ]]; then
    if ! python3 - "${TIMELINE}" <<'PY'
import json
import pathlib
import sys

records = [json.loads(line) for line in pathlib.Path(sys.argv[1]).read_text().splitlines()]
raise SystemExit(0 if any(item.get('kind') == 'cleanup_complete' for item in records) else 1)
PY
    then
      record_timeline cleanup_complete
    fi
  fi
  return "${cleanup_status}"
}

finalize_authoritative_run() {
  local finalization_status
  begin_process_publication
  if ((FINALIZATION_STATE == 2)); then
    finalization_status="${FINALIZATION_STATUS}"
    end_process_publication
    return "${finalization_status}"
  fi
  if ((FINALIZATION_STATE == 1)); then
    log 'Refusing re-entrant finalization.'
    finalization_status=1
    end_process_publication
    return "${finalization_status}"
  fi
  FINALIZATION_STATE=1
  if finalize_live_cleanup; then CLEANUP_STATUS=0; else CLEANUP_STATUS=$?; fi
  if python3 "${HELPER}" compose --run-directory "${RUN_DIRECTORY}"; then
    COMPOSE_STATUS=0
  else
    COMPOSE_STATUS=$?
  fi
  if python3 "${HELPER}" checksums --run-directory "${RUN_DIRECTORY}"; then
    CHECKSUM_STATUS=0
  else
    CHECKSUM_STATUS=$?
  fi
  if freeze_live_evidence; then
    FREEZE_STATUS=0
  else
    FREEZE_STATUS=$?
  fi
  if ((FREEZE_STATUS == 0)) && publish_live_evidence; then
    PUBLICATION_STATUS=0
    log "Evidence published atomically: ${PUBLIC_RUN_DIRECTORY}"
  else
    PUBLICATION_STATUS=$?
    ((PUBLICATION_STATUS != 0)) || PUBLICATION_STATUS=1
  fi
  LIVE_REMOVAL_STATUS=0
  if ((CLEANUP_STATUS == 0 && CHECKSUM_STATUS == 0 &&
    FREEZE_STATUS == 0 && PUBLICATION_STATUS == 0)); then
    if remove_published_live_evidence; then
      LIVE_REMOVAL_STATUS=0
    else
      LIVE_REMOVAL_STATUS=$?
      log "Retaining live evidence after removal failure: ${RUN_DIRECTORY}"
    fi
  else
    log "Retaining live evidence after incomplete finalization: ${RUN_DIRECTORY}"
  fi
  FINALIZATION_STATUS=0
  if ((CLEANUP_STATUS != 0 || COMPOSE_STATUS != 0 || CHECKSUM_STATUS != 0 ||
    FREEZE_STATUS != 0 || PUBLICATION_STATUS != 0 || LIVE_REMOVAL_STATUS != 0)); then
    FINALIZATION_STATUS=1
  fi
  FINALIZATION_STATE=2
  FINALIZED=1
  finalization_status="${FINALIZATION_STATUS}"
  end_process_publication
  return "${finalization_status}"
}

on_exit() {
  local status=$?
  trap - EXIT HUP INT TERM
  ((EXIT_HANDLER_ACTIVE == 0)) || exit "${status}"
  EXIT_HANDLER_ACTIVE=1
  set +e
  if ((APPLY)) && ((FINALIZED == 0)) && ((LIVE_RUN_OWNED)); then
    finalize_authoritative_run
    local finalization_status=$?
    if ((status == 0 && finalization_status != 0)); then
      status=1
    fi
  fi
  validated_remove_static_temp
  validated_remove_static_worker_scratch
  validated_remove_package_rebuild_scratch
  exit "${status}"
}

trap on_exit EXIT
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

while (($#)); do
  case "$1" in
    --apply)
      APPLY=1
      shift
      ;;
    --package-dir)
      (($# >= 2)) || die '--package-dir requires a directory.'
      PACKAGE_DIRECTORY="$2"
      shift 2
      ;;
    --lifecycle-evidence)
      (($# >= 2)) || die '--lifecycle-evidence requires a file.'
      LIFECYCLE_EVIDENCE="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      die "Unknown argument: $1"
      ;;
  esac
done

PROJECT_USER="$(stat -c '%U' -- "${PROJECT_ROOT}")"
PROJECT_GROUP="$(stat -c '%G' -- "${PROJECT_ROOT}")"
[[ "${PROJECT_USER}" != "root" ]] || die 'The source workspace must be non-root owned.'
[[ "${STATIC_WORKER_MODE}" == 0 || "${STATIC_WORKER_MODE}" == 1 ]] ||
  die 'Invalid internal static-worker mode.'

if ((STATIC_WORKER_MODE)); then
  ((APPLY == 0)) || die 'The internal static worker cannot enter apply mode.'
  ((EUID != 0)) || die 'The internal static worker refuses root execution.'
  [[ "$(id -un)" == "${PROJECT_USER}" ]] ||
    die 'The internal static worker is not the workspace owner.'
  case "${STATIC_TEMP}" in
    /tmp/robotest-phase4-worker.*/output) ;;
    *) die "Invalid internal static worker directory: ${STATIC_TEMP}" ;;
  esac
  [[ "$(realpath -e -- "${STATIC_TEMP}")" == "${STATIC_TEMP}" &&
    "$(stat -c '%U:%a' -- "${STATIC_TEMP}")" == "$(id -un):700" ]] ||
    die 'Internal static worker directory is not canonical and user-owned.'
  resolve_package_directory
  resolve_lifecycle_evidence
  run_static_checks_worker
  [[ -f "${STATIC_TEMP}/package-source-rebuild.json" &&
    ! -L "${STATIC_TEMP}/package-source-rebuild.json" ]] ||
    die 'Internal static worker did not retain its rebuild attestation.'
  STATIC_TEMP=""
  exit 0
fi

[[ -z "${STATIC_TEMP}" ]] || die 'Refusing inherited internal static-worker state.'
if ((EUID == 0 && APPLY == 0)); then
  die 'Static-only verification refuses root; rerun without sudo.'
fi

if ((APPLY)); then
  ((EUID == 0)) || die '--apply requires root; rerun the exact command with sudo.'
  require_command git
  verify_apply_source_clean
fi

resolve_package_directory
resolve_lifecycle_evidence
if ((EUID == 0)); then
  for command_name in awk bash chown cmp env find grep id mkdir mktemp python3 \
    realpath runuser stat; do
    require_command "${command_name}"
  done
  project_group_id="$(stat -c '%g' -- "${PROJECT_ROOT}")"
  project_user_group_ids=" $(id -G "${PROJECT_USER}") "
  [[ "${project_user_group_ids}" == *" ${project_group_id} "* ]] ||
    die "Workspace owner ${PROJECT_USER} is not a member of ${PROJECT_GROUP}."
  run_static_checks_as_project_user
else
  run_static_checks_worker
fi

if ((APPLY == 0)); then
  validated_remove_static_temp
  log 'STATIC PREFLIGHT PASS ONLY; no Phase 4 release verdict was produced.'
  log 'After Phase 3 is STABLE, run the authoritative command with sudo and --apply.'
  exit 0
fi

for command_name in chmod chown curl dpkg-query env grep id install journalctl mkdir mv \
  ps runuser setsid ss stat systemctl taskset timeout tr; do
  require_command "${command_name}"
done

verify_apply_source_clean
exec 9>"${LOCK_FILE}"
flock --nonblock 9 || die 'Another Phase 4 verifier owns the live-run lock.'

if [[ -e "${LIVE_RUN_ROOT}" || -L "${LIVE_RUN_ROOT}" ]]; then
  [[ -d "${LIVE_RUN_ROOT}" && ! -L "${LIVE_RUN_ROOT}" &&
    "$(stat -c '%U:%G:%a' -- "${LIVE_RUN_ROOT}")" == 'root:root:700' ]] ||
    die "Unsafe live evidence root: ${LIVE_RUN_ROOT}"
else
  install -d -m 0700 -o root -g root -- "${LIVE_RUN_ROOT}"
fi

RUN_ID="phase4-$(date -u +%Y%m%dT%H%M%SZ)-$$"
[[ "${RUN_ID}" =~ ^phase4-[0-9]{8}T[0-9]{6}Z-[0-9]+$ ]] ||
  die "Generated invalid run ID: ${RUN_ID}"
RUN_DIRECTORY="${LIVE_RUN_ROOT}/${RUN_ID}"
PUBLIC_RUN_DIRECTORY="${PROJECT_ROOT}/artifacts/evidence/phase4/runs/${RUN_ID}"
STATE_RUN_DIRECTORY="/var/lib/robotest-supervisor/${RUN_ID}"
STATE_STAGING_DIRECTORY="${STATE_RUN_DIRECTORY}.staging-${RUN_ID}"
HEARTBEAT="${STATE_RUN_DIRECTORY}/robotest-stack.heartbeat"
STARTUP_RESULT="${STATE_RUN_DIRECTORY}/lifecycle-startup-result.json"
TIMELINE="${RUN_DIRECTORY}/timeline.jsonl"
DROPIN_STAGING_FILE="${DROPIN_DIRECTORY}/.phase4-${RUN_ID}.tmp"
[[ ! -e "${RUN_DIRECTORY}" && ! -L "${RUN_DIRECTORY}" ]] ||
  die "Live run directory already exists: ${RUN_DIRECTORY}"
[[ ! -e "${PUBLIC_RUN_DIRECTORY}" && ! -L "${PUBLIC_RUN_DIRECTORY}" ]] ||
  die "Public run directory already exists: ${PUBLIC_RUN_DIRECTORY}"
mkdir -m 0700 -- "${RUN_DIRECTORY}"
[[ "$(stat -c '%U:%G:%a' -- "${RUN_DIRECTORY}")" == 'root:root:700' ]] ||
  die 'New live run directory ownership or mode is not exact.'
printf '%s\n' "${RUN_ID}" >"${RUN_DIRECTORY}/.phase4-live-owned"
chmod 0600 -- "${RUN_DIRECTORY}/.phase4-live-owned"
LIVE_RUN_OWNED=1

[[ -f "${PACKAGE_SOURCE_REBUILD}" && ! -L "${PACKAGE_SOURCE_REBUILD}" ]] ||
  die 'Fresh package source rebuild evidence is missing or linked.'
install -m 0600 -o root -g root -- \
  "${PACKAGE_SOURCE_REBUILD}" "${RUN_DIRECTORY}/package-source-rebuild.json"
validated_remove_static_temp

python3 "${HELPER}" source-snapshot \
  --repository "${PROJECT_ROOT}" \
  --output "${RUN_DIRECTORY}/source-snapshot-before.json"
check_package_candidate \
  "${RUN_DIRECTORY}/package-binding.json" \
  "${RUN_DIRECTORY}/package-integrity.json"
check_lifecycle_evidence
python3 "${HELPER}" capture-installed-state \
  --evidence "${LIFECYCLE_EVIDENCE}" \
  --upgrade "${UPGRADE_DEB}" \
  --baseline "${BASELINE_DEB}" \
  --smoke-script "${PROJECT_ROOT}/packaging/debian/tests/installed" \
  --output "${RUN_DIRECTORY}/installed-package-state.json"
[[ ! -e "${DROPIN_FILE}" ]] || die "Refusing to overwrite existing drop-in: ${DROPIN_FILE}"
[[ ! -e "${STATE_RUN_DIRECTORY}" ]] ||
  die "Refusing to overwrite existing run state: ${STATE_RUN_DIRECTORY}"
[[ ! -e "${STATE_STAGING_DIRECTORY}" ]] ||
  die "Refusing to overwrite existing staged run state: ${STATE_STAGING_DIRECTORY}"
[[ ! -e "${DROPIN_STAGING_FILE}" ]] ||
  die "Refusing to overwrite existing staged drop-in: ${DROPIN_STAGING_FILE}"
[[ ! -e "${HEARTBEAT}" ]] || die "Refusing to overwrite pre-existing heartbeat: ${HEARTBEAT}"
[[ ! -e "${STARTUP_RESULT}" ]] ||
  die "Refusing to overwrite pre-existing lifecycle result: ${STARTUP_RESULT}"
if ss -H -ltn 'sport = :9080' | grep -q .; then
  die 'Loopback supervisor port 9080 is already in use.'
fi

log 'Applying the immutable runtime overlay (explicit --apply gate).'
stage_runtime_overlay_safely
active_overlay="$(realpath -e -- /opt/robotest-lab)"
install -m 0644 -- "${active_overlay}/staging-provenance.json" \
  "${RUN_DIRECTORY}/overlay-provenance.json"
install -m 0644 -- "${active_overlay}/source-manifest.json" \
  "${RUN_DIRECTORY}/overlay-source-manifest.json"
install -m 0644 -- "${active_overlay}/staging-manifest.json" \
  "${RUN_DIRECTORY}/overlay-install-manifest.json"

log 'Binding the previously completed package lifecycle proof to this run.'
check_lifecycle_evidence
install -m 0644 -- "${LIFECYCLE_EVIDENCE}" \
  "${RUN_DIRECTORY}/package-lifecycle.json"

[[ "$(systemctl is-enabled "${SERVICE}")" == "disabled" ]] ||
  die 'Package lifecycle left the service enabled.'
if systemctl is-active --quiet "${SERVICE}"; then
  die 'Package lifecycle left the service active.'
fi

python3 "${HELPER}" allocate-isolation --run-id "${RUN_ID}" \
  --output "${RUN_DIRECTORY}/isolation.json"
read -r ROS_DOMAIN_ID GZ_PARTITION < <(
  python3 - "${RUN_DIRECTORY}/isolation.json" <<'PY'
import json
import pathlib
import sys

value = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding='utf-8'))
print(value['ros_domain_id'], value['gz_partition'])
PY
)
export ROS_DOMAIN_ID GZ_PARTITION

python3 "${HELPER}" render-config \
  --template "${PROJECT_ROOT}/packaging/debian/config.json" \
  --output "${RUN_DIRECTORY}/supervisor-config.json" \
  --state-directory "${STATE_RUN_DIRECTORY}" \
  --ros-domain-id "${ROS_DOMAIN_ID}" \
  --gz-partition "${GZ_PARTITION}"
config_check_staging="${RUN_DIRECTORY}/.supervisor-config-check.txt"
[[ -x /usr/bin/robotest-supervisor && -f /usr/bin/robotest-supervisor &&
  ! -L /usr/bin/robotest-supervisor ]] ||
  die 'Installed supervisor binary is missing, linked, or not executable.'
if ! /usr/bin/robotest-supervisor \
  --config "${RUN_DIRECTORY}/supervisor-config.json" --check-config \
  >"${config_check_staging}"; then
  rm -f -- "${config_check_staging}"
  die 'Installed supervisor rejected the exact run-scoped configuration.'
fi
if ! printf 'configuration valid\n' | cmp -s - "${config_check_staging}"; then
  rm -f -- "${config_check_staging}"
  die 'Installed supervisor did not produce the canonical config-check result.'
fi
mv --no-target-directory -- "${config_check_staging}" \
  "${RUN_DIRECTORY}/supervisor-config-check.txt"
install -d -m 0750 -o robotest-supervisor -g robotest-supervisor -- \
  "${STATE_STAGING_DIRECTORY}"
printf '%s\n' "${RUN_ID}" >"${STATE_STAGING_DIRECTORY}/.phase4-owned"
chown robotest-supervisor:robotest-supervisor "${STATE_STAGING_DIRECTORY}/.phase4-owned"
chmod 0600 "${STATE_STAGING_DIRECTORY}/.phase4-owned"
install -m 0640 -o root -g robotest-supervisor -- \
  "${RUN_DIRECTORY}/supervisor-config.json" "${STATE_STAGING_DIRECTORY}/config.json"
mv --no-target-directory -- "${STATE_STAGING_DIRECTORY}" "${STATE_RUN_DIRECTORY}"
chmod 0644 "${RUN_DIRECTORY}/supervisor-config.json"
python3 "${HELPER}" render-followup \
  --output "${RUN_DIRECTORY}/followup-mission.json"

git_commit="$(sanitized_git rev-parse HEAD)"
git_dirty=false
[[ -n "$(sanitized_git status --porcelain=v1 --untracked-files=all \
  --ignore-submodules=none)" ]] && git_dirty=true
python3 - "${RUN_DIRECTORY}/context.json" "${RUN_ID}" \
  "${RUN_DIRECTORY}/isolation.json" "${git_commit}" "${git_dirty}" \
  "${PACKAGE_DIRECTORY}" "${UPGRADE_DEB}" "${BASELINE_DEB}" \
  "${active_overlay}" "${LIFECYCLE_EVIDENCE}" <<'PY'
import hashlib
import json
import os
import pathlib
import sys
import tempfile
from datetime import datetime, timezone

(
    output, run_id, isolation_path, commit, dirty, package_directory,
    upgrade_deb, baseline_deb, active_overlay, lifecycle_evidence,
) = sys.argv[1:]
isolation = json.loads(pathlib.Path(isolation_path).read_text(encoding='utf-8'))

def digest(path):
    value = hashlib.sha256()
    with pathlib.Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()

value = {
    'schema_version': 1,
    'run_id': run_id,
    'started_utc': datetime.now(timezone.utc).isoformat(),
    'source_git_commit': commit,
    'source_git_dirty': dirty == 'true',
    'isolation': isolation,
    'package_directory': package_directory,
    'upgrade_package': {'path': upgrade_deb, 'sha256': digest(upgrade_deb)},
    'baseline_package': {'path': baseline_deb, 'sha256': digest(baseline_deb)},
    'lifecycle_evidence': {
        'path': lifecycle_evidence,
        'sha256': digest(lifecycle_evidence),
    },
    'active_overlay_target': active_overlay,
    'cpuset': '0-5',
}
path = pathlib.Path(output)
descriptor, temporary = tempfile.mkstemp(prefix='.context.', dir=path.parent)
with os.fdopen(descriptor, 'w', encoding='utf-8') as target:
    json.dump(value, target, separators=(',', ':'), sort_keys=True)
    target.write('\n')
    target.flush()
    os.fsync(target.fileno())
os.replace(temporary, path)
PY

if [[ ! -d "${DROPIN_DIRECTORY}" ]]; then
  DROPIN_DIRECTORY_CREATED=1
fi
install -d -m 0755 -o root -g root -- "${DROPIN_DIRECTORY}"
printf '[Service]\nExecStart=\nExecStart=/usr/bin/robotest-supervisor --config %s\n' \
  "${STATE_RUN_DIRECTORY}/config.json" >"${RUN_DIRECTORY}/systemd-dropin.conf"
chmod 0644 "${RUN_DIRECTORY}/systemd-dropin.conf"
install -m 0644 -o root -g root -- \
  "${RUN_DIRECTORY}/systemd-dropin.conf" "${DROPIN_STAGING_FILE}"
DROPIN_SHA256="$(sha256sum "${DROPIN_STAGING_FILE}" | awk '{print $1}')"
mv --no-target-directory -- "${DROPIN_STAGING_FILE}" "${DROPIN_FILE}"
systemctl daemon-reload
systemd-analyze verify "/usr/lib/systemd/system/${SERVICE}" \
  >"${RUN_DIRECTORY}/systemd-analyze-installed.log" 2>&1
[[ "$(systemctl is-enabled "${SERVICE}")" == "disabled" ]] ||
  die 'Installing the run drop-in changed the disabled state.'

log 'Starting the single packaged supervisor service.'
systemctl start "${SERVICE}"
SERVICE_STARTED=1

ready_status=0
for _attempt in {1..1200}; do
  health_status="$(python3 "${HELPER}" probe-http --timeline "${TIMELINE}" \
    --kind startup_health_probe --url "${BASE_URL}/healthz" --timeout 0.5)"
  ready_status="$(python3 "${HELPER}" probe-http --timeline "${TIMELINE}" \
    --kind startup_ready_probe --url "${BASE_URL}/readyz" --timeout 0.5)"
  if [[ "${health_status}" == 200 && "${ready_status}" == 200 ]]; then
    break
  fi
  sleep 0.1
done
[[ "${ready_status}" == 200 ]] || die 'Supervisor did not become ready within startup bound.'
python3 "${HELPER}" fetch-json --url "${BASE_URL}/v1/status" \
  --output "${RUN_DIRECTORY}/initial-status.json"

read -r supervisor_main_pid original_child_pid ORIGINAL_PGID initial_restart_count < <(
  python3 - "${RUN_DIRECTORY}/initial-status.json" <<'PY'
import json
import pathlib
import subprocess
import sys

value = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding='utf-8'))
children = value.get('children')
if value.get('ready') is not True or not isinstance(children, list) or len(children) != 1:
    raise SystemExit('initial supervisor status is not exactly ready with one child')
child = children[0]
main_pid = int(subprocess.check_output(
    ['systemctl', 'show', '--property=MainPID', '--value', 'robotest-supervisor.service'],
    text=True,
).strip())
print(main_pid, child['pid'], child['pgid'], child['restart_count'])
PY
)
[[ "${original_child_pid}" == "${ORIGINAL_PGID}" &&
  "${initial_restart_count}" == 0 ]] ||
  die 'Initial managed child PID/PGID/restart count is invalid.'
group_exists "${ORIGINAL_PGID}" || die 'Initial managed process group is absent.'
initial_nrestarts="$(systemctl show --property=NRestarts --value "${SERVICE}")"
initial_ownership_counts="$(capture_stable_runtime_ownership \
  initial "${supervisor_main_pid}" "${original_child_pid}" "${ORIGINAL_PGID}" \
  "${RUN_DIRECTORY}/systemd-owners-initial.json" \
  "${RUN_DIRECTORY}/runtime-affinity-initial.json")" ||
  die 'Initial runtime ownership did not stabilize.'
read -r initial_owner_count initial_affinity_count <<<"${initial_ownership_counts}"
[[ "${initial_owner_count}" == 1 && "${initial_affinity_count}" =~ ^[0-9]+$ &&
  "${initial_affinity_count}" -ge 3 ]] ||
  die 'Initial unit-cgroup affinity evidence is incomplete.'
record_timeline initial_ready \
  "main_pid=${supervisor_main_pid}" \
  "child_pid=${original_child_pid}" \
  "child_pgid=${ORIGINAL_PGID}" \
  "systemd_nrestarts=${initial_nrestarts}" \
  "systemd_owned_ros_service_count=${initial_owner_count}"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /opt/robotest-lab/install/setup.bash
set -u
export ROS_DOMAIN_ID GZ_PARTITION RCUTILS_LOGGING_BUFFERED_STREAM=1

interrupted_command=(
  taskset -c 0-5
  timeout --signal=TERM --kill-after=5s 90s
  ros2 run robotest_missions mission_runner
  --mission "${PROJECT_ROOT}/scenarios/phase2_baseline.yaml"
  --json "${RUN_DIRECTORY}/interrupted-result.json"
  --csv "${RUN_DIRECTORY}/interrupted-result.csv"
  --ros-args -r __ns:=/robotest
)
begin_process_publication
setsid "${interrupted_command[@]}" >"${RUN_DIRECTORY}/interrupted-mission.log" 2>&1 &
MISSION_PID=$!
MISSION_PGID="${MISSION_PID}"
MISSION_LAST_PGID="${MISSION_PGID}"
end_process_publication
captured_mission_pgid="$(captured_pgid "${MISSION_PID}")" ||
  die 'Could not capture the interrupted mission process group.'
[[ "${captured_mission_pgid}" == "${MISSION_PGID}" ]] ||
  die 'Interrupted mission did not own its expected setsid process group.'
record_timeline mission_started "pid=${MISSION_PID}" "pgid=${MISSION_PGID}"

begin_process_publication
setsid taskset -c 0-5 timeout --signal=TERM --kill-after=3s 35s \
  python3 "${ACTIVE_GOAL_PROBE}" \
    --mission-pid "${MISSION_PID}" \
    --wall-timeout 30 \
    --output "${RUN_DIRECTORY}/active-goal.json" \
  >"${RUN_DIRECTORY}/active-goal-probe.log" 2>&1 &
PROBE_PID=$!
PROBE_PGID="${PROBE_PID}"
PROBE_LAST_PGID="${PROBE_PGID}"
end_process_publication
captured_probe_pgid="$(captured_pgid "${PROBE_PID}")" ||
  die 'Could not capture the active-goal probe process group.'
[[ "${captured_probe_pgid}" == "${PROBE_PGID}" ]] ||
  die 'Active-goal probe did not own its expected setsid process group.'
record_timeline active_goal_probe_started "pid=${PROBE_PID}" "pgid=${PROBE_PGID}"
set +e
wait "${PROBE_PID}"
probe_status=$?
set -e
PROBE_PID=""
if stop_owned_group active-goal-probe "" "${PROBE_PGID}"; then
  PROBE_PGID=""
else
  log 'Active-goal probe left a residual process; finalization will retry cleanup.'
fi
((probe_status == 0)) || die "Active-goal proof failed with exit ${probe_status}."
group_exists "${MISSION_PGID}" || die 'Mission ended before fault injection.'

target_pid="$(python3 "${HELPER}" find-controller \
  --root-pid "${original_child_pid}" \
  --pgid "${ORIGINAL_PGID}" \
  --ros-domain-id "${ROS_DOMAIN_ID}" \
  --gz-partition "${GZ_PARTITION}" \
  --output "${RUN_DIRECTORY}/controller-target.json")"
python3 "${HELPER}" assert-identity \
  --identity "${RUN_DIRECTORY}/controller-target.json"
event_sequence_before="$(last_supervisor_event_sequence)"

log "Injecting one exact nested controller crash at PID ${target_pid}."
signaled_pid="$(python3 "${HELPER}" signal-identity \
  --identity "${RUN_DIRECTORY}/controller-target.json" --signal TERM)"
[[ "${signaled_pid}" == "${target_pid}" ]] ||
  die 'Immediate identity revalidation signaled an unexpected PID.'
record_timeline failure_injected \
  "target_pid=${target_pid}" \
  "original_pgid=${ORIGINAL_PGID}" \
  "mission_pid=${MISSION_PID}" \
  "event_sequence_before=${event_sequence_before}"

unavailable_seen=0
group_empty_seen=0
restored_seen=0
for _attempt in {1..450}; do
  health_status="$(python3 "${HELPER}" probe-http --timeline "${TIMELINE}" \
    --kind health_probe --url "${BASE_URL}/healthz" --timeout 0.5)"
  ready_status="$(python3 "${HELPER}" probe-http --timeline "${TIMELINE}" \
    --kind ready_probe --url "${BASE_URL}/readyz" --timeout 0.5)"
  if ((unavailable_seen == 0)) && [[ "${ready_status}" == 503 ]]; then
    record_timeline ready_unavailable "http_status=503"
    unavailable_seen=1
  fi
  if ((group_empty_seen == 0)) && ! group_exists "${ORIGINAL_PGID}"; then
    python3 "${HELPER}" process-group --pgid "${ORIGINAL_PGID}" \
      --require-empty --output "${RUN_DIRECTORY}/original-group-empty.json"
    record_timeline original_group_empty 'member_count=0'
    group_empty_seen=1
  fi
  if ((unavailable_seen)) && ((restored_seen == 0)) && [[ "${ready_status}" == 200 ]]; then
    if python3 "${HELPER}" fetch-json --url "${BASE_URL}/v1/status" \
      --output "${RUN_DIRECTORY}/ready-restored-status.json"; then
      read -r restored_child_pid restored_child_pgid restored_restart_count < <(
        python3 - "${RUN_DIRECTORY}/ready-restored-status.json" <<'PY'
import json
import pathlib
import sys

value = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding='utf-8'))
children = value.get('children')
if value.get('ready') is not True or not isinstance(children, list) or len(children) != 1:
    raise SystemExit(1)
child = children[0]
if child.get('running') is not True or child.get('heartbeat_fresh') is not True:
    raise SystemExit(1)
print(child['pid'], child['pgid'], child['restart_count'])
PY
      ) || true
      if [[ "${restored_child_pid:-}" =~ ^[0-9]+$ &&
        "${restored_child_pgid:-}" =~ ^[0-9]+$ &&
        "${restored_child_pid}" == "${restored_child_pgid}" &&
        "${restored_child_pgid}" != "${ORIGINAL_PGID}" &&
        "${restored_restart_count:-}" == 1 ]]; then
        restored_main_pid="$(systemctl show --property=MainPID --value "${SERVICE}")"
        restored_nrestarts="$(systemctl show --property=NRestarts --value "${SERVICE}")"
        if ! restored_ownership_counts="$(capture_stable_runtime_ownership \
          restored "${restored_main_pid}" "${restored_child_pid}" \
          "${restored_child_pgid}" \
          "${RUN_DIRECTORY}/systemd-owners-restored.json" \
          "${RUN_DIRECTORY}/runtime-affinity-restored.json")"; then
          sleep 0.05
          continue
        fi
        read -r restored_owner_count restored_affinity_count \
          <<<"${restored_ownership_counts}"
        if [[ "${restored_owner_count}" != 1 ||
          ! "${restored_affinity_count}" =~ ^[0-9]+$ ||
          "${restored_affinity_count}" -lt 3 ]]; then
          rm -f -- \
            "${RUN_DIRECTORY}/runtime-affinity-restored.json" \
            "${RUN_DIRECTORY}/systemd-owners-restored.json"
          sleep 0.05
          continue
        fi
        recovery_event_sequence="$(last_supervisor_event_sequence)"
        RESTORED_PGID="${restored_child_pgid}"
        record_timeline ready_restored \
          'http_status=200' \
          "main_pid=${restored_main_pid}" \
          "child_pid=${restored_child_pid}" \
          "child_pgid=${restored_child_pgid}" \
          "systemd_nrestarts=${restored_nrestarts}" \
          "systemd_owned_ros_service_count=${restored_owner_count}" \
          "event_sequence_after_recovery=${recovery_event_sequence}"
        restored_seen=1
      fi
    fi
  fi
  if ((unavailable_seen && group_empty_seen && restored_seen)); then
    break
  fi
  [[ "${health_status}" == 200 ]] || true
  sleep 0.05
done
((unavailable_seen)) || die '/readyz never became 503 after injection.'
((group_empty_seen)) || die 'Original managed process group did not become empty.'
((restored_seen)) || die 'Supervisor did not restore exact readiness.'

interrupted_terminated=false
for _attempt in {1..100}; do
  kill -0 "${MISSION_PID}" 2>/dev/null || break
  sleep 0.05
done
if kill -0 "${MISSION_PID}" 2>/dev/null; then
  interrupted_terminated=true
  stop_owned_group interrupted-mission "" "${MISSION_PGID}" || true
fi
set +e
wait "${MISSION_PID}"
interrupted_status=$?
set -e
MISSION_PID=""
if stop_owned_group interrupted-mission "" "${MISSION_PGID}"; then
  MISSION_PGID=""
else
  log 'Interrupted mission left a residual process; finalization will retry cleanup.'
fi
interrupted_group_empty=false
! group_exists "${MISSION_LAST_PGID}" && interrupted_group_empty=true
interrupted_json_present=false
interrupted_csv_present=false
[[ -f "${RUN_DIRECTORY}/interrupted-result.json" ]] && interrupted_json_present=true
[[ -f "${RUN_DIRECTORY}/interrupted-result.csv" ]] && interrupted_csv_present=true
python3 - "${RUN_DIRECTORY}/interrupted-outcome.json" \
  "${interrupted_status}" "${interrupted_json_present}" \
  "${interrupted_csv_present}" "${interrupted_terminated}" \
  "${interrupted_group_empty}" <<'PY'
import json
import os
import pathlib
import sys
import tempfile
from datetime import datetime, timezone

path = pathlib.Path(sys.argv[1])
value = {
    'schema_version': 1,
    'captured_utc': datetime.now(timezone.utc).isoformat(),
    'process_exit_code': int(sys.argv[2]),
    'json_present': sys.argv[3] == 'true',
    'csv_present': sys.argv[4] == 'true',
    'bounded_wait': True,
    'terminated_by_harness': sys.argv[5] == 'true',
    'process_group_empty': sys.argv[6] == 'true',
}
descriptor, temporary = tempfile.mkstemp(prefix='.interrupted.', dir=path.parent)
with os.fdopen(descriptor, 'w', encoding='utf-8') as target:
    json.dump(value, target, separators=(',', ':'), sort_keys=True)
    target.write('\n')
    target.flush()
    os.fsync(target.fileno())
os.replace(temporary, path)
PY
record_timeline interrupted_mission_finished \
  "exit_code=${interrupted_status}" \
  "terminated_by_harness=${interrupted_terminated}" \
  "process_group_empty=${interrupted_group_empty}"

log 'Running the fresh one-waypoint recovery mission.'
followup_command=(
  taskset -c 0-5
  timeout --signal=TERM --kill-after=5s 130s
  ros2 run robotest_missions mission_runner
  --mission "${RUN_DIRECTORY}/followup-mission.json"
  --json "${RUN_DIRECTORY}/followup-result.json"
  --csv "${RUN_DIRECTORY}/followup-result.csv"
  --ros-args -r __ns:=/robotest
)
begin_process_publication
setsid "${followup_command[@]}" >"${RUN_DIRECTORY}/followup-mission.log" 2>&1 &
FOLLOWUP_PID=$!
FOLLOWUP_PGID="${FOLLOWUP_PID}"
FOLLOWUP_LAST_PGID="${FOLLOWUP_PGID}"
end_process_publication
captured_followup_pgid="$(captured_pgid "${FOLLOWUP_PID}")" ||
  die 'Could not capture follow-up mission process group.'
[[ "${captured_followup_pgid}" == "${FOLLOWUP_PGID}" ]] ||
  die 'Follow-up mission did not own its expected setsid process group.'
record_timeline followup_started "pid=${FOLLOWUP_PID}" "pgid=${FOLLOWUP_PGID}"
set +e
wait "${FOLLOWUP_PID}"
followup_status=$?
set -e
FOLLOWUP_PID=""
if stop_owned_group followup-mission "" "${FOLLOWUP_PGID}"; then
  FOLLOWUP_PGID=""
else
  log 'Follow-up mission left a residual process; finalization will retry cleanup.'
fi
record_timeline followup_finished "exit_code=${followup_status}"
((followup_status == 0)) || log "Follow-up mission returned ${followup_status}; evidence will fail closed."

set +e
finalize_authoritative_run
finalization_status=$?
set -e

((FREEZE_STATUS == 0)) || die "Live evidence freeze failed; retained at ${RUN_DIRECTORY}"
((PUBLICATION_STATUS == 0)) ||
  die "Evidence publication failed; live evidence retained at ${RUN_DIRECTORY}"
((CHECKSUM_STATUS == 0)) || die 'Evidence checksum generation failed.'
((CLEANUP_STATUS == 0)) ||
  die "Owned cleanup failed; evidence published at ${PUBLIC_RUN_DIRECTORY}"
((COMPOSE_STATUS == 0)) ||
  die "Scenario 6 evidence verdict is FAIL: ${PUBLIC_RUN_DIRECTORY}"
((LIVE_REMOVAL_STATUS == 0)) ||
  die "Published evidence is valid but live staging was retained at ${RUN_DIRECTORY}"
((finalization_status == 0)) || die 'Authoritative finalization failed.'
log "PHASE 4 PASS: ${PUBLIC_RUN_DIRECTORY}"

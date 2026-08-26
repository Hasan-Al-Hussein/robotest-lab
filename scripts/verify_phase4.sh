#!/usr/bin/env bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

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
readonly HEARTBEAT="/var/lib/robotest-supervisor/robotest-stack.heartbeat"
readonly STARTUP_RESULT="/var/lib/robotest-supervisor/lifecycle-startup-result.json"
readonly DROPIN_DIRECTORY="/run/systemd/system/${SERVICE}.d"
readonly DROPIN_FILE="${DROPIN_DIRECTORY}/90-robotest-phase4-verifier.conf"
readonly LOCK_FILE="/run/lock/robotest-phase4-verifier.lock"

APPLY=0
PACKAGE_DIRECTORY=""
STATIC_TEMP=""
RUN_ID=""
RUN_DIRECTORY=""
STATE_RUN_DIRECTORY=""
STATE_STAGING_DIRECTORY=""
TIMELINE=""
ROS_DOMAIN_ID=""
GZ_PARTITION=""
UPGRADE_DEB=""
BASELINE_DEB=""
PACKAGE_MANIFEST=""
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
FINALIZED=0
FINALIZATION_STATE=0
FINALIZATION_STATUS=0
CLEANUP_STATUS=0
COMPOSE_STATUS=0
CHECKSUM_STATUS=0
CHOWN_STATUS=0
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
    --output "${integrity_output}"
  if ! python3 "${HELPER}" package-binding \
    --repository "${PROJECT_ROOT}" \
    --manifest "${PACKAGE_MANIFEST}" \
    --output "${binding_output}"; then
    sed -n '1,160p' "${binding_output}" >&2 || true
    die 'Package candidate source manifest does not match current package source.'
  fi
}

run_static_checks() {
  log 'Running bounded static, unit, concurrency, and package-source gates.'
  STATIC_TEMP="$(mktemp -d /tmp/robotest-phase4-static.XXXXXX)"

  for command_name in bash dpkg dpkg-deb find flock git go gofmt python3 \
    realpath sha256sum shellcheck systemd-analyze; do
    require_command "${command_name}"
  done

  bash -n \
    "${SCRIPT_DIR}/build_debian_package.sh" \
    "${SCRIPT_DIR}/create_debian_upgrade_fixture.sh" \
    "${SCRIPT_DIR}/stage_runtime_overlay.sh" \
    "${SCRIPT_DIR}/test_debian_package_lifecycle.sh" \
    "${SCRIPT_DIR}/verify_debian_reproducibility.sh" \
    "${SCRIPT_DIR}/verify_phase4.sh"
  shellcheck \
    "${SCRIPT_DIR}/build_debian_package.sh" \
    "${SCRIPT_DIR}/create_debian_upgrade_fixture.sh" \
    "${SCRIPT_DIR}/stage_runtime_overlay.sh" \
    "${SCRIPT_DIR}/test_debian_package_lifecycle.sh" \
    "${SCRIPT_DIR}/verify_debian_reproducibility.sh" \
    "${SCRIPT_DIR}/verify_phase4.sh"
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
  check_lifecycle_evidence
  python3 "${HELPER}" source-snapshot \
    --repository "${PROJECT_ROOT}" \
    --output "${STATIC_TEMP}/source-snapshot.json"
  log "Static gates PASS; package candidate: ${PACKAGE_DIRECTORY}"
  validated_remove_static_temp
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

begin_process_publication() {
  PENDING_PUBLICATION_SIGNAL=0
  trap 'PENDING_PUBLICATION_SIGNAL=129' HUP
  trap 'PENDING_PUBLICATION_SIGNAL=130' INT
  trap 'PENDING_PUBLICATION_SIGNAL=143' TERM
}

end_process_publication() {
  local pending_signal="${PENDING_PUBLICATION_SIGNAL}"
  trap 'exit 129' HUP
  trap 'exit 130' INT
  trap 'exit 143' TERM
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
  for source in events.jsonl events.meta.json status.json; do
    [[ -f "${STATE_RUN_DIRECTORY}/${source}" &&
      ! -L "${STATE_RUN_DIRECTORY}/${source}" ]] || continue
    case "${source}" in
      events.jsonl) destination=supervisor-events.jsonl ;;
      events.meta.json) destination=supervisor-events.meta.json ;;
      status.json) destination=supervisor-status-final.json ;;
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
  [[ -n "${RUN_DIRECTORY}" && -d "${RUN_DIRECTORY}" ]] || return 0

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
  if ((FINALIZATION_STATE == 2)); then
    return "${FINALIZATION_STATUS}"
  fi
  if ((FINALIZATION_STATE == 1)); then
    log 'Refusing re-entrant finalization.'
    return 1
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
  if chown -R "${PROJECT_USER}:${PROJECT_GROUP}" -- "${RUN_DIRECTORY}"; then
    CHOWN_STATUS=0
  else
    CHOWN_STATUS=$?
  fi
  FINALIZATION_STATUS=0
  if ((CLEANUP_STATUS != 0 || COMPOSE_STATUS != 0 || CHECKSUM_STATUS != 0 || CHOWN_STATUS != 0)); then
    FINALIZATION_STATUS=1
  fi
  FINALIZATION_STATE=2
  FINALIZED=1
  return "${FINALIZATION_STATUS}"
}

on_exit() {
  local status=$?
  trap - EXIT HUP INT TERM
  ((EXIT_HANDLER_ACTIVE == 0)) || exit "${status}"
  EXIT_HANDLER_ACTIVE=1
  set +e
  if ((APPLY)) && ((FINALIZED == 0)) && [[ -n "${RUN_DIRECTORY}" ]]; then
    finalize_authoritative_run
    local finalization_status=$?
    if ((status == 0 && finalization_status != 0)); then
      status=1
    fi
  fi
  validated_remove_static_temp
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

resolve_package_directory
resolve_lifecycle_evidence
run_static_checks

if ((APPLY == 0)); then
  log 'STATIC PREFLIGHT PASS ONLY; no Phase 4 release verdict was produced.'
  log 'After Phase 3 is STABLE, run the authoritative command with sudo and --apply.'
  exit 0
fi

((EUID == 0)) || die '--apply requires root; rerun the exact command with sudo.'
for command_name in curl dpkg-query grep id install journalctl mv ps setsid ss stat \
  systemctl taskset timeout tr; do
  require_command "${command_name}"
done

exec 9>"${LOCK_FILE}"
flock --nonblock 9 || die 'Another Phase 4 verifier owns the live-run lock.'

PROJECT_USER="$(stat -c '%U' -- "${PROJECT_ROOT}")"
PROJECT_GROUP="$(stat -c '%G' -- "${PROJECT_ROOT}")"
[[ "${PROJECT_USER}" != "root" ]] || die 'The source workspace must be non-root owned.'

RUN_ID="phase4-$(date -u +%Y%m%dT%H%M%SZ)-$$"
[[ "${RUN_ID}" =~ ^phase4-[0-9]{8}T[0-9]{6}Z-[0-9]+$ ]] ||
  die "Generated invalid run ID: ${RUN_ID}"
RUN_DIRECTORY="${PROJECT_ROOT}/artifacts/evidence/phase4/runs/${RUN_ID}"
STATE_RUN_DIRECTORY="/var/lib/robotest-supervisor/${RUN_ID}"
STATE_STAGING_DIRECTORY="${STATE_RUN_DIRECTORY}.staging-${RUN_ID}"
TIMELINE="${RUN_DIRECTORY}/timeline.jsonl"
DROPIN_STAGING_FILE="${DROPIN_DIRECTORY}/.phase4-${RUN_ID}.tmp"
[[ ! -e "${RUN_DIRECTORY}" ]] || die "Run directory already exists: ${RUN_DIRECTORY}"
install -d -m 0755 -o "${PROJECT_USER}" -g "${PROJECT_GROUP}" -- "${RUN_DIRECTORY}"

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
"${SCRIPT_DIR}/stage_runtime_overlay.sh" --apply
"${SCRIPT_DIR}/stage_runtime_overlay.sh" --check
install -m 0644 -- "${PROJECT_ROOT}/artifacts/evidence/phase4/runtime-staging.json" \
  "${RUN_DIRECTORY}/runtime-staging.json"
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

git_commit="$(git -c safe.directory="${PROJECT_ROOT}" -C "${PROJECT_ROOT}" rev-parse HEAD)"
git_dirty=false
[[ -n "$(git -c safe.directory="${PROJECT_ROOT}" -C "${PROJECT_ROOT}" \
  status --porcelain=v1 --untracked-files=all)" ]] && git_dirty=true
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
initial_owner_count="$(python3 "${HELPER}" systemd-owners \
  --ros-domain-id "${ROS_DOMAIN_ID}" --gz-partition "${GZ_PARTITION}" \
  --output "${RUN_DIRECTORY}/systemd-owners-initial.json")"
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
      if [[ "${restored_child_pgid:-}" =~ ^[0-9]+$ &&
        "${restored_child_pgid}" != "${ORIGINAL_PGID}" &&
        "${restored_restart_count:-}" == 1 ]]; then
        restored_main_pid="$(systemctl show --property=MainPID --value "${SERVICE}")"
        restored_nrestarts="$(systemctl show --property=NRestarts --value "${SERVICE}")"
        restored_owner_count="$(python3 "${HELPER}" systemd-owners \
          --ros-domain-id "${ROS_DOMAIN_ID}" --gz-partition "${GZ_PARTITION}" \
          --output "${RUN_DIRECTORY}/systemd-owners-restored.json")"
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

((CLEANUP_STATUS == 0)) || die 'Owned cleanup did not complete cleanly.'
((COMPOSE_STATUS == 0)) || die "Scenario 6 evidence verdict is FAIL: ${RUN_DIRECTORY}"
((CHECKSUM_STATUS == 0)) || die 'Evidence checksum generation failed.'
((CHOWN_STATUS == 0)) || die 'Evidence ownership restoration failed.'
((finalization_status == 0)) || die 'Authoritative finalization failed.'
log "PHASE 4 PASS: ${RUN_DIRECTORY}"

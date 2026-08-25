#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_ROOT
readonly MANIFEST="${PROJECT_ROOT}/config/apt-packages.txt"
readonly EVIDENCE_DIR="${PROJECT_ROOT}/artifacts/evidence/phase0"
readonly GIB=$((1024 * 1024 * 1024))
readonly MAX_PROJECT_BYTES=$((18 * GIB))
readonly MIN_HOST_FREE_BYTES=$((20 * GIB))
readonly PLANNED_WORKSPACE_BYTES=$((8 * GIB))

declare -a PACKAGES=()
TEMP_DIR=""

log() {
  printf '[verify_phase0_preinstall] %s\n' "$*"
}

die() {
  printf '[verify_phase0_preinstall] ERROR: %s\n' "$*" >&2
  exit 1
}

cleanup() {
  if [[ -n "${TEMP_DIR}" && -d "${TEMP_DIR}" ]]; then
    rm -rf -- "${TEMP_DIR}"
  fi
}
trap cleanup EXIT

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

require_noble() {
  [[ -r /etc/os-release ]] || die "/etc/os-release is unavailable."
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" && "${VERSION_CODENAME:-}" == "noble" ]] ||
    die "This gate requires Ubuntu 24.04 Noble; Ubuntu-20.04 is protected."
  [[ "${WSL_DISTRO_NAME:-}" != "Ubuntu-20.04" ]] ||
    die "Refusing to operate in the protected Ubuntu-20.04 distribution."
}

load_manifest() {
  local line line_number=0

  [[ -f "${MANIFEST}" ]] || die "Missing apt manifest: ${MANIFEST}"
  while IFS= read -r line || [[ -n "${line}" ]]; do
    ((line_number += 1))
    [[ -z "${line}" ]] && continue
    [[ "${line}" =~ ^[a-z0-9][a-z0-9.+-]*$ ]] ||
      die "Invalid literal apt package at ${MANIFEST}:${line_number}: ${line@Q}"
    PACKAGES+=("${line}")
  done <"${MANIFEST}"

  ((${#PACKAGES[@]} > 0)) || die "The apt manifest is empty."
  mapfile -t PACKAGES < <(printf '%s\n' "${PACKAGES[@]}" | LC_ALL=C sort -u)
}

integer_or_die() {
  [[ "$1" =~ ^[0-9]+$ ]] || die "Expected an integer byte count; received ${1@Q}."
}

require_noble
for command_name in apt-cache apt-get awk df dpkg-query du python3 sha256sum sort; do
  require_command "${command_name}"
done
load_manifest

"${SCRIPT_DIR}/setup_ros2_repository.sh" --check

mkdir -p -- "${EVIDENCE_DIR}"
TEMP_DIR="$(mktemp -d "${EVIDENCE_DIR}/.preinstall.XXXXXX")"
readonly SIMULATION_FILE="${TEMP_DIR}/apt-simulation.txt"
readonly RESOLUTION_FILE="${TEMP_DIR}/apt-resolution.tsv"
readonly PLANNED_FILE="${TEMP_DIR}/apt-planned-packages.txt"
readonly JSON_FILE="${TEMP_DIR}/phase0-preinstall.json"

printf 'package\tcandidate_version\tinstalled_version\n' >"${RESOLUTION_FILE}"
for package_name in "${PACKAGES[@]}"; do
  candidate_version="$(
    apt-cache policy "${package_name}" 2>/dev/null |
      awk '$1 == "Candidate:" && !seen { print $2; seen = 1 }'
  )"
  [[ -n "${candidate_version}" && "${candidate_version}" != "(none)" ]] ||
    die "No apt candidate is available for manifest package ${package_name}."
  installed_version="$(dpkg-query -W -f='${Version}' "${package_name}" 2>/dev/null || true)"
  printf '%s\t%s\t%s\n' \
    "${package_name}" \
    "${candidate_version}" \
    "${installed_version}" >>"${RESOLUTION_FILE}"
done

log "Simulating ${#PACKAGES[@]} exact manifest entries without installing packages."
if ! LC_ALL=C apt-get \
  --simulate \
  install "${PACKAGES[@]}" >"${SIMULATION_FILE}" 2>&1; then
  cat "${SIMULATION_FILE}" >&2
  die "Apt simulation failed; no installation is permitted."
fi

if grep -q '^Remv ' "${SIMULATION_FILE}"; then
  cat "${SIMULATION_FILE}" >&2
  die "Apt simulation proposes package removals; no installation is permitted."
fi

awk '$1 == "Inst" { print $2 }' "${SIMULATION_FILE}" | LC_ALL=C sort -u >"${PLANNED_FILE}"

additional_dependency_bytes=0
while IFS= read -r planned_package; do
  [[ -z "${planned_package}" ]] && continue
  installed_size_kib="$(
    apt-cache show "${planned_package}" 2>/dev/null |
      awk '$1 == "Installed-Size:" && !seen { print $2; seen = 1 }'
  )"
  [[ "${installed_size_kib}" =~ ^[0-9]+$ ]] ||
    die "Could not determine Installed-Size for planned package ${planned_package}."
  additional_dependency_bytes=$((additional_dependency_bytes + installed_size_kib * 1024))
done <"${PLANNED_FILE}"

current_project_bytes="$(du -s --block-size=1 "${PROJECT_ROOT}" | awk '{ print $1 }')"
existing_ros_bytes=0
if [[ -d /opt/ros/jazzy ]]; then
  existing_ros_bytes="$(du -s --block-size=1 /opt/ros/jazzy | awk '{ print $1 }')"
fi
[[ -d /mnt/c ]] || die "/mnt/c is unavailable; Windows-host reserve cannot be verified."
host_free_bytes="$(df -B1 --output=avail /mnt/c | awk 'NR == 2 { print $1 }')"

integer_or_die "${current_project_bytes}"
integer_or_die "${existing_ros_bytes}"
integer_or_die "${host_free_bytes}"

workspace_projection_bytes="${PLANNED_WORKSPACE_BYTES}"
if ((current_project_bytes > workspace_projection_bytes)); then
  workspace_projection_bytes="${current_project_bytes}"
fi

projected_total_bytes=$((workspace_projection_bytes + existing_ros_bytes + additional_dependency_bytes))
remaining_workspace_growth_bytes=$((workspace_projection_bytes - current_project_bytes))
projected_host_growth_bytes=$((remaining_workspace_growth_bytes + additional_dependency_bytes))
projected_host_free_bytes=$((host_free_bytes - projected_host_growth_bytes))

((projected_total_bytes <= MAX_PROJECT_BYTES)) ||
  die "Projected project/dependency use ${projected_total_bytes} bytes exceeds the 18 GiB limit."
((projected_host_free_bytes >= MIN_HOST_FREE_BYTES)) ||
  die "Projected Windows-host free space ${projected_host_free_bytes} bytes is below the 20 GiB reserve."

manifest_sha256="$(sha256sum "${MANIFEST}" | awk '{ print $1 }')"
planned_package_count="$(awk 'NF { count += 1 } END { print count + 0 }' "${PLANNED_FILE}")"

python3 - \
  "${manifest_sha256}" \
  "${#PACKAGES[@]}" \
  "${planned_package_count}" \
  "${current_project_bytes}" \
  "${existing_ros_bytes}" \
  "${additional_dependency_bytes}" \
  "${workspace_projection_bytes}" \
  "${projected_total_bytes}" \
  "${MAX_PROJECT_BYTES}" \
  "${host_free_bytes}" \
  "${projected_host_growth_bytes}" \
  "${projected_host_free_bytes}" \
  "${MIN_HOST_FREE_BYTES}" \
  "${RESOLUTION_FILE}" \
  "${PLANNED_FILE}" >"${JSON_FILE}" <<'PY'
import csv
import json
import sys
from datetime import datetime, timezone

(
    manifest_sha256,
    manifest_count,
    planned_count,
    current_project_bytes,
    existing_ros_bytes,
    additional_dependency_bytes,
    workspace_projection_bytes,
    projected_total_bytes,
    max_project_bytes,
    host_free_bytes,
    projected_host_growth_bytes,
    projected_host_free_bytes,
    min_host_free_bytes,
    resolution_path,
    planned_path,
) = sys.argv[1:]

with open(resolution_path, encoding="utf-8", newline="") as handle:
    resolutions = list(csv.DictReader(handle, delimiter="\t"))
with open(planned_path, encoding="utf-8") as handle:
    planned_packages = [line.strip() for line in handle if line.strip()]

json.dump(
    {
        "schema_version": 1,
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "status": "passed",
        "platform": {
            "os": "Ubuntu 24.04 Noble",
            "protected_distribution": "Ubuntu-20.04",
        },
        "manifest": {
            "path": "config/apt-packages.txt",
            "sha256": manifest_sha256,
            "entry_count": int(manifest_count),
            "resolutions": resolutions,
        },
        "apt_simulation": {
            "planned_package_count": int(planned_count),
            "planned_packages": planned_packages,
            "proposed_removals": 0,
        },
        "storage_gate": {
            "current_project_bytes": int(current_project_bytes),
            "existing_opt_ros_jazzy_bytes": int(existing_ros_bytes),
            "additional_dependency_bytes_conservative": int(additional_dependency_bytes),
            "workspace_projection_bytes": int(workspace_projection_bytes),
            "projected_project_and_dependency_bytes": int(projected_total_bytes),
            "maximum_project_and_dependency_bytes": int(max_project_bytes),
            "current_windows_host_free_bytes": int(host_free_bytes),
            "projected_windows_host_growth_bytes": int(projected_host_growth_bytes),
            "projected_windows_host_free_bytes": int(projected_host_free_bytes),
            "minimum_windows_host_free_bytes": int(min_host_free_bytes),
        },
    },
    sys.stdout,
    indent=2,
    sort_keys=True,
)
sys.stdout.write("\n")
PY

mv -f -- "${SIMULATION_FILE}" "${EVIDENCE_DIR}/apt-simulation.txt"
mv -f -- "${RESOLUTION_FILE}" "${EVIDENCE_DIR}/apt-resolution.tsv"
mv -f -- "${PLANNED_FILE}" "${EVIDENCE_DIR}/apt-planned-packages.txt"
mv -f -- "${JSON_FILE}" "${EVIDENCE_DIR}/phase0-preinstall.json"

log "PASS: projected project/dependency bytes=${projected_total_bytes} (limit=${MAX_PROJECT_BYTES})."
log "PASS: projected Windows-host free bytes=${projected_host_free_bytes} (reserve=${MIN_HOST_FREE_BYTES})."
log "Evidence: ${EVIDENCE_DIR}/phase0-preinstall.json"

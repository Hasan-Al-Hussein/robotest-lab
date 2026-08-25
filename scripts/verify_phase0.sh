#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_ROOT
readonly MANIFEST="${PROJECT_ROOT}/config/apt-packages.txt"
readonly EVIDENCE_DIR="${PROJECT_ROOT}/artifacts/evidence/phase0"
readonly ROS_APT_SOURCE_VERSION="1.2.0"
readonly RUFF_VERSION="0.16.4"

declare -a PACKAGES=()
TEMP_DIR=""

log() {
  printf '[verify_phase0] %s\n' "$*"
}

die() {
  printf '[verify_phase0] ERROR: %s\n' "$*" >&2
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
    die "This verifier requires Ubuntu 24.04 Noble; Ubuntu-20.04 is protected."
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

version_at_least() {
  dpkg --compare-versions "$1" ge "$2"
}

version_less_than() {
  dpkg --compare-versions "$1" lt "$2"
}

version_matches_source_pin() {
  local version="$1"
  [[ "${version}" == "${ROS_APT_SOURCE_VERSION}" ||
    "${version}" == "${ROS_APT_SOURCE_VERSION}."* ||
    "${version}" == "${ROS_APT_SOURCE_VERSION}+"* ||
    "${version}" == "${ROS_APT_SOURCE_VERSION}~"* ]]
}

record_tool() {
  local key="$1"
  local value="$2"
  value="${value//$'\t'/ }"
  value="${value//$'\r'/}"
  value="${value//$'\n'/ | }"
  printf '%s\t%s\n' "${key}" "${value}" >>"${TOOLS_FILE}"
}

require_noble
for command_name in dpkg dpkg-query python3 sha256sum; do
  require_command "${command_name}"
done
load_manifest

mkdir -p -- "${EVIDENCE_DIR}"
TEMP_DIR="$(mktemp -d "${EVIDENCE_DIR}/.verify-phase0.XXXXXX")"
readonly PACKAGES_FILE="${TEMP_DIR}/installed-packages.tsv"
readonly TOOLS_FILE="${TEMP_DIR}/tool-versions.tsv"
readonly ROS_DOCTOR_FILE="${TEMP_DIR}/ros2-doctor.txt"
readonly JSON_FILE="${TEMP_DIR}/phase0-versions.json"

printf 'package\tversion\tarchitecture\n' >"${PACKAGES_FILE}"
for package_name in "${PACKAGES[@]}"; do
  package_record="$(
    dpkg-query -W -f='${Status}\t${Version}\t${Architecture}' "${package_name}" 2>/dev/null || true
  )"
  [[ "${package_record}" == 'install ok installed'$'\t'* ]] ||
    die "Manifest package is not installed: ${package_name}"
  package_record="${package_record#*$'\t'}"
  package_version="${package_record%%$'\t'*}"
  package_architecture="${package_record#*$'\t'}"
  printf '%s\t%s\t%s\n' \
    "${package_name}" \
    "${package_version}" \
    "${package_architecture}" >>"${PACKAGES_FILE}"
done

source_version="$(dpkg-query -W -f='${Version}' ros2-apt-source 2>/dev/null || true)"
version_matches_source_pin "${source_version}" ||
  die "ros2-apt-source ${source_version:-missing} does not match the reviewed ${ROS_APT_SOURCE_VERSION} pin."

[[ -r /opt/ros/jazzy/setup.bash ]] || die "Missing /opt/ros/jazzy/setup.bash."
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
[[ "${ROS_DISTRO:-}" == "jazzy" ]] || die "Expected ROS_DISTRO=jazzy; found ${ROS_DISTRO:-unset}."
[[ "${ROS_VERSION:-}" == "2" ]] || die "Expected ROS_VERSION=2; found ${ROS_VERSION:-unset}."

for command_name in \
  clang-format \
  clang-tidy \
  cmake \
  colcon \
  cppcheck \
  g++ \
  git \
  go \
  gz \
  lintian \
  pre-commit \
  ros2 \
  rosdep \
  shellcheck; do
  require_command "${command_name}"
done

python_version="$(python3 -c 'import platform; print(platform.python_version())')"
version_at_least "${python_version}" "3.12" || die "Python ${python_version} is older than 3.12."
version_less_than "${python_version}" "3.13" || die "Python ${python_version} is outside the reviewed 3.12 series."

cpp_version="$(g++ -dumpfullversion -dumpversion)"
version_at_least "${cpp_version}" "13" || die "G++ ${cpp_version} is older than the Noble GCC 13 baseline."

cmake_version="$(cmake --version | awk 'NR == 1 { print $3 }')"
version_at_least "${cmake_version}" "3.28" || die "CMake ${cmake_version} is older than 3.28."

go_version="$(go version | awk '{ sub(/^go/, "", $3); print $3 }')"
version_at_least "${go_version}" "1.22" || die "Go ${go_version} is older than 1.22."
version_less_than "${go_version}" "2" || die "Unexpected Go major version: ${go_version}."

colcon_version="$(python3 -c 'from importlib.metadata import version; print(version("colcon-core"))')"
[[ -n "${colcon_version}" ]] || die "Could not resolve the colcon-core version."

if ! gazebo_output="$(gz sim --version 2>&1)"; then
  gazebo_output="$(gz sim --versions 2>&1)" || die "Unable to query Gazebo Sim version."
fi
gazebo_version="$(
  grep -Eo '[0-9]+\.[0-9]+\.[0-9]+' <<<"${gazebo_output}" |
    awk '!seen { print; seen = 1 }'
)"
[[ "${gazebo_version}" == 8.* ]] ||
  die "Expected Gazebo Harmonic / Sim 8.x; found ${gazebo_version:-unknown}."

ros_gz_prefix="$(ros2 pkg prefix ros_gz_bridge 2>/dev/null)" || die "ros_gz_bridge is not discoverable."
nav2_prefix="$(ros2 pkg prefix nav2_bringup 2>/dev/null)" || die "nav2_bringup is not discoverable."
[[ "${ros_gz_prefix}" == /opt/ros/jazzy* ]] || die "ros_gz_bridge resolved outside the Jazzy underlay: ${ros_gz_prefix}."
[[ "${nav2_prefix}" == /opt/ros/jazzy* ]] || die "nav2_bringup resolved outside the Jazzy underlay: ${nav2_prefix}."

if ! ros2 doctor --report >"${ROS_DOCTOR_FILE}" 2>&1; then
  cat "${ROS_DOCTOR_FILE}" >&2
  die "ros2 doctor --report failed."
fi

python3 -c 'import rclpy, yaml, jsonschema, matplotlib' ||
  die "One or more required Python runtime modules cannot be imported."

[[ -x "${PROJECT_ROOT}/.venv/bin/ruff" ]] || die "Missing repository-local Ruff executable."
ruff_version="$("${PROJECT_ROOT}/.venv/bin/ruff" --version | awk '{ print $2 }')"
[[ "${ruff_version}" == "${RUFF_VERSION}" ]] ||
  die "Expected Ruff ${RUFF_VERSION}; found ${ruff_version:-unknown}."

[[ -s /etc/ros/rosdep/sources.list.d/20-default.list ]] || die "rosdep is not initialized."

printf 'tool\tversion\n' >"${TOOLS_FILE}"
record_tool os "Ubuntu 24.04 Noble"
record_tool ros_distro "${ROS_DISTRO}"
record_tool ros_apt_source "${source_version}"
record_tool ros_base "$(dpkg-query -W -f='${Version}' ros-jazzy-ros-base)"
record_tool ros_gz_bridge_prefix "${ros_gz_prefix}"
record_tool nav2_bringup_prefix "${nav2_prefix}"
record_tool gazebo_sim "${gazebo_version}"
record_tool colcon_core "${colcon_version}"
record_tool cpp_compiler "g++ ${cpp_version}"
record_tool cmake "${cmake_version}"
record_tool python "${python_version}"
record_tool go "${go_version}"
record_tool ruff "${ruff_version}"
record_tool clang_format "$(clang-format --version)"
record_tool clang_tidy "$(clang-tidy --version | sed -n '1p')"
record_tool cppcheck "$(cppcheck --version)"
record_tool pre_commit "$(pre-commit --version | awk '{ print $2 }')"
record_tool shellcheck "$(shellcheck --version | awk '$1 == "version:" { print $2 }')"
record_tool lintian "$(lintian --version | sed -n '1p')"
record_tool git "$(git --version | awk '{ print $3 }')"

manifest_sha256="$(sha256sum "${MANIFEST}" | awk '{ print $1 }')"
python3 - \
  "${manifest_sha256}" \
  "${PACKAGES_FILE}" \
  "${TOOLS_FILE}" >"${JSON_FILE}" <<'PY'
import csv
import json
import sys
from datetime import datetime, timezone

manifest_sha256, packages_path, tools_path = sys.argv[1:]
with open(packages_path, encoding="utf-8", newline="") as handle:
    packages = list(csv.DictReader(handle, delimiter="\t"))
with open(tools_path, encoding="utf-8", newline="") as handle:
    tools = {row["tool"]: row["version"] for row in csv.DictReader(handle, delimiter="\t")}

json.dump(
    {
        "schema_version": 1,
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "status": "passed",
        "verification_scope": "Installed-tool provenance; no ROS graph or simulation was executed",
        "manifest": {
            "path": "config/apt-packages.txt",
            "sha256": manifest_sha256,
            "installed_entry_count": len(packages),
            "packages": packages,
        },
        "tools": tools,
    },
    sys.stdout,
    indent=2,
    sort_keys=True,
)
sys.stdout.write("\n")
PY

mv -f -- "${PACKAGES_FILE}" "${EVIDENCE_DIR}/installed-packages.tsv"
mv -f -- "${TOOLS_FILE}" "${EVIDENCE_DIR}/tool-versions.tsv"
mv -f -- "${ROS_DOCTOR_FILE}" "${EVIDENCE_DIR}/ros2-doctor.txt"
mv -f -- "${JSON_FILE}" "${EVIDENCE_DIR}/phase0-versions.json"

log "PASS: all ${#PACKAGES[@]} manifest packages are installed."
log "PASS: Jazzy, Gazebo Harmonic, ros_gz, Nav2, colcon, C++, Python, and Go provenance verified."
log "Evidence: ${EVIDENCE_DIR}/phase0-versions.json"

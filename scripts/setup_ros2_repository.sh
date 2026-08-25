#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_ROOT
readonly EVIDENCE_DIR="${PROJECT_ROOT}/artifacts/evidence/phase0"
readonly ROS_APT_SOURCE_VERSION="1.2.0"
readonly ROS_APT_SOURCE_ASSET="ros2-apt-source_1.2.0.noble_all.deb"
readonly ROS_APT_SOURCE_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/${ROS_APT_SOURCE_ASSET}"
readonly ROS_APT_SOURCE_SHA256="0804d9b13db770eb87019be414cd78378835228ad5fa801fc88758596dd8f7e5"

MODE="check"
TEMP_DIR=""

log() {
  printf '[setup_ros2_repository] %s\n' "$*"
}

die() {
  printf '[setup_ros2_repository] ERROR: %s\n' "$*" >&2
  exit 1
}

cleanup() {
  if [[ -n "${TEMP_DIR}" && -d "${TEMP_DIR}" ]]; then
    rm -rf -- "${TEMP_DIR}"
  fi
}
trap cleanup EXIT

usage() {
  cat <<'EOF'
Usage: scripts/setup_ros2_repository.sh [--check|--apply]

  --check  Verify the pinned official ROS apt source without changing the OS.
  --apply  Install the SHA-256-verified apt-source package and refresh indexes.

The script accepts no implicit apply mode and rejects non-Ubuntu-24.04 hosts.
EOF
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

require_noble() {
  [[ -r /etc/os-release ]] || die "/etc/os-release is unavailable."

  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" ]] || die "Expected Ubuntu; found ${ID:-unknown}."
  [[ "${VERSION_ID:-}" == "24.04" ]] ||
    die "Expected Ubuntu 24.04; found ${VERSION_ID:-unknown}. Ubuntu-20.04 is protected."
  [[ "${VERSION_CODENAME:-}" == "noble" ]] ||
    die "Expected Noble; found ${VERSION_CODENAME:-unknown}."
  [[ "${WSL_DISTRO_NAME:-}" != "Ubuntu-20.04" ]] ||
    die "Refusing to operate in the protected Ubuntu-20.04 distribution."
}

version_matches_pin() {
  local version="$1"
  [[ "${version}" == "${ROS_APT_SOURCE_VERSION}" ||
    "${version}" == "${ROS_APT_SOURCE_VERSION}."* ||
    "${version}" == "${ROS_APT_SOURCE_VERSION}+"* ||
    "${version}" == "${ROS_APT_SOURCE_VERSION}~"* ]]
}

installed_source_version() {
  dpkg-query -W -f='${Version}' ros2-apt-source 2>/dev/null || return 1
}

official_source_file() {
  local candidate
  local -a candidates=()

  mapfile -t candidates < <(
    dpkg-query -L ros2-apt-source 2>/dev/null |
      grep -E '^/etc/apt/sources\.list\.d/.*\.(list|sources)$' || true
  )

  for candidate in "${candidates[@]}"; do
    [[ -f "${candidate}" ]] || continue
    if grep -Fq 'packages.ros.org/ros2/ubuntu' "${candidate}"; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

verify_repository() {
  local installed_version source_file candidate_version

  installed_version="$(installed_source_version)" ||
    die "ros2-apt-source is not installed. Run this script with --apply."
  version_matches_pin "${installed_version}" ||
    die "Installed ros2-apt-source version ${installed_version} does not match the reviewed ${ROS_APT_SOURCE_VERSION} pin."

  source_file="$(official_source_file)" ||
    die "The installed package does not expose an official packages.ros.org ROS 2 source."

  candidate_version="$(
    apt-cache policy ros-jazzy-ros-base 2>/dev/null |
      awk '$1 == "Candidate:" && !seen { print $2; seen = 1 }'
  )"
  [[ -n "${candidate_version}" && "${candidate_version}" != "(none)" ]] ||
    die "No ros-jazzy-ros-base candidate is visible. Refresh the official indexes with --apply."

  mkdir -p -- "${EVIDENCE_DIR}"
  local evidence_tmp
  evidence_tmp="$(mktemp "${EVIDENCE_DIR}/.ros-apt-source.XXXXXX")"
  python3 - \
    "${MODE}" \
    "${installed_version}" \
    "${source_file}" \
    "${candidate_version}" \
    "${ROS_APT_SOURCE_URL}" \
    "${ROS_APT_SOURCE_SHA256}" >"${evidence_tmp}" <<'PY'
import json
import sys
from datetime import datetime, timezone

mode, installed_version, source_file, candidate, url, sha256 = sys.argv[1:]
json.dump(
    {
        "schema_version": 1,
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "status": "passed",
        "mode": mode,
        "ros_apt_source": {
            "package": "ros2-apt-source",
            "installed_version": installed_version,
            "source_file": source_file,
            "ros_jazzy_ros_base_candidate": candidate,
            "asset_url": url,
            "asset_sha256": sha256,
        },
    },
    sys.stdout,
    indent=2,
    sort_keys=True,
)
sys.stdout.write("\n")
PY
  mv -f -- "${evidence_tmp}" "${EVIDENCE_DIR}/ros-apt-source.json"

  log "Verified ros2-apt-source ${installed_version}."
  log "Official source: ${source_file}"
  log "ros-jazzy-ros-base candidate: ${candidate_version}"
}

apply_repository() {
  local installed_version deb_path actual_sha package_name package_version

  require_command curl
  require_command sha256sum
  require_command dpkg-deb
  require_command apt-get
  require_command sudo

  if installed_version="$(installed_source_version)"; then
    version_matches_pin "${installed_version}" ||
      die "ros2-apt-source ${installed_version} is already installed. Refusing an implicit downgrade or replacement."
    log "Pinned ros2-apt-source ${installed_version} is already installed."
  else
    TEMP_DIR="$(mktemp -d)"
    deb_path="${TEMP_DIR}/${ROS_APT_SOURCE_ASSET}"

    log "Downloading the reviewed official ROS apt-source asset."
    curl \
      --fail \
      --location \
      --proto '=https' \
      --show-error \
      --silent \
      --tlsv1.2 \
      --output "${deb_path}" \
      "${ROS_APT_SOURCE_URL}"

    actual_sha="$(sha256sum "${deb_path}" | awk '{ print $1 }')"
    [[ "${actual_sha}" == "${ROS_APT_SOURCE_SHA256}" ]] ||
      die "SHA-256 mismatch for ${ROS_APT_SOURCE_ASSET}: ${actual_sha}."

    package_name="$(dpkg-deb --field "${deb_path}" Package)"
    package_version="$(dpkg-deb --field "${deb_path}" Version)"
    [[ "${package_name}" == "ros2-apt-source" ]] ||
      die "Unexpected package name in reviewed asset: ${package_name}."
    version_matches_pin "${package_version}" ||
      die "Unexpected package version in reviewed asset: ${package_version}."

    log "Installing the verified ${package_name} ${package_version} package."
    sudo apt-get install --yes "${deb_path}"
  fi

  log "Refreshing apt indexes after official ROS source setup."
  sudo apt-get update
}

if (($# > 1)); then
  usage >&2
  exit 2
fi

if (($# == 1)); then
  case "$1" in
    --check)
      MODE="check"
      ;;
    --apply)
      MODE="apply"
      ;;
    --help | -h)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
fi

require_noble
require_command dpkg-query
require_command apt-cache
require_command python3

if [[ "${MODE}" == "apply" ]]; then
  apply_repository
fi

verify_repository

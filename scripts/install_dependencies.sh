#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_ROOT
readonly MANIFEST="${PROJECT_ROOT}/config/apt-packages.txt"
readonly EVIDENCE_DIR="${PROJECT_ROOT}/artifacts/evidence/phase0"
readonly RUFF_VERSION="0.16.4"

declare -a PACKAGES=()

log() {
  printf '[install_dependencies] %s\n' "$*"
}

die() {
  printf '[install_dependencies] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: scripts/install_dependencies.sh --apply

Installs the exact validated apt manifest, initializes rosdep, creates the
repository-local Python tooling environment, and runs Phase 0 verification.
There is no implicit install mode.
EOF
}

require_noble() {
  [[ -r /etc/os-release ]] || die "/etc/os-release is unavailable."
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" && "${VERSION_CODENAME:-}" == "noble" ]] ||
    die "This installer requires Ubuntu 24.04 Noble; Ubuntu-20.04 is protected."
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

if (($# != 1)) || [[ "$1" != "--apply" ]]; then
  usage >&2
  exit 2
fi

require_noble
command -v sudo >/dev/null 2>&1 || die "sudo is required for the explicit install step."
load_manifest

"${SCRIPT_DIR}/setup_ros2_repository.sh" --check
"${SCRIPT_DIR}/verify_phase0_preinstall.sh"

mkdir -p -- "${EVIDENCE_DIR}"
transcript_tmp="$(mktemp "${EVIDENCE_DIR}/.install-transcript.XXXXXX")"
trap 'rm -f -- "${transcript_tmp}"' EXIT

log "Installing ${#PACKAGES[@]} validated manifest entries and resolver-required dependencies."
sudo env DEBIAN_FRONTEND=noninteractive apt-get \
  install \
  --yes \
  "${PACKAGES[@]}" 2>&1 | tee "${transcript_tmp}"
mv -f -- "${transcript_tmp}" "${EVIDENCE_DIR}/install-transcript.txt"
trap - EXIT

if [[ ! -s /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  log "Initializing the system rosdep source list."
  sudo rosdep init
fi
log "Refreshing the Jazzy rosdep cache for the current user."
rosdep update --rosdistro jazzy

if [[ -f "${PROJECT_ROOT}/.venv/pyvenv.cfg" ]] &&
  ! grep -Eq '^include-system-site-packages = true$' "${PROJECT_ROOT}/.venv/pyvenv.cfg"; then
  die "Existing .venv does not expose ROS system packages; remove it explicitly and rerun."
fi

if [[ ! -x "${PROJECT_ROOT}/.venv/bin/python" ]]; then
  log "Creating the Python 3.12 tooling environment with ROS system packages visible."
  python3 -m venv --system-site-packages "${PROJECT_ROOT}/.venv"
fi

log "Installing the pinned repository-local Ruff ${RUFF_VERSION} tool."
"${PROJECT_ROOT}/.venv/bin/python" -m pip install \
  --disable-pip-version-check \
  --no-deps \
  "ruff==${RUFF_VERSION}"

"${SCRIPT_DIR}/verify_phase0.sh"

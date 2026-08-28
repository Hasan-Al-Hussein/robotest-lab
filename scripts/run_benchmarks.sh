#!/usr/bin/env bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

set -Eeuo pipefail
IFS=$'\n\t'
umask 022

SCRIPT_PATH="$(readlink -f -- "$0")"
WORKSPACE="$(cd -- "$(dirname -- "$SCRIPT_PATH")/.." && pwd -P)"
ROBOTEST_CPUSET="${ROBOTEST_CPUSET:-0-5}"

if [[ "$#" -eq 1 && ( "$1" == '--help' || "$1" == '-h' ) ]]; then
  exec python3 "$WORKSPACE/tests/phase3_benchmark_runner.py" \
    --workspace "$WORKSPACE" "$1"
fi

if [[ "$ROBOTEST_CPUSET" != "0-5" ]]; then
  echo 'Phase 3 candidate execution is frozen to ROBOTEST_CPUSET=0-5' >&2
  exit 2
fi

if [[ "${ROBOTEST_PHASE3_AFFINITY_APPLIED:-0}" != "1" ]]; then
  exec taskset -c 0-5 env \
    ROBOTEST_PHASE3_AFFINITY_APPLIED=1 \
    ROBOTEST_CPUSET=0-5 \
    "$SCRIPT_PATH" "$@"
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

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo 'ROS 2 Jazzy setup is unavailable' >&2
  exit 2
fi
if [[ ! -f "$WORKSPACE/install/setup.bash" ]]; then
  echo 'installed overlay is unavailable; run scripts/verify_phase3.sh first' >&2
  exit 2
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"
set -u

export CMAKE_BUILD_PARALLEL_LEVEL=4
export MAKEFLAGS=-j4
export RCUTILS_COLORIZED_OUTPUT=0
export ROS2CLI_NO_DAEMON=1

exec python3 "$WORKSPACE/tests/phase3_benchmark_runner.py" \
  --workspace "$WORKSPACE" "$@"

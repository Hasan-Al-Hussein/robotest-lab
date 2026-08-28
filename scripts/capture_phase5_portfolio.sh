#!/usr/bin/env bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

set -Eeuo pipefail
IFS=$'\n\t'
umask 022
export PYTHONDONTWRITEBYTECODE=1

SCRIPT_PATH="$(readlink -f -- "$0")"
SCRIPT_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd -P)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly SCRIPT_PATH SCRIPT_DIR PROJECT_ROOT
readonly HELPER="${PROJECT_ROOT}/tests/phase5_portfolio_evidence.py"

MODE=""
REPOSITORY=""
CANDIDATE_SHA=""
PHASE3_CANDIDATE_ROOT=""
ATTEMPT_ID=""
PORTFOLIO_ROOT=""
PORTFOLIO_PROOF=""
ARCHITECTURE_SVG=""
RELEASE_FLOW_SVG=""
VISUAL_REVIEW=""
AUTHORIZE_MUTATING_COMMANDS=0

die() {
  printf '[capture_phase5_portfolio] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage:
  scripts/capture_phase5_portfolio.sh --documentation-replay \
    --repository <path> \
    --candidate-sha <40-character-lowercase-commit-sha> \
    --phase3-candidate-root <path> \
    [--attempt-id <YYYYMMDDTHHMMSSZ-serial>] \
    [--authorize-mutating-commands]

  scripts/capture_phase5_portfolio.sh --finalize \
    --repository <path> \
    --candidate-sha <40-character-lowercase-commit-sha> \
    --phase3-candidate-root <path> \
    --attempt-id <YYYYMMDDTHHMMSSZ-serial> \
    --architecture-svg <path> \
    --release-flow-svg <path> \
    --visual-review <path>

  scripts/capture_phase5_portfolio.sh --project \
    --repository <path> \
    --candidate-sha <40-character-lowercase-commit-sha> \
    --phase3-candidate-root <path> \
    --portfolio-root <path> \
    --attempt-id <YYYYMMDDTHHMMSSZ-serial>

  scripts/capture_phase5_portfolio.sh --validate \
    --repository <path> \
    --candidate-sha <40-character-lowercase-commit-sha> \
    --phase3-candidate-root <path> \
    --portfolio-root <path> \
    --portfolio-proof <path>

The documentation replay uses a fresh detached checkout of the exact candidate
for every README command unit. It never overwrites an existing attempt. A
successful replay exits 3 after writing a hash-bound render request because
separately rendered SVG bytes and an explicit human PASS review are required.
The attempt serial is 1-10 digits and its first digit is 1-9.
EOF
}

set_mode() {
  local requested="$1"
  [[ -z "${MODE}" ]] || die "Exactly one mode is required; already selected ${MODE}."
  MODE="${requested}"
}

require_value() {
  local option="$1"
  local remaining="$2"
  ((remaining >= 2)) || die "${option} requires a value."
}

while (($# > 0)); do
  case "$1" in
    --documentation-replay)
      set_mode prepare
      shift
      ;;
    --finalize)
      set_mode finalize
      shift
      ;;
    --project)
      set_mode project
      shift
      ;;
    --validate)
      set_mode validate
      shift
      ;;
    --repository)
      require_value "$1" "$#"
      [[ -z "${REPOSITORY}" ]] || die 'Duplicate --repository.'
      REPOSITORY="$2"
      shift 2
      ;;
    --candidate-sha)
      require_value "$1" "$#"
      [[ -z "${CANDIDATE_SHA}" ]] || die 'Duplicate --candidate-sha.'
      CANDIDATE_SHA="$2"
      shift 2
      ;;
    --phase3-candidate-root)
      require_value "$1" "$#"
      [[ -z "${PHASE3_CANDIDATE_ROOT}" ]] || die 'Duplicate --phase3-candidate-root.'
      PHASE3_CANDIDATE_ROOT="$2"
      shift 2
      ;;
    --attempt-id)
      require_value "$1" "$#"
      [[ -z "${ATTEMPT_ID}" ]] || die 'Duplicate --attempt-id.'
      ATTEMPT_ID="$2"
      shift 2
      ;;
    --portfolio-root)
      require_value "$1" "$#"
      [[ -z "${PORTFOLIO_ROOT}" ]] || die 'Duplicate --portfolio-root.'
      PORTFOLIO_ROOT="$2"
      shift 2
      ;;
    --portfolio-proof)
      require_value "$1" "$#"
      [[ -z "${PORTFOLIO_PROOF}" ]] || die 'Duplicate --portfolio-proof.'
      PORTFOLIO_PROOF="$2"
      shift 2
      ;;
    --architecture-svg)
      require_value "$1" "$#"
      [[ -z "${ARCHITECTURE_SVG}" ]] || die 'Duplicate --architecture-svg.'
      ARCHITECTURE_SVG="$2"
      shift 2
      ;;
    --release-flow-svg)
      require_value "$1" "$#"
      [[ -z "${RELEASE_FLOW_SVG}" ]] || die 'Duplicate --release-flow-svg.'
      RELEASE_FLOW_SVG="$2"
      shift 2
      ;;
    --visual-review)
      require_value "$1" "$#"
      [[ -z "${VISUAL_REVIEW}" ]] || die 'Duplicate --visual-review.'
      VISUAL_REVIEW="$2"
      shift 2
      ;;
    --authorize-mutating-commands)
      ((AUTHORIZE_MUTATING_COMMANDS == 0)) || die 'Duplicate --authorize-mutating-commands.'
      AUTHORIZE_MUTATING_COMMANDS=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      die 'Positional arguments are not accepted.'
      ;;
    *)
      die "Unknown argument: $1"
      ;;
  esac
done

[[ -n "${MODE}" ]] || die 'Exactly one mode is required.'
[[ -n "${REPOSITORY}" ]] || die '--repository is required.'
[[ -n "${CANDIDATE_SHA}" ]] || die '--candidate-sha is required.'
[[ -n "${PHASE3_CANDIDATE_ROOT}" ]] || die '--phase3-candidate-root is required.'
[[ -f "${HELPER}" && ! -L "${HELPER}" ]] || die "Missing regular helper: ${HELPER}"
command -v python3 >/dev/null 2>&1 || die 'Required command not found: python3'

common_arguments=(
  --repository "${REPOSITORY}"
  --candidate-sha "${CANDIDATE_SHA}"
  --phase3-candidate-root "${PHASE3_CANDIDATE_ROOT}"
)

case "${MODE}" in
  prepare)
    [[ -z "${PORTFOLIO_ROOT}${PORTFOLIO_PROOF}${ARCHITECTURE_SVG}${RELEASE_FLOW_SVG}${VISUAL_REVIEW}" ]] ||
      die 'Replay received an option reserved for a later stage.'
    prepare_arguments=("${common_arguments[@]}")
    if [[ -n "${ATTEMPT_ID}" ]]; then
      prepare_arguments+=(--attempt-id "${ATTEMPT_ID}")
    fi
    if ((AUTHORIZE_MUTATING_COMMANDS == 1)); then
      prepare_arguments+=(--authorize-mutating-commands)
    fi
    exec python3 "${HELPER}" prepare "${prepare_arguments[@]}"
    ;;
  finalize)
    ((AUTHORIZE_MUTATING_COMMANDS == 0)) ||
      die '--authorize-mutating-commands is valid only with --documentation-replay.'
    [[ -z "${PORTFOLIO_ROOT}${PORTFOLIO_PROOF}" ]] ||
      die 'Finalize received an option reserved for another stage.'
    [[ -n "${ATTEMPT_ID}" ]] || die '--attempt-id is required with --finalize.'
    [[ -n "${ARCHITECTURE_SVG}" ]] || die '--architecture-svg is required with --finalize.'
    [[ -n "${RELEASE_FLOW_SVG}" ]] || die '--release-flow-svg is required with --finalize.'
    [[ -n "${VISUAL_REVIEW}" ]] || die '--visual-review is required with --finalize.'
    exec python3 "${HELPER}" finalize \
      "${common_arguments[@]}" \
      --attempt-id "${ATTEMPT_ID}" \
      --architecture-svg "${ARCHITECTURE_SVG}" \
      --release-flow-svg "${RELEASE_FLOW_SVG}" \
      --visual-review "${VISUAL_REVIEW}"
    ;;
  project)
    ((AUTHORIZE_MUTATING_COMMANDS == 0)) ||
      die '--authorize-mutating-commands is valid only with --documentation-replay.'
    [[ -z "${PORTFOLIO_PROOF}${ARCHITECTURE_SVG}${RELEASE_FLOW_SVG}${VISUAL_REVIEW}" ]] ||
      die 'Project received an option reserved for another stage.'
    [[ -n "${ATTEMPT_ID}" ]] || die '--attempt-id is required with --project.'
    [[ -n "${PORTFOLIO_ROOT}" ]] || die '--portfolio-root is required with --project.'
    exec python3 "${HELPER}" project \
      "${common_arguments[@]}" \
      --portfolio-root "${PORTFOLIO_ROOT}" \
      --attempt-id "${ATTEMPT_ID}"
    ;;
  validate)
    ((AUTHORIZE_MUTATING_COMMANDS == 0)) ||
      die '--authorize-mutating-commands is valid only with --documentation-replay.'
    [[ -z "${ATTEMPT_ID}${ARCHITECTURE_SVG}${RELEASE_FLOW_SVG}${VISUAL_REVIEW}" ]] ||
      die 'Validate received an option reserved for another stage.'
    [[ -n "${PORTFOLIO_ROOT}" ]] || die '--portfolio-root is required with --validate.'
    [[ -n "${PORTFOLIO_PROOF}" ]] || die '--portfolio-proof is required with --validate.'
    exec python3 "${HELPER}" validate \
      "${common_arguments[@]}" \
      --portfolio-root "${PORTFOLIO_ROOT}" \
      --portfolio-proof "${PORTFOLIO_PROOF}"
    ;;
  *)
    die "Internal mode error: ${MODE}"
    ;;
esac

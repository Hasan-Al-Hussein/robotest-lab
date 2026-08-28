#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_ROOT
ORIGINAL_CWD="$(pwd -P)"
readonly ORIGINAL_CWD
readonly EVIDENCE_DIR="${PROJECT_ROOT}/artifacts/evidence/phase0"
readonly RELEASE_EVIDENCE_DIR="${PROJECT_ROOT}/artifacts/evidence/phase5/release"
readonly RELEASE_HELPER="${PROJECT_ROOT}/tests/phase5_release_evidence.py"
readonly LOCAL_RESULT_NAME='verify-all.json'
readonly RELEASE_RESULT_NAME='release-evidence.json'
readonly -a ALLOWED_GENERATED_DELTA=(
  'artifacts/evidence/phase0/installed-packages.tsv'
  'artifacts/evidence/phase0/phase0-versions.json'
  'artifacts/evidence/phase0/ros2-doctor.txt'
  'artifacts/evidence/phase0/tool-versions.tsv'
)

mode='static'
local_aggregate=''
phase3_candidate_root=''
phase3_aggregate=''
phase4_run_directory=''
phase4_scenario6=''
phase5_portfolio_root=''
phase5_portfolio_proof=''
phase5_remote_proof=''
phase5_evidence_commit_remote_proof=''

usage() {
  cat <<'EOF'
Usage:
  scripts/verify_all.sh
  scripts/verify_all.sh --release-evidence \
    --local-aggregate <exact-prior-verify-all.json> \
    --phase3-candidate-root <exact-path> \
    --phase3-aggregate <exact-path> \
    --phase4-run-directory <exact-path> \
    --phase4-scenario6 <exact-path> \
    --phase5-portfolio-root <exact-path> \
    --phase5-portfolio-proof <exact-path> \
    --phase5-remote-proof <exact-tracked-json-path> \
    --phase5-evidence-commit-remote-proof <exact-ignored-json-path>

Bare mode runs the normal Phase 0-5 verifier routing, invokes Phase 5 only as
--local, and exits 3/INCOMPLETE because it never authorizes acceptance work.

--release-evidence is evidence-only. It invokes no phase verifier and only
revalidates the nine exact caller-selected paths. It never starts the Phase 3
campaign, invokes Phase 4 --apply, publishes, or discovers latest evidence.
The prior local aggregate plus Phase 3, Phase 4, portfolio, and public-CI
evidence must bind one clean candidate Git SHA. Its separate result never
overwrites the prior aggregate.
EOF
}

if (($# == 1)) && [[ "$1" == '--help' || "$1" == '-h' ]]; then
  usage
  exit 0
elif (($# > 0)); then
  [[ "$1" == '--release-evidence' ]] || {
    usage >&2
    exit 2
  }
  mode='release-evidence'
  shift
  while (($#)); do
    (($# >= 2)) || {
      usage >&2
      exit 2
    }
    case "$1" in
      --local-aggregate)
        [[ -z "${local_aggregate}" ]] || {
          printf 'Duplicate %s\n' "$1" >&2
          exit 2
        }
        local_aggregate="$2"
        ;;
      --phase3-candidate-root)
        [[ -z "${phase3_candidate_root}" ]] || {
          printf 'Duplicate %s\n' "$1" >&2
          exit 2
        }
        phase3_candidate_root="$2"
        ;;
      --phase3-aggregate)
        [[ -z "${phase3_aggregate}" ]] || {
          printf 'Duplicate %s\n' "$1" >&2
          exit 2
        }
        phase3_aggregate="$2"
        ;;
      --phase4-run-directory)
        [[ -z "${phase4_run_directory}" ]] || {
          printf 'Duplicate %s\n' "$1" >&2
          exit 2
        }
        phase4_run_directory="$2"
        ;;
      --phase4-scenario6)
        [[ -z "${phase4_scenario6}" ]] || {
          printf 'Duplicate %s\n' "$1" >&2
          exit 2
        }
        phase4_scenario6="$2"
        ;;
      --phase5-portfolio-root)
        [[ -z "${phase5_portfolio_root}" ]] || {
          printf 'Duplicate %s\n' "$1" >&2
          exit 2
        }
        phase5_portfolio_root="$2"
        ;;
      --phase5-portfolio-proof)
        [[ -z "${phase5_portfolio_proof}" ]] || {
          printf 'Duplicate %s\n' "$1" >&2
          exit 2
        }
        phase5_portfolio_proof="$2"
        ;;
      --phase5-remote-proof)
        [[ -z "${phase5_remote_proof}" ]] || {
          printf 'Duplicate %s\n' "$1" >&2
          exit 2
        }
        phase5_remote_proof="$2"
        ;;
      --phase5-evidence-commit-remote-proof)
        [[ -z "${phase5_evidence_commit_remote_proof}" ]] || {
          printf 'Duplicate %s\n' "$1" >&2
          exit 2
        }
        phase5_evidence_commit_remote_proof="$2"
        ;;
      *)
        usage >&2
        exit 2
        ;;
    esac
    shift 2
  done
  [[ -n "${local_aggregate}" && -n "${phase3_candidate_root}" && \
    -n "${phase3_aggregate}" && -n "${phase4_run_directory}" && \
    -n "${phase4_scenario6}" && -n "${phase5_portfolio_root}" && \
    -n "${phase5_portfolio_proof}" && -n "${phase5_remote_proof}" && \
    -n "${phase5_evidence_commit_remote_proof}" ]] || {
    usage >&2
    exit 2
  }
fi
readonly mode local_aggregate phase3_candidate_root phase3_aggregate
readonly phase4_run_directory phase4_scenario6 phase5_remote_proof
readonly phase5_portfolio_root phase5_portfolio_proof
readonly phase5_evidence_commit_remote_proof

WORK_ROOT="$(mktemp -d /tmp/robotest-verify-all.XXXXXX)"
readonly WORK_ROOT
cleanup() {
  [[ "${WORK_ROOT}" == /tmp/robotest-verify-all.* && -d "${WORK_ROOT}" && \
    ! -L "${WORK_ROOT}" ]] || return 1
  rm -rf -- "${WORK_ROOT}"
}
trap cleanup EXIT

prepare_destination() {
  local destination="$1"
  local relative current component
  local -a components=()
  [[ "${destination}" == "${PROJECT_ROOT}/"* ]] || {
    printf '[verify_all] Evidence destination escapes the repository: %s\n' \
      "${destination}" >&2
    return 1
  }
  relative="${destination#"${PROJECT_ROOT}/"}"
  current="${PROJECT_ROOT}"
  IFS='/' read -r -a components <<<"${relative}"
  for component in "${components[@]}"; do
    [[ -n "${component}" && "${component}" != '.' && "${component}" != '..' ]] || return 1
    current="${current}/${component}"
    [[ ! -L "${current}" ]] || {
      printf '[verify_all] Evidence destination contains a symlink: %s\n' "${current}" >&2
      return 1
    }
    if [[ -e "${current}" ]]; then
      [[ -d "${current}" ]] || return 1
    else
      mkdir -- "${current}"
    fi
  done
  [[ "$(realpath --canonicalize-existing -- "${destination}")" == "${destination}" ]]
}

checksum_result() {
  local root="$1"
  local source_name="$2"
  local stem="${source_name%.json}"
  (
    cd -- "${root}"
    LC_ALL=C sha256sum "${source_name}" >"${stem}.SHA256SUMS"
    LC_ALL=C sha256sum -c --strict "${stem}.SHA256SUMS" \
      >"${stem}.checksum-validation.txt"
  )
}

publish_result() {
  local root="$1"
  local source_name="$2"
  local destination="$3"
  local stem="${source_name%.json}"
  prepare_destination "${destination}"
  mv -f -- "${root}/${source_name}" "${destination}/${source_name}"
  mv -f -- "${root}/${stem}.SHA256SUMS" "${destination}/${stem}.SHA256SUMS"
  mv -f -- "${root}/${stem}.checksum-validation.txt" \
    "${destination}/${stem}.checksum-validation.txt"
}

if [[ "${mode}" == 'release-evidence' ]]; then
  [[ -f "${RELEASE_HELPER}" && ! -L "${RELEASE_HELPER}" ]] || {
    printf '[verify_all] Missing regular release helper: %s\n' "${RELEASE_HELPER}" >&2
    exit 2
  }
  release_output="${WORK_ROOT}/${RELEASE_RESULT_NAME}"
  printf '[verify_all] Revalidating exact caller-selected evidence; no verifier is invoked.\n'
  if GIT_OPTIONAL_LOCKS=0 GIT_NO_LAZY_FETCH=1 GIT_TERMINAL_PROMPT=0 \
    python3 -B "${RELEASE_HELPER}" \
    --repository "${PROJECT_ROOT}" \
    --local-aggregate "${local_aggregate}" \
    --phase3-candidate-root "${phase3_candidate_root}" \
    --phase3-aggregate "${phase3_aggregate}" \
    --phase4-run-directory "${phase4_run_directory}" \
    --phase4-scenario6 "${phase4_scenario6}" \
    --phase5-portfolio-root "${phase5_portfolio_root}" \
    --phase5-portfolio-proof "${phase5_portfolio_proof}" \
    --phase5-remote-proof "${phase5_remote_proof}" \
    --phase5-evidence-commit-remote-proof "${phase5_evidence_commit_remote_proof}" \
    --output "${release_output}"; then
    checksum_result "${WORK_ROOT}" "${RELEASE_RESULT_NAME}"
    publish_result "${WORK_ROOT}" "${RELEASE_RESULT_NAME}" "${RELEASE_EVIDENCE_DIR}"
    trap - EXIT
    cleanup
    printf '[verify_all] PASS: release_eligible=true. Evidence: %s/%s\n' \
      "${RELEASE_EVIDENCE_DIR}" "${RELEASE_RESULT_NAME}"
    exit 0
  else
    release_status=$?
  fi
  python3 -B - "${release_output}" <<'PY'
import json
import pathlib
import sys
from datetime import datetime, timezone

path = pathlib.Path(sys.argv[1])
path.write_text(
    json.dumps(
        {
            'checked_at': datetime.now(timezone.utc).isoformat(),
            'release_eligible': False,
            'schema_version': 2,
            'status': 'FAIL',
            'verification_scope': (
                'Evidence-only validation failed; no phase verifier or acceptance action ran'
            ),
        },
        separators=(',', ':'),
        sort_keys=True,
    )
    + '\n',
    encoding='utf-8',
)
PY
  checksum_result "${WORK_ROOT}" "${RELEASE_RESULT_NAME}"
  publish_result "${WORK_ROOT}" "${RELEASE_RESULT_NAME}" "${RELEASE_EVIDENCE_DIR}"
  trap - EXIT
  cleanup
  printf '[verify_all] FAILED release evidence. Evidence: %s/%s\n' \
    "${RELEASE_EVIDENCE_DIR}" "${RELEASE_RESULT_NAME}" >&2
  exit "${release_status}"
fi

prepare_destination "${EVIDENCE_DIR}"

command -v git >/dev/null 2>&1 || {
  printf '[verify_all] git is required.\n' >&2
  exit 2
}
git_sha_start="$(git -C "${PROJECT_ROOT}" rev-parse --verify HEAD)"
[[ "${git_sha_start}" =~ ^[0-9a-f]{40}$ ]] || {
  printf '[verify_all] Unable to resolve an exact starting Git SHA.\n' >&2
  exit 2
}
git_status_start="$(git -C "${PROJECT_ROOT}" status --porcelain=v1 --untracked-files=all)"
if [[ -n "${git_status_start}" ]]; then
  printf '[verify_all] Refusing to route phase verifiers from a dirty worktree.\n' >&2
  exit 2
fi
git_dirty_start=false

results_tmp="${WORK_ROOT}/results.tsv"
printf 'phase\tverifier\tstatus\texit_code\n' >"${results_tmp}"
overall_status='passed'
overall_exit=0

for phase in 0 1 2 3 4 5; do
  verifier="${SCRIPT_DIR}/verify_phase${phase}.sh"
  if [[ ! -f "${verifier}" || -L "${verifier}" ]]; then
    printf '%s\t%s\tfailed\t127\n' \
      "${phase}" "${verifier#"${PROJECT_ROOT}/"}" >>"${results_tmp}"
    overall_status='failed'
    overall_exit=127
    break
  fi
  verifier_arguments=()
  if [[ "${phase}" == '5' ]]; then
    verifier_arguments=(--local)
  fi
  printf '[verify_all] Running Phase %s verifier: %s\n' "${phase}" "${verifier}"
  if bash "${verifier}" "${verifier_arguments[@]}"; then
    printf '%s\t%s\tpassed\t0\n' \
      "${phase}" "${verifier#"${PROJECT_ROOT}/"}" >>"${results_tmp}"
  else
    exit_code=$?
    printf '%s\t%s\tfailed\t%s\n' \
      "${phase}" "${verifier#"${PROJECT_ROOT}/"}" "${exit_code}" >>"${results_tmp}"
    overall_status='failed'
    overall_exit="${exit_code}"
    break
  fi
done

if [[ "${overall_status}" == 'passed' ]]; then
  overall_status='incomplete'
  overall_exit=3
fi
git_sha_end="$(git -C "${PROJECT_ROOT}" rev-parse --verify HEAD)"
git_status_end="$(git -C "${PROJECT_ROOT}" status --porcelain=v1 --untracked-files=all)"
[[ -n "${git_status_end}" ]] && git_dirty_end=true || git_dirty_end=false
delta_tmp="${WORK_ROOT}/generated-delta.tsv"
printf 'path\tbytes\tsha256\n' >"${delta_tmp}"
delta_valid=true
phase0_version_changed=false
while IFS= read -r status_line; do
  [[ -z "${status_line}" ]] && continue
  matched=false
  for allowed_path in "${ALLOWED_GENERATED_DELTA[@]}"; do
    if [[ "${status_line}" == " M ${allowed_path}" ]]; then
      matched=true
      [[ "${allowed_path}" == 'artifacts/evidence/phase0/phase0-versions.json' ]] && \
        phase0_version_changed=true
      changed_path="${PROJECT_ROOT}/${allowed_path}"
      printf '%s\t%s\t%s\n' \
        "${allowed_path}" \
        "$(stat -c %s -- "${changed_path}")" \
        "$(sha256sum "${changed_path}" | awk '{print $1}')" >>"${delta_tmp}"
      break
    fi
  done
  [[ "${matched}" == true ]] || delta_valid=false
done <<<"${git_status_end}"
[[ "${phase0_version_changed}" == true ]] || delta_valid=false
if [[ "${git_dirty_start}" == false && -z "${git_status_start}" && \
  "${git_sha_start}" == "${git_sha_end}" && "${delta_valid}" == true ]]; then
  source_unchanged=true
else
  source_unchanged=false
fi
if [[ "${overall_status}" == 'incomplete' && "${source_unchanged}" != true ]]; then
  overall_status='failed'
  overall_exit=1
fi

local_output="${WORK_ROOT}/${LOCAL_RESULT_NAME}"
python3 - \
  "${overall_status}" \
  "${results_tmp}" \
  "$0" \
  "${ORIGINAL_CWD}" \
  "${git_sha_start}" \
  "${git_dirty_start}" \
  "${git_status_start}" \
  "${git_sha_end}" \
  "${git_dirty_end}" \
  "${git_status_end}" \
  "${source_unchanged}" \
  "${delta_tmp}" \
  "${local_output}" <<'PY'
import csv
import json
import pathlib
import sys
from datetime import datetime, timezone

(
    overall_status,
    results_path,
    command,
    cwd,
    git_sha_start,
    git_dirty_start,
    git_status_start,
    git_sha_end,
    git_dirty_end,
    git_status_end,
    source_unchanged,
    delta_path,
    output_path,
) = sys.argv[1:]
with open(results_path, encoding='utf-8', newline='') as handle:
    results = list(csv.DictReader(handle, delimiter='\t'))
with open(delta_path, encoding='utf-8', newline='') as handle:
    generated_files = list(csv.DictReader(handle, delimiter='\t'))
for record in generated_files:
    record['bytes'] = int(record['bytes'])
document = {
    'checked_at': datetime.now(timezone.utc).isoformat(),
    'command': {'argv': [command], 'cwd': cwd},
    'mode': 'static',
    'outstanding_gates': (
        [
            'Phase 3 authorized 15-run campaign evidence',
            'Phase 4 privileged --apply acceptance evidence',
            'Phase 5 exact pushed-SHA remote CI and evidence-commit verification',
        ]
        if overall_status == 'incomplete'
        else []
    ),
    'release_eligible': False,
    'release_evidence': None,
    'results': results,
    'schema_version': 4,
    'source': {
        'git_dirty_end': git_dirty_end == 'true',
        'git_dirty_start': git_dirty_start == 'true',
        'git_sha_end': git_sha_end,
        'git_sha_start': git_sha_start,
        'git_status_porcelain_end': git_status_end,
        'git_status_porcelain_start': git_status_start,
        'generated_evidence_delta': {
            'allowed_paths': [
                'artifacts/evidence/phase0/installed-packages.tsv',
                'artifacts/evidence/phase0/phase0-versions.json',
                'artifacts/evidence/phase0/ros2-doctor.txt',
                'artifacts/evidence/phase0/tool-versions.tsv',
            ],
            'changed_files': generated_files,
        },
        'source_unchanged': source_unchanged == 'true',
    },
    'status': overall_status,
    'verification_scope': (
        'Aggregate static/local routing only; live, campaign, privileged, and remote acceptance '
        'require separate explicitly authorized commands and evidence.'
    ),
}
path = pathlib.Path(output_path)
path.write_text(
    json.dumps(document, separators=(',', ':'), sort_keys=True) + '\n',
    encoding='utf-8',
)
PY
checksum_result "${WORK_ROOT}" "${LOCAL_RESULT_NAME}"
publish_result "${WORK_ROOT}" "${LOCAL_RESULT_NAME}" "${EVIDENCE_DIR}"
trap - EXIT
cleanup

if [[ "${overall_status}" == 'failed' ]]; then
  printf '[verify_all] FAILED. Evidence: %s/%s\n' \
    "${EVIDENCE_DIR}" "${LOCAL_RESULT_NAME}" >&2
  exit "${overall_exit}"
fi

printf '[verify_all] INCOMPLETE: static/local checks passed; exact acceptance evidence is required. Evidence: %s/%s\n' \
  "${EVIDENCE_DIR}" "${LOCAL_RESULT_NAME}" >&2
exit "${overall_exit}"

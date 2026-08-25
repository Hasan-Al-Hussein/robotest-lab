#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly PROJECT_ROOT
readonly EVIDENCE_DIR="${PROJECT_ROOT}/artifacts/evidence/phase0"

mkdir -p -- "${EVIDENCE_DIR}"
results_tmp="$(mktemp "${EVIDENCE_DIR}/.verify-all.XXXXXX")"
json_tmp="$(mktemp "${EVIDENCE_DIR}/.verify-all-json.XXXXXX")"
trap 'rm -f -- "${results_tmp}" "${json_tmp}"' EXIT

printf 'phase\tverifier\tstatus\texit_code\n' >"${results_tmp}"
overall_status="passed"
overall_exit=0

for phase in 0 1 2 3 4 5; do
  verifier="${SCRIPT_DIR}/verify_phase${phase}.sh"
  [[ -f "${verifier}" ]] || continue

  printf '[verify_all] Running Phase %s verifier: %s\n' "${phase}" "${verifier}"
  if bash "${verifier}"; then
    printf '%s\t%s\tpassed\t0\n' "${phase}" "${verifier#"${PROJECT_ROOT}/"}" >>"${results_tmp}"
  else
    exit_code=$?
    printf '%s\t%s\tfailed\t%s\n' \
      "${phase}" \
      "${verifier#"${PROJECT_ROOT}/"}" \
      "${exit_code}" >>"${results_tmp}"
    overall_status="failed"
    overall_exit="${exit_code}"
    break
  fi
done

python3 - "${overall_status}" "${results_tmp}" >"${json_tmp}" <<'PY'
import csv
import json
import sys
from datetime import datetime, timezone

overall_status, results_path = sys.argv[1:]
with open(results_path, encoding="utf-8", newline="") as handle:
    results = list(csv.DictReader(handle, delimiter="\t"))

json.dump(
    {
        "schema_version": 1,
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "status": overall_status,
        "results": results,
    },
    sys.stdout,
    indent=2,
    sort_keys=True,
)
sys.stdout.write("\n")
PY

mv -f -- "${json_tmp}" "${EVIDENCE_DIR}/verify-all.json"
rm -f -- "${results_tmp}"
trap - EXIT

if [[ "${overall_status}" != "passed" ]]; then
  printf '[verify_all] FAILED. Evidence: %s\n' "${EVIDENCE_DIR}/verify-all.json" >&2
  exit "${overall_exit}"
fi

printf '[verify_all] PASS. Evidence: %s\n' "${EVIDENCE_DIR}/verify-all.json"

#!/usr/bin/env bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly PROJECT_ROOT

die() {
  printf '[verify_debian_reproducibility] ERROR: %s\n' "$*" >&2
  exit 1
}

if (($# != 1)); then
  printf 'usage: %s OUTPUT_DIRECTORY\n' "$0" >&2
  exit 2
fi

output_directory="$(realpath -m -- "$1")"
case "${output_directory}" in
  "${PROJECT_ROOT}/artifacts/packages/"*) ;;
  *) die 'OUTPUT_DIRECTORY must be under artifacts/packages.' ;;
esac
mkdir -p -- "${output_directory}"
if find "${output_directory}" -mindepth 1 -maxdepth 1 -print -quit | grep -q .; then
  die "Output directory must be empty: ${output_directory}"
fi

"${SCRIPT_DIR}/build_debian_package.sh" "${output_directory}/build-a"
"${SCRIPT_DIR}/build_debian_package.sh" "${output_directory}/build-b"

mapfile -t first_names < <(
  find "${output_directory}/build-a" -mindepth 1 -maxdepth 1 -type f \
    -printf '%f\n' | LC_ALL=C sort
)
mapfile -t second_names < <(
  find "${output_directory}/build-b" -mindepth 1 -maxdepth 1 -type f \
    -printf '%f\n' | LC_ALL=C sort
)
((${#first_names[@]} > 0)) || die 'The first build produced no files.'
[[ "${first_names[*]}" == "${second_names[*]}" ]] ||
  die 'The two package builds produced different file sets.'

python3 - "${output_directory}" "${first_names[@]}" <<'PY'
import hashlib
import json
import os
import pathlib
import re
import sys
import tempfile
from datetime import datetime, timezone

root = pathlib.Path(sys.argv[1]).resolve(strict=True)
names = sys.argv[2:]
files = []


def normalize_generated_metadata(name: str, payload: bytes) -> bytes:
    """Remove only dpkg's documented wall-clock Build-Date cascade."""
    text = payload.decode("utf-8")
    if name.endswith(".buildinfo"):
        text, count = re.subn(
            r"^Build-Date: .+$",
            "Build-Date: <dpkg-wall-clock-build-date>",
            text,
            count=1,
            flags=re.MULTILINE,
        )
        if count != 1:
            raise SystemExit(f"{name} has no unique Build-Date field")
        return text.encode()
    if name.endswith(".changes"):
        text, count = re.subn(
            r"^(\s+)\S+(\s+.*\.buildinfo)$",
            r"\1<buildinfo-checksum>\2",
            text,
            flags=re.MULTILINE,
        )
        if count != 3:
            raise SystemExit(
                f"{name} has {count} buildinfo checksum entries instead of 3"
            )
        return text.encode()
    if name == "SHA256SUMS":
        text, buildinfo_count = re.subn(
            r"^[0-9a-f]{64}(  \S+\.buildinfo)$",
            r"<buildinfo-sha256>\1",
            text,
            flags=re.MULTILINE,
        )
        text, changes_count = re.subn(
            r"^[0-9a-f]{64}(  \S+\.changes)$",
            r"<changes-sha256>\1",
            text,
            flags=re.MULTILINE,
        )
        if buildinfo_count != 1 or changes_count != 1:
            raise SystemExit(f"{name} has an unexpected generated-metadata shape")
        return text.encode()
    return payload


for name in names:
    first = root / "build-a" / name
    second = root / "build-b" / name
    first_payload = first.read_bytes()
    second_payload = second.read_bytes()
    comparison = "byte_identical"
    normalized_payload = first_payload
    if first_payload != second_payload:
        if not (
            name == "SHA256SUMS"
            or name.endswith(".buildinfo")
            or name.endswith(".changes")
        ):
            raise SystemExit(f"binary/source reproducibility mismatch: {name}")
        normalized_payload = normalize_generated_metadata(name, first_payload)
        normalized_second = normalize_generated_metadata(name, second_payload)
        if normalized_payload != normalized_second:
            raise SystemExit(f"generated metadata differs beyond Build-Date: {name}")
        comparison = "normalized_dpkg_build_date_only"
    files.append(
        {
            "comparison": comparison,
            "name": name,
            "build_a_sha256": hashlib.sha256(first_payload).hexdigest(),
            "build_b_sha256": hashlib.sha256(second_payload).hexdigest(),
            "normalized_sha256": hashlib.sha256(normalized_payload).hexdigest(),
            "size_bytes": len(first_payload),
        }
    )
value = {
    "schema_version": 1,
    "completed_utc": datetime.now(timezone.utc).isoformat(),
    "verdict": "PASS",
    "builds": ["build-a", "build-b"],
    "binary_packages_byte_identical": True,
    "source_manifest_byte_identical": True,
    "generated_metadata_policy": (
        "dpkg .buildinfo Build-Date is wall-clock metadata; .changes and SHA256SUMS "
        "may differ only through the cascading .buildinfo/.changes checksums"
    ),
    "files": files,
}
payload = (json.dumps(value, indent=2, sort_keys=True) + "\n").encode()
descriptor, temporary_name = tempfile.mkstemp(prefix=".reproducibility-", dir=root)
try:
    os.fchmod(descriptor, 0o644)
    with os.fdopen(descriptor, "wb", closefd=True) as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary_name, root / "reproducibility.json")
    directory = os.open(root, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)
except BaseException:
    try:
        os.close(descriptor)
    except OSError:
        pass
    pathlib.Path(temporary_name).unlink(missing_ok=True)
    raise
PY

printf '[verify_debian_reproducibility] PASS: %s\n' \
  "${output_directory}/reproducibility.json"

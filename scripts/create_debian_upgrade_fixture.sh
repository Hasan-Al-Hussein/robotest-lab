#!/usr/bin/env bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly PROJECT_ROOT
readonly BASELINE_VERSION="0.0.9"
readonly BASELINE_LISTEN_ADDRESS="127.0.0.1:9079"

die() {
  printf '[create_debian_upgrade_fixture] ERROR: %s\n' "$*" >&2
  exit 1
}

if (($# != 2)); then
  printf 'usage: %s FINAL_PACKAGE_DEB OUTPUT_BASELINE_DEB\n' "$0" >&2
  exit 2
fi

final_package="$(realpath -e -- "$1")"
output_package="$(realpath -m -- "$2")"
case "${output_package}" in
  "${PROJECT_ROOT}/artifacts/packages/"*.deb) ;;
  *) die 'OUTPUT_BASELINE_DEB must be under artifacts/packages and end in .deb.' ;;
esac
[[ ! -e "${output_package}" && ! -e "${output_package}.fixture.json" ]] ||
  die 'The output package or its provenance sidecar already exists.'
[[ "$(dpkg-deb -f "${final_package}" Package)" == "robotest-supervisor" ]] ||
  die 'FINAL_PACKAGE_DEB is not robotest-supervisor.'
final_version="$(dpkg-deb -f "${final_package}" Version)"
dpkg --compare-versions "${BASELINE_VERSION}" lt "${final_version}" ||
  die "Baseline ${BASELINE_VERSION} is not older than final ${final_version}."

working_directory="$(mktemp -d /tmp/robotest-upgrade-fixture.XXXXXX)"
cleanup() {
  case "${working_directory}" in
    /tmp/robotest-upgrade-fixture.*) rm -rf -- "${working_directory}" ;;
    *) printf '[create_debian_upgrade_fixture] refusing unsafe cleanup: %s\n' \
      "${working_directory}" >&2 ;;
  esac
}
trap cleanup EXIT

package_root="${working_directory}/root"
dpkg-deb --raw-extract "${final_package}" "${package_root}"
python3 - \
  "${package_root}" \
  "${BASELINE_VERSION}" \
  "${BASELINE_LISTEN_ADDRESS}" <<'PY'
import hashlib
import json
import os
import pathlib
import re
import sys

root = pathlib.Path(sys.argv[1]).resolve(strict=True)
version = sys.argv[2]
listen_address = sys.argv[3]
control_path = root / "DEBIAN" / "control"
control = control_path.read_text(encoding="utf-8")
control, substitutions = re.subn(
    r"^Version: .+$", f"Version: {version}", control, count=1, flags=re.MULTILINE
)
if substitutions != 1:
    raise SystemExit("fixture control has no unique Version field")
control_path.write_text(control, encoding="utf-8")

config_path = root / "etc" / "robotest-supervisor" / "config.json"
config = json.loads(config_path.read_text(encoding="utf-8"))
config["listen_address"] = listen_address
config_path.write_text(json.dumps(config, indent=2, sort_keys=True) + "\n", encoding="utf-8")

entries = []
for directory, directory_names, file_names in os.walk(root):
    directory_path = pathlib.Path(directory)
    directory_names[:] = sorted(
        name for name in directory_names if directory_path / name != root / "DEBIAN"
    )
    if directory_path == root / "DEBIAN":
        directory_names[:] = []
        continue
    for name in sorted(file_names):
        path = directory_path / name
        if root / "DEBIAN" in path.parents:
            continue
        relative = path.relative_to(root).as_posix()
        digest = hashlib.md5(path.read_bytes(), usedforsecurity=False).hexdigest()
        entries.append(f"{digest}  {relative}")
(root / "DEBIAN" / "md5sums").write_text("\n".join(entries) + "\n", encoding="ascii")
PY

mkdir -p -- "$(dirname -- "${output_package}")"
dpkg-deb --root-owner-group --build "${package_root}" "${output_package}" >/dev/null
python3 - \
  "${final_package}" \
  "${output_package}" \
  "${final_version}" \
  "${BASELINE_VERSION}" \
  "${BASELINE_LISTEN_ADDRESS}" <<'PY'
import hashlib
import json
import os
import pathlib
import sys
import tempfile
from datetime import datetime, timezone

final_path = pathlib.Path(sys.argv[1]).resolve(strict=True)
fixture_path = pathlib.Path(sys.argv[2]).resolve(strict=True)
final_version, fixture_version, listen_address = sys.argv[3:]
value = {
    "schema_version": 1,
    "created_utc": datetime.now(timezone.utc).isoformat(),
    "fixture_kind": "genuine_lower_version_different_default_conffile",
    "final_package": {
        "name": final_path.name,
        "version": final_version,
        "sha256": hashlib.sha256(final_path.read_bytes()).hexdigest(),
    },
    "fixture_package": {
        "name": fixture_path.name,
        "version": fixture_version,
        "sha256": hashlib.sha256(fixture_path.read_bytes()).hexdigest(),
        "listen_address": listen_address,
    },
}
payload = (json.dumps(value, indent=2, sort_keys=True) + "\n").encode()
sidecar = pathlib.Path(str(fixture_path) + ".fixture.json")
descriptor, temporary_name = tempfile.mkstemp(prefix=".fixture-", dir=sidecar.parent)
try:
    os.fchmod(descriptor, 0o644)
    with os.fdopen(descriptor, "wb", closefd=True) as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary_name, sidecar)
except BaseException:
    try:
        os.close(descriptor)
    except OSError:
        pass
    pathlib.Path(temporary_name).unlink(missing_ok=True)
    raise
PY

printf '[create_debian_upgrade_fixture] created %s\n' "${output_package}"

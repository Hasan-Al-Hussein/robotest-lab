#!/usr/bin/env bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly PROJECT_ROOT
readonly CONFIG_PATH="/etc/robotest-supervisor/config.json"
readonly STATE_DIRECTORY="/var/lib/robotest-supervisor"
readonly LOG_DIRECTORY="/var/log/robotest-supervisor"
readonly SENTINEL="${STATE_DIRECTORY}/package-lifecycle-sentinel"

BASELINE_DEB=""
UPGRADE_DEB=""
EVIDENCE_JSON=""
EVIDENCE_TEMP=""
PHASE="preflight"
RECOVERY_AUTHORIZED=0

die() {
  printf '[test_debian_package_lifecycle] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: sudo scripts/test_debian_package_lifecycle.sh \
  --apply BASELINE_DEB UPGRADE_DEB EVIDENCE_JSON

Runs the destructive-to-this-package-only install/upgrade/remove/purge test.
BASELINE_DEB must be an explicitly provenance-bound lower version with a
different default conffile; UPGRADE_DEB is the release candidate. The package
must initially be absent, the service must be inactive, and the evidence path
must be under artifacts/evidence/phase4. The test never removes the service
account or state/log directories and finishes with the release candidate
reinstalled, disabled, and stopped.
EOF
}

validate_package_manifest() {
  python3 - "$1" <<'PY'
import pathlib
import subprocess
import sys

package = pathlib.Path(sys.argv[1]).resolve(strict=True)
expected = {
    "./",
    "./etc/",
    "./etc/robotest-supervisor/",
    "./etc/robotest-supervisor/config.json",
    "./usr/",
    "./usr/bin/",
    "./usr/bin/robotest-supervisor",
    "./usr/lib/",
    "./usr/lib/systemd/",
    "./usr/lib/systemd/system/",
    "./usr/lib/systemd/system/robotest-supervisor.service",
    "./usr/lib/sysusers.d/",
    "./usr/lib/sysusers.d/robotest-supervisor.conf",
    "./usr/lib/tmpfiles.d/",
    "./usr/lib/tmpfiles.d/robotest-supervisor.conf",
    "./usr/libexec/",
    "./usr/libexec/robotest-supervisor/",
    "./usr/libexec/robotest-supervisor/start-robotest-stack",
    "./usr/share/",
    "./usr/share/doc/",
    "./usr/share/doc/robotest-supervisor/",
    "./usr/share/doc/robotest-supervisor/README.Debian",
    "./usr/share/doc/robotest-supervisor/README.md",
    "./usr/share/doc/robotest-supervisor/changelog.gz",
    "./usr/share/doc/robotest-supervisor/copyright",
    "./usr/share/man/",
    "./usr/share/man/man8/",
    "./usr/share/man/man8/robotest-supervisor.8.gz",
}
result = subprocess.run(
    ["dpkg-deb", "--contents", str(package)],
    check=True,
    capture_output=True,
    text=True,
)
entries = {}
for line in result.stdout.splitlines():
    fields = line.split(maxsplit=5)
    if len(fields) != 6:
        raise SystemExit(f"unparseable package manifest line: {line!r}")
    mode, path = fields[0], fields[5]
    if path in entries:
        raise SystemExit(f"duplicate package path: {path}")
    entries[path] = mode
if set(entries) != expected:
    missing = sorted(expected - set(entries))
    extra = sorted(set(entries) - expected)
    raise SystemExit(f"package manifest mismatch; missing={missing}, extra={extra}")
executables = {
    "./usr/bin/robotest-supervisor",
    "./usr/libexec/robotest-supervisor/start-robotest-stack",
}
for path, mode in entries.items():
    if path.endswith("/"):
        if mode != "drwxr-xr-x":
            raise SystemExit(f"directory mode mismatch: {path}={mode}")
    elif path in executables:
        if mode != "-rwxr-xr-x":
            raise SystemExit(f"executable mode mismatch: {path}={mode}")
    elif mode != "-rw-r--r--":
        raise SystemExit(f"regular-file mode mismatch: {path}={mode}")
PY
}

package_config_sha256() {
  dpkg-deb --fsys-tarfile "$1" |
    tar -xO ./etc/robotest-supervisor/config.json |
    sha256sum | awk '{print $1}'
}

cleanup() {
  rm -f -- "${EVIDENCE_TEMP}"
}

on_error() {
  local status=$?
  local failed_line="${1:-unknown}"
  local failed_command="${2:-unknown}"
  local recovery_command_failed=0
  local recovery_verification_failed=0
  trap - ERR
  printf '[test_debian_package_lifecycle] failed during %s at line %s (exit %d): %s\n' \
    "${PHASE}" "${failed_line}" "${status}" "${failed_command}" >&2
  if ((RECOVERY_AUTHORIZED)); then
    set +e
    systemctl stop robotest-supervisor.service >/dev/null 2>&1 ||
      recovery_command_failed=1
    env DEBIAN_FRONTEND=noninteractive dpkg --purge robotest-supervisor \
      >/dev/null 2>&1 || recovery_command_failed=1
    env DEBIAN_FRONTEND=noninteractive dpkg --install "${UPGRADE_DEB}" \
      >/dev/null 2>&1 || recovery_command_failed=1
    systemctl daemon-reload >/dev/null 2>&1 || recovery_command_failed=1
    systemctl disable --now robotest-supervisor.service >/dev/null 2>&1 ||
      recovery_command_failed=1
    rm -f -- "${SENTINEL}" || recovery_command_failed=1
    if [[ "$(dpkg-query -W -f='${Version}' robotest-supervisor 2>/dev/null)" != \
      "${upgrade_version}" ]]; then
      recovery_verification_failed=1
    fi
    if [[ ! -f "${CONFIG_PATH}" || \
      "$(sha256sum "${CONFIG_PATH}" 2>/dev/null | awk '{print $1}')" != \
      "${upgrade_vendor_config_sha256}" ]]; then
      recovery_verification_failed=1
    fi
    if [[ "$(systemctl is-enabled robotest-supervisor.service 2>/dev/null)" != \
      "disabled" || \
      "$(systemctl is-active robotest-supervisor.service 2>/dev/null)" != \
      "inactive" ]]; then
      recovery_verification_failed=1
    fi
    if find /etc/robotest-supervisor -maxdepth 1 -type f \
      -name 'config.json.dpkg-*' -print -quit 2>/dev/null | grep -q .; then
      recovery_verification_failed=1
    fi
    set -e
    if ((recovery_verification_failed)); then
      printf '[test_debian_package_lifecycle] ERROR: release-candidate recovery did not verify\n' >&2
    elif ((recovery_command_failed)); then
      printf '[test_debian_package_lifecycle] verified release-candidate recovery after a nonzero recovery step\n' >&2
    else
      printf '[test_debian_package_lifecycle] verified release-candidate recovery\n' >&2
    fi
  fi
  exit "${status}"
}

trap cleanup EXIT
trap 'on_error "${LINENO}" "${BASH_COMMAND}"' ERR

if (($# != 4)) || [[ "$1" != "--apply" ]]; then
  usage >&2
  exit 2
fi
((EUID == 0)) || die 'This exact lifecycle test must run as root.'

BASELINE_DEB="$(realpath -e -- "$2")"
UPGRADE_DEB="$(realpath -e -- "$3")"
EVIDENCE_JSON="$(realpath -m -- "$4")"
case "${EVIDENCE_JSON}" in
  "${PROJECT_ROOT}/artifacts/evidence/phase4/"*.json) ;;
  *) die 'Evidence JSON must be under artifacts/evidence/phase4.' ;;
esac
[[ "${BASELINE_DEB}" == *.deb && "${UPGRADE_DEB}" == *.deb ]] ||
  die 'Both package arguments must name .deb files.'

for command_name in awk dpkg dpkg-deb dpkg-query find grep id install lintian mv \
  python3 realpath runuser sha256sum stat systemctl systemd-analyze tar tr; do
  command -v "${command_name}" >/dev/null 2>&1 ||
    die "Required command not found: ${command_name}"
done
for package_path in "${BASELINE_DEB}" "${UPGRADE_DEB}"; do
  [[ "$(dpkg-deb -f "${package_path}" Package)" == "robotest-supervisor" ]] ||
    die "Package is not robotest-supervisor: ${package_path}"
  validate_package_manifest "${package_path}"
done
baseline_version="$(dpkg-deb -f "${BASELINE_DEB}" Version)"
upgrade_version="$(dpkg-deb -f "${UPGRADE_DEB}" Version)"
baseline_architecture="$(dpkg-deb -f "${BASELINE_DEB}" Architecture)"
upgrade_architecture="$(dpkg-deb -f "${UPGRADE_DEB}" Architecture)"
baseline_sha256="$(sha256sum "${BASELINE_DEB}" | awk '{print $1}')"
upgrade_sha256="$(sha256sum "${UPGRADE_DEB}" | awk '{print $1}')"
baseline_vendor_config_sha256="$(package_config_sha256 "${BASELINE_DEB}")"
upgrade_vendor_config_sha256="$(package_config_sha256 "${UPGRADE_DEB}")"
dpkg --compare-versions "${baseline_version}" lt "${upgrade_version}" ||
  die "Baseline ${baseline_version} is not older than upgrade ${upgrade_version}."
[[ "${baseline_architecture}" == "${upgrade_architecture}" &&
  "${upgrade_architecture}" == "$(dpkg --print-architecture)" ]] ||
  die 'Package architectures do not match the target host.'
[[ "${baseline_vendor_config_sha256}" != "${upgrade_vendor_config_sha256}" ]] ||
  die 'Baseline and upgrade packages must ship different default conffiles.'
[[ -f "${BASELINE_DEB}.fixture.json" ]] ||
  die 'The baseline package has no provenance sidecar.'
python3 - \
  "${BASELINE_DEB}.fixture.json" \
  "${baseline_version}" \
  "${baseline_sha256}" \
  "${upgrade_version}" \
  "${upgrade_sha256}" <<'PY'
import json
import pathlib
import sys

path = pathlib.Path(sys.argv[1])
baseline_version, baseline_sha, upgrade_version, upgrade_sha = sys.argv[2:]
value = json.loads(path.read_text(encoding="utf-8"))
if value.get("schema_version") != 1:
    raise SystemExit("unsupported baseline provenance schema")
if value.get("fixture_kind") != "genuine_lower_version_different_default_conffile":
    raise SystemExit("baseline provenance has the wrong fixture_kind")
baseline = value.get("fixture_package", {})
upgrade = value.get("final_package", {})
if baseline.get("version") != baseline_version or baseline.get("sha256") != baseline_sha:
    raise SystemExit("baseline package does not match its provenance")
if upgrade.get("version") != upgrade_version or upgrade.get("sha256") != upgrade_sha:
    raise SystemExit("upgrade package does not match baseline provenance")
PY
project_owner="$(stat -c '%U' -- "${PROJECT_ROOT}")"
[[ "${project_owner}" =~ ^[a-z_][a-z0-9_-]*$ && "${project_owner}" != "root" ]] ||
  die 'The project workspace must have a non-root owner for Lintian.'
lintian_output="$(runuser --user "${project_owner}" -- \
  lintian --display-info --display-experimental --pedantic \
  "${UPGRADE_DEB}" 2>&1)" || die 'Lintian could not analyze the upgrade package.'
unexpected_lintian="$(printf '%s\n' "${lintian_output}" |
  grep -Ev '^$|^W: robotest-supervisor: unknown-field Static-Built-Using$' || true)"
[[ -z "${unexpected_lintian}" ]] ||
  die "Lintian reported an unexpected diagnostic: ${unexpected_lintian}"
if dpkg-query -W robotest-supervisor >/dev/null 2>&1; then
  die 'robotest-supervisor must be absent before the lifecycle test.'
fi
[[ ! -e "${CONFIG_PATH}" ]] ||
  die "Refusing to overwrite pre-existing configuration: ${CONFIG_PATH}"
[[ ! -e "${SENTINEL}" ]] ||
  die "Refusing to overwrite pre-existing state: ${SENTINEL}"
if systemctl is-active --quiet robotest-supervisor.service; then
  die 'robotest-supervisor.service must be inactive.'
fi
mkdir -p -- "$(dirname -- "${EVIDENCE_JSON}")"
chown --reference="${PROJECT_ROOT}" "$(dirname -- "${EVIDENCE_JSON}")"
EVIDENCE_TEMP="$(mktemp "$(dirname -- "${EVIDENCE_JSON}")/.package-lifecycle.XXXXXX")"

PHASE="initial-install"
RECOVERY_AUTHORIZED=1
env DEBIAN_FRONTEND=noninteractive dpkg --install "${BASELINE_DEB}"
systemctl daemon-reload
[[ "$(dpkg-query -W -f='${Version}' robotest-supervisor)" == "${baseline_version}" ]]
[[ "$(systemctl is-enabled robotest-supervisor.service 2>/dev/null || true)" == \
  "disabled" ]]
[[ "$(systemctl is-active robotest-supervisor.service 2>/dev/null || true)" == \
  "inactive" ]]
robotest-supervisor -check-config -config "${CONFIG_PATH}" |
  grep -Fx 'configuration valid' >/dev/null
[[ "$(sha256sum "${CONFIG_PATH}" | awk '{print $1}')" == \
  "${baseline_vendor_config_sha256}" ]]
install -m 0600 -o robotest-supervisor -g robotest-supervisor \
  /dev/null "${SENTINEL}"

PHASE="modified-conffile"
python3 - "${CONFIG_PATH}" <<'PY'
import json
import os
import pathlib
import tempfile
import sys

path = pathlib.Path(sys.argv[1])
value = json.loads(path.read_text(encoding="utf-8"))
value["listen_address"] = "127.0.0.1:9081"
payload = json.dumps(value, indent=2, sort_keys=True).encode() + b"\n"
descriptor, temporary_name = tempfile.mkstemp(prefix=".config.", dir=path.parent)
try:
    os.fchmod(descriptor, 0o644)
    with os.fdopen(descriptor, "wb", closefd=True) as handle:
        handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary_name, path)
    directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
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
modified_config_sha256="$(sha256sum "${CONFIG_PATH}" | awk '{print $1}')"
[[ "${modified_config_sha256}" != "${baseline_vendor_config_sha256}" ]]
[[ "${modified_config_sha256}" != "${upgrade_vendor_config_sha256}" ]]
robotest-supervisor -check-config -config "${CONFIG_PATH}" |
  grep -Fx 'configuration valid' >/dev/null

PHASE="versioned-upgrade"
env DEBIAN_FRONTEND=noninteractive dpkg --force-confold --install "${UPGRADE_DEB}"
[[ "$(dpkg-query -W -f='${Version}' robotest-supervisor)" == "${upgrade_version}" ]]
[[ "$(sha256sum "${CONFIG_PATH}" | awk '{print $1}')" == \
  "${modified_config_sha256}" ]]
[[ "$(systemctl is-enabled robotest-supervisor.service 2>/dev/null || true)" == \
  "disabled" ]]
[[ "$(systemctl is-active robotest-supervisor.service 2>/dev/null || true)" == \
  "inactive" ]]

PHASE="remove"
dpkg --remove robotest-supervisor
[[ ! -e /usr/bin/robotest-supervisor ]]
[[ ! -e /usr/lib/systemd/system/robotest-supervisor.service ]]
[[ -f "${CONFIG_PATH}" ]]
[[ "$(sha256sum "${CONFIG_PATH}" | awk '{print $1}')" == \
  "${modified_config_sha256}" ]]
[[ -f "${SENTINEL}" ]]

PHASE="purge"
dpkg --purge robotest-supervisor
[[ ! -e "${CONFIG_PATH}" ]]
[[ -f "${SENTINEL}" ]]
[[ -d "${STATE_DIRECTORY}" && -d "${LOG_DIRECTORY}" ]]
rm -f -- "${SENTINEL}"

PHASE="final-reinstall"
env DEBIAN_FRONTEND=noninteractive dpkg --install "${UPGRADE_DEB}"
systemctl daemon-reload
[[ "$(dpkg-query -W -f='${Version}' robotest-supervisor)" == \
  "${upgrade_version}" ]]
[[ "$(sha256sum "${CONFIG_PATH}" | awk '{print $1}')" == \
  "${upgrade_vendor_config_sha256}" ]]
[[ "$(systemctl is-enabled robotest-supervisor.service 2>/dev/null || true)" == \
  "disabled" ]]
[[ "$(systemctl is-active robotest-supervisor.service 2>/dev/null || true)" == \
  "inactive" ]]
systemd-analyze verify /usr/lib/systemd/system/robotest-supervisor.service
[[ -x /usr/bin/robotest-supervisor ]]
[[ -x /usr/libexec/robotest-supervisor/start-robotest-stack ]]
[[ ! -e /usr/lib/robotest-supervisor/start-robotest-stack ]]
[[ "$(stat -c '%a:%U:%G' "${CONFIG_PATH}")" == "644:root:root" ]]
[[ "$(stat -c '%a:%U:%G' "${STATE_DIRECTORY}")" == \
  "750:robotest-supervisor:robotest-supervisor" ]]
[[ "$(stat -c '%a:%U:%G' "${LOG_DIRECTORY}")" == \
  "750:robotest-supervisor:robotest-supervisor" ]]
[[ "$(stat -c '%a:%U:%G' /usr/bin/robotest-supervisor)" == "755:root:root" ]]
[[ "$(stat -c '%a:%U:%G' \
  /usr/libexec/robotest-supervisor/start-robotest-stack)" == \
  "755:root:root" ]]
[[ "$(stat -c '%a:%U:%G' \
  /usr/lib/systemd/system/robotest-supervisor.service)" == \
  "644:root:root" ]]
[[ -z "$(dpkg --verify robotest-supervisor)" ]]
if find /etc/robotest-supervisor -maxdepth 1 -type f \
  -name 'config.json.dpkg-*' -print -quit | grep -q .; then
  die 'A stale dpkg conffile artifact remains after the final install.'
fi
id -nG robotest-supervisor | tr ' ' '\n' | grep -Fx video >/dev/null
id -nG robotest-supervisor | tr ' ' '\n' | grep -Fx render >/dev/null
"${PROJECT_ROOT}/packaging/debian/tests/installed" >/dev/null

installed_binary_sha256="$(sha256sum /usr/bin/robotest-supervisor | awk '{print $1}')"
installed_helper_sha256="$(
  sha256sum /usr/libexec/robotest-supervisor/start-robotest-stack | awk '{print $1}'
)"
installed_unit_sha256="$(
  sha256sum /usr/lib/systemd/system/robotest-supervisor.service | awk '{print $1}'
)"
lintian_legacy_warning_present=false
if printf '%s\n' "${lintian_output}" |
  grep -Fx 'W: robotest-supervisor: unknown-field Static-Built-Using' \
    >/dev/null; then
  lintian_legacy_warning_present=true
fi

python3 - \
  "${baseline_version}" \
  "${baseline_architecture}" \
  "${baseline_sha256}" \
  "${baseline_vendor_config_sha256}" \
  "${upgrade_version}" \
  "${upgrade_architecture}" \
  "${upgrade_sha256}" \
  "${upgrade_vendor_config_sha256}" \
  "${modified_config_sha256}" \
  "${installed_binary_sha256}" \
  "${installed_helper_sha256}" \
  "${installed_unit_sha256}" \
  "${lintian_legacy_warning_present}" >"${EVIDENCE_TEMP}" <<'PY'
import json
import sys
from datetime import datetime, timezone

(
    baseline_version,
    baseline_architecture,
    baseline_package_sha,
    baseline_vendor_config_sha,
    upgrade_version,
    upgrade_architecture,
    upgrade_package_sha,
    upgrade_vendor_config_sha,
    modified_config_sha,
    installed_binary_sha,
    installed_helper_sha,
    installed_unit_sha,
    legacy_lintian_warning,
) = sys.argv[1:]
json.dump(
    {
        "schema_version": 1,
        "completed_utc": datetime.now(timezone.utc).isoformat(),
        "verdict": "PASS",
        "baseline_package": {
            "name": "robotest-supervisor",
            "version": baseline_version,
            "architecture": baseline_architecture,
            "sha256": baseline_package_sha,
            "vendor_config_sha256": baseline_vendor_config_sha,
        },
        "upgrade_package": {
            "name": "robotest-supervisor",
            "version": upgrade_version,
            "architecture": upgrade_architecture,
            "sha256": upgrade_package_sha,
            "vendor_config_sha256": upgrade_vendor_config_sha,
        },
        "conffile": {
            "modified_sha256": modified_config_sha,
            "modified_value": {"listen_address": "127.0.0.1:9081"},
            "preserved_during_upgrade": True,
            "preserved_during_remove": True,
            "removed_during_purge": True,
            "upgrade_vendor_default_restored_on_final_install": True,
        },
        "state": {
            "sentinel_preserved_during_remove": True,
            "sentinel_preserved_during_purge": True,
            "state_directory_preserved": True,
            "log_directory_preserved": True,
        },
        "final_state": {
            "package_installed": True,
            "service_enabled": False,
            "service_active": False,
            "configuration_mode_owner": "644:root:root",
            "state_directory_mode_owner": (
                "750:robotest-supervisor:robotest-supervisor"
            ),
            "log_directory_mode_owner": (
                "750:robotest-supervisor:robotest-supervisor"
            ),
            "installed_binary_sha256": installed_binary_sha,
            "installed_helper_sha256": installed_helper_sha,
            "installed_unit_sha256": installed_unit_sha,
            "systemd_unit_verified": True,
            "dpkg_verify_clean": True,
            "service_account_video_group": True,
            "service_account_render_group": True,
            "installed_smoke_contract_passed": True,
            "stale_dpkg_conffile_artifacts": 0,
        },
        "quality": {
            "both_package_manifests_exact": True,
            "genuine_versioned_upgrade": True,
            "baseline_provenance_verified": True,
            "lintian_unexpected_diagnostics": 0,
            "legacy_noble_lintian_static_built_using_warning": (
                legacy_lintian_warning == "true"
            ),
        },
    },
    sys.stdout,
    indent=2,
    sort_keys=True,
)
sys.stdout.write("\n")
PY
chmod 0644 "${EVIDENCE_TEMP}"
chown --reference="${PROJECT_ROOT}" "${EVIDENCE_TEMP}"
mv -f -- "${EVIDENCE_TEMP}" "${EVIDENCE_JSON}"
EVIDENCE_TEMP=""
PHASE="complete"
RECOVERY_AUTHORIZED=0
printf '[test_debian_package_lifecycle] PASS: %s\n' "${EVIDENCE_JSON}"

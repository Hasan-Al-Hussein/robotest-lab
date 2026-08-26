#!/usr/bin/env bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly SCRIPT_DIR
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly PROJECT_ROOT
readonly RELEASE_ROOT="/opt/robotest-lab-releases"
readonly ACTIVE_LINK="/opt/robotest-lab"
readonly EVIDENCE_DIR="${PROJECT_ROOT}/artifacts/evidence/phase4"
readonly LOCK_FILE="/run/lock/robotest-stage-runtime-overlay.lock"

MODE="check"
SOURCE_MANIFEST=""
SOURCE_MANIFEST_END=""
TARGET_MANIFEST=""
PROVENANCE=""
PACKAGE_LIST=""
EVIDENCE_TEMP=""
STAGING_DIRECTORY=""
STAGING_OWNED=0
TEMP_LINK=""
ROLLBACK_LINK=""
PREVIOUS_ACTIVE_TARGET=""
PROMOTION_PENDING=0

log() {
  printf '[stage_runtime_overlay] %s\n' "$*"
}

run_root() {
  if ((EUID == 0)); then
    "$@"
  else
    sudo "$@"
  fi
}

die() {
  printf '[stage_runtime_overlay] ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: scripts/stage_runtime_overlay.sh [--check|--apply]

  --check  Verify the active immutable overlay and refresh repository evidence
           without changing the installed runtime selection or contents.
  --apply  Build a non-symlink colcon overlay directly in a versioned,
           root-owned release and atomically select it at /opt/robotest-lab.

The service must be inactive. The script never enables or starts it and never
deletes a completed release. Apply mode must be run as root.
EOF
}

cleanup() {
  rm -f -- \
    "${SOURCE_MANIFEST}" \
    "${SOURCE_MANIFEST_END}" \
    "${TARGET_MANIFEST}" \
    "${PROVENANCE}" \
    "${PACKAGE_LIST}" \
    "${EVIDENCE_TEMP}"
  if [[ -n "${TEMP_LINK}" ]]; then
    run_root rm -f -- "${TEMP_LINK}" 2>/dev/null || true
  fi
  if ((PROMOTION_PENDING)); then
    if [[ -n "${PREVIOUS_ACTIVE_TARGET}" ]]; then
      ROLLBACK_LINK="${RELEASE_ROOT}/.rollback-link.$$"
      if run_root ln -s -- "${PREVIOUS_ACTIVE_TARGET}" "${ROLLBACK_LINK}" &&
        run_root mv -Tf -- "${ROLLBACK_LINK}" "${ACTIVE_LINK}"; then
        printf '[stage_runtime_overlay] restored previous active overlay: %s\n' \
          "${PREVIOUS_ACTIVE_TARGET}" >&2
      else
        printf '[stage_runtime_overlay] ERROR: failed to restore previous overlay\n' >&2
      fi
      ROLLBACK_LINK=""
    else
      run_root rm -f -- "${ACTIVE_LINK}" 2>/dev/null || true
    fi
  fi
  if [[ -n "${ROLLBACK_LINK}" ]]; then
    run_root rm -f -- "${ROLLBACK_LINK}" 2>/dev/null || true
  fi
  if [[ -n "${STAGING_DIRECTORY}" ]] && ((STAGING_OWNED)); then
    case "${STAGING_DIRECTORY}" in
      "${RELEASE_ROOT}/"*)
        run_root rm -rf -- "${STAGING_DIRECTORY}" 2>/dev/null || true
        ;;
      *)
        printf '[stage_runtime_overlay] refusing unsafe staging cleanup: %s\n' \
          "${STAGING_DIRECTORY}" >&2
        ;;
    esac
  fi
}
trap cleanup EXIT

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

require_noble() {
  [[ -r /etc/os-release ]] || die '/etc/os-release is unavailable.'
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" && \
    "${VERSION_CODENAME:-}" == "noble" ]] ||
    die 'Runtime staging requires Ubuntu 24.04 Noble.'
  [[ "${WSL_DISTRO_NAME:-}" != "Ubuntu-20.04" ]] ||
    die 'Refusing to operate in the protected Ubuntu-20.04 distribution.'
}

write_manifest() {
  local tree_root="$1"
  local output_path="$2"
  shift 2
  local -a allowed_roots=("$@")
  python3 - "${tree_root}" "${allowed_roots[@]}" >"${output_path}" <<'PY'
import hashlib
import json
import os
import pathlib
import stat
import sys

root = pathlib.Path(sys.argv[1]).resolve(strict=True)
allowed_roots = [pathlib.Path(value).resolve(strict=True) for value in sys.argv[2:]]
entries = []
for directory, directory_names, file_names in os.walk(root, followlinks=True):
    directory_names[:] = sorted(
        name for name in directory_names if name != "__pycache__"
    )
    for name in sorted(file_names):
        if name.endswith((".pyc", ".pyo")):
            continue
        path = pathlib.Path(directory, name)
        resolved = path.resolve(strict=True)
        if not any(
            resolved == allowed or allowed in resolved.parents for allowed in allowed_roots
        ):
            raise SystemExit(f"overlay entry escapes allowed roots: {path} -> {resolved}")
        if not resolved.is_file():
            raise SystemExit(f"non-regular overlay entry: {path}")
        digest = hashlib.sha256()
        with resolved.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        metadata = resolved.stat()
        entries.append(
            {
                "mode": format(stat.S_IMODE(metadata.st_mode), "04o"),
                "path": path.relative_to(root).as_posix(),
                "sha256": digest.hexdigest(),
                "size_bytes": metadata.st_size,
            }
        )

json.dump(
    {"schema_version": 1, "files": sorted(entries, key=lambda item: item["path"])},
    sys.stdout,
    ensure_ascii=False,
    separators=(",", ":"),
)
sys.stdout.write("\n")
PY
}

write_source_manifest() {
  local output_path="$1"
  python3 - "${PROJECT_ROOT}" >"${output_path}" <<'PY'
import hashlib
import json
import os
import pathlib
import stat
import sys

project_root = pathlib.Path(sys.argv[1]).resolve(strict=True)
runtime_roots = [project_root / name for name in ("config", "scenarios", "src")]
entries = []
for runtime_root in runtime_roots:
    if not runtime_root.is_dir():
        raise SystemExit(f"missing runtime source directory: {runtime_root}")
    for directory, directory_names, file_names in os.walk(
        runtime_root, followlinks=False
    ):
        for name in directory_names:
            if pathlib.Path(directory, name).is_symlink():
                raise SystemExit(
                    f"symbolic-link runtime source directory is forbidden: "
                    f"{pathlib.Path(directory, name)}"
                )
        directory_names[:] = sorted(
            name
            for name in directory_names
            if name not in {"__pycache__", ".pytest_cache", ".ruff_cache"}
        )
        for name in sorted(file_names):
            if name.endswith((".pyc", ".pyo")):
                continue
            path = pathlib.Path(directory, name)
            if path.is_symlink():
                raise SystemExit(f"symbolic-link runtime source file is forbidden: {path}")
            resolved = path.resolve(strict=True)
            if not (resolved == project_root or project_root in resolved.parents):
                raise SystemExit(f"runtime source escapes the workspace: {path}")
            if not resolved.is_file():
                raise SystemExit(f"non-regular runtime source: {path}")
            digest = hashlib.sha256()
            with resolved.open("rb") as handle:
                for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                    digest.update(chunk)
            metadata = resolved.stat()
            entries.append(
                {
                    "mode": format(stat.S_IMODE(metadata.st_mode), "04o"),
                    "path": path.relative_to(project_root).as_posix(),
                    "sha256": digest.hexdigest(),
                    "size_bytes": metadata.st_size,
                }
            )

json.dump(
    {"schema_version": 1, "files": sorted(entries, key=lambda item: item["path"])},
    sys.stdout,
    ensure_ascii=False,
    separators=(",", ":"),
)
sys.stdout.write("\n")
PY
}

verify_release_overlay() {
  local release_target="$1"
  local release_state="$2"
  local recorded_manifest
  local required_executable

  case "${release_target}" in
    "${RELEASE_ROOT}/"*) ;;
    *) die "Release resolves outside ${RELEASE_ROOT}: ${release_target}" ;;
  esac
  [[ -d "${release_target}" && ! -L "${release_target}" ]] ||
    die "Release is not an owned directory: ${release_target}"
  [[ -r "${release_target}/install/setup.bash" ]] ||
    die 'Release has no readable install/setup.bash.'
  [[ -x "${release_target}" && -x "${release_target}/install" ]] ||
    die 'Release directories are not traversable.'
  case "${release_state}" in
    complete)
      [[ ! -e "${release_target}/.robotest-incomplete" ]] ||
        die 'Complete release retains its incomplete marker.'
      ;;
    incomplete)
      [[ -f "${release_target}/.robotest-incomplete" &&
        ! -L "${release_target}/.robotest-incomplete" ]] ||
        die 'Owned candidate has no regular incomplete marker.'
      ;;
    *) die "Unknown release verification state: ${release_state}" ;;
  esac
  [[ -f "${release_target}/staging-manifest.json" ]] ||
    die 'Release has no staging-manifest.json.'
  [[ -f "${release_target}/source-manifest.json" ]] ||
    die 'Release has no source-manifest.json.'
  [[ -f "${release_target}/staging-provenance.json" ]] ||
    die 'Release has no staging-provenance.json.'
  if find "${release_target}/install" -type l -print -quit | grep -q .; then
    die 'Release contains a symbolic link.'
  fi
  if grep -R -I -F -l -- "${PROJECT_ROOT}" "${release_target}/install" |
    grep -q .; then
    die 'Release contains a dependency on the development workspace.'
  fi
  for required_executable in \
    robotest_description/lib/robotest_description/generate_collision_coverage.py \
    robotest_faults/lib/robotest_faults/fault_proxy_node \
    robotest_metrics/lib/robotest_metrics/metrics_aggregate \
    robotest_metrics/lib/robotest_metrics/metrics_analyze \
    robotest_metrics/lib/robotest_metrics/metrics_collector \
    robotest_metrics/lib/robotest_metrics/metrics_lifecycle_sampler \
    robotest_metrics/lib/robotest_metrics/metrics_report \
    robotest_missions/lib/robotest_missions/mission_runner \
    robotest_navigation/lib/robotest_navigation/generate_map.py \
    robotest_navigation/lib/robotest_navigation/lifecycle_startup_trigger \
    robotest_scenarios/lib/robotest_scenarios/contact_control_driver \
    robotest_scenarios/lib/robotest_scenarios/scenario_controller \
    robotest_sim/lib/robotest_sim/phase1_runtime_probe.py; do
    [[ -f "${release_target}/install/${required_executable}" &&
      -x "${release_target}/install/${required_executable}" ]] ||
      die "Release executable is missing or not executable: ${required_executable}"
  done

  if [[ -z "${TARGET_MANIFEST}" ]]; then
    TARGET_MANIFEST="$(mktemp /tmp/robotest-overlay-target-manifest.XXXXXX)"
  fi
  write_manifest "${release_target}/install" "${TARGET_MANIFEST}" "${release_target}"
  recorded_manifest="${release_target}/staging-manifest.json"
  cmp --silent "${TARGET_MANIFEST}" "${recorded_manifest}" ||
    die 'Release content does not match its staging manifest.'
  python3 - \
    "${release_target}/staging-provenance.json" \
    "${release_target}" \
    "$(sha256sum "${recorded_manifest}" | awk '{print $1}')" \
    "$(sha256sum "${release_target}/source-manifest.json" | awk '{print $1}')" <<'PY'
import json
import pathlib
import re
import sys

provenance_path, active_target, install_sha, source_sha = sys.argv[1:]
with pathlib.Path(provenance_path).open(encoding="utf-8") as handle:
    value = json.load(handle)
expected = {
    "active_path",
    "build_command",
    "created_utc",
    "git_commit",
    "git_dirty",
    "install_manifest_sha256",
    "packages",
    "release_id",
    "schema_version",
    "source_manifest_sha256",
    "source_workspace",
}
if not isinstance(value, dict) or set(value) != expected:
    raise SystemExit("staging provenance has the wrong schema")
if value["schema_version"] != 1 or value["active_path"] != "/opt/robotest-lab":
    raise SystemExit("staging provenance has invalid fixed fields")
if not isinstance(value["git_dirty"], bool):
    raise SystemExit("staging provenance git_dirty is not boolean")
if not re.fullmatch(r"[0-9a-f]{40}", value["git_commit"]):
    raise SystemExit("staging provenance git_commit is invalid")
if value["release_id"] != pathlib.Path(active_target).name:
    raise SystemExit("staging provenance release_id does not match its directory")
if value["install_manifest_sha256"] != install_sha:
    raise SystemExit("staging provenance install manifest hash mismatch")
if value["source_manifest_sha256"] != source_sha:
    raise SystemExit("staging provenance source manifest hash mismatch")
expected_packages = [
    "robotest_description",
    "robotest_faults",
    "robotest_interfaces",
    "robotest_metrics",
    "robotest_missions",
    "robotest_navigation",
    "robotest_scenarios",
    "robotest_sim",
]
if value["packages"] != expected_packages:
    raise SystemExit("staging provenance package list is not the frozen runtime set")
if not isinstance(value["build_command"], list) or not value["build_command"]:
    raise SystemExit("staging provenance build command is empty")
PY
}

verify_active_overlay() {
  local active_target

  [[ -L "${ACTIVE_LINK}" ]] || die "Active overlay is not an owned symlink: ${ACTIVE_LINK}"
  active_target="$(realpath -e -- "${ACTIVE_LINK}")"
  verify_release_overlay "${active_target}" complete

  mkdir -p -- "${EVIDENCE_DIR}"
  if ((EUID == 0)); then
    chown --reference="${PROJECT_ROOT}" "${EVIDENCE_DIR}"
  fi
  EVIDENCE_TEMP="$(mktemp "${EVIDENCE_DIR}/.runtime-staging.XXXXXX")"
  install -m 0644 -- "${active_target}/staging-provenance.json" \
    "${EVIDENCE_TEMP}"
  if ((EUID == 0)); then
    chown --reference="${PROJECT_ROOT}" "${EVIDENCE_TEMP}"
  fi
  mv -f -- "${EVIDENCE_TEMP}" "${EVIDENCE_DIR}/runtime-staging.json"
  EVIDENCE_TEMP=""
  log "Verified active overlay: ${active_target}"
}

apply_overlay() {
  local active_target build_group build_user git_commit git_dirty install_manifest_sha release_id
  local release_path source_manifest_sha
  local -a build_command packages

  ((EUID == 0)) ||
    die 'Apply mode requires root; rerun this exact script with sudo.'
  [[ -d "${PROJECT_ROOT}/src" && -d "${PROJECT_ROOT}/config" && \
    -d "${PROJECT_ROOT}/scenarios" ]] ||
    die 'Runtime source/config/scenario roots are incomplete.'
  if systemctl is-active --quiet robotest-supervisor.service; then
    die 'robotest-supervisor.service must be inactive before staging.'
  fi
  if [[ -e "${ACTIVE_LINK}" && ! -L "${ACTIVE_LINK}" ]]; then
    die "Refusing to replace non-symlink path: ${ACTIVE_LINK}"
  fi
  build_user="$(stat -c '%U' -- "${PROJECT_ROOT}")"
  build_group="$(stat -c '%G' -- "${PROJECT_ROOT}")"
  [[ "${build_user}" =~ ^[a-z_][a-z0-9_-]*$ && "${build_user}" != "root" ]] ||
    die "The source workspace must be owned by a non-root build user: ${build_user}"
  id "${build_user}" >/dev/null 2>&1 || die "Unknown build user: ${build_user}"

  SOURCE_MANIFEST="$(mktemp /tmp/robotest-overlay-source-manifest.XXXXXX)"
  SOURCE_MANIFEST_END="$(mktemp /tmp/robotest-overlay-source-end.XXXXXX)"
  TARGET_MANIFEST="$(mktemp /tmp/robotest-overlay-target-manifest.XXXXXX)"
  PROVENANCE="$(mktemp /tmp/robotest-overlay-provenance.XXXXXX)"
  PACKAGE_LIST="$(mktemp /tmp/robotest-overlay-packages.XXXXXX)"
  write_source_manifest "${SOURCE_MANIFEST}"
  source_manifest_sha="$(sha256sum "${SOURCE_MANIFEST}" | awk '{print $1}')"
  git_commit="$(git -c safe.directory="${PROJECT_ROOT}" -C "${PROJECT_ROOT}" rev-parse HEAD)"
  if [[ -n "$(git -c safe.directory="${PROJECT_ROOT}" -C "${PROJECT_ROOT}" \
    status --porcelain=v1 --untracked-files=all)" ]]; then
    git_dirty=true
  else
    git_dirty=false
  fi
  release_id="${git_commit:0:12}-${source_manifest_sha:0:16}-${git_dirty}"
  release_path="${RELEASE_ROOT}/${release_id}"

  run_root install -d -m 0755 -o root -g root -- "${RELEASE_ROOT}"
  if [[ -e "${release_path}" ]]; then
    [[ -d "${release_path}" && ! -L "${release_path}" ]] ||
      die "Existing release is not a directory: ${release_path}"
    if [[ -f "${release_path}/.robotest-incomplete" ]]; then
      active_target="$(realpath -e -- "${ACTIVE_LINK}" 2>/dev/null || true)"
      [[ "${active_target}" != "${release_path}" ]] ||
        die 'Refusing to remove an incomplete release selected as active.'
      run_root rm -rf -- "${release_path}"
    else
      cmp --silent "${SOURCE_MANIFEST}" "${release_path}/source-manifest.json" ||
        die "Existing release ID has different source content: ${release_path}"
    fi
  fi

  if [[ ! -e "${release_path}" ]]; then
    run_root mkdir -- "${release_path}"
    STAGING_DIRECTORY="${release_path}"
    STAGING_OWNED=1
    run_root chmod 0755 -- "${release_path}"
    run_root chown root:root -- "${release_path}"
    run_root touch -- "${release_path}/.robotest-incomplete"
    run_root chown -R "${build_user}:${build_group}" -- "${release_path}"

    mapfile -t packages < <(
      # Expansion is intentionally deferred to the unprivileged inner shell.
      # shellcheck disable=SC2016
      runuser --user "${build_user}" -- bash -c '
        set -Eeuo pipefail
        set +u
        source /opt/ros/jazzy/setup.bash
        set -u
        exec colcon list --base-paths "$1" --names-only
      ' bash "${PROJECT_ROOT}/src" | LC_ALL=C sort
    )
    ((${#packages[@]} > 0)) || die 'No colcon packages were discovered.'
    printf '%s\n' "${packages[@]}" >"${PACKAGE_LIST}"

    build_command=(
      colcon
      --log-base "${release_path}/.log"
      build
      --base-paths "${PROJECT_ROOT}/src"
      --build-base "${release_path}/.build"
      --install-base "${release_path}/install"
      --executor parallel
      --parallel-workers 4
      --event-handlers console_direct+
      --cmake-args -DBUILD_TESTING=OFF
    )
    # Expansion is intentionally deferred to the unprivileged inner shell.
    # shellcheck disable=SC2016
    runuser --user "${build_user}" -- bash -c '
      set -Eeuo pipefail
      set +u
      # shellcheck disable=SC1091
      source /opt/ros/jazzy/setup.bash
      set -u
      exec "$@"
    ' bash "${build_command[@]}"

    run_root rm -rf -- "${release_path}/.build" "${release_path}/.log"
    run_root find "${release_path}/install" -type f \
      \( -name '*.pyc' -o -name '*.pyo' \) -delete
    run_root find "${release_path}/install" -depth -type d \
      -name '__pycache__' -empty -delete
    if find "${release_path}/install" -type l -print -quit | grep -q .; then
      die 'Non-symlink colcon staging produced a symbolic link.'
    fi
    if grep -R -I -F -l -- "${PROJECT_ROOT}" "${release_path}/install" |
      grep -q .; then
      die 'Staged overlay contains a dependency on the development workspace.'
    fi

    write_source_manifest "${SOURCE_MANIFEST_END}"
    cmp --silent "${SOURCE_MANIFEST}" "${SOURCE_MANIFEST_END}" ||
      die 'Runtime source changed while the release was being built.'
    write_manifest \
      "${release_path}/install" \
      "${TARGET_MANIFEST}" \
      "${release_path}"
    install_manifest_sha="$(sha256sum "${TARGET_MANIFEST}" | awk '{print $1}')"

    python3 - \
      "${git_commit}" \
      "${git_dirty}" \
      "${source_manifest_sha}" \
      "${install_manifest_sha}" \
      "${release_id}" \
      "${PROJECT_ROOT}" \
      "${release_path}" \
      "${PACKAGE_LIST}" >"${PROVENANCE}" <<'PY'
import json
import pathlib
import sys
from datetime import datetime, timezone

(
    commit,
    dirty,
    source_manifest_sha,
    install_manifest_sha,
    release_id,
    source_workspace,
    release_path,
    package_list_path,
) = sys.argv[1:]
packages = pathlib.Path(package_list_path).read_text(encoding="utf-8").splitlines()
build_command = [
    "colcon",
    "--log-base",
    f"{release_path}/.log",
    "build",
    "--base-paths",
    f"{source_workspace}/src",
    "--build-base",
    f"{release_path}/.build",
    "--install-base",
    f"{release_path}/install",
    "--executor",
    "parallel",
    "--parallel-workers",
    "4",
    "--event-handlers",
    "console_direct+",
    "--cmake-args",
    "-DBUILD_TESTING=OFF",
]
json.dump(
    {
        "schema_version": 1,
        "active_path": "/opt/robotest-lab",
        "build_command": build_command,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "git_commit": commit,
        "git_dirty": dirty == "true",
        "install_manifest_sha256": install_manifest_sha,
        "packages": packages,
        "release_id": release_id,
        "source_manifest_sha256": source_manifest_sha,
        "source_workspace": source_workspace,
    },
    sys.stdout,
    indent=2,
    sort_keys=True,
)
sys.stdout.write("\n")
PY
    run_root install -m 0644 -o root -g root -- \
      "${SOURCE_MANIFEST}" "${release_path}/source-manifest.json"
    run_root install -m 0644 -o root -g root -- \
      "${TARGET_MANIFEST}" "${release_path}/staging-manifest.json"
    run_root install -m 0644 -o root -g root -- \
      "${PROVENANCE}" "${release_path}/staging-provenance.json"
    run_root chown -R root:root -- "${release_path}"
    run_root chmod 0755 -- "${release_path}"
    run_root find "${release_path}" -type d -exec chmod go-w -- {} +
    run_root find "${release_path}" -type f -exec chmod a+r,go-w -- {} +
  fi

  if ((STAGING_OWNED)); then
    verify_release_overlay "${release_path}" incomplete
    run_root rm -f -- "${release_path}/.robotest-incomplete"
    verify_release_overlay "${release_path}" complete
    STAGING_DIRECTORY=""
    STAGING_OWNED=0
  else
    verify_release_overlay "${release_path}" complete
  fi

  if [[ -L "${ACTIVE_LINK}" ]]; then
    PREVIOUS_ACTIVE_TARGET="$(realpath -e -- "${ACTIVE_LINK}")"
    case "${PREVIOUS_ACTIVE_TARGET}" in
      "${RELEASE_ROOT}/"*) ;;
      *) die "Previous active overlay resolves outside ${RELEASE_ROOT}." ;;
    esac
  fi
  TEMP_LINK="${RELEASE_ROOT}/.active-link.${release_id}.$$"
  run_root ln -s -- "${release_path}" "${TEMP_LINK}"
  run_root mv -Tf -- "${TEMP_LINK}" "${ACTIVE_LINK}"
  TEMP_LINK=""
  PROMOTION_PENDING=1
  verify_active_overlay
  PROMOTION_PENDING=0
}

if (($# > 1)); then
  usage >&2
  exit 2
fi
if (($# == 1)); then
  case "$1" in
    --check) MODE="check" ;;
    --apply) MODE="apply" ;;
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
for command_name in awk cmp find flock git grep install mkdir mv python3 realpath sha256sum sort systemctl; do
  require_command "${command_name}"
done
if [[ "${MODE}" == "apply" ]]; then
  for command_name in colcon id runuser stat; do
    require_command "${command_name}"
  done
fi

if [[ "${MODE}" == "apply" ]]; then
  [[ ! -L "${LOCK_FILE}" ]] || die 'The staging lock path must not be a symbolic link.'
  touch -- "${LOCK_FILE}"
  chmod 0644 -- "${LOCK_FILE}"
  chown root:root -- "${LOCK_FILE}"
  exec 9<>"${LOCK_FILE}"
else
  if [[ ! -e "${LOCK_FILE}" ]]; then
    (umask 022; set -o noclobber; : >"${LOCK_FILE}") 2>/dev/null || true
  fi
  [[ -f "${LOCK_FILE}" && ! -L "${LOCK_FILE}" && -r "${LOCK_FILE}" ]] ||
    die 'The staging lock has not been safely initialized by apply mode.'
  exec 9<"${LOCK_FILE}"
fi
flock -n 9 || die 'Another runtime staging or verification operation is active.'

if [[ "${MODE}" == "apply" ]]; then
  apply_overlay
else
  verify_active_overlay
fi

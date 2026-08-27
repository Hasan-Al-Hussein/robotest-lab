#!/bin/bash
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

set -Eeuo pipefail

die() {
  printf '[build_debian_package] ERROR: %s\n' "$*" >&2
  exit 1
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

write_source_manifest() {
  local layout="$1"
  local tree_root="$2"
  local output_path="$3"
  python3 - "${layout}" "${tree_root}" >"${output_path}" <<'PY'
import hashlib
import json
import os
import pathlib
import stat
import sys

layout, root_value = sys.argv[1:]
root = pathlib.Path(root_value).resolve(strict=True)
if layout == "repository":
    mappings = (
        (root / "supervisor" / "cmd", pathlib.Path("cmd")),
        (root / "supervisor" / "internal", pathlib.Path("internal")),
        (root / "supervisor" / "go.mod", pathlib.Path("go.mod")),
        (root / "supervisor" / "README.md", pathlib.Path("README.md")),
        (
            root / "supervisor" / "config.example.json",
            pathlib.Path("config.example.json"),
        ),
        (root / "LICENSE", pathlib.Path("LICENSE")),
        (root / "packaging" / "debian", pathlib.Path("debian")),
    )
elif layout == "staged":
    mappings = tuple((root / name, pathlib.Path(name)) for name in (
        "cmd",
        "internal",
        "go.mod",
        "README.md",
        "config.example.json",
        "LICENSE",
        "debian",
    ))
else:
    raise SystemExit(f"unsupported source layout: {layout}")

entries = []


def add_file(path: pathlib.Path, destination: pathlib.Path) -> None:
    if path.is_symlink():
        raise SystemExit(f"symbolic-link package source is forbidden: {path}")
    resolved = path.resolve(strict=True)
    if not resolved.is_file():
        raise SystemExit(f"non-regular package source: {path}")
    digest = hashlib.sha256()
    with resolved.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    metadata = resolved.stat()
    entries.append(
        {
            "mode": format(stat.S_IMODE(metadata.st_mode), "04o"),
            "path": destination.as_posix(),
            "sha256": digest.hexdigest(),
            "size_bytes": metadata.st_size,
        }
    )


for source, destination in mappings:
    if source.is_dir() and not source.is_symlink():
        for directory, directory_names, file_names in os.walk(source, followlinks=False):
            directory_path = pathlib.Path(directory)
            for name in directory_names:
                candidate = directory_path / name
                if candidate.is_symlink():
                    raise SystemExit(
                        f"symbolic-link package source directory is forbidden: {candidate}"
                    )
            directory_names[:] = sorted(directory_names)
            for name in sorted(file_names):
                candidate = directory_path / name
                add_file(candidate, destination / candidate.relative_to(source))
    else:
        add_file(source, destination)

json.dump(
    {"schema_version": 1, "files": sorted(entries, key=lambda item: item["path"])},
    sys.stdout,
    ensure_ascii=False,
    separators=(",", ":"),
    sort_keys=True,
)
sys.stdout.write("\n")
PY
}

if [[ $# -ne 1 ]]; then
  printf 'usage: %s OUTPUT_DIRECTORY\n' "$0" >&2
  exit 2
fi

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
output_directory=$(realpath -m -- "$1")
case "${output_directory}" in
  /|"${repo_root}")
    printf 'refusing unsafe output directory: %s\n' "${output_directory}" >&2
    exit 2
    ;;
esac
mkdir -p -- "${output_directory}"
if find "${output_directory}" -mindepth 1 -maxdepth 1 -print -quit | grep -q .; then
  printf 'output directory must be empty: %s\n' "${output_directory}" >&2
  exit 2
fi

[[ -r /etc/os-release ]] || die '/etc/os-release is unavailable.'
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" && \
  "${VERSION_CODENAME:-}" == "noble" ]] ||
  die 'Package builds require Ubuntu 24.04 Noble.'
[[ "${WSL_DISTRO_NAME:-}" != "Ubuntu-20.04" ]] ||
  die 'Refusing to build in the protected Ubuntu-20.04 distribution.'
for command_name in cmp dpkg-buildpackage dpkg-parsechangelog git install python3 sha256sum sort; do
  require_command "${command_name}"
done
if find "${repo_root}/supervisor" -type l -print -quit | grep -q .; then
  die 'Supervisor source must not contain symbolic links.'
fi

package_version=$(dpkg-parsechangelog \
  -l"${repo_root}/packaging/debian/changelog" \
  -SVersion)
[[ "${package_version}" =~ ^[0-9][0-9A-Za-z.+:~\-]*$ ]] ||
  die "Invalid changelog version: ${package_version}"

staging=$(mktemp -d /tmp/robotest-supervisor-build.XXXXXX)
source_manifest_pre=${staging}/source-manifest-pre.json
source_manifest_staged=${staging}/source-manifest-staged.json
source_manifest_post=${staging}/source-manifest-post.json
cleanup() {
  case "${staging}" in
    /tmp/robotest-supervisor-build.*) rm -rf -- "${staging}" ;;
    *) printf 'refusing unsafe staging cleanup: %s\n' "${staging}" >&2 ;;
  esac
}
trap cleanup EXIT

source_root=${staging}/robotest-supervisor-${package_version}
write_source_manifest repository "${repo_root}" "${source_manifest_pre}"
mkdir -p -- "${source_root}/debian"
cp -a -- "${repo_root}/supervisor/cmd" "${source_root}/cmd"
cp -a -- "${repo_root}/supervisor/internal" "${source_root}/internal"
install -m 0644 -- "${repo_root}/supervisor/go.mod" "${source_root}/go.mod"
install -m 0644 -- "${repo_root}/supervisor/README.md" "${source_root}/README.md"
install -m 0644 -- "${repo_root}/supervisor/config.example.json" \
  "${source_root}/config.example.json"
install -m 0644 -- "${repo_root}/LICENSE" "${source_root}/LICENSE"
cp -a -- "${repo_root}/packaging/debian/." "${source_root}/debian/"
write_source_manifest staged "${source_root}" "${source_manifest_staged}"
cmp --silent "${source_manifest_pre}" "${source_manifest_staged}" ||
  die 'The staged package source does not match its pre-copy manifest.'

export DEB_BUILD_OPTIONS=parallel=4
export SOURCE_DATE_EPOCH
SOURCE_DATE_EPOCH=$(git -C "${repo_root}" log -1 --format=%ct)
(
  cd -- "${source_root}"
  dpkg-buildpackage --build=binary --no-sign --jobs=4
)
write_source_manifest repository "${repo_root}" "${source_manifest_post}"
cmp --silent "${source_manifest_pre}" "${source_manifest_post}" ||
  die 'Package source changed while the package was being built.'

artifacts=()
while IFS= read -r -d '' artifact; do
  install -m 0644 -- "${artifact}" "${output_directory}/$(basename -- "${artifact}")"
  artifacts+=("$(basename -- "${artifact}")")
done < <(
  find "${staging}" -type f \
    \( -name "robotest-supervisor_${package_version}_*.deb" \
    -o -name "robotest-supervisor-dbgsym_${package_version}_*.ddeb" \
    -o -name "robotest-supervisor_${package_version}_*.buildinfo" \
    -o -name "robotest-supervisor_${package_version}_*.changes" \) \
    -print0 | sort -z
)
[[ ${#artifacts[@]} -eq 4 ]] ||
  die "Package build produced ${#artifacts[@]} artifacts; expected exactly 4."
install -m 0644 -- "${source_manifest_pre}" "${output_directory}/SOURCE-MANIFEST.json"
(
  cd -- "${output_directory}"
  sha256sum -- "${artifacts[@]}" SOURCE-MANIFEST.json > SHA256SUMS
)
printf 'package artifacts: %s\n' "${output_directory}"

# RoboTest Lab dependency plan

Evidence timestamp: **2026-08-25 22:19:03 UTC+04:00** (Asia/Dubai / Arabian Standard Time)

## Current status

No dependency transaction has been performed. The package manifest is a reviewed **projected desired state**, not proof that the packages are installed.

Measured apt state:

- Ubuntu release: 24.04.1 LTS, codename `noble`.
- Active Ubuntu suites: `noble`, `noble-updates`, `noble-backports`, and `noble-security`.
- Active components: `main universe restricted multiverse`.
- Official ROS 2 apt source: **absent**.
- Newest cached Ubuntu InRelease metadata observed: 2026-08-16. Package versions and size projections must be refreshed before installation.

## Provenance

Only official upstream sources are approved:

- ROS 2 Jazzy Ubuntu deb installation: <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>
- ROS apt-source packaging and key/source ownership: <https://github.com/ros-infrastructure/ros-apt-source>
- Gazebo Harmonic with ROS: <https://gazebosim.org/docs/harmonic/ros_installation/>
- Nav2 documentation: <https://docs.nav2.org/getting_started/index.html>
- WSL global and per-distribution configuration: <https://learn.microsoft.com/en-us/windows/wsl/wsl-config>
- Ubuntu Noble package index: <https://packages.ubuntu.com/noble/>
- Debian Policy: <https://www.debian.org/doc/debian-policy/>
- Ruff release provenance: <https://pypi.org/project/ruff/0.16.4/>

Do not add a third-party ROS mirror, Gazebo PPA, arbitrary keyserver key, or testing repository. The ROS apt-source package embeds and owns the expected source/key material.

## Pinned ROS apt-source bootstrap — projected

The official GitHub release API reported this production asset on the audit date:

| Field | Pinned value |
| --- | --- |
| Release | `1.2.0` |
| Published | 2026-04-23 |
| Asset | `ros2-apt-source_1.2.0.noble_all.deb` |
| URL | <https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.noble_all.deb> |
| SHA-256 | `0804d9b13db770eb87019be414cd78378835228ad5fa801fc88758596dd8f7e5` |
| Reported size | 4,434 bytes |

The future setup procedure must download this exact asset to a `mktemp -d` directory, compare its SHA-256 before package inspection or installation, reject any mismatch, and clean the temporary directory through a trap. The similarly named `ros2-testing-apt-source` asset is not approved.

This pin must be reviewed when intentionally refreshing dependencies; silently following a moving “latest” URL is not reproducible.

## Manifest contract

`config/apt-packages.txt` is UTF-8/LF and contains one literal package name per nonblank line, without comments, shell fragments, version expressions, or inline whitespace. An installer must validate every line against:

```text
^[a-z0-9][a-z0-9.+-]*$
```

It must pass validated names as an argument array and never use `eval` or interpolate the file into a shell command.

The manifest intentionally records direct project tooling even when a metapackage could currently pull it transitively. This makes test, visualization, and packaging requirements visible and protects against future metapackage changes.

## Package rationale

### Bootstrap and native build

`ca-certificates`, `curl`, and `gnupg` establish authenticated source retrieval.
`git`, `build-essential`, `cmake`, `ninja-build`, and `pkg-config` support the
mixed C++/Python workspace. `libssl-dev` supplies the OpenSSL EVP SHA-256 API
used to recompute canonical fault-schedule digests, and
`nlohmann-json3-dev` supplies the strict bounded C++ JSON parser used for
mode-specific fault parameters. Several packages are already installed, but
remain in the desired-state manifest for reproducibility.

### Static analysis and developer checks

`clang-format`, `clang-tidy`, `cppcheck`, `shellcheck`, and `pre-commit` provide complementary formatting and static checks. `clang-tidy` is retained despite its LLVM footprint because it is a master quality requirement; it should not run in latency-sensitive simulation processes.

### Python experiments and tests

`python3-pip`, `python3-venv`, `python3-pytest`, `python3-pytest-cov`, `python3-yaml`, `python3-jsonschema`, and `python3-matplotlib` support isolated developer tooling, schema validation, tests, metrics, and plots.

Ubuntu Noble has no apt candidate named `python3-ruff` in the measured standard sources. Ruff is therefore deliberately absent from the apt manifest. Use a project-local virtual environment and pin:

```text
ruff==0.16.4
```

The version was verified against PyPI on the evidence date. Do not install Ruff into the system Python. When the Python development lock file is created, record hashes for the selected Noble-compatible wheel and install it with hash checking.

### Go supervisor and Debian package QA

`golang-go` provides Go 1.22 on Noble. `dpkg-dev`, `debhelper`, `dh-golang`, `fakeroot`, and `lintian` cover source package construction, Go integration, unprivileged builds, and policy QA.

`devscripts` is intentionally omitted. The host already has `dpkg-dev` and `fakeroot`; the selected packaging tools support the required local `.deb` workflow without the additional helper suite.

### ROS 2 Jazzy, Gazebo Harmonic, and Nav2

- `ros-dev-tools`: colcon/rosdep/vcs and ROS development workflow.
- `ros-jazzy-ros-base`: lean ROS 2 runtime and command-line foundation.
- `ros-jazzy-ros-gz`: Jazzy-aligned ROS/Gazebo integration; for Jazzy the intended simulator family is Gazebo Harmonic.
- `ros-jazzy-navigation2` and `ros-jazzy-nav2-bringup`: Nav2 runtime and bringup.
- `ros-jazzy-rviz2`: manual visualization and evidence, never an automatic headless dependency.
- `ros-jazzy-xacro`, `ros-jazzy-robot-state-publisher`, and `ros-jazzy-joint-state-publisher`: robot description and TF support.
- `ros-jazzy-teleop-twist-keyboard` and `ros-jazzy-tf2-tools`: bounded manual smoke tests and TF diagnostics.
- `ros-jazzy-rosbag2-storage-mcap`: explicit MCAP experiment recording.
- `ros-jazzy-launch-testing-ament-cmake` and `ros-jazzy-ament-cmake-gtest`: launch/integration and C++ unit-test integration.

No TurtleBot, vendor robot, camera, GPU, ML, Docker, SLAM, or hardware-driver package is included.

## Measured availability

Already installed during the audit included `ca-certificates`, `curl`, `gnupg`, `git`, `build-essential`, `python3-pip`, `python3-yaml`, `python3-jsonschema`, `dpkg-dev`, and `fakeroot`.

The standard Noble cache contained candidates for all selected non-ROS packages except Ruff. It had no candidates for `ros-dev-tools` or any `ros-jazzy-*` package because the official ROS source had not yet been configured.

## Projected footprint

A read-only `apt-get -s` transaction for the selected missing non-ROS quality set, including `clang-tidy` and excluding `devscripts`, projected:

- 53 dependency upgrades;
- 284 newly installed packages;
- approximately 382.9 MiB of archives;
- approximately 1,438.6 MiB of new installed payload.

These figures were derived from the cached candidate `Size` and `Installed-Size` fields. They exclude ROS 2, Gazebo, Nav2, RViz, apt/dpkg overhead, build outputs, bags, logs, and upgrade-size deltas. They are **not** an installation promise.

## Installation gate

Before any real transaction:

1. Confirm the command targets `Ubuntu`, never `Ubuntu-20.04`.
2. Verify the pinned apt-source asset checksum and inspect its package metadata.
3. Install the apt-source package, then run one authorized `apt update`.
4. Confirm every manifest entry has a candidate from the expected Ubuntu or official ROS origin.
5. Run `apt-get -s install` for the exact manifest and record package counts and projected disk use.
6. Recheck physical C: free space and host memory pressure.
7. Install once, then capture `dpkg-query`, `ros2 doctor`, ROS/Gazebo/Nav2 versions, and import/build smoke-test evidence.

Use normal recommended dependencies for ROS metapackages unless each omission has been verified. A global `--no-install-recommends` optimization is not approved for the ROS/Gazebo/Nav2 transaction.

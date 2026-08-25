# RoboTest Lab environment audit

Audit timestamp: **2026-08-25 22:19:03 UTC+04:00** (Asia/Dubai / Arabian Standard Time)

## Scope and evidence policy

This Phase 0 audit was observational. It used Windows CIM and `wsl.exe` queries plus one-shot, noninteractive read commands in the registered Linux distributions. It did not install or update packages, add repositories, edit WSL configuration, change a default distribution, terminate a distribution, or shut down WSL.

The terms in this document are deliberate:

- **Measured** means the value was returned by a command on this machine during the audit.
- **Projected** means a desired setting, package footprint, or future result that has not yet been applied or verified.
- Guest `df` capacity is not host capacity. WSL virtual disks are sparse, so the Windows C: free-space figure is the binding storage evidence.

## Windows host — measured

| Item | Observation |
| --- | --- |
| Operating system | Microsoft Windows 11 Pro, version `10.0.26200`, build `26200.9168`, DisplayVersion `25H2` |
| Computer | HP Laptop 15s-fq5xxx |
| Processor | 12th Gen Intel Core i7-1255U; 10 physical cores; 12 logical processors |
| Hypervisor | `HypervisorPresent=True`; live WSL 2 guests confirm the virtualization substrate is operational |
| Physical memory | 15.68 GiB total |
| Available memory before guest probes | 2.12–2.22 GiB |
| Available memory after Ubuntu was awakened beside Docker Desktop | 0.90 GiB |
| C: storage | 475.92 GiB total; 47.72 GiB free |

`Win32_Processor` reported `VirtualizationFirmwareEnabled=False` and `SecondLevelAddressTranslationExtensions=False` while the Microsoft hypervisor was active. Those two fields are not treated as blockers because `HypervisorPresent=True` and functioning WSL 2 guests are stronger runtime evidence.

## WSL host — measured

| Item | Observation |
| --- | --- |
| WSL runtime | `2.7.12.0` |
| Linux kernel | `6.18.33.2-2` (`6.18.33.2-microsoft-standard-WSL2` in guests) |
| WSLg | `1.0.73.2` |
| Default distribution | `Ubuntu-20.04` |
| Default WSL version | 2 |
| Host `.wslconfig` | Absent at `C:\Users\hp\.wslconfig` |
| Observed networking mode | NAT |

Initial distribution state from `wsl.exe --list --verbose`:

| Distribution | Initial state | WSL version | Identity |
| --- | --- | --- | --- |
| `Ubuntu-20.04` | Stopped | 2 | Ubuntu 20.04.6 LTS (Focal) |
| `Ubuntu` | Stopped | 2 | Ubuntu 24.04.1 LTS (Noble) |
| `docker-desktop` | Running | 2 | Not part of the RoboTest Lab core design |

The one-shot guest audit awakened the Ubuntu distributions. `Ubuntu-20.04` returned to Stopped without intervention. `Ubuntu` remained Running; it was not terminated because termination and shutdown were outside the read-only audit authorization.

Registered virtual-disk file lengths were 5.16 GiB for `Ubuntu-20.04`, 10.79 GiB for `Ubuntu`, and 0.09 GiB for `docker-desktop`. These are not free-space measurements.

## Guest readiness — measured

### Preserved Ubuntu 20.04

- `/etc/wsl.conf` contains `[boot] systemd=true`.
- PID 1 is `systemd`; after cold-start settling, systemd reported `running` with zero failed units.
- GCC/G++ 9.4.0 and Python 3.8.10 are present.
- CMake, pip3, Go, colcon, ROS 2, Gazebo, and Nav2 are absent.
- This distribution remains the Windows-side default and is not a project target.

### Project Ubuntu 24.04

- `/etc/os-release` identifies Ubuntu 24.04.1 LTS (`noble`).
- `/etc/wsl.conf` contains `[boot] systemd=true`.
- PID 1 is `systemd`; systemd reported `running` with zero failed units.
- GCC/G++ 13.3.0, Python 3.12.3, and pip 24.0 are present.
- CMake, Go, colcon, ROS 2 Jazzy, Gazebo Harmonic / `gz`, Nav2, and RViz are absent.
- This existing distribution is the selected RoboTest Lab target. A second Ubuntu 24.04 distribution is unnecessary.

## Resource envelope and deferred `.wslconfig`

The running WSL utility VM measured 7.6 GiB usable memory, 12 processors, and 2.0 GiB swap. The project envelope is **projected** as 8 GiB memory, 6 processors, and 4 GiB swap.

Creating `.wslconfig` is deferred for two reasons:

1. It is a global host configuration that affects every WSL 2 distribution, including the preserved Ubuntu 20.04 and Docker Desktop workload.
2. Applying it requires `wsl --shutdown`, which was explicitly outside the read-only audit and can interrupt running work.

The future configuration gate must show the proposed file, confirm no conflicting workload is active, obtain the required authorization, apply the change once, and then re-measure `free -h`, `nproc`, and swap inside `Ubuntu`. Until that evidence exists, 8/6/4 remains a target rather than a claim.

## Risks and decisions

- **Preservation:** never unregister, upgrade in place, retarget, or implicitly execute project commands in `Ubuntu-20.04`. Every project command must name `Ubuntu` explicitly with `wsl.exe -d Ubuntu -- ...`.
- **Memory pressure:** only 0.90 GiB host memory was available when Docker Desktop and Ubuntu were active. Docker is not part of the core design; close it and other nonessential workloads before CPU-only simulation when practical.
- **Storage pressure:** 47.72 GiB host free space is workable but limited for ROS/Gazebo, build trees, apt caches, logs, and bags. Recheck it before installation and before long experiments; apply bounded retention.
- **Filesystem placement:** keep the colcon workspace, build, install, log, bags, and benchmarks under the Linux ext4 home, not OneDrive or `/mnt/c`, for Linux semantics and predictable build performance.
- **Headless default:** Gazebo server and automated experiments must run without GUI by default. RViz and Gazebo GUI are manual evidence tools, not service dependencies.

## Phase 0 exit gate

The host and existing Ubuntu 24.04 distribution are suitable for a CPU-only,
headless ROS 2 Jazzy / Gazebo Harmonic project. A global `.wslconfig` change is
not required to pass Phase 0 while unrelated containers are active: the
measured memory ceiling is already approximately 8 GiB, and RoboTest commands
will enforce six-CPU affinity plus four-worker build limits locally. The
4 GiB-swap setting remains a documented future host-maintenance option, not an
unverified completion claim.

The remaining state-changing gates are:

1. Configure the pinned official ROS 2 apt-source package and verify its checksum.
2. Refresh apt metadata, re-run package policy and simulated installation, and confirm host disk headroom.
3. Install only after the previous evidence is recorded.

## Command evidence

Representative commands used were `Get-CimInstance Win32_OperatingSystem`, `Get-CimInstance Win32_ComputerSystem`, `Get-CimInstance Win32_Processor`, `Get-CimInstance Win32_LogicalDisk`, `wsl.exe --version`, `wsl.exe --status`, `wsl.exe --list --verbose`, and per-distro reads of `/etc/os-release`, `/etc/wsl.conf`, `uname`, `free`, `df`, `systemctl`, compiler/runtime versions, `/opt/ros`, and package status.

Authoritative WSL configuration reference: <https://learn.microsoft.com/en-us/windows/wsl/wsl-config>

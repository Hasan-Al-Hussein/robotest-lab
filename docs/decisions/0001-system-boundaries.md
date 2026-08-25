# ADR 0001: System and Evidence Boundaries

- Status: Accepted for Phase 0
- Date: 2026-08-25
- Decision owners: RoboTest Lab architecture gate

## Context

RoboTest Lab must demonstrate ROS 2 autonomy validation without exceeding the
host budget, altering an unrelated Ubuntu installation, letting validation
truth leak into autonomy, or allowing the deployment layers to restart the same
process independently.

The verified development target is the existing WSL2 distribution named
`Ubuntu`, running Ubuntu 24.04.1 with systemd. The repository lives at
`/home/hasan/robotest-lab` on the Linux ext4 filesystem. The Windows
OneDrive workspace and `Ubuntu-20.04` are outside project scope.

## Decision

### Host and guest

- Every host-initiated Linux command explicitly targets `wsl -d Ubuntu`.
- No command changes, unregisters, exports, imports, or uses
  `Ubuntu-20.04`.
- The project is built and run from `/home/hasan/robotest-lab`, not
  `/mnt/c` or a synchronized Windows directory.
- A global `%UserProfile%\.wslconfig` change and `wsl --shutdown` are
  deferred while unrelated containers are active. The project instead limits
  build parallelism to four workers and launches the runtime under CPU affinity
  `0-5`.
- Docker is not part of the core local workflow.

The Phase 0 pre-install gate rejects a dependency projection above 18 GiB or a
post-install host reserve below 20 GiB. Later runtime gates measure the project
process tree; they do not claim unrelated WSL/container usage as project usage.

### ROS/Gazebo data boundary

Gazebo publishes sensor and odometry data into `/robotest/raw/*`. The
always-present C++ fault proxy owns the validated sensor plane
`/robotest/{scan,odom,imu}` and `odom -> base_footprint`. Nav2 and AMCL
consume only this validated plane.

Ground truth, contacts, and world statistics live under
`/robotest/validation/*`. They are read-only evidence inputs for metrics and
tests. They are not published into TF and cannot influence planning,
localization, control, recovery, or mission status.

`/clock` has one Gazebo publisher and is bridged Gazebo-to-ROS. Every
simulation participant uses simulation time. Steady wall time is reserved for
process supervision, resource sampling, and escape timeouts.

### ROS package responsibilities

| Component | Responsibility | Must not own |
| --- | --- | --- |
| `robotest_interfaces` | Shared fault/event interfaces | Runtime policy or business logic |
| `robotest_description` | Original robot geometry, frames, footprint | World, navigation, or fault behavior |
| `robotest_sim` | World, Gazebo systems, bridges, truth/contact publication, launch | Validation verdicts or Nav2 configuration |
| `robotest_navigation` | Map, AMCL, Nav2, costmaps, controller, recovery configuration | Ground-truth access |
| `robotest_missions` | YAML validation, Nav2 action orchestration, timeout/cancel, result writing | Metric fabrication or process restart |
| `robotest_faults` | Always-present bounded proxy, schedule validation, deterministic fault application, fault events, odometry TF | Mission success decisions |
| `robotest_metrics` | Evidence observation, formulas, result comparison, reports/charts | Autonomy commands or goal decisions |
| Go supervisor | Process groups, heartbeat, bounded restart, localhost status/metrics | ROS message transformation or mission verdicts |

### Command and process ownership

Nav2 produces the navigation command. A single collision monitor/final-command
arbiter owns the command bridged to Gazebo. Multiple final-command publishers
are a verification failure.

The Go supervisor owns ROS child process groups, heartbeat freshness, graceful
shutdown, escalation, bounded exponential backoff, and circuit breaking.
systemd owns only the Go supervisor. It must not independently restart the
supervisor's ROS children.

The HTTP server binds only to `127.0.0.1`:

- `/healthz`: the supervisor process can serve requests;
- `/readyz`: required children and heartbeat are healthy;
- `/v1/status`: structured state and last failure;
- `/metrics`: bounded Prometheus-compatible text.

There is no remote restart endpoint.

### Deployment and packaging

The supervisor `.deb` contains the Go binary, wrapper, default conffile,
systemd unit, state/log directory declarations, and documentation. It does not
contain the ROS workspace.

Phase 4 stages the tested runtime overlay under `/opt/robotest-lab` using the
owned staging script. The systemd unit is installed disabled by default and
contains:

`ConditionPathExists=/opt/robotest-lab/install/setup.bash`

Writable state belongs under `/var/lib/robotest-supervisor`; administrator
configuration belongs under `/etc`. Removal does not delete unrelated files
or silently discard modified configuration. Hardening must be compatible with
these explicit paths.

### Evidence boundary

Targets, observations, and verdicts are separate:

- frozen numerical targets:
  `docs/testing/acceptance-criteria.md`;
- calculation definitions:
  `docs/architecture/metrics-contract.md`;
- future observations: canonical per-run JSON/CSV and raw evidence;
- verdicts: generated by comparison, never hand-entered.

Every claim names its verification level. Static review, unit tests, simulation
runtime, public CI, and manual GUI inspection are not interchangeable.

## Consequences

### Positive

- Fault injection cannot be bypassed by a direct raw-topic or duplicate-TF
  path.
- Validation truth cannot make autonomy appear more capable than it is.
- The stack is reproducible without depending on OneDrive, Docker, paid
  services, or a dedicated GPU.
- Go and systemd have non-overlapping restart responsibilities.
- Metrics and portfolio claims remain traceable to run artifacts.

### Costs

- The pass-through proxy is required even in baseline runs.
- Runtime deployment needs an explicit staging step before the systemd service
  can become ready.
- Software-rendered LiDAR and repeated trials require aggressive resource and
  artifact controls.
- Some Definition-of-Done items remain incomplete when public CI or genuine
  media cannot be produced.

## Rejected alternatives

- Modify global WSL resources now: rejected because it would interrupt or
  affect unrelated active workloads.
- Build under the Windows/OneDrive tree: rejected because Linux build,
  permission, symlink, and synchronization behavior would contaminate evidence.
- Let Nav2 subscribe directly to Gazebo topics in baseline mode: rejected
  because later fault tests could use a different data path.
- Publish ground truth as the autonomy odometry/TF source: rejected because it
  invalidates localization and drift experiments.
- Let systemd and Go both restart ROS children: rejected because it creates
  races, duplicate processes, and unbounded restart behavior.
- Package the whole ROS workspace in the supervisor `.deb`: rejected because
  the requested package boundary is the supervisor; the deployable overlay is
  staged and verified separately.

# Phase 4 verifier operation

`scripts/verify_phase4.sh` has two deliberately different modes.

## Static preflight

```bash
taskset -c 0-5 scripts/verify_phase4.sh
```

This mode is non-mutating with respect to `/opt`, dpkg, systemd, and running
processes. It runs the literal Go race/vet gates, Go coverage, Python verifier
tests, ShellCheck, systemd unit parsing, reproducible-package checks, exact
package-source binding, lifecycle-evidence binding, and an extracted-candidate
`--check-config` run against a rendered Phase 4 config. Its successful terminal
message is `STATIC PREFLIGHT PASS ONLY`. It creates no Scenario 6 result and is
not a Phase 4 acceptance verdict. Static-only verification refuses root. During
an authoritative run, the complete static worker runs as the non-root workspace
owner with a scrubbed environment and private `/tmp` caches; root imports only
bounded, regular, single-link output after replay validation.

## Authoritative acceptance

Run this only after the Phase 3 source set is stable and the selected candidate
has a matching completed lifecycle proof:

```bash
sudo taskset -c 0-5 scripts/verify_phase4.sh --apply
```

Use `--package-dir` and `--lifecycle-evidence` to select explicit artifacts when
more than one audited candidate exists. The default resolver selects the newest
reproducibility-PASS candidate with an exact lower-version fixture, then the
newest lifecycle PASS whose upgrade SHA-256 matches that candidate.

Lifecycle evidence is accepted only when its top-level and nested key sets are
exact and every required fact is present with the expected value. This includes
modified-conffile preservation across upgrade and remove, purge removal,
sentinel/state/log preservation, final vendor-config restoration, exact
permissions and ownership, installed payload hashes, service-account groups,
the installed smoke contract, zero stale dpkg conffile artifacts, and the final
installed/disabled/inactive state. Package metadata, vendor configuration, and
installed payload hashes are independently recomputed from both selected
`.deb` files. The authoritative run also directly re-observes the installed
version, architecture, configuration hash, paths, groups, dpkg verification,
smoke contract, and service state into `installed-package-state.json` before
any staging or service start.

Package reproducibility is not trusted from a precomputed `PASS` label or from
either build's local checksum file. The verifier first checks each local
`SHA256SUMS`, then independently compares the build-a/build-b `.deb`, `.ddeb`,
and `SOURCE-MANIFEST.json` bytes. It recomputes every exact-schema
`reproducibility.json` file record (including the narrowly normalized dpkg
wall-clock metadata), binds the selected candidate to build-a, and parses the
lower-version fixture sidecar back to both package bytes and vendor defaults.
Before any live mutation, a fresh controlled source build must reproduce the
selected build-a `.deb`, `.ddeb`, and canonical `SOURCE-MANIFEST.json` bytes.
The exact build-script hash, changelog version, artifact names, sizes, hashes,
and equality results are retained in `package-source-rebuild.json`. The
authoritative path also rejects a dirty Git tree, hidden assume-unchanged or
skip-worktree index flags, a changed HEAD, or modified submodules before static
work and again before acquiring live resources. All privileged Git inspection
uses a fixed repository/work-tree and sanitized configuration with fsmonitor
and hooks disabled.

The live path holds `/run/lock/robotest-phase4-verifier.lock`, stages the
immutable overlay, and uses a run-owned config and temporary systemd drop-in.
It never edits the packaged conffile. The config receives a currently unused
ROS domain, a unique Gazebo partition, and one run-scoped state directory that
also owns the exact heartbeat and lifecycle-startup result paths. The child
environment binds `ROBOTEST_RUNTIME_STATE_DIRECTORY` to that directory. Before
starting the service, the installed candidate must return exactly
`configuration valid` for that config; the stdout is retained as
`supervisor-config-check.txt`. The drop-in changes only `ExecStart` to select
that config; the packaged service remains disabled.

After initial readiness, the verifier starts the baseline mission in a recorded
process group. A ROS status probe proves one exact `EXECUTING` FollowWaypoints
UUID and derives mission-runner goal capability from the exact
`/robotest/follow_waypoints/_action/send_goal` service-client endpoint with type
`nav2_msgs/action/FollowWaypoints_SendGoal`. Jazzy's projected action-client
participants are retained in `active-goal.json` as diagnostics, but they are
not accepted as goal-submission ownership. The injection target must be the
single nested `controller_server` that simultaneously matches:

- descendant lineage from the configured managed child;
- the original managed PGID;
- `robotest-supervisor.service` cgroup ownership;
- the run's ROS domain and Gazebo partition; and
- an executable whose resolved basename is exactly `controller_server`.

The same helper that sends `SIGTERM` immediately re-reads and compares the
complete root-to-target lineage, root and target PGIDs, start times, executable,
NUL-delimited command hash, cgroup, environment, and non-zombie state. A
command-line-only `controller_server` spoof or any drift aborts before a signal
is sent. It opens a Linux pidfd before that revalidation and signals only
through `pidfd_send_signal`; unavailable or failed pidfd operations fail closed,
with no numeric-PID fallback.

The packaged unit fixes `CPUAffinity=0-5`; the wrapper's `taskset` remains a
defense in depth. The verifier captures every process in the exact unit cgroup
after initial readiness and again after recovery. Each stable snapshot binds
MainPID, managed child PID/PGID, the sole `controller_server`, start ticks,
lineage, executable, cgroup, and both Linux CPU-mask encodings. Every retained
mask must be a nonempty subset of CPUs 0–5. These canonical snapshots are
`runtime-affinity-initial.json` and `runtime-affinity-restored.json`.

The verifier then retains monotonic HTTP/process observations, the supervisor's
structured event log and metadata, status snapshots, journal, interrupted and
follow-up mission evidence, overlay/package/source manifests, and cleanup
ledger. Each HTTP record has an exact endpoint, integer status, canonical body,
and body SHA-256. Startup and recovery are replayed as adjacent health/ready
pairs; the observed recovery bound is measured from the first raw readiness 503
to the final paired 200, with no 200-to-503 flap. `initial-status.json`,
`ready-restored-status.json`, `supervisor-status-final.json`, both exact systemd
owner snapshots, `original-group-empty.json`, `service-final.json`, and the
run-specific `systemd-dropin.conf` are schema- and sequence-joined rather than
trusted through timeline summaries. Evaluation fails closed on missing or
ambiguous evidence. A successful
canonical result requires all frozen Scenario 6 thresholds, exactly one Go
restart, no systemd supervisor restart, no original process-group survivor, a
non-successful interrupted mission, a successful fresh one-waypoint mission,
zero event loss, exact package/overlay/source binding, and complete owned
cleanup. The retained lifecycle-startup result must also be canonical PASS and
finish before the initial affinity capture.

The before/after source snapshots replay the complete authoritative input tree,
but deliberately exclude generated `docs/results/**`, evidence/build outputs,
and the later-generated `config/release-claims.json` to avoid a result hash
cycle. Runtime-overlay source manifests likewise exclude release claims because
they are not runtime input. Changing an actual runtime config, scenario, or
source file still changes the corresponding manifest and fails the join.

The compositor requires the canonical startup prefix (`supervisor_started`,
original `child_started`, `heartbeat_fresh`, readiness true) and then reconciles
an exact transition chain through the last pre-stop event: one
`unexpected_exit` for the original PID/PGID with its real wait error, readiness
false, one 1000 ms restart schedule, one replacement PID/PGID start, replacement
`heartbeat_fresh`, and readiness true, all in strict monotonic order with no
unrelated or duplicate transition. After the observation window, only the exact
bounded shutdown grammar through `supervisor_stopped` is accepted; no failure,
restart, or child start may appear in that suffix. It also
requires exactly one interrupted-mission completion matching its outcome
ledger and, when interrupted mission artifacts exist, a non-boolean integer
exit code matching both the observed process exit and the sole timeline
completion. Terminal-result presence, goal status code/name, mission status,
and success/failure verdict fields must also agree. Exactly one follow-up
completion with exit code zero must match the canonical mission JSON/CSV.

The Go supervisor never schedules a replacement until the old child leader is
reaped and its entire process group is proven absent. TERM and KILL waits are
bounded. If either proof remains incomplete, the supervisor opens its circuit,
retains the unresolved identity, stays alive but unready, records
`process_group_cleanup_incomplete`, and emits no replacement start.

The cleanup path stops only captured process groups, stops and disables the
service, removes the drop-in only if its checksum is unchanged, and removes the
run state directory only when its exact ownership marker is present and it
contains no symlink. All privileged writes and process output remain in a new,
root-owned `/run/robotest-phase4-verifier/<run-id>` directory. After cleanup,
composition, checksums, and source freezing, a sanitized `runuser` helper copies
only bounded regular non-symlink files to a random user-owned sibling under
`artifacts/evidence/phase4/runs`, verifies every relative byte/hash, fsyncs the
copy, and performs a no-replace atomic rename to the canonical public run path.
Failure evidence is published too. The live `/run` stage is removed only after
publication replay and service inactivity are both proven; otherwise it is
retained and reported. Root never redirects or copies into user-controlled
evidence leaves. The installed candidate and immutable overlay remain in place.
The public run contains canonical `scenario6-result.json`,
`scenario6-result.csv`, and a verified `SHA256SUMS`.

State and drop-in ownership are published through run-unique staging paths and
atomic renames. Harness process-group identities are assigned before handled
signals can trigger finalization; a signal observed in that short publication
window is deferred until the PID/PGID is recoverable. Finalization is
write-once and idempotent, retries every last published harness PGID, and
requires the original and replacement supervisor child PGIDs to be empty.
Runtime state, the drop-in, heartbeat, and startup result are retained whenever
exact systemd inactivity (`ActiveState=inactive`, `MainPID=0`) cannot be
proven, so a cleanup failure cannot erase the resources needed to diagnose it.

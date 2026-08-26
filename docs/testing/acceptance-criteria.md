# RoboTest Lab Acceptance Criteria

Status: **Frozen Phase 3 target set, revision 2**

These values are targets, not results. No measurement in this document is
claimed to have occurred. Revision 2 freezes the Phase 3 fault, metric,
collision, scenario, and repeated-trial semantics before the first Phase 3
candidate suite. Run manifests copy this target-set revision and hash before
execution. Results are recorded elsewhere and are never written back into the
target fields.

The normative calculation details live in the
[metrics contract](../architecture/metrics-contract.md), the deterministic
fault-control protocol lives in
[ADR 0005](../decisions/0005-phase3-deterministic-fault-protocol.md), and the
scenario mechanics live in
[ADR 0006](../decisions/0006-phase3-scenario-mechanics.md). Those files and
their SHA-256 hashes are part of the target set.

Changing a target requires a dated decision record before the affected run.
Changing a threshold after seeing a result invalidates that result for
acceptance.

## Global gates

| ID | Target criterion |
| --- | --- |
| G-01 | Development runs only in WSL2 `Ubuntu` identified as Ubuntu 24.04; `Ubuntu-20.04` remains untouched. |
| G-02 | Dependency plus project high-water projection is at most 18 GiB and projected Windows C: reserve is at least 20 GiB before installation. |
| G-03 | Builds use at most four workers; runtime descendants inherit CPU affinity limited to logical CPUs 0–5. |
| G-04 | The local runtime process-tree peak `rss_sum` is at most 6.0 GiB, with no OOM kill. |
| G-05 | Headless benchmark median real-time factor is at least 0.80 and p5 is at least 0.50, excluding declared pauses. |
| G-06 | No automated test or run has an unbounded wait, queue, log, bag, process restart, or artifact directory. |
| G-07 | All simulation nodes use `/clock`; no participating node silently uses wall time for simulated events. |
| G-08 | No AMCL/Nav2/autonomy endpoint subscribes to `/robotest/validation/*`; no validator publishes autonomy inputs. |
| G-09 | Exactly one publisher owns `odom -> base_footprint`, `map -> odom`, and the final `/robotest/cmd_vel` according to the architecture contract. |
| G-10 | Every accepted run has canonical JSON and matching CSV, command/exit evidence, Git SHA, scenario/target hashes, seeds, versions, and artifact hashes. |

The memory target applies to the project's process-tree RSS sum. WSL-wide
memory is contextual because unrelated workloads may be active. The Phase 1
resource probe may reduce ray count, update rates, or world complexity, but it
may not loosen these targets without a prior decision record.

## Common mission criteria

Unless a scenario below overrides a value:

- benchmark acceptance requires **3 of 3** repeated trials;
- allowed collision count is **0**;
- mission simulation-time timeout is **180 s**;
- steady wall-clock escape timeout is **300 s**;
- required localization-error sample coverage is at least **95%**;
- maximum ground-truth/interpolation gap is **0.25 simulation seconds**;
- the contact gate's active-pair expiry is **0.25 simulation seconds**, with
  release proven only by a completed authoritative snapshot whose stamp is
  strictly greater than the pair's last-seen stamp plus that gap;
- public contact snapshot source gaps are at most **0.22 simulation seconds**
  throughout the accepted interval; the gate measures source cadence from
  causally ordered finalized raw stamps and separately detects pending/raw
  silence;
- passive evidence consumers retain each callback's cached `/clock` offset as
  diagnostic telemetry, not DDS transport age or an acceptance bound, because
  the contact and `/clock` subscriptions have no causal callback order; they
  instead require an explicitly caught-up final snapshot/clock bracket of at
  most **0.22 simulation seconds** within a bounded wall wait; the Phase 1/2
  probes additionally reject a public-contact inter-receipt silence longer
  than their existing **2.0 steady-wall-second** bracket timeout as observer
  liveness evidence, not simulation-time freshness;
- the active positive-control driver separately fails closed when its own
  callback-time offset exceeds **0.22 simulation seconds** and proves the
  qualifying stop command within **0.10 simulation seconds**; no stricter
  consecutive-private-callback spacing is inferred from the simulator's
  delivered raw sequence;
- terminal drain is an actually retained, collector-acknowledged public
  snapshot `q`, with `q > T_terminal + 0.25 s`; the target boundary alone is
  not completion evidence;
- zero-command tolerance is
  `abs(linear.x) <= 0.02 m/s` and
  `abs(angular.z) <= 0.05 rad/s`.

The Phase 3 benchmark candidate is exactly 15 cold-stack trials: Scenarios 1
through 5 in order, with repetition indices 0, 1, and 2. Every trial uses the
same clean commit, the seeds frozen in its scenario file, `retries: 0`, a new
ROS domain, Gazebo partition, process group, and artifact directory, and a
confirmed reset fault state. A failed index remains failed and cannot be
replaced by an unrecorded retry. The canonical Phase 3 run result, rather than
the mission process exit alone, owns the benchmark verdict.

All trials, including failures and timeouts, remain in the denominator.

## Scenario 1 — Baseline navigation

Target configuration:

- fixed original world and map;
- three ordered waypoints;
- fixed mission seed;
- no fault schedule;
- no inserted obstacle beyond the mapped world.

Pass criteria for every trial:

1. Nav2 returns `SUCCEEDED` after all three waypoints.
2. Collision count is 0.
3. Completion time is at most 180 simulation seconds and 300 wall seconds.
4. The cumulative first-valid-per-waypoint-leg planned reference and actual
   ground-truth path are finite and greater than 0.1 m.
5. Path efficiency is at least 0.75.
6. JSON and CSV agree and the process exit code is 0.

Any incomplete waypoint set, cancellation, timeout, missing terminal action
result, missing evidence, or validation-topic leak is failure.

## Scenario 2 — Deterministic static obstacle replan

The inserted entity is `phase3_static_block`, an axis-aligned static box of
size `0.40 x 0.40 x 0.80 m` at world pose
`(-1.00, -3.50, 0.40, yaw 0.0)`. It is absent from the map and world at stack
startup. The controller requires a valid initial plan for waypoint leg 0, then
inserts the obstacle at the first controller opportunity no earlier than
`T0 + 2.0 s`. Missing the plan-before-insertion ordering or an insertion stamp
later than `T0 + 2.25 s` is infrastructure failure, not a valid trial.

Pass criteria for every trial:

1. The obstacle appears after the initial-path timestamp inside the frozen
   insertion window, and the spawn response plus validation world-pose stream
   confirm its exact name, geometry hash, pose, and first-observed stamp.
2. At least one later valid plan for leg 0 has a different canonical geometry
   hash and every segment avoids the obstacle rectangle expanded by the frozen
   `0.35 m` costmap inflation radius.
3. The mission reaches its goal with Nav2 `SUCCEEDED`.
4. Collision count is 0.
5. Completion remains within the common timeouts.
6. Path efficiency is at least 0.60.

Moving the obstacle before the initial path, changing the map to include it, or
counting repeated publication of the same path is failure.

Actor cleanup is source-spanned: after the exact successful delete response,
the permanent 10 Hz `ground_plane` pose heartbeat must advance through a
0.25 s simulation-time quiet interval with no further target-actor pose. A
delete response plus `/clock` advancement without that pose-path heartbeat is
incomplete evidence.

## Scenario 3 — Deterministic dynamic obstacle

The entity `phase3_dynamic_block` is a pose-controlled static box of size
`0.35 x 0.35 x 0.80 m`, centered at `z=0.40 m`. Its trajectory is anchored to
the first valid feedback transition to waypoint index 2 at simulation stamp
`T_leg2`: move at constant speed from `(-0.80, 1.50)` to `(0.00, 1.50)` over
4.0 s, dwell there for 4.0 s, then move to `(0.80, 1.50)` over 4.0 s. Pose
commands occur at 10 Hz simulation time, including exact segment endpoints.
The validation world-pose stream, not request values alone, proves the actual
trajectory and its hash.

Pass criteria for every trial:

1. The mission reaches its goal within the common timeouts.
2. Collision count is 0.
3. While the obstacle occupies the frozen central blocking region, the
   collision monitor reports `STOP` and the final command remains within the
   zero-command tolerance for at least 0.20 continuous simulation seconds.
4. No nonzero final command is published during a collision-monitor `STOP`
   interval. A replan may be recorded as diagnostic evidence but cannot replace
   the required stop proof.
5. Obstacle trajectory hash and actual start/end stamps are present.

A successful goal without recorded obstacle interaction does not exercise the
scenario and is failure.

## Scenario 4 — Temporary LiDAR dropout

Frozen initial fault target:

- activation: 10.0 simulation seconds after accepted mission goal;
- duration: 2.0 simulation seconds;
- raw LiDAR nominal rate: 5 Hz;
- validated-scan stale threshold: 0.60 simulation seconds;
- deterministic seed recorded even though pure dropout uses no random sample.

Pass criteria for every trial:

1. Raw scans continue through the active interval.
2. No validated scan is published in the actual dropout interval.
3. Fault event counts reconcile with raw and validated message counts.
4. The final command reaches zero-command tolerance within 1.0 simulation
   second of the configured sensor-stale threshold and remains safe until a
   restored scan is accepted.
5. Sensor recovery satisfies the metrics contract's complete 1.0-second
   stability window and is declared within 10.0 simulation seconds after
   actual restoration.
6. The mission subsequently succeeds within the common timeouts.
7. Collision count is 0.

The effective Nav2 collision-monitor `source_timeout` must be exactly
`0.60 s`; a different installed or runtime value invalidates the trial.

Publishing empty/stale substitute scans, continuing unsafe motion, or labeling
the run successful before recovery is failure.

## Scenario 5 — Odometry drift

Frozen initial fault target:

- activation: 10.0 simulation seconds after accepted mission goal;
- duration: 20.0 simulation seconds;
- odom-frame x offset rate: 0.010 m/s;
- yaw offset rate: 0.005 rad/s;
- no random component in revision 1.

At the end of the full active interval, the target offsets are 0.200 m in odom
x and 0.100 rad in yaw.

Pass criteria for every trial:

1. Validated odometry and `odom -> base_footprint` carry the same
   deterministic left-composed SE(2) offset and timestamp, where
   `T_validated = D(elapsed) * T_raw` in the odom frame.
2. The last active validated/raw pair is no more than 0.25 s before the
   configured interval end. For `T_validated * inverse(T_raw)`, its x offset is
   within 0.020 m of 0.200 m and yaw offset is within 0.010 rad of 0.100 rad.
3. Raw odometry remains unchanged by the proxy.
4. Localization-error coverage is at least 95%, with RMSE, p95, and maximum
   reported rather than compared to a fabricated baseline.
5. The mission reaches its declared `SUCCEEDED` expected outcome within the
   common timeouts.
6. Collision count is 0.

This scenario passes based on correct injection, evidence, and the declared
mission outcome. It does not require localization error to equal the odometry
offset because AMCL may correct part of it.

## Scenario 6 — Navigation process crash

Frozen supervisor targets:

- heartbeat period: 0.5 wall seconds;
- stale threshold: 2.0 wall seconds;
- backoff: 1, 2, 4, then 8 wall seconds maximum;
- restart attempt limit: 4 within a 60 wall-second window;
- stable-run reset window: 60 wall seconds;
- graceful termination allowance: 5 wall seconds before forced termination.

The test terminates the configured Nav2 or mission child after the stack is
ready and a mission is active.

Pass criteria:

1. `/healthz` remains 200 while the Go supervisor is alive.
2. `/readyz` becomes 503 within 3.0 wall seconds of failure.
3. The original process group has no surviving child after the termination
   allowance.
4. Exactly one restart is scheduled for the single injected crash.
5. Required children and heartbeat return, and `/readyz` returns 200 within
   30.0 wall seconds of failure detection.
6. The interrupted mission is not labeled successful.
7. A fresh short follow-up mission succeeds.
8. Detection time, backoff, start time, ready-restored time, restart count,
   JSON/CSV result, and structured events reconcile.

A false-ready interval, an orphan, an infinite retry, or recovery achieved by a
second systemd-owned ROS process is failure.

## Component acceptance

### Fault proxy

- Pass-through outputs preserve input stamp, frame, finite values, and payload.
- QoS histories are bounded.
- Preload, arm, and reset implement the frozen `RESET -> PREPARED -> ARMED`
  protocol. Preload is atomic and inert; only an exact UUID/T0/hash/generation
  arm can activate a prepared schedule.
- The proxy independently recomputes the canonical schedule SHA-256, accepts at
  most 16 non-overlapping same-target specifications, and rejects unsupported
  modes or parameters without changing prior state.
- Arm completes at least 0.50 simulation seconds before the earliest fault
  activation. Exact duplicate requests follow the frozen idempotency policy;
  conflicting replays fail closed.
- Same input sequence, configuration, and seed produce identical transformed
  outputs and event metadata within floating-point test tolerance.
- Reset returns to pass-through and resets deterministic state.

### Mission runner

- YAML schema rejects unknown fault modes, missing required fields, non-finite
  values, negative durations, invalid retry counts, and unreachable result
  paths.
- Timeout causes cancellation and a nonzero timeout-specific exit.
- Canceled, rejected, aborted, timed-out, and infrastructure-error outcomes
  are distinct.
- Artifact write failure makes the run fail.

### Metrics

- Formula tests cover empty, one-sample, duplicate/non-monotonic timestamp,
  zero-length, angular wrap, contact burst, missing feedback, and alignment-gap
  cases.
- Rendered-SDF collision coverage includes every robot collision geometry, and
  every candidate suite references a passing hash-compatible positive-control
  contact run. A silent contact stream alone cannot establish collision zero.
- Every collector buffer and artifact writer respects the capacities and byte
  limits in the metrics contract; any overflow, truncation, or missing
  terminal contact drain fails closed.
- Reports derive all displayed numbers from canonical run JSON.
- Null/unavailable values never become zero.

### Supervisor

- Child processes run in owned process groups.
- SIGINT/SIGTERM perform graceful group shutdown.
- Backoff and exhaustion use a fake clock in unit tests.
- HTTP binds to `127.0.0.1` only.
- Metrics text is parseable and bounded in cardinality.
- No remote mutation/restart endpoint exists.

### Debian/systemd

- The unit is installed disabled by default.
- It contains the required condition for
  `/opt/robotest-lab/install/setup.bash`.
- `systemd-analyze verify` and `lintian` have no project-caused errors.
- Install, upgrade, remove, and purge tests do not overwrite or delete
  unrelated data.
- Modified conffile behavior is demonstrated.

## Portfolio release criteria

A release tag and completed CV bullets require all of:

- every phase gate passed at its claimed verification level;
- scenarios 1–6 satisfied their frozen targets;
- public standard-runner CI passed on the tagged commit;
- README commands replayed from the clean-environment matrix;
- genuine demonstration media corresponds to a recorded passing run;
- every benchmark table value resolves to a canonical JSON field and run ID;
- no paid service, proprietary dataset, GPU requirement, or fabricated result.

If an external item cannot be produced, the repository may still document local
work, but the full Definition of Done remains explicitly incomplete.

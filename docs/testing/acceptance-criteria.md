# RoboTest Lab Acceptance Criteria

Status: **Frozen Phase 0 target set, revision 1**

These values are targets, not results. No measurement in this document is
claimed to have occurred. Future run manifests copy this target-set revision
and hash before execution. Results are recorded elsewhere and are never written
back into the target fields.

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
- collision release gap is **0.25 simulation seconds**;
- zero-command tolerance is
  `abs(linear.x) <= 0.02 m/s` and
  `abs(angular.z) <= 0.05 rad/s`.

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
4. Initial and actual path lengths are finite and greater than 0.1 m.
5. Path efficiency is at least 0.75.
6. JSON and CSV agree and the process exit code is 0.

Any incomplete waypoint set, cancellation, timeout, missing terminal action
result, missing evidence, or validation-topic leak is failure.

## Scenario 2 — Deterministic static obstacle replan

The obstacle's geometry and insertion pose are known to the test, but it is
inserted only after the initial global path has been recorded. It is therefore
newly observed by autonomy and can prove replanning rather than merely initial
planning around a mapped obstacle.

Pass criteria for every trial:

1. The obstacle appears after the initial-path timestamp at the frozen
   simulation offset.
2. At least one later global path has a different geometry hash and avoids the
   obstacle footprint plus configured inflation.
3. The mission reaches its goal with Nav2 `SUCCEEDED`.
4. Collision count is 0.
5. Completion remains within the common timeouts.
6. Path efficiency is at least 0.60.

Moving the obstacle before the initial path, changing the map to include it, or
counting repeated publication of the same path is failure.

## Scenario 3 — Deterministic dynamic obstacle

One lightweight obstacle follows a versioned deterministic trajectory that
intersects the initially preferred route.

Pass criteria for every trial:

1. The mission reaches its goal within the common timeouts.
2. Collision count is 0.
3. Evidence records at least one safe response:
   - a geometrically changed global/local plan; or
   - a final command within the zero-command tolerance for at least
     0.20 simulation seconds while the obstacle blocks the route.
4. The robot does not command motion through a collision-monitor stop state.
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
5. Sensor recovery is declared within 10.0 simulation seconds after actual
   restoration.
6. The mission subsequently succeeds within the common timeouts.
7. Collision count is 0.

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
   deterministic offset and timestamp.
2. End-of-interval x offset is within 0.020 m of 0.200 m, and yaw offset is
   within 0.010 rad of 0.100 rad.
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
- Schedule loading is atomic and invalid schedules activate nothing.
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

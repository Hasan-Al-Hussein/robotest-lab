# RoboTest Lab Metrics Contract

Status: **Phase 0 normative contract**

This document defines how evidence is calculated. It contains no benchmark
results. Numerical pass targets live in
`docs/testing/acceptance-criteria.md`; future measured values live only in
run artifacts and generated result reports.

## Evidence model

Every run has a unique `run_id` and a machine-readable manifest containing:

- UTC creation time and simulation start/end stamps
- Git commit SHA and dirty-worktree flag
- scenario name, scenario file hash, expected outcome, and repetition index
- mission seed, fault schedule hash, and all effective parameters
- ROS, Gazebo, Nav2, ros_gz, compiler, Python, and Go versions
- RMW implementation, WSL identity, CPU affinity, and build type
- target-set version/hash
- paths and SHA-256 hashes for JSON, CSV, logs, charts, and retained bags
- command, working directory, exit code, and verification level

A dirty worktree may be used during development, but portfolio benchmark runs
must identify it and cannot be called release evidence.

The canonical per-run JSON contains separate top-level objects:

```json
{
  "identity": {},
  "targets": {},
  "measurements": {},
  "events": [],
  "quality": {},
  "verdict": {}
}
```

`targets` is copied from the frozen target set before the run.
`measurements` is populated only from observed messages, process samples, and
terminal results. Target values must never be copied into measurements.

CSV is a flattened one-row-per-run comparison view. Column names include units,
for example `completion_time_sim_s`, `actual_path_length_m`, and
`peak_rss_sum_mib`. Null JSON values become blank CSV fields, never zero.

## Source-of-truth table

| Measurement | Source | Clock |
| --- | --- | --- |
| Mission result and waypoint progress | Nav2 action result/feedback recorded by mission runner | Simulation stamps plus steady wall escape timer |
| Actual path | `/robotest/validation/ground_truth` | Simulation |
| Planned paths | `/robotest/navigation/plan` | Simulation |
| Collisions | `/robotest/validation/contacts` | Simulation |
| Localization estimate | TF `map -> odom -> base_footprint` | Simulation |
| Fault interval and affected messages | `/robotest/faults/events` | Simulation |
| Final command response | `/robotest/cmd_vel` | Simulation |
| Real-time factor | `/robotest/validation/world_stats` | Simulation and steady wall deltas |
| CPU and memory | Linux process-tree sampler | Steady wall clock |
| Nav2 recovery count | Nav2 action feedback | Simulation |
| Supervisor restarts/readiness | Structured Go events and localhost endpoints | Steady wall clock |

Validation data is observational. It does not feed goal submission, planning,
control, localization, or recovery decisions.

## Mission status and time

A mission is `SUCCEEDED` only when:

1. Nav2 returns a succeeded terminal action result;
2. every ordered waypoint is acknowledged complete;
3. no mission timeout or cancellation is active;
4. collision count is within the frozen allowance; and
5. the JSON and CSV artifacts are flushed and cross-validated.

An accepted goal, a moving robot, arrival near only the last waypoint, process
exit zero without a terminal action result, or ground-truth proximity alone is
not success. Incomplete, canceled, timed-out, and infrastructure-error outcomes
remain distinct.

`completion_time_sim_s = terminal_action_stamp - accepted_goal_stamp`

Also store wall duration. Paused simulation time does not increase completion
time, but the wall escape timeout can still end the run as infrastructure
failure.

## Path metrics

### Actual path length

For ordered valid ground-truth planar samples `p_i=(x_i,y_i)`:

`actual_path_length_m = sum(hypot(x_i-x_(i-1), y_i-y_(i-1)))`

The interval begins at accepted goal and ends at the terminal action result.
Duplicate timestamps are rejected. Non-monotonic stamps, non-finite values, or
a sample gap above the versioned alignment limit invalidate the metric rather
than being silently filtered. The raw sample count and maximum gap are stored.

### Planned path lengths

For each received `nav_msgs/msg/Path`, length is the sum of planar distances
between consecutive poses. Store:

- `initial_planned_path_length_m`
- `latest_planned_path_length_m`
- `planned_path_lengths_m[]` with timestamps and path hashes
- `replan_count`, counting a geometrically changed plan after the initial one

Repeated publication of an identical path hash is not a replan.

### Efficiency and overrun

The reference is always the initial valid planned path:

`path_efficiency = initial_planned_path_length_m / actual_path_length_m`

`path_overrun_ratio = actual_path_length_m / initial_planned_path_length_m`

Values are not clamped. They may expose map changes or measurement problems. If
either length is missing or below the configured epsilon, both ratios are null
with a reason.

## Collision events

Gazebo may publish many contact samples during one physical contact. A collision
event begins on the first contact involving a robot collision geometry and ends
after the versioned `contact_release_gap_s` has elapsed with no such contact.

Store event start/end, robot and counterpart collision names, maximum reported
normal force when available, and duration. Contacts between robot-internal
collision geometries are excluded by an explicit allowlist. No event is deleted
because it is short.

`collision_count` is the number of de-duplicated events, not the number of
contact messages.

## Localization error

The estimated pose is TF `map -> base_footprint`. Ground truth is transformed
to `map` by the fixed validation-only `world -> map` alignment.

For each estimate timestamp, ground truth is linearly interpolated between
bracketing samples for x/y and by the shortest angular arc for yaw. If the
bracketing gap exceeds the frozen maximum, that sample is unavailable.

`position_error_m = hypot(x_est-x_truth, y_est-y_truth)`

`yaw_error_rad = abs(normalize_angle(yaw_est-yaw_truth))`

Report sample count, unavailable count, RMSE, median, p95, and maximum for
position and yaw. A run below the frozen coverage ratio is invalid rather than
reported as low error.

## Fault and recovery metrics

Fault events distinguish:

- configured activation/deactivation times;
- actual first-affected and first-restored message times;
- raw input count, validated output count, and affected count.

For LiDAR dropout, raw scans must continue while the validated count is zero in
the actual active interval.

Sensor recovery time is:

`sensor_recovery_time_sim_s = recovered_stamp - actual_deactivation_stamp`

`recovered_stamp` is the first time all of these remain true for the frozen
stability window:

- the restored stream is fresh at its accepted rate;
- required Nav2 lifecycle nodes are active;
- the mission has not failed; and
- navigation progress resumes or the goal succeeds.

Supervisor recovery is separate:

`supervisor_recovery_time_wall_s = ready_restored_wall - failure_detected_wall`

Readiness must be false throughout the unavailable interval. Initial process
start is not a restart. `restart_count` increments only on a structured
`restart_scheduled` event followed by a new child start.

## Nav2 recovery count

Use the action feedback's recovery counter when present. Store the maximum
monotonic value and the feedback source. If the installed interface lacks that
field, store null and a compatibility reason; do not infer the count by parsing
human log text.

## Real-time factor

For adjacent unpaused world-statistics samples:

`rtf_i = delta_sim_time / delta_steady_wall_time`

Exclude declared pauses and intervals with non-positive deltas, recording every
exclusion. Report count, mean, median, p5, and minimum. Gazebo's reported RTF
may also be stored, but the source is named and is not mixed with the calculated
series.

## CPU and memory

The sampler follows the launched process group and its unique descendant PIDs.

`cpu_percent_i = 100 * delta_process_cpu_seconds / delta_steady_wall_seconds`

Thus 100 percent means one fully used logical CPU and the six-CPU affinity has a
theoretical 600 percent ceiling. Report mean, p95, and peak.

`rss_sum_bytes` is the sum of resident-set values for unique PIDs. Shared
pages may be counted in more than one process, so it is deliberately named an
RSS sum rather than physical machine memory. Report peak RSS sum and WSL-wide
memory/swap as contextual observations, not as the project process total.

Sampling starts before stack launch and continues through shutdown. Missing
samples or PID reuse invalidates the corresponding interval.

## Repeated trials and aggregation

- Store every trial, including failures and timeouts.
- Never discard an outlier without preserving it and recording the predeclared
  exclusion rule.
- Scenario acceptance uses the repetition count frozen in the acceptance
  criteria.
- Aggregate reports include numerator/denominator, success rate, median, p95
  where meaningful, and the full run IDs.
- A single CI smoke run proves integration only; it is not benchmark evidence.

## Artifact integrity

Small JSON/CSV results, generated charts, target definitions, and summary
reports may be committed under `benchmarks/` and `docs/results/`. Large
bags and raw logs remain in the ignored run-artifact area; tracked manifests
retain their paths, sizes, and hashes.

Generated Markdown/HTML and PNG files must read exclusively from canonical run
JSON. Hand-entered benchmark numbers are prohibited. A report with a missing
or mismatched source hash fails verification.

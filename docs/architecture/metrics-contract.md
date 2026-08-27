# RoboTest Lab Metrics Contract

Status: **Phase 3 normative contract, revision 2**

This document defines how evidence is calculated. It contains no benchmark
results. Numerical pass targets live in
`docs/testing/acceptance-criteria.md`; future measured values live only in
run artifacts and generated result reports.

Revision 2 freezes the Phase 3 measurement semantics before the first Phase 3
benchmark. In particular, a `FollowWaypoints` mission uses the cumulative
first valid plan for every feedback-derived leg, collision evidence covers the
complete rendered robot collision set and is qualified by a positive-control
run, every retained collector is bounded, and aggregate statistics have one
deterministic definition. The numerical acceptance thresholds are unchanged.

## Evidence model

Every run has a unique `run_id` and a machine-readable manifest containing:

- UTC creation time and simulation start/end stamps
- Git commit SHA and dirty-worktree flag
- scenario name, scenario file hash, expected outcome, repetition index, and
  the ordered benchmark-suite index
- mission seed, fault schedule hash, and all effective parameters
- ROS, Gazebo, Nav2, ros_gz, compiler, Python, and Go versions
- RMW implementation, WSL identity, CPU affinity, and build type
- target-set version/hash and this metrics-contract SHA-256
- collector configuration/hash, capacities, ingress/retained/invalid counts,
  and overflow state for every evidence stream
- rendered collision-coverage manifest/hash and the qualifying
  positive-control run ID/JSON hash
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

`quality` holds coverage, alignment, overflow, provenance, and positive-control
gates. A required measurement with a failed quality gate is null with an
explicit reason; it is never computed from the remaining prefix and presented
as complete.

CSV is a flattened one-row-per-run comparison view. Column names include units,
for example `completion_time_sim_s`, `actual_path_length_m`, and
`peak_rss_sum_mib`. Null JSON values become blank CSV fields, never zero.

## Source-of-truth table

| Measurement | Source | Clock |
| --- | --- | --- |
| Mission result and waypoint progress | UUID-bound `FollowWaypoints` action result and feedback recorded by the mission runner | Simulation stamps plus steady wall escape timer |
| Actual path | `/robotest/validation/ground_truth` | Simulation |
| Planned paths | `/robotest/navigation/plan` | Simulation |
| Collisions | Coverage-qualified `/robotest/validation/contacts` plus its bound positive-control evidence | Simulation |
| Localization estimate | TF `map -> odom -> base_footprint` | Simulation |
| Fault interval and affected messages | `/robotest/faults/events` | Simulation |
| Final command response | `/robotest/cmd_vel` | Simulation |
| Real-time factor | `/robotest/validation/world_stats` | Simulation and steady wall deltas |
| CPU and memory | Linux process-tree sampler | Steady wall clock |
| Nav2 recovery count | Nav2 action feedback | Simulation |
| Supervisor restarts/readiness | Structured Go events and localhost endpoints | Steady wall clock |

Validation data is observational. It does not feed goal submission, planning,
control, localization, or recovery decisions.

## Time, ordering, and bounded collection

Simulation timestamps are stored as signed integer nanoseconds. A callback also
receives a monotonically increasing collector sequence number. The pair
`(sim_stamp_ns, collector_sequence)` provides deterministic ordering when two
callbacks observe the same simulation stamp. Messages with a header use that
header stamp. Headerless action feedback is stamped from the node's ROS clock
at callback entry, while steady-wall nanoseconds are retained separately for
diagnosis. Steady-wall time is used only for wall deadlines and host/process
measurements; it is never substituted for a missing simulation stamp.

The Phase 3 collector uses prefix-retaining buffers with these hard capacities:

| Evidence stream | Retained capacity |
| --- | ---: |
| Ground-truth pose | 8,192 samples |
| Each configured dynamic TF edge | 8,192 samples |
| Raw and validated odometry | 8,192 samples per stream |
| Raw and validated scan | 2,048 samples per stream |
| Final velocity command | 4,096 samples |
| Global plans | 1,024 messages and 65,536 poses total |
| Public contact snapshots | 8,192 snapshot summaries and 32,768 normalized snapshot records total |
| World statistics | 4,096 samples |
| Mission, lifecycle, obstacle, and process state transitions | 1,024 events |
| Fault-control and fault-application events | 512 events |
| `/clock` | Constant-space first/latest/count/gap/regression summary; no raw sample buffer |

Only state **transitions** are stored in the 1,024-event state buffer; repeated
unchanged action or lifecycle observations increment counters without consuming
entries. Every buffer also keeps constant-space ingress, accepted, invalid, and
overflow counters plus the first overflow sequence and stamp. On the first item
beyond a capacity, the collector preserves the existing prefix, increments the
overflow counter, and marks the run failed. It does not overwrite old samples,
silently downsample, grow the buffer, or compute an acceptance metric from the
truncated prefix. Exceeding either the global-plan message limit or the total
plan-pose limit is overflow.

A retained scan sample is bounded metadata (stamp, frame, range count, and
payload hash), not a copy of an unbounded range array. Public contact snapshots
contain 1--16 records and are normalized on receipt; the collector retains the
snapshot stamp, empty frame, callback-time cached clock offset, names, and
delivered snapshot force/depth maxima. The offset is arithmetic and diagnostic:
independently scheduled contact and `/clock` callbacks do not form a transport-
age measurement. Source-stamp gaps and an explicitly caught-up terminal clock
bracket own passive liveness acceptance. The collector does not claim to retain
every raw physics sample or an intermediate peak. Crossing either contact limit
is overflow.
Variable-length strings are UTF-8 validated and capped at
4,096 bytes per field; an over-limit field is invalid evidence and is never
truncated into a misleading value.

Invalid, duplicate where uniqueness is required, or non-monotonic samples are
counted separately from capacity overflow. DDS loss that occurs before the
callback is not described as collector overflow; sequence/source counters when
available, observed rate, maximum gap, and coverage gates expose that distinct
quality failure. Every accepted benchmark requires zero overflow on every
configured evidence stream.

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

The integration interval is the closed simulation-time interval from the
UUID-matched accepted-goal stamp `T0` through the terminal action-result stamp
`T_terminal`. Ground truth must bracket both boundaries. When a boundary falls
between two samples whose gap is within the frozen 0.25 s alignment limit, its
planar position is linearly interpolated and inserted. Extrapolation is
prohibited. A sample exactly on a boundary is used directly.

Let `q_0 ... q_n` be the resulting sequence: the inserted or exact `T0`
position, every original ground-truth `(x, y)` sample strictly inside the
interval, then the inserted or exact `T_terminal` position. The metric is the
observed polyline length:

`actual_path_length_m = sum(hypot(q_i.x-q_(i-1).x, q_i.y-q_(i-1).y))`

No smoothing, resampling, pose-estimate substitution, minimum-motion deadband,
or straight-line shortcut is applied. Duplicate ground-truth timestamps,
non-monotonic stamps, non-finite values, a non-positive interval, a missing
boundary bracket, or any gap above 0.25 s invalidates the metric. Store raw and
in-interval sample counts, the two boundary interpolation records, maximum
gap, and the exact first/last stamps used.

### Planned path lengths

`FollowWaypoints` is a sequence of point-to-point navigation legs. For waypoint
indices `0 ... N-1`, leg 0 begins at `T0`; leg `k>0` begins at the first valid
action-feedback transition to `current_waypoint=k`. A later leg ends at the
next valid transition, and the final leg ends at `T_terminal`. Feedback indices
must start at 0, remain in range, and advance monotonically without skipping an
index. Same-index feedback repetitions do not create a new leg.

A `nav_msgs/msg/Path` is valid only when it:

- arrives in the corresponding feedback-derived leg interval, ordered by
  `(sim_stamp_ns, collector_sequence)`;
- has frame `map`, at least two poses, finite planar coordinates, and a
  length greater than the frozen `path_length_epsilon_m=0.000001`; and
- ends within the frozen 0.05 m plan-to-waypoint matching tolerance of that
  leg's configured waypoint.

Leg 0 may accept a matching plan after `T0` but before its first feedback
sample. A path for a later waypoint received before the corresponding feedback
transition is retained as diagnostic evidence but is not reassigned across the
action boundary. Missing or ambiguous feedback, a feedback regression/skip, or
a leg with no valid matching plan makes the mission-level planned-path metrics
null.

Each valid path length is the sum of planar distances between consecutive raw
finite pose coordinates; quantization is never used for the length. For the
geometry hash only, map each coordinate in metres to a signed integer
micrometre with round-half-away-from-zero:

`q(v) = sign(v) * floor(abs(v) * 1,000,000 + 0.5)`

Treat exact zero as integer zero, require the result to fit signed 64 bits, and
collapse consecutive poses whose quantized `(q(x), q(y))` pair is identical.
The geometry hash is SHA-256 over the ASCII version tag
`robotest-plan-geometry-v1\0`, the UTF-8 frame ID with an unsigned 32-bit
network-byte-order length prefix, the remaining point count as unsigned 64-bit
network byte order, and each x/y pair as signed 64-bit two's-complement network
byte order.
Header stamps, pose stamps, z, and orientation are excluded. Therefore stamp
changes and sub-micrometre jitter do not create a replan, while the metric
length still reflects the raw published path.

For each leg `k`, `leg_initial_plan_length_m[k]` is the first valid plan in
that leg and `leg_latest_plan_length_m[k]` is the last. The mission fields are:

```text
initial_planned_path_length_m = sum(leg_initial_plan_length_m[k])
latest_planned_path_length_m  = sum(leg_latest_plan_length_m[k])
```

The historical field name `initial_planned_path_length_m` therefore means the
**cumulative first-valid-per-leg reference**, not the first global path message
of the whole mission. Store `planned_path_lengths_m[]` with leg index, stamps,
collector sequence, length, endpoint error, and geometry hash.

Within each leg, the first valid plan is never a replan. Thereafter,
`leg_replan_count[k]` increments when a valid plan's geometry hash differs from
the most recently accepted valid plan hash for that same leg. Identical
republication does not increment it. The first plan of every later leg starts a
new comparison chain and is not counted merely because it differs from the
previous leg's plan.

`replan_count = sum(leg_replan_count[k])`

### Efficiency and overrun

The reference is the cumulative first-valid-per-leg planned length:

`path_efficiency = initial_planned_path_length_m / actual_path_length_m`

`path_overrun_ratio = actual_path_length_m / initial_planned_path_length_m`

Values are not clamped. They may expose map changes or measurement problems. If
either length is missing or not greater than `path_length_epsilon_m`, both
ratios are null with a reason.

## Collision events

### Coverage and exclusions

Zero contact snapshots do not by themselves prove zero collisions. Before a
benchmark candidate is accepted, the rendered SDF is inspected and a canonical
coverage manifest lists every collision geometry in the `robotest` model. For
the current model that set includes the chassis, both wheels, both casters,
LiDAR body, and IMU body after fixed-joint lumping/name conversion. The manifest
stores every exact scoped Gazebo collision name, its semantic role, the contact
source that covers it, the rendered-SDF SHA-256, and the source Xacro/world and
bridge hashes. Revision 3 additionally binds the private raw bridge, compiled
gate source/CMake inventory, public gate owner, QoS, batching, heartbeat,
expiry, liveness, and bounded-state policy. Revision-2 raw evidence is not
interpreted as authoritative snapshot evidence.

The `ros_gz_interfaces/Contacts` IDL is itself unbounded. The gate's record,
array, string, and active-state caps apply after DDS has deserialized a callback
from the proven sole private `parameter_bridge` publisher. Exact private-topic
cardinality, endpoint identity, type/QoS, source provenance, and gate survival
therefore form an explicit trusted-ingress boundary; the contract does not
claim bounded memory against an arbitrary hostile DDS writer admitted before
deserialization.

The contact pipeline must observe robot-versus-environment contacts involving
**any** member of that complete set. A chassis-only sensor is insufficient.
Coverage may use one complete contact source or multiple bounded sources, but
the manifest must prove their union equals the rendered robot collision set
with no missing or unknown geometry.

Contact pairs are unordered before classification. Exactly these contacts are
excluded:

1. a pair for which both collisions belong to the `robotest` model
   (robot-internal contact); and
2. the left/right wheel or front/rear caster support collision paired with the
   exact `ground_plane` ground collision.

Wheel or caster contact with a wall, obstacle, or any non-ground counterpart is
counted. Chassis, LiDAR, or IMU contact with the ground is counted. No other
counterpart, duration, depth, or force-based exclusion is permitted. The exact
resolved support-pair allowlist is stored and hashed in the coverage manifest;
substring matching such as `contains("ground")` is prohibited.

### Positive-control qualification

Each benchmark set is bound to a separate, bounded positive-control contact
run made from the same built source and contact configuration. The control run
uses a hash-identified scenario to drive the robot conservatively into one
named test counterpart, observes at least one correctly named non-excluded
snapshot record, publishes a final zero command, and shuts down its owned process
group. The positive-control contact is intentional test evidence and is never
included in a mission's `collision_count`.

The positive-control canonical JSON stores its run ID, scenario hash, rendered
SDF and coverage-manifest hashes, contact configuration/bridge hash, expected
pair, observed normalized pair, event stamps, command trace, collector quality,
and its own SHA-256 sidecar. Every benchmark run stores that run ID and JSON
hash. Qualification fails when the control run did not pass, a referenced hash
does not match, or any source/rendered/configuration hash differs between the
control and benchmark candidates. In those cases `collision_count` is null and
the benchmark fails; a silent mission contact topic cannot be reported as zero.
The external positive binding also stores the exact capture and command-progress
hashes; both must agree with the driver acknowledgement and the final capture.

Positive-control motion is authorized by a stationary, artifact-bound
handshake. The driver completes preparation, atomically writes READY, and keeps
spinning with graph/source checks active but no nonzero motion. The runner must
finish the positive runtime gate, validate its canonical semantic PASS, confirm
that its process group is empty, and recheck that the launch, collector, and
driver are alive. It then atomically writes the exact canonical run-bound ARM
artifact, bound by hash to both READY and runtime-gate evidence. The driver
rejects a missing, stale, oversized, symlinked, malformed, noncanonical,
wrong-run, wrong-producer, or hash-mismatched artifact.

After accepting ARM, the driver records its clock baseline and continues
spinning until it observes a strictly newer positive `/clock` sample. It then
waits for exactly the two frozen `/robotest/cmd_vel` subscriptions, publishes
one untracked safe-zero delivery probe, and waits for the metrics collector's
canonical `command-progress.json`. That immutable progress document is written
only for the collector's first retained command and has exact schema/producer,
run ID, public topic, retained count `1`, positive callback simulation and
steady stamps, and a three-axis zero vector. The driver rejects a probe unless
the collector callback's simulation stamp is within an absolute 100 ms of the
publish-side simulation stamp. That publish stamp must be strictly newer than
the arm-observed clock baseline and no later than the clock stamp stored in
ARMED.

The driver then atomically writes its ARMED acknowledgement, bound to ARM,
READY, the runtime gate, and the exact SHA-256 of `command-progress.json`.
Steady evidence proves both partial orders
`arm_observed <= match <= publish_start <= publish_return <= armed` and
`publish_start <= collector_observed <= armed`; publisher return and collector
callback are deliberately not ordered against one another. Both reported
subscription counts are exactly two. Only that completed acknowledgement
authorizes the first nonzero publication. A fail-safe zero may be published
during pre-arm failure cleanup, but never authorizes motion. Missing evidence
or an identity, cardinality, order, simulation-bracket, or hash violation fails
closed.

The complete positive-control fixture, including preparation, runtime gate,
ARM wait, fresh-clock wait, motion, release, and cleanup, shares one 30 s
steady-wall deadline. READY, ARM, and ARMED never start or reset a deadline.

Actuator-facing `cmd_vel` endpoints remain RELIABLE, VOLATILE, and
KEEP_LAST(1). The independent metrics observer alone uses a bounded
KEEP_LAST(4096) reader history equal to its retained command capacity, so a
short single-thread callback backlog cannot overwrite admissible evidence or
queue stale commands to the actuator. Qualification requires a distinct first
retained zero observation that exactly matches `command-progress.json`,
followed by a distinct collector observation for every component publication
within 100 ms. The captured command count is exactly the component trace count
plus one: reconciliation cannot reuse the delivery probe as a component match
or admit unmatched leading, interleaved, or trailing observations. This history
exception does not relax the latency or
completeness rule. The active stop command must likewise be issued within
100 ms of the qualifying contact.

The component and metrics collector are independent observers of the public
snapshot stream. Their retained traces are reconciled bijectively from the
component's first snapshot through the exact qualified release snapshot `q`:
strict snapshot stamps and the sorted multiset of every normalized pair must
match at every stamp. This includes support, robot-internal, and counted records
and preserves duplicate multiplicity. Collector prefix data before the component
subscribes is outside the join, and suffix data in either retained trace after
`q` is truncated from the bijection. The full component suffix remains
validated and any post-`q` countable recontact fails the positive criterion;
support or robot-internal suffix snapshots are permitted while `/clock` catches
up. Independent callback-clock stamps are validated separately rather than
compared for equality.

### Event de-duplication

The stock per-sensor Gazebo Contact publisher is not part of the accepted
pipeline. A source-bound system observes all seven contact components after
every physics step and emits one bounded aggregate at each 20 ms boundary. Pair
membership is the union over that interval; each pair carries the complete
latest physics-step record group observed for that pair, in source order. A
one-step transient therefore survives to the boundary without retaining a pair
into the next interval. Repeated physics samples are intentionally reduced and
are not claimed as an exact pre-aggregate sample stream or as raw force/depth
peaks.

Binding discovery is exhaustive until exactly one top-level `robotest` model
and the seven unique sensor/link/collision mappings are locked and all seven
`ContactSensorData` components exist. Thereafter every completed, unpaused
physics step retains an O(7) direct check of the locked entity types, names,
parents, and contact-data presence before reading all seven payloads. A
separate cache-scoped state check covers every existing Model, ContactSensor,
Link, and Collision entity's primary, `Name`, and `ParentEntity` component
without a global entity traversal. Any new entity, entity marked for removal,
removed component, or relevant cached structural state change triggers the
same exhaustive inventory/SDF scan before observation. On one-time activity,
conditional typed scans over Model, ContactSensor, Link, and Collision check
each entity's primary, `Name`, and `ParentEntity` state, including structural
components added to an existing entity that was not previously in a typed
cache. Unrelated Pose and physics-rate payload changes do not. Ambiguity,
absence, rebinding, or structural drift remains fatal. This
event-filtered validation changes neither
the 500 Hz physics-step observation nor the 50 Hz aggregate contract; see
[ADR 0008](../decisions/0008-phase3-contact-aggregation-performance.md).
Runtime component writers must use Gazebo ECM change state for configuration
mutations; an unsignalled in-place SDF edit is outside that writer contract.

Each already-validated step map updates the interval map in place, after which
the complete union is revalidated. An over-limit union latches policy fatal
state before publication, returns no output, and returns the same fatal result
on later observations. Policy reset clears the possibly partial interval,
stamps, stream state, and fatal detail. The deployed system additionally
latches fatal and emits Stop; its Gazebo Reset callback does not clear that
system latch, so a live failure requires a clean stack restart. Removing the
former full-map copy does not authorize an invalid partial aggregate.

The gate consumes one private aggregate per stamp and publishes a strictly
increasing, complete snapshot of its delivered active-pair state. A
collision episode begins when a counterpart is present in an authoritative
snapshot and ends at the first later authoritative snapshot where that
counterpart is absent. The gate omits a pair only when a completed absent stamp
is strictly greater than its last delivered raw sighting plus 0.25 s; equality
remains continuous. Offline analysis never infers release from elapsed clock
time. Pair migration for one counterpart remains one episode; simultaneous
distinct counterpart models remain distinct episodes.

The sole producer emits on an exact 20 ms grid after its first nonempty
interval. The gate rejects a delivered private aggregate gap greater than
20 ms, in addition to regression, duplicate, malformed, overflow, causal
public-gap, and pending/raw `/clock` liveness checks. This fixed-grid source
sequence turns a missing aggregate into an explicit fail-closed transport
violation.

Snapshot records are grouped in canonical normalized-pair order. Duplicate
records within one pair retain the selected latest physics-step group's record
order. Each selected record is an exact copy of its bounded ROS `Contact`
projection. Protobuf-only Contact/world metadata, Entity and Vector3 headers,
inner Wrench headers and force offsets, and unknown fields are canonicalized
away before interval storage. The admitted outer JointWrench stamp and
`frame_id` plus every other ROS-visible field remain field-exact. The Gazebo
aggregate container and per-Contact headers are rewritten to the interval
boundary; the ROS Contact type does not expose the latter header.

Contact collection remains active until it retains an actual authoritative
snapshot with stamp strictly greater than `T_terminal + 0.25 s`. The exact
qualifying stamp is collector-acknowledged and present in the final capture;
the target timestamp alone is never fabricated as completion. Only episodes
with a first snapshot stamp in `[T0, T_terminal]` contribute
to the mission count; later-stamped contacts are retained as post-terminal
diagnostics. Failure to advance and complete the full drain, a stamp regression,
or collector overflow invalidates `collision_count` rather than treating the
undrained stream as quiet.

Store event start/end, counterpart model, every robot/counterpart collision
pair seen, sampled snapshot-record count, maximum delivered-snapshot
penetration depth, maximum delivered-snapshot reported normal force when
available, and duration. These maxima do not claim unseen raw-physics peaks.
No counted event is deleted because it is short or low force.

`collision_count` is the number of de-duplicated counterpart episodes, not the
number of public snapshots, contact points, or collision pairs.

## Localization error

The validation-only `world -> map` SE(2) alignment is frozen to the identity
transform: translation `(0.0 m, 0.0 m)` and yaw `0.0 rad`. It is stored in the
scenario targets and source hash. It may not be fitted, learned, shifted to the
first pose, or changed after observing a trajectory. Ground-truth poses in
`world` therefore map to the same x/y/yaw values in `map`.

Ground-truth stamps, not independent TF callback times, define the localization
sample grid. For every valid ground-truth sample in `[T0, T_terminal]`, query
TF at that exact simulation stamp and compose `map -> odom` with
`odom -> base_footprint` to obtain the estimated `map -> base_footprint` pose.
Using the latest transform, receipt-time pairing, or nearest-neighbour TF is
prohibited. tf2 interpolation between bracketing transforms is permitted only
when both required edges are available without extrapolation and each bracket
gap is at most the frozen 0.25 s alignment limit. Otherwise that ground-truth
stamp is unavailable.

`position_error_m = hypot(x_est-x_truth, y_est-y_truth)`

`yaw_error_rad = abs(normalize_angle(yaw_est-yaw_truth))`

Report sample count, unavailable count, RMSE, median, p95, and maximum for
position and yaw. Coverage is available aligned samples divided by eligible
valid ground-truth samples in the mission interval. Store each TF edge's
bracket stamps/gaps and lookup reason. A run below the frozen 95% coverage
ratio is invalid rather than reported as low error.

## Fault and recovery metrics

Fault events distinguish:

- configured activation/deactivation times;
- actual first-affected, last-affected, and first-restored message times;
- raw input count, validated output count, and affected count.

For LiDAR dropout, raw scans must continue while the validated count is zero in
the actual active interval. The proxy's half-open interval and binding event
are governed by
[ADR 0005](../decisions/0005-phase3-deterministic-fault-protocol.md). For
metrics, `actual_deactivation_stamp` is the stamp of the first raw scan at or
after the configured interval end that is published unchanged on the validated
stream. The raw/validated stamps and proxy restoration event must reconcile.

Sensor recovery time is:

`sensor_recovery_time_sim_s = recovered_stamp - actual_deactivation_stamp`

The Phase 3 stability window is 1.0 simulation second. Evaluate every candidate
closed window `[w_start, w_end]` with `w_start >= actual_deactivation_stamp`
in deterministic timestamp/collector-sequence order. The first `w_end` for
which every rule below passes is `recovered_stamp`:

1. A restored validated scan exists at or before `w_start`, no scan stamp
   regresses, every adjacent restored-scan gap intersecting the window is at
   most 0.40 simulation seconds, and the most recent scan at `w_end` is no more
   than 0.40 s old.
2. Every required Nav2 lifecycle node has an `active` snapshot no more than
   0.20 simulation seconds before `w_start`, another `active` snapshot at or no
   more than 0.20 s after `w_end`, and no observed transition away from
   `active` between them. Missing, unknown, stale, or overflowed lifecycle
   evidence fails the candidate window.
3. The bound mission has no rejected, canceled, aborted, timed-out, or
   infrastructure-error terminal state, and no disallowed collision begins in
   the window.
4. Navigation progress is demonstrated within the same window by at least one
   of: ground-truth planar displacement of 0.05 m or more; absolute
   shortest-arc yaw change of 0.10 rad or more; a monotonic
   `current_waypoint` advance; or the bound action reaching `SUCCEEDED`.

Ground-truth displacement is measured between the boundary-interpolated poses
at `w_start` and `w_end`; it is not accumulated odometry. A candidate window
is reset by a stale scan, inactive/unknown lifecycle sample, failed mission,
collision, non-monotonic evidence, or data-quality failure. The JSON stores
the window bounds, scan count/maximum gap, lifecycle samples, chosen progress
predicate and value, and all rejected candidate reasons. Recovery is null when
no qualifying window exists; a single restored message is never recovery.

Supervisor recovery is separate:

`supervisor_recovery_time_wall_s = ready_restored_wall - failure_detected_wall`

Readiness must be false throughout the unavailable interval. Initial process
start is not a restart. `restart_count` increments only on a structured
`restart_scheduled` event followed by a new child start.

## Nav2 recovery count

Use the action feedback's recovery counter when present. Store the maximum
monotonic value and the feedback source. If the installed interface lacks that
field, store null and a compatibility reason; do not infer the count by parsing
human log text. The installed Jazzy `FollowWaypoints` feedback exposes only
`current_waypoint`, so a Phase 3 run using that interface records null unless a
separately versioned, structured Nav2 recovery source is added and named.

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

The Phase 3 candidate suite is exactly 15 cold-stack trials: Scenarios 1
through 5 in numerical order, with repetition indices 0, 1, and 2 for each
scenario before advancing to the next scenario. All 15 use the same clean Git
commit and dirty-worktree flag `false`. Each trial starts a new Gazebo/ROS/Nav2,
fault-proxy, mission, and metrics process group with a new run ID, ROS domain,
Gazebo partition, reset simulation/fault state, and empty run-artifact
directory. The previous process group and discovery endpoints must be proven
gone first. Reusing a warm simulator, proxy generation, localization state, or
collector is prohibited; rebuilding unchanged sources between trials is not
required.

The scenario file supplies every seed. The repetition index never silently
changes a seed; a multi-seed policy must be explicit in, and hashed with, the
frozen scenario. A failed, timed-out, invalid, or infrastructure-error trial
remains at its ordered index. It cannot be replaced by a fourth attempt. After
a code, configuration, target, or source-bound evidence change, a new candidate
suite uses 15 new run IDs rather than mixing old and new trials.

The Phase 3 canonical `run-result.json` is the sole per-trial benchmark
verdict. Mission, fault, probe, resource, and positive-control files are hashed
inputs to that result, not independent PASS authorities. The process exit code,
flattened CSV, and report must agree with `run-result.json`; any disagreement or
artifact-finalization failure makes the trial fail. Aggregate reports consume
only those 15 canonical run results.

An aggregate group requires identical scenario, target-set, metrics-contract,
collector, source/configuration, collision-coverage, and positive-control
hashes wherever those fields are expected to match. Preserve ordered run IDs
and per-run verdicts. Report success numerator, frozen denominator, success
rate, valid numeric count, null count and reasons, and every source value.
Failures and nulls remain in the acceptance denominator even though a numeric
statistic can only use finite available values. Scenario acceptance remains
3 of 3; no statistical summary can turn a failed trial into a pass.

For finite values sorted ascending as `x_1 ... x_n`, the median is `x_((n+1)/2)`
for odd `n` and `(x_(n/2) + x_(n/2+1))/2` for even `n`. Every reported
percentile uses nearest rank: `p_r = x_(ceil(r*n))`, with one-based indexing;
therefore p95 is `x_(ceil(0.95*n))` and p5 is `x_(ceil(0.05*n))`. Empty inputs
produce null, never zero. No interpolation, library-default percentile method,
outlier trimming, winsorization, or post-observation exclusion is allowed.

A single CI smoke run proves integration only; it is not benchmark evidence.

## Artifact byte limits

Artifact production is bounded in addition to the in-memory collector:

| Artifact scope | Hard cap |
| --- | ---: |
| One canonical per-run JSON | 32 MiB (33,554,432 bytes) |
| One per-run flattened CSV | 1 MiB (1,048,576 bytes) |
| One captured stdout/stderr log | 8 MiB (8,388,608 bytes) |
| One PNG chart | 4 MiB (4,194,304 bytes), at most 8 charts per run |
| Complete per-run artifact directory, including any bounded bag | 256 MiB (268,435,456 bytes) |
| Complete 15-trial aggregate/report directory | 64 MiB (67,108,864 bytes) |

Record actual byte counts and cap configuration in the manifest. Writers must
use bounded capture/rotation or recorder size limits; discovering the cap only
after unbounded production is not compliance. Exceeding a cap sets an explicit
quality failure, stops further optional artifact generation safely, and makes
the run or aggregate fail. Required canonical JSON/CSV may not be truncated.
Generated Markdown/HTML lives within its enclosing directory cap and may not
embed raw bags or unbounded data URIs.

## Artifact integrity

Small JSON/CSV results, generated charts, target definitions, and summary
reports may be committed under `benchmarks/` and `docs/results/`. Large
bags and raw logs remain in the ignored run-artifact area; tracked manifests
retain their paths, sizes, and hashes.

Generated Markdown/HTML and PNG files must read exclusively from canonical run
JSON. Hand-entered benchmark numbers are prohibited. A report with a missing
or mismatched source hash fails verification.

# ADR 0006: Phase 3 Scenario Mechanics and Trial Independence

- Status: Accepted for target-set revision 2; Phase 3 route coordinates superseded by ADR 0007
- Date: 2026-08-26
- Decision owners: RoboTest Lab scenario, metrics, safety, and benchmark gate

## Context

The [acceptance criteria](../testing/acceptance-criteria.md) freeze five Phase 3
benchmark scenarios, and the [metrics contract](../architecture/metrics-contract.md)
defines their calculations and evidence quality. The scenario controller still
needs one deterministic contract for actor geometry, event anchors, control
samples, fault schedules, cleanup, repeated trials, and result ownership.
Without it, two individually plausible implementations could exercise different
systems while reporting the same scenario name.

This ADR supplies that contract. It is a target definition, not a claim that a
Scenario 1-5 run has passed. Fault control remains governed by
[ADR 0005](0005-phase3-deterministic-fault-protocol.md), including the
preload-before-goal and UUID-bound arm ordering introduced by
[ADR 0003](0003-two-phase-fault-schedule-arming.md). Command ownership remains
governed by [ADR 0004](0004-phase2-command-ownership.md), and validation data
remains observational under the
[topic and TF contract](../architecture/topic-and-tf-contract.md).

The route coordinates below remain the historical revision-2 decision.
[ADR 0007](0007-phase3-northbound-lane.md) supersedes only waypoint 1 and
waypoint 2 for Phase 3 target-set revision 3; it does not rewrite evidence
produced under this decision.

## Frozen source frame

All five scenarios use the existing `robotest_lab` world, map, robot, and
three-waypoint mission. The frozen coordinate interpretation is:

- Gazebo `world` and Nav2 `map` are aligned by the identity SE(2) transform;
- x/y values below are metres, z values are metres above the ground plane, and
  yaw is radians;
- the mission starts at `(0.0, -3.5, 0.0)` in `map`;
- the ordered goals are `(-2.0, -3.5, 0.0)`, `(0.0, 0.0, 0.0)`, and
  `(0.0, 3.5, 0.0)`; and
- `T0` means only the positive immutable `GoalInfo.stamp` from the
  UUID-matched `FollowWaypoints` status entry. A client submission stamp,
  SendGoal response stamp, or receipt timestamp is not `T0`.

The coordinate audit was performed against these exact inputs:

| Input | SHA-256 |
| --- | --- |
| [World SDF](../../src/robotest_sim/worlds/robotest_lab.sdf) | `cb800041e2a47baaefa74d4c0bd82bcd1c09c67a315351e11ef4fe8231a493e6` |
| [Map YAML](../../src/robotest_navigation/maps/robotest_lab.yaml) | `ed23b53a3677e6ed738e056bf0689973a39c548a6e957b9b8cce591f72dce37f` |
| [Map PGM](../../src/robotest_navigation/maps/robotest_lab.pgm) | `ac1ad5de0b370c9a18f97e34c4334b1e276fdd85eb848febfcfda5c1f6fd8bd9` |
| [Baseline mission](../../scenarios/phase2_baseline.yaml) | `892a06adffaa1e731d2c07d2887dedfbe77490bc23704c32999466114aea9d12` |

The generated map is `248 x 248` at `0.05 m` resolution with origin
`(-6.2, -6.2, 0.0)`. The candidate suite records and checks these hashes
before and after execution. A later source change requires a new decision or a
new frozen target-set revision before benchmark use.

## Common scenario configuration

Every Scenario 1-5 file normalizes to the following common values:

```yaml
schema_version: 1
mission_seed: 42
simulator_seed: 42
scenario_controller_seed: 42
frame_id: map
start_pose: {x: 0.0, y: -3.5, yaw: 0.0}
waypoints:
  - {x: -2.0, y: -3.5, yaw: 0.0}
  - {x: 0.0, y: 0.0, yaw: 0.0}
  - {x: 0.0, y: 3.5, yaw: 0.0}
mission_timeout_sim_s: 180.0
wall_escape_timeout_s: 300.0
allowed_collision_count: 0
expected_outcome: succeeded
retries: 0
```

The scenario-controller seed is recorded even where the controller has no
random branch. Empty schedules use `fault_seed: null`; active faults use seed
`42`. Any additional configurable stochastic participant must also be set to
`42` and named in the manifest. A component without a seed interface is
recorded as unavailable rather than silently described as seeded.

Every simulation-time wait also has a monotonic steady-wall deadline and
watches the owned stack process. A stopped clock, dead stack, missed event
anchor, late control action, trace overflow, or unavailable terminal evidence
fails closed. It does not cause a retry.

## Scenario 1: baseline navigation

Scenario 1 is the common configuration without a scenario actor or active
fault:

```yaml
scenario_id: 1
scenario_name: baseline_navigation
actor: null
fault_schedule:
  schema_version: 1
  faults: []
fault_schedule_sha256: 26080d7dc8f4108a369962ecad1d2e29941a991af68beb415067eae1dc1de6f8
fault_seed: null
```

It uses the unmodified world and map. It must prove three ordered completed
waypoints, zero qualified collisions, the common timeouts, and path efficiency
of at least `0.75`. The planned reference is the cumulative first valid plan
for each feedback-derived leg; it is not the first plan of the whole mission.
Configured waypoint evidence is accepted only in the mission schema's exact
`{x, y, yaw}` shape with finite JSON numbers. The analyzer converts `x` and `y`
once to its internal metre-suffixed pose representation; configured
`{x_m, y_m}` values, numeric strings, booleans, and extra fields fail closed.

## Scenario 2: static obstacle replan

Scenario 2 inserts one entity absent from both the startup world and static
map:

```yaml
scenario_id: 2
scenario_name: deterministic_static_obstacle_replan
entity:
  name: phase3_static_block
  static: true
  geometry: {type: box, size_x_m: 0.40, size_y_m: 0.40, size_z_m: 0.80}
  world_pose: {x_m: -1.0, y_m: -3.5, z_m: 0.4, yaw_rad: 0.0}
trigger:
  anchor: accepted_goal_t0
  target_offset_sim_s: 2.0
  latest_first_observed_offset_sim_s: 2.25
  required_leg_index: 0
fault_schedule:
  schema_version: 1
  faults: []
fault_schedule_sha256: 26080d7dc8f4108a369962ecad1d2e29941a991af68beb415067eae1dc1de6f8
fault_seed: null
```

The actor's axis-aligned footprint is `x=[-1.20, -0.80]` and
`y=[-3.70, -3.30]`. All 64 map-cell centres under that footprint are free in
the frozen PGM. Its acceptance exclusion rectangle includes the frozen
`0.35 m` costmap inflation radius and is therefore
`x=[-1.55, -0.45]`, `y=[-4.05, -2.95]`.

### Ordering and insertion

Before `T0 + 2.0 s`, the controller must retain the first valid leg-0 plan. A
valid plan has frame `map`, at least two finite poses, length greater than
`0.000001 m`, and an endpoint within `0.05 m` of waypoint 0. The plan's
simulation stamp, collector sequence, canonical geometry hash, and length are
recorded.

At the first controller observation of simulation time at or after
`T0 + 2.0 s`:

1. absence of that valid initial plan is infrastructure failure; the actor is
   not inserted and the goal is boundedly canceled;
2. the validation ground-truth centre of the robot must be at least `0.70 m`
   from `(-1.0, -3.5)`; otherwise insertion is unsafe and the trial fails
   without spawning; and
3. if both gates pass, exactly one bounded `SpawnEntity` request inserts
   `phase3_static_block`. A rejected, timed-out, or ambiguous response fails
   the trial and is never hidden by a second request.

The `0.70 m` guard is conservative for the current `0.48 x 0.42 m` navigation
footprint and padding: robot circumscribed radius, padding, obstacle
half-diagonal, and `0.05 m` margin total less than `0.70 m`. It protects
against a faster-than-expected run reaching the target before insertion while
preserving the required coordinate.

The controller records requested stamp, service call/response stamps, response,
and the first independently observed Gazebo world pose. The initial plan stamp
must precede the insertion request, the request may not precede
`T0 + 2.0 s`, and the entity must first be observed no later than
`T0 + 2.25 s`. The observed name, geometry hash, position within `0.01 m`,
and yaw within `0.01 rad` must match the target.

At least one later valid leg-0 plan must have a different canonical geometry
hash. Every segment of the accepted replanned geometry must avoid the expanded
rectangle. Repeated publication or timestamp-only changes do not constitute a
replan. The mission must succeed with zero collisions and efficiency at least
`0.60`.

## Scenario 3: pose-controlled crossing obstacle

Scenario 3 preloads one static entity before goal submission:

```yaml
scenario_id: 3
scenario_name: deterministic_dynamic_obstacle
entity:
  name: phase3_dynamic_block
  static: true
  pose_controlled: true
  geometry: {type: box, size_x_m: 0.35, size_y_m: 0.35, size_z_m: 0.80}
  initial_world_pose: {x_m: -0.8, y_m: 1.5, z_m: 0.4, yaw_rad: 0.0}
trigger:
  anchor: first_feedback_transition_to_waypoint_index_2
control_rate_hz: 10.0
move_1_duration_sim_s: 4.0
dwell_duration_sim_s: 4.0
move_2_duration_sim_s: 4.0
fault_schedule:
  schema_version: 1
  faults: []
fault_schedule_sha256: 26080d7dc8f4108a369962ecad1d2e29941a991af68beb415067eae1dc1de6f8
fault_seed: null
```

The world divider has west bounds `x=[-6.0, -0.6]` and east bounds
`x=[0.6, 6.0]`, both with `y=[1.42, 1.58]`. The requested parked actor at
`x=-0.8` spans `[-0.975, -0.625]` and is deliberately embedded inside the
west static divider; the final actor at `x=0.8` spans `[0.625, 0.975]` and is
embedded inside the east static divider. This is not accidental free-space
placement. Because both the divider and actor are static, the parking overlap
does not create a dynamic contact impulse, keeps the actor inaccessible to the
robot, and hides it within already occupied map geometry. An implementation
that makes the actor dynamic, lets the parked actor respond to physics, or
changes either endpoint is a different scenario.

The open passage is `x=(-0.6, 0.6)`. Accounting for the actor half-width, its
actual centre is in the exposed crossing region when
`x in (-0.425, 0.425)`. At `x=0` the actor occupies the centre of the
`1.2 m` passage and, with the frozen inflation configuration, exercises the
command safety chain.

### Feedback anchor and trajectory

Let `T_leg2` be the simulation stamp of the first valid feedback transition
from waypoint index 1 to index 2 for the accepted goal UUID. Ordered feedback
covering indices 0, 1, and 2 is required. A missing, skipped, regressed,
wrong-UUID, non-positive, or overflowed transition fails the scenario without
moving the actor from its parked pose.

For `u = t - T_leg2` in seconds, the commanded world pose is:

```text
x(u) = -0.8 + 0.2*u       for 0.0 <= u <= 4.0
x(u) =  0.0               for 4.0 <  u <= 8.0
x(u) =  0.2*(u - 8.0)     for 8.0 <  u <= 12.0
y(u) =  1.5
z(u) =  0.4
yaw(u) = 0.0
```

The controller issues exactly 121 target poses at
`t_n = T_leg2 + n * 0.1 s` for integer `n=0..120`. Thus both segment
endpoints and both dwell boundaries are explicit samples: `n=0` is
`x=-0.8`, `n=40` and `n=80` are `x=0`, and `n=120` is `x=0.8`. Each
target is applied once through a bounded `SetEntityPose` transaction. Missing
a target, applying targets out of order, using wall time, or retrying an
ambiguous transaction fails the trial. The full trajectory must finish before
the action terminal stamp.

### Observed-pose proof

Requested poses alone are not evidence. A validation-only Gazebo world-pose
stream must observe the exact entity name and supply simulation stamps. The
scenario collector retains:

- every target index, target stamp, request/response outcome, and target pose;
- the first observed pose at or after each target stamp;
- position and shortest-arc yaw errors;
- early-motion, missing-sample, late-sample, invalid-value, and overflow
  counters; and
- canonical target-trajectory and observed-trajectory SHA-256 values.

The actor must remain within `0.01 m` and `0.01 rad` of the parked target
before `T_leg2`. Each target must have a corresponding observed pose no more
than `0.20` simulation seconds later, within `0.02 m` position and
`0.01 rad` yaw error. Observed x must be monotonic nondecreasing during each
move, constant within the position tolerance during the dwell, and finish at
the commanded endpoint. Any missing observation or trace overflow invalidates
the interaction.

The collision-monitor `STOP` and final-command evidence is evaluated over the
observed, not requested, exposed crossing interval. There must be at least
`0.20` continuous simulation seconds of final zero command while `STOP` is
active, and no nonzero final command may appear in a `STOP` interval. The goal
must still succeed with zero qualified collisions.

## Scenario 4: temporary LiDAR dropout

Scenario 4 uses no actor. Its single fault is exactly the ADR 0005 known-answer
schedule:

```yaml
scenario_id: 4
scenario_name: temporary_lidar_dropout
actor: null
fault_seed: 42
fault_schedule_sha256: 5f93838ca7c0be214858ffe8f62fa351b82d260f6223f139dfaf6bd3dcd224c8
fault:
  schema_version: 1
  fault_id: scan_drop
  target: 1
  mode: 1
  start_offset_ns: 10000000000
  duration_ns: 2000000000
  seed: 42
  parameters: {}
```

The schedule is preloaded before the goal and armed to the accepted UUID and
`T0` with at least `0.50 s` measured margin. Its half-open active interval is
`[T0 + 10.0 s, T0 + 12.0 s)`. Raw scan remains nominal at `5 Hz` while
validated scan is suppressed; empty or stale substitutes are prohibited. The
effective collision-monitor `source_timeout` is exactly `0.60 s`.

Final command must enter the common zero tolerance within `1.0 s` after the
`0.60 s` stale threshold and remain safe until a restored scan is accepted.
Recovery is not a single message: it is the first complete `1.0 s` stable
window defined in the metrics contract and must be declared within `10.0 s`
after actual restoration. The same action must then succeed. No process
restart, mission retry, schedule reload, or hidden re-arm is allowed.

## Scenario 5: odometry drift

Scenario 5 uses no actor. Its single fault is the other ADR 0005 known-answer
schedule:

```yaml
scenario_id: 5
scenario_name: deterministic_odometry_drift
actor: null
fault_seed: 42
fault_schedule_sha256: 3c72bc48e05221a52058112e58c94b578550deb2ee1ff4ad0d719675917335eb
fault:
  schema_version: 1
  fault_id: odom_drift
  target: 2
  mode: 4
  start_offset_ns: 10000000000
  duration_ns: 20000000000
  seed: 42
  parameters:
    x_rate_nm_per_s: 10000000
    yaw_rate_nrad_per_s: 5000000
```

The armed half-open interval is `[T0 + 10.0 s, T0 + 30.0 s)`. The proxy
left-composes the deterministic SE(2) offset in `odom`:

```text
T_validated = D(elapsed) * T_raw
dx = 0.010 * elapsed metres
dtheta = 0.005 * elapsed radians
```

Raw odometry is unchanged. Validated odometry and
`odom -> base_footprint` use the same transformed pose and raw stamp. The final
active sample must be no more than `0.25 s` before the configured end. The
full-duration targets are `0.200 m` x offset and `0.100 rad` yaw, with
tolerances `0.020 m` and `0.010 rad`. These are measured as
`T_validated * inverse(T_raw)`; they are not copied from the target into the
result. The action must succeed with zero qualified collisions.

## Scenario actor interfaces and isolation

The future scenario bridge/controller resolves relative names under the run
namespace and exposes the following ROS interfaces:

| Relative name | Type | Purpose |
| --- | --- | --- |
| `scenario/spawn_entity` | `ros_gz_interfaces/srv/SpawnEntity` | One-shot Scenario 2 insertion and pre-goal actor creation |
| `scenario/set_entity_pose` | `ros_gz_interfaces/srv/SetEntityPose` | Scenario 3 pose control |
| `scenario/delete_entity` | `ros_gz_interfaces/srv/DeleteEntity` | Bounded cleanup |
| `validation/scenario_entity_poses` | `tf2_msgs/msg/TFMessage` | Observed Gazebo world poses only |

The scenario controller calls service names relative to its namespace; it does
not hard-code `/robotest` internally. The validation pose stream is RELIABLE,
VOLATILE, KEEP_LAST(10), contains observed Gazebo state rather than echoed
targets, and must have no subscriber in Nav2, AMCL, the mission runner, the
fault proxy data path, or the command chain. It is never published on `/tf` or
`/tf_static`. A permanent 10 Hz `ground_plane` PosePublisher is merged into the
same stream. After an actor's successful delete response, that heartbeat must
span the full 0.25 s simulation-time quiet interval while no target-actor pose
is observed; `/clock` alone is not accepted as proof that the pose path stayed
live.

Spawn/set/delete availability and response waits are steady-wall bounded.
Entity names are unique within a trial, exact, and never repaired by suffixing
or a second spawn. A trial is incomplete until its actor is absent and the
cleanup observation is retained.

## Collision positive control

Collision zero is unavailable until a separate positive-control run qualifies
the exact rendered collision manifest and contact configuration used by the
candidate suite. The control is not one of the 15 mission trials and its
intentional collision is not added to a mission collision count.

The frozen control uses the same world, built robot, rendered-SDF hash,
source-bound Gazebo contact aggregator, private raw contact bridge, compiled
contact-stream gate, collector, and schema-v3
collision-coverage manifest. The manifest binds the gate CMake/header/source/
node inventory, plus the aggregator/configuration sources. Uniquely tagged
digests in the loaded plugin DSO and running gate ELF must match that inventory.
The sole private bridge publisher feeds only the gate; the
gate is the sole public publisher. It suppresses its first finalized raw stamp,
then provides a seeded, strictly increasing authoritative delivered active-pair
snapshot stream before motion. The control starts the robot at
`(0.0, -3.5, 0.0)` with Nav2 and the collision monitor absent. A dedicated
`contact_control_driver` is the sole publisher of the final Gazebo command
topic for this isolated fixture.

Motion uses this frozen fail-closed authorization sequence:

1. The driver completes preparation, observes its required positive `/clock`
   and later authoritative contact state, proves its stable graph, atomically
   writes READY, and remains stationary while continuing to spin and audit the
   graph and sources.
2. The runner completes the positive runtime gate while the driver is
   stationary. It requires gate exit zero, an empty gate process group, a
   canonical semantic PASS with the frozen ownership, subscriber, and QoS
   booleans true, and the launch, collector, and driver still alive.
3. The runner atomically writes the exact canonical run-bound ARM artifact,
   bound by SHA-256 to READY and the runtime-gate artifact. The driver rejects
   missing, stale, oversized, symlinked, malformed, noncanonical, wrong-run,
   wrong-producer, or hash-mismatched ARM evidence.
4. After accepting ARM, the driver records its current clock sample and keeps
   spinning until a strictly newer positive `/clock` sample arrives.
5. The driver requires exactly two subscriptions matched to its command
   publisher, publishes one untracked safe-zero probe, and waits for the
   collector's canonical first-retained-command progress artifact. It validates
   the artifact hash, schema, producer, run ID, topic, single zero-valued
   command, callback time, and simulation stamp. Evidence must satisfy
   `arm_observed <= match <= publish_start <= publish_return <= armed` and
   `publish_start <= collector_observed <= armed` in steady time, plus
   `arm_observed_sim < probe_sim <= armed_sim` and an absolute 100 ms bracket
   between the probe and retained-command simulation stamps.
6. The driver atomically writes the driver-owned ARMED acknowledgement bound to
   ARM, READY, runtime-gate, and command-progress evidence. Only a valid ARMED
   acknowledgement permits the first nonzero command. Any missing artifact,
   hash mismatch, cardinality error, or ordering violation fails closed. A
   best-effort zero remains allowed before arm solely for fail-safe cleanup.

During preparation, before READY, the driver spawns:

```yaml
entity:
  name: phase3_contact_control_wall
  static: true
  geometry: {type: box, size_x_m: 0.10, size_y_m: 0.80, size_z_m: 0.80}
  world_pose: {x_m: 0.70, y_m: -3.50, z_m: 0.40, yaw_rad: 0.0}
```

Service discovery retains its bounded `2.0 s` readiness check. Once the
non-idempotent spawn request is dispatched, however, the driver never reissues
it: the one in-flight response may use the remaining operational portion of
the never-reset fixture deadline. Normal work stops at the precomputed `25 s`
operational boundary. A response that has not completed by that boundary is an
infrastructure failure followed by the same mandatory actor cleanup. The final
`5 s` is reserved for exactly-once deletion and the pose-source absence proof;
entering cleanup exposes that precomputed tail and never resets either bound.

The wall's near face is `x=0.65`; the current chassis front begins at
`x=0.24`. The driver commands exactly `linear.x=0.05 m/s` and
`angular.z=0` until the first qualifying contact or a `12.0 s` simulation
deadline. Missing contact fails the control. On contact it publishes zero
within `0.10 s`, holds for `0.25 s`, reverses at `-0.05 m/s` for
`1.0 s`, and publishes final zero. Release completes only on an actually
retained authoritative snapshot `q` with the wall absent and
`q > final_zero_stamp + 0.25 s`; equality and a clock-only target do not close
the episode. The collector acknowledges and retains that exact `q`. A `30 s`
steady-wall escape bounds the complete fixture, including preparation, the
stationary runtime gate, ARM, fresh-clock, matched-subscription, and delivered
zero-probe waits, motion, release, and cleanup. The runner's READY wait is
`35 s`, outside the complete fixture bound but inside the driver's `45 s`
process wrapper, so it cannot terminate bounded cleanup first. READY, ARM, and
ARMED do not start or reset any deadline.

The retained start sample may predate spawn. Its alignment error reconstructs
the clock at start verification and must satisfy
`max(spawn response, wall observation, start sample) <= verified start <= first control`.
It need not equal first control because the stationary READY/ARM protocol occurs
after preparation.

The expected non-excluded pair is the coverage manifest's exact rendered
chassis collision and
`phase3_contact_control_wall::link::collision`. At least one retained public
snapshot record and
exactly one de-duplicated counterpart episode must be observed, named, closed,
and reconciled from snapshot presence/absence. The test also requires public
source gaps and an explicitly caught-up final clock bracket within `0.22 s`.
Passive collector callback offsets are retained as noncausal diagnostics; the
component replay also treats offsets on snapshots whose `collector_sequence`
precedes the first exact `FORWARD` command as diagnostic, even above `0.22 s`.
The validated command trace supplies that boundary. Each callback's summary
and exact contiguous records form a non-overlapping sequence interval that may
not straddle it; a wholly earlier interval's cached delivery-clock stamp cannot
exceed the first `FORWARD` simulation stamp. Delivery-clock stamps cannot
regress, and an active interval cannot precede that stamp. Every interval at
or after the boundary retains the absolute `0.22 s` callback-time bound.
Source gap and order, record linkage, component/capture bijection,
qualifying-contact, release, and final clock-bracket checks remain fail-closed. Exact graph
endpoint cardinality/GID continuity, the final command being zero, false
collector overflows, actor deletion, and termination of both owned process
groups are also required. The captured command stream begins with the distinct delivered
zero probe and then contains exactly one observation for every component
command publication within `0.10 s`; its cardinality is exactly the component
trace length plus one, with no unmatched record. The active contact stop retains
the same `0.10 s` maximum response bound.

The coverage manifest, not this one chassis contact alone, must enumerate and
bind every rendered robot collision geometry. A chassis-only production
contact source does not qualify the suite. Every one of the 15 trials stores
the passing control run ID and canonical JSON hash; any source, rendered SDF,
coverage, bridge, or collector hash mismatch invalidates that reference.

## Exact candidate-suite order and isolation

A candidate is exactly 15 cold-stack trials. The immutable suite index is:

| Suite indices | Scenario | Repetition indices |
| --- | --- | --- |
| 0, 1, 2 | Scenario 1 | 0, 1, 2 |
| 3, 4, 5 | Scenario 2 | 0, 1, 2 |
| 6, 7, 8 | Scenario 3 | 0, 1, 2 |
| 9, 10, 11 | Scenario 4 | 0, 1, 2 |
| 12, 13, 14 | Scenario 5 | 0, 1, 2 |

The suite starts only from a clean Git worktree: HEAD is fixed,
`git status --porcelain=v1` is empty, the dirty flag is false, and the exact
source/configuration/contract hashes are captured. All 15 indices use that
same commit and installed-source binding. A source/configuration/target change
invalidates the candidate; execution restarts with 15 new run IDs rather than
mixing trials.

Each index performs this sequence:

1. prove the previous trial's recorded PGIDs and exact descendants are gone;
2. allocate a never-before-used run ID and empty artifact directory;
3. allocate `ROS_DOMAIN_ID = domain_base + suite_index`, where `domain_base`
   is recorded, `0 <= domain_base`, and `domain_base + 14 <= 232`;
4. allocate
   `GZ_PARTITION=robotest_p3_<candidate_id>_<zero-padded-suite-index>` after
   validating its characters and proving it is unused;
5. create fresh, separately recorded stack and sampler session leaders/PGIDs;
6. launch new Gazebo, ROS, Nav2, fault-proxy, mission, scenario, and metrics
   processes with CPU affinity limited to logical CPUs 0-5;
7. prove a reset fault state, empty actor set, fresh localization, and expected
   graph before dispatch;
8. execute once with `retries: 0` and the frozen seeds;
9. finalize terminal/drain evidence, atomically finalize artifacts, then
   terminate only the owned process groups; and
10. prove cleanup, source immutability, and artifact checksums before advancing.

The orchestrator must reject a domain or partition that has any participant;
it does not auto-increment or silently choose another. PID identity includes
Linux start ticks so PID reuse cannot satisfy cleanup. Global `pkill`, ROS CLI
daemons, a warm simulator, inherited fault generation, reused localization,
or reuse of a collector is prohibited. A failure or timeout remains at its
ordered index and in the denominator; there is no fourth attempt or
replacement run.

## Bounded collection and artifacts

The metrics contract's prefix-retaining capacities apply per trial:

| Evidence stream | Hard retained capacity |
| --- | ---: |
| Ground-truth pose | 8,192 samples |
| Each configured dynamic TF edge | 8,192 samples |
| Raw and validated odometry | 8,192 per stream |
| Raw and validated scan metadata | 2,048 per stream |
| Final velocity command | 4,096 samples |
| Global plans | 1,024 messages and 65,536 poses total |
| Contact snapshot evidence | 8,192 summaries and 32,768 normalized snapshot records |
| World statistics | 4,096 samples |
| Mission/lifecycle/obstacle/process state | 1,024 transitions |
| Fault-control/application events | 512 events |
| Mission-runner feedback trace | 4,096 entries |
| Clock | Constant-space first/latest/count/gap/regression summary |

Scenario 3's 121 targets, matched observations, and controller transitions fit
inside the 1,024 obstacle-state capacity and are never placed in an unbounded
side list. Every buffer retains the prefix and fails on its first overflow;
ring overwrite, silent downsampling, truncation, or acceptance from a partial
trace is prohibited.

Artifact caps are `32 MiB` for canonical per-run JSON, `1 MiB` for its
one-row CSV, `8 MiB` for each captured log, `4 MiB` for each of at most eight
PNG charts, `256 MiB` for a complete trial directory, and `64 MiB` for the
complete aggregate/report directory. Actual sizes, configured limits, ingress
counts, retained counts, invalid counts, first-overflow evidence, and
checksums are machine-readable. Exceeding any cap fails the affected run.

Contact collection continues until it retains and acknowledges a public
snapshot strictly later than `T_terminal + 0.25 s`; its wall escape is `5.0 s`.
Only episodes beginning in `[T0, T_terminal]` count, but post-terminal
snapshot records remain diagnostic.
Failure to advance, drain, or close evidence makes collision count null and the
trial fail.

The project process-tree `rss_sum` must remain at or below `6.0 GiB`. Runtime
descendants remain on CPUs 0-5. Headless candidate aggregation requires median
RTF at least `0.80` and nearest-rank p5 at least `0.50` after only the
contract-declared exclusions.

## Result ownership and aggregation

The mission runner owns the bounded action outcome and its JSON/CSV. The fault
proxy owns fault-control/application events. The scenario controller owns
actor/trigger/control evidence and a bounded `scenario-result.json`. The
metrics process owns observed measurements and quality results. None of those
component artifacts may declare the benchmark trial passed.

After every producer is terminal and its file hash is fixed, the suite
orchestrator alone atomically writes the canonical per-trial
`run-result.json` and matching one-row `run-result.csv`. That canonical JSON
is the sole per-trial benchmark verdict. A succeeded action with a scenario,
collision, fault, quality, cap, checksum, cleanup, or mutation failure remains
a failed trial.

The per-run CSV retains scalar leaves and represents each JSON array by a
canonical `{kind: sequence, element_count, sha256}` descriptor. Reserved root
columns bind projection contract `bounded_scalar_summary_v1` and the SHA-256 of
the complete canonical JSON, so the 1 MiB comparison view does not duplicate
the JSON's bounded raw sequences. If primary artifact finalization fails, exit
31 is finalized from a fresh compact immutable-context-bound FAIL result; the
failure path never deep-copies the oversized measurements or component traces.
Its explicit `quality.artifact_finalization` record is `FAIL`; the replacement
result's schema-required projection preflight is `PASS` because the compact
replacement itself was successfully preflighted and finalized.
The bounded scalar-summary contract is per-run only. Aggregate CSV retains the
legacy complete dotted-key projection, including full canonical array values,
through a separate aggregate writer so Phase 5 replay bytes do not drift.

Aggregate reports consume exactly the 15 canonical JSON files in suite-index
order. For finite values sorted ascending as `x_1...x_n`:

```text
nearest_rank_p(r) = x_ceiling(r*n)
```

with one-based indexing. Consequently, with three repetitions, p95 is the
maximum and p5 is the minimum. Median uses the middle value for odd `n` and
the mean of the two middle values for even `n`. Empty numeric input is null,
not zero. No interpolation, outlier removal, winsorization, replacement,
post-observation exclusion, or library-default percentile is permitted.

Each scenario passes only with 3 of 3 successful trials. Failures and nulls
remain in the acceptance denominator. Aggregates record all source values,
valid count, null count/reasons, success numerator, denominator, and ordered
run IDs.

## Failure and recovery policy

Nav2's configured in-process recovery behavior may execute and is retained as
evidence; it does not authorize a process restart or a second mission goal.
The installed Jazzy `FollowWaypoints` feedback has no structured recovery
counter, so the recovery count remains null unless a separately versioned
structured source is added before the candidate and named in the target set.
Human log parsing is not a substitute.

Scenario 4's sensor-restoration window is the only fault-recovery acceptance
gate. Scenario 5 does not reset its fault before the complete active interval.
For all scenarios, cancellation after an infrastructure or safety failure is
cleanup, not a retry and not success.

## Consequences

### Positive

- Every run exercises the same world, mission, actor timeline, fault schedule,
  and evidence rules.
- Scenario 2 cannot spawn an obstacle into the robot or manufacture a replan
  by inserting before the initial path.
- Scenario 3 is anchored to action progress and proves actual Gazebo motion,
  not merely service requests.
- Cold-stack repetition and single-result ownership prevent a warm-state,
  replacement-run, or component-only false pass.

### Costs

- Scenario 3 requires a bounded set-pose controller and observed-world-pose
  bridge.
- Complete collision coverage and the positive control must exist before a
  zero-collision result is available.
- One failed index invalidates 3-of-3 acceptance for that scenario and cannot
  be silently replaced.

## Implementation status

As of this decision, the Phase 3 scenario controller, actor bridge, complete
collision-contact coverage, positive-control fixture, fault protocol, metrics
collector, 15-trial runner, and canonical benchmark compositor are targets
awaiting implementation and verification. This ADR records no benchmark
result.

### Update — 2026-08-26

The scenario-controller and actor interfaces, collision-coverage checks,
positive-control fixture, metrics collector, bounded staged runner, and
canonical per-run and aggregate composition paths are now implemented with
unit/contract tests. The development Phase 3 verifier at
`artifacts/evidence/phase3/20260826T052402Z-3022/` passed its static, fresh
build, package-test, installed-CLI, and dependency gates. The package lifecycle
evidence at `artifacts/evidence/phase4/package-lifecycle-20260826T045138Z.json`
also passed.

The staged positive control, runtime smoke, and exact 15-trial cold-stack
candidate have not yet supplied authoritative Phase 3 benchmark evidence.
This ADR therefore still records no benchmark result and makes no campaign
PASS claim.

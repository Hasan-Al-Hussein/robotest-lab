# ADR 0005: Phase 3 Deterministic Fault Protocol

- Status: Accepted for Phase 3 implementation
- Date: 2026-08-26
- Decision owners: RoboTest Lab fault, mission, metrics, and safety gate

## Context

[ADR 0003](0003-two-phase-fault-schedule-arming.md) established that a fault
schedule must be prepared before a navigation goal and armed only after the
goal has an authoritative UUID and accepted-goal stamp. It intentionally left
several wire-level choices open. In particular, it did not freeze duplicate
request behavior, generation allocation, canonical schedule bytes, event
schema version 2, or the exact odometry-drift transform.

The current Phase 1 implementation exposes
`robotest_interfaces/srv/LoadFaultSchedule`, accepts as many as 256
specifications, and combines a schedule with `mission_start`. Its
`FaultEvent` schema version 1 cannot prove a prepared generation, an atomic
arm, a UUID-bound `T0`, an exact control replay, or restoration counts. Those
interfaces remain valid evidence of Phase 1 pass-through behavior, but they
cannot be extended in place without leaving ambiguous Phase 3 results.

Phase 3 implements only deterministic LiDAR dropout and odometry drift after
the no-fault path. It also needs a precise collision-monitor timeout and
recovery declaration so a passing result cannot depend on callback timing or
human interpretation.

## Decision

This ADR completes and narrows ADR 0003 for Phase 3. Where a detailed rule
below conflicts with an open choice in ADR 0003, this ADR controls. The
preload-before-goal and UUID-matched arm ordering from ADR 0003 remains
mandatory.

### Control interfaces

The implementation replaces the exposed Phase 1 load operation with these
Phase 3 controls:

| Service | Required request | Required response |
| --- | --- | --- |
| `/robotest/faults/preload_schedule` | Claimed lowercase SHA-256 and `FaultSpec[<=16]` | Accepted, replayed, resulting state, committed hash, count, generation, and bounded diagnostic |
| `/robotest/faults/arm_schedule` | Claimed hash, generation, accepted goal UUID, and authoritative accepted-goal stamp | Accepted, replayed, resulting state, bound hash/generation/UUID/`T0`, arm commit stamp, and bounded diagnostic |
| `/robotest/faults/reset` | Empty `std_srvs/srv/Trigger` request | Success and bounded diagnostic |

The new custom services are named `PreloadFaultSchedule.srv` and
`ArmFaultSchedule.srv`. The legacy `LoadFaultSchedule` service must not be
present in a Phase 3 runtime graph. Generated service QoS remains RELIABLE and
every client availability/response wait has a steady wall-clock deadline.

The proxy serializes every control transaction through one state lock. It
validates a complete candidate before changing any committed state. A response
and its corresponding event describe the same atomic result; another request
cannot observe an intermediate schedule or binding.

### State machine and generation

The externally visible states are:

```text
RESET --new valid preload--> PREPARED --valid arm--> ARMED
  ^                              |                    |
  +------------ reset ----------+-------- reset -----+
```

- `RESET`: no current generation, schedule, goal binding, or active
  deterministic state. All streams are pass-through.
- `PREPARED`: one completely validated, inert generation is stored. It
  cannot affect any stream.
- `ARMED`: that exact generation is bound to one goal UUID and `T0`.
  Faults follow their half-open intervals. The state remains `ARMED` after
  the final interval until reset, although all streams are then pass-through.

The wire values are fixed as `STATE_RESET=0`, `STATE_PREPARED=1`, and
`STATE_ARMED=2`. They are append-only values and must not be renumbered.

Generation is an unsigned 64-bit value scoped to one proxy process lifetime.
Zero means no committed generation. The first new successful preload returns
one. Each later successful preload of different canonical bytes increments
the generation exactly once. Reset invalidates the current token but does not
rewind the allocation counter. Rejected requests and exact replays never
increment it. Generation must never wrap; allocation at `UINT64_MAX` is
rejected until the proxy is restarted. Run evidence binds the generation to
the recorded proxy process identity and start time.

The transition and replay policy is exact:

| Current state and request | Result |
| --- | --- |
| `RESET` + valid preload | Commit new generation; enter `PREPARED` |
| `PREPARED` + same canonical bytes/hash | Accept as replay; retain generation and state |
| `PREPARED` + different valid schedule | Atomically replace it with the next generation; remain `PREPARED` |
| `ARMED` + same canonical preload | Accept as replay; retain the armed binding |
| `ARMED` + any different preload | Reject without mutation |
| `PREPARED` + matching valid arm | Bind atomically; enter `ARMED` |
| `ARMED` + identical hash/generation/UUID/`T0` arm | Accept as replay; retain the original arm commit stamp |
| `ARMED` + any non-identical arm | Reject without mutation |
| `RESET` + arm, or an arm with any mismatch | Reject without mutation |
| Any state + reset | Clear the binding, fault state, stream counters, and transformation PRNG state; enter `RESET` |
| `RESET` + reset | Accept idempotently and remain `RESET` |

“Same” means byte-identical normalized canonical schedule content, not merely a
caller-supplied matching digest. An alternate wire order or whitespace spelling
that normalizes to the same canonical bytes is the same schedule. A SHA-256
collision is not treated as equality without canonical-byte equality.

### Canonical schedule and SHA-256

Both the Python mission runner and C++ fault proxy independently construct the
same canonical byte sequence. The proxy never trusts the claimed digest.

Validation and serialization occur in this order:

1. Normalize each ROS duration to
   `seconds * 1_000_000_000 + nanoseconds` using checked integer arithmetic.
2. Parse `parameters_json` as UTF-8 JSON, reject duplicate keys and any value
   outside the mode-specific object below, and normalize the object.
3. Sort specifications by
   `(start_offset_ns, target, fault_id)` using unsigned numeric order for the
   first two fields and bytewise ASCII order for `fault_id`.
4. Emit UTF-8 JSON with no byte-order mark, insignificant whitespace, or final
   newline. All property names and string values are ASCII. Integers use base
   10 with no leading plus sign or leading zero except the value `0`.
5. Compute SHA-256 over exactly those bytes and render 64 lowercase
   hexadecimal characters.

The top-level property order is exactly:

```json
{"schema_version":1,"faults":[]}
```

Each fault object uses exactly this property order:

```json
{"schema_version":1,"fault_id":"scan_drop","target":1,"mode":1,"start_offset_ns":10000000000,"duration_ns":2000000000,"seed":42,"parameters":{}}
```

The `faults` array contains the sorted objects. `parameters` is the parsed
object, never a JSON-encoded string inside the canonical schedule.

The only Phase 3 parameter objects are:

| Mode | Canonical parameters |
| --- | --- |
| `MODE_PASS_THROUGH` | `{}` |
| `MODE_LIDAR_DROPOUT` | `{}` |
| `MODE_ODOM_DRIFT` | `{"x_rate_nm_per_s":<int64>,"yaw_rate_nrad_per_s":<int64>}` in that property order |

Integer nanometres and nanoradians per second avoid cross-language floating
point formatting differences. The runtime conversions are
`x_rate_m_per_s = x_rate_nm_per_s * 1e-9` and
`yaw_rate_rad_per_s = yaw_rate_nrad_per_s * 1e-9`.

These known-answer fixtures are mandatory in both language test suites:

```text
bytes:  {"schema_version":1,"faults":[]}
sha256: 26080d7dc8f4108a369962ecad1d2e29941a991af68beb415067eae1dc1de6f8

bytes:  {"schema_version":1,"faults":[{"schema_version":1,"fault_id":"scan_drop","target":1,"mode":1,"start_offset_ns":10000000000,"duration_ns":2000000000,"seed":42,"parameters":{}}]}
sha256: 5f93838ca7c0be214858ffe8f62fa351b82d260f6223f139dfaf6bd3dcd224c8

bytes:  {"schema_version":1,"faults":[{"schema_version":1,"fault_id":"odom_drift","target":2,"mode":4,"start_offset_ns":10000000000,"duration_ns":20000000000,"seed":42,"parameters":{"x_rate_nm_per_s":10000000,"yaw_rate_nrad_per_s":5000000}}]}
sha256: 3c72bc48e05221a52058112e58c94b578550deb2ee1ff4ad0d719675917335eb
```

Preload rejects a claimed hash that differs from the recomputed value. Its
response returns the recomputed value. Canonical bytes and their digest are
retained in run evidence.

### Supported modes and validation bounds

Phase 3 accepts only:

- `MODE_PASS_THROUGH` on `TARGET_SCAN`, `TARGET_ODOM`, or
  `TARGET_IMU`;
- `MODE_LIDAR_DROPOUT` on `TARGET_SCAN`; and
- `MODE_ODOM_DRIFT` on `TARGET_ODOM`.

Every other currently declared mode or target/mode pairing is rejected. New
modes require a later decision, canonical parameter schema, pure tests, and
runtime evidence before they can be accepted.

A valid Phase 3 schedule also satisfies all of these bounds:

- at most 16 specifications; an empty no-fault schedule is valid;
- schema version exactly one;
- unique ASCII `fault_id` matching
  `[A-Za-z0-9][A-Za-z0-9._-]{0,63}`;
- normalized `start_offset_ns >= 500_000_000` for every non-empty schedule;
- `duration_ns > 0`;
- every start, duration, and end is at most
  `3_600_000_000_000` nanoseconds;
- `abs(x_rate_nm_per_s) <= 1_000_000_000` and
  `abs(yaw_rate_nrad_per_s) <= 1_000_000_000`;
- at least one odometry-drift rate is nonzero; and
- no two specifications for the same target have intersecting half-open
  intervals. Touching endpoints are allowed. Cross-target overlap is allowed.

The same-target overlap rule applies to pass-through specifications too. It
keeps transformation order, affected counts, and restoration ownership
unambiguous.

### Goal binding, `T0`, and arming margin

The mission runner subscribes to
`/robotest/follow_waypoints/_action/status` before goal submission and
matches the exact accepted goal UUID. The only authoritative `T0` is the
first positive, normalized, immutable
`GoalStatusArray.goal_info.stamp` observed for that UUID. Repeated identical
status stamps are allowed. A different stamp for the same UUID is an
infrastructure failure. The installed Jazzy SendGoal response's zero stamp is
retained as unavailable provenance and is never substituted for `T0`.

An arm request is valid only when:

- the UUID is not all zero;
- `T0` is positive, normalized, and no later than the proxy's arm commit
  time on the same ROS clock;
- hash and generation match the prepared state; and
- for a non-empty schedule,
  `arm_commit_time <= T0 + earliest_start_offset - 0.5 s`.

Thus every successful non-empty arm has at least 0.5 simulation seconds of
measured margin before the first configured activation. The response and
`EVENT_ARMED` retain the commit stamp and calculated margin. A late,
unavailable, rejected, or inconsistent arm causes bounded goal cancellation,
terminal-result collection, reset, and an infrastructure-failure result. No
fault may activate, and no mission may be reported successful, on that path.

### Fault intervals and LiDAR dropout

For a message stamped `t`, a specification is active exactly when:

```text
T0 + start_offset <= t < T0 + start_offset + duration
```

All comparisons use checked integer nanoseconds. Simulation pause pauses the
schedule. A steady wall-clock escape deadline remains mandatory for all waits.

LiDAR dropout suppresses each matching validated scan while raw scans continue.
It publishes neither an empty scan nor a stale replacement. The first
suppressed raw scan produces the first-affected evidence. The first raw scan
at or after the interval end is forwarded unchanged and is the first-restored
scan.

### Odometry-drift transform

For an active odometry-drift specification, let:

```text
tau = message_stamp - (T0 + start_offset)
dx = x_rate_m_per_s * tau
dtheta = yaw_rate_rad_per_s * tau
```

with `tau` in seconds and `0 <= tau < duration`. Define the planar offset
in the `odom` frame:

```text
         [ cos(dtheta)  -sin(dtheta)  dx ]
D(tau) = [ sin(dtheta)   cos(dtheta)   0 ]
         [      0             0        1 ]
```

If `T_raw` is the raw planar `odom -> base_footprint` pose, the validated
pose is:

```text
T_validated = D(tau) * T_raw
```

The measured injected transform is therefore:

```text
D_measured = T_validated * inverse(T_raw)
```

Evidence compares the x translation and normalized yaw of `D_measured` with
`dx` and `dtheta`. The proxy preserves the raw header stamp, frame IDs,
z value, twist, pose covariance, and twist covariance. It changes only the
planar pose and emits a normalized quaternion. The
`odom -> base_footprint` TF is derived from that same validated pose and
stamp; it is never calculated through a separate state path. Raw odometry is
not mutated.

At the half-open interval end, output returns to unmodified pass-through; no
drift remains latched. Because a message exactly at the end is already
restored, Scenario 5 endpoint evidence uses the final active sample and is
valid only when:

```text
0 <= configured_end - final_active_sample_stamp <= 0.25 s
```

The artifact records that gap, the expected offset at the sample, the measured
offset, and the full-duration target. It does not interpolate or invent an
unobserved endpoint.

### FaultEvent schema version 2

`FaultEvent` schema version 2 preserves event values 1 through 6 and appends
arm and restoration events:

| Value | Event |
| --- | --- |
| 1 | `EVENT_SCHEDULE_LOADED`, meaning preload accepted or exact preload replay |
| 2 | `EVENT_SCHEDULE_REJECTED` |
| 3 | `EVENT_RESET` |
| 4 | `EVENT_ACTIVATED` |
| 5 | `EVENT_FIRST_AFFECTED` |
| 6 | `EVENT_DEACTIVATED` |
| 7 | `EVENT_ARMED`, meaning arm accepted or exact arm replay |
| 8 | `EVENT_ARM_REJECTED` |
| 9 | `EVENT_FIRST_RESTORED` |

Every event carries bounded, machine-readable fields sufficient to reconcile
the transition without parsing `detail`:

- schema/event version, event sequence, event type, state before and after,
  accepted, and replayed;
- requested and committed schedule hash, generation, and fault count;
- requested and bound goal UUID and `T0`;
- fault ID, target, mode, and seed for data-path events;
- configured activation/deactivation, actual event time, arm commit time, and
  signed arm-margin nanoseconds;
- per-target input sequence plus cumulative raw-input, validated-output, and
  affected-message counts; and
- bounded diagnostic detail for operators only.

Unused UUIDs are all zero, unused generations are zero, and unavailable times
are zero with the event type determining applicability. The monotonically
increasing event sequence orders events that share a simulation timestamp and
must not wrap.

Control events use the atomic commit/rejection stamp as `actual_time`.
Data-path events use the triggering input message stamp. An activation event
does not imply a message was affected; `EVENT_FIRST_AFFECTED` proves that
separately. `EVENT_DEACTIVATED` records the first post-interval input, while
`EVENT_FIRST_RESTORED` records the first corresponding unmodified validated
output. For dropout and drift they normally share a stamp but remain distinct
facts.

The event topic remains RELIABLE, VOLATILE, KEEP_LAST(100). With the 16-fault
runtime bound and at most four lifecycle events per fault, one normal run plus
its preload, arm, and reset fits the history. The evidence subscriber must be
ready before preload. A missing event, sequence gap, counter mismatch, or
collector overflow invalidates the run rather than being reconstructed from
logs.

### Collision-monitor timeout and recovery stability

The collision monitor's validated-scan source timeout is exactly 0.60
simulation seconds. It uses the node's ROS clock with `use_sim_time=true`.
The Phase 3 verifier proves the effective runtime value and proves the final
command reaches the frozen zero-command tolerance after the source becomes
stale. A different installed or launch-overridden value invalidates Scenario
4.

For LiDAR recovery, the first restored scan starts a candidate interval.
Recovery is declared only at the end of a continuous 1.0 simulation-second
window in which:

- restored validated scans remain fresh, with no inter-scan gap above 0.40
  simulation seconds;
- every required Nav2 lifecycle node remains active;
- the mission is neither canceled, aborted, timed out, nor otherwise failed;
  and
- the window contains evidence of resumed navigation: at least 0.05 m planar
  displacement, at least 0.10 rad absolute normalized yaw change, a waypoint
  completion increment, or terminal goal success.

Any freshness, lifecycle, or mission-condition break resets the candidate.
The artifact stores the candidate-start stamp and the later confirmation stamp.
`recovered_stamp` is the confirmation stamp, so
`sensor_recovery_time_sim_s = recovered_stamp - actual_deactivation_stamp`
includes the proof window. A single restored message or instantaneous motion
sample is not recovery.

### Clean-trial and failure requirements

Every Phase 3 trial starts from `RESET`, confirms pass-through, preloads once,
and arms only the accepted goal it records. Trial teardown always performs a
bounded reset and confirms no armed generation survives. Repeated benchmark
trials use a cold mission/fault state and retain failed trials.

Preload, arm, event, stream, and reset paths use bounded queues and bounded
steady wall-clock waits. Rejection, timeout, stopped `/clock`, event loss,
hash disagreement, generation disagreement, UUID/`T0` ambiguity, inadequate
arming margin, missing restoration, or reset failure is an infrastructure
failure. It is never converted to a scenario success.

## Required proof before Phase 3 acceptance

- Python and C++ known-answer tests produce all canonical bytes and hashes in
  this ADR, including alternate input order and whitespace normalizing to the
  same bytes.
- Pure state tests cover every transition and replay row, generation
  allocation, replacement, reset, overflow, and concurrent requests.
- Validation tests cover all mode/target pairs, the 16-fault bound, time
  horizon, parameter bounds, duplicate IDs, same-target overlap rejection,
  touching endpoints, and hash mismatch.
- Boundary tests cover the half-open interval at activation and restoration.
- Drift tests prove left-multiplied SE(2) composition,
  `D_measured = T_validated * inverse(T_raw)`, quaternion normalization,
  unchanged twist/covariance, matching odometry/TF, and endpoint-gap handling.
- Mission tests cover UUID-matched status `T0`, conflicting status stamps,
  0.5-second arm margin, exact service replays, late arm cancellation, reset,
  stopped simulation time, and distinct infrastructure outcomes.
- Runtime Scenario 4 and 5 evidence reconciles every event sequence and
  counter, proves source timeout 0.60, proves the 1.0-second recovery window,
  and retains raw and validated streams.

## Consequences

### Positive

- At-least-once service delivery cannot allocate extra generations or bind a
  schedule to a different goal.
- Python and C++ agree on schedule identity without floating-point JSON
  formatting differences.
- Same-target overlap rejection gives every transformed message one fault
  owner.
- Odometry and TF evidence share one explicit SE(2) definition.
- Recovery and source-stale claims have numeric timing and continuity rules.

### Costs

- Phase 3 needs two new services and a coordinated `FaultEvent` schema
  revision across interfaces, proxy, mission runner, metrics, and tests.
- The 16-fault and 3600-second limits are intentionally smaller than the
  legacy interface.
- Fixed-point drift parameters require explicit conversion at configuration
  boundaries.
- A late arm fails the run even if the first fault would not yet have affected
  a message.

## Rejected alternatives

- Trust the caller's schedule hash: rejected because malformed or differently
  normalized content could be mislabeled.
- Hash YAML or raw `parameters_json`: rejected because whitespace, mapping
  order, and parser spelling are not semantic schedule identity.
- Use floating-point rates in canonical JSON: rejected because equivalent
  values can serialize differently across Python and C++.
- Allocate a generation on every duplicate preload: rejected because response
  loss would change subsequent arm semantics.
- Allow same-target overlaps with a priority rule: rejected because priority,
  transformation composition, counters, and restoration would become
  scenario-dependent.
- Right-multiply drift in the robot frame: rejected because Scenario 5 freezes
  an odom-frame offset.
- Declare recovery on the first restored scan: rejected because one sample
  does not prove a stable stream or resumed navigation.

## Implementation status

This decision freezes the Phase 3 contract before code changes. At acceptance,
the legacy Phase 1 load service and `FaultEvent` version 1 still exist; the
new services, state machine, fault transforms, version 2 events, and runtime
proof described here are not yet implemented or verified.

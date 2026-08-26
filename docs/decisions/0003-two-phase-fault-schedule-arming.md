# ADR 0003: Two-Phase Fault-Schedule Arming

- Status: Accepted and implemented; authoritative Phase 3 benchmark evidence pending
- Date: 2026-08-26
- Decision owners: RoboTest Lab fault and mission contract gate

## Context

The [topic and TF contract](../architecture/topic-and-tf-contract.md) requires
the complete fault schedule to be accepted before the first navigation goal.
It also defines mission `T0` as the timebase for every fault offset. The
[acceptance criteria](../testing/acceptance-criteria.md) define the first
LiDAR-dropout and odometry-drift activations as 10 simulation seconds after
the mission goal is accepted.

The current `LoadFaultSchedule` interface combines the schedule contents and
`mission_start` in one request. This cannot simultaneously prove that the
complete schedule was accepted before the goal and use the action server's
accepted-goal stamp as `T0`: the accepted goal UUID and its UUID-matched action
status become available only after `FollowWaypoints` accepts the goal.

The existing Phase 1 proxy validates and replaces a pass-through schedule
atomically. It does not yet implement a separate preload and arm state, fault
application, or goal binding. Nothing in this decision claims those features
are present.

For the installed Jazzy `rclcpp_action` 28.1.21 server used by Nav2, the
SendGoal response is sent before `rcl_action_accept_new_goal` assigns the
server-side `GoalInfo` stamp. The response stamp retained by the `rclpy`
`ClientGoalHandle` is therefore zero and unavailable as `T0`. After internal
acceptance, the server publishes the authoritative stamp in
`GoalStatusArray.goal_info` on `follow_waypoints/_action/status`. The future
mission runner must subscribe with the action-status QoS, match the exact goal
UUID, require one immutable positive `GoalInfo.stamp`, preserve the raw zero
response stamp as provenance, and never substitute a separately sampled
client clock.

## Decision

Split schedule handling into two explicit future operations:

1. **Preload and validate.** Before sending a navigation goal, the mission
   runner submits the complete canonical fault specification, its SHA-256, and
   all deterministic seeds. The proxy validates every element before replacing
   the prepared schedule. A successful response returns the matching hash,
   loaded count, and a monotonic schedule generation or equivalent opaque
   token. The prepared schedule is inert.
2. **Arm atomically.** After `FollowWaypoints` accepts the goal, the mission
   runner sends the accepted goal UUID and accepted-goal stamp together with
   the prepared schedule hash and generation. A successful arm operation
   atomically binds all prepared faults to that action goal and sets the
   accepted-goal stamp as `T0`.

For every specification:

`activation = T0 + start_offset`

and the active interval remains half-open:

`T0 + start_offset <= message_stamp < T0 + start_offset + duration`.

The proxy remains in deterministic pass-through while the schedule is merely
prepared. No individual fault may become active before the complete arm state
has been committed. A rejected, timed-out, or partially processed request
must leave no partially armed generation.

### Required future state machine

The proxy will make these states externally observable through bounded control
responses and fault events:

```text
RESET -> PREPARED -> ARMED -> RESET
```

- `RESET`: no prepared or armed schedule; pass-through only.
- `PREPARED`: one fully validated inert generation is stored.
- `ARMED`: that exact generation is bound to one accepted goal UUID and `T0`.

A new preload while armed is rejected. Reset disarms the active generation,
clears deterministic state, and invalidates its token. Repeating the exact arm
request may be idempotent, but an arm request with a different goal UUID,
stamp, hash, or generation is rejected. The implementation must choose and
unit-test one explicit duplicate-request policy before runtime testing.

### Required future interfaces

The implementation must split the present combined control semantics. The
exact interface names may follow repository naming conventions, but the wire
contract must provide:

- a preload request containing the canonical schedule hash and the complete
  bounded `FaultSpec` sequence, with a response containing acceptance, hash,
  loaded count, generation/token, and bounded diagnostic text;
- an arm request containing that hash and generation/token,
  `unique_identifier_msgs/msg/UUID` for the accepted `FollowWaypoints` goal,
  and its accepted-goal `builtin_interfaces/msg/Time`, with a response that
  confirms the bound UUID, `T0`, generation, and arm outcome; and
- reset behavior that invalidates prepared and armed state.

The current `LoadFaultSchedule.srv`, proxy state, and fault-event schema will
therefore require a future, coordinated revision. That revision must be made
atomically across `robotest_interfaces`, `robotest_faults`,
`robotest_missions`, tests, and the normative contracts. This ADR does not make
that change.

### Mission-runner ordering and failure behavior

The future mission runner must execute this bounded sequence:

1. validate the mission and canonical schedule locally;
2. reset the proxy and confirm the reset response;
3. preload the complete schedule and reconcile hash, count, and generation;
4. send the `FollowWaypoints` goal;
5. if the goal is rejected, leave the schedule inert, reset it, and return the
   rejected-goal outcome;
6. if accepted, take the goal handle's UUID, obtain the exact UUID-matched
   action-status `GoalInfo.stamp`, and use those as the binding values before
   requesting arming immediately;
7. require arming to finish before `T0 + minimum(start_offset)`; and
8. only then continue the evidence state machine for the mission.

All service availability, response, action, cancellation, and reset waits use
steady wall-clock deadlines. No stopped `/clock` may create an unbounded wait.

The currently frozen fault scenarios have a 10-second earliest offset. This
provides an arming window; it is not permission to wait until the boundary. If
arming is rejected, times out, or cannot be proven complete before the first
window, the mission runner requests goal cancellation, waits for bounded
cancellation acknowledgement and a terminal action result, resets the proxy,
and reports an infrastructure failure. It must not label that run successful.

A future scenario with an earlier first offset must either prove its bounded
arming deadline and margin before execution or introduce an explicit command
hold. It may not silently reuse the 10-second assumption.

## Required evidence

Implementation and Phase 2/3 verification must retain:

- canonical schedule bytes, SHA-256, seed set, count, and generation/token;
- preload request/response stamps, latency, and reconciled response fields;
- action goal UUID, client submission stamp, raw SendGoal response stamp,
  UUID-matched action-status `GoalInfo.stamp` used as `T0`, its evidence
  source, and the goal acceptance outcome;
- arm request/response stamps, latency, bound UUID, bound `T0`, hash,
  generation, and the earliest configured activation;
- fault-control events for preload accepted/rejected, arm accepted/rejected,
  reset, activation, restoration, and affected message counts;
- cancellation request, acknowledgement, terminal status, and final command
  evidence on every arm-failure path; and
- proof that PREPARED schedules do not affect validated streams and that no
  fault is applied before the successful atomic arm.

Canonical JSON and its flattened CSV must reconcile these fields. Human log
text alone is not acceptable evidence.

## Required tests before acceptance

- Pure state tests cover reset, valid preload, invalid preload, arm without
  preload, wrong hash/generation, invalid or zero goal UUID, invalid stamp,
  duplicate arm, reset-after-arm, and preload-while-armed.
- Atomicity tests prove one invalid specification rejects the entire preload
  and concurrent control requests cannot expose a partially replaced or
  partially armed schedule.
- Boundary tests prove the half-open interval at `T0`, activation, and
  deactivation stamps.
- Mission-runner tests cover rejected goals, unavailable services, preload and
  arm timeouts, arm rejection, cancellation rejection, stopped simulation
  time, artifact-write failure, and successful empty/pass-through schedules.
- Runtime tests prove prepared schedules are inert, the successful arm event
  precedes the first fault window, and actual affected-message stamps are no
  earlier than `T0 + start_offset`.

## Consequences

### Positive

- Fault contents are validated before any navigation action can begin.
- Fault offsets use the action server's authoritative accepted-goal stamp.
- A schedule is bound to one goal UUID, preventing evidence from being
  accidentally attributed to another mission.
- Prepared and active state have an explicit, testable boundary.

### Costs

- One additional bounded control round trip occurs after goal acceptance.
- The existing interface and proxy state machine require a coordinated future
  revision.
- Arm failure requires cancellation and cleanup while a goal may already be
  executing, so that failure path needs first-class evidence.

## Rejected alternatives

- Put the accepted-goal time in the preload request: rejected because it does
  not exist before the action server accepts the goal.
- Treat the pre-goal sample time as accepted-goal `T0`: rejected because it
  changes the frozen scenario semantics and hides action-acceptance latency.
- Reload the full schedule after goal acceptance: rejected because it repeats
  validation after motion can begin and does not prove pre-goal acceptance.
- Allow each fault to arm independently: rejected because partial activation
  would make the run and its evidence ambiguous.

## Implementation status

This is a contract-resolution decision only. As of this ADR, the two-phase
interfaces, proxy states, mission-runner sequence, events, and runtime evidence
described above are not implemented or verified.

### Update — 2026-08-26

The two-phase interfaces, proxy state machine, mission-runner ordering,
machine-readable events, and their unit/contract tests are now implemented.
The development Phase 3 verifier at
`artifacts/evidence/phase3/20260826T052402Z-3022/` passed its static, fresh
build, package-test, installed-interface, and dependency gates. The package
lifecycle evidence at
`artifacts/evidence/phase4/package-lifecycle-20260826T045138Z.json` also passed.

Those records are implementation and package evidence only. The Phase 3
positive control, runtime smoke, and exact 15-trial cold-stack candidate remain
pending, so this update does not claim an authoritative benchmark PASS.

# ADR 0008: Phase 3 Contact-Aggregation Performance

- Status: Accepted; fresh live qualification required
- Date: 2026-08-27
- Decision owners: RoboTest Lab simulation, safety, metrics, and benchmark gate

## Context

Phase 3 retains a 2 ms Gazebo maximum step size, so the contact source is
observed at 500 Hz while simulation advances. The source publishes one
aggregate on a 20 ms grid, or 50 Hz. Qualification still requires a calculated
real-time-factor median of at least 0.80 and p5 of at least 0.50.

Four immutable development smokes with the source-bound contact aggregator
missed both RTF limits. The contract values below are recomputed from adjacent
unpaused `world_stats` samples in each retained `capture.json`; Gazebo's
reported series is shown only as supporting diagnostics.
Each row names the directory below
`artifacts/evidence/phase3-benchmarks/`; its `smoke/capture.json` supplies
the samples and its `build-binding.json` supplies the clean Git identity.

| Development smoke | Git revision | Valid intervals | Calculated median | Calculated p5 | Reported median | Reported p5 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `phase3-1457752-001` | `145775223ac2f958846461a45eae3b02e2b0dbb1` | 704 | 0.642304 | 0.395215 | 0.605886 | 0.256470 |
| `phase3-d283bf0-001` | `d283bf01b17459bc10112a87c320a3de35dbd92d` | 1,037 | 0.655477 | 0.393050 | 0.629637 | 0.245581 |
| `phase3-4264651-001` | `4264651bf3494850801ae33bd7c98a58d7362e03` | 1,442 | 0.623870 | 0.373439 | 0.584268 | 0.243179 |
| `phase3-0d86ff3-002` | `0d86ff3baa904db6094992fc4c61df0548c86086` | 3,030 | 0.623951 | 0.373058 | 0.588279 | 0.243633 |

These are non-candidate diagnostic runs. They establish a repeatable
performance problem, not an accepted baseline result.

A later immutable diagnostic smoke, `phase3-bdf220a-001`, was bound to clean
Git revision `bdf220adadd31bdca4782f697d0b274b7a55fd29`. Its 1,332 valid
intervals produced a calculated RTF median of `0.7454219259800696` and p5 of
`0.39448131683228504`, still below the unchanged `0.80` and `0.50` limits.
The smoke's metrics capture retained 34,282 explicit clock observations; the
scenario result retained 35,791. Its runtime gate recorded two distinct
BEST_EFFORT, VOLATILE `/clock` subscriber endpoints for each of
`/robotest/metrics_collector` and `/robotest/scenario_controller`.

The associated host profile is explicitly incomplete because one sampled
thread vanished before its `/proc` record could be read. Its retained partial
samples are diagnostic only. Within those samples, the metrics-collector
process leader accumulated 83.42 CPU-seconds and the scenario-controller
process leader accumulated 54.61 CPU-seconds. Static inspection then showed
that both Python nodes enabled `use_sim_time`, whose rclpy `TimeSource` owns a
BEST_EFFORT `KEEP_LAST(1)` `/clock` subscription, and also created a second
subscription solely for repository evidence.

A bounded local 2,000-message collector benchmark isolated that duplication:
the two-reader node used 1.437 process CPU-seconds, while a one-reader variant
used 0.786 process CPU-seconds and produced the same collector-core clock
count. A separate 5,000-iteration zero-time spin benchmark measured 2.3788
process CPU-seconds for repeated convenience `rclpy.spin_once` calls and
2.2094 for one dedicated executor, an estimated 2.8-second saving at the
smoke's spin volume. These local measurements motivate a controlled internal
change; they do not prove a whole-stack RTF improvement or repair the failed
profile.

The strongest current correlation is the RTF reduction after the source-bound
aggregator entered the Phase 3 stack, with a full entity/SDF inventory scan on
every 2 ms step as a plausible hot path. That is not causal proof. The
Phase 2 and Phase 3 captures differ in more than this system, and no isolated
A/B trial or sampled CPU profile has yet attributed the miss to inventory
validation, contact projection, interval storage, another Gazebo system, or
host scheduling. This decision therefore authorizes a fidelity-preserving
optimization and its measurement; it does not claim that RTF improved.

## Decision

The contact pipeline keeps its externally observable contract unchanged:

- Gazebo physics remains configured for 2 ms steps and the source continues to
  observe all seven `ContactSensorData` components after every completed,
  unpaused physics step;
- aggregation remains on the exact 20 ms grid, with the same right-closed
  interval union, latest-step group selection, canonical pair ordering,
  ROS-visible field projection, record/string bounds, and fail-closed gaps;
- the private/public topics, bridges, QoS, gate, runtime-gate evidence, DSO/ELF
  attestations, positive control, and Phase 5 replay rules do not change; and
- the RTF acceptance limits remain median >=0.80 and p5 >=0.50. Neither a
  threshold, physics rate, source rate, sensor, ray count, controller, nor
  evidence stream may be reduced to make the candidate pass.

Only two internal work reductions are accepted for controlled evaluation.

### Locked binding validation

Before binding lock, the system performs the existing exhaustive relevant
entity and sensor-SDF inventory. It must still prove exactly one top-level
`robotest` model, exactly seven uniquely named contact sensors, their exact
link/collision mappings, one unique collision entity per source, and provision
all seven `ContactSensorData` components before observation begins.

After lock, the per-step direct binding check is O(7): it validates the cached
model, sensor, link, and collision entity identities; their required component
types, names, parents, and contact-data presence. A separate state check walks
the cached IDs of every existing Model, ContactSensor, Link, and Collision and
examines their primary, `Name`, and `ParentEntity` component states without a
global entity traversal. Any new entity, entity marked for removal, removed
component, or relevant structural change triggers the same exhaustive
inventory and SDF rescan before contact observation. When Gazebo reports
one-time activity, the system conditionally scans the typed Model,
ContactSensor, Link, and Collision views and checks each entity's primary,
`Name`, and `ParentEntity` state. Adding one of those structural components to
a previously uncached existing entity therefore cannot evade the rescan.
Cached and type-reported periodic structural changes also retrigger it.
A locked entity marked for removal fails immediately. Unrelated physics-rate
payload and Pose changes do not reopen the exhaustive SDF path. Ambiguity,
rebinding, absence, or structural drift remains fatal and stops the simulator.

Runtime writers must report a component mutation through Gazebo ECM change
state (`SetChanged`); mutating the shared sensor SDF in place without that
signal violates the ECM writer contract and is not treated as a supported
runtime transition. Source/configuration hashes and the installed-DSO binding
remain the authority for immutable launch-time sensor definitions.

This is an event-filtered validation change, not a sampling change. The seven
contact payloads are still read synchronously every completed, unpaused
physics step.

### In-place interval union

Each physics step is still fully validated into a temporary bounded step map
before it can affect interval state. Its groups then replace matching entries
directly in the interval map, and the complete updated union is validated.
The former full-map transactional copy is removed.

If the in-place update makes the complete union exceed any bound, policy fatal
state latches before publication. No output is returned on that call, and all
later observations return the same fatal result; therefore a partially updated
or over-limit map cannot escape as evidence. `ContactAggregatorPolicy::reset`
clears the interval map, stamps, stream state, and policy fatal detail before
the policy can be reused. In the deployed Gazebo system, a policy fatal also
latches the system fatal and emits Stop; the system fatal is deliberately not
cleared by the Gazebo Reset callback, so live recovery requires a clean stack
restart rather than silent continuation.

### Single-reader Python clock observation

The metrics collector and scenario controller keep `use_sim_time=true` and
retain every existing clock count, gap, duplicate, regression, and readiness
rule. Immediately after the base rclpy node constructor returns, each node
locates the exactly one public `node.subscriptions` entry for `/clock` that
`TimeSource` created. It fails construction unless that entry has message type
`rosgraph_msgs/msg/Clock` and exactly BEST_EFFORT, VOLATILE `KEEP_LAST(1)`
QoS. The subscription remains strongly referenced by the node.

The existing callback is replaced with a one-message wrapper. The wrapper
first invokes the captured public subscription callback so all attached ROS
clocks advance, then invokes the repository evidence callback. It does not
inspect or depend on rclpy's private `TimeSource._clock_sub`. Scenario evidence
still runs through the existing guarded callback, including cleanup-mode and
fatal-error behavior; metrics evidence exceptions remain visible to the
executor. Before fusion, the existing `use_sim_time` descriptor is replaced
with a statically typed, read-only boolean descriptor. Runtime attempts to
unset, disable, or directly undeclare it are rejected before the TimeSource
parameter callback can destroy or replace the fused reader. Thus each node
owns one `/clock` DDS reader without changing the clock publisher, message
rate, evidence semantics, or acceptance thresholds.

The metrics collector also creates one context-bound
`SingleThreadedExecutor`, requires that adding the node succeeds, and uses the
same executor during startup and steady-state collection. It removes the node
and shuts the executor down once during finalization. This bounded executor
lifecycle is a companion reduction; the measured duplicate-reader cost is the
primary optimization.

## Required measurement

No performance conclusion may be written from static inspection or the four
historical captures. Before this revision can qualify Phase 3:

1. Rebuild from the clean, source-bound optimized revision with seed `42`, CPU
   affinity 0-5, headless/RViz settings, physics, navigation, collection, and
   host conditions unchanged.
2. Capture a bounded per-thread CPU profile during the next already-required
   smoke and retain the exact source/binary identity, calculated RTF series,
   process resources, run queue, paging, and renderer identity. Report the
   remaining time in locked-state checks, contact projection, interval-map
   operations, protobuf work, and the rest of Gazebo. RTF alone is not causal
   attribution.
3. Run the contact policy and real-ECM state-selection tests, system
   structure/build tests, manifest-v3 source inventory and installed-DSO
   attestations, runtime gate, and full bounded positive control. Fixtures
   must produce the same canonical aggregate bytes and fatal decisions for
   the same inputs.
4. Run a fresh whole-capture Scenario 1 smoke. It must satisfy every unchanged
   metric and evidence gate, including median RTF >=0.80 and p5 >=0.50. A
   contact, source-binding, cleanup, resource, or performance failure rejects
   the candidate and requires diagnosis before any repeated campaign.

The next runtime gate must show exactly one `/clock` subscriber endpoint for
each fused Python node. Unit tests must prove the exact message type and QoS,
TimeSource-before-evidence callback order, ROS-clock and evidence updates from
the one callback, fail-closed discovery and read-only set/undeclare parameter
boundaries, preserved scenario guard behavior, visible metrics errors, and the
collector executor's single add/remove/shutdown lifecycle. The fresh capture
remains responsible for proving that all published clock samples needed by the
unchanged evidence contract are observed; neither the local callback benchmark
nor a static endpoint count qualifies the candidate.

The one required profiling run uses the repository-owned passive profiler and
the contact system's exact opt-in environment flag. Run the complete block
below from the repository root inside WSL after `prepare` and
`positive-control`; replace the two example identity values with the exact
candidate and verifier run already used by those stages. The block redeclares
every shell variable because the preparation command may have run in a prior
shell. The profiler is kept off the benchmark's frozen CPU set, starts before
the smoke, never controls a ROS or Gazebo process, and writes immutable
canonical evidence plus a GNU-style sidecar below
`artifacts/evidence/phase3/performance-profiles/`:

The plugin's profile record schema is version 2. Every PASS record retains
sorted, unique, cumulative per-Linux-TID contributions and exact aggregate
totals for all five timing buckets plus observation, rescan, and publication
counts. Callback migration therefore adds a contributor instead of disabling
measurement or restarting the five-simulation-second cadence. The host profile
binds every contributing TID through the plural `host_thread_bindings` array to
one sampled thread identity in the exact DSO host; a missing, reused, foreign,
or unsampled identity rejects the profile.

Profiler failures are explicit bounded schema-v2 FAIL records on the retained
stderr stream. Clock, identity, contribution-cap, counter, category, and output
failures latch and retry emission until the stream confirms the write; they
cannot silently disable profiling. At most 128 cumulative contact records and
128 contributor identities are accepted. Host sampling permits at most 512
live threads per atomic sample, 524,288 retained thread records, and a 256 MiB
canonical profile. A sample that would exceed a bound is rejected before any
sample or thread counter mutates.

```bash
CANDIDATE_ID=phase3-candidate-001
DOMAIN_BASE=100
OUTPUT_ROOT=artifacts/evidence/phase3-benchmarks
BUILD_BINDING=artifacts/evidence/phase3/VERIFY_RUN_ID/build-binding.json
CANDIDATE_ROOT="$PWD/$OUTPUT_ROOT/$CANDIDATE_ID"
PROFILE_PID=
cleanup_profile() {
  if [[ -n "${PROFILE_PID:-}" ]] && kill -0 "$PROFILE_PID" 2>/dev/null; then
    kill -TERM -- "$PROFILE_PID" 2>/dev/null || true
    wait "$PROFILE_PID" 2>/dev/null || true
  fi
}
trap cleanup_profile EXIT INT TERM

export ROBOTEST_CONTACT_PROFILE=1
taskset -c 6-11 python3 tests/phase3_smoke_host_profiler.py \
  --workspace "$PWD" \
  --candidate-root "$CANDIDATE_ROOT" \
  --candidate-id "$CANDIDATE_ID" \
  --build-binding "$CANDIDATE_ROOT/build-binding.json" \
  --ros-domain-id "$((DOMAIN_BASE + 16))" \
  --gz-partition "robotest_p3_${CANDIDATE_ID}-smoke_00" &
PROFILE_PID=$!

SMOKE_RC=0
scripts/run_benchmarks.sh --mode smoke \
  --candidate-id "$CANDIDATE_ID" \
  --domain-base "$DOMAIN_BASE" \
  --output-root "$OUTPUT_ROOT" \
  --build-binding "$BUILD_BINDING" || SMOKE_RC=$?
PROFILE_RC=0
wait "$PROFILE_PID" || PROFILE_RC=$?
PROFILE_PID=
trap - EXIT INT TERM
unset ROBOTEST_CONTACT_PROFILE
((SMOKE_RC == 0 && PROFILE_RC == 0))
```

The profiler PASS means the profile itself is complete and source-bound; it
does not replace the smoke verdict. Both return codes must be zero. The
campaign is run later with `ROBOTEST_CONTACT_PROFILE` unset, so the accepted
candidate timings and process topology retain their normal production shape.
Before campaign mode creates a run or aggregate directory or starts any
process, it independently derives the one canonical profile path and validates
the canonical JSON, GNU checksum sidecar, PASS schema and producer, candidate
and smoke identity, build/plan/prepared/Git hashes, plugin and source inventory,
profiler source, completed smoke result, process lifecycle, sampling, renderer,
and contact-profile joins. Missing, failed, moved outside the derived canonical
artifact location, internally inconsistent, or tampered profile evidence
blocks all 15 trials. Phase 5 invokes the same
clone-local validator and records the exact accepted profile path and hash in
the release-evidence report.

The implementation is kept when the correctness/evidence checks and fresh
candidate pass unchanged acceptance. It is reverted or revised if the fresh
run is neutral/worse than the retained diagnostic band, exposes a new contact
or cleanup difference, or still misses either RTF threshold. A separate
alternating baseline/optimized A/B (at least three pairs, identical clean
build and host conditions) is required before making a causal public claim
about how much improvement came from locked validation versus the no-copy
union; it is not substituted for the whole-candidate qualification gate.

After the implementation and this ADR are committed, collision coverage,
source-configuration, source-tree, install, manifest, aggregator DSO, and gate
ELF hashes must be regenerated. Historical positive controls and smokes cannot
be rebound. A new candidate ID, clean build binding, runtime gate, positive
control, and fresh smoke are mandatory before any 15-run campaign. If that
smoke misses either unchanged RTF threshold, the candidate cannot proceed even
when the isolated optimization met its keep threshold.

## Consequences

The design removes repeated exhaustive work while retaining fail-closed
structural validation and physics-step contact fidelity. It adds explicit
tests for event selection, locked identity validation, interval-union
overflow, fatal latching, and reset clearing. It also removes one redundant
high-rate DDS reader from each of the two Python observers while retaining
rclpy simulated time and repository clock evidence in the same ordered
callback.

The collector change alters the collector-configuration and Phase 3 source-tree
bindings; the scenario change alters its installed-module source binding and
the Phase 3 source-tree binding; this ADR alters both the source-configuration
and source-tree bindings. The package manifests now declare the direct
`rcl_interfaces` dependency used by the read-only descriptor and parameter
rejection callback, and the dependency-license inventory records its audited
Jazzy manifest. There is no public interface change, no apt-package addition,
and no generated manifest is rewritten in this change. Existing build
bindings, runtime gates, profiles,
positive controls, and smokes remain immutable; a clean commit must regenerate
the applicable bindings and hashes before new live evidence is collected.

No existing result becomes valid, and no release or Phase 5 claim is expanded.
Until the required profile, positive control, and fresh candidate smoke pass,
the only supported statement is that the optimization is source-frozen and
statically verified; a live performance improvement is unproven. Even after
qualification, a causal claim that separates the locked-validation gain from
the no-copy-union gain still requires the paired A/B experiment above.

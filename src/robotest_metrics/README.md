# robotest_metrics

`robotest_metrics` is RoboTest Lab's observer-only Phase 3 evidence package. It
collects bounded message prefixes, computes metrics with pure functions, and
writes the one canonical per-run benchmark verdict. It never publishes a goal,
velocity command, fault command, lifecycle transition, or scenario action.

## Live capture

Start the collector before the evidence-ready gate and give it stale-free
output, ready-file, and stop-file paths. The collector rejects any pre-existing
path rather than overwriting evidence:

```bash
ros2 run robotest_metrics metrics_collector \
  --output /run/robotest/capture.json \
  --ready-file /run/robotest/metrics.ready.json \
  --stop-file /run/robotest/metrics.stop \
  --contact-progress-file /run/robotest/contact-progress.json \
  --wall-timeout-s 360 \
  --ros-args -r __ns:=/robotest
```

The runner waits for the atomically written ready file, executes the scenario,
then waits for a retained public contact snapshot strictly beyond
`T_terminal + 0.25 s` and its atomic progress acknowledgement before creating
the stop file. READY is emitted only after the collector has observed a positive `/clock`
and retained a subsequent authoritative public contact snapshot. Contact
callbacks that arrive before the first positive `/clock` callback are counted in READY
and discarded as a pre-evidence prefix; they are never assigned a fabricated
delivery-clock value. The collector writes `capture.json` atomically even when
its wall deadline expires. Exit codes are:

| Exit | Meaning |
| ---: | --- |
| 0 | Stop file observed and every collector remained within capacity |
| 20 | Bounded wall deadline expired; partial capture is diagnostic only |
| 21 | A prefix/plan/contact capacity overflowed; prefix is diagnostic only |
| 22 | Required capture artifact could not be finalized |
| 23 | ROS runtime ended before the requested stop |

Every stream retains its first valid observations and never overwrites them.
The hard limits are 8,192 ground-truth samples; 8,192 per TF edge and odometry
stream; 2,048 per scan stream; 4,096 final commands; 1,024 plans and 65,536
total plan poses; 8,192 authoritative public contact snapshots and 32,768
normalized snapshot records;
4,096 world-statistics samples; 1,024 state transitions; and 512 fault events.
The clock uses a constant-space summary. UTF-8 strings are limited to 4,096
bytes. The first overflow preserves the prefix and fails closed.

## Offline analysis

The scenario/suite runner creates a schema-valid analysis request containing
immutable identity/targets, the UUID-bound mission outcome, the independently
hashed scenario-controller result, the complete capture, and strict
orchestrator evidence for provenance, resources, isolation, cleanup, and
artifact limits. Every Phase 3 request also carries the reset/preload/arm/
teardown fault protocol, even when its schedule is empty, plus hash-bound
collision qualification:

```bash
ros2 run robotest_metrics metrics_analyze \
  --trial-context /run/robotest/trial-context.json \
  --input /run/robotest/analysis-request.json \
  --output-dir /run/robotest/result
```

The benchmark orchestrator must atomically create the canonical trial context
before any trial mutation. Its producer is exactly
`robotest_phase3/benchmark_orchestrator`; it binds the ordered identity, all
target hashes, and intended wall timeout. For a normal attempt its `failure`
is null. When an upstream launch, readiness, mission, capture, shutdown, or
cleanup failure prevents an analysis request, `failure` is the exact bounded
machine evidence object `{stage,kind,exit_code,wall_timed_out,reason,
evidence_sha256}` and `--input` is omitted. If a supplied analysis request is
missing, malformed, schema-invalid, context-mismatched, or analytically
invalid, the analyzer itself converts that observation into the same canonical
infrastructure-failure result. The runner never authors a run verdict.

The command writes canonical `run-result.json`, its exact flattened one-row
`run-result.csv` projection, source-hash-bound `report.md` and `report.html`,
up to four PNG charts, `run-artifacts.manifest.json`, and the exact detached
sidecar `run-artifacts.manifest.json.sha256`. The manifest hashes and sizes
every output except itself and its sidecar; its sidecar is exactly
`<64 lowercase hex>  run-artifacts.manifest.json\n`. The analyzer then rereads
the manifest, sidecar, canonical result, CSV projection, hashes, path set, and
caps before returning. This avoids any result self-hash or predicted-size
cycle. JSON null becomes a blank CSV field. Reports and charts read only the
finalized canonical JSON. Exit `0` is a Phase 3 PASS, `30` is a valid canonical
FAIL, `31` is artifact-finalization failure, `32` is a schema-valid canonical
infrastructure FAIL written at the ordered trial index, `33` rejects stale
output artifacts without overwriting them, and `34` means no verdict could be
safely composed because the immutable trial context or invocation was invalid.

Orchestrator input artifact evidence covers finalized prerequisites only. Its
`artifacts` object binds a canonical prerequisite-manifest hash, prerequisite
count/total/maximum sizes, finalized/checksum/cap booleans, and finalized
runtime stdout/stderr sizes. It never predicts metrics JSON, CSV, report, PNG,
manifest, or final output-directory sizes. Output integrity belongs exclusively
to the metrics-authored manifest above.

The result follows the six-object contract:

```json
{"events":[],"identity":{},"measurements":{},"quality":{},"targets":{},"verdict":{}}
```

`run-result.json` is the sole per-trial benchmark verdict. Required unavailable
metrics are null with a reason. It uses exact ground-truth boundary
interpolation, cumulative first-valid-per-feedback-leg planning, micrometre
geometry hashes, per-leg replan chains, coverage-qualified contact episodes,
exact-stamp TF composition, conservative 1.0 s recovery windows, unpaused RTF
intervals, exact Scenario 5 `T_validated * inverse(T_raw)` odometry-drift
checks, capture-derived Scenario 3/4 final-command safety gates, and
nearest-rank percentiles. A component PASS never authorizes the benchmark:
identity/hash mismatch, warm state, source mutation, dirty Git, missing
full-lifetime resource samples, RSS above 6 GiB, wrong affinity, QoS/graph/
namespace isolation failure, timeout, orphan, checksum failure, or artifact-cap
failure makes the canonical result fail.

Collision acceptance is enabled only when the request binds the exact coverage
manifest and canonical `contact_control_driver` result. The control must prove
the frozen ground-truth start pose, exact robot/wall pair, exactly one closed
counterpart episode, bounded traces, stop within 0.10 s, final zero, release,
actor cleanup, graph isolation, and no overflow. Separate positive-control and
benchmark provenance objects must match for bridge, contact configuration,
coverage manifest, rendered SDF, robot description, world, and collector
hashes. External checksum, collector reconciliation, and owned-process-group
shutdown gates are mandatory.

Scenario 4 lifecycle freshness is collected by a dedicated bounded sampler.
The schedule contains one to 96 strictly increasing simulation stamps and the
sampler queries the exact nine Nav2 `GetState` services using relative names:

```bash
ros2 run robotest_metrics metrics_lifecycle_sampler \
  --schedule /run/robotest/lifecycle-schedule.json \
  --output /run/robotest/lifecycle-snapshot.json \
  --ready-file /run/robotest/lifecycle.ready.json \
  --stop-file /run/robotest/lifecycle.stop \
  --wall-timeout-s 360 \
  --request-timeout-s 2 \
  --ros-args -r __ns:=/robotest
```

READY is atomic and is emitted only after all nine services are discovered.
The sampler retains at most 864 request records and exposes only successful
ordered samples to analysis. Its `quality.complete` gate, schedule hash, run ID,
stop reason, counters, and zero-overflow proof are mandatory; hand-assembled
lifecycle snapshots are not accepted.

To regenerate only derived reports:

```bash
ros2 run robotest_metrics metrics_report \
  --input /run/robotest/result/run-result.json \
  --output-dir /run/robotest/result
```

When reports are regenerated in the canonical result directory, the existing
bundle is verified first and the non-circular manifest/sidecar are refreshed.
Aggregation verifies every input bundle manifest and sidecar before accepting
its `run-result.json`.

After 15 clean, cold-stack run results exist in exact suite order, aggregate
them without replacement or warm-state filtering:

```bash
ros2 run robotest_metrics metrics_aggregate \
  --input scenario1-rep0/run-result.json \
  --input scenario1-rep1/run-result.json \
  --input scenario1-rep2/run-result.json \
  ... \
  --input scenario5-rep2/run-result.json \
  --metric measurements.completion_time_sim_s \
  --metric measurements.path_efficiency \
  --output-dir /run/robotest/aggregate
```

The command requires exactly 15 canonical sources, a clean common Git SHA,
explicit cold-stack proof, unique run/domain/partition identities, scenario
order `1..5 × repetitions 0..2`, and matching global/scenario hashes. Failed
and null runs remain in each denominator. Numeric median, p5, and p95 use the
contract's deterministic nearest-rank definitions.

## Test

```bash
colcon build --packages-select robotest_metrics --symlink-install --parallel-workers 4
colcon test --packages-select robotest_metrics --parallel-workers 4
colcon test-result --test-result-base build/robotest_metrics --verbose
```

Copyright 2026 Hasan Ahmed. Licensed under Apache-2.0.

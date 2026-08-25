# ADR 0002: CPU-Friendly Phase 1 Physics

- Status: Accepted for Phase 1
- Date: 2026-08-25
- Decision owners: RoboTest Lab simulation gate

## Context

Phase 1 needs a deterministic, resource-bounded headless simulation on the
verified WSL2 development host. The world contains a primitive differential
drive robot, a 360-ray 5 Hz LiDAR, an IMU, wheel odometry, ground truth, and
contact sensing. The current renderer is Mesa `llvmpipe`, so graphics and
sensor rendering consume CPU resources rather than a dedicated GPU.

A 1 ms physics step at 1,000 updates per second is unnecessarily aggressive
for this Phase 1 model and host. The acceptance targets are based on observable
rates, motion, resource use, and real-time-factor windows; they do not require
a 1 kHz physics loop.

Earlier development evidence is not a controlled A/B comparison. The runtime
probe's command publisher and calculated-RTF method also changed before the
passing run, so no isolated causal improvement is claimed for the physics
change.

## Decision

Use this Phase 1 world configuration:

| Setting | Value |
| --- | ---: |
| Maximum physics step | 0.002 simulation seconds |
| Requested update rate | 500 Hz |
| Requested real-time factor | 1.0 |

Keep the frozen Phase 1 acceptance targets unchanged. In particular, retain
the 360-ray 5 Hz LiDAR configuration and evaluate sensor rates in simulation
time. Continue applying the established six-CPU affinity and bounded build
parallelism rather than changing global WSL resource settings.

This decision is scoped to the simple Phase 1 robot and world. Revisit the
step size when later phases add Nav2 dynamics, denser collision behavior, or
other systems whose numerical stability or controller behavior may depend on
finer integration.

## Evidence

The seeded bounded headless run `20260825T200725Z-1333` passed with this
configuration and recorded `simulator_seed=42` with seed status
`fixed_simulator_seed_recorded`. See the
[tracked result](../results/phase-1/20260825T200725Z-1333.md).

Measured observations from that run include:

- calculated RTF median `0.9999416661302953` and p5
  `0.9162105536383762`, above frozen targets `0.80` and `0.50`;
- raw and validated LiDAR rates of `5.0` simulation Hz, above the `4.0 Hz`
  minimum;
- raw and validated odometry rates of `20.0` simulation Hz, above the
  `10.0 Hz` minimum;
- raw and validated IMU rates of `46.87854710556186` and
  `46.82179341657208` simulation Hz, above the `20.0 Hz` minimum;
- `0.29619887895167835 m` displacement followed by a recorded final-zero
  command sequence;
- peak launch-process-group RSS of `795,476 KiB` (`776.83 MiB`), below the
  `6 GiB` limit; and
- successful build, model validation, QoS/stamp/trace runtime probes, and
  test-result gate: 103 tests, zero errors or failures, and 10 cppcheck-version
  skips.

The contacts endpoint existed but emitted no messages, so this run does not
establish collision absence. The run-local `SHA256SUMS` manifest and persisted
sidecar agree on all 45 hashed artifacts. Git was dirty at run start; these are
verified development observations, not release or repeated benchmark evidence.

The run's OGRE log identifies `llvmpipe (LLVM 20.1.2, 256 bits)` and an
`EGL_MESA_device_software` device. This is direct evidence of software
rendering for that run, not hardware acceleration.

## Consequences

### Positive

- The physics loop leaves more CPU time for software-rendered sensors, ROS 2
  bridges, the fault proxy, and evidence collection.
- The passing configuration stays within the measured CPU-affinity and memory
  envelope without a global `.wslconfig` change.
- The observable Phase 1 target set remains intact.

### Costs and follow-up

- A 2 ms step provides less temporal resolution than a 1 ms step.
- Phase 2 must re-check wheel motion, odometry, contacts, controller behavior,
  and RTF under its integrated Nav2 workload.
- A controlled benchmark with source revision, dirty state, target-set hash,
  and repeated trials is still needed before making a general performance
  claim.
- P1-07 was passed separately in bounded RViz run
  `rviz-20260825T200856Z-2873`, also with simulator seed `42`; that visual
  result does not alter or strengthen
  the physics-performance attribution.
- Fast DDS reported live history and depth as `UNKNOWN` and `0`; bounded queue
  depths remain a static/source-and-test proof rather than a complete live
  introspection result.
- TF endpoint owners and required edges were observed, but Jazzy `rclpy` did
  not provide per-message publisher-GID attribution for each TF edge.

## Rejected alternatives

- Keep 1 ms / 1,000 Hz solely as a conservative default: rejected for Phase 1
  because the model's frozen observable targets do not require that update
  rate and the host is software-rendered.
- Relax the frozen RTF or topic-rate targets: rejected because that would make
  the gate easier instead of improving the runtime configuration.
- Reduce LiDAR ray count or update rate: rejected because the 360-ray 5 Hz
  sensor is part of the Phase 1 contract and passed with the selected physics.
- Change global WSL limits: deferred because it affects unrelated workloads
  and is unnecessary for this passing bounded run.

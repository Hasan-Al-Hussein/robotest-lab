# robotest_scenarios

This ROS 2 Jazzy package owns bounded Phase 3 scenario interaction evidence.
It does not decide a benchmark verdict: the suite compositor consumes the
component result after the mission, fault, and metrics producers are terminal.

The controller binds exactly one post-readiness `FollowWaypoints` UUID and its
positive immutable action-status stamp. Scenario 2 performs the frozen one-shot
static insertion and replan proof. Scenario 3 pre-spawns the parked actor and
performs the exact 121-sample simulation-time trajectory. Scenarios 1, 4, and 5
emit bounded no-actor and cleanup evidence.

```bash
ros2 run robotest_scenarios scenario_controller \
  --scenario scenarios/phase3_s2_static_obstacle.yaml \
  --output artifacts/run/scenario-result.json \
  --ready-file artifacts/run/scenario-ready.json \
  --run-id RUN --candidate-id CANDIDATE --repetition-index 0 --suite-index 3
```

The isolated positive-control driver is the sole `cmd_vel` publisher in its
fixture and must run without Nav2 or the collision monitor:

```bash
ros2 run robotest_scenarios contact_control_driver \
  --output artifacts/control/contact-control-result.json \
  --ready-file artifacts/control/contact-control-ready.json \
  --arm-file artifacts/control/contact-control-arm.json \
  --armed-file artifacts/control/contact-control-armed.json \
  --command-progress-file artifacts/control/command-progress.json \
  --run-id CONTROL \
  --coverage-manifest config/collision-coverage.yaml
```

The driver completes preparation and emits READY only after a positive `/clock`
sample and a later authoritative contact snapshot have both been observed.
Contact callbacks that arrive before the first positive `/clock` are counted as
a discarded pre-evidence prefix; post-clock snapshots retain the full active
safety checks. READY also requires graph prerequisites to pass for 100 ms. One
missing observation-source publisher query is treated as a graph discovery
diagnostic; the driver fails when the same source is missing in another
observation at least 100 ms later. Command ownership, forbidden nodes, and
contact endpoint identity or QoS changes remain immediately fatal.

READY means prepared and stationary, not permission to move. The driver keeps
spinning and checking its graph while the runner completes the positive
runtime gate, verifies its canonical PASS artifact, and confirms that the gate
process group is empty. Only then may the runner atomically write the exact
run-bound ARM artifact. The driver validates its canonical encoding, exact
schema and types, run ID, producer, READY hash, and runtime-gate hash; records a
pre-arm clock baseline; and waits for a strictly newer positive `/clock`
sample. It then requires exactly two subscriptions matched to its publisher,
publishes one untracked safe-zero probe, and waits for the collector's
canonical first-retained-command progress artifact. The driver binds that
artifact's hash, schema, run ID, topic, single zero-valued command, callback
time, and simulation stamp to the probe. The two steady-time partial orders and
the absolute 100 ms simulation-time bracket must hold before the driver
atomically writes its ARMED acknowledgement. No nonzero command may be
published before that acknowledgement. A best-effort zero remains permitted
for fail-safe cleanup before arm.

The retained collector command stream begins with that distinct zero probe and
then contains exactly one ordered observation for every component command; its
cardinality is therefore the component trace length plus one, with no leading,
interleaved, duplicated, or trailing observations.

Missing, stale, oversized, symlinked, malformed, noncanonical, wrongly bound,
or hash-mismatched handshake artifacts fail closed, as does any ordering
violation. The existing 30 s steady-wall deadline bounds the complete fixture,
including the pre-arm wait, and is never reset by READY, ARM, or ARMED.

Both executables use relative RoboTest names under their launch namespace.
The standardized Gazebo `/clock` topic is the single global-name exception.
Artifacts are canonical UTF-8 JSON written atomically with a sibling
`.sha256` sidecar. Stable process exits are: `0` complete, `2` validation,
`20` wall timeout, `21` protocol/data quality, `22` ROS infrastructure,
`23` artifact, and `24` scenario interaction failure.

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
  --run-id CONTROL \
  --coverage-manifest config/collision-coverage.yaml
```

Both executables use relative RoboTest names under their launch namespace.
The standardized Gazebo `/clock` topic is the single global-name exception.
Artifacts are canonical UTF-8 JSON written atomically with a sibling
`.sha256` sidecar. Stable process exits are: `0` complete, `2` validation,
`20` wall timeout, `21` protocol/data quality, `22` ROS infrastructure,
`23` artifact, and `24` scenario interaction failure.

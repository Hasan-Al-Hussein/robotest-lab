# robotest_missions

`robotest_missions` provides the bounded Phase 2 waypoint-mission client for
RoboTest Lab. It validates a strict mission document, sends exactly one
`nav2_msgs/action/FollowWaypoints` goal through a direct `rclpy` action client,
and writes a canonical JSON result plus a reconciled one-row CSV projection.

The internal action name is relative (`follow_waypoints`), so a node launched in
the `robotest` namespace resolves it to `/robotest/follow_waypoints`. The runner
uses simulation time for the mission deadline and an independent monotonic wall
escape. Goal response, cancellation acknowledgement, and post-cancel result
waits are all bounded.

Before dispatch, the runner requires two positive, advancing ROS-time
observations. The `SendGoal` response stamp is retained separately as raw
transport evidence and may be zero on Jazzy. The authoritative acceptance T0 is
the positive `goal_info.stamp` from the exact UUID-matched
`follow_waypoints/_action/status` entry, observed with the standard action-status
QoS. Failure to obtain that proof triggers bounded cancellation and terminal
result collection before the runner exits with an infrastructure error. Mission
deadline arithmetic uses a separate positive client-local clock observation, so
server/client observation skew cannot create a false clock-regression result.
Repeated status entries must retain the same immutable GoalInfo stamp and a
valid action status code; these invariants remain gated through terminal result
receipt. A nonzero raw response stamp must equal the status T0. The client clock
must catch up to T0 within a wall-time bound, and a successful artifact is
rejected unless its terminal stamp is at or after T0.

The feedback evidence trace has a fixed capacity of 4096 observations. The
canonical JSON and CSV record that capacity, whether overflow occurred, and the
number of observations rejected after the bound. The first excess observation
fails the mission closed: the runner performs its bounded cancellation and
terminal-result protocol and cannot report success.

## Run contract

```bash
ros2 run robotest_missions mission_runner \
  --mission scenarios/phase2_baseline.yaml \
  --json artifacts/evidence/phase2/mission-result.json \
  --csv artifacts/evidence/phase2/mission-result.csv \
  --ros-args -r __ns:=/robotest
```

The baseline has three ordered map-frame goals: `(-2.0, -3.5)`, `(0, 0)`,
and `(0, 3.5)`. It requests `number_of_loops=0` and `goal_index=0`. Fault
schedule and fault seed are intentionally null; simulator and mission seeds are
fixed at 42.

| Exit | Meaning |
|---:|---|
| 0 | Action succeeded without a Nav2 error or missed waypoint |
| 10 | CLI or mission validation failed |
| 20 | Action infrastructure or cancellation protocol failed |
| 21 | Goal was rejected |
| 22 | Goal aborted, or terminal result reported an error/missed waypoint |
| 23 | Goal was canceled outside the runner's deadline path |
| 24 | Simulation deadline or steady-wall escape completed bounded cancellation |
| 25 | Artifact write or JSON/CSV reconciliation failed |

This runner proves only the bounded mission/action integration result. Collision
count, actual path length, path efficiency, and the full Scenario 1 acceptance
verdict remain explicitly `NOT_EVALUATED` in Phase 2. Those independent metrics
are deferred to Phase 3 and the planned `robotest_metrics` package; the Phase 2
verifier does not supply them.

## Test

```bash
colcon build --packages-select robotest_missions --parallel-workers 4
colcon test --packages-select robotest_missions --parallel-workers 4
colcon test-result --all --verbose
```

Copyright 2026 Hasan Ahmed. Licensed under Apache-2.0.

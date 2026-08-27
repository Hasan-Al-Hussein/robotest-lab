# RoboTest Lab Topic and TF Contract

Status: **Normative contract, revised for Phase 3**
Default ROS namespace: `/robotest`
Target platform: ROS 2 Jazzy, Gazebo Harmonic, Nav2

This document freezes the communication boundary before implementation. Code
uses relative names and receives the `robotest` namespace from launch; the
absolute names below describe the default single-robot deployment.

## Non-negotiable invariants

1. Gazebo data enters a raw plane. Autonomy consumes only the proxy output
   plane.
2. Ground truth, contact truth, and world statistics are validation-only.
   AMCL, Nav2, recovery behaviors, and command generation must never subscribe
   to `/robotest/validation/*`.
3. The fault proxy is always present. With no active fault, it is a transparent
   relay and remains the sole publisher of `odom -> base_footprint`.
4. Every queue is bounded. `KEEP_ALL` is prohibited.
5. Gazebo is the sole source of `/clock`. All ROS nodes that participate in
   simulation set `use_sim_time=true`.
6. Ground truth is data, not TF. It must never create a second
   `map -> odom` or `odom -> base_footprint` transform.
7. Exactly one component owns the final velocity command at a time.

## Topic contract

QoS values are requested contracts, not assumptions about defaults. Phase 1
must compare them with the live endpoints using `ros2 topic info -v`.

| Default topic | ROS type | Direction and owner | Allowed consumers | QoS contract |
| --- | --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo -> ROS through a one-way `ros_gz_bridge` | Every simulation-time ROS node | BEST_EFFORT, VOLATILE, KEEP_LAST(1) |
| `/robotest/map` | `nav_msgs/msg/OccupancyGrid` | Nav2 map server | AMCL, global costmap, visualization, and evidence probes | RELIABLE, TRANSIENT_LOCAL, KEEP_LAST(1) |
| `/robotest/raw/scan` | `sensor_msgs/msg/LaserScan` | Gazebo -> bridge | Fault proxy; validation probes only | BEST_EFFORT, VOLATILE, KEEP_LAST(5) |
| `/robotest/scan` | `sensor_msgs/msg/LaserScan` | Fault proxy | Nav2 obstacle layers, collision monitor, metrics | BEST_EFFORT, VOLATILE, KEEP_LAST(5) |
| `/robotest/raw/odom` | `nav_msgs/msg/Odometry` | Gazebo differential-drive odometry -> bridge | Fault proxy; validation probes only | BEST_EFFORT, VOLATILE, KEEP_LAST(10) |
| `/robotest/odom` | `nav_msgs/msg/Odometry` | Fault proxy | Nav2 consumers, collision monitor, metrics | BEST_EFFORT, VOLATILE, KEEP_LAST(10) |
| `/robotest/raw/imu` | `sensor_msgs/msg/Imu` | Gazebo -> bridge | Fault proxy; validation probes only | BEST_EFFORT, VOLATILE, KEEP_LAST(10) |
| `/robotest/imu` | `sensor_msgs/msg/Imu` | Fault proxy | Autonomy consumers and metrics | BEST_EFFORT, VOLATILE, KEEP_LAST(10) |
| `/robotest/navigation/plan` | `nav_msgs/msg/Path` | Nav2 planner server | Metrics and visualization | RELIABLE, VOLATILE, KEEP_LAST(5) |
| `/robotest/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Nav2 controller server only | Velocity smoother | RELIABLE, VOLATILE, KEEP_LAST(1) |
| `/robotest/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | Velocity smoother only | Collision monitor | RELIABLE, VOLATILE, KEEP_LAST(1) |
| `/robotest/cmd_vel` | `geometry_msgs/msg/Twist` | Collision monitor only | Gazebo command bridge and independent metrics observer | RELIABLE, VOLATILE; actuator bridge KEEP_LAST(1), metrics observer KEEP_LAST(4096) |
| `/robotest/cmd_vel_behavior_unused` | `geometry_msgs/msg/Twist` | Isolated Nav2 behavior-server output | None; this topic is not bridged or connected to an actuator path | RELIABLE, VOLATILE, KEEP_LAST(1) |
| `/robotest/collision_monitor_state` | `nav2_msgs/msg/CollisionMonitorState` | Collision monitor | Metrics and evidence probes | RELIABLE, VOLATILE, KEEP_LAST(10) |
| `/robotest/validation/ground_truth` | `nav_msgs/msg/Odometry` | Gazebo model truth -> bridge | Metrics, validation tests, evidence recorder | RELIABLE, VOLATILE, KEEP_LAST(10) |
| `/robotest/internal/raw_contacts` | `ros_gz_interfaces/msg/Contacts` | Sole `/robotest/internal/contact_aggregate` Gazebo source -> sole `parameter_bridge` publisher | Compiled `contact_stream_gate` only | RELIABLE, VOLATILE, KEEP_LAST(64) |
| `/robotest/validation/contacts` | `ros_gz_interfaces/msg/Contacts` | Sole compiled `contact_stream_gate` publisher; authoritative delivered active-pair snapshots | Metrics and validation tests | RELIABLE, VOLATILE, KEEP_LAST(10) |
| `/robotest/validation/world_stats` | `ros_gz_interfaces/msg/WorldStatistics` | Gazebo world statistics -> bridge | Metrics and resource recorder | RELIABLE, VOLATILE, KEEP_LAST(10) |
| `/robotest/faults/events` | `robotest_interfaces/msg/FaultEvent` | Fault proxy | Metrics and evidence recorder | RELIABLE, VOLATILE, KEEP_LAST(100) |

The bridge for `/clock`, sensors, validation truth, private raw contacts, and
world statistics is Gazebo-to-ROS only. The stock seven-writer Contact system
is absent. A source-bound Gazebo plugin observes every physics step and emits
one nonempty complete 20 ms interval aggregate; raw contacts are never bridged
directly to the public validation topic. The gate consumes one aggregate per
stamp, publishes complete nonempty delivered-state snapshots with strictly
increasing stamps, and fails closed on structural, semantic, capacity, or
liveness violations. The final velocity bridge is ROS-to-Gazebo only. Bidirectional
bridges are not used where direction is known.

### Command ownership by phase

- Phase 1 bounded-motion verification uses one test command publisher. It must
  publish a final zero command and exit before another command source starts.
- Phase 2 implements the exact chain approved in
  [ADR 0004](../decisions/0004-phase2-command-ownership.md):
  `controller_server -> cmd_vel_nav -> velocity_smoother ->`
  `cmd_vel_smoothed -> collision_monitor -> cmd_vel ->` the one-way Gazebo
  bridge.
- The behavior server is Wait-only. Its generic command output is remapped to
  `cmd_vel_behavior_unused`, which has no subscribers. Spin, BackUp,
  DriveOnHeading, AssistedTeleop, and other motion recoveries are prohibited
  until a prior arbitration decision changes this contract.
- The collision monitor alone publishes `cmd_vel`; the controller alone
  publishes `cmd_vel_nav`; the smoother alone publishes `cmd_vel_smoothed`.
- Multiple compatible publishers on `/robotest/cmd_vel` are a gate failure.

Jazzy's default Nav2 command type is `geometry_msgs/msg/Twist`. Any later
decision to enable stamped commands must change the controller, arbiter,
bridge, tests, and this contract atomically.

### Phase 2 action and lifecycle interface

The mission runner is a direct `nav2_msgs/action/FollowWaypoints` client of
`/robotest/follow_waypoints`; it does not use a convenience navigator that can
hide goals or terminal results. Before submitting the three-waypoint baseline,
the verifier requires active lifecycle state for `map_server`, `amcl`,
`planner_server`, `controller_server`, `behavior_server`, `bt_navigator`,
`waypoint_follower`, `velocity_smoother`, and `collision_monitor`. Collision
monitor activation is checked again immediately before the goal. Action,
cancellation, and terminal-result waits retain steady wall-clock deadlines.

Phase 3 adds two passive evidence consumers before mission launch:
`/robotest/metrics_collector` and `/robotest/scenario_controller`. They observe
the raw FollowWaypoints feedback and status endpoints without owning goal
submission, but Jazzy's action-graph projection reports those endpoint bundles
as action clients. The graph probe retains those projected participants as
diagnostics, but derives goal-capable ownership from the exact hidden
`/_action/send_goal` service-client endpoint. The pre-mission graph gate
therefore requires zero goal-capable clients. After mission launch it requires
exactly `/robotest/mission_runner`; any missing, mistyped, or additional goal
client remains a gate failure.

## Fault-control interfaces

The Phase 3 interface package owns:

- `robotest_interfaces/msg/FaultSpec`
- `robotest_interfaces/msg/FaultEvent`
- `robotest_interfaces/srv/PreloadFaultSchedule`
- `robotest_interfaces/srv/ArmFaultSchedule`

The Phase 3 proxy exposes:

| Service | Type | Contract |
| --- | --- | --- |
| `/robotest/faults/preload_schedule` | `robotest_interfaces/srv/PreloadFaultSchedule` | Recompute the canonical schedule hash, atomically validate and prepare at most 16 bounded specifications, or idempotently replay the exact committed content; PREPARED remains inert |
| `/robotest/faults/arm_schedule` | `robotest_interfaces/srv/ArmFaultSchedule` | Atomically bind the exact prepared hash/generation to one UUID-matched accepted goal and authoritative `T0`, with at least 0.50 s margin before the first fault |
| `/robotest/faults/reset` | `std_srvs/srv/Trigger` | Disable every fault, clear schedule state, reset deterministic generators, and confirm the pass-through state |

Generated service/action QoS remains the Jazzy default reliable profile unless
live endpoint inspection proves an incompatibility. Service calls have bounded
availability and response timeouts; no callback waits synchronously on a
service future.

Each `FaultSpec` conveys a stable fault ID, target stream, supported mode,
simulation offset from `T0`, duration, deterministic seed, and mode-specific
parameters. The proxy independently canonicalizes the bounded sequence and
recomputes its SHA-256; a rejected request cannot mutate committed state.

[ADR 0005](../decisions/0005-phase3-deterministic-fault-protocol.md) freezes
the exact `RESET -> PREPARED -> ARMED` state machine, generation and replay
policy, canonical bytes, supported modes, event schema, arming margin, drift
math, and failure behavior. The legacy `faults/load_schedule` service is not
allowed in a Phase 3 runtime graph.

## Simulation-time semantics

The following Phase 3 semantics apply exactly as specified by ADR 0005:

1. The complete schedule is preloaded, validated, and inert before the first
   navigation goal.
2. After the action server accepts the goal, the mission runner uses the goal
   handle's UUID and the immutable, exact UUID-matched
   `GoalStatusArray.goal_info.stamp` as `T0`, then arms the exact prepared
   generation before its earliest activation boundary. The installed Jazzy
   `rclcpp_action` SendGoal response stamp is zero and is retained only as
   unavailable-response provenance.
3. For an input message stamped `t`, the proxy applies a specification only
   when `T0 + start_offset <= t < T0 + start_offset + duration`.
4. Configured activation time and actual first-affected-message time are both
   recorded. Deactivation is the first unaffected message after the interval.
5. A paused simulation pauses the schedule. A steady wall-clock escape timeout
   remains outside the simulated-time state machine so a stopped `/clock`
   cannot hang a test.
6. Resetting a run resets all transformation state and counters while the
   per-process generation allocator remains monotonic. Seeds, canonical bytes,
   schedule hash, generation, event/input sequences, and raw/validated/affected
   counts are evidence fields.

Every control, action, cancellation, and reset wait has a steady wall-clock
deadline. A stopped simulation clock must not hang either the Phase 2 mission
runner or a future schedule-control operation.

LiDAR dropout means omission: raw scans continue, while matching validated
scans are not published. It must not publish an empty or stale scan. Frozen
readings and noise are separate future modes.

Odometry drift is the deterministic left-composed odom-frame offset
`T_validated = D(elapsed) * T_raw`. The same validated pose supplies the
`Odometry` message and `odom -> base_footprint` transform; stamps and frame IDs
match, while raw odometry, z, twist, and both covariance arrays remain
unchanged.

## TF contract

| Transform | Owner | Static/dynamic | Source |
| --- | --- | --- | --- |
| `map -> odom` | AMCL | Dynamic | Validated scan plus map/localization state |
| `odom -> base_footprint` | Fault proxy | Dynamic | Validated odometry, pass-through or drifted |
| `base_footprint -> base_link` | `robot_state_publisher` | Static fixed joint | URDF/Xacro |
| `base_link -> lidar_link` | `robot_state_publisher` | Static/fixed | URDF/Xacro |
| `base_link -> imu_link` | `robot_state_publisher` | Static/fixed | URDF/Xacro |
| Wheel transforms | `robot_state_publisher` from joint states | Dynamic | Joint state source |

Gazebo's original odometry TF output must be disabled. No validator publishes
ground truth on `/tf` or `/tf_static`.

The `map -> odom` edge is expected only from Phase 2, when AMCL is present.
Phase 1 proves the remaining robot-local chain and treats an absent
`map -> odom` as expected; once Phase 2 begins, absence or duplication of
that edge is a failure.

Frame conventions:

- World/map: right-handed ENU, metres and radians.
- `base_link`: x forward, y left, z up.
- `base_footprint`: planar projection of `base_link`.
- LaserScan: zero angle along sensor x, positive counter-clockwise about z.
- Validated odometry: `header.frame_id=odom`,
  `child_frame_id=base_footprint`.
- Ground truth: `header.frame_id=world`; the metrics configuration records
  the fixed `world -> map` alignment used only in validation calculations.

All dynamic transforms and measurements use simulation timestamps. Static
transforms use transient-local durability through the standard TF publisher.

## Autonomy/validation isolation

Allowed validation-topic subscribers are the metrics node, evidence recorder,
and explicitly named test probes. The following are prohibited:

- AMCL or any Nav2 node subscribing to `/robotest/validation/*`
- A launch remap from validation ground truth into odometry, pose, TF, costmap,
  planner, controller, or collision-monitor inputs
- A metric or test publishing validation truth back into the autonomy graph
- Using ground truth to decide mission success before the Nav2 action reaches a
  terminal result

Raw topics may be observed for injection verification, but only the proxy may
turn them into autonomy inputs.

## Phase 1 proof

The topic/TF portion of `scripts/verify_phase1.sh` must:

1. inspect every endpoint with `ros2 topic info <topic> -v`;
2. prove no `KEEP_ALL` endpoint and no incompatible QoS;
3. prove exactly one publisher for `/robotest/cmd_vel`;
4. enumerate `/tf` publishers and prove one owner for every edge required in
   Phase 1;
5. compare raw and validated messages in no-fault pass-through mode;
6. fail if any autonomy node subscribes to a validation topic; and
7. save the graph, QoS, rate, timestamp, and TF evidence under the current run
   ID.

Raw-versus-validated dropout behavior is a Phase 3 proof after that fault mode
exists.

## Phase 2 proof

The topic/TF portion of `scripts/verify_phase2.sh` must:

1. prove the map is available, required Nav2 lifecycle nodes are active, and
   both `map -> odom` and the Phase 1 robot-local TF chain are present;
2. prove the exact ADR 0004 publisher and connected-subscriber sets for
   `cmd_vel_nav`, `cmd_vel_smoothed`, and `cmd_vel` at one live observation
   point, plus no subscriber on `cmd_vel_behavior_unused`;
3. compare command QoS with this contract, reject `KEEP_ALL`, and report
   unknown live history/depth honestly while retaining the static bounded-queue
   proof;
4. retain a bounded trace through all connected command stages, including a
   final command within the frozen zero tolerance;
5. fail on any autonomy subscriber to `/robotest/validation/*`, any extra
   command owner, duplicate required TF owner, or mixed simulation time;
6. preserve the no-fault three-waypoint action result, waypoint progress,
   mission JSON/CSV, seed, resource/RTF evidence, source hashes, and process
   cleanup evidence; and
7. label full Scenario 1 collision/path/efficiency/repeated-trial metrics as
   deferred to Phase 3 rather than copying target values into measurements.

The Phase 2 baseline does not call fault preload or arm operations and does not
claim that ADR 0003's future state machine exists.

## Phase 3 proof

The topic, service, and TF portion of `scripts/verify_phase3.sh` must:

1. reject the legacy `faults/load_schedule` service and require the exact
   PRELOAD, ARM, reset, event-topic, and message/service type contracts;
2. prove every evidence subscriber is ready before preload, PREPARED streams
   remain pass-through, the UUID/T0-bound arm event precedes the earliest fault
   by at least 0.50 simulation seconds, and reset restores pass-through;
3. retain exact endpoint/QoS/owner evidence for raw and validated scan,
   odometry, fault events, collision-monitor state, command stages, validation
   truth, and every required TF edge, rejecting `KEEP_ALL` and incompatible
   reliability or durability;
4. reconcile the monotonically ordered FaultEvent version-2 control/data
   events and raw/validated/affected counters with observed streams;
5. prove LiDAR omission and restoration without an empty/stale substitute;
6. prove raw odometry is unchanged and the same left-composed drifted pose and
   stamp appear in validated odometry and `odom -> base_footprint`;
7. enforce the exact collision-monitor `source_timeout=0.60`, final-command
   safety, validation/autonomy isolation, and bounded evidence queues; and
8. preserve unique domains/partitions/process groups plus full cleanup and
   source/install/provenance bindings for every cold-stack trial.

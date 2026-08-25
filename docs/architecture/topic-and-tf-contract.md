# RoboTest Lab Topic and TF Contract

Status: **Phase 0 normative contract**
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
| `/robotest/raw/scan` | `sensor_msgs/msg/LaserScan` | Gazebo -> bridge | Fault proxy; validation probes only | BEST_EFFORT, VOLATILE, KEEP_LAST(5) |
| `/robotest/scan` | `sensor_msgs/msg/LaserScan` | Fault proxy | Nav2 obstacle layers, collision monitor, metrics | BEST_EFFORT, VOLATILE, KEEP_LAST(5) |
| `/robotest/raw/odom` | `nav_msgs/msg/Odometry` | Gazebo differential-drive odometry -> bridge | Fault proxy; validation probes only | BEST_EFFORT, VOLATILE, KEEP_LAST(10) |
| `/robotest/odom` | `nav_msgs/msg/Odometry` | Fault proxy | Nav2 consumers, collision monitor, metrics | BEST_EFFORT, VOLATILE, KEEP_LAST(10) |
| `/robotest/raw/imu` | `sensor_msgs/msg/Imu` | Gazebo -> bridge | Fault proxy; validation probes only | BEST_EFFORT, VOLATILE, KEEP_LAST(10) |
| `/robotest/imu` | `sensor_msgs/msg/Imu` | Fault proxy | Autonomy consumers and metrics | BEST_EFFORT, VOLATILE, KEEP_LAST(10) |
| `/robotest/navigation/plan` | `nav_msgs/msg/Path` | Remapped Nav2 planner output | Metrics and visualization | RELIABLE, VOLATILE, KEEP_LAST(5) |
| `/robotest/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Nav2 controller | Collision monitor or the configured final-command arbiter | RELIABLE, VOLATILE, KEEP_LAST(1) |
| `/robotest/cmd_vel` | `geometry_msgs/msg/Twist` | Collision monitor/final-command arbiter | Gazebo command bridge and metrics | RELIABLE, VOLATILE, KEEP_LAST(1) |
| `/robotest/validation/ground_truth` | `nav_msgs/msg/Odometry` | Gazebo model truth -> bridge | Metrics, validation tests, evidence recorder | RELIABLE, VOLATILE, KEEP_LAST(10) |
| `/robotest/validation/contacts` | `ros_gz_interfaces/msg/Contacts` | Gazebo contact sensor -> bridge | Metrics and validation tests | RELIABLE, VOLATILE, KEEP_LAST(10) |
| `/robotest/validation/world_stats` | `ros_gz_interfaces/msg/WorldStatistics` | Gazebo world statistics -> bridge | Metrics and resource recorder | RELIABLE, VOLATILE, KEEP_LAST(10) |
| `/robotest/faults/events` | `robotest_interfaces/msg/FaultEvent` | Fault proxy | Metrics and evidence recorder | RELIABLE, VOLATILE, KEEP_LAST(100) |

The bridge for `/clock`, sensors, validation truth, and world statistics is
Gazebo-to-ROS only. The final velocity bridge is ROS-to-Gazebo only.
Bidirectional bridges are not used where direction is known.

### Command ownership by phase

- Phase 1 bounded-motion verification uses one test command publisher. It must
  publish a final zero command and exit before another command source starts.
- Phase 2 onward Nav2 publishes `cmd_vel_nav`; the collision monitor or
  configured arbiter alone publishes `cmd_vel`.
- Multiple compatible publishers on `/robotest/cmd_vel` are a gate failure.

Jazzy's default Nav2 command type is `geometry_msgs/msg/Twist`. Any later
decision to enable stamped commands must change the controller, arbiter,
bridge, tests, and this contract atomically.

## Fault-control interfaces

The interface package owns:

- `robotest_interfaces/msg/FaultSpec`
- `robotest_interfaces/msg/FaultEvent`
- `robotest_interfaces/srv/LoadFaultSchedule`

The proxy exposes:

| Service | Type | Contract |
| --- | --- | --- |
| `/robotest/faults/load_schedule` | `robotest_interfaces/srv/LoadFaultSchedule` | Atomically validate and preload all specifications before mission motion |
| `/robotest/faults/reset` | `std_srvs/srv/Trigger` | Disable every fault, clear schedule state, reset deterministic generators, and confirm the pass-through state |

Generated service/action QoS remains the Jazzy default reliable profile unless
live endpoint inspection proves an incompatibility. Service calls have bounded
availability and response timeouts; no callback waits synchronously on a
service future.

Each `FaultSpec` conveys a stable fault ID, target stream, mode, simulation
offset from mission start, duration, deterministic seed, and mode-specific
parameters. The load request includes the canonical schedule hash. A rejected
schedule activates nothing.

## Simulation-time semantics

1. The mission runner samples mission start `T0` from `/clock`.
2. The complete schedule is accepted before the first navigation goal.
3. For an input message stamped `t`, the proxy applies a specification only
   when `T0 + start_offset <= t < T0 + start_offset + duration`.
4. Configured activation time and actual first-affected-message time are both
   recorded. Deactivation is the first unaffected message after the interval.
5. A paused simulation pauses the schedule. A steady wall-clock escape timeout
   remains outside the simulated-time state machine so a stopped `/clock`
   cannot hang a test.
6. Resetting a run resets all PRNG state. Seeds, schedule hash, input sequence
   counts, and affected message counts are evidence fields.

LiDAR dropout means omission: raw scans continue, while matching validated
scans are not published. It must not publish an empty or stale scan. Frozen
readings and noise are separate future modes.

Odometry drift is a deterministic SE(2) offset evaluated from simulation time.
The same offset is applied to the validated `Odometry` pose and the
`odom -> base_footprint` transform. Their stamps and frame identifiers must
match. Twist remains the raw measured twist unless a later, separately
specified fault mode changes it.

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

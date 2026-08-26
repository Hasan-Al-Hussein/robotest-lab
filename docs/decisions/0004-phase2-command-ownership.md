# ADR 0004: Phase 2 Navigation Command Ownership

- Status: Accepted and implemented for Phase 2
- Date: 2026-08-26
- Decision owners: RoboTest Lab navigation and safety gate

## Context

The [system-boundary decision](0001-system-boundaries.md) and
[topic and TF contract](../architecture/topic-and-tf-contract.md) require one
owner of the final command bridged into Gazebo. The contracted topics are:

- `/robotest/cmd_vel_nav`: upstream Nav2 command;
- `/robotest/cmd_vel`: final command consumed by the Gazebo bridge.

The installed ROS 2 Jazzy Nav2 1.3.12
`nav2_bringup/launch/navigation_launch.py` remaps both `controller_server` and
`behavior_server` command output to `cmd_vel_nav`. Its installed default
behavior tree includes `Spin` and `BackUp`, and its default behavior-server
configuration loads multiple motion behaviors. Copying those defaults would
make the current statement that the controller owns `cmd_vel_nav` false at the
endpoint level and would permit recovery motion outside the intended Phase 2
controller path.

The same installed launch and parameter files already define a useful bounded
chain: the velocity smoother consumes `cmd_vel_nav`, emits
`cmd_vel_smoothed`, and the collision monitor emits `cmd_vel`. This decision
preserves that chain while isolating unused behavior output.

The project-owned Phase 2 navigation configuration implements this chain. The
bounded headless acceptance run `20260826T010218Z-466` verified its static
wiring and live command ownership as Phase 2 development evidence.

## Decision

Use this Phase 2 command path under the `/robotest` namespace:

```text
controller_server
  -> cmd_vel_nav
  -> velocity_smoother
  -> cmd_vel_smoothed
  -> collision_monitor
  -> cmd_vel
  -> one-way ROS-to-Gazebo bridge
```

Ownership is fixed as follows:

| Topic | Connected publisher | Allowed consumer |
| --- | --- | --- |
| `/robotest/cmd_vel_nav` | `controller_server` only | `velocity_smoother` |
| `/robotest/cmd_vel_smoothed` | `velocity_smoother` only | `collision_monitor` |
| `/robotest/cmd_vel` | `collision_monitor` only | Gazebo command bridge and evidence probes |
| `/robotest/cmd_vel_behavior_unused` | isolated `behavior_server` command endpoint | none |

The collision monitor remains the sole final-command owner in every Phase 2
run. Neither the mission runner, verifier, teleoperation, controller server,
velocity smoother, behavior server, nor Gazebo may publish directly to
`/robotest/cmd_vel`.

### Behavior-server restriction

Configure `behavior_server` with only the installed
`nav2_behaviors::Wait` plugin. Remap its command output to the isolated,
relative name `cmd_vel_behavior_unused`, which resolves to
`/robotest/cmd_vel_behavior_unused`. That topic has no subscriber and is not
bridged, smoothed, multiplexed, recorded as a final command, or connected to
an actuator path.

The project-owned Phase 2 behavior tree may use bounded Wait and costmap-clear
operations. It must not contain or invoke `Spin`, `BackUp`,
`DriveOnHeading`, `AssistedTeleop`, or another motion-recovery action. A static
test rejects those nodes and rejects loading their behavior plugins.

The behavior server remains present because the chosen tree uses the Wait
action and because lifecycle behavior should stay explicit. Isolating its
generic command endpoint avoids relying on an assumption that a non-motion
plugin will never construct or publish through that endpoint.

### Launch and configuration requirements

The project-owned Nav2 launch preserves the installed Jazzy controller,
smoother, and collision-monitor topic semantics while overriding behavior
output:

- remap the controller server's `cmd_vel` to `cmd_vel_nav`;
- remap the velocity smoother's `cmd_vel` input to `cmd_vel_nav` and retain its
  `cmd_vel_smoothed` output;
- set the collision monitor's `cmd_vel_in_topic` to `cmd_vel_smoothed` and
  `cmd_vel_out_topic` to `cmd_vel`;
- remap the behavior server's `cmd_vel` to `cmd_vel_behavior_unused`;
- use bounded RELIABLE, VOLATILE, KEEP_LAST command queues; and
- keep the final bridge one-way from ROS to Gazebo.

Every name is relative in package configuration and receives the `robotest`
namespace from launch. No hidden absolute remap may bypass the chain.

Phase 2 uses `use_respawn=false`; later process restart remains the Go
supervisor's responsibility under the system-boundary decision. Lifecycle
activation must include the collision monitor before a mission goal is sent.

## Required static proof

Before any integrated motion run, tests must prove:

- only `controller_server` maps to connected `cmd_vel_nav`;
- the behavior server maps only to `cmd_vel_behavior_unused`;
- only `collision_monitor` maps to final `cmd_vel`;
- the Gazebo bridge subscribes to final `cmd_vel` and has no reverse command
  direction;
- the behavior plugin list is exactly the approved Wait-only set;
- the custom behavior tree contains no prohibited motion behavior;
- no mission, test helper, teleop node, or direct Nav2 output bypasses the
  collision monitor; and
- source and installed launch/config copies resolve to the same reviewed
  content for the run.

Static configuration proves intended wiring, not live DDS ownership.

## Required live ownership evidence

The Phase 2 verifier must capture `ros2 topic info <topic> -v`, node
information, the process list, and a bounded message trace. It must fail unless
all of the following are true at the same observation point:

1. `/robotest/cmd_vel_nav` has exactly one compatible publisher,
   `controller_server`, and the expected velocity-smoother subscriber.
2. `/robotest/cmd_vel_smoothed` has exactly one compatible publisher,
   `velocity_smoother`, and the expected collision-monitor subscriber.
3. `/robotest/cmd_vel` has exactly one compatible publisher,
   `collision_monitor`, and the expected Gazebo bridge subscriber.
4. `/robotest/cmd_vel_behavior_unused` has no subscriber; any message observed
   there remains isolated and never appears as an additional publisher on a
   connected command topic.
5. No duplicate node name or stale ROS daemon view hides a second process or
   endpoint.
6. Command QoS is compatible and every history is statically bounded even if
   the Jazzy Fast DDS graph reports unknown history or depth zero.
7. A bounded motion command is observable at each connected stage, a final
   zero command is observed, and validation truth confirms motion stops.

The evidence names node, namespace, endpoint GID when available, QoS, topic,
run ID, command timestamps, and process identity. Topic counts alone are not
sufficient if duplicate processes or stale discovery data remain possible.

## Consequences

### Positive

- Collision monitoring cannot be bypassed by normal Nav2 control or recovery
  wiring.
- The connected navigation-command topic has one unambiguous publisher.
- Phase 2 behavior is deterministic and excludes unvalidated recovery motion.
- Future command-path evidence has clear ownership expectations at every
  stage.

### Costs

- The project must own a small Jazzy-specific launch/config adaptation instead
  of using `navigation_launch.py` unchanged.
- The Wait-only policy provides fewer recovery options and may cause a mission
  to abort where a motion recovery could have succeeded.
- Live ownership and end-to-end stop proof add runtime verification work beyond
  static launch inspection.

## Revisit conditions

Revisit this decision before any of the following changes:

- enabling Spin, BackUp, DriveOnHeading, AssistedTeleop, docking motion, or a
  new behavior that can publish velocity;
- adding teleoperation, a twist multiplexer, or another command arbiter;
- replacing or removing the collision monitor;
- changing from `geometry_msgs/msg/Twist` to stamped velocity commands;
- composing nodes in a way that changes process or endpoint evidence; or
- moving from simulation to hardware.

Enabling a motion behavior requires a prior decision that defines explicit
arbitration into the connected command path, validates geometry and clearance,
updates the topic contract, and adds failure-path and stop-response evidence.
Simply remapping the isolated behavior topic back to `cmd_vel_nav` is not an
approved change.

## Rejected alternatives

- Use the installed Nav2 launch unchanged: rejected because it connects both
  controller and behavior command endpoints to `cmd_vel_nav` and loads motion
  recoveries not approved for Phase 2.
- Let the behavior server publish directly to final `cmd_vel`: rejected because
  it bypasses smoothing and collision monitoring.
- Remove the collision monitor and bridge `cmd_vel_nav` directly: rejected
  because it violates the established final-command boundary and weakens later
  sensor-stale safety tests.
- Rely only on behavior-tree sequencing with two connected publishers:
  rejected because the DDS graph would still have ambiguous endpoint ownership
  and a configuration or action error could expose competing commands.

## Implementation status

The Wait-only behavior configuration, custom behavior tree, isolated remap,
project-owned launch, and live ownership evidence are implemented. Bounded
headless run `20260826T010218Z-466` accepted this Phase 2 command chain; the
revisit conditions above remain mandatory for any future command-path change.

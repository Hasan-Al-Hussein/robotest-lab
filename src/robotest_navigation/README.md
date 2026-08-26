# robotest_navigation

`robotest_navigation` is the project-owned ROS 2 Jazzy navigation
configuration for the primitive RoboTest Lab differential-drive robot. It
installs the static map, AMCL/Nav2 parameters, an actuation-free behavior
tree, and the canonical Phase 2 launch file.

## Frozen geometry and mission coordinates

- Map resolution: `0.05 m/cell`.
- Map frame and Gazebo world frame: identity-aligned ENU coordinates.
- Robot footprint: `x=+/-0.24 m`, `y=+/-0.21 m`, plus `0.02 m` costmap
  footprint padding.
- AMCL initial pose: `(x=0.0, y=-3.5, yaw=0.0)`.
- Ordered mission waypoints: `(-2.0, -3.5)`, `(0.0, 0.0)`, and
  `(0.0, 3.5)` in `map`.

The final leg crosses the `1.2 m` opening in the divider at `y=1.5`. The
checked-in map is generated directly from the collision geometry in
`robotest_sim/worlds/robotest_lab.sdf`; it does not use a hand-painted or
SLAM-derived raster.

## Command ownership

The launch file implements the Phase 2 command boundary:

```text
controller_server
  -> cmd_vel_nav
  -> velocity_smoother
  -> cmd_vel_smoothed
  -> collision_monitor
  -> cmd_vel
  -> one-way Gazebo bridge
```

The behavior server loads only `nav2_behaviors::Wait`. Its generic command
endpoint is isolated on `cmd_vel_behavior_unused`, which has no actuator
consumer. The custom behavior tree permits planning, following, costmap
clears, and bounded Wait; it contains no Spin, BackUp, DriveOnHeading, or
AssistedTeleop action.

All topic names in configuration are relative. With the default namespace,
the connected command topics resolve to `/robotest/cmd_vel_nav`,
`/robotest/cmd_vel_smoothed`, and `/robotest/cmd_vel`.

## Launch

After building and sourcing the workspace:

```bash
ros2 launch robotest_navigation phase2.launch.py \
  seed:=42 headless:=true render_sensors:=true rviz:=false
```

Launch arguments are:

- `namespace` (`robotest`)
- `seed` (`42`, forwarded unchanged to `robotest_sim`)
- `headless` (`true`)
- `render_sensors` (`true`)
- `rviz` (`true`)
- `autostart` (`true`)
- `map` (the installed `maps/robotest_lab.yaml`)
- `params_file` (the installed `config/nav2_params.yaml`)
- `navigation_start_delay_sec` (`7.0`)
- `lifecycle_discovery_grace_sec` (`4.0`)
- `lifecycle_service_timeout_sec` (`20.0`)
- `lifecycle_response_timeout_sec` (`60.0`)
- `lifecycle_startup_result_path` (empty; disables result output)
- `log_level` (`info`)

Simulation time is always enabled. Node composition and process respawn are
intentionally disabled for Phase 2; process restart belongs to the later Go
supervisor.

The Nav2 lifecycle manager itself always starts with `autostart=false`. When
the public `autostart` argument is `true`, the project-owned
`lifecycle_startup_trigger` waits for the manager service, holds a stable
`4.0 s` monotonic wall-clock discovery grace, and then sends exactly one
bounded `nav2_msgs/srv/ManageLifecycleNodes` `STARTUP` request. This avoids
depending on ROS CLI daemon discovery and gives DDS response readers time to
match on resource-constrained WSL hosts. The default leaves margin above the
`0.868 s` and `2.807 s` manager-client-initialization-to-start intervals seen
in successful evidence runs while remaining configurable. Set
`autostart:=false` to leave all Nav2 lifecycle nodes unmanaged; no trigger
process is launched.

Set `lifecycle_startup_result_path` to capture the lifecycle manager's own
bounded STARTUP outcome. The helper writes the JSON atomically only after the
manager response arrives; `PASS` therefore means the manager returned
`success=true` after completing startup. The strict schema records the exact
service and STARTUP command, all effective wall-time bounds, optional watched
PID, finite elapsed wall time, stable failure kind, process exit code, and UTC
start/completion timestamps. The empty default performs no file write.

The helper can also be invoked directly for bounded diagnostics:

```bash
ros2 run robotest_navigation lifecycle_startup_trigger \
  --namespace /robotest \
  --manager-node lifecycle_manager_navigation \
  --discovery-grace-sec 4.0 \
  --service-timeout-sec 20.0 \
  --response-timeout-sec 60.0 \
  --watch-pid "$NAV_LAUNCH_PID" \
  --result-json artifacts/evidence/phase2/lifecycle-startup-result.json \
  --ros-args -p use_sim_time:=true
```

All grace and timeout bounds use monotonic wall time even though the helper's
ROS parameter `use_sim_time` is required to be `true` with the rest of Phase
2. The optional `--watch-pid` fails early if its owner process exits.

## Regenerating the map

From the workspace root:

```bash
python3 src/robotest_navigation/tools/generate_map.py \
  --world src/robotest_sim/worlds/robotest_lab.sdf \
  --output-dir src/robotest_navigation/maps
```

The package test regenerates the map into a temporary directory and requires
byte-for-byte equality with the checked-in PGM and YAML.

## Verification scope

`colcon test --packages-select robotest_navigation` checks map/world
reproducibility, collision geometry, waypoint clearance, exact plugin IDs,
validated-only inputs, command ownership, behavior isolation, forbidden BT
actions, launch arguments, and install discovery. Those are static/unit and
launch-description checks. A successful navigation action, lifecycle graph,
QoS ownership, localization quality, and physical stop remain Phase 2 runtime
gates and are not claimed by this package alone.

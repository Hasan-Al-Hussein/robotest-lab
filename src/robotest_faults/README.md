# robotest_faults

`robotest_faults` is the always-present validation and deterministic fault
boundary for RoboTest Lab. In `RESET` and `PREPARED`, valid raw messages are
forwarded unchanged. In `ARMED`, the proxy applies only the bounded Phase 3
faults frozen by ADR 0005.

## Runtime graph

With the default `/robotest` namespace, `fault_proxy_node`:

- subscribes to `raw/scan`, `raw/odom`, and `raw/imu`;
- publishes validated `scan`, `odom`, and `imu`;
- broadcasts `odom -> base_footprint` from the same validated odometry pose;
- publishes reliable, volatile, keep-last-100 evidence on `faults/events`;
- provides `faults/preload_schedule`, `faults/arm_schedule`, and
  `faults/reset`.

The legacy `faults/load_schedule` service is intentionally absent.

## Fault protocol

Preload independently validates and canonicalizes at most 16 specifications,
recomputes their SHA-256 with OpenSSL, and commits a generation atomically.
Arm binds that exact prepared generation to one nonzero goal UUID and its
authoritative accepted-goal stamp. The earliest activation must remain at
least 0.5 simulation seconds after the arm commit. Reset clears the current
binding and counters without rewinding the process-lifetime generation
allocator.

Supported transformations are LiDAR dropout and left-composed planar
odometry drift. Raw messages are never mutated. Odometry twist, covariances,
z position, frame IDs, and stamp remain unchanged, and validated TF is derived
from the resulting odometry message.

## Build and test

```bash
source /opt/ros/jazzy/setup.bash
colcon build --parallel-workers 4 --packages-up-to robotest_faults
colcon test --parallel-workers 4 --packages-select robotest_interfaces robotest_faults
colcon test-result --verbose
```

The package requires ROS 2 Jazzy, OpenSSL development headers, and
`nlohmann-json3-dev`.

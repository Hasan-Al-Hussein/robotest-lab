# RoboTest robot description

This Apache-2.0 ROS 2 package defines the original primitive differential-drive
robot used by RoboTest Lab. The model intentionally uses only boxes, cylinders,
and spheres so it is reproducible, easy to inspect, and inexpensive to simulate.

## Model contract

- Root frame: 'base_footprint', with a fixed transform to 'base_link'.
- Drive: two 0.075 m radius wheels separated by 0.38 m.
- Support: low-friction front and rear spherical casters keep the chassis level.
- Sensors: a 360-sample, 5 Hz GPU lidar and a 50 Hz IMU.
- Validation: chassis contacts and world-frame ground-truth odometry.
- All physical links have nonzero mass, collision geometry, and valid inertia.

The chassis is 0.48 x 0.32 x 0.12 m and has 0.075 m ground clearance. A
conservative planar navigation footprint is the rectangle with corners
(-0.26, -0.22), (-0.26, 0.22), (0.26, 0.22), and (0.26, -0.22), expressed in
'base_footprint'.

## Topic and TF isolation

With the default '/robotest' namespace, Gazebo uses:

| Purpose | Gazebo Transport topic |
| --- | --- |
| Velocity input | '/robotest/cmd_vel' |
| Raw lidar | '/robotest/raw/scan' |
| Raw wheel odometry | '/robotest/raw/odom' |
| Raw IMU | '/robotest/raw/imu' |
| Joint states | '/robotest/joint_states' |
| Chassis contacts | '/robotest/validation/contacts' |
| Ground truth | '/robotest/validation/ground_truth' |

Gazebo Harmonic's DiffDrive and OdometryPublisher systems always advertise
Gazebo Transport TF. Their TF outputs are intentionally moved to
'/robotest/internal/diff_drive_tf_unbridged' and
'/robotest/internal/ground_truth_tf_unbridged'. The bridge configuration must
not bridge either topic. This preserves the architectural rule that only the
fault-proxy layer may publish the ROS 'odom -> base_footprint' transform.

The world must load Gazebo's Sensors and IMU systems plus RoboTest's compiled
contact-aggregate system before spawning this model. The aggregate system
replaces Gazebo's stock multi-publisher Contact system. The simulation package
owns those world-level systems and the explicit ROS-Gazebo bridge allowlist.

## Collision-coverage provenance

The repository-root `config/collision-coverage.yaml` is generated evidence,
not a hand-maintained allowlist. From the repository root, verify it with:

    python3 src/robotest_description/tools/generate_collision_coverage.py \
      --repository-root . --mode check

After an intentional Xacro, contact sensor, bridge, or world change,
regenerate it atomically with `--mode write` and review the complete diff.
The generator expands the exact launch arguments, converts the URDF with
`gz sdf --precision 17 -p`, and proves that every one of the seven rendered
collision geometries has exactly one 5 Hz contact sensor on the frozen topic.
The 5 Hz value is declared source metadata only: Gazebo Sim 8 does not enforce
it. Each sensor has a unique unbridged defensive source topic. A source-bound
Gazebo system observes every completed physics step and publishes one bounded
20 ms interval aggregate on `/robotest/internal/contact_aggregate`; each
interval contains the union of observed normalized pairs and the latest exact
record group for each pair. That sole aggregate is bridged privately to
`/robotest/internal/raw_contacts`; the compiled `contact_stream_gate` owns the
public `/robotest/validation/contacts` stream. It publishes complete delivered
active-pair snapshots at a 5 Hz unchanged-state heartbeat and immediately on
pair-set transitions, with strict bounds and fail-closed overflow behavior.

The manifest binds raw bridge and world bytes, the aggregate plugin and gate's
shared CMake/header/source/configuration inventory and launch wiring, a
canonical inventory of every Xacro source plus render arguments, the printed
rendered SDF bytes, and a canonical semantic projection of the seven contact
sensors. Its declared
`manifest_sha256` is SHA-256 over sorted compact UTF-8 JSON plus LF with the
self-hash field omitted. YAML formatting and comments are not authoritative.
The four support exclusions are exact wheel/caster-to-ground pairs; no other
collision can be excluded.

## Inspect without Gazebo

After building and sourcing the workspace:

    ros2 launch robotest_description view_model.launch.py

Generate and validate the model directly:

    xacro urdf/robotest.urdf.xacro > /tmp/robotest.urdf
    check_urdf /tmp/robotest.urdf

Pass an absolute topic namespace when expanding the xacro, for example
'namespace:=/robot_2'. The inspection launch accepts the same namespace without
the leading slash.

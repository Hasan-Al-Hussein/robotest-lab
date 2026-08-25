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

The world must load Gazebo's Sensors, IMU, and Contact systems before spawning
this model. The simulation package owns those world-level systems and the
explicit ROS-Gazebo bridge allowlist.

## Inspect without Gazebo

After building and sourcing the workspace:

    ros2 launch robotest_description view_model.launch.py

Generate and validate the model directly:

    xacro urdf/robotest.urdf.xacro > /tmp/robotest.urdf
    check_urdf /tmp/robotest.urdf

Pass an absolute topic namespace when expanding the xacro, for example
'namespace:=/robot_2'. The inspection launch accepts the same namespace without
the leading slash.

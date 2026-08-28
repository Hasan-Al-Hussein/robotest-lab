# RoboTest Lab demo guide

[Back to the project overview](../README.md) | [Understand the pipeline](pipeline.md) |
[Troubleshooting](troubleshooting.md)

This guide offers three different views of the project. Start with the visual
sensor inspection. It is the lightest and clearest introduction.

| Goal | Recommended route | What it proves |
| --- | --- | --- |
| See the robot and sensor data | Two-minute RViz inspection | A human-readable development view only |
| Watch the robot drive | Optional two-terminal manual mission | A presentation demo only |
| Produce bounded Phase 2 evidence | Automated Phase 2 verifier | The documented development mission gate |

None of these demo routes substitutes for the Phase 3 campaign, privileged
Phase 4 acceptance, public CI, or final release decision. Their authoritative
status is recorded in the bounded final-status block in the
[root README](../README.md).

## Before you start

- Use Windows PowerShell.
- Confirm the WSL distribution is named `Ubuntu`.
- The project must already be built at `/home/hasan/robotest-lab`, with
  `install/setup.bash` present.
- Run only one RoboTest, Gazebo, or RViz session at a time.
- Close heavy applications if the laptop is under memory pressure.

## Demo 1: inspect the robot and sensors

Paste this one command into PowerShell:

```powershell
wsl.exe -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && scripts/run_phase1_rviz_inspection.sh'
```

Allow up to 60 seconds for RViz to become ready. The inspection then stays open
for a total bounded window of about 120 seconds.

### What you should see

- A small blue differential-drive robot near the center.
- A gray measurement grid.
- Red points or lines around nearby geometry. These are LiDAR returns, not
  errors.
- Colored TF axes attached to the robot.
- Checked displays named **RobotModel**, **TF**, **Lidar**, and **Odometry** in
  the left panel.
- **Global Status: Ok** in the left panel.

The display names and status text provide non-color confirmation of what RViz
is showing. Use the mouse wheel to zoom and drag to orbit the camera. The robot
is expected to remain stationary in this inspection.

When the bounded window ends, RViz may ask about unsaved changes. Choose
**Discard**. The script intentionally uses the installed configuration and does
not need your temporary camera changes. To end early, return to PowerShell and
press `Ctrl+C` once, then allow cleanup to finish.

The script writes local inspection details and logs below
`artifacts/evidence/phase1/rviz-*`. That generated directory is diagnostic
evidence and is not a release verdict.

## Demo 2: watch the waypoint mission

This optional route opens Gazebo and RViz, then sends the three-point Phase 2
mission. It is easier to understand visually, but it is not the authoritative
Phase 2 verifier.

Open two PowerShell windows. Make sure no other RoboTest session is running.

### PowerShell 1: start the robot, Nav2, Gazebo, and RViz

```powershell
wsl.exe -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=190 && export GZ_PARTITION=robotest_manual_demo && ros2 launch robotest_navigation phase2.launch.py namespace:=robotest seed:=42 headless:=false render_sensors:=true rviz:=true autostart:=true'
```

Wait 45 to 60 seconds for the simulation and Nav2 lifecycle startup. Gazebo and
RViz should open as Windows applications through WSLg.

### PowerShell 2: send the ordered mission

```powershell
wsl.exe -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=190 && export GZ_PARTITION=robotest_manual_demo && DEMO_DIR="artifacts/evidence/phase2/manual-demo-$(date -u +%Y%m%dT%H%M%SZ)" && mkdir -p "$DEMO_DIR" && ros2 run robotest_missions mission_runner --mission scenarios/phase2_baseline.yaml --json "$DEMO_DIR/mission-result.json" --csv "$DEMO_DIR/mission-result.csv" --ros-args -r __ns:=/robotest'
```

The robot starts at `(0.0, -3.5)` and attempts these points in order:

1. `(-2.0, -3.5)`
2. `(0.0, 0.0)`
3. `(0.0, 3.5)`

In RViz, look for the robot model, laser returns, transforms, and odometry. In
Gazebo, look for the physical robot moving through the original world. The two
views represent ROS observations and the simulator state.

PowerShell 2 returns when the mission reaches a terminal result. When finished,
press `Ctrl+C` once in PowerShell 1 and allow the launch process to shut down.
If either window reports an error, preserve the text and use the
[troubleshooting guide](troubleshooting.md) rather than starting overlapping
copies.

## Demo 3: run the bounded Phase 2 development gate

This route is headless. It does not open a moving visual window, but it runs the
documented build, launch, graph, lifecycle, mission, resource, cleanup, and
artifact checks together:

```powershell
wsl.exe -d Ubuntu -- bash -lc 'cd /home/hasan/robotest-lab && ROBOTEST_SIM_SEED=42 ROBOTEST_CPUSET=0-5 scripts/verify_phase2.sh'
```

Do not close the PowerShell window while the gate is active. The verifier owns
only the process groups it starts and performs bounded cleanup when it exits.
Generated evidence is written below `artifacts/evidence/phase2/`.

A successful new run is still development evidence unless it satisfies the
separate clean-source and release-binding requirements. The committed reference
result remains the [Phase 2 development report](results/phase-2/20260826T010218Z-466.md).

## Suggested three-minute explanation

Use this short narration while showing the screenshot or live RViz view:

> RoboTest Lab is a CPU-only ROS 2 testing platform for autonomous navigation.
> Gazebo simulates an original robot and world. Raw sensors cross an explicit
> bridge and validation layer before Nav2 uses them. A bounded mission client
> sends three ordered waypoints, while separate metrics observe the robot,
> safety signals, resources, and terminal action result. Fault scenarios can
> introduce reviewed LiDAR dropout or odometry drift. Every engineering claim
> must be tied to source, configuration, runtime evidence, and checksums. The
> first two development gates are recorded. The root README's bounded final
> status and linked evidence state whether the later acceptance gates passed.

## What not to claim during the demo

- Do not call the Phase 1 screenshot a navigation benchmark.
- Do not call the manual moving demo an acceptance run.
- Do not say the 15-run Phase 3 campaign passed until its authoritative evidence
  exists and validates.
- Do not say Debian installation, upgrade, restart recovery, or removal passed
  until the privileged Phase 4 acceptance gate completes.
- Do not describe the project as a final release until public CI and the exact
  release-evidence gate both pass.

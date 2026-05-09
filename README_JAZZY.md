# ZebraT ROS 2 Jazzy workspace

This workspace is the ROS 2 / Gazebo Harmonic migration of the ROS 1 Noetic
`clb_ws` simulator. The first working target is Gazebo + RViz + Ackermann
control for the R1 robot.

## Build

```bash
cd /home/k/clb_ws_jazzy
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
cd /home/k/clb_ws_jazzy
bash run_jazzy.sh world:=area rviz:=true
```

For headless Gazebo:

```bash
bash run_jazzy.sh world:=area rviz:=false gui:=false
```

## Command The Robot

```bash
ros2 topic pub /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.2, steering_angle: 0.2}}"
```

The ROS 2 port keeps `/ackermann_cmd` as the external command topic. The
controller publishes joint group commands to `ros2_control`, and publishes a
simple command-integrated `/odom` transform for first-stage testing.

## Nav2 Area Bringup

The first Nav2 target uses the simplified Harmonic `area.world` and the
matching `maps/area_jazzy.yaml` map:

```bash
cd /home/k/clb_ws_jazzy
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch zebrat nav2_area.launch.py world:=area rviz:=true
```

Headless validation:

```bash
ros2 launch zebrat nav2_area.launch.py world:=area gui:=false rviz:=false
```

In RViz, use the `2D Goal Pose` tool with frame `map`. For a command-line smoke
test in the simplified world:

```bash
ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

The navigation command chain is:

```text
Nav2 /cmd_vel -> /ackermann_cmd_nav -> /ackermann_cmd_safety_in -> /ackermann_cmd
```

`/scan` and `/imu/data` are bridged from Gazebo; `/camera/depth/image_raw` and
`/camera/camera_info` are also available. The first Nav2 map keeps the global
costmap static and uses the laser in the local costmap, which is more stable for
the small simplified test world. The planner is `SmacPlannerHybrid` and the
controller is MPPI.

## Full Area Scene

The fuller Harmonic scene is migrated from the static parts of the old
`area_classic.world`. It keeps the simplified `area.world` untouched and uses a
separate map and Nav2 params file. The full scene keeps `SmacPlannerHybrid` for
global planning and uses Regulated Pure Pursuit for the first stable navigation
smoke test; `/cmd_vel` still flows through the Ackermann adapter and safety
supervisor.

```bash
cd /home/k/clb_ws_jazzy
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch zebrat nav2_area_full.launch.py rviz:=true
```

Headless validation:

```bash
ros2 launch zebrat nav2_area_full.launch.py gui:=false rviz:=false
```

Command-line goals for the full scene:

```bash
ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

ros2 action send_goal -f /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 8.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

The first full-scene pass migrates static models only. Classic plugin examples,
dynamic actors, and mechanism demo models remain excluded until the Harmonic
equivalents are implemented.

## Full Area Multi-Sensor Fusion

The ROS1-style fusion entry keeps the full static scene but replaces the
smoke-test odometry with a wheel-odometry plus IMU EKF chain:

```text
/joint_states -> /wheel/odom
/imu/data     -> robot_localization EKF -> /odom and odom->base_link
/scan         -> AMCL -> map->odom
```

Run it with:

```bash
ros2 launch zebrat nav2_area_full_fusion.launch.py rviz:=true
```

This entry disables the temporary static `map->odom` transform and disables the
Ackermann controller's command-integrated `/odom`, so Nav2 consumes the fused
localization chain instead.

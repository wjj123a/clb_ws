# ZebraT ROS 2 Jazzy workspace

This workspace is the ROS 2 / Gazebo Harmonic migration of the ROS 1 Noetic
`clb_ws` simulator. The current default entry starts the full `area_full`
Gazebo scene, RViz, Nav2, the Ackermann control chain, and the RGBD/RTAB-Map
debug chain for the R1 robot.

## Build

```bash
cd /home/k/clb_ws_jazzy
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

The convenience script is the recommended default entry. It sources Jazzy,
builds the workspace, and launches the
full navigation scene with both Gazebo GUI and RViz enabled by default. RTAB-Map
is disabled in the convenience entry so the static-map navigation baseline is
quiet and easier to validate:

```bash
/home/k/clb_ws_jazzy/run_jazzy.sh
```

This is equivalent to launching `zebrat nav2_area_full.launch.py` with
`gui:=true`, `rviz:=true`, `rtabmap:=false`, `controller_start_delay:=6.0`,
and `nav2_start_delay:=14.0`. You can still override those launch arguments:

```bash
/home/k/clb_ws_jazzy/run_jazzy.sh gui:=false rviz:=false
/home/k/clb_ws_jazzy/run_jazzy.sh rtabmap:=true
/home/k/clb_ws_jazzy/run_jazzy.sh nav2_start_delay:=0 controller_start_delay:=0
```

For the lower-level Ackermann-only simulation bringup, launch
`sim_control.launch.py` directly.

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
`/camera/camera_info` are also available, with ROS1-style compatibility aliases
at `/front_camera/depth/image_raw` and `/front_camera/depth/camera_info`. The
first Nav2 map keeps the global costmap static and uses the laser in the local
costmap, which is more stable for the small simplified test world. The planner
is `SmacPlannerHybrid` and the controller is MPPI.

## Full Area Scene

The fuller Harmonic scene is migrated from the static parts of the old
`area_classic.world`. It keeps the simplified `area.world` untouched and uses a
separate map and Nav2 params file. This default full-scene entry now uses the
ROS1-style fusion chain: wheel odometry plus IMU through `robot_localization`,
with AMCL publishing `map->odom`. `/cmd_vel` still flows through the Ackermann
adapter and safety supervisor. The full-scene Nav2 stack uses
`SmacPlannerHybrid` with a forward-only Dubins search model and an Ackermann
`MPPIController` for local control.

```bash
cd /home/k/clb_ws_jazzy
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch zebrat nav2_area_full.launch.py gui:=true rviz:=true controller_start_delay:=6 nav2_start_delay:=14
```

The same entry is available through the convenience script:

```bash
bash run_jazzy.sh
```

Headless validation:

```bash
/home/k/clb_ws_jazzy/run_jazzy.sh gui:=false rviz:=false
```

RTAB-Map is available in the full-scene entry, but the convenience script keeps
it off by default while validating static-map Nav2. To enable the RGBD
SLAM/debug mapping sidecar:

```bash
/home/k/clb_ws_jazzy/run_jazzy.sh rtabmap:=true
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

The localization chain is:

```text
/joint_states -> /wheel/odom
/imu/data     -> robot_localization EKF -> /odom and odom->base_link
/scan         -> AMCL -> map->odom
```

The ROS1-style RGBD chain is also restored:

```text
Gazebo RGB/depth camera -> /front_camera/rgb/image_raw
Gazebo RGB/depth camera -> /front_camera/depth/image_raw
Gazebo depth camera     -> /front_camera/depth/image_viz
/front_camera/depth/image_raw + camera_info -> /front_camera/depth/points
/odom + RGB + depth + /scan -> /rtabmap/info, /rtabmap/mapGraph, /rtabmap/mapData
```

The local Nav2 costmap consumes both `/scan` and
`/front_camera/depth/points`, so the current full-scene entry is a static-map
Nav2 baseline with multi-sensor local obstacle input. It is not yet the final
no-map online SLAM navigation entry; that next entry should disable AMCL and
`map_server`, let RTAB-Map publish `/map` and `map->odom`, and let Nav2 consume
the live RTAB-Map occupancy map.

The front lidar is mounted at the front top of the robot in the Jazzy URDF
(`x=0.62`, `z=0.55`) with a `0.18m` minimum range, which keeps the local
costmap from marking the robot body as an obstacle at startup.

The RViz config intentionally mirrors the old ROS1 navigation and RTAB-Map
debug views: map, robot model, TF, laser scan, `/front_camera` image displays,
global/local costmaps, global/local plans, AMCL particles, fused/wheel odometry,
and RTAB-Map info/graph/cloud displays. The RTAB-Map database defaults to
`~/.ros/rtabmap/area_full_jazzy.db`; pass `rtabmap_delete_db_on_start:=true`
when you want a fresh mapping database.

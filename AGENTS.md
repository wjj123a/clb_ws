# Repository Guidelines

## Project Structure & Module Organization

This is a ROS Noetic catkin workspace. The active package is `zebrat/`; `src/zebrat` is a workspace link and should not be committed. Main runtime code lives in `zebrat/scripts/`, launch files in `zebrat/launch/`, navigation and controller parameters in `zebrat/config/`, robot descriptions in `zebrat/urdf/`, RViz layouts in `zebrat/rviz/`, Gazebo worlds in `zebrat/worlds/`, and meshes/models under `zebrat/meshes/` and `zebrat/gazebo_models/`. Map snapshots belong in `zebrat/maps/`, but generated `.yaml`, `.pgm`, and RTAB-Map `.db` files are ignored except for placeholders and documentation.

## Build, Test, and Development Commands

Run commands from the workspace root:

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

`catkin_make` builds the package and installs Python ROS nodes into the devel space. Use `bash run_noetic.sh` for the default Gazebo, RViz, AMCL, `move_base`, TEB, and Ackermann safety chain. Use `bash run_noetic.sh navigation_mode:=slam` to rebuild maps. Start teleop in a second terminal with `bash teleop_noetic.sh`. For headless checks, run `roslaunch zebrat r1_navigation_regression.launch navigation_mode:=map`.

## Coding Style & Naming Conventions

Python nodes target ROS Noetic Python 3. Use 4-space indentation, `snake_case` names, executable shebangs, and `rospy.log*` for runtime diagnostics. Keep ROS topic, frame, and parameter names lowercase with underscores. Launch arguments should expose tunable behavior instead of hardcoding geometry or safety limits. YAML config keys should match ROS parameter names and stay grouped by subsystem, such as `TebLocalPlannerROS` or costmap sections.

## Testing Guidelines

There is no formal unit-test suite yet. Validate Python syntax with:

```bash
python3 -m py_compile zebrat/scripts/*.py
```

Then run `catkin_make` and at least one relevant `roslaunch --nodes` or regression launch. For navigation changes, check dumped parameters with `roslaunch --dump-params zebrat zebrat_with_world.launch` and verify the `/cmd_vel_nav -> /ackermann_cmd_nav -> /ackermann_cmd` safety chain remains intact.

## Commit & Pull Request Guidelines

Recent commits use short Chinese summaries, for example `加入teb`, `重构`, and `修复前轮脱离情况`. Keep commit titles concise and behavior-focused; mention the affected subsystem when useful. Pull requests should describe the scenario tested, list launch arguments changed, note generated assets intentionally omitted, and include screenshots only for RViz/Gazebo visual changes.

## Configuration Notes

Do not commit `build/`, `devel/`, generated maps, RTAB-Map databases, `__pycache__/`, or local Gazebo model caches. If static map mode depends on a map file, document how it is produced because map outputs are intentionally ignored.

# Repository Guidelines

## Project Structure & Module Organization

This is a ROS 2 Jazzy workspace for the ZebraT R1 simulator. The active package is `src/zebrat`, built with `ament_cmake`. Launch entries live in `src/zebrat/launch`, runtime parameters in `src/zebrat/config`, robot descriptions in `src/zebrat/urdf`, Gazebo worlds/models in `src/zebrat/worlds` and `src/zebrat/gazebo_models`, maps in `src/zebrat/maps`, RViz layouts in `src/zebrat/rviz`, and Python ROS nodes in `src/zebrat/scripts`. Generated workspace outputs (`build/`, `install/`, `log/`) should not be treated as source.

## Build, Test, and Development Commands

Source ROS before building:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --base-paths src
source install/setup.bash
```

Use `./run_jazzy.sh` for the default full-scene Nav2 simulation; it builds, sources the install space, and launches `zebrat nav2_area_full.launch.py` with GUI and RViz enabled. Useful variants:

```bash
./run_jazzy.sh gui:=false rviz:=false
./run_jazzy.sh rtabmap:=true
ros2 launch zebrat nav2_area.launch.py world:=area gui:=false rviz:=false
colcon test --base-paths src --packages-select zebrat
```

## Coding Style & Naming Conventions

Python scripts use Python 3, four-space indentation, `snake_case` functions/variables, and `PascalCase` node classes. Keep ROS parameters and topic names explicit and stable. Name launch files and YAML configs by subsystem and scenario, for example `nav2_area_full.launch.py` or `navigation_global_costmap.yaml`. If adding a new executable script, include it in `src/zebrat/CMakeLists.txt` under `install(PROGRAMS ...)`.

## Testing Guidelines

There is no dedicated test suite yet. At minimum, run `colcon build` and a headless launch smoke test before submitting navigation, Gazebo, or URDF changes. When adding tests, prefer ROS 2/ament conventions under a package-local `test/` directory and run them with `colcon test --base-paths src --packages-select zebrat`.

## Commit & Pull Request Guidelines

The current history uses short descriptive subjects such as `Port simulator to ROS 2 Jazzy`. Keep commit subjects concise, imperative or descriptive, and mention the affected area when useful, such as `Nav2`, `Gazebo`, `URDF`, or `RTAB-Map`.

Pull requests should describe the scenario changed, list validation commands, and include screenshots or logs for RViz/Gazebo behavior changes. Link related issues when available and call out required local configuration, such as ROS distro, GPU/Gazebo settings, or map database handling.

## Configuration & Assets

Keep maps as matching `.yaml`/`.pgm` pairs. Avoid committing local runtime databases such as `~/.ros/rtabmap/*.db`. Large mesh/model updates should be intentional and tied to a launch or world change that uses them.

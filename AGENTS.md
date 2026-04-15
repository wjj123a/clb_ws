# Repository Guidelines

## Project Structure & Module Organization
This repository is a ROS 1 Catkin workspace. Edit source under `src/`; avoid hand-editing generated output in `build/` and `devel/`. The active package is `r1`, stored under `src/R1`, with robot assets in `config/`, `launch/`, `maps/`, `meshes/`, `rviz/`, `textures/`, `urdf/`, and `worlds/`. Runtime Python lives in `src/R1/scripts/`, currently `docking_controller.py`. Package metadata and install rules are in `src/R1/package.xml` and `src/R1/CMakeLists.txt`.

## Build, Test, and Development Commands
Run commands from the workspace root:

```bash
catkin_make --pkg r1
```

Builds the `r1` package and refreshes `devel/` and `build/`.

```bash
source devel/setup.bash
roslaunch r1 display.launch
roslaunch r1 gazebo.launch
```

Use `display.launch` for URDF and RViz checks, and `gazebo.launch` for simulator validation. For navigation and docking flows, use `roslaunch r1 navigation.launch` or `roslaunch r1 docking.launch`.

## Coding Style & Naming Conventions
Use 4-space indentation in Python and keep imports grouped like the existing script. Follow PEP 8 for new Python logic, but prefer ROS-friendly readability over heavy abstraction. Use 2 spaces in XML and YAML. Keep package paths stable, for example `package://r1/meshes/base_link.STL`. New launch, config, and asset files should use descriptive snake_case names such as `local_costmap.yaml` or `sim_bringup.launch`.

## Testing Guidelines
There is no committed automated test suite in this checkout, so rely on targeted smoke tests:

```bash
python3 -m py_compile src/R1/scripts/docking_controller.py
check_urdf src/R1/urdf/R1.urdf
roslaunch r1 display.launch
```

If you change navigation or docking behavior, also run the relevant Gazebo or launch scenario and confirm package paths, frame names, and topic/service names still resolve.

## Commit & Pull Request Guidelines
No `.git` history is present in this workspace snapshot, so there is no verified house style to copy. Use short imperative commit subjects such as `Tune docking yaw tolerance` or `Add charging arena map assets`. PRs should summarize behavior changes, list affected launch/config/URDF files, and include RViz or Gazebo screenshots when visuals or motion changed.

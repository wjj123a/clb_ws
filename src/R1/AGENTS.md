# Repository Guidelines

## Project Structure & Module Organization
This repository is a ROS `catkin` description package for the `R1` robot. The package name is `r1`, and the source lives under `src/R1`. Core assets live in `urdf/` (`R1.urdf`, exported geometry data in `R1.csv`), `meshes/` (`*.STL` link meshes), `launch/` (`display.launch`, `gazebo.launch`), and `config/` (`joint_names_R1.yaml`). Package metadata is in `package.xml`, and install rules are in `CMakeLists.txt`.

## Build, Test, and Development Commands
Build from the catkin workspace root, not this package directory:

```bash
catkin_make --pkg r1
```

Builds and installs the package share directories declared in `CMakeLists.txt`.

```bash
source devel/setup.bash
roslaunch r1 display.launch
```

Loads the URDF in RViz with `joint_state_publisher_gui`.

```bash
source devel/setup.bash
roslaunch r1 gazebo.launch
```

Spawns the robot into Gazebo for a basic integration check.

## Coding Style & Naming Conventions
Keep XML and YAML indented with two spaces. Preserve the existing robot naming: package name `r1`, URDF file `R1.urdf`, joint names `J1` through `J4`, and config files that match the robot name, such as `joint_names_R1.yaml`. When adding meshes, use stable, descriptive link-based names and keep `package://r1/...` paths valid.

## Testing Guidelines
There is no automated test suite in this snapshot. Validate changes with smoke tests:

```bash
check_urdf urdf/R1.urdf
roslaunch r1 display.launch
roslaunch r1 gazebo.launch
```

Before submitting changes, confirm every referenced mesh exists, launch files resolve package paths correctly, and any new joint or link names stay synchronized across `urdf/`, `config/`, and launch usage.

## Commit & Pull Request Guidelines
No `.git` history is present in this checkout, so there is no repository-specific commit pattern to copy. Use short imperative commit subjects such as `Update Gazebo spawn model path` or `Adjust J3 joint limits`. Pull requests should describe the robot-model change, list affected files, and include RViz or Gazebo screenshots when visuals or kinematics changed.

## Configuration Notes
Keep `package.xml` dependencies aligned with launch-time requirements. If you add a new top-level asset directory, update the install loop in `CMakeLists.txt` so catkin exports it with the package.

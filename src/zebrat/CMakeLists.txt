cmake_minimum_required(VERSION 3.8)
project(zebrat)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    gazebo_models
    launch
    maps
    meshes
    rviz
    urdf
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/ackermann_cmd_arbiter.py
    scripts/ackermann_cmd_safety_supervisor.py
    scripts/gazebo_auto_unpause.py
    scripts/gazebo_odometry.py
    scripts/keyboard_ackermann_teleop.py
    scripts/r1_ackermann_controller.py
    scripts/twist_to_ackermann.py
    scripts/wheel_odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

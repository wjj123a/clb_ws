cmake_minimum_required(VERSION 2.8.3)

project(r1)

find_package(catkin REQUIRED COMPONENTS
  ackermann_steering_controller
  actionlib
  base_local_planner
  clear_costmap_recovery
  controller_manager
  gazebo_msgs
  gazebo_ros_control
  geometry_msgs
  hardware_interface
  move_base_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_srvs
  tf
  tf2_ros
  topic_tools
  transmission_interface
)

catkin_package(
  CATKIN_DEPENDS
    ackermann_steering_controller
    actionlib
    base_local_planner
    clear_costmap_recovery
    controller_manager
    gazebo_msgs
    gazebo_ros_control
    geometry_msgs
    hardware_interface
    move_base_msgs
    nav_msgs
    rospy
    sensor_msgs
    std_srvs
    tf
    tf2_ros
    topic_tools
    transmission_interface
)

find_package(roslaunch)

catkin_install_python(PROGRAMS
  scripts/ackermann_cmd_vel_adapter.py
  scripts/cmd_vel_collision_guard.py
  scripts/cmd_vel_limiter.py
  scripts/docking_controller.py
  scripts/goal_sanitizer.py
  scripts/gazebo_ackermann_effort_bridge.py
  scripts/ground_truth_localization.py
  scripts/odom_sanitizer.py
  scripts/planar_pose_guard.py
  scripts/spawn_robot_model.py
  scripts/static_joint_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch maps meshes rviz textures urdf worlds)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
  endif()
endforeach(dir)

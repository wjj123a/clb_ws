cmake_minimum_required(VERSION 2.8.3)

project(r1)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_srvs
  tf
)

catkin_package()

find_package(roslaunch)

catkin_install_python(PROGRAMS
  scripts/cmd_vel_limiter.py
  scripts/docking_controller.py
  scripts/odom_sanitizer.py
  scripts/planar_pose_guard.py
  scripts/static_joint_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch maps meshes rviz textures urdf worlds)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
  endif()
endforeach(dir)

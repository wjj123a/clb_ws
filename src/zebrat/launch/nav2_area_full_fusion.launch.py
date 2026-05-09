#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("zebrat")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    nav2_start_delay = LaunchConfiguration("nav2_start_delay")
    controller_start_delay = LaunchConfiguration("controller_start_delay")

    ekf_config = os.path.join(pkg_share, "config", "ekf_area_full.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI"),
            DeclareLaunchArgument("rviz", default_value="true", description="Start RViz"),
            DeclareLaunchArgument(
                "nav2_start_delay",
                default_value="60.0",
                description="Seconds to wait before starting Nav2 nodes",
            ),
            DeclareLaunchArgument(
                "controller_start_delay",
                default_value="30.0",
                description="Seconds to wait before spawning ros2_control controllers",
            ),
            Node(
                package="zebrat",
                executable="wheel_odometry.py",
                name="wheel_odometry",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "joint_states_topic": "/joint_states",
                        "odom_topic": "/wheel/odom",
                        "publish_tf": False,
                    }
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_config],
                remappings=[("odometry/filtered", "/odom")],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "nav2_area.launch.py")),
                launch_arguments={
                    "world": "area_full",
                    "gui": gui,
                    "rviz": rviz,
                    "map": os.path.join(pkg_share, "maps", "area_full_jazzy.yaml"),
                    "params_file": os.path.join(pkg_share, "config", "nav2_area_full.yaml"),
                    "nav2_start_delay": nav2_start_delay,
                    "controller_start_delay": controller_start_delay,
                    "static_map_to_odom": "false",
                    "amcl_tf_broadcast": "true",
                    "ackermann_publish_odom": "false",
                    "ackermann_publish_tf": "false",
                    "ackermann_odom_topic": "/ackermann/odom",
                }.items(),
            ),
        ]
    )

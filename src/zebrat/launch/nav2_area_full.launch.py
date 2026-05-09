#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("zebrat")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    nav2_start_delay = LaunchConfiguration("nav2_start_delay")
    controller_start_delay = LaunchConfiguration("controller_start_delay")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI"),
            DeclareLaunchArgument("rviz", default_value="true", description="Start RViz"),
            DeclareLaunchArgument(
                "nav2_start_delay",
                default_value="55.0",
                description="Seconds to wait before starting Nav2 nodes",
            ),
            DeclareLaunchArgument(
                "controller_start_delay",
                default_value="30.0",
                description="Seconds to wait before spawning ros2_control controllers",
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
                }.items(),
            ),
        ]
    )

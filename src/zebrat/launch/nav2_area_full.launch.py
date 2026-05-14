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
    rtabmap = LaunchConfiguration("rtabmap")
    rtabmap_start_delay = LaunchConfiguration("rtabmap_start_delay")
    rtabmap_database_path = LaunchConfiguration("rtabmap_database_path")
    rtabmap_delete_db_on_start = LaunchConfiguration("rtabmap_delete_db_on_start")

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI"),
            DeclareLaunchArgument("rviz", default_value="true", description="Start RViz"),
            DeclareLaunchArgument(
                "nav2_start_delay",
                default_value="0.0",
                description="Seconds to wait before starting Nav2 nodes",
            ),
            DeclareLaunchArgument(
                "controller_start_delay",
                default_value="0.0",
                description="Seconds to wait before spawning ros2_control controllers",
            ),
            DeclareLaunchArgument("rtabmap", default_value="false", description="Start RGBD RTAB-Map"),
            DeclareLaunchArgument(
                "rtabmap_start_delay",
                default_value="0.0",
                description="Seconds to wait before starting RGBD RTAB-Map",
            ),
            DeclareLaunchArgument(
                "rtabmap_database_path",
                default_value="~/.ros/rtabmap/area_full_jazzy.db",
                description="RTAB-Map database path",
            ),
            DeclareLaunchArgument(
                "rtabmap_delete_db_on_start",
                default_value="false",
                description="Delete RTAB-Map database at startup",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "nav2_area_full_fusion.launch.py")
                ),
                launch_arguments={
                    "gui": gui,
                    "rviz": rviz,
                    "nav2_start_delay": nav2_start_delay,
                    "controller_start_delay": controller_start_delay,
                    "rtabmap": rtabmap,
                    "rtabmap_start_delay": rtabmap_start_delay,
                    "rtabmap_database_path": rtabmap_database_path,
                    "rtabmap_delete_db_on_start": rtabmap_delete_db_on_start,
                }.items(),
            ),
        ]
    )

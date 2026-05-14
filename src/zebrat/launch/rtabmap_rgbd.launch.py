#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _setup(context, *_args, **_kwargs):
    enabled = _as_bool(LaunchConfiguration("enabled").perform(context))
    if not enabled:
        return []

    start_delay = float(LaunchConfiguration("start_delay").perform(context))
    database_path = os.path.expanduser(LaunchConfiguration("database_path").perform(context))
    delete_db_on_start = _as_bool(LaunchConfiguration("delete_db_on_start").perform(context))
    os.makedirs(os.path.dirname(database_path), exist_ok=True)

    rtabmap_args = []
    if delete_db_on_start:
        rtabmap_args.append("--delete_db_on_start")

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        namespace="rtabmap",
        name="rtabmap",
        output="screen",
        emulate_tty=True,
        arguments=rtabmap_args,
        parameters=[
            {
                "use_sim_time": True,
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "",
                "publish_tf": False,
                "subscribe_rgb": True,
                "subscribe_depth": True,
                "subscribe_scan": True,
                "subscribe_rgbd": False,
                "approx_sync": True,
                "topic_queue_size": 10,
                "sync_queue_size": 10,
                "wait_for_transform": 1.0,
                "database_path": database_path,
                "qos_image": 2,
                "qos_camera_info": 2,
                "qos_scan": 2,
                "qos_odom": 1,
                "RGBD/CreateOccupancyGrid": "true",
                "RGBD/NeighborLinkRefining": "true",
                "RGBD/ProximityBySpace": "true",
                "RGBD/ProximityByTime": "false",
                "RGBD/ProximityPathMaxNeighbors": "10",
                "RGBD/OptimizeFromGraphEnd": "false",
                "RGBD/OptimizeMaxError": "4",
                "RGBD/LinearUpdate": "0.05",
                "RGBD/AngularUpdate": "0.05",
                "RGBD/LocalRadius": "5",
                "Reg/Strategy": "2",
                "Reg/Force3DoF": "true",
                "Grid/Sensor": "2",
                "Grid/CellSize": "0.05",
                "Grid/RangeMax": "8.0",
                "Grid/RayTracing": "true",
                "Grid/Scan2dUnknownSpaceFilled": "true",
                "GridGlobal/MinSize": "20",
                "GridGlobal/ProbMiss": "0.35",
                "Mem/STMSize": "30",
                "Mem/IncrementalMemory": "true",
                "Mem/InitWMWithAllNodes": "false",
                "Vis/MinInliers": "12",
                "Icp/CorrespondenceRatio": "0.2",
                "Icp/Strategy": "0",
                "Icp/PointToPlane": "false",
                "Icp/MaxCorrespondenceDistance": "0.15",
                "Icp/VoxelSize": "0.05",
            }
        ],
        remappings=[
            ("rgb/image", "/front_camera/rgb/image_raw"),
            ("depth/image", "/front_camera/depth/image_raw"),
            ("rgb/camera_info", "/front_camera/rgb/camera_info"),
            ("scan", "/scan"),
            ("odom", "/odom"),
            ("grid_map", "/rtabmap/grid_map"),
        ],
    )

    return [TimerAction(period=start_delay, actions=[rtabmap_node])]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("enabled", default_value="true", description="Start RTAB-Map RGBD chain"),
            DeclareLaunchArgument(
                "start_delay",
                default_value="35.0",
                description="Seconds to wait before starting RTAB-Map nodes",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="~/.ros/rtabmap/area_full_jazzy.db",
                description="RTAB-Map database path",
            ),
            DeclareLaunchArgument(
                "delete_db_on_start",
                default_value="false",
                description="Delete the RTAB-Map database at startup",
            ),
            OpaqueFunction(function=_setup),
        ]
    )

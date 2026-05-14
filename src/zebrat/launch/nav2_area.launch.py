#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("zebrat")
    nav2_bt_share = get_package_share_directory("nav2_bt_navigator")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    nav2_start_delay = LaunchConfiguration("nav2_start_delay")
    controller_start_delay = LaunchConfiguration("controller_start_delay")
    static_map_to_odom = LaunchConfiguration("static_map_to_odom")
    amcl_tf_broadcast = LaunchConfiguration("amcl_tf_broadcast")
    ackermann_publish_odom = LaunchConfiguration("ackermann_publish_odom")
    ackermann_publish_tf = LaunchConfiguration("ackermann_publish_tf")
    ackermann_odom_topic = LaunchConfiguration("ackermann_odom_topic")
    default_nav_to_pose_bt_xml = os.path.join(
        nav2_bt_share, "behavior_trees", "navigate_to_pose_w_replanning_and_recovery.xml"
    )

    lifecycle_nodes = [
        "map_server",
        "amcl",
        "planner_server",
        "smoother_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_nav2",
        output="screen",
        arguments=["-d", os.path.join(pkg_share, "rviz", "nav2_area.rviz")],
        condition=IfCondition(rviz),
        parameters=[{"use_sim_time": True}],
    )

    nav2_nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            output="screen",
            condition=IfCondition(static_map_to_odom),
            arguments=[
                "--x",
                "0.0",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--roll",
                "0.0",
                "--pitch",
                "0.0",
                "--yaw",
                "0.0",
                "--frame-id",
                "map",
                "--child-frame-id",
                "odom",
            ],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": True, "yaml_filename": map_yaml}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                params_file,
                {"tf_broadcast": ParameterValue(amcl_tf_broadcast, value_type=bool)},
            ],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                params_file,
                {"default_nav_to_pose_bt_xml": default_nav_to_pose_bt_xml},
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "autostart": True,
                    "bond_timeout": 0.0,
                    "node_names": lifecycle_nodes,
                }
            ],
        ),
        Node(
            package="zebrat",
            executable="twist_to_ackermann.py",
            name="twist_to_ackermann",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "input_topic": "/cmd_vel",
                    "output_topic": "/ackermann_cmd_nav",
                    "stamped_input": False,
                    "wheelbase": 0.41893,
                    "max_speed": 0.22,
                    "max_steering_angle": 0.45,
                    "cmd_angle_instead_rotvel": False,
                }
            ],
        ),
        Node(
            package="zebrat",
            executable="ackermann_cmd_arbiter.py",
            name="ackermann_cmd_arbiter",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "nav_topic": "/ackermann_cmd_nav",
                    "teleop_topic": "/ackermann_cmd_teleop",
                    "output_topic": "/ackermann_cmd_safety_in",
                    "nav_timeout": 0.5,
                    "teleop_timeout": 0.75,
                    "frame_id": "base_link",
                }
            ],
        ),
        Node(
            package="zebrat",
            executable="ackermann_cmd_safety_supervisor.py",
            name="ackermann_cmd_safety_supervisor",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "input_topic": "/ackermann_cmd_safety_in",
                    "output_topic": "/ackermann_cmd",
                    "scan_topic": "/scan",
                    "hard_stop_distance": 0.25,
                    "slowdown_distance": 0.65,
                    "front_angle": 0.55,
                    "rear_angle": 0.55,
                    "reaction_time": 0.25,
                    "max_deceleration": 0.60,
                    "ttc_stop_time": 0.45,
                    "frame_id": "base_link",
                }
            ],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="area", description="World name from zebrat/worlds"),
            DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI"),
            DeclareLaunchArgument("rviz", default_value="true", description="Start RViz"),
            DeclareLaunchArgument("map", default_value=os.path.join(pkg_share, "maps", "area_jazzy.yaml")),
            DeclareLaunchArgument("params_file", default_value=os.path.join(pkg_share, "config", "nav2_area.yaml")),
            DeclareLaunchArgument(
                "nav2_start_delay",
                default_value="14.0",
                description="Seconds to wait before starting Nav2 nodes",
            ),
            DeclareLaunchArgument(
                "controller_start_delay",
                default_value="6.0",
                description="Seconds to wait before spawning ros2_control controllers",
            ),
            DeclareLaunchArgument(
                "static_map_to_odom",
                default_value="true",
                description="Publish a static map to odom transform for smoke-test localization",
            ),
            DeclareLaunchArgument(
                "amcl_tf_broadcast",
                default_value="false",
                description="Let AMCL publish map to odom",
            ),
            DeclareLaunchArgument(
                "ackermann_publish_odom",
                default_value="true",
                description="Pass-through for sim_control Ackermann odometry publishing",
            ),
            DeclareLaunchArgument(
                "ackermann_publish_tf",
                default_value="true",
                description="Pass-through for sim_control Ackermann odom TF publishing",
            ),
            DeclareLaunchArgument(
                "ackermann_odom_topic",
                default_value="/odom",
                description="Pass-through for sim_control Ackermann odometry topic",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "sim_control.launch.py")),
                launch_arguments={
                    "world": world,
                    "gui": gui,
                    "sim_rviz": "false",
                    "controller_start_delay": controller_start_delay,
                    "ackermann_publish_odom": ackermann_publish_odom,
                    "ackermann_publish_tf": ackermann_publish_tf,
                    "ackermann_odom_topic": ackermann_odom_topic,
                }.items(),
            ),
            TimerAction(period=nav2_start_delay, actions=[rviz_node, *nav2_nodes]),
        ]
    )

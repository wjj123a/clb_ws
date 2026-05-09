#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *_args, **_kwargs):
    pkg_share = get_package_share_directory("zebrat")
    world_name = LaunchConfiguration("world").perform(context)
    rviz_enabled = LaunchConfiguration("rviz")
    controller_start_delay = LaunchConfiguration("controller_start_delay")
    ackermann_publish_odom = LaunchConfiguration("ackermann_publish_odom")
    ackermann_publish_tf = LaunchConfiguration("ackermann_publish_tf")
    ackermann_odom_topic = LaunchConfiguration("ackermann_odom_topic")
    gui_enabled = LaunchConfiguration("gui").perform(context).lower() in ("1", "true", "yes")

    world_path = os.path.join(pkg_share, "worlds", f"{world_name}.world")
    if not os.path.exists(world_path):
        raise RuntimeError(f"World '{world_name}' does not exist at {world_path}")

    robot_xacro = os.path.join(pkg_share, "urdf", "R1_jazzy.urdf.xacro")
    controllers_file = os.path.join(pkg_share, "config", "r1_ros2_control.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "r1_jazzy.rviz")
    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                robot_xacro,
                " controllers_file:=",
                controllers_file,
            ]
        )
    }

    gz_command = ["gz", "sim", "-r"]
    if not gui_enabled:
        gz_command.append("-s")
    gz_command.append(world_path)

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    spawn_front_steering_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["front_steering_controller", "--controller-manager", "/controller_manager"],
    )
    spawn_front_wheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["front_wheel_controller", "--controller-manager", "/controller_manager"],
    )
    ackermann_controller = Node(
        package="zebrat",
        executable="r1_ackermann_controller.py",
        name="r1_ackermann_controller",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "publish_odom": ParameterValue(ackermann_publish_odom, value_type=bool),
                "publish_tf": ParameterValue(ackermann_publish_tf, value_type=bool),
                "odom_topic": ackermann_odom_topic,
            }
        ],
    )

    return [
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[
                pkg_share,
                ":",
                os.path.dirname(pkg_share),
                ":",
                os.path.join(pkg_share, "gazebo_models"),
                ":",
                os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            ],
        ),
        SetEnvironmentVariable(name="GALLIUM_DRIVER", value="d3d12"),
        SetEnvironmentVariable(
            name="GZ_SIM_SYSTEM_PLUGIN_PATH",
            value=[
                "/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins",
                ":",
                "/opt/ros/jazzy/lib",
                ":",
                os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""),
            ],
        ),
        SetEnvironmentVariable(
            name="LD_LIBRARY_PATH",
            value=[
                "/opt/ros/jazzy/lib",
                ":",
                os.environ.get("LD_LIBRARY_PATH", ""),
            ],
        ),
        ExecuteProcess(cmd=gz_command, output="screen"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": True}],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_sensor_bridge",
            output="screen",
            arguments=[
                "/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                "/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ],
            remappings=[
                ("/world/default/clock", "/clock"),
                ("/camera/depth/camera_info", "/camera/camera_info"),
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="front_laser_frame",
            output="screen",
            arguments=[
                "--x",
                "0.53879978611665",
                "--y",
                "0.0",
                "--z",
                "0.36",
                "--roll",
                "0.0",
                "--pitch",
                "0.0",
                "--yaw",
                "0.0",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "r1/base_link/front_laser",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_sensor_frame",
            output="screen",
            arguments=[
                "--x",
                "0.21879978611665",
                "--y",
                "0.0",
                "--z",
                "0.22",
                "--roll",
                "0.0",
                "--pitch",
                "0.0",
                "--yaw",
                "0.0",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "r1/base_link/imu_sensor",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="front_camera_frame",
            output="screen",
            arguments=[
                "--x",
                "0.49879978611665",
                "--y",
                "0.0",
                "--z",
                "0.28",
                "--roll",
                "0.0",
                "--pitch",
                "0.0",
                "--yaw",
                "0.0",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "r1/base_link/front_camera",
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    name="spawn_r1",
                    output="screen",
                    arguments=[
                        "-world",
                        "default",
                        "-name",
                        "r1",
                        "-allow_renaming",
                        "false",
                        "-topic",
                        "robot_description",
                        "-x",
                        "0.0",
                        "-y",
                        "0.0",
                        "-z",
                        "0.12",
                    ],
                )
            ],
        ),
        TimerAction(
            period=controller_start_delay,
            actions=[spawn_joint_state_broadcaster],
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_joint_state_broadcaster,
                on_exit=[spawn_front_steering_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_front_steering_controller,
                on_exit=[spawn_front_wheel_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_front_wheel_controller,
                on_exit=[ackermann_controller],
            )
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz_enabled),
            parameters=[{"use_sim_time": True}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="area", description="World name from zebrat/worlds"),
            DeclareLaunchArgument("rviz", default_value="true", description="Start RViz"),
            DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI"),
            DeclareLaunchArgument(
                "controller_start_delay",
                default_value="6.0",
                description="Seconds to wait before spawning ros2_control controllers",
            ),
            DeclareLaunchArgument(
                "ackermann_publish_odom",
                default_value="true",
                description="Let the Ackermann controller publish command-integrated odometry",
            ),
            DeclareLaunchArgument(
                "ackermann_publish_tf",
                default_value="true",
                description="Let the Ackermann controller publish odom to base_link TF",
            ),
            DeclareLaunchArgument(
                "ackermann_odom_topic",
                default_value="/odom",
                description="Ackermann controller odometry topic when enabled",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

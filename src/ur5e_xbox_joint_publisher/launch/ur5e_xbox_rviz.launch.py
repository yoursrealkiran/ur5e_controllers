#!/usr/bin/env python3
"""
Launch UR5e in RViz and control its joints using an Xbox controller.
This version disables the Joint State Publisher GUI to avoid conflicts.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Arguments ---
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type of the UR robot (e.g., ur5e, ur10e, etc.)",
        ),
    ]
    ur_type = LaunchConfiguration("ur_type")

    # --- Paths ---
    ur_description_pkg = FindPackageShare("ur_description")
    ur_launch_file = PathJoinSubstitution([ur_description_pkg, "launch", "view_ur.launch.py"])

    # --- Include the official URDF viewer launch (with GUI disabled) ---
    ur5e_view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_launch_file),
        launch_arguments={
            "ur_type": ur_type,
            # The following disables the GUI sliders if supported
            "gui": "false"
        }.items(),
    )

    # --- Joystick driver node ---
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    # --- Xbox teleoperation node ---
    xbox_joint_pub = Node(
        package="ur5e_xbox_joint_publisher",
        executable="ur5e_xbox_joint_publisher",
        name="ur5e_xbox_joint_publisher",
        output="screen",
    )

    # --- Launch description ---
    return LaunchDescription(declared_arguments + [ur5e_view, joy_node, xbox_joint_pub])

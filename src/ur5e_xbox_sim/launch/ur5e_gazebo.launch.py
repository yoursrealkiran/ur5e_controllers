#!/usr/bin/env python3
#
# Launch UR5e robot in Gazebo using the Universal_Robots_ROS2_Description package
# and spawn the ros2_control velocity controller.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():

    # === Package paths ===
    ur_desc_pkg = FindPackageShare("ur_description")
    sim_pkg = FindPackageShare("ur5e_xbox_sim")
    gazebo_pkg = FindPackageShare("gazebo_ros")

    # === Files ===
    urdf_file = PathJoinSubstitution(
        [ur_desc_pkg, "urdf", "ur.urdf.xacro"]
    )
    controller_yaml = PathJoinSubstitution(
        [sim_pkg, "config", "ur5e_controllers.yaml"]
    )

    # === Robot description using xacro ===
    robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    urdf_file,
    " ",
    "name:=", "ur",
    " ",
    "ur_type:=", "ur5e",
    " ",
    "safety_limits:=", "true",
    " ",
    "safety_pos_margin:=", "0.15",
    " ",
    "safety_k_position:=", "20",
    " ",
    "use_mock_hardware:=", "true",
    " ",
    "mock_sensor_commands:=", "false",
    " ",
    "use_gazebo:=", "true"
    ])


    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # === Gazebo world launch ===
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_pkg, "launch", "gazebo.launch.py"])
        )
    )

    # === Nodes ===
    # Publish robot state
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Load controller manager parameters
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_yaml],
        output="screen",
    )

    # Spawners for controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur5e_joint_velocity_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # === Launch Description ===
    ld = LaunchDescription()

    ld.add_action(gazebo)
    ld.add_action(robot_state_pub)
    ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(velocity_controller_spawner)

    return ld

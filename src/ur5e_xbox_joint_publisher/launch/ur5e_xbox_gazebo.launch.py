import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Paths and Arguments
    pkg_share = FindPackageShare("ur5e_xbox_joint_publisher")
    
    # Get URDF via Xacro (Standard UR Driver way)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
        " name:=ur",
        " ur_type:=ur5e",
        " sim_gz:=true", # Note: Modern UR driver uses sim_gz for Jazzy
        " sim_ignition:=true",    # Older Ignition naming (sometimes used in Jazzy)
        " simulation_controllers:=", 
        PathJoinSubstitution([pkg_share, "config", "ur5e_controllers.yaml"])
    ])
    robot_description = {"robot_description": robot_description_content}

    # 2. Start Gazebo Sim (The Harmonic/Jazzy version)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. Spawn Robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'ur5e', '-z', '0.1'],
        output='screen',
    )

    # 4. Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 5. Controller Spawners
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur5e_arm_controller"],
    )

    # 6. Xbox & Joy Nodes
    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[{"deadzone": 0.1, "autorepeat_rate": 20.0, "use_sim_time": True}]
    )

    xbox_to_gazebo = Node(
        package="ur5e_xbox_joint_publisher",
        executable="ur5e_xbox_gazebo", # The new C++ node we built
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gz_sim,
        node_robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        arm_controller_spawner,
        joy_node,
        xbox_to_gazebo
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- Arguments ---
    # Using LaunchConfiguration directly as a variable
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur5e",
        description="Type of the UR robot (e.g., ur5e, ur10e, etc.)",
    )
    
    ur_type = LaunchConfiguration("ur_type")

    # --- Paths ---
    # In Jazzy, PathJoinSubstitution is robust, but the Source often likes a list
    ur_description_pkg = FindPackageShare("ur_description")
    ur_launch_file = PathJoinSubstitution([ur_description_pkg, "launch", "view_ur.launch.py"])

    # --- Include URDF viewer ---
    # We use a list [ur_launch_file] and a list of tuples for arguments
    ur5e_view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ur_launch_file]),
        launch_arguments=[
            ("ur_type", ur_type),
            ("gui", "false"),        # Standard for ur_description
            ("use_gui", "false")     # Added for redundancy across versions
        ],
    )

    # --- Joystick driver node ---
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "deadzone": 0.1,
            "autorepeat_rate": 20.0,
        }]
    )

    # --- Xbox teleoperation node ---
    xbox_joint_pub = Node(
        package="ur5e_xbox_joint_publisher",
        executable="ur5e_xbox_joint_publisher",
        name="xbox_joint_publisher", # Changed name to avoid package name conflict
        output="screen",
    )

    return LaunchDescription([
        ur_type_arg,
        ur5e_view,
        joy_node,
        xbox_joint_pub
    ])
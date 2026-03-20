import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Paths and Setup
    pkg_share = FindPackageShare("ur5e_xbox_joint_publisher")
    ur_description_share = get_package_share_directory('ur_description')

    # FIX: Set the Gazebo resource path so it can find the meshes automatically
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + ur_description_share
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = ur_description_share

    # Point to your NEW wrapper xacro file
    robot_xacro_file = PathJoinSubstitution([pkg_share, "urdf", "gazebo_ur5e.xacro"])

    # 2. Get URDF via Xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        robot_xacro_file
    ])
    robot_description = {"robot_description": robot_description_content}

    # 3. Start Gazebo Sim with an empty world 
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 4. Spawn Robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'ur5e', '-z', '0.1'],
        output='screen',
    )

    # 5. Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 6. Controller Spawners
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

    # 7. Xbox & Joy Nodes
    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[{"deadzone": 0.1, "autorepeat_rate": 20.0, "use_sim_time": True}]
    )

    xbox_to_gazebo = Node(
        package="ur5e_xbox_joint_publisher",
        executable="ur5e_xbox_gazebo", 
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    # Including RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        # Optional: point to a saved .rviz config file
        # arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'view_robot.rviz'])]
    )

    # This bridge specifically translates the Gazebo clock into a ROS 2 /clock message
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
                   ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        node_robot_state_publisher,
        spawn_robot,
        joint_state_broadcaster,
        arm_controller_spawner,
        joy_node,
        xbox_to_gazebo,
        ros_gz_bridge,
        rviz_node
    ])
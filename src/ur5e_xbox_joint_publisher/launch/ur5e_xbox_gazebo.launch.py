import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Paths and Setup
    pkg_share = FindPackageShare("ur5e_xbox_joint_publisher")
    
    # IMPORTANT: We point to the PARENT folder of the descriptions 
    # so that 'model://ur_description' resolves correctly in Gazebo.
    ur_desc_path = os.path.join(get_package_share_directory('ur_description'), '..')
    robotiq_desc_path = os.path.join(get_package_share_directory('robotiq_description'), '..')
    pkg_urdf_path = os.path.join(get_package_share_directory('ur5e_xbox_joint_publisher'), '..')

    # Environment variable for Gazebo Sim (Jazzy) to find meshes
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.pathsep.join([
            ur_desc_path, 
            robotiq_desc_path,
            pkg_urdf_path
        ])]
    )

    # Point to your wrapper xacro file
    robot_xacro_file = PathJoinSubstitution([pkg_share, "urdf", "gazebo_ur5e.xacro"])

    # 2. Get URDF via Xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        robot_xacro_file
    ])
    robot_description = {"robot_description": robot_description_content}

    # 3. Start Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r --physics-engine gz-physics-bullet-featherstone-plugin empty.sdf'}.items(),
    )

    # 4. Spawn Robot
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

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
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

    # 8. ROS-GZ Bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 9. Sequential Controller Loading
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster],
        )
    )

    load_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller_spawner],
        )
    )

    load_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # Use if you want red cube in simulation
       
    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'red_cube',
            '-file', os.path.expanduser('src/ur5e_xbox_joint_publisher/urdf/red_cube.sdf'),
            '-x', '0.5', '-y', '0.0', '-z', '0.05'
        ],
        output='screen',
    )

    spawn_plate = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'green_plate',
            '-file', os.path.expanduser('src/ur5e_xbox_joint_publisher/urdf/green_plate.sdf'),
            '-x', '0.5', '-y', '0.2', '-z', '0.005'
        ],
        output='screen',
    )

    # Use if you want red ball in simulation

    spawn_ball = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'red_ball',  # NEW: Change name to match SDF
            '-file', os.path.expanduser('src/ur5e_xbox_joint_publisher/urdf/red_ball.sdf'),
            '-x', '0.5', '-y', '0.0', '-z', '0.05'
        ],
        output='screen',
    )


    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        node_robot_state_publisher,
        spawn_robot,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
        joy_node,
        xbox_to_gazebo,
        ros_gz_bridge,
        rviz_node,
        spawn_cube,   # Use if you want red cube in simulation
        spawn_plate,
        # spawn_ball,  # Use if you want red ball in simulation
    
    ])
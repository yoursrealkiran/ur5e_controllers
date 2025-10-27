from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    controller_name = LaunchConfiguration('controller_name', default='ur5e_cartesian_controller')
    return LaunchDescription([
        DeclareLaunchArgument('controller_name', default_value='ur5e_cartesian_controller'),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawner',
            output='screen',
            arguments=[controller_name]
        )
    ])

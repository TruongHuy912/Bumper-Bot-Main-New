import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('bumperbot_active_slam'),
            'config',
            'active_slam.yaml',
        ),
        description='Full path to the Active SLAM parameter file',
    )

    active_slam_node = Node(
        package='bumperbot_active_slam',
        executable='active_slam_node',
        name='active_slam_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        active_slam_node,
    ])

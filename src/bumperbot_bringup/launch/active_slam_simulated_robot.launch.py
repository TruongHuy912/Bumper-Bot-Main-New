import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world_name')
    autostart = LaunchConfiguration('autostart')
    active_slam_delay_sec = LaunchConfiguration('active_slam_delay_sec')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
    )
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='active_slam_test',
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
    )
    active_slam_delay_sec_arg = DeclareLaunchArgument(
        'active_slam_delay_sec',
        default_value='20.0',
        description='Seconds to wait before starting Active SLAM so SLAM Toolbox can publish map TF.',
    )
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='-0.5',
        description='Robot spawn x position. Use a free area if Nav2 reports start in lethal space.',
    )
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Robot spawn y position. Use a free area if Nav2 reports start in lethal space.',
    )
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.05',
    )
    spawn_yaw_arg = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='0.0',
    )

    simulated_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bumperbot_bringup'),
                'launch',
                'simulated_robot.launch.py',
            )
        ),
        launch_arguments={
            'use_slam': 'true',
            'use_sim_time': use_sim_time,
            'world_name': world_name,
            'autostart': autostart,
            'spawn_x': spawn_x,
            'spawn_y': spawn_y,
            'spawn_z': spawn_z,
            'spawn_yaw': spawn_yaw,
        }.items(),
    )

    active_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bumperbot_active_slam'),
                'launch',
                'active_slam.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_name_arg,
        autostart_arg,
        active_slam_delay_sec_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        simulated_robot,
        TimerAction(
            period=active_slam_delay_sec,
            actions=[active_slam],
        ),
    ])

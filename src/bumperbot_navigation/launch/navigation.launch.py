import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _make_navigation_nodes(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    enable_velocity_smoother = _as_bool(
        LaunchConfiguration('enable_velocity_smoother').perform(context)
    )
    enable_collision_monitor = _as_bool(
        LaunchConfiguration('enable_collision_monitor').perform(context)
    )

    bumperbot_navigation_pkg = get_package_share_directory('bumperbot_navigation')
    actions = []

    try:
        get_package_share_directory('nav2_velocity_smoother')
    except PackageNotFoundError:
        enable_velocity_smoother = False
        actions.append(
            LogInfo(
                msg='nav2_velocity_smoother not found; velocity smoother disabled.'
            )
        )

    try:
        get_package_share_directory('nav2_collision_monitor')
    except PackageNotFoundError:
        enable_collision_monitor = False
        actions.append(
            LogInfo(
                msg='nav2_collision_monitor not found; collision monitor disabled.'
            )
        )

    use_cmd_vel_chain = enable_velocity_smoother or enable_collision_monitor
    controller_remappings = []
    if use_cmd_vel_chain:
        controller_remappings.append(('cmd_vel', 'cmd_vel_raw'))
    behavior_remappings = []
    if use_cmd_vel_chain:
        behavior_remappings.append(('cmd_vel', 'cmd_vel_raw'))

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
    ]

    nav2_controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[
            os.path.join(
                bumperbot_navigation_pkg,
                'config',
                'controller_server.yaml',
            ),
            {'use_sim_time': use_sim_time},
        ],
        remappings=controller_remappings,
    )

    nav2_planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            os.path.join(
                bumperbot_navigation_pkg,
                'config',
                'planner_server.yaml',
            ),
            {'use_sim_time': use_sim_time},
        ],
    )

    nav2_behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            os.path.join(
                bumperbot_navigation_pkg,
                'config',
                'behavior_server.yaml',
            ),
            {'use_sim_time': use_sim_time},
        ],
        remappings=behavior_remappings,
    )

    nav2_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            os.path.join(
                bumperbot_navigation_pkg,
                'config',
                'bt_navigator.yaml',
            ),
            {'use_sim_time': use_sim_time},
        ],
    )

    nav2_smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[
            os.path.join(
                bumperbot_navigation_pkg,
                'config',
                'smoother_server.yaml',
            ),
            {'use_sim_time': use_sim_time},
        ],
    )

    actions.extend(
        [
            nav2_controller_server,
            nav2_planner_server,
            nav2_smoother_server,
            nav2_behaviors,
            nav2_bt_navigator,
        ]
    )

    if enable_velocity_smoother:
        lifecycle_nodes.append('velocity_smoother')
        smoother_output = (
            'cmd_vel_smoothed' if enable_collision_monitor else 'cmd_vel'
        )
        actions.append(
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[
                    os.path.join(
                        bumperbot_navigation_pkg,
                        'config',
                        'velocity_smoother.yaml',
                    ),
                    {'use_sim_time': use_sim_time},
                ],
                remappings=[
                    ('cmd_vel', 'cmd_vel_raw'),
                    ('cmd_vel_smoothed', smoother_output),
                ],
            )
        )

    if enable_collision_monitor:
        lifecycle_nodes.append('collision_monitor')
        collision_input = (
            'cmd_vel_smoothed' if enable_velocity_smoother else 'cmd_vel_raw'
        )
        actions.append(
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                parameters=[
                    os.path.join(
                        bumperbot_navigation_pkg,
                        'config',
                        'collision_monitor.yaml',
                    ),
                    {
                        'use_sim_time': use_sim_time,
                        'cmd_vel_in_topic': collision_input,
                        'cmd_vel_out_topic': 'cmd_vel',
                    },
                ],
            )
        )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'node_names': lifecycle_nodes},
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
        ],
    )
    actions.append(
        TimerAction(
            period=8.0,
            actions=[nav2_lifecycle_manager],
        )
    )

    return actions


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
    )
    enable_velocity_smoother_arg = DeclareLaunchArgument(
        'enable_velocity_smoother',
        default_value='true',
    )
    enable_collision_monitor_arg = DeclareLaunchArgument(
        'enable_collision_monitor',
        default_value='true',
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            autostart_arg,
            enable_velocity_smoother_arg,
            enable_collision_monitor_arg,
            OpaqueFunction(function=_make_navigation_nodes),
        ]
    )

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_name = LaunchConfiguration("world_name")
    autostart = LaunchConfiguration("autostart")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="active_slam_test"
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true"
    )
    spawn_x_arg = DeclareLaunchArgument(
        "spawn_x",
        default_value="-0.5"
    )
    spawn_y_arg = DeclareLaunchArgument(
        "spawn_y",
        default_value="0.0"
    )
    spawn_z_arg = DeclareLaunchArgument(
        "spawn_z",
        default_value="0.05"
    )
    spawn_yaw_arg = DeclareLaunchArgument(
        "spawn_yaw",
        default_value="0.0"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={
            "world_name": world_name,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
        }.items(),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False",
            "use_sim_time": use_sim_time,
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )
    
    return LaunchDescription([
        use_slam_arg,
        use_sim_time_arg,
        world_name_arg,
        autostart_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        gazebo,
        controller,
        joystick,
        localization,
        slam,
        navigation,
        rviz,
    ])
